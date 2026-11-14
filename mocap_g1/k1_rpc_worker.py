"""Booster loco RPCs in an isolated child process, each with a deadline.

A blocking SDK call (``GetMode``, ``UpperBodyCustomControl``, ``ChangeMode`` ...)
can hang for as long as the robot's RPC server does, and a lock around it then
blocks every other caller.  Here the calls run in a child process that owns its
own ``B1LocoClient`` and DDS participant; the parent only queues requests and
reads replies.  A monitor thread in the parent enforces the deadlines: an attempt
past its deadline is failed, the child is SIGKILLed and a fresh one started, and
the request is tried once more on it if its total deadline still allows.

The child is started with ``spawn``, not ``fork``: a forked child would inherit
the parent's SDK and DDS state (threads, sockets) in an undefined condition.

Only one request runs at a time, so the parent always knows which request a hung
child is stuck in.  A ``preempt`` request (release, damping) does not wait behind
a running ordinary request: that one is failed as ``preempted`` and the child
replaced at once.  Two ``preempt`` requests run in order, so a release in flight
is never killed by another stop request.  A child is kept started at all times,
so a restart's SDK start overlaps the wait instead of eating a request deadline.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
import importlib
import importlib.util
import itertools
import math
import multiprocessing
from multiprocessing.connection import Connection
import os
from queue import SimpleQueue
import signal
import threading
import time
from types import ModuleType
from typing import Any

SDK_MODULE = "booster_robotics_sdk_python"
#: Tests only: path of a fake SDK module file for the child to load instead.
FAKE_SDK_ENV = "K1_RPC_FAKE_SDK"
#: The child refuses ``FAKE_SDK_ENV`` unless this is exactly "1".
ALLOW_FAKE_ENV = "K1_RPC_ALLOW_FAKE_SDK"
REQUESTS = frozenset({"GetMode", "UpperBodyCustomControl", "GetRobotInfo", "ChangeMode",
                      "MoveCommand"})
ROBOT_INFO_FIELDS = ("model", "name", "serial_number", "version")
MONITOR_PERIOD_S = 0.005
#: Attempts per request: the first, and one retry on a fresh child.
MAX_ATTEMPTS = 2
#: A child that has not finished its SDK and DDS start by then is replaced.
START_TIMEOUT_S = 5.0


@dataclass(frozen=True)
class Deadline:
    """``attempt_s`` bounds one try; ``total_s`` bounds the request, retry included."""

    attempt_s: float
    total_s: float

    def __post_init__(self) -> None:
        if not (math.isfinite(self.attempt_s) and math.isfinite(self.total_s)
                and 0 < self.attempt_s <= self.total_s):
            raise ValueError("deadline needs 0 < attempt_s <= total_s, finite")


#: A missed mode query is "mode unknown" (no time for a retry);
#: damping 0.5 s.  Release (2.0 s total) is budgeted by the transport.
MODE_QUERY = Deadline(0.3, 0.3)
DAMPING = Deadline(0.5, 0.5)
TAKEOVER = Deadline(1.0, 2.0)
ROBOT_INFO = Deadline(1.0, 2.0)


class RpcClosed(RuntimeError):
    """The worker was closed; no request is accepted."""


@dataclass(frozen=True)
class RpcResult:
    id: int
    name: str
    ok: bool
    value: Any = None
    error: str | None = None
    #: The SDK reported its own timeout (code 100), or a deadline passed.
    timeout: bool = False
    attempts: int = 0
    submitted_at: float = 0.0
    finished_at: float = 0.0


@dataclass
class _Request:
    id: int
    name: str
    args: tuple[Any, ...]
    deadline: Deadline
    preempt: bool
    submitted_at: float
    attempts: int = 0
    attempt_end: float = math.inf
    done: threading.Event = field(default_factory=threading.Event)
    result: RpcResult | None = None

    @property
    def total_end(self) -> float:
        return self.submitted_at + self.deadline.total_s


class RpcWorker:
    """Parent side: ``submit`` returns at once; ``poll``/``result`` read the outcome.

    Callers and the monitor thread share no lock: requests pass through a
    ``SimpleQueue`` and outcomes through a per-request ``Event``, so no caller
    can ever wait on the monitor, the pipe or the child.
    """

    def __init__(self, *, domain_id: int = 0, network_interface: str = "",
                 clock: Callable[[], float] = time.monotonic,
                 start_timeout_s: float = START_TIMEOUT_S) -> None:
        self._args = (int(domain_id), str(network_interface))
        self._clock = clock
        self._start_timeout_s = start_timeout_s
        self._ctx = multiprocessing.get_context("spawn")
        self._inbox: SimpleQueue[_Request] = SimpleQueue()
        self._requests: dict[int, _Request] = {}
        self._ids = itertools.count(1)
        self._stop = threading.Event()
        # Owned by the monitor thread alone.
        self._queue: deque[_Request] = deque()
        self._proc: Any = None
        self._conn: Connection | None = None
        self._ready = False
        self._started_at = 0.0
        #: The child reported an SDK start failure: no restart until a new request.
        self._init_failed = False
        self._running: _Request | None = None
        self.starts = 0
        self.kills = 0
        self.last_kill_reason: str | None = None
        self._monitor = threading.Thread(target=self._run, name="k1-rpc-monitor", daemon=True)
        self._monitor.start()

    # -- caller side ----------------------------------------------------------

    def submit(self, name: str, args: Sequence[Any] = (), *, deadline: Deadline,
               preempt: bool = False) -> int:
        """Queue one request and return its id without waiting."""
        if name not in REQUESTS:
            raise ValueError(f"unknown loco request {name!r}")
        if self._stop.is_set():
            raise RpcClosed("the RPC worker is closed")
        request = _Request(next(self._ids), name, tuple(args), deadline, preempt, self._clock())
        self._requests[request.id] = request
        self._inbox.put(request)
        return request.id

    def poll(self, request_id: int) -> RpcResult | None:
        """The outcome, or ``None`` while the request is pending."""
        return self._requests[request_id].result

    def result(self, request_id: int, timeout: float) -> RpcResult | None:
        """Wait at most ``timeout`` seconds for the outcome."""
        request = self._requests[request_id]
        request.done.wait(max(0.0, timeout))
        return request.result

    def forget(self, request_id: int) -> None:
        """Drop a finished request from the table."""
        request = self._requests.get(request_id)
        if request is not None and request.result is not None:
            del self._requests[request_id]

    def call(self, name: str, args: Sequence[Any] = (), *, deadline: Deadline,
             preempt: bool = False) -> RpcResult:
        """Submit and wait for the outcome: at most the total deadline plus a
        few monitor periods."""
        request_id = self.submit(name, args, deadline=deadline, preempt=preempt)
        result = self.result(request_id, deadline.total_s + 0.1)
        self.forget(request_id)
        if result is None:  # the monitor itself is stuck: report, never wait longer
            return RpcResult(request_id, name, False, error="no outcome within the deadline",
                             timeout=True)
        return result

    def close(self) -> None:
        self._stop.set()
        self._monitor.join(timeout=2.0)

    # -- monitor thread -------------------------------------------------------

    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                self._drain_inbox()
                self._receive()
                self._service(self._clock())
        finally:
            self._kill("closed")
            self._drain_inbox()
            for request in self._queue:
                self._finish(request, False, error="RPC worker closed")
            self._queue.clear()

    def _drain_inbox(self) -> None:
        while not self._inbox.empty():
            request = self._inbox.get()
            self._init_failed = False
            if request.preempt:
                self._queue.appendleft(request)
            else:
                self._queue.append(request)

    def _receive(self) -> None:
        conn = self._conn
        if conn is None:
            self._stop.wait(MONITOR_PERIOD_S)
            return
        try:
            if not conn.poll(MONITOR_PERIOD_S):
                return
            message = conn.recv()
        except (EOFError, OSError):
            self._crashed("worker exited")
            return
        self._handle(message)

    def _handle(self, message: tuple[Any, ...]) -> None:
        kind = message[0]
        if kind == "ready":
            self._ready = True
            return
        if kind == "init_failed":
            # A child that cannot start will not start next time either:
            # fail what waits now instead of restarting in a loop.
            self._kill("init failed")
            self._init_failed = True
            for request in self._queue:
                self._finish(request, False, error=f"SDK init failed: {message[1]}")
            self._queue.clear()
            return
        running = self._running
        if running is None or message[1] != running.id:
            return  # a late reply for a request already failed
        self._running = None
        if kind == "ok":
            self._finish(running, True, value=message[2])
        else:
            self._finish(running, False, error=message[2], timeout=bool(message[3]))

    def _service(self, now: float) -> None:
        if self._proc is not None and not self._proc.is_alive():
            self._crashed("worker died")
        running = self._running
        if running is not None and now >= running.attempt_end:
            self._kill(f"{running.name} passed its deadline")
            self._retry_or_fail(running, now, "deadline passed")
        elif running is not None and not running.preempt and self._queue \
                and self._queue[0].preempt:
            self._kill(f"{running.name} preempted by {self._queue[0].name}")
            self._finish(running, False, error=f"preempted by {self._queue[0].name}")
        for request in [r for r in self._queue if now >= r.total_end]:
            self._queue.remove(request)
            self._finish(request, False, error="deadline passed before it ran", timeout=True)
        if self._proc is not None and not self._ready \
                and now - self._started_at >= self._start_timeout_s:
            self._kill("worker did not start in time")
        if self._proc is None and not self._init_failed:
            self._start()
        if self._queue and self._ready and self._running is None:
            self._dispatch(self._queue.popleft(), now)

    def _dispatch(self, request: _Request, now: float) -> None:
        assert self._conn is not None
        request.attempts += 1
        request.attempt_end = min(now + request.deadline.attempt_s, request.total_end)
        self._running = request
        try:
            self._conn.send((request.id, request.name, request.args))
        except (OSError, ValueError):
            self._crashed("worker pipe closed")

    def _crashed(self, why: str) -> None:
        running = self._running
        self._kill(why)
        if running is not None:
            self._retry_or_fail(running, self._clock(), why)

    def _retry_or_fail(self, request: _Request, now: float, why: str) -> None:
        if request.attempts < MAX_ATTEMPTS and now < request.total_end:
            self._queue.appendleft(request)  # on the fresh child, before anything else
        else:
            self._finish(request, False, error=why, timeout=True)

    def _start(self) -> None:
        parent, child = self._ctx.Pipe()
        proc = self._ctx.Process(target=worker_main, args=(child, *self._args),
                                 name="k1-rpc-worker", daemon=True)
        proc.start()
        child.close()
        self._proc, self._conn, self._ready = proc, parent, False
        self._started_at = self._clock()
        self.starts += 1

    def _kill(self, why: str) -> None:
        proc, conn = self._proc, self._conn
        self._proc, self._conn, self._ready, self._running = None, None, False, None
        if conn is not None:
            conn.close()
        if proc is not None:
            self.last_kill_reason = why
            if proc.is_alive() and proc.pid is not None:
                try:
                    os.kill(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self.kills += 1
            proc.join(timeout=1.0)

    def _finish(self, request: _Request, ok: bool, *, value: Any = None,
                error: str | None = None, timeout: bool = False) -> None:
        request.result = RpcResult(request.id, request.name, ok, value, error, timeout,
                                   request.attempts, request.submitted_at, self._clock())
        request.done.set()


# -- child process ------------------------------------------------------------

def load_sdk() -> ModuleType:
    """The Booster bindings, or a test fake when both test-only variables allow it."""
    fake = os.environ.get(FAKE_SDK_ENV)
    if fake:
        if os.environ.get(ALLOW_FAKE_ENV) != "1":
            raise RuntimeError(f"{FAKE_SDK_ENV} is set but {ALLOW_FAKE_ENV} is not 1; "
                               "refusing a fake SDK outside the tests")
        spec = importlib.util.spec_from_file_location("k1_fake_booster_sdk", fake)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"cannot load the fake SDK {fake}")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    return importlib.import_module(SDK_MODULE)


def execute(sdk: ModuleType, client: Any, name: str, args: tuple[Any, ...]) -> Any:
    """One loco request; returns a picklable value or raises."""
    if name == "GetMode":
        return int(_checked(name, client.GetMode()).mode)
    if name == "GetRobotInfo":
        info = _checked(name, client.GetRobotInfo())
        return {key: str(getattr(info, key)) for key in ROBOT_INFO_FIELDS}
    if name == "UpperBodyCustomControl":
        _checked(name, client.UpperBodyCustomControl(bool(args[0])))
    elif name == "ChangeMode":
        _checked(name, client.ChangeMode(getattr(sdk.RobotMode, f"k{str(args[0]).title()}")))
    elif name == "MoveCommand":
        _checked(name, client.MoveCommand(*(float(value) for value in args)))
    else:
        raise ValueError(f"unknown loco request {name!r}")
    return None


def _checked(name: str, value: Any) -> Any:
    # The C++ client returns an int32 status (0 = success); a binding that
    # passes it through must not turn a refused call into a confirmed one.
    if isinstance(value, int) and not isinstance(value, bool) and value != 0:
        raise RuntimeError(f"{name} returned code = {value}")
    return value


def worker_main(conn: Connection, domain_id: int, network_interface: str) -> None:
    """Child entry point: start the SDK, then serve requests until the pipe closes."""
    try:
        sdk = load_sdk()
        sdk.ChannelFactory.Instance().Init(domain_id, network_interface)
        client = sdk.B1LocoClient()
        client.Init()
    except Exception as exc:  # report, then exit: the parent decides
        conn.send(("init_failed", f"{type(exc).__name__}: {exc}"))
        return
    conn.send(("ready", os.getpid()))
    while True:
        try:
            message = conn.recv()
        except (EOFError, OSError):
            return
        request_id, name, args = message
        try:
            value = execute(sdk, client, name, args)
        except Exception as exc:
            conn.send(("error", request_id, f"{name} failed: {exc}", "code = 100" in str(exc)))
        else:
            conn.send(("ok", request_id, value))
