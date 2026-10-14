"""Passive Xsens UDP stream monitor for the Windows app's Setup screen.

A background thread listens on the Xsens UDP port and keeps rolling
statistics (rate, sample-counter loss, joint angles, last sender) so the app
can show whether Xsens data is arriving before anything else runs.  It only
reads datagrams; it never forwards them.

Only one program can own the port at a time: the socket is bound without
``SO_REUSEADDR`` (and with ``SO_EXCLUSIVEADDRUSE`` on Windows), so a setup
job that needs the port (stream check, live simulation) must :meth:`pause`
the monitor first and :meth:`resume` it afterwards.  Pauses nest: the socket
reopens only when every pause has been resumed.  When another program holds
the port the state is ``port_busy`` and the bind is retried every
``retry_s``.

Parsing reuses :mod:`mocap_g1.mxtp02`; ``clock`` and ``socket_factory`` are
injectable for tests.  All public methods are thread-safe.
"""

from __future__ import annotations

from collections import deque
import socket
import threading
import time
from typing import Any, Callable

from ..mxtp02 import (
    ProtocolError,
    counter_delta,
    parse_body_datagram,
    parse_joint_angles_datagram,
)


STREAM_STATES = ("off", "none", "receiving", "stale", "port_busy", "paused")
#: Rolling window for rate, loss and joint-angle presence.
WINDOW_S = 2.0
#: A body packet younger than this means the stream is ``receiving``.
RECEIVING_S = 1.0
#: A sample-counter jump larger than this is a stream restart, not loss.
_RESTART_GAP = 10_000
_BODY_ID = b"MXTP02"
_JOINT_ANGLES_ID = b"MXTP20"


def default_socket_factory() -> socket.socket:
    """A UDP socket that claims its port exclusively (Windows) and never shares it."""

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    exclusive = getattr(socket, "SO_EXCLUSIVEADDRUSE", None)
    if exclusive is not None:  # Windows only
        try:
            sock.setsockopt(socket.SOL_SOCKET, exclusive, 1)
        except OSError:
            pass
    return sock


class StreamMonitor:
    """Listen on ``bind:port`` and summarise the Xsens stream as a StreamSnapshot."""

    def __init__(
        self,
        *,
        bind: str = "0.0.0.0",
        port: int = 5555,
        clock: Callable[[], float] = time.monotonic,
        socket_factory: Callable[[], Any] = default_socket_factory,
        poll_s: float = 0.2,
        retry_s: float = 2.0,
    ) -> None:
        self._bind = bind
        self._port = int(port)
        self._clock = clock
        self._socket_factory = socket_factory
        self.poll_s = float(poll_s)
        self.retry_s = float(retry_s)
        self._lock = threading.Lock()
        self._changed = threading.Condition(self._lock)
        self._thread: threading.Thread | None = None
        self._closed = False
        self._pauses = 0
        self._generation = 0  # bumped by configure() to force a rebind
        self._socket: Any = None  # owned by the thread; None while not bound
        self._binding = False  # the thread is between choosing an address and owning a socket
        self._state = "off"
        self._reset_stats_locked()

    # ----------------------------------------------------------- lifecycle

    def start(self) -> None:
        """Start the listener thread (idempotent)."""

        with self._lock:
            if self._thread is not None or self._closed:
                return
            # The state becomes "none" once the thread has bound the port.
            self._thread = threading.Thread(
                target=self._run, name="mocap-stream-monitor", daemon=True
            )
            thread = self._thread
        thread.start()

    def pause(self, timeout: float = 3.0) -> None:
        """Release the port; returns once the socket is closed (bounded by ``timeout``)."""

        with self._changed:
            self._pauses += 1
            self._state = "paused"
            self._changed.notify_all()
            if self._thread is None:
                return
            deadline = time.monotonic() + timeout
            while self._socket is not None or self._binding:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._changed.wait(remaining)

    def resume(self) -> None:
        """Undo one :meth:`pause`; the port is reopened when no pause is left."""

        with self._changed:
            if self._pauses == 0:
                return
            self._pauses -= 1
            if self._pauses == 0 and self._thread is None:
                self._state = "off"
            # Otherwise the thread rebinds and reports "none" (or "port_busy").
            self._changed.notify_all()

    def configure(self, *, bind: str, port: int) -> None:
        """Change the listening address; rebinds (and resets statistics) if it changed."""

        with self._changed:
            if (bind, int(port)) == (self._bind, self._port):
                return
            self._bind, self._port = bind, int(port)
            self._generation += 1
            self._changed.notify_all()

    def close(self, timeout: float = 3.0) -> None:
        """Stop the thread and release the port."""

        with self._changed:
            self._closed = True
            self._state = "off"
            self._changed.notify_all()
            thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout)

    @property
    def paused(self) -> bool:
        with self._lock:
            return self._pauses > 0

    # ------------------------------------------------------------ snapshot

    def snapshot(self) -> dict[str, Any]:
        """The StreamSnapshot document (see the Windows app contracts)."""

        with self._lock:
            now = self._clock()
            self._expire_locked(now)
            state = self._state
            last = self._last_body_at
            if state in ("none", "receiving", "stale") and self._body_frames:
                state = "receiving" if last is not None and now - last < RECEIVING_S else "stale"
            return {
                "state": state,
                "bind": self._bind,
                "port": self._port,
                "source": self._source,
                "rate_hz": self._rate_locked(now),
                "loss_fraction": self._loss_locked(),
                "body_frames": self._body_frames,
                "joint_angles": bool(self._joint_times),
                "last_packet_age_s": None if last is None else round(max(0.0, now - last), 3),
                "malformed": self._malformed,
            }

    # ---------------------------------------------------------------- stats

    def _reset_stats_locked(self) -> None:
        self._body: deque[tuple[float, int]] = deque()  # (received_at, sample counter)
        self._joint_times: deque[float] = deque()
        self._body_frames = 0
        self._malformed = 0
        self._source: str | None = None
        self._last_body_at: float | None = None
        self._first_body_at: float | None = None

    def _expire_locked(self, now: float) -> None:
        while self._body and now - self._body[0][0] > WINDOW_S:
            self._body.popleft()
        while self._joint_times and now - self._joint_times[0] > WINDOW_S:
            self._joint_times.popleft()

    def _rate_locked(self, now: float) -> float | None:
        if not self._body_frames or self._first_body_at is None:
            return None
        count = len(self._body)
        if not count:
            return 0.0
        if now - self._first_body_at >= WINDOW_S:
            return round(count / WINDOW_S, 2)
        # The window is not full yet: average over the packets seen so far.
        first, last = self._body[0][0], self._body[-1][0]
        if count < 2 or last <= first:
            return None
        return round((count - 1) / (last - first), 2)

    def _loss_locked(self) -> float | None:
        if len(self._body) < 2:
            return None
        reference = self._body[0][1]
        offsets = {counter_delta(counter, reference) for _, counter in self._body}
        expected = max(offsets) - min(offsets) + 1
        return round(max(0, expected - len(offsets)) / expected, 4)

    def observe(self, data: bytes, source: tuple[str, int] | None, received_at: float) -> None:
        """Account one datagram (called by the listener thread; public for tests)."""

        with self._lock:
            if data[:6] == _BODY_ID:
                try:
                    body = parse_body_datagram(data)
                except ProtocolError:
                    self._malformed += 1
                    return
                counter = body.sample_counter
                if self._body:
                    jump = counter_delta(counter, self._body[-1][1])
                    if abs(jump) > _RESTART_GAP:
                        self._body.clear()  # a restarted stream is not packet loss
                self._body.append((received_at, counter))
                self._body_frames += 1
                if self._first_body_at is None:
                    self._first_body_at = received_at
                self._last_body_at = received_at
                if source is not None:
                    self._source = f"{source[0]}:{source[1]}"
            elif data[:6] == _JOINT_ANGLES_ID:
                try:
                    parse_joint_angles_datagram(data)
                except ProtocolError:
                    self._malformed += 1
                    return
                self._joint_times.append(received_at)
            elif data[:4] != b"MXTP":
                self._malformed += 1
            # Other MXTP message types (time code, centre of mass, ...) are ignored.
            self._expire_locked(received_at)

    # --------------------------------------------------------------- thread

    def _wanted_locked(self) -> bool:
        return not self._closed and self._pauses == 0

    def _set_socket(self, sock: Any) -> None:
        with self._changed:
            self._socket = sock
            self._changed.notify_all()

    def _close_socket(self, sock: Any) -> None:
        try:
            sock.close()
        except OSError:
            pass
        self._set_socket(None)

    def _run(self) -> None:
        while True:
            with self._changed:
                while not self._closed and self._pauses:
                    self._state = "paused"
                    self._changed.wait()
                if self._closed:
                    self._state = "off"
                    return
                bind, port, generation = self._bind, self._port, self._generation
                self._binding = True
            try:
                sock = self._socket_factory()
            except OSError:
                sock = None
            try:
                if sock is None:
                    raise OSError("cannot create a UDP socket")
                sock.bind((bind, port))
                sock.settimeout(self.poll_s)
            except OSError:
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass
                with self._changed:
                    self._binding = False
                    self._changed.notify_all()
                    if self._wanted_locked() and generation == self._generation:
                        self._state = "port_busy"
                        self._reset_stats_locked()
                        self._changed.wait(self.retry_s)
                continue
            with self._changed:
                self._binding = False
                self._socket = sock
                self._reset_stats_locked()
                if self._wanted_locked():
                    self._state = "none"
                self._changed.notify_all()
            # _listen closes the socket at once if a pause arrived meanwhile.
            self._listen(sock, generation)

    def _listen(self, sock: Any, generation: int) -> None:
        try:
            while True:
                with self._lock:
                    if not self._wanted_locked() or generation != self._generation:
                        return
                try:
                    data, source = sock.recvfrom(65535)
                except (socket.timeout, TimeoutError, BlockingIOError):
                    continue
                except OSError:
                    # A transient receive error (e.g. Windows ICMP port unreachable).
                    with self._lock:
                        if not self._wanted_locked():
                            return
                    continue
                self.observe(data, source, self._clock())
        finally:
            self._close_socket(sock)


def off_snapshot(*, bind: str = "0.0.0.0", port: int = 5555) -> dict[str, Any]:
    """The StreamSnapshot of a monitor that is not running."""

    return {
        "state": "off",
        "bind": bind,
        "port": int(port),
        "source": None,
        "rate_hz": None,
        "loss_fraction": None,
        "body_frames": 0,
        "joint_angles": False,
        "last_packet_age_s": None,
        "malformed": 0,
    }


__all__ = ["STREAM_STATES", "StreamMonitor", "default_socket_factory", "off_snapshot"]
