"""K1 head tracking (K6): the performer's head yaw and pitch on the K1 head.

**Math.**  ``R_rel = R_chest⁻¹ · R_head`` from the Xsens ``Chest`` (T8) and ``Head``
segments.  With ``R_rel0`` stored at the neutral calibration, the head rotation in
the chest frame is ``Q = R_rel · R_rel0ᵀ``, decomposed in the K1 joint order
``Q ≈ Rz(yaw) · Ry(pitch) · Rx(roll)``; roll is dropped (the K1 head has no roll).
No axis map is needed, and the K1 model proves it: MVN segment
frames are x forward, y left, z up in the calibration pose, the K1 trunk is the same
(the left arm sits at +y in ``K1_serial.xml``), and the K1 head chain is
``AAHead_yaw`` about z, then ``Head_pitch`` about y (positive pitch = looking down).

**Commander.**  Booster owns the head (our ``LowCmd`` keeps head kp = 0); it moves
through ``RotateHead(pitch, yaw)`` (pitch first), streamed at 50 Hz (a real K1
followed with about 40 ms lag; one jump stops at about 0.2 rad).  The commander has
its own thread and its own RPC backend (an isolated ``RpcWorker`` on the robot), so
a slow or hung head call never touches the arms.  One request in flight, the newest
target pending, no queue.  The arm limiter (``ArmSafetyEnvelope``, 2 rad/s) steps
from the last *confirmed* command; at the start and after a failed or late call it
starts again from the fresh measured head with a 0.5 rad/s ramp.  The arm loop
feeds a lease every tick; without one for 0.1 s, or while the arms do not follow,
the head holds its last command.  RPC errors for 0.5 s switch the head off for the
run; the arms go on.  ``inhibit()`` (non-blocking) comes before every arm release;
``finish()`` (after the release) sends the neutral ``RotateHeadWithTime(0, 0, 1000)``
only when asked to (normal end or STOP) and only if the head moved.  The real K1 refuses
it (code 400) right after the arm release, so a refused neutral is sent again every
0.3 s, at most 10 times.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
import itertools
import math
import sys
import threading
import time
from typing import Any

import numpy as np

from .k1_rpc_worker import Deadline, RpcResult
from .live_robots import K1_JOINT_RANGES, K1_JOINTS, K1_VELOCITY_LIMITS
from .safety import ArmSafetyEnvelope, SafetyConfig, SafetyError
from .schema import JointLimit

HEAD_JOINTS = K1_JOINTS[:2]  # ("AAHead_yaw", "Head_pitch"): JointIndexK1 motors 0 and 1
YAW, PITCH = HEAD_JOINTS
_RANGES = {name: (lower, upper) for name, lower, upper in K1_JOINT_RANGES}
_VELOCITY = dict(K1_VELOCITY_LIMITS)
#: The K1 head range (the decoder's check) and the clamp of every head target inside it.
HEAD_RANGES = {name: _RANGES[name] for name in HEAD_JOINTS}
MARGIN_RAD = 0.05
HEAD_CLAMP = {name: (lower + MARGIN_RAD, upper - MARGIN_RAD)
              for name, (lower, upper) in HEAD_RANGES.items()}
if HEAD_JOINTS != ("AAHead_yaw", "Head_pitch"):
    raise ImportError("K1 profile head joints changed: fix mocap_g1/k1_head.py")

RATE_HZ = 50.0
PERIOD_S = 1.0 / RATE_HZ
SPEED_RAD_S, ACCEL_RAD_S2 = 2.0, 10.0
START_SPEED_RAD_S = 0.5
#: The start ramp ends when the ramped target is this close to the performer's.
RAMPED_RAD = 0.02
LEASE_S = 0.1
STATE_FRESH_S = 0.1
OFF_AFTER_S = 0.5
#: One RotateHead: a result after this is thrown away (the worker kills a hung child).
ROTATE = Deadline(0.1, 0.1)
NEUTRAL = Deadline(0.5, 1.0)
NEUTRAL_MS = 1000
#: The real K1 refuses the neutral (code 400) right after the arm release: send it again.
NEUTRAL_TRIES, NEUTRAL_RETRY_S = 10, 0.3
HEAD_LIMITS = {name: JointLimit(lower, upper, _VELOCITY[name])
               for name, (lower, upper) in HEAD_RANGES.items()}
#: The arm envelope on the two head joints; the stall bound is off (the commander holds
#: on purpose) and ``max_step_dt_s`` still caps the motion credited to one step.
HEAD_SAFETY = SafetyConfig(position_margin_rad=MARGIN_RAD,
                           velocity_limit_fraction=SPEED_RAD_S / min(_VELOCITY[n] for n in HEAD_JOINTS),
                           acceleration_limit_rad_s2=ACCEL_RAD_S2, tick_stall_timeout_s=3600.0)


# -- math -------------------------------------------------------------------------


def quat_matrix(wxyz: Sequence[float]) -> np.ndarray:
    w, x, y, z = (float(v) for v in wxyz)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def relative(frame: Mapping[str, Any]) -> np.ndarray:
    """``R_chest⁻¹ · R_head`` of a GMR frame (``name -> (position, quaternion wxyz)``)."""
    return quat_matrix(frame["Chest"][1]).T @ quat_matrix(frame["Head"][1])


def head_angles(frame: Mapping[str, Any], rel0: np.ndarray) -> tuple[float, float]:
    """``(yaw, pitch)`` of ``Q = R_rel · R_rel0ᵀ = Rz(yaw) · Ry(pitch) · Rx(roll)``; roll dropped."""
    q = relative(frame) @ rel0.T
    return math.atan2(q[1, 0], q[0, 0]), math.atan2(-q[2, 0], math.hypot(q[0, 0], q[1, 0]))


def clamp(name: str, value: float) -> float:
    lower, upper = HEAD_CLAMP[name]
    return min(max(float(value), lower), upper)


class HeadReference:
    """The live tool's head: angles written into the reference ``qpos`` before the filter."""

    def __init__(self, dof_indices: Sequence[int]) -> None:
        self.indices = tuple(dof_indices)  # (yaw, pitch) in the robot's dof order
        self.rel0 = np.eye(3)

    def write(self, frame: Mapping[str, Any], qpos: np.ndarray) -> None:
        yaw, pitch = head_angles(frame, self.rel0)
        qpos[7 + self.indices[0]] = clamp(YAW, yaw)
        qpos[7 + self.indices[1]] = clamp(PITCH, pitch)


# -- practice robot and fakes ----------------------------------------------------------


class SimulatedHead:
    """The practice head: each command acts ``LAG_S`` later; one ``RotateHead`` moves at
    most ``JUMP_RAD`` from where the head is (as the real K1 did); ``RotateHeadWithTime``
    moves linearly to its goal.  Thread-safe."""

    LAG_S, JUMP_RAD = 0.04, 0.2

    def __init__(self, yaw: float, pitch: float, now: float) -> None:
        self._lock = threading.Lock()
        #: (begin, start, end, duration) per command, in time order.
        self._moves: list[tuple[float, tuple[float, float], tuple[float, float], float]] = [
            (now, (yaw, pitch), (yaw, pitch), 0.0)]

    def _at(self, t: float) -> tuple[float, float]:
        current = self._moves[0][1]
        for begin, start, end, duration in self._moves:
            if begin > t:
                break
            share = 1.0 if duration <= 0 else min(1.0, (t - begin) / duration)
            current = (start[0] + (end[0] - start[0]) * share, start[1] + (end[1] - start[1]) * share)
        return current

    def position(self, now: float) -> tuple[float, float]:
        with self._lock:
            return self._at(now)

    def rotate(self, yaw: float, pitch: float, now: float, duration_s: float = 0.0) -> None:
        with self._lock:
            begin = now + self.LAG_S
            start = self._at(begin)
            end = (yaw, pitch) if duration_s > 0 else tuple(
                s + min(max(g - s, -self.JUMP_RAD), self.JUMP_RAD) for s, g in zip(start, (yaw, pitch)))
            self._moves.append((begin, start, (end[0], end[1]), duration_s))
            while len(self._moves) > 1 and self._moves[1][0] <= now - 1.0:
                del self._moves[0]


def apply_head_call(head: SimulatedHead, name: str, args: Sequence[Any], now: float) -> None:
    """One SDK head call (``RotateHead(pitch, yaw)``, ``RotateHeadWithTime(pitch, yaw, ms)``)."""
    if name == "RotateHead":
        head.rotate(float(args[1]), float(args[0]), now)
    elif name == "RotateHeadWithTime":
        head.rotate(float(args[1]), float(args[0]), now, float(args[2]) / 1000.0)
    else:
        raise ValueError(f"not a head call: {name}")


class SimulatedHeadRpc:
    """The ``RpcWorker`` interface on a :class:`SimulatedHead`: every call succeeds at once.
    The practice robot's head backend: practice never opens an SDK connection."""

    def __init__(self, head: SimulatedHead, clock: Callable[[], float]) -> None:
        self.head, self.clock = head, clock
        self._ids = itertools.count(1)
        self._results: dict[int, RpcResult] = {}

    def submit(self, name: str, args: Sequence[Any] = (), *, deadline: Deadline,
               preempt: bool = False) -> int:
        now = self.clock()
        apply_head_call(self.head, name, args, now)
        request = next(self._ids)
        self._results[request] = RpcResult(request, name, True, attempts=1, submitted_at=now,
                                           finished_at=now)
        return request

    def poll(self, request_id: int) -> RpcResult | None:
        return self._results.get(request_id)

    def result(self, request_id: int, timeout: float) -> RpcResult | None:
        return self._results.get(request_id)

    def forget(self, request_id: int) -> None:
        self._results.pop(request_id, None)

    def call(self, name: str, args: Sequence[Any] = (), *, deadline: Deadline,
             preempt: bool = False) -> RpcResult:
        request = self.submit(name, args, deadline=deadline)
        return self._results.pop(request)

    def close(self) -> None:
        pass


# -- commander ------------------------------------------------------------------------


class HeadCommander:
    """See the module docstring.  ``tick(now)`` is one 50 Hz step; ``start()`` runs it on
    a daemon thread (a virtual-clock run ticks it from the arm loop instead)."""

    def __init__(self, rpc: Any, state: Callable[[], Any], *,
                 clock: Callable[[], float] = time.monotonic,
                 sleeper: Callable[[float], None] = time.sleep) -> None:
        self.rpc, self.state, self.clock, self.sleeper = rpc, state, clock, sleeper
        self._lock = threading.Lock()
        self._inhibit = threading.Event()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._feed: tuple[float, tuple[float, float] | None, bool] | None = None
        self._pending: tuple[int, dict[str, float], float] | None = None
        self._abandoned: list[int] = []
        self.envelope = ArmSafetyEnvelope(HEAD_LIMITS, HEAD_SAFETY, joints=HEAD_JOINTS)
        self._ramp: dict[str, float] | None = None
        self._last_step = -math.inf
        self._failing_since: float | None = None
        self._holding = True
        #: The last confirmed, the newest sent command, and the clamped performer target.
        self.confirmed: dict[str, float] | None = None
        self.command: dict[str, float] | None = None
        self.target: dict[str, float] | None = None
        self.off = False
        self.moved = False
        self.sent = self.confirmations = self.failures = 0
        self.events: list[tuple[float, str]] = []

    # -- arm-loop side (never blocks) ---------------------------------------------

    def feed(self, now: float, target: Sequence[float] | None, *, following: bool) -> None:
        """The lease: this tick's ``(yaw, pitch)`` target (``None``: a v1 packet) and
        whether the arms follow now."""
        with self._lock:
            self._feed = (now, None if target is None else (float(target[0]), float(target[1])),
                          bool(following))

    def inhibit(self) -> None:
        """No head command from here on (before every arm release)."""
        with self._lock:  # held only around a non-blocking submit
            self._inhibit.set()

    @property
    def status(self) -> str:
        if self.off:
            return "off"
        if self._inhibit.is_set():
            return "stopped"
        if not self.moved:
            return "waiting"
        return "holding" if self._holding else "following"

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="k1-head", daemon=True)
        self._thread.start()

    # -- head thread ------------------------------------------------------------------

    def _run(self) -> None:
        next_at = self.clock()
        while not self._stop.is_set():
            try:
                self.tick(self.clock())
            except Exception as exc:  # noqa: BLE001 - a head bug is not an arm stop
                self._switch_off(self.clock(), f"head commander error: {exc}")
                return
            next_at += PERIOD_S
            delay = next_at - self.clock()
            if delay < 0:
                next_at, delay = self.clock(), 0.0
            self._stop.wait(delay)

    def tick(self, now: float) -> None:
        if self._inhibit.is_set() or self.off:
            return
        self._drop_abandoned()
        if self._pending is not None and not self._settle(now):
            return  # one request in flight; the newest target waits in the feed
        if self.off:
            return
        with self._lock:
            feed = self._feed
        if feed is None or feed[1] is None or not feed[2] or now - feed[0] > LEASE_S:
            self._holding = True  # stale lease, arms not following, or a v1 stream
            return
        if not getattr(self.rpc, "ready", True):
            self._holding = True  # its worker is still starting: not an error
            return
        self._holding = False
        command = self._next(now, feed[1])
        if command is None:
            return
        try:
            with self._lock:
                if self._inhibit.is_set():
                    return
                request = self.rpc.submit("RotateHead", (command[PITCH], command[YAW]),
                                          deadline=ROTATE)
        except Exception as exc:  # noqa: BLE001 - RpcClosed or a backend error
            self._failed(now, f"RotateHead not sent: {exc}")
            return
        self._pending = (request, command, now)
        self.command, self.moved = command, True
        self.sent += 1

    def _settle(self, now: float) -> bool:
        """Read the request in flight; ``True`` once nothing is in flight."""
        assert self._pending is not None
        request, command, sent_at = self._pending
        result = self.rpc.poll(request)
        if result is None:
            if now - sent_at <= ROTATE.total_s + 2 * PERIOD_S:
                return False
            self._abandoned.append(request)  # its late result is thrown away
            self._pending = None
            self._failed(now, "RotateHead: no result within the deadline")
            return True
        self._pending = None
        self.rpc.forget(request)
        if result.ok and result.finished_at - result.submitted_at <= ROTATE.total_s:
            self.confirmed, self._failing_since = command, None
            self.confirmations += 1
        else:
            self._failed(now, result.error or "RotateHead: late result thrown away")
        return True

    def _drop_abandoned(self) -> None:
        for request in [r for r in self._abandoned if self.rpc.poll(r) is not None]:
            self.rpc.forget(request)
            self._abandoned.remove(request)

    def _failed(self, now: float, why: str) -> None:
        self.failures += 1
        self.confirmed = None  # the next command ramps from the fresh measured head
        if self._failing_since is None:
            self._failing_since = now
        self.event(now, f"head RPC failed: {why}")
        if now - self._failing_since >= OFF_AFTER_S:
            self._switch_off(now, "head RPC errors for 0.5 s")

    def _switch_off(self, now: float, why: str) -> None:
        self.off = True
        self.event(now, f"head off for this run ({why}); the arms continue")
        print(f"HEAD OFF: {why}; the arms continue", file=sys.stderr, flush=True)

    def measured(self, now: float) -> dict[str, float] | None:
        """The fresh measured head, or ``None``."""
        state = self.state()
        if state is None or not -0.005 <= now - state.receive_time_s < STATE_FRESH_S:
            return None
        head = state.head_positions()
        if not all(math.isfinite(value) for value in head.values()):
            return None
        return head

    def _next(self, now: float, target: tuple[float, float]) -> dict[str, float] | None:
        goal = {YAW: clamp(YAW, target[0]), PITCH: clamp(PITCH, target[1])}
        self.target = goal
        try:
            if self.confirmed is None:  # start, or after a failed or late call
                measured = self.measured(now)
                if measured is None:
                    return None
                start = {name: min(max(measured[name], HEAD_RANGES[name][0]), HEAD_RANGES[name][1])
                         for name in HEAD_JOINTS}
                self._last_step = now - PERIOD_S
                self.envelope.reset(start, now=self._last_step)
                self._ramp = dict(start)
            elif now - self._last_step > LEASE_S:  # after a hold: from the held command, at rest
                self.envelope.reset(self.confirmed, now=self._last_step)
            aim = goal
            if self._ramp is not None:
                step = START_SPEED_RAD_S * min(now - self._last_step, HEAD_SAFETY.max_step_dt_s)
                self._ramp = {name: value + min(max(goal[name] - value, -step), step)
                              for name, value in self._ramp.items()}
                aim = self._ramp
                if all(abs(goal[name] - self._ramp[name]) < RAMPED_RAD for name in HEAD_JOINTS):
                    self._ramp = None
            command = self.envelope.step(aim, source_timestamp=now, now=now, enable=True)
        except SafetyError as exc:
            self.event(now, f"head limiter: {exc}")
            self.confirmed = None
            return None
        self._last_step = now
        return dict(command.positions)

    def event(self, now: float, text: str) -> None:
        self.events.append((round(now, 4), text))

    # -- after the arm release -----------------------------------------------------------

    def finish(self, *, neutral: bool) -> None:
        """Stop, send the neutral if asked (and the head moved and its state is known), close."""
        self._inhibit.set()
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        now = self.clock()
        if not self.moved:
            pass
        elif not neutral:
            self.event(now, "no head neutral (the run did not end normally)")
        elif self.off or self.measured(now) is None:
            self.event(now, "no head neutral (head state unknown)")
        else:
            self.event(now, "head neutral requested: RotateHeadWithTime(0, 0, 1000)")
            for attempt in range(1, NEUTRAL_TRIES + 1):
                try:
                    result = self.rpc.call("RotateHeadWithTime", (0.0, 0.0, NEUTRAL_MS),
                                           deadline=NEUTRAL)
                    ok, why, again = result.ok, result.error, not result.timeout
                except Exception as exc:  # noqa: BLE001 - logged; the arms are already released
                    ok, why, again = False, str(exc), False
                if ok or not again or attempt == NEUTRAL_TRIES:
                    break
                self.sleeper(NEUTRAL_RETRY_S)  # a refusal: the robot answered, so ask again
            self.event(self.clock(), f"head neutral confirmed (RPC returned OK, attempt {attempt})"
                       if ok else f"head neutral failed after {attempt} attempt(s): {why}")
        try:
            self.rpc.close()
        except Exception as exc:  # noqa: BLE001
            self.event(self.clock(), f"head RPC close failed: {exc}")

    def summary(self) -> dict[str, Any]:
        return {"state": self.status, "moved": self.moved, "sent": self.sent,
                "confirmed": self.confirmations, "failures": self.failures, "off": self.off,
                "events": [f"{t:.3f} {text}" for t, text in self.events[-20:]]}


def inhibit_on_release(transport: Any, commander: HeadCommander) -> None:
    """Make every arm release (``upper_body_control(False)``) inhibit the head first."""
    release = transport.upper_body_control

    def guarded(enable: bool) -> None:
        if not enable:
            commander.inhibit()
        release(enable)

    transport.upper_body_control = guarded
