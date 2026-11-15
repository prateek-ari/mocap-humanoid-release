"""Booster K1 arms through the SDK's upper-body custom control, behind the safety envelope.

The K1's arm seam (Booster SDK ``booster_robotics_sdk`` @87a9a26, firmware 1.8.0;
Python bindings ``booster_robotics_sdk_python`` 1.6.3, inspected 2026-10-26):
``B1LocoClient.UpperBodyCustomControl(True)`` hands the arms to this publisher,
whose ``LowCmd`` on ``rt/joint_ctrl`` then carries all 22 motors in
``JointIndexK1`` order (equal to ``mocap_g1.live_robots.K1_JOINTS``, asserted
below).  The command contract copies the SDK's own ``ArmController``: every motor
``mode = 0x0A``, ``dq = 0``, ``tau = 0``; the eight arm motors (indices 2-9)
carry a position and kp/kd; the head and legs ``q = 0, kp = 0, kd = 0`` so
Booster's controller keeps them.  ``weight`` and ``cmd_type`` stay at the
binding defaults (0 and PARALLEL), as ``ArmController`` leaves them: their
meaning is undocumented, so nothing here relies on them.  Measured joint
positions come from ``LowState.motor_state_parallel``, as ``ArmController``
reads them.  ``UpperBodyCustomControl(False)`` hands the arms back.

Unlike the G1 bridge (``mocap_g1/arm_bridge.py``) there is no authority weight:
the arms are this publisher's from takeover to release.  A smooth start comes
from the envelope instead: it is reset from a joint state measured *after* the
takeover returned, and its positions are speed- and acceleration-bounded from
there.  A normal stop decelerates the arms to rest, then releases; a fault stops
publishing and releases at once.  A release that cannot be confirmed latches
``release_unconfirmed``: no rearm, and the operator must stop the robot by hand.

UNVERIFIED UNTIL HARDWARE (docs/k1-bringup-runbook.md): the robot mode
``UpperBodyCustomControl`` needs, whether Booster balances while it is on, the
meaning of ``mode = 0x0A``, ``weight`` and ``cmd_type``, whether ``rt/fall_down``
is periodic, the robot's behaviour when commands stop, joint signs and zero
offsets, and the gains.  The real transport therefore runs only with an
acceptance record from the hoisted day-one procedure (:class:`AcceptanceRecord`).
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections import deque
from collections.abc import Callable, Iterator, Mapping, Sequence
from contextlib import contextmanager
from dataclasses import dataclass, field, replace
import json
import math
import os
from pathlib import Path
import threading
import time
from typing import Any, Literal, get_args

import numpy as np

from .k1_head import HEAD_JOINTS, SimulatedHead, SimulatedHeadRpc
from .k1_rpc_worker import DAMPING, MODE_QUERY, ROBOT_INFO, TAKEOVER, Deadline, RpcClosed, RpcWorker
from .live_robots import K1_JOINTS, K1_UPPER_BODY_JOINTS
from .safety import ArmSafetyEnvelope, SafetyCommand, SafetyConfig, SafetyError
from .schema import JointLimit

#: Bumped whenever the published frame or the lifecycle changes; an acceptance
#: record approves one contract.
COMMAND_CONTRACT = "k1-upper-body/1"
JOINT_CTRL_TOPIC = "rt/joint_ctrl"
LOW_STATE_TOPIC = "rt/low_state"
FALL_DOWN_TOPIC = "rt/fall_down"
MOTOR_COUNT = 22
#: ``JointIndexK1`` of the SDK: head yaw/pitch 0-1, left arm 2-5, right arm 6-9, legs 10-21.
ARM_MOTOR_INDEX = {name: index for index, name in enumerate(K1_JOINTS) if name in K1_UPPER_BODY_JOINTS}
HEAD_MOTOR_INDEX = {name: K1_JOINTS.index(name) for name in HEAD_JOINTS}
MOTOR_MODE = 0x0A
#: ``ArmController``'s gains: the ceiling unless ``allow_gains_above_example``.
EXAMPLE_KP, EXAMPLE_KD = 60.0, 3.0
#: ``RobotMode`` values of the SDK, and the fall state that allows motion.
ROBOT_MODES = {"damping": 0, "prepare": 1, "walking": 2, "custom": 3, "soccer": 4}
FALL_READY = 0
#: Pause between release retries. A K1 on firmware 1.8.1.2 refused the release 6 ms after the
#: takeover; Booster's ArmController waits 0.5 s after enabling and retries the release 1 s apart.
RELEASE_RETRY_S = 0.5
TRACKING_LAG_S = 0.5  # owner 2026-11-15: a late arm is fine if it mimics; see tracking_errors
LEG_START = 10
#: How far a measured arm joint may sit past its range end at takeover.
RANGE_TOLERANCE_RAD = 0.02
BINDING_PACKAGE = "booster_robotics_sdk_python"
#: One process may command a K1 from this PC at a time (it cannot see other PCs).
MOTION_LOCK_PATH = Path("/tmp/k1-motion.lock")

if len(K1_JOINTS) != MOTOR_COUNT or list(ARM_MOTOR_INDEX.values()) != list(range(2, 10)):
    raise ImportError("K1 profile joint order no longer matches the SDK's JointIndexK1")

STATES = ("idle", "tracking", "released", "fault_latched", "release_unconfirmed")
#: Where a confirmed release may leave the bridge; mypy refuses any other string.
ReleasedState = Literal["idle", "released", "fault_latched"]
if not set(get_args(ReleasedState)) <= set(STATES):
    raise ImportError("ReleasedState names a state outside STATES")


class K1BridgeError(RuntimeError):
    """The bridge refused to start, or could not take over the arms."""


class LocoError(RuntimeError):
    """A loco RPC failed; ``timeout`` when the SDK reported code 100 or a deadline
    passed; ``final`` when the action's time budget is spent (retrying is pointless)."""

    def __init__(self, message: str, *, timeout: bool = False, final: bool = False) -> None:
        super().__init__(message)
        self.timeout = timeout
        self.final = final


@dataclass(frozen=True)
class MotorCommand:
    q: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    mode: int = MOTOR_MODE
    dq: float = 0.0
    tau: float = 0.0


@dataclass(frozen=True)
class K1Frame:
    """One ``LowCmd``: 22 motors in ``JointIndexK1`` order."""

    motors: tuple[MotorCommand, ...]

    def arm_positions(self) -> dict[str, float]:
        return {name: self.motors[index].q for name, index in ARM_MOTOR_INDEX.items()}


@dataclass(frozen=True)
class K1State:
    """Measured joint positions (``motor_state_parallel[i].q``) and when they arrived."""

    positions: tuple[float, ...]
    receive_time_s: float

    def arm_positions(self) -> dict[str, float]:
        return {name: self.positions[index] for name, index in ARM_MOTOR_INDEX.items()}

    def head_positions(self) -> dict[str, float]:
        """Head yaw and pitch (motors 0 and 1), moved by Booster's RotateHead."""
        return {name: self.positions[index] for name, index in HEAD_MOTOR_INDEX.items()}


@dataclass(frozen=True)
class Reading:
    """A robot mode or fall state and when it was read."""

    value: int
    time_s: float


def measured_state(positions: Sequence[float], receive_time_s: float) -> K1State | None:
    """A ``K1State`` from 22 joint positions, or ``None`` when malformed."""
    values = tuple(float(value) for value in positions)
    if len(values) != MOTOR_COUNT or not all(math.isfinite(value) for value in values):
        return None
    return K1State(values, float(receive_time_s))


def joint_state(parallel: Sequence[float], serial: Sequence[float],
                receive_time_s: float) -> K1State | None:
    """Head and arms from ``motor_state_parallel`` (as ``ArmController`` reads
    them), legs from ``motor_state_serial``: in parallel order indices 14/15 and
    20/21 are the ankle crank motors, not ankle pitch and roll."""
    if len(parallel) != MOTOR_COUNT or len(serial) != MOTOR_COUNT:
        return None
    return measured_state([*parallel[:LEG_START], *serial[LEG_START:]], receive_time_s)


class K1Transport(ABC):
    """Moves frames and readings; the bridge decides everything else."""

    @abstractmethod
    def publish(self, frame: K1Frame) -> None: ...

    @abstractmethod
    def latest_state(self) -> K1State | None: ...

    @abstractmethod
    def latest_mode(self) -> Reading | None: ...

    @abstractmethod
    def latest_fall(self) -> Reading | None: ...

    @abstractmethod
    def upper_body_control(self, enable: bool) -> None:
        """``UpperBodyCustomControl``; raises :class:`LocoError` when it fails."""

    def close(self) -> None:
        pass


class FakeK1Transport(K1Transport):
    """Deterministic in-process stand-in; ``rpc`` scripts the loco RPC outcomes.

    With ``streaming_clock`` it behaves like a robot that streams state: every
    reading is timestamped now, the arm encoders follow the published commands
    and the head follows ``RotateHead`` calls of :meth:`head_rpc` like a real K1
    (:class:`~mocap_g1.k1_head.SimulatedHead`: 40 ms lag, one jump stops at
    0.2 rad).  Without it, readings keep the time they were fed.
    """

    def __init__(self, *, rpc: Callable[[bool], None] | None = None,
                 streaming_clock: Callable[[], float] | None = None) -> None:
        self.published: list[K1Frame] = []
        self.rpc_calls: list[bool] = []
        self.state: K1State | None = None
        self.mode: Reading | None = None
        self.fall: Reading | None = None
        self.rpc = rpc
        self._stream = streaming_clock
        self.head: SimulatedHead | None = None

    def head_rpc(self) -> SimulatedHeadRpc | None:
        """The head's RPC backend (in-process, no SDK); ``None`` without a streaming clock."""
        if self._stream is None or self.head is None:
            return None
        return SimulatedHeadRpc(self.head, self._stream)

    def publish(self, frame: K1Frame) -> None:
        self.published.append(frame)
        if self._stream is not None and self.state is not None:
            positions = list(self.state.positions)
            for index in ARM_MOTOR_INDEX.values():
                positions[index] = frame.motors[index].q
            self.state = K1State(tuple(positions), self.state.receive_time_s)

    def latest_state(self) -> K1State | None:
        if self._stream is not None and self.state is not None:
            now = self._stream()
            positions = self.state.positions
            if self.head is not None:
                positions = (*self.head.position(now), *positions[2:])
            return K1State(positions, now)
        return self.state

    def latest_mode(self) -> Reading | None:
        if self._stream is not None and self.mode is not None:
            return Reading(self.mode.value, self._stream())
        return self.mode

    def latest_fall(self) -> Reading | None:
        if self._stream is not None and self.fall is not None:
            return Reading(self.fall.value, self._stream())
        return self.fall

    def upper_body_control(self, enable: bool) -> None:
        self.rpc_calls.append(enable)
        if self.rpc is not None:
            self.rpc(enable)

    def feed(self, positions: Sequence[float], now: float, *, mode: int = ROBOT_MODES["walking"],
             fall: int = FALL_READY) -> None:
        """Deliver one joint state, mode and fall reading at ``now``."""
        self.state = measured_state(positions, now)
        if self._stream is not None and self.head is None and self.state is not None:
            self.head = SimulatedHead(self.state.positions[0], self.state.positions[1], now)
        self.mode = Reading(mode, now)
        self.fall = Reading(fall, now)


class BoosterTransport(K1Transport):
    """The Booster SDK over DDS.  Every loco RPC (mode polling, takeover,
    release, robot info) goes through one client under one lock, so they are
    strictly serialized."""

    MODE_POLL_S = 0.5

    def __init__(self, *, domain_id: int = 0, network_interface: str = "",
                 clock: Callable[[], float] = time.monotonic) -> None:
        try:
            import booster_robotics_sdk_python as sdk
        except ImportError as exc:  # the optional "booster" extra
            raise K1BridgeError(
                f"{BINDING_PACKAGE} is not installed: python3 -m pip install -e '.[booster]'"
            ) from exc
        self._sdk = sdk
        self._clock = clock
        self._lock = threading.Lock()
        self._rpc_lock = threading.Lock()
        self._state: K1State | None = None
        self._mode: Reading | None = None
        self._fall: Reading | None = None
        self.malformed_states = 0
        sdk.ChannelFactory.Instance().Init(int(domain_id), network_interface)
        self._start_loco(int(domain_id), network_interface)
        self._publisher = sdk.B1LowCmdPublisher()
        self._publisher.InitChannel()
        self._state_subscriber = sdk.B1LowStateSubscriber(self._on_low_state)
        self._state_subscriber.InitChannel()
        self._fall_subscriber = sdk.B1FallDownStateSubscriber(self._on_fall)
        self._fall_subscriber.InitChannel()
        self._stop = threading.Event()
        self._poller = threading.Thread(target=self._poll_mode, name="k1-mode", daemon=True)
        self._poller.start()

    def _start_loco(self, domain_id: int, network_interface: str) -> None:
        self._client = self._sdk.B1LocoClient()
        self._client.Init()

    def _rpc(self, call: Callable[[], Any], what: str) -> Any:
        with self._rpc_lock:
            try:
                return call()
            except RuntimeError as exc:
                raise LocoError(f"{what} failed: {exc}", timeout="code = 100" in str(exc)) from exc

    def _poll_mode(self) -> None:
        while True:  # the first reading at once, then every MODE_POLL_S
            try:
                mode = int(self._rpc(self._client.GetMode, "GetMode").mode)
            except (LocoError, AttributeError, TypeError, ValueError):
                pass  # the reading goes stale and the bridge stops
            else:
                with self._lock:
                    self._mode = Reading(mode, self._clock())
            if self._stop.wait(self.MODE_POLL_S):
                return

    def _on_low_state(self, message: Any) -> None:
        now = self._clock()
        try:
            state = joint_state([motor.q for motor in message.motor_state_parallel],
                                [motor.q for motor in message.motor_state_serial], now)
        except (AttributeError, TypeError, ValueError):
            state = None
        with self._lock:
            if state is None:
                self.malformed_states += 1
            else:
                self._state = state

    def _on_fall(self, message: Any) -> None:
        now = self._clock()
        with self._lock:
            self._fall = Reading(int(message.fall_down_state), now)

    def publish(self, frame: K1Frame) -> None:
        command = self._sdk.LowCmd()
        motors = []
        for motor in frame.motors:
            sdk_motor = self._sdk.MotorCmd()  # weight stays at the binding default
            sdk_motor.mode, sdk_motor.q, sdk_motor.dq = motor.mode, motor.q, motor.dq
            sdk_motor.tau, sdk_motor.kp, sdk_motor.kd = motor.tau, motor.kp, motor.kd
            motors.append(sdk_motor)
        command.motor_cmd = motors  # cmd_type stays at the binding default
        if not self._publisher.Write(command):
            raise K1BridgeError("LowCmd publish failed")

    def latest_state(self) -> K1State | None:
        with self._lock:
            return self._state

    def latest_mode(self) -> Reading | None:
        with self._lock:
            return self._mode

    def latest_fall(self) -> Reading | None:
        with self._lock:
            return self._fall

    def upper_body_control(self, enable: bool) -> None:
        self._rpc(lambda: self._client.UpperBodyCustomControl(bool(enable)),
                  f"UpperBodyCustomControl({enable})")

    def robot_info(self) -> dict[str, str]:
        info = self._rpc(self._client.GetRobotInfo, "GetRobotInfo")
        return {key: str(getattr(info, key)) for key in ("model", "name", "serial_number", "version")}

    def close(self) -> None:
        self._stop.set()
        self._poller.join(timeout=2.0)
        for channel in (self._state_subscriber, self._fall_subscriber, self._publisher):
            channel.CloseChannel()


@dataclass
class ActionStatus:
    """When an RPC action was issued and when the robot confirmed it:
    separate states in the log and the window)."""

    requested_at: float | None = None
    confirmed_at: float | None = None
    #: The latest failed attempt; a later confirmation still wins.
    failure: str | None = None

    @property
    def state(self) -> str:
        if self.confirmed_at is not None:
            return "confirmed"
        if self.failure is not None:
            return "failed"
        return "none" if self.requested_at is None else "requested"


class IsolatedBoosterTransport(BoosterTransport):
    """Pub/sub (low state, fall, low cmd) in this process; every loco RPC in an
    isolated child process with a deadline (:mod:`mocap_g1.k1_rpc_worker`).  Same :class:`K1Transport` interface, so :class:`K1ArmBridge` is
    unchanged; :class:`BoosterTransport` stays for the commissioning tool.

    Mode polling never blocks a control tick: it runs on its own thread and a
    missed query (0.3 s) leaves the reading to age into the bridge's
    ``mode_unknown`` fault.  A release is one episode of ``RELEASE.total_s``
    from its first request after a takeover: the bridge's retries share that
    budget; a failure once it is spent is ``final``, so the bridge stops retrying
    and latches ``release_unconfirmed`` (by the budget plus at most one 0.2 s
    retry pause that began inside it).
    """

    RELEASE = Deadline(0.9, 2.0)

    def _start_loco(self, domain_id: int, network_interface: str) -> None:
        self._loco_args = (domain_id, network_interface)
        self.rpc = RpcWorker(domain_id=domain_id, network_interface=network_interface,
                             clock=self._clock)
        self._release_end: float | None = None
        self.actions = {"release": ActionStatus(), "damping": ActionStatus()}
        #: (time, text) of every issued, confirmed and failed action, for the log.
        self.events: deque[tuple[float, str]] = deque(maxlen=256)

    def _loco(self, name: str, args: tuple[Any, ...], deadline: Deadline, *,
              preempt: bool = False) -> Any:
        try:
            result = self.rpc.call(name, args, deadline=deadline, preempt=preempt)
        except RpcClosed as exc:
            raise LocoError(f"{name} failed: {exc}") from exc
        if not result.ok:
            raise LocoError(f"{name} failed: {result.error}", timeout=result.timeout)
        return result.value

    def _poll_mode(self) -> None:
        while True:  # the first reading at once, then every MODE_POLL_S
            try:
                mode = int(self._loco("GetMode", (), MODE_QUERY))
            except (LocoError, TypeError, ValueError):
                pass  # the reading goes stale and the bridge stops
            else:
                with self._lock:
                    self._mode = Reading(mode, self._clock())
            if self._stop.wait(self.MODE_POLL_S):
                return

    def _event(self, text: str) -> float:
        now = self._clock()
        self.events.append((now, text))
        return now

    def upper_body_control(self, enable: bool) -> None:
        if enable:
            self._release_end = None
            self._loco("UpperBodyCustomControl", (True,), TAKEOVER)
            return
        if self._release_end is None:
            now = self._event("release requested")
            self._release_end = now + self.RELEASE.total_s
            self.actions["release"] = ActionStatus(requested_at=now)
        status = self.actions["release"]
        remaining = self._release_end - self._clock()
        try:
            if remaining <= 0:
                raise LocoError("release deadline spent", timeout=True, final=True)
            self._loco("UpperBodyCustomControl", (False,),
                       Deadline(min(self.RELEASE.attempt_s, remaining), remaining), preempt=True)
        except LocoError as exc:
            if self._clock() >= self._release_end:
                exc.final = True
            status.failure = str(exc)
            self._event(f"release attempt failed: {exc}")
            raise
        status.confirmed_at = self._event("release confirmed (RPC returned OK)")
        self._release_end = None

    def damping(self) -> None:
        """``ChangeMode(kDamping)`` within ``DAMPING``; confirmed = the RPC returned OK."""
        status = self.actions["damping"] = ActionStatus(requested_at=self._event("damping requested"))
        try:
            self._loco("ChangeMode", ("damping",), DAMPING, preempt=True)
        except LocoError as exc:
            status.failure = str(exc)
            self._event(f"damping failed: {exc}")
            raise
        status.confirmed_at = self._event("damping confirmed (RPC returned OK)")

    def action_status(self, name: str) -> ActionStatus:
        """A copy of ``"release"`` or ``"damping"`` status."""
        return replace(self.actions[name])

    def robot_info(self) -> dict[str, str]:
        return dict(self._loco("GetRobotInfo", (), ROBOT_INFO))

    def head_rpc(self) -> RpcWorker:
        """A second isolated worker for the head (K6): a slow head call never waits in
        front of a release.  The head commander owns and closes it."""
        domain_id, network_interface = self._loco_args
        return RpcWorker(domain_id=domain_id, network_interface=network_interface, clock=self._clock)

    def close(self) -> None:
        super().close()
        self.rpc.close()


def binding_version() -> str | None:
    """Installed version of the SDK's Python bindings, or ``None``."""
    from importlib.metadata import PackageNotFoundError, version

    try:
        return version(BINDING_PACKAGE)
    except PackageNotFoundError:
        return None


@dataclass(frozen=True)
class K1BridgeConfig:
    """Commissioning settings.  The defaults are **unvalidated starting values**."""

    kp: float = 30.0
    kd: float = 1.5
    allow_gains_above_example: bool = False
    #: Arm speed and acceleration caps (rad/s, rad/s^2), separate from the model limits.
    speed_max_rad_s: float = 1.0
    accel_max_rad_s2: float = 2.0
    #: Robot modes (``ROBOT_MODES`` values) that allow takeover and tracking.
    allowed_modes: frozenset[int] = frozenset({ROBOT_MODES["walking"]})
    mode_timeout_s: float = 1.5
    #: Newest fall reading must be younger than this (3 x the measured period).
    fall_timeout_s: float = 1.5
    #: A joint state older than this latches a fault (it binds the contact check too).
    state_stale_timeout_s: float = 0.10
    #: Wait for a joint state received after the takeover RPC returned.
    takeover_state_timeout_s: float = 0.50
    tracking_error_limit_rad: float = 0.30
    release_attempts: int = 4  # 0.5 s apart: a K1 (1.8.1.2) took 1.5 s after a takeover to accept the release
    #: Longest normal-stop deceleration before the release.
    stop_timeout_s: float = 3.0

    def validate(self) -> None:
        for name in ("kp", "kd", "speed_max_rad_s", "accel_max_rad_s2", "mode_timeout_s",
                     "fall_timeout_s", "state_stale_timeout_s",
                     "takeover_state_timeout_s", "tracking_error_limit_rad", "stop_timeout_s"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) \
                    or not math.isfinite(value) or value < 0:
                raise K1BridgeError(f"{name} must be a finite, non-negative number")
        if not self.allow_gains_above_example and (self.kp > EXAMPLE_KP or self.kd > EXAMPLE_KD):
            raise K1BridgeError(
                f"kp={self.kp:g}/kd={self.kd:g} exceed ArmController's gains "
                f"(kp<={EXAMPLE_KP:g}, kd<={EXAMPLE_KD:g})"
            )
        if self.speed_max_rad_s <= 0 or self.accel_max_rad_s2 <= 0:
            raise K1BridgeError("speed and acceleration caps must be positive")
        if self.state_stale_timeout_s <= 0:
            raise K1BridgeError("state_stale_timeout_s must be positive")
        if not self.allowed_modes or not set(self.allowed_modes) <= set(ROBOT_MODES.values()):
            raise K1BridgeError("allowed_modes must name SDK robot modes")
        if not 1 <= self.release_attempts <= 10:
            raise K1BridgeError("release_attempts must be 1-10")

    def safety_config(self, limits: Mapping[str, JointLimit]) -> SafetyConfig:
        """The envelope settings that cap every arm joint at ``speed_max_rad_s``."""
        slowest = min(limit.velocity for limit in limits.values())
        return SafetyConfig(velocity_limit_fraction=min(1.0, self.speed_max_rad_s / slowest),
                            acceleration_limit_rad_s2=self.accel_max_rad_s2)


@dataclass(frozen=True)
class K1Step:
    """What one control tick did."""

    frame: K1Frame | None
    reason: str
    state: str
    state_age_s: float
    command: SafetyCommand | None = None


@dataclass
class K1ArmBridge:
    """Drive the K1 arms behind :class:`ArmSafetyEnvelope`; see the module docstring."""

    transport: K1Transport
    limits: Mapping[str, JointLimit]
    config: K1BridgeConfig = field(default_factory=K1BridgeConfig)
    #: ``mocap_g1.arm_contact.ArmContact`` for the K1, bound to the measured pose each tick.
    contact: Any = None
    clock: Callable[[], float] = time.monotonic
    sleeper: Callable[[float], None] = time.sleep

    def __post_init__(self) -> None:
        self.config.validate()
        self.limits = {name: self.limits[name] for name in K1_UPPER_BODY_JOINTS}
        self.envelope = ArmSafetyEnvelope(self.limits, self.config.safety_config(self.limits),
                                          joints=K1_UPPER_BODY_JOINTS)
        self.state = "idle"
        self.fault: str | None = None
        self._last_positions: dict[str, float] | None = None
        #: (publish time, arm positions) of published commands, for the tracking check.
        self._history: deque[tuple[float, dict[str, float]]] = deque(maxlen=64)

    # -- checks -------------------------------------------------------------

    def _fresh_state(self, now: float) -> tuple[K1State | None, float]:
        state = self.transport.latest_state()
        if state is None:
            return None, math.inf
        return state, now - state.receive_time_s

    def _robot_ready(self, now: float) -> str | None:
        """Why the mode or fall reading forbids motion now, or ``None``."""
        mode, fall = self.transport.latest_mode(), self.transport.latest_fall()
        if mode is None or now - mode.time_s > self.config.mode_timeout_s:
            return "mode_unknown"
        if mode.value not in self.config.allowed_modes:
            return f"mode_not_allowed:{mode.value}"
        if fall is None or now - fall.time_s > self.config.fall_timeout_s:
            return "fall_state_unknown"
        if fall.value != FALL_READY:
            return f"fall_state:{fall.value}"
        return None

    # -- lifecycle ------------------------------------------------------------

    def takeover(self) -> K1State:
        """Hand the arms to this publisher and reset the envelope from them."""
        if self.state != "idle":
            raise K1BridgeError(f"cannot take over from state {self.state}")
        now = self.clock()
        state, age = self._fresh_state(now)
        if state is None or not 0 <= age < self.config.state_stale_timeout_s:
            raise K1BridgeError("cannot take over: no fresh joint state")
        problem = self._robot_ready(now)
        if problem is not None:
            raise K1BridgeError(f"cannot take over: {problem}")
        try:
            self.transport.upper_body_control(True)
        except LocoError as exc:
            self._release_rpc("idle")
            raise K1BridgeError(f"takeover failed ({exc}); arms {self.state}") from exc
        try:
            return self._start_tracking()
        except BaseException as exc:  # the arms are ours: hand them back before anything else
            self._release_rpc("idle")
            if isinstance(exc, K1BridgeError):
                raise K1BridgeError(f"{exc}; arms {self.state}") from exc
            raise

    def _start_tracking(self) -> K1State:
        """After the takeover returned: a newer state, then the envelope reset."""
        returned = self.clock()
        deadline = returned + self.config.takeover_state_timeout_s
        while True:
            now = self.clock()
            state = self.transport.latest_state()
            if state is not None and state.receive_time_s > returned \
                    and self._robot_ready(now) is None:
                break
            if now >= deadline:
                raise K1BridgeError("no fresh joint, mode and fall state after takeover")
            self.sleeper(0.005)
        measured = normalized_arm_pose(state.arm_positions(), self.limits)
        try:
            self.envelope.reset(measured, now=now)
        except SafetyError as exc:
            raise K1BridgeError(f"cannot start from the measured arms: {exc}") from exc
        self._last_positions = dict(measured)
        self._history.clear()
        self.state = "tracking"
        return state

    def check(self, now: float) -> tuple[str | None, K1State | None, float]:
        """The per-tick checks of :meth:`step`, without publishing: ``(fault reason
        or None, measured state, state age)``."""
        state, age = self._fresh_state(now)
        if state is None or age >= self.config.state_stale_timeout_s or age < -0.005:
            # Stale arms, head or legs: the contact check would screen a pose the robot left.
            return ("state_stale" if age >= 0 else "state_clock"), state, age
        problem = self._robot_ready(now) or self._tracking_error(state)
        return problem, state, age

    def step(self, targets: Mapping[str, float], *, source_timestamp: float, now: float,
             enable: bool = True) -> K1Step:
        """One control tick: check, bound, publish."""
        if self.state != "tracking":
            return K1Step(None, self.state, self.state, math.inf)
        problem, state, age = self.check(now)
        if problem is not None or state is None:
            return self._fault_step(problem or "state_stale", age)
        contact = self.contact.at(np.asarray(state.positions)) if self.contact is not None else None
        try:
            command = self.envelope.step(targets, source_timestamp=source_timestamp, now=now,
                                         enable=enable, contact=contact)
        except SafetyError as exc:
            return self._fault_step(f"safety_fault:{exc}", age)
        if command.reason == "hard_timeout":
            return self._fault_step("source_hard_timeout", age)
        try:
            frame = self.build_frame(command.positions)
            self.transport.publish(frame)
        except RuntimeError as exc:  # the SDK's errors, K1BridgeError, LocoError
            return self._fault_step(f"publish_failed:{exc}", age)
        self._last_positions = dict(command.positions)
        self._history.append((now, dict(command.positions)))
        return K1Step(frame, command.reason, self.state, age, command)

    def stop(self) -> K1Step:
        """Normal stop: decelerate to rest, then hand the arms back."""
        if self.state != "tracking":
            return K1Step(None, self.state, self.state, math.inf)
        start = self.clock()
        last: K1Step | None = None
        while True:
            self.sleeper(0.02)  # one 50 Hz control period after the previous command
            now = self.clock()
            if now - start > self.config.stop_timeout_s:
                break
            last = self.step(self._last_positions or {}, source_timestamp=now, now=now,
                             enable=False)
            if self.state != "tracking":
                return last
            if last.command is not None and all(
                    abs(value) < 1e-3 for value in last.command.velocities.values()):
                break
        self._release_rpc("released")
        return K1Step(None, "stopped", self.state, last.state_age_s if last else math.inf)

    def abort(self, reason: str) -> K1Step:
        """Latch a fault from outside (lost real time, an operator abort)."""
        if self.state != "tracking":
            return K1Step(None, self.state, self.state, math.inf)
        return self._fault_step(reason, math.inf)

    def _fault_step(self, reason: str, age: float) -> K1Step:
        """Latch a fault: no further frames, hand the arms back at once."""
        self.fault = reason
        self._release_rpc("fault_latched")
        return K1Step(None, reason, self.state, age)

    def _release_rpc(self, released_state: ReleasedState) -> None:
        """``UpperBodyCustomControl(False)`` with retries; ``released_state`` only
        when the SDK confirms it, else ``release_unconfirmed`` (no rearm)."""
        for attempt in range(self.config.release_attempts):
            try:
                self.transport.upper_body_control(False)
            except LocoError as exc:
                if exc.final:
                    break
                if attempt + 1 < self.config.release_attempts:
                    self.sleeper(RELEASE_RETRY_S)
                continue
            self.state = released_state
            return
        self.state = "release_unconfirmed"

    # -- frames -------------------------------------------------------------

    def tracking_errors(self, state: K1State) -> dict[str, float] | None:
        """Per arm joint, |measured - the closest command published in the
        ``TRACKING_LAG_S`` before the state arrived|; ``None`` before the first command.

        The K1 follows about 0.1 s late (2026-11-15), so a fast but faithful arm is not
        counted as an error; a blocked arm leaves every recent command and still is.

        Known limit: each joint takes its own closest command, so an arm blocked at a
        position the commands pass again within every ``TRACKING_LAG_S`` (a fast
        back-and-forth move) is not detected.  This is a divergence fault, not a
        guaranteed blocked-arm stop."""
        sent = [(published, positions) for published, positions in self._history
                if published <= state.receive_time_s]
        if not sent:
            return None
        recent = [positions for published, positions in sent
                  if published >= state.receive_time_s - TRACKING_LAG_S] or [sent[-1][1]]
        measured = state.arm_positions()
        return {name: min(abs(measured[name] - positions[name]) for positions in recent)
                for name in recent[0]}

    def _tracking_error(self, state: K1State) -> str | None:
        errors = self.tracking_errors(state)
        if not errors:
            return None
        name = max(errors, key=errors.__getitem__)
        if errors[name] <= self.config.tracking_error_limit_rad:
            return None
        return f"tracking_error:{name} {errors[name]:.3f} rad"

    def build_frame(self, positions: Mapping[str, float]) -> K1Frame:
        """The ``ArmController`` frame for these arm positions (range-checked)."""
        motors = [MotorCommand() for _ in range(MOTOR_COUNT)]
        for name, index in ARM_MOTOR_INDEX.items():
            value = float(positions[name])
            limit = self.limits[name]
            if not (math.isfinite(value) and limit.lower <= value <= limit.upper):
                raise K1BridgeError(f"commanded {name}={value} is outside [{limit.lower}, {limit.upper}]")
            motors[index] = MotorCommand(q=value, kp=self.config.kp, kd=self.config.kd)
        return K1Frame(tuple(motors))


@dataclass(frozen=True)
class AcceptanceRecord:
    """The day-one hoisted commissioning result that unlocks the real transport.

    Written by ``tools/control/k1_commission.py record`` from the day-one
    evidence; see docs/k1-bringup-runbook.md for every field.  Schema 2 approves
    one exact kp/kd pair (the tested one) and speed/acceleration ceilings.
    """

    data: Mapping[str, Any]

    SCHEMA = 2
    TRUE_FIELDS = ("release_verified", "rpc_ordering_verified", "signs_and_zero_offsets_verified",
                   "balance_during_upper_body")

    @classmethod
    def load(cls, path: Path) -> AcceptanceRecord:
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise K1BridgeError(f"cannot read acceptance record {path}: {exc}") from exc
        if not isinstance(data, dict):
            raise K1BridgeError("acceptance record must be a JSON object")
        return cls(data)

    def problems(self) -> list[str]:
        """What the record lacks for real replay (empty when complete)."""
        data, found = self.data, []
        if data.get("schema") != self.SCHEMA:
            found.append(f"schema must be {self.SCHEMA} (redo it with tools/control/k1_commission.py)")
        if data.get("practice") is not False:
            found.append("practice must be false: a record from the fake robot unlocks nothing")
        if data.get("robot") != "booster_k1":
            found.append("robot must be booster_k1")
        if data.get("command_contract") != COMMAND_CONTRACT:
            found.append(f"command_contract must be {COMMAND_CONTRACT}")
        for key in ("firmware", "sdk_binding_version", "date", "operator"):
            if not isinstance(data.get(key), str) or not data[key]:
                found.append(f"{key} is missing")
        identity = data.get("robot_identity")
        if not isinstance(identity, dict) or not identity.get("serial_number"):
            found.append("robot_identity.serial_number is missing")
        if data.get("identity_source") not in ("queried", "attested"):
            found.append("identity_source must be queried or attested")
        if data.get("takeover_mode") not in ROBOT_MODES:
            found.append("takeover_mode must name a robot mode")
        for key in self.TRUE_FIELDS:
            if data.get(key) is not True:
                found.append(f"{key} must be true")
        fall = data.get("fall_topic")
        if not (isinstance(fall, dict) and fall.get("kind") == "periodic"
                and _positive(fall.get("period_s"))):
            found.append("fall_topic must be periodic with period_s (event-only is not enough)")
        if not _positive(data.get("low_state_period_s")):
            found.append("low_state_period_s is missing")
        loss = data.get("command_loss")
        if not (isinstance(loss, dict) and isinstance(loss.get("behaviour"), str)
                and loss["behaviour"] and loss.get("acceptable") is True):
            found.append("command_loss needs an observed behaviour and acceptable: true")
        approved = data.get("approved")
        if not (isinstance(approved, dict) and all(
                _positive(approved.get(key)) for key in
                ("kp", "kd", "speed_max_rad_s", "accel_max_rad_s2"))):
            found.append("approved needs kp, kd, speed_max_rad_s and accel_max_rad_s2")
        return found

    def setting_mismatches(self, config: K1BridgeConfig, binding: str | None) -> list[str]:
        """Requested settings or installed bindings the record does not approve
        (checked before connecting)."""
        data, found = self.data, []
        if binding != data.get("sdk_binding_version"):
            found.append(f"SDK bindings {binding} != recorded {data.get('sdk_binding_version')}")
        approved = data.get("approved", {})
        for name, value in (("kp", config.kp), ("kd", config.kd)):  # only the tested pair
            if not _positive(approved.get(name)) or value != approved[name]:
                found.append(f"{name} {value:g} is not the approved (tested) {approved.get(name)}")
        for name, value, key in (("speed", config.speed_max_rad_s, "speed_max_rad_s"),
                                 ("acceleration", config.accel_max_rad_s2, "accel_max_rad_s2")):
            if not _positive(approved.get(key)) or value > approved[key]:
                found.append(f"{name} {value:g} exceeds the approved {approved.get(key)}")
        return found

    def identity_mismatches(self, robot_info: Mapping[str, str] | None,
                            attested_robot_id: str | None) -> list[str]:
        """Where the connected robot differs from the record (checked before takeover);
        ``robot_info`` is ``None`` when ``GetRobotInfo`` failed."""
        data, found = self.data, []
        serial = data.get("robot_identity", {}).get("serial_number")
        if robot_info is not None:
            if robot_info.get("serial_number") != serial:
                found.append(f"robot {robot_info.get('serial_number')} != recorded {serial}")
            if robot_info.get("version") != data.get("firmware"):
                found.append(f"firmware {robot_info.get('version')} != recorded {data.get('firmware')}")
        elif data.get("identity_source") != "attested":
            found.append("robot identity cannot be queried and the record is not attested")
        elif attested_robot_id != serial:
            found.append("--attested-robot-id must equal the record's serial number")
        return found

    def bridge_config(self, config: K1BridgeConfig) -> K1BridgeConfig:
        """``config`` with the recorded takeover mode and fall-topic freshness."""
        return replace(config, allowed_modes=frozenset({ROBOT_MODES[self.data["takeover_mode"]]}),
                       fall_timeout_s=3.0 * float(self.data["fall_topic"]["period_s"]))


def normalized_arm_pose(arms: Mapping[str, float],
                        limits: Mapping[str, JointLimit]) -> dict[str, float]:
    """Measured arm positions as the envelope starts from them: up to
    ``RANGE_TOLERANCE_RAD`` past a range end counts as the end, further out raises."""
    pose = {}
    for name, value in arms.items():
        limit = limits[name]
        # A straight elbow rests on its range end (0.0); encoder noise may pass it.
        if not limit.lower - RANGE_TOLERANCE_RAD <= value <= limit.upper + RANGE_TOLERANCE_RAD:
            raise K1BridgeError(f"measured {name}={value:.3f} is outside its range")
        pose[name] = min(max(value, limit.lower), limit.upper)
    return pose


@contextmanager
def motion_lock(path: Path | None = None) -> Iterator[None]:
    """Hold the PC-wide K1 motion lock, or raise :class:`K1BridgeError` at once."""
    import fcntl  # POSIX only, like the Booster SDK

    path = MOTION_LOCK_PATH if path is None else path
    try:  # read-only: flock needs no write access, so another user's lock file works too
        try:
            fd = os.open(path, os.O_RDONLY)
        except FileNotFoundError:
            fd = os.open(path, os.O_CREAT | os.O_RDONLY, 0o644)
    except OSError as exc:
        raise K1BridgeError(f"cannot open the K1 motion lock {path}: {exc}") from exc
    try:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise K1BridgeError(f"another K1 motion process holds {path}; stop it first") from exc
        yield
    finally:
        os.close(fd)  # closing releases the lock


def _positive(value: object) -> bool:
    return (not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(value) and value > 0)
