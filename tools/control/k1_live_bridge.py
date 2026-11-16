#!/usr/bin/env python3
"""Booster K1 live arms: follow the suit stream from the live tool (K4, stage S1/S2).

    live_g1_sim.py --robot booster_k1 --robot-stream udp://127.0.0.1:47011
        ──UDP──▶ this bridge ──▶ K1ArmBridge + envelope ──▶ rt/joint_ctrl

Quick start (one command from cold): the probe makes a new session (fake = practice robot,
booster = the real K1), auto-check 1 (robot: serial, firmware, walking mode, not fallen, fresh
joint state, joints in range), auto-check 2 (stream: fresh, calibrated, tracking on, model,
clock, arms within the takeover gap), a summary, then Go: ``--go`` on the command line, else
one question here.  Stage-gate problems and a missing acceptance record are warnings in the
summary; without a record the run uses attempt 049's settings (kp 40, kd 2, 2 rad/s, 8 rad/s^2).

    python3 tools/control/k1_live_bridge.py --transport booster --net 192.168.10.10 --go
    python3 tools/control/k1_live_bridge.py --session DIR --stage hoist
    python3 tools/control/k1_live_bridge.py --session DIR --stage hoist --drill stream-loss
    python3 tools/control/k1_live_bridge.py --session DIR --stage hoist --drill stop
    python3 tools/control/k1_live_bridge.py --session DIR --stage hoist --evaluate --operator NAME

One run = one attempt = one takeover/release cycle: wait for the start gate
(stream fresh, calibrated, tracking on for 1 s), take over, move the robot's arms
to the performer's arm pose at <= 0.2 rad/s (from any gap; the envelope and the
contact check apply) until every gap < 0.1 rad, then follow.  A new
session id, a new calibration epoch or tracking off stops the arms (decelerate,
release): start again.  Lag policy: a gap > 1.0 rad for 1.0 s holds (bounded
deceleration), then re-aligns at up to 1.0 rad/s; a hold longer than 6 s stops.  Stream loss holds
at 0.1 s and releases at the hard timeout t_s + t_b + 0.1 s, after the bounded
deceleration finished.  STOP = Ctrl-C (decelerate, release).

``--evaluate`` judges the session against the stage PASS and writes the
ledger entry.  On the real robot the stage gate must allow the stage first.

Exit status: 0 PASS (or a confirmed release after STOP), 1 FAIL or an
unconfirmed release, 2 refused (a failed auto-check, or no Go).
"""

from __future__ import annotations

import argparse
from collections import deque
import getpass
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field, replace
import json
import math
from pathlib import Path
import socket
import sys
import threading
from types import SimpleNamespace
import time
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from mocap_g1.arm_bridge import ArmBridgeError  # noqa: E402
from mocap_g1.io_utils import sha256_file  # noqa: E402
from mocap_g1.k1_bridge import (  # noqa: E402
    FALL_READY,
    RANGE_TOLERANCE_RAD,
    ROBOT_MODES,
    AcceptanceRecord,
    K1ArmBridge,
    K1BridgeConfig,
    K1BridgeError,
    motion_lock,
)
from mocap_g1.k1_head import HEAD_JOINTS, HeadCommander, inhibit_on_release  # noqa: E402
from mocap_g1.k1_live_packet import (  # noqa: E402
    PacketError,
    LivePacket,
    decode,
    joint_order_hash,
    model_hash_prefix,
)
from mocap_g1.live_robots import K1_JOINTS, K1_UPPER_BODY_JOINTS as ARMS, robot_joint_ranges  # noqa: E402
from mocap_g1.safety import ArmSafetyEnvelope, SafetyConfig  # noqa: E402
from mocap_g1.schema import JointLimit  # noqa: E402
from mocap_g1.time_namespace import time_namespace_problem  # noqa: E402
from tools.control import k1_commission as commission  # noqa: E402
from tools.control.g1_arm_bridge import iter_dds_replay_ticks, sigint_gate  # noqa: E402
from tools.control.k1_arm_bridge import (  # noqa: E402
    MANUAL_STOP,
    NO_RECORD,
    READINGS_TIMEOUT_S,
    contact_check,
    wait_for_readings,
)

LIMITS = commission.LIMITS
CONTROL_DT = 0.02  # 50 Hz, the tested rate (100 Hz only after S2)
DEFAULT_PORT = 47011
#: Where the live tool writes its suit captures (mocap_g1/suit_recorder.py; option --suit-root).
SUIT_ROOT = REPO_ROOT / "artifacts" / "xsens-live" / "suit"

# Stream rules (3.2, 10.2)
STALE_S = 0.10               # t_s: hold when the newest fresh suit frame is older
STALL_S = 0.10               # tick_stall_timeout_s: 5 ticks at 50 Hz
MAX_STOP_BOUND_S = 1.5       # refuse settings whose hard timeout exceeds this
#: A sender time this far ahead of this PC's clock proves a clock mismatch (stream clock rule 3).
AHEAD_S = 0.002
RANGE_SLACK_RAD = 0.05       # further outside the K1 range: dropped; inside it: clamped
DROP_FAULT_PER_S = 20
FROZEN_PACKETS = 3
# Takeover and lag policy (3.2, 10.3)
#: The start gate (fresh, calibrated, tracking on) must hold this long.  There is no arm-gap
#: limit since 2026-11-16 (owner): the alignment moves the robot's arms to the performer's.
COARSE_S = 1.0
ALIGN_SPEED_RAD_S, ALIGNED_RAD, REALIGNED_S = 0.2, 0.1, 0.5
#: Was 0.35 / 0.5 / 3.0, then 0.5 / 0.5 / 6.0: fast performers lag-stopped a real K1 (2026-11-15).
LAG_GAP_RAD, LAG_S, LAG_STOP_S = 1.0, 1.0, 6.0
REALIGN_SPEED_RAD_S = 1.0  # after a lag hold; 0.2 took > LAG_STOP_S for a 0.8 rad gap (047)
#: A stream that stays stale (late, never lost) holds the arms; this long, it stops them.
LATE_STOP_S = 3.0
AT_REST_RAD_S = 0.05
#: Hold the takeover pose this long before aligning (commissioning SETTLE_S).
TAKEOVER_SETTLE_S = 1.5
GATE_WAIT_S = 60.0
# S1 hoist: shoulder pitch and roll stay within this of rest until strap clearance is confirmed
HOIST_SHOULDER_RAD = 0.6
SHOULDERS = tuple(name for name in ARMS if "Shoulder" in name)
TIME_LIMIT_S, MAX_TIME_LIMIT_S = 120.0, 600.0
# Quick start
QUICK_PROBE_S = 5.0          # auto-check 1's read-only probe on the real robot
STREAM_CHECK_S = 10.0        # auto-check 2 waits this long for the stream conditions
SUPPORT_DEFAULT = "held by a person"
# Stage PASS and requalification
PASS_FOLLOW_S, REQUALIFY_FOLLOW_S = 600.0, 120.0
PASS_CYCLES = 5
COVERAGE_FRACTION, SPEED_FRACTION = 0.5, 0.8
TRACKING_P95_RAD = 0.15
DRILLS = ("stream-loss", "stop")
EXPECTED_FAULT = {"stream-loss": "source_hard_timeout", "stop": "operator_stop"}
#: Faults a run of this stage must never have; one revokes the gate stage it ran under.
REVOKING = ("release_unconfirmed", "fall_state", "contact_infeasible", "state_stale")

ROLES = {
    "hoist": "the performer in the suit; an operator at STOP (Ctrl-C or the window's STOP); "
             "a person on site with the remote (LT+BACK = damping)",
    "standing": "the performer in the suit; an operator at STOP (Ctrl-C or the window's STOP); "
                "a person on site with the remote (LT+BACK = damping); a spotter at the robot",
}
CHECKLIST = """\
Live arms, {stage}. Roles needed now: {roles}.
Before the robot moves:
  - {support};
  - the live tool streams (--robot-stream), the performer stands still with arms down and the
    console's Calibrate and Tracking on are done;
  - nothing and nobody within reach of the arms; the performer can see the robot;
  - the robot is in walking mode (set with the remote).
After Go the robot's arms move slowly (0.2 rad/s) to the performer's arm pose, then follow:
the performer holds still until the status says following.
STOP: Ctrl-C here (or the window's STOP): the arms slow down, then go back to Booster."""
SUPPORT = {
    "hoist": "the robot stands with its feet loaded and the hoist strap slack but attached",
    "standing": "the robot stands free with the strap slack and attached; the spotter is ready",
}

#: The stage gate module (mocap_g1/stage_gate.py); tests replace it with a fake.
GATE: Any = None
#: Builds the real transport from the session meta: loco RPCs in an isolated process with
#: deadlines; tests replace it.
def _isolated_transport(meta: Mapping[str, Any]) -> Any:
    from mocap_g1.k1_bridge import IsolatedBoosterTransport

    return IsolatedBoosterTransport(domain_id=int(meta.get("domain_id", 0)),
                                    network_interface=str(meta.get("net", "")))


REAL_TRANSPORT: Callable[[Mapping[str, Any]], Any] | None = _isolated_transport
#: Receives a status dict about every ``STATUS_EVERY_S`` (the window's live panel: the GUI
#: child sets it to emit ``@@K1GUI {"event": "live", ...}``); ``None`` on the command line.
STATUS: Callable[[dict[str, Any]], None] | None = None
STATUS_EVERY_S = 0.2
#: Receives the auto-checks line by line (``{"key", "ok", "text"}``; ``ok`` None = not
#: checked yet): the window's checklist (the GUI child emits ``checks`` events); ``None`` on
#: the command line, which prints the same results as text.
CHECKS: Callable[[list[dict[str, Any]]], None] | None = None
CHECKS_EVERY_S = 0.5
PHASE_STATE = {"aligning": "aligning", "realign": "aligning", "following": "following",
               "hold": "lag-hold"}


def report_checks(*items: tuple[str, bool | None, str]) -> None:
    """Send check lines ``(key, ok, text)`` to the window; never stops a check."""
    if CHECKS is None:
        return
    try:
        CHECKS([{"key": key, "ok": ok, "text": text} for key, ok, text in items])
    except OSError:
        pass


def stage_gate() -> Any:
    if GATE is not None:
        return GATE
    from mocap_g1 import stage_gate as gate  # lazy: the gate lands on another branch

    return gate


# -- stream ---------------------------------------------------------------------


@dataclass(frozen=True)
class StreamView:
    """What the control loop reads from the receiver each tick."""

    packet: LivePacket | None
    #: When the stream was last fresh, on this PC's clock (the same clock as the sender's,
    #: rule 1): the older of the suit receive time of the newest fresh packet (counter
    #: advancing, sender time) and this PC's receive time of the last counter advance (rule 4).
    fresh_time: float | None
    #: Latched ``new_session`` / ``new_epoch`` once the run locked the stream; ``clock_mismatch``
    #: (rule 3) at any time.
    event: str | None
    drops_last_s: int
    #: Why the stream's clock cannot be this PC's (rule 3), or ``None``.
    clock_error: str | None = None


def stream_limits(model: Path | None) -> dict[str, Any]:
    """The range a streamed arm target may have before it is dropped as bad data: the K1 model's
    joint range (GMR's IK reaches up to it; wider than the official limits) joined with the
    official limits.  Values inside it but outside the envelope are clamped by the envelope."""
    if model is None:
        return dict(LIMITS)
    import mujoco

    mj = mujoco.MjModel.from_xml_path(str(model))
    ranges: dict[str, Any] = {}
    for name in ARMS:
        lower, upper = LIMITS[name].lower, LIMITS[name].upper
        try:
            joint = mj.joint(name)
        except KeyError:  # a model without this joint: keep the official (stricter) range
            joint = None
        if joint is not None and mj.jnt_limited[joint.id]:
            lower = min(lower, float(mj.jnt_range[joint.id][0]))
            upper = max(upper, float(mj.jnt_range[joint.id][1]))
        ranges[name] = SimpleNamespace(lower=lower, upper=upper)
    return ranges


class StreamReceiver:
    """The drop and event rules of 3.2/10.2 over decoded packets; thread-safe."""

    def __init__(self, *, model_hash: bytes | None, limits: Mapping[str, Any] = LIMITS) -> None:
        self.model_hash = model_hash
        self.limits = limits
        self._lock = threading.Lock()
        self._packet: LivePacket | None = None
        self._fresh_time: float | None = None
        self._repeats = 0
        self._locked: tuple[int, int] | None = None  # (session id, epoch) adopted at takeover
        self._event: str | None = None
        self._drop_times: deque[float] = deque()
        self._advanced_at: float | None = None  # this PC's time of the last counter advance
        self._clock_error: str | None = None
        self.received = 0
        self.accepted = 0
        self.drops: dict[str, int] = {}
        #: Bridge receive time minus suit receive time of accepted packets (latency).
        self.transit_s: deque[float] = deque(maxlen=20000)

    def offer(self, data: bytes, now: float) -> str | None:
        """Apply one datagram received at ``now``; returns its drop reason or ``None``."""
        with self._lock:
            self.received += 1
            try:
                packet = decode(data)
            except PacketError as exc:
                return self._drop(f"malformed:{exc}", now)
            if packet.joint_order != joint_order_hash() or (
                    self.model_hash is not None and packet.model_hash != self.model_hash):
                return self._drop("model_or_joint_order", now)
            if self._locked is not None:  # events come before every wrong-session drop
                session, epoch = self._locked
                if packet.session_id != session:
                    self._event = self._event or "new_session"
                    return None
                if packet.calibration_epoch != epoch:
                    self._event = self._event or "new_epoch"
                    return None
            last = self._packet
            if last is not None and packet.session_id != last.session_id:
                last = None  # before takeover a restarted sender simply starts over
            if last is not None and packet.sequence <= last.sequence:
                return self._drop("sequence", now)
            if last is not None and packet.generation < last.generation:
                return self._drop("generation", now)
            for name, value in zip(ARMS, packet.targets):
                limit = self.limits[name]
                if not limit.lower - RANGE_SLACK_RAD <= value <= limit.upper + RANGE_SLACK_RAD:
                    return self._drop("range", now)
            ahead = max(packet.sender_monotonic_s, packet.suit_receive_s) - now
            if ahead > AHEAD_S:  # rule 3: one clock cannot stamp a time not yet reached
                self._clock_error = self._clock_error or (
                    f"the stream's clock is {ahead * 1e3:.1f} ms ahead of this PC's: run the "
                    "live tool on this PC, on the host's clock (Docker with --network host, "
                    "no time namespace)")
                self._event = self._event or "clock_mismatch"
                return "clock_ahead"
            self._repeats = self._repeats + 1 if (
                last is not None and packet.sample_counter == last.sample_counter) else 0
            if self._repeats == 0:
                self._advanced_at = now
            self._packet = packet
            self.accepted += 1
            if self._repeats < FROZEN_PACKETS:
                self._fresh_time = packet.suit_receive_s
                self.transit_s.append(now - packet.suit_receive_s)
            return None

    def _drop(self, reason: str, now: float) -> str:
        key = reason.split(":")[0]
        self.drops[key] = self.drops.get(key, 0) + 1
        self._drop_times.append(now)
        return reason

    def view(self, now: float) -> StreamView:
        with self._lock:
            while self._drop_times and self._drop_times[0] <= now - 1.0:
                self._drop_times.popleft()
            fresh = (None if self._fresh_time is None or self._advanced_at is None
                     else min(self._fresh_time, self._advanced_at))
            return StreamView(self._packet, fresh, self._event, len(self._drop_times),
                              self._clock_error)

    def lock(self) -> None:
        """Adopt the current session and epoch: any change from here on is an event."""
        with self._lock:
            if self._packet is None:
                raise K1BridgeError("no stream to lock")
            self._locked = (self._packet.session_id, self._packet.calibration_epoch)

    def statistics(self) -> dict[str, Any]:
        with self._lock:
            transit = sorted(self.transit_s)
        return {"received": self.received, "accepted": self.accepted, "drops": dict(self.drops),
                "transit_p50_s": _percentile(transit, 50), "transit_p95_s": _percentile(transit, 95)}


def serve(sock: socket.socket, receiver: StreamReceiver, clock: Callable[[], float],
          stop: threading.Event) -> None:
    """Receiver thread: every datagram goes through the rules at once (newest wins)."""
    sock.settimeout(0.05)
    while not stop.is_set():
        try:
            data, _source = sock.recvfrom(1024)
        except socket.timeout:
            continue
        except OSError:
            return
        receiver.offer(data, clock())


def open_stream(port: int) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("127.0.0.1", port))
    except OSError as exc:
        sock.close()
        raise K1BridgeError(f"cannot listen on 127.0.0.1:{port}: {exc}") from exc
    return sock


# -- settings -------------------------------------------------------------------


def stop_bound_s(config: K1BridgeConfig) -> float:
    """The envelope's hard timeout: stale detection + braking + one scheduling allowance."""
    return STALE_S + config.speed_max_rad_s / config.accel_max_rad_s2 + 0.1


def live_safety_config(config: K1BridgeConfig) -> SafetyConfig:
    hard = stop_bound_s(config)
    if hard > MAX_STOP_BOUND_S:
        raise K1BridgeError(f"stopping bound {hard:.2f} s (0.1 + speed/accel + 0.1) exceeds "
                            f"{MAX_STOP_BOUND_S:g} s: lower the speed cap or raise the acceleration cap")
    return replace(config.safety_config(LIMITS), stale_timeout_s=STALE_S, hard_timeout_s=hard,
                   tick_stall_timeout_s=STALL_S)


def stop_settings(config: K1BridgeConfig) -> dict[str, Any]:
    return {"stale_timeout_s": STALE_S, "hard_timeout_s": round(stop_bound_s(config), 6),
            "tick_stall_timeout_s": STALL_S, "lag_gap_rad": LAG_GAP_RAD, "lag_s": LAG_S,
            "lag_stop_s": LAG_STOP_S, "realign_speed_rad_s": REALIGN_SPEED_RAD_S,
            "realigned_rad": ALIGNED_RAD, "realigned_s": REALIGNED_S,
            "stop_timeout_s": config.stop_timeout_s, "release_attempts": config.release_attempts}


def envelope_sha256() -> str:
    from mocap_g1.stage_gate import envelope_sha256 as gate_hash

    return str(gate_hash(LIMITS))


def gate_settings(config: K1BridgeConfig, model_sha: str | None) -> dict[str, Any]:
    """What the stage gate compares (common interface: exact pair, caps, rate, hashes, stops)."""
    return {"kp": config.kp, "kd": config.kd, "speed_max_rad_s": config.speed_max_rad_s,
            "accel_max_rad_s2": config.accel_max_rad_s2, "control_rate_hz": 1.0 / CONTROL_DT,
            "model_sha256": model_sha, "envelope_sha256": envelope_sha256(),
            "stop_settings": stop_settings(config)}


# -- one run ----------------------------------------------------------------------


@dataclass
class Plan:
    stage: str  # hoist | standing
    drill: str | None
    time_limit_s: float
    strap_clear: bool
    config: K1BridgeConfig
    #: K6: the head follows a v3 stream (``--no-head``: no head call at all).
    head: bool = True

    @property
    def key(self) -> str:
        return f"live-arms@{self.stage}" + (f":drill:{self.drill}" if self.drill else "")


@dataclass
class RunResult:
    trigger: str | None = None
    phase_reached: str = "waiting"
    follow_s: float = 0.0
    released_at: float | None = None
    last_fresh: float | None = None
    rows: list[dict[str, Any]] = field(default_factory=list)
    after: list[dict[str, Any]] = field(default_factory=list)  # measured, after the release
    takeovers: int = 0
    lag_holds: int = 0


def hoist_limited(targets: Mapping[str, float], rest: Mapping[str, float]) -> dict[str, float]:
    """Shoulder pitch and roll within ``HOIST_SHOULDER_RAD`` of their rest (strap not cleared)."""
    out = dict(targets)
    for name in SHOULDERS:
        out[name] = min(max(out[name], rest[name] - HOIST_SHOULDER_RAD), rest[name] + HOIST_SHOULDER_RAD)
    return out


def hoist_limits(rest: Mapping[str, float]) -> dict[str, JointLimit]:
    """Envelope limits whose usable range (inside the margin) is the hoist shoulder range, so
    the envelope brakes in time; a target clamp alone would overshoot by v^2 / 2a."""
    margin = SafetyConfig().position_margin_rad
    limits = dict(LIMITS)
    for name in SHOULDERS:
        limit = LIMITS[name]
        limits[name] = JointLimit(max(limit.lower, rest[name] - HOIST_SHOULDER_RAD - margin),
                                  min(limit.upper, rest[name] + HOIST_SHOULDER_RAD + margin),
                                  limit.velocity)
    return limits


def clamp_to_limits(targets: Mapping[str, float]) -> dict[str, float]:
    return {name: min(max(value, LIMITS[name].lower), LIMITS[name].upper)
            for name, value in targets.items()}


def ramp(last: Mapping[str, float], target: Mapping[str, float], step: float) -> dict[str, float]:
    return {name: last[name] + min(max(target[name] - last[name], -step), step) for name in ARMS}


class LiveRun:
    """Wait for the coarse gate, take over, align, follow; always end released."""

    def __init__(self, rig: Any, receiver: StreamReceiver, plan: Plan, *,
                 pump: Callable[[float], None] | None = None, log: Any = None) -> None:
        self.rig = rig
        self.receiver = receiver
        self.plan = plan
        self.pump = pump or (lambda _now: None)
        self.log = log
        self.result = RunResult()
        self.bridge = K1ArmBridge(rig.transport, LIMITS, plan.config, contact=rig.contact,
                                  clock=rig.clock, sleeper=rig.sleeper)
        self.bridge.envelope = ArmSafetyEnvelope(self.bridge.limits, live_safety_config(plan.config),
                                                 joints=ARMS)
        self.rest: dict[str, float] = {}
        self.last_gaps: dict[str, float] = {}
        #: K6 head commander (mocap_g1/k1_head.py), started after the takeover.
        self.head: HeadCommander | None = None
        self._status_at = -math.inf
        self._rate_mark: tuple[float, int] | None = None

    def targets(self, packet: LivePacket, rest: Mapping[str, float] | None = None) -> dict[str, float]:
        targets = clamp_to_limits(packet.arm_targets())
        if self.plan.stage == "hoist" and not self.plan.strap_clear:
            targets = hoist_limited(targets, rest or self.rest)
        return targets

    def gate_problem(self, now: float) -> tuple[str | None, float]:
        """Why the start gate is closed now (``None`` = open), and the worst arm gap (shown
        only: the alignment after the takeover closes any gap)."""
        self.last_gaps = {}
        view = self.receiver.view(now)
        packet = view.packet
        if view.clock_error is not None:
            raise K1BridgeError(view.clock_error)
        if packet is None:
            return "no stream", math.inf
        if view.fresh_time is None or now - view.fresh_time >= STALE_S:
            return "stream stale", math.inf
        if packet.calibration_epoch == 0:
            return "not calibrated (Calibrate in the console)", math.inf
        if not packet.enabled:
            return "tracking is off (console)", math.inf
        state = self.rig.transport.latest_state()
        if state is None:
            return "no joint state", math.inf
        measured = state.arm_positions()
        targets = self.targets(packet, rest=measured)  # before the takeover the rest is now
        self.last_gaps = {name: abs(targets[name] - measured[name]) for name in ARMS}
        gap = max(self.last_gaps.values())
        return None, gap

    def wait_for_gate(self) -> str | None:
        """Block until the start gate held for ``COARSE_S``; ``None`` or why it never did."""
        start = self.rig.clock()
        since: float | None = None
        said = ""
        while True:
            self.rig.sleeper(CONTROL_DT)
            now = self.rig.clock()
            self.pump(now)
            problem, _gap = self.gate_problem(now)
            self.status(now, "waiting", self.last_gaps, gate=problem or "coarse gate held")
            if problem is None:
                since = now if since is None else since
                if now - since >= COARSE_S:
                    return None
            else:
                since = None
                if problem != said:
                    said = problem
                    print(f"Waiting: {problem}", flush=True)
            if now - start >= GATE_WAIT_S:
                return f"coarse gate not met in {GATE_WAIT_S:g} s: {said or 'stream'}"

    def run(self, gate: Any) -> RunResult:
        """Take over and follow until STOP, the time limit, an event or a fault."""
        result, bridge = self.result, self.bridge
        self.receiver.lock()
        if self.plan.stage == "hoist" and not self.plan.strap_clear:
            state = bridge.transport.latest_state()
            if state is None:
                raise K1BridgeError("cannot take over: no joint state")
            bridge.envelope = ArmSafetyEnvelope(hoist_limits(state.arm_positions()),
                                                live_safety_config(self.plan.config), joints=ARMS)
        self.rest = bridge.takeover().arm_positions()
        result.takeovers = 1
        start = self.rig.clock()
        steps = int(math.ceil((self.plan.time_limit_s + 2 * GATE_WAIT_S) / CONTROL_DT))
        paced = not isinstance(self.rig.clock, commission.VirtualClock)
        self.start_head(threaded=paced)
        ticks: Any = (iter_dds_replay_ticks(steps=steps, control_dt=CONTROL_DT, start_time=start,
                                            clock=self.rig.clock, sleeper=self.rig.sleeper)
                      if paced else range(steps))
        phase, aligned_since, lag_since, hold_since = "aligning", None, None, None
        late_since: float | None = None
        result.phase_reached = phase
        gate.allow_interrupt()
        for tick in ticks:
            if paced:
                now = tick.now_s
            else:
                self.rig.sleeper(CONTROL_DT)
                now = self.rig.clock()
            self.pump(now)
            view = self.receiver.view(now)
            packet = view.packet
            assert packet is not None
            result.last_fresh = view.fresh_time
            if view.event is not None or not packet.enabled:
                result.trigger = view.event or "tracking_off"
                break
            if view.drops_last_s > DROP_FAULT_PER_S:
                bridge.abort(f"stream_drops:{view.drops_last_s}/s")
                break
            if result.follow_s >= self.plan.time_limit_s:
                result.trigger = "time_limit"
                break
            target = self.targets(packet)
            last = bridge._last_positions or target
            gaps = {name: abs(target[name] - last[name]) for name in ARMS}
            worst = max(gaps.values())
            enable = True
            if phase == "aligning" and now - start < TAKEOVER_SETTLE_S:
                # A K1 (1.8.1.2) ignores arm commands for about 1 s after the takeover: hold first.
                command, aligned_since = dict(self.rest), None
            elif phase in ("aligning", "realign"):
                speed = ALIGN_SPEED_RAD_S if phase == "aligning" else REALIGN_SPEED_RAD_S
                command = ramp(last, target, speed * CONTROL_DT)
                aligned_since = (aligned_since or now) if worst < ALIGNED_RAD else None
                needed = 0.0 if phase == "aligning" else REALIGNED_S
                if aligned_since is not None and now - aligned_since >= needed:
                    phase, lag_since, hold_since = "following", None, None
                    command = target
            elif phase == "following":
                command = target
                lag_since = (lag_since or now) if worst > LAG_GAP_RAD else None
                if lag_since is not None and now - lag_since > LAG_S:
                    phase, hold_since = "hold", now
                    result.lag_holds += 1
                    print("Performer too fast: holding, then re-aligning", flush=True)
            if phase == "hold":
                command, enable = dict(last), False
            if hold_since is not None and now - hold_since > LAG_STOP_S:
                result.trigger = "lag_stop"
                break
            source_time = view.fresh_time if view.fresh_time is not None else -math.inf
            step = bridge.step(command, source_timestamp=source_time, now=now, enable=enable)
            if self.head is not None:  # the lease: target and "arms are following"
                self.head.feed(now, packet.head,
                               following=phase == "following" and step.reason == "tracking")
                if not paced:
                    self.head.tick(now)
            if phase == "hold" and step.command is not None and all(
                    abs(v) < AT_REST_RAD_S for v in step.command.velocities.values()):
                phase, aligned_since = "realign", None
            late_since = (late_since or now) if step.reason == "stale_source" else None
            if late_since is not None and now - late_since > LATE_STOP_S:
                result.trigger = "stream_late"
                self.record(now, phase, step, view)
                break
            if phase == "following" and step.reason == "tracking":
                result.follow_s += CONTROL_DT
                result.phase_reached = "following"
            self.record(now, phase, step, view)
            self.status(now, PHASE_STATE[phase], gaps)
            if bridge.state != "tracking":
                result.released_at = self.rig.clock()
                break
        else:
            result.trigger = "run_limit"  # never reached the following time limit
        return result

    def start_head(self, *, threaded: bool) -> None:
        """The head commander on the transport's own head RPC backend; every arm release
        inhibits it first.  None with ``--no-head`` or a transport without a head backend."""
        backend = getattr(self.rig.transport, "head_rpc", None)
        rpc = backend() if self.plan.head and backend is not None else None
        if rpc is None:
            return
        self.head = HeadCommander(rpc, self.rig.transport.latest_state, clock=self.rig.clock,
                                  sleeper=self.rig.sleeper)
        inhibit_on_release(self.rig.transport, self.head)
        if threaded:
            self.head.start()

    def end_head(self) -> None:
        """After the arm release: the neutral only after a normal stop, then close the head."""
        if self.head is not None:
            self.head.finish(neutral=self.bridge.state == "released" and self.bridge.fault is None)

    def head_summary(self) -> dict[str, Any]:
        """Head state, calls and errors, and over the following ticks the p95 of
        |target - command| and |command - measured| (rad, worst of yaw and pitch)."""
        if self.head is None:
            return {"state": "disabled" if not self.plan.head else "none"}
        rows = [row["head"] for row in self.result.rows
                if row["phase"] == "following" and row.get("head") and row["head"]["command"]]

        def p95(a: str, b: str) -> float | None:
            return _percentile([max(abs(row[a][n] - row[b][n]) for n in HEAD_JOINTS)
                                for row in rows if row[a] and row[b]], 95)

        return {**self.head.summary(), "target_command_p95_rad": p95("target", "command"),
                "command_measured_p95_rad": p95("command", "measured")}

    def record(self, now: float, phase: str, step: Any, view: StreamView) -> None:
        state = self.rig.transport.latest_state()
        command = step.command
        row = {"t": round(now, 4), "phase": phase, "reason": step.reason,
               "seq": view.packet.sequence if view.packet else None,
               "source_age_s": None if command is None else round(command.source_age_s, 4),
               "contact": None if command is None else command.contact,
               # the performer's arm targets before the ramp, lag policy and envelope
               "target": self.targets(view.packet) if view.packet else None,
               "command": None if command is None else command.positions,
               "velocities": None if command is None else command.velocities,
               "measured": state.arm_positions() if state else None,
               "measured_t": state.receive_time_s if state else None,
               "errors": self.bridge.tracking_errors(state) if state else None}
        head = self.head
        if head is not None:
            row["head"] = {"state": head.status, "target": head.target, "command": head.command,
                           "measured": state.head_positions() if state else None}
        self.result.rows.append(row)
        if self.log is not None:
            self.log.write(json.dumps(row) + "\n")

    def status(self, now: float, state: str, gaps: Mapping[str, float] | None = None, *,
               force: bool = False, **extra: Any) -> None:
        """One status for the window (at most every ``STATUS_EVERY_S`` unless ``force``)."""
        if STATUS is None or (not force and now - self._status_at < STATUS_EVERY_S):
            return
        self._status_at = now
        view = self.receiver.view(now)
        packet = view.packet
        accepted = self.receiver.accepted
        mark, self._rate_mark = self._rate_mark, (now, accepted)
        rate = (accepted - mark[1]) / (now - mark[0]) if mark and now > mark[0] else None
        worst = max(gaps, key=lambda name: gaps[name]) if gaps else None
        fields = {"state": state, "stage": self.plan.stage, "drill": self.plan.drill,
                  "rate_hz": None if rate is None else round(rate, 1),
                  "source_age_s": None if view.fresh_time is None else round(now - view.fresh_time, 3),
                  "epoch": packet.calibration_epoch if packet else None,
                  "enabled": bool(packet.enabled) if packet else None,
                  "worst_joint": worst, "worst_gap_rad": round(gaps[worst], 3) if gaps and worst else None,
                  "following_s": round(self.result.follow_s, 1), "time_limit_s": self.plan.time_limit_s,
                  "lag_holds": self.result.lag_holds,
                  "bridge": self.bridge.state, "release": release_state(self.rig),
                  "head": self.head.status if self.head is not None
                  else ("disabled" if not self.plan.head else "none"), **extra}
        try:
            STATUS(fields)
        except OSError:  # a status line must never stop the control loop or the release
            pass

    def watch_after_release(self, seconds: float) -> None:
        """Measured arms after the release (no commands), for the stream-loss drill."""
        end = self.rig.clock() + seconds
        while self.rig.clock() < end:
            self.rig.sleeper(CONTROL_DT)
            state = self.rig.transport.latest_state()
            if state is not None:
                self.result.after.append({"t": round(self.rig.clock(), 4),
                                          "measured": state.arm_positions()})


# -- judging ------------------------------------------------------------------------


def _percentile(values: list[float], q: float) -> float | None:
    return round(float(np.percentile(values, q)), 4) if values else None


def measured_speed_at(samples: list[tuple[float, Mapping[str, float]]], when: float) -> float | None:
    """Fastest measured arm joint (rad/s) over the two samples around ``when``."""
    for (t0, p0), (t1, p1) in zip(samples, samples[1:]):
        if t0 <= when <= t1 and t1 > t0:
            return max(abs(p1[name] - p0[name]) / (t1 - t0) for name in ARMS)
    return None


def summarize(plan: Plan, run: LiveRun, *, release_confirmed: bool, rpc_seconds: float) -> dict[str, Any]:
    """The attempt's evidence fields: settings outcome, stream statistics and checks."""
    result, rows = run.result, run.result.rows
    commands = [row for row in rows if row["command"]]
    following = [row for row in commands if row["phase"] == "following"]
    joint_range = {name: [min(row["command"][name] for row in following),
                          max(row["command"][name] for row in following)] if following else None
                   for name in ARMS}
    top_speed = {name: max((abs(row["velocities"][name]) for row in following), default=0.0)
                 for name in ARMS}
    errors = [max(row["errors"].values()) for row in following if row.get("errors")]
    ages = [row["source_age_s"] for row in commands if row["source_age_s"] is not None]
    fault = run.bridge.fault or ""
    summary: dict[str, Any] = {
        "trigger": result.trigger, "fault": run.bridge.fault, "release_confirmed": release_confirmed,
        "phase_reached": result.phase_reached, "following_s": round(result.follow_s, 3),
        "cycles": result.takeovers, "lag_holds": result.lag_holds,
        "joint_range": joint_range, "top_speed_rad_s": {k: round(v, 4) for k, v in top_speed.items()},
        "tracking_p95_rad": _percentile(errors, 95),
        "contact_infeasible": sum(1 for row in commands if row["contact"] == "infeasible"),
        "state_stale": int(fault.startswith("state_stale")),
        "fall": int(fault.startswith("fall_state")),
        "source_age_p50_s": _percentile(ages, 50), "source_age_p95_s": _percentile(ages, 95),
        "stream": run.receiver.statistics(), "release_rpc_s": rpc_seconds,
        "strap_clear": plan.strap_clear, "head": run.head_summary(),
    }
    if plan.drill == "stream-loss":
        summary["drill"] = stream_loss_checks(plan, result, release_confirmed, rpc_seconds)
    elif plan.drill == "stop":
        summary["drill"] = stop_checks(result, run.bridge.state, release_confirmed)
    return summary


def stream_loss_checks(plan: Plan, result: RunResult, release_confirmed: bool,
                       rpc_seconds: float) -> dict[str, Any]:
    """10.3: measured arms at rest at t_s + t_b + 0.2 s after the last packet, the commanded
    ones at rest before the release, the release within the hard timeout + 2 ticks + RPC."""
    last = result.last_fresh
    braking = plan.config.speed_max_rad_s / plan.config.accel_max_rad_s2
    samples = [(row["measured_t"], row["measured"]) for row in result.rows if row["measured"]]
    samples += [(row["t"], row["measured"]) for row in result.after]
    measured = None if last is None else measured_speed_at(samples, last + STALE_S + braking + 0.2)
    commanded = [row for row in result.rows if row["velocities"]]
    commanded_speed = max(abs(v) for v in commanded[-1]["velocities"].values()) if commanded else None
    release_after = (None if last is None or result.released_at is None
                     else result.released_at - last)
    bound = STALE_S + braking + 0.1 + 2 * CONTROL_DT + rpc_seconds
    checks = {
        "was_following": result.phase_reached == "following",
        "expected_fault": result.trigger == EXPECTED_FAULT["stream-loss"],
        "release_confirmed": release_confirmed,
        "measured_at_rest": measured is not None and measured < AT_REST_RAD_S,
        "commanded_at_rest": commanded_speed is not None and commanded_speed < AT_REST_RAD_S,
        "release_in_bound": release_after is not None and release_after <= bound,
    }
    return {"checks": checks, "measured_speed_rad_s": measured, "commanded_speed_rad_s": commanded_speed,
            "release_after_s": release_after, "release_bound_s": round(bound, 4)}


def stop_checks(result: RunResult, state: str, release_confirmed: bool) -> dict[str, Any]:
    """STOP: decelerated (a normal release, not a fault) and the arms still afterwards."""
    samples = [(row["t"], row["measured"]) for row in result.after]
    speeds = [measured_speed_at(samples, t) for t, _ in samples[:-1]]
    speed = max((v for v in speeds if v is not None), default=None)
    checks = {"was_following": result.phase_reached == "following",
              "expected_fault": result.trigger == EXPECTED_FAULT["stop"],
              "release_confirmed": release_confirmed and state == "released",
              "measured_at_rest": speed is not None and speed < AT_REST_RAD_S}
    return {"checks": checks, "measured_speed_rad_s": speed}


def verdict(plan: Plan, summary: Mapping[str, Any]) -> tuple[str, str | None]:
    if not summary["release_confirmed"]:
        return "FAIL", "release not confirmed"
    if plan.drill:
        failed = [name for name, ok in summary["drill"]["checks"].items() if not ok]
        return ("PASS", None) if not failed else ("FAIL", "drill: " + ", ".join(failed))
    if summary["fault"]:
        return "FAIL", summary["fault"]
    if summary["trigger"] not in ("operator_stop", "time_limit"):
        return "FAIL", f"stopped: {summary['trigger']}"
    if summary["following_s"] <= 0:
        return "FAIL", "never followed"
    return "PASS", None


def revoking_reason(summary: Mapping[str, Any]) -> str | None:
    """A fault this stage forbids; drills' expected faults never revoke."""
    if not summary["release_confirmed"]:
        return "release_unconfirmed"
    if summary["fall"]:
        return "fall_state"
    if summary["contact_infeasible"]:
        return "contact_infeasible"
    if summary["state_stale"]:
        return "state_stale"
    drill = summary.get("drill")
    if drill and drill["checks"].get("was_following") and drill["checks"].get("expected_fault") \
            and not all(drill["checks"].values()):
        return "drill bound breached"
    return None


def evaluate(attempts: list[dict[str, Any]], stage: str, *, requalify: bool) -> dict[str, Any]:
    """The stage PASS of 3.3 (or the requalification of 1.1) over a session's attempts."""
    key = f"live-arms@{stage}"
    runs = [a for a in attempts if a.get("step") == key]
    all_runs = [a for a in attempts if str(a.get("step", "")).startswith(key)]
    counted = [a for a in runs if a.get("verdict") == "PASS"]
    drills = {name: next((a for a in reversed(attempts) if a.get("step") == f"{key}:drill:{name}"), None)
              for name in DRILLS}
    settings = {json.dumps(a.get("gate_settings"), sort_keys=True) for a in counted}
    for drill in drills.values():
        if drill is not None:
            settings.add(json.dumps(drill.get("gate_settings"), sort_keys=True))
    follow = sum(a["following_s"] for a in counted)
    checks: dict[str, bool] = {
        "following_s": follow >= (REQUALIFY_FOLLOW_S if requalify else PASS_FOLLOW_S),
        "release_unconfirmed": all(a.get("release_confirmed") for a in all_runs),
        "contact_infeasible": not any(a.get("contact_infeasible") for a in all_runs),
        "state_stale": not any(a.get("state_stale") for a in all_runs),
        "tracking_p95": bool(counted) and all((a.get("tracking_p95_rad") or 0.0) < TRACKING_P95_RAD
                                              for a in counted),
        "drill_stream_loss": (drills["stream-loss"] or {}).get("verdict") == "PASS",
        "drill_stop": (drills["stop"] or {}).get("verdict") == "PASS",
        "same_settings": len(settings) == 1,
    }
    coverage: dict[str, float] = {}
    speed: dict[str, float] = {}
    if not requalify:
        checks["cycles"] = len(counted) >= PASS_CYCLES
        for name in ARMS:
            spans = [a["joint_range"][name] for a in counted if a["joint_range"].get(name)]
            usable = LIMITS[name].upper - LIMITS[name].lower - 2 * commission.MARGIN
            coverage[name] = round((max(s[1] for s in spans) - min(s[0] for s in spans)) / usable, 3) \
                if spans else 0.0
            cap = counted[0]["settings"]["speed_max_rad_s"] if counted else math.inf
            speed[name] = round(max((a["top_speed_rad_s"][name] for a in counted), default=0.0) / cap, 3)
        checks["coverage"] = all(value >= COVERAGE_FRACTION for value in coverage.values())
        checks["speed"] = all(value >= SPEED_FRACTION for value in speed.values())
    return {"verdict": "PASS" if all(checks.values()) else "FAIL", "checks": checks,
            "following_s": round(follow, 3), "cycles": len(counted), "coverage": coverage,
            "speed_fraction": speed, "requalify": requalify,
            "attempts": [a["id"] for a in counted] + [d["id"] for d in drills.values() if d],
            "gate_settings": counted[0].get("gate_settings") if counted else None}


# -- command line ---------------------------------------------------------------------


def identity_of(meta: Mapping[str, Any]) -> dict[str, str]:
    return {"serial": str(meta.get("serial")), "firmware": str(meta.get("firmware")),
            "sdk_binding_version": str(meta.get("sdk_binding_version"))}


def ledger_for(meta: Mapping[str, Any]) -> Any:
    gate = stage_gate()
    practice = bool(meta.get("practice"))
    return gate.Ledger(gate.commissioning_root(), str(meta.get("serial")), practice=practice)


def is_requalification(ledger: Any, stage_key: str, identity: Mapping[str, str]) -> bool:
    """A firmware or SDK binding change after this stage passed: 120 s + drills."""
    previous = ledger.latest(stage_key)
    if not previous or previous.get("verdict") != "PASS":
        return False
    before = previous.get("identity") or {}
    return before.get("serial") == identity["serial"] and (
        before.get("firmware") != identity["firmware"]
        or before.get("sdk_binding_version") != identity["sdk_binding_version"])


def bridge_config(args: argparse.Namespace, record: AcceptanceRecord | None) -> K1BridgeConfig:
    approved = record.data.get("approved") if record is not None else None
    approved = approved if isinstance(approved, dict) else {}

    def setting(given: float | None, key: str) -> Any:
        return given if given is not None else approved.get(key, NO_RECORD[key])

    config = K1BridgeConfig(kp=setting(args.kp, "kp"), kd=setting(args.kd, "kd"),
                            speed_max_rad_s=setting(args.max_speed, "speed_max_rad_s"),
                            accel_max_rad_s2=setting(args.max_accel, "accel_max_rad_s2"))
    config.validate()
    if record is not None:
        refused = record.problems() or record.setting_mismatches(config, commission.binding_version())
        if refused:
            raise K1BridgeError("acceptance record refuses this run:\n  " + "\n  ".join(refused))
        config = record.bridge_config(config)
    live_safety_config(config)  # refuses a stopping bound above 1.5 s before anything connects
    return config


def practice_transport() -> Any:
    """The fake robot in real time; a state is read just after it arrived, never before."""
    fake = commission.PracticeTransport(streaming_clock=lambda: time.monotonic() - 0.001)
    fake.feed(commission.PRACTICE_POSE, time.monotonic(), mode=commission.ROBOT_MODES["walking"])
    return fake


def open_rig(meta: Mapping[str, Any], model: Path | None) -> Any:
    if meta["transport"] != "booster":
        return commission.Rig(practice_transport(), time.monotonic, time.sleep, False, model)
    if REAL_TRANSPORT is not None:
        return commission.Rig(REAL_TRANSPORT(meta), time.monotonic, time.sleep, True, model)
    return commission.open_rig(meta, virtual=False, model=model)


def run_inputs(session: Any, args: argparse.Namespace) -> tuple[
        Path | None, AcceptanceRecord | None, K1BridgeConfig, dict[str, Any], list[str]]:
    """The model, record, bridge settings, gate settings and summary warnings of a run
    (refuses as a run would).  The start gates are warnings: a missing acceptance
    record and the stage gate's problems; practice is never gated."""
    commission.check_versions(session)
    real = session.meta["transport"] == "booster"
    model = commission.session_model(session, args.model)
    if real and model is None:
        raise commission.CommissionError("a real live run needs the K1 model (contact check, MJCF hash)")
    record = AcceptanceRecord.load(args.acceptance_record) if args.acceptance_record else None
    config = bridge_config(args, record)  # a record that is given must still be valid
    settings = gate_settings(config, sha256_file(model) if model else None)
    warnings: list[str] = []
    if real and record is None:
        warnings.append("no acceptance record: attempt 049's settings unless given "
                        "(kp 40, kd 2, 2 rad/s, 8 rad/s^2, 50 Hz)")
    if real:
        warnings += [f"stage gate live-arms@{args.stage}: {problem}" for problem in stage_gate().require(
            f"live-arms@{args.stage}", ledger=ledger_for(session.meta),
            identity=identity_of(session.meta), settings=settings)]
    return model, record, config, settings, warnings


def lock_reasons(session_path: Path, stage: str, record: Path | None) -> list[str]:
    """Why a live run of ``stage`` would be refused now (``[]``: it may start); the window
    shows these.  Stage-gate problems are not reasons any more: the run shows them as
    warnings before Go."""
    try:
        session = commission.Session(session_path)
        argv = ["--session", str(session_path), "--stage", stage]
        args = build_parser().parse_args(argv + (["--acceptance-record", str(record)] if record else []))
        run_inputs(session, args)
        return []
    except (commission.CommissionError, K1BridgeError, OSError, ValueError) as exc:
        return [str(exc)]


def quick_session(args: argparse.Namespace) -> Any:
    """Auto-check 1, part 1: the existing probe (read-only) makes a new session."""
    root = Path(args.root)
    before = set(root.iterdir()) if root.is_dir() else set()
    print("Auto-check 1 (robot): probe, read-only", flush=True)
    report_checks(("answers", None, "probing the robot (read-only)..."))
    try:
        code = commission.run_probe(argparse.Namespace(
            transport=args.transport, net=args.net, domain_id=args.domain_id, session=None,
            root=root, model=args.model,
            seconds=QUICK_PROBE_S if args.transport == "booster" else None,
            attested_robot_id=args.attested_robot_id, firmware=args.firmware))
    except (commission.CommissionError, K1BridgeError) as exc:
        report_checks(("answers", False, f"the probe failed: {exc}"))
        raise
    made = [path for path in set(root.iterdir()) - before if (path / "session.json").is_file()] \
        if root.is_dir() else []
    if code != 0 or len(made) != 1:
        report_checks(("answers", False, "the probe did not pass (its lines are in the log)"))
        raise commission.CommissionError("auto-check 1 (robot) failed: the probe did not pass "
                                         "(see its checks above); fix it, then check again")
    print(f"Auto-check 1 (robot): probe PASS, session {made[0]} (the quick start goes on; "
          "ignore the probe's 'Next:' line)", flush=True)
    return commission.Session(made[0])


def run_live(args: argparse.Namespace) -> int:
    clock = time_namespace_problem()  # stream clock rule 1, before any stream input
    if clock:
        raise K1BridgeError(clock)
    session = commission.Session(args.session) if args.session else quick_session(args)
    meta = session.meta
    model, record, config, settings, warnings = run_inputs(session, args)
    model_sha = settings["model_sha256"]
    receiver = StreamReceiver(model_hash=model_hash_prefix(model_sha) if model_sha else None,
                              limits=stream_limits(model))
    sock = open_stream(args.port)
    stop = threading.Event()
    thread = threading.Thread(target=serve, args=(sock, receiver, time.monotonic, stop),
                              name="k1-live-stream", daemon=True)
    thread.start()
    try:
        with motion_lock():
            rig = open_rig(meta, model)
            try:
                rig.contact = contact_check(model) if model else None
                return attempt(session, rig, receiver, args, config, settings, record,
                               warnings=warnings)
            finally:
                rig.transport.close()
    finally:
        stop.set()
        thread.join(timeout=1.0)
        sock.close()


def checklist(stage: str) -> str:
    return CHECKLIST.format(stage=stage, roles=ROLES[stage], support=SUPPORT[stage])


def head_available() -> bool:
    """The head module (package C) is in this build; without it ``--no-head`` does nothing."""
    try:
        import mocap_g1.k1_head  # noqa: F401
    except Exception:  # noqa: BLE001 - absent or broken: the window hides the tick box
        return False
    return True


def robot_problems(rig: Any, config: K1BridgeConfig) -> list[str]:
    """Auto-check 1 on the open connection (read-only): mode, fall state, a fresh joint
    state, arm and head joints inside their ranges."""
    transport = rig.transport
    mode, fall, state = transport.latest_mode(), transport.latest_fall(), transport.latest_state()
    now = rig.clock()  # after the readings: one stored while they are read is not from the future
    names = {value: name for name, value in ROBOT_MODES.items()}
    allowed = " or ".join(names.get(value, str(value)) for value in sorted(config.allowed_modes))
    found = []
    if mode is None or mode.value not in config.allowed_modes:
        found.append(f"robot mode is {names.get(mode.value, mode.value) if mode else 'unknown'}: "
                     f"set {allowed} with the remote")
    if fall is None or fall.value != FALL_READY:
        found.append("the robot reports a fall (or no fall state): stand it up, then check again")
    if state is None or not 0 <= now - state.receive_time_s < config.state_stale_timeout_s:
        found.append("no fresh joint state from the robot")
        return found
    for name, low, high in robot_joint_ranges(commission.ROBOT):
        if name in ARMS or name in HEAD_JOINTS:
            value = state.positions[K1_JOINTS.index(name)]
            if not low - RANGE_TOLERANCE_RAD <= value <= high + RANGE_TOLERANCE_RAD:
                found.append(f"{name} {value:.3f} rad is outside its range {low:.2f} to {high:.2f}")
    return found


def check_stream(run: LiveRun) -> str | None:
    """Auto-check 2 (read-only): the start gate's conditions once, within ``STREAM_CHECK_S``
    (fresh, calibrated, tracking on, model, stream clock); the arm gap is shown, not judged."""
    start = run.rig.clock()
    shown = -math.inf
    while True:
        run.rig.sleeper(CONTROL_DT)
        now = run.rig.clock()
        run.pump(now)
        try:
            problem, _gap = run.gate_problem(now)
        except K1BridgeError:  # the stream clock (rule 3): show the line, then refuse
            report_checks(*stream_items(run, None))
            raise
        done = problem is None or now - start >= STREAM_CHECK_S
        if done or now - shown >= CHECKS_EVERY_S:
            shown = now
            report_checks(*stream_items(run, problem))
        if done:
            return problem


def stream_items(run: LiveRun, problem: str | None) -> list[tuple[str, bool | None, str]]:
    """Auto-check 2 as checklist lines, from the receiver and the gaps ``gate_problem`` just
    measured (display only: ``gate_problem`` decides)."""
    view = run.receiver.view(run.rig.clock())
    packet = view.packet
    wrong_model = run.receiver.drops.get("model_or_joint_order", 0)
    if packet is None:
        sending = (False, "nothing received" + (f" ({wrong_model} packets of another model "
                                               "dropped)" if wrong_model else ""))
    elif problem == "stream stale":
        sending = (False, "packets arrive, but the motion is frozen or late")
    else:
        sending = (True, "fresh")
    model = ((True, "the stream's model is this K1 model") if packet is not None
             else (False, "the stream is for another robot model or joint order")
             if wrong_model else (None, "waits for the stream"))
    clock = ((False, view.clock_error) if view.clock_error
             else (True, "same clock") if packet is not None else (None, "waits for the stream"))
    calibrated = ((None, "waits for the stream") if packet is None
                  else (True, f"calibration {packet.calibration_epoch}")
                  if packet.calibration_epoch else (False, "not calibrated"))
    tracking = ((None, "waits for the stream") if packet is None
                else (True, "on") if packet.enabled else (False, "off"))
    gaps = run.last_gaps
    arms: tuple[bool | None, str]
    if gaps:
        worst = max(gaps, key=lambda name: gaps[name])
        arms = (True, f"gap {gaps[worst]:.2f} rad at {worst}: after Go the robot's arms move "
                "there slowly")
    else:
        arms = (None, "no joint state" if problem == "no joint state"
                else "waits for the lines above")
    return [("sending", *sending), ("calibrated", *calibrated), ("tracking", *tracking),
            ("model", *model), ("clock", *clock), ("arms", *arms)]


def go_summary(meta: Mapping[str, Any], args: argparse.Namespace, plan: Plan,
               warnings: list[str], head: str) -> str:
    """The one Go screen: what was checked, the settings, the per-attempt lines, warnings."""
    config = plan.config
    lines = [
        ("REAL ROBOT" if meta["transport"] == "booster" else "PRACTICE (fake robot)")
        + f" {meta.get('serial')}, firmware {meta.get('firmware')}",
        "Auto-check 1 (robot): PASS (walking, not fallen, fresh joint state, joints in range)",
        "Auto-check 2 (stream): PASS (fresh, calibrated, tracking on, model, clock)",
        f"Settings: kp {config.kp:g}, kd {config.kd:g}, speed cap {config.speed_max_rad_s:g} rad/s, "
        f"acceleration cap {config.accel_max_rad_s2:g} rad/s^2, {1.0 / CONTROL_DT:g} Hz, "
        f"stage {plan.stage}" + (f", drill {plan.drill}" if plan.drill else "")
        + f", time limit {plan.time_limit_s:g} s",
        f"Head follows: {head}",
        f"Support: {args.support}",
        "Strap: " + ("clear of both arms (full arm range)" if plan.strap_clear
                     else f"NOT clear: shoulders stay within {HOIST_SHOULDER_RAD:g} rad of rest"),
        f"Roles: {ROLES[plan.stage]}",
        "After Go the robot's arms move slowly (0.2 rad/s) to the performer's arm pose, then "
        "follow. The performer holds still until the status says following.",
        "STOP: Ctrl-C here (or the window's STOP). The real stop is the remote: LT+BACK.",
    ]
    lines += ["Warnings:"] + [f"  - {text}" for text in warnings] if warnings else ["Warnings: none"]
    return "\n".join(lines)


def find_capture(root: Path, session_id: int | None) -> tuple[dict[str, Any] | str, str | None]:
    """The suit capture whose ``capture.json`` names the stream session ``session_id`` (plan B2):
    ``({"folder", "capture_id"}, None)``, or ``("not found" | "ambiguous", warning)``. Never
    refuses and never picks one of several."""
    found = []
    for meta_path in sorted(Path(root).glob("*/capture.json")):
        try:
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        if session_id is not None and meta.get("session_id") == session_id:
            found.append(meta_path.parent)
    if len(found) == 1:
        return {"folder": str(found[0].resolve()), "capture_id": found[0].name}, None
    if not found:
        return "not found", (f"suit recording not found: no capture in {root} for stream session "
                             f"{session_id}; this run's suit data will be missing")
    return "ambiguous", (f"suit recording ambiguous: {len(found)} captures in {root} for stream "
                         f"session {session_id} ({', '.join(p.name for p in found)})")


def owner_go(args: argparse.Namespace, summary: str, warnings: list[str], session: Any,
             settings: Mapping[str, Any]) -> dict[str, Any]:
    """The owner's Go for exactly this summary, session and settings (no Go: refused)."""
    text = "QUICK START SUMMARY\n" + summary
    if args.go:
        print(text, flush=True)
        via = "--go"
    elif commission.confirm(text + "\nGo starts the arms now."):
        via = "question"
    else:
        raise commission.CommissionError("not confirmed")
    return {"operator": args.operator or getpass.getuser(), "time": commission.now_iso(),
            "via": via, "warnings": list(warnings), "session": str(session.path),
            "settings": dict(settings), "summary": summary}


def attempt(session: Any, rig: Any, receiver: StreamReceiver, args: argparse.Namespace,
            config: K1BridgeConfig, settings: dict[str, Any], record: AcceptanceRecord | None, *,
            warnings: list[str] | tuple[str, ...] = (),
            pump: Callable[[float], None] | None = None) -> int:
    """Auto-checks, Go, attempt file, gate wait, takeover, follow, release, verdict."""
    meta = session.meta
    try:
        wait_for_readings(rig.transport, READINGS_TIMEOUT_S)
    except K1BridgeError as exc:
        report_checks(("answers", False, str(exc)))
        raise
    problems = commission.identity_problems(rig, meta, args.attested_robot_id)
    if problems:
        report_checks(("answers", False, "another robot than this session's: " + "; ".join(problems)))
        raise commission.CommissionError("this is not the session's robot:\n  " + "\n  ".join(problems))
    problems = robot_problems(rig, config)
    report_checks(("answers", True, f"robot {meta.get('serial')} answers"),
                  ("robot", not problems, "; ".join(problems) or "walking, not fallen, joints in range"))
    strap_clear = args.stage != "hoist" or not args.strap_not_clear
    plan = Plan(args.stage, args.drill, args.time_limit, strap_clear, config,
                head=not args.no_head)
    run = LiveRun(rig, receiver, plan, pump=pump)
    problem = check_stream(run)  # read-only, also after a failed check 1: the window shows both
    if problems:
        raise commission.CommissionError("auto-check 1 (robot) failed; fix this, then check "
                                         "again:\n  " + "\n  ".join(problems))
    print("Auto-check 1 (robot): PASS", flush=True)
    if problem:
        raise commission.CommissionError(f"auto-check 2 (stream) failed: {problem}; fix it "
                                         "(live tool, Calibrate, Tracking on, arms down), then "
                                         "check again")
    print("Auto-check 2 (stream): PASS", flush=True)
    head = ("off" if args.no_head else "on") if head_available() else "off (no head code in this build)"
    packet = receiver.view(rig.clock()).packet  # the stream the checks just passed
    suit, suit_warning = find_capture(args.suit_root or SUIT_ROOT,
                                      None if packet is None else packet.session_id)
    if suit_warning is not None:
        warnings = [*warnings, suit_warning]
    go = owner_go(args, go_summary(meta, args, plan, list(warnings), head), list(warnings),
                  session, settings)
    run_settings = {"stage": args.stage, "drill": args.drill, "time_limit_s": args.time_limit,
                    "strap_clear": strap_clear, "kp": config.kp, "kd": config.kd,
                    "speed_max_rad_s": config.speed_max_rad_s, "accel_max_rad_s2": config.accel_max_rad_s2,
                    "port": args.port, "roles": ROLES[args.stage], "support": args.support,
                    "head": head, "align_speed_rad_s": ALIGN_SPEED_RAD_S}
    started = session.start(plan.key, run_settings, {})
    started.data["gate_settings"] = settings
    started.data["owner_go"] = go
    if isinstance(suit, dict):
        suit.update(go_monotonic=time.monotonic(), release_monotonic="unknown")
    started.data["suit_recording"] = suit
    commission.atomic_write(started.path, started.data)  # an interrupted attempt keeps Go and the capture
    if plan.drill:
        started.data["expected_fault"] = {"drill": plan.drill, "fault": EXPECTED_FAULT[plan.drill]}
    rig.instrument()
    log_path = started.path.with_suffix(".jsonl")
    with log_path.open("w", encoding="utf-8") as log:
        run.log = log
        with sigint_gate() as gate:
            try:
                gate.allow_interrupt()
                why = run.wait_for_gate()  # stream and joint state again, right before the takeover
                changed = robot_problems(rig, config) if why is None else []
                if changed:
                    why = "the robot changed since the checks: " + "; ".join(changed)
            except K1BridgeError as exc:
                why = str(exc)
            except KeyboardInterrupt:
                why = "operator stop before the takeover"
            gate.defer()
            if why is not None:
                ended(suit)
                started.finish("FAIL", reason=why, release_confirmed=True, gate_settings=settings,
                               tick_log=str(log_path))
                return commission.report_end(session, started, "FAIL", why, next_step=None)
            follow(run, gate, rig)
            ended(suit)
    rpc = [call["seconds"] for call in rig.rpc if not call["enable"] and call["ok"]]
    released = run.bridge.state in ("released", "fault_latched", "idle")
    if plan.drill and released:
        run.watch_after_release(0.3)
    summary = summarize(plan, run, release_confirmed=released, rpc_seconds=max(rpc, default=0.0))
    outcome, reason = verdict(plan, summary)
    revoke = revoking_reason(summary)
    started.finish(outcome, **summary, reason=reason, rpc=rig.rpc, gate_settings=settings,
                   tick_log=str(log_path), log_sha256=sha256_file(log_path))
    if not released:
        print(MANUAL_STOP, file=sys.stderr)
    if revoke is not None:
        revoke_gate(meta, f"live-arms@{args.stage}", revoke, started.path)
    print(f"Stream: {json.dumps(summary['stream'])}; following {summary['following_s']} s")
    print("Suit recording: " + (suit["folder"] if isinstance(suit, dict) else suit))
    return commission.report_end(session, started, outcome, reason, next_step=None)


def ended(suit: dict[str, Any] | str) -> None:
    """The release time (this PC's monotonic clock, as the suit capture) for the export."""
    if isinstance(suit, dict):
        suit["release_monotonic"] = time.monotonic()


def release_state(rig: Any) -> str:
    """``none``, ``requested``, ``confirmed`` or ``failed``: the isolated transport's own
    status when it has one, else the last timed release RPC of the rig."""
    status = getattr(rig.transport, "action_status", None)
    if status is not None:
        return str(status("release").state)
    calls = [call for call in rig.rpc if not call["enable"]]
    return "none" if not calls else "confirmed" if calls[-1]["ok"] else "failed"


def final_status(run: LiveRun, rig: Any) -> None:
    events = [text for _t, text in getattr(rig.transport, "events", ())]
    run.status(rig.clock(), "released" if run.bridge.state in ("released", "idle") else run.bridge.state,
               force=True, trigger=run.result.trigger, fault=run.bridge.fault,
               release_events=events[-6:])


def follow(run: LiveRun, gate: Any, rig: Any) -> None:
    """Take over and follow; every exit path ends with the arms handed back."""
    try:
        _follow(run, gate, rig)
    finally:
        run.end_head()  # after the release: neutral (normal stop only), then close
        final_status(run, rig)


def _follow(run: LiveRun, gate: Any, rig: Any) -> None:
    bridge = run.bridge
    try:
        run.run(gate)
    except K1BridgeError as exc:  # the takeover failed; its own path released
        run.result.trigger = f"takeover_failed:{exc}"
        return
    except KeyboardInterrupt:
        gate.defer()
        run.result.trigger = "operator_stop"
        print("Stopping: decelerating, then releasing the arms.", flush=True)
    except ArmBridgeError as exc:  # lost real time (> 0.1 s overrun)
        gate.defer()
        bridge.abort(f"real_time_lost:{exc}")
    except BaseException as exc:  # a bug or an I/O error: the arms go back first
        gate.defer()
        commission.abort_unexpected(bridge, exc)
        raise
    gate.defer()
    if bridge.state == "tracking":
        run.status(rig.clock(), "stopping", force=True, trigger=run.result.trigger,
                   release="requested")
        bridge.stop()
        run.result.released_at = rig.clock()
    run.result.trigger = run.result.trigger or bridge.fault


def revoke_gate(meta: Mapping[str, Any], stage_key: str, reason: str, evidence: Path) -> None:
    gate = stage_gate()
    ledger = ledger_for(meta)
    for prerequisite in gate.STAGES[stage_key].gate:
        ledger.record_revocation(prerequisite, reason=f"{stage_key}: {reason}",
                                 evidence={"attempt": evidence})
        print(f"REVOKED {prerequisite}: {reason} in {stage_key}", file=sys.stderr)


def run_evaluate(args: argparse.Namespace) -> int:
    session = commission.Session(args.session)
    meta = session.meta
    stage_key = f"live-arms@{args.stage}"
    identity = identity_of(meta)
    ledger = ledger_for(meta)
    report = evaluate(session.attempts(), args.stage,
                      requalify=is_requalification(ledger, stage_key, identity))
    for name, ok in report["checks"].items():
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    print(json.dumps({k: report[k] for k in ("following_s", "cycles", "coverage", "speed_fraction",
                                             "requalify")}, indent=2))
    if report["verdict"] != "PASS":
        print(f"{stage_key}: FAIL (not written to the ledger)")
        return 1
    if meta["transport"] == "booster":
        refused = stage_gate().require(stage_key, ledger=ledger, identity=identity,
                                       settings=report["gate_settings"])
        if refused:
            raise commission.CommissionError("the stage gate no longer allows this stage:\n  "
                                             + "\n  ".join(refused))
    evidence: dict[str, Path] = {}
    for attempt_id in report["attempts"]:
        path = session.path / "attempts" / f"{attempt_id}.json"
        evidence[f"attempt-{attempt_id}"] = path
        if path.with_suffix(".jsonl").is_file():
            evidence[f"ticks-{attempt_id}"] = path.with_suffix(".jsonl")
    entry = ledger.record_pass(stage_key, identity=identity, settings=report["gate_settings"],
                               evidence=evidence, operator=args.operator,
                               extra={"session": str(session.path), "requalify": report["requalify"],
                                      "following_s": report["following_s"], "cycles": report["cycles"]})
    print(f"{stage_key}: PASS written to {ledger.path}"
          + (" (PRACTICE: unlocks nothing)" if meta.get("practice") else ""))
    return 0 if entry else 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--session", type=Path,
                        help="a session folder probe created (default: the quick start probes "
                             "the robot and makes a new one)")
    parser.add_argument("--transport", choices=("fake", "booster"), default="fake",
                        help="quick start: fake = the practice robot, booster = the real K1")
    parser.add_argument("--net", default="", help="quick start, booster: this PC's interface or IP")
    parser.add_argument("--domain-id", type=int, default=0, help="quick start: DDS domain")
    parser.add_argument("--root", type=Path, default=commission.commissioning_root(),
                        help="quick start: where the new session is made")
    parser.add_argument("--firmware", help="quick start, with --attested-robot-id: the firmware")
    parser.add_argument("--go", action="store_true",
                        help="the owner's Go: start after the two auto-checks without asking")
    parser.add_argument("--no-head", action="store_true", help="the head does not follow")
    parser.add_argument("--support", default=SUPPORT_DEFAULT,
                        help=f"how the robot is held (Go screen; default: {SUPPORT_DEFAULT})")
    parser.add_argument("--strap-not-clear", action="store_true",
                        help="hoist: the strap may touch the arms (shoulders stay within 0.6 rad)")
    parser.add_argument("--stage", choices=("hoist", "standing"), default="hoist")
    parser.add_argument("--drill", choices=DRILLS,
                        help="stream-loss: stop the live tool while following; stop: press STOP")
    parser.add_argument("--evaluate", action="store_true",
                        help="judge the session against the stage PASS and write the ledger entry")
    parser.add_argument("--operator", help="who gives the Go; with --evaluate: who signs the ledger")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="loopback UDP port")
    parser.add_argument("--suit-root", type=Path,
                        help="the live tool's suit captures (default: artifacts/xsens-live/suit)")
    parser.add_argument("--time-limit", type=float, default=TIME_LIMIT_S,
                        help=f"seconds of following (max {MAX_TIME_LIMIT_S:g})")
    parser.add_argument("--acceptance-record", type=Path, help="booster: the S0 acceptance record")
    parser.add_argument("--attested-robot-id", help="the serial, when the robot cannot report it")
    parser.add_argument("--model", type=Path, help="the K1 MJCF (default: the session's)")
    parser.add_argument("--kp", type=float)
    parser.add_argument("--kd", type=float)
    parser.add_argument("--max-speed", type=float)
    parser.add_argument("--max-accel", type=float)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not (math.isfinite(args.time_limit) and 0 < args.time_limit <= MAX_TIME_LIMIT_S):
        parser.error(f"--time-limit must be 0-{MAX_TIME_LIMIT_S:g} s")
    if not 1024 <= args.port <= 65535:
        parser.error("--port must be 1024-65535")
    if args.evaluate and not args.operator:
        parser.error("--evaluate needs --operator")
    if args.evaluate and args.session is None:
        parser.error("--evaluate needs --session")
    try:
        return run_evaluate(args) if args.evaluate else run_live(args)
    except (commission.CommissionError, K1BridgeError) as exc:
        print(f"REFUSED: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
