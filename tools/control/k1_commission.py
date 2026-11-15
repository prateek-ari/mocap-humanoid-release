#!/usr/bin/env python3
"""Step-by-step Booster K1 hardware tests: the day-one commissioning on a hoist.

The guide is docs/k1-bringup-runbook.md.  Each step is a subcommand and writes
its evidence into a session folder; ``record`` turns valid evidence into the
acceptance record that unlocks real replay (tools/control/k1_arm_bridge.py).

    python3 tools/control/k1_commission.py preflight [--transport booster --net 192.168.10.10]
    python3 tools/control/k1_commission.py probe [--transport booster --net 192.168.10.10]
    python3 tools/control/k1_commission.py status --session DIR
    python3 tools/control/k1_commission.py <step> --session DIR

``--transport fake`` (the default for preflight and probe) practises every step
on a fake robot: the session it creates can never unlock a real robot.  Every
step after ``probe`` takes the transport from the session.

Motion steps reuse mocap_g1.k1_bridge.K1ArmBridge (envelope, per-tick checks,
release on any fault) with fixed low limits, show the simulated plan before the
takeover, and ask questions only before the takeover or after the release.
Keep a second person on the remote (LT+BACK = damping) during every motion step.

Exit status: 0 PASS, 1 FAIL (2 lines tell why and what to check), 2 refused.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
import datetime as dt
import ipaddress
import json
import math
import os
from pathlib import Path
import platform
import re
import secrets
import subprocess
import sys
import time
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from mocap_g1.arm_bridge import ArmBridgeError  # noqa: E402
from mocap_g1.io_utils import sha256_file, write_json_atomic  # noqa: E402
from mocap_g1.k1_bridge import (  # noqa: E402
    COMMAND_CONTRACT,
    EXAMPLE_KD,
    EXAMPLE_KP,
    ROBOT_MODES,
    AcceptanceRecord,
    BoosterTransport,
    FakeK1Transport,
    K1ArmBridge,
    K1BridgeConfig,
    K1BridgeError,
    K1State,
    LocoError,
    binding_version,
    motion_lock,
    normalized_arm_pose,
)
from mocap_g1.live_robots import (  # noqa: E402
    K1_JOINTS,
    K1_UPPER_BODY_JOINTS as ARMS,
    LIVE_ROBOTS,
    robot_envelope_limits,
)
from mocap_g1.safety import ArmSafetyEnvelope, SafetyConfig  # noqa: E402
from mocap_g1.stage_gate import Ledger, commissioning_root  # noqa: E402
from tools.control.g1_arm_bridge import iter_dds_replay_ticks, sigint_gate  # noqa: E402
from tools.control.k1_arm_bridge import (  # noqa: E402
    MANUAL_STOP,
    READINGS_TIMEOUT_S,
    contact_check,
    probe as probe_robot,
    resolve_model,
    wait_for_readings,
)

ROBOT = LIVE_ROBOTS["booster_k1"]
LIMITS = {name: limit for name, limit in robot_envelope_limits(ROBOT).items() if name in ARMS}
MARGIN = SafetyConfig().position_margin_rad
CONTACT_TOLERANCE_M = SafetyConfig().contact_tolerance_m
ROBOT_IP = "192.168.10.102"  # Booster's documented default; --robot-ip if the robot differs
EXPECTED_SDK = "1.6.3"  # the pyproject.toml "booster" extra
CONTROL_DT = 0.02

# Hard limits of the commissioning steps (fixed; not flags).
SAFE = {"kp": 20.0, "kd": 1.0, "speed_max_rad_s": 0.3, "accel_max_rad_s2": 1.0}
SETTLE_MAX_RAD = 0.10       # settling into the envelope margin, per joint
POSE_MATCH_RAD = 0.10       # confirmed pose vs the pose after the takeover / settling
SETTLE_S = 1.5
HOLD_MAX_S = 60.0
BALANCE_S = 30.0
SIGNS_RAD = 0.20  # 11 deg (0.3 hit the torso in the contact check): 0.1 rad was below the 0.16 rad hold error seen on the K1 (2026-11-15)
#: Shoulder pitch (lifting the arm) gets a bigger, visible move; the other joints keep SIGNS_RAD.
SIGNS_LIFT_RAD = 0.60
TPOSE_HOLD_S = 3.0


def signs_amplitude(name: str) -> float:
    return SIGNS_LIFT_RAD if name.endswith("Shoulder_Pitch") else SIGNS_RAD
GAP_LEVELS = (0.1, 0.25, 0.5, 1.0)
#: Slowest fall-state period the probe accepts. The real K1 (firmware 1.8.1.2) publishes it at
#: 1 Hz; motion then treats a reading older than 3 periods as unknown (bridge_config).
FALL_PERIOD_MAX_S = 1.5
#: Quick-release waits this long after the takeover before the release (Booster waits 0.5 s).
TAKEOVER_SETTLE_S = 0.5
GAP_RECORD_MIN_S = 0.5
GAP_DRIFT_RAD = 0.10
GAP_START_DRIFT_RAD = 0.05
OVERRUN_S = 0.10
POST_TAKEOVER_SIM_S = 0.10
RELEASE_WATCH_S = 2.0
#: Gains levels: (excursion rad, speed rad/s, acceleration rad/s^2).
GAINS_LEVELS = {"L1": (0.10, 0.25, 1.0), "L2": (0.20, 0.45, 1.5), "L3": (0.30, 0.60, 2.0),
                "L4": (0.45, 1.00, 3.0),  # L4: live arms need more than 0.6 rad/s (2026-11-15)
                # L5/L6: a recorded performer (Session-002 on K1) exceeds 1 rad/s 7 % and 3 rad/s^2 16 %
                # of the time; 3 rad/s and 12 rad/s^2 cover ~99 %. Arm motors: 33.5 rad/s, 14 Nm.
                "L5": (0.75, 2.00, 8.0), "L6": (1.00, 3.00, 12.0)}
GAINS_MAX_ERROR_RAD, GAINS_P95_RAD = 0.2, 0.1
SUSTAIN_TICKS = 5           # 0.1 s at 50 Hz
COVERAGE = 0.9
ZERO_TOLERANCE_DEG = 5.0
ZERO_STILL_RAD = 0.01
#: The practice robot: standing, arms hanging (shoulder roll toward the body),
#: straight elbows resting on the elbow-yaw range end (0), as the real one rests.
PRACTICE_POSE = tuple(
    {"Left_Shoulder_Roll": -1.3, "Right_Shoulder_Roll": 1.3}.get(name, 0.0) for name in K1_JOINTS)

#: What to check for each fault reason (prefix match), shown on a FAIL.
HINTS = {
    "state_stale": "joint states stopped arriving: check the cable, --net, and that no other "
                   "program talks to the robot",
    "state_clock": "joint-state times run ahead of this PC's clock: rerun the step",
    "mode_unknown": "the robot mode could not be read: the loco RPC is not answering (network?)",
    "mode_not_allowed": "the robot left the takeover mode: set it again with the remote, or use "
                        "--mode to match it",
    "fall_state_unknown": "no fall-state messages: rerun probe; the fall topic must be periodic",
    "fall_state": "the robot reports a fall: stop and check the robot on the hoist",
    "tracking_error": "an arm did not follow its command by more than 0.3 rad: look for an "
                      "obstacle, check the mirror and the joint signs; gains may be too low",
    "safety_fault": "the safety envelope refused a command (see the message)",
    "source_hard_timeout": "the control loop stalled for 0.5 s: close other programs",
    "publish_failed": "sending the command failed: check the network",
    "real_time_lost": "the control loop fell behind real time: close other programs",
    "post_takeover": "the robot's pose changed during the takeover: rerun with the robot still",
    "not_settled": "the arms did not reach the start pose: look for an obstacle; check the gains",
    "takeover_failed": "UpperBodyCustomControl(true) failed: is the robot in the --mode mode "
                       "(walking by default)? See guide step 5",
    "command_gap:drift": "the arms moved more than 0.1 rad while commands were stopped: the robot "
                         "does not hold the arms on command loss; send the log to the developers",
    "command_gap:contact": "the measured pose came close to the body during the gap",
    "command_gap:loop_overrun": "this PC fell behind during the gap: close other programs",
    "unexpected": "a program error: send the session folder to the developers",
}

ask: Callable[[str], str] = input  # tests replace it


class CommissionError(RuntimeError):
    """A step refused to run (bad input, missing evidence, a mismatch)."""


# -- operator ------------------------------------------------------------------


def ask_yes(question: str) -> bool:
    while True:
        reply = ask(f"{question} [y/n]: ").strip().lower()
        if reply in ("y", "yes"):
            return True
        if reply in ("n", "no"):
            return False


def confirm(text: str) -> bool:
    print(text)
    return ask("Type yes to continue (anything else stops): ").strip().lower() == "yes"


CHECKLIST = """\
Before the robot moves:
  - the robot hangs in the hoist, with the hoist taking its weight if it falls;
  - a second person holds the remote, ready for LT+BACK (damping), and watches the arms;
  - nothing and nobody within reach of the arms;
  - the robot is in {mode} mode (set with the remote);
  - the mirror is open in a second terminal (optional for this step).
If anything looks wrong at any time: LT+BACK on the remote, then Ctrl-C here."""


def hint(reason: str | None) -> str:
    if not reason:
        return ""
    for prefix in sorted(HINTS, key=len, reverse=True):
        if reason.startswith(prefix):
            return HINTS[prefix]
    return ""


# -- sessions and evidence ------------------------------------------------------


def now_iso() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds")


def atomic_write(path: Path, data: Mapping[str, Any]) -> None:
    write_json_atomic(path, data, overwrite=True)  # strict JSON; no partial file


@dataclass
class Attempt:
    path: Path
    data: dict[str, Any]

    @property
    def id(self) -> str:
        return str(self.data["id"])

    def finish(self, verdict: str, **fields: Any) -> None:
        """Replace the INCOMPLETE file at once; a failure here leaves it INCOMPLETE."""
        atomic_write(self.path, {**self.data, **fields, "verdict": verdict, "finished": now_iso()})
        self.data.update(fields, verdict=verdict)


class Session:
    """``session.json`` (robot identity and versions) and immutable attempts."""

    def __init__(self, path: Path) -> None:
        self.path = Path(path)
        try:
            self.meta = json.loads((self.path / "session.json").read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            raise CommissionError(f"{path} is not a commissioning session ({exc})") from exc

    @classmethod
    def create(cls, root: Path, meta: Mapping[str, Any]) -> Session:
        stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%d-%H%M%S")
        serial = re.sub(r"[^A-Za-z0-9_.-]", "_", str(meta["serial"])) or "unknown"
        path = Path(root) / f"{stamp}-{serial}-{secrets.token_hex(3)}"
        (path / "attempts").mkdir(parents=True)
        atomic_write(path / "session.json", meta)
        return cls(path)

    def attempts(self) -> list[dict[str, Any]]:
        found = []
        files = (self.path / "attempts").glob("[0-9][0-9][0-9]*.json")
        for file in sorted(files, key=lambda f: int(f.stem) if f.stem.isdecimal() else -1):
            try:
                data = json.loads(file.read_text(encoding="utf-8"))
            except (OSError, ValueError):  # a crash while writing: never valid
                data = {"id": file.stem, "step": "?", "verdict": "INCOMPLETE"}
            found.append(data)
        return found

    def latest(self, key: str) -> dict[str, Any] | None:
        last = None
        for attempt in self.attempts():
            if attempt.get("step") == key:
                last = attempt
        return last

    def problem(self, attempt: Mapping[str, Any] | None, key: str = "") -> str | None:
        """Why ``attempt`` is not valid evidence, or ``None``: it must be PASS and
        every attempt it used must still be the latest of its step and valid."""
        if attempt is None:
            return f"{key} has not run"
        if attempt.get("verdict") != "PASS":
            return f"{attempt.get('step')} attempt {attempt.get('id')} is {attempt.get('verdict')}"
        for used_key, used_id in (attempt.get("uses") or {}).items():
            latest = self.latest(used_key)
            if latest is None or latest.get("id") != used_id:
                return (f"{attempt.get('step')} attempt {attempt.get('id')} used {used_key} "
                        f"attempt {used_id}, which is no longer the latest: rerun it")
            inner = self.problem(latest, used_key)
            if inner:
                return inner
        return None

    def require(self, keys: list[str]) -> dict[str, str]:
        """``{key: attempt id}`` of valid prerequisites, else :class:`CommissionError`."""
        uses = {}
        for key in keys:
            latest = self.latest(key)
            problem = self.problem(latest, key)
            if problem:
                raise CommissionError(f"needs a valid {key} first: {problem}")
            uses[key] = str(latest["id"])  # type: ignore[index]
        return uses

    def start(self, key: str, settings: Mapping[str, Any], uses: Mapping[str, str]) -> Attempt:
        """Allocate the next attempt number atomically and write it as INCOMPLETE."""
        directory = self.path / "attempts"
        number = len(list(directory.glob("[0-9][0-9][0-9]*.json"))) + 1
        while True:
            path = directory / f"{number:03d}.json"
            try:
                fd = os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
            except FileExistsError:
                number += 1
                continue
            break
        data = {"id": f"{number:03d}", "step": key, "verdict": "INCOMPLETE", "started": now_iso(),
                "settings": dict(settings), "uses": dict(uses)}
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(data, handle, indent=2, sort_keys=True)
        return Attempt(path, data)


# -- robot connection -----------------------------------------------------------


class VirtualClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += max(0.0, seconds)


class PracticeTransport(FakeK1Transport):
    """The fake robot: echoes arm commands as measured positions."""

    malformed_states = 0

    def robot_info(self) -> dict[str, str]:
        return {"model": "K1", "name": "practice", "serial_number": "PRACTICE",
                "version": "practice"}


@dataclass
class Rig:
    transport: Any
    clock: Callable[[], float]
    sleeper: Callable[[float], None]
    real: bool
    model_path: Path | None
    contact: Any = None
    rpc: list[dict[str, Any]] = field(default_factory=list)
    frames: int = 0

    def instrument(self) -> None:
        """Time every takeover/release RPC and count published frames."""
        call, publish = self.transport.upper_body_control, self.transport.publish

        def timed(enable: bool) -> None:
            start = time.monotonic()
            ok, error = False, None
            try:
                call(enable)
                ok = True
            except Exception as exc:
                error = str(exc)
                raise
            finally:
                self.rpc.append({"enable": bool(enable), "ok": ok, "at_s": start,
                                 "seconds": round(time.monotonic() - start, 4), "error": error})

        def counted(frame: Any) -> None:
            publish(frame)
            self.frames += 1

        self.transport.upper_body_control = timed
        self.transport.publish = counted


def open_rig(meta: Mapping[str, Any], *, mode: str = "walking", virtual: bool = True,
             model: Path | None = None) -> Rig:
    if meta["transport"] == "booster":
        transport = BoosterTransport(domain_id=int(meta.get("domain_id", 0)),
                                     network_interface=str(meta.get("net", "")))
        return Rig(transport, time.monotonic, time.sleep, True, model)
    if virtual:
        clock = VirtualClock()
        fake = PracticeTransport(streaming_clock=clock)
        fake.feed(PRACTICE_POSE, 0.0, mode=ROBOT_MODES[mode])
        return Rig(fake, clock, clock.sleep, False, model)
    fake = PracticeTransport(streaming_clock=time.monotonic)
    fake.feed(PRACTICE_POSE, time.monotonic(), mode=ROBOT_MODES[mode])
    return Rig(fake, time.monotonic, time.sleep, False, model)


def session_model(session: Session, given: Path | None) -> Path | None:
    """The K1 MJCF this session was bound to; refuses a different file."""
    path = given or (Path(session.meta["model_path"]) if session.meta.get("model_path") else None)
    sha = sha256_file(path) if path is not None and path.is_file() else None
    if sha != session.meta.get("model_sha256"):
        raise CommissionError("the K1 model differs from the one this session was started with "
                              "(--model); start a new session with probe")
    return path


def check_versions(session: Session) -> None:
    meta = session.meta
    if meta.get("command_contract") != COMMAND_CONTRACT:
        raise CommissionError(f"session contract {meta.get('command_contract')} != {COMMAND_CONTRACT}:"
                              " the bridge changed; start a new session with probe")
    if meta.get("transport") == "booster" and binding_version() != meta.get("sdk_binding_version"):
        raise CommissionError(f"SDK bindings {binding_version()} != the session's "
                              f"{meta.get('sdk_binding_version')}; start a new session with probe")


def identity_problems(rig: Rig, meta: Mapping[str, Any], attested: str | None) -> list[str]:
    """Compare the connected robot with the session (every real motion attempt)."""
    try:
        info = rig.transport.robot_info()
    except LocoError:
        info = None
    expected = AcceptanceRecord({"robot_identity": {"serial_number": meta.get("serial")},
                                 "firmware": meta.get("firmware"),
                                 "identity_source": meta.get("identity_source")})
    return expected.identity_mismatches(info, attested)


# -- motion plans ---------------------------------------------------------------


@dataclass(frozen=True)
class Segment:
    name: str
    targets: Mapping[str, float]
    seconds: float
    joint: str | None = None
    phase: str = ""


def settle_targets(pose: Mapping[str, float]) -> dict[str, float]:
    """Each arm joint moved inward into the envelope's margin-bounded range."""
    return {name: min(max(value, LIMITS[name].lower + MARGIN), LIMITS[name].upper - MARGIN)
            for name, value in pose.items()}


def room(settled: Mapping[str, float], name: str, excursion: float) -> int:
    """+1 or -1: the direction an ``excursion`` fits inside the margin, 0 if neither."""
    limit = LIMITS[name]
    if settled[name] + excursion <= limit.upper - MARGIN + 1e-9:
        return 1
    if settled[name] - excursion >= limit.lower + MARGIN - 1e-9:
        return -1
    return 0


def ticks(seconds: float) -> int:
    return max(1, round(seconds / CONTROL_DT))


def simulate(start: Mapping[str, float], plan: list[Segment], config: K1BridgeConfig,
             contact: Any = None, deadline: float | None = None) -> list[dict[str, Any]]:
    """The envelope's commands for ``plan`` from ``start`` (deterministic, 50 Hz)."""
    envelope = ArmSafetyEnvelope(LIMITS, config.safety_config(LIMITS), joints=ARMS)
    envelope.reset(dict(start), now=0.0)
    samples, now = [], 0.0
    for index, segment in enumerate(plan):
        for _ in range(ticks(segment.seconds)):
            if deadline is not None and time.perf_counter() > deadline:
                raise CommissionError("plan simulation missed its deadline")
            now += CONTROL_DT
            command = envelope.step(dict(segment.targets), source_timestamp=now, now=now,
                                    enable=True, contact=contact)
            samples.append({"t": now, "segment": index, "positions": command.positions,
                            "velocities": command.velocities, "contact": command.contact})
    return samples


def plan_problems(start: Mapping[str, float], plan: list[Segment],
                  samples: list[dict[str, Any]]) -> list[str]:
    """Settling moves only inward (≤ SETTLE_MAX_RAD); later samples stay inside the
    margin; the contact check never brakes."""
    settled = plan[0].targets
    found = [f"{name} would settle {abs(settled[name] - start[name]):.3f} rad (max {SETTLE_MAX_RAD})"
             for name in ARMS if abs(settled[name] - start[name]) > SETTLE_MAX_RAD + 1e-9]
    for sample in samples:
        for name, value in sample["positions"].items():
            limit = LIMITS[name]
            if sample["segment"] == 0:
                low, high = sorted((start[name], settled[name]))
                if not low - 1e-6 <= value <= high + 1e-6:
                    found.append(f"{name} would move away from its settling path at "
                                 f"t={sample['t']:.2f} s")
            elif not limit.lower + MARGIN - 1e-6 <= value <= limit.upper - MARGIN + 1e-6:
                found.append(f"{name}={value:.3f} would leave the margin at t={sample['t']:.2f} s")
        if sample["contact"] not in ("clear", "unchecked"):
            found.append(f"the contact check would brake at t={sample['t']:.2f} s "
                         f"({sample['contact']})")
    return sorted(set(found))[:12]


def describe_plan(start: Mapping[str, float], plan: list[Segment],
                  samples: list[dict[str, Any]]) -> str:
    settled = plan[0].targets
    lines = [f"Plan: {len(plan)} segments, {sum(seg.seconds for seg in plan):.1f} s; "
             "angles in degrees",
             f"  {'joint':24s} {'now':>7s} {'settle':>7s} {'max move':>9s} {'total':>7s}"]
    for name in ARMS:
        after = [s["positions"][name] for s in samples if s["segment"] > 0] or [settled[name]]
        excursion = max(abs(value - settled[name]) for value in after)
        total = max(abs(value - start[name]) for value in after)
        lines.append(f"  {name:24s} {math.degrees(start[name]):7.1f} "
                     f"{math.degrees(settled[name] - start[name]):+7.1f} "
                     f"{math.degrees(excursion):9.1f} {math.degrees(total):7.1f}")
    return "\n".join(lines)


def hold_plan(settled: Mapping[str, float], seconds: float) -> list[Segment]:
    return [Segment("settle", settled, SETTLE_S), Segment("hold", settled, seconds)]


def signs_plan(settled: Mapping[str, float]) -> list[Segment]:
    plan = [Segment("settle", settled, SETTLE_S)]
    for name in ARMS:
        amp = signs_amplitude(name)
        direction = room(settled, name, amp)
        if not direction:
            raise CommissionError(f"no room to move {name} by {amp} rad from "
                                  f"{settled[name]:.3f}")
        seconds = max(2.0, amp / SAFE["speed_max_rad_s"] + 1.0)
        plan += [Segment("out", {**settled, name: settled[name] + direction * amp}, seconds, name),
                 Segment("back", settled, max(1.5, seconds - 0.5), name)]
    return plan


def gains_plan(settled: Mapping[str, float], level: str) -> list[Segment]:
    excursion, speed, accel = GAINS_LEVELS[level]
    move = excursion / speed + speed / accel + 0.5  # the move plus a dwell
    out = {}
    for name in ARMS:
        direction = room(settled, name, excursion)
        if not direction:
            raise CommissionError(f"no room to move {name} by {excursion} rad from "
                                  f"{settled[name]:.3f}")
        out[name] = settled[name] + direction * excursion
    plan = [Segment("settle", settled, SETTLE_S)]
    for name in ARMS:
        plan += [Segment("out", {**settled, name: out[name]}, move, name, "alone"),
                 Segment("back", settled, move, name, "alone")]
    plan += [Segment("out", out, move, None, "together"),
             Segment("back", settled, move, None, "together")]
    return plan


def coverage(rows: list[dict[str, Any]], plan: list[Segment]) -> dict[str, dict[str, float]]:
    """Per phase and joint: the speed and the acceleration sustained for
    SUSTAIN_TICKS ticks, from the commanded velocities."""
    result: dict[str, dict[str, float]] = {}
    for phase in ("alone", "together"):
        for name in ARMS:
            picked = [row for row in rows if row.get("velocities")  # a fault tick has none
                      and plan[row["segment"]].phase == phase
                      and plan[row["segment"]].joint in (name, None)]
            speed = accel = 0.0
            for i in range(len(picked) - SUSTAIN_TICKS):
                window = picked[i:i + SUSTAIN_TICKS + 1]
                if window[-1]["t"] - window[0]["t"] > (SUSTAIN_TICKS + 0.5) * CONTROL_DT:
                    continue  # not consecutive
                velocities = [abs(row["velocities"][name]) for row in window]
                speed = max(speed, min(velocities))
                accel = max(accel, abs(window[-1]["velocities"][name] - window[0]["velocities"][name])
                            / (window[-1]["t"] - window[0]["t"]))
            result[f"{phase}:{name}"] = {"speed": speed, "accel": accel}
    return result


# -- running a motion attempt ---------------------------------------------------


@dataclass
class MotionResult:
    completed: bool = False
    trigger: str | None = None
    release_confirmed: bool = False
    rows: list[dict[str, Any]] = field(default_factory=list)
    gap: dict[str, Any] = field(default_factory=dict)
    released_drift_rad: float | None = None
    rpc_order: list[bool] = field(default_factory=list)
    frames: int = 0
    in_gap: bool = False


def motion_config(mode: str, **gains: float) -> K1BridgeConfig:
    values = {**SAFE, **gains}
    return K1BridgeConfig(kp=values["kp"], kd=values["kd"],
                          speed_max_rad_s=values["speed_max_rad_s"],
                          accel_max_rad_s2=values["accel_max_rad_s2"],
                          allowed_modes=frozenset({ROBOT_MODES[mode]}))


Vet = Callable[[list[dict[str, Any]], list[Segment]], list[str]]


def run_motion(rig: Rig, config: K1BridgeConfig, build: Callable[[dict[str, float]], list[Segment]] | None,
               *, mode: str, meta: Mapping[str, Any], attested: str | None,
               begin: Callable[[], Path | None], vet: Vet | None = None,
               gap_s: float | None = None) -> tuple[MotionResult, list[Segment]]:
    """Plan, confirm, take over, validate, move, and end with the arms released.

    ``build`` None is the quick release: takeover, then release, no frames.
    ``begin`` runs once the operator confirmed, before any command; it starts the
    attempt and returns the tick-log path.  ``vet`` adds checks on the simulated plan.
    """
    result = MotionResult()
    wait_for_readings(rig.transport, READINGS_TIMEOUT_S)
    problems = identity_problems(rig, meta, attested)
    if problems:
        raise CommissionError("this is not the session's robot:\n  " + "\n  ".join(problems))
    before: K1State = rig.transport.latest_state()
    plan: list[Segment] = []
    start: dict[str, float] = {}
    text = CHECKLIST.format(mode=mode)
    if build is not None:
        start = normalized_arm_pose(before.arm_positions(), LIMITS)
        plan = build(settle_targets(start))
        contact = rig.contact.at(np.asarray(before.positions)) if rig.contact else None
        samples = simulate(start, plan, config, contact)
        found = plan_problems(start, plan, samples) + (vet(samples, plan) if vet else [])
        if found:
            raise CommissionError("the plan is refused:\n  " + "\n  ".join(found))
        text = describe_plan(start, plan, samples) + "\n\n" + text
    if not confirm(text):
        raise CommissionError("not confirmed")
    log = begin()
    rig.instrument()
    bridge = K1ArmBridge(rig.transport, LIMITS, config, contact=rig.contact, clock=rig.clock,
                         sleeper=rig.sleeper)
    handle = log.open("w", encoding="utf-8") if log else None
    try:
        with sigint_gate() as gate:  # Ctrl-C waits for the takeover call, then stops cleanly
            try:
                after = bridge.takeover()
            except K1BridgeError as exc:
                result.trigger = f"takeover_failed:{exc}"
                result.release_confirmed = bridge.state != "release_unconfirmed"
                return result, plan
            if build is None:
                rig.sleeper(TAKEOVER_SETTLE_S)  # as Booster's ArmController: settle, then release
                bridge.abort("quick_release")
            else:
                try:
                    problem = post_takeover_problem(before, after, start, plan, config, rig)
                    if problem is None:
                        problem = bridge.check(rig.clock())[0]
                except BaseException as exc:  # a bug or an I/O error: the arms go back first
                    abort_unexpected(bridge, exc)
                    raise
                if problem is not None:
                    bridge.abort(f"post_takeover:{problem}")
                else:
                    try:
                        gate.allow_interrupt()
                        move(rig, bridge, plan, result, handle)
                        if gap_s is not None and bridge.state == "tracking":
                            command_gap(rig, bridge, gap_s, result)
                    except KeyboardInterrupt:
                        gate.defer()
                        result.trigger = "operator_stop"
                        if result.in_gap:  # no frame after a command gap began
                            bridge.abort("command_gap:operator_stop")
                        print("Stopping: releasing the arms.")
                    except ArmBridgeError as exc:  # lost real time
                        gate.defer()
                        bridge.abort(f"real_time_lost:{exc}")
                    except BaseException as exc:  # a bug or an I/O error: the arms go back first
                        gate.defer()
                        abort_unexpected(bridge, exc)
                        raise
                    gate.defer()
                    if bridge.state == "tracking":
                        bridge.stop()
    finally:
        if handle:
            handle.close()
        # idle: the takeover was refused or failed and its release was confirmed
        result.release_confirmed = bridge.state in ("released", "fault_latched", "idle")
        result.trigger = result.trigger or bridge.fault
        result.rpc_order = [call["enable"] for call in rig.rpc]
        result.frames = rig.frames
    if result.release_confirmed:
        result.released_drift_rad = watch_release(rig)
    return result, plan


def abort_unexpected(bridge: K1ArmBridge, exc: BaseException) -> None:
    """Hand the arms back after an error that will escape main(), and warn here
    when that release is not confirmed: main() never gets to print it."""
    bridge.abort(f"unexpected:{exc!r}")
    if bridge.state == "release_unconfirmed":
        print(MANUAL_STOP, file=sys.stderr)


def post_takeover_problem(before: K1State, after: K1State, start: Mapping[str, float],
                          plan: list[Segment], config: K1BridgeConfig, rig: Rig) -> str | None:
    """Right after the takeover, before the first frame: the pose still matches the
    confirmed plan, and the settling re-simulated from it passes the same checks."""
    moved = max(abs(a - b) for a, b in zip(after.positions, before.positions))
    if moved > POSE_MATCH_RAD:
        return f"pose_changed {moved:.3f} rad"
    try:
        now_pose = normalized_arm_pose(after.arm_positions(), LIMITS)
        contact = rig.contact.at(np.asarray(after.positions)) if rig.contact else None
        settle = [plan[0]]
        samples = simulate(now_pose, settle, config, contact,
                           deadline=time.perf_counter() + POST_TAKEOVER_SIM_S)
    except (K1BridgeError, CommissionError) as exc:
        return str(exc)
    found = plan_problems(now_pose, settle, samples)
    return found[0] if found else None


def move(rig: Rig, bridge: K1ArmBridge, plan: list[Segment], result: MotionResult,
         handle: Any) -> None:
    ends = np.cumsum([ticks(segment.seconds) * CONTROL_DT for segment in plan])
    steps = int(round(ends[-1] / CONTROL_DT))
    start = rig.clock()
    if rig.real:
        tick_iter: Any = iter_dds_replay_ticks(steps=steps, control_dt=CONTROL_DT, start_time=start,
                                               clock=rig.clock, sleeper=rig.sleeper)
    else:
        tick_iter = range(1, steps + 1)
    settled_checked = False
    for tick in tick_iter:
        if rig.real:
            elapsed, now = tick.replay_time_s, tick.now_s
        else:
            rig.sleeper(CONTROL_DT)
            elapsed, now = tick * CONTROL_DT, rig.clock()
        index = min(int(np.searchsorted(ends, elapsed - 1e-9)), len(plan) - 1)
        if index > 0 and not settled_checked:
            settled_checked = True
            state = rig.transport.latest_state()
            off = max(abs(state.arm_positions()[name] - plan[0].targets[name]) for name in ARMS)
            if off > POSE_MATCH_RAD:
                bridge.abort(f"not_settled {off:.3f} rad")
                return
        step = bridge.step(plan[index].targets, source_timestamp=now, now=now)
        state = rig.transport.latest_state()
        row = {"t": round(elapsed, 4), "segment": index, "reason": step.reason,
               "command": step.command.positions if step.command else None,
               "velocities": step.command.velocities if step.command else None,
               "measured": state.arm_positions() if state else None,
               "errors": bridge.tracking_errors(state) if state else None}
        result.rows.append(row)
        if handle:
            handle.write(json.dumps(row) + "\n")
        if bridge.state != "tracking":
            return
    result.completed = True


def command_gap(rig: Rig, bridge: K1ArmBridge, gap_s: float, result: MotionResult) -> None:
    """Stop publishing for ``gap_s`` while running the bridge's checks, then release
    with ``abort`` (never ``stop``: no further frames)."""
    last = result.rows[-1]["command"] if result.rows else None
    now = rig.clock()
    problem, state, _age = bridge.check(now)
    if problem or state is None or last is None:
        bridge.abort(f"command_gap:before:{problem}")
        return
    if max(abs(state.arm_positions()[n] - last[n]) for n in ARMS) >= GAP_START_DRIFT_RAD:
        bridge.abort("command_gap:not_still")
        return
    start = previous = now
    worst = dict.fromkeys(ARMS, 0.0)
    reason = "deadline"
    result.in_gap = True
    while True:
        rig.sleeper(CONTROL_DT)
        now = rig.clock()
        if now - previous > CONTROL_DT + OVERRUN_S:
            reason = "loop_overrun"
            break
        previous = now
        problem, state, _age = bridge.check(now)
        if problem or state is None:
            reason = f"check:{problem}"
            break
        arms = state.arm_positions()
        for name in ARMS:
            worst[name] = max(worst[name], abs(arms[name] - last[name]))
        if max(worst.values()) > GAP_DRIFT_RAD:
            reason = "drift"
            break
        if rig.contact is not None and max(
                rig.contact.at(np.asarray(state.positions)).depths(arms).values(),
                default=0.0) > CONTACT_TOLERANCE_M:
            reason = "contact"
            break
        if now - start >= gap_s:
            break
    released = time.monotonic()
    bridge.abort(f"command_gap:{reason}")
    result.gap = {"seconds": gap_s, "ended_by": reason, "lasted_s": round(now - start, 3),
                  "max_drift_rad": {name: round(value, 4) for name, value in worst.items()},
                  "release_s": round(time.monotonic() - released, 3)}


def watch_release(rig: Rig) -> float | None:
    """How far the arms moved in the RELEASE_WATCH_S after the release."""
    first = rig.transport.latest_state()
    if first is None:
        return None
    worst, end = 0.0, rig.clock() + RELEASE_WATCH_S
    while rig.clock() < end:
        rig.sleeper(0.05)
        state = rig.transport.latest_state()
        if state is not None:
            worst = max(worst, max(abs(a - b) for a, b in
                                   zip(state.arm_positions().values(), first.arm_positions().values())))
    return round(worst, 4)


def tracking_summary(rows: list[dict[str, Any]]) -> dict[str, dict[str, float]]:
    summary = {}
    for name in ARMS:
        values = [row["errors"][name] for row in rows if row.get("errors")]
        summary[name] = ({"max": round(max(values), 4), "p95": round(float(np.percentile(values, 95)), 4)}
                         if values else {"max": 0.0, "p95": 0.0})
    return summary


# -- steps ---------------------------------------------------------------------


@dataclass
class Outcome:
    verdict: str
    fields: dict[str, Any]
    reason: str | None = None


def preflight(args: argparse.Namespace) -> int:
    real = args.transport == "booster"
    try:
        robot_net = ipaddress.ip_network(f"{args.robot_ip}/24", strict=False)
    except ValueError as exc:
        raise CommissionError(f"--robot-ip: {exc}") from exc
    checks: list[tuple[str, bool, str]] = []
    checks.append(("Python 3.10-3.14", (3, 10) <= sys.version_info[:2] <= (3, 14),
                   platform.python_version()))
    model = resolve_model(args.model)
    if model is not None or real:
        checks.append(("K1 model (contact check, mirror, zero-check)", model is not None,
                       str(model) if model else "not found: pass --model "
                       "<GMR>/assets/booster_k1/K1_serial.xml"))
    else:
        print("  NOTE  no K1 model: practice runs without the contact check, mirror and "
              "zero-check (pass --model <GMR>/assets/booster_k1/K1_serial.xml)")
    try:
        import mujoco  # noqa: F401
        checks.append(("MuJoCo", True, mujoco.__version__))
    except ImportError as exc:
        checks.append(("MuJoCo", False, str(exc)))
    if real:
        checks.append(("Linux (the Booster SDK is Linux-only)", sys.platform.startswith("linux"),
                       sys.platform))
        version = binding_version()
        checks.append((f"Booster SDK bindings {EXPECTED_SDK}", version == EXPECTED_SDK,
                       version or "not installed: bash tools/setup/install.sh core --with-booster"))
        addresses = local_addresses()
        net_ok = bool(args.net) and any(
            args.net in (name, str(address)) and address in robot_net
            for name, address in addresses)
        checks.append((f"--net is this PC's interface or address on {robot_net}", net_ok,
                       f"--net {args.net or '(missing)'}; found "
                       + (", ".join(f"{n}={a}" for n, a in addresses) or "no addresses")))
        try:
            answered = subprocess.run(["ping", "-c", "2", "-W", "1", args.robot_ip],
                                      capture_output=True, text=True, check=False).returncode == 0
            detail = "ok" if answered else "no answer: cable, power, address"
        except OSError as exc:  # no ping program
            answered, detail = False, f"cannot run ping ({exc}); install iputils-ping"
        checks.append((f"robot answers ping at {args.robot_ip}", answered, detail))
    for name, ok, detail in checks:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}: {detail}")
    passed = all(ok for _name, ok, _detail in checks)
    print("PREFLIGHT " + ("PASS" if passed else "FAIL"))
    if passed:
        print("Next: python3 tools/control/k1_commission.py probe"
              + (f" --transport booster --net {args.net}" if real else ""))
    return 0 if passed else 1


def local_addresses() -> list[tuple[str, Any]]:
    try:
        output = subprocess.run(["ip", "-o", "-4", "addr", "show"], capture_output=True,
                                text=True, check=False).stdout
    except OSError:
        return []
    found = []
    for line in output.splitlines():
        parts = line.split()
        if len(parts) >= 4:
            found.append((parts[1], ipaddress.ip_interface(parts[3]).ip))
    return found


def run_probe(args: argparse.Namespace) -> int:
    session = Session(args.session) if args.session else None
    meta = dict(session.meta) if session else {
        "transport": args.transport, "net": args.net, "domain_id": args.domain_id}
    model = resolve_model(args.model) if session is None else session_model(session, args.model)
    if session is not None:
        check_versions(session)
    rig = open_rig(meta, virtual=False, model=model)
    try:
        seconds = args.seconds if args.seconds is not None else (10.0 if rig.real else 1.0)
        report = probe_robot(rig.transport, seconds)
    finally:
        rig.transport.close()
    info = report["robot_info"] if isinstance(report["robot_info"], dict) else {}
    serial = info.get("serial_number") or ""
    identity = {"serial": serial, "firmware": info.get("version") or "", "source": "queried"}
    if not serial:
        if not (args.attested_robot_id and args.firmware):
            print("ERROR: the robot did not report its serial number. Read the serial and the "
                  "firmware version in Booster's app and pass --attested-robot-id and --firmware.",
                  file=sys.stderr)
            return 2
        identity = {"serial": args.attested_robot_id, "firmware": args.firmware,
                    "source": "attested"}
    if session is None:
        session = Session.create(args.root, {
            **meta, "serial": identity["serial"], "firmware": identity["firmware"],
            "identity_source": identity["source"], "model": info.get("model") or "K1",
            "sdk_binding_version": binding_version(), "command_contract": COMMAND_CONTRACT,
            "model_path": str(model.resolve()) if model else None,
            "model_sha256": sha256_file(model) if model else None, "created": now_iso(),
            "practice": not rig.real})
    elif (identity["serial"], identity["firmware"]) != (session.meta["serial"],
                                                        session.meta["firmware"]):
        print(f"ERROR: this robot ({identity['serial']}, firmware {identity['firmware']}) is not "
              f"the session's; start a new session (probe without --session).", file=sys.stderr)
        return 2
    attempt = session.start("probe", {"seconds": seconds}, {})
    fall_period = report["fall_period_s"]
    checks = {
        "joint state arrives": report["low_state_period_s"] is not None,
        "robot mode readable": bool(report["robot_modes"]),
        "fall topic periodic": report["fall_messages"] >= 3 and fall_period is not None
                               and fall_period <= FALL_PERIOD_MAX_S,
        "no malformed joint states": report["malformed_low_states"] == 0,
    }
    verdict = "PASS" if all(checks.values()) else "FAIL"
    fields = {"report": report, "checks": checks,
              "fall_topic": ({"kind": "periodic", "period_s": fall_period}
                             if checks["fall topic periodic"] else {"kind": "event-or-missing"})}
    attempt.finish(verdict, **fields)
    print(json.dumps(report, indent=2))
    for name, ok in checks.items():
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    if not checks["fall topic periodic"]:
        print("  The fall topic is event-only or missing: motion stays blocked (a cached "
              "'ready' is no evidence the robot stands). Send this report to the developers.")
    return report_end(session, attempt, verdict, None if verdict == "PASS" else "probe checks",
                      next_step="zero-check")


def report_end(session: Session, attempt: Attempt, verdict: str, reason: str | None, *,
               next_step: str | None, extra: str = "") -> int:
    print(f"\n{attempt.data['step']} attempt {attempt.id}: {verdict}"
          + (f" ({reason})" if reason else ""))
    if verdict != "PASS" and hint(reason):
        print(f"What to check: {hint(reason)}")
    print(f"Evidence: {attempt.path}")
    if verdict == "PASS" and next_step:
        print(f"Next: python3 tools/control/k1_commission.py {next_step} --session {session.path}"
              + extra + (' --operator "<your name>"' if next_step == "record" else ""))
    return 0 if verdict == "PASS" else 1


def motion_step(args: argparse.Namespace, key: str, prerequisites: list[str],
                settings: dict[str, Any], config: K1BridgeConfig,
                build: Callable[[dict[str, float]], list[Segment]] | None,
                judge: Callable[[MotionResult, list[Segment]], Outcome], *,
                gap_s: float | None = None, vet: Vet | None = None,
                next_step: str | None = None) -> int:
    session = Session(args.session)
    check_versions(session)
    model = session_model(session, args.model)
    real = session.meta["transport"] == "booster"
    if real and model is None:
        raise CommissionError("a real motion step needs the K1 model (contact check)")
    uses = session.require(prerequisites)
    started: list[Attempt] = []

    def begin() -> Path:
        started.append(session.start(key, {**settings, "mode": args.mode, "kp": config.kp,
                                           "kd": config.kd, "speed": config.speed_max_rad_s,
                                           "accel": config.accel_max_rad_s2}, uses))
        return started[0].path.with_suffix(".jsonl")

    with motion_lock():
        rig = open_rig(session.meta, mode=args.mode, model=model)
        try:
            rig.contact = contact_check(model) if model else None
            result, plan = run_motion(rig, config, build, mode=args.mode, meta=session.meta,
                                      attested=args.attested_robot_id, begin=begin, vet=vet,
                                      gap_s=gap_s)
        finally:
            rig.transport.close()
    attempt = started[0]
    base = {"trigger": result.trigger, "release_confirmed": result.release_confirmed,
            "completed": result.completed, "rpc": rig.rpc, "rpc_order": result.rpc_order,
            "frames_published": rig.frames,
            "released_drift_rad": result.released_drift_rad}
    if not result.release_confirmed:
        print(MANUAL_STOP, file=sys.stderr)
        attempt.finish("FAIL", **base, reason="release not confirmed")
        report_end(session, attempt, "FAIL", "release not confirmed", next_step=None)
        return 1
    outcome = judge(result, plan)
    attempt.finish(outcome.verdict, **base, **outcome.fields, reason=outcome.reason)
    extra = (f" --mode {args.mode}" if args.mode != "walking" else "") + (
        f" --attested-robot-id {args.attested_robot_id}" if args.attested_robot_id else "")
    return report_end(session, attempt, outcome.verdict, outcome.reason, next_step=next_step,
                      extra=extra if next_step != "record" else extra.split(" --attested")[0])


def run_quick_release(args: argparse.Namespace) -> int:
    def judge(result: MotionResult, _plan: list[Segment]) -> Outcome:
        # bridge-level order: takeover, a fresh state, release (retries allowed); no frame
        order = result.rpc_order
        ok = (result.trigger == "quick_release" and order[:1] == [True] and len(order) >= 2
              and not any(order[1:]) and result.frames == 0)
        answer = ok and ask_yes("Are the arms back under Booster's control, the robot normal?")
        reason = None if ok and answer else (
            f"unexpected RPC order {order} or frames {result.frames}" if not ok
            else "operator: not normal")
        return Outcome("PASS" if ok and answer else "FAIL", {"operator_normal": answer}, reason)

    return motion_step(args, f"quick-release@{args.mode}", ["probe"], {}, motion_config(args.mode),
                       None, judge, next_step="hold")


def held(result: MotionResult) -> tuple[bool, str | None]:
    if not result.completed or result.trigger:
        return False, result.trigger or "not completed"
    return True, None


def run_hold(args: argparse.Namespace) -> int:
    seconds = args.seconds
    if not (math.isfinite(seconds) and 0 < seconds <= HOLD_MAX_S):
        raise CommissionError(f"--seconds must be > 0 and <= {HOLD_MAX_S:g}")

    def judge(result: MotionResult, _plan: list[Segment]) -> Outcome:
        ok, reason = held(result)
        still = ok and ask_yes("Did the arms stay still while held?")
        back = ok and ask_yes("After the release, did Booster take the arms back smoothly "
                              "(no drop, no jerk)?")
        verdict = "PASS" if ok and still and back else "FAIL"
        return Outcome(verdict, {"arms_still": still, "release_smooth": back,
                                 "tracking": tracking_summary(result.rows)},
                       reason or (None if verdict == "PASS" else "operator: no"))

    return motion_step(args, f"hold@{args.mode}", [f"quick-release@{args.mode}"],
                       {"seconds": seconds}, motion_config(args.mode),
                       lambda settled: hold_plan(settled, seconds), judge, next_step="balance")


def run_balance(args: argparse.Namespace) -> int:
    print("Balance test: the arms hold still for 30 s. Push the robot gently at the chest, "
          "several times, while it is still supported by the hoist.")

    def judge(result: MotionResult, _plan: list[Segment]) -> Outcome:
        ok, reason = held(result)
        kept = ok and ask_yes("Did the robot keep its balance through the pushes?")
        support = ask("Describe the support (hoist slack, feet on the floor?): ").strip() if ok else ""
        verdict = "PASS" if ok and kept and support else "FAIL"
        return Outcome(verdict, {"balance_kept": kept, "support": support or None,
                                 "evidence_kind": "hoist-supported"},
                       reason or (None if verdict == "PASS" else "operator: no / no support text"))

    return motion_step(args, f"balance@{args.mode}", [f"hold@{args.mode}"],
                       {"seconds": BALANCE_S}, motion_config(args.mode),
                       lambda settled: hold_plan(settled, BALANCE_S), judge, next_step="command-loss")


def run_command_loss(args: argparse.Namespace) -> int:
    gap = args.gap
    if gap not in GAP_LEVELS:
        raise CommissionError(f"--gap must be one of {GAP_LEVELS}")
    index = GAP_LEVELS.index(gap)
    previous = f"command-loss@{args.mode}@{GAP_LEVELS[index - 1]}" if index else f"hold@{args.mode}"

    def judge(result: MotionResult, _plan: list[Segment]) -> Outcome:
        ended = result.gap.get("ended_by")
        if ended != "deadline":
            return Outcome("FAIL", {"gap": result.gap}, f"command_gap:{ended or result.trigger}")
        drift = result.gap["max_drift_rad"]
        behaviour = (f"commands stopped for {gap:g} s: max arm drift left "
                     f"{max(v for k, v in drift.items() if 'Left' in k):.3f} rad, right "
                     f"{max(v for k, v in drift.items() if 'Right' in k):.3f} rad; then released "
                     f"in {result.gap['release_s']:.2f} s")
        print(behaviour)
        acceptable = ask_yes("Was what the arms did acceptable?")
        return Outcome("PASS" if acceptable else "FAIL",
                       {"gap": result.gap, "behaviour": behaviour, "acceptable": acceptable},
                       None if acceptable else "operator: not acceptable")

    nxt = (f"command-loss --gap {GAP_LEVELS[index + 1]}" if index + 1 < len(GAP_LEVELS) else "signs")
    return motion_step(args, f"command-loss@{args.mode}@{gap:g}", [previous], {"gap_s": gap},
                       motion_config(args.mode), lambda settled: hold_plan(settled, 0.5), judge,
                       gap_s=gap, next_step=nxt)


def run_signs(args: argparse.Namespace) -> int:
    print(f"Signs test: each arm joint moves {SIGNS_RAD:.2f} rad ({math.degrees(SIGNS_RAD):.0f} degrees) and back, one at a time, in this "
          f"order: {', '.join(ARMS)}. Watch the robot and the mirror together.")

    def judge(result: MotionResult, plan: list[Segment]) -> Outcome:
        ok, reason = held(result)
        joints: dict[str, Any] = {}
        if ok:
            for index, segment in enumerate(plan):
                if segment.name != "out":
                    continue
                name = segment.joint
                assert name is not None
                commanded = segment.targets[name] - plan[0].targets[name]
                out = window_mean(result.rows, index, name)
                base = window_mean(result.rows, index - 1, name)
                change = out - base
                good = (math.copysign(1, change) == math.copysign(1, commanded)
                        and 0.5 * abs(commanded) <= abs(change) <= 1.5 * abs(commanded))
                joints[name] = {"commanded_rad": round(commanded, 3), "measured_rad": round(change, 3),
                                "auto_ok": good}
                print(f"  {name:24s} commanded {commanded:+.3f}  measured {change:+.3f}  "
                      f"{'ok' if good else 'WRONG'}")
            for name in ARMS:
                joints[name]["same_as_mirror"] = ask_yes(
                    f"{name}: did the robot move the same way as the mirror?")
        verdict = "PASS" if ok and joints and all(
            j["auto_ok"] and j["same_as_mirror"] for j in joints.values()) else "FAIL"
        return Outcome(verdict, {"joints": joints},
                       reason or (None if verdict == "PASS" else "a joint moved wrongly"))

    return motion_step(args, f"signs@{args.mode}", [f"hold@{args.mode}"], {},
                       motion_config(args.mode), signs_plan, judge, next_step="gains --level L1")


def tpose_plan(settled: Mapping[str, float]) -> list[Segment]:
    """Demo: arms out to the T-pose (every arm joint 0, elbows just inside their range end),
    hold, and back."""
    target = settle_targets({name: 0.0 for name in ARMS})
    seconds = max(abs(target[n] - settled[n]) for n in ARMS) / SAFE["speed_max_rad_s"] + 1.5
    return [Segment("settle", settled, SETTLE_S), Segment("out", target, seconds),
            Segment("hold", target, TPOSE_HOLD_S), Segment("back", settled, seconds)]


def run_tpose(args: argparse.Namespace) -> int:
    def judge(result: MotionResult, _plan: list[Segment]) -> Outcome:
        ok, reason = held(result)
        seen = ok and ask_yes("Did the robot reach the T-pose and come back smoothly?")
        verdict = "PASS" if ok and seen else "FAIL"
        return Outcome(verdict, {"seen": seen, "tracking": tracking_summary(result.rows)},
                       reason or (None if verdict == "PASS" else "operator: no"))

    return motion_step(args, f"tpose@{args.mode}", [f"hold@{args.mode}"], {},
                       motion_config(args.mode), tpose_plan, judge, next_step="gains --level L1")


SYSID_TESTS = ("sweep", "steps", "sine", "poses")
SYSID_SINE_HZ = (0.25, 0.5, 1.0, 1.5, 2.0)


def sysid_plan(joint: str | None, test: str, amp: float) -> Any:
    """System identification plans (2026-11-15): exact 50 Hz profiles through the envelope."""
    def build(settled: Mapping[str, float]) -> list[Segment]:
        plan = [Segment("settle", settled, SETTLE_S)]
        if test == "poses":
            for name, pose in (("tpose", {n: 0.0 for n in ARMS}),
                               ("forward", {**settled, "ALeft_Shoulder_Pitch": -amp,
                                            "ARight_Shoulder_Pitch": -amp})):
                target = settle_targets(pose)
                move = max(abs(target[n] - settled[n]) for n in ARMS) / SAFE["speed_max_rad_s"] + 1.0
                plan += [Segment(name, target, move), Segment(name + "-hold", target, 4.0),
                         Segment("back", settled, move), Segment("rest", settled, 1.5)]
            return plan
        assert joint is not None
        lo, hi = LIMITS[joint].lower + MARGIN, LIMITS[joint].upper - MARGIN
        base = settled[joint]
        clip = lambda v: min(max(v, lo), hi)
        def at(v: float, seconds: float, name: str) -> Segment:
            return Segment(name, {**settled, joint: clip(v)}, seconds, joint)
        def ramp(a: float, b: float, speed: float, name: str) -> list[Segment]:
            n = max(1, int(abs(b - a) / speed / CONTROL_DT))
            return [at(a + (b - a) * (i + 1) / n, CONTROL_DT, name) for i in range(n)]
        if test == "sweep":
            lo_t, hi_t = clip(base - amp), clip(base + amp)
            plan += ramp(base, lo_t, 0.3, "sweep-down") + [at(lo_t, 1.0, "dwell-low")]
            plan += ramp(lo_t, hi_t, 0.3, "sweep-up") + [at(hi_t, 1.0, "dwell-high")]
            plan += ramp(hi_t, base, 0.3, "sweep-back") + [at(base, 1.0, "rest")]
        elif test == "steps":
            sign = room(settled, joint, amp) or 1
            for size in (0.1, 0.25, amp):
                plan += [at(base + sign * size, 1.5, f"step+{size:g}"), at(base, 1.5, f"step-{size:g}")]
        elif test == "sine":
            sign = room(settled, joint, 2 * amp) or 1
            centre = base + sign * amp
            plan += ramp(base, centre, 0.3, "to-centre") + [at(centre, 0.5, "centre")]
            for hz in SYSID_SINE_HZ:
                n = int(round(3 / hz / CONTROL_DT))
                plan += [at(centre + amp * math.sin(2 * math.pi * hz * (i + 1) * CONTROL_DT),
                            CONTROL_DT, f"sine-{hz:g}Hz") for i in range(n)]
                plan += [at(centre, 0.5, "centre")]
            plan += ramp(centre, base, 0.3, "back") + [at(base, 1.0, "rest")]
        return plan
    return build


def run_sysid(args: argparse.Namespace) -> int:
    def judge(result: MotionResult, _plan: list[Segment]) -> Outcome:
        ok, reason = held(result)
        return Outcome("PASS" if ok else "FAIL", {"tracking": tracking_summary(result.rows)}, reason)

    key = f"sysid@{args.mode}@{args.test}@{args.joint or 'both'}@{args.amp:g}"
    return motion_step(args, key, [f"hold@{args.mode}"], {"test": args.test, "joint": args.joint,
                       "amp": args.amp}, motion_config(args.mode, kp=args.kp, kd=args.kd, speed_max_rad_s=3.0,
                                     accel_max_rad_s2=12.0),
                       sysid_plan(args.joint, args.test, args.amp), judge, next_step=None)


def window_mean(rows: list[dict[str, Any]], segment: int, name: str, seconds: float = 0.5) -> float:
    picked = [row for row in rows if row["segment"] == segment and row["measured"]]
    tail = [row["measured"][name] for row in picked if row["t"] >= picked[-1]["t"] - seconds]
    return float(np.mean(tail))


def gains_key(mode: str, level: str, kp: float, kd: float) -> str:
    return f"gains@{mode}@{level}@kp{kp:g}@kd{kd:g}"


def run_gains(args: argparse.Namespace) -> int:
    level, kp, kd = args.level, args.kp, args.kd
    if level not in GAINS_LEVELS:
        raise CommissionError(f"--level must be one of {', '.join(GAINS_LEVELS)}")
    if not all(math.isfinite(v) and v > 0 for v in (kp, kd)) or kp > EXAMPLE_KP or kd > EXAMPLE_KD:
        raise CommissionError(f"--kp must be 0-{EXAMPLE_KP:g} and --kd 0-{EXAMPLE_KD:g}")
    excursion, speed, accel = GAINS_LEVELS[level]
    names = list(GAINS_LEVELS)
    prerequisites = ([f"signs@{args.mode}", f"balance@{args.mode}"] if level == "L1" else
                     [gains_key(args.mode, names[names.index(level) - 1], kp, kd)])
    config = motion_config(args.mode, kp=kp, kd=kd, speed_max_rad_s=speed, accel_max_rad_s2=accel)

    def vet(samples: list[dict[str, Any]], plan: list[Segment]) -> list[str]:
        """The simulated plan must demonstrate the level in both phases."""
        return [f"{where}: the plan reaches only {v['speed']:.3f} rad/s, {v['accel']:.3f} rad/s^2"
                for where, v in coverage(samples, plan).items()
                if v["speed"] < COVERAGE * speed or v["accel"] < COVERAGE * accel]

    def judge(result: MotionResult, plan: list[Segment]) -> Outcome:
        ok, reason = held(result)
        tracking = tracking_summary(result.rows)
        covered = coverage(result.rows, plan)
        speeds = [v["speed"] for v in covered.values()]
        accels = [v["accel"] for v in covered.values()]
        problems = [f"{n} tracking max {t['max']:.3f} / p95 {t['p95']:.3f} rad"
                    for n, t in tracking.items()
                    if t["max"] >= GAINS_MAX_ERROR_RAD or t["p95"] >= GAINS_P95_RAD]
        if ok and (min(speeds) < COVERAGE * speed or min(accels) < COVERAGE * accel):
            problems.append(f"coverage not reached: speed {min(speeds):.3f}, accel {min(accels):.3f}")
        approved = {"kp": kp, "kd": kd,  # rounded down; 1e-6 absorbs float noise at a cap
                    "speed_max_rad_s": math.floor(min(speeds) * 20 + 1e-6) / 20 if speeds else 0.0,
                    "accel_max_rad_s2": math.floor(min(accels) * 10 + 1e-6) / 10 if accels else 0.0}
        verdict = "PASS" if ok and not problems and approved["speed_max_rad_s"] > 0 \
            and approved["accel_max_rad_s2"] > 0 else "FAIL"
        for name, value in tracking.items():
            print(f"  {name:24s} tracking max {value['max']:.3f}  p95 {value['p95']:.3f} rad")
        print(f"  demonstrated: speed {min(speeds, default=0):.3f} rad/s, "
              f"accel {min(accels, default=0):.3f} rad/s^2 (minimum over joints and phases)")
        return Outcome(verdict, {"tracking": tracking, "coverage": covered,
                                 "approvable": approved if verdict == "PASS" else None},
                       reason or ("; ".join(problems) if problems else None))

    nxt = (f"gains --level {names[names.index(level) + 1]} --kp {kp:g} --kd {kd:g}"
           if level != names[-1] else "record")
    return motion_step(args, gains_key(args.mode, level, kp, kd), prerequisites,
                       {"level": level, "excursion_rad": excursion}, config,
                       lambda settled: gains_plan(settled, level), judge, vet=vet, next_step=nxt)


# -- read-only views ------------------------------------------------------------


class ModelPose:
    """The K1 MJCF posed from measured joints (fixed upright base)."""

    def __init__(self, model_path: Path) -> None:
        import mujoco

        self.mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.data.qpos[:] = self.model.qpos0
        self.data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
        mujoco.mj_kinematics(self.model, self.data)
        self.data.qpos[2] = 0.03 - float(np.min(self.data.xpos[1:, 2]))  # feet near the floor

    def set(self, positions: Any) -> None:
        self.data.qpos[7:] = np.asarray(positions, dtype=float)
        self.mujoco.mj_kinematics(self.model, self.data)

    def link_angles(self) -> dict[str, float]:
        """Per arm, in degrees: the upper arm from the torso's down axis, and the
        elbow bend (upper arm to forearm).  Torso frame, so torso tilt drops out."""
        data, angles = self.data, {}
        down = -data.body("Trunk").xmat.reshape(3, 3)[:, 2]
        for side, sign in (("Left", 1.0), ("Right", -1.0)):
            shoulder = data.body(f"{side}_Arm_2").xpos
            elbow = data.body(f"{side.lower()}_hand_link").xpos
            hand = data.body(f"{side.lower()}_hand_link").xmat.reshape(3, 3)[:, 1] * sign
            upper = elbow - shoulder
            angles[f"{side.lower()}_upper_arm_from_down"] = _angle(upper, down)
            angles[f"{side.lower()}_elbow_bend"] = _angle(upper, hand)
        return angles


def _angle(a: np.ndarray, b: np.ndarray) -> float:
    cos = float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)))
    return math.degrees(math.acos(max(-1.0, min(1.0, cos))))


def run_zero_check(args: argparse.Namespace) -> int:
    session = Session(args.session)
    check_versions(session)
    model = session_model(session, args.model)
    if model is None:
        raise CommissionError("zero-check needs the K1 model (--model)")
    uses = session.require(["probe"])
    pose = ModelPose(model)
    print(ZERO_GUIDE)
    measured = {}
    for key, label in ZERO_MEASUREMENTS:
        while True:
            reply = ask(f"{label} (degrees): ").strip()
            try:
                value = float(reply)
            except ValueError:
                continue
            if math.isfinite(value) and 0.0 <= value <= 180.0:
                measured[key] = value
                break
    rig = open_rig(session.meta, virtual=False, model=model)
    attempt = session.start("zero-check", {}, uses)
    try:
        print("Keep the robot still: sampling the joints for 1 s.")
        samples = []
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            state = rig.transport.latest_state()
            if state is not None:
                samples.append(state.positions)
            time.sleep(0.02)
    finally:
        rig.transport.close()
    if len(samples) < 10:
        attempt.finish("FAIL", reason="state_stale: too few joint states")
        return report_end(session, attempt, "FAIL", "state_stale", next_step=None)
    array = np.asarray(samples)
    arm_columns = [K1_JOINTS.index(name) for name in ARMS]
    spread = float(np.max(np.ptp(array[:, arm_columns], axis=0)))
    if spread > ZERO_STILL_RAD:
        attempt.finish("FAIL", reason=f"pose unstable ({spread:.3f} rad)")
        return report_end(session, attempt, "FAIL", "the robot moved while sampling; hold it still "
                          "and measure again", next_step=None)
    pose.set(array.mean(axis=0))
    model_angles = pose.link_angles()
    rows = {key: {"measured_deg": measured[key], "model_deg": round(model_angles[key], 1),
                  "ok": abs(measured[key] - model_angles[key]) <= ZERO_TOLERANCE_DEG}
            for key, _label in ZERO_MEASUREMENTS}
    for key, row in rows.items():
        print(f"  {key:28s} measured {row['measured_deg']:6.1f}  model {row['model_deg']:6.1f}  "
              f"{'ok' if row['ok'] else 'OFF by more than 5 degrees'}")
    verdict = "PASS" if all(row["ok"] for row in rows.values()) else "FAIL"
    attempt.finish(verdict, angles=rows, scope=ZERO_SCOPE,
                   encoder_pose={n: round(float(v), 4) for n, v in zip(K1_JOINTS, array.mean(axis=0))})
    return report_end(session, attempt, verdict, None if verdict == "PASS" else "a link angle is off",
                      next_step="quick-release")


ZERO_MEASUREMENTS = (
    ("left_upper_arm_from_down", "Left upper arm: angle from the torso's down direction"),
    ("left_elbow_bend", "Left elbow bend: angle between upper arm and forearm (0 = straight)"),
    ("right_upper_arm_from_down", "Right upper arm: angle from the torso's down direction"),
    ("right_elbow_bend", "Right elbow bend: angle between upper arm and forearm (0 = straight)"),
)
ZERO_SCOPE = {
    "measured": ["shoulder pitch and roll together (upper arm from down)",
                 "Elbow_Yaw (elbow bend)"],
    "visual": ["Elbow_Pitch (upper-arm twist)", "the split between shoulder pitch and roll"],
}
ZERO_GUIDE = """\
Zero check (read-only: the robot is not commanded).
Booster holds the robot standing on the hoist (walking mode), arms in Booster's own pose.
Measure on the robot with a protractor or goniometer (see the guide for the axes):
  - upper arm: the line from the shoulder joint to the elbow joint, against the torso's
    vertical axis (0 = arm hanging straight down along the body);
  - elbow bend: the upper-arm line against the forearm line (0 = arm straight).
Type the four values; then keep the robot still while the tool samples its joints."""


def run_mirror(args: argparse.Namespace) -> int:
    session = Session(args.session)
    model = session_model(session, args.model)
    if model is None:
        raise CommissionError("the mirror needs the K1 model (--model)")
    import mujoco.viewer

    pose = ModelPose(model)
    rig = open_rig(session.meta, virtual=False, model=model)
    low = np.full(len(K1_JOINTS), np.inf)
    high = np.full(len(K1_JOINTS), -np.inf)
    last_print = 0.0
    try:
        with mujoco.viewer.launch_passive(pose.model, pose.data) as viewer:
            print("Mirror: the K1 model posed from the robot's measured joints. "
                  "Close the window or press Ctrl-C to stop.")
            while viewer.is_running():
                state = rig.transport.latest_state()
                if state is not None:
                    pose.set(state.positions)
                    values = np.asarray(state.positions)
                    low, high = np.minimum(low, values), np.maximum(high, values)
                    viewer.sync()
                    if time.monotonic() - last_print > 1.0:
                        last_print = time.monotonic()
                        angles = pose.link_angles()
                        print("  arms (deg): " + "  ".join(
                            f"{n[1:] if n.startswith('A') else n}="
                            f"{math.degrees(state.arm_positions()[n]):.0f}" for n in ARMS)
                            + " | " + "  ".join(f"{k}={v:.0f}" for k, v in angles.items()))
                time.sleep(1 / 30)
    except KeyboardInterrupt:
        pass
    finally:
        rig.transport.close()
    if np.isfinite(low).all():
        path = session.path / f"mirror-{dt.datetime.now(dt.timezone.utc):%H%M%S}.json"
        atomic_write(path, {name: {"min": round(float(a), 4), "max": round(float(b), 4)}
                            for name, a, b in zip(K1_JOINTS, low, high)})
        print(f"Joint ranges seen: {path}")
    return 0


# -- status and record ----------------------------------------------------------


def ladder(mode: str) -> list[str]:
    return ["probe", "zero-check", f"quick-release@{mode}", f"hold@{mode}", f"balance@{mode}",
            *(f"command-loss@{mode}@{gap:g}" for gap in GAP_LEVELS), f"signs@{mode}"]


def run_status(args: argparse.Namespace) -> int:
    session = Session(args.session)
    meta = session.meta
    print(f"Session {session.path}\n  robot {meta.get('serial')} firmware {meta.get('firmware')} "
          f"({meta.get('identity_source')}), transport {meta.get('transport')}"
          + ("  PRACTICE: can never unlock a real robot" if meta.get("practice") else ""))
    keys = ladder(args.mode) + sorted({a["step"] for a in session.attempts()
                                       if str(a.get("step", "")).startswith(f"gains@{args.mode}@")})
    for key in keys:
        latest = session.latest(key)
        problem = session.problem(latest, key)
        state = "valid" if problem is None else problem
        print(f"  {key:32s} {latest.get('id') if latest else '---'}  {state}")
    return 0


def run_record(args: argparse.Namespace) -> int:
    session = Session(args.session)
    check_versions(session)
    session_model(session, args.model)
    meta, mode = session.meta, args.mode
    gaps = [f"command-loss@{mode}@{gap:g}" for gap in GAP_LEVELS if gap >= GAP_RECORD_MIN_S]
    valid_gaps = [key for key in gaps if session.problem(session.latest(key), key) is None]
    if not valid_gaps:
        raise CommissionError(f"needs a valid command-loss at {GAP_RECORD_MIN_S:g} s or longer")
    gains = [a for a in session.attempts() if str(a.get("step", "")).startswith(f"gains@{mode}@")
             and session.latest(a["step"]).get("id") == a.get("id")  # type: ignore[union-attr]
             and session.problem(a) is None]
    if not gains:
        raise CommissionError("needs a valid gains attempt")
    for attempt in gains:
        approvable = attempt["approvable"]
        print(f"  gains attempt {attempt['id']} {attempt['step']}: speed "
              f"{approvable['speed_max_rad_s']} rad/s, accel {approvable['accel_max_rad_s2']} rad/s^2")
    chosen_id = args.gains_attempt or ask("Which gains attempt do you approve (its number)? ").strip()
    chosen = next((a for a in gains if a["id"] == chosen_id), None)
    if chosen is None:
        raise CommissionError(f"{chosen_id} is not a valid gains attempt of this session")
    keys = ["probe", "zero-check", f"quick-release@{mode}", f"hold@{mode}", f"balance@{mode}",
            valid_gaps[-1], f"signs@{mode}", chosen["step"]]
    uses = session.require(keys)
    if uses[chosen["step"]] != chosen["id"]:
        raise CommissionError("the chosen gains attempt is no longer the latest of its step")
    attempts = {key: session.latest(key) for key in keys}
    files = {key: session.path / "attempts" / f"{uses[key]}.json" for key in keys}
    walking = ask("Check 11 (for later phases): does Booster's walking (Move) keep working while "
                  "the arms are custom? (yes / no / not tested): ").strip()
    head = ask("Check 11: which controller owns the head joints while Booster's controller runs, "
               "and can the SDK command them? (text / not tested): ").strip()
    practice = bool(meta.get("practice"))
    probe_attempt, loss = attempts["probe"], attempts[valid_gaps[-1]]
    record = {
        "schema": AcceptanceRecord.SCHEMA, "practice": practice, "robot": "booster_k1",
        "command_contract": COMMAND_CONTRACT, "date": dt.date.today().isoformat(),
        "operator": args.operator, "firmware": meta["firmware"],
        "sdk_binding_version": meta.get("sdk_binding_version") or "none (practice)",
        "robot_identity": {"serial_number": "PRACTICE" if practice else meta["serial"],
                           "model": meta.get("model", "K1")},
        "identity_source": meta["identity_source"], "takeover_mode": mode,
        "balance_during_upper_body": True, "balance_evidence": "hoist-supported",
        "release_verified": True, "rpc_ordering_verified": True,
        "signs_and_zero_offsets_verified": True, "zero_offsets": ZERO_SCOPE,
        "fall_topic": probe_attempt["fall_topic"],  # type: ignore[index]
        "low_state_period_s": probe_attempt["report"]["low_state_period_s"],  # type: ignore[index]
        "command_loss": {"behaviour": loss["behaviour"], "acceptable": True,  # type: ignore[index]
                         "gap_s": loss["gap"]["seconds"]},  # type: ignore[index]
        "approved": chosen["approvable"],
        "notes": {"walking_with_custom_arms": walking or "not tested",
                  "head_owner": head or "not tested"},
        "evidence": {key: {"attempt": uses[key], "sha256": sha256_file(files[key])} for key in keys},
        "session": str(session.path),
    }
    problems = AcceptanceRecord(record).problems()
    if problems and not (practice and problems == [
            "practice must be false: a record from the fake robot unlocks nothing"]):
        raise CommissionError("the record would be refused:\n  " + "\n  ".join(problems))
    print(json.dumps(record, indent=2))
    print("Measured vs visual zero-offset scope: " + json.dumps(ZERO_SCOPE))
    if practice:
        print("PRACTICE record: it can never unlock a real robot.")
    if ask(f"Sign off: type your name ({args.operator}) to write the record: ").strip() != args.operator:
        raise CommissionError("not signed off")
    for key, path in files.items():  # the evidence must not have changed while reviewing
        if sha256_file(path) != record["evidence"][key]["sha256"]:
            raise CommissionError(f"evidence {path} changed; run record again")
    out = args.out or session.path / ("k1-acceptance-PRACTICE.json" if practice else "k1-acceptance.json")
    atomic_write(Path(out), record)
    print(f"Acceptance record: {out}")
    register_acceptance(record, Path(out), files)
    if not practice:
        approved = record["approved"]
        print("First real replay (hoist, short, slow, qualified recording):\n"
              f"  python3 tools/control/k1_arm_bridge.py <motion.pkl> --transport booster "
              f"--net {meta.get('net')} --session {session.path} --stage hoist "
              f"--acceptance-record {out} --model {meta.get('model_path')} "
              + (f"--attested-robot-id {meta['serial']} " if meta["identity_source"] == "attested"
                 else "") +
              f"--kp {approved['kp']:g} --kd {approved['kd']:g} --duration 10 "
              "--yes-i-have-read-the-unverified-section --jsonl first-replay.jsonl")
    return 0


def register_acceptance(record: Mapping[str, Any], out: Path, files: Mapping[str, Path]) -> None:
    """S0 in the stage ledger: the record and every attempt it cites are its evidence."""
    practice = bool(record["practice"])
    serial = str(record["robot_identity"]["serial_number"])
    ledger = Ledger(commissioning_root(), serial, practice=practice)
    approved = record["approved"]
    ledger.record_pass(
        "acceptance",
        identity={"serial": serial, "firmware": record["firmware"],
                  "sdk_binding_version": record["sdk_binding_version"]},
        settings={key: approved[key] for key in ("kp", "kd", "speed_max_rad_s", "accel_max_rad_s2")},
        evidence={"acceptance-record": out, **{f"attempt:{key}": path for key, path in files.items()}},
        operator=str(record["operator"]), extra={"takeover_mode": record["takeover_mode"]})
    print(f"Stage ledger: {ledger.path} (acceptance PASS"
          + (", PRACTICE: unlocks nothing)" if practice else ")"))


# -- command line ---------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    steps = parser.add_subparsers(dest="step", required=True)

    def add(name: str, help_text: str, *, session: bool = True, mode: bool = False) -> Any:
        sub = steps.add_parser(name, help=help_text)
        if session:
            sub.add_argument("--session", type=Path, required=True,
                             help="the folder probe created")
        if mode:
            sub.add_argument("--mode", choices=("walking", "prepare"), default="walking",
                             help="robot mode for the takeover (set it with the remote)")
            sub.add_argument("--attested-robot-id",
                             help="the serial number, when the robot cannot report it")
        sub.add_argument("--model", type=Path, help="the K1 MJCF (GMR's K1_serial.xml)")
        return sub

    sub = add("preflight", "check this PC before the robot is on", session=False)
    sub.add_argument("--transport", choices=("fake", "booster"), default="fake")
    sub.add_argument("--net", default="", help="this PC's interface or IP on the robot network")
    sub.add_argument("--robot-ip", default=ROBOT_IP, help=f"the robot's address (default {ROBOT_IP})")
    sub = add("probe", "read-only report; starts a session", session=False)
    sub.add_argument("--transport", choices=("fake", "booster"), default="fake")
    sub.add_argument("--net", default="")
    sub.add_argument("--domain-id", type=int, default=0, help="DDS domain (Booster default 0)")
    sub.add_argument("--session", type=Path, help="probe again in this session")
    sub.add_argument("--root", type=Path, default=commissioning_root(),
                     help="where sessions are made (default $K1_COMMISSIONING_ROOT, else "
                     "<repo>/k1-commissioning)")
    sub.add_argument("--seconds", type=float)
    sub.add_argument("--attested-robot-id")
    sub.add_argument("--firmware", help="with --attested-robot-id: the firmware Booster's app shows")
    add("status", "the ladder: each step's latest attempt and why it is not valid",
        mode=False).add_argument(
        "--mode", choices=("walking", "prepare"), default="walking")
    add("mirror", "read-only 3D view of the measured robot pose")
    add("zero-check", "read-only: measured link angles vs the model")
    add("quick-release", "takeover, then release at once (no motion)", mode=True)
    sub = add("hold", "arms hold still, then a normal release", mode=True)
    sub.add_argument("--seconds", type=float, default=10.0)
    add("balance", "30 s hold while you push gently (hoist)", mode=True)
    sub = add("command-loss", "commands stop for a short gap", mode=True)
    sub.add_argument("--gap", type=float, default=GAP_LEVELS[0], help=f"one of {GAP_LEVELS}")
    add("signs", "each arm joint 0.1 rad and back", mode=True)
    add("tpose", "demo: arms out to the T-pose, hold, back", mode=True)
    sub = add("sysid", "measure the arms: sweep, steps, sine or poses", mode=True)
    sub.add_argument("--test", choices=SYSID_TESTS, required=True)
    sub.add_argument("--joint", choices=ARMS, help="one arm joint (not for poses)")
    sub.add_argument("--amp", type=float, default=0.5, help="rad (sweep/steps/sine/forward)")
    sub.add_argument("--kp", type=float, default=40.0)
    sub.add_argument("--kd", type=float, default=2.0)
    sub = add("gains", "tracking at one kp/kd pair and level", mode=True)
    sub.add_argument("--level", default="L1", help="L1, L2 or L3")
    sub.add_argument("--kp", type=float, default=SAFE["kp"])
    sub.add_argument("--kd", type=float, default=SAFE["kd"])
    sub = add("record", "write the acceptance record from valid evidence")
    sub.add_argument("--mode", choices=("walking", "prepare"), default="walking")
    sub.add_argument("--operator", required=True)
    sub.add_argument("--gains-attempt", help="the gains attempt number to approve")
    sub.add_argument("--out", type=Path)
    return parser


STEPS: dict[str, Callable[[argparse.Namespace], int]] = {
    "preflight": preflight, "probe": run_probe, "status": run_status, "mirror": run_mirror,
    "zero-check": run_zero_check, "quick-release": run_quick_release, "hold": run_hold,
    "balance": run_balance, "command-loss": run_command_loss, "signs": run_signs, "sysid": run_sysid, "tpose": run_tpose,
    "gains": run_gains, "record": run_record,
}


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return STEPS[args.step](args)
    except (CommissionError, K1BridgeError) as exc:
        print(f"REFUSED: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
