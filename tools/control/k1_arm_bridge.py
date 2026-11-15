#!/usr/bin/env python3
"""Replay a recorded Booster K1 motion through the K1 arm bridge (mocap_g1/k1_bridge.py).

The input is the K1 recorded route's motion pickle
(``tools/retarget/retarget_robot_recorded.py --robot booster_k1``).  Pickles are
trusted local input only: loading one runs code.

``--transport fake`` (default) runs in process: the fake robot reports each
command back as its measured arm position, in walking mode and not fallen.
``--transport booster --probe`` connects to a real K1 read-only: it never
publishes and never takes over the arms, and reports what the acceptance
record needs (state rate, robot mode, fall messages, robot info, bindings).
``--transport booster`` replays on a real K1 as a gated stage:
``--session`` (the commissioning session) and ``--stage hoist|standing`` are
required, the stage gate (mocap_g1/stage_gate.py) must allow ``replay@<stage>``
for this robot and these settings, and the session's acceptance record must be
the one registered in the stage ledger.  Each run writes an attempt into the
session; a PASS (full duration of at least REPLAY_PASS_MIN_S, no fault, release
confirmed, tracking p95 below 0.15 rad) is written to the ledger, and an
unconfirmed release or a fall revokes the stage the replay ran under.  With
``--transport fake --session`` the same happens in the PRACTICE ledger only.

    python3 tools/control/k1_arm_bridge.py k1-preview.pkl
    python3 tools/control/k1_arm_bridge.py --transport booster --probe --net 192.168.10.10
    python3 tools/control/k1_arm_bridge.py k1.pkl --transport booster --net 192.168.10.10 \\
        --session k1-commissioning/<session> --stage hoist \\
        --yes-i-have-read-the-unverified-section

Exit status: 0 when the replay ended with the arms released, 1 when it ended on
a fault (arms released) or could not confirm the release, 2 for invalid input or
a refused start.
"""

from __future__ import annotations

import argparse
import contextlib
from collections.abc import Mapping
from dataclasses import dataclass
import json
import math
from pathlib import Path
import pickle
import sys
import time
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from mocap_g1.arm_bridge import ArmBridgeError  # noqa: E402
from mocap_g1.gmr_pkl import RootQuatOrderError, resolve_root_quat_order  # noqa: E402
from mocap_g1.io_utils import sha256_file  # noqa: E402
from mocap_g1.k1_bridge import (  # noqa: E402
    ARM_MOTOR_INDEX,
    AcceptanceRecord,
    BoosterTransport,
    FakeK1Transport,
    K1ArmBridge,
    K1BridgeConfig,
    K1BridgeError,
    LocoError,
    binding_version,
    motion_lock,
)
from mocap_g1.live_robots import (  # noqa: E402
    K1_JOINTS,
    LIVE_ROBOTS,
    robot_envelope_limits,
    robot_joint_ranges,
)
from mocap_g1.stage_gate import (  # noqa: E402
    STAGES,
    Ledger,
    StageGateError,
    commissioning_root,
    envelope_sha256,
    require,
)
from tools.control.g1_arm_bridge import (  # noqa: E402
    DdsReplayTick,
    iter_dds_replay_ticks,
    sigint_gate,
)

ROBOT = LIVE_ROBOTS["booster_k1"]
REPLAY_PASS_MIN_S = 10.0
REPLAY_P95_LIMIT_RAD = 0.15
#: Faults that revoke the stage a replay ran under; others are a FAIL only.
REVOKING_FAULTS = ("fall_state:",)
MANUAL_STOP = ("Release NOT confirmed: the arms may still be under this program's last command. "
               "Stop the robot by hand now: remote LT+BACK (damping), keep it on the hoist.")


def load_motion(path: Path, *, model_sha256: str | None, real: bool) -> tuple[np.ndarray, float]:
    """``(dof_pos [frames, 22], fps)`` of an admitted K1 recording; raises ``ValueError``."""
    with Path(path).open("rb") as handle:
        motion = pickle.load(handle)  # trusted local input (see the module docstring)
    if not isinstance(motion, dict):
        raise ValueError("motion pickle must hold a dict")
    metadata = motion.get("metadata") or {}
    if metadata.get("robot") != ROBOT.key:
        raise ValueError(f"motion is for {metadata.get('robot')!r}, not booster_k1")
    names = metadata.get("layout", {}).get("dof_joint_names")
    if list(names or ()) != list(K1_JOINTS):
        raise ValueError("motion joint order differs from the K1 profile")
    if model_sha256 is not None and metadata.get("mjcf", {}).get("sha256") != model_sha256:
        raise ValueError("motion was retargeted on a different K1 model than the one in use")
    try:
        resolve_root_quat_order(motion, source=path)
    except RootQuatOrderError as exc:
        raise ValueError(str(exc)) from exc
    dof = np.asarray(motion.get("dof_pos"), dtype=np.float64)
    fps = float(motion.get("fps", 0.0))
    if dof.ndim != 2 or dof.shape[1] != len(K1_JOINTS) or len(dof) < 2:
        raise ValueError(f"dof_pos must be [frames >= 2, {len(K1_JOINTS)}], got {dof.shape}")
    if not np.isfinite(dof).all() or not (math.isfinite(fps) and fps > 0):
        raise ValueError("motion has non-finite values or no positive fps")
    if real and not metadata.get("calibration", {}).get("quality_approved"):
        problem = arm_replay_problem(metadata, dof)
        if problem:
            raise ValueError(f"preview-grade calibration: refused on a real robot ({problem})")
    return dof, fps


def arm_replay_problem(metadata: dict[str, Any], dof: np.ndarray) -> str | None:
    """Why a preview-grade K1 recording may not replay its arms on a real robot, or
    ``None``: config arms read no offsets, so the official limits and a pinned model
    are what make it safe to replay."""
    if metadata.get("arm_replay_eligible") is not True:
        return "not arm-replay-eligible: retarget it again with the K1 config arm mode"
    if metadata.get("layout", {}).get("arm_mode") != "config":
        return "its arms were not retargeted in the config arm mode"
    if not (metadata.get("joint_limits") or {}).get("clamped"):
        return "it was not clamped to the official K1 limits"
    if not metadata.get("mjcf", {}).get("sha256"):
        return "it records no K1 model SHA-256"
    ranges = robot_joint_ranges(ROBOT)
    lower = np.array([low for _name, low, _high in ranges])
    upper = np.array([high for _name, _low, high in ranges])
    if ((dof < lower - 1e-9) | (dof > upper + 1e-9)).any():
        return "values lie outside the official K1 limits"
    return None


def arm_targets(frame: np.ndarray) -> dict[str, float]:
    return {name: float(frame[index]) for name, index in ARM_MOTOR_INDEX.items()}


def resolve_model(model: Path | None) -> Path | None:
    """The K1 MJCF for the contact check: ``--model`` or GMR's registered model."""
    if model is not None:
        return model
    try:
        from general_motion_retargeting import params
    except ImportError:
        return None
    path = Path(params.ROBOT_XML_DICT.get(ROBOT.key, ""))
    return path if path.is_file() else None


READINGS_TIMEOUT_S = 2.0
#: Hold the takeover pose this long before the replay starts (commissioning SETTLE_S).
TAKEOVER_SETTLE_S = 1.5


def wait_for_readings(transport: Any, timeout_s: float) -> None:
    """Wait until the robot has reported a joint state, its mode and a fall state."""
    end = time.monotonic() + timeout_s
    while not (transport.latest_state() and transport.latest_mode() and transport.latest_fall()):
        if time.monotonic() >= end:
            missing = [name for name, reading in (
                ("joint state", transport.latest_state()), ("robot mode", transport.latest_mode()),
                ("fall state", transport.latest_fall())) if reading is None]
            raise K1BridgeError(f"no {', '.join(missing)} from the robot after {timeout_s:g} s")
        time.sleep(0.01)


def contact_check(model_path: Path) -> Any:
    import mujoco

    from mocap_g1.arm_contact import ArmContact

    model = mujoco.MjModel.from_xml_path(str(model_path))
    return ArmContact(mujoco, model, ROBOT.arm_collision, ROBOT.dof_joints)


def probe(transport: BoosterTransport, seconds: float) -> dict[str, Any]:
    """Watch the robot without commanding it."""
    states, modes, falls = [], [], []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        for readings, latest in ((states, transport.latest_state()), (modes, transport.latest_mode()),
                                 (falls, transport.latest_fall())):
            if latest is not None and (not readings or readings[-1] != latest):
                readings.append(latest)
        time.sleep(0.001)

    def period(times: list[float]) -> float | None:
        return float(np.median(np.diff(times))) if len(times) > 2 else None

    try:
        info: dict[str, str] | str = transport.robot_info()
    except LocoError as exc:
        info = f"unavailable: {exc}"
    return {
        "sdk_binding_version": binding_version(),
        "robot_info": info,
        "low_state_period_s": period([state.receive_time_s for state in states]),
        "malformed_low_states": transport.malformed_states,
        "robot_modes": sorted({reading.value for reading in modes}),
        "fall_messages": len(falls),
        "fall_period_s": period([reading.time_s for reading in falls]),
        "fall_states": sorted({reading.value for reading in falls}),
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", type=Path, nargs="?", help="K1 motion pickle (recorded route)")
    parser.add_argument("--transport", choices=("fake", "booster"), default="fake")
    parser.add_argument("--probe", action="store_true",
                        help="booster only: read-only report, no publishing, no takeover")
    parser.add_argument("--probe-seconds", type=float, default=10.0)
    parser.add_argument("--acceptance-record", type=Path,
                        help="booster replay: the hoisted day-one acceptance record (JSON); "
                             "default: the session's k1-acceptance.json")
    parser.add_argument("--session", type=Path,
                        help="the commissioning session: the replay writes its attempt there "
                             "(required with --transport booster)")
    parser.add_argument("--stage", choices=("hoist", "standing"),
                        help="replay@hoist (S0r) or replay@standing (S1r); required with "
                             "--transport booster")
    parser.add_argument("--operator", help="who runs it (default: the record's operator)")
    parser.add_argument("--attested-robot-id",
                        help="the robot serial number, when GetRobotInfo cannot report it")
    parser.add_argument("--yes-i-have-read-the-unverified-section", action="store_true",
                        help="required with --transport booster")
    parser.add_argument("--net", dest="network_interface", default="",
                        help="booster: this PC's interface or IP on the robot network")
    parser.add_argument("--domain-id", type=int, default=0)
    parser.add_argument("--model", type=Path,
                        help="K1 MJCF for the contact check (default: GMR's booster_k1 model)")
    parser.add_argument("--rate", type=float, default=50.0, help="control rate (Hz)")
    parser.add_argument("--duration", type=float, help="seconds of motion (default: all)")
    defaults = K1BridgeConfig()
    parser.add_argument("--kp", type=float,
                        help=f"default: the acceptance record's approved kp, else {defaults.kp:g}")
    parser.add_argument("--kd", type=float,
                        help=f"default: the acceptance record's approved kd, else {defaults.kd:g}")
    parser.add_argument("--allow-gains-above-example", action="store_true")
    parser.add_argument("--max-speed", type=float,
                        help="arm speed cap (rad/s); default: the record's approved value, "
                             f"else {defaults.speed_max_rad_s:g}")
    parser.add_argument("--max-accel", type=float,
                        help="arm acceleration cap (rad/s^2); default: the record's approved "
                             f"value, else {defaults.accel_max_rad_s2:g}")
    parser.add_argument("--jsonl", type=Path, help="write one JSON record per control tick")
    return parser


@dataclass
class ReplayStage:
    """A replay run as a stage: its commissioning session, ledger and binding."""

    stage: str
    session: Any  # tools.control.k1_commission.Session
    ledger: Ledger
    identity: dict[str, str]
    settings: dict[str, Any]
    operator: str
    attempt: Any = None


def stop_settings(config: K1BridgeConfig, limits: Mapping[str, Any]) -> dict[str, Any]:
    """The timeouts and bounds that decide how a run stops (bound to a PASS exactly)."""
    safety = config.safety_config(limits)
    return {"source_stale_timeout_s": safety.stale_timeout_s,
            "source_hard_timeout_s": safety.hard_timeout_s,
            "state_stale_timeout_s": config.state_stale_timeout_s,
            "mode_timeout_s": config.mode_timeout_s, "fall_timeout_s": config.fall_timeout_s,
            "tracking_error_limit_rad": config.tracking_error_limit_rad,
            "release_attempts": config.release_attempts, "stop_timeout_s": config.stop_timeout_s}


def replay_settings(config: K1BridgeConfig, rate_hz: float, model_sha256: str | None,
                    limits: Mapping[str, Any]) -> dict[str, Any]:
    return {"kp": config.kp, "kd": config.kd, "speed_max_rad_s": config.speed_max_rad_s,
            "accel_max_rad_s2": config.accel_max_rad_s2, "control_rate_hz": rate_hz,
            "model_sha256": model_sha256, "envelope_sha256": envelope_sha256(limits),
            "stop_settings": stop_settings(config, limits)}


def open_stage(args: argparse.Namespace, *, real: bool, record: AcceptanceRecord | None,
               config: K1BridgeConfig, model_path: Path | None,
               limits: Mapping[str, Any]) -> ReplayStage | None:
    """The gated stage of this run (``None``: a plain fake replay); raises
    :class:`K1BridgeError` when the gate refuses it."""
    if args.session is None:
        if real:
            raise K1BridgeError("--transport booster requires --session (the commissioning "
                                "session) and --stage hoist|standing")
        return None
    if real and args.stage is None:
        raise K1BridgeError("--transport booster requires --stage hoist|standing")
    from tools.control.k1_commission import CommissionError, Session

    try:
        session = Session(args.session)
    except CommissionError as exc:
        raise K1BridgeError(str(exc)) from exc
    meta, stage = session.meta, f"replay@{args.stage or 'hoist'}"
    settings = replay_settings(config, args.rate, sha256_file(model_path) if model_path else None,
                               limits)
    if not real:  # practice: no gate, and only the PRACTICE ledger
        identity = {"serial": "PRACTICE", "firmware": str(meta.get("firmware")),
                    "sdk_binding_version": str(meta.get("sdk_binding_version"))}
        return ReplayStage(stage, session, Ledger(commissioning_root(), "PRACTICE", practice=True),
                           identity, settings, args.operator or "practice")
    assert record is not None
    if meta.get("transport") != "booster" or meta.get("practice") is not False:
        raise K1BridgeError("a practice session cannot run a real replay")
    serial = str(record.data["robot_identity"]["serial_number"])
    if meta.get("serial") != serial:
        raise K1BridgeError(f"the acceptance record's robot {serial} is not the session's "
                            f"{meta.get('serial')}")
    identity = {"serial": serial, "firmware": str(record.data["firmware"]),
                "sdk_binding_version": binding_version() or ""}
    ledger = Ledger(commissioning_root(), serial, practice=False)
    problems = require(stage, ledger=ledger, identity=identity, settings=settings)
    if problems:
        raise K1BridgeError(f"the stage gate refuses {STAGES[stage].title}:\n  "
                            + "\n  ".join(problems))
    acceptance = ledger.latest("acceptance") or {}
    registered = (acceptance.get("evidence") or {}).get("acceptance-record") or {}
    if registered.get("sha256") != sha256_file(args.acceptance_record):
        raise K1BridgeError("the acceptance record is not the one registered in the stage "
                            "ledger: run 11 · Record again")
    return ReplayStage(stage, session, ledger, identity, settings,
                       args.operator or str(record.data["operator"]))


def finish_stage(stage: ReplayStage, bridge: K1ArmBridge, *, completed: bool, played_s: float,
                 errors: Mapping[str, list[float]], reasons: Mapping[str, int],
                 jsonl: Path | None, problem: str | None = None) -> str:
    """Judge the attempt, write it, and write its PASS or revocation to the ledger."""
    p95 = {name: round(float(np.percentile(values, 95)), 4) if values else 0.0
           for name, values in errors.items()}
    worst = max(p95.values(), default=0.0)
    fault = problem or bridge.fault
    checks = {
        f"full duration (at least {REPLAY_PASS_MIN_S:g} s)":
            completed and played_s >= REPLAY_PASS_MIN_S - 1e-9,
        "no fault": fault is None,
        "release confirmed": bridge.state == "released",
        f"tracking p95 below {REPLAY_P95_LIMIT_RAD:g} rad": worst < REPLAY_P95_LIMIT_RAD,
    }
    verdict = "PASS" if all(checks.values()) else "FAIL"
    stage.attempt.finish(verdict, checks=checks, completed=completed, played_s=round(played_s, 3),
                         fault=fault, arms=bridge.state, tracking_p95_rad=p95, reasons=dict(reasons),
                         release_confirmed=bridge.state != "release_unconfirmed",
                         ticks=str(jsonl) if jsonl else None, identity=stage.identity)
    evidence = {"attempt": stage.attempt.path, **({"ticks": jsonl} if jsonl and jsonl.is_file()
                                                 else {})}
    revoke = bridge.state == "release_unconfirmed" or str(fault or "").startswith(REVOKING_FAULTS)
    gate = STAGES[stage.stage].gate[0]
    try:
        if verdict == "PASS":
            stage.ledger.record_pass(stage.stage, identity=stage.identity, settings=stage.settings,
                                     evidence=evidence, operator=stage.operator,
                                     extra={"session": str(stage.session.path),
                                            "attempt": stage.attempt.id, "played_s": played_s,
                                            "tracking_p95_rad": worst})
        if revoke:
            stage.ledger.record_revocation(
                gate, reason=f"{stage.stage} attempt {stage.attempt.id}: "
                             + ("release not confirmed" if bridge.state == "release_unconfirmed"
                                else str(fault)), evidence=evidence)
    except StageGateError as exc:
        print(f"ERROR: the stage ledger was not updated: {exc}", file=sys.stderr)
        verdict = "FAIL"
    print(f"{STAGES[stage.stage].title}: attempt {stage.attempt.id} {verdict}"
          + "".join(f"\n  {'PASS' if ok else 'FAIL'}  {name}" for name, ok in checks.items())
          + (f"\n  REVOKED {STAGES[gate].title}" if revoke else "")
          + f"\nEvidence: {stage.attempt.path}; ledger {stage.ledger.path}")
    return verdict


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    real = args.transport == "booster"
    if args.probe:
        if not real:
            parser.error("--probe needs --transport booster")
        transport = BoosterTransport(domain_id=args.domain_id,
                                     network_interface=args.network_interface)
        try:
            print(json.dumps(probe(transport, args.probe_seconds), indent=2))
        finally:
            transport.close()
        return 0
    if args.input is None:
        parser.error("a K1 motion pickle is required")
    if not (math.isfinite(args.rate) and 25.0 <= args.rate <= 500.0):
        parser.error("--rate must be 25-500 Hz")
    if real and not args.yes_i_have_read_the_unverified_section:
        parser.error("--transport booster requires --yes-i-have-read-the-unverified-section")
    record = None
    try:
        if real:
            if args.acceptance_record is None and args.session is not None:
                args.acceptance_record = args.session / "k1-acceptance.json"
            if args.acceptance_record is None:
                raise K1BridgeError("--transport booster requires --session and --stage")
            record = AcceptanceRecord.load(args.acceptance_record)
        approved = record.data.get("approved") if record is not None else None
        approved = approved if isinstance(approved, dict) else {}
        defaults = K1BridgeConfig()

        def setting(given: float | None, key: str, default: float) -> Any:
            return given if given is not None else approved.get(key, default)

        config = K1BridgeConfig(
            kp=setting(args.kp, "kp", defaults.kp), kd=setting(args.kd, "kd", defaults.kd),
            speed_max_rad_s=setting(args.max_speed, "speed_max_rad_s", defaults.speed_max_rad_s),
            accel_max_rad_s2=setting(args.max_accel, "accel_max_rad_s2", defaults.accel_max_rad_s2),
            allow_gains_above_example=args.allow_gains_above_example)
        config.validate()
        if record is not None:
            refused = record.problems() or record.setting_mismatches(config, binding_version())
            if refused:
                raise K1BridgeError("acceptance record refuses this run:\n  " + "\n  ".join(refused))
            config = record.bridge_config(config)
        model_path = resolve_model(args.model)
        if real and model_path is None:
            raise K1BridgeError("the contact check needs the K1 MJCF (--model)")
        dof, fps = load_motion(args.input, real=real,
                               model_sha256=sha256_file(model_path) if model_path else None)
        limits = robot_envelope_limits(ROBOT)
        stage = open_stage(args, real=real, record=record, config=config, model_path=model_path,
                           limits=limits)
    except (K1BridgeError, ValueError, OSError, pickle.UnpicklingError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    for name, value in arm_targets(dof[0]).items():
        if not limits[name].lower <= value <= limits[name].upper:
            print(f"ERROR: first-frame {name}={value:.3f} is outside its range", file=sys.stderr)
            return 2
    contact = contact_check(model_path) if model_path else None
    if contact is None:
        print("Contact check off: no K1 MJCF found (fake transport only).")

    control_dt = 1.0 / args.rate
    duration = (len(dof) - 1) / fps if args.duration is None else min(args.duration, (len(dof) - 1) / fps)
    steps = max(1, int(duration / control_dt))
    virtual = [0.0]  # the fake replay's clock

    def fake_clock() -> float:
        return virtual[0]

    def fake_sleep(seconds: float) -> None:
        virtual[0] += seconds

    transport: Any
    stack = contextlib.ExitStack()
    if real:
        clock, sleeper = time.monotonic, time.sleep
        try:
            stack.enter_context(motion_lock())
            transport = BoosterTransport(domain_id=args.domain_id,
                                         network_interface=args.network_interface)
        except K1BridgeError as exc:
            stack.close()
            print(f"ERROR: {exc}", file=sys.stderr)
            return 2
    else:
        clock, sleeper = fake_clock, fake_sleep
        transport = FakeK1Transport(streaming_clock=fake_clock)
        transport.feed(dof[0], 0.0)
    bridge = K1ArmBridge(transport, limits, config, contact=contact, clock=clock, sleeper=sleeper)
    jsonl = args.jsonl
    if stage is not None:
        stage.attempt = stage.session.start(stage.stage, stage.settings, {})
        jsonl = jsonl or stage.attempt.path.with_suffix(".jsonl")
    handle = jsonl.open("w", encoding="utf-8") if jsonl else None
    reasons: dict[str, int] = {}
    errors: dict[str, list[float]] = {name: [] for name in ARM_MOTOR_INDEX}
    completed, played_s, problem = False, 0.0, None
    finished, verdict = False, ""

    def finish() -> str:
        nonlocal finished
        if handle:
            handle.close()
        finished = True
        assert stage is not None
        return finish_stage(stage, bridge, completed=completed, played_s=played_s, errors=errors,
                            reasons=reasons, jsonl=jsonl, problem=problem)

    try:
        with sigint_gate() as gate:  # Ctrl-C waits for the takeover call, then stops cleanly
            try:
                if record is not None:
                    try:
                        info = transport.robot_info()
                    except LocoError:
                        info = None
                    mismatched = record.identity_mismatches(info, args.attested_robot_id)
                    if mismatched:
                        raise K1BridgeError("acceptance record does not match this robot:\n  "
                                            + "\n  ".join(mismatched))
                    wait_for_readings(transport, READINGS_TIMEOUT_S)
                hold = bridge.takeover().arm_positions()
            except K1BridgeError as exc:
                print(f"ERROR: {exc}", file=sys.stderr)
                problem = f"takeover:{exc}"
                if stage is not None:
                    finish()
                if bridge.state == "release_unconfirmed":
                    print(MANUAL_STOP, file=sys.stderr)
                    return 1
                return 2
            # A K1 (firmware 1.8.1.2) ignores arm commands for about 1 s after the takeover;
            # hold the takeover pose first, as the commissioning steps do (SETTLE_S).
            settle_end = clock() + TAKEOVER_SETTLE_S
            while clock() < settle_end:
                sleeper(control_dt)
                bridge.step(hold, source_timestamp=clock(), now=clock())
                if bridge.state != "tracking":
                    break
            start = clock()
            if real:
                ticks: Any = iter_dds_replay_ticks(steps=steps, control_dt=control_dt,
                                                   start_time=start, clock=clock, sleeper=sleeper)
            else:
                ticks = (DdsReplayTick(tick, tick * control_dt, start + tick * control_dt, 0)
                         for tick in range(1, steps + 1))
            try:
                gate.allow_interrupt()
                for tick in ticks:
                    if not real:
                        virtual[0] = tick.now_s
                    frame = dof[min(len(dof) - 1, int(tick.replay_time_s * fps))]
                    step = bridge.step(arm_targets(frame), source_timestamp=tick.now_s,
                                       now=tick.now_s)
                    reasons[step.reason] = reasons.get(step.reason, 0) + 1
                    measured = transport.latest_state()
                    tick_errors = bridge.tracking_errors(measured) if measured else None
                    for name, value in (tick_errors or {}).items():
                        errors[name].append(value)
                    if handle:
                        handle.write(json.dumps({
                            "tick": tick.tick, "t": tick.replay_time_s, "reason": step.reason,
                            "state": step.state, "state_age_s": step.state_age_s,
                            "arm": step.frame.arm_positions() if step.frame else None,
                            "errors": tick_errors,
                        }) + "\n")
                    if bridge.state != "tracking":
                        break
                    played_s = tick.replay_time_s
                else:
                    completed = True
            except KeyboardInterrupt:
                gate.defer()
                print("Stopping: decelerating, then releasing the arms.")
            except ArmBridgeError as exc:  # lost real time
                gate.defer()
                bridge.abort(f"real_time_lost:{exc}")
            except BaseException as exc:  # a bug or an I/O error: the arms go back first
                gate.defer()
                bridge.abort(f"unexpected:{exc!r}")
                problem = f"unexpected:{exc!r}"
                raise
            gate.defer()
            bridge.stop()
    finally:
        if handle:
            handle.close()
        transport.close()
        stack.close()
        if stage is not None and not finished:
            verdict = finish()
    print(f"K1 replay: {sum(reasons.values())} ticks, {reasons}; arms {bridge.state}"
          + (f"; fault {bridge.fault}" if bridge.fault else ""))
    if bridge.state == "release_unconfirmed":
        print(MANUAL_STOP, file=sys.stderr)
        return 1
    if stage is not None:
        return 0 if verdict == "PASS" else 1
    return 0 if bridge.fault is None else 1


if __name__ == "__main__":
    raise SystemExit(main())
