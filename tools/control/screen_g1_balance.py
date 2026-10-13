#!/usr/bin/env python3
"""Run the floating-base standing-balance screen on an offline G1 NPZ.

This is a SCREEN, not balance validation -- see mocap_g1/balance_screen.py
and docs/balance-screen-2026-08-16.md.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from mocap_g1.balance_screen import (  # noqa: E402
    BalanceScreenConfig,
    BalanceScreenError,
    run_balance_screen,
)
from mocap_g1.conditioning import load_qpos_npz  # noqa: E402
from mocap_g1.io_utils import sha256_file, write_json_atomic  # noqa: E402


DEFAULT_MODEL = (
    Path(os.environ["MOCAP_WS"]) / "GMR/assets/unitree_g1/g1_mocap_29dof.xml"
    if os.environ.get("MOCAP_WS")
    else None
)

_DEFAULTS = BalanceScreenConfig()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument(
        "--model",
        type=Path,
        default=DEFAULT_MODEL,
        required=DEFAULT_MODEL is None,
        help="G1 mocap MJCF (defaults to $MOCAP_WS/GMR/assets/unitree_g1/g1_mocap_29dof.xml)",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--start-seconds", type=float, default=0.0)
    parser.add_argument("--end-seconds", type=float)

    integration = parser.add_argument_group("integration")
    integration.add_argument("--timestep", type=float, default=_DEFAULTS.timestep_s)
    integration.add_argument("--settle-seconds", type=float, default=_DEFAULTS.settle_seconds)
    integration.add_argument("--speed-factor", type=float, default=_DEFAULTS.speed_factor)

    stance = parser.add_argument_group("stance (Unitree g1.yaml default_angles)")
    stance.add_argument("--stance-hip-pitch", type=float, default=_DEFAULTS.stance_hip_pitch_rad)
    stance.add_argument("--stance-knee", type=float, default=_DEFAULTS.stance_knee_rad)
    stance.add_argument(
        "--stance-ankle-pitch", type=float, default=_DEFAULTS.stance_ankle_pitch_rad
    )

    gains = parser.add_argument_group("gains")
    gains.add_argument(
        "--arm-kp", type=float, default=_DEFAULTS.arm_kp, help="published arm_sdk value"
    )
    gains.add_argument(
        "--arm-kd", type=float, default=_DEFAULTS.arm_kd, help="published arm_sdk value"
    )
    gains.add_argument(
        "--leg-kp", type=float, default=_DEFAULTS.leg_kp, help="screening surrogate, not published"
    )
    gains.add_argument(
        "--ankle-kp",
        type=float,
        default=_DEFAULTS.ankle_kp,
        help="screening surrogate; must exceed m*g*h_com/2 or the model topples",
    )
    gains.add_argument("--waist-kp", type=float, default=_DEFAULTS.waist_kp)
    gains.add_argument(
        "--stance-damping-ratio", type=float, default=_DEFAULTS.stance_damping_ratio
    )

    actuation = parser.add_argument_group("actuation")
    actuation.add_argument(
        "--torque-fraction", type=float, default=_DEFAULTS.torque_limit_fraction
    )
    actuation.add_argument(
        "--effort-urdf",
        type=Path,
        default=None,
        help="URDF supplying joint effort limits (defaults to eval/assets/g1_29dof_official.urdf)",
    )
    actuation.add_argument(
        "--limits-urdf",
        type=Path,
        default=None,
        help="mechanical URDF whose joint position limits replace the MJCF ranges "
             "(default: the vendored official G1 URDF)",
    )

    thresholds = parser.add_argument_group("gate thresholds")
    thresholds.add_argument("--com-margin-min-m", type=float, default=_DEFAULTS.com_margin_min_m)
    thresholds.add_argument(
        "--ankle-utilization-max", type=float, default=_DEFAULTS.ankle_torque_utilization_max
    )
    thresholds.add_argument(
        "--pelvis-tilt-max-rad", type=float, default=_DEFAULTS.pelvis_tilt_max_rad
    )
    thresholds.add_argument(
        "--pelvis-angular-speed-max-rad-s",
        type=float,
        default=_DEFAULTS.pelvis_angular_speed_max_rad_s,
    )
    thresholds.add_argument(
        "--pelvis-height-drop-max-m", type=float, default=_DEFAULTS.pelvis_height_drop_max_m
    )
    thresholds.add_argument("--contact-height-m", type=float, default=_DEFAULTS.contact_height_m)
    return parser


def _write_json(path: Path, value: dict[str, object]) -> None:
    write_json_atomic(path.resolve(), value, exists_error=BalanceScreenError, label="output")


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        qpos, fps, metadata = load_qpos_npz(args.input)
        if args.start_seconds < 0.0:
            raise BalanceScreenError("--start-seconds cannot be negative")
        if args.end_seconds is not None and args.end_seconds <= args.start_seconds:
            raise BalanceScreenError("--end-seconds must be greater than --start-seconds")
        start = int(round(args.start_seconds * fps))
        end = len(qpos) if args.end_seconds is None else int(round(args.end_seconds * fps))
        if start < 0 or start >= len(qpos) or end > len(qpos) or end - start < 3:
            raise BalanceScreenError("selected source range must contain at least three frames")
        report = run_balance_screen(
            qpos[start:end],
            fps,
            model_path=args.model,
            config=BalanceScreenConfig(
                timestep_s=args.timestep,
                settle_seconds=args.settle_seconds,
                speed_factor=args.speed_factor,
                stance_hip_pitch_rad=args.stance_hip_pitch,
                stance_knee_rad=args.stance_knee,
                stance_ankle_pitch_rad=args.stance_ankle_pitch,
                arm_kp=args.arm_kp,
                arm_kd=args.arm_kd,
                leg_kp=args.leg_kp,
                ankle_kp=args.ankle_kp,
                waist_kp=args.waist_kp,
                stance_damping_ratio=args.stance_damping_ratio,
                torque_limit_fraction=args.torque_fraction,
                effort_urdf_path=str(args.effort_urdf) if args.effort_urdf else None,
                limits_urdf_path=str(args.limits_urdf) if args.limits_urdf else None,
                com_margin_min_m=args.com_margin_min_m,
                ankle_torque_utilization_max=args.ankle_utilization_max,
                pelvis_tilt_max_rad=args.pelvis_tilt_max_rad,
                pelvis_angular_speed_max_rad_s=args.pelvis_angular_speed_max_rad_s,
                pelvis_height_drop_max_m=args.pelvis_height_drop_max_m,
                contact_height_m=args.contact_height_m,
            ),
            source_sha256=sha256_file(args.input),
            source_metadata=metadata,
        )
        report["source_selection"] = {
            "start_frame_inclusive": start,
            "end_frame_exclusive": end,
            "exact_start_seconds": start / fps,
            "exact_end_seconds_exclusive": end / fps,
        }
        _write_json(args.output, report)
    except (BalanceScreenError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    gates = report["gates"]
    margin = gates["com_support_margin"]["worst_margin_m"]
    print(
        f"{report['balance_screen']['status']}: "
        f"com_margin={'n/a' if margin is None else format(margin, '.4f')} m, "
        f"ankle_utilization={gates['ankle_torque_headroom']['worst_utilization']:.3f}, "
        f"pelvis_tilt={gates['pelvis_attitude']['max_tilt_rad']:.4f} rad, "
        f"pelvis_omega={gates['pelvis_attitude']['max_angular_speed_rad_s']:.3f} rad/s"
    )
    if report["balance_screen"]["reasons"]:
        print("reasons: " + ", ".join(report["balance_screen"]["reasons"]))
    print(f"report: {args.output}")
    return 0 if report["balance_screen"]["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
