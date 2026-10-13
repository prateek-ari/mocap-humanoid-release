#!/usr/bin/env python3
"""Run a fixed-base bounded-PD MuJoCo screen on an offline G1 NPZ."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from mocap_g1.conditioning import load_qpos_npz  # noqa: E402
from mocap_g1.controller_screen import (  # noqa: E402
    ARM_SDK_KD,
    ARM_SDK_KP,
    ControllerScreenConfig,
    ControllerScreenError,
    run_upper_body_screen,
)
from mocap_g1.io_utils import sha256_file, write_json_atomic  # noqa: E402


DEFAULT_MODEL = (
    Path(os.environ["MOCAP_WS"]) / "GMR/assets/unitree_g1/g1_mocap_29dof.xml"
    if os.environ.get("MOCAP_WS")
    else None
)

_DEFAULTS = ControllerScreenConfig()


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
    parser.add_argument("--timestep", type=float, default=_DEFAULTS.timestep_s)
    parser.add_argument("--kp", type=float, default=ARM_SDK_KP)
    parser.add_argument("--kd", type=float, default=ARM_SDK_KD)
    parser.add_argument("--torque-fraction", type=float, default=_DEFAULTS.torque_limit_fraction)
    parser.add_argument("--speed-factor", type=float, default=_DEFAULTS.speed_factor)
    parser.add_argument("--settle-seconds", type=float, default=_DEFAULTS.settle_seconds)
    parser.add_argument("--tracking-p95-limit-rad", type=float,
                        default=_DEFAULTS.tracking_p95_limit_rad)
    parser.add_argument("--tracking-max-limit-rad", type=float,
                        default=_DEFAULTS.tracking_max_limit_rad)
    parser.add_argument("--saturation-fraction-limit", type=float,
                        default=_DEFAULTS.saturation_fraction_limit)
    parser.add_argument(
        "--limits-urdf", type=Path, default=None,
        help="mechanical URDF whose joint position limits replace the MJCF ranges "
             "(default: the vendored official G1 URDF)",
    )
    return parser


def _write_json(path: Path, value: dict[str, object]) -> None:
    write_json_atomic(path.resolve(), value, exists_error=ControllerScreenError, label="output")


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        qpos, fps, metadata = load_qpos_npz(args.input)
        if args.start_seconds < 0.0:
            raise ControllerScreenError("--start-seconds cannot be negative")
        if args.end_seconds is not None and args.end_seconds <= args.start_seconds:
            raise ControllerScreenError("--end-seconds must be greater than --start-seconds")
        start = int(round(args.start_seconds * fps))
        end = len(qpos) if args.end_seconds is None else int(round(args.end_seconds * fps))
        if start < 0 or start >= len(qpos) or end > len(qpos) or end - start < 3:
            raise ControllerScreenError("selected source range must contain at least three frames")
        report = run_upper_body_screen(
            qpos[start:end],
            fps,
            model_path=args.model,
            config=ControllerScreenConfig(
                timestep_s=args.timestep,
                kp=args.kp,
                kd=args.kd,
                torque_limit_fraction=args.torque_fraction,
                speed_factor=args.speed_factor,
                settle_seconds=args.settle_seconds,
                tracking_p95_limit_rad=args.tracking_p95_limit_rad,
                tracking_max_limit_rad=args.tracking_max_limit_rad,
                saturation_fraction_limit=args.saturation_fraction_limit,
                limits_urdf_path=str(args.limits_urdf) if args.limits_urdf else None,
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
    except (ControllerScreenError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    print(
        f"{report['controller_screen']['status']}: p95="
        f"{report['tracking']['pooled_p95_abs_rad']:.6f} rad, max="
        f"{report['tracking']['pooled_max_abs_rad']:.6f} rad, saturation="
        f"{100.0 * report['actuation']['torque_saturation_sample_fraction']:.3f}%"
    )
    print(f"report: {args.output}")
    return 0 if report["controller_screen"]["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
