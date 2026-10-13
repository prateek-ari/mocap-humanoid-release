#!/usr/bin/env python3
"""Condition an offline G1 preview NPZ into a new, capped trajectory.

This is a feasibility/smoothing stage, not a calibration procedure.  The
input is never modified and existing output/report files are never replaced.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from mocap_g1.conditioning import (  # noqa: E402
    ConditioningConfig,
    MotionConditioningError,
    condition_g1_motion,
    load_qpos_npz,
    write_conditioned_npz,
    write_json_report,
)
from mocap_g1.io_utils import sha256_file as _sha256  # noqa: E402
from mocap_g1.schema import G1_29DOF_JOINTS, load_official_joint_limits  # noqa: E402


DEFAULT_URDF = REPO_ROOT / "eval/assets/g1_29dof_official.urdf"


def _overrides(path: Path | None, label: str) -> dict[str, float]:
    if path is None:
        return {}
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise MotionConditioningError(f"cannot read {label} {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise MotionConditioningError(f"{label} must contain a JSON object")
    unknown = sorted(set(value) - set(G1_29DOF_JOINTS))
    if unknown:
        raise MotionConditioningError(f"{label} contains unknown joints: {unknown}")
    try:
        return {str(name): float(cap) for name, cap in value.items()}
    except (TypeError, ValueError) as exc:
        raise MotionConditioningError(f"{label} values must be numeric") from exc


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="source preview NPZ (left unchanged)")
    parser.add_argument("output", type=Path, help="new conditioned NPZ")
    parser.add_argument(
        "--start-seconds",
        type=float,
        default=0.0,
        help="start of source window in seconds (rounded to nearest input frame)",
    )
    parser.add_argument(
        "--end-seconds",
        type=float,
        help="exclusive end of source window (default: end of input)",
    )
    parser.add_argument(
        "--report",
        type=Path,
        help="new JSON report (default: <output stem>.report.json)",
    )
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument(
        "--velocity-fraction",
        type=float,
        default=0.25,
        help="fraction of official named joint velocity limits (default: 0.25)",
    )
    parser.add_argument(
        "--velocity-caps-json",
        type=Path,
        help="optional named per-joint velocity-cap overrides",
    )
    parser.add_argument(
        "--acceleration-cap",
        type=float,
        default=4.0,
        help="default acceleration cap for every joint, rad/s^2 (default: 4)",
    )
    parser.add_argument(
        "--acceleration-caps-json",
        type=Path,
        help="optional named per-joint acceleration-cap overrides",
    )
    parser.add_argument("--decimate", type=int, default=1, help="integer output decimation")
    parser.add_argument(
        "--cutoff-hz",
        type=float,
        help="FIR cutoff; default is 20%% of post-decimation FPS",
    )
    parser.add_argument("--filter-taps", type=int, default=31)
    parser.add_argument("--spike-threshold-rad", type=float, default=0.20)
    parser.add_argument("--max-time-scale", type=float, default=20.0)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    report_path = args.report or args.output.with_suffix(".report.json")
    try:
        if args.input.resolve() == args.output.resolve():
            raise MotionConditioningError("input and output paths must be different")
        if args.output.exists() or report_path.exists():
            existing = args.output if args.output.exists() else report_path
            raise MotionConditioningError(f"refusing to overwrite existing file: {existing}")
        if args.velocity_fraction <= 0.0:
            raise MotionConditioningError("--velocity-fraction must be positive")
        if args.acceleration_cap <= 0.0:
            raise MotionConditioningError("--acceleration-cap must be positive")

        qpos, fps, metadata = load_qpos_npz(args.input)
        source_frames = len(qpos)
        if args.start_seconds < 0.0:
            raise MotionConditioningError("--start-seconds cannot be negative")
        if args.end_seconds is not None and args.end_seconds <= args.start_seconds:
            raise MotionConditioningError("--end-seconds must be greater than --start-seconds")
        start_frame = int(round(args.start_seconds * fps))
        end_frame = (
            source_frames
            if args.end_seconds is None
            else int(round(args.end_seconds * fps))
        )
        if start_frame < 0 or start_frame >= source_frames:
            raise MotionConditioningError("selected start frame is outside the input")
        if end_frame > source_frames:
            raise MotionConditioningError("selected end frame is outside the input")
        if end_frame - start_frame < 3:
            raise MotionConditioningError("selected window must contain at least 3 frames")
        source_selection = {
            "source_total_frames": source_frames,
            "requested_start_seconds": args.start_seconds,
            "requested_end_seconds": args.end_seconds,
            "start_frame_inclusive": start_frame,
            "end_frame_exclusive": end_frame,
            "exact_start_seconds": start_frame / fps,
            "exact_end_seconds_exclusive": end_frame / fps,
        }
        qpos = qpos[start_frame:end_frame].copy()
        official = load_official_joint_limits(args.urdf)
        velocity_caps = {
            name: official[name].velocity * args.velocity_fraction
            for name in G1_29DOF_JOINTS
        }
        velocity_caps.update(_overrides(args.velocity_caps_json, "velocity caps JSON"))
        acceleration_caps = {
            name: args.acceleration_cap for name in G1_29DOF_JOINTS
        }
        acceleration_caps.update(
            _overrides(args.acceleration_caps_json, "acceleration caps JSON")
        )
        conditioned = condition_g1_motion(
            qpos,
            fps,
            velocity_caps=velocity_caps,
            acceleration_caps=acceleration_caps,
            # Mechanical limits from the same official URDF: FIR overshoot is
            # clamped, and an in-range source can never leave range.
            position_limits={
                name: (official[name].lower, official[name].upper)
                for name in G1_29DOF_JOINTS
            },
            config=ConditioningConfig(
                decimation=args.decimate,
                lowpass_cutoff_hz=args.cutoff_hz,
                filter_taps=args.filter_taps,
                spike_threshold_rad=args.spike_threshold_rad,
                max_time_scale=args.max_time_scale,
            ),
        )
        conditioned.report["source_selection"] = source_selection
        report = dict(conditioned.report)
        report["provenance"] = {
            "source_path": str(args.input.resolve()),
            "source_sha256": _sha256(args.input),
            "output_path": str(args.output.resolve()),
            "urdf_path": str(args.urdf.resolve()),
            "urdf_sha256": _sha256(args.urdf),
        }
        write_conditioned_npz(args.output, conditioned, metadata)
        try:
            report["provenance"]["output_sha256"] = _sha256(args.output)
            write_json_report(report_path, report)
        except Exception:
            # The NPZ was just created by this invocation; removing it restores
            # the all-or-nothing CLI contract and never touches caller input.
            args.output.unlink(missing_ok=True)
            raise
    except (MotionConditioningError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    limits = report["limits"]["remaining_violations"]
    print(
        f"PASS: {report['input']['frames']} -> {report['output']['frames']} frames, "
        f"{report['output']['duration_s']:.3f} s, time scale "
        f"{report['time_scaling']['factor']:.3f}x; remaining velocity/acceleration "
        f"violations={limits['velocity_rad_s']['samples_over_cap']}/"
        f"{limits['acceleration_rad_s2']['samples_over_cap']}"
    )
    print(f"conditioned NPZ: {args.output}")
    print(f"report: {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
