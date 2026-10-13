#!/usr/bin/env python3
"""Create a bounded, fail-closed collision-projected G1 NPZ derivative."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from mocap_g1.collision_gate import (  # noqa: E402
    CollisionGateConfig,
    CollisionGateError,
    gate_reference_self_collisions,
    write_projected_npz,
)
from mocap_g1.conditioning import load_qpos_npz  # noqa: E402
from mocap_g1.io_utils import sha256_file, write_json_atomic  # noqa: E402


DEFAULT_MODEL = (
    Path(os.environ["MOCAP_WS"]) / "GMR/assets/unitree_g1/g1_mocap_29dof.xml"
    if os.environ.get("MOCAP_WS")
    else None
)


def _write_report(path: Path, report: dict[str, object]) -> None:
    write_json_atomic(path.resolve(), report, exists_error=CollisionGateError, label="report")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument(
        "--model", type=Path, default=DEFAULT_MODEL, required=DEFAULT_MODEL is None,
        help="G1 mocap MJCF (defaults under $MOCAP_WS)",
    )
    parser.add_argument("--max-per-joint-rad", type=float, default=0.25)
    parser.add_argument("--max-total-rad", type=float, default=0.35)
    parser.add_argument("--max-correction-step-rad", type=float, default=0.05)
    parser.add_argument("--max-correction-velocity-rad-s", type=float, default=1.0)
    parser.add_argument("--max-correction-acceleration-rad-s2", type=float, default=4.0)
    parser.add_argument("--search-steps", type=int, default=24)
    parser.add_argument("--refinement-steps", type=int, default=18)
    parser.add_argument("--global-alpha-guard", type=float, default=0.02)
    parser.add_argument(
        "--projection-mode",
        choices=("per-frame-then-global", "per-frame-only", "global-only"),
        default="per-frame-then-global",
        help="use per-frame retreat with coherent fallback, or force one strategy",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        output = args.output.resolve()
        report_path = args.report.resolve()
        if output == report_path:
            raise CollisionGateError("--output and --report must be different paths")
        if output.exists():
            raise CollisionGateError(f"refusing to overwrite output: {output}")
        if report_path.exists():
            raise CollisionGateError(f"refusing to overwrite report: {report_path}")

        source_hash_before = sha256_file(args.input)
        qpos, fps, metadata = load_qpos_npz(args.input)
        result = gate_reference_self_collisions(
            qpos,
            fps,
            model_path=args.model,
            config=CollisionGateConfig(
                max_per_joint_deviation_rad=args.max_per_joint_rad,
                max_total_deviation_rad=args.max_total_rad,
                max_correction_step_rad=args.max_correction_step_rad,
                max_correction_velocity_rad_s=args.max_correction_velocity_rad_s,
                max_correction_acceleration_rad_s2=args.max_correction_acceleration_rad_s2,
                search_steps=args.search_steps,
                refinement_steps=args.refinement_steps,
                global_alpha_guard_fraction=args.global_alpha_guard,
                projection_mode=args.projection_mode.replace("-", "_"),
            ),
            source_sha256=source_hash_before,
            source_metadata=metadata,
        )
        source_hash_after = sha256_file(args.input)
        result.report["input"]["source_file_sha256_before"] = source_hash_before
        result.report["input"]["source_file_sha256_after"] = source_hash_after
        result.report["input"]["source_file_preserved"] = source_hash_before == source_hash_after
        if source_hash_after != source_hash_before:
            raise CollisionGateError("source file changed during collision projection")

        if result.qpos is not None:
            write_projected_npz(output, result, metadata)
            result.report["output"] = {
                "path": str(output),
                "sha256": sha256_file(output),
                "written": True,
            }
        else:
            result.report["output"] = {
                "path": str(output),
                "sha256": None,
                "written": False,
            }
        _write_report(report_path, result.report)
    except (CollisionGateError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    summary = result.report["summary"]
    print(
        f"{result.report['status']}: projected={summary['projected_frames']}/"
        f"{summary['frames']}, final_collisions={summary['final_self_collision_frames']}, "
        f"max_joint_delta={summary['max_per_joint_deviation_rad']:.6f} rad"
    )
    print(f"report: {report_path}")
    if result.qpos is not None:
        print(f"output: {output}")
    return 0 if result.report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
