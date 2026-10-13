#!/usr/bin/env python3
"""Create a bounded, fail-closed world-contact-refined G1 NPZ derivative."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from mocap_g1.contact_refine import (  # noqa: E402
    ContactRefineConfig,
    ContactRefineError,
    refine_reference_contacts,
    write_refined_npz,
)
from mocap_g1.conditioning import load_qpos_npz  # noqa: E402
from mocap_g1.controller_screen import ControllerScreenError  # noqa: E402
from mocap_g1.io_utils import sha256_file, write_json_atomic  # noqa: E402


DEFAULT_MODEL = (
    Path(os.environ["MOCAP_WS"]) / "GMR/assets/unitree_g1/g1_mocap_29dof.xml"
    if os.environ.get("MOCAP_WS")
    else None
)

_DEFAULTS = ContactRefineConfig()


def _write_report(path: Path, report: dict[str, object]) -> None:
    write_json_atomic(path.resolve(), report, exists_error=ContactRefineError, label="report")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument(
        "--model", type=Path, default=DEFAULT_MODEL, required=DEFAULT_MODEL is None,
        help="G1 mocap MJCF (defaults under $MOCAP_WS)",
    )
    parser.add_argument("--contact-height-band-m", type=float,
                        default=_DEFAULTS.contact_height_band_m,
                        help="absolute world-frame band above z=0 within which a sole "
                             "counts as a contact candidate")
    parser.add_argument("--contact-vz-max-m-s", type=float,
                        default=_DEFAULTS.contact_vz_max_m_s)
    parser.add_argument("--penetration-tolerance-m", type=float,
                        default=_DEFAULTS.penetration_tolerance_m)
    parser.add_argument("--target-clearance-m", type=float,
                        default=_DEFAULTS.target_clearance_m)
    parser.add_argument("--max-ankle-correction-rad", type=float,
                        default=_DEFAULTS.max_ankle_correction_rad)
    parser.add_argument("--max-root-z-lift-m", type=float,
                        default=_DEFAULTS.max_root_z_lift_m)
    parser.add_argument("--no-skate-repair", action="store_true",
                        help="detect skating but never repair it (still rejects)")
    parser.add_argument("--skate-speed-threshold-m-s", type=float,
                        default=_DEFAULTS.skate_speed_threshold_m_s)
    parser.add_argument("--max-root-shift-step-m", type=float,
                        default=_DEFAULTS.max_root_shift_step_m)
    parser.add_argument("--max-segment-pinning-displacement-m", type=float,
                        default=_DEFAULTS.max_segment_pinning_displacement_m)
    parser.add_argument(
        "--limits-urdf", type=Path, default=None,
        help="mechanical URDF bounding the ankle corrections (default: the vendored "
             "official G1 URDF; never the MJCF's IK-narrowed ranges)",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    output: Path | None = None
    created_output_identity: tuple[int, int] | None = None
    try:
        output = args.output.resolve()
        report_path = args.report.resolve()
        if output == report_path:
            raise ContactRefineError("--output and --report must be different paths")
        if output.exists():
            raise ContactRefineError(f"refusing to overwrite output: {output}")
        if report_path.exists():
            raise ContactRefineError(f"refusing to overwrite report: {report_path}")

        source_hash_before = sha256_file(args.input)
        qpos, fps, metadata = load_qpos_npz(args.input)
        result = refine_reference_contacts(
            qpos,
            fps,
            model_path=args.model,
            config=ContactRefineConfig(
                contact_height_band_m=args.contact_height_band_m,
                contact_vz_max_m_s=args.contact_vz_max_m_s,
                penetration_tolerance_m=args.penetration_tolerance_m,
                target_clearance_m=args.target_clearance_m,
                max_ankle_correction_rad=args.max_ankle_correction_rad,
                max_root_z_lift_m=args.max_root_z_lift_m,
                skate_repair_enabled=not args.no_skate_repair,
                skate_speed_threshold_m_s=args.skate_speed_threshold_m_s,
                max_root_shift_step_m=args.max_root_shift_step_m,
                max_segment_pinning_displacement_m=args.max_segment_pinning_displacement_m,
                limits_urdf_path=str(args.limits_urdf) if args.limits_urdf else None,
            ),
            source_sha256=source_hash_before,
            source_metadata=metadata,
        )
        source_hash_after = sha256_file(args.input)
        result.report["input"]["source_file_sha256_before"] = source_hash_before
        result.report["input"]["source_file_sha256_after"] = source_hash_after
        result.report["input"]["source_file_preserved"] = source_hash_before == source_hash_after
        if source_hash_after != source_hash_before:
            raise ContactRefineError("source file changed during contact refinement")

        if result.qpos is not None:
            write_refined_npz(output, result, metadata)
            created_stat = os.stat(output, follow_symlinks=False)
            created_output_identity = (created_stat.st_dev, created_stat.st_ino)
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
    except (ContactRefineError, ControllerScreenError, OSError, ValueError) as exc:
        cleanup_error: OSError | None = None
        if output is not None and created_output_identity is not None:
            try:
                current_stat = os.stat(output, follow_symlinks=False)
                current_identity = (current_stat.st_dev, current_stat.st_ino)
                if current_identity == created_output_identity:
                    output.unlink()
            except FileNotFoundError:
                pass
            except OSError as unlink_exc:
                cleanup_error = unlink_exc
        print(f"ERROR: {exc}", file=sys.stderr)
        if cleanup_error is not None:
            print(
                f"ERROR: could not remove incomplete output {output}: {cleanup_error}",
                file=sys.stderr,
            )
        return 2

    summary = result.report["summary"]
    print(
        f"{result.report['status']}: penetration_repaired={summary['penetration_repaired_frames']}/"
        f"{summary['frames']}, penetration_failed={summary['penetration_failed_frames']}, "
        f"skating_segments_repaired={summary['skating_segments_repaired']}, "
        f"residual_penetrations={summary['residual_penetration_frames']}, "
        f"residual_skating={summary['residual_skating_frames']}"
    )
    print(f"report: {report_path}")
    if result.qpos is not None:
        print(f"output: {output}")
    return 0 if result.report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
