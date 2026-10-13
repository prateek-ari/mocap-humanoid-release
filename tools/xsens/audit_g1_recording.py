#!/usr/bin/env python3
"""Audit a retargeted G1 NPZ in deterministic fixed-duration windows."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from mocap_g1.io_utils import write_text_atomic  # noqa: E402
from mocap_g1.schema import load_official_joint_limits  # noqa: E402
from mocap_g1.window_audit import (  # noqa: E402
    AuditConfig,
    SCHEMA_VERSION,
    WindowAuditError,
    audit_npz,
)


DEFAULT_URDF = REPO_ROOT / "eval/assets/g1_29dof_official.urdf"
_DEFAULTS = AuditConfig()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read-only G1 NPZ audit: fixed windows, joint speed/acceleration, "
            "official position/velocity limits, and one-frame discontinuities."
        )
    )
    parser.add_argument("recording", type=Path, help="G1 .npz recording/cache")
    parser.add_argument(
        "--output",
        type=Path,
        help="write JSON atomically to this separate path (default: stdout)",
    )
    parser.add_argument("--window-seconds", type=float, default=_DEFAULTS.window_seconds)
    parser.add_argument(
        "--discontinuity-rad",
        type=float,
        default=_DEFAULTS.discontinuity_rad,
        help="absolute one-frame joint delta threshold (default: %(default)s rad)",
    )
    parser.add_argument(
        "--limit-fraction",
        type=float,
        default=_DEFAULTS.limit_fraction,
        help="fraction of official position range and velocity limit (default: %(default)s)",
    )
    parser.add_argument(
        "--position-tolerance-rad",
        type=float,
        default=_DEFAULTS.position_tolerance_rad,
        help="position-limit comparison tolerance (default: %(default)s rad)",
    )
    parser.add_argument("--candidate-count", type=int, default=_DEFAULTS.candidate_count)
    parser.add_argument("--limits-urdf", type=Path, default=DEFAULT_URDF)
    return parser


def _failure(recording: Path, message: str) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "source": {"path": str(recording.resolve())},
        "summary": {
            "status": "FAIL",
            "reasons": ["input_validation_failed"],
        },
        "error": {"type": "WindowAuditError", "message": message},
    }


def _write_json(path: Path, payload: str) -> None:
    write_text_atomic(path.resolve(), payload, overwrite=True)


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        limits = load_official_joint_limits(args.limits_urdf)
        report = audit_npz(
            args.recording,
            limits,
            AuditConfig(
                window_seconds=args.window_seconds,
                discontinuity_rad=args.discontinuity_rad,
                limit_fraction=args.limit_fraction,
                position_tolerance_rad=args.position_tolerance_rad,
                candidate_count=args.candidate_count,
            ),
        )
        exit_code = 0 if report["summary"]["status"] == "PASS" else 1
    except (OSError, ValueError, WindowAuditError) as exc:
        report = _failure(args.recording, str(exc))
        exit_code = 2

    payload = json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if args.output is None:
        sys.stdout.write(payload)
    else:
        try:
            if args.output.resolve() == args.recording.resolve():
                raise WindowAuditError("--output must not overwrite the source recording")
            _write_json(args.output, payload)
        except (OSError, WindowAuditError) as exc:
            sys.stderr.write(f"cannot write audit report: {exc}\n")
            return 2
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
