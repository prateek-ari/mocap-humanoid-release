#!/usr/bin/env python3
"""Audit every G1 NPZ reference frame for fixed-base self-collision."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from mocap_g1.conditioning import load_qpos_npz  # noqa: E402
from mocap_g1.controller_screen import (  # noqa: E402
    ControllerScreenError,
    scan_reference_self_collisions,
)
from mocap_g1.io_utils import sha256_file, write_json_atomic  # noqa: E402


DEFAULT_MODEL = (
    Path(os.environ["MOCAP_WS"]) / "GMR/assets/unitree_g1/g1_mocap_29dof.xml"
    if os.environ.get("MOCAP_WS")
    else None
)


def _write(path: Path, report: dict[str, object]) -> None:
    write_json_atomic(path.resolve(), report, exists_error=ControllerScreenError, label="output")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--model", type=Path, default=DEFAULT_MODEL, required=DEFAULT_MODEL is None,
        help="G1 mocap MJCF (defaults under $MOCAP_WS)",
    )
    parser.add_argument("--window-seconds", type=float, default=10.0)
    args = parser.parse_args(argv)
    try:
        qpos, fps, metadata = load_qpos_npz(args.input)
        report = scan_reference_self_collisions(
            qpos,
            fps,
            model_path=args.model,
            window_seconds=args.window_seconds,
            source_sha256=sha256_file(args.input),
            source_metadata=metadata,
        )
        _write(args.output, report)
    except (ControllerScreenError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    print(
        f"{report['summary']['status']}: {report['summary']['self_collision_frames']}/"
        f"{report['summary']['frames']} frames self-collide; report: {args.output}"
    )
    return 0 if report["summary"]["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
