#!/usr/bin/env python3
"""Validate, retarget, score, export, and manifest an Xsens-to-G1 dataset job."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from mocap_g1.dataset import DatasetError, load_and_validate_job, run_pipeline  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("job", help="dataset job JSON")
    parser.add_argument("--output-root", required=True, help="new dataset parent directory")
    parser.add_argument("--gmr-root", help="pinned GMR checkout used for retargeting")
    parser.add_argument("--python", default=sys.executable, help="GMR runtime Python")
    parser.add_argument(
        "--existing-pkl-dir",
        help="finalize existing <clip>.pkl files instead of invoking GMR",
    )
    parser.add_argument(
        "--skip-score",
        action="store_true",
        help="export and manifest without MuJoCo scoring (recorded in manifest)",
    )
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="validate job, calibration, and BVH metadata without writing output",
    )
    args = parser.parse_args()

    try:
        job = load_and_validate_job(args.job)
        if args.validate_only:
            print(
                f"PASS: {job['dataset_id']} — {len(job['_clips'])} clip(s), "
                f"{job['_fps']:.3f} FPS, explicit offsets calibration present"
            )
            return 0
        manifest = run_pipeline(
            job,
            args.output_root,
            gmr_root=args.gmr_root,
            python=args.python,
            existing_pkl_dir=args.existing_pkl_dir,
            skip_score=args.skip_score,
        )
    except DatasetError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    print(f"PASS: dataset generated with provenance manifest: {manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
