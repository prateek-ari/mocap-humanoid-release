#!/usr/bin/env python3
"""Headlessly retarget a calibrated 3DSM Xsens BVH to canonical G1 GMR output."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import pickle
import shutil
import sys
import tempfile
import time

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))
from mocap_g1.dataset import DatasetError, validate_offsets_for_bvh  # noqa: E402
from mocap_g1.gmr_pkl import (  # noqa: E402
    GMR_ROOT_QUAT_ORDER,
    ROOT_QUAT_ORDER_KEY,
    wxyz_to_xyzw,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gmr-root", required=True, type=Path)
    parser.add_argument("--bvh-file", required=True, type=Path)
    parser.add_argument("--offsets-file", required=True, type=Path)
    parser.add_argument("--save-path", required=True, type=Path)
    parser.add_argument("--robot", choices=("unitree_g1",), default="unitree_g1")
    parser.add_argument("--scale", type=float, required=True)
    parser.add_argument("--bvh-format", choices=("3DSM",), default="3DSM")
    parser.add_argument("--start", type=int)
    parser.add_argument("--end", type=int)
    parser.add_argument("--reset-to-zero", action="store_true")
    args = parser.parse_args()

    gmr_root = args.gmr_root.resolve()
    if not (gmr_root / "general_motion_retargeting").is_dir():
        parser.error(f"not a GMR checkout: {gmr_root}")
    if not args.bvh_file.is_file():
        parser.error(f"BVH does not exist: {args.bvh_file}")
    try:
        offsets = json.loads(args.offsets_file.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        parser.error(f"cannot read offsets calibration {args.offsets_file}: {exc}")
    if not isinstance(offsets, dict) or not offsets:
        parser.error("--offsets-file must be a non-empty JSON object")
    try:
        validate_offsets_for_bvh(offsets, args.bvh_file)
    except DatasetError as exc:
        parser.error(str(exc))
    if args.scale <= 0:
        parser.error("--scale must be positive")

    sys.path.insert(0, str(gmr_root))
    from general_motion_retargeting import GeneralMotionRetargeting  # noqa: E402
    from general_motion_retargeting.utils.xsens import load_xsens_file  # noqa: E402

    # The pinned loader consumes an argparse-like object and silently falls back
    # to zero rotation when ./offsets.json is absent. Run it in an isolated
    # directory containing the explicit calibration so that cannot happen.
    loader_args = argparse.Namespace(
        bvh_file=str(args.bvh_file.resolve()),
        scale=args.scale,
        start=args.start,
        end=args.end,
        reset_to_zero=args.reset_to_zero,
        bvh_format=args.bvh_format,
    )
    started = time.perf_counter()
    previous_cwd = Path.cwd()
    with tempfile.TemporaryDirectory(prefix="g1-xsens-calibration-") as temporary:
        calibration_dir = Path(temporary)
        shutil.copy2(args.offsets_file.resolve(), calibration_dir / "offsets.json")
        try:
            os.chdir(calibration_dir)
            frames, human_height, frame_time = load_xsens_file(loader_args)
        finally:
            os.chdir(previous_cwd)
    if not frames or frame_time <= 0:
        raise RuntimeError("Xsens loader returned no frames or an invalid frame time")
    retargeter = GeneralMotionRetargeting(
        src_human="bvh_xsens",
        tgt_robot=args.robot,
        actual_human_height=human_height,
    )
    qpos = np.asarray([retargeter.retarget(frame) for frame in frames], dtype=np.float64)
    if qpos.ndim != 2 or qpos.shape[1] != 36 or not np.isfinite(qpos).all():
        raise RuntimeError(f"retarget output must be finite [T,36], got {qpos.shape}")

    fps = int(round(1.0 / frame_time))
    # GMR pkl convention: root_rot is scalar-last xyzw (MuJoCo qpos is wxyz),
    # with the order declared explicitly -- see mocap_g1/gmr_pkl.py.  This
    # writer used to store the raw wxyz qpos slice, unlike every other writer.
    motion = {
        "fps": fps,
        "root_pos": qpos[:, :3],
        "root_rot": np.ascontiguousarray(wxyz_to_xyzw(qpos[:, 3:7])),
        ROOT_QUAT_ORDER_KEY: GMR_ROOT_QUAT_ORDER,
        "dof_pos": qpos[:, 7:],
        "local_body_pos": None,
        "link_body_list": None,
    }
    args.save_path.parent.mkdir(parents=True, exist_ok=True)
    with args.save_path.open("wb") as handle:
        pickle.dump(motion, handle, protocol=pickle.HIGHEST_PROTOCOL)
    elapsed = time.perf_counter() - started
    print(json.dumps({
        "frames": len(frames),
        "fps": fps,
        "human_height_m": human_height,
        "elapsed_s": elapsed,
        "throughput_fps": len(frames) / max(elapsed, 1e-9),
        "output": str(args.save_path.resolve()),
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
