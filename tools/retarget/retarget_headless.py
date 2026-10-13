"""Headless replica of GMR scripts/bvh_to_robot.py save path (no viewer).

Replicates the exact loop semantics of bvh_to_robot.py, including the
append-after-break ordering (last BVH frame is retargeted but NOT saved),
so outputs are bit-comparable with Phase-0 reference pkls.
Does not modify GMR source.
"""
import argparse
import pickle
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
from mocap_g1.gmr_pkl import (  # noqa: E402
    GMR_ROOT_QUAT_ORDER,
    ROOT_QUAT_ORDER_KEY,
    wxyz_to_xyzw,
)

from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting.utils.lafan1 import load_bvh_file

parser = argparse.ArgumentParser()
parser.add_argument("--bvh_file", required=True)
parser.add_argument("--format", default="lafan1")
parser.add_argument("--robot", default="unitree_g1")
parser.add_argument("--save_path", required=True)
parser.add_argument("--motion_fps", default=30, type=int)
args = parser.parse_args()

lafan1_data_frames, actual_human_height = load_bvh_file(args.bvh_file, format=args.format)

retargeter = GMR(
    src_human=f"bvh_{args.format}",
    tgt_robot=args.robot,
    actual_human_height=actual_human_height,
)

qpos_list = []
n = len(lafan1_data_frames)
t0 = time.perf_counter()
i = 0
while True:
    qpos = retargeter.retarget(lafan1_data_frames[i])
    i += 1
    if i >= n:
        break  # matches bvh_to_robot.py: last frame retargeted but not appended
    qpos_list.append(qpos)
elapsed = time.perf_counter() - t0

root_pos = np.array([q[:3] for q in qpos_list])
root_rot = np.array([wxyz_to_xyzw(q[3:7]) for q in qpos_list])
dof_pos = np.array([q[7:] for q in qpos_list])

motion_data = {
    "fps": args.motion_fps,
    "root_pos": root_pos,
    "root_rot": root_rot,
    ROOT_QUAT_ORDER_KEY: GMR_ROOT_QUAT_ORDER,  # explicit; see mocap_g1/gmr_pkl.py
    "dof_pos": dof_pos,
    "local_body_pos": None,
    "link_body_list": None,
}
Path(args.save_path).parent.mkdir(parents=True, exist_ok=True)
with open(args.save_path, "wb") as f:
    pickle.dump(motion_data, f)

print(
    f"RESULT bvh={args.bvh_file} bvh_frames={n} saved_frames={len(qpos_list)} "
    f"elapsed_s={elapsed:.2f} retarget_fps={n / elapsed:.2f} save_path={args.save_path}"
)
