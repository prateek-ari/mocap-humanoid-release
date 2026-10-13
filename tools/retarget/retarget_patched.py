"""Patched-suite driver (GMR issue #180 fix applied in the working tree).

Identical to the unpatched G1 driver (same src_human tag, pkl schema, and
wxyz->xyzw root_rot conversion) EXCEPT it calls
retarget(frame, offset_to_ground=True) so the now-fixed
offset_human_data_to_ground() runs with the per-config self.ground_offset
(default 0.0) instead of upstream's hard-coded 0.1 float.
--drop_last replicates bvh_to_robot.py's append-after-break behavior when
strict upstream frame parity is required.
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
parser.add_argument("--robot", default="unitree_g1")
parser.add_argument("--save_path", required=True)
parser.add_argument("--motion_fps", type=int, default=30)
parser.add_argument("--drop_last", action="store_true",
                    help="replicate bvh_to_robot.py: last BVH frame retargeted but not saved")
args = parser.parse_args()

lafan1_data_frames, actual_human_height = load_bvh_file(args.bvh_file, format="lafan1")

retargeter = GMR(
    src_human="bvh_lafan1",
    tgt_robot=args.robot,
    actual_human_height=actual_human_height,
    verbose=False,
)

qpos_list = []
n = len(lafan1_data_frames)
t0 = time.perf_counter()
for i, frame in enumerate(lafan1_data_frames):
    qpos = retargeter.retarget(frame, offset_to_ground=True)
    if args.drop_last and i == n - 1:
        break
    qpos_list.append(qpos.copy())
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
    f"RESULT robot={args.robot} bvh={Path(args.bvh_file).name} bvh_frames={n} "
    f"saved_frames={len(qpos_list)} elapsed_s={elapsed:.2f} retarget_fps={n / elapsed:.2f} "
    f"save_path={args.save_path}"
)
