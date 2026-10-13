#!/usr/bin/env python3
"""Independent G1 Non-Skating re-derivation without importing eval.metrics."""

from __future__ import annotations

import argparse
import pickle
from pathlib import Path

import sys

import mujoco as mj
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
# Only the pkl quaternion-ORDER contract is shared; the metric itself stays
# independent of eval.metrics by design.
from mocap_g1.gmr_pkl import root_rot_as  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pkl", type=Path)
    parser.add_argument("--mjcf", type=Path, required=True)
    parser.add_argument("--quat-order", choices=("xyzw", "wxyz"), default=None,
                        help="LEGACY override for pkls WITHOUT a root_quat_order key; "
                             "default requires the key")
    args = parser.parse_args()

    model = mj.MjModel.from_xml_path(str(args.mjcf))
    data = mj.MjData(model)
    with args.pkl.open("rb") as handle:
        motion = pickle.load(handle)
    fps = motion["fps"]
    frames = len(motion["dof_pos"])
    root_rot_wxyz = root_rot_as(motion, "wxyz", legacy_order=args.quat_order, source=args.pkl)
    feet = {
        side: mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, f"{side}_ankle_roll_link")
        for side in ("left", "right")
    }
    geoms = {
        side: [
            geom_id
            for geom_id in range(model.ngeom)
            if model.geom_bodyid[geom_id] == body_id and model.geom_contype[geom_id]
        ]
        for side, body_id in feet.items()
    }
    positions = {side: np.zeros((frames, 3)) for side in feet}
    lowest = {side: np.zeros(frames) for side in feet}
    for frame in range(frames):
        data.qpos[:3] = motion["root_pos"][frame]
        data.qpos[3:7] = root_rot_wxyz[frame]
        data.qpos[7:] = motion["dof_pos"][frame]
        mj.mj_forward(model, data)
        for side in feet:
            positions[side][frame] = data.xpos[feet[side]]
            lowest[side][frame] = min(
                data.geom_xpos[geom_id][2] - model.geom_size[geom_id][0]
                for geom_id in geoms[side]
            )

    acceptable = np.ones(frames, dtype=bool)
    any_contact = np.zeros(frames, dtype=bool)
    for side in feet:
        vertical_speed = np.gradient(lowest[side], 1.0 / fps)
        planar_speed = np.linalg.norm(
            np.gradient(positions[side][:, :2], 1.0 / fps, axis=0), axis=1
        )
        contact = (
            (lowest[side] < np.percentile(lowest[side], 5.0) + 0.03)
            & (np.abs(vertical_speed) < 0.2)
        )
        any_contact |= contact
        acceptable &= ~contact | (planar_speed < 0.10)
    if not any_contact.any():
        print("FAIL: no contact frames inferred")
        return 1
    score = 100.0 * acceptable[any_contact].mean()
    print(f"independent Non-Skating: {score:.2f}% ({int(any_contact.sum())} contact frames)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
