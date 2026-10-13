#!/usr/bin/env python3
"""Validate the patched G1 retarget suite against its unpatched baseline.

Exit status is 0 only if EVERY clip passes EVERY criterion; 1 otherwise (a
missing or unreadable pkl counts as a failed clip).  Criteria, per clip:

  * patched root_pos/root_rot/dof_pos are all finite;
  * patched root quaternions are unit length within
    ``UNIT_QUATERNION_TOLERANCE`` (1e-3, the repo-wide loader tolerance);
  * patched and unpatched frame counts match;
  * 100 % of patched dof samples are inside the MJCF joint ranges
    (+-1e-9 rad numerical allowance);
  * both pkls' root quaternion order resolves (``root_quat_order`` key, or
    an explicit ``--quat-order`` legacy override for keyless suites).

The root-height delta, max |dof diff| and foot-height statistics are
reported for review but are not pass/fail criteria.
"""

from __future__ import annotations

import argparse
import os
import pickle
from pathlib import Path
import sys

import mujoco as mj
import numpy as np
from general_motion_retargeting.params import ROBOT_XML_DICT

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
from mocap_g1.gmr_pkl import (  # noqa: E402
    UNIT_QUATERNION_TOLERANCE,
    root_rot_as,
    xyzw_to_wxyz,
)


CLIPS = [
    "dance1_subject2",
    "walk3_subject1",
    "run1_subject5",
    "fight1_subject3",
    "obstacles1_subject1",
    "fallAndGetUp3_subject1",
]
ROBOT = "unitree_g1"


def load_pickle(path: Path) -> dict:
    with path.open("rb") as handle:
        return pickle.load(handle)


def foot_min_z(model: mj.MjModel, data: mj.MjData, motion: dict,
               stride: int = 10, root_rot_wxyz: np.ndarray | None = None) -> np.ndarray:
    foot_bodies = [
        body_id
        for body_id in range(model.nbody)
        if any(
            token in (mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, body_id) or "")
            for token in ("ankle_roll", "foot", "Foot")
        )
    ]
    if not foot_bodies:
        raise RuntimeError("G1 MJCF has no recognizable foot bodies")

    values = []
    for frame in range(0, motion["root_pos"].shape[0], stride):
        qpos = np.zeros(model.nq)
        qpos[:3] = motion["root_pos"][frame]
        if root_rot_wxyz is None:
            qpos[3:7] = xyzw_to_wxyz(motion["root_rot"][frame])
        else:
            qpos[3:7] = root_rot_wxyz[frame]
        qpos[7:] = motion["dof_pos"][frame]
        data.qpos[:] = qpos
        mj.mj_forward(model, data)
        values.append(min(data.xpos[body_id][2] for body_id in foot_bodies))
    return np.asarray(values)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--workspace",
        type=Path,
        default=Path(os.environ["MOCAP_WS"]) if os.environ.get("MOCAP_WS") else None,
        help="runtime workspace (or set MOCAP_WS); used to derive suite paths",
    )
    parser.add_argument(
        "--patched",
        type=Path,
        default=None,
        help="directory containing patched <clip>.pkl files",
    )
    parser.add_argument(
        "--unpatched",
        type=Path,
        default=None,
        help="directory containing unpatched <clip>.pkl files",
    )
    parser.add_argument("--stride", type=int, default=10, help="FK sampling stride")
    parser.add_argument(
        "--quat-order", choices=("xyzw", "wxyz"), default=None,
        help="LEGACY override: root_rot order for pkls WITHOUT a root_quat_order key "
             "(GMR-native suites are xyzw); default requires the key",
    )
    args = parser.parse_args()
    if args.patched is None or args.unpatched is None:
        if args.workspace is None:
            parser.error("set --workspace/MOCAP_WS or provide both --patched and --unpatched")
        args.patched = args.patched or args.workspace / "out/g1_baseline/patched"
        args.unpatched = args.unpatched or args.workspace / "out/g1_baseline/unpatched"

    model = mj.MjModel.from_xml_path(str(ROBOT_XML_DICT[ROBOT]))
    data = mj.MjData(model)
    joint_ranges = model.jnt_range[1:]
    print(f"== G1 patched validation ({ROBOT}), ndof={model.nq - 7}")

    failed: dict[str, list[str]] = {}
    for clip in CLIPS:
        try:
            patched = load_pickle(args.patched / f"{clip}.pkl")
            unpatched = load_pickle(args.unpatched / f"{clip}.pkl")
            patched_wxyz = root_rot_as(patched, "wxyz", legacy_order=args.quat_order,
                                       source=f"patched/{clip}.pkl")
            unpatched_wxyz = root_rot_as(unpatched, "wxyz", legacy_order=args.quat_order,
                                         source=f"unpatched/{clip}.pkl")
        except (OSError, pickle.UnpicklingError, EOFError, KeyError, ValueError) as exc:
            failed[clip] = [f"unreadable: {type(exc).__name__}: {exc}"]
            print(f"{clip}: FAIL unreadable ({type(exc).__name__}: {exc})")
            continue
        finite = all(
            np.isfinite(np.asarray(patched[key], dtype=float)).all()
            for key in ("root_pos", "root_rot", "dof_pos")
        )
        quaternion_error = float(np.abs(
            np.linalg.norm(patched["root_rot"], axis=1) - 1
        ).max())
        frames_match = patched["root_pos"].shape[0] == unpatched["root_pos"].shape[0]
        in_range = np.mean(
            (patched["dof_pos"] >= joint_ranges[:, 0] - 1e-9)
            & (patched["dof_pos"] <= joint_ranges[:, 1] + 1e-9)
        ) * 100
        root_delta = (
            patched["root_pos"][:, 2].mean() - unpatched["root_pos"][:, 2].mean()
        )
        max_dof_diff = (
            np.abs(patched["dof_pos"] - unpatched["dof_pos"]).max()
            if frames_match
            else float("nan")
        )
        foot_z_patched = foot_min_z(model, data, patched, args.stride, patched_wxyz)
        foot_z_unpatched = foot_min_z(model, data, unpatched, args.stride, unpatched_wxyz)
        problems = []
        if not finite:
            problems.append("non-finite values")
        if not quaternion_error <= UNIT_QUATERNION_TOLERANCE:
            problems.append(f"maxquaterr {quaternion_error:.2e} > {UNIT_QUATERNION_TOLERANCE:g}")
        if not frames_match:
            problems.append("frame count mismatch")
        if not in_range == 100.0:
            problems.append(f"dof_in_range {in_range:.4f}% < 100%")
        if problems:
            failed[clip] = problems
        print(
            f"{clip}: frames={patched['root_pos'].shape[0]} match={frames_match} "
            f"finite={finite} maxquaterr={quaternion_error:.2e} "
            f"dof_in_range={in_range:.2f}% mean_root_dz={root_delta:+.4f} "
            f"max|dof_diff|={max_dof_diff:.4f} "
            f"footmin_z patched[med={np.median(foot_z_patched):+.4f} "
            f"min={foot_z_patched.min():+.4f}] "
            f"unpatched[med={np.median(foot_z_unpatched):+.4f} "
            f"min={foot_z_unpatched.min():+.4f}] "
            f"{'FAIL ' + '; '.join(problems) if problems else 'PASS'}"
        )

    if failed:
        print(f"FAIL: {len(failed)}/{len(CLIPS)} clip(s) failed: {sorted(failed)}")
        return 1
    print(f"PASS: all {len(CLIPS)} clips met every criterion")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
