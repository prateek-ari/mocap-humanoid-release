"""GMR motion pkl loading, robot MJCF resolution and URDF limit parsing.

Split out of ``eval/metrics.py`` (still the public facade; see its module
docstring for the metric definitions and the changelog).
"""

from __future__ import annotations

import os
import pickle
from pathlib import Path

import numpy as np

from mocap_g1.gmr_pkl import (  # noqa: F401  (order constants re-exported)
    GMR_ROOT_QUAT_ORDER,
    ROOT_QUAT_ORDER_KEY,
    UNIT_QUATERNION_TOLERANCE,
    RootQuatOrderError,
    root_rot_as,
)

# ---------------------------------------------------------------------------
# Robot / asset resolution
# ---------------------------------------------------------------------------

_EVAL_DIR = Path(__file__).resolve().parents[1]

MOCAP_WS = Path(os.environ["MOCAP_WS"]).expanduser().resolve() if os.environ.get("MOCAP_WS") else None

# URDFs used as the source of joint VELOCITY limits (MJCF has no
# velocity-limit field). D5 (T4, 2026-08-04): the official vendor URDF is
# authoritative for anything quoted externally, so it is the default for G1
# (same vendored file as MECHANICAL_URDF, which already supplies POSITION
# limits) -- see retarget_configs/velocity-limit-sources.md for the 8-joint
# companion-vs-official table and full rationale.
DEFAULT_URDF = {
    "unitree_g1": _EVAL_DIR / "assets/g1_29dof_official.urdf",
    "unitree_g1_with_hands": _EVAL_DIR / "assets/g1_29dof_official.urdf",
}

# GMR's companion URDF (g1_custom_collision_29dof.urdf), kept only as the
# pre-D5 velocity-limit source for A/B comparison. NOT used by default as of
# D5 -- differs from the official URDF in velocity on 8 G1 joints (ankle
# pitch/roll x2, hip_roll x2, waist pitch/roll; see
# retarget_configs/velocity-limit-sources.md). To reproduce pre-D5 scores,
# pass ``--urdf`` pointed at this path explicitly.
LEGACY_COMPANION_URDF = ({
    "unitree_g1": MOCAP_WS / "GMR/assets/unitree_g1/g1_custom_collision_29dof.urdf",
    "unitree_g1_with_hands": MOCAP_WS / "GMR/assets/unitree_g1/g1_custom_collision_29dof.urdf",
} if MOCAP_WS else {})

# Mechanical URDFs used as the source of joint POSITION limits (D1): the MJCF
# jnt_range can be IK-narrowed (GMR's g1_mocap_29dof.xml narrows 15/29 G1
# joints, e.g. hip_pitch +-1.57 vs mechanical [-2.5307, 2.8798]), which is
# wrong for a feasibility metric. The vendored g1_29dof_official.urdf is the
# unmodified unitreerobotics/unitree_ros g1_description file (fetched
# 2026-08-03); its position limits are byte-identical to GMR's
# g1_custom_collision_29dof.urdf on all 29 joints.
MECHANICAL_URDF = {
    "unitree_g1": _EVAL_DIR / "assets/g1_29dof_official.urdf",
    "unitree_g1_with_hands": _EVAL_DIR / "assets/g1_29dof_official.urdf",
}


def resolve_robot_xml(robot: str, mjcf_override: str | None = None) -> Path:
    """Resolve a G1-family robot name through GMR's MJCF registry."""
    if mjcf_override:
        p = Path(mjcf_override)
        if not p.is_file():
            raise FileNotFoundError(f"--mjcf {p} does not exist")
        return p
    from general_motion_retargeting.params import ROBOT_XML_DICT  # editable install

    if robot in DEFAULT_URDF and robot in ROBOT_XML_DICT:
        return Path(ROBOT_XML_DICT[robot]).resolve()
    raise KeyError(
        f"Unsupported robot {robot!r}; expected one of {sorted(DEFAULT_URDF)} "
        "registered in GMR ROBOT_XML_DICT"
    )


def load_gmr_pkl(path: str | Path, quat_order: str | None = None) -> dict:
    """Load and validate a GMR motion pkl; root_rot is returned as wxyz (MuJoCo).

    ``quat_order`` is an explicit LEGACY override for pre-contract pkls that
    carry no ``root_quat_order`` key (see ``mocap_g1.gmr_pkl``); it must
    agree with the key when one is present.  ``None`` (default) requires the
    key, so an unlabeled file is refused instead of being guessed at.

    Fails closed (``ValueError``) on: a missing key, a non-positive or
    non-finite fps, mismatched frame counts or shapes, zero frames, any
    non-finite value in root_pos/root_rot/dof_pos, any root quaternion
    whose norm differs from 1 by more than ``UNIT_QUATERNION_TOLERANCE``, and
    a malformed optional ``joint_names`` (not unique strings, or a length
    different from dof_pos's column count).
    """
    with open(path, "rb") as f:
        d = pickle.load(f)
    if not isinstance(d, dict):
        raise ValueError(f"{path}: not a GMR motion pkl (top level is {type(d).__name__})")
    for key in ("fps", "root_pos", "root_rot", "dof_pos"):
        if key not in d:
            raise ValueError(f"{path}: missing key {key!r}; not a GMR motion pkl")
    try:
        fps_value = float(d["fps"])
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{path}: fps must be a number, got {d['fps']!r}") from exc
    if not np.isfinite(fps_value) or fps_value <= 0 or int(fps_value) < 1:
        raise ValueError(f"{path}: fps must be finite and >= 1, got {d['fps']!r}")
    root_pos = np.asarray(d["root_pos"], dtype=float)
    dof_pos = np.asarray(d["dof_pos"], dtype=float)
    if root_pos.ndim != 2 or root_pos.shape[1] != 3 or root_pos.shape[0] < 1:
        raise ValueError(f"{path}: root_pos must have shape (T>=1, 3), got {root_pos.shape}")
    if dof_pos.ndim != 2 or dof_pos.shape[0] != root_pos.shape[0]:
        raise ValueError(
            f"{path}: dof_pos must have shape (T={root_pos.shape[0]}, D), got {dof_pos.shape}")
    for name, values in (("root_pos", root_pos), ("dof_pos", dof_pos)):
        if not np.isfinite(values).all():
            raise ValueError(f"{path}: {name} contains non-finite values")
    # RootQuatOrderError (a ValueError) for a missing/mismatched order key.
    root_rot = root_rot_as(d, "wxyz", legacy_order=quat_order, source=path)
    if root_rot.shape[0] != root_pos.shape[0]:
        raise ValueError(
            f"{path}: root_rot has {root_rot.shape[0]} frames, root_pos has {root_pos.shape[0]}")
    joint_names = d.get("joint_names")
    if joint_names is not None:
        if (
            not isinstance(joint_names, (list, tuple))
            or not all(isinstance(name, str) and name for name in joint_names)
        ):
            raise ValueError(f"{path}: joint_names must be a list of non-empty strings")
        if len(joint_names) != dof_pos.shape[1]:
            raise ValueError(
                f"{path}: joint_names lists {len(joint_names)} joints but dof_pos has "
                f"{dof_pos.shape[1]} columns")
        if len(set(joint_names)) != len(joint_names):
            raise ValueError(f"{path}: joint_names contains duplicates")
        joint_names = list(joint_names)
    return {
        "fps": int(fps_value),
        "root_pos": root_pos,
        "root_rot_wxyz": root_rot,
        "dof_pos": dof_pos,
        # None for a LEGACY pkl that predates the key; run_fk_pass verifies a
        # present list against the MJCF and scoring flags an absent one.
        "joint_names": joint_names,
    }


def parse_urdf_limits(urdf_path: str | Path) -> dict[str, dict[str, float | None]]:
    """joint name -> {lower, upper, velocity, effort} (rad, rad/s, N*m; None if
    absent) from URDF ``<limit lower=... upper=... velocity=... effort=...>``.

    ``effort`` was added 2026-08-16 for the torque-feasibility metric; it is a
    new key on the returned dicts and does not alter ``lower``/``upper``/
    ``velocity``, so existing callers are unaffected.  All 29 G1 joints in
    ``eval/assets/g1_29dof_official.urdf`` carry an ``effort`` attribute, and
    the values are strongly asymmetric: 139 N*m knee, 88 N*m hip / waist_yaw,
    35 N*m ankle / waist roll+pitch, 25 N*m shoulder / elbow / wrist_roll and
    only 5 N*m wrist_pitch and wrist_yaw.
    """
    import xml.etree.ElementTree as ET

    limits: dict[str, dict[str, float | None]] = {}
    root = ET.parse(str(urdf_path)).getroot()
    for joint in root.iter("joint"):
        if joint.get("type") in ("fixed", None):
            continue
        lim = joint.find("limit")
        if lim is None:
            continue
        limits[joint.get("name")] = {
            k: (float(lim.get(k)) if lim.get(k) is not None else None)
            for k in ("lower", "upper", "velocity", "effort")
        }
    return limits


