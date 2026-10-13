"""Regression tests for the Xsens-to-Booster-T1 frame convention."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np


CONFIG = (
    Path(__file__).resolve().parents[1]
    / "retarget_configs/bvh_xsens_to_booster_t1_29dof.json"
)

LOWER_BODY_AND_TORSO = {
    "Waist",
    "Hip_Yaw_Left",
    "Shank_Left",
    "left_foot_link",
    "Hip_Yaw_Right",
    "Shank_Right",
    "right_foot_link",
    "Trunk",
}
ARMS = {"AL2", "AL4", "AL5", "AR2", "AR4", "AR5"}


def test_xsens_offsets_use_xsens_frames_in_both_task_tables() -> None:
    config = json.loads(CONFIG.read_text(encoding="utf-8"))
    identity = [1.0, 0.0, 0.0, 0.0]

    for table_name in ("ik_match_table1", "ik_match_table2"):
        table = config[table_name]
        for target in LOWER_BODY_AND_TORSO:
            np.testing.assert_allclose(table[target][4], identity, atol=1e-12)
        for target in ARMS:
            np.testing.assert_allclose(table[target][4], identity, atol=1e-12)


def test_all_configured_rotation_offsets_are_unit_quaternions() -> None:
    config = json.loads(CONFIG.read_text(encoding="utf-8"))
    for table_name in ("ik_match_table1", "ik_match_table2"):
        for target, entry in config[table_name].items():
            assert np.isclose(np.linalg.norm(entry[4]), 1.0), (table_name, target)
