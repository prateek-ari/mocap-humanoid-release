"""Tests for the live robot table and the live T1 Xsens MVN IK config."""

from __future__ import annotations

import json
import math
from pathlib import Path
import unittest

from mocap_g1.live_robots import (
    ARM_ALIGNMENTS,
    LIVE_ROBOTS,
    MVN_ARM_BODIES,
    T1_MOCAP_JOINT_RANGES,
    T1_MOCAP_JOINTS,
    T1_UPPER_BODY_JOINTS,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
GMR_ROOT = REPO_ROOT.parent / "mocap_g1_runtime" / "GMR"
T1_MOCAP_XML = GMR_ROOT / "assets/booster_t1_29dof/t1_mocap.xml"
LIVE_T1_CONFIG = REPO_ROOT / "retarget_configs/xsens_mvn_to_booster_t1_29dof.json"
RECORDED_T1_CONFIG = REPO_ROOT / "retarget_configs/bvh_xsens_to_booster_t1_29dof.json"

#: Segment names of GMR's ``xsens_mvn`` live source (MXTP02 converted).
MVN_SEGMENTS = {
    "Pelvis",
    "Chest",
    *(
        f"{side}_{segment}"
        for side in ("Left", "Right")
        for segment in ("UpperLeg", "LowerLeg", "Foot", "UpperArm", "Forearm", "Hand")
    ),
}


class LiveRobotTableTests(unittest.TestCase):
    def test_every_entry_has_a_consistent_joint_layout(self):
        self.assertEqual(set(LIVE_ROBOTS), {"unitree_g1", "booster_t1_29dof"})
        for key, robot in LIVE_ROBOTS.items():
            with self.subTest(robot=key):
                self.assertEqual(robot.key, key)
                self.assertEqual(robot.nq, 7 + len(robot.dof_joints))
                self.assertEqual(len(set(robot.dof_joints)), len(robot.dof_joints))
                self.assertTrue(set(robot.upper_body_joints) <= set(robot.dof_joints))
                self.assertIn(robot.default_arm_alignment, ARM_ALIGNMENTS)
                self.assertTrue(robot.feet)
                for body, depth in robot.feet:
                    self.assertIsInstance(body, str)
                    self.assertTrue(math.isfinite(depth) and depth > 0)

    def test_g1_defaults(self):
        g1 = LIVE_ROBOTS["unitree_g1"]
        self.assertEqual(g1.label, "G1")
        self.assertEqual(g1.nq, 36)
        self.assertIsNone(g1.ik_config)
        self.assertEqual(g1.default_arm_alignment, "config")
        self.assertEqual(g1.arm_abduction_deg, 10.0)

    def test_t1_defaults(self):
        t1 = LIVE_ROBOTS["booster_t1_29dof"]
        self.assertEqual(t1.label, "T1")
        self.assertEqual(len(t1.dof_joints), 27)
        self.assertEqual(t1.nq, 34)
        self.assertEqual(t1.dof_joints, T1_MOCAP_JOINTS)
        self.assertEqual(t1.upper_body_joints, T1_UPPER_BODY_JOINTS)
        self.assertIsNotNone(t1.ik_config)
        self.assertTrue(t1.ik_config.is_file(), t1.ik_config)
        self.assertEqual(t1.ik_config, LIVE_T1_CONFIG)
        self.assertEqual(t1.default_arm_alignment, "position-only")
        self.assertEqual(t1.arm_abduction_deg, 22.0)

    def test_t1_ranges_are_ordered_and_upper_body_excludes_legs(self):
        self.assertEqual(len(T1_MOCAP_JOINT_RANGES), 27)
        for name, lower, upper in T1_MOCAP_JOINT_RANGES:
            with self.subTest(joint=name):
                self.assertLess(lower, upper)
        self.assertEqual(len(T1_UPPER_BODY_JOINTS), 15)  # waist + 2 x 7 arm hinges
        self.assertIn("Waist", T1_UPPER_BODY_JOINTS)
        for name in T1_UPPER_BODY_JOINTS:
            self.assertNotRegex(name, "Hip|Knee|Ankle")

    def test_arm_bodies_are_mvn_segments(self):
        self.assertTrue(set(MVN_ARM_BODIES) <= MVN_SEGMENTS)


@unittest.skipUnless(T1_MOCAP_XML.is_file(), "GMR T1 mocap MJCF is unavailable")
class T1MjcfDriftTests(unittest.TestCase):
    def test_hinge_names_order_and_ranges_match_the_gmr_model(self):
        import mujoco

        model = mujoco.MjModel.from_xml_path(str(T1_MOCAP_XML))
        hinges = []
        for joint in range(model.njnt):
            if model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_HINGE:
                continue
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
            lower, upper = (float(value) for value in model.jnt_range[joint])
            hinges.append((name, lower, upper))
        self.assertEqual(model.nq, LIVE_ROBOTS["booster_t1_29dof"].nq)
        self.assertEqual([name for name, *_ in hinges], list(T1_MOCAP_JOINTS))
        for (name, lower, upper), (_expected, exp_lower, exp_upper) in zip(
            hinges, T1_MOCAP_JOINT_RANGES
        ):
            with self.subTest(joint=name):
                self.assertAlmostEqual(lower, exp_lower, places=6)
                self.assertAlmostEqual(upper, exp_upper, places=6)


class LiveT1IkConfigTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = json.loads(LIVE_T1_CONFIG.read_text(encoding="utf-8"))
        cls.recorded = json.loads(RECORDED_T1_CONFIG.read_text(encoding="utf-8"))

    def tables(self):
        return (self.config["ik_match_table1"], self.config["ik_match_table2"])

    def test_uses_only_mvn_segment_names(self):
        self.assertEqual(self.config["human_root_name"], "Pelvis")
        self.assertTrue(self.config["use_ik_match_table1"])
        self.assertTrue(self.config["use_ik_match_table2"])
        for table in self.tables():
            for frame, entry in table.items():
                with self.subTest(frame=frame):
                    self.assertIn(entry[0], MVN_SEGMENTS)
        self.assertTrue(set(self.config["human_scale_table"]) <= MVN_SEGMENTS)

    def test_every_rotation_offset_is_identity(self):
        for table in self.tables():
            for frame, entry in table.items():
                with self.subTest(frame=frame):
                    _body, _pos_w, _rot_w, position_offset, rotation = entry
                    self.assertEqual(len(position_offset), 3)
                    self.assertEqual(len(rotation), 4)
                    self.assertAlmostEqual(math.fsum(v * v for v in rotation), 1.0)
                    self.assertEqual(rotation, [1.0, 0.0, 0.0, 0.0])

    def test_scale_table_covers_every_matched_body(self):
        used = {entry[0] for table in self.tables() for entry in table.values()}
        self.assertTrue(used <= set(self.config["human_scale_table"]), used)
        self.assertIn("Pelvis", self.config["human_scale_table"])

    def test_robot_frames_match_the_recorded_t1_config(self):
        for name in ("ik_match_table1", "ik_match_table2"):
            with self.subTest(table=name):
                self.assertEqual(set(self.config[name]), set(self.recorded[name]))
        self.assertEqual(self.config["robot_root_name"], self.recorded["robot_root_name"])


if __name__ == "__main__":
    unittest.main()
