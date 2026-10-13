from __future__ import annotations

from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from mocap_g1.assets import official_urdf_path
from mocap_g1.controller_screen import (
    ControllerScreenConfig,
    ControllerScreenError,
    run_upper_body_screen,
    scan_reference_self_collisions,
)
from mocap_g1.schema import G1_29DOF_JOINTS
from conftest import G1_MJCF as MODEL, requires_g1_mjcf


REPO_ROOT = Path(__file__).resolve().parents[1]


def reference(frames: int = 31, fps: float = 60.0) -> np.ndarray:
    values = np.zeros((frames, 36), dtype=np.float64)
    values[:, 3] = 1.0
    time = np.arange(frames) / fps
    values[:, 7 + 15] = 0.05 * np.sin(2.0 * np.pi * 0.5 * time)
    values[:, 7 + 22] = 0.05 * np.sin(2.0 * np.pi * 0.5 * time)
    return values


class ControllerScreenTests(unittest.TestCase):
    def test_rejects_invalid_reference_and_config(self):
        with self.assertRaisesRegex(ControllerScreenError, "shape"):
            run_upper_body_screen(
                np.zeros((3, 35)), 60.0, model_path=MODEL
            )
        with self.assertRaisesRegex(ControllerScreenError, "torque_limit_fraction"):
            ControllerScreenConfig(torque_limit_fraction=1.1).validate()

    @requires_g1_mjcf
    def test_small_reference_runs_through_bounded_pd(self):
        result = run_upper_body_screen(
            reference(),
            60.0,
            model_path=MODEL,
            config=ControllerScreenConfig(
                timestep_s=0.002,
                settle_seconds=0.02,
                tracking_p95_limit_rad=0.2,
                tracking_max_limit_rad=0.4,
                saturation_fraction_limit=0.2,
            ),
            source_metadata={"offsets_key": "zero-preview"},
        )
        self.assertEqual(result["controller_screen"]["status"], "PASS")
        self.assertEqual(result["hardware_candidate"]["status"], "BLOCKED")
        self.assertEqual(result["model"]["base"], "pelvis_freejoint_removed")
        self.assertEqual(result["events"]["joint_position_violation_steps"], 0)

    @requires_g1_mjcf
    def test_neutral_reference_collision_scan_passes(self):
        result = scan_reference_self_collisions(
            reference(frames=6),
            60.0,
            model_path=MODEL,
            window_seconds=0.05,
        )
        self.assertEqual(result["summary"]["status"], "PASS")
        self.assertEqual(result["summary"]["self_collision_frames"], 0)
        self.assertEqual(result["summary"]["longest_collision_free_run"]["end_frame_exclusive"], 6)

    @requires_g1_mjcf
    def test_reference_collision_scan_fails_when_the_arm_enters_the_torso(self):
        # On this MJCF left_shoulder_roll <= -0.3 rad puts the upper arm
        # inside torso_link (see _collision_free_model), well inside the URDF.
        values = reference(frames=6)
        values[2:4, 7 + G1_29DOF_JOINTS.index("left_shoulder_roll_joint")] = -0.5
        result = scan_reference_self_collisions(
            values, 60.0, model_path=MODEL, window_seconds=0.05
        )
        summary = result["summary"]
        self.assertEqual(summary["status"], "FAIL")
        self.assertEqual(summary["self_collision_frames"], 2)
        self.assertEqual(summary["collision_free_frames"], 4)
        run = summary["longest_collision_free_run"]
        self.assertEqual((run["start_frame"], run["end_frame_exclusive"]), (0, 2))
        pairs = {(pair["body_a"], pair["body_b"]) for pair in result["collision_pairs"]}
        self.assertIn(("left_shoulder_roll_link", "torso_link"), pairs)
        self.assertTrue(all(pair["frames"] == 2 for pair in result["collision_pairs"]))
        self.assertTrue(all(
            pair["minimum_contact_distance_m"] < 0.0 for pair in result["collision_pairs"]
        ))
        # 3-frame windows: frame 2 falls in the first, frame 3 in the second.
        self.assertEqual(
            [(w["status"], w["self_collision_frames"]) for w in result["windows"]],
            [("FAIL", 1), ("FAIL", 1)],
        )

    @requires_g1_mjcf
    def test_upper_body_screen_fails_on_self_collision(self):
        values = reference()
        values[:, 7 + G1_29DOF_JOINTS.index("left_shoulder_roll_joint")] = -0.5
        result = run_upper_body_screen(
            values, 60.0, model_path=MODEL,
            config=ControllerScreenConfig(settle_seconds=0.1, saturation_fraction_limit=0.2),
        )
        self.assertEqual(result["controller_screen"]["status"], "FAIL")
        self.assertIn("self_collision", result["controller_screen"]["reasons"])
        self.assertGreater(result["events"]["self_collision_steps"], 0)
        self.assertEqual(result["hardware_candidate"]["status"], "BLOCKED")
        self.assertIn("self_collision", result["hardware_candidate"]["reasons"])

    @requires_g1_mjcf
    def test_upper_body_screen_fails_on_an_untrackable_step(self):
        # A 1 rad elbow step between two frames cannot be tracked by the
        # bounded PD; collision is disabled so tracking is the only failure.
        values = reference()
        values[15:, 7 + G1_29DOF_JOINTS.index("left_elbow_joint")] = 1.0
        with tempfile.TemporaryDirectory() as directory:
            model = _collision_free_model(Path(directory))
            config = ControllerScreenConfig(settle_seconds=0.1, saturation_fraction_limit=0.2)
            result = run_upper_body_screen(values, 60.0, model_path=model, config=config)
        self.assertEqual(result["controller_screen"]["status"], "FAIL")
        self.assertEqual(result["controller_screen"]["reasons"], ["tracking_max_exceeded"])
        self.assertGreater(
            result["tracking"]["pooled_max_abs_rad"], config.tracking_max_limit_rad
        )
        self.assertEqual(result["hardware_candidate"]["status"], "BLOCKED")



def _collision_free_model(directory: Path) -> Path:
    """The real G1 MJCF with every geom's collision disabled.

    On g1_mocap_29dof.xml the left_shoulder_roll_link geometry intersects
    torso_link for ANY roll <= -0.3 rad, whatever the other arm joints do, so
    a self-collision would mask the joint-limit behaviour under test.
    """
    import mujoco

    spec = mujoco.MjSpec.from_file(str(MODEL))
    spec.meshdir = str((MODEL.parent / spec.meshdir).resolve())
    for geom in spec.geoms:
        geom.contype = 0
        geom.conaffinity = 0
    path = directory / "g1_no_collision.xml"
    path.write_text(spec.to_xml(), encoding="utf-8")
    return path


def _urdf_with_lower(directory: Path, joint: str, lower: float) -> Path:
    tree = ET.parse(official_urdf_path())
    for element in tree.getroot().iter("joint"):
        if element.get("name") == joint:
            element.find("limit").set("lower", repr(lower))
    path = directory / "narrowed.urdf"
    tree.write(path)
    return path


def _ramp(joint: str, start: float, end: float, frames: int = 61, fps: float = 60.0
          ) -> np.ndarray:
    values = np.zeros((frames, 36), dtype=np.float64)
    values[:, 3] = 1.0
    values[:, 7 + G1_29DOF_JOINTS.index(joint)] = np.linspace(start, end, frames)
    return values


# saturation_fraction_limit is relaxed because ANY moving clip saturates the
# wrist-roll actuators (~5-9 % of samples at the 0.5 torque fraction) on this
# MJCF -- identical before and after the T1-7 change, and not what these
# tests are about.  Tracking and joint-limit gates keep their defaults.
_FAST = dict(settle_seconds=0.1, saturation_fraction_limit=0.2)


@requires_g1_mjcf
class JointLimitSourceTests(unittest.TestCase):
    """T1-7: the mechanical URDF, not the MJCF's IK ranges, bounds stage 5."""

    def setUp(self):
        self._directory = tempfile.TemporaryDirectory()
        self.root = Path(self._directory.name)

    def tearDown(self):
        self._directory.cleanup()

    def test_mjcf_range_is_narrower_than_the_urdf_for_shoulder_roll(self):
        import mujoco

        model = mujoco.MjModel.from_xml_path(str(MODEL))
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "left_shoulder_roll_joint")
        self.assertGreater(model.jnt_range[joint, 0], -1.0)  # MJCF stops at -0.6

    def test_shoulder_roll_inside_urdf_but_outside_mjcf_tracks(self):
        model = _collision_free_model(self.root)
        result = run_upper_body_screen(
            _ramp("left_shoulder_roll_joint", -0.2, -1.0), 60.0, model_path=model,
            config=ControllerScreenConfig(**_FAST),
        )
        self.assertEqual(result["controller_screen"]["status"], "PASS",
                         result["controller_screen"]["reasons"])
        self.assertLess(result["tracking"]["pooled_max_abs_rad"], 0.1)
        self.assertEqual(result["schema_version"], 2)
        self.assertEqual(result["joint_limits"]["source"], "urdf")
        self.assertEqual(result["joint_limits"]["path"], str(official_urdf_path().resolve()))
        self.assertEqual(len(result["joint_limits"]["sha256"]), 64)

    def test_mjcf_like_limit_source_reproduces_the_old_tracking_failure(self):
        model = _collision_free_model(self.root)
        narrowed = _urdf_with_lower(self.root, "left_shoulder_roll_joint", -0.6)
        result = run_upper_body_screen(
            _ramp("left_shoulder_roll_joint", -0.2, -1.0), 60.0, model_path=model,
            config=ControllerScreenConfig(limits_urdf_path=str(narrowed), **_FAST),
        )
        # With the old -0.6 bound the clip is (correctly) a limit violation of
        # the REFERENCE, no longer a misleading tracking_max_exceeded.
        reasons = result["controller_screen"]["reasons"]
        self.assertIn("joint_position_limit_violation", reasons)
        self.assertFalse([r for r in reasons if r.startswith("tracking_")], reasons)
        self.assertEqual(result["joint_limits"]["path"], str(narrowed.resolve()))

    def test_reference_outside_urdf_is_a_limit_violation_not_tracking(self):
        model = _collision_free_model(self.root)
        result = run_upper_body_screen(
            _ramp("left_shoulder_roll_joint", -1.2, -1.8), 60.0, model_path=model,
            config=ControllerScreenConfig(**_FAST),
        )
        self.assertEqual(result["controller_screen"]["status"], "FAIL")
        reasons = result["controller_screen"]["reasons"]
        self.assertIn("joint_position_limit_violation", reasons)
        self.assertFalse([r for r in reasons if r.startswith("tracking_")], reasons)
        violations = result["events"]["reference_joint_position_violations"]
        self.assertEqual([v["joint"] for v in violations], ["left_shoulder_roll_joint"])
        self.assertGreater(violations[0]["frames"], 0)

    def test_real_model_reference_outside_urdf_on_wrist_roll(self):
        values = np.zeros((31, 36), dtype=np.float64)
        values[:, 3] = 1.0
        values[:, 7 + G1_29DOF_JOINTS.index("left_wrist_roll_joint")] = -2.1  # URDF -1.972
        result = run_upper_body_screen(values, 60.0, model_path=MODEL,
                                       config=ControllerScreenConfig(**_FAST))
        self.assertEqual(result["controller_screen"]["reasons"],
                         ["joint_position_limit_violation"])

    def test_real_model_waist_yaw_inside_urdf_outside_mjcf_passes(self):
        result = run_upper_body_screen(
            _ramp("waist_yaw_joint", 1.2, 2.0), 60.0, model_path=MODEL,
            config=ControllerScreenConfig(**_FAST),
        )
        self.assertEqual(result["controller_screen"]["status"], "PASS",
                         result["controller_screen"]["reasons"])

    def test_missing_limits_urdf_is_refused(self):
        with self.assertRaisesRegex(ControllerScreenError, "joint position limits"):
            run_upper_body_screen(
                _ramp("waist_yaw_joint", 0.0, 0.1), 60.0, model_path=MODEL,
                config=ControllerScreenConfig(limits_urdf_path=str(self.root / "nope.urdf")),
            )


if __name__ == "__main__":
    unittest.main()
