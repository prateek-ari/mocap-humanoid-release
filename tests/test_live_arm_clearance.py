"""Tests for ``mocap_g1.live_arm_clearance`` (arm abduction about the shoulder)."""

from __future__ import annotations

import math
import sys
import types
import unittest
from unittest import mock

import numpy as np

from mocap_g1.live_arm_clearance import ARM_SEGMENTS, ArmAbduction, abduct_arms

IDENTITY = np.array([1.0, 0.0, 0.0, 0.0])
SHOULDER_Z = 1.4
ARM_LENGTH = 0.6


def axis_angle(axis, degrees):
    axis = np.asarray(axis, dtype=np.float64) / np.linalg.norm(axis)
    half = math.radians(degrees) / 2.0
    return np.array([math.cos(half), *(math.sin(half) * axis)])


def hanging_arms(chest=IDENTITY):
    """Both arms straight down from shoulders at y = +/-0.2 (hand 0.6 m below)."""
    scaled = {
        "Pelvis": (np.array([0.0, 0.0, 1.0]), IDENTITY.copy()),
        "Chest": (np.array([0.0, 0.0, 1.3]), np.asarray(chest, dtype=np.float64)),
    }
    for side, y in (("Left", 0.2), ("Right", -0.2)):
        for segment, drop in zip(ARM_SEGMENTS, (0.0, 0.3, ARM_LENGTH)):
            scaled[f"{side}_{segment}"] = (
                np.array([0.0, y, SHOULDER_Z - drop]),
                IDENTITY.copy(),
            )
    return scaled


def assert_same_rotation(test, actual, expected):
    test.assertAlmostEqual(abs(float(np.dot(actual, expected))), 1.0, places=9)


class AbductArmsTests(unittest.TestCase):
    def test_positive_degrees_move_hands_outward_about_the_shoulder(self):
        degrees = 10.0
        theta = math.radians(degrees)
        scaled = hanging_arms()
        moved = abduct_arms(scaled, degrees)
        self.assertEqual(
            set(moved), {f"{side}_{seg}" for side in ("Left", "Right") for seg in ARM_SEGMENTS}
        )
        for side, sign in (("Left", 1.0), ("Right", -1.0)):
            with self.subTest(side=side):
                shoulder = scaled[f"{side}_UpperArm"][0]
                np.testing.assert_allclose(moved[f"{side}_UpperArm"][0], shoulder, atol=1e-12)
                hand = moved[f"{side}_Hand"][0]
                np.testing.assert_allclose(
                    hand - shoulder,
                    [0.0, sign * ARM_LENGTH * math.sin(theta), -ARM_LENGTH * math.cos(theta)],
                    atol=1e-12,
                )
                expected = axis_angle((1, 0, 0), sign * degrees)
                for segment in ARM_SEGMENTS:
                    assert_same_rotation(self, moved[f"{side}_{segment}"][1], expected)
        # The input dict is left unchanged.
        np.testing.assert_array_equal(scaled["Left_Hand"][0], [0.0, 0.2, SHOULDER_Z - ARM_LENGTH])

    def test_zero_degrees_is_identity(self):
        scaled = hanging_arms(chest=axis_angle((0, 0, 1), 35))
        scaled["Left_Hand"] = (np.array([0.3, 0.4, 0.9]), axis_angle((1, 1, 0), 20))
        moved = abduct_arms(scaled, 0.0)
        for name, (position, orientation) in moved.items():
            with self.subTest(segment=name):
                np.testing.assert_allclose(position, scaled[name][0], atol=1e-12)
                np.testing.assert_allclose(orientation, scaled[name][1], atol=1e-12)

    def test_axis_follows_the_chest_forward_direction(self):
        # Chest yawed 90 deg: forward is +y, so the arms swing in the x-z plane.
        degrees = 20.0
        theta = math.radians(degrees)
        moved = abduct_arms(hanging_arms(chest=axis_angle((0, 0, 1), 90)), degrees)
        left = moved["Left_Hand"][0] - moved["Left_UpperArm"][0]
        np.testing.assert_allclose(
            left, [-ARM_LENGTH * math.sin(theta), 0.0, -ARM_LENGTH * math.cos(theta)], atol=1e-12
        )
        assert_same_rotation(self, moved["Left_Forearm"][1], axis_angle((0, 1, 0), degrees))

    def test_orientations_compose_with_the_segment_rotation(self):
        scaled = hanging_arms()
        segment = axis_angle((0, 0, 1), 40)
        scaled["Left_Forearm"] = (scaled["Left_Forearm"][0], segment)
        moved = abduct_arms(scaled, 15.0)
        rotation = axis_angle((1, 0, 0), 15.0)
        w1, x1, y1, z1 = rotation
        w2, x2, y2, z2 = segment
        expected = np.array(
            [
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            ]
        )
        np.testing.assert_allclose(moved["Left_Forearm"][1], expected, atol=1e-12)


class FakeTask:
    def __init__(self):
        self.targets = []

    def set_target(self, target):
        self.targets.append(target)


class FakeRetargeter:
    def __init__(self):
        self.update_calls = []
        self.scaled_human_data = None
        self.human_body_to_task1 = {
            "Pelvis": FakeTask(),
            "Left_UpperArm": FakeTask(),
            "Left_Hand": FakeTask(),
            "Right_Hand": FakeTask(),
        }
        self.human_body_to_task2 = {"Left_Hand": FakeTask(), "Right_Forearm": FakeTask()}

    def update_targets(self, human_data, offset_to_ground=False):
        self.update_calls.append((human_data, offset_to_ground))
        self.scaled_human_data = hanging_arms()


def fake_mink():
    module = types.ModuleType("mink")

    class SO3:
        def __init__(self, wxyz):
            self.wxyz = np.asarray(wxyz)

    class SE3:
        def __init__(self, rotation, translation):
            self.rotation = rotation
            self.translation = np.asarray(translation)

        @classmethod
        def from_rotation_and_translation(cls, rotation, translation):
            return cls(rotation, translation)

    module.SO3 = SO3
    module.SE3 = SE3
    return module


class ArmAbductionTests(unittest.TestCase):
    def test_wraps_update_targets_and_sets_abducted_targets(self):
        retargeter = FakeRetargeter()
        original = retargeter.update_targets
        abduction = ArmAbduction(retargeter, 10)
        self.assertEqual(abduction.degrees, 10.0)
        self.assertNotEqual(retargeter.update_targets, original)
        with mock.patch.dict(sys.modules, {"mink": fake_mink()}):
            retargeter.update_targets({"frame": 1}, True)
        self.assertEqual(retargeter.update_calls, [({"frame": 1}, True)])

        expected = abduct_arms(hanging_arms(), 10.0)
        scaled = retargeter.scaled_human_data
        for name, (position, orientation) in expected.items():
            np.testing.assert_allclose(scaled[name][0], position)
            np.testing.assert_allclose(scaled[name][1], orientation)
        np.testing.assert_array_equal(scaled["Pelvis"][0], [0.0, 0.0, 1.0])

        task1, task2 = retargeter.human_body_to_task1, retargeter.human_body_to_task2
        self.assertEqual(task1["Pelvis"].targets, [])
        for task, name in (
            (task1["Left_UpperArm"], "Left_UpperArm"),
            (task1["Left_Hand"], "Left_Hand"),
            (task1["Right_Hand"], "Right_Hand"),
            (task2["Left_Hand"], "Left_Hand"),
            (task2["Right_Forearm"], "Right_Forearm"),
        ):
            with self.subTest(task=name):
                self.assertEqual(len(task.targets), 1)
                target = task.targets[0]
                np.testing.assert_allclose(target.translation, expected[name][0])
                np.testing.assert_allclose(target.rotation.wxyz, expected[name][1])
        # The left hand moved outward (+y), the right hand outward (-y).
        self.assertGreater(task1["Left_Hand"].targets[0].translation[1], 0.2)
        self.assertLess(task1["Right_Hand"].targets[0].translation[1], -0.2)


if __name__ == "__main__":
    unittest.main()
