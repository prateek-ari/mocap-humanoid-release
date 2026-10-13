"""Tests for ``mocap_g1.live_arm_alignment`` against a duck-typed fake GMR."""

from __future__ import annotations

import math
import types
import unittest

import numpy as np

from mocap_g1.live_arm_alignment import (
    CALIBRATION_SOLVES,
    ArmAlignmentError,
    LiveArmAlignment,
    floor_offset_m,
    offset_wxyz,
)

ARMS = ("Left_UpperArm", "Left_Hand")


def quat_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def axis_angle(axis, degrees):
    axis = np.asarray(axis, dtype=np.float64) / np.linalg.norm(axis)
    half = math.radians(degrees) / 2.0
    return np.array([math.cos(half), *(math.sin(half) * axis)])


def assert_same_rotation(test, actual, expected):
    actual, expected = np.asarray(actual), np.asarray(expected)
    test.assertAlmostEqual(abs(float(np.dot(actual, expected))), 1.0, places=9)


class FakeRotation:
    """Stands in for scipy's Rotation: ``from_quat(q, scalar_first=True)``."""

    def __init__(self, wxyz):
        self.wxyz = np.asarray(wxyz, dtype=np.float64)

    @classmethod
    def from_quat(cls, quat, scalar_first=False):
        assert scalar_first is True
        return cls(quat)


class FakeTask:
    def __init__(self, frame_name, cost):
        self.frame_name = frame_name
        self.orientation_cost = np.asarray(cost, dtype=np.float64)
        self.cost_history = []

    def set_orientation_cost(self, cost):
        self.orientation_cost = np.asarray(cost, dtype=np.float64)
        self.cost_history.append(self.orientation_cost.copy())


class FakeModel:
    def __init__(self, bodies):
        self.ids = {name: index for index, name in enumerate(bodies)}

    def body(self, name):
        return types.SimpleNamespace(id=self.ids[name])


class FakeRetargeter:
    BODIES = ("world", "pelvis_link", "left_upper_arm_link", "left_hand_link", "foot_l", "foot_r")

    def __init__(self, *, fail_on_call=None):
        self.human_body_to_task1 = {
            "Pelvis": FakeTask("pelvis_link", 10.0),
            "Left_UpperArm": FakeTask("left_upper_arm_link", 10.0),
            "Left_Hand": FakeTask("left_hand_link", 5.0),
        }
        self.human_body_to_task2 = {
            "Left_UpperArm": FakeTask("left_upper_arm_link", 3.0),
        }
        self.rot_offsets1 = {name: FakeRotation((1, 0, 0, 0)) for name in self.human_body_to_task1}
        self.rot_offsets2 = {name: FakeRotation((1, 0, 0, 0)) for name in self.human_body_to_task2}
        self.model = FakeModel(self.BODIES)
        xquat = np.tile([1.0, 0.0, 0.0, 0.0], (len(self.BODIES), 1))
        self.configuration = types.SimpleNamespace(
            data=types.SimpleNamespace(xquat=xquat, xpos=np.zeros((len(self.BODIES), 3)))
        )
        self.fail_on_call = fail_on_call
        self.calls = []

    def set_robot_quat(self, body, wxyz):
        self.configuration.data.xquat[self.model.ids[body]] = wxyz

    def all_tasks(self):
        return [*self.human_body_to_task1.values(), *self.human_body_to_task2.values()]

    def retarget(self, frame, offset_to_ground=True):
        costs = [float(np.asarray(task.orientation_cost).max()) for task in self.all_tasks()]
        self.calls.append({"frame": frame, "offset_to_ground": offset_to_ground, "costs": costs})
        if self.fail_on_call is not None and len(self.calls) == self.fail_on_call:
            raise RuntimeError("solver diverged")
        return np.zeros(10)


def neutral_frame(left_upper=(1, 0, 0, 0), left_hand=(1, 0, 0, 0)):
    return {
        "Pelvis": (np.zeros(3), np.array([1.0, 0, 0, 0])),
        "Left_UpperArm": (np.array([0.0, 0.2, 1.4]), np.asarray(left_upper, dtype=float)),
        "Left_Hand": (np.array([0.0, 0.2, 0.8]), np.asarray(left_hand, dtype=float)),
    }


class OffsetMathTests(unittest.TestCase):
    def test_identity_cases(self):
        q = axis_angle((1, 2, 3), 70)
        np.testing.assert_allclose(offset_wxyz((1, 0, 0, 0), (1, 0, 0, 0)), [1, 0, 0, 0])
        np.testing.assert_allclose(offset_wxyz(q, q), [1, 0, 0, 0], atol=1e-12)
        np.testing.assert_allclose(offset_wxyz((1, 0, 0, 0), q), q, atol=1e-12)

    def test_offset_is_inverse_human_times_robot(self):
        human = axis_angle((0, 1, 0), 40)
        robot = axis_angle((1, 0, 1), -125)
        offset = offset_wxyz(human, robot)
        self.assertAlmostEqual(float(np.linalg.norm(offset)), 1.0)
        assert_same_rotation(self, quat_multiply(human, offset), robot)

    def test_result_has_non_negative_w_and_inputs_are_normalised(self):
        robot = axis_angle((0, 0, 1), 300)  # w < 0 as constructed
        self.assertLess(robot[0], 0)
        offset = offset_wxyz((2.0, 0, 0, 0), 3.0 * robot)
        self.assertGreaterEqual(offset[0], 0.0)
        np.testing.assert_allclose(offset, -robot, atol=1e-12)

    def test_unusable_quaternions_raise(self):
        for bad in ((0, 0, 0, 0), (1, 0, 0), (math.nan, 0, 0, 1)):
            with self.subTest(bad=bad):
                with self.assertRaises(ArmAlignmentError):
                    offset_wxyz(bad, (1, 0, 0, 0))


class LiveArmAlignmentTests(unittest.TestCase):
    def test_requires_arm_tasks(self):
        with self.assertRaises(ArmAlignmentError):
            LiveArmAlignment(FakeRetargeter(), ("Right_Hand",))

    def test_use_position_only_zeroes_only_arm_costs(self):
        retargeter = FakeRetargeter()
        alignment = LiveArmAlignment(retargeter, ARMS)
        self.assertEqual(alignment.mode, "config")
        alignment.use_position_only()
        self.assertEqual(alignment.mode, "position-only")
        self.assertEqual(float(retargeter.human_body_to_task1["Left_UpperArm"].orientation_cost), 0.0)
        self.assertEqual(float(retargeter.human_body_to_task1["Left_Hand"].orientation_cost), 0.0)
        self.assertEqual(float(retargeter.human_body_to_task2["Left_UpperArm"].orientation_cost), 0.0)
        self.assertEqual(float(retargeter.human_body_to_task1["Pelvis"].orientation_cost), 10.0)

    def test_calibrate_writes_offsets_and_restores_costs(self):
        retargeter = FakeRetargeter()
        alignment = LiveArmAlignment(retargeter, ARMS)
        alignment.use_position_only()
        human_upper = axis_angle((0, 1, 0), 30)
        robot_upper = axis_angle((1, 0, 0), 90)
        robot_hand = axis_angle((0, 0, 1), -45)
        retargeter.set_robot_quat("left_upper_arm_link", robot_upper)
        retargeter.set_robot_quat("left_hand_link", robot_hand)
        frame = neutral_frame(left_upper=human_upper)

        derived = alignment.calibrate(frame)

        self.assertEqual(alignment.mode, "calibrated")
        self.assertEqual(alignment.calibrations, 1)
        self.assertEqual(set(derived), set(ARMS))
        # Every neutral solve is position-only and ungrounded.
        self.assertEqual(len(retargeter.calls), CALIBRATION_SOLVES)
        for call in retargeter.calls:
            self.assertFalse(call["offset_to_ground"])
            self.assertEqual(call["costs"], [10.0, 0.0, 0.0, 0.0])
            self.assertEqual(set(call["frame"]), set(frame))
        # Costs come back as configured, not as the zeroed position-only ones.
        self.assertEqual(float(retargeter.human_body_to_task1["Left_UpperArm"].orientation_cost), 10.0)
        self.assertEqual(float(retargeter.human_body_to_task1["Left_Hand"].orientation_cost), 5.0)
        self.assertEqual(float(retargeter.human_body_to_task2["Left_UpperArm"].orientation_cost), 3.0)
        # GMR applies human * offset; that must reproduce the robot frame.
        for table in (retargeter.rot_offsets1, retargeter.rot_offsets2):
            assert_same_rotation(
                self, quat_multiply(human_upper, table["Left_UpperArm"].wxyz), robot_upper
            )
            self.assertIsInstance(table["Left_UpperArm"], FakeRotation)
        assert_same_rotation(self, retargeter.rot_offsets1["Left_Hand"].wxyz, robot_hand)
        np.testing.assert_allclose(derived["Left_Hand"], retargeter.rot_offsets1["Left_Hand"].wxyz)
        self.assertNotIn("Left_Hand", retargeter.rot_offsets2)
        np.testing.assert_array_equal(retargeter.rot_offsets1["Pelvis"].wxyz, [1, 0, 0, 0])

    def test_missing_arm_segment_raises_without_touching_state(self):
        retargeter = FakeRetargeter()
        alignment = LiveArmAlignment(retargeter, ARMS)
        frame = neutral_frame()
        del frame["Left_Hand"]
        with self.assertRaisesRegex(ArmAlignmentError, "Left_Hand"):
            alignment.calibrate(frame)
        self.assertEqual(alignment.mode, "config")
        self.assertEqual(retargeter.calls, [])
        self.assertEqual(float(retargeter.human_body_to_task1["Left_Hand"].orientation_cost), 5.0)

    def test_retarget_failure_restores_offsets_and_config_mode(self):
        retargeter = FakeRetargeter(fail_on_call=3)
        alignment = LiveArmAlignment(retargeter, ARMS)
        before = {name: dict(getattr(retargeter, name)) for name in ("rot_offsets1", "rot_offsets2")}
        with self.assertRaisesRegex(RuntimeError, "solver diverged"):
            alignment.calibrate(neutral_frame())
        self.assertEqual(alignment.mode, "config")
        self.assertEqual(alignment.calibrations, 0)
        self.assertEqual(retargeter.rot_offsets1, before["rot_offsets1"])
        self.assertEqual(retargeter.rot_offsets2, before["rot_offsets2"])
        self.assertEqual(float(retargeter.human_body_to_task1["Left_UpperArm"].orientation_cost), 10.0)

    def test_failure_after_partial_writes_restores_offsets_and_position_only_mode(self):
        retargeter = FakeRetargeter()
        alignment = LiveArmAlignment(retargeter, ARMS)
        alignment.use_position_only()
        retargeter.set_robot_quat("left_upper_arm_link", axis_angle((1, 0, 0), 90))
        retargeter.set_robot_quat("left_hand_link", (0.0, 0.0, 0.0, 0.0))  # unusable
        before = dict(retargeter.rot_offsets1)
        with self.assertRaises(ArmAlignmentError):
            alignment.calibrate(neutral_frame())
        self.assertEqual(alignment.mode, "position-only")
        self.assertEqual(retargeter.rot_offsets1, before)
        for task in retargeter.all_tasks()[1:]:
            self.assertEqual(float(task.orientation_cost), 0.0)


class FloorOffsetTests(unittest.TestCase):
    def test_lowest_sole_is_moved_to_zero(self):
        retargeter = FakeRetargeter()
        ids = retargeter.model.ids
        retargeter.configuration.data.xpos[ids["foot_l"]] = (0.0, 0.1, 0.10)
        retargeter.configuration.data.xpos[ids["foot_r"]] = (0.0, -0.1, 0.07)
        offset = floor_offset_m(retargeter, (("foot_l", 0.04), ("foot_r", 0.04)))
        self.assertAlmostEqual(offset, -0.03)
        retargeter.configuration.data.xpos[ids["foot_r"]] = (0.0, -0.1, -0.02)
        self.assertAlmostEqual(
            floor_offset_m(retargeter, (("foot_l", 0.04), ("foot_r", 0.04))), 0.06
        )

    def test_no_feet_or_non_finite_positions_raise(self):
        retargeter = FakeRetargeter()
        with self.assertRaises(ArmAlignmentError):
            floor_offset_m(retargeter, ())
        retargeter.configuration.data.xpos[retargeter.model.ids["foot_l"]] = (0, 0, math.nan)
        with self.assertRaises(ArmAlignmentError):
            floor_offset_m(retargeter, (("foot_l", 0.04),))


if __name__ == "__main__":
    unittest.main()
