"""Tests for ``LiveRangePreviewController`` and the N calibration key."""

from __future__ import annotations

import math
import unittest

import numpy as np

from mocap_g1.live_robots import T1_MOCAP_JOINT_RANGES, T1_UPPER_BODY_JOINTS
from mocap_g1.live_safety import (
    CLAMP_REPORT_TOLERANCE_RAD,
    LiveOperatorControls,
    LiveRangePreviewController,
)
from mocap_g1.safety import SafetyConfig, SafetyError

RANGES = (
    ("leg_a", -1.0, 1.0),
    ("arm_a", -1.0, 1.0),
    ("leg_b", 0.0, 2.0),
    ("arm_b", -0.5, 0.5),
)
UPPER = ("arm_a", "arm_b")
LEGS = np.array([True, False, True, False])
ARMS = ~LEGS


def config(**overrides):
    values = dict(
        position_margin_rad=0.0,
        stale_timeout_s=0.10,
        hard_timeout_s=0.30,
        blend_rate_per_s=4.0,
    )
    values.update(overrides)
    return SafetyConfig(**values)


class LiveRangePreviewControllerTests(unittest.TestCase):
    def make(self, **overrides):
        controller = LiveRangePreviewController(RANGES, UPPER, config(**overrides))
        controller.reset(np.zeros(4), now=10.0)
        return controller

    def test_waiting_for_fresh_target_holds_the_measured_pose(self):
        controller = LiveRangePreviewController(RANGES, UPPER, config())
        measured = np.array([0.1, 0.2, 0.3, 0.4])
        output = controller.step(measured, now=1.0, enable=True)  # resets implicitly
        self.assertEqual(output.reason, "waiting_for_fresh_target")
        self.assertFalse(output.engaged)
        self.assertFalse(controller.has_target)
        np.testing.assert_array_equal(output.dof_pos, measured)
        output = controller.step(measured, now=1.02, enable=True)
        np.testing.assert_array_equal(output.dof_pos, measured)
        self.assertEqual(output.velocities, {"arm_a": 0.0, "arm_b": 0.0})

    def test_tracking_blends_the_upper_body_in_at_the_blend_rate(self):
        controller = self.make()
        target = np.array([0.5, 0.8, 1.5, 0.4])
        controller.update_target(target, source_timestamp=10.05)
        output = controller.step(np.zeros(4), now=10.05, enable=True)
        self.assertEqual(output.reason, "tracking")
        self.assertTrue(output.engaged)
        self.assertAlmostEqual(output.blend_weight, 0.2)  # 4/s * 0.05 s
        np.testing.assert_allclose(output.dof_pos[ARMS], 0.2 * target[ARMS])
        np.testing.assert_allclose(output.dof_pos[LEGS], target[LEGS])
        self.assertAlmostEqual(output.velocities["arm_a"], 0.16 / 0.05)
        controller.update_target(target, source_timestamp=10.30)
        output = controller.step(np.zeros(4), now=10.30, enable=True)
        self.assertEqual(output.blend_weight, 1.0)
        np.testing.assert_allclose(output.dof_pos, target)
        self.assertEqual(output.clamped_joints, ())

    def test_targets_are_clamped_to_ranges_and_only_real_clamps_reported(self):
        controller = self.make(blend_rate_per_s=1000.0)
        small = 0.5 * CLAMP_REPORT_TOLERANCE_RAD
        target = np.array([3.0, -1.0 - small, -0.2, 0.9])
        controller.update_target(target, source_timestamp=10.01)
        output = controller.step(np.zeros(4), now=10.01, enable=True)
        np.testing.assert_allclose(output.dof_pos, [1.0, -1.0, 0.0, 0.5])
        self.assertEqual(output.clamped_joints, ("leg_a", "leg_b", "arm_b"))

    def test_position_margin_narrows_the_range(self):
        controller = self.make(position_margin_rad=0.1, blend_rate_per_s=1000.0)
        controller.update_target(np.array([5.0, 5.0, -5.0, -5.0]), source_timestamp=10.01)
        output = controller.step(np.zeros(4), now=10.01, enable=True)
        np.testing.assert_allclose(output.dof_pos, [0.9, 0.9, 0.1, -0.4])

    def test_disabled_freezes_the_upper_body_but_legs_follow_the_bounded_target(self):
        controller = self.make(blend_rate_per_s=1000.0)
        controller.update_target(np.array([0.1, 0.3, 0.5, 0.2]), source_timestamp=10.01)
        tracked = controller.step(np.zeros(4), now=10.01, enable=True).dof_pos
        np.testing.assert_allclose(tracked, [0.1, 0.3, 0.5, 0.2])
        controller.update_target(np.array([4.0, -0.9, 0.7, -0.3]), source_timestamp=10.03)
        output = controller.step(np.zeros(4), now=10.03, enable=False)
        self.assertEqual(output.reason, "disabled")
        self.assertFalse(output.engaged)
        self.assertEqual(output.blend_weight, 0.0)
        np.testing.assert_allclose(output.dof_pos[ARMS], tracked[ARMS])
        np.testing.assert_allclose(output.dof_pos[LEGS], [1.0, 0.7])
        self.assertEqual(output.clamped_joints, ("leg_a",))
        # Re-enabling blends from the frozen pose rather than jumping.
        controller = self.make()
        controller.update_target(np.array([0.0, 0.5, 0.0, 0.0]), source_timestamp=10.01)
        controller.step(np.zeros(4), now=10.01, enable=False)
        output = controller.step(np.zeros(4), now=10.06, enable=True)
        self.assertAlmostEqual(output.dof_pos[1], 0.2 * 0.5)

    def test_start_disabled_never_moves_the_upper_body(self):
        controller = self.make()
        controller.update_target(np.array([0.5, 0.8, 1.5, 0.4]), source_timestamp=10.02)
        for now in (10.02, 10.04, 10.06):
            output = controller.step(np.zeros(4), now=now, enable=False)
            np.testing.assert_array_equal(output.dof_pos[ARMS], [0.0, 0.0])
            np.testing.assert_allclose(output.dof_pos[LEGS], [0.5, 1.5])

    def test_stale_source_holds_the_last_output(self):
        controller = self.make(blend_rate_per_s=1000.0)
        controller.update_target(np.array([0.1, 0.3, 0.5, 0.2]), source_timestamp=10.0)
        last = controller.step(np.zeros(4), now=10.02, enable=True).dof_pos
        output = controller.step(np.full(4, 0.7), now=10.15, enable=True)
        self.assertEqual(output.reason, "stale_source")
        self.assertFalse(output.engaged)
        self.assertAlmostEqual(output.source_age_s, 0.15)
        self.assertFalse(output.reset_required)
        np.testing.assert_array_equal(output.dof_pos, last)

    def test_fresh_targets_after_a_stale_gap_blend_back_in(self):
        controller = self.make()
        first = np.array([0.0, 0.2, 0.5, 0.1])
        controller.update_target(first, source_timestamp=10.0)
        for now in (10.1, 10.2, 10.3):  # blend reaches 1
            controller.update_target(first, source_timestamp=now)
            held = controller.step(np.zeros(4), now=now, enable=True).dof_pos
        stale = controller.step(np.zeros(4), now=10.45, enable=True)
        self.assertEqual(stale.reason, "stale_source")
        jump = np.array([0.0, -0.8, 0.5, -0.4])
        controller.update_target(jump, source_timestamp=10.47)
        output = controller.step(np.zeros(4), now=10.47, enable=True)
        self.assertEqual(output.reason, "tracking")
        # 4/s * 0.02 s of blend from the held pose, not a jump to the new target.
        self.assertAlmostEqual(output.blend_weight, 0.08)
        np.testing.assert_allclose(
            output.dof_pos[ARMS], held[ARMS] + 0.08 * (jump[ARMS] - held[ARMS])
        )

    def test_hard_timeout_latches_until_reset(self):
        controller = self.make(blend_rate_per_s=1000.0)
        controller.update_target(np.array([0.1, 0.3, 0.5, 0.2]), source_timestamp=10.0)
        last = controller.step(np.zeros(4), now=10.02, enable=True).dof_pos
        output = controller.step(np.zeros(4), now=10.4, enable=True)
        self.assertEqual(output.reason, "hard_timeout")
        self.assertTrue(output.reset_required)
        self.assertTrue(controller.reset_required)
        np.testing.assert_array_equal(output.dof_pos, last)
        # A fresh target does not clear the latch.
        controller.update_target(np.array([0.2, 0.2, 0.2, 0.2]), source_timestamp=10.41)
        output = controller.step(np.zeros(4), now=10.42, enable=True)
        self.assertEqual(output.reason, "hard_timeout")
        np.testing.assert_array_equal(output.dof_pos, last)
        measured = np.array([0.05, -0.1, 0.4, 0.0])
        controller.reset(measured, now=10.5)
        self.assertFalse(controller.reset_required)
        self.assertFalse(controller.has_target)
        output = controller.step(measured, now=10.52, enable=True)
        self.assertEqual(output.reason, "waiting_for_fresh_target")
        self.assertFalse(output.reset_required)
        np.testing.assert_array_equal(output.dof_pos, measured)

    def test_invalid_shapes_and_non_finite_values_raise(self):
        controller = self.make()
        for bad in (np.zeros(3), np.zeros(5), np.zeros((2, 2)), np.array([0, math.nan, 0, 0])):
            with self.subTest(value=bad.tolist()):
                with self.assertRaises(SafetyError):
                    controller.update_target(bad, source_timestamp=10.0)
                with self.assertRaises(SafetyError):
                    controller.step(bad, now=10.1, enable=True)
                with self.assertRaises(SafetyError):
                    controller.reset(bad, now=10.1)
        with self.assertRaises(SafetyError):
            controller.update_target(np.array([0, math.inf, 0, 0]), source_timestamp=10.0)
        with self.assertRaises(SafetyError):
            controller.update_target(np.zeros(4), source_timestamp=math.nan)

    def test_constructor_rejects_unknown_upper_body_joints_and_collapsed_ranges(self):
        with self.assertRaisesRegex(SafetyError, "not_a_joint"):
            LiveRangePreviewController(RANGES, ("arm_a", "not_a_joint"), config())
        with self.assertRaisesRegex(SafetyError, "arm_b"):
            LiveRangePreviewController(RANGES, UPPER, config(position_margin_rad=0.6))

    def test_t1_layout_is_accepted_with_zero_margin(self):
        controller = LiveRangePreviewController(
            T1_MOCAP_JOINT_RANGES, T1_UPPER_BODY_JOINTS, config()
        )
        self.assertEqual(len(controller.joint_names), 27)
        output = controller.step(np.zeros(27), now=0.0, enable=True)
        self.assertEqual(set(output.velocities), set(T1_UPPER_BODY_JOINTS))


class CalibrationKeyTests(unittest.TestCase):
    def test_n_key_requests_one_calibration(self):
        controls = LiveOperatorControls()
        self.assertFalse(controls.take_calibration_request())
        controls.keyboard_callback(ord("N"))
        self.assertTrue(controls.take_calibration_request())
        self.assertFalse(controls.take_calibration_request())
        controls.keyboard_callback(ord("n"))
        controls.keyboard_callback(ord("n"))
        self.assertTrue(controls.take_calibration_request())
        self.assertFalse(controls.take_calibration_request())

    def test_n_key_is_independent_of_rearm_and_enable(self):
        controls = LiveOperatorControls(enabled=True)
        controls.keyboard_callback(ord("n"))
        self.assertFalse(controls.take_rearm_request())
        self.assertTrue(controls.enabled)
        self.assertTrue(controls.take_calibration_request())
        controls.keyboard_callback(ord("r"))
        self.assertFalse(controls.take_calibration_request())
        self.assertTrue(controls.take_rearm_request())


if __name__ == "__main__":
    unittest.main()
