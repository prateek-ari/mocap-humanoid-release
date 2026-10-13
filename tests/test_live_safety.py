from __future__ import annotations

from pathlib import Path
import threading
import time
import unittest

import numpy as np

from mocap_g1.live_safety import (
    LiveG1SafetyController,
    LiveOperatorControls,
    named_arm_positions,
    replace_named_arm_positions,
)
from mocap_g1.safety import SafetyConfig, SafetyError
from mocap_g1.schema import (
    G1_29DOF_JOINTS,
    G1_ARM_SDK_JOINTS,
    load_official_joint_limits,
)


REPO_ROOT = Path(__file__).resolve().parents[1]


class LiveSafetyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.limits = load_official_joint_limits(
            REPO_ROOT / "eval/assets/g1_29dof_official.urdf"
        )

    def config(self):
        return SafetyConfig(
            velocity_limit_fraction=0.10,
            acceleration_limit_rad_s2=2.0,
            stale_timeout_s=0.10,
            hard_timeout_s=0.30,
            blend_rate_per_s=4.0,
        )

    def test_named_mapping_replaces_only_waist_and_arms(self):
        original = np.arange(len(G1_29DOF_JOINTS), dtype=np.float64)
        extracted = named_arm_positions(original)
        self.assertEqual(tuple(extracted), G1_ARM_SDK_JOINTS)
        for name in G1_ARM_SDK_JOINTS:
            self.assertEqual(extracted[name], G1_29DOF_JOINTS.index(name))

        replacements = {
            name: -100.0 - index for index, name in enumerate(G1_ARM_SDK_JOINTS)
        }
        result = replace_named_arm_positions(original, replacements)
        np.testing.assert_array_equal(result[:12], original[:12])
        np.testing.assert_array_equal(
            result[12:], np.asarray(list(replacements.values()), dtype=np.float64)
        )
        np.testing.assert_array_equal(original, np.arange(29, dtype=np.float64))

    def test_mapping_rejects_wrong_shape_nonfinite_and_incomplete_names(self):
        with self.assertRaisesRegex(SafetyError, "shape"):
            named_arm_positions(np.zeros(28))
        nonfinite = np.zeros(29)
        nonfinite[3] = np.nan
        with self.assertRaisesRegex(SafetyError, "non-finite"):
            named_arm_positions(nonfinite)
        with self.assertRaisesRegex(SafetyError, "exactly"):
            replace_named_arm_positions(np.zeros(29), {})

    def test_reset_uses_measured_pose_and_fresh_legs_pass_through(self):
        controller = LiveG1SafetyController(self.limits, self.config())
        measured = np.zeros(29)
        target = np.zeros(29)
        target[:12] = np.linspace(-0.2, 0.2, 12)
        target[12:] = 0.5

        controller.reset(measured, now=1.0)
        waiting = controller.step(measured, now=1.01, enable=True)
        self.assertEqual(waiting.reason, "waiting_for_fresh_target")
        np.testing.assert_array_equal(waiting.dof_pos, measured)

        controller.update_target(target, source_timestamp=1.01)
        tracking = controller.step(measured, now=1.02, enable=True)
        self.assertEqual(tracking.reason, "tracking")
        np.testing.assert_allclose(tracking.dof_pos[:12], target[:12])
        self.assertTrue(np.all(tracking.dof_pos[12:] > 0.0))
        self.assertTrue(np.all(tracking.dof_pos[12:] < target[12:]))

    def test_waiting_ticks_keep_control_clock_current_without_faking_a_target(self):
        controller = LiveG1SafetyController(self.limits, self.config())
        measured = np.zeros(29)
        controller.reset(measured, now=0.0)
        for now in (0.10, 0.20, 0.40):
            waiting = controller.step(measured, now=now, enable=True)
            measured = waiting.dof_pos
            self.assertEqual(waiting.reason, "waiting_for_fresh_target")
            self.assertFalse(waiting.engaged)
            self.assertFalse(waiting.reset_required)

        target = measured.copy()
        target[12:] = 0.2
        controller.update_target(target, source_timestamp=0.41)
        tracking = controller.step(measured, now=0.42, enable=True)
        self.assertEqual(tracking.reason, "tracking")
        self.assertFalse(tracking.reset_required)

    def test_stale_decelerates_and_hard_timeout_latches_until_reset(self):
        controller = LiveG1SafetyController(self.limits, self.config())
        measured = np.zeros(29)
        target = np.zeros(29)
        target[12:] = 0.5
        controller.reset(measured, now=0.0)

        now = 0.0
        for _ in range(10):
            now += 0.02
            controller.update_target(target, source_timestamp=now)
            output = controller.step(measured, now=now, enable=True)
            measured = output.dof_pos
        moving_velocity = output.velocities["left_elbow_joint"]
        reasons = []
        for _ in range(20):
            now += 0.02
            output = controller.step(measured, now=now, enable=True)
            measured = output.dof_pos
            reasons.append(output.reason)
        self.assertIn("stale_source", reasons)
        self.assertIn("hard_timeout", reasons)
        self.assertTrue(output.reset_required)
        self.assertEqual(output.blend_weight, 0.0)
        self.assertLessEqual(abs(output.velocities["left_elbow_joint"]), moving_velocity)

        controller.update_target(target, source_timestamp=now)
        still_latched = controller.step(measured, now=now + 0.02, enable=True)
        self.assertEqual(still_latched.reason, "hard_timeout")
        self.assertTrue(still_latched.reset_required)

        controller.reset(still_latched.dof_pos, now=now + 0.03)
        self.assertFalse(controller.reset_required)
        controller.update_target(target, source_timestamp=now + 0.04)
        rearmed = controller.step(
            still_latched.dof_pos, now=now + 0.05, enable=True
        )
        self.assertEqual(rearmed.reason, "tracking")
        self.assertFalse(rearmed.reset_required)

    def test_control_loop_stall_fails_closed_to_measured_state(self):
        controller = LiveG1SafetyController(self.limits, self.config())
        measured = np.zeros(29)
        target = np.zeros(29)
        target[12:] = 0.4
        controller.reset(measured, now=0.0)
        controller.update_target(target, source_timestamp=0.02)
        moving = controller.step(measured, now=0.02, enable=True)

        measured_after_stall = moving.dof_pos.copy()
        measured_after_stall[0] = 0.123
        stalled = controller.step(measured_after_stall, now=0.52, enable=True)
        self.assertEqual(stalled.reason, "control_stall_reset_required")
        self.assertTrue(stalled.reset_required)
        np.testing.assert_array_equal(stalled.dof_pos, measured_after_stall)
        self.assertIn("control loop stalled", controller.last_error)

        controller.update_target(target, source_timestamp=0.53)
        still_stalled = controller.step(measured_after_stall, now=0.53, enable=True)
        self.assertEqual(still_stalled.reason, "control_stall_reset_required")
        controller.reset(measured_after_stall, now=0.54)
        controller.update_target(target, source_timestamp=0.55)
        recovered = controller.step(measured_after_stall, now=0.56, enable=True)
        self.assertEqual(recovered.reason, "tracking")

    def test_disable_decelerates_without_latching_reset(self):
        controller = LiveG1SafetyController(self.limits, self.config())
        measured = np.zeros(29)
        target = np.zeros(29)
        target[12:] = 0.5
        controller.reset(measured, now=0.0)
        now = 0.0
        for _ in range(10):
            now += 0.02
            controller.update_target(target, source_timestamp=now)
            tracking = controller.step(measured, now=now, enable=True)
            measured = tracking.dof_pos

        moving_velocity = tracking.velocities["left_elbow_joint"]
        previous_position = measured[18]
        now += 0.02
        controller.update_target(target, source_timestamp=now)
        disabled = controller.step(measured, now=now, enable=False)
        self.assertEqual(disabled.reason, "disabled")
        self.assertFalse(disabled.reset_required)
        self.assertGreater(disabled.velocities["left_elbow_joint"], 0.0)
        self.assertLess(disabled.velocities["left_elbow_joint"], moving_velocity)
        self.assertGreater(disabled.dof_pos[18], previous_position)

    def test_operator_controls_toggle_and_consume_rearm(self):
        controls = LiveOperatorControls(enabled=True)
        controls.keyboard_callback(ord(" "))
        self.assertFalse(controls.enabled)
        controls.keyboard_callback(ord(" "))
        self.assertTrue(controls.enabled)
        controls.keyboard_callback(ord("R"))
        self.assertTrue(controls.take_rearm_request())
        self.assertFalse(controls.take_rearm_request())
        controls.keyboard_callback(-1)
        self.assertTrue(controls.enabled)

    def test_rearm_press_during_take_is_not_dropped(self):
        # take_rearm_request used to read the flag and then clear it, so a
        # viewer-thread press landing between the two was silently lost.
        controls = LiveOperatorControls(enabled=True)
        main_thread = threading.current_thread()
        presser: list[threading.Thread] = []

        class PressInsideTake:
            """Lock that injects a viewer press while the take holds it."""

            def __init__(self) -> None:
                self._inner = threading.Lock()

            def __enter__(self):
                self._inner.acquire()
                if threading.current_thread() is main_thread and not presser:
                    thread = threading.Thread(
                        target=controls.keyboard_callback, args=(ord("r"),)
                    )
                    presser.append(thread)
                    thread.start()
                    # Give the press every chance to land mid-take.
                    time.sleep(0.02)
                return self

            def __exit__(self, *exc_info) -> None:
                self._inner.release()

        controls.keyboard_callback(ord("r"))
        controls._lock = PressInsideTake()  # type: ignore[assignment]
        self.assertTrue(controls.take_rearm_request())
        presser[0].join(timeout=5.0)
        self.assertFalse(presser[0].is_alive())
        self.assertTrue(controls.take_rearm_request(), "mid-take press was dropped")
        self.assertFalse(controls.take_rearm_request())


if __name__ == "__main__":
    unittest.main()
