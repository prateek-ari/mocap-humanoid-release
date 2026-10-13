from __future__ import annotations

from pathlib import Path
import unittest

import numpy as np

from mocap_g1.fault_replay import (
    FaultReplayConfig,
    FaultReplayError,
    TimeInterval,
    replay_arm_faults,
)
from mocap_g1.safety import SafetyConfig
from mocap_g1.schema import G1_ARM_SDK_JOINTS, load_official_joint_limits


REPO_ROOT = Path(__file__).resolve().parents[1]


def reference(frames: int = 401, fps: float = 50.0) -> np.ndarray:
    qpos = np.zeros((frames, 36), dtype=np.float64)
    qpos[:, 3] = 1.0
    time = np.arange(frames) / fps
    for column, _name in enumerate(G1_ARM_SDK_JOINTS):
        qpos[:, 7 + 12 + column] = 0.12 * np.sin(
            2.0 * np.pi * 0.20 * time + 0.07 * column
        )
    return qpos


class FaultReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.limits = load_official_joint_limits(
            REPO_ROOT / "eval/assets/g1_29dof_official.urdf"
        )

    def test_newest_frame_selection_is_monotonic_under_reorder_and_loss(self):
        config = FaultReplayConfig(
            control_rate_hz=40.0,
            latency_s=0.01,
            packet_loss_every=9,
            reorder_every=5,
            reorder_delay_s=0.08,
            tracking_p95_limit_rad=0.5,
            tracking_max_limit_rad=1.0,
            require_reasons=("tracking",),
        )
        first = replay_arm_faults(reference(frames=151), 50.0, limits=self.limits,
                                  replay_config=config)
        second = replay_arm_faults(reference(frames=151), 50.0, limits=self.limits,
                                   replay_config=config)
        self.assertEqual(first, second)
        self.assertEqual(first["status"], "PASS")
        self.assertGreater(first["transport"]["packets_dropped_by_loss"], 0)
        self.assertGreater(first["transport"]["out_of_order_arrivals"], 0)
        self.assertGreater(
            first["transport"]["packets_discarded_by_newest_frame_selection"], 0
        )
        # Late packets really did try to move the selection backwards, and
        # the newest-sequence rule rejected every one of them.
        self.assertGreater(first["transport"]["stale_selections_rejected"], 0)
        self.assertEqual(first["transport"]["selected_sequence_regressions"], 0)
        self.assertEqual(
            first["assertions"]["selected_sequence_regressions"]["status"], "PASS"
        )

    def test_stale_hard_latch_disable_and_control_gap_fail_closed(self):
        result = replay_arm_faults(
            reference(),
            50.0,
            limits=self.limits,
            replay_config=FaultReplayConfig(
                control_rate_hz=50.0,
                latency_s=0.02,
                packet_loss_every=11,
                reorder_every=7,
                reorder_delay_s=0.07,
                source_stalls=(TimeInterval(2.0, 0.75),),
                disable_intervals=(TimeInterval(0.6, 0.30),),
                control_gaps=(TimeInterval(4.5, 0.60),),
                source_hard_reset_delay_s=0.10,
                tracking_p95_limit_rad=0.5,
                tracking_max_limit_rad=1.0,
                require_reasons=("tracking", "disabled", "stale_source", "hard_timeout"),
            ),
            safety_config=SafetyConfig(
                velocity_limit_fraction=0.25,
                acceleration_limit_rad_s2=4.0,
                stale_timeout_s=0.10,
                hard_timeout_s=0.50,
                blend_rate_per_s=1.0,
            ),
        )
        self.assertEqual(result["status"], "PASS")
        for reason in ("tracking", "disabled", "stale_source", "hard_timeout"):
            self.assertGreater(result["control"]["reason_counts"][reason], 0)
        self.assertGreater(
            result["fault_response"]["fresh_commands_held_in_hard_timeout_latch"], 0
        )
        self.assertTrue(
            result["fault_response"]["tracking_resumed_after_source_hard_reset"]
        )
        self.assertEqual(result["fault_response"]["control_gap_stall_errors"], 1)
        self.assertEqual(result["fault_response"]["control_gap_resets"], 1)
        self.assertEqual(result["safety_compliance"]["position_violation_samples"], 0)
        self.assertEqual(result["safety_compliance"]["velocity_violation_samples"], 0)
        self.assertEqual(result["safety_compliance"]["acceleration_violation_samples"], 0)

    def test_rejects_invalid_input_and_fault_configuration(self):
        with self.assertRaisesRegex(FaultReplayError, "shape"):
            replay_arm_faults(np.zeros((3, 35)), 50.0, limits=self.limits)
        with self.assertRaisesRegex(FaultReplayError, "reorder_delay_s"):
            FaultReplayConfig(reorder_every=3).validate()
        with self.assertRaisesRegex(FaultReplayError, "duration"):
            TimeInterval(1.0, 0.0).validate("test")
        with self.assertRaisesRegex(FaultReplayError, "hard_timeout_s"):
            replay_arm_faults(
                reference(frames=3),
                50.0,
                limits=self.limits,
                safety_config=SafetyConfig(stale_timeout_s=0.5, hard_timeout_s=0.5),
            )

    def test_uses_the_canonical_safety_and_limit_validators(self):
        from dataclasses import replace

        # Each of these passed the old hand-written checks and then escaped as
        # a raw SafetyError (or was silently accepted).
        invalid_safety = (
            SafetyConfig(blend_rate_per_s=True),
            SafetyConfig(source_future_tolerance_s=-1.0),
        )
        for safety in invalid_safety:
            with self.subTest(safety=safety):
                with self.assertRaises(FaultReplayError):
                    replay_arm_faults(
                        reference(frames=3), 50.0, limits=self.limits, safety_config=safety
                    )

        name = G1_ARM_SDK_JOINTS[0]
        inverted = dict(self.limits)
        inverted[name] = replace(
            self.limits[name], lower=self.limits[name].upper, upper=self.limits[name].lower
        )
        missing = dict(self.limits)
        del missing[name]
        for limits in (inverted, missing):
            with self.subTest(limits=len(limits)):
                with self.assertRaisesRegex(FaultReplayError, name):
                    replay_arm_faults(reference(frames=3), 50.0, limits=limits)


if __name__ == "__main__":
    unittest.main()
