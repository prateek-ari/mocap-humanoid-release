from __future__ import annotations

import copy
from dataclasses import replace
from pathlib import Path
import unittest

from mocap_g1.safety import ArmSafetyEnvelope, SafetyConfig, SafetyError
from mocap_g1.schema import G1_ARM_SDK_JOINTS, load_official_joint_limits


REPO_ROOT = Path(__file__).resolve().parents[1]


class SafetyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.limits = load_official_joint_limits(
            REPO_ROOT / "eval/assets/g1_29dof_official.urdf"
        )

    def initial(self):
        return {
            name: (self.limits[name].lower + self.limits[name].upper) / 2.0
            for name in G1_ARM_SDK_JOINTS
        }

    def assert_transition_is_bounded(self, previous, command, dt, config):
        # A tick longer than max_step_dt_s is credited only max_step_dt_s of
        # motion; the rest is a stall, so one command can never move a joint
        # farther than a nominal step at the velocity cap.
        step_dt = min(dt, config.max_step_dt_s)
        for name in G1_ARM_SDK_JOINTS:
            velocity_cap = self.limits[name].velocity * config.velocity_limit_fraction
            displacement = command.positions[name] - previous.positions[name]
            self.assertLessEqual(abs(displacement), velocity_cap * step_dt + 1e-12)
            emitted_velocity = displacement / step_dt
            self.assertAlmostEqual(command.velocities[name], emitted_velocity, places=10)
            self.assertLessEqual(abs(emitted_velocity), velocity_cap + 1e-9)
            self.assertLessEqual(
                abs(command.velocities[name] - previous.velocities[name]) / step_dt,
                config.acceleration_limit_rad_s2 + 1e-8,
            )

    def test_requires_reset_and_exact_named_input(self):
        envelope = ArmSafetyEnvelope(self.limits)
        with self.assertRaises(SafetyError):
            envelope.step({}, source_timestamp=0.0, now=0.02, enable=True)
        envelope.reset(self.initial(), now=0.0)
        target = self.initial()
        target.pop(next(iter(target)))
        with self.assertRaises(SafetyError):
            envelope.step(target, source_timestamp=0.02, now=0.02, enable=True)

    def test_rejects_invalid_safety_configuration(self):
        invalid = (
            ("position_margin_rad", -0.01),
            ("position_margin_rad", float("nan")),
            ("velocity_limit_fraction", 0.0),
            ("velocity_limit_fraction", 1.01),
            ("acceleration_limit_rad_s2", 0.0),
            ("acceleration_limit_rad_s2", float("inf")),
            ("stale_timeout_s", 0.0),
            ("hard_timeout_s", SafetyConfig.stale_timeout_s),
            ("hard_timeout_s", float("nan")),
            ("blend_rate_per_s", 0.0),
            ("blend_rate_per_s", float("inf")),
            ("source_future_tolerance_s", -0.001),
            ("source_future_tolerance_s", SafetyConfig.stale_timeout_s),
            ("source_future_tolerance_s", float("nan")),
            ("max_step_dt_s", 0.0),
            ("max_step_dt_s", SafetyConfig.hard_timeout_s),
            ("max_step_dt_s", float("nan")),
            ("max_step_dt_s", True),
        )
        for field, value in invalid:
            with self.subTest(field=field, value=value):
                config = replace(SafetyConfig(), **{field: value})
                with self.assertRaisesRegex(SafetyError, field):
                    ArmSafetyEnvelope(self.limits, config)

    def test_non_finite_and_future_source_timestamps_fail_closed(self):
        # max(0, now - ts) used to map NaN, +inf and any future timestamp to a
        # source age of 0, so stale and hard timeouts could never fire.
        config = SafetyConfig()
        for source_timestamp in (
            float("nan"),
            float("inf"),
            float("-inf"),
            0.02 + config.source_future_tolerance_s + 0.001,
            1_000_000.0,
        ):
            with self.subTest(source_timestamp=source_timestamp):
                envelope = ArmSafetyEnvelope(self.limits, config)
                envelope.reset(self.initial(), now=0.0)
                with self.assertRaises(SafetyError):
                    envelope.step(
                        self.initial(),
                        source_timestamp=source_timestamp,
                        now=0.02,
                        enable=True,
                    )

        # A callback race inside the tolerance is still accepted as fresh.
        envelope = ArmSafetyEnvelope(self.limits, config)
        envelope.reset(self.initial(), now=0.0)
        command = envelope.step(
            self.initial(),
            source_timestamp=0.02 + config.source_future_tolerance_s / 2.0,
            now=0.02,
            enable=True,
        )
        self.assertEqual(command.reason, "tracking")
        self.assertEqual(command.source_age_s, 0.0)

    def test_reset_rejects_non_finite_time(self):
        envelope = ArmSafetyEnvelope(self.limits)
        for now in (float("nan"), float("inf"), float("-inf")):
            with self.subTest(now=now):
                with self.assertRaisesRegex(SafetyError, "reset time must be finite"):
                    envelope.reset(self.initial(), now=now)
        with self.assertRaisesRegex(SafetyError, "reset from measured state"):
            envelope.step(self.initial(), source_timestamp=0.0, now=0.02, enable=True)

    def test_rejects_position_margin_that_collapses_a_joint_range(self):
        narrowest_width = min(
            self.limits[name].upper - self.limits[name].lower
            for name in G1_ARM_SDK_JOINTS
        )
        config = SafetyConfig(position_margin_rad=narrowest_width / 2.0)
        with self.assertRaisesRegex(SafetyError, "position margin collapses the range"):
            ArmSafetyEnvelope(self.limits, config)

    def test_clamps_rate_acceleration_and_times_out(self):
        config = SafetyConfig(
            velocity_limit_fraction=0.10,
            acceleration_limit_rad_s2=1.0,
            stale_timeout_s=0.10,
            hard_timeout_s=0.30,
            blend_rate_per_s=2.0,
        )
        envelope = ArmSafetyEnvelope(self.limits, config)
        initial = self.initial()
        envelope.reset(initial, now=0.0)
        target = {name: self.limits[name].upper + 10.0 for name in G1_ARM_SDK_JOINTS}
        tracking = envelope.step(target, source_timestamp=0.02, now=0.02, enable=True)
        self.assertEqual(set(tracking.clamped_joints), set(G1_ARM_SDK_JOINTS))
        self.assertAlmostEqual(tracking.blend_weight, 0.04)
        for name, velocity in tracking.velocities.items():
            self.assertLessEqual(abs(velocity), 0.0200001)
            self.assertLessEqual(
                abs(velocity), self.limits[name].velocity * config.velocity_limit_fraction
            )

        stale = envelope.step(target, source_timestamp=0.02, now=0.14, enable=True)
        self.assertEqual(stale.reason, "stale_source")
        self.assertLess(stale.blend_weight, tracking.blend_weight)
        hard = envelope.step(target, source_timestamp=0.02, now=0.40, enable=True)
        self.assertEqual(hard.reason, "hard_timeout")
        self.assertEqual(hard.blend_weight, 0.0)
        self.assertFalse(hard.engaged)

    def test_saturation_and_reversal_use_emitted_position_velocity(self):
        config = SafetyConfig(
            velocity_limit_fraction=0.10,
            acceleration_limit_rad_s2=8.0,
            stale_timeout_s=0.20,
            hard_timeout_s=1.0,
            blend_rate_per_s=10.0,
        )
        envelope = ArmSafetyEnvelope(self.limits, config)
        initial = self.initial()
        envelope.reset(initial, now=0.0)
        previous = type("InitialCommand", (), {
            "positions": initial,
            "velocities": {name: 0.0 for name in G1_ARM_SDK_JOINTS},
        })()
        now = 0.0

        for direction in (1.0, -1.0):
            target = {
                name: (
                    self.limits[name].upper + 10.0
                    if direction > 0.0
                    else self.limits[name].lower - 10.0
                )
                for name in G1_ARM_SDK_JOINTS
            }
            for _ in range(250):
                dt = 0.01
                now += dt
                command = envelope.step(
                    target, source_timestamp=now, now=now, enable=True
                )
                self.assert_transition_is_bounded(previous, command, dt, config)
                for name in G1_ARM_SDK_JOINTS:
                    self.assertGreaterEqual(
                        command.positions[name],
                        self.limits[name].lower + config.position_margin_rad - 1e-9,
                    )
                    self.assertLessEqual(
                        command.positions[name],
                        self.limits[name].upper - config.position_margin_rad + 1e-9,
                    )
                previous = command

        self.assertEqual(set(command.clamped_joints), set(G1_ARM_SDK_JOINTS))

    def test_disable_while_moving_decelerates_instead_of_freezing(self):
        config = SafetyConfig(
            velocity_limit_fraction=0.10,
            acceleration_limit_rad_s2=2.0,
            stale_timeout_s=0.20,
            hard_timeout_s=1.0,
            blend_rate_per_s=10.0,
        )
        envelope = ArmSafetyEnvelope(self.limits, config)
        initial = self.initial()
        target = {name: initial[name] + 0.5 for name in G1_ARM_SDK_JOINTS}
        envelope.reset(initial, now=0.0)
        previous = type("InitialCommand", (), {
            "positions": initial,
            "velocities": {name: 0.0 for name in G1_ARM_SDK_JOINTS},
        })()
        now = 0.0
        for _ in range(10):
            now += 0.02
            previous = envelope.step(
                target, source_timestamp=now, now=now, enable=True
            )

        moving_velocity = previous.velocities["left_elbow_joint"]
        old_position = previous.positions["left_elbow_joint"]
        now += 0.02
        disabled = envelope.step(
            target, source_timestamp=now, now=now, enable=False
        )
        self.assertEqual(disabled.reason, "disabled")
        self.assertGreater(disabled.velocities["left_elbow_joint"], 0.0)
        self.assertLess(disabled.velocities["left_elbow_joint"], moving_velocity)
        self.assertGreater(disabled.positions["left_elbow_joint"], old_position)
        self.assert_transition_is_bounded(previous, disabled, 0.02, config)

        previous = disabled
        for _ in range(20):
            now += 0.02
            command = envelope.step(
                target, source_timestamp=now, now=now, enable=False
            )
            self.assert_transition_is_bounded(previous, command, 0.02, config)
            previous = command
        self.assertAlmostEqual(previous.velocities["left_elbow_joint"], 0.0)

        now += 0.02
        reenabled = envelope.step(
            target, source_timestamp=now, now=now, enable=True
        )
        self.assertEqual(reenabled.reason, "tracking")
        self.assert_transition_is_bounded(previous, reenabled, 0.02, config)

    def test_stale_and_hard_timeout_decelerate_and_hard_timeout_latches(self):
        config = SafetyConfig(
            velocity_limit_fraction=0.10,
            acceleration_limit_rad_s2=2.0,
            stale_timeout_s=0.10,
            hard_timeout_s=0.30,
            blend_rate_per_s=4.0,
        )
        envelope = ArmSafetyEnvelope(self.limits, config)
        initial = self.initial()
        target = {name: initial[name] + 0.5 for name in G1_ARM_SDK_JOINTS}
        envelope.reset(initial, now=0.0)
        previous = type("InitialCommand", (), {
            "positions": initial,
            "velocities": {name: 0.0 for name in G1_ARM_SDK_JOINTS},
        })()
        now = 0.0
        for _ in range(10):
            now += 0.02
            previous = envelope.step(
                target, source_timestamp=now, now=now, enable=True
            )

        frozen_source_timestamp = now
        saw_stale = False
        saw_hard = False
        for _ in range(20):
            now += 0.02
            command = envelope.step(
                target,
                source_timestamp=frozen_source_timestamp,
                now=now,
                enable=True,
            )
            self.assert_transition_is_bounded(previous, command, 0.02, config)
            saw_stale |= command.reason == "stale_source"
            saw_hard |= command.reason == "hard_timeout"
            previous = command
        self.assertTrue(saw_stale)
        self.assertTrue(saw_hard)
        self.assertEqual(previous.blend_weight, 0.0)

        now += 0.02
        fresh_but_latched = envelope.step(
            target, source_timestamp=now, now=now, enable=True
        )
        self.assertEqual(fresh_but_latched.reason, "hard_timeout")
        self.assertFalse(fresh_but_latched.engaged)
        self.assert_transition_is_bounded(previous, fresh_but_latched, 0.02, config)

        envelope.reset(fresh_but_latched.positions, now=now)
        now += 0.02
        rearmed = envelope.step(target, source_timestamp=now, now=now, enable=True)
        self.assertEqual(rearmed.reason, "tracking")
        self.assertTrue(rearmed.engaged)

    def test_variable_dt_is_bounded_and_long_control_stall_requires_reset(self):
        config = SafetyConfig(
            velocity_limit_fraction=0.10,
            acceleration_limit_rad_s2=3.0,
            stale_timeout_s=0.20,
            hard_timeout_s=0.50,
            blend_rate_per_s=2.0,
        )
        envelope = ArmSafetyEnvelope(self.limits, config)
        initial = self.initial()
        target = {name: initial[name] + 1.0 for name in G1_ARM_SDK_JOINTS}
        envelope.reset(initial, now=0.0)
        previous = type("InitialCommand", (), {
            "positions": initial,
            "velocities": {name: 0.0 for name in G1_ARM_SDK_JOINTS},
        })()
        now = 0.0
        for dt in (0.005, 0.031, 0.012, 0.087, 0.019):
            now += dt
            command = envelope.step(
                target, source_timestamp=now, now=now, enable=True
            )
            self.assert_transition_is_bounded(previous, command, dt, config)
            previous = command

        now += config.hard_timeout_s
        with self.assertRaisesRegex(SafetyError, "control loop stalled"):
            envelope.step(target, source_timestamp=now, now=now, enable=True)
        with self.assertRaisesRegex(SafetyError, "reset from measured state"):
            envelope.step(target, source_timestamp=now + 0.01, now=now + 0.01, enable=True)

        envelope.reset(previous.positions, now=now + 0.01)
        now += 0.03
        rearmed = envelope.step(target, source_timestamp=now, now=now, enable=True)
        self.assertEqual(rearmed.reason, "tracking")

    def _tracking_at_speed(self, config):
        """Return an envelope (and its last command) after 1 s of 50 Hz tracking."""
        envelope = ArmSafetyEnvelope(self.limits, config)
        initial = self.initial()
        target = {
            name: self.limits[name].upper - config.position_margin_rad
            for name in G1_ARM_SDK_JOINTS
        }
        envelope.reset(initial, now=0.0)
        now = 0.0
        command = None
        for _ in range(50):
            now += 0.02
            command = envelope.step(target, source_timestamp=now, now=now, enable=True)
        assert command is not None
        return envelope, command, target, now

    def test_long_tick_displacement_is_bounded_to_one_credited_step(self):
        # Velocity alone used to bound a tick, so any dt below hard_timeout_s
        # legalised velocity_cap * dt of displacement in one command.
        config = SafetyConfig()
        envelope, previous, target, now = self._tracking_at_speed(config)
        self.assertEqual(previous.reason, "tracking")
        self.assertGreater(abs(previous.velocities["left_shoulder_pitch_joint"]), 1.0)

        for long_dt in (0.12, 0.49):
            with self.subTest(dt=long_dt):
                long_tick = copy.deepcopy(envelope)
                nominal = copy.deepcopy(envelope)
                later = now + long_dt
                command = long_tick.step(
                    target, source_timestamp=later, now=later, enable=True
                )
                self.assertEqual(command.reason, "tracking")
                self.assert_transition_is_bounded(previous, command, long_dt, config)
                step = config.max_step_dt_s
                for name in G1_ARM_SDK_JOINTS:
                    # At most one credited step at the previous speed plus one
                    # step of acceleration (0.27 rad at 0.12 s and 0.67 rad at
                    # 0.49 s on left_shoulder_pitch before the bound).
                    reachable_speed = (
                        abs(previous.velocities[name])
                        + config.acceleration_limit_rad_s2 * step
                    )
                    self.assertLessEqual(
                        abs(command.positions[name] - previous.positions[name]),
                        reachable_speed * step + 1e-12,
                    )
                self.assertLess(
                    abs(
                        command.positions["left_shoulder_pitch_joint"]
                        - previous.positions["left_shoulder_pitch_joint"]
                    ),
                    0.11,
                )

                # The command after the gap is exactly the command a single
                # max_step_dt_s tick would have produced from the same state.
                reference_time = now + config.max_step_dt_s
                expected = nominal.step(
                    target,
                    source_timestamp=reference_time,
                    now=reference_time,
                    enable=True,
                )
                for name in G1_ARM_SDK_JOINTS:
                    self.assertAlmostEqual(
                        command.positions[name], expected.positions[name], places=12
                    )
                    self.assertAlmostEqual(
                        command.velocities[name], expected.velocities[name], places=9
                    )
                self.assertAlmostEqual(command.blend_weight, expected.blend_weight)

    def test_long_tick_ramp_down_uses_wall_clock_time(self):
        config = SafetyConfig(blend_rate_per_s=1.0)
        envelope, previous, target, now = self._tracking_at_speed(config)
        weight = previous.blend_weight
        self.assertGreater(weight, 0.4)
        now += 0.3
        command = envelope.step(target, source_timestamp=now, now=now, enable=False)
        self.assertEqual(command.reason, "disabled")
        self.assertAlmostEqual(command.blend_weight, max(0.0, weight - 0.3))
        self.assert_transition_is_bounded(previous, command, 0.3, config)


if __name__ == "__main__":
    unittest.main()
