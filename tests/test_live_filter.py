from __future__ import annotations

from pathlib import Path
import unittest

import numpy as np

from mocap_g1.live_filter import (
    DEFAULT_REFERENCE_SINE_RAD,
    DEFAULT_SOURCE_RATE_HZ,
    DEFAULT_STEP_RAD,
    DOCUMENTED_JITTER_ATTENUATION_MIN_DB,
    DOCUMENTED_SINE_LAG_BOUND_MS,
    DOCUMENTED_STEP_LAG_50_BOUND_MS,
    DOCUMENTED_STEP_LAG_90_BOUND_MS,
    FILTER_KINDS,
    CommandUpsampler,
    CriticallyDampedConfig,
    FrameTimeline,
    LatencyRecorder,
    LiveFilterError,
    OneEuroConfig,
    UpsamplerConfig,
    default_config,
    format_configuration_report,
    make_filter,
    measure_jitter_attenuation,
    measure_phase_lag,
    measure_step_lag,
    upsampler_lag_ms,
)
from mocap_g1.live_safety import LiveG1SafetyController
from mocap_g1.safety import SafetyConfig
from mocap_g1.schema import (
    G1_29DOF_JOINTS,
    G1_ARM_SDK_JOINTS,
    load_official_joint_limits,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
SOURCE_PERIOD = 1.0 / DEFAULT_SOURCE_RATE_HZ


def run_sequence(config, samples, *, sample_rate_hz=DEFAULT_SOURCE_RATE_HZ):
    """Filter a 1-D sequence, returning the emitted values in order."""
    joint_filter = make_filter(config, 1)
    dt = 1.0 / sample_rate_hz
    return [
        float(joint_filter.step(np.asarray([value]), now=index * dt).values[0])
        for index, value in enumerate(samples)
    ]


class FilterCausalityTests(unittest.TestCase):
    def test_output_prefix_is_unchanged_when_future_samples_change(self):
        rng = np.random.default_rng(7)
        base = np.cumsum(rng.normal(0.0, 0.05, 90))
        split = 40
        for kind in FILTER_KINDS:
            config = default_config(kind)
            original = run_sequence(config, base)
            mutated_input = base.copy()
            mutated_input[split + 1 :] = -3.0 * mutated_input[split + 1 :] + 5.0
            mutated = run_sequence(config, mutated_input)
            with self.subTest(kind=kind):
                np.testing.assert_array_equal(
                    original[: split + 1], mutated[: split + 1]
                )
                self.assertNotAlmostEqual(original[split + 1], mutated[split + 1])

    def test_step_call_does_not_mutate_caller_or_returned_state(self):
        joint_filter = make_filter(OneEuroConfig(), 3)
        values = np.asarray([0.1, -0.2, 0.3])
        joint_filter.step(values, now=0.0)
        np.testing.assert_array_equal(values, np.asarray([0.1, -0.2, 0.3]))
        second = joint_filter.step(np.ones(3), now=SOURCE_PERIOD)
        second.values[:] = 99.0
        third = joint_filter.step(np.ones(3), now=2 * SOURCE_PERIOD)
        self.assertTrue(np.all(np.abs(third.values) < 2.0))

    def test_first_sample_passes_through_and_joints_stay_independent(self):
        joint_filter = make_filter(CriticallyDampedConfig(), 4)
        first = joint_filter.step(np.asarray([0.5, -0.5, 0.0, 0.25]), now=0.0)
        np.testing.assert_allclose(first.values, [0.5, -0.5, 0.0, 0.25])
        self.assertTrue(first.reinitialized)
        moved = joint_filter.step(np.asarray([0.5, -0.5, 1.0, 0.25]), now=SOURCE_PERIOD)
        np.testing.assert_allclose(moved.values[[0, 1, 3]], [0.5, -0.5, 0.25])
        self.assertGreater(moved.values[2], 0.0)
        self.assertLess(moved.values[2], 1.0)


class FilterResponseTests(unittest.TestCase):
    def test_step_lag_is_within_the_documented_bound(self):
        for kind in FILTER_KINDS:
            response = measure_step_lag(default_config(kind))
            with self.subTest(kind=kind):
                self.assertGreater(response.lag_50_ms, 0.0)
                self.assertLessEqual(response.lag_50_ms, DOCUMENTED_STEP_LAG_50_BOUND_MS)
                self.assertLessEqual(response.lag_90_ms, DOCUMENTED_STEP_LAG_90_BOUND_MS)
                self.assertLess(response.overshoot_fraction, 0.01)
                self.assertEqual(response.step_rad, DEFAULT_STEP_RAD)

    def test_sine_phase_lag_is_within_the_documented_bound(self):
        for kind in FILTER_KINDS:
            response = measure_phase_lag(default_config(kind))
            with self.subTest(kind=kind):
                self.assertGreater(response.phase_lag_ms, 0.0)
                self.assertLessEqual(response.phase_lag_ms, DOCUMENTED_SINE_LAG_BOUND_MS)
                self.assertGreater(response.gain, 0.9)
                self.assertLessEqual(response.gain, 1.05)
                self.assertEqual(response.amplitude_rad, DEFAULT_REFERENCE_SINE_RAD)

    def test_noisy_sine_reaches_the_command_with_less_jitter_than_raw(self):
        for kind in FILTER_KINDS:
            response = measure_jitter_attenuation(default_config(kind))
            with self.subTest(kind=kind):
                self.assertLess(
                    response.command_jitter_rms_rad, response.raw_jitter_rms_rad
                )
                self.assertGreaterEqual(
                    response.attenuation_db, DOCUMENTED_JITTER_ATTENUATION_MIN_DB
                )

    def test_one_euro_trades_lag_for_speed_while_damped_lag_is_constant(self):
        slow = measure_phase_lag(OneEuroConfig(), amplitude_rad=0.05)
        fast = measure_phase_lag(OneEuroConfig(), amplitude_rad=1.0)
        self.assertLess(fast.phase_lag_ms, slow.phase_lag_ms - 10.0)
        damped_slow = measure_phase_lag(CriticallyDampedConfig(), amplitude_rad=0.05)
        damped_fast = measure_phase_lag(CriticallyDampedConfig(), amplitude_rad=1.0)
        self.assertAlmostEqual(
            damped_slow.phase_lag_ms, damped_fast.phase_lag_ms, places=6
        )

    def test_report_covers_the_configured_pipeline(self):
        report = format_configuration_report(OneEuroConfig(), UpsamplerConfig())
        self.assertIn("one_euro", report)
        self.assertIn("upsampler", report)
        self.assertIn("clamps remain the last authority", report)
        self.assertIn("disabled", format_configuration_report(None, None))


class FilterContractTests(unittest.TestCase):
    def test_rejects_bad_shapes_values_and_clocks(self):
        joint_filter = make_filter(OneEuroConfig(), 2)
        with self.assertRaisesRegex(LiveFilterError, "shape"):
            joint_filter.step(np.zeros(3), now=0.0)
        with self.assertRaisesRegex(LiveFilterError, "non-finite"):
            joint_filter.step(np.asarray([0.0, np.nan]), now=0.0)
        joint_filter.step(np.zeros(2), now=1.0)
        with self.assertRaisesRegex(LiveFilterError, "monotonic"):
            joint_filter.step(np.zeros(2), now=0.9)
        with self.assertRaisesRegex(LiveFilterError, "monotonic"):
            joint_filter.step(np.zeros(2), now=1.0)

    def test_long_reference_gap_reinitializes_instead_of_ramping(self):
        joint_filter = make_filter(OneEuroConfig(max_reference_gap_s=0.25), 1)
        for index in range(10):
            joint_filter.step(np.zeros(1), now=index * SOURCE_PERIOD)
        after_gap = joint_filter.step(np.asarray([0.4]), now=10 * SOURCE_PERIOD + 0.5)
        self.assertTrue(after_gap.reinitialized)
        self.assertAlmostEqual(float(after_gap.values[0]), 0.4)
        self.assertEqual(joint_filter.reinitializations, 2)

    def test_reset_drops_history_so_the_next_sample_passes_through(self):
        joint_filter = make_filter(CriticallyDampedConfig(), 1)
        for index in range(20):
            joint_filter.step(np.zeros(1), now=index * SOURCE_PERIOD)
        joint_filter.reset()
        self.assertFalse(joint_filter.initialized)
        rearmed = joint_filter.step(np.asarray([0.7]), now=100.0)
        self.assertTrue(rearmed.reinitialized)
        self.assertAlmostEqual(float(rearmed.values[0]), 0.7)

    def test_configuration_validation(self):
        with self.assertRaises(LiveFilterError):
            make_filter(OneEuroConfig(min_cutoff_hz=0.0), 1)
        with self.assertRaises(LiveFilterError):
            make_filter(OneEuroConfig(beta=-1.0), 1)
        with self.assertRaises(LiveFilterError):
            make_filter(CriticallyDampedConfig(cutoff_hz=float("inf")), 1)
        with self.assertRaises(LiveFilterError):
            make_filter(OneEuroConfig(), 0)
        with self.assertRaisesRegex(LiveFilterError, "unknown filter kind"):
            default_config("butterworth")


def _rk4_critically_damped(value, velocity, target, omega, duration, substeps=400):
    """Integrate x'' = -2*w*x' - w^2*(x - target) numerically (classic RK4)."""
    h = duration / substeps

    def derivative(x, v):
        return v, -2.0 * omega * v - omega * omega * (x - target)

    x, v = np.array(value, dtype=np.float64), np.array(velocity, dtype=np.float64)
    for _ in range(substeps):
        k1x, k1v = derivative(x, v)
        k2x, k2v = derivative(x + 0.5 * h * k1x, v + 0.5 * h * k1v)
        k3x, k3v = derivative(x + 0.5 * h * k2x, v + 0.5 * h * k2v)
        k4x, k4v = derivative(x + h * k3x, v + h * k3v)
        x = x + (h / 6.0) * (k1x + 2.0 * k2x + 2.0 * k3x + k4x)
        v = v + (h / 6.0) * (k1v + 2.0 * k2v + 2.0 * k3v + k4v)
    return x, v


class CriticallyDampedOdeReferenceTests(unittest.TestCase):
    """The closed-form step matches a numerical solution of its defining ODE."""

    def check_against_rk4(self, config: CriticallyDampedConfig, seed: int) -> None:
        rng = np.random.default_rng(seed)
        joint_filter = make_filter(config, 3)
        omega = joint_filter.omega
        # H(s) = w0^2/(s + w0)^2, so |H| = 1/(1 + (f/f0)^2) = 1/sqrt(2) at f/f0 = sqrt(sqrt(2) - 1).
        self.assertAlmostEqual(
            omega, 2.0 * np.pi * config.cutoff_hz / np.sqrt(np.sqrt(2.0) - 1.0), places=9
        )

        now = 0.0
        first = rng.uniform(-1.0, 1.0, size=3)
        output = joint_filter.step(first, now=now)
        np.testing.assert_array_equal(output.values, first)
        value, velocity = first.copy(), np.zeros(3)
        for _ in range(120):
            # Irregular source intervals, all inside max_reference_gap_s.
            dt = float(rng.uniform(0.2, 2.5) * SOURCE_PERIOD)
            now += dt
            sample = rng.uniform(-1.0, 1.0, size=3)
            output = joint_filter.step(sample, now=now)
            self.assertFalse(output.reinitialized)
            # The new sample is the target held constant over the interval.
            value, velocity = _rk4_critically_damped(value, velocity, sample, omega, dt)
            np.testing.assert_allclose(output.values, value, rtol=0.0, atol=1e-10)
            np.testing.assert_allclose(joint_filter._velocity, velocity, rtol=0.0, atol=1e-8)

    def test_closed_form_matches_rk4_at_the_default_cutoff(self):
        self.check_against_rk4(CriticallyDampedConfig(), seed=11)

    def test_closed_form_matches_rk4_at_low_and_high_cutoffs(self):
        self.check_against_rk4(CriticallyDampedConfig(cutoff_hz=0.5), seed=12)
        self.check_against_rk4(CriticallyDampedConfig(cutoff_hz=20.0), seed=13)

    def test_step_response_never_overshoots(self):
        joint_filter = make_filter(CriticallyDampedConfig(), 1)
        joint_filter.step(np.zeros(1), now=0.0)
        values = [
            float(joint_filter.step(np.ones(1), now=index * SOURCE_PERIOD).values[0])
            for index in range(1, 200)
        ]
        self.assertTrue(all(b >= a for a, b in zip(values, values[1:])))
        self.assertLessEqual(max(values), 1.0)
        self.assertGreater(values[-1], 0.999)


class UpsamplerTests(unittest.TestCase):
    def config(self, **overrides):
        base = dict(command_rate_hz=250.0, source_rate_hz=60.0, stall_after_s=0.10)
        base.update(overrides)
        return UpsamplerConfig(**base)

    def test_emits_the_expected_command_count_and_period(self):
        upsampler = CommandUpsampler(self.config(), 2)
        clock = 100.0
        upsampler.push(np.zeros(2), now=clock, source_timestamp=clock)
        commands = upsampler.pump(clock + 0.100)
        # 250 Hz over [t0, t0 + 0.1], including the tick at t0 itself.
        self.assertEqual(len(commands), 26)
        times = np.asarray([command.command_time for command in commands])
        np.testing.assert_allclose(np.diff(times), 0.004, rtol=0.0, atol=1e-9)
        self.assertAlmostEqual(times[0], clock)
        self.assertEqual(upsampler.pump(clock + 0.100), [])
        stats = upsampler.stats()
        self.assertEqual(stats.commands, 26)
        self.assertAlmostEqual(stats.achieved_rate_hz, 250.0, places=6)
        self.assertEqual(stats.dropped_ticks, 0)
        self.assertEqual(stats.references, 1)

    def test_interpolates_toward_the_newest_reference_without_extrapolating(self):
        upsampler = CommandUpsampler(self.config(), 1)
        clock = 10.0
        upsampler.push(np.zeros(1), now=clock, source_timestamp=clock)
        upsampler.pump(clock)
        clock += SOURCE_PERIOD
        upsampler.push(np.asarray([1.0]), now=clock, source_timestamp=clock)
        # One source period of ramp, then the segment saturates: the command
        # walks to the newest reference and never past it.
        commands = upsampler.pump(clock + 2 * SOURCE_PERIOD)
        values = [float(command.values[0]) for command in commands]
        self.assertTrue(all(0.0 <= value <= 1.0 for value in values), values)
        self.assertTrue(all(b >= a for a, b in zip(values, values[1:])), values)
        self.assertLess(values[0], 0.35)
        self.assertAlmostEqual(values[-1], 1.0, places=9)
        self.assertTrue(commands[-1].saturated)
        self.assertFalse(commands[0].saturated)
        self.assertGreater(commands[0].interpolation_lag_s, 0.0)
        self.assertAlmostEqual(commands[-1].interpolation_lag_s, 0.0)

    def test_holds_last_reference_and_flags_when_the_source_stalls(self):
        upsampler = CommandUpsampler(self.config(), 1)
        clock = 5.0
        upsampler.push(np.zeros(1), now=clock, source_timestamp=clock)
        upsampler.pump(clock)
        clock += SOURCE_PERIOD
        upsampler.push(np.asarray([0.5]), now=clock, source_timestamp=clock)
        source_time = clock

        commands = upsampler.pump(clock + 0.300)
        held = [command for command in commands if command.saturated]
        self.assertTrue(held)
        for command in held:
            self.assertAlmostEqual(float(command.values[0]), 0.5, places=9)
            self.assertEqual(command.source_timestamp, source_time)
        stalled = [command for command in commands if command.stalled]
        self.assertTrue(stalled)
        self.assertTrue(all(command.source_age_s >= 0.10 for command in stalled))
        self.assertTrue(
            all(
                not command.stalled
                for command in commands
                if command.source_age_s < 0.10
            )
        )
        self.assertGreater(upsampler.stats().stalled_commands, 0)
        self.assertIn("stall_flagged", upsampler.format_stats())

    def test_hold_mode_is_a_zero_order_hold(self):
        upsampler = CommandUpsampler(self.config(mode="hold"), 1)
        clock = 0.0
        upsampler.push(np.zeros(1), now=clock, source_timestamp=clock)
        upsampler.pump(clock)
        clock += SOURCE_PERIOD
        upsampler.push(np.asarray([1.0]), now=clock, source_timestamp=clock)
        commands = upsampler.pump(clock + SOURCE_PERIOD)
        self.assertTrue(all(float(command.values[0]) == 1.0 for command in commands))
        self.assertEqual(upsampler_lag_ms(self.config(mode="hold")), 0.0)
        self.assertAlmostEqual(upsampler_lag_ms(self.config()), 1000.0 / 60.0, places=6)

    def test_catch_up_after_a_long_pump_gap_is_bounded(self):
        upsampler = CommandUpsampler(self.config(max_commands_per_pump=8), 1)
        clock = 0.0
        upsampler.push(np.zeros(1), now=clock, source_timestamp=clock)
        commands = upsampler.pump(clock + 2.0)
        self.assertEqual(len(commands), 8)
        self.assertEqual(upsampler.stats().dropped_ticks, 501 - 8)
        self.assertAlmostEqual(commands[-1].command_time, clock + 2.0, places=9)

    def test_forwards_the_source_timestamp_and_reports_late_emission(self):
        upsampler = CommandUpsampler(self.config(), 1)
        upsampler.push(np.zeros(1), now=1.0, source_timestamp=0.980)
        commands = upsampler.pump(1.010)
        self.assertTrue(all(command.source_timestamp == 0.980 for command in commands))
        self.assertAlmostEqual(commands[0].source_age_s, 0.020, places=9)
        self.assertGreater(upsampler.stats().late_ms_max, 0.0)

    def test_lateness_history_is_bounded_but_totals_cover_the_whole_run(self):
        from unittest import mock

        with mock.patch("mocap_g1.live_filter.LATE_MS_WINDOW", 50):
            upsampler = CommandUpsampler(self.config(), 1)
        upsampler.push(np.zeros(1), now=0.0, source_timestamp=0.0)
        # One very late pump early on, then 1000 commands emitted on time.
        upsampler.pump(0.020)
        for tick in range(6, 1006):
            upsampler.pump(tick * upsampler.period_s)

        stats = upsampler.stats()
        self.assertEqual(len(upsampler._late_ms), 50)
        self.assertEqual(stats.commands, 1006)
        self.assertAlmostEqual(stats.late_ms_max, 20.0, places=6)
        self.assertLess(stats.late_ms_p95, 1e-6)
        self.assertAlmostEqual(stats.achieved_rate_hz, 250.0, places=6)

    def test_reset_clears_the_schedule(self):
        upsampler = CommandUpsampler(self.config(), 1)
        upsampler.push(np.zeros(1), now=0.0, source_timestamp=0.0)
        upsampler.pump(0.01)
        upsampler.reset()
        self.assertFalse(upsampler.has_reference)
        self.assertEqual(upsampler.pump(1.0), [])
        upsampler.push(np.asarray([0.3]), now=1.0, source_timestamp=1.0)
        commands = upsampler.pump(1.0)
        self.assertEqual(len(commands), 1)
        self.assertAlmostEqual(float(commands[0].values[0]), 0.3)

    def test_configuration_and_input_validation(self):
        with self.assertRaisesRegex(LiveFilterError, "at least source_rate_hz"):
            CommandUpsampler(self.config(command_rate_hz=30.0), 1)
        with self.assertRaisesRegex(LiveFilterError, "interpolate"):
            CommandUpsampler(self.config(mode="spline"), 1)
        upsampler = CommandUpsampler(self.config(), 2)
        with self.assertRaisesRegex(LiveFilterError, "shape"):
            upsampler.push(np.zeros(3), now=0.0, source_timestamp=0.0)
        with self.assertRaisesRegex(LiveFilterError, "non-finite"):
            upsampler.push(np.asarray([0.0, np.inf]), now=0.0, source_timestamp=0.0)
        upsampler.push(np.zeros(2), now=1.0, source_timestamp=1.0)
        with self.assertRaisesRegex(LiveFilterError, "backwards"):
            upsampler.push(np.zeros(2), now=0.5, source_timestamp=0.5)


class LatencyRecorderTests(unittest.TestCase):
    def timeline(self, recorder, base, offsets):
        timeline = recorder.open(base)
        for stage, offset in offsets.items():
            timeline.mark(stage, base + offset)
        return timeline

    def test_stage_and_end_to_end_statistics(self):
        recorder = LatencyRecorder()
        for index in range(10):
            recorder.commit(
                self.timeline(
                    recorder,
                    float(index),
                    {
                        "parse": 0.001,
                        "retarget": 0.011,
                        "filter": 0.0111,
                        "emit": 0.0131,
                    },
                )
            )
        recorder.note_algorithmic_lag(filter_lag_ms=26.0, upsampler_lag_ms=16.7)
        summary = recorder.summary()
        self.assertEqual(summary.frames, 10)
        self.assertAlmostEqual(summary.stages["parse"].mean_ms, 1.0, places=6)
        self.assertAlmostEqual(summary.stages["retarget"].mean_ms, 10.0, places=6)
        self.assertAlmostEqual(summary.stages["filter"].mean_ms, 0.1, places=6)
        self.assertAlmostEqual(summary.stages["emit"].mean_ms, 2.0, places=6)
        self.assertAlmostEqual(summary.end_to_end.p95_ms, 13.1, places=6)
        self.assertAlmostEqual(summary.algorithmic_lag_ms, 42.7, places=6)
        self.assertAlmostEqual(summary.budget_p95_ms, 55.8, places=6)
        report = recorder.format_report()
        self.assertIn("compute end-to-end", report)
        self.assertIn("algorithmic lag", report)

    def test_rejects_incomplete_and_out_of_order_timelines(self):
        recorder = LatencyRecorder()
        partial = recorder.open(0.0)
        partial.mark("parse", 0.001)
        with self.assertRaisesRegex(LiveFilterError, "missing stages"):
            recorder.commit(partial)
        with self.assertRaisesRegex(LiveFilterError, "unknown latency stage"):
            partial.mark("ik", 0.002)
        backwards = self.timeline(
            recorder,
            0.0,
            {"parse": 0.002, "retarget": 0.001, "filter": 0.003, "emit": 0.004},
        )
        with self.assertRaisesRegex(LiveFilterError, "earlier"):
            recorder.commit(backwards)

    def test_capacity_bounds_memory_and_reports_drops(self):
        recorder = LatencyRecorder(capacity=2)
        for index in range(5):
            recorder.commit(
                self.timeline(
                    recorder,
                    float(index),
                    {"parse": 0.001, "retarget": 0.002, "filter": 0.003, "emit": 0.004},
                )
            )
        self.assertEqual(recorder.frames, 2)
        self.assertEqual(recorder.dropped_records, 3)

    def test_empty_report_is_explicit(self):
        self.assertIn("no frames recorded", LatencyRecorder().format_report())
        self.assertIsNone(LatencyRecorder().summary().end_to_end)

    def test_mark_defaults_to_the_monotonic_clock(self):
        timeline = FrameTimeline(packet_monotonic=0.0)
        first = timeline.mark("parse")
        second = timeline.mark("retarget")
        self.assertGreaterEqual(second, first)


class SafetyCompositionTests(unittest.TestCase):
    """Conditioning must not weaken the envelope's clamp authority."""

    @classmethod
    def setUpClass(cls):
        cls.limits = load_official_joint_limits(
            REPO_ROOT / "eval/assets/g1_29dof_official.urdf"
        )

    def test_filtered_and_upsampled_commands_are_still_clamped(self):
        config = SafetyConfig(
            position_margin_rad=0.05,
            velocity_limit_fraction=0.25,
            acceleration_limit_rad_s2=4.0,
            stale_timeout_s=0.10,
            hard_timeout_s=0.50,
            blend_rate_per_s=1.0,
        )
        controller = LiveG1SafetyController(self.limits, config)
        joint_filter = make_filter(OneEuroConfig(), len(G1_29DOF_JOINTS))
        upsampler = CommandUpsampler(
            UpsamplerConfig(command_rate_hz=250.0, source_rate_hz=60.0),
            len(G1_29DOF_JOINTS),
        )

        measured = np.zeros(len(G1_29DOF_JOINTS))
        # Far outside every official URDF range, in both directions.
        reference = np.zeros(len(G1_29DOF_JOINTS))
        reference[12:] = 50.0
        reference[15] = -50.0

        clock = 1000.0
        controller.reset(measured, now=clock)
        clamped_seen = False
        for _ in range(120):
            clock += SOURCE_PERIOD
            filtered = joint_filter.step(reference, now=clock)
            upsampler.push(filtered.values, now=clock, source_timestamp=clock)
            commands = upsampler.pump(clock)
            self.assertTrue(commands)
            command = commands[-1]
            controller.update_target(
                command.values, source_timestamp=command.source_timestamp
            )
            output = controller.step(measured, now=clock, enable=True)
            clamped_seen |= bool(output.clamped_joints)
            measured = output.dof_pos

        self.assertTrue(clamped_seen)
        self.assertEqual(output.reason, "tracking")
        for name in G1_ARM_SDK_JOINTS:
            limit = self.limits[name]
            value = float(measured[G1_29DOF_JOINTS.index(name)])
            self.assertGreaterEqual(
                value, limit.lower + config.position_margin_rad - 1e-9
            )
            self.assertLessEqual(value, limit.upper - config.position_margin_rad + 1e-9)
            self.assertLessEqual(
                abs(output.velocities[name]),
                limit.velocity * config.velocity_limit_fraction + 1e-9,
            )

    def test_conditioning_cannot_hide_a_stale_source_from_the_envelope(self):
        controller = LiveG1SafetyController(self.limits, SafetyConfig())
        joint_filter = make_filter(OneEuroConfig(), len(G1_29DOF_JOINTS))
        upsampler = CommandUpsampler(UpsamplerConfig(), len(G1_29DOF_JOINTS))

        measured = np.zeros(len(G1_29DOF_JOINTS))
        reference = np.zeros(len(G1_29DOF_JOINTS))
        reference[12:] = 0.4
        clock = 500.0
        controller.reset(measured, now=clock)
        for _ in range(20):
            clock += SOURCE_PERIOD
            filtered = joint_filter.step(reference, now=clock)
            upsampler.push(filtered.values, now=clock, source_timestamp=clock)
            command = upsampler.pump(clock)[-1]
            controller.update_target(
                command.values, source_timestamp=command.source_timestamp
            )
            output = controller.step(measured, now=clock, enable=True)
            measured = output.dof_pos
        self.assertEqual(output.reason, "tracking")

        # The source stops.  The upsampler keeps emitting held commands, and
        # the envelope must still see the frozen source timestamp, decelerate,
        # and latch its hard timeout.
        reasons = []
        frozen = None
        for index in range(40):
            clock += SOURCE_PERIOD
            command = upsampler.pump(clock)[-1]
            if index > 0:
                # After the final ramp segment is consumed the command is
                # pinned to the last reference; it is never extrapolated.
                self.assertTrue(command.saturated)
                if frozen is None:
                    frozen = command.values.copy()
                np.testing.assert_allclose(command.values, frozen)
            controller.update_target(
                command.values, source_timestamp=command.source_timestamp
            )
            output = controller.step(measured, now=clock, enable=True)
            measured = output.dof_pos
            reasons.append(output.reason)
        self.assertIn("stale_source", reasons)
        self.assertIn("hard_timeout", reasons)
        self.assertTrue(output.reset_required)


if __name__ == "__main__":
    unittest.main()
