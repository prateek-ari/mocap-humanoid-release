"""Tests for the ``rt/arm_sdk`` bridge.  FakeTransport only -- no robot, no SDK.

The CRC known-answer vectors in :class:`CrcKnownAnswerTests` were produced by
executing upstream's own ``unitree_sdk2py.utils.crc.CRC._crc_py`` body (extracted
verbatim from the published source) over frames packed with upstream's own
``__packFmtHGLowCmd`` layout.  They are therefore real vectors for the published
algorithm rather than a self-consistency check of this port.  They have *not*
been compared against bytes captured from a physical G1; see the
UNVERIFIED UNTIL HARDWARE section of ``mocap_g1.arm_bridge``.
"""

from __future__ import annotations

import contextlib
import io
import json
from pathlib import Path
import random
import signal
import subprocess
import sys
import tempfile
import threading
import unittest
from unittest import mock

import numpy as np

from mocap_g1.arm_bridge import (
    ARM_SDK_MOTOR_INDEX,
    ARM_SDK_TOPIC,
    G1_MOTOR_INDEX,
    HG_LOWCMD_PACKED_BYTES,
    LOWSTATE_TOPIC,
    NUM_MOTOR_SLOTS,
    UPSTREAM_EXAMPLE_KD,
    UPSTREAM_EXAMPLE_KP,
    WEIGHT_CHANNEL_INDEX,
    ArmBridgeConfig,
    ArmBridgeError,
    DdsTransport,
    FakeTransport,
    G1ArmBridge,
    LowCmdFrame,
    LowStateSnapshot,
    MotorCommand,
    arm_targets_from_qpos,
    crc32_core,
    lowcmd_crc,
    lowcmd_crc_words,
    pack_hg_lowcmd,
)
from mocap_g1.safety import SafetyConfig
from mocap_g1.schema import G1_29DOF_JOINTS, G1_ARM_SDK_JOINTS, load_official_joint_limits
from tools.control import g1_arm_bridge as bridge_cli


REPO_ROOT = Path(__file__).resolve().parents[1]
URDF = REPO_ROOT / "eval/assets/g1_29dof_official.urdf"
CLI = REPO_ROOT / "tools/control/g1_arm_bridge.py"


def load_limits():
    return load_official_joint_limits(URDF)


def midpoints(limits) -> dict[str, float]:
    return {
        name: (limits[name].lower + limits[name].upper) / 2.0
        for name in G1_ARM_SDK_JOINTS
    }


def make_bridge(limits, *, config=None, safety_config=None):
    transport = FakeTransport()
    bridge = G1ArmBridge(
        transport,
        limits,
        config=config or ArmBridgeConfig(),
        safety_config=safety_config or SafetyConfig(),
    )
    return transport, bridge


# ---------------------------------------------------------------------------
# Motor index map
# ---------------------------------------------------------------------------


class MotorIndexMapTests(unittest.TestCase):
    def test_map_covers_the_canonical_order(self):
        self.assertEqual(len(G1_MOTOR_INDEX), 29)
        for position, name in enumerate(G1_29DOF_JOINTS):
            self.assertEqual(
                G1_MOTOR_INDEX[name],
                position,
                f"{name} must map to Unitree motor index {position}",
            )

    def test_arm_sdk_joints_are_motors_12_through_28(self):
        self.assertEqual(len(ARM_SDK_MOTOR_INDEX), 17)
        self.assertEqual(list(ARM_SDK_MOTOR_INDEX.values()), list(range(12, 29)))
        self.assertEqual(tuple(ARM_SDK_MOTOR_INDEX), G1_ARM_SDK_JOINTS)

    def test_weight_channel_is_not_a_joint(self):
        self.assertEqual(WEIGHT_CHANNEL_INDEX, 29)
        self.assertNotIn(WEIGHT_CHANNEL_INDEX, set(G1_MOTOR_INDEX.values()))

    def test_topics_and_slot_count(self):
        self.assertEqual(ARM_SDK_TOPIC, "rt/arm_sdk")
        self.assertEqual(LOWSTATE_TOPIC, "rt/lowstate")
        self.assertEqual(NUM_MOTOR_SLOTS, 35)

    def test_each_joint_lands_on_its_declared_motor_index(self):
        """Round-trip: a unique value per joint must appear at its motor slot."""
        limits = load_limits()
        _, bridge = make_bridge(limits)
        # A distinct, in-range position per joint so no two can be confused.
        positions = {}
        for offset, name in enumerate(G1_ARM_SDK_JOINTS):
            limit = limits[name]
            span = limit.upper - limit.lower
            positions[name] = limit.lower + span * (0.20 + 0.03 * offset)
        velocities = {
            name: 0.01 * (index + 1) for index, name in enumerate(G1_ARM_SDK_JOINTS)
        }

        frame = bridge.build_lowcmd(positions, velocities, weight=0.5)

        for name, motor_index in ARM_SDK_MOTOR_INDEX.items():
            motor = frame.motor_cmd[motor_index]
            self.assertAlmostEqual(motor.q, positions[name], places=12, msg=name)
            self.assertAlmostEqual(motor.dq, velocities[name], places=12, msg=name)
            self.assertEqual(frame.joint_q(name), positions[name])
        # And the inverse direction: nothing landed on a slot it does not own.
        commanded = set(ARM_SDK_MOTOR_INDEX.values())
        for motor_index in range(NUM_MOTOR_SLOTS):
            if motor_index in commanded:
                continue
            motor = frame.motor_cmd[motor_index]
            if motor_index == WEIGHT_CHANNEL_INDEX:
                self.assertEqual(motor.q, 0.5)
            else:
                self.assertEqual(motor.q, 0.0, f"motor slot {motor_index} was written")
            self.assertEqual(motor.kp, 0.0)
            self.assertEqual(motor.kd, 0.0)

    def test_leg_motors_are_never_commanded(self):
        limits = load_limits()
        _, bridge = make_bridge(limits)
        frame = bridge.build_lowcmd(midpoints(limits), weight=1.0)
        for name in G1_29DOF_JOINTS[:12]:
            motor = frame.motor_cmd[G1_MOTOR_INDEX[name]]
            self.assertEqual(
                (motor.q, motor.dq, motor.tau, motor.kp, motor.kd), (0.0,) * 5
            )
            self.assertEqual(motor.mode, 0)

    def test_qpos_slice_matches_the_repo_convention(self):
        row = np.zeros(36)
        row[3] = 1.0
        for motor_index in G1_MOTOR_INDEX.values():
            row[7 + motor_index] = motor_index + 0.5
        targets = arm_targets_from_qpos(row)
        self.assertEqual(set(targets), set(G1_ARM_SDK_JOINTS))
        for name, motor_index in ARM_SDK_MOTOR_INDEX.items():
            self.assertEqual(targets[name], motor_index + 0.5)
        with self.assertRaises(ArmBridgeError):
            arm_targets_from_qpos(np.zeros(35))


# ---------------------------------------------------------------------------
# CRC
# ---------------------------------------------------------------------------


def zeroed_lowcmd() -> LowCmdFrame:
    return LowCmdFrame(
        mode_pr=0,
        mode_machine=0,
        motor_cmd=[MotorCommand() for _ in range(NUM_MOTOR_SLOTS)],
    )


def reference_crc32_core(words) -> int:
    """The original bit-at-a-time port of upstream ``CRC._crc_py``.

    Kept verbatim as the reference the table-driven ``crc32_core`` must match.
    """
    crc = 0xFFFFFFFF
    for word in words:
        current = int(word) & 0xFFFFFFFF
        bit = 1 << 31
        for _ in range(32):
            if crc & 0x80000000:
                crc = (crc << 1) & 0xFFFFFFFF
                crc ^= 0x04C11DB7
            else:
                crc = (crc << 1) & 0xFFFFFFFF
            if current & bit:
                crc ^= 0x04C11DB7
            bit >>= 1
    return crc


class CrcKnownAnswerTests(unittest.TestCase):
    """Known answers derived from upstream ``CRC._crc_py`` (see module docstring)."""

    def test_pack_format_is_1004_bytes(self):
        self.assertEqual(HG_LOWCMD_PACKED_BYTES, 1004)
        self.assertEqual(len(pack_hg_lowcmd(zeroed_lowcmd())), 1004)

    def test_raw_word_vectors(self):
        # Structure-independent vectors for crc32_core itself.
        self.assertEqual(crc32_core([]), 0xFFFFFFFF)
        self.assertEqual(crc32_core([0, 1, 2, 3]), 0x3874616D)
        self.assertEqual(crc32_core([0xFFFFFFFF]), 0x00000000)

    def test_reference_bit_loop_reproduces_the_known_answers(self):
        self.assertEqual(reference_crc32_core([]), 0xFFFFFFFF)
        self.assertEqual(reference_crc32_core([0, 1, 2, 3]), 0x3874616D)
        self.assertEqual(reference_crc32_core([0xFFFFFFFF]), 0x00000000)
        self.assertEqual(
            reference_crc32_core(lowcmd_crc_words(pack_hg_lowcmd(zeroed_lowcmd()))),
            0xFE172F9F,
        )

    def test_table_crc_matches_the_reference_bit_loop(self):
        rng = random.Random(20261012)
        edge_words = [0, 1, 0x80000000, 0x7FFFFFFF, 0xFFFFFFFF, 0x04C11DB7]
        cases = [edge_words, [edge_words[-1]] * 7]
        for length in (1, 2, 3, 5, 17, 64, 250):
            for _ in range(8):
                cases.append([rng.getrandbits(32) for _ in range(length)])
        for words in cases:
            with self.subTest(length=len(words), head=words[:2]):
                self.assertEqual(crc32_core(words), reference_crc32_core(words))

        # Whole packed frames with random commanded fields, as built on the wire.
        for _ in range(20):
            frame = zeroed_lowcmd()
            frame.mode_pr = rng.randrange(2)
            frame.mode_machine = rng.randrange(256)
            for motor in frame.motor_cmd:
                motor.mode = rng.randrange(256)
                motor.q = rng.uniform(-3.0, 3.0)
                motor.dq = rng.uniform(-10.0, 10.0)
                motor.kp = rng.uniform(0.0, 60.0)
                motor.kd = rng.uniform(0.0, 1.5)
            words = lowcmd_crc_words(pack_hg_lowcmd(frame))
            self.assertEqual(lowcmd_crc(frame), reference_crc32_core(words))

    def test_all_zero_lowcmd_vector(self):
        self.assertEqual(lowcmd_crc(zeroed_lowcmd()), 0xFE172F9F)

    def test_weight_channel_only_vector(self):
        frame = zeroed_lowcmd()
        frame.motor_cmd[WEIGHT_CHANNEL_INDEX].q = 1.0
        self.assertEqual(lowcmd_crc(frame), 0xC6E5768D)

    def test_arm_sdk_shaped_frame_vector(self):
        frame = zeroed_lowcmd()
        frame.mode_machine = 4
        for motor_index in range(12, 29):
            motor = frame.motor_cmd[motor_index]
            motor.mode = 1
            motor.q = motor_index / 100.0
            motor.kp = 60.0
            motor.kd = 1.5
        frame.motor_cmd[WEIGHT_CHANNEL_INDEX].q = 1.0
        self.assertEqual(lowcmd_crc(frame), 0xFA64B1CA)

    def test_crc_ignores_the_existing_crc_field(self):
        """__Trans drops the trailing word, so the crc must not feed itself."""
        frame = zeroed_lowcmd()
        baseline = lowcmd_crc(frame)
        frame.crc = 0xDEADBEEF
        self.assertEqual(lowcmd_crc(frame), baseline)

    def test_crc_is_sensitive_to_every_commanded_field(self):
        base = lowcmd_crc(zeroed_lowcmd())
        mutations = (
            lambda f: setattr(f, "mode_pr", 1),
            lambda f: setattr(f, "mode_machine", 3),
            lambda f: setattr(f.motor_cmd[12], "q", 0.001),
            lambda f: setattr(f.motor_cmd[28], "kd", 1.5),
            lambda f: setattr(f.motor_cmd[WEIGHT_CHANNEL_INDEX], "q", 1.0),
        )
        for mutate in mutations:
            frame = zeroed_lowcmd()
            mutate(frame)
            self.assertNotEqual(lowcmd_crc(frame), base)

    def test_build_lowcmd_stamps_a_matching_crc(self):
        limits = load_limits()
        _, bridge = make_bridge(limits)
        frame = bridge.build_lowcmd(midpoints(limits), weight=0.25)
        self.assertNotEqual(frame.crc, 0)
        self.assertEqual(frame.crc, lowcmd_crc(frame))


# ---------------------------------------------------------------------------
# Arming from measured state
# ---------------------------------------------------------------------------


class ArmingTests(unittest.TestCase):
    def setUp(self):
        self.limits = load_limits()

    def test_arm_refuses_without_measured_state(self):
        _, bridge = make_bridge(self.limits)
        with self.assertRaises(ArmBridgeError) as ctx:
            bridge.arm(now=0.0)
        self.assertIn("LowState", str(ctx.exception))
        self.assertFalse(bridge.armed)

    def test_arm_refuses_stale_measured_state(self):
        transport, bridge = make_bridge(self.limits)
        transport.feed_arm_positions(midpoints(self.limits), receive_time_s=0.0)
        with self.assertRaises(ArmBridgeError):
            bridge.arm(now=5.0)

    def test_arm_refuses_state_from_a_different_future_clock(self):
        transport, bridge = make_bridge(self.limits)
        transport.feed_arm_positions(midpoints(self.limits), receive_time_s=1_000_000.0)
        with self.assertRaisesRegex(ArmBridgeError, "clocks do not match"):
            bridge.arm(now=0.0)
        self.assertFalse(bridge.armed)

    def test_future_clock_tolerance_must_be_smaller_than_stale_timeout(self):
        with self.assertRaisesRegex(ArmBridgeError, "state_future_tolerance_s"):
            make_bridge(
                self.limits,
                config=ArmBridgeConfig(
                    state_stale_timeout_s=0.1,
                    state_future_tolerance_s=0.1,
                ),
            )

    def test_state_timeouts_must_be_finite_and_positive(self):
        invalid_configs = (
            ArmBridgeConfig(state_stale_timeout_s=float("nan")),
            ArmBridgeConfig(state_stale_timeout_s=float("inf")),
            ArmBridgeConfig(state_hard_timeout_s=float("inf")),
            ArmBridgeConfig(state_stale_timeout_s=0.0),
            ArmBridgeConfig(state_hard_timeout_s=-1.0),
            # Equal timeouts leave a zero-length stale ramp-down window.
            ArmBridgeConfig(state_stale_timeout_s=0.5, state_hard_timeout_s=0.5),
        )
        for config in invalid_configs:
            with self.subTest(config=config):
                with self.assertRaisesRegex(ArmBridgeError, "state timeouts must be finite"):
                    make_bridge(self.limits, config=config)

    def test_mode_fields_reject_boolean_and_fractional_coercion(self):
        invalid_configs = (
            ArmBridgeConfig(motor_mode=True),
            ArmBridgeConfig(motor_mode=1.5),
            ArmBridgeConfig(mode_pr=True),
            ArmBridgeConfig(mode_pr=0.5),
            ArmBridgeConfig(mode_pr=2),
            ArmBridgeConfig(mode_machine_override=True),
            ArmBridgeConfig(mode_machine_override=9.5),
        )
        for config in invalid_configs:
            with self.subTest(config=config):
                with self.assertRaisesRegex(ArmBridgeError, "must be"):
                    config.validate()

    def test_real_valued_fields_reject_boolean_coercion(self):
        invalid_configs = (
            ArmBridgeConfig(kp=True),
            ArmBridgeConfig(kd=False),
            ArmBridgeConfig(tau_ff=True),
            ArmBridgeConfig(state_stale_timeout_s=True),
            ArmBridgeConfig(state_hard_timeout_s=True),
            ArmBridgeConfig(state_future_tolerance_s=False),
        )
        for config in invalid_configs:
            with self.subTest(config=config):
                with self.assertRaises(ArmBridgeError):
                    config.validate()

    def test_reset_uses_measured_lowstate_q_not_midpoint_defaults(self):
        """envelope.reset must receive encoder values, not neutral defaults."""
        transport, bridge = make_bridge(self.limits)

        measured = {}
        for offset, name in enumerate(G1_ARM_SDK_JOINTS):
            limit = self.limits[name]
            span = limit.upper - limit.lower
            # Deliberately far from the midpoint and distinct per joint.
            measured[name] = limit.lower + span * (0.15 + 0.02 * offset)
        neutral = midpoints(self.limits)
        self.assertTrue(
            all(abs(measured[n] - neutral[n]) > 1e-3 for n in G1_ARM_SDK_JOINTS),
            "test fixture must differ from the midpoint defaults",
        )

        # Populate the whole 35-slot array so a wrong index would pick up a
        # different (leg) value rather than silently agreeing.
        q = [0.0] * NUM_MOTOR_SLOTS
        for motor_index in range(12):
            q[motor_index] = -1.0 - motor_index  # leg encoders, must be ignored
        for name, motor_index in ARM_SDK_MOTOR_INDEX.items():
            q[motor_index] = measured[name]
        transport.feed_state(
            LowStateSnapshot(receive_time_s=0.0, q=tuple(q), mode_machine=4)
        )

        bridge.arm(now=0.0)

        self.assertEqual(bridge.arm_count, 1)
        recorded = bridge.reset_positions[0]
        self.assertEqual(set(recorded), set(G1_ARM_SDK_JOINTS))
        for name in G1_ARM_SDK_JOINTS:
            self.assertAlmostEqual(recorded[name], measured[name], places=12, msg=name)
            self.assertNotAlmostEqual(recorded[name], neutral[name], places=4, msg=name)

        # The very first emitted command starts from the measured pose, not the
        # midpoint, and mode_machine was echoed from the observed LowState_.
        step = bridge.step(measured, source_timestamp=0.0, now=0.02, enable=True)
        self.assertEqual(step.frame.mode_machine, 4)
        for name in G1_ARM_SDK_JOINTS:
            self.assertAlmostEqual(step.frame.joint_q(name), measured[name], places=6)

    def test_mode_machine_override_beats_the_observed_value(self):
        transport, bridge = make_bridge(
            self.limits, config=ArmBridgeConfig(mode_machine_override=9)
        )
        transport.feed_arm_positions(
            midpoints(self.limits), receive_time_s=0.0, mode_machine=4
        )
        bridge.arm(now=0.0)
        step = bridge.step(midpoints(self.limits), source_timestamp=0.0, now=0.02)
        self.assertEqual(step.frame.mode_machine, 9)

    def test_out_of_range_measured_state_is_rejected(self):
        transport, bridge = make_bridge(self.limits)
        bad = midpoints(self.limits)
        bad["left_elbow_joint"] = 100.0
        transport.feed_arm_positions(bad, receive_time_s=0.0)
        with self.assertRaises(ArmBridgeError):
            bridge.arm(now=0.0)
        self.assertFalse(bridge.armed)


# ---------------------------------------------------------------------------
# Weight channel
# ---------------------------------------------------------------------------


class WeightChannelTests(unittest.TestCase):
    def setUp(self):
        self.limits = load_limits()
        self.safety = SafetyConfig()
        self.transport, self.bridge = make_bridge(self.limits, safety_config=self.safety)
        self.neutral = midpoints(self.limits)
        self.dt = 0.02

    def _feed(self, positions, now):
        self.transport.feed_arm_positions(positions, receive_time_s=now)

    def _run(self, ticks, *, start_tick, enable=True):
        """Advance the bridge, echoing commands back as simulated measured state."""
        weights = []
        reasons = []
        for offset in range(ticks):
            now = (start_tick + offset) * self.dt
            step = self.bridge.step(
                self.neutral, source_timestamp=now, now=now, enable=enable
            )
            weights.append(step.weight)
            reasons.append(step.reason)
            if step.command is not None:
                self._feed(step.command.positions, now)
        return weights, reasons

    def test_weight_follows_blend_weight_through_arm_disarm_rearm(self):
        self._feed(self.neutral, 0.0)
        self.bridge.arm(now=0.0)

        # --- soft start: weight ramps up at blend_rate_per_s, from zero -------
        # blend_rate_per_s=1.0 at dt=0.02 needs 50 ticks to reach full authority.
        ramp_ticks = int(round(1.0 / (self.safety.blend_rate_per_s * self.dt)))
        weights, reasons = self._run(ramp_ticks, start_tick=1)
        self.assertEqual(set(reasons), {"tracking"})
        self.assertAlmostEqual(
            weights[0], self.safety.blend_rate_per_s * self.dt, places=9
        )
        self.assertTrue(
            all(b >= a for a, b in zip(weights, weights[1:])), "ramp must be monotonic"
        )
        self.assertLess(weights[0], 0.1, "authority must not appear instantly")
        self.assertAlmostEqual(weights[-1], 1.0, places=9)
        # Every published frame carried the envelope's blend weight verbatim.
        for frame, weight in zip(self.transport.published, weights):
            self.assertAlmostEqual(
                frame.motor_cmd[WEIGHT_CHANNEL_INDEX].q, weight, places=9
            )

        # --- operator disable: weight ramps back down, never jumps -----------
        disable_start = 1 + ramp_ticks
        down, reasons = self._run(ramp_ticks + 5, start_tick=disable_start, enable=False)
        self.assertEqual(set(reasons), {"disabled"})
        self.assertTrue(
            all(b <= a for a, b in zip(down, down[1:])), "ramp-down must be monotonic"
        )
        self.assertAlmostEqual(
            down[0], 1.0 - self.safety.blend_rate_per_s * self.dt, places=9
        )
        self.assertEqual(down[-1], 0.0)

        # --- explicit disarm: immediate weight 0, tracking refused ----------
        disarm_tick = disable_start + ramp_ticks + 5
        released = self.bridge.disarm(now=disarm_tick * self.dt)
        self.assertEqual(released.weight, 0.0)
        self.assertEqual(released.reason, "disarmed")
        self.assertFalse(self.bridge.armed)
        held_tick = disarm_tick + 1
        held = self.bridge.step(
            self.neutral, source_timestamp=held_tick * self.dt, now=held_tick * self.dt
        )
        self.assertEqual(held.weight, 0.0)
        self.assertEqual(held.reason, "disarmed")
        self.assertIsNone(held.command)

        # --- rearm: fresh measured state, weight ramps from zero again ------
        rearm_tick = held_tick + 1
        rearm_now = rearm_tick * self.dt
        self._feed(self.neutral, rearm_now)
        self.bridge.arm(now=rearm_now)
        self.assertEqual(self.bridge.arm_count, 2)
        again, reasons = self._run(5, start_tick=rearm_tick + 1)
        self.assertEqual(set(reasons), {"tracking"})
        self.assertAlmostEqual(
            again[0], self.safety.blend_rate_per_s * self.dt, places=9
        )
        self.assertTrue(all(b >= a for a, b in zip(again, again[1:])))
        self.assertLess(again[-1], 0.2, "rearm must not restore full authority instantly")

    def test_weight_is_clamped_into_the_unit_interval(self):
        _, bridge = make_bridge(self.limits)
        self.assertEqual(bridge.build_lowcmd(self.neutral, weight=4.0).weight, 1.0)
        self.assertEqual(bridge.build_lowcmd(self.neutral, weight=-2.0).weight, 0.0)
        with self.assertRaises(ArmBridgeError):
            bridge.build_lowcmd(self.neutral, weight=float("nan"))


# ---------------------------------------------------------------------------
# Fail-closed on stale measured state
# ---------------------------------------------------------------------------


class StaleStateTests(unittest.TestCase):
    def setUp(self):
        self.limits = load_limits()
        self.safety = SafetyConfig()
        self.config = ArmBridgeConfig()
        self.transport, self.bridge = make_bridge(
            self.limits, config=self.config, safety_config=self.safety
        )
        self.neutral = midpoints(self.limits)
        # A target well away from neutral, so a "hold" is distinguishable from
        # continued tracking.
        self.target = {name: value + 0.30 for name, value in self.neutral.items()}
        self.dt = 0.02

    def test_stale_lowstate_fails_closed_to_zero_weight_and_holds(self):
        self.transport.feed_arm_positions(self.neutral, receive_time_s=0.0)
        self.bridge.arm(now=0.0)

        # Reach full authority while state keeps arriving.
        for tick in range(1, 61):
            now = tick * self.dt
            step = self.bridge.step(self.target, source_timestamp=now, now=now)
            self.transport.feed_arm_positions(step.command.positions, receive_time_s=now)
        self.assertAlmostEqual(step.weight, 1.0, places=9)
        self.assertEqual(step.reason, "tracking")
        engaged_positions = dict(step.command.positions)

        # Now freeze the measured-state stream.  The command source stays fresh,
        # so only LowState_ staleness can be responsible for what follows.
        weights = []
        reasons = []
        positions = []
        for tick in range(61, 141):
            now = tick * self.dt
            step = self.bridge.step(self.target, source_timestamp=now, now=now)
            weights.append(step.weight)
            reasons.append(step.reason)
            if step.command is not None:
                positions.append(dict(step.command.positions))

        # Staleness is detected on schedule, then escalates to a latched rearm
        # requirement.  The last state arrived one tick before this loop began,
        # so the deadlines land N-1 samples in.
        self.assertIn("stale_state", reasons)
        self.assertIn("state_hard_timeout", reasons)
        self.assertEqual(reasons[0], "tracking", "fresh state must still track")
        self.assertEqual(
            reasons.index("stale_state"),
            int(round(self.config.state_stale_timeout_s / self.dt)) - 1,
        )
        self.assertEqual(
            reasons.index("state_hard_timeout"),
            int(round(self.config.state_hard_timeout_s / self.dt)) - 1,
        )
        self.assertLess(reasons.index("stale_state"), reasons.index("state_hard_timeout"))
        self.assertTrue(all(b <= a for a, b in zip(weights, weights[1:])), weights)
        self.assertEqual(weights[-1], 0.0)
        self.assertTrue(self.bridge.rearm_required)
        self.assertFalse(self.bridge.armed)

        # Hold, not track: no joint kept advancing toward the far-away target.
        for name in G1_ARM_SDK_JOINTS:
            travel = abs(positions[-1][name] - engaged_positions[name])
            self.assertLess(travel, 0.05, f"{name} kept tracking while state was stale")

        # The last published frame is a released one.
        last = self.transport.published[-1]
        self.assertEqual(last.motor_cmd[WEIGHT_CHANNEL_INDEX].q, 0.0)

    def test_latched_bridge_refuses_to_track_until_rearmed(self):
        self.transport.feed_arm_positions(self.neutral, receive_time_s=0.0)
        self.bridge.arm(now=0.0)
        for tick in range(1, 6):
            now = tick * self.dt
            step = self.bridge.step(self.target, source_timestamp=now, now=now)
            self.transport.feed_arm_positions(step.command.positions, receive_time_s=now)

        # Jump past the hard state timeout without delivering state.
        far = 5 * self.dt + self.config.state_hard_timeout_s
        step = self.bridge.step(self.target, source_timestamp=far, now=far)
        self.assertEqual(step.reason, "state_hard_timeout")
        self.assertEqual(step.weight, 0.0)
        self.assertTrue(self.bridge.rearm_required)

        # Even with state flowing again, tracking stays refused until arm().
        resumed = far + self.dt
        self.transport.feed_arm_positions(self.neutral, receive_time_s=resumed)
        step = self.bridge.step(self.target, source_timestamp=resumed, now=resumed)
        self.assertEqual(step.reason, "rearm_required")
        self.assertEqual(step.weight, 0.0)

        self.bridge.arm(now=resumed)
        self.assertFalse(self.bridge.rearm_required)
        after = resumed + self.dt
        step = self.bridge.step(self.target, source_timestamp=after, now=after)
        self.assertEqual(step.reason, "tracking")
        self.assertGreater(step.weight, 0.0)

    def test_control_loop_stall_fails_closed_instead_of_raising(self):
        """A SafetyError from the envelope must become a released frame."""
        self.transport.feed_arm_positions(self.neutral, receive_time_s=0.0)
        self.bridge.arm(now=0.0)
        for tick in range(1, 6):
            now = tick * self.dt
            step = self.bridge.step(self.target, source_timestamp=now, now=now)
            self.transport.feed_arm_positions(step.command.positions, receive_time_s=now)

        # A control gap at/above the envelope's hard timeout while measured
        # state is still fresh: the envelope raises, and the bridge must convert
        # that into weight 0 rather than propagating.
        stall_now = 5 * self.dt + self.safety.hard_timeout_s
        self.transport.feed_arm_positions(self.neutral, receive_time_s=stall_now)
        step = self.bridge.step(self.target, source_timestamp=stall_now, now=stall_now)
        self.assertTrue(step.reason.startswith("safety_fault"))
        self.assertEqual(step.weight, 0.0)
        self.assertTrue(self.bridge.rearm_required)

    def test_future_state_while_armed_releases_and_requires_rearm(self):
        self.transport.feed_arm_positions(self.neutral, receive_time_s=0.0)
        self.bridge.arm(now=0.0)
        self.transport.feed_arm_positions(self.neutral, receive_time_s=1_000_000.0)

        step = self.bridge.step(self.target, source_timestamp=0.02, now=0.02)

        self.assertEqual(step.reason, "state_clock_invalid")
        self.assertLess(step.state_age_s, 0.0)
        self.assertEqual(step.weight, 0.0)
        self.assertTrue(step.rearm_required)
        self.assertFalse(step.armed)

    def test_stale_command_source_latches_rearm_and_releases(self):
        self.transport.feed_arm_positions(self.neutral, receive_time_s=0.0)
        self.bridge.arm(now=0.0)
        for tick in range(1, 6):
            now = tick * self.dt
            step = self.bridge.step(self.target, source_timestamp=now, now=now)
            self.transport.feed_arm_positions(step.command.positions, receive_time_s=now)
        self.assertEqual(step.reason, "tracking")
        held_positions = dict(step.command.positions)

        # Measured state stays fresh; only the command source is stale, by far
        # more than the envelope's hard timeout.
        now = 6 * self.dt
        self.transport.feed_arm_positions(held_positions, receive_time_s=now)
        published_before = len(self.transport.published)
        step = self.bridge.step(
            self.target, source_timestamp=now - 10.0 * self.safety.hard_timeout_s, now=now
        )
        self.assertEqual(step.reason, "source_hard_timeout")
        self.assertEqual(step.weight, 0.0)
        self.assertFalse(step.engaged)
        self.assertIsNone(step.command)
        self.assertTrue(step.rearm_required)
        self.assertFalse(step.armed)
        self.assertTrue(self.bridge.rearm_required)
        self.assertEqual(len(self.transport.published), published_before + 1)
        released = self.transport.published[-1]
        self.assertEqual(released.weight, 0.0)
        self.assertEqual(released.crc, lowcmd_crc(released))
        for name in G1_ARM_SDK_JOINTS:
            self.assertEqual(released.joint_q(name), held_positions[name])

        # A fresh source alone does not resume tracking; arm() does.
        now += self.dt
        self.transport.feed_arm_positions(held_positions, receive_time_s=now)
        step = self.bridge.step(self.target, source_timestamp=now, now=now)
        self.assertEqual(step.reason, "rearm_required")
        self.assertEqual(step.weight, 0.0)
        self.bridge.arm(now=now)
        now += self.dt
        step = self.bridge.step(self.target, source_timestamp=now, now=now)
        self.assertEqual(step.reason, "tracking")
        self.assertGreater(step.weight, 0.0)

    def test_disarm_with_out_of_range_state_while_unarmed_publishes_inert_frame(self):
        # Measured state is only range-checked by arm(); release must not
        # raise when it builds from an unvalidated snapshot.
        bad = dict(self.neutral)
        bad["left_elbow_joint"] = 100.0
        self.transport.feed_arm_positions(bad, receive_time_s=0.0)

        stepped = self.bridge.step(self.neutral, source_timestamp=0.0, now=0.0)
        self.assertEqual(stepped.reason, "disarmed")

        released = self.bridge.disarm(now=0.02)
        self.assertEqual(released.reason, "disarmed")
        self.assertEqual(released.weight, 0.0)
        self.assertFalse(released.armed)
        self.assertEqual(
            len(self.transport.published), 1 + self.config.release_repeat_count
        )
        for frame in (stepped.frame, released.frame, self.transport.published[-1]):
            self.assertEqual(frame.weight, 0.0)
            self.assertEqual(frame.crc, lowcmd_crc(frame))
            for name in G1_ARM_SDK_JOINTS:
                motor = frame.motor_cmd[ARM_SDK_MOTOR_INDEX[name]]
                self.assertEqual((motor.kp, motor.kd), (0.0, 0.0))

    def test_tick_reads_one_lowstate_snapshot(self):
        class CountingTransport(FakeTransport):
            reads = 0

            def latest_state(self):
                self.reads += 1
                return super().latest_state()

        transport = CountingTransport()
        bridge = G1ArmBridge(transport, self.limits, config=self.config)
        transport.feed_arm_positions(self.neutral, receive_time_s=0.0)
        bridge.arm(now=0.0)
        for tick in range(1, 4):
            now = tick * self.dt
            transport.reads = 0
            step = bridge.step(self.target, source_timestamp=now, now=now)
            self.assertEqual(step.reason, "tracking")
            self.assertEqual(transport.reads, 1)
            transport.feed_arm_positions(step.command.positions, receive_time_s=now)

    def test_no_state_at_all_publishes_an_inert_frame(self):
        _, bridge = make_bridge(self.limits)
        step = bridge.step(self.neutral, source_timestamp=0.0, now=0.0)
        self.assertEqual(step.reason, "disarmed")
        self.assertEqual(step.weight, 0.0)
        for name in G1_ARM_SDK_JOINTS:
            motor = step.frame.motor_cmd[ARM_SDK_MOTOR_INDEX[name]]
            self.assertEqual((motor.kp, motor.kd), (0.0, 0.0))


# ---------------------------------------------------------------------------
# Transport plumbing
# ---------------------------------------------------------------------------


class TransportTests(unittest.TestCase):
    def test_published_frames_are_snapshots(self):
        limits = load_limits()
        transport, bridge = make_bridge(limits)
        neutral = midpoints(limits)
        transport.feed_arm_positions(neutral, receive_time_s=0.0)
        bridge.arm(now=0.0)
        for tick in range(1, 4):
            now = tick * 0.02
            step = bridge.step(neutral, source_timestamp=now, now=now)
            transport.feed_arm_positions(step.command.positions, receive_time_s=now)
        self.assertEqual(len(transport.published), 3)
        self.assertEqual(len(transport.published_bytes), 3)
        self.assertTrue(all(len(raw) == 1004 for raw in transport.published_bytes))
        weights = [f.motor_cmd[WEIGHT_CHANNEL_INDEX].q for f in transport.published]
        self.assertEqual(weights, sorted(weights))
        self.assertNotEqual(weights[0], weights[-1])

    def test_dds_backend_is_not_required_to_import_or_test(self):
        self.assertIsNone(sys.modules.get("unitree_sdk2py"))
        self.assertIsNone(sys.modules.get("cyclonedds"))

    def test_dds_transport_names_the_robot_extra_when_sdk_is_missing(self):
        try:
            import unitree_sdk2py  # noqa: F401
        except ImportError:
            pass
        else:  # pragma: no cover - only on a machine with the SDK installed
            self.skipTest("unitree_sdk2py is installed")
        from mocap_g1.arm_bridge import DdsTransport

        with self.assertRaises(ArmBridgeError) as ctx:
            DdsTransport().connect()
        message = str(ctx.exception)
        self.assertIn("robot", message)
        self.assertIn("unitree_sdk2py", message)

    def test_dds_connect_closes_partially_initialized_handles(self):
        class Handle:
            def __init__(self, *, fail_init: bool = False):
                self.fail_init = fail_init
                self.closed = False

            def Init(self, *_args):
                if self.fail_init:
                    raise RuntimeError("subscriber init failed")

            def Close(self):
                self.closed = True

        publisher = Handle()
        subscriber = Handle(fail_init=True)
        sdk = (
            lambda *_args: None,
            lambda *_args: publisher,
            lambda *_args: subscriber,
            (object(), object()),
            object,
        )
        transport = DdsTransport()

        with mock.patch.object(DdsTransport, "_import_sdk", return_value=sdk):
            with self.assertRaisesRegex(ArmBridgeError, "failed to initialize DDS"):
                transport.connect(clock=lambda: 0.0)

        self.assertTrue(publisher.closed)
        self.assertTrue(subscriber.closed)
        self.assertIsNone(transport._publisher)
        self.assertIsNone(transport._subscriber)

    def test_lowstate_snapshot_validates_its_shape(self):
        with self.assertRaises(ArmBridgeError):
            LowStateSnapshot(receive_time_s=0.0, q=(0.0,) * 10)
        with self.assertRaises(ArmBridgeError):
            LowStateSnapshot(
                receive_time_s=0.0, q=(float("nan"),) + (0.0,) * (NUM_MOTOR_SLOTS - 1)
            )
        with self.assertRaises(ArmBridgeError):
            LowStateSnapshot.from_arm_positions(
                {"left_hip_pitch_joint": 0.0}, receive_time_s=0.0
            )


# ---------------------------------------------------------------------------
# CLI smoke
# ---------------------------------------------------------------------------


class DdsReplayPacingTests(unittest.TestCase):
    class Clock:
        def __init__(self):
            self.now = 0.0
            self.sleep_calls: list[float] = []

        def __call__(self) -> float:
            return self.now

        def sleep(self, delay: float) -> None:
            self.sleep_calls.append(delay)
            self.now += delay

    def test_missed_samples_are_skipped_instead_of_burst_published(self):
        clock = self.Clock()
        ticks = bridge_cli.iter_dds_replay_ticks(
            steps=5,
            control_dt=0.02,
            start_time=0.0,
            clock=clock,
            sleeper=clock.sleep,
        )

        first = next(ticks)
        clock.now += 0.05
        remaining = list(ticks)
        emitted = [first, *remaining]

        self.assertEqual([entry.tick for entry in emitted], [1, 3, 4, 5])
        self.assertEqual([entry.skipped_samples for entry in emitted], [0, 1, 0, 0])
        self.assertTrue(all(delay > 0.0 for delay in clock.sleep_calls))

    def test_100ms_deadline_overrun_aborts_without_an_overdue_tick(self):
        clock = self.Clock()
        ticks = bridge_cli.iter_dds_replay_ticks(
            steps=20,
            control_dt=0.02,
            start_time=0.0,
            clock=clock,
            sleeper=clock.sleep,
        )

        self.assertEqual(next(ticks).tick, 1)
        clock.now += bridge_cli.DDS_MAX_CONTROL_OVERRUN_S + 0.021
        with self.assertRaisesRegex(ArmBridgeError, "deadline overrun.*aborting"):
            next(ticks)

    def test_early_sleep_return_is_retried_until_the_deadline(self):
        clock = self.Clock()

        def early_sleep(delay: float) -> None:
            clock.sleep_calls.append(delay)
            clock.now += min(delay, 0.007)

        emitted = list(
            bridge_cli.iter_dds_replay_ticks(
                steps=2,
                control_dt=0.02,
                start_time=0.0,
                clock=clock,
                sleeper=early_sleep,
            )
        )

        self.assertEqual([entry.tick for entry in emitted], [1, 2])
        self.assertGreater(len(clock.sleep_calls), len(emitted))
        self.assertTrue(
            all(entry.now_s >= entry.replay_time_s for entry in emitted)
        )

    def test_interrupted_sleep_aborts_instead_of_publishing_early(self):
        clock = self.Clock()

        def interrupted_sleep(_delay: float) -> None:
            raise InterruptedError("test signal")

        ticks = bridge_cli.iter_dds_replay_ticks(
            steps=1,
            control_dt=0.02,
            start_time=0.0,
            clock=clock,
            sleeper=interrupted_sleep,
        )
        with self.assertRaisesRegex(ArmBridgeError, "sleep was interrupted.*aborting"):
            next(ticks)

    def test_sleep_without_clock_progress_aborts(self):
        clock = self.Clock()
        ticks = bridge_cli.iter_dds_replay_ticks(
            steps=1,
            control_dt=0.02,
            start_time=0.0,
            clock=clock,
            sleeper=lambda _delay: None,
        )
        with self.assertRaisesRegex(ArmBridgeError, "clock did not advance.*aborting"):
            next(ticks)

    def test_scheduler_rejects_nonfinite_timing_inputs(self):
        clock = self.Clock()
        with self.assertRaisesRegex(ArmBridgeError, "control_dt"):
            next(
                bridge_cli.iter_dds_replay_ticks(
                    steps=1,
                    control_dt=float("inf"),
                    start_time=0.0,
                    clock=clock,
                    sleeper=clock.sleep,
                )
            )


def write_synthetic_npz(path: Path, *, frames: int = 40, fps: float = 50.0) -> Path:
    """A tiny valid preview-cache NPZ: exactly {metadata, qpos, fps}."""
    qpos = np.zeros((frames, 36), dtype=np.float64)
    qpos[:, 3] = 1.0  # unit root quaternion, wxyz
    time = np.arange(frames) / fps
    for column in range(17):
        qpos[:, 7 + 12 + column] = 0.10 * np.sin(
            2.0 * np.pi * 0.25 * time + 0.05 * column
        )
    metadata = json.dumps(
        {
            "robot": "unitree_g1",
            "root_quaternion_order": "wxyz",
            "offsets_key": "synthetic-test",
        }
    )
    np.savez(path, qpos=qpos, fps=np.array(fps), metadata=np.array(metadata))
    return path


class CliSmokeTests(unittest.TestCase):
    def run_cli(self, *args: str) -> subprocess.CompletedProcess:
        return subprocess.run(
            [sys.executable, str(CLI), *args],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            timeout=180,
        )

    def test_dry_run_prints_constructed_lowcmds(self):
        with tempfile.TemporaryDirectory() as tmp:
            npz = write_synthetic_npz(Path(tmp) / "tiny.npz")
            done = self.run_cli(str(npz), "--dry-run", "2")
        self.assertEqual(done.returncode, 0, done.stderr)
        self.assertIn("LowCmd #0", done.stdout)
        self.assertIn("LowCmd #1", done.stdout)
        self.assertNotIn("LowCmd #2", done.stdout)
        self.assertIn("arm_sdk weight", done.stdout)
        self.assertIn("left_shoulder_pitch_joint", done.stdout)
        self.assertIn("packed=1004 bytes", done.stdout)
        self.assertIn("nothing was sent to a robot", done.stdout)
        # Motor indices are shown, and no leg slot appears among them.
        self.assertIn("motor[12]", done.stdout)
        self.assertIn("motor[28]", done.stdout)
        self.assertIn(f"motor[{WEIGHT_CHANNEL_INDEX}]", done.stdout)
        self.assertNotIn("motor[ 0]", done.stdout)

    def test_full_fake_replay_reaches_authority(self):
        with tempfile.TemporaryDirectory() as tmp:
            npz = write_synthetic_npz(Path(tmp) / "tiny.npz")
            jsonl = Path(tmp) / "out.jsonl"
            done = self.run_cli(str(npz), "--rate", "50", "--jsonl", str(jsonl))
            self.assertEqual(done.returncode, 0, done.stderr)
            records = [json.loads(line) for line in jsonl.read_text().splitlines()]
        self.assertIn("PASS", done.stdout)
        self.assertIn("shutdown: published explicit disarm", done.stdout)
        self.assertIn("weight=0.000000", done.stdout)
        self.assertGreater(len(records), 10)
        # t=0 is the measured-state reset instant; the first command is one
        # control period later, because the envelope requires dt > 0.
        self.assertEqual(records[0]["tick"], 1)
        self.assertGreater(records[0]["time_s"], 0.0)
        self.assertTrue(all(0.0 <= r["weight"] <= 1.0 for r in records))
        self.assertGreater(max(r["weight"] for r in records), 0.5)
        self.assertTrue(
            all(set(r["positions"]) == set(G1_ARM_SDK_JOINTS) for r in records)
        )

    def test_injected_state_stall_fails_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            npz = write_synthetic_npz(Path(tmp) / "tiny.npz")
            jsonl = Path(tmp) / "out.jsonl"
            done = self.run_cli(
                str(npz),
                "--rate",
                "50",
                "--stale-state-at",
                "0.20",
                "--jsonl",
                str(jsonl),
            )
            self.assertEqual(done.returncode, 0, done.stderr)
            records = [json.loads(line) for line in jsonl.read_text().splitlines()]
        reasons = {record["reason"].split(":", 1)[0] for record in records}
        self.assertIn("stale_state", reasons)
        self.assertEqual(records[-1]["weight"], 0.0)

    def test_dds_transport_requires_the_acknowledgement_flag(self):
        with tempfile.TemporaryDirectory() as tmp:
            npz = write_synthetic_npz(Path(tmp) / "tiny.npz")
            done = self.run_cli(str(npz), "--transport", "dds")
        self.assertEqual(done.returncode, 2)
        self.assertIn("yes-i-have-read-the-unverified-section", done.stderr)

    def test_preloop_bridge_failure_closes_dds_and_jsonl_resources(self):
        with tempfile.TemporaryDirectory() as tmp:
            npz = write_synthetic_npz(Path(tmp) / "tiny.npz")
            jsonl = Path(tmp) / "out.jsonl"
            output = io.StringIO()
            transport = mock.Mock()
            transport.connect.return_value = transport
            transport.close.side_effect = RuntimeError("test DDS close failed")
            real_path_open = Path.open

            def tracked_open(path, *args, **kwargs):
                if path == jsonl:
                    return output
                return real_path_open(path, *args, **kwargs)

            with (
                mock.patch.object(Path, "open", new=tracked_open),
                mock.patch.object(bridge_cli, "DdsTransport", return_value=transport),
                mock.patch.object(
                    bridge_cli,
                    "G1ArmBridge",
                    side_effect=ArmBridgeError("bridge construction failed"),
                ),
            ):
                result = bridge_cli.main(
                    [
                        str(npz),
                        "--transport",
                        "dds",
                        "--yes-i-have-read-the-unverified-section",
                        "--jsonl",
                        str(jsonl),
                    ]
                )

        self.assertEqual(result, 2)
        transport.close.assert_called_once_with()
        self.assertTrue(output.closed)

    def test_replay_and_teardown_errors_do_not_mask_or_skip_cleanup(self):
        with tempfile.TemporaryDirectory() as tmp:
            npz = write_synthetic_npz(Path(tmp) / "tiny.npz")
            transport = FakeTransport(record_limit=0)
            transport.close = mock.Mock(side_effect=RuntimeError("test close failed"))
            holder = {}

            def failing_bridge(*args, **kwargs):
                bridge = G1ArmBridge(*args, **kwargs)
                bridge.step = mock.Mock(side_effect=ArmBridgeError("test replay failed"))
                bridge.disarm = mock.Mock(
                    side_effect=ArmBridgeError("test disarm failed")
                )
                holder["bridge"] = bridge
                return bridge

            with (
                mock.patch.object(
                    bridge_cli, "FakeTransport", return_value=transport
                ),
                mock.patch.object(
                    bridge_cli, "G1ArmBridge", side_effect=failing_bridge
                ),
            ):
                result = bridge_cli.main([str(npz), "--duration", "0.1"])

        self.assertEqual(result, 1)
        holder["bridge"].disarm.assert_called_once()
        transport.close.assert_called_once_with()

    def test_jsonl_open_failure_happens_before_dds_is_acquired(self):
        with tempfile.TemporaryDirectory() as tmp:
            npz = write_synthetic_npz(Path(tmp) / "tiny.npz")
            jsonl = Path(tmp) / "unwritable.jsonl"
            real_path_open = Path.open

            def failing_open(path, *args, **kwargs):
                if path == jsonl:
                    raise PermissionError("test output is unwritable")
                return real_path_open(path, *args, **kwargs)

            with (
                mock.patch.object(Path, "open", new=failing_open),
                mock.patch.object(bridge_cli, "DdsTransport") as dds_transport,
            ):
                result = bridge_cli.main(
                    [
                        str(npz),
                        "--transport",
                        "dds",
                        "--yes-i-have-read-the-unverified-section",
                        "--jsonl",
                        str(jsonl),
                    ]
                )

        self.assertEqual(result, 2)
        dds_transport.assert_not_called()

    def test_missing_input_is_an_input_error(self):
        done = self.run_cli("does-not-exist.npz")
        self.assertEqual(done.returncode, 2)
        self.assertIn("ERROR", done.stderr)


# ---------------------------------------------------------------------------
# Review-finding regressions: clocks, gains, tracking error, reseed, release
# ---------------------------------------------------------------------------


class BridgeHardeningTests(unittest.TestCase):
    def setUp(self):
        self.limits = load_limits()
        self.safety = SafetyConfig()
        self.config = ArmBridgeConfig()
        self.transport, self.bridge = make_bridge(
            self.limits, config=self.config, safety_config=self.safety
        )
        self.neutral = midpoints(self.limits)
        self.dt = 0.02

    def _arm(self):
        self.transport.feed_arm_positions(self.neutral, receive_time_s=0.0)
        self.bridge.arm(now=0.0)

    def _run(self, ticks, *, start_tick, enable=True, echo=True):
        steps = []
        for offset in range(ticks):
            now = (start_tick + offset) * self.dt
            step = self.bridge.step(
                self.neutral, source_timestamp=now, now=now, enable=enable
            )
            steps.append(step)
            if echo and step.command is not None:
                self.transport.feed_arm_positions(
                    step.command.positions, receive_time_s=now
                )
        return steps

    def test_arm_rejects_non_finite_now(self):
        self.transport.feed_arm_positions(self.neutral, receive_time_s=0.0)
        for now in (float("nan"), float("inf"), float("-inf")):
            with self.subTest(now=now):
                with self.assertRaises(ArmBridgeError):
                    self.bridge.arm(now=now)
                self.assertFalse(self.bridge.armed)
        self.assertEqual(self.bridge.arm_count, 0)

    def test_non_finite_control_time_releases_and_latches(self):
        self._arm()
        step = self.bridge.step(
            self.neutral, source_timestamp=0.02, now=float("nan")
        )
        self.assertEqual(step.reason, "control_clock_invalid")
        self.assertEqual(step.weight, 0.0)
        self.assertTrue(self.bridge.rearm_required)

    def test_future_source_timestamp_is_a_latched_safety_fault(self):
        self._arm()
        step = self.bridge.step(self.neutral, source_timestamp=10.0, now=0.02)
        self.assertTrue(step.reason.startswith("safety_fault"), step.reason)
        self.assertIn("future", step.reason)
        self.assertEqual(step.weight, 0.0)
        self.assertTrue(self.bridge.rearm_required)

    def test_gains_above_upstream_example_require_explicit_override(self):
        for config in (
            ArmBridgeConfig(kp=60.01),
            ArmBridgeConfig(kd=1.51),
            ArmBridgeConfig(kp=1e6, kd=1e6),
        ):
            with self.subTest(config=config):
                with self.assertRaisesRegex(ArmBridgeError, "upstream example"):
                    config.validate()
        ArmBridgeConfig(kp=60.0, kd=1.5).validate()
        ArmBridgeConfig(kp=80.0, kd=2.0, allow_gains_above_example=True).validate()
        with self.assertRaises(ArmBridgeError):
            ArmBridgeConfig(allow_gains_above_example=1).validate()

    def test_cli_rejects_gains_above_example_without_override(self):
        with tempfile.TemporaryDirectory() as tmp:
            npz = write_synthetic_npz(Path(tmp) / "tiny.npz")
            with mock.patch("sys.stdout", new=io.StringIO()), mock.patch(
                "sys.stderr", new=io.StringIO()
            ) as stderr:
                rejected = bridge_cli.main([str(npz), "--kp", "100", "--dry-run", "1"])
                self.assertIn("upstream example", stderr.getvalue())
                accepted = bridge_cli.main(
                    [
                        str(npz),
                        "--kp",
                        "100",
                        "--allow-gains-above-example",
                        "--dry-run",
                        "1",
                    ]
                )
        self.assertEqual(rejected, 2)
        self.assertEqual(accepted, 0)

    def test_tracking_error_config_is_validated(self):
        for config in (
            ArmBridgeConfig(tracking_error_limit_rad=0.0),
            ArmBridgeConfig(tracking_error_limit_rad=float("inf")),
            ArmBridgeConfig(tracking_error_limit_rad=5.0),
            ArmBridgeConfig(tracking_error_min_weight=0.0),
            ArmBridgeConfig(tracking_error_min_weight=1.5),
            ArmBridgeConfig(release_repeat_count=0),
            ArmBridgeConfig(release_repeat_count=True),
        ):
            with self.subTest(config=config):
                with self.assertRaises(ArmBridgeError):
                    config.validate()

    def test_tracking_error_at_high_weight_latches_a_fault(self):
        self._arm()
        steps = self._run(60, start_tick=1)
        self.assertEqual(steps[-1].weight, 1.0)
        self.assertEqual({step.reason for step in steps}, {"tracking"})

        # The arm did not follow: one joint is well away from its command.
        diverged = dict(steps[-1].command.positions)
        diverged["left_shoulder_pitch_joint"] += 2.0 * self.config.tracking_error_limit_rad
        now = 61 * self.dt
        self.transport.feed_arm_positions(diverged, receive_time_s=now)
        step = self.bridge.step(self.neutral, source_timestamp=now, now=now)

        self.assertTrue(step.reason.startswith("tracking_error"), step.reason)
        self.assertIn("left_shoulder_pitch_joint", step.reason)
        self.assertEqual(step.weight, 0.0)
        self.assertTrue(self.bridge.rearm_required)
        self.assertEqual(self.transport.published[-1].weight, 0.0)

    def test_tracking_error_is_ignored_at_low_weight_and_when_not_published(self):
        self._arm()
        # Low weight: the on-board controller still owns most of the motion.
        self._run(5, start_tick=1)
        diverged = dict(self.neutral)
        diverged["left_shoulder_pitch_joint"] += 2.0 * self.config.tracking_error_limit_rad
        self.transport.feed_arm_positions(diverged, receive_time_s=6 * self.dt)
        step = self.bridge.step(
            self.neutral, source_timestamp=6 * self.dt, now=6 * self.dt
        )
        self.assertEqual(step.reason, "tracking")

        # Dry-run (publish=False) commands are never expected to be followed.
        transport, bridge = make_bridge(self.limits)
        transport.feed_arm_positions(self.neutral, receive_time_s=0.0)
        bridge.arm(now=0.0)
        for tick in range(1, 61):
            now = tick * self.dt
            transport.feed_arm_positions(diverged, receive_time_s=now)
            step = bridge.step(
                self.neutral, source_timestamp=now, now=now, publish=False
            )
        self.assertEqual(step.reason, "tracking")
        self.assertEqual(step.weight, 1.0)

    def test_reenable_after_disable_reseeds_from_measured_state(self):
        self._arm()
        self._run(55, start_tick=1)
        down = self._run(55, start_tick=56, enable=False)
        self.assertEqual(down[-1].weight, 0.0)
        frozen = dict(down[-1].command.positions)
        self.assertTrue(self.bridge.armed)

        # With authority back on-board, the arm drifted away from the frozen
        # envelope pose before the operator re-enables.
        drifted = {name: value + 0.2 for name, value in frozen.items()}
        now = 111 * self.dt
        self.transport.feed_arm_positions(drifted, receive_time_s=now)
        first = self.bridge.step(self.neutral, source_timestamp=now, now=now)

        self.assertEqual(first.reason, "reseeded")
        self.assertEqual(first.weight, 0.0)
        for name in G1_ARM_SDK_JOINTS:
            self.assertAlmostEqual(first.frame.joint_q(name), drifted[name], places=6)
        self.assertEqual(self.bridge.reset_positions[-1], drifted)

        now = 112 * self.dt
        self.transport.feed_arm_positions(drifted, receive_time_s=now)
        second = self.bridge.step(self.neutral, source_timestamp=now, now=now)
        self.assertEqual(second.reason, "tracking")
        self.assertAlmostEqual(second.weight, self.safety.blend_rate_per_s * self.dt)
        for name in G1_ARM_SDK_JOINTS:
            # The ramp starts at the measured pose, not the frozen one.
            self.assertLess(abs(second.frame.joint_q(name) - drifted[name]), 0.01)

    def test_disarm_repeats_the_final_weight_zero_frame(self):
        self._arm()
        self._run(10, start_tick=1)
        before = len(self.transport.published)
        pauses = []
        released = self.bridge.disarm(now=11 * self.dt, between=lambda: pauses.append(1))
        frames = self.transport.published[before:]
        self.assertEqual(len(frames), self.config.release_repeat_count)
        self.assertEqual(len(pauses), self.config.release_repeat_count - 1)
        self.assertTrue(all(frame.weight == 0.0 for frame in frames))
        self.assertTrue(all(frame.crc == released.frame.crc for frame in frames))

        self.bridge.disarm(now=12 * self.dt, repeat=2)
        self.assertEqual(len(self.transport.published), before + frames.__len__() + 2)
        with self.assertRaises(ArmBridgeError):
            self.bridge.disarm(now=13 * self.dt, repeat=0)

    def _run_cli_recording(self, argv, *, step_hook=None):
        recorder = FakeTransport()
        real_bridge = G1ArmBridge

        def make(*args, **kwargs):
            bridge = real_bridge(*args, **kwargs)
            if step_hook is not None:
                real_step = bridge.step
                bridge.step = lambda *a, **k: step_hook(real_step, *a, **k)
            return bridge

        with tempfile.TemporaryDirectory() as tmp:
            npz = write_synthetic_npz(Path(tmp) / "long.npz", frames=120)
            with (
                mock.patch.object(bridge_cli, "FakeTransport", return_value=recorder),
                mock.patch.object(bridge_cli, "G1ArmBridge", side_effect=make),
                mock.patch("sys.stdout", new=io.StringIO()),
                mock.patch("sys.stderr", new=io.StringIO()),
            ):
                try:
                    result = bridge_cli.main([str(npz), "--rate", "50", *argv])
                except KeyboardInterrupt:  # must not abort the test session
                    self.fail("operator stop escaped the CLI without a release")
        return result, [frame.weight for frame in recorder.published]

    def assert_ramped_release(self, weights, *, elapsed_ticks=1):
        repeat = self.config.release_repeat_count
        self.assertGreater(max(weights), 0.5)
        self.assertEqual(weights[-repeat:], [0.0] * repeat)
        # The envelope ramps at blend_rate_per_s of elapsed control time; an
        # interrupted tick may leave one period without a published frame.
        step = self.safety.blend_rate_per_s * self.dt * elapsed_ticks
        drops = [a - b for a, b in zip(weights, weights[1:])]
        self.assertLessEqual(max(drops), step + 1e-9, "weight dropped in one frame")

    def test_normal_shutdown_ramps_down_then_repeats_weight_zero(self):
        result, weights = self._run_cli_recording([])
        self.assertEqual(result, 0)
        self.assert_ramped_release(weights)

    def test_operator_stop_ramps_down_then_repeats_weight_zero(self):
        calls = {"count": 0}

        def interrupt_once(real_step, *args, **kwargs):
            calls["count"] += 1
            if calls["count"] == 80:
                raise KeyboardInterrupt
            return real_step(*args, **kwargs)

        result, weights = self._run_cli_recording([], step_hook=interrupt_once)
        self.assertEqual(result, 130)
        self.assert_ramped_release(weights, elapsed_ticks=2)

    def test_failed_replay_releases_immediately_with_repeats(self):
        calls = {"count": 0}

        def fail_once(real_step, *args, **kwargs):
            calls["count"] += 1
            if calls["count"] == 80:
                raise ArmBridgeError("injected replay failure")
            return real_step(*args, **kwargs)

        result, weights = self._run_cli_recording([], step_hook=fail_once)
        self.assertEqual(result, 1)
        repeat = self.config.release_repeat_count
        self.assertEqual(weights[-repeat:], [0.0] * repeat)
        self.assertGreater(weights[-repeat - 1], 0.5, "fault exit must not ramp")


class DdsLowStateCallbackTests(unittest.TestCase):
    """``DdsTransport._on_low_state`` without the SDK: plain attribute objects."""

    @staticmethod
    def message(count, *, q=0.0):
        from types import SimpleNamespace

        return SimpleNamespace(
            motor_state=[SimpleNamespace(q=q, dq=0.0, tau_est=0.0) for _ in range(count)],
            mode_machine=3,
            mode_pr=0,
            tick=7,
            crc=0,
        )

    def test_short_or_malformed_motor_state_is_dropped(self):
        transport = DdsTransport()
        transport._clock = lambda: 1.5
        for message in (
            self.message(NUM_MOTOR_SLOTS - 1),
            self.message(10),
            self.message(0),
            self.message(NUM_MOTOR_SLOTS, q=float("nan")),
            object(),
        ):
            with self.subTest(message=message):
                transport._on_low_state(message)
                self.assertIsNone(transport.latest_state())

        transport._on_low_state(self.message(NUM_MOTOR_SLOTS))
        state = transport.latest_state()
        self.assertIsNotNone(state)
        self.assertEqual(state.receive_time_s, 1.5)
        self.assertEqual(state.mode_machine, 3)

        # A later short message must not replace the last good state.
        transport._on_low_state(self.message(12))
        self.assertIs(transport.latest_state(), state)


class ReleaseInterruptTests(unittest.TestCase):
    """A second Ctrl-C during the paced terminal release must not skip repeats."""

    def run_main_with_sigint_during_release(self, *, sigints: int):
        published: list[float] = []

        class RecordingTransport(FakeTransport):
            def publish(self, frame):
                published.append(frame.weight)
                super().publish(frame)

        real_disarm = G1ArmBridge.disarm

        def disarm_with_interrupts(bridge, **kwargs):
            # Simulate DDS pacing: the operator presses Ctrl-C while the CLI
            # sleeps between weight-0 repeats.
            remaining = [sigints]

            def between():
                if remaining[0] > 0:
                    remaining[0] -= 1
                    signal.raise_signal(signal.SIGINT)

            kwargs["between"] = between
            return real_disarm(bridge, **kwargs)

        previous_handler = signal.getsignal(signal.SIGINT)
        with tempfile.TemporaryDirectory() as tmp:
            npz = write_synthetic_npz(Path(tmp) / "tiny.npz")
            with mock.patch.object(bridge_cli, "FakeTransport", RecordingTransport), \
                    mock.patch.object(G1ArmBridge, "disarm", disarm_with_interrupts), \
                    mock.patch("sys.stdout", new=io.StringIO()), \
                    mock.patch("sys.stderr", new=io.StringIO()) as stderr:
                code = bridge_cli.main([str(npz), "--rate", "50"])
        self.assertIs(signal.getsignal(signal.SIGINT), previous_handler)
        return code, published, stderr.getvalue()

    def test_sigint_between_release_repeats_still_sends_every_repeat(self):
        repeats = ArmBridgeConfig().release_repeat_count
        for sigints in (1, repeats - 1):
            with self.subTest(sigints=sigints):
                code, published, stderr = self.run_main_with_sigint_during_release(
                    sigints=sigints
                )
                self.assertEqual(code, 130, stderr)
                self.assertEqual(published[-repeats:], [0.0] * repeats)
                self.assertIn("operator stop during release", stderr)

    def test_release_without_interrupt_is_unchanged(self):
        code, published, stderr = self.run_main_with_sigint_during_release(sigints=0)
        self.assertEqual(code, 0, stderr)
        repeats = ArmBridgeConfig().release_repeat_count
        self.assertEqual(published[-repeats:], [0.0] * repeats)
        self.assertNotIn("operator stop", stderr)

    def test_deferred_sigint_restores_the_previous_handler(self):
        previous = signal.getsignal(signal.SIGINT)
        with bridge_cli.deferred_sigint() as received:
            signal.raise_signal(signal.SIGINT)
            signal.raise_signal(signal.SIGINT)
        self.assertEqual(received, [signal.SIGINT, signal.SIGINT])
        self.assertIs(signal.getsignal(signal.SIGINT), previous)
        with self.assertRaises(KeyboardInterrupt):
            with bridge_cli.deferred_sigint():
                raise KeyboardInterrupt  # only signals are deferred
        self.assertIs(signal.getsignal(signal.SIGINT), previous)

        # Off the main thread no handler can be installed; the block still runs.
        outcome: list[object] = []

        def worker():
            with bridge_cli.deferred_sigint() as received:
                outcome.append(received)

        thread = threading.Thread(target=worker)
        thread.start()
        thread.join(timeout=10)
        self.assertEqual(outcome, [[]])
        self.assertIs(signal.getsignal(signal.SIGINT), previous)

    def test_sigint_gate_raises_at_most_once_then_defers(self):
        previous = signal.getsignal(signal.SIGINT)
        with bridge_cli.sigint_gate() as gate:
            gate.allow_interrupt()
            with self.assertRaises(KeyboardInterrupt):
                signal.raise_signal(signal.SIGINT)
            # The interrupt disarmed itself: later signals are only recorded.
            signal.raise_signal(signal.SIGINT)
            gate.defer()
            signal.raise_signal(signal.SIGINT)
            self.assertEqual(gate.received, [signal.SIGINT, signal.SIGINT])
        self.assertIs(signal.getsignal(signal.SIGINT), previous)

        with bridge_cli.sigint_gate() as gate:
            signal.raise_signal(signal.SIGINT)  # recorded before interrupts allowed
            with self.assertRaises(KeyboardInterrupt):
                gate.allow_interrupt()
            self.assertEqual(gate.received, [])
            signal.raise_signal(signal.SIGINT)
            self.assertEqual(gate.received, [signal.SIGINT])
        self.assertIs(signal.getsignal(signal.SIGINT), previous)

    # -- SIGINT at the boundaries that lead into the terminal release ---------

    class SignalingStream(io.StringIO):
        """Raise a real SIGINT while the CLI writes a line containing ``marker``."""

        def __init__(self, marker: str | None = None):
            super().__init__()
            self.marker = marker
            self.fired = 0

        def write(self, text):
            written = super().write(text)
            if self.marker is not None and self.marker in text:
                self.fired += 1
                signal.raise_signal(signal.SIGINT)
            return written

    def run_main_with_hooks(
        self,
        *,
        step_hook=None,
        stdout_marker=None,
        stderr_marker=None,
        patches=(),
    ):
        published: list[float] = []

        class RecordingTransport(FakeTransport):
            def publish(self, frame):
                published.append(frame.weight)
                super().publish(frame)

        real_bridge = G1ArmBridge

        def make(*args, **kwargs):
            bridge = real_bridge(*args, **kwargs)
            if step_hook is not None:
                real_step = bridge.step
                bridge.step = lambda *a, **k: step_hook(real_step, *a, **k)
            return bridge

        stdout = self.SignalingStream(stdout_marker)
        stderr = self.SignalingStream(stderr_marker)
        previous_handler = signal.getsignal(signal.SIGINT)
        with tempfile.TemporaryDirectory() as tmp:
            npz = write_synthetic_npz(Path(tmp) / "long.npz", frames=120)
            with contextlib.ExitStack() as stack:
                stack.enter_context(
                    mock.patch.object(bridge_cli, "FakeTransport", RecordingTransport)
                )
                stack.enter_context(
                    mock.patch.object(bridge_cli, "G1ArmBridge", side_effect=make)
                )
                stack.enter_context(mock.patch("sys.stdout", new=stdout))
                stack.enter_context(mock.patch("sys.stderr", new=stderr))
                for patcher in patches:
                    stack.enter_context(patcher)
                try:
                    code = bridge_cli.main([str(npz), "--rate", "50"])
                except KeyboardInterrupt:
                    self.fail(
                        "Ctrl-C escaped the CLI; published tail="
                        f"{published[-8:]} stderr={stderr.getvalue()!r}"
                    )
        self.assertIs(signal.getsignal(signal.SIGINT), previous_handler)
        for stream, marker in ((stdout, stdout_marker), (stderr, stderr_marker)):
            if marker is not None:
                self.assertEqual(stream.fired, 1, f"{marker!r} was never written")
        return code, published, stderr.getvalue()

    def assert_full_release(self, published, stderr):
        repeats = ArmBridgeConfig().release_repeat_count
        self.assertEqual(published[-repeats:], [0.0] * repeats, stderr)

    def test_sigint_inside_ramp_interrupt_handler_still_sends_every_repeat(self):
        # The ramp is cut short (here by a KeyboardInterrupt raised from the
        # ramp step itself); a further Ctrl-C lands in the ramp's exception
        # handler, i.e. after the ramp and before the release.
        ramp_calls = {"count": 0}

        def interrupt_ramp(real_step, *args, **kwargs):
            if kwargs.get("enable") is False:
                ramp_calls["count"] += 1
                if ramp_calls["count"] == 3:
                    raise KeyboardInterrupt
            return real_step(*args, **kwargs)

        code, published, stderr = self.run_main_with_hooks(
            step_hook=interrupt_ramp,
            stderr_marker="shutdown ramp interrupted",
        )
        self.assertEqual(code, 130, stderr)
        self.assert_full_release(published, stderr)

    def test_sigint_before_the_ramp_starts_still_sends_every_repeat(self):
        # Ctrl-C after the replay but before the ramp's try (while its length
        # is computed) skips the ramp and releases immediately.
        real_ramp_ticks = bridge_cli.shutdown_ramp_ticks

        def ramp_ticks_with_sigint(*args, **kwargs):
            signal.raise_signal(signal.SIGINT)
            return real_ramp_ticks(*args, **kwargs)

        code, published, stderr = self.run_main_with_hooks(
            patches=(
                mock.patch.object(
                    bridge_cli, "shutdown_ramp_ticks", side_effect=ramp_ticks_with_sigint
                ),
            ),
        )
        self.assertEqual(code, 130, stderr)
        self.assert_full_release(published, stderr)
        repeats = ArmBridgeConfig().release_repeat_count
        self.assertGreater(published[-repeats - 1], 0.5, "no ramp tick may follow Ctrl-C")

    def test_sigint_after_a_completed_ramp_still_sends_every_repeat(self):
        code, published, stderr = self.run_main_with_hooks(
            stdout_marker="shutdown: ramped arm_sdk weight",
        )
        self.assertEqual(code, 130, stderr)
        self.assert_full_release(published, stderr)

    def test_second_sigint_in_operator_stop_handler_still_releases(self):
        # First Ctrl-C (a real signal) stops the replay; the second lands while
        # the stop is being reported, before the shutdown section is entered.
        replay_calls = {"count": 0}

        def stop_replay(real_step, *args, **kwargs):
            if kwargs.get("enable") is not False:
                replay_calls["count"] += 1
                if replay_calls["count"] == 80:
                    signal.raise_signal(signal.SIGINT)
            return real_step(*args, **kwargs)

        code, published, stderr = self.run_main_with_hooks(
            step_hook=stop_replay,
            stderr_marker="operator stop: ramping",
        )
        self.assertEqual(code, 130, stderr)
        self.assert_full_release(published, stderr)
        self.assertEqual(replay_calls["count"], 80, "first Ctrl-C must stop the replay")

    def test_first_sigint_during_ramp_starts_the_release_promptly(self):
        ramp_calls = {"count": 0}

        def signal_during_ramp(real_step, *args, **kwargs):
            if kwargs.get("enable") is False:
                ramp_calls["count"] += 1
                if ramp_calls["count"] == 3:
                    signal.raise_signal(signal.SIGINT)
            return real_step(*args, **kwargs)

        code, published, stderr = self.run_main_with_hooks(step_hook=signal_during_ramp)
        self.assertEqual(code, 130, stderr)
        self.assert_full_release(published, stderr)
        self.assertLessEqual(ramp_calls["count"], 3, "ramp continued after Ctrl-C")
        self.assertIn("shutdown ramp interrupted", stderr)
        repeats = ArmBridgeConfig().release_repeat_count
        self.assertGreater(published[-repeats - 1], 0.5, "release must not wait for the ramp")


class ArmBridgeConfigValidationTableTests(unittest.TestCase):
    """Exact message for every invalid field, plus which check wins on ties.

    Pins ``ArmBridgeConfig.validate`` so its isinstance chains can be folded
    into helpers without changing a single message or the check order.
    """

    KP = "kp must be finite and non-negative"
    KD = "kd must be finite and non-negative"
    ALLOW = "allow_gains_above_example must be a bool"
    TAU = "tau_ff must be finite"
    MOTOR = "motor_mode must be an integer fitting in a uint8"
    MODE_PR = "mode_pr must be integer 0 (PR) or 1 (AB)"
    MACHINE = "mode_machine_override must be an integer fitting in a uint8"
    TIMEOUTS = (
        "state timeouts must be finite and require "
        "0 < state_stale_timeout_s < state_hard_timeout_s"
    )
    FUTURE = (
        "state_future_tolerance_s must be finite, non-negative, and smaller "
        "than state_stale_timeout_s"
    )
    TRACKING = "tracking_error_limit_rad must be finite, positive, and at most 1 rad"
    MIN_WEIGHT = "tracking_error_min_weight must be greater than 0 and at most 1"
    RELEASE = "release_repeat_count must be an integer in [1, 100]"

    @staticmethod
    def gains(kp: float, kd: float) -> str:
        return (
            f"kp={kp:g}/kd={kd:g} exceed the upstream example gains "
            "(kp<=60, kd<=1.5); set allow_gains_above_example to exceed them deliberately"
        )

    def assert_rejected(self, message: str, **fields) -> None:
        with self.subTest(**{key: repr(value) for key, value in fields.items()}):
            with self.assertRaises(ArmBridgeError) as caught:
                ArmBridgeConfig(**fields).validate()
            self.assertEqual(str(caught.exception), message)

    def test_every_invalid_field_reports_its_exact_message(self):
        nan, inf = float("nan"), float("inf")
        for value in (-1.0, -1e-12, nan, inf, -inf, True, False, None, "1"):
            self.assert_rejected(self.KP, kp=value)
            self.assert_rejected(self.KD, kd=value)
        for value in (1, 0, None, "yes", np.bool_(True)):
            self.assert_rejected(self.ALLOW, allow_gains_above_example=value)
        self.assert_rejected(self.gains(60.01, 1.5), kp=60.01)
        self.assert_rejected(self.gains(60, 1.51), kd=1.51)
        self.assert_rejected(self.gains(1e6, 1e6), kp=1e6, kd=1e6)
        for value in (nan, inf, -inf, True, None, "0"):
            self.assert_rejected(self.TAU, tau_ff=value)
        for value in (-1, 256, True, False, 1.5, 0.0, None, "0"):
            self.assert_rejected(self.MOTOR, motor_mode=value)
        for value in (-1, 2, True, False, 0.0, 0.5, None):
            self.assert_rejected(self.MODE_PR, mode_pr=value)
        for value in (-1, 256, True, False, 9.5, 9.0, "9"):
            self.assert_rejected(self.MACHINE, mode_machine_override=value)
        for value in (nan, inf, 0.0, -0.1, 0.5, 0.6, True, None):
            self.assert_rejected(self.TIMEOUTS, state_stale_timeout_s=value)
        for value in (nan, inf, 0.1, 0.05, -1.0, True, None):
            self.assert_rejected(self.TIMEOUTS, state_hard_timeout_s=value)
        for value in (-1e-9, nan, inf, 0.1, 0.2, False, None):
            self.assert_rejected(self.FUTURE, state_future_tolerance_s=value)
        for value in (0.0, -0.1, 1.0 + 1e-9, 5.0, nan, inf, True, None):
            self.assert_rejected(self.TRACKING, tracking_error_limit_rad=value)
        for value in (0.0, -0.5, 1.0 + 1e-9, 1.5, nan, inf, True, None):
            self.assert_rejected(self.MIN_WEIGHT, tracking_error_min_weight=value)
        for value in (0, -1, 101, True, False, 2.0, None):
            self.assert_rejected(self.RELEASE, release_repeat_count=value)

    def test_earlier_check_wins_when_several_fields_are_invalid(self):
        nan = float("nan")
        self.assert_rejected(self.KP, kp=-1.0, kd=-1.0, tau_ff=nan)
        self.assert_rejected(self.KD, kd=-1.0, allow_gains_above_example=1)
        self.assert_rejected(self.ALLOW, allow_gains_above_example=1, kp=100.0)
        self.assert_rejected(self.gains(100, 1.5), kp=100.0, tau_ff=nan)
        self.assert_rejected(self.TAU, tau_ff=nan, motor_mode=-1)
        self.assert_rejected(self.MOTOR, motor_mode=300, mode_pr=5)
        self.assert_rejected(self.MODE_PR, mode_pr=5, mode_machine_override=300)
        self.assert_rejected(
            self.MACHINE, mode_machine_override=300, state_stale_timeout_s=nan
        )
        self.assert_rejected(
            self.TIMEOUTS, state_stale_timeout_s=nan, state_future_tolerance_s=-1.0
        )
        self.assert_rejected(
            self.FUTURE, state_future_tolerance_s=-1.0, tracking_error_limit_rad=0.0
        )
        self.assert_rejected(
            self.TRACKING, tracking_error_limit_rad=0.0, tracking_error_min_weight=0.0
        )
        self.assert_rejected(
            self.MIN_WEIGHT, tracking_error_min_weight=0.0, release_repeat_count=0
        )

    def test_boundary_and_numpy_values_are_accepted(self):
        for fields in (
            {},
            {"kp": 0.0, "kd": 0.0},
            {"kp": 0, "kd": 0},
            {"kp": UPSTREAM_EXAMPLE_KP, "kd": UPSTREAM_EXAMPLE_KD},
            {"kp": 1e6, "kd": 1e6, "allow_gains_above_example": True},
            {"tau_ff": -3.5},
            {"motor_mode": 0},
            {"motor_mode": 255},
            {"motor_mode": np.uint8(1)},
            {"mode_pr": 1},
            {"mode_pr": np.int64(0)},
            {"mode_machine_override": 0},
            {"mode_machine_override": 255},
            {"mode_machine_override": np.int32(9)},
            {"state_stale_timeout_s": np.float64(0.2), "state_hard_timeout_s": 0.21},
            {"state_future_tolerance_s": 0.0},
            {"state_future_tolerance_s": 0.0999},
            {"tracking_error_limit_rad": 1.0},
            {"tracking_error_limit_rad": 1e-9},
            {"tracking_error_min_weight": 1.0},
            {"tracking_error_min_weight": np.float32(0.5)},
            {"release_repeat_count": 1},
            {"release_repeat_count": 100},
            {"release_repeat_count": np.int16(7)},
        ):
            with self.subTest(**{key: repr(value) for key, value in fields.items()}):
                ArmBridgeConfig(**fields).validate()


if __name__ == "__main__":
    unittest.main()
