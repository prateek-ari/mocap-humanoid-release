"""Deterministic, hardware-free fault replay for G1 upper-body commands.

The replay feeds recorded qpos frames through :class:`ArmSafetyEnvelope` as
timestamped network packets.  It deliberately models only command selection
and the software safety state machine; it is neither hardware validation nor
a calibration procedure.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any, Mapping, Sequence

import numpy as np

from .safety import ArmSafetyEnvelope, SafetyConfig, SafetyError
from .schema import G1_ARM_SDK_JOINTS, JointLimit


class FaultReplayError(ValueError):
    """Raised when a replay request is invalid or internally inconsistent."""


@dataclass(frozen=True)
class TimeInterval:
    start_s: float
    duration_s: float

    @property
    def end_s(self) -> float:
        return self.start_s + self.duration_s

    def contains(self, time_s: float) -> bool:
        return self.start_s <= time_s < self.end_s

    def validate(self, label: str) -> None:
        if not math.isfinite(float(self.start_s)) or self.start_s < 0.0:
            raise FaultReplayError(f"{label} start must be finite and non-negative")
        if not math.isfinite(float(self.duration_s)) or self.duration_s <= 0.0:
            raise FaultReplayError(f"{label} duration must be finite and positive")


@dataclass(frozen=True)
class FaultReplayConfig:
    control_rate_hz: float = 60.0
    latency_s: float = 0.020
    packet_loss_every: int = 0
    reorder_every: int = 0
    reorder_delay_s: float = 0.0
    source_stalls: tuple[TimeInterval, ...] = ()
    disable_intervals: tuple[TimeInterval, ...] = ()
    control_gaps: tuple[TimeInterval, ...] = ()
    source_hard_reset_delay_s: float | None = 0.10
    tracking_p95_limit_rad: float = 0.35
    tracking_max_limit_rad: float = 0.75
    clamp_command_fraction_limit: float = 0.01
    require_reasons: tuple[str, ...] = ()

    def validate(self) -> None:
        positive = (
            "control_rate_hz",
            "tracking_p95_limit_rad",
            "tracking_max_limit_rad",
        )
        for name in positive:
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise FaultReplayError(f"{name} must be finite and positive")
        if not math.isfinite(float(self.latency_s)) or self.latency_s < 0.0:
            raise FaultReplayError("latency_s must be finite and non-negative")
        if not math.isfinite(float(self.reorder_delay_s)) or self.reorder_delay_s < 0.0:
            raise FaultReplayError("reorder_delay_s must be finite and non-negative")
        if self.packet_loss_every < 0 or self.reorder_every < 0:
            raise FaultReplayError("packet-loss/reorder periods cannot be negative")
        if self.reorder_every == 1:
            raise FaultReplayError("reorder_every=1 delays every packet and cannot reorder them")
        if self.reorder_every > 0 and self.reorder_delay_s <= 0.0:
            raise FaultReplayError("positive reorder_delay_s is required when reordering")
        if not 0.0 <= self.clamp_command_fraction_limit <= 1.0:
            raise FaultReplayError("clamp_command_fraction_limit must be in [0,1]")
        if self.source_hard_reset_delay_s is not None:
            value = float(self.source_hard_reset_delay_s)
            if not math.isfinite(value) or value < 0.0:
                raise FaultReplayError(
                    "source_hard_reset_delay_s must be finite and non-negative"
                )
        allowed_reasons = {"tracking", "disabled", "stale_source", "hard_timeout"}
        unknown = sorted(set(self.require_reasons) - allowed_reasons)
        if unknown:
            raise FaultReplayError(f"unknown required reasons: {unknown}")
        for label, intervals in (
            ("source stall", self.source_stalls),
            ("disable interval", self.disable_intervals),
            ("control gap", self.control_gaps),
        ):
            for interval in intervals:
                interval.validate(label)


@dataclass(frozen=True)
class _Packet:
    sequence: int
    source_time_s: float
    arrival_time_s: float


def _contains(intervals: Sequence[TimeInterval], time_s: float) -> bool:
    return any(interval.contains(time_s) for interval in intervals)


def _validate_reference(qpos: Any, fps: Any) -> tuple[np.ndarray, float]:
    values = np.asarray(qpos, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 36 or len(values) < 3:
        raise FaultReplayError("qpos must have shape [T>=3,36]")
    if not np.isfinite(values).all():
        raise FaultReplayError("qpos contains non-finite values")
    rate = float(fps)
    if not math.isfinite(rate) or rate <= 0.0:
        raise FaultReplayError("fps must be finite and positive")
    return values, rate


def _build_envelope(
    limits: Mapping[str, JointLimit], config: SafetyConfig
) -> ArmSafetyEnvelope:
    """Validate limits and safety settings with the envelope's own checks."""
    try:
        return ArmSafetyEnvelope(limits, config)
    except SafetyError as exc:
        raise FaultReplayError(f"invalid safety configuration or limits: {exc}") from exc


def _assertion(actual: float | int, limit: float | int, passed: bool) -> dict[str, Any]:
    return {"actual": actual, "limit": limit, "status": "PASS" if passed else "FAIL"}


def replay_arm_faults(
    qpos: Any,
    fps: Any,
    *,
    limits: Mapping[str, JointLimit],
    replay_config: FaultReplayConfig | None = None,
    safety_config: SafetyConfig | None = None,
    source_sha256: str | None = None,
    source_metadata: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Replay qpos arm targets through deterministic transport and safety faults.

    Packet delivery uses newest-frame semantics: at a control tick every packet
    whose arrival time has elapsed is drained, but only the greatest source
    sequence may replace the current target.  A late older packet can therefore
    never move the selected sequence backwards.
    """

    reference, source_fps = _validate_reference(qpos, fps)
    config = replay_config or FaultReplayConfig()
    config.validate()
    safety = safety_config or SafetyConfig()
    envelope = _build_envelope(limits, safety)

    arm_reference = reference[:, 7 + 12 : 7 + 29]
    if arm_reference.shape[1] != len(G1_ARM_SDK_JOINTS):
        raise FaultReplayError("internal arm reference mapping is inconsistent")
    initial = {
        name: float(arm_reference[0, column])
        for column, name in enumerate(G1_ARM_SDK_JOINTS)
    }
    for name, value in initial.items():
        limit = limits[name]
        if not limit.lower <= value <= limit.upper:
            raise FaultReplayError(
                f"initial recorded position {name}={value} is outside the official range"
            )

    packets: list[_Packet] = []
    loss_count = 0
    stall_suppressed_count = 0
    delayed_for_reorder_count = 0
    for sequence in range(len(reference)):
        source_time = sequence / source_fps
        if _contains(config.source_stalls, source_time):
            stall_suppressed_count += 1
            continue
        if config.packet_loss_every and (sequence + 1) % config.packet_loss_every == 0:
            loss_count += 1
            continue
        delay = config.latency_s
        if config.reorder_every and (sequence + 1) % config.reorder_every == 0:
            delay += config.reorder_delay_s
            delayed_for_reorder_count += 1
        packets.append(_Packet(sequence, source_time, source_time + delay))
    packets.sort(key=lambda packet: (packet.arrival_time_s, packet.sequence))

    control_dt = 1.0 / config.control_rate_hz
    source_duration = (len(reference) - 1) / source_fps
    final_arrival = max((packet.arrival_time_s for packet in packets), default=0.0)
    last_gap_end = max((interval.end_s for interval in config.control_gaps), default=0.0)
    simulation_end = max(source_duration + config.latency_s, final_arrival, last_gap_end) + control_dt

    envelope.reset(initial, now=0.0)
    last_positions = dict(initial)
    last_velocities = {name: 0.0 for name in G1_ARM_SDK_JOINTS}
    last_emission_time = 0.0
    latest_sequence = 0
    latest_source_time = 0.0
    latest_target = dict(initial)
    packet_cursor = 0
    highest_arrival_sequence = -1
    selected_sequence_regressions = 0
    stale_selections_rejected = 0
    reordered_arrivals = 0
    discarded_not_newest = 0
    packets_delivered = 0
    commands = 0
    clamp_commands = 0
    clamp_joint_samples = 0
    reason_counts = {reason: 0 for reason in ("tracking", "disabled", "stale_source", "hard_timeout")}
    reason_transitions: list[dict[str, Any]] = []
    reset_events: list[dict[str, Any]] = []
    last_reason: str | None = None
    source_latch_fresh_observations = 0
    source_latch_fresh_start: float | None = None
    source_hard_reset_count = 0
    control_gap_error_count = 0
    control_gap_reset_count = 0
    tracking_after_source_reset = False
    awaiting_tracking_after_source_reset = False
    skipped_control_ticks = 0
    tracking_errors: list[float] = []
    max_velocity_ratio = 0.0
    max_acceleration_ratio = 0.0
    position_violation_samples = 0
    velocity_violation_samples = 0
    acceleration_violation_samples = 0

    steps = int(math.floor(simulation_end / control_dt + 1e-12))
    for tick in range(1, steps + 1):
        now = tick * control_dt
        if _contains(config.control_gaps, now):
            skipped_control_ticks += 1
            continue

        arrived: list[_Packet] = []
        while packet_cursor < len(packets) and packets[packet_cursor].arrival_time_s <= now + 1e-12:
            packet = packets[packet_cursor]
            packet_cursor += 1
            packets_delivered += 1
            if packet.sequence < highest_arrival_sequence:
                reordered_arrivals += 1
            highest_arrival_sequence = max(highest_arrival_sequence, packet.sequence)
            arrived.append(packet)
        previous_selected_sequence = latest_sequence
        if arrived:
            newest = max(arrived, key=lambda packet: packet.sequence)
            discarded_not_newest += len(arrived) - 1
            if newest.sequence <= latest_sequence:
                discarded_not_newest += 1
                if newest.sequence < latest_sequence:
                    # A latest-arrival selector would have moved backwards
                    # here; the newest-sequence rule rejected it.
                    stale_selections_rejected += 1
            else:
                latest_sequence = newest.sequence
                latest_source_time = newest.source_time_s
                latest_target = {
                    name: float(arm_reference[newest.sequence, column])
                    for column, name in enumerate(G1_ARM_SDK_JOINTS)
                }
        # Invariant check on the selection actually used for this tick.
        if latest_sequence < previous_selected_sequence:
            selected_sequence_regressions += 1

        if source_latch_fresh_start is not None and config.source_hard_reset_delay_s is not None:
            if now - source_latch_fresh_start + 1e-12 >= config.source_hard_reset_delay_s:
                envelope.reset(last_positions, now=now)
                last_velocities = {name: 0.0 for name in G1_ARM_SDK_JOINTS}
                last_emission_time = now
                source_hard_reset_count += 1
                reset_events.append(
                    {
                        "time_s": now,
                        "kind": "source_hard_timeout_measured_state_reset",
                        "measured_state_model": "last_emitted_command",
                    }
                )
                source_latch_fresh_start = None
                awaiting_tracking_after_source_reset = True
                last_reason = None
                continue

        enable = not _contains(config.disable_intervals, now)
        try:
            command = envelope.step(
                latest_target,
                source_timestamp=latest_source_time,
                now=now,
                enable=enable,
            )
        except SafetyError as exc:
            if "control loop stalled" not in str(exc):
                raise FaultReplayError(f"safety envelope rejected replay: {exc}") from exc
            control_gap_error_count += 1
            envelope.reset(last_positions, now=now)
            last_velocities = {name: 0.0 for name in G1_ARM_SDK_JOINTS}
            last_emission_time = now
            control_gap_reset_count += 1
            reset_events.append(
                {
                    "time_s": now,
                    "kind": "control_gap_measured_state_reset",
                    "measured_state_model": "last_emitted_command",
                    "error": str(exc),
                }
            )
            source_latch_fresh_start = None
            last_reason = None
            continue

        dt = now - last_emission_time
        commands += 1
        reason_counts[command.reason] += 1
        if command.reason != last_reason:
            reason_transitions.append(
                {
                    "time_s": now,
                    "from": last_reason,
                    "to": command.reason,
                    "source_age_s": command.source_age_s,
                }
            )
            last_reason = command.reason
        if command.clamped_joints:
            clamp_commands += 1
            clamp_joint_samples += len(command.clamped_joints)

        for name in G1_ARM_SDK_JOINTS:
            limit = limits[name]
            position = command.positions[name]
            velocity = command.velocities[name]
            allowed_velocity = limit.velocity * safety.velocity_limit_fraction
            acceleration = (velocity - last_velocities[name]) / dt
            max_velocity_ratio = max(max_velocity_ratio, abs(velocity) / allowed_velocity)
            max_acceleration_ratio = max(
                max_acceleration_ratio,
                abs(acceleration) / safety.acceleration_limit_rad_s2,
            )
            if not limit.lower - 1e-9 <= position <= limit.upper + 1e-9:
                position_violation_samples += 1
            if abs(velocity) > allowed_velocity + 1e-8:
                velocity_violation_samples += 1
            if abs(acceleration) > safety.acceleration_limit_rad_s2 + 1e-7:
                acceleration_violation_samples += 1
            if command.reason == "tracking":
                tracking_errors.append(position - latest_target[name])

        source_is_fresh = command.source_age_s < safety.stale_timeout_s
        if command.reason == "hard_timeout" and source_is_fresh:
            source_latch_fresh_observations += 1
            if source_latch_fresh_start is None:
                source_latch_fresh_start = now
        if awaiting_tracking_after_source_reset and command.reason == "tracking":
            tracking_after_source_reset = True
            awaiting_tracking_after_source_reset = False
        last_positions = dict(command.positions)
        last_velocities = dict(command.velocities)
        last_emission_time = now

    error_array = np.abs(np.asarray(tracking_errors, dtype=np.float64))
    tracking_rms = float(np.sqrt(np.mean(np.square(error_array)))) if len(error_array) else 0.0
    tracking_p95 = float(np.percentile(error_array, 95)) if len(error_array) else 0.0
    tracking_max = float(np.max(error_array)) if len(error_array) else 0.0
    clamp_fraction = clamp_commands / commands if commands else 0.0
    required_missing = [reason for reason in config.require_reasons if reason_counts[reason] == 0]

    source_hard_was_required = "hard_timeout" in config.require_reasons
    latch_reset_pass = (
        not source_hard_was_required
        or (
            source_latch_fresh_observations > 0
            and source_hard_reset_count > 0
            and tracking_after_source_reset
        )
    )
    long_control_gap_configured = any(
        interval.duration_s >= safety.hard_timeout_s for interval in config.control_gaps
    )
    control_gap_pass = (
        not long_control_gap_configured
        or (control_gap_error_count > 0 and control_gap_error_count == control_gap_reset_count)
    )
    assertions = {
        "tracking_p95_abs_rad": _assertion(
            tracking_p95, config.tracking_p95_limit_rad,
            tracking_p95 <= config.tracking_p95_limit_rad,
        ),
        "tracking_max_abs_rad": _assertion(
            tracking_max, config.tracking_max_limit_rad,
            tracking_max <= config.tracking_max_limit_rad,
        ),
        "clamp_command_fraction": _assertion(
            clamp_fraction, config.clamp_command_fraction_limit,
            clamp_fraction <= config.clamp_command_fraction_limit,
        ),
        "position_violation_samples": _assertion(
            position_violation_samples, 0, position_violation_samples == 0
        ),
        "velocity_violation_samples": _assertion(
            velocity_violation_samples, 0, velocity_violation_samples == 0
        ),
        "acceleration_violation_samples": _assertion(
            acceleration_violation_samples, 0, acceleration_violation_samples == 0
        ),
        "selected_sequence_regressions": _assertion(
            selected_sequence_regressions, 0, selected_sequence_regressions == 0
        ),
        "required_reason_coverage": {
            "actual_missing": required_missing,
            "limit": [],
            "status": "PASS" if not required_missing else "FAIL",
        },
        "hard_timeout_latch_and_reset": {
            "actual": {
                "fresh_while_latched_commands": source_latch_fresh_observations,
                "source_hard_resets": source_hard_reset_count,
                "tracking_after_reset": tracking_after_source_reset,
            },
            "required": source_hard_was_required,
            "status": "PASS" if latch_reset_pass else "FAIL",
        },
        "control_gap_fail_closed_and_reset": {
            "actual": {
                "stall_errors": control_gap_error_count,
                "resets": control_gap_reset_count,
            },
            "required": long_control_gap_configured,
            "status": "PASS" if control_gap_pass else "FAIL",
        },
    }
    overall = "PASS" if all(value["status"] == "PASS" for value in assertions.values()) else "FAIL"

    return {
        "schema_version": 1,
        "purpose": "deterministic_hardware_free_deployment_fault_replay",
        "status": overall,
        "scope": {
            "hardware_validation": False,
            "calibration": False,
            "claims": [
                "software packet selection and ArmSafetyEnvelope state-machine behavior only",
                "does not model motors, communication firmware, balance, contacts, battery, or thermal limits",
            ],
            "measured_state_reset_model": "last_emitted_command_not_encoder_feedback",
        },
        "input": {
            "frames": len(reference),
            "fps": source_fps,
            "duration_s": source_duration,
            "source_sha256": source_sha256,
            "source_metadata": dict(source_metadata or {}),
        },
        "configuration": {
            "replay": {
                **asdict(config),
                "source_stalls": [asdict(interval) for interval in config.source_stalls],
                "disable_intervals": [asdict(interval) for interval in config.disable_intervals],
                "control_gaps": [asdict(interval) for interval in config.control_gaps],
            },
            "safety": asdict(safety),
        },
        "transport": {
            "source_packets": len(reference),
            "packets_suppressed_by_source_stalls": stall_suppressed_count,
            "packets_dropped_by_loss": loss_count,
            "packets_scheduled": len(packets),
            "packets_delivered": packets_delivered,
            "packets_delayed_for_reorder": delayed_for_reorder_count,
            "out_of_order_arrivals": reordered_arrivals,
            "packets_discarded_by_newest_frame_selection": discarded_not_newest,
            "selected_sequence_regressions": selected_sequence_regressions,
            "stale_selections_rejected": stale_selections_rejected,
            "last_selected_sequence": latest_sequence,
        },
        "control": {
            "simulation_duration_s": steps * control_dt,
            "commands_emitted": commands,
            "skipped_control_ticks": skipped_control_ticks,
            "reason_counts": reason_counts,
            "reason_transitions": reason_transitions,
            "reset_events": reset_events,
            "clamp_commands": clamp_commands,
            "clamp_joint_samples": clamp_joint_samples,
            "clamp_command_fraction": clamp_fraction,
        },
        "tracking_during_tracking_reason": {
            "joint_samples": len(tracking_errors),
            "rms_rad": tracking_rms,
            "p95_abs_rad": tracking_p95,
            "max_abs_rad": tracking_max,
        },
        "safety_compliance": {
            "max_emitted_velocity_to_limit_ratio": max_velocity_ratio,
            "max_emitted_acceleration_to_limit_ratio": max_acceleration_ratio,
            "position_violation_samples": position_violation_samples,
            "velocity_violation_samples": velocity_violation_samples,
            "acceleration_violation_samples": acceleration_violation_samples,
        },
        "fault_response": {
            "fresh_commands_held_in_hard_timeout_latch": source_latch_fresh_observations,
            "source_hard_timeout_resets": source_hard_reset_count,
            "tracking_resumed_after_source_hard_reset": tracking_after_source_reset,
            "control_gap_stall_errors": control_gap_error_count,
            "control_gap_resets": control_gap_reset_count,
        },
        "assertions": assertions,
    }
