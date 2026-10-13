"""Simulator-side adapter from G1 DOF arrays to the named arm safety envelope.

This module deliberately has no MuJoCo, networking, GMR, or Unitree SDK
dependency.  The live viewer supplies its current simulated DOF positions as
the measured state used for reset/rearm.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
from typing import Mapping

import numpy as np

from .safety import ArmSafetyEnvelope, SafetyConfig, SafetyError
from .schema import G1_29DOF_JOINTS, G1_ARM_SDK_JOINTS, JointLimit


_DOF_INDEX = {name: index for index, name in enumerate(G1_29DOF_JOINTS)}


def _validated_dof(dof_pos: np.ndarray, *, label: str) -> np.ndarray:
    values = np.asarray(dof_pos, dtype=np.float64)
    expected = (len(G1_29DOF_JOINTS),)
    if values.shape != expected:
        raise SafetyError(f"{label} must have shape {expected}, got {values.shape}")
    if not np.isfinite(values).all():
        raise SafetyError(f"{label} contains a non-finite value")
    return values


def named_arm_positions(dof_pos: np.ndarray) -> dict[str, float]:
    """Extract waist and arm positions from a canonical G1 29-DOF vector."""
    values = _validated_dof(dof_pos, label="dof_pos")
    return {name: float(values[_DOF_INDEX[name]]) for name in G1_ARM_SDK_JOINTS}


def replace_named_arm_positions(
    dof_pos: np.ndarray,
    positions: Mapping[str, float],
) -> np.ndarray:
    """Return a copy with exactly the named waist and arm values replaced."""
    values = _validated_dof(dof_pos, label="dof_pos").copy()
    missing = [name for name in G1_ARM_SDK_JOINTS if name not in positions]
    unknown = sorted(set(positions) - set(G1_ARM_SDK_JOINTS))
    if missing or unknown:
        raise SafetyError(
            f"positions require exactly the arm-SDK joints; missing={missing}, unknown={unknown}"
        )
    for name in G1_ARM_SDK_JOINTS:
        value = float(positions[name])
        if not math.isfinite(value):
            raise SafetyError(f"position for {name} is not finite")
        values[_DOF_INDEX[name]] = value
    return values


@dataclass(frozen=True)
class LiveSafetyOutput:
    """One simulator command and its safety state."""

    dof_pos: np.ndarray
    velocities: dict[str, float]
    blend_weight: float
    engaged: bool
    reason: str
    clamped_joints: tuple[str, ...]
    source_age_s: float
    reset_required: bool


class LiveG1SafetyController:
    """Apply :class:`ArmSafetyEnvelope` to canonical live G1 DOF targets.

    Only the named waist and arm joints are guarded.  The twelve leg joints in
    each fresh target pass through unchanged; this is therefore a kinematic
    MuJoCo preview seam, not a whole-body or hardware controller.
    """

    def __init__(
        self,
        limits: Mapping[str, JointLimit],
        config: SafetyConfig | None = None,
    ) -> None:
        self.envelope = ArmSafetyEnvelope(limits, config)
        self._target_dof: np.ndarray | None = None
        self._target_arm: dict[str, float] | None = None
        self._source_timestamp: float | None = None
        self._stall_reset_required = False
        self.last_error: str | None = None

    @property
    def has_target(self) -> bool:
        return self._target_dof is not None

    @property
    def reset_required(self) -> bool:
        # The envelope's own latch already covers a source hard timeout until
        # the next measured-state reset; a loop stall is tracked here.
        return self._stall_reset_required or self.envelope.hard_timeout_latched

    def reset(self, measured_dof_pos: np.ndarray, *, now: float) -> None:
        """Reset from the current simulated state and require a new target."""
        measured = _validated_dof(measured_dof_pos, label="measured_dof_pos")
        self.envelope.reset(named_arm_positions(measured), now=now)
        self._target_dof = None
        self._target_arm = None
        self._source_timestamp = None
        self._stall_reset_required = False
        self.last_error = None

    def update_target(
        self,
        target_dof_pos: np.ndarray,
        *,
        source_timestamp: float,
    ) -> None:
        """Store a fresh retargeted pose using its local monotonic arrival time."""
        values = _validated_dof(target_dof_pos, label="target_dof_pos")
        timestamp = float(source_timestamp)
        if not math.isfinite(timestamp):
            raise SafetyError("source_timestamp is not finite")
        self._target_dof = values.copy()
        self._target_arm = named_arm_positions(values)
        self._source_timestamp = timestamp

    def step(
        self,
        measured_dof_pos: np.ndarray,
        *,
        now: float,
        enable: bool,
    ) -> LiveSafetyOutput:
        """Produce a bounded simulator command or fail closed after a loop stall."""
        measured = _validated_dof(measured_dof_pos, label="measured_dof_pos")
        if self._stall_reset_required:
            return self._stalled_output(measured, now=now)
        if self._target_dof is None or self._target_arm is None or self._source_timestamp is None:
            try:
                idle = self.envelope.step(
                    named_arm_positions(measured),
                    source_timestamp=now,
                    now=now,
                    enable=False,
                )
            except SafetyError as exc:
                self._stall_reset_required = True
                self.last_error = str(exc)
                return self._stalled_output(measured, now=now)
            return LiveSafetyOutput(
                dof_pos=replace_named_arm_positions(measured, idle.positions),
                velocities=idle.velocities,
                blend_weight=0.0,
                engaged=False,
                reason="waiting_for_fresh_target",
                clamped_joints=(),
                source_age_s=0.0,
                reset_required=False,
            )

        try:
            command = self.envelope.step(
                self._target_arm,
                source_timestamp=self._source_timestamp,
                now=now,
                enable=enable,
            )
        except SafetyError as exc:
            # A control-loop gap means the envelope's stored state is no longer
            # a trustworthy command origin.  Keep the measured simulator pose
            # and require an explicit reset instead of automatically rearming.
            self._stall_reset_required = True
            self.last_error = str(exc)
            return self._stalled_output(measured, now=now)

        return LiveSafetyOutput(
            dof_pos=replace_named_arm_positions(self._target_dof, command.positions),
            velocities=command.velocities,
            blend_weight=command.blend_weight,
            engaged=command.engaged,
            reason=command.reason,
            clamped_joints=command.clamped_joints,
            source_age_s=command.source_age_s,
            reset_required=self.reset_required,
        )

    def _stalled_output(self, measured: np.ndarray, *, now: float) -> LiveSafetyOutput:
        if self._source_timestamp is None:
            source_age = 0.0
        else:
            source_age = max(0.0, float(now) - self._source_timestamp)
        return LiveSafetyOutput(
            dof_pos=measured.copy(),
            velocities={name: 0.0 for name in G1_ARM_SDK_JOINTS},
            blend_weight=0.0,
            engaged=False,
            reason="control_stall_reset_required",
            clamped_joints=(),
            source_age_s=source_age,
            reset_required=True,
        )


class LiveOperatorControls:
    """Small keyboard state holder kept separate from the MuJoCo callback."""

    def __init__(self, *, enabled: bool = True) -> None:
        self.enabled = bool(enabled)
        # The viewer thread sets the request while the control loop takes it;
        # the lock makes take-and-clear atomic so a press is never dropped.
        self._lock = threading.Lock()
        self._rearm_requested = False
        self._calibration_requested = False

    def keyboard_callback(self, keycode: int) -> None:
        try:
            key = chr(keycode).lower()
        except (TypeError, ValueError):
            return
        if key == " ":
            self.enabled = not self.enabled
        elif key == "r":
            with self._lock:
                self._rearm_requested = True
        elif key == "n":
            with self._lock:
                self._calibration_requested = True

    def take_rearm_request(self) -> bool:
        with self._lock:
            requested = self._rearm_requested
            self._rearm_requested = False
        return requested

    def take_calibration_request(self) -> bool:
        """Whether N (neutral-pose arm alignment) was pressed since the last call."""
        with self._lock:
            requested = self._calibration_requested
            self._calibration_requested = False
        return requested


#: Smallest range clamp the preview controller reports.
CLAMP_REPORT_TOLERANCE_RAD = 1e-3


class LiveRangePreviewController:
    """Simulator-only command screening for robots without a hardware path.

    Same interface and operator semantics as :class:`LiveG1SafetyController`
    (Space gates the upper-body joints, a stale source holds the last command,
    a hard timeout latches until R), but it bounds commands only by the MJCF
    position ranges.  There is no velocity/acceleration envelope because no
    hardware velocity limits exist for these models; the live filter provides
    the smoothing.  It must never be used to command real hardware.
    """

    def __init__(
        self,
        joint_ranges: tuple[tuple[str, float, float], ...],
        upper_body_joints: tuple[str, ...],
        config: SafetyConfig | None = None,
    ) -> None:
        self.config = config or SafetyConfig()
        self.config.validate()
        self.joint_names = tuple(name for name, _lower, _upper in joint_ranges)
        unknown = sorted(set(upper_body_joints) - set(self.joint_names))
        if unknown:
            raise SafetyError(f"upper-body joints are not in the joint layout: {unknown}")
        margin = self.config.position_margin_rad
        self._lower = np.asarray([lower + margin for _name, lower, _upper in joint_ranges])
        self._upper = np.asarray([upper - margin for _name, _lower, upper in joint_ranges])
        collapsed = [name for name, low, high in zip(self.joint_names, self._lower, self._upper)
                     if low > high]
        if collapsed:
            raise SafetyError("position margin collapses the range for: " + ", ".join(collapsed))
        self._upper_body = np.asarray([name in upper_body_joints for name in self.joint_names])
        self._target: np.ndarray | None = None
        self._source_timestamp: float | None = None
        self._output: np.ndarray | None = None
        self._anchor: np.ndarray | None = None
        self._blend = 0.0
        self._last_time: float | None = None
        self._latched = False
        self.last_error: str | None = None

    @property
    def has_target(self) -> bool:
        return self._target is not None

    @property
    def reset_required(self) -> bool:
        return self._latched

    def _validated(self, values: np.ndarray, *, label: str) -> np.ndarray:
        array = np.asarray(values, dtype=np.float64)
        if array.shape != (len(self.joint_names),):
            raise SafetyError(f"{label} must have shape ({len(self.joint_names)},), got {array.shape}")
        if not np.isfinite(array).all():
            raise SafetyError(f"{label} contains a non-finite value")
        return array

    def reset(self, measured_dof_pos: np.ndarray, *, now: float) -> None:
        """Hold the current simulated pose and require a new target."""
        measured = self._validated(measured_dof_pos, label="measured_dof_pos")
        self._output = measured.copy()
        self._anchor = measured.copy()
        self._target = None
        self._source_timestamp = None
        self._blend = 0.0
        self._last_time = float(now)
        self._latched = False
        self.last_error = None

    def update_target(self, target_dof_pos: np.ndarray, *, source_timestamp: float) -> None:
        values = self._validated(target_dof_pos, label="target_dof_pos")
        timestamp = float(source_timestamp)
        if not math.isfinite(timestamp):
            raise SafetyError("source_timestamp is not finite")
        self._target = values.copy()
        self._source_timestamp = timestamp

    def _output_for(self, dof: np.ndarray, reason: str, *, engaged: bool, clamped: tuple[str, ...],
                    source_age: float, dt: float) -> LiveSafetyOutput:
        previous = self._output if self._output is not None else dof
        rate = (dof - previous) / dt if dt > 0 else np.zeros_like(dof)
        self._output = dof.copy()
        return LiveSafetyOutput(
            dof_pos=dof.copy(),
            velocities={name: float(rate[i]) for i, name in enumerate(self.joint_names)
                        if self._upper_body[i]},
            blend_weight=self._blend,
            engaged=engaged,
            reason=reason,
            clamped_joints=clamped,
            source_age_s=source_age,
            reset_required=self._latched,
        )

    def step(self, measured_dof_pos: np.ndarray, *, now: float, enable: bool) -> LiveSafetyOutput:
        measured = self._validated(measured_dof_pos, label="measured_dof_pos")
        if self._output is None or self._anchor is None or self._last_time is None:
            self.reset(measured, now=now)
        assert self._output is not None and self._anchor is not None and self._last_time is not None
        dt = max(0.0, float(now) - self._last_time)
        self._last_time = float(now)
        hold = self._output.copy()
        if self._target is None or self._source_timestamp is None:
            return self._output_for(hold, "waiting_for_fresh_target", engaged=False, clamped=(),
                                    source_age=0.0, dt=dt)
        source_age = max(0.0, float(now) - self._source_timestamp)
        if self._latched or source_age > self.config.hard_timeout_s:
            self._latched = True
            self._blend = 0.0
            return self._output_for(hold, "hard_timeout", engaged=False, clamped=(),
                                    source_age=source_age, dt=dt)
        if source_age > self.config.stale_timeout_s:
            # Hold here and blend back in from this pose once fresh targets resume.
            self._blend = 0.0
            self._anchor = hold.copy()
            return self._output_for(hold, "stale_source", engaged=False, clamped=(),
                                    source_age=source_age, dt=dt)

        bounded = np.clip(self._target, self._lower, self._upper)
        # GMR's IK already respects the MJCF ranges; report only real clamps,
        # not solver round-off at a limit.
        clamped = tuple(name for i, name in enumerate(self.joint_names)
                        if abs(bounded[i] - self._target[i]) > CLAMP_REPORT_TOLERANCE_RAD)
        command = bounded.copy()
        if enable:
            self._blend = min(1.0, self._blend + self.config.blend_rate_per_s * dt)
            reason = "tracking"
        else:
            # Disabled: freeze the upper body where it is and re-blend from
            # there when tracking is enabled again.
            self._blend = 0.0
            self._anchor = hold.copy()
            reason = "disabled"
        upper = self._upper_body
        command[upper] = self._anchor[upper] + self._blend * (bounded[upper] - self._anchor[upper])
        return self._output_for(command, reason, engaged=enable and self._blend > 0.0,
                                clamped=clamped, source_age=source_age, dt=dt)
