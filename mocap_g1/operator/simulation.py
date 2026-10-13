"""Fail-closed recorded-trajectory simulation for the operator backend.

This runtime replays the repository's strict ``qpos`` NPZ contract against the
existing arm safety envelope.  It is an in-process kinematic preview only: it
does not create a robot transport, grant hardware authority, or model physics.
"""

from __future__ import annotations

from hashlib import sha256
import math
from pathlib import Path
import threading
import time
from typing import Any, Callable, Mapping

import numpy as np

from ..assets import official_urdf_path
from ..conditioning import MotionConditioningError, load_qpos_npz
from ..live_safety import LiveG1SafetyController, LiveSafetyOutput
from ..safety import SafetyConfig
from ..schema import G1_29DOF_JOINTS, G1_ARM_SDK_JOINTS, load_official_joint_limits
from .models import HealthSnapshot


# Packaged with ``mocap_g1`` so a non-editable install can run ``--simulation``.
DEFAULT_LIMITS_URDF = official_urdf_path()
DEFAULT_COMMAND_RATE_HZ = 60.0
_MAX_CATCH_UP_STEPS = 10_000
_DOF_INDEX = {name: index for index, name in enumerate(G1_29DOF_JOINTS)}
_LIMITATIONS = (
    "Recorded NPZ playback only; there is no live mocap or robot transport",
    (
        "Root and leg motion are kinematic references; only the named waist and "
        "arm joints pass through LiveG1SafetyController"
    ),
    (
        "No actuator dynamics, balance control, contact stability, torque, or "
        "hardware validation is modeled"
    ),
)


class RecordedSimulationConfigurationError(ValueError):
    """Raised when a recorded-simulation source cannot be loaded safely."""


def resolve_simulation_path(value: str | Path) -> Path:
    """Resolve one local NPZ path without accepting directories or other types."""

    source = Path(value).expanduser()
    if source.suffix.lower() != ".npz":
        raise RecordedSimulationConfigurationError(
            "recorded simulation source must end in .npz"
        )
    try:
        resolved = source.resolve(strict=True)
    except OSError as exc:
        raise RecordedSimulationConfigurationError(
            f"recorded simulation NPZ does not exist: {source}"
        ) from exc
    if not resolved.is_file():
        raise RecordedSimulationConfigurationError(
            f"recorded simulation source is not a regular file: {resolved}"
        )
    return resolved


def _file_sha256(path: Path) -> str:
    digest = sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise RecordedSimulationConfigurationError(
            f"cannot read recorded simulation NPZ {path}: {exc}"
        ) from exc
    return digest.hexdigest()


class RecordedSimulationRuntime:
    """Replay a validated qpos sequence on a monotonic, safety-bounded timeline."""

    # ``is_simulated`` selects fake-mode configuration.  The demo hardware
    # bypass remains false; virtual state transitions never alter the locked
    # hardware capability or create a transport.
    is_simulated = True
    allows_hardware_bypass = False
    uses_recorded_source = True
    uses_virtual_robot = True
    supports_operator_recovery = True

    def __init__(
        self,
        source: str | Path,
        *,
        clock: Callable[[], float] = time.monotonic,
        command_rate_hz: float = DEFAULT_COMMAND_RATE_HZ,
        safety_config: SafetyConfig | None = None,
        limits_urdf: str | Path = DEFAULT_LIMITS_URDF,
    ) -> None:
        self.source_path = resolve_simulation_path(source)
        try:
            qpos, fps, metadata = load_qpos_npz(self.source_path)
        except MotionConditioningError as exc:
            raise RecordedSimulationConfigurationError(str(exc)) from exc

        rate = float(command_rate_hz)
        if not math.isfinite(rate) or rate <= 0.0:
            raise RecordedSimulationConfigurationError(
                "recorded simulation command_rate_hz must be positive and finite"
            )
        try:
            limits = load_official_joint_limits(limits_urdf)
            safety = LiveG1SafetyController(limits, safety_config)
        except (OSError, ValueError) as exc:
            raise RecordedSimulationConfigurationError(
                f"cannot initialize recorded simulation safety limits: {exc}"
            ) from exc

        self._lock = threading.RLock()
        self._clock = clock
        self._qpos = qpos
        self._fps = float(fps)
        self._metadata = metadata
        self._limits = limits
        self._safety_config = safety_config or SafetyConfig()
        self._safety = safety
        self._command_rate_hz = rate
        self._command_period_s = 1.0 / rate
        self._frame_count = int(len(qpos))
        self._duration_s = (self._frame_count - 1) / self._fps
        self._sha256 = _file_sha256(self.source_path)
        self._source_available = True
        self._tracking_notified = False

        now = self._now()
        self._reference_qpos = self._qpos[0].copy()
        self._bounded_qpos, initial_clamped = self._safe_initial_qpos(
            self._reference_qpos
        )
        self._safety.reset(self._bounded_qpos[7:], now=now)
        self._state = "ready"
        self._frame_index = 0
        self._playhead_s = 0.0
        self._last_command_time = now
        self._source_age_ms = 0.0
        self._safety_reason = "waiting_for_start"
        self._blend_weight = 0.0
        self._reset_required = False
        self._clamped_joint_count = initial_clamped

    def _now(self) -> float:
        now = float(self._clock())
        if not math.isfinite(now):
            raise RecordedSimulationConfigurationError(
                "recorded simulation monotonic clock returned a non-finite value"
            )
        return now

    def _safe_initial_qpos(self, reference: np.ndarray) -> tuple[np.ndarray, int]:
        bounded = np.asarray(reference, dtype=np.float64).copy()
        clamped = 0
        for name in G1_ARM_SDK_JOINTS:
            index = 7 + _DOF_INDEX[name]
            limit = self._limits[name]
            lower = limit.lower + self._safety_config.position_margin_rad
            upper = limit.upper - self._safety_config.position_margin_rad
            value = float(np.clip(bounded[index], lower, upper))
            if value != bounded[index]:
                clamped += 1
            bounded[index] = value
        return bounded, clamped

    def check(self, health: HealthSnapshot, config: Mapping[str, Any]) -> Mapping[str, Any]:
        del health, config
        with self._lock:
            if self._state == "stopped":
                self._state = "ready"
            return {
                "mocap_online": True,
                "mocap_rate_hz": self._fps,
                "packet_loss_percent": 0.0,
                "source_age_ms": 0.0,
                "controller_ready": False,
                "command_authority": False,
                "overall": "checking",
                "messages": (
                    "Recorded NPZ source loaded; no live mocap or hardware authority",
                ),
            }

    def start_simulation(
        self, health: HealthSnapshot, config: Mapping[str, Any]
    ) -> Mapping[str, Any]:
        del health, config
        with self._lock:
            now = self._now()
            self._reference_qpos = self._qpos[0].copy()
            self._bounded_qpos, initial_clamped = self._safe_initial_qpos(
                self._reference_qpos
            )
            self._safety.reset(self._bounded_qpos[7:], now=now)
            self._safety.update_target(
                self._reference_qpos[7:], source_timestamp=now
            )
            self._frame_index = 0
            self._playhead_s = 0.0
            self._last_command_time = now
            self._source_available = True
            self._tracking_notified = False
            self._state = "playing"
            self._source_age_ms = 0.0
            self._safety_reason = "disabled"
            self._blend_weight = 0.0
            self._reset_required = False
            self._clamped_joint_count = initial_clamped
            return {
                "simulation_ready": True,
                "command_authority": False,
                "overall": "ready",
                "messages": (
                    "Recorded kinematic simulation started with hardware authority disabled",
                ),
            }

    def connect_robot(
        self, health: HealthSnapshot, config: Mapping[str, Any]
    ) -> Mapping[str, Any]:
        del health, config
        return {
            "robot_connected": True,
            "robot_state_fresh": True,
            "controller_ready": True,
            "command_authority": False,
            "messages": (
                "Virtual recorded-simulation robot ready; no transport was opened",
            ),
        }

    def tick(self, health: HealthSnapshot) -> Mapping[str, Any]:
        """Advance generated safety samples to the current monotonic time."""

        del health
        with self._lock:
            if self._state == "playing":
                self._advance_locked(self._now())
            return {"source_age_ms": self._source_age_ms}

    def _advance_locked(self, now: float) -> None:
        delta = now - self._last_command_time
        if delta < 0.0:
            self._state = "blocked_reset_required"
            self._safety_reason = "non_monotonic_clock"
            self._blend_weight = 0.0
            self._reset_required = True
            return

        steps = int(math.floor(delta / self._command_period_s + 1e-9))
        if steps <= 0:
            return
        if steps > _MAX_CATCH_UP_STEPS:
            output = self._safety.step(
                self._bounded_qpos[7:], now=now, enable=False
            )
            self._last_command_time = now
            self._apply_output_locked(output, fresh_reference=False)
            self._state = "blocked_reset_required"
            return

        for _ in range(steps):
            command_time = self._last_command_time + self._command_period_s
            fresh_reference = self._source_available
            if fresh_reference:
                self._playhead_s = min(
                    self._duration_s,
                    self._playhead_s + self._command_period_s,
                )
                self._frame_index = min(
                    int(math.floor(self._playhead_s * self._fps + 1e-9)),
                    self._frame_count - 1,
                )
                self._reference_qpos = self._qpos[self._frame_index].copy()
                self._safety.update_target(
                    self._reference_qpos[7:], source_timestamp=command_time
                )

            output = self._safety.step(
                self._bounded_qpos[7:],
                now=command_time,
                enable=self._tracking_notified,
            )
            self._last_command_time = command_time
            self._apply_output_locked(output, fresh_reference=fresh_reference)
            if output.reset_required:
                self._state = "blocked_reset_required"
                return
            if self._playhead_s >= self._duration_s:
                self._frame_index = self._frame_count - 1
                self._reference_qpos = self._qpos[-1].copy()
                if self._tracking_notified:
                    self._restart_timeline_locked(command_time)
                    continue
                self._state = "completed"
                return

    def _apply_output_locked(
        self, output: LiveSafetyOutput, *, fresh_reference: bool
    ) -> None:
        if fresh_reference:
            self._bounded_qpos[:7] = self._reference_qpos[:7]
        self._bounded_qpos[7:] = output.dof_pos
        self._source_age_ms = max(0.0, float(output.source_age_s) * 1000.0)
        self._safety_reason = output.reason
        self._blend_weight = float(output.blend_weight)
        self._reset_required = bool(output.reset_required)
        self._clamped_joint_count = len(output.clamped_joints)

    def set_tracking(self, enabled: bool) -> None:
        """Accept the controller's optional tracking notification without authority."""

        with self._lock:
            requested = bool(enabled)
            if requested:
                now = self._now()
                self._tracking_notified = True
                self._restart_timeline_locked(now)
            else:
                self._tracking_notified = False

    def _restart_timeline_locked(self, now: float) -> None:
        """Restart the recording from the current bounded arm pose."""

        self._reference_qpos = self._qpos[0].copy()
        # Root and legs are kinematic.  Preserve the bounded waist/arm pose so
        # the safety envelope, not a loop boundary, controls their transition.
        self._bounded_qpos[:19] = self._reference_qpos[:19]
        self._safety.reset(self._bounded_qpos[7:], now=now)
        self._safety.update_target(
            self._reference_qpos[7:], source_timestamp=now
        )
        self._frame_index = 0
        self._playhead_s = 0.0
        self._last_command_time = now
        self._source_age_ms = 0.0
        self._safety_reason = "tracking" if self._tracking_notified else "disabled"
        self._blend_weight = 0.0
        self._reset_required = False
        self._clamped_joint_count = 0
        self._state = "playing"

    def safe_stop(self, health: HealthSnapshot) -> Mapping[str, Any]:
        del health
        with self._lock:
            if self._state == "playing":
                self._advance_locked(self._now())
            self._tracking_notified = False
            self._state = "stopped"
            self._blend_weight = 0.0
            self._safety_reason = "safe_stop"
            return {
                "command_authority": False,
                "overall": "ready",
                "messages": (
                    "Recorded simulation stopped; hardware authority remains disabled",
                ),
            }

    def shutdown(self, health: HealthSnapshot) -> Mapping[str, Any]:
        changes = dict(self.safe_stop(health))
        with self._lock:
            self._state = "stopped"
            changes["simulation_ready"] = False
        return changes

    def set_source_available(self, available: bool) -> None:
        """Test/diagnostic seam for exercising stale-source behavior."""

        with self._lock:
            self._source_available = bool(available)

    def recover(
        self, health: HealthSnapshot | None = None
    ) -> Mapping[str, Any]:
        """Explicitly reset the envelope after a source timeout or loop stall."""

        del health
        with self._lock:
            now = self._now()
            self._source_available = True
            self._tracking_notified = False
            self._restart_timeline_locked(now)
            self._safety_reason = "recovered"
            return {
                "source_age_ms": 0.0,
                "simulation_ready": True,
                "robot_connected": True,
                "robot_state_fresh": True,
                "controller_ready": True,
                "command_authority": False,
                "overall": "ready",
                "messages": (
                    "Recorded simulation safety state reset; tracking remains disabled",
                ),
            }

    def simulation_snapshot(self) -> dict[str, Any]:
        with self._lock:
            progress = (
                100.0
                if self._duration_s == 0.0
                else min(100.0, 100.0 * self._playhead_s / self._duration_s)
            )
            return {
                "mode": "recorded_simulation",
                "state": self._state,
                "source": {
                    "name": self.source_path.name,
                    "frame_count": self._frame_count,
                    "fps": self._fps,
                    "duration_s": self._duration_s,
                    "sha256": self._sha256,
                },
                "frame_index": self._frame_index,
                "progress_pct": progress,
                "reference_qpos": self._reference_qpos.tolist(),
                "bounded_qpos": self._bounded_qpos.tolist(),
                "telemetry": {
                    "source_age_ms": self._source_age_ms,
                    "source_rate_hz": self._fps,
                    "command_rate_hz": self._command_rate_hz,
                    "safety_reason": self._safety_reason,
                    "blend_weight": self._blend_weight,
                    "reset_required": self._reset_required,
                    "clamped_joint_count": self._clamped_joint_count,
                },
                "limitations": list(_LIMITATIONS),
            }


__all__ = [
    "DEFAULT_COMMAND_RATE_HZ",
    "DEFAULT_LIMITS_URDF",
    "RecordedSimulationConfigurationError",
    "RecordedSimulationRuntime",
    "resolve_simulation_path",
]
