"""Pure-Python safety envelope for a future G1 arm-SDK publisher."""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real
from typing import Mapping

from .schema import G1_ARM_SDK_JOINTS, JointLimit


@dataclass(frozen=True)
class SafetyConfig:
    position_margin_rad: float = 0.05
    velocity_limit_fraction: float = 0.25
    acceleration_limit_rad_s2: float = 4.0
    stale_timeout_s: float = 0.10
    hard_timeout_s: float = 0.50
    blend_rate_per_s: float = 1.0
    #: Small allowance for a source timestamp taken just after the caller
    #: sampled ``now``. Larger future offsets indicate mixed clock domains;
    #: they would otherwise read as a perfectly fresh source and must fail
    #: closed instead.
    source_future_tolerance_s: float = 0.005
    #: Longest control interval credited as motion time for one command. A
    #: longer tick (anything short of ``hard_timeout_s``) is treated as a
    #: stall followed by one step of this length, so a loop gap cannot turn
    #: into a ``velocity_cap * dt`` position jump in a single command. The
    #: default is twice the 50 Hz bridge period; slower loops should raise it.
    max_step_dt_s: float = 0.04

    def validate(self) -> None:
        """Reject settings which could weaken or disable the safety envelope."""

        values = {
            "position_margin_rad": self.position_margin_rad,
            "velocity_limit_fraction": self.velocity_limit_fraction,
            "acceleration_limit_rad_s2": self.acceleration_limit_rad_s2,
            "stale_timeout_s": self.stale_timeout_s,
            "hard_timeout_s": self.hard_timeout_s,
            "blend_rate_per_s": self.blend_rate_per_s,
            "source_future_tolerance_s": self.source_future_tolerance_s,
            "max_step_dt_s": self.max_step_dt_s,
        }
        for name, value in values.items():
            if isinstance(value, bool) or not isinstance(value, Real):
                raise SafetyError(f"{name} must be a finite number")
            if not math.isfinite(float(value)):
                raise SafetyError(f"{name} must be a finite number")

        if self.position_margin_rad < 0.0:
            raise SafetyError("position_margin_rad must be non-negative")
        if not 0.0 < self.velocity_limit_fraction <= 1.0:
            raise SafetyError("velocity_limit_fraction must be greater than 0 and at most 1")
        if self.acceleration_limit_rad_s2 <= 0.0:
            raise SafetyError("acceleration_limit_rad_s2 must be positive")
        if self.stale_timeout_s <= 0.0:
            raise SafetyError("stale_timeout_s must be positive")
        if self.hard_timeout_s <= self.stale_timeout_s:
            raise SafetyError("hard_timeout_s must be greater than stale_timeout_s")
        if self.blend_rate_per_s <= 0.0:
            raise SafetyError("blend_rate_per_s must be positive")
        if not 0.0 <= self.source_future_tolerance_s < self.stale_timeout_s:
            raise SafetyError(
                "source_future_tolerance_s must be non-negative and smaller than "
                "stale_timeout_s"
            )
        if not 0.0 < self.max_step_dt_s < self.hard_timeout_s:
            # Any dt >= hard_timeout_s is refused outright, so a larger value
            # would silently disable the per-command displacement bound.
            raise SafetyError(
                "max_step_dt_s must be positive and smaller than hard_timeout_s"
            )


@dataclass(frozen=True)
class SafetyCommand:
    positions: dict[str, float]
    velocities: dict[str, float]
    blend_weight: float
    engaged: bool
    reason: str
    clamped_joints: tuple[str, ...]
    source_age_s: float


class SafetyError(ValueError):
    """Raised when a command cannot be validated safely."""


class ArmSafetyEnvelope:
    """Stateful position/rate/acceleration/staleness guard.

    The class has no Unitree SDK dependency. A hardware adapter must publish
    only the returned positions and blend weight, and must initialize/reset the
    envelope from measured robot state before enabling control. Hard source
    timeouts and control-loop stalls require another measured-state reset.
    """

    def __init__(self, limits: Mapping[str, JointLimit], config: SafetyConfig | None = None):
        self.config = config or SafetyConfig()
        self.config.validate()
        missing = [name for name in G1_ARM_SDK_JOINTS if name not in limits]
        if missing:
            raise SafetyError(f"missing limits for: {', '.join(missing)}")
        self.limits = {name: limits[name] for name in G1_ARM_SDK_JOINTS}
        for name, limit in self.limits.items():
            values = (limit.lower, limit.upper, limit.velocity)
            if not all(math.isfinite(float(value)) for value in values):
                raise SafetyError(f"limits for {name} must be finite")
            if limit.lower >= limit.upper:
                raise SafetyError(f"position limits for {name} must have lower < upper")
            if limit.velocity <= 0.0:
                raise SafetyError(f"velocity limit for {name} must be positive")
            lower = limit.lower + self.config.position_margin_rad
            upper = limit.upper - self.config.position_margin_rad
            if lower >= upper:
                raise SafetyError(f"position margin collapses the range for {name}")
        self._positions: dict[str, float] | None = None
        self._velocities = {name: 0.0 for name in G1_ARM_SDK_JOINTS}
        self._last_time: float | None = None
        self._blend = 0.0
        self._hard_timeout_latched = False
        self._reset_required = False

    @property
    def hard_timeout_latched(self) -> bool:
        """True after a source hard timeout until the next measured-state reset."""
        return self._hard_timeout_latched

    def reset(self, measured_positions: Mapping[str, float], *, now: float) -> None:
        now = float(now)
        if not math.isfinite(now):
            raise SafetyError("reset time must be finite")
        missing = [name for name in G1_ARM_SDK_JOINTS if name not in measured_positions]
        unknown = sorted(set(measured_positions) - set(G1_ARM_SDK_JOINTS))
        if missing or unknown:
            raise SafetyError(
                f"reset requires exactly the arm-SDK joints; missing={missing}, unknown={unknown}"
            )
        positions = {}
        for name in G1_ARM_SDK_JOINTS:
            value = float(measured_positions[name])
            if not math.isfinite(value):
                raise SafetyError(f"reset position for {name} is not finite")
            limit = self.limits[name]
            if not limit.lower <= value <= limit.upper:
                raise SafetyError(f"measured {name}={value} is outside the official URDF range")
            positions[name] = value
        self._positions = positions
        self._velocities = {name: 0.0 for name in G1_ARM_SDK_JOINTS}
        self._last_time = now
        self._blend = 0.0
        self._hard_timeout_latched = False
        self._reset_required = False

    def step(
        self,
        target_positions: Mapping[str, float],
        *,
        source_timestamp: float,
        now: float,
        enable: bool,
    ) -> SafetyCommand:
        if self._positions is None or self._last_time is None:
            raise SafetyError("reset from measured state before calling step")
        if self._reset_required:
            raise SafetyError("reset from measured state after a control-loop stall")
        now = float(now)
        dt = now - self._last_time
        if not math.isfinite(dt) or dt <= 0:
            raise SafetyError(f"control time must increase monotonically; dt={dt}")
        if dt >= self.config.hard_timeout_s:
            # No command was emitted during this interval, so the stored state
            # is no longer a trustworthy starting point for another command.
            # Requiring measured-state reset avoids treating the whole gap as
            # time in which a large position step could safely occur.
            self._hard_timeout_latched = True
            self._blend = 0.0
            self._reset_required = True
            raise SafetyError(
                f"control loop stalled for {dt:.6g}s; reset from measured state"
            )
        source_time = float(source_timestamp)
        if not math.isfinite(source_time):
            raise SafetyError("source timestamp is not finite")
        raw_source_age = now - source_time
        if raw_source_age < -self.config.source_future_tolerance_s:
            # Clamping this to zero would make a future (mixed-clock or forged)
            # timestamp look permanently fresh, so stale and hard timeouts
            # could never fire.
            raise SafetyError(
                f"source timestamp is {-raw_source_age:.6g}s in the future; "
                "source and control clocks do not match"
            )
        # Only a callback race within the tolerance reaches this clamp.
        source_age = max(0.0, raw_source_age)

        missing = [name for name in G1_ARM_SDK_JOINTS if name not in target_positions]
        unknown = sorted(set(target_positions) - set(G1_ARM_SDK_JOINTS))
        if missing or unknown:
            raise SafetyError(
                f"target requires exactly the arm-SDK joints; missing={missing}, unknown={unknown}"
            )

        if source_age >= self.config.hard_timeout_s:
            self._hard_timeout_latched = True

        if self._hard_timeout_latched:
            requested = False
            self._blend = 0.0
            reason = "hard_timeout"
        elif source_age >= self.config.stale_timeout_s:
            requested = False
            reason = "stale_source"
        elif not enable:
            requested = False
            reason = "disabled"
        else:
            requested = True
            reason = "tracking"

        # Motion is evaluated over at most ``max_step_dt_s``. Commands were
        # held during the rest of a long tick, so crediting that remainder as
        # motion time would legalise a ``velocity_cap * dt`` jump in this one
        # command. Treating it as a stall instead makes the command after a gap
        # look like a nominal step, and every bound below (velocity,
        # acceleration, stopping distance) holds with ``step_dt`` in place of
        # ``dt``.
        step_dt = min(dt, self.config.max_step_dt_s)

        # Authority ramps up over the same credited interval, but ramps down
        # over wall-clock time so a hand-back is never slowed by a long tick.
        if requested:
            self._blend = min(1.0, self._blend + self.config.blend_rate_per_s * step_dt)
        else:
            self._blend = max(0.0, self._blend - self.config.blend_rate_per_s * dt)

        next_positions: dict[str, float] = {}
        next_velocities: dict[str, float] = {}
        clamped: list[str] = []
        for name in G1_ARM_SDK_JOINTS:
            current_position = self._positions[name]
            previous_velocity = self._velocities[name]
            limit = self.limits[name]
            lower = limit.lower + self.config.position_margin_rad
            upper = limit.upper - self.config.position_margin_rad
            if lower >= upper:
                raise SafetyError(f"position margin collapses the range for {name}")

            if requested:
                raw = float(target_positions[name])
                if not math.isfinite(raw):
                    raise SafetyError(f"target for {name} is not finite")
                target = min(max(raw, lower), upper)
                if target != raw:
                    clamped.append(name)
                desired_velocity = (target - current_position) / step_dt
            else:
                # Disengagement is a bounded deceleration, not an instantaneous
                # internal stop. This keeps disable and timeout commands within
                # the same acceleration envelope as tracking commands.
                desired_velocity = 0.0

            max_velocity = limit.velocity * self.config.velocity_limit_fraction
            acceleration = self.config.acceleration_limit_rad_s2
            accel_delta = acceleration * step_dt

            # If reset starts in the URDF-valid margin region, allow a gradual
            # move back into the preferred range without either jumping to the
            # margin or moving farther toward the official hard stop.
            effective_lower = min(lower, current_position)
            effective_upper = max(upper, current_position)

            # Bound this step's velocity so that, after emitting the resulting
            # position, continuous max-acceleration braking can still stop at
            # the effective position boundary.  The -a*step_dt term accounts
            # for this step's forward-Euler displacement.
            lower_distance = max(0.0, current_position - effective_lower)
            upper_distance = max(0.0, effective_upper - current_position)
            negative_stopping_speed = max(
                0.0,
                math.sqrt(accel_delta * accel_delta + 2.0 * acceleration * lower_distance)
                - accel_delta,
            )
            positive_stopping_speed = max(
                0.0,
                math.sqrt(accel_delta * accel_delta + 2.0 * acceleration * upper_distance)
                - accel_delta,
            )

            minimum_velocity = max(
                -max_velocity,
                previous_velocity - accel_delta,
                -negative_stopping_speed,
            )
            maximum_velocity = min(
                max_velocity,
                previous_velocity + accel_delta,
                positive_stopping_speed,
            )
            if minimum_velocity > maximum_velocity + 1e-12:
                raise SafetyError(
                    f"internal safety state for {name} cannot satisfy position and "
                    "acceleration limits; reset from measured state"
                )
            if minimum_velocity > maximum_velocity:
                # Collapse a floating-point-only interval inversion.
                midpoint = (minimum_velocity + maximum_velocity) / 2.0
                minimum_velocity = maximum_velocity = midpoint

            velocity = min(max(desired_velocity, minimum_velocity), maximum_velocity)
            position = current_position + velocity * step_dt
            position = min(max(position, effective_lower), effective_upper)

            # Reconcile state to what will actually be emitted. This matters at
            # a position boundary: retaining the pre-saturation velocity makes
            # the next command's acceleration calculation unsafe.  The
            # velocity is over ``step_dt``, the interval the position change
            # was credited to, so it stays consistent with the displacement.
            emitted_velocity = (position - current_position) / step_dt
            next_positions[name] = position
            next_velocities[name] = emitted_velocity

        self._positions = next_positions
        self._velocities = next_velocities
        self._last_time = now
        return SafetyCommand(
            positions=dict(next_positions),
            velocities=dict(next_velocities),
            blend_weight=self._blend,
            engaged=requested and self._blend > 0.0,
            reason=reason,
            clamped_joints=tuple(clamped),
            source_age_s=source_age,
        )
