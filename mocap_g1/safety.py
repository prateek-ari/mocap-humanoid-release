"""Pure-Python safety envelope for a future G1 arm-SDK publisher."""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real
from typing import Collection, Mapping, Protocol, Sequence

from .schema import G1_ARM_SDK_JOINTS, JointLimit


#: Arm-through-body bound (metres): no arm deeper than this inside the body, a
#: few millimetres being the tolerance for the simplified collision shapes.
CONTACT_BOUND_M = 0.005


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
    #: longer tick (anything short of ``tick_stall_s``) is treated as a
    #: stall followed by one step of this length, so a loop gap cannot turn
    #: into a ``velocity_cap * dt`` position jump in a single command. The
    #: default is twice the 50 Hz bridge period; slower loops should raise it.
    max_step_dt_s: float = 0.04
    #: Penetration (metres) a contact check tolerates for its simplified
    #: collision shapes; deeper candidate commands are braked.  It sits 1 mm
    #: inside ``CONTACT_BOUND_M``, so path sampling at the limit cannot cross it.
    contact_tolerance_m: float = 0.004
    #: Longest gap between two control ticks before the envelope latches a
    #: stall (reset from measured state).  ``None`` means ``hard_timeout_s``,
    #: as before; a live bridge sets it apart from the source timeout so a
    #: long braking-derived source timeout cannot weaken the stall check.
    tick_stall_timeout_s: float | None = None

    @property
    def tick_stall_s(self) -> float:
        """The effective control-loop stall bound."""
        return self.hard_timeout_s if self.tick_stall_timeout_s is None else self.tick_stall_timeout_s

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
            "contact_tolerance_m": self.contact_tolerance_m,
            "tick_stall_timeout_s": self.tick_stall_s,
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
        if self.contact_tolerance_m < 0.0:
            raise SafetyError("contact_tolerance_m must be non-negative")
        if self.tick_stall_s <= self.max_step_dt_s:
            # A stall bound at or below one credited step would fault nominal ticks.
            raise SafetyError("tick_stall_timeout_s must be greater than max_step_dt_s")


@dataclass(frozen=True)
class SafetyCommand:
    positions: dict[str, float]
    velocities: dict[str, float]
    blend_weight: float
    engaged: bool
    reason: str
    clamped_joints: tuple[str, ...]
    source_age_s: float
    #: "unchecked" without a contact check; otherwise "clear", "braked" (some
    #: joints braked to stay clear) or "infeasible" (braking could not stay clear).
    contact: str = "unchecked"
    #: Deepest penetration of the emitted pose, in metres.
    contact_depth_m: float = 0.0
    braked_joints: tuple[str, ...] = ()


class SafetyError(ValueError):
    """Raised when a command cannot be validated safely."""


class ContactCheck(Protocol):
    """Penetration of candidate poses, checked inside the envelope.

    ``groups`` maps each contact group (one arm, say) to the guarded joints
    that move only it; guarded joints in no group (the waist) move every group.
    """

    groups: Mapping[str, tuple[str, ...]]

    def depths(self, positions: Mapping[str, float]) -> Mapping[str, float]:
        """Deepest penetration (metres, >= 0) of each group at these guarded positions."""
        ...

    # Optional: ``clear_of(positions, travel) -> bool``, True only when no
    # group can touch while each joint moves at most ``travel`` radians; lets
    # the envelope skip the full check far from the body.


#: Mean joint travel (radians, about 1 cm at a G1 hand) between the poses
#: checked along a candidate command's braking path; the poses are evenly spaced
#: in time, so the first interval is about twice this.  From 3 poses (a short
#: path sliding along a curved surface can dip between its ends) to
#: ``CONTACT_LOOKAHEAD_MAX_SAMPLES``.  Above about 1.6 rad/s the cap spaces the
#: poses wider, so a thin link could fall between two of them; the emitted pose
#: is still checked every tick, so that shows as a late "infeasible", reported.
CONTACT_LOOKAHEAD_STEP_RAD = 0.02
CONTACT_LOOKAHEAD_MIN_SAMPLES = 3
CONTACT_LOOKAHEAD_MAX_SAMPLES = 16
#: Single joints a blocked arm may try to release per tick.
CONTACT_MAX_RELEASES = 4
#: Finite-difference step for the depth gradient a blocked arm slides along.
CONTACT_GRADIENT_STEP_RAD = 1e-3
#: A blocked arm deeper than half the tolerance is pushed back out over this time.
CONTACT_RECOVERY_S = 0.1


def stopping_speed(distance: float, acceleration: float, accel_delta: float) -> float:
    """Fastest speed this step from which max-acceleration braking stops within
    ``distance``; the ``-accel_delta`` term accounts for this step's own
    forward-Euler displacement."""
    return max(
        0.0,
        math.sqrt(accel_delta * accel_delta + 2.0 * acceleration * distance) - accel_delta,
    )


#: What :meth:`ArmSafetyEnvelope.snapshot` returns: positions, velocities,
#: targets, target velocities, last time, blend, hard-timeout latch and
#: reset-required flag.
EnvelopeState = tuple[
    dict[str, float] | None, dict[str, float], dict[str, float], dict[str, float],
    float | None, float, bool, bool
]


class ArmSafetyEnvelope:
    """Stateful position/rate/acceleration/staleness guard.

    The class has no Unitree SDK dependency. A hardware adapter must publish
    only the returned positions and blend weight, and must initialize/reset the
    envelope from measured robot state before enabling control. Hard source
    timeouts and control-loop stalls require another measured-state reset.
    """

    def __init__(
        self,
        limits: Mapping[str, JointLimit],
        config: SafetyConfig | None = None,
        joints: Sequence[str] = G1_ARM_SDK_JOINTS,
    ):
        """``joints`` are the guarded joints, in command order (default: the G1
        waist and arms the arm SDK drives)."""
        self.config = config or SafetyConfig()
        self.config.validate()
        self.joints = tuple(joints)
        if not self.joints:
            raise SafetyError("the envelope needs at least one guarded joint")
        missing = [name for name in self.joints if name not in limits]
        if missing:
            raise SafetyError(f"missing limits for: {', '.join(missing)}")
        self.limits = {name: limits[name] for name in self.joints}
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
        self._velocities = {name: 0.0 for name in self.joints}
        #: Last clamped tracking targets, for the target velocity.
        self._targets: dict[str, float] = {}
        #: Last target velocities, for the target acceleration.
        self._target_velocities: dict[str, float] = {}
        self._last_time: float | None = None
        self._blend = 0.0
        self._hard_timeout_latched = False
        self._reset_required = False

    def snapshot(self) -> EnvelopeState:
        """Copy of the state :meth:`step` changes, for :meth:`restore`."""
        return (None if self._positions is None else dict(self._positions),
                dict(self._velocities), dict(self._targets), dict(self._target_velocities),
                self._last_time, self._blend, self._hard_timeout_latched, self._reset_required)

    def restore(self, snapshot: EnvelopeState) -> None:
        """Undo the steps taken since ``snapshot`` (to try another command)."""
        (self._positions, self._velocities, self._targets, self._target_velocities,
         self._last_time, self._blend, self._hard_timeout_latched,
         self._reset_required) = snapshot

    @property
    def hard_timeout_latched(self) -> bool:
        """True after a source hard timeout until the next measured-state reset."""
        return self._hard_timeout_latched

    def reset(self, measured_positions: Mapping[str, float], *, now: float) -> None:
        now = float(now)
        if not math.isfinite(now):
            raise SafetyError("reset time must be finite")
        missing = [name for name in self.joints if name not in measured_positions]
        unknown = sorted(set(measured_positions) - set(self.joints))
        if missing or unknown:
            raise SafetyError(
                f"reset requires exactly the arm-SDK joints; missing={missing}, unknown={unknown}"
            )
        positions = {}
        for name in self.joints:
            value = float(measured_positions[name])
            if not math.isfinite(value):
                raise SafetyError(f"reset position for {name} is not finite")
            limit = self.limits[name]
            if not limit.lower <= value <= limit.upper:
                raise SafetyError(f"measured {name}={value} is outside the official URDF range")
            positions[name] = value
        self._positions = positions
        self._velocities = {name: 0.0 for name in self.joints}
        self._targets = {}
        self._target_velocities = {}
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
        contact: ContactCheck | None = None,
        yield_groups: Collection[str] = (),
    ) -> SafetyCommand:
        """One bounded command.  With ``contact``, candidate commands whose pose
        or braking path penetrates past the tolerance are braked instead.  The
        arms named in ``yield_groups`` move away from the body as if they were
        in contact (to make room for a leg)."""
        if self._positions is None or self._last_time is None:
            raise SafetyError("reset from measured state before calling step")
        if self._reset_required:
            raise SafetyError("reset from measured state after a control-loop stall")
        now = float(now)
        dt = now - self._last_time
        if not math.isfinite(dt) or dt <= 0:
            raise SafetyError(f"control time must increase monotonically; dt={dt}")
        if dt >= self.config.tick_stall_s:
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

        missing = [name for name in self.joints if name not in target_positions]
        unknown = sorted(set(target_positions) - set(self.joints))
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

        acceleration = self.config.acceleration_limit_rad_s2
        accel_delta = acceleration * step_dt
        clamped: list[str] = []
        next_targets: dict[str, float] = {}
        next_target_velocities: dict[str, float] = {}
        #: Per joint: (min velocity, max velocity, effective lower, effective upper).
        bounds: dict[str, tuple[float, float, float, float]] = {}
        tracking: dict[str, float] = {}
        for name in self.joints:
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
                # Brake ahead of the target: move with the target's own
                # velocity, plus a closing speed no faster than max-acceleration
                # braking can stop within the gap, so a step is reached without
                # overshoot and a moving target is followed without lag.  The
                # gap is to the target's previous position: its motion since
                # then is the target velocity's share of this step.
                previous_target = self._targets.get(name)
                if previous_target is None:
                    target_velocity, gap = 0.0, target - current_position
                else:
                    target_velocity = (target - previous_target) / dt
                    gap = previous_target - current_position
                next_targets[name] = target
                next_target_velocities[name] = target_velocity
                # Plan the closing speed with only the acceleration the target
                # leaves when it accelerates into the gap: planned with the whole
                # cap, it cannot be braked through the reversal of a target that
                # uses most of it, and the envelope cycles instead of locking on.
                # A target beyond the cap (a step, jitter) is not anticipated.
                target_acceleration = (
                    target_velocity - self._target_velocities.get(name, target_velocity)
                ) / dt
                into_gap = -target_acceleration if gap >= 0.0 else target_acceleration
                closing = acceleration
                if 0.0 < into_gap <= acceleration:
                    closing -= into_gap
                desired_velocity = target_velocity + math.copysign(
                    stopping_speed(abs(gap), closing, closing * step_dt), gap
                )
            else:
                # Disengagement is a bounded deceleration, not an instantaneous
                # internal stop. This keeps disable and timeout commands within
                # the same acceleration envelope as tracking commands.
                desired_velocity = 0.0

            max_velocity = limit.velocity * self.config.velocity_limit_fraction

            # If reset starts in the URDF-valid margin region, allow a gradual
            # move back into the preferred range without either jumping to the
            # margin or moving farther toward the official hard stop.
            effective_lower = min(lower, current_position)
            effective_upper = max(upper, current_position)

            # Bound this step's velocity so that, after emitting the resulting
            # position, continuous max-acceleration braking can still stop at
            # the effective position boundary.
            lower_distance = max(0.0, current_position - effective_lower)
            upper_distance = max(0.0, effective_upper - current_position)
            negative_stopping_speed = stopping_speed(lower_distance, acceleration, accel_delta)
            positive_stopping_speed = stopping_speed(upper_distance, acceleration, accel_delta)

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

            bounds[name] = (minimum_velocity, maximum_velocity, effective_lower, effective_upper)
            tracking[name] = min(max(desired_velocity, minimum_velocity), maximum_velocity)

        velocities = tracking
        contact_state, contact_depth = "unchecked", 0.0
        braked: tuple[str, ...] = ()
        if contact is not None:
            velocities, contact_state, contact_depth, braked = self._avoid_contact(
                contact, tracking, bounds, step_dt, set(yield_groups)
            )

        next_positions: dict[str, float] = {}
        next_velocities: dict[str, float] = {}
        for name in self.joints:
            current_position = self._positions[name]
            position = self._advance(name, velocities[name], bounds, step_dt)
            # Reconcile state to what will actually be emitted. This matters at
            # a position boundary: retaining the pre-saturation velocity makes
            # the next command's acceleration calculation unsafe.  The
            # velocity is over ``step_dt``, the interval the position change
            # was credited to, so it stays consistent with the displacement.
            next_positions[name] = position
            next_velocities[name] = (position - current_position) / step_dt

        self._positions = next_positions
        self._velocities = next_velocities
        self._targets = next_targets
        self._target_velocities = next_target_velocities
        self._last_time = now
        return SafetyCommand(
            positions=dict(next_positions),
            velocities=dict(next_velocities),
            blend_weight=self._blend,
            engaged=requested and self._blend > 0.0,
            reason=reason,
            clamped_joints=tuple(clamped),
            source_age_s=source_age,
            contact=contact_state,
            contact_depth_m=contact_depth,
            braked_joints=braked,
        )

    def _advance(
        self,
        name: str,
        velocity: float,
        bounds: Mapping[str, tuple[float, float, float, float]],
        step_dt: float,
    ) -> float:
        assert self._positions is not None
        _minimum, _maximum, lower, upper = bounds[name]
        return min(max(self._positions[name] + velocity * step_dt, lower), upper)

    def _avoid_contact(
        self,
        contact: ContactCheck,
        tracking: Mapping[str, float],
        bounds: Mapping[str, tuple[float, float, float, float]],
        step_dt: float,
        yield_groups: set[str],
    ) -> tuple[dict[str, float], str, float, tuple[str, ...]]:
        """Brake the joints of each group whose candidate pose or braking path
        penetrates past the tolerance, then the shared joints; every choice
        stays inside this step's velocity bounds, so the caps always win.

        A group already deeper than the tolerance (legs moved into it, or a
        reset in contact), or asked to yield, starts blocked: it may not go
        deeper, and its slide pushes it back out.
        """
        assert self._positions is not None
        groups = {group: tuple(joints) for group, joints in contact.groups.items()}
        unknown = sorted({name for joints in groups.values() for name in joints} - set(self.joints))
        if unknown:
            raise SafetyError(f"contact groups name unguarded joints: {unknown}")
        acceleration = self.config.acceleration_limit_rad_s2
        clear_of = getattr(contact, "clear_of", None)
        if clear_of is not None and clear_of(
            self._positions,
            {name: abs(v) * step_dt + v * v / (2.0 * acceleration) for name, v in tracking.items()},
        ):
            # Far from the body: neither tracking nor its braking path can
            # reach it, so the full check would accept tracking unchanged.
            return dict(tracking), "clear", 0.0, ()
        shared = set(self.joints).difference(*groups.values())
        # Braking: the velocity in bounds nearest zero (max deceleration when moving).
        braking = {name: min(max(0.0, low), high) for name, (low, high, _, _) in bounds.items()}
        current = contact.depths(self._positions)
        tolerance = self.config.contact_tolerance_m
        allowed = {group: max(tolerance, float(current[group])) for group in groups}
        #: What each joint does when it is not tracking: brake, or later slide.
        held = dict(braking)

        def trial(braked: set[str]) -> tuple[dict[str, float], Mapping[str, float], list[str]]:
            velocities = {
                name: held[name] if name in braked else tracking[name] for name in self.joints
            }
            emitted, worst = self._lookahead_depths(contact, velocities, bounds, step_dt)
            return velocities, emitted, [g for g in groups if worst[g] > allowed[g]]

        braked: set[str] = set()
        state = "clear"
        while True:
            velocities, emitted, over = trial(braked)
            if not over:
                break
            more = {name for group in over for name in groups[group]} - braked
            if not more:
                more = shared - braked
            if not more:
                # Everything that moves the offending groups already brakes:
                # the dynamic limits win and the penetration is reported.
                state = "infeasible"
                break
            braked |= more
            state = "braked"
        # An arm already deeper than the tolerance (a leg moved into it, or a
        # reset in contact), or asked to yield, also slides, so it backs out.
        recovering = {
            group for group in groups
            if float(current[group]) > tolerance or group in yield_groups
        }
        if state != "infeasible":
            # Braking a whole arm can stall it against the body when the
            # straight path to its target starts inward.  First let each
            # blocked arm slide: track without the part of its motion that goes
            # deeper (plus a push back out), then let single joints track,
            # the largest move first, wherever that stays clear.
            for group, joints in groups.items():
                blocked = list(joints) if group in recovering else [
                    name for name in joints if name in braked
                ]
                if not blocked:
                    continue
                before = dict(held)
                held.update(self._slide(contact, group, blocked, tracking, bounds,
                                        float(current[group])))
                candidate = braked | set(blocked)
                moved, moved_emitted, over = trial(candidate)
                if over:
                    held = before
                else:
                    braked, velocities, emitted = candidate, moved, moved_emitted
                    state = "braked"
            # In joint order before sorting: ties must not follow set (hash)
            # order.  A recovering arm keeps its push-out: none of its joints
            # go back to plain tracking.
            pushing = {name for group in recovering for name in groups[group]}
            ordered = [name for name in self.joints
                       if name in braked and name not in pushing
                       and abs(tracking[name] - held[name]) > 1e-9]
            # Only the largest few: each try checks a whole braking path.
            for name in sorted(ordered, key=lambda n: -abs(tracking[n] - held[n]))[
                :CONTACT_MAX_RELEASES
            ]:
                candidate = braked - {name}
                moved, moved_emitted, over = trial(candidate)
                if not over:
                    braked, velocities, emitted = candidate, moved, moved_emitted
            braked = {name for name in braked if held[name] != tracking[name]}
            if state == "braked" and not braked:
                state = "clear"  # every tried change ended up tracking
        depth = max((float(emitted[group]) for group in groups), default=0.0)
        return velocities, state, depth, tuple(name for name in self.joints if name in braked)

    def _slide(
        self,
        contact: ContactCheck,
        group: str,
        joints: list[str],
        tracking: Mapping[str, float],
        bounds: Mapping[str, tuple[float, float, float, float]],
        depth: float,
    ) -> dict[str, float]:
        """Tracking velocities of ``joints`` with the motion into the body removed.

        The depth gradient comes from finite differences at the current pose.
        The depth may change at no more than the rate that would bring it back
        to half the tolerance over ``CONTACT_RECOVERY_S``; out of contact the
        gradient is zero and the joints simply track.
        """
        assert self._positions is not None
        base = float(contact.depths(self._positions)[group])
        gradient = []
        for name in joints:
            nudged = dict(self._positions)
            nudged[name] += CONTACT_GRADIENT_STEP_RAD
            gradient.append(
                (float(contact.depths(nudged)[group]) - base) / CONTACT_GRADIENT_STEP_RAD
            )
        velocity = [tracking[name] for name in joints]
        norm = sum(g * g for g in gradient)
        rate = -(depth - 0.5 * self.config.contact_tolerance_m) / CONTACT_RECOVERY_S
        excess = sum(g * v for g, v in zip(gradient, velocity)) - rate
        if norm > 0.0 and excess > 0.0:
            velocity = [v - excess / norm * g for g, v in zip(gradient, velocity)]
        return {
            name: min(max(value, bounds[name][0]), bounds[name][1])
            for name, value in zip(joints, velocity)
        }

    def _lookahead_depths(
        self,
        contact: ContactCheck,
        velocities: Mapping[str, float],
        bounds: Mapping[str, tuple[float, float, float, float]],
        step_dt: float,
    ) -> tuple[Mapping[str, float], dict[str, float]]:
        """Depths of the candidate pose, and the deepest along its braking path
        (every joint decelerating at the cap from its candidate velocity)."""
        acceleration = self.config.acceleration_limit_rad_s2
        pose = {name: self._advance(name, velocities[name], bounds, step_dt) for name in self.joints}
        emitted = contact.depths(pose)
        worst = {group: float(emitted[group]) for group in contact.groups}
        top_speed = max(abs(velocity) for velocity in velocities.values())
        stop_time = top_speed / acceleration
        samples = 0 if stop_time <= 0.0 else min(
            CONTACT_LOOKAHEAD_MAX_SAMPLES,
            max(CONTACT_LOOKAHEAD_MIN_SAMPLES,
                math.ceil(top_speed * stop_time / 2 / CONTACT_LOOKAHEAD_STEP_RAD)),
        )
        for sample in range(1, samples + 1):
            elapsed = stop_time * sample / samples
            braking_pose = {}
            for name, velocity in velocities.items():
                moving = min(elapsed, abs(velocity) / acceleration)
                travel = velocity * moving - math.copysign(acceleration, velocity) * moving**2 / 2
                _minimum, _maximum, lower, upper = bounds[name]
                braking_pose[name] = min(max(pose[name] + travel, lower), upper)
            for group, depth in contact.depths(braking_pose).items():
                worst[group] = max(worst[group], float(depth))
        return emitted, worst
