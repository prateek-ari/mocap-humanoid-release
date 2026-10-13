"""Causal live conditioning between retargeting and the G1 safety envelope.

Position in the live path::

    suit packet -> parse -> retarget -> [ this module ] -> ArmSafetyEnvelope -> command

Everything in this module is *reference shaping*: it smooths the retargeted
reference stream and resamples it onto a fixed command clock.  It is
deliberately not a limiter.  The position, velocity, acceleration, staleness
and hard-timeout clamps in :mod:`mocap_g1.safety` remain the last authority
over every emitted command, and nothing here may be used to bypass, relax or
replace them.  The upsampler forwards the *source* timestamp of the newest
reference unchanged, so the envelope's stale/hard-timeout machinery sees
exactly the same freshness signal it saw before this module existed; the
upsampler only reports staleness, it never decides on it.

Contrast with :mod:`mocap_g1.conditioning`, which is offline and zero-phase
(its FIR reads future samples and therefore cannot run in a controller).
Every filter here is strictly causal: the output at time ``t`` is a function of
the samples at and before ``t`` and of nothing else.

Causal smoothing always costs phase lag.  That cost is measurable rather than
assumed: :func:`measure_step_lag`, :func:`measure_phase_lag` and
:func:`measure_jitter_attenuation` characterise a configuration offline, and
:class:`LatencyRecorder` measures the wall-clock compute latency of the live
stages.  ``docs/live-latency-2026-08-16.md`` records the numbers for the
shipped defaults.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math
import time
from typing import Any, ClassVar, Sequence

import numpy as np


LIVE_FILTER_VERSION = 1

DEFAULT_SOURCE_RATE_HZ = 60.0
DEFAULT_COMMAND_RATE_HZ = 250.0

# Documented behaviour of the shipped defaults at a 60 Hz source rate, measured
# with the utilities at the bottom of this module and recorded in
# docs/live-latency-2026-08-16.md.  The tests assert against these bounds so a
# parameter change cannot silently move the live latency budget.
DEFAULT_STEP_RAD = 0.20
DEFAULT_REFERENCE_SINE_HZ = 1.0
DEFAULT_REFERENCE_SINE_RAD = 0.30
DOCUMENTED_STEP_LAG_50_BOUND_MS = 35.0
DOCUMENTED_STEP_LAG_90_BOUND_MS = 130.0
DOCUMENTED_SINE_LAG_BOUND_MS = 50.0
DOCUMENTED_JITTER_ATTENUATION_MIN_DB = 6.0


class LiveFilterError(ValueError):
    """Raised when live conditioning cannot produce a trustworthy value."""


def _positive_finite(value: Any, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise LiveFilterError(f"{name} must be a positive finite number") from exc
    if not math.isfinite(number) or number <= 0.0:
        raise LiveFilterError(f"{name} must be a positive finite number")
    return number


def _non_negative_finite(value: Any, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise LiveFilterError(f"{name} must be a non-negative finite number") from exc
    if not math.isfinite(number) or number < 0.0:
        raise LiveFilterError(f"{name} must be a non-negative finite number")
    return number


# --------------------------------------------------------------------------
# Filter configuration
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class OneEuroConfig:
    """Speed-adaptive first-order low-pass (Casiez et al., "1u filter").

    The cutoff rises with the observed joint speed, so slow poses are smoothed
    hard (jitter removed) while fast motion is smoothed lightly (lag kept
    small).  Defaults are chosen for a 60 Hz Xsens retarget stream in radians.
    """

    KIND: ClassVar[str] = "one_euro"

    min_cutoff_hz: float = 2.0
    beta: float = 0.6
    derivative_cutoff_hz: float = 1.0
    max_reference_gap_s: float = 0.25

    def validated(self) -> "OneEuroConfig":
        _positive_finite(self.min_cutoff_hz, "min_cutoff_hz")
        _non_negative_finite(self.beta, "beta")
        _positive_finite(self.derivative_cutoff_hz, "derivative_cutoff_hz")
        _positive_finite(self.max_reference_gap_s, "max_reference_gap_s")
        return self

    def describe(self) -> str:
        return (
            f"one_euro(min_cutoff={self.min_cutoff_hz:g}Hz, beta={self.beta:g}, "
            f"d_cutoff={self.derivative_cutoff_hz:g}Hz, "
            f"max_gap={self.max_reference_gap_s:g}s)"
        )


@dataclass(frozen=True)
class CriticallyDampedConfig:
    """Second-order critically damped low-pass, integrated exactly per step.

    No overshoot by construction, and an amplitude-independent group delay of
    ``2 / omega`` seconds at low frequency — the alternative to pick when a
    constant, predictable lag matters more than the One-Euro tradeoff.
    ``cutoff_hz`` is the -3 dB frequency; the natural frequency is
    ``2*pi*cutoff_hz / 0.6436``.  The default is chosen so this filter matches
    the One-Euro default at the reference sine (1 Hz, 0.30 rad).
    """

    KIND: ClassVar[str] = "critically_damped"

    cutoff_hz: float = 4.0
    max_reference_gap_s: float = 0.25

    def validated(self) -> "CriticallyDampedConfig":
        _positive_finite(self.cutoff_hz, "cutoff_hz")
        _positive_finite(self.max_reference_gap_s, "max_reference_gap_s")
        return self

    def describe(self) -> str:
        return (
            f"critically_damped(cutoff={self.cutoff_hz:g}Hz, "
            f"max_gap={self.max_reference_gap_s:g}s)"
        )


FilterConfig = OneEuroConfig | CriticallyDampedConfig

# -3 dB frequency of w^2 / (s + w)^2 expressed as a fraction of w.
_CRITICAL_CUTOFF_FRACTION = math.sqrt(math.sqrt(2.0) - 1.0)


@dataclass(frozen=True)
class FilterOutput:
    """One filtered reference plus the state needed to audit it."""

    values: np.ndarray
    dt_s: float
    reinitialized: bool
    max_cutoff_hz: float


class LiveJointFilter:
    """Strictly causal per-joint smoother over a fixed-width DOF vector.

    Subclasses implement :meth:`_advance`.  The base class owns validation,
    the monotonic-clock contract, and the reference-gap reinitialisation rule:
    after a gap longer than ``max_reference_gap_s`` the stored history is no
    longer a trustworthy predictor, so the filter restarts from the incoming
    sample rather than ramping out of stale state.  A gap that long is already
    a stale/hard-timeout event for the safety envelope, which keeps authority
    over what the robot is allowed to do about it.
    """

    def __init__(self, joint_count: int, *, max_reference_gap_s: float) -> None:
        if isinstance(joint_count, bool) or not isinstance(joint_count, int):
            raise LiveFilterError("joint_count must be a positive integer")
        if joint_count < 1:
            raise LiveFilterError("joint_count must be a positive integer")
        self.joint_count = joint_count
        self.max_reference_gap_s = _positive_finite(
            max_reference_gap_s, "max_reference_gap_s"
        )
        self._last_time: float | None = None
        self.reinitializations = 0
        self.samples = 0

    @property
    def initialized(self) -> bool:
        return self._last_time is not None

    def reset(self) -> None:
        """Drop all history so the next sample is passed through unchanged."""
        self._last_time = None
        self._reset_state()

    def step(self, values: Any, *, now: float) -> FilterOutput:
        """Filter one reference sample taken at monotonic time ``now``."""
        sample = np.asarray(values, dtype=np.float64)
        if sample.shape != (self.joint_count,):
            raise LiveFilterError(
                f"values must have shape ({self.joint_count},), got {sample.shape}"
            )
        if not np.isfinite(sample).all():
            raise LiveFilterError("values contain a non-finite entry")
        moment = float(now)
        if not math.isfinite(moment):
            raise LiveFilterError("now must be finite")

        if self._last_time is None:
            dt = 0.0
            reinitialized = True
        else:
            dt = moment - self._last_time
            if not math.isfinite(dt) or dt <= 0.0:
                raise LiveFilterError(
                    f"filter time must increase monotonically; dt={dt}"
                )
            reinitialized = dt > self.max_reference_gap_s

        if reinitialized:
            self._initialize_state(sample)
            self.reinitializations += 1
            output = sample.copy()
            max_cutoff = float("inf")
        else:
            output, max_cutoff = self._advance(sample, dt)

        self._last_time = moment
        self.samples += 1
        return FilterOutput(
            values=output,
            dt_s=dt,
            reinitialized=reinitialized,
            max_cutoff_hz=max_cutoff,
        )

    # Subclass hooks -------------------------------------------------------

    def _reset_state(self) -> None:
        raise NotImplementedError

    def _initialize_state(self, sample: np.ndarray) -> None:
        raise NotImplementedError

    def _advance(self, sample: np.ndarray, dt: float) -> tuple[np.ndarray, float]:
        raise NotImplementedError

    def describe(self) -> str:
        raise NotImplementedError


def _low_pass_alpha(cutoff_hz: np.ndarray | float, dt: float) -> np.ndarray | float:
    tau = 1.0 / (2.0 * math.pi * np.asarray(cutoff_hz, dtype=np.float64))
    return 1.0 / (1.0 + tau / dt)


class OneEuroFilter(LiveJointFilter):
    """Vectorised One-Euro filter over ``joint_count`` independent joints."""

    def __init__(self, config: OneEuroConfig, joint_count: int) -> None:
        self.config = config.validated()
        super().__init__(joint_count, max_reference_gap_s=config.max_reference_gap_s)
        self._value: np.ndarray | None = None
        self._derivative: np.ndarray | None = None

    def _reset_state(self) -> None:
        self._value = None
        self._derivative = None

    def _initialize_state(self, sample: np.ndarray) -> None:
        self._value = sample.copy()
        self._derivative = np.zeros(self.joint_count, dtype=np.float64)

    def _advance(self, sample: np.ndarray, dt: float) -> tuple[np.ndarray, float]:
        assert self._value is not None and self._derivative is not None
        raw_derivative = (sample - self._value) / dt
        derivative_alpha = _low_pass_alpha(self.config.derivative_cutoff_hz, dt)
        derivative = (
            derivative_alpha * raw_derivative
            + (1.0 - derivative_alpha) * self._derivative
        )
        cutoff = self.config.min_cutoff_hz + self.config.beta * np.abs(derivative)
        alpha = _low_pass_alpha(cutoff, dt)
        value = alpha * sample + (1.0 - alpha) * self._value
        self._value = value
        self._derivative = derivative
        return value.copy(), float(np.max(cutoff))

    def describe(self) -> str:
        return self.config.describe()


class CriticallyDampedFilter(LiveJointFilter):
    """Vectorised critically damped second-order low-pass.

    The per-step update is the closed-form solution of ``x'' = -2*w*x' -
    w^2*(x - target)`` over ``dt`` with the target held constant, so it is
    unconditionally stable at any step size and never overshoots.
    """

    def __init__(self, config: CriticallyDampedConfig, joint_count: int) -> None:
        self.config = config.validated()
        super().__init__(joint_count, max_reference_gap_s=config.max_reference_gap_s)
        self.omega = 2.0 * math.pi * config.cutoff_hz / _CRITICAL_CUTOFF_FRACTION
        self._value: np.ndarray | None = None
        self._velocity: np.ndarray | None = None

    def _reset_state(self) -> None:
        self._value = None
        self._velocity = None

    def _initialize_state(self, sample: np.ndarray) -> None:
        self._value = sample.copy()
        self._velocity = np.zeros(self.joint_count, dtype=np.float64)

    def _advance(self, sample: np.ndarray, dt: float) -> tuple[np.ndarray, float]:
        assert self._value is not None and self._velocity is not None
        omega = self.omega
        error = self._value - sample
        decay = math.exp(-omega * dt)
        coefficient = self._velocity + omega * error
        value = sample + (error + coefficient * dt) * decay
        velocity = (self._velocity - omega * coefficient * dt) * decay
        self._value = value
        self._velocity = velocity
        return value.copy(), self.config.cutoff_hz

    def describe(self) -> str:
        return self.config.describe()


def make_filter(config: FilterConfig, joint_count: int) -> LiveJointFilter:
    """Build the filter implementation named by ``config``."""
    if isinstance(config, OneEuroConfig):
        return OneEuroFilter(config, joint_count)
    if isinstance(config, CriticallyDampedConfig):
        return CriticallyDampedFilter(config, joint_count)
    raise LiveFilterError(f"unsupported live filter config: {type(config).__name__}")


FILTER_KINDS = (OneEuroConfig.KIND, CriticallyDampedConfig.KIND)


def default_config(kind: str) -> FilterConfig:
    """Return the shipped default configuration for a filter kind."""
    if kind == OneEuroConfig.KIND:
        return OneEuroConfig()
    if kind == CriticallyDampedConfig.KIND:
        return CriticallyDampedConfig()
    raise LiveFilterError(
        f"unknown filter kind {kind!r}; expected one of {', '.join(FILTER_KINDS)}"
    )


# --------------------------------------------------------------------------
# Fixed-rate command upsampler
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class UpsamplerConfig:
    """Fixed-rate resampling of a ~60 Hz reference stream onto a command clock.

    ``mode="interpolate"`` ramps the emitted command from its current value to
    the newest reference over one source interval.  The output is continuous
    and never extrapolates past a received reference, at the documented cost of
    running up to one source interval (~16.7 ms at 60 Hz) behind the newest
    reference.  ``mode="hold"`` is a zero-order hold: no added lag, stepped
    output.
    """

    command_rate_hz: float = DEFAULT_COMMAND_RATE_HZ
    mode: str = "interpolate"
    source_rate_hz: float = DEFAULT_SOURCE_RATE_HZ
    max_segment_s: float = 0.05
    max_commands_per_pump: int = 64
    stall_after_s: float = 0.10

    def validated(self) -> "UpsamplerConfig":
        _positive_finite(self.command_rate_hz, "command_rate_hz")
        _positive_finite(self.source_rate_hz, "source_rate_hz")
        _positive_finite(self.max_segment_s, "max_segment_s")
        _positive_finite(self.stall_after_s, "stall_after_s")
        if self.mode not in ("interpolate", "hold"):
            raise LiveFilterError("mode must be 'interpolate' or 'hold'")
        if (
            isinstance(self.max_commands_per_pump, bool)
            or not isinstance(self.max_commands_per_pump, int)
            or self.max_commands_per_pump < 1
        ):
            raise LiveFilterError("max_commands_per_pump must be an integer >= 1")
        if self.command_rate_hz < self.source_rate_hz:
            raise LiveFilterError(
                "command_rate_hz must be at least source_rate_hz; this stage "
                "upsamples and must never decimate the reference stream"
            )
        return self

    def describe(self) -> str:
        return (
            f"upsampler({self.command_rate_hz:g}Hz, mode={self.mode}, "
            f"source={self.source_rate_hz:g}Hz, "
            f"max_segment={self.max_segment_s * 1e3:g}ms)"
        )


@dataclass(frozen=True)
class CommandSample:
    """One scheduled command.

    ``source_timestamp`` is the arrival time of the newest reference that
    contributed to this command, forwarded unchanged so the safety envelope's
    stale/hard-timeout decisions are exactly what they were before the
    upsampler existed.  ``saturated`` and ``stalled`` are reports, not
    decisions.
    """

    command_time: float
    emit_time: float
    values: np.ndarray
    source_timestamp: float
    source_age_s: float
    interpolation_lag_s: float
    saturated: bool
    stalled: bool


@dataclass(frozen=True)
class UpsamplerStats:
    commands: int
    saturated_commands: int
    stalled_commands: int
    dropped_ticks: int
    references: int
    span_s: float
    achieved_rate_hz: float
    #: p95 over the most recent ``LATE_MS_WINDOW`` commands; max is all-time.
    late_ms_p95: float
    late_ms_max: float


#: Emit-lateness samples retained for the p95 statistic.  At 250 Hz this is
#: the most recent ~4.4 minutes; ``commands`` and ``late_ms_max`` still cover
#: the whole run.  Bounded so a long live session cannot grow without limit.
LATE_MS_WINDOW = 65_536


class CommandUpsampler:
    """Monotonic-clock scheduler turning references into fixed-rate commands.

    The upsampler sits *before* the safety envelope.  It changes only the
    timing and continuity of the reference stream; the envelope still clamps
    position, velocity, acceleration and staleness on every command it is
    given, and remains the last authority.  When the source stalls, the
    upsampler holds the last reference (it never extrapolates) and raises
    ``stalled`` on the emitted samples, while the envelope independently
    decelerates and eventually hard-times-out on the same forwarded source
    timestamp.
    """

    def __init__(self, config: UpsamplerConfig, joint_count: int) -> None:
        self.config = config.validated()
        if isinstance(joint_count, bool) or not isinstance(joint_count, int):
            raise LiveFilterError("joint_count must be a positive integer")
        if joint_count < 1:
            raise LiveFilterError("joint_count must be a positive integer")
        self.joint_count = joint_count
        self.period_s = 1.0 / self.config.command_rate_hz
        self._start: np.ndarray | None = None
        self._end: np.ndarray | None = None
        self._segment_start_time: float | None = None
        self._segment_end_time: float | None = None
        self._last_push_time: float | None = None
        self._source_timestamp: float | None = None
        self._epoch: float | None = None
        self._emitted = 0
        self._saturated = 0
        self._stalled = 0
        self._dropped_ticks = 0
        self._references = 0
        self._late_ms: deque[float] = deque(maxlen=LATE_MS_WINDOW)
        self._commands = 0
        self._late_ms_max = float("nan")
        self._last_command_time: float | None = None
        self._closed_span_s = 0.0

    @property
    def has_reference(self) -> bool:
        return self._end is not None

    def reset(self) -> None:
        """Forget the schedule and the reference history (rearm/stall path).

        Cumulative diagnostics survive a reset; only the command clock and the
        reference state are dropped, so a rearm restarts the schedule from the
        next reference instead of trying to catch up on the gap.
        """
        self._closed_span_s += self._active_span_s()
        self._start = None
        self._end = None
        self._segment_start_time = None
        self._segment_end_time = None
        self._last_push_time = None
        self._source_timestamp = None
        self._epoch = None
        self._emitted = 0
        self._last_command_time = None

    def push(self, values: Any, *, now: float, source_timestamp: float) -> None:
        """Accept a filtered reference produced from a packet at ``source_timestamp``."""
        sample = np.asarray(values, dtype=np.float64)
        if sample.shape != (self.joint_count,):
            raise LiveFilterError(
                f"values must have shape ({self.joint_count},), got {sample.shape}"
            )
        if not np.isfinite(sample).all():
            raise LiveFilterError("values contain a non-finite entry")
        moment = float(now)
        source = float(source_timestamp)
        if not math.isfinite(moment) or not math.isfinite(source):
            raise LiveFilterError("push timestamps must be finite")
        if self._last_push_time is not None and moment < self._last_push_time:
            raise LiveFilterError("push time must not move backwards")

        if self._end is None or self.config.mode == "hold":
            # First reference, or zero-order hold: the command is the reference.
            duration = 0.0
            start = sample.copy()
        else:
            # Ramp from where the command actually is now to the new reference
            # over the observed source interval, bounded so that a dropped
            # frame cannot stretch one segment into a slow crawl.
            assert self._last_push_time is not None
            gap = moment - self._last_push_time
            if gap <= 0.0:
                gap = 1.0 / self.config.source_rate_hz
            duration = min(gap, self.config.max_segment_s)
            start = self._value_at(moment)

        self._start = start
        self._end = sample.copy()
        self._segment_start_time = moment
        self._segment_end_time = moment + duration
        self._last_push_time = moment
        self._source_timestamp = source
        self._references += 1
        if self._epoch is None:
            self._epoch = moment

    def pump(self, now: float) -> list[CommandSample]:
        """Emit every command scheduled at or before ``now``.

        Catch-up is bounded: after a long stall the scheduler skips ahead
        instead of emitting a burst of thousands of backdated commands, and
        counts the skipped ticks.
        """
        moment = float(now)
        if not math.isfinite(moment):
            raise LiveFilterError("pump time must be finite")
        if self._epoch is None or self._end is None:
            return []

        # A tick that lands within a nanosecond-scale rounding error of ``now``
        # is due: at 250 Hz the schedule and the clock are both accumulations
        # of binary fractions, so an exact comparison drops ticks at segment
        # boundaries.
        next_time = self._epoch + self._emitted * self.period_s
        offset = (moment - next_time) / self.period_s + 1e-9
        if offset < 0.0:
            return []
        due = int(math.floor(offset)) + 1
        if due > self.config.max_commands_per_pump:
            skipped = due - self.config.max_commands_per_pump
            self._emitted += skipped
            self._dropped_ticks += skipped
            due = self.config.max_commands_per_pump

        commands: list[CommandSample] = []
        for _ in range(due):
            command_time = self._epoch + self._emitted * self.period_s
            commands.append(self._sample_at(command_time, emit_time=moment))
            self._emitted += 1
            self._last_command_time = command_time
        return commands

    # Internals ------------------------------------------------------------

    def _fraction_at(self, when: float) -> float:
        assert self._segment_start_time is not None and self._segment_end_time is not None
        duration = self._segment_end_time - self._segment_start_time
        if duration <= 0.0:
            return 1.0
        return min(max((when - self._segment_start_time) / duration, 0.0), 1.0)

    def _value_at(self, when: float) -> np.ndarray:
        assert self._start is not None and self._end is not None
        fraction = self._fraction_at(when)
        return self._start + (self._end - self._start) * fraction

    def _sample_at(self, command_time: float, *, emit_time: float) -> CommandSample:
        assert self._segment_end_time is not None and self._source_timestamp is not None
        assert self._segment_start_time is not None
        fraction = self._fraction_at(command_time)
        values = self._value_at(command_time)
        duration = self._segment_end_time - self._segment_start_time
        saturated = fraction >= 1.0
        source_age = max(0.0, command_time - self._source_timestamp)
        stalled = source_age >= self.config.stall_after_s
        if saturated:
            self._saturated += 1
        if stalled:
            self._stalled += 1
        late_ms = (emit_time - command_time) * 1e3
        self._late_ms.append(late_ms)
        self._commands += 1
        if not late_ms <= self._late_ms_max:  # also replaces the initial NaN
            self._late_ms_max = late_ms
        return CommandSample(
            command_time=command_time,
            emit_time=emit_time,
            values=values,
            source_timestamp=self._source_timestamp,
            source_age_s=source_age,
            interpolation_lag_s=(1.0 - fraction) * max(duration, 0.0),
            saturated=saturated,
            stalled=stalled,
        )

    def _active_span_s(self) -> float:
        """Scheduled time covered by the current epoch, one period per tick."""
        if self._epoch is None or self._last_command_time is None:
            return 0.0
        return max(0.0, self._last_command_time - self._epoch) + self.period_s

    def stats(self) -> UpsamplerStats:
        late = np.asarray(self._late_ms, dtype=np.float64)
        span = self._closed_span_s + self._active_span_s()
        achieved = self._commands / span if span > 0.0 else 0.0
        return UpsamplerStats(
            commands=self._commands,
            saturated_commands=self._saturated,
            stalled_commands=self._stalled,
            dropped_ticks=self._dropped_ticks,
            references=self._references,
            span_s=span,
            achieved_rate_hz=achieved,
            late_ms_p95=float(np.percentile(late, 95)) if len(late) else float("nan"),
            late_ms_max=self._late_ms_max,
        )

    def format_stats(self) -> str:
        stats = self.stats()
        return (
            f"upsampler: commands={stats.commands}, references={stats.references}, "
            f"achieved={stats.achieved_rate_hz:.1f} Hz "
            f"(configured {self.config.command_rate_hz:g} Hz), "
            f"at_newest_reference={stats.saturated_commands}, "
            f"stall_flagged={stats.stalled_commands}, "
            f"dropped_ticks={stats.dropped_ticks}, "
            f"emit_late_p95={stats.late_ms_p95:.2f} ms, "
            f"emit_late_max={stats.late_ms_max:.2f} ms"
        )


# --------------------------------------------------------------------------
# Latency instrumentation
# --------------------------------------------------------------------------


STAGE_ORDER = ("parse", "retarget", "filter", "emit")


@dataclass
class FrameTimeline:
    """Monotonic stage timestamps for one suit packet."""

    packet_monotonic: float
    stages: dict[str, float] = field(default_factory=dict)

    def mark(self, stage: str, when: float | None = None) -> float:
        if stage not in STAGE_ORDER:
            raise LiveFilterError(
                f"unknown latency stage {stage!r}; expected one of {', '.join(STAGE_ORDER)}"
            )
        moment = time.monotonic() if when is None else float(when)
        if not math.isfinite(moment):
            raise LiveFilterError("stage timestamp must be finite")
        self.stages[stage] = moment
        return moment

    @property
    def complete(self) -> bool:
        return all(stage in self.stages for stage in STAGE_ORDER)


@dataclass(frozen=True)
class StageStats:
    count: int
    mean_ms: float
    p50_ms: float
    p95_ms: float
    max_ms: float


@dataclass(frozen=True)
class LatencySummary:
    frames: int
    stages: dict[str, StageStats]
    end_to_end: StageStats | None
    filter_lag_ms: float
    upsampler_lag_ms: float

    @property
    def algorithmic_lag_ms(self) -> float:
        return self.filter_lag_ms + self.upsampler_lag_ms

    @property
    def budget_p95_ms(self) -> float:
        if self.end_to_end is None:
            return float("nan")
        return self.end_to_end.p95_ms + self.algorithmic_lag_ms


def _stage_stats(values: Sequence[float]) -> StageStats:
    array = np.asarray(values, dtype=np.float64)
    if not len(array):
        nan = float("nan")
        return StageStats(0, nan, nan, nan, nan)
    return StageStats(
        count=int(len(array)),
        mean_ms=float(np.mean(array)),
        p50_ms=float(np.percentile(array, 50)),
        p95_ms=float(np.percentile(array, 95)),
        max_ms=float(np.max(array)),
    )


class LatencyRecorder:
    """Per-frame suit-packet -> command timing, with a summary reporter.

    Two different quantities make up the live latency budget and are reported
    separately because they cannot be traded against each other:

    * *compute latency* — measured wall-clock time from packet arrival to
      command emission, recorded here per frame;
    * *algorithmic lag* — the filter's phase lag plus the upsampler's
      interpolation lag.  These are properties of the configuration, measured
      offline by :func:`measure_step_lag` and reported through
      :meth:`note_algorithmic_lag`, not by timing the code.
    """

    def __init__(self, *, capacity: int = 50_000) -> None:
        if isinstance(capacity, bool) or not isinstance(capacity, int) or capacity < 1:
            raise LiveFilterError("capacity must be a positive integer")
        self.capacity = capacity
        self._stage_ms: dict[str, list[float]] = {stage: [] for stage in STAGE_ORDER}
        self._end_to_end_ms: list[float] = []
        self._filter_lag_ms = 0.0
        self._upsampler_lag_ms = 0.0
        self.dropped_records = 0

    def open(self, packet_monotonic: float) -> FrameTimeline:
        moment = float(packet_monotonic)
        if not math.isfinite(moment):
            raise LiveFilterError("packet timestamp must be finite")
        return FrameTimeline(packet_monotonic=moment)

    def note_algorithmic_lag(
        self, *, filter_lag_ms: float = 0.0, upsampler_lag_ms: float = 0.0
    ) -> None:
        self._filter_lag_ms = _non_negative_finite(filter_lag_ms, "filter_lag_ms")
        self._upsampler_lag_ms = _non_negative_finite(
            upsampler_lag_ms, "upsampler_lag_ms"
        )

    def commit(self, timeline: FrameTimeline) -> None:
        if not timeline.complete:
            missing = [stage for stage in STAGE_ORDER if stage not in timeline.stages]
            raise LiveFilterError(f"timeline is missing stages: {', '.join(missing)}")
        previous = timeline.packet_monotonic
        deltas: dict[str, float] = {}
        for stage in STAGE_ORDER:
            moment = timeline.stages[stage]
            if moment < previous:
                raise LiveFilterError(
                    f"latency stage {stage!r} is earlier than the previous stage"
                )
            deltas[stage] = (moment - previous) * 1e3
            previous = moment
        if len(self._end_to_end_ms) >= self.capacity:
            self.dropped_records += 1
            return
        for stage, value in deltas.items():
            self._stage_ms[stage].append(value)
        self._end_to_end_ms.append(
            (timeline.stages["emit"] - timeline.packet_monotonic) * 1e3
        )

    @property
    def frames(self) -> int:
        return len(self._end_to_end_ms)

    def summary(self) -> LatencySummary:
        return LatencySummary(
            frames=self.frames,
            stages={stage: _stage_stats(self._stage_ms[stage]) for stage in STAGE_ORDER},
            end_to_end=_stage_stats(self._end_to_end_ms) if self._end_to_end_ms else None,
            filter_lag_ms=self._filter_lag_ms,
            upsampler_lag_ms=self._upsampler_lag_ms,
        )

    def format_report(self) -> str:
        summary = self.summary()
        lines = ["=== live latency (suit packet -> command) ==="]
        if summary.frames == 0:
            lines.append("  no frames recorded")
            return "\n".join(lines)
        lines.append(f"  frames                 {summary.frames}")
        if self.dropped_records:
            lines.append(f"  records dropped        {self.dropped_records} (capacity)")
        header = f"  {'stage':<22s} {'mean':>8s} {'p50':>8s} {'p95':>8s} {'max':>8s}   (ms)"
        lines.append(header)
        for stage in STAGE_ORDER:
            stats = summary.stages[stage]
            lines.append(
                f"  {stage:<22s} {stats.mean_ms:8.2f} {stats.p50_ms:8.2f} "
                f"{stats.p95_ms:8.2f} {stats.max_ms:8.2f}"
            )
        assert summary.end_to_end is not None
        end = summary.end_to_end
        lines.append(
            f"  {'compute end-to-end':<22s} {end.mean_ms:8.2f} {end.p50_ms:8.2f} "
            f"{end.p95_ms:8.2f} {end.max_ms:8.2f}"
        )
        lines.append(
            f"  algorithmic lag        filter {summary.filter_lag_ms:.1f} ms + "
            f"upsampler {summary.upsampler_lag_ms:.1f} ms "
            f"= {summary.algorithmic_lag_ms:.1f} ms (configuration, measured offline)"
        )
        lines.append(
            f"  budget (compute p95 + algorithmic)  {summary.budget_p95_ms:.1f} ms"
        )
        lines.append(
            "  excludes suit->host transport and any hardware SDK publish path"
        )
        return "\n".join(lines)


# --------------------------------------------------------------------------
# Offline measurement utilities
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class StepResponse:
    """Effective lag of a configuration, measured on a commanded step."""

    step_rad: float
    sample_rate_hz: float
    lag_50_ms: float
    lag_63_ms: float
    lag_90_ms: float
    overshoot_fraction: float

    def describe(self) -> str:
        return (
            f"step {self.step_rad:g} rad @ {self.sample_rate_hz:g} Hz: "
            f"lag50={self.lag_50_ms:.1f} ms, lag63={self.lag_63_ms:.1f} ms, "
            f"lag90={self.lag_90_ms:.1f} ms, overshoot={self.overshoot_fraction * 100:.1f}%"
        )


@dataclass(frozen=True)
class SineResponse:
    """Gain and phase lag of a configuration at one excitation frequency."""

    signal_hz: float
    amplitude_rad: float
    sample_rate_hz: float
    gain: float
    phase_lag_ms: float
    phase_lag_deg: float

    def describe(self) -> str:
        return (
            f"sine {self.signal_hz:g} Hz / {self.amplitude_rad:g} rad: "
            f"gain={self.gain:.3f}, lag={self.phase_lag_ms:.1f} ms "
            f"({self.phase_lag_deg:.1f} deg)"
        )


@dataclass(frozen=True)
class JitterResponse:
    """Jitter rejection of a configuration versus the unfiltered stream."""

    signal_hz: float
    noise_std_rad: float
    sample_rate_hz: float
    raw_jitter_rms_rad: float
    command_jitter_rms_rad: float
    attenuation_db: float

    def describe(self) -> str:
        return (
            f"jitter {self.noise_std_rad:g} rad std on a {self.signal_hz:g} Hz sine: "
            f"raw_rms={self.raw_jitter_rms_rad:.4f}, "
            f"command_rms={self.command_jitter_rms_rad:.4f}, "
            f"attenuation={self.attenuation_db:.1f} dB"
        )


def _run_filter(
    config: FilterConfig, samples: np.ndarray, sample_rate_hz: float
) -> np.ndarray:
    rate = _positive_finite(sample_rate_hz, "sample_rate_hz")
    filt = make_filter(config, 1)
    dt = 1.0 / rate
    output = np.empty(len(samples), dtype=np.float64)
    for index, value in enumerate(samples):
        result = filt.step(np.asarray([value], dtype=np.float64), now=index * dt)
        output[index] = result.values[0]
    return output


def _crossing_time(times: np.ndarray, values: np.ndarray, level: float) -> float:
    for index, value in enumerate(values):
        if value >= level:
            if index == 0:
                return float(times[0])
            previous = values[index - 1]
            if value == previous:
                return float(times[index])
            fraction = (level - previous) / (value - previous)
            return float(times[index - 1] + fraction * (times[index] - times[index - 1]))
    return float("nan")


def measure_step_lag(
    config: FilterConfig,
    *,
    sample_rate_hz: float = DEFAULT_SOURCE_RATE_HZ,
    step_rad: float = DEFAULT_STEP_RAD,
    settle_s: float = 2.0,
    warmup_s: float = 0.5,
) -> StepResponse:
    """Measure how long a configuration takes to follow a commanded step.

    One-Euro is speed-adaptive and therefore nonlinear: its lag depends on the
    step size, so ``step_rad`` is part of the reported result.
    """
    rate = _positive_finite(sample_rate_hz, "sample_rate_hz")
    amplitude = _positive_finite(step_rad, "step_rad")
    warmup = int(round(_positive_finite(warmup_s, "warmup_s") * rate))
    settle = int(round(_positive_finite(settle_s, "settle_s") * rate))
    samples = np.concatenate([np.zeros(warmup), np.full(settle, amplitude)])
    output = _run_filter(config, samples, rate)[warmup:]
    times = np.arange(len(output), dtype=np.float64) / rate
    return StepResponse(
        step_rad=amplitude,
        sample_rate_hz=rate,
        lag_50_ms=_crossing_time(times, output, 0.50 * amplitude) * 1e3,
        lag_63_ms=_crossing_time(times, output, 0.632 * amplitude) * 1e3,
        lag_90_ms=_crossing_time(times, output, 0.90 * amplitude) * 1e3,
        overshoot_fraction=float(max(0.0, np.max(output) - amplitude) / amplitude),
    )


def _dft_component(signal: np.ndarray, sample_rate_hz: float, freq_hz: float) -> complex:
    count = len(signal)
    times = np.arange(count, dtype=np.float64) / sample_rate_hz
    reference = np.exp(-2j * math.pi * freq_hz * times)
    return complex(2.0 * np.sum(signal * reference) / count)


def measure_phase_lag(
    config: FilterConfig,
    *,
    sample_rate_hz: float = DEFAULT_SOURCE_RATE_HZ,
    signal_hz: float = DEFAULT_REFERENCE_SINE_HZ,
    amplitude_rad: float = DEFAULT_REFERENCE_SINE_RAD,
    periods: int = 20,
    warmup_periods: int = 10,
) -> SineResponse:
    """Measure steady-state gain and phase lag on a sinusoidal joint sweep."""
    rate = _positive_finite(sample_rate_hz, "sample_rate_hz")
    frequency = _positive_finite(signal_hz, "signal_hz")
    amplitude = _positive_finite(amplitude_rad, "amplitude_rad")
    if frequency >= 0.5 * rate:
        raise LiveFilterError("signal_hz must be below the Nyquist frequency")
    per_period = rate / frequency
    warmup = int(round(warmup_periods * per_period))
    analysed = int(round(periods * per_period))
    times = np.arange(warmup + analysed, dtype=np.float64) / rate
    samples = amplitude * np.sin(2.0 * math.pi * frequency * times)
    output = _run_filter(config, samples, rate)

    reference = _dft_component(samples[warmup:], rate, frequency)
    filtered = _dft_component(output[warmup:], rate, frequency)
    gain = abs(filtered) / abs(reference)
    phase = math.atan2(filtered.imag, filtered.real) - math.atan2(
        reference.imag, reference.real
    )
    while phase > 0.0:
        phase -= 2.0 * math.pi
    while phase <= -2.0 * math.pi:
        phase += 2.0 * math.pi
    lag_s = -phase / (2.0 * math.pi * frequency)
    return SineResponse(
        signal_hz=frequency,
        amplitude_rad=amplitude,
        sample_rate_hz=rate,
        gain=float(gain),
        phase_lag_ms=float(lag_s * 1e3),
        phase_lag_deg=float(math.degrees(-phase)),
    )


def measure_jitter_attenuation(
    config: FilterConfig,
    *,
    sample_rate_hz: float = DEFAULT_SOURCE_RATE_HZ,
    signal_hz: float = 0.5,
    amplitude_rad: float = DEFAULT_REFERENCE_SINE_RAD,
    noise_std_rad: float = 0.01,
    duration_s: float = 20.0,
    seed: int = 20260816,
) -> JitterResponse:
    """Measure how much suit jitter survives into the command stream.

    The same filter is run twice, once on a clean sine and once on the same
    sine plus white noise.  The difference between the two outputs is the
    jitter that reaches the command; comparing it with the injected noise
    isolates jitter rejection from the phase lag reported separately by
    :func:`measure_phase_lag`.  (The two runs are exactly separable for the
    linear filter and are the honest analogue for the nonlinear One-Euro.)
    """
    rate = _positive_finite(sample_rate_hz, "sample_rate_hz")
    noise_std = _positive_finite(noise_std_rad, "noise_std_rad")
    amplitude = _positive_finite(amplitude_rad, "amplitude_rad")
    frequency = _positive_finite(signal_hz, "signal_hz")
    count = int(round(_positive_finite(duration_s, "duration_s") * rate))
    warmup = int(round(rate))
    if count <= warmup + 1:
        raise LiveFilterError("duration_s must cover more than one second of warmup")
    times = np.arange(count, dtype=np.float64) / rate
    clean = amplitude * np.sin(2.0 * math.pi * frequency * times)
    noisy = clean + np.random.default_rng(seed).normal(0.0, noise_std, count)

    clean_output = _run_filter(config, clean, rate)
    noisy_output = _run_filter(config, noisy, rate)

    raw_jitter = noisy[warmup:] - clean[warmup:]
    command_jitter = noisy_output[warmup:] - clean_output[warmup:]
    raw_rms = float(np.sqrt(np.mean(np.square(raw_jitter))))
    command_rms = float(np.sqrt(np.mean(np.square(command_jitter))))
    attenuation = (
        20.0 * math.log10(raw_rms / command_rms) if command_rms > 0.0 else float("inf")
    )
    return JitterResponse(
        signal_hz=frequency,
        noise_std_rad=noise_std,
        sample_rate_hz=rate,
        raw_jitter_rms_rad=raw_rms,
        command_jitter_rms_rad=command_rms,
        attenuation_db=float(attenuation),
    )


def upsampler_lag_ms(config: UpsamplerConfig) -> float:
    """Worst-case lag the upsampler adds by never extrapolating a reference."""
    validated = config.validated()
    if validated.mode == "hold":
        return 0.0
    return 1e3 * min(1.0 / validated.source_rate_hz, validated.max_segment_s)


def format_configuration_report(
    filter_config: FilterConfig | None,
    upsampler_config: UpsamplerConfig | None,
    *,
    sample_rate_hz: float = DEFAULT_SOURCE_RATE_HZ,
    step_rad: float = DEFAULT_STEP_RAD,
) -> str:
    """Human-readable lag/jitter characterisation of a live conditioning setup."""
    lines = ["=== live conditioning configuration ==="]
    if filter_config is None:
        lines.append("  filter                 disabled (raw retarget references)")
    else:
        step = measure_step_lag(
            filter_config, sample_rate_hz=sample_rate_hz, step_rad=step_rad
        )
        sine = measure_phase_lag(filter_config, sample_rate_hz=sample_rate_hz)
        jitter = measure_jitter_attenuation(
            filter_config, sample_rate_hz=sample_rate_hz
        )
        lines.append(f"  filter                 {filter_config.describe()}")
        lines.append(f"  step response          {step.describe()}")
        lines.append(f"  sine response          {sine.describe()}")
        lines.append(f"  jitter rejection       {jitter.describe()}")
    if upsampler_config is None:
        lines.append("  upsampler              disabled (reference rate commands)")
    else:
        lines.append(f"  upsampler              {upsampler_config.describe()}")
        lines.append(
            f"  upsampler added lag    {upsampler_lag_ms(upsampler_config):.1f} ms "
            "(worst case; never extrapolates)"
        )
    lines.append(
        "  ordering               filter -> upsampler -> safety envelope "
        "(clamps remain the last authority)"
    )
    return "\n".join(lines)


if __name__ == "__main__":  # pragma: no cover - reproduces the documented numbers
    for kind in FILTER_KINDS:
        print(
            format_configuration_report(
                default_config(kind), UpsamplerConfig()
            )
        )
        print()
