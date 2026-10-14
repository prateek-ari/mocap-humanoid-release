#!/usr/bin/env python3
"""Retarget live Xsens MXTP02 into a MuJoCo Unitree G1 or Booster T1 preview.

This is a kinematic simulator integration, not a robot hardware SDK bridge.
G1 waist/arm commands pass the G1 safety envelope; the simulation-only T1 is
screened by its MJCF joint ranges (see ``LiveRangePreviewController``).
"""

from __future__ import annotations

import argparse
from collections.abc import Callable
from dataclasses import dataclass, field, replace
import math
from pathlib import Path
import signal
import sys
import time
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from mocap_g1.assets import official_urdf_path
from mocap_g1.live_filter import (
    DEFAULT_COMMAND_RATE_HZ,
    FILTER_KINDS,
    CommandUpsampler,
    CriticallyDampedConfig,
    FrameTimeline,
    LatencyRecorder,
    LiveJointFilter,
    LiveFilterError,
    OneEuroConfig,
    UpsamplerConfig,
    format_configuration_report,
    make_filter,
    measure_step_lag,
    upsampler_lag_ms,
)
from mocap_g1.live_arm_alignment import ArmAlignmentError, LiveArmAlignment, floor_offset_m
from mocap_g1.live_arm_clearance import ArmAbduction, abduct_frame_rotations
from mocap_g1.live_ik import configure_live_ik_limits, install_solve_ik_fix
from mocap_g1.live_robots import (
    ARM_ALIGNMENTS,
    LIVE_ROBOTS,
    MVN_ARM_BODIES,
    T1_MOCAP_JOINT_RANGES,
    LiveRobot,
)
from mocap_g1.live_safety import (
    LiveG1SafetyController,
    LiveOperatorControls,
    LiveRangePreviewController,
    LiveSafetyOutput,
)
from mocap_g1.mxtp02 import ProtocolError, counter_delta
from mocap_g1.safety import SafetyConfig, SafetyError
from mocap_g1.schema import (
    G1_29DOF_JOINTS,
    G1_ARM_SDK_JOINTS,
    load_official_joint_limits,
)
from mocap_g1.xsens_live import GmrFrameConverter, LatestBodyReceiver


RUNNING = True
DEFAULT_LIMITS_URDF = official_urdf_path()


def _stop(_signum, _frame) -> None:
    global RUNNING
    RUNNING = False


def _running() -> bool:
    return RUNNING


def _print_receiver_diagnostics(receiver: LatestBodyReceiver) -> None:
    if receiver.unexpected_sources:
        print(f"Ignored unexpected sources: {sorted(receiver.unexpected_sources)}")
    print(
        f"Receiver: body_packets={receiver.total_body_packets}, "
        f"ignored_packets={receiver.total_ignored_packets}, "
        f"malformed_packets={receiver.total_malformed_packets}, "
        f"reordered_body_packets={receiver.total_reordered_body_packets}, "
        f"overload_events={receiver.overload_events}, "
        "discarded_over_budget_packets="
        f"{receiver.total_discarded_over_budget_packets}, "
        f"drain_budget={receiver.max_drain_packets}"
    )


def _dropped_between(previous_counter: int, counter: int) -> int:
    """Frames skipped between two consecutively accepted body counters.

    The receiver only hands over frames that advance the stream or a confirmed
    stream restart. A restart makes the signed delta negative; that is a new
    stream, not ~2**32 dropped frames.
    """
    return max(0, counter_delta(counter, previous_counter) - 1)


def _validate_safety_args(args: argparse.Namespace, parser: argparse.ArgumentParser) -> None:
    if not math.isfinite(args.control_rate) or args.control_rate <= 0:
        parser.error("--control-rate must be positive")
    if not math.isfinite(args.position_margin) or args.position_margin < 0:
        parser.error("--position-margin cannot be negative")
    if (
        not math.isfinite(args.velocity_limit_fraction)
        or not 0 < args.velocity_limit_fraction <= 1
    ):
        parser.error("--velocity-limit-fraction must be in (0, 1]")
    if not math.isfinite(args.acceleration_limit) or args.acceleration_limit <= 0:
        parser.error("--acceleration-limit must be positive")
    if not math.isfinite(args.stale_timeout) or args.stale_timeout <= 0:
        parser.error("--stale-timeout must be positive")
    if not math.isfinite(args.hard_timeout) or args.hard_timeout <= args.stale_timeout:
        parser.error("--hard-timeout must be greater than --stale-timeout")
    if not math.isfinite(args.blend_rate) or args.blend_rate <= 0:
        parser.error("--blend-rate must be positive")
    if 1.0 / args.control_rate >= args.hard_timeout:
        parser.error("--control-rate period must be shorter than --hard-timeout")
    if 1.0 / args.control_rate > SafetyConfig.max_step_dt_s:
        # The envelope credits at most max_step_dt_s of motion per command, so
        # a slower loop would track in slow motion rather than in real time.
        parser.error(
            f"--control-rate must be at least {1.0 / SafetyConfig.max_step_dt_s:g} Hz "
            "(the safety envelope's per-command step bound)"
        )
    if not args.limits_urdf.is_file():
        parser.error(f"--limits-urdf does not exist: {args.limits_urdf}")


def _build_live_conditioning(
    args: argparse.Namespace, parser: argparse.ArgumentParser
) -> tuple[OneEuroConfig | CriticallyDampedConfig | None, UpsamplerConfig | None]:
    """Validate the live filter/upsampler flags into configurations.

    Returns ``(None, None)`` for ``--no-filter``, which restores the pre-filter
    live path: raw retargeted references go straight to the safety envelope.
    """
    if not args.filter:
        return None, None
    if not math.isfinite(args.command_rate) or args.command_rate <= 0:
        parser.error("--command-rate must be positive")
    if args.command_rate < args.control_rate:
        parser.error("--command-rate must be at least --control-rate")
    if args.filter_type == OneEuroConfig.KIND:
        filter_config: OneEuroConfig | CriticallyDampedConfig = OneEuroConfig(
            min_cutoff_hz=args.filter_min_cutoff,
            beta=args.filter_beta,
            derivative_cutoff_hz=args.filter_derivative_cutoff,
            max_reference_gap_s=args.hard_timeout,
        )
    else:
        filter_config = CriticallyDampedConfig(
            cutoff_hz=args.filter_cutoff,
            max_reference_gap_s=args.hard_timeout,
        )
    upsampler_config = UpsamplerConfig(
        command_rate_hz=args.command_rate,
        mode=args.command_interpolation,
        source_rate_hz=args.control_rate,
        max_segment_s=2.0 / args.control_rate,
        stall_after_s=args.stale_timeout,
    )
    try:
        filter_config.validated()
        upsampler_config.validated()
    except LiveFilterError as exc:
        parser.error(str(exc))
    return filter_config, upsampler_config


def build_parser() -> argparse.ArgumentParser:
    """The CLI definition, separate from ``main`` so it is testable without GMR."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5555)
    parser.add_argument("--source-ip", default=None)
    parser.add_argument("--robot", choices=tuple(LIVE_ROBOTS), default="unitree_g1")
    parser.add_argument(
        "--arm-alignment",
        choices=ARM_ALIGNMENTS,
        default=None,
        help=(
            "arm orientation until N is pressed in a neutral pose: the IK config's "
            "offsets, or positions only (default: config for G1, position-only for T1)"
        ),
    )
    parser.add_argument("--human-height", type=float, default=None)
    parser.add_argument("--startup-timeout", type=float, default=60.0)
    parser.add_argument("--duration", type=float, default=0.0, help="zero runs until interrupted")
    parser.add_argument("--rate-limit", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--control-rate",
        type=float,
        default=60.0,
        help="MuJoCo safety-command rate in Hz (default: 60)",
    )
    parser.add_argument(
        "--start-disabled",
        action="store_true",
        help="start with waist/arm tracking disabled; press Space to enable",
    )
    parser.add_argument(
        "--limits-urdf",
        type=Path,
        default=DEFAULT_LIMITS_URDF,
        help="official G1 URDF supplying named waist/arm limits",
    )
    parser.add_argument(
        "--position-margin",
        type=float,
        default=0.05,
        metavar="RAD",
        help="G1 waist/arm limit margin (the simulation-only T1 uses exact MJCF ranges)",
    )
    parser.add_argument("--velocity-limit-fraction", type=float, default=0.25)
    parser.add_argument(
        "--acceleration-limit", type=float, default=4.0, metavar="RAD_S2"
    )
    parser.add_argument("--stale-timeout", type=float, default=0.10, metavar="SECONDS")
    parser.add_argument("--hard-timeout", type=float, default=0.50, metavar="SECONDS")
    parser.add_argument("--blend-rate", type=float, default=1.0, metavar="PER_SECOND")
    parser.add_argument(
        "--filter",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "causal joint smoothing plus fixed-rate command upsampling ahead of "
            "the safety envelope (default: on); --no-filter restores the "
            "pre-filter path where raw retargeted references reach the envelope"
        ),
    )
    parser.add_argument(
        "--filter-type",
        choices=FILTER_KINDS,
        default=OneEuroConfig.KIND,
        help="live smoothing filter family (default: one_euro)",
    )
    parser.add_argument(
        "--filter-min-cutoff",
        type=float,
        default=OneEuroConfig().min_cutoff_hz,
        metavar="HZ",
        help="one_euro cutoff at zero joint speed",
    )
    parser.add_argument(
        "--filter-beta",
        type=float,
        default=OneEuroConfig().beta,
        metavar="HZ_PER_RAD_S",
        help="one_euro speed-adaptive cutoff gain",
    )
    parser.add_argument(
        "--filter-derivative-cutoff",
        type=float,
        default=OneEuroConfig().derivative_cutoff_hz,
        metavar="HZ",
        help="one_euro cutoff for its internal speed estimate",
    )
    parser.add_argument(
        "--filter-cutoff",
        type=float,
        default=CriticallyDampedConfig().cutoff_hz,
        metavar="HZ",
        help="critically_damped -3 dB cutoff",
    )
    parser.add_argument(
        "--command-rate",
        type=float,
        default=DEFAULT_COMMAND_RATE_HZ,
        metavar="HZ",
        help=(
            "SDK-facing command rate produced by the upsampler (default: 250); "
            "in this simulator the schedule is exercised and measured, but "
            "MuJoCo is still stepped at --control-rate"
        ),
    )
    parser.add_argument(
        "--command-interpolation",
        choices=("interpolate", "hold"),
        default="interpolate",
        help=(
            "interpolate ramps toward the newest reference over one source "
            "period; hold is a zero-order hold with no added lag"
        ),
    )
    parser.add_argument("--record-video", action="store_true")
    parser.add_argument(
        "--g1-hardware-envelope",
        action="store_true",
        help=(
            "screen G1 waist/arm commands with the hardware safety envelope "
            "(velocity/acceleration limited, for real-robot readiness checks); "
            "the default simulation preview shows the retargeted pose, bounded by "
            "joint ranges, because the envelope lags fast arm motion"
        ),
    )
    parser.add_argument(
        "--arm-abduction",
        type=float,
        default=None,
        metavar="DEGREES",
        help=(
            "rotate the arm targets outward about the shoulders so hanging arms clear "
            "the hips (default: 10 for G1, 0 for T1)"
        ),
    )
    parser.add_argument(
        "--calibrate-after",
        type=float,
        default=None,
        metavar="SECONDS",
        help=(
            "press N automatically this many seconds after the stream starts, for a "
            "performer who cannot reach the keyboard (stand still, arms down)"
        ),
    )
    parser.add_argument(
        "--video-path", default=None, help="default: artifacts/xsens-live/<robot>-live.mp4"
    )
    return parser


#: Frame-local failures (rejected packet, retarget or filter error) are counted
#: in full but only the first few are printed, so a bad stream cannot flood
#: the console.
MAX_PRINTED_FAILURES = 3
#: Seconds between periodic ``G1 live:`` stats lines.
STATS_INTERVAL_S = 2.0


def pending_frame_expired(source_timestamp: float, now: float, stale_timeout_s: float) -> bool:
    """Whether a received but not yet retargeted frame is too old to use.

    Runtime construction can take longer than the safety timeout, so the
    startup packet may be stale by the time the loop starts.  It must never be
    presented as a newly arrived command.
    """
    return now - source_timestamp >= stale_timeout_s


@dataclass
class LiveLoopStats:
    """Counters behind the periodic ``<robot> live:`` line."""

    label: str = "G1"
    frames: int = 0
    failures: int = 0
    estimated_drops: int = 0
    safety_ticks: int = 0
    previous_counter: int | None = None
    reported_at: float = 0.0

    def record_failure(self, message: str) -> str | None:
        """Count one frame-local failure; return ``message`` if it should print."""
        self.failures += 1
        return message if self.failures <= MAX_PRINTED_FAILURES else None

    def accept_counter(self, counter: int) -> None:
        """Account for frames skipped before this accepted body counter."""
        if self.previous_counter is not None:
            self.estimated_drops += _dropped_between(self.previous_counter, counter)
        self.previous_counter = counter

    def periodic_line(self, *, now: float, started: float, safety_reason: str) -> str | None:
        """The stats line, once every :data:`STATS_INTERVAL_S` seconds."""
        if now - self.reported_at < STATS_INTERVAL_S:
            return None
        fps = self.frames / max(now - started, 1e-9)
        self.reported_at = now
        return (
            f"{self.label} live: {fps:.1f} FPS, frames={self.frames}, "
            f"safety_ticks={self.safety_ticks}, estimated_drops={self.estimated_drops}, "
            f"failures={self.failures}, safety={safety_reason}"
        )


@dataclass
class SafetyStatusReporter:
    """Turns safety-envelope outputs into change-only console lines."""

    previous_status: tuple[str, bool, bool] | None = None
    previous_clamped: tuple[str, ...] = field(default_factory=tuple)

    def status_line(
        self, output: LiveSafetyOutput, *, enabled: bool, last_error: str | None
    ) -> str | None:
        status = (output.reason, enabled, output.reset_required)
        if status == self.previous_status:
            return None
        self.previous_status = status
        reset_hint = " Press R to reset/rearm." if output.reset_required else ""
        error_detail = f" Detail: {last_error}." if last_error is not None else ""
        return (
            f"SIM SAFETY {output.reason}: enabled={enabled}, "
            f"engaged={output.engaged}, "
            f"source_age={output.source_age_s:.3f}s, "
            f"blend={output.blend_weight:.3f}."
            f"{reset_hint}{error_detail}"
        )

    def clamp_line(self, clamped_joints: tuple[str, ...]) -> str | None:
        if clamped_joints == self.previous_clamped:
            return None
        self.previous_clamped = clamped_joints
        if not clamped_joints:
            return None
        return "SIM SAFETY position clamp: " + ", ".join(clamped_joints)


class ReferencePipeline:
    """Live conditioning between retargeting and the safety envelope.

    It shapes the reference stream only: the optional joint filter smooths
    each retargeted reference and the optional upsampler schedules commands
    from it.  The envelope still clamps every target it is handed and remains
    the last authority.  Without an upsampler (``--no-filter``) each reference
    becomes one envelope target, as on the pre-filter path.
    """

    def __init__(
        self,
        safety: LiveG1SafetyController | LiveRangePreviewController,
        latency: LatencyRecorder,
        *,
        joint_filter: LiveJointFilter | None = None,
        upsampler: CommandUpsampler | None = None,
    ) -> None:
        self.safety = safety
        self.latency = latency
        self.joint_filter = joint_filter
        self.upsampler = upsampler
        #: Timeline of the newest pushed reference, closed by the first command
        #: scheduled from it.
        self.open_timeline: FrameTimeline | None = None

    def reset(self) -> None:
        """Drop all conditioning history (on rearm)."""
        self.open_timeline = None
        # The next frame must pass through unsmoothed rather than ramping out
        # of pre-rearm state.
        if self.joint_filter is not None:
            self.joint_filter.reset()
        if self.upsampler is not None:
            self.upsampler.reset()

    def condition(self, reference: np.ndarray, clock: Callable[[], float]) -> np.ndarray:
        """Filter one reference; raises :class:`LiveFilterError`."""
        if self.joint_filter is None:
            return reference
        return self.joint_filter.step(reference, now=clock()).values

    def submit(
        self,
        reference: np.ndarray,
        *,
        source_timestamp: float,
        timeline: FrameTimeline | None,
        clock: Callable[[], float],
    ) -> None:
        """Hand one conditioned reference on toward the envelope."""
        if self.upsampler is None:
            # Pre-filter timing: one envelope target per frame.
            self.safety.update_target(reference, source_timestamp=source_timestamp)
            if timeline is not None:
                timeline.mark("emit")
                self.latency.commit(timeline)
        else:
            self.upsampler.push(reference, now=clock(), source_timestamp=source_timestamp)
            self.open_timeline = timeline

    def pump(self, clock: Callable[[], float]) -> None:
        """Make the newest scheduled command this tick's envelope target.

        Honest simulator semantics: every scheduled command is generated and
        timed, but MuJoCo is stepped at --control-rate, so only the most recent
        command becomes this tick's envelope target.  On hardware the same
        object would be pumped by the SDK publish loop instead.
        """
        if self.upsampler is None:
            return
        commands = self.upsampler.pump(clock())
        if commands:
            command = commands[-1]
            self.safety.update_target(command.values, source_timestamp=command.source_timestamp)
            if self.open_timeline is not None:
                self.open_timeline.mark("emit", commands[0].emit_time)
                self.latency.commit(self.open_timeline)
                self.open_timeline = None

    def report_lines(self) -> list[str]:
        """The shutdown latency, upsampler and filter reports."""
        lines = [self.latency.format_report()]
        if self.upsampler is not None:
            lines.append(self.upsampler.format_stats())
        if self.joint_filter is not None and self.joint_filter.reinitializations > 1:
            lines.append(
                f"filter: {self.joint_filter.reinitializations - 1} reference-gap "
                f"reinitializations over {self.joint_filter.samples} samples"
            )
        return lines


@dataclass(frozen=True)
class StartupFrame:
    """The first valid body frame, which proves the stream before GMR starts."""

    body: Any
    frame: Any
    source: tuple[str, int]
    source_timestamp: float


def wait_for_first_frame(
    receiver: Any,
    converter: Any,
    *,
    timeout_s: float,
    clock: Callable[[], float],
    running: Callable[[], bool],
) -> StartupFrame | None:
    """Wait for one convertible body frame.

    Returns ``None`` if the operator stopped first, and raises
    :class:`TimeoutError` if no valid frame arrives within ``timeout_s``.
    """
    startup_deadline = clock() + timeout_s
    first_body = first_frame = source = None
    first_source_timestamp = None
    startup_failures = 0
    while running() and first_frame is None:
        remaining = startup_deadline - clock()
        if remaining <= 0:
            raise TimeoutError
        try:
            first_body, source = receiver.receive_latest(timeout=min(0.25, remaining))
            first_source_timestamp = clock()
            first_frame = converter.convert(first_body)
        except TimeoutError:
            continue
        except ProtocolError as exc:
            startup_failures += 1
            if startup_failures <= MAX_PRINTED_FAILURES:
                print(f"Rejected startup frame: {exc}", file=sys.stderr)
    if not running():
        return None
    assert (
        first_body is not None
        and first_frame is not None
        and source is not None
        and first_source_timestamp is not None
    )
    return StartupFrame(first_body, first_frame, source, first_source_timestamp)


class LiveLoop:
    """The receive -> retarget -> condition -> envelope -> viewer loop.

    I/O stays at the edges and is injected: ``receiver`` (UDP), ``converter``
    (MXTP02 -> GMR frame), ``retargeter`` (GMR), ``viewer`` (MuJoCo) and the
    monotonic ``clock``.  Each :meth:`tick` is one MuJoCo control period.
    """

    def __init__(
        self,
        *,
        receiver: Any,
        converter: Any,
        retargeter: Any,
        viewer: Any,
        safety: LiveG1SafetyController | LiveRangePreviewController,
        controls: LiveOperatorControls,
        pipeline: ReferencePipeline,
        startup: StartupFrame,
        control_rate_hz: float,
        stale_timeout_s: float,
        duration_s: float = 0.0,
        rate_limit: bool = True,
        clock: Callable[[], float] | None = None,
        running: Callable[[], bool] = _running,
        expected_nq: int = 7 + len(G1_29DOF_JOINTS),
        label: str = "G1",
        arm_alignment: LiveArmAlignment | None = None,
        feet: tuple[tuple[str, float], ...] = (),
        calibrate_after_s: float | None = None,
        abduction_deg: float = 0.0,
    ) -> None:
        self.receiver = receiver
        self.converter = converter
        self.retargeter = retargeter
        self.viewer = viewer
        self.safety = safety
        self.controls = controls
        self.pipeline = pipeline
        self.control_rate_hz = control_rate_hz
        self.stale_timeout_s = stale_timeout_s
        self.duration_s = duration_s
        self.rate_limit = rate_limit
        self.clock = clock if clock is not None else time.monotonic
        self.running = running
        self.expected_nq = expected_nq
        self.arm_alignment = arm_alignment
        self.feet = feet
        self.calibrate_after_s = calibrate_after_s
        self.abduction_deg = abduction_deg
        #: Root height correction from the last N-key neutral calibration.
        self.floor_offset = 0.0
        self.stats = LiveLoopStats(label=label)
        self.reporter = SafetyStatusReporter()
        self.started = 0.0
        self.pending_frame: Any = startup.frame
        self.pending_counter: int = startup.body.sample_counter
        self.pending_source_timestamp: float = startup.source_timestamp
        # The startup frame is parsed before the recorder exists and is usually
        # discarded as stale; leave it out rather than record a fake
        # zero-length parse stage.
        self.pending_timeline: FrameTimeline | None = None
        self.last_root: np.ndarray = np.zeros(7, dtype=np.float64)
        self.last_human_frame: Any = None

    def _measured_dof(self) -> np.ndarray:
        return np.asarray(self.viewer.data.qpos[7:], dtype=np.float64)

    def _report_failure(self, message: str) -> None:
        line = self.stats.record_failure(message)
        if line is not None:
            print(line, file=sys.stderr)

    def _clear_pending(self) -> None:
        self.pending_frame = None
        self.pending_timeline = None

    def start(self) -> None:
        """Arm the envelope from the viewer's current pose."""
        self.started = self.clock()
        self.safety.reset(self._measured_dof(), now=self.started)
        self.stats.reported_at = self.started
        self.last_root = np.asarray(self.viewer.data.qpos[:7], dtype=np.float64).copy()

    def should_continue(self) -> bool:
        return self.running() and (
            self.duration_s == 0 or self.clock() - self.started < self.duration_s
        )

    def run(self) -> None:
        while self.should_continue():
            self.tick()

    def tick(self) -> None:
        self.handle_rearm()
        self.expire_stale_pending()
        if self.pending_frame is None:
            self.receive()
        if self.pending_frame is not None:
            self.process_pending()
        self.pipeline.pump(self.clock)
        output = self.step_safety()
        self.viewer.step(
            root_pos=self.last_root[:3],
            root_rot=self.last_root[3:7],
            dof_pos=output.dof_pos,
            human_motion_data=self.last_human_frame,
            rate_limit=self.rate_limit,
            follow_camera=True,
        )
        clamp_line = self.reporter.clamp_line(output.clamped_joints)
        if clamp_line is not None:
            print(clamp_line, file=sys.stderr)
        stats_line = self.stats.periodic_line(
            now=self.clock(), started=self.started, safety_reason=output.reason
        )
        if stats_line is not None:
            print(stats_line)

    def handle_rearm(self) -> None:
        if not self.controls.take_rearm_request():
            return
        reset_time = self.clock()
        self.safety.reset(self._measured_dof(), now=reset_time)
        self._clear_pending()
        # Rearming invalidates the conditioning history too.
        self.pipeline.reset()
        print("SIM SAFETY rearmed from current MuJoCo qpos; waiting for a fresh frame")

    def expire_stale_pending(self) -> None:
        # Never present an old packet as a newly arrived command; discard it
        # and drain the receiver for a current packet instead.
        if self.pending_frame is not None and pending_frame_expired(
            self.pending_source_timestamp, self.clock(), self.stale_timeout_s
        ):
            self._clear_pending()

    def receive(self) -> None:
        try:
            body, _source = self.receiver.receive_latest(timeout=1.0 / self.control_rate_hz)
            self.pending_source_timestamp = self.clock()
            self.pending_timeline = self.pipeline.latency.open(self.pending_source_timestamp)
            self.pending_frame = self.converter.convert(body)
            self.pending_timeline.mark("parse")
            self.pending_counter = body.sample_counter
        except TimeoutError:
            self.pending_timeline = None
        except ProtocolError as exc:
            self.pending_timeline = None
            self._report_failure(f"Rejected frame: {exc}")

    def calibrate_arms(self) -> None:
        """Neutral-pose calibration from the pending frame (operator pressed N)."""
        if self.arm_alignment is None:
            print("ARM ALIGNMENT unavailable for this retargeter", file=sys.stderr)
            return
        try:
            offsets = {}
            if self.arm_alignment.mode != "config":
                offsets = self.arm_alignment.calibrate(
                    abduct_frame_rotations(self.pending_frame, self.abduction_deg)
                )
            floor = ""
            if self.feet:
                self.retargeter.retarget(
                    {name: (pose[0].copy(), pose[1].copy())
                     for name, pose in self.pending_frame.items()},
                    offset_to_ground=False,
                )
                self.floor_offset = floor_offset_m(self.retargeter, self.feet)
                floor = f"; feet placed on the floor (root {self.floor_offset:+.3f} m)"
        except Exception as exc:  # solver errors stay local, as in process_pending
            print(f"ARM ALIGNMENT failed: {exc}; arms unchanged", file=sys.stderr)
            return
        arms = (
            f"arm orientation from the current pose ({len(offsets)} arm segments)"
            if offsets else "arms keep the IK config's orientations"
        )
        print(f"NEUTRAL CALIBRATION: {arms}{floor}. Press N again in a neutral pose to redo it.")

    def process_pending(self) -> None:
        calibrate = self.controls.take_calibration_request()
        if (
            self.calibrate_after_s is not None
            and self.pending_source_timestamp - self.started >= self.calibrate_after_s
        ):
            self.calibrate_after_s = None
            calibrate = True
        if calibrate:
            self.calibrate_arms()
        timeline = self.pending_timeline
        try:
            qpos = np.asarray(
                self.retargeter.retarget(self.pending_frame, offset_to_ground=False),
                dtype=np.float64,
            )
            if qpos.shape != (self.expected_nq,) or not np.isfinite(qpos).all():
                raise ValueError(f"GMR returned invalid qpos shape/content: {qpos.shape}")
        except Exception as exc:  # upstream solver errors are frame-local
            self._report_failure(f"Retargeting failed: {exc}")
            self._clear_pending()
            return
        if timeline is not None:
            timeline.mark("retarget")
        try:
            reference = self.pipeline.condition(qpos[7:], self.clock)
        except LiveFilterError as exc:  # conditioning stays frame-local
            self._report_failure(f"Live filtering failed: {exc}")
            self._clear_pending()
            return
        if timeline is not None:
            timeline.mark("filter")
        self.stats.accept_counter(self.pending_counter)
        self.pipeline.submit(
            reference,
            source_timestamp=self.pending_source_timestamp,
            timeline=timeline,
            clock=self.clock,
        )
        self.last_root = qpos[:7].copy()
        self.last_root[2] += self.floor_offset
        self.last_human_frame = self.retargeter.scaled_human_data
        self.stats.frames += 1
        self._clear_pending()

    def step_safety(self) -> LiveSafetyOutput:
        now = self.clock()
        output = self.safety.step(self._measured_dof(), now=now, enable=self.controls.enabled)
        self.stats.safety_ticks += 1
        line = self.reporter.status_line(
            output, enabled=self.controls.enabled, last_error=self.safety.last_error
        )
        if line is not None:
            print(line)
        return output


def layout_error(retargeter: Any, robot: LiveRobot) -> str | None:
    """Why the loaded GMR model does not match the robot's live joint layout."""
    model = getattr(retargeter, "model", None)
    if model is None:
        return None
    import mujoco

    hinges = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        for joint in range(model.njnt)
        if model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_HINGE
    ]
    if model.nq != robot.nq or tuple(hinges) != robot.dof_joints:
        return (
            f"GMR model for {robot.key} has nq={model.nq} and hinges {hinges}; "
            f"the live tool expects nq={robot.nq} and {list(robot.dof_joints)}"
        )
    return None


def configure_ik(retargeter: Any, abduction_deg: float) -> str | None:
    """Joint limits that GMR really applies, plus arm abduction; ``None`` for fakes."""
    if not hasattr(retargeter, "ik_limits"):
        return None
    import mink
    from general_motion_retargeting import motion_retarget

    install_solve_ik_fix(motion_retarget)
    configure_live_ik_limits(retargeter, mink=mink)
    if not abduction_deg:
        return "IK: joint-range limits."
    ArmAbduction(retargeter, abduction_deg)
    return f"IK: joint-range limits; arms abducted {abduction_deg:g} deg to clear the hips."


def build_arm_alignment(retargeter: Any, mode: str) -> LiveArmAlignment | None:
    """Prepare N-key arm calibration; ``None`` if the retargeter lacks IK tasks."""
    if not hasattr(retargeter, "human_body_to_task1"):
        return None
    try:
        alignment = LiveArmAlignment(retargeter, MVN_ARM_BODIES)
    except ArmAlignmentError:
        return None
    if mode == "position-only":
        alignment.use_position_only()
    return alignment


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if args.startup_timeout <= 0:
        parser.error("--startup-timeout must be positive")
    if args.duration < 0:
        parser.error("--duration cannot be negative")
    if args.calibrate_after is not None and not (
        math.isfinite(args.calibrate_after) and args.calibrate_after >= 0
    ):
        parser.error("--calibrate-after must be zero or positive")
    _validate_safety_args(args, parser)
    filter_config, upsampler_config = _build_live_conditioning(args, parser)
    robot = LIVE_ROBOTS[args.robot]
    arm_alignment_mode = args.arm_alignment or robot.default_arm_alignment
    if args.video_path is None:
        args.video_path = f"artifacts/xsens-live/{robot.label.lower()}-live.mp4"
    if args.record_video:
        Path(args.video_path).expanduser().resolve().parent.mkdir(parents=True, exist_ok=True)

    safety_config = SafetyConfig(
        position_margin_rad=args.position_margin,
        velocity_limit_fraction=args.velocity_limit_fraction,
        acceleration_limit_rad_s2=args.acceleration_limit,
        stale_timeout_s=args.stale_timeout,
        hard_timeout_s=args.hard_timeout,
        blend_rate_per_s=args.blend_rate,
    )
    safety: LiveG1SafetyController | LiveRangePreviewController
    hardware_envelope = robot.key == "unitree_g1" and args.g1_hardware_envelope
    if hardware_envelope:
        limits = load_official_joint_limits(args.limits_urdf)
        collapsed = [
            name
            for name in G1_ARM_SDK_JOINTS
            if limits[name].lower + args.position_margin
            >= limits[name].upper - args.position_margin
        ]
        if collapsed:
            parser.error(
                "--position-margin collapses the safe range for: " + ", ".join(collapsed)
            )
        safety = LiveG1SafetyController(limits, safety_config)
    else:
        if robot.key == "unitree_g1":
            urdf = load_official_joint_limits(args.limits_urdf)
            joint_ranges = tuple(
                (name, urdf[name].lower, urdf[name].upper) for name in G1_29DOF_JOINTS
            )
        else:
            joint_ranges = T1_MOCAP_JOINT_RANGES
        try:
            # Simulation only: bound by the exact joint ranges.  A hardware margin
            # would clamp joints whose neutral sits at a limit (T1 knees, elbow yaw),
            # and the hardware envelope's velocity/acceleration caps make the arms
            # lag and cut through the body (measured: 0 -> 377 of 1680 frames).
            safety = LiveRangePreviewController(
                joint_ranges,
                robot.upper_body_joints,
                replace(safety_config, position_margin_rad=0.0),
            )
        except SafetyError as exc:
            parser.error(str(exc))
    controls = LiveOperatorControls(enabled=not args.start_disabled)

    latency = LatencyRecorder()
    pipeline = ReferencePipeline(
        safety,
        latency,
        joint_filter=(
            make_filter(filter_config, len(robot.dof_joints))
            if filter_config is not None
            else None
        ),
        upsampler=(
            CommandUpsampler(upsampler_config, len(robot.dof_joints))
            if upsampler_config is not None
            else None
        ),
    )
    if filter_config is not None:
        latency.note_algorithmic_lag(
            filter_lag_ms=measure_step_lag(
                filter_config, sample_rate_hz=args.control_rate
            ).lag_50_ms,
            upsampler_lag_ms=(
                upsampler_lag_ms(upsampler_config) if upsampler_config is not None else 0.0
            ),
        )
    print(
        format_configuration_report(
            filter_config, upsampler_config, sample_rate_hz=args.control_rate
        )
    )

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    if hasattr(signal, "SIGBREAK"):  # Windows console stop (CTRL_BREAK_EVENT)
        signal.signal(signal.SIGBREAK, _stop)

    try:
        from general_motion_retargeting import GeneralMotionRetargeting, RobotMotionViewer
    except ImportError as exc:
        print(f"ERROR: GMR live runtime is incomplete: {exc}", file=sys.stderr)
        return 2
    if robot.ik_config is not None:
        # GMR ships no xsens_mvn config for this robot; install the project's
        # in GMR's process-local registry (the checkout on disk is unchanged).
        if not robot.ik_config.is_file():
            print(f"ERROR: live IK config is missing: {robot.ik_config}", file=sys.stderr)
            return 2
        from general_motion_retargeting import params

        params.IK_CONFIG_DICT.setdefault("xsens_mvn", {})[robot.key] = str(robot.ik_config)

    receiver = LatestBodyReceiver(
        bind=args.bind,
        port=args.port,
        expected_source=args.source_ip,
        strict=True,
    )
    converter = GmrFrameConverter()
    viewer = None
    source_label = args.source_ip or "any source"
    print(f"Waiting for strict MXTP02 on {args.bind}:{args.port} from {source_label}...")

    try:
        startup = wait_for_first_frame(
            receiver,
            converter,
            timeout_s=args.startup_timeout,
            clock=time.monotonic,
            running=_running,
        )
        if startup is None:
            return 0
        print(f"Receiving Xsens body frames from {startup.source[0]}:{startup.source[1]}")

        retargeter = GeneralMotionRetargeting(
            src_human="xsens_mvn",
            tgt_robot=args.robot,
            actual_human_height=args.human_height,
            solver="daqp",
            damping=1.0,
            use_velocity_limit=True,
            verbose=False,
        )
        mismatch = layout_error(retargeter, robot)
        if mismatch is not None:
            print(f"ERROR: {mismatch}", file=sys.stderr)
            return 2
        abduction = robot.arm_abduction_deg if args.arm_abduction is None else args.arm_abduction
        ik_line = configure_ik(retargeter, abduction)
        arm_alignment = build_arm_alignment(retargeter, arm_alignment_mode)
        viewer = RobotMotionViewer(
            robot_type=args.robot,
            motion_fps=args.control_rate,
            transparent_robot=0,
            record_video=args.record_video,
            video_path=args.video_path,
            keyboard_callback=controls.keyboard_callback,
        )

        loop = LiveLoop(
            receiver=receiver,
            converter=converter,
            retargeter=retargeter,
            viewer=viewer,
            safety=safety,
            controls=controls,
            pipeline=pipeline,
            startup=startup,
            control_rate_hz=args.control_rate,
            stale_timeout_s=args.stale_timeout,
            duration_s=args.duration,
            rate_limit=args.rate_limit,
            clock=time.monotonic,
            running=_running,
            expected_nq=robot.nq,
            label=robot.label,
            arm_alignment=arm_alignment,
            feet=robot.feet,
            calibrate_after_s=args.calibrate_after,
            abduction_deg=abduction,
        )
        loop.start()

        if hardware_envelope:
            print(
                "MuJoCo waist/arm safety enabled: "
                f"stale={args.stale_timeout:g}s, hard={args.hard_timeout:g}s, "
                f"velocity={args.velocity_limit_fraction:g}x URDF, "
                f"acceleration={args.acceleration_limit:g} rad/s^2."
            )
        else:
            print(
                f"MuJoCo {robot.label} preview screening: joint ranges, "
                f"stale={args.stale_timeout:g}s, "
                f"hard={args.hard_timeout:g}s. No hardware velocity envelope: "
                "simulation only."
            )
        print(
            "Viewer controls: Space enables/disables bounded waist/arm tracking; "
            "R resets/rearms from the current MuJoCo pose."
        )
        if ik_line is not None:
            print(ik_line)
        if arm_alignment is not None:
            target = "floor height" if arm_alignment.mode == "config" else (
                "arm orientation and floor height"
            )
            print(
                f"Arms: {arm_alignment.mode} orientation. Stand still in a neutral pose "
                f"(arms down) and press N to calibrate {target}."
            )
        if args.calibrate_after is not None:
            print(
                f"Automatic N calibration in {args.calibrate_after:g} s: the performer "
                "should stand still with arms down."
            )
        sdk = "Unitree" if robot.key == "unitree_g1" else "robot"
        print(
            "Scope: simulator-only waist/arm command screening; legs and floating base are "
            f"still a kinematic preview, with no {sdk} SDK or real-robot state involved."
        )

        loop.run()
    except TimeoutError:
        print(
            f"ERROR: no valid MXTP02 body frame arrived on UDP {args.port} within "
            f"{args.startup_timeout:g} seconds",
            file=sys.stderr,
        )
        _print_receiver_diagnostics(receiver)
        return 3
    finally:
        if viewer is not None:
            viewer.close()
        receiver.close()
        for line in pipeline.report_lines():
            print(line)

    _print_receiver_diagnostics(receiver)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
