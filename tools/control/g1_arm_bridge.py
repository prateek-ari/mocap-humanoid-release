#!/usr/bin/env python3
"""Replay a recorded G1 waist/arm reference through the real ``rt/arm_sdk`` bridge.

Defaults to ``--transport fake``, which constructs and CRC-stamps every LowCmd
exactly as the DDS backend would but keeps them in process.  ``--transport dds``
publishes to a real robot and requires the optional ``robot`` dependency group.

The safety envelope is used exactly as ``tools/control/mock_g1_arm_bridge.py``
uses it: official URDF limits, default :class:`SafetyConfig`, and a
measured-state reset before any command is emitted.  The difference is that the
reset now comes from a ``LowState_`` snapshot rather than from midpoint defaults.

Examples::

    # inspect the first three constructed LowCmds
    python3 tools/control/g1_arm_bridge.py motion.npz --dry-run 3

    # full fake-transport replay, exercising the stale-state fail-closed path
    python3 tools/control/g1_arm_bridge.py motion.npz --stale-state-at 2.0

    # hardware (NOT yet validated -- read the arm_bridge module docstring first)
    python3 tools/control/g1_arm_bridge.py motion.npz --transport dds --net eth0

Exit status: 0 on success, 1 if the replay ran but failed its checks, 2 if the
inputs or arguments were invalid.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Iterator
import contextlib
from dataclasses import dataclass
import json
import math
from pathlib import Path
import signal
import sys
import time

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from mocap_g1.arm_bridge import (  # noqa: E402
    ARM_SDK_MOTOR_INDEX,
    WEIGHT_CHANNEL_INDEX,
    ArmBridgeConfig,
    UPSTREAM_EXAMPLE_KD,
    UPSTREAM_EXAMPLE_KP,
    ArmBridgeError,
    DdsTransport,
    FakeTransport,
    G1ArmBridge,
    LowStateSnapshot,
    arm_targets_from_qpos,
)
from mocap_g1.conditioning import MotionConditioningError, load_qpos_npz  # noqa: E402
from mocap_g1.safety import SafetyConfig, SafetyError  # noqa: E402
from mocap_g1.schema import G1_ARM_SDK_JOINTS, load_official_joint_limits  # noqa: E402


DEFAULT_URDF = REPO_ROOT / "eval/assets/g1_29dof_official.urdf"
DDS_MAX_CONTROL_OVERRUN_S = 0.100
#: Upper bound on the controlled shutdown ramp, in seconds of control ticks.
SHUTDOWN_RAMP_MAX_S = 2.0


@dataclass(frozen=True)
class DdsReplayTick:
    """One real-time replay deadline selected for publication."""

    tick: int
    replay_time_s: float
    now_s: float
    skipped_samples: int


def iter_dds_replay_ticks(
    *,
    steps: int,
    control_dt: float,
    start_time: float,
    clock: Callable[[], float],
    sleeper: Callable[[float], None],
    max_overrun_s: float = DDS_MAX_CONTROL_OVERRUN_S,
) -> Iterator[DdsReplayTick]:
    """Pace DDS replay without publishing a burst after a scheduler pause.

    When more than one sample deadline has passed, only the newest due replay
    sample is yielded. An overrun of 100 ms or more is treated as a loss of
    real-time control and aborts the replay instead of resuming with stale work.
    The injected clock and sleeper keep the policy deterministic in tests.
    """
    if steps <= 0:
        raise ArmBridgeError("DDS replay steps must be positive")
    if not math.isfinite(control_dt) or control_dt <= 0.0:
        raise ArmBridgeError("DDS replay control_dt must be finite and positive")
    if not math.isfinite(start_time):
        raise ArmBridgeError("DDS replay start_time must be finite")
    if not math.isfinite(max_overrun_s) or max_overrun_s <= 0.0:
        raise ArmBridgeError("DDS maximum control overrun must be finite and positive")

    tick = 1
    while tick <= steps:
        deadline = start_time + tick * control_dt
        now = float(clock())
        if not math.isfinite(now):
            raise ArmBridgeError("DDS replay clock returned a non-finite value")
        while now < deadline:
            before_sleep = now
            try:
                sleeper(deadline - now)
            except InterruptedError as exc:
                raise ArmBridgeError(
                    "DDS replay sleep was interrupted; aborting replay"
                ) from exc
            now = float(clock())
            if not math.isfinite(now):
                raise ArmBridgeError("DDS replay clock returned a non-finite value")
            if now <= before_sleep:
                raise ArmBridgeError(
                    "DDS replay clock did not advance while sleeping; aborting replay"
                )

        overrun = max(0.0, now - deadline)
        if overrun >= max_overrun_s:
            raise ArmBridgeError(
                f"DDS control deadline overrun was {overrun:.6g}s "
                f"(limit {max_overrun_s:.6g}s); aborting replay"
            )

        newest_due_tick = min(steps, max(tick, int(math.floor((now - start_time) / control_dt))))
        skipped = newest_due_tick - tick
        tick = newest_due_tick
        yield DdsReplayTick(
            tick=tick,
            replay_time_s=tick * control_dt,
            now_s=now,
            skipped_samples=skipped,
        )
        tick += 1


def shutdown_ramp_ticks(weight: float, *, blend_rate_per_s: float, control_dt: float) -> int:
    """Ticks a non-latching shutdown may spend ramping ``weight`` down to 0.

    The envelope ramps at ``blend_rate_per_s``; one spare tick absorbs float
    rounding, and the total is capped at :data:`SHUTDOWN_RAMP_MAX_S` so a slow
    configured blend rate cannot hold authority indefinitely.  Whatever is
    left at the cap is dropped by the terminal disarm.
    """
    if weight <= 0.0:
        return 0
    needed = int(math.ceil(weight / (blend_rate_per_s * control_dt))) + 1
    cap = max(1, int(math.floor(SHUTDOWN_RAMP_MAX_S / control_dt)))
    return min(needed, cap)


class SigintGate:
    """SIGINT handler state for the replay and its must-complete shutdown.

    While :meth:`allow_interrupt` is in effect, the *first* SIGINT raises
    ``KeyboardInterrupt`` (the usual Ctrl-C) and disarms itself in the same
    handler call, so at most one ``KeyboardInterrupt`` is ever raised.  Every
    other SIGINT is only appended to :attr:`received` for the caller to poll.
    """

    def __init__(self) -> None:
        self.received: list[int] = []
        self._interrupt_next = False

    def handle(self, signum: int, _frame: object) -> None:
        if self._interrupt_next:
            self._interrupt_next = False
            raise KeyboardInterrupt
        self.received.append(signum)

    def allow_interrupt(self) -> None:
        """Let the next SIGINT raise ``KeyboardInterrupt`` (once).

        A SIGINT recorded before this call is delivered now, as that
        interrupt.  Call it inside the ``try`` that handles the interrupt.
        """
        if self.received:
            self.received.clear()
            raise KeyboardInterrupt
        self._interrupt_next = True

    def defer(self) -> None:
        """From now on, only record SIGINT."""
        self._interrupt_next = False


@contextlib.contextmanager
def sigint_gate() -> Iterator[SigintGate]:
    """Install a :class:`SigintGate` (deferring) for the block.

    The previous handler is always restored.  Outside the main thread (where
    a handler cannot be installed and ``KeyboardInterrupt`` is never
    delivered anyway) the block runs unguarded.
    """
    gate = SigintGate()
    try:
        previous = signal.signal(signal.SIGINT, gate.handle)
    except ValueError:  # not the main thread
        installed = False
    else:
        installed = True
    try:
        yield gate
    finally:
        if installed:
            signal.signal(
                signal.SIGINT,
                previous if previous is not None else signal.default_int_handler,
            )


@contextlib.contextmanager
def deferred_sigint() -> Iterator[list[int]]:
    """Record SIGINT instead of raising ``KeyboardInterrupt`` inside the block.

    The caller inspects the yielded list afterwards.  See :func:`sigint_gate`.
    """
    with sigint_gate() as gate:
        yield gate.received


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("input", type=Path, help="recorded/conditioned qpos NPZ")
    parser.add_argument(
        "--transport",
        choices=("fake", "dds"),
        default="fake",
        help="fake (default, in-process) or dds (real robot; needs the robot extra)",
    )
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--rate", type=float, default=50.0, help="control rate in Hz")
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="stop after this many seconds of replay (default: whole clip)",
    )
    parser.add_argument(
        "--dry-run",
        type=int,
        nargs="?",
        const=3,
        default=None,
        metavar="N",
        help="print the first N constructed LowCmds and exit without a full replay",
    )
    parser.add_argument(
        "--kp",
        type=float,
        default=ArmBridgeConfig.kp,
        help=f"position gain; at most {UPSTREAM_EXAMPLE_KP:g} (upstream example) "
        "unless --allow-gains-above-example",
    )
    parser.add_argument(
        "--kd",
        type=float,
        default=ArmBridgeConfig.kd,
        help=f"damping gain; at most {UPSTREAM_EXAMPLE_KD:g} (upstream example) "
        "unless --allow-gains-above-example",
    )
    parser.add_argument(
        "--allow-gains-above-example",
        action="store_true",
        help="permit --kp/--kd above the upstream arm_sdk example values",
    )
    parser.add_argument(
        "--stale-state-at",
        type=float,
        default=None,
        help="fake transport only: stop delivering LowState_ at this replay time, "
        "to exercise the fail-closed path",
    )
    parser.add_argument(
        "--disable-at",
        type=float,
        default=None,
        help="request tracking disable from this replay time onward",
    )
    parser.add_argument("--jsonl", type=Path, help="write one JSON record per control tick")
    parser.add_argument("--domain-id", type=int, default=0, help="dds only: DDS domain id")
    parser.add_argument(
        "--net",
        dest="network_interface",
        default=None,
        help="dds only: network interface bound by ChannelFactoryInitialize",
    )
    parser.add_argument(
        "--yes-i-have-read-the-unverified-section",
        action="store_true",
        help="required with --transport dds; nothing here has run on hardware",
    )
    return parser


def format_lowcmd(index: int, step, *, limit: int | None = None) -> str:
    """Render one constructed LowCmd for --dry-run."""
    frame = step.frame
    lines = [
        f"--- LowCmd #{index}  reason={step.reason}  weight={step.weight:.6f}  "
        f"crc=0x{frame.crc:08X} ---",
        f"    mode_pr={frame.mode_pr}  mode_machine={frame.mode_machine}  "
        f"packed={len(frame.pack())} bytes  state_age={step.state_age_s:.6f}s",
        f"    motor[{WEIGHT_CHANNEL_INDEX}] (kNotUsedJoint / arm_sdk weight): "
        f"q={frame.motor_cmd[WEIGHT_CHANNEL_INDEX].q:.6f}",
    ]
    names = list(ARM_SDK_MOTOR_INDEX)
    if limit is not None:
        names = names[:limit]
    for name in names:
        motor_index = ARM_SDK_MOTOR_INDEX[name]
        motor = frame.motor_cmd[motor_index]
        lines.append(
            f"    motor[{motor_index:2d}] {name:<28s} q={motor.q:+.6f} dq={motor.dq:+.6f} "
            f"kp={motor.kp:g} kd={motor.kd:g} tau={motor.tau:g} mode={motor.mode}"
        )
    if limit is not None and limit < len(ARM_SDK_MOTOR_INDEX):
        lines.append(f"    ... {len(ARM_SDK_MOTOR_INDEX) - limit} more arm_sdk joints")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.rate <= 0 or not math.isfinite(args.rate):
        parser.error("--rate must be a positive, finite number of Hz")
    if 1.0 / args.rate > SafetyConfig.max_step_dt_s:
        # The envelope credits at most max_step_dt_s of motion per command, so
        # a slower loop would track in slow motion rather than in real time.
        parser.error(
            f"--rate must be at least {1.0 / SafetyConfig.max_step_dt_s:g} Hz "
            "(the safety envelope's per-command step bound)"
        )
    if args.duration is not None and (
        not math.isfinite(args.duration) or args.duration <= 0
    ):
        parser.error("--duration must be a positive, finite number")
    if args.dry_run is not None and args.dry_run <= 0:
        parser.error("--dry-run count must be positive")
    if args.transport == "dds" and not args.yes_i_have_read_the_unverified_section:
        parser.error(
            "--transport dds requires --yes-i-have-read-the-unverified-section; "
            "no part of this bridge has been validated against a physical G1"
        )
    if args.transport == "dds" and args.stale_state_at is not None:
        parser.error("--stale-state-at is a fake-transport simulation only")
    for option, value in (
        ("--stale-state-at", args.stale_state_at),
        ("--disable-at", args.disable_at),
    ):
        if value is not None and (not math.isfinite(value) or value < 0.0):
            parser.error(f"{option} must be a finite, non-negative replay time")

    try:
        qpos, fps, metadata = load_qpos_npz(args.input)
    except MotionConditioningError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    try:
        limits = load_official_joint_limits(args.urdf)
    except (OSError, ValueError) as exc:
        print(f"ERROR: cannot load joint limits from {args.urdf}: {exc}", file=sys.stderr)
        return 2

    safety_config = SafetyConfig()
    bridge_config = ArmBridgeConfig(
        kp=args.kp,
        kd=args.kd,
        allow_gains_above_example=args.allow_gains_above_example,
    )
    try:
        safety_config.validate()
        bridge_config.validate()
    except (ArmBridgeError, SafetyError) as exc:
        print(f"ERROR: invalid safety/bridge configuration: {exc}", file=sys.stderr)
        return 2

    control_dt = 1.0 / args.rate
    frames = len(qpos)
    clip_duration = (frames - 1) / fps
    duration = clip_duration if args.duration is None else min(args.duration, clip_duration)
    # The envelope is reset at t=0 and requires strictly increasing control
    # time, so the first command tick is at t=control_dt, not at t=0.
    steps = max(1, int(math.floor(duration / control_dt)))
    if args.dry_run is not None:
        steps = min(steps, args.dry_run)

    initial_targets = arm_targets_from_qpos(qpos[0])
    for name, value in initial_targets.items():
        limit = limits[name]
        if not limit.lower <= value <= limit.upper:
            print(
                f"ERROR: initial recorded {name}={value} is outside the official range",
                file=sys.stderr,
            )
            return 2

    try:
        handle = args.jsonl.open("w", encoding="utf-8") if args.jsonl else None
    except OSError as exc:
        print(f"ERROR: cannot open JSONL output {args.jsonl}: {exc}", file=sys.stderr)
        return 2

    transport = None
    bridge = None
    preloop_complete = False
    try:
        if args.transport == "fake":
            transport = FakeTransport(record_limit=0)
        else:
            transport = DdsTransport(
                domain_id=args.domain_id, network_interface=args.network_interface
            )
            transport.connect(clock=time.monotonic)

        bridge = G1ArmBridge(
            transport, limits, config=bridge_config, safety_config=safety_config
        )

        # Seed measured state. For the fake transport this stands in for
        # encoders: the first reference frame arms the envelope, and thereafter
        # the last emitted command is echoed back. This is a simulation
        # convenience, not a plant model or hardware feedback.
        if args.transport == "fake":
            transport.feed_arm_positions(
                initial_targets, receive_time_s=0.0, mode_machine=0
            )

        # DDS callbacks and safety checks share the same monotonic clock. Wait
        # for the first measured state instead of racing subscriber startup.
        if args.transport == "dds":
            state_deadline = time.monotonic() + bridge_config.state_hard_timeout_s
            while (
                transport.latest_state() is None
                and time.monotonic() < state_deadline
            ):
                time.sleep(min(control_dt, 0.01))
            if transport.latest_state() is None:
                raise ArmBridgeError(
                    "cannot arm bridge: no LowState_ arrived before the "
                    "measured-state timeout"
                )
            arm_time = time.monotonic()
        else:
            arm_time = 0.0

        bridge.arm(now=arm_time)
        preloop_complete = True
    except (ArmBridgeError, SafetyError) as exc:
        print(f"ERROR: cannot initialize or arm bridge: {exc}", file=sys.stderr)
        return 2
    finally:
        if not preloop_complete:
            try:
                if transport is not None:
                    transport.close()
            except Exception as exc:
                print(f"ERROR: transport cleanup failed: {exc}", file=sys.stderr)
            finally:
                try:
                    if handle:
                        handle.close()
                except Exception as exc:
                    print(f"ERROR: JSONL cleanup failed: {exc}", file=sys.stderr)
    reason_counts: dict[str, int] = {}
    max_weight = 0.0
    final_weight = 0.0
    printed = 0
    crc_values: list[int] = []
    stale_seen = False
    engaged_seen = False

    tick = 0
    executed_steps = 0
    skipped_samples = 0
    last_control_now = arm_time
    last_targets = initial_targets
    replay_failed = False
    operator_stopped = False
    teardown_errors: list[str] = []

    def echo_fake_state(step, now: float, replay_time: float) -> None:
        # Simulated encoder echo for the fake transport only.
        if args.transport != "fake":
            return
        deliver = args.stale_state_at is None or replay_time < args.stale_state_at
        if deliver and step.command is not None:
            transport.feed_state(
                LowStateSnapshot.from_arm_positions(
                    step.command.positions,
                    receive_time_s=now,
                    mode_machine=0,
                )
            )
    # SIGINT is routed through one gate for the replay and the shutdown that
    # follows it.  During the replay the first Ctrl-C raises KeyboardInterrupt
    # as usual; every later one -- and any once shutdown begins -- is only
    # recorded, so no code path between the replay and the end of the
    # terminal release can be cut short by a repeated Ctrl-C.
    with sigint_gate() as gate:
        try:
            try:
                gate.allow_interrupt()
                if args.transport == "dds":
                    control_ticks: Iterator[DdsReplayTick] = iter_dds_replay_ticks(
                        steps=steps,
                        control_dt=control_dt,
                        start_time=arm_time,
                        clock=time.monotonic,
                        sleeper=time.sleep,
                    )
                else:
                    control_ticks = (
                        DdsReplayTick(
                            tick=fake_tick,
                            replay_time_s=fake_tick * control_dt,
                            now_s=fake_tick * control_dt,
                            skipped_samples=0,
                        )
                        for fake_tick in range(1, steps + 1)
                    )

                for control_tick in control_ticks:
                    tick = control_tick.tick
                    replay_time = control_tick.replay_time_s
                    now = control_tick.now_s
                    executed_steps += 1
                    skipped_samples += control_tick.skipped_samples
                    last_control_now = now
                    source_index = min(frames - 1, int(math.floor(replay_time * fps)))
                    source_timestamp = source_index / fps
                    targets = arm_targets_from_qpos(qpos[source_index])
                    last_targets = targets
                    enable = args.disable_at is None or replay_time < args.disable_at

                    step = bridge.step(
                        targets,
                        # This replay sampled the target in the current control tick.
                        # The recorded timestamp selects the frame but is not a live
                        # arrival timestamp in the safety clock domain.
                        source_timestamp=now,
                        now=now,
                        enable=enable,
                        # ``--dry-run`` must be construction-only even when a DDS
                        # transport was selected.  Connecting for measured state is
                        # permitted; publishing authority is not.
                        publish=args.dry_run is None,
                    )

                    reason_key = step.reason.split(":", 1)[0]
                    reason_counts[reason_key] = reason_counts.get(reason_key, 0) + 1
                    max_weight = max(max_weight, step.weight)
                    final_weight = step.weight
                    engaged_seen |= step.engaged
                    stale_seen |= reason_key in {"stale_state", "state_hard_timeout"}
                    crc_values.append(step.frame.crc)

                    if args.dry_run is not None and printed < args.dry_run:
                        print(format_lowcmd(printed, step))
                        printed += 1

                    if handle:
                        handle.write(
                            json.dumps(
                                {
                                    "tick": tick,
                                    "time_s": replay_time,
                                    "recorded_time_s": source_timestamp,
                                    "skipped_samples": control_tick.skipped_samples,
                                    "source_index": source_index,
                                    "reason": step.reason,
                                    "weight": step.weight,
                                    "engaged": step.engaged,
                                    "state_age_s": (
                                        None if math.isinf(step.state_age_s) else step.state_age_s
                                    ),
                                    "crc": step.frame.crc,
                                    "positions": {
                                        name: step.frame.joint_q(name) for name in G1_ARM_SDK_JOINTS
                                    },
                                },
                                sort_keys=True,
                            )
                            + "\n"
                        )

                    echo_fake_state(step, now, replay_time)
            except (ArmBridgeError, ValueError) as exc:
                replay_failed = True
                print(f"ERROR: replay failed at tick {tick}: {exc}", file=sys.stderr)
        except KeyboardInterrupt:
            operator_stopped = True
            print("operator stop: ramping arm_sdk authority down", file=sys.stderr)
        finally:
            try:
                # Only the gate's one interrupt can still be pending here; any
                # later SIGINT is recorded in gate.received, never raised.
                gate.defer()
            except KeyboardInterrupt:
                operator_stopped = True
                print("operator stop: ramping arm_sdk authority down", file=sys.stderr)
            # Always hand authority back before releasing DDS resources.  Closing
            # the transport is not itself a robot stop: the last accepted
            # ``rt/arm_sdk`` weight could otherwise remain non-zero until another
            # publisher or a robot-side watchdog intervenes.
            #
            # Non-latching exits (end of replay, operator Ctrl-C) first ramp the
            # weight down through the envelope with enable=False, so positions
            # decelerate and authority fades at blend_rate_per_s.  A failed replay
            # or a latched bridge skips the ramp: its state is not trusted, and
            # the disarm below drops authority at once.
            #
            # From here on SIGINT is deferred by the gate, so a further Ctrl-C can
            # never skip the terminal release.  The ramp polls gate.received each
            # tick: a Ctrl-C recorded before or during the ramp ends it within one
            # control period and the release starts at once.
            reported_interrupts = 0
            if (
                not replay_failed
                and args.dry_run is None
                and bridge.armed
                and final_weight > 0.0
            ):
                ramp_ticks = shutdown_ramp_ticks(
                    final_weight,
                    blend_rate_per_s=safety_config.blend_rate_per_s,
                    control_dt=control_dt,
                )
                try:
                    ramp_interrupted = False
                    for ramp_index in range(1, ramp_ticks + 1):
                        if args.transport == "dds":
                            time.sleep(control_dt)
                            ramp_now = time.monotonic()
                        else:
                            ramp_now = last_control_now + control_dt
                        if gate.received:
                            ramp_interrupted = True
                            break
                        ramp_step = bridge.step(
                            last_targets,
                            source_timestamp=ramp_now,
                            now=ramp_now,
                            enable=False,
                        )
                        last_control_now = ramp_now
                        final_weight = ramp_step.weight
                        echo_fake_state(
                            ramp_step, ramp_now, (steps + ramp_index) * control_dt
                        )
                        if ramp_step.weight <= 0.0 or ramp_step.command is None:
                            break
                    if ramp_interrupted:
                        reported_interrupts = len(gate.received)
                        print(
                            "ERROR: shutdown ramp interrupted; releasing immediately",
                            file=sys.stderr,
                        )
                        operator_stopped = True
                    else:
                        print(f"shutdown: ramped arm_sdk weight to {final_weight:.6f}")
                except KeyboardInterrupt:
                    # Not a SIGINT (those are deferred): a KeyboardInterrupt raised
                    # directly by the step.  Treated like an operator stop.
                    print(
                        "ERROR: shutdown ramp interrupted; releasing immediately",
                        file=sys.stderr,
                    )
                    operator_stopped = True
                except Exception as exc:
                    message = f"shutdown ramp failed: {type(exc).__name__}: {exc}"
                    teardown_errors.append(message)
                    print(f"ERROR: {message}", file=sys.stderr)
            # The terminal release must complete even if the operator presses
            # Ctrl-C again: every weight-0 repeat is still sent, and the run then
            # exits non-zero (130) instead of escaping half-way through.
            try:
                shutdown_now = (
                    last_control_now
                    if args.transport == "dds"
                    else max(control_dt, last_control_now + control_dt)
                )
                if args.transport == "dds":
                    try:
                        shutdown_now = max(shutdown_now, time.monotonic())
                    except Exception as exc:
                        message = (
                            "shutdown clock failed; using last control timestamp: "
                            f"{type(exc).__name__}: {exc}"
                        )
                        teardown_errors.append(message)
                        print(f"ERROR: {message}", file=sys.stderr)
                try:
                    # Repeated weight-0 frames, paced one control period apart
                    # on DDS so they are distinct samples rather than one burst.
                    shutdown = bridge.disarm(
                        now=shutdown_now,
                        publish=args.dry_run is None,
                        between=(
                            (lambda: time.sleep(control_dt))
                            if args.transport == "dds"
                            else None
                        ),
                    )
                    if args.dry_run is None:
                        print(
                            "shutdown: published explicit disarm "
                            f"x{bridge_config.release_repeat_count} "
                            f"(reason={shutdown.reason}, weight={shutdown.weight:.6f})"
                        )
                except Exception as exc:
                    message = f"explicit disarm failed: {type(exc).__name__}: {exc}"
                    teardown_errors.append(message)
                    print(f"ERROR: {message}", file=sys.stderr)
                if len(gate.received) > reported_interrupts:
                    print(
                        "operator stop during release: all weight-0 repeats were still sent",
                        file=sys.stderr,
                    )
                if gate.received:
                    operator_stopped = True
            finally:
                try:
                    transport.close()
                except Exception as exc:
                    message = f"transport cleanup failed: {type(exc).__name__}: {exc}"
                    teardown_errors.append(message)
                    print(f"ERROR: {message}", file=sys.stderr)
                finally:
                    try:
                        if handle:
                            handle.close()
                    except Exception as exc:
                        message = f"JSONL cleanup failed: {type(exc).__name__}: {exc}"
                        teardown_errors.append(message)
                        print(f"ERROR: {message}", file=sys.stderr)

    if replay_failed or teardown_errors:
        return 1
    if operator_stopped:
        return 130

    if args.dry_run is not None:
        print(
            f"\nDRY RUN: constructed {executed_steps} LowCmd(s) for "
            f"{len(ARM_SDK_MOTOR_INDEX)} "
            f"arm_sdk joints via --transport {args.transport}; nothing was sent to a robot."
        )
        if skipped_samples:
            print(f"  skipped overdue replay samples: {skipped_samples}")
        return 0

    print(
        f"replayed {executed_steps} control ticks at {args.rate:g} Hz from {args.input} "
        f"({frames} frames @ {fps:g} fps, metadata.offsets_key="
        f"{metadata.get('offsets_key')!r})"
    )
    if skipped_samples:
        print(f"  skipped overdue replay samples: {skipped_samples}")
    print(f"  reasons: {json.dumps(reason_counts, sort_keys=True)}")
    print(f"  arm_sdk weight: max={max_weight:.6f} final={final_weight:.6f}")
    print(f"  distinct LowCmd CRCs: {len(set(crc_values))} of {len(crc_values)}")

    if not engaged_seen:
        print("FAIL: the bridge never engaged; no command carried authority", file=sys.stderr)
        return 1
    if args.stale_state_at is not None and not stale_seen:
        print("FAIL: injected measured-state stall did not fail closed", file=sys.stderr)
        return 1
    if args.stale_state_at is not None and final_weight != 0.0:
        print(
            f"FAIL: authority weight did not reach 0 after the state stall "
            f"(final={final_weight:.6f})",
            file=sys.stderr,
        )
        return 1
    print("PASS: bridge armed from measured state and produced CRC-stamped LowCmds")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
