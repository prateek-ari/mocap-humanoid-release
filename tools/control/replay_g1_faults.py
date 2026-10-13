#!/usr/bin/env python3
"""Replay recorded G1 commands through deterministic transport/control faults.

This is a hardware-free software state-machine test, not hardware validation
and not calibration. Existing report files are never overwritten.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from mocap_g1.conditioning import load_qpos_npz  # noqa: E402
from mocap_g1.fault_replay import (  # noqa: E402
    FaultReplayConfig,
    FaultReplayError,
    TimeInterval,
    replay_arm_faults,
)
from mocap_g1.io_utils import sha256_file, write_json_atomic  # noqa: E402
from mocap_g1.safety import SafetyConfig  # noqa: E402
from mocap_g1.schema import load_official_joint_limits  # noqa: E402


DEFAULT_URDF = REPO_ROOT / "eval/assets/g1_29dof_official.urdf"


def _interval(value: str) -> TimeInterval:
    try:
        start, duration = value.split(":", maxsplit=1)
        interval = TimeInterval(float(start), float(duration))
        interval.validate("fault interval")
        return interval
    except (ValueError, FaultReplayError) as exc:
        raise argparse.ArgumentTypeError(
            f"expected START_SECONDS:DURATION_SECONDS, got {value!r}: {exc}"
        ) from exc


def _write_report(path: Path, report: dict[str, object]) -> None:
    write_json_atomic(path.resolve(), report, exists_error=FaultReplayError, label="report")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="recorded/conditioned qpos NPZ")
    parser.add_argument("--output", type=Path, required=True, help="new JSON report")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--control-rate", type=float, default=60.0)
    parser.add_argument("--latency-ms", type=float, default=20.0)
    parser.add_argument("--packet-loss-every", type=int, default=11)
    parser.add_argument("--reorder-every", type=int, default=13)
    parser.add_argument("--reorder-delay-ms", type=float, default=55.0)
    parser.add_argument(
        "--source-stall",
        action="append",
        type=_interval,
        default=[],
        metavar="START:DURATION",
        help="suppress source packets in interval; repeatable",
    )
    parser.add_argument(
        "--disable",
        action="append",
        type=_interval,
        default=[],
        metavar="START:DURATION",
        help="set enable false in interval; repeatable",
    )
    parser.add_argument(
        "--control-gap",
        action="append",
        type=_interval,
        default=[],
        metavar="START:DURATION",
        help="skip control ticks in interval; repeatable",
    )
    parser.add_argument("--source-hard-reset-delay", type=float, default=0.10)
    parser.add_argument("--tracking-p95-limit-rad", type=float, default=0.35)
    parser.add_argument("--tracking-max-limit-rad", type=float, default=0.75)
    parser.add_argument("--clamp-command-fraction-limit", type=float, default=0.01)
    parser.add_argument("--velocity-fraction", type=float, default=0.25)
    parser.add_argument("--acceleration-limit", type=float, default=4.0)
    parser.add_argument("--stale-timeout", type=float, default=0.10)
    parser.add_argument("--hard-timeout", type=float, default=0.50)
    parser.add_argument("--blend-rate", type=float, default=1.0)
    parser.add_argument(
        "--require-reason",
        action="append",
        choices=("tracking", "disabled", "stale_source", "hard_timeout"),
        default=[],
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    if not math.isfinite(args.control_rate) or args.control_rate <= 0:
        parser.error("--control-rate must be a positive, finite number of Hz")
    if 1.0 / args.control_rate > SafetyConfig.max_step_dt_s:
        # The envelope credits at most max_step_dt_s of motion per command, so
        # a slower loop would replay in slow motion rather than in real time.
        parser.error(
            f"--control-rate must be at least {1.0 / SafetyConfig.max_step_dt_s:g} Hz "
            "(the safety envelope's per-command step bound)"
        )
    try:
        qpos, fps, metadata = load_qpos_npz(args.input)
        limits = load_official_joint_limits(args.urdf)
        report = replay_arm_faults(
            qpos,
            fps,
            limits=limits,
            replay_config=FaultReplayConfig(
                control_rate_hz=args.control_rate,
                latency_s=args.latency_ms / 1000.0,
                packet_loss_every=args.packet_loss_every,
                reorder_every=args.reorder_every,
                reorder_delay_s=args.reorder_delay_ms / 1000.0,
                source_stalls=tuple(args.source_stall),
                disable_intervals=tuple(args.disable),
                control_gaps=tuple(args.control_gap),
                source_hard_reset_delay_s=args.source_hard_reset_delay,
                tracking_p95_limit_rad=args.tracking_p95_limit_rad,
                tracking_max_limit_rad=args.tracking_max_limit_rad,
                clamp_command_fraction_limit=args.clamp_command_fraction_limit,
                require_reasons=tuple(args.require_reason),
            ),
            safety_config=SafetyConfig(
                velocity_limit_fraction=args.velocity_fraction,
                acceleration_limit_rad_s2=args.acceleration_limit,
                stale_timeout_s=args.stale_timeout,
                hard_timeout_s=args.hard_timeout,
                blend_rate_per_s=args.blend_rate,
            ),
            source_sha256=sha256_file(args.input),
            source_metadata=metadata,
        )
        report["provenance"] = {
            "input_path": str(args.input.resolve()),
            "input_sha256": sha256_file(args.input),
            "urdf_path": str(args.urdf.resolve()),
            "urdf_sha256": sha256_file(args.urdf),
        }
        _write_report(args.output, report)
    except (FaultReplayError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    tracking = report["tracking_during_tracking_reason"]
    transport = report["transport"]
    response = report["fault_response"]
    print(
        f"{report['status']}: p95 tracking={tracking['p95_abs_rad']:.6f} rad, "
        f"max={tracking['max_abs_rad']:.6f} rad; loss={transport['packets_dropped_by_loss']}, "
        f"out-of-order={transport['out_of_order_arrivals']}, "
        f"stale/hard={report['control']['reason_counts']['stale_source']}/"
        f"{report['control']['reason_counts']['hard_timeout']}, "
        f"source/control resets={response['source_hard_timeout_resets']}/"
        f"{response['control_gap_resets']}"
    )
    print(f"report: {args.output}")
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
