#!/usr/bin/env python3
"""Listen for Xsens UDP packets and write a machine-readable acceptance report."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import signal
import socket
import sys
import time

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from mocap_g1.mxtp02 import (  # noqa: E402
    XSENS_JOINT_SEGMENT_PAIRS,
    XSENS_SEGMENTS,
    build_body_datagram,
    build_joint_angles_datagram,
)
from mocap_g1.xsens_stream import XsensStreamAcceptance  # noqa: E402


def _synthetic_positions() -> list[tuple[float, float, float]]:
    by_name = {
        "pelvis": (0.0, 0.0, 1.0),
        "l5": (0.0, 0.0, 1.12),
        "l3": (0.0, 0.0, 1.24),
        "t12": (0.0, 0.0, 1.36),
        "t8": (0.0, 0.0, 1.48),
        "neck": (0.0, 0.0, 1.65),
        "head": (0.0, 0.0, 1.82),
        "right_shoulder": (0.0, -0.18, 1.55),
        "right_upper_arm": (0.0, -0.43, 1.52),
        "right_forearm": (0.0, -0.68, 1.50),
        "right_hand": (0.0, -0.90, 1.48),
        "left_shoulder": (0.0, 0.18, 1.55),
        "left_upper_arm": (0.0, 0.43, 1.52),
        "left_forearm": (0.0, 0.68, 1.50),
        "left_hand": (0.0, 0.90, 1.48),
        "right_upper_leg": (0.0, -0.10, 0.82),
        "right_lower_leg": (0.0, -0.10, 0.43),
        "right_foot": (0.10, -0.10, 0.08),
        "right_toe": (0.28, -0.10, 0.05),
        "left_upper_leg": (0.0, 0.10, 0.82),
        "left_lower_leg": (0.0, 0.10, 0.43),
        "left_foot": (0.10, 0.10, 0.08),
        "left_toe": (0.28, 0.10, 0.05),
    }
    return [by_name[name] for name in XSENS_SEGMENTS]


def _run_self_test(acceptance: XsensStreamAcceptance) -> None:
    positions = _synthetic_positions()
    quaternions = [(1.0, 0.0, 0.0, 0.0)] * len(XSENS_SEGMENTS)
    rotations = [(0.0, 0.0, 0.0)] * len(XSENS_JOINT_SEGMENT_PAIRS)
    frames = max(acceptance.minimum_body_frames, 60)
    start = 1000.0
    for sample in range(frames):
        received_at = start + sample / 60.0
        time_code_ms = round(sample * 1000 / 60)
        acceptance.observe(
            build_body_datagram(sample, time_code_ms, positions, quaternions),
            received_at=received_at,
            source="127.0.0.1:self-test",
        )
        acceptance.observe(
            build_joint_angles_datagram(sample, time_code_ms, rotations),
            received_at=received_at,
            source="127.0.0.1:self-test",
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--bind", default="0.0.0.0", help="local address to listen on")
    parser.add_argument("--port", type=int, default=9763)
    parser.add_argument(
        "--expected-source",
        help="accept packets only from this sender IPv4 address and fail if others are observed",
    )
    parser.add_argument("--duration", type=float, default=10.0, help="capture duration in seconds")
    parser.add_argument("--minimum-body-frames", type=int, default=30)
    parser.add_argument("--minimum-rate", type=float, default=50.0, help="minimum MXTP02 rate in Hz")
    parser.add_argument("--maximum-loss", type=float, default=0.02, help="maximum packet loss fraction")
    parser.add_argument(
        "--require-joint-angles",
        action="store_true",
        help="require paired MXTP20 packets for the ROS 2 path",
    )
    parser.add_argument("--output", type=Path, help="write the JSON report to this path")
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="grade deterministic synthetic packets instead of opening a UDP socket",
    )
    args = parser.parse_args()
    if hasattr(signal, "SIGBREAK"):  # Windows Ctrl+Break -> same path as Ctrl+C
        signal.signal(signal.SIGBREAK, signal.default_int_handler)

    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.self_test and args.expected_source:
        parser.error("--expected-source cannot be combined with --self-test")

    acceptance = XsensStreamAcceptance(
        minimum_body_frames=args.minimum_body_frames,
        minimum_rate_hz=args.minimum_rate,
        maximum_loss_fraction=args.maximum_loss,
        require_joint_angles=args.require_joint_angles,
    )

    if args.self_test:
        _run_self_test(acceptance)
        listener = {"mode": "self-test"}
    else:
        listener = {
            "mode": "udp",
            "bind": args.bind,
            "port": args.port,
            "duration_s": args.duration,
            "expected_source": args.expected_source,
        }
        unexpected_sources: set[str] = set()
        unexpected_datagrams = 0
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind((args.bind, args.port))
            sock.settimeout(0.25)
            print(
                f"Listening for Xsens UDP on {args.bind}:{args.port} for "
                f"{args.duration:g} s...",
                file=sys.stderr,
            )
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline:
                try:
                    data, source = sock.recvfrom(65535)
                except socket.timeout:
                    continue
                if args.expected_source and source[0] != args.expected_source:
                    unexpected_sources.add(source[0])
                    unexpected_datagrams += 1
                    continue
                acceptance.observe(
                    data,
                    received_at=time.monotonic(),
                    source=f"{source[0]}:{source[1]}",
                )
        finally:
            sock.close()

    report = acceptance.report()
    report["listener"] = listener
    if not args.self_test:
        report["listener"]["unexpected_datagrams"] = unexpected_datagrams
        report["listener"]["unexpected_sources"] = sorted(unexpected_sources)
        if unexpected_sources:
            report["failures"].append(
                "received UDP datagrams from unexpected source(s): "
                + ", ".join(sorted(unexpected_sources))
            )
            report["verdict"] = "FAIL"
    rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0 if report["verdict"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
