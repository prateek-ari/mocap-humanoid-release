#!/usr/bin/env python3
"""Dependency-free MXTP02/MXTP20 codec and UDP loopback smoke test."""

from __future__ import annotations

import argparse
from pathlib import Path
import socket
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from mocap_g1.mxtp02 import (  # noqa: E402
    XSENS_JOINT_SEGMENT_PAIRS,
    XSENS_SEGMENTS,
    build_body_datagram,
    build_joint_angles_datagram,
    parse_body_datagram,
    parse_joint_angles_datagram,
)


def make_frame():
    positions = [(index / 100.0, -index / 200.0, 0.9 + index / 1000.0)
                 for index in range(len(XSENS_SEGMENTS))]
    quaternions = [(1.0, 0.0, 0.0, 0.0) for _ in XSENS_SEGMENTS]
    return positions, quaternions


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--udp-loopback",
        action="store_true",
        help="also send the generated packet through a local UDP socket",
    )
    args = parser.parse_args()

    positions, quaternions = make_frame()
    packet = build_body_datagram(0x1_0000_0007, 0x1_0000_0009, positions, quaternions)
    joint_packet = build_joint_angles_datagram(
        0x1_0000_0007,
        0x1_0000_0009,
        [(0.0, 0.0, 0.0)] * len(XSENS_JOINT_SEGMENT_PAIRS),
    )
    decoded = parse_body_datagram(packet)
    decoded_joints = parse_joint_angles_datagram(joint_packet)
    assert len(packet) == 760
    assert decoded.sample_counter == 7
    assert decoded.time_code_ms == 9
    assert tuple(decoded.segments) == XSENS_SEGMENTS
    for index, name in enumerate(XSENS_SEGMENTS):
        pose = decoded.segments[name]
        assert max(abs(a - b) for a, b in zip(pose.position, positions[index])) < 1e-6
        assert pose.quaternion_wxyz == quaternions[index]
    assert len(joint_packet) == 464
    assert tuple(
        (joint.parent_segment_id, joint.child_segment_id) for joint in decoded_joints.joints
    ) == XSENS_JOINT_SEGMENT_PAIRS

    if args.udp_loopback:
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            receiver.bind(("127.0.0.1", 0))
            receiver.settimeout(2.0)
            sender.sendto(packet, receiver.getsockname())
            received, _ = receiver.recvfrom(2048)
            assert parse_body_datagram(received) == decoded
            sender.sendto(joint_packet, receiver.getsockname())
            received, _ = receiver.recvfrom(2048)
            assert parse_joint_angles_datagram(received) == decoded_joints
        finally:
            sender.close()
            receiver.close()

    mode = "codec + UDP loopback" if args.udp_loopback else "codec"
    print(
        f"PASS: MXTP02/MXTP20 {mode}; {len(decoded.segments)} segments, "
        f"{len(decoded_joints.joints)} joints"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
