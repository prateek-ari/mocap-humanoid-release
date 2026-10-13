#!/usr/bin/env python3
"""Exercise the G1 arm/waist safety envelope with a simulated publisher.

SUPERSEDED by ``tools/control/g1_arm_bridge.py`` for anything robot-facing.

This script predates any robot-facing code: it is a pure in-process safety-
envelope exerciser that never constructs a ``LowCmd_``, never computes a CRC and
never opens a socket, and it resets the envelope from *midpoint defaults* rather
than from measured robot state.  It is retained because that self-contained
envelope stress test (blend ramp, position/velocity/acceleration limiting, stale
disengagement, hard timeout) is still worth running on its own.

Use ``tools/control/g1_arm_bridge.py`` instead when you want the real thing:
``rt/arm_sdk`` LowCmd construction against the Unitree motor index map, the
arm_sdk weight channel, CRC stamping, and ``envelope.reset`` driven by measured
``LowState_`` encoders -- with ``--transport fake`` by default and
``--transport dds`` for hardware.  See ``mocap_g1/arm_bridge.py``.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from mocap_g1.safety import ArmSafetyEnvelope, SafetyConfig  # noqa: E402
from mocap_g1.schema import G1_ARM_SDK_JOINTS, load_official_joint_limits  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument(
        "--inject-stale-at",
        type=float,
        default=2.0,
        help="simulation time at which source timestamps stop advancing",
    )
    parser.add_argument("--jsonl", type=Path, help="write every mock command as JSONL")
    args = parser.parse_args()
    if args.duration <= 0 or args.rate <= 0:
        parser.error("duration and rate must be positive")

    urdf = REPO_ROOT / "eval/assets/g1_29dof_official.urdf"
    limits = load_official_joint_limits(urdf)
    config = SafetyConfig()
    envelope = ArmSafetyEnvelope(limits, config)
    initial = {
        name: (limits[name].lower + limits[name].upper) / 2.0
        for name in G1_ARM_SDK_JOINTS
    }
    envelope.reset(initial, now=0.0)

    handle = args.jsonl.open("w", encoding="utf-8") if args.jsonl else None
    stale_seen = hard_timeout_seen = False
    max_velocity_ratio = 0.0
    try:
        steps = int(args.duration * args.rate)
        dt = 1.0 / args.rate
        frozen_source_time = args.inject_stale_at
        for index in range(1, steps + 1):
            now = index * dt
            source_time = now if now < args.inject_stale_at else frozen_source_time
            target = {
                name: initial[name] + 0.35 * math.sin(2.0 * math.pi * 0.25 * now + offset * 0.1)
                for offset, name in enumerate(G1_ARM_SDK_JOINTS)
            }
            command = envelope.step(
                target,
                source_timestamp=source_time,
                now=now,
                enable=True,
            )
            stale_seen |= command.reason == "stale_source"
            hard_timeout_seen |= command.reason == "hard_timeout"
            for name, velocity in command.velocities.items():
                allowed = limits[name].velocity * config.velocity_limit_fraction
                max_velocity_ratio = max(max_velocity_ratio, abs(velocity) / allowed)
            if handle:
                handle.write(json.dumps({
                    "time_s": now,
                    "source_age_s": command.source_age_s,
                    "reason": command.reason,
                    "blend_weight": command.blend_weight,
                    "engaged": command.engaged,
                    "clamped_joints": command.clamped_joints,
                    "positions": command.positions,
                }, sort_keys=True) + "\n")
    finally:
        if handle:
            handle.close()

    if not stale_seen or not hard_timeout_seen:
        print("FAIL: injected stale stream did not exercise both disengagement stages", file=sys.stderr)
        return 1
    if max_velocity_ratio > 1.000001:
        print(f"FAIL: velocity envelope exceeded ({max_velocity_ratio:.6f}x)", file=sys.stderr)
        return 1
    print(
        "PASS: mock arm bridge exercised selection, blend ramp, official position/velocity "
        f"limits, acceleration limiting, stale disengagement, and hard timeout; "
        f"max velocity ratio={max_velocity_ratio:.3f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
