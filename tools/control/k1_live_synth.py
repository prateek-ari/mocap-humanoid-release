#!/usr/bin/env python3
"""Stream K1 live arm packets without the suit: a slow pattern or a K1 motion pickle.

For practice with tools/control/k1_live_bridge.py on the fake robot, and for tests.
The pattern starts at the practice robot's rest pose (arms hanging) and moves every
arm joint toward the middle of its range and back, so the bridge's coarse gate opens.

    python3 tools/control/k1_live_synth.py --port 47011 --seconds 60
    python3 tools/control/k1_live_synth.py --port 47011 --motion k1-preview.pkl --model K1_serial.xml
    python3 tools/control/k1_live_synth.py --port 47011 --seconds 20   # then it stops: a stream-loss drill

Pickles are trusted local input only: loading one runs code.  Exit status 0, or 2
for invalid input.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
import math
from pathlib import Path
import sys
import time

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from mocap_g1.io_utils import sha256_file  # noqa: E402
from mocap_g1.k1_bridge import ARM_MOTOR_INDEX  # noqa: E402
from mocap_g1.k1_live_packet import LivePacketSender  # noqa: E402
from mocap_g1.live_robots import K1_JOINTS, K1_UPPER_BODY_JOINTS as ARMS  # noqa: E402
from tools.control.k1_commission import LIMITS, PRACTICE_POSE  # noqa: E402

REST = {name: PRACTICE_POSE[K1_JOINTS.index(name)] for name in ARMS}
#: Without --model the bridge must run without one too (practice only).
NO_MODEL_SHA = "0" * 64


def pattern(amplitude: float, period_s: float) -> Callable[[float], list[float]]:
    """Each joint from rest toward its range middle by up to ``amplitude`` and back."""
    def frame(t: float) -> list[float]:
        weight = amplitude * (1.0 - math.cos(2.0 * math.pi * t / period_s)) / 2.0
        values = []
        for name in ARMS:
            middle = (LIMITS[name].lower + LIMITS[name].upper) / 2.0
            values.append(REST[name] + math.copysign(min(weight, abs(middle - REST[name])),
                                                     middle - REST[name]))
        return values
    return frame


def motion(path: Path) -> tuple[Callable[[float], list[float]], float]:
    from tools.control.k1_arm_bridge import load_motion

    dof, fps = load_motion(path, model_sha256=None, real=False)
    indices = list(ARM_MOTOR_INDEX.values())

    def frame(t: float) -> list[float]:
        return [float(value) for value in dof[min(len(dof) - 1, int(t * fps))][indices]]
    return frame, (len(dof) - 1) / fps


def stream(sender: LivePacketSender, frame: Callable[[float], Sequence[float]], *, rate_hz: float,
           seconds: float, epoch: int = 1, enabled: bool = True,
           clock: Callable[[], float] = time.monotonic,
           sleeper: Callable[[float], None] = time.sleep) -> int:
    """Send ``seconds`` of ``frame(t)`` at ``rate_hz``; returns the packets sent."""
    start = clock()
    count = 0
    while True:
        now = clock()
        if now - start >= seconds:
            return count
        sender.send(frame(now - start), suit_receive_s=now, counter=count + 1, generation=0,
                    epoch=epoch, enabled=enabled)
        count += 1
        sleeper(max(0.0, start + count / rate_hz - clock()))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", type=int, default=47011)
    parser.add_argument("--rate", type=float, default=60.0, help="packets per second")
    parser.add_argument("--seconds", type=float, default=60.0, help="then stop (stream loss)")
    parser.add_argument("--motion", type=Path, help="a K1 motion pickle instead of the pattern")
    parser.add_argument("--model", type=Path, help="the K1 MJCF the bridge uses (its hash)")
    parser.add_argument("--amplitude", type=float, default=0.5, help="pattern reach (rad)")
    parser.add_argument("--period", type=float, default=6.0, help="pattern period (s)")
    parser.add_argument("--epoch", type=int, default=1, help="calibration epoch (0 = uncalibrated)")
    parser.add_argument("--disabled", action="store_true", help="send tracking off")
    args = parser.parse_args(argv)
    if not (1 <= args.port <= 65535 and 1.0 <= args.rate <= 500.0 and args.seconds > 0
            and args.period > 0 and args.amplitude >= 0):
        parser.error("invalid --port, --rate, --seconds, --period or --amplitude")
    try:
        frame, length = motion(args.motion) if args.motion else (
            pattern(args.amplitude, args.period), math.inf)
    except (ValueError, OSError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    sender = LivePacketSender(("127.0.0.1", args.port), arm_indices=range(len(ARMS)),
                              model_sha256=sha256_file(args.model) if args.model else NO_MODEL_SHA)
    try:
        sent = stream(sender, frame, rate_hz=args.rate, seconds=min(args.seconds, length),
                      epoch=args.epoch, enabled=not args.disabled)
    except KeyboardInterrupt:
        sent = sender.sent
    finally:
        sender.close()
    print(f"Sent {sent} packets (session {sender.session_id}); stream stopped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
