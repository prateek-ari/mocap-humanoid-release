#!/usr/bin/env python3
"""Play a suit recording (mocap_g1/suit_recorder.py) back to a UDP port, with its timing.

    python tools/xsens/play_suit_recording.py artifacts/xsens-live/suit/<capture id> \\
        --to 127.0.0.1:5555

The datagrams now come from this PC, so start the live tool with ``--source-ip 127.0.0.1``.
A truncated last record (a crash) is skipped; gaps are played as they are.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import socket
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from mocap_g1.suit_recorder import read_capture  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("capture", type=Path, help="the capture folder")
    parser.add_argument("--to", default="127.0.0.1:5555", help="HOST:PORT (default 127.0.0.1:5555)")
    parser.add_argument("--speed", type=float, default=1.0, help="2 plays twice as fast")
    args = parser.parse_args(argv)
    host, _, port = args.to.rpartition(":")
    if not host or not port.isdigit() or not args.speed > 0:
        parser.error("--to needs HOST:PORT and --speed must be positive")
    chunks = read_capture(args.capture)
    records = [record for chunk in chunks for record in chunk.records]
    for chunk in chunks:
        if chunk.error or chunk.tail_bytes:
            print(f"{chunk.name}: {chunk.error or f'{chunk.tail_bytes} bytes after the last record'}",
                  file=sys.stderr)
    if not records:
        print(f"No records in {args.capture}", file=sys.stderr)
        return 1
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    start, first = time.monotonic(), records[0].monotonic
    try:
        for record in records:
            delay = (record.monotonic - first) / args.speed - (time.monotonic() - start)
            if delay > 0:
                time.sleep(delay)
            sock.sendto(record.payload, (host, int(port)))
    except KeyboardInterrupt:
        return 130
    finally:
        sock.close()
    print(f"Played {len(records)} datagrams ({records[-1].monotonic - first:.1f} s) to {args.to}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
