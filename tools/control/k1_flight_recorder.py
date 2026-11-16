"""Read-only K1 flight recorder: every LowState (about 500 Hz) to compressed .npz chunks.

Usage: ``python tools/control/k1_flight_recorder.py [out_dir] [network_interface] [dds_domain]``.
The K1 window starts it for real-robot live runs, into the session's ``robot-data/`` folder.
Per sample: time.monotonic() (same clock as the live bridge's "t"), time.time(), and for all
22 serial motors q, dq, ddq, tau_est, temperature, lost, plus the IMU (rpy, gyro, acc).
Never publishes. Stop with Ctrl-C or SIGTERM; the last chunk is flushed.
"""

from __future__ import annotations

import importlib
import os
from pathlib import Path
import signal
import sys
import threading
import time
from typing import Any

import numpy as np

MOTORS = 22
COLUMNS = (["t_monotonic", "t_wall"]
           + [f"m{i}_{f}" for i in range(MOTORS)
              for f in ("q", "dq", "ddq", "tau_est", "temperature", "lost")]
           + [f"imu_{k}{j}" for k in ("rpy", "gyro", "acc") for j in "xyz"])
FLUSH_S = 10.0
DEFAULT_OUT = "~/k1-flight"
DEFAULT_NET = "192.168.10.10"


class FlightRecorder:
    def __init__(self, out: Path) -> None:
        self.out = out
        self._lock = threading.Lock()
        self._rows: list[list[float]] = []

    def on_state(self, msg: Any) -> None:
        row = [time.monotonic(), time.time()]
        for x in msg.motor_state_serial:
            row += [x.q, x.dq, x.ddq, x.tau_est, float(x.temperature), float(x.lost)]
        imu = msg.imu_state
        row += list(imu.rpy) + list(imu.gyro) + list(imu.acc)
        with self._lock:
            self._rows.append(row)

    def flush(self) -> Path | None:
        with self._lock:
            data, self._rows = self._rows, []
        if not data:
            return None
        stem = time.strftime("lowstate-%Y%m%d-%H%M%S")
        path, n = self.out / f"{stem}.npz", 1
        while path.exists():  # two flushes in one second must not overwrite
            path, n = self.out / f"{stem}-{n}.npz", n + 1
        np.savez_compressed(path, data=np.asarray(data, dtype=np.float64), columns=np.array(COLUMNS))
        print(f"{path}: {len(data)} samples", flush=True)
        return path


def subscribe(sdk: Any, recorder: FlightRecorder, net: str, domain: int = 0) -> Any:
    sdk.ChannelFactory.Instance().Init(domain, net)
    subscriber = sdk.B1LowStateSubscriber(recorder.on_state)
    subscriber.InitChannel()
    return subscriber


def main(argv: list[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    out = Path(os.path.expanduser(args[0] if args else DEFAULT_OUT))
    out.mkdir(parents=True, exist_ok=True)
    recorder = FlightRecorder(out)
    subscribe(importlib.import_module("booster_robotics_sdk_python"), recorder,
              args[1] if len(args) > 1 and args[1] else DEFAULT_NET,
              int(args[2]) if len(args) > 2 else 0)
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    try:
        while not stop.wait(FLUSH_S):
            recorder.flush()
    except KeyboardInterrupt:
        pass
    recorder.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
