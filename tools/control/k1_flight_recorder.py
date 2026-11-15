"""Read-only K1 flight recorder: every LowState (about 500 Hz) to compressed .npz chunks.

Per sample: time.monotonic() (same clock as the live bridge's "t"), time.time(), and for all
22 serial motors q, dq, ddq, tau_est, temperature, lost, plus the IMU (rpy, gyro, acc).
Never publishes. Stop with Ctrl-C or SIGTERM; the last chunk is flushed.
"""
import os, signal, sys, threading, time
import numpy as np
import booster_robotics_sdk_python as sdk

out = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/k1-flight")
net = sys.argv[2] if len(sys.argv) > 2 else "192.168.10.10"
os.makedirs(out, exist_ok=True)
lock, rows, stop = threading.Lock(), [], threading.Event()

def on_state(msg):
    t, w = time.monotonic(), time.time()
    m = msg.motor_state_serial
    imu = msg.imu_state
    row = [t, w]
    for x in m:
        row += [x.q, x.dq, x.ddq, x.tau_est, float(x.temperature), float(x.lost)]
    row += list(imu.rpy) + list(imu.gyro) + list(imu.acc)
    with lock:
        rows.append(row)

def flush():
    with lock:
        data, rows[:] = list(rows), []
    if data:
        a = np.asarray(data, dtype=np.float64)
        name = os.path.join(out, time.strftime("lowstate-%Y%m%d-%H%M%S.npz"))
        np.savez_compressed(name, data=a, columns=np.array(COLUMNS))
        print(f"{name}: {len(a)} samples", flush=True)

sdk.ChannelFactory.Instance().Init(0, net)
sub = sdk.B1LowStateSubscriber(on_state)
sub.InitChannel()
COLUMNS = ["t_monotonic", "t_wall"] + [f"m{i}_{f}" for i in range(22) for f in
           ("q", "dq", "ddq", "tau_est", "temperature", "lost")] + \
          [f"imu_{k}{j}" for k in ("rpy", "gyro", "acc") for j in "xyz"]
signal.signal(signal.SIGTERM, lambda *_: stop.set())
try:
    while not stop.wait(10.0):
        flush()
except KeyboardInterrupt:
    pass
flush()
