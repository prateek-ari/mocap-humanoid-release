#!/usr/bin/env python3
"""
Profile GMR's live Xsens ingest path against a synthetic MXTP02 stream.

Consumes UDP MXTP02 datagrams (from tools/mxtp02_replayer.py, or a real MVN
network streamer) exactly the way GMR's scripts/xsens_live_streaming.py does:
same jiminghe xsens_mvn_robot wheel API, same XsensToGMR adapter, same GMR
retarget call (src_human="xsens_mvn", robot unitree_g1, solver=daqp,
damping=1.0, use_velocity_limit=True). The MuJoCo viewer step is intentionally
skipped (fragile headless; it also rate-limits, which would mask the numbers we
want). Measures:

  - achieved retarget FPS (retargeted frames / wall time)
  - per-stage latency: parse (wheel read + adapter dict + yaw normalization),
    IK (GMR retarget), live filter and command upsampler (mocap_g1.live_filter),
    mean / p50 / p95 / max in ms
  - end-to-end synthetic suit -> command latency: the frame's age when we start
    processing it, plus the compute time through the conditioning stages, plus
    the configuration's algorithmic lag (filter phase lag + upsampler
    interpolation lag, measured offline). Use --no-filter for the pre-filter
    baseline.
  - receive staleness over time: consumer wall-clock minus the sender wall-clock
    stamp the replayer puts in the MXTP time-code field (same-host clocks), i.e.
    the age of the frame when we start processing it — this is the "queue depth
    in time". Also sample-counter gaps (frames the sender emitted that we never
    processed).

Ingest modes (--ingest):
  wheel  (default) — the wheel + XsensToGMR adapter, as xsens_live_streaming.py.
           NOTE (verified): the wheel's C++ acquisition thread drains the socket
           continuously and exposes only the newest sample, so it already has
           keep-newest / drop-stale semantics.
  naive  — pure-Python ingest that processes EVERY datagram in arrival order
           (one recv per loop iteration; the kernel socket buffer is the queue).
           This is the pathological baseline: when IK is slower than the send
           rate, staleness grows without bound until the buffer overflows.
  newest — pure-Python drop-stale ingest: drain the socket each iteration and
           keep only the newest datagram. The fix for the naive mode.

The pure-Python modes vendor a minimal MXTP02 parser (mirrors the replayer's
packer) and reuse the adapter's segment-name mapping + yaw normalization so the
human_frame fed to GMR is identical across modes.

Requires the xsens_mvn_robot wheel (binary, needs GLIBCXX_3.4.32). On Ubuntu
22.04 the system libstdc++ is too old; this script auto-re-execs itself with
LD_LIBRARY_PATH pointing at tools/vendor/libstdcxx13/lib (GCC 13.2 libstdc++
extracted from conda-forge libstdcxx-ng, no sudo needed).

PROCESS-ISOLATION WARNING: the replayer (or any datagram sender) MUST run as a
separate OS process (second shell, `&`, or subprocess.Popen) — never as a
thread inside this process. XsensWrapper.init() (called via
XsensToGMR.initialize() in WheelIngest) blocks while holding the GIL until a
datagram arrives, so an in-process sender thread is never scheduled and both
sides deadlock (60 s hang reproduced 2026-08-03).

Usage (two shells, or run the replayer with `&`):
  VENVPY tools/mxtp02_replayer.py --bvh WS/data/lafan1/dance1_subject2.bvh \
      --rate 60 --loop --quiet
  VENVPY tools/profile_live_ingest.py --duration 30
"""

import argparse
import json
import os
import socket
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))
from mocap_g1.mxtp02 import ProtocolError, parse_body_datagram  # noqa: E402
from mocap_g1.live_filter import (  # noqa: E402
    DEFAULT_COMMAND_RATE_HZ,
    DEFAULT_SOURCE_RATE_HZ,
    FILTER_KINDS,
    CommandUpsampler,
    CriticallyDampedConfig,
    LiveFilterError,
    OneEuroConfig,
    UpsamplerConfig,
    format_configuration_report,
    make_filter,
    measure_step_lag,
    upsampler_lag_ms,
)

VENDOR_LIBSTDCXX = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "vendor", "libstdcxx13", "lib")


def _ensure_wheel_importable():
    """Import the wheel; on GLIBCXX failure re-exec with vendored libstdc++."""
    try:
        import xsens_mvn_robot  # noqa: F401
        return
    except ImportError as e:
        if "GLIBCXX" not in str(e):
            raise
        if os.environ.get("_MXTP_REEXEC") == "1":
            print(f"Re-exec with vendored libstdc++ still failed: {e}",
                  file=sys.stderr)
            raise
        env = dict(os.environ)
        env["LD_LIBRARY_PATH"] = VENDOR_LIBSTDCXX + (
            ":" + env["LD_LIBRARY_PATH"] if env.get("LD_LIBRARY_PATH") else "")
        env["_MXTP_REEXEC"] = "1"
        os.execve(sys.executable, [sys.executable] + sys.argv, env)


import numpy as np  # noqa: E402


def parse_mxtp02(data):
    """-> (sample_counter, time_code_ms, {segment_name: (pos3, quat_wxyz4)}) or None."""
    try:
        parsed = parse_body_datagram(data, strict=False)
    except ProtocolError:
        return None
    segments = {
        name: (
            np.asarray(pose.position, dtype=np.float64),
            np.asarray(pose.quaternion_wxyz, dtype=np.float64),
        )
        for name, pose in parsed.segments.items()
    }
    return parsed.sample_counter, parsed.time_code_ms, segments


class PyIngest:
    """Pure-Python UDP ingest producing the same human_frame as XsensToGMR."""

    def __init__(self, port, mode):
        from scipy.spatial.transform import Rotation
        from general_motion_retargeting.utils.xsens_vendor.xsens_to_gmr_adapter import (
            XsensToGMR)
        self.rotation_class = Rotation
        self.mapping = XsensToGMR.XSENS_TO_GMR_MAPPING  # xsens name -> GMR name
        self.mode = mode  # "naive" | "newest"
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Big kernel buffer so "naive" really queues instead of dropping early.
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(1.0)
        self.rcvbuf = self.sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        self.last_counter = None
        self.yaw_inv = None

    def _to_human_frame(self, segments):
        frame = {}
        for xsens_name, gmr_name in self.mapping.items():
            if xsens_name in segments:
                pos, quat = segments[xsens_name]
                frame[gmr_name] = (pos, quat / np.linalg.norm(quat))
        # initial-yaw normalization, same as XsensToGMR._apply_yaw_normalization
        if "Pelvis" in frame:
            if self.yaw_inv is None:
                yaw = self.rotation_class.from_quat(
                    frame["Pelvis"][1], scalar_first=True
                ).as_euler("ZYX")[0]
                self.yaw_inv = self.rotation_class.from_euler("Z", -yaw)
            out = {}
            for name, (pos, quat) in frame.items():
                out[name] = (self.yaw_inv.apply(pos),
                             (self.yaw_inv *
                              self.rotation_class.from_quat(quat, scalar_first=True)
                              ).as_quat(scalar_first=True))
            frame = out
        return frame

    def get(self):
        """-> (human_frame, time_code_ms, counter_gap) or None on timeout."""
        try:
            data = self.sock.recv(65535)
        except socket.timeout:
            return None
        if self.mode == "newest":
            # drain: keep only the newest datagram currently queued
            self.sock.setblocking(False)
            try:
                while True:
                    try:
                        data = self.sock.recv(65535)
                    except BlockingIOError:
                        break
            finally:
                self.sock.setblocking(True)
                self.sock.settimeout(1.0)
        parsed = parse_mxtp02(data)
        if parsed is None:
            return None
        counter, time_code, segments = parsed
        gap = 1 if self.last_counter is None else counter - self.last_counter
        self.last_counter = counter
        return self._to_human_frame(segments), time_code, gap

    def close(self):
        self.sock.close()


class WheelIngest:
    """The exact xsens_live_streaming.py ingest: wheel + XsensToGMR adapter.

    WARNING: initialize() -> XsensWrapper.init() blocks HOLDING THE GIL until
    the first datagram arrives. The sender must therefore be a separate OS
    process; an in-process sender thread deadlocks (see module docstring).
    """

    def __init__(self, port):
        _ensure_wheel_importable()
        from general_motion_retargeting.utils.xsens_vendor.xsens_to_gmr_adapter import (
            XsensToGMR)
        self.adapter = XsensToGMR(port=port, verbose=False)
        if not self.adapter.initialize():
            raise RuntimeError(
                "XsensToGMR.initialize() failed — is the replayer streaming?")
        self.adapter.start()
        self.prev_counter = None

    def get(self):
        """-> (human_frame, time_code_ms, counter_gap) or None if no new frame."""
        frame = self.adapter.get_human_frame()
        if frame is None:
            return None
        counter = self.adapter.last_sample_counter
        gap = 1 if self.prev_counter is None else counter - self.prev_counter
        self.prev_counter = counter
        # get_frame_time() returns the u32 wire time code as a SIGNED int32
        # (values past 2^31 come back negative) — mask to recover the u32.
        time_code = int(self.adapter.device.get_frame_time()) & 0xFFFFFFFF
        return frame, time_code, gap

    def close(self):
        self.adapter.stop()


def staleness_ms(time_code):
    """Age of frame vs local wall clock; replayer stamps time.time()*1000."""
    now = int(time.time() * 1000) & 0xFFFFFFFF
    d = (now - (time_code & 0xFFFFFFFF)) & 0xFFFFFFFF
    if d > 1 << 31:                      # clock skew / wrapped negative
        d -= 1 << 32
    return float(d)


def pct(a, q):
    return float(np.percentile(a, q)) if len(a) else float("nan")


def main():
    ap = argparse.ArgumentParser(
        description="Profile GMR live ingest against an MXTP02 stream",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--port", type=int, default=9763)
    ap.add_argument("--robot", default="unitree_g1")
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--human-height", type=float, default=None)
    ap.add_argument("--ingest", choices=["wheel", "naive", "newest"],
                    default="wheel")
    ap.add_argument("--json", default=None, help="Write results JSON here")
    ap.add_argument("--filter", action=argparse.BooleanOptionalAction, default=True,
                    help="time the causal live filter + command upsampler "
                         "(mocap_g1.live_filter) after each retarget")
    ap.add_argument("--filter-type", choices=FILTER_KINDS,
                    default=OneEuroConfig.KIND)
    ap.add_argument("--filter-min-cutoff", type=float,
                    default=OneEuroConfig().min_cutoff_hz, metavar="HZ")
    ap.add_argument("--filter-beta", type=float, default=OneEuroConfig().beta)
    ap.add_argument("--filter-derivative-cutoff", type=float,
                    default=OneEuroConfig().derivative_cutoff_hz, metavar="HZ")
    ap.add_argument("--filter-cutoff", type=float,
                    default=CriticallyDampedConfig().cutoff_hz, metavar="HZ")
    ap.add_argument("--command-rate", type=float, default=DEFAULT_COMMAND_RATE_HZ,
                    metavar="HZ", help="SDK-facing upsampler command rate")
    ap.add_argument("--command-interpolation", choices=("interpolate", "hold"),
                    default="interpolate")
    ap.add_argument("--source-rate", type=float, default=DEFAULT_SOURCE_RATE_HZ,
                    metavar="HZ", help="nominal retarget reference rate")
    args = ap.parse_args()

    filter_config = upsampler_config = None
    algorithmic_lag_ms = 0.0
    if args.filter:
        if args.filter_type == OneEuroConfig.KIND:
            filter_config = OneEuroConfig(
                min_cutoff_hz=args.filter_min_cutoff,
                beta=args.filter_beta,
                derivative_cutoff_hz=args.filter_derivative_cutoff)
        else:
            filter_config = CriticallyDampedConfig(cutoff_hz=args.filter_cutoff)
        upsampler_config = UpsamplerConfig(
            command_rate_hz=args.command_rate,
            mode=args.command_interpolation,
            source_rate_hz=args.source_rate,
            max_segment_s=2.0 / args.source_rate)
        try:
            filter_config.validated()
            upsampler_config.validated()
        except LiveFilterError as e:
            ap.error(str(e))
        algorithmic_lag_ms = (
            measure_step_lag(filter_config,
                             sample_rate_hz=args.source_rate).lag_50_ms
            + upsampler_lag_ms(upsampler_config))
    print(format_configuration_report(filter_config, upsampler_config,
                                      sample_rate_hz=args.source_rate))

    from general_motion_retargeting import GeneralMotionRetargeting as GMR

    print(f"[profile] ingest={args.ingest} robot={args.robot} "
          f"port={args.port} duration={args.duration}s")

    if args.ingest == "wheel":
        ingest = WheelIngest(args.port)
    else:
        ingest = PyIngest(args.port, args.ingest)

    retargeter = GMR(
        src_human="xsens_mvn",
        tgt_robot=args.robot,
        actual_human_height=args.human_height,
        solver="daqp",
        damping=1.0,
        verbose=False,
        use_velocity_limit=True,
    )

    parse_ms, ik_ms, stale_ms, gaps = [], [], [], []
    filter_ms, upsample_ms, e2e_ms = [], [], []
    t_frames = []          # (t_rel, stale, gap) for the over-time series
    polls_empty = 0
    retargeted = 0
    ik_failures = 0
    conditioning_failures = 0
    commands_emitted = 0
    first_ik_ms = None
    joint_filter = upsampler = None

    t0 = time.perf_counter()
    t_end = t0  # updated each loop; excludes ingest.close() teardown time
    try:
        while time.perf_counter() - t0 < args.duration:
            tp0 = time.perf_counter()
            got = ingest.get()
            tp1 = time.perf_counter()
            if got is None:
                polls_empty += 1
                time.sleep(0.001)  # same idle backoff as xsens_live_streaming.py
                continue
            arrival = time.monotonic()   # conditioning stages need a monotonic clock
            human_frame, time_code, gap = got
            stale = staleness_ms(time_code)

            ti0 = time.perf_counter()
            try:
                qpos = np.asarray(
                    retargeter.retarget(human_frame, offset_to_ground=False),
                    dtype=np.float64)
            except Exception as e:
                ik_failures += 1
                if ik_failures <= 3:
                    print(f"[profile] retarget failed: {e}")
                continue
            ti1 = time.perf_counter()

            # Live conditioning, in the same order as tools/xsens/live_g1_sim.py:
            # filter -> upsampler -> (safety envelope, not exercised here).
            joints = qpos[7:]
            if filter_config is not None and joint_filter is None:
                joint_filter = make_filter(filter_config, len(joints))
                upsampler = CommandUpsampler(upsampler_config, len(joints))
            tf0 = time.perf_counter()
            try:
                if joint_filter is not None:
                    joints = joint_filter.step(joints, now=time.monotonic()).values
                tf1 = time.perf_counter()
                if upsampler is not None:
                    upsampler.push(joints, now=time.monotonic(),
                                   source_timestamp=arrival)
                    commands_emitted += len(upsampler.pump(time.monotonic()))
            except LiveFilterError as e:
                conditioning_failures += 1
                if conditioning_failures <= 3:
                    print(f"[profile] live conditioning failed: {e}")
                continue
            tu1 = time.perf_counter()

            if first_ik_ms is None:
                first_ik_ms = (ti1 - ti0) * 1e3
            parse_ms.append((tp1 - tp0) * 1e3)
            ik_ms.append((ti1 - ti0) * 1e3)
            filter_ms.append((tf1 - tf0) * 1e3)
            upsample_ms.append((tu1 - tf1) * 1e3)
            # Age when we picked the frame up, plus everything we then spend on
            # it, plus the lag the configuration adds by design.
            e2e_ms.append(stale + (tu1 - tp1) * 1e3 + algorithmic_lag_ms)
            stale_ms.append(stale)
            gaps.append(gap)
            t_frames.append((time.perf_counter() - t0, stale, gap))
            retargeted += 1
            t_end = time.perf_counter()
    except KeyboardInterrupt:
        print("[profile] interrupted")
    finally:
        ingest.close()

    elapsed = t_end - t0
    fps = retargeted / elapsed if elapsed > 0 else 0.0
    parse = np.array(parse_ms)
    ik = np.array(ik_ms)
    stale = np.array(stale_ms)
    filt = np.array(filter_ms)
    ups = np.array(upsample_ms)
    e2e = np.array(e2e_ms)
    gaps_a = np.array(gaps, dtype=float)

    def row(name, a):
        return (f"  {name:<22s} mean {np.mean(a):7.2f}  p50 {pct(a, 50):7.2f}  "
                f"p95 {pct(a, 95):7.2f}  max {np.max(a):7.2f}" if len(a)
                else f"  {name:<22s} (no data)")

    print()
    print(f"=== results (ingest={args.ingest}) ===")
    print(f"  elapsed                {elapsed:.1f} s")
    print(f"  retargeted frames      {retargeted}")
    print(f"  achieved retarget FPS  {fps:.1f}")
    print(f"  empty polls            {polls_empty}")
    print(f"  ik failures            {ik_failures}")
    print(f"  conditioning failures  {conditioning_failures}")
    print(f"  first IK call          "
          f"{first_ik_ms:.1f} ms (warm-up, included in stats)"
          if first_ik_ms is not None else "  first IK call          n/a")
    print(row("parse ms", parse))
    print(row("ik ms", ik))
    print(row("filter ms", filt))
    print(row("upsample ms", ups))
    print(row("staleness ms", stale))
    print(row("suit->command e2e ms", e2e))
    print(f"  algorithmic lag        {algorithmic_lag_ms:.1f} ms "
          "(filter phase lag + upsampler interpolation lag, included in e2e)")
    if upsampler is not None:
        print(f"  commands emitted       {commands_emitted}")
        print("  " + upsampler.format_stats())
        print("  (the upsampler is pumped once per retargeted frame here, so "
              "emit_late reflects this loop, not the command schedule)")
    if len(gaps_a):
        print(f"  counter gap            mean {np.mean(gaps_a):5.2f}  "
              f"max {int(np.max(gaps_a))}  "
              f"(gap>1 on {int(np.sum(gaps_a > 1))}/{len(gaps_a)} frames -> "
              f"sender frames skipped: {int(np.sum(gaps_a) - len(gaps_a))})")
    # staleness over time, 5 s buckets — shows queue growth in naive mode
    if t_frames:
        print("  staleness over time (5 s buckets, mean ms):")
        tf = np.array([x[0] for x in t_frames])
        sf = np.array([x[1] for x in t_frames])
        for b0 in np.arange(0, elapsed, 5.0):
            m = (tf >= b0) & (tf < b0 + 5.0)
            if m.any():
                print(f"    [{b0:4.0f}-{min(b0 + 5.0, elapsed):4.0f}s] "
                      f"{np.mean(sf[m]):8.1f}")

    if args.json:
        out = {
            "ingest": args.ingest, "robot": args.robot, "elapsed_s": elapsed,
            "retargeted": retargeted, "fps": fps, "empty_polls": polls_empty,
            "ik_failures": ik_failures, "first_ik_ms": first_ik_ms,
            "parse_ms": {"mean": float(np.mean(parse)) if len(parse) else None,
                         "p50": pct(parse, 50), "p95": pct(parse, 95),
                         "max": float(np.max(parse)) if len(parse) else None},
            "ik_ms": {"mean": float(np.mean(ik)) if len(ik) else None,
                      "p50": pct(ik, 50), "p95": pct(ik, 95),
                      "max": float(np.max(ik)) if len(ik) else None},
            "staleness_ms": {"mean": float(np.mean(stale)) if len(stale) else None,
                             "p50": pct(stale, 50), "p95": pct(stale, 95),
                             "max": float(np.max(stale)) if len(stale) else None},
            "counter_gap": {"mean": float(np.mean(gaps_a)) if len(gaps_a) else None,
                            "max": int(np.max(gaps_a)) if len(gaps_a) else None,
                            "skipped": int(np.sum(gaps_a) - len(gaps_a)) if len(gaps_a) else None},
            "filter_ms": {"mean": float(np.mean(filt)) if len(filt) else None,
                          "p50": pct(filt, 50), "p95": pct(filt, 95),
                          "max": float(np.max(filt)) if len(filt) else None},
            "upsample_ms": {"mean": float(np.mean(ups)) if len(ups) else None,
                            "p50": pct(ups, 50), "p95": pct(ups, 95),
                            "max": float(np.max(ups)) if len(ups) else None},
            "end_to_end_ms": {"mean": float(np.mean(e2e)) if len(e2e) else None,
                              "p50": pct(e2e, 50), "p95": pct(e2e, 95),
                              "max": float(np.max(e2e)) if len(e2e) else None},
            "conditioning": {
                "filter": filter_config.describe() if filter_config else None,
                "upsampler": upsampler_config.describe() if upsampler_config else None,
                "algorithmic_lag_ms": algorithmic_lag_ms,
                "conditioning_failures": conditioning_failures,
                "commands_emitted": commands_emitted,
            },
        }
        with open(args.json, "w") as f:
            json.dump(out, f, indent=2)
        print(f"[profile] wrote {args.json}")


if __name__ == "__main__":
    main()
