#!/usr/bin/env python3
"""
Synthetic Xsens MVN network-streamer replayer (MXTP02 pose datagrams).

Loads a LAFAN1 BVH clip via GMR's loader, converts the skeleton to the 23-segment
Xsens MVN body model, and streams MXTP02 datagrams over UDP at a configurable
rate. This exercises GMR's live ingest path (scripts/xsens_live_streaming.py +
the jiminghe xsens_mvn_robot wheel) without any Xsens hardware or license.

Wire format (MVN real-time network streaming protocol spec MV0305P.N):
  - UDP, default port 9763. All integers big-endian, floats 32-bit big-endian.
  - 24-byte header: 6s ID "MXTP02" | u32 sample counter | u8 datagram counter
    (0x80 = single datagram of sample) | u8 number of items | u32 time code (ms)
    | u8 character ID | u8 body segments (23) | u8 props (0) | u8 finger
    segments (0) | 2 bytes reserved | u16 payload size.
  - MXTP02 segment record (32 B): u32 segment ID (= index + 1) | f32 x,y,z
    position, Z-up right-handed | f32 q1..q4 quaternion (w,x,y,z).
  - Segment order: Pelvis, L5, L3, T12, T8, Neck, Head, R shoulder/upper-arm/
    forearm/hand, L arm, R upper-leg/lower-leg/foot/toe, L leg.

UNITS NOTE (verified empirically 2026-08-03): the spec/dossier says MXTP
positions are in cm, but the jiminghe xsens_mvn_robot wheel passes position
floats through UNCHANGED and documents its output as meters (GMR's adapter also
assumes meters). Since the wheel demonstrably works against real MVN streams,
we send METERS by default so the downstream GMR path sees correct scale.
Use --units cm to emit spec-literal centimeters instead. Re-verify against a
real MVN stream on day 1 of any trial license.

ROS DRIVER NOTE: the official ROS 2 stream driver builds its human model from
both MXTP02 poses and MXTP20 joint angles. ``--with-joint-angles`` emits the
canonical 22-joint body tree with zero rotations beside every pose. This is a
structural, hardware-free driver fixture; it is not a biomechanics generator.

TIME CODE FIELD: we stamp wall-clock milliseconds (time.time()*1000 mod 2^32)
so a consumer on the same host can compute end-to-end wire+parse latency by
comparing against its own clock. On the wire this is an UNSIGNED u32, but the
jiminghe wheel's get_frame_time() returns it as a SIGNED int32 — values past
2^31 come back negative. Readers MUST mask with `& 0xFFFFFFFF` before storing
or differencing (tools/profile_live_ingest.py does this in WheelIngest.get()
and staleness_ms()).

PROCESS-ISOLATION WARNING: run this replayer as its own OS process (separate
shell, `&`, or subprocess.Popen) — NEVER as a thread inside the consumer
process. The jiminghe wheel's XsensWrapper.init() blocks while holding the
GIL until a datagram arrives, so an in-process sender thread is never
scheduled and both sides deadlock (60 s hang reproduced 2026-08-03).

LAFAN1 -> Xsens mapping: GMR's loader outputs global positions (meters, Z-up)
and global quaternions (w,x,y,z) per bone, which matches the MXTP02 convention
directly (after m -> cm). The only gap is the spine: LAFAN1 has 3 spine bones
(Spine, Spine1, Spine2) vs Xsens 4 (L5, L3, T12, T8); T12 duplicates Spine1.

XSENS BVH REPLAY (--xsens-bvh): a real Xsens recording exported as BVH with the
3DS Max preset, loaded with GMR's Xsens loader (centimetres, zero offsets) and
streamed as MVN segments: recorded joint-centre positions and global segment
orientations, unchanged. GMR's xsens_mvn_to_g1.json arm offsets (tuned by the
Xsens author on a live MVN stream) track these orientations with ~5 cm hand
error and no reversed elbows on a recorded MVN take, so the 3DS Max BVH segment
frames match the live MVN convention closely enough for replay testing.

Usage:
  VENVPY tools/mxtp02_replayer.py --xsens-bvh "/path/to/take.bvh" --rate 60 --loop
  VENVPY tools/mxtp02_replayer.py --bvh /path/to/clip.bvh --rate 60 --loop
  VENVPY tools/mxtp02_replayer.py --synthetic --rate 60   # no BVH needed
"""

import argparse
import math
import socket
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))
from mocap_g1.mxtp02 import (  # noqa: E402
    XSENS_JOINT_SEGMENT_PAIRS,
    XSENS_SEGMENTS,
    build_body_datagram,
    build_joint_angles_datagram,
)


# LAFAN1 bone providing each Xsens segment (T12 duplicates Spine1).
LAFAN1_TO_XSENS = [
    "Hips", "Spine", "Spine1", "Spine1", "Spine2", "Neck", "Head",
    "RightShoulder", "RightArm", "RightForeArm", "RightHand",
    "LeftShoulder", "LeftArm", "LeftForeArm", "LeftHand",
    "RightUpLeg", "RightLeg", "RightFoot", "RightToe",
    "LeftUpLeg", "LeftLeg", "LeftFoot", "LeftToe",
]

N_SEG = len(XSENS_SEGMENTS)

# Xsens 3DS Max BVH bone (GMR Xsens loader names) providing each MVN segment.
XSENS_BVH_TO_SEGMENT = {
    "pelvis": "Hips", "l5": "Chest", "l3": "Chest2", "t12": "Chest3", "t8": "Chest4",
    "neck": "Neck", "head": "Head",
    "right_shoulder": "RightCollar", "right_upper_arm": "RightShoulder",
    "right_forearm": "RightElbow", "right_hand": "RightWrist",
    "left_shoulder": "LeftCollar", "left_upper_arm": "LeftShoulder",
    "left_forearm": "LeftElbow", "left_hand": "LeftWrist",
    "right_upper_leg": "RightHip", "right_lower_leg": "RightKnee",
    "right_foot": "RightAnkle", "right_toe": "RightToe",
    "left_upper_leg": "LeftHip", "left_lower_leg": "LeftKnee",
    "left_foot": "LeftAnkle", "left_toe": "LeftToe",
}
def xsens_bvh_segments(frame):
    """One GMR Xsens-loader frame -> (pos (23,3) metres, quat (23,4) wxyz) in MVN order."""
    pos = np.zeros((N_SEG, 3), dtype=np.float64)
    quat = np.zeros((N_SEG, 4), dtype=np.float64)
    for i, segment in enumerate(XSENS_SEGMENTS):
        position, orientation = frame[XSENS_BVH_TO_SEGMENT[segment]]
        q = np.asarray(orientation, dtype=np.float64)
        pos[i] = position
        quat[i] = q / np.linalg.norm(q)
    return pos, quat


def load_xsens_bvh_frames(bvh_file, start_seconds=0.0, seconds=0.0):
    """Load a 3DS Max Xsens BVH via GMR -> (pos_m (F,23,3), quat (F,23,4), fps).

    ``start_seconds``/``seconds`` (0 = to the end) select a window before
    parsing; a long 240 Hz take otherwise takes tens of seconds to load.
    """
    import argparse as _argparse
    import tempfile

    from general_motion_retargeting.utils.xsens import load_xsens_file
    from tools.retarget.retarget_robot_recorded import (
        _load_xsens_with_offsets,
        _write_zero_offsets,
    )

    from tools.retarget.retarget_robot_recorded import _bvh_frame_time

    bvh_path = Path(bvh_file).expanduser().resolve()
    frame_time = _bvh_frame_time(bvh_path)
    start = int(round(start_seconds / frame_time))
    end = start + int(round(seconds / frame_time)) if seconds > 0 else None
    loader_args = _argparse.Namespace(
        bvh_file=str(bvh_path), scale=0.01, start=start, end=end,
        reset_to_zero=False, bvh_format="3DSM",
    )
    with tempfile.TemporaryDirectory(prefix="mxtp02-replay-") as temporary:
        offsets, _count = _write_zero_offsets(bvh_path, Path(temporary))
        frames, _height, frame_time = _load_xsens_with_offsets(
            load_xsens_file, loader_args, offsets
        )
    pos = np.zeros((len(frames), N_SEG, 3), dtype=np.float64)
    quat = np.zeros((len(frames), N_SEG, 4), dtype=np.float64)
    for f, frame in enumerate(frames):
        pos[f], quat[f] = xsens_bvh_segments(frame)
    return pos, quat, 1.0 / float(frame_time)


def build_datagram(sample_counter, time_code_ms, pos, quat_wxyz):
    """pos: (23,3) float (units chosen by caller), quat_wxyz: (23,4) -> bytes."""
    return build_body_datagram(sample_counter, time_code_ms, pos, quat_wxyz)


def load_bvh_frames(bvh_file):
    """Load BVH via GMR's LAFAN1 loader -> (pos_m (F,23,3), quat (F,23,4), bvh_fps)."""
    from general_motion_retargeting.utils.lafan1 import load_bvh_file
    frames, _human_height = load_bvh_file(bvh_file)  # meters, Z-up, quat wxyz
    n = len(frames)
    pos = np.zeros((n, N_SEG, 3), dtype=np.float64)
    quat = np.zeros((n, N_SEG, 4), dtype=np.float64)
    for f, frame in enumerate(frames):
        for i, bone in enumerate(LAFAN1_TO_XSENS):
            p, q = frame[bone]
            pos[f, i] = p
            quat[f, i] = q
    return pos, quat, 30.0  # LAFAN1 is 30 fps; positions stay in meters


def synthetic_frames(rate, seconds=10.0):
    """Static T-pose with sinusoidal arm swing. Z-up, meters. Fallback when no BVH."""
    identity = np.array([1.0, 0.0, 0.0, 0.0])
    # name -> (x, y, z) in meters, Z-up, facing +X, pelvis at origin-ish
    base = {
        "pelvis": (0, 0, 0.95), "l5": (0, 0, 1.05), "l3": (0, 0, 1.15),
        "t12": (0, 0, 1.25), "t8": (0, 0, 1.35), "neck": (0, 0, 1.55),
        "head": (0, 0, 1.68),
        "right_shoulder": (0, -0.10, 1.50), "right_upper_arm": (0, -0.20, 1.48),
        "right_forearm": (0, -0.48, 1.48), "right_hand": (0, -0.75, 1.48),
        "left_shoulder": (0, 0.10, 1.50), "left_upper_arm": (0, 0.20, 1.48),
        "left_forearm": (0, 0.48, 1.48), "left_hand": (0, 0.75, 1.48),
        "right_upper_leg": (0, -0.10, 0.90), "right_lower_leg": (0, -0.10, 0.50),
        "right_foot": (0, -0.10, 0.08), "right_toe": (0.15, -0.10, 0.03),
        "left_upper_leg": (0, 0.10, 0.90), "left_lower_leg": (0, 0.10, 0.50),
        "left_foot": (0, 0.10, 0.08), "left_toe": (0.15, 0.10, 0.03),
    }
    n = int(seconds * rate)
    pos = np.zeros((n, N_SEG, 3))
    quat = np.zeros((n, N_SEG, 4))
    quat[:, :, :] = identity
    for f in range(n):
        t = f / rate
        swing = 0.25 * np.sin(2 * np.pi * 0.5 * t)  # +-25 cm arm swing at 0.5 Hz
        for i, name in enumerate(XSENS_SEGMENTS):
            x, y, z = base[name]
            if "hand" in name or "forearm" in name:
                x = x + swing
            pos[f, i] = (x, y, z)
    return pos, quat, float(rate)


def positive_rate(text: str) -> float:
    """argparse type for a send rate in Hz: finite and strictly positive."""
    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"invalid rate {text!r}") from None
    if not math.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError(f"rate must be a finite value > 0 Hz, got {text}")
    return value


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1],
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--bvh", default=None, help="LAFAN1 BVH file to replay")
    ap.add_argument("--xsens-bvh", default=None,
                    help="real Xsens recording exported as 3DS Max BVH (needs GMR)")
    ap.add_argument("--start-seconds", type=float, default=0.0,
                    help="--xsens-bvh: first second of the take to replay")
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="--xsens-bvh: seconds to replay (0 = to the end of the take)")
    ap.add_argument("--synthetic", action="store_true",
                    help="Use synthetic T-pose + arm swing instead of a BVH")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=9763)
    ap.add_argument("--rate", type=positive_rate, default=60.0, help="Send rate in Hz")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="Stop after this many seconds (0 = until clip ends / forever)")
    ap.add_argument("--loop", action="store_true", help="Loop the clip forever")
    # NOTE: BVH frames always advance at their native fps regardless of --rate
    # (frames are repeated/skipped as needed) — see the frame-pick logic in the
    # send loop. A dead `--natural-speed` flag for this was removed 2026-08-03.
    ap.add_argument("--units", choices=["m", "cm"], default="m",
                    help="Position units on the wire (see UNITS NOTE in docstring)")
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument(
        "--with-joint-angles",
        action="store_true",
        help="also emit canonical zero-angle MXTP20 packets for the ROS 2 driver",
    )
    args = ap.parse_args()

    if args.xsens_bvh is not None:
        if args.bvh is not None or args.synthetic:
            ap.error("--xsens-bvh cannot be combined with --bvh or --synthetic")
        if args.start_seconds < 0 or args.seconds < 0:
            ap.error("--start-seconds and --seconds cannot be negative")
        pos_data, quat, src_fps = load_xsens_bvh_frames(
            args.xsens_bvh, args.start_seconds, args.seconds)
        src_name = args.xsens_bvh
        loop = args.loop
    elif args.synthetic or args.bvh is None:
        if not args.synthetic and args.bvh is None:
            print("No --bvh given; using --synthetic skeleton.", file=sys.stderr)
        pos_data, quat, src_fps = synthetic_frames(args.rate)
        src_name = "synthetic-tpose-armswing"
        loop = True  # synthetic clip always loops
    else:
        pos_data, quat, src_fps = load_bvh_frames(args.bvh)
        src_name = args.bvh
        loop = args.loop
    if args.units == "cm":
        pos_data = pos_data * 100.0
    n_frames = pos_data.shape[0]

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (args.host, args.port)
    dt = 1.0 / args.rate
    if not args.quiet:
        packet_description = "MXTP02 + MXTP20" if args.with_joint_angles else "MXTP02"
        body_size = len(build_datagram(0, 0, pos_data[0], quat[0]))
        print(f"Streaming {packet_description}: {src_name} "
              f"({n_frames} frames @ {src_fps:g} fps source) "
              f"-> udp://{dest[0]}:{dest[1]} at {args.rate:g} Hz "
              f"({body_size} B/body datagram)")

    sample = 0
    sent = 0
    t_start = time.perf_counter()
    next_t = t_start
    try:
        while True:
            now = time.perf_counter()
            elapsed = now - t_start
            if args.duration > 0 and elapsed >= args.duration:
                break
            # Pick source frame at native speed (always on): the clip plays at
            # src_fps wall time no matter the datagram --rate; frames repeat
            # when rate > src_fps and are skipped when rate < src_fps.
            f = int(elapsed * src_fps)
            if f >= n_frames:
                if loop:
                    f %= n_frames
                else:
                    break
            wall_ms = int(time.time() * 1000.0)  # wall-clock stamp for latency calc
            body_datagram = build_datagram(sample, wall_ms, pos_data[f], quat[f])
            sock.sendto(body_datagram, dest)
            if args.with_joint_angles:
                zero_rotations = [(0.0, 0.0, 0.0)] * len(XSENS_JOINT_SEGMENT_PAIRS)
                joint_datagram = build_joint_angles_datagram(sample, wall_ms, zero_rotations)
                sock.sendto(joint_datagram, dest)
            sample += 1
            sent += 1
            if not args.quiet and sent % int(max(args.rate, 1) * 5) == 0:
                packets = sent * (2 if args.with_joint_angles else 1)
                print(f"  sent {sent} frames / {packets} datagrams "
                      f"({sent / elapsed:.1f} Hz avg), frame {f}/{n_frames}")
            next_t += dt
            sleep = next_t - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.perf_counter()  # fell behind; resync
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        elapsed = time.perf_counter() - t_start
        packets = sent * (2 if args.with_joint_angles else 1)
        print(f"Done: {sent} frames / {packets} datagrams in {elapsed:.1f} s "
              f"({sent / max(elapsed, 1e-9):.1f} Hz achieved)")


if __name__ == "__main__":
    main()
