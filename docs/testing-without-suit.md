# Test the installation without an Xsens suit

Run these checks on the Linux simulation PC after installing the live route ([setup guide step 02](engineer-setup.html#step-02)). They need the built GMR image, Docker GPU access and the logged-in X11 desktop. They need no Windows capture PC, Xsens license or physical robot. Run commands from the repository root as your normal desktop user.

These are synthetic installation checks. They do not establish calibration, anatomical motion quality, real network performance or demo readiness. Production launchers deliberately reject loopback source addresses; the explicit Docker command below uses loopback only for this test.

## 1. Verify dependencies and GPU access

```bash
.venv/bin/python tools/checks/setup_doctor.py core --with-operator
.venv/bin/python tools/xsens/stream_acceptance.py --self-test --require-joint-angles
docker run --rm --gpus all --entrypoint nvidia-smi gmr-live:bb1bbe4 -L
```

Expected: core `READY`, self-test `PASS`, and the GPU listed from inside Docker. If an image is missing, rerun `bash tools/setup/install.sh live --with-ros2`. For pip, GPU-runtime or display failures, use setup guide step 12 and the [engineering reference](engineer-reference.md#full-troubleshooting). A live-doctor block for an unset real source IP is expected at this stage.

## 2. Start the finite G1 viewer — terminal A

Check the dedicated test port. If this prints a listener, identify it and use an idle port consistently in both terminals; do not stop an unrelated process.

```bash
ss -H -lun | rg ':15555\b'
```

No output is expected (`rg` returns 1 when there is no match). Keep terminal A open. This creates a fresh ignored output folder and opens a MuJoCo window once packets arrive:

```bash
mkdir -p artifacts
export TEST_DIR="$(mktemp -d "$PWD/artifacts/g1-install-test-XXXXXX")"
test_xauth=${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}
printf 'Test output: %s\n' "$TEST_DIR"
set -o pipefail
docker run --rm --name gmr-install-test \
  --network host --gpus all --shm-size 1g \
  --user "$(id -u):$(id -g)" \
  --cap-drop ALL --security-opt no-new-privileges \
  -e NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility,compute \
  -e DISPLAY="${DISPLAY:-:0}" -e XAUTHORITY=/tmp/xauthority \
  -e HOME=/tmp/gmr-home \
  --tmpfs "/tmp/gmr-home:rw,nosuid,nodev,size=256m,uid=$(id -u),gid=$(id -g)" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v "$test_xauth":/tmp/xauthority:ro \
  -v "$TEST_DIR":/data \
  gmr-live:bb1bbe4 \
  --bind 127.0.0.1 --port 15555 --source-ip 127.0.0.1 \
  --human-height 1.75 --startup-timeout 60 --duration 10 \
  --record-video --video-path /data/g1-synthetic.mp4 \
  2>&1 | tee "$TEST_DIR/receiver.log"
```

`1.75` is a synthetic test parameter, not a measurement of a performer. Wait for `Waiting for strict MXTP02`, then start terminal B within 60 seconds. Do not close the viewer early: let the ten-second run finish and finalize its video. If the container name is already in use, inspect that exact container before retrying.

## 3. Send synthetic motion — terminal B

Open a second terminal in the same repository and run:

```bash
.venv/bin/python tools/mxtp02_replayer.py \
  --synthetic --host 127.0.0.1 --port 15555 \
  --rate 60 --duration 25 --with-joint-angles
```

The sender exits after 25 seconds. The viewer runs for ten seconds after initialization; the longer sender duration gives it startup time. Expect a moving G1, periodic `G1 live` counters and a final video-saved message. The input is a synthetic skeleton, so do not use its pose to approve real retargeting quality.

## 4. Check the result — terminal A

Require a successful viewer exit, positive frame counts, `failures=0`, `malformed_packets=0`, and a completed nonempty MP4. An exit code of zero alone is insufficient because the viewer catches frame-local retargeting errors. Inspect the complete receiver log for errors or faults.

```bash
rg 'G1 live:|Receiver:|Video saved|Retargeting failed|Traceback' "$TEST_DIR/receiver.log"
test -s "$TEST_DIR/g1-synthetic.mp4"
```

Verify decoding and extract a preview using dependencies already in the image:

```bash
docker run --rm -i --network none --user "$(id -u):$(id -g)" \
  -v "$TEST_DIR":/data --entrypoint python gmr-live:bb1bbe4 - <<'PY'
import imageio.v2 as imageio
with imageio.get_reader('/data/g1-synthetic.mp4') as reader:
    count = reader.count_frames()
    assert count > 0, 'Video has no frames'
    print('Decoded frames:', count)
    print('Video metadata:', reader.get_meta_data())
    imageio.imwrite('/data/g1-preview.png', reader.get_data(min(120, count - 1)))
PY
```

Open the video or preview locally and confirm that the G1 and floor are rendered. FPS can vary with machine load; neither encoding FPS nor host-side compute timing is measured person-to-screen latency.

## 5. Optional ROS 2 stream and recording check

This existing smoke test uses UDP **9763**, not 15555 or the production 5555. First check that 9763 is free. Run after the viewer test has finished:

```bash
ss -H -lun | rg ':9763\b'
SMOKE_DURATION_S=40 tools/checks/ros2_synthetic_smoke.sh
```

Expected: `PASS: paired synthetic datagrams produced decoded ROS 2 motion topics and a rosbag.` The script checks actual motion messages and nonzero `/joint_states`, `/link_states` and `/tf` counts. It prints a temporary artifact directory containing logs, `bag-info.txt` and the rosbag. Copy that printed directory into your session evidence folder before temporary-file cleanup. Its receiver uses host networking; run on a controlled test network without another source sending to 9763.

## 6. Stop and proceed to real commissioning

The finite runs exit automatically and their containers are removed. To interrupt your viewer test, use Ctrl+C in terminal A; if that terminal is lost, stop only the named test container:

```bash
docker stop --timeout 5 gmr-install-test
ss -H -lun | rg ':15555\b'
```

Stop the sender with Ctrl+C if needed. A missing container or no port match is normal after completion. Preserve logs and completed outputs; do not reuse an interrupted MP4 as success evidence.

Now follow setup guide steps 03–07: enter the real session values in the Setup console (or with `bash tools/setup/new_session.sh`), calibrate real Xsens, pass real acceptance, and inspect real motion. Do not carry loopback addresses or the synthetic height into that session.

## Replay a real recording

This plays a real Xsens take, exported as BVH with the 3DS Max preset, into the live viewer as if the suit were streaming it, for either robot. It needs the recorded runtime from the installer (option 2) because the replayer loads the BVH with GMR, and a local desktop. It uses loopback, so it runs the viewer directly rather than through `live_operator.sh`, which rejects loopback sources.

Terminal A, the live viewer (`--calibrate-after 0.5` applies the N calibration automatically; start the replay at a moment where the performer stands still):

```bash
source session.env
PYTHONPATH="$MOCAP_WS/GMR" "$MOCAP_WS/venv/bin/python" tools/xsens/live_g1_sim.py \
  --robot "$ROBOT" --bind 127.0.0.1 --port 15556 --source-ip 127.0.0.1 \
  --human-height "$HUMAN_HEIGHT_M" --calibrate-after 0.5
```

Terminal B, the replayer, once terminal A prints its waiting message:

```bash
source session.env
"$MOCAP_WS/venv/bin/python" tools/mxtp02_replayer.py --xsens-bvh "$MOTION_BVH" \
  --start-seconds 0 --seconds 60 --loop --host 127.0.0.1 --port 15556 --rate 60
```

Expected: the robot follows the recording at about 60 FPS with `failures=0`, and the terminal prints `NEUTRAL CALIBRATION` once. Stop both with Ctrl+C. A replay checks the retargeting and viewer, not the network, the suit or real-stream acceptance.

## Reference result from 2026-10-11

On the tested Ubuntu 24.04/RTX 3080 workstation, the ten-second synthetic GMR viewer test produced 587 retargeted/video frames, zero retargeting failures and zero malformed packets. The last periodic cumulative rate was 58.8 FPS. The MP4 decoded as 640×480, 60 FPS encoding, 9.78 seconds. The ROS smoke bag contained 2,307 messages. These are local installation results, not minimum performance guarantees or real-suit acceptance.

Local logs and video are under ignored `artifacts/pc-test-setup-2026-10-11/`; another clone will not contain those results. The commands above are the portable reproduction procedure. Record your own source revision, Docker image IDs, reports and observations.
