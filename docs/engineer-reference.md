# Engineering reference · Xsens mocap → humanoid simulation

Details behind the [step-by-step setup guide](engineer-setup-guide.md). Use the guide for normal setup; come here for edge cases, advanced calibration, full troubleshooting and maintainer notes. Step numbers below refer to the guide.

## Capability boundary

| Workflow | Booster K1 (default) | Unitree G1 | Booster T1 |
| --- | --- | --- | --- |
| Real suit → live UDP → MuJoCo | Implemented; commission with real-stream acceptance | Implemented; commission with real-stream acceptance | Implemented (simulation only); commission with real-stream acceptance |
| Real recording → 3DS Max BVH → MuJoCo | Implemented with project K1 mapping (config arms) | Implemented | Implemented with project T1 mapping |
| Browser operator console | Setup console and passive MP4 review; no K1 virtual simulator | Passive MP4 review or virtual G1 NPZ simulation | Passive MP4 review; no T1 virtual simulator |
| Physical robot control | Arm bridge built and tested on a fake robot; locked until the step-by-step hardware tests (`tools/control/k1_commission.py`, [K1 hardware test guide](k1-bringup-runbook.md)) write its acceptance record | Outside this guide; whole-body authority remains locked | Simulation comparator only |

**What is proven:** recorded G1/T1 previews, synthetic live/ROS ingest, and live G1/T1 retargeting of a real recorded take replayed as MXTP02 (no hand/arm overlap with the hips on either robot, about 59 FPS). **What is outstanding:** real MXTP02/MXTP20 stream acceptance and full reviewed Xsens offsets. Existing demonstration takes remain preview-grade. MuJoCo here shows kinematic motion; it does not prove standing balance or a tracking controller. A physical robot is optional and later.

The data flow is Windows Xsens capture → Linux acceptance → K1, G1 or T1 live MuJoCo, or Windows BVH export → Linux GMR retargeter → K1, G1 or T1 MuJoCo. ROS 2 is an optional separate consumer. The browser console is a separate review tool. See the [project README](../README.md) for the overview.

## Workstation requirements

**Windows capture PC:** a physical, supported 64-bit Windows machine that meets the installed Xsens Humanoid release requirements; the delivered MVN Link suit, Body Pack, router, charged sensors, cables and matching vendor manual; a valid Xsens entitlement with streaming/export features. Use the permanent capture PC for the one-seat System License.

**Linux simulation PC:** native Ubuntu 24.04 (default) or 22.04. Python 3.10–3.12; 3.11 or 3.12 is preferred for a fresh install. Live containers require an NVIDIA driver, NVIDIA Container Toolkit and a local X11 display. Native recorded rendering requires working MuJoCo/OpenGL; NVIDIA is required by the Docker launchers, not by inverse kinematics itself. Docker Desktop/WSL live reception is not the commissioned path.

Keep Internet on Wi-Fi and connect both PCs' Ethernet to the Xsens router. Do not connect a physical robot and do not bridge capture and robot networks. Plan free space for raw `.mvn`, BVH, virtual environments, Docker images and MP4s; the historical full GMR install used about 5.2 GiB of dependencies.

## Installer details

`bash tools/setup/install.sh` with no arguments asks the setup questions. Scripts and administrators can call a profile directly:

| Command | Installs |
| --- | --- |
| `bash tools/setup/install.sh core` | Project `.venv`, runtime/operator dependencies, doctor and decoder self-test |
| `bash tools/setup/install.sh live` | Core plus the live image (every robot), after Docker/NVIDIA/X11 preflight |
| `bash tools/setup/install.sh recorded --workspace "$HOME/mocap-runtime" --robot booster_k1` | Core plus the pinned GMR runtime (`--robot unitree_g1` or `booster_t1_29dof` for the others); also needed to replay recordings into the live viewer |
| `bash tools/setup/install.sh live --with-ros2` | Live plus the optional ROS 2 image |

Options: `--system-packages` installs any missing Ubuntu packages with sudo (skipped when all are present); `--dry-run` prints the commands without running anything; `--python python3.12` picks the interpreter for a new `.venv`. `--workspace` defaults to `~/mocap-runtime` and must be outside the checkout. The installer stops at the first failure and is safe to rerun. It never runs pip as root and never deletes an existing environment.

Verify the delivered checkout includes the setup tools before installing; uncommitted local files are not in a Git clone:

```bash
git rev-parse HEAD
test -f tools/setup/install.sh && test -f tools/setup/new_session.sh && test -f tools/checks/setup_doctor.py
```

### Which Python is used

| Environment | Location | Used for |
| --- | --- | --- |
| Project venv | `.venv/bin/python` | Console, acceptance, doctor and project checks |
| GMR runtime venv | `$MOCAP_WS/venv/bin/python` | Native recorded G1/T1 retargeting and rendering |
| Live Docker image | `/opt/venv/bin/python` inside image | Live G1/T1 preview; the launcher supplies the environment |

If an existing `.venv` reports `No module named pip`, repair it with `.venv/bin/python -m ensurepip --upgrade`. If `ensurepip` is unavailable, install the Ubuntu `python3-venv` package for your interpreter. Preserve an incompatible environment (move it aside) rather than deleting it.

### Recorded runtime

The recorded profile runs `tools/bootstrap_g1_workspace.py`, which downloads the exact GMR revision `bb1bbe40774794fceb2a7c579a3464a28e68c844`, applies project patches, creates its own `venv/` and writes `runtime-lock.txt`. Its check JSON must contain `ok: true`, `prepared_ok: true` and `runtime_ok: true`. If `prepared_ok` is true but `runtime_ok` is false, rerun bootstrap with dependency installation, not just `--prepare-only`. If an existing GMR folder has a different revision, use a new workspace folder; do not reset a colleague's checkout.

## Session values

The Setup console's **03 Session** step and `bash tools/setup/new_session.sh` write the same file: these variables to `<session folder>/session.env` and links `session.env` in the checkout to it. `--defaults` accepts every default without asking; `--route`, `--robot` and `--workspace` preset answers. To set them by hand instead:

```bash
export MOCAP_WS="$HOME/mocap-runtime"
export SESSION_DIR="$HOME/mocap-sessions/session-001"
export MOTION_BVH="$HOME/mocap-data/motion.bvh"
export NPOSE_BVH="$HOME/mocap-data/neutral.bvh"
export OFFSETS_FILE=''
export HUMAN_HEIGHT_M='1.75'
export BVH_SCALE='0.01'
export START_FRAME='0'
export END_FRAME='600'
export FRAME_STEP='1'
export NPOSE_START_S='2.0'
export NPOSE_DURATION_S='1.0'
export WINDOWS_XSENS_IP='<Windows ipconfig address>'
export LINUX_XSENS_IP='<Linux ip -br -4 address>'
export XSENS_SOURCE_IP="$WINDOWS_XSENS_IP"
export XSENS_UDP_PORT=5555
export ROBOT='booster_k1'
export IK_CONFIG="$PWD/retarget_configs/bvh_xsens_to_booster_k1.json"
mkdir -p "$SESSION_DIR"
```

For the Unitree G1, set `ROBOT=unitree_g1` and `IK_CONFIG="$MOCAP_WS/GMR/general_motion_retargeting/ik_configs/bvh_xsens_to_g1.json"`; for Booster T1, `ROBOT=booster_t1_29dof` and `IK_CONFIG="$PWD/retarget_configs/bvh_xsens_to_booster_t1_29dof.json"`. The **Linux IP is the destination entered in Xsens**; the **Windows IP is the source filter used on Linux**. IP addresses come from DHCP; confirm them every session. Do not enter license keys or activation data.

## Terminal equivalents of the Setup console

Each Setup console button runs one of these commands from the project folder (its log shows the exact command). Use them over SSH, in scripts, or when the console is unavailable. Load the session values in every new terminal first:

```bash
source session.env
```

| Setup console | Terminal |
| --- | --- |
| 03 · Save session | `bash tools/setup/new_session.sh` |
| 03 · Run system check | `.venv/bin/python tools/checks/setup_doctor.py live` (recorded: `recorded --workspace "$MOCAP_WS" --robot "$ROBOT"`) |
| 06 · Run 30-second check | `tools/xsens/live_operator.sh acceptance`, then copy `artifacts/xsens-live/<newest>/stream-acceptance.json` to `$SESSION_DIR` |
| 07 · Start live simulation | `tools/xsens/live_operator.sh gmr --robot "$ROBOT" --human-height "$HUMAN_HEIGHT_M"` (add `--calibrate-after 5`, `--record-video`); Ctrl+C stops it |
| 07 · Stop leftover viewer | `docker stop gmr-xsens-live` |
| 08 · Check files | `rg -n '^(Frames:\|Frame Time:)' "$MOTION_BVH" "$NPOSE_BVH"` and `sha256sum "$MOTION_BVH" "$NPOSE_BVH" > "$SESSION_DIR/bvh-sha256.txt"` |
| 11 · Save report | `git rev-parse HEAD > "$SESSION_DIR/project-revision.txt"` and `.venv/bin/python tools/checks/setup_doctor.py core --with-operator --json > "$SESSION_DIR/core-doctor.json"` |

Step 09 · **Make preview**:

```bash
"$MOCAP_WS/venv/bin/python" tools/retarget/retarget_robot_recorded.py \
  --gmr-root "$MOCAP_WS/GMR" --robot "$ROBOT" --ik-config "$IK_CONFIG" \
  --bvh-file "$MOTION_BVH" --save-path "$SESSION_DIR/$ROBOT-preview.pkl" \
  --scale "$BVH_SCALE" --human-height "$HUMAN_HEIGHT_M" \
  --start "$START_FRAME" --end "$END_FRAME" --frame-step "$FRAME_STEP" \
  --allow-zero-offsets --arm-mode position-only --overwrite
```

Step 09 · **Make arm-aligned (N-pose)** (see [N-pose arm alignment](#n-pose-arm-alignment)):

```bash
"$MOCAP_WS/venv/bin/python" tools/retarget/retarget_robot_recorded.py \
  --gmr-root "$MOCAP_WS/GMR" --robot "$ROBOT" --ik-config "$IK_CONFIG" \
  --bvh-file "$MOTION_BVH" --save-path "$SESSION_DIR/$ROBOT-npose.pkl" \
  --scale "$BVH_SCALE" --human-height "$HUMAN_HEIGHT_M" \
  --start "$START_FRAME" --end "$END_FRAME" --frame-step "$FRAME_STEP" \
  --allow-zero-offsets --arm-mode npose \
  --calibration-bvh-file "$NPOSE_BVH" \
  --calibration-start-seconds "$NPOSE_START_S" --calibration-seconds "$NPOSE_DURATION_S" \
  --calibration-output "$SESSION_DIR/$ROBOT-arm-calibration.json" \
  --accept-reviewed-calibration-window --overwrite
```

Step 10 · **Record MP4** (drop the last line for **Open viewer**; use `-npose` for the arm-aligned motion):

```bash
"$MOCAP_WS/venv/bin/python" tools/retarget/render_robot_motion.py \
  --gmr-root "$MOCAP_WS/GMR" --robot "$ROBOT" \
  --motion "$SESSION_DIR/$ROBOT-preview.pkl" --loops 1 \
  --record-video --video-path "$SESSION_DIR/$ROBOT-preview.mp4" --overwrite-video
```

To review a video in the browser without the Setup console: `.venv/bin/mocap-g1-console --playback "$SESSION_DIR/$ROBOT-preview.mp4" --open`.

## Docker, NVIDIA and display

Follow [Docker's Ubuntu instructions](https://docs.docker.com/engine/install/ubuntu/) and [NVIDIA's Container Toolkit instructions](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html). Do not mix Docker package sources. `docker` must work as your normal desktop user; do not run the installer or launchers with sudo to hide a permission failure.

Check the driver, toolkit and registered runtimes:

```bash
nvidia-smi -L
nvidia-ctk --version
docker info --format '{{range $key,$value := .Runtimes}}{{$key}} {{end}}'
```

If the toolkit exists but `nvidia` is not listed, back up any existing Docker configuration, register the runtime, validate and reload:

```bash
if sudo test -f /etc/docker/daemon.json; then
  sudo cp -a /etc/docker/daemon.json "/etc/docker/daemon.json.before-xsens-$(date +%Y%m%d-%H%M%S)"
fi
sudo nvidia-ctk runtime configure --runtime=docker && \
  sudo dockerd --validate --config-file=/etc/docker/daemon.json && \
  sudo systemctl reload docker
```

Reload worked without restarting existing containers on the tested PC. If registration still fails, arrange a Docker restart with the workstation administrator; running services may be interrupted. See [Docker runtime configuration](https://docs.docker.com/reference/cli/dockerd/#configure-runtimes-using-daemonjson).

**Display:** log into a local Xorg/X11 desktop. The launchers use `DISPLAY` and an existing Xauthority file, defaulting to `/run/user/$(id -u)/gdm/Xauthority`; on another desktop, export `XAUTHORITY` to its real path. Do not copy the file contents or run `xhost +`. Check with:

```bash
printf 'Display: %s\nSession type: %s\n' "$DISPLAY" "$XDG_SESSION_TYPE"
ls /tmp/.X11-unix/
```

The live install checks image build and GPU access only; it does not open MuJoCo or verify suit traffic. [Docker images](#docker-images) below explains mounts and output folders. An unset `XSENS_SOURCE_IP` is a deliberate live-doctor blocker until the real capture PC is known; do not invent an address to get `READY`.

### Docker images

**`gmr-live:bb1bbe4`** (live route, both robots). Ubuntu 24.04 image that receives strict Xsens MXTP02 body frames over UDP, retargets them through the pinned GMR commit with the project's dependency-free decoder, and shows the 29-DOF G1 or the T1 (`--robot booster_t1_29dof`) in MuJoCo. GMR's inverse kinematics is CPU-bound; the NVIDIA passthrough supplies accelerated OpenGL for the viewer, and CUDA is not required. The installer's live profile builds it; by hand:

```bash
docker build -t gmr-live:bb1bbe4 -f docker/gmr-live/Dockerfile .
XSENS_SOURCE_IP="$WINDOWS_XSENS_IP" docker/gmr-live/run.sh --robot unitree_g1 --human-height "$HUMAN_HEIGHT_M"
```

`tools/xsens/live_operator.sh gmr` wraps that launcher. Environment overrides: `XSENS_UDP_PORT` (default `5555`), `GMR_LIVE_RUNTIME_ROOT` (output folder; default the ignored `artifacts/xsens-live/`, where `--record-video` writes `g1-live.mp4` or `t1-live.mp4`) and `XAUTHORITY` (defaults to GDM's path). The launcher runs in the foreground, waits for the first valid body frame, and names the container `gmr-xsens-live` (`--rm`, so Ctrl+C removes it). It uses host networking, filters packets to the configured source IP, drops stale queued frames, ignores the paired MXTP20 packets, drops Linux capabilities, runs as the host UID/GID, and mounts only the Xauthority file and the output directory. The X11 socket grants access to the desktop session and host networking removes Docker's network isolation, so run it only on the trusted capture LAN. No receiver is installed as a background service.

The same image loops a recorded 3DS Max BVH on the G1 (`docker/gmr-live/run-offline.sh '/absolute/path/to/motion.bvh'`, container `gmr-bvh-loop`, `--detach` to leave it running). It caches the retargeted poses under ignored `artifacts/xsens-offline/` keyed by BVH content, frame selection, scale, height, calibration hash, GMR commit, model and IK config (`--rebuild-cache` forces regeneration); the BVH is mounted read-only and the container has no network. Without a reviewed offsets file it uses zero offsets and position-only arms, labelled as a structural preview; `GMR_OFFSETS_FILE=/absolute/reviewed-offsets.json ... --arm-mode calibrated` enables full orientation tracking. A neutral-pose take can supply arm offsets instead (`GMR_CALIBRATION_BVH_FILE`, `GMR_NPOSE_START_SECONDS`, `GMR_NPOSE_SECONDS`, and `GMR_NPOSE_REVIEWED=1` to acknowledge that the window was visually checked; it is not a quality approval). Never derive BVH Euler offsets by negating the N-pose angles. `--loop-mode repeat --loops 1 --record-video --video-path /data/motion-g1.mp4` exports one pass as MP4 (`--video-frame-step 4 --video-width 480 --video-height 352` for a small review export); the exporter writes a `.partial.mp4` first and promotes it atomically, so retry an interrupted export with a new name or `--overwrite-video`. `loop_bvh_g1.py` and `live_g1_sim.py` are baked into the image: rebuild it after editing them.

**`xsens-ros2:jazzy`** (optional ROS 2 topics). Containerised build of the official [xsens/xsens_mvn_ros2](https://github.com/xsens/xsens_mvn_ros2) driver (ROS 2 Jazzy, lifecycle nodes), pinned to upstream commit `d2a515b876e2b9f4f127f3fbde1d20c5189adc95` by the `XSENS_REF` build argument (`docker build --build-arg XSENS_REF=<sha> ...` to bump). It builds the `xsens_mvn_*` packages in Release mode with `BUILD_TESTING=ON`; `rviz2` is not installed, so pass `launch_rviz:=false` to the launch files and run RViz on the host.

```bash
docker build -t xsens-ros2:jazzy docker/xsens-ros2
docker run --rm xsens-ros2:jazzy ros2 pkg list | grep xsens                      # sanity check
docker run --rm -it --network=host xsens-ros2:jazzy                               # interactive shell
docker run --rm xsens-ros2:jazzy bash -c 'cd /colcon_ws && colcon test --base-paths src/pkgs src/deps && colcon test-result --verbose'
```

`tools/xsens/live_operator.sh ros2` starts the lifecycle stream node with `udp_port=5555` in container `xsens-stream-5555`, with host networking (UDP delivery without NAT, and DDS discovery with other ROS 2 nodes on the host; a bridge mapping can deliver UDP but is not equivalent evidence) and dropped Linux capabilities. The upstream default port `9763` (and the `9736` mentioned in Xsens's ROS guide) is not used by this project; the Dockerfile's `EXPOSE 9763/udp` only records that default. The unmodified upstream launch remains available for testing it: `docker run --rm -it --network=host xsens-ros2:jazzy ros2 launch xsens_mvn_ros2 xsens_stream.launch.py launch_rviz:=false`. The direct-suit XME mode (`xsens_xme.launch.py`, USB passthrough, licence on that machine) is upstream reference material and is not part of this workflow. The hardware-free smoke test `tools/checks/ros2_synthetic_smoke.sh` (UDP 9763; `XSENS_ROS_IMAGE`, `SMOKE_DURATION_S` and `EXPECTED_TOPIC_PATTERNS` select a custom image or launch) is described in [Testing without an Xsens suit](testing-without-suit.md).

## Windows Xsens details

1. Download Xsens Humanoid, License Manager and the matching MVN Link hardware manual from [Xsens software and documentation](https://www.xsens.com/support/software-documentation). Check that release's system requirements.
2. Activate the System License interactively on the chosen physical PC; confirm product and validity dates. Do not activate the one-seat license in WSL, Docker, a VM or a second PC.
3. Physical sensor count and the software's 23-segment body model are different quantities; the parser expects 23 segments, not 23 sensors.
4. Save actor provenance with the take; do not reuse an old actor because the skeleton appears.
5. Repeat calibration after changing suit fit or sensor placement. An exporter bind/reference frame is not a measured N-pose.

> SCREENSHOT: W01 · License Manager after activation. Show product and validity only; remove key, protection ID, email and customer details.

See [the Windows runbook](windows-xsens-humanoid-setup.md). Manus gloves are not planned: the Booster K1 has no hands.

## Live route details

### Network checks

Ping can help diagnose routing, but ping success does not prove UDP delivery. Check the route, listeners and firewall:

```bash
ip route get "$WINDOWS_XSENS_IP"
ss -lunp | rg ':5555\b'
sudo ufw status numbered
```

No output from `ss` means no listener (and `rg` exits 1). If it shows another process, stop that known receiver; do not kill unrelated applications. `0.0.0.0` is a receiver bind address, never the Windows destination. If a non-UFW firewall is active, ask its administrator for an equivalent scoped rule; do not disable it.

### Overlapping Wi-Fi and Xsens subnets

If two separate routers use the same subnet, route priority is ambiguous. Use the NetworkManager helper only for that topology. Find adapter names with `ip -br link` and `nmcli device status`; get the Xsens router MAC from its label or admin page.

```bash
export XSENS_ETHERNET_INTERFACE='<your Xsens Ethernet interface>'
export XSENS_WIFI_INTERFACE='<your Internet Wi-Fi interface>'
export XSENS_GATEWAY_MAC='<confirmed Xsens router MAC>'
tools/xsens/setup_xsens_dual_network.sh --dry-run
tools/xsens/setup_xsens_dual_network.sh --configure-only
```

Review the dry run first. The helper persistently configures an `Xsens Ethernet` profile and a host route to the Windows IP. It may request sudo and does not change firewall rules.

### Stream acceptance

`tools/xsens/live_operator.sh acceptance` samples 30 seconds, filters `XSENS_SOURCE_IP`, requires at least 300 body frames at 50 Hz or more, at most 1% packet loss, and paired body/joint-angle frames. It writes `artifacts/xsens-live/<timestamp>/stream-acceptance.json` (ignored by Git; contains network details, so sanitize before sharing). MXTP12/13 only means metadata/scaling arrived. An animated Windows view, received UDP bytes or the synthetic self-test are not an acceptance pass. Do not reuse an old report after changing source, settings or calibration.

### Optional ROS 2 topics

Skip this unless you need ROS topics. ROS 2 does not feed the GMR viewer; both receive UDP independently, so run it **after acceptance has exited**:

```bash
tools/xsens/live_operator.sh ros2
```

From a second terminal:

```bash
docker exec xsens-stream-5555 /entrypoint.sh ros2 topic list
docker exec xsens-stream-5555 /entrypoint.sh ros2 topic echo --once /link_states
docker exec xsens-stream-5555 /entrypoint.sh ros2 topic hz /link_states
```

Expect `/link_states`, `/joint_states`, `/tf`, `/com` and `/diagnostics` while Xsens streams. The ROS node has no source-IP filter, so keep the scoped firewall. Its `--rm` container has no persistent rosbag mount; add a reviewed writable mount before recording real data. Two listeners on 5555 are not supported. See [Docker images](#docker-images); this route uses port **5555**, not the upstream defaults.

### Live viewer behaviour

`tools/xsens/live_operator.sh gmr --robot booster_k1|unitree_g1|booster_t1_29dof` loads the robot (22-joint K1, the default; 29-joint G1 or 27-hinge T1), retargets the newest MXTP02 frame and ignores MXTP20. Space enables waist/arm tracking; R resets/rearms from the current pose; N (or `--calibrate-after SECONDS`) runs the neutral-pose calibration. On a stale/hard-timeout latch, restore the source and inspect the cause before pressing R. Legs and the free base are kinematic preview. Reported FPS and latency do not measure full suit-to-display latency. See [the live implementation](../tools/xsens/live_g1_sim.py).

What the live viewer does, and why (measured on a recorded MVN take replayed as MXTP02):

- **IK configs.** G1 uses GMR's upstream `xsens_mvn_to_g1.json`; its arm offsets were tuned by the Xsens author on a live MVN stream and keep every elbow bending the performer's way. T1 uses the project's [`xsens_mvn_to_booster_t1_29dof.json`](../retarget_configs/xsens_mvn_to_booster_t1_29dof.md) and K1 the project's [`xsens_mvn_to_booster_k1.json`](../retarget_configs/xsens_mvn_to_booster_k1.json), which takes the T1 arm offsets; GMR ships none.
- **Arm abduction.** Every arm target is rotated outward about the shoulder, 10° on the G1 and 22° on the T1 and K1 (`--arm-abduction` overrides), because both robots' hip housings stand out further than their shoulders: hanging arms otherwise pass through the hips (G1 429 → 0, T1 561 → 0 of 840 frames).
- **Neutral calibration (N).** Puts the feet on the floor (the live stream is not grounded; the G1 otherwise sits about 8 cm low). For the T1, whose arms start position-only, it also derives arm orientation offsets from the neutral pose.
- **Command screening.** Simulation screens commands by joint ranges with stale hold and hard-timeout latch. `--g1-hardware-envelope` instead applies the G1 hardware safety envelope (speed/acceleration limited); in simulation it lags fast arm motion (up to 182°) and cuts through the body, so it is for real-robot readiness checks only.
- **GMR limits.** The pinned GMR passes its limits to mink in the `safety_break` slot, so they never reach the solver; the live process corrects this ([`mocap_g1/live_ik.py`](../mocap_g1/live_ik.py)) and uses joint-range limits only. mink 1.2.0's `CollisionAvoidanceLimit` was tried and rejected: it made the robot vibrate while standing.

## Recorded route advanced

### Export and frame selection

Export with the 3DS Max preset; the pinned importer supports `3DSM` only. Preserve `.mvn`, actor metadata and a matching `.mvnx` as provenance; they are not GMR inputs. Use `--scale 0.01` only for centimetre exports. `Frame Time` is seconds per frame. `--start` is inclusive and `--end` exclusive. For 240 Hz input, use `--frame-step 4` for about 60 Hz output. If the exporter injected a bind/reference first frame, start at frame 1; otherwise do not skip frame 0. A neutral window from one take is specific to that take; review it in Xsens each time.

> SCREENSHOT: W05 · BVH export dialog. Show 3DS Max preset, units, sample rate and Overwrite first frame setting.

Retarget outputs embed metadata (robot, source/config hashes, quaternion order). Existing outputs are protected; pass `--overwrite` deliberately. A `.partial` file is not a finished result. Do not unpickle files from untrusted sources.

### N-pose arm alignment

The N-pose offsets are derived per robot arm chain. The Booster K1 does not use them: its forearm has no hand target, so the N-pose reference solve leaves it free to twist. Its preview motion uses `--arm-mode config --arm-abduction 22` instead, which matches the live K1 arms within 1.6° RMS. Do not copy G1 arm quaternions to T1 or reuse LAFAN offsets. `--allow-zero-offsets` keeps the result preview-grade (`preview_calibration: true`, `quality_approved: false`). A checked N-pose is not complete segment/axial-twist calibration. Pass `--accept-reviewed-calibration-window` only after reviewing the window in Xsens.

### Complete offsets

Use a complete Xsens BVH offsets JSON from a reviewed calibration workflow; an all-zero file is refused without preview mode. This guide cannot invent performer calibration. A retargeting engineer can create a candidate with the pinned GMR editor:

```bash
mkdir -p "$SESSION_DIR/calibration-work"
(
  cd "$SESSION_DIR/calibration-work" || exit
  "$MOCAP_WS/venv/bin/python" \
    "$MOCAP_WS/GMR/general_motion_retargeting/utils/xsens_vendor/mujoco_xsens_bvh_view.py" \
    --bvh_file "$NPOSE_BVH" --scale "$BVH_SCALE"
)
```

1. Select each joint/channel and review its orientation against the source pose and Xsens 3DSM convention. Do not negate all neutral Euler angles or copy another performer's file.
2. Set **JSON Path** to a new absolute candidate path and use **Apply and Preview**. The default filename is `offsets.json`.
3. Check every articulated joint has finite `x`, `y`, `z`. Review neutral stance, arm raises, forearm/hand twist, torso rotation and feet on a separate motion take.
4. Only after review, set `OFFSETS_FILE` (rerun `new_session.sh` and choose to change offsets) and record the file hash.

> SCREENSHOT: L01 · Pinned GMR offsets editor. Show joint/channel selectors, JSON Path and Apply and Preview with a sanitized candidate file.

Calibrated retarget, **G1 only**:

```bash
"$MOCAP_WS/venv/bin/python" tools/retarget/retarget_robot_recorded.py \
  --gmr-root "$MOCAP_WS/GMR" --robot "$ROBOT" --ik-config "$IK_CONFIG" \
  --bvh-file "$MOTION_BVH" --save-path "$SESSION_DIR/$ROBOT-calibrated.pkl" \
  --scale "$BVH_SCALE" --human-height "$HUMAN_HEIGHT_M" \
  --start "$START_FRAME" --end "$END_FRAME" --frame-step "$FRAME_STEP" \
  --offsets-file "$OFFSETS_FILE" --arm-mode calibrated
```

**T1 with complete offsets** keeps the robot-specific N-pose arm alignment, because the T1 mapping has identity arm-orientation fallbacks:

```bash
"$MOCAP_WS/venv/bin/python" tools/retarget/retarget_robot_recorded.py \
  --gmr-root "$MOCAP_WS/GMR" --robot "$ROBOT" --ik-config "$IK_CONFIG" \
  --bvh-file "$MOTION_BVH" --save-path "$SESSION_DIR/$ROBOT-full-offsets-npose.pkl" \
  --scale "$BVH_SCALE" --human-height "$HUMAN_HEIGHT_M" \
  --start "$START_FRAME" --end "$END_FRAME" --frame-step "$FRAME_STEP" \
  --offsets-file "$OFFSETS_FILE" --arm-mode npose \
  --calibration-bvh-file "$NPOSE_BVH" \
  --calibration-start-seconds "$NPOSE_START_S" --calibration-seconds "$NPOSE_DURATION_S" \
  --calibration-output "$SESSION_DIR/$ROBOT-full-arm-calibration.json" \
  --accept-reviewed-calibration-window
```

`quality_approved` reflects supplied offsets/preview flags; it is not an independent quality test. An offsets file's existence is not proof of its quality. Complete T1 offsets remain a commissioning task.

### Arm/body clearance correction

After visual review, run the robot-specific gate on the N-pose output. It corrects distal arm/hand versus torso/leg proxies only:

```bash
"$MOCAP_WS/venv/bin/python" tools/retarget/project_robot_arm_collisions.py \
  --gmr-root "$MOCAP_WS/GMR" --robot "$ROBOT" \
  --motion "$SESSION_DIR/$ROBOT-npose.pkl" \
  --output "$SESSION_DIR/$ROBOT-clearance.pkl" \
  --report "$SESSION_DIR/$ROBOT-arm-clearance.json"
```

If constraints or drift ceilings fail, keep the report and choose a feasible interval or improve mapping; do not bypass the gate.

### G1/T1 comparison

Repeat the retarget steps with the other robot using the **same source, frame interval, scale, height and output rate**, with separate output names. G1 has 29 hinges (`nq=36`); the T1 mocap asset has 27 active hinges (`nq=34`) despite the registry key `booster_t1_29dof`. Do not copy arrays between them. GMR is stateful: retarget every source frame and downsample the output afterwards (`--frame-step` does this), never the input. Render each robot sequentially (two concurrent MuJoCo recorders cannot share EGL contexts) and combine the MP4s with an `hstack` filter if a side-by-side is wanted. The T1 mapping's origin and conventions are in [`retarget_configs/bvh_xsens_to_booster_t1_29dof.md`](../retarget_configs/bvh_xsens_to_booster_t1_29dof.md).

### G1 Docker BVH shortcut

If the live image and display are configured, this caches and loops G1 motion (not T1):

```bash
docker/gmr-live/run-offline.sh "$MOTION_BVH" --scale "$BVH_SCALE" --start "$START_FRAME" --frame-step "$FRAME_STEP"
```

Without these overrides it skips frame 0 and takes every fourth frame, which matches 240 Hz exports with an injected first frame. Outputs go to ignored `artifacts/xsens-offline/`. See [Docker images](#docker-images).

### Operator console

The Setup workspace (`--setup`, started by the **Mocap Setup** menu entry) runs the simulation-only jobs of the setup guide. Without `--setup` the same program is a passive status console: it does not launch Xsens, the UDP receiver or MuJoCo. Its other modes are not needed for normal use: `.venv/bin/mocap-g1-console --demo --open` shows synthetic status, `--playback <mp4>` reviews a recorded video for G1 or T1, and interactive virtual-G1 simulation needs a **G1 qpos NPZ**, not a `.pkl` and not T1 arrays:

```bash
.venv/bin/mocap-g1-console --simulation '/absolute/path/to/g1-qpos.npz' --open
```

In every mode hardware authority stays locked; the console never connects to a robot.

## Full troubleshooting

| Symptom | Check first | Recovery and success evidence |
| --- | --- | --- |
| `externally-managed-environment` | Are you using system pip? | Activate `.venv`; install there. Do not use `sudo pip`. |
| `No module named pip` | Existing venv was created without pip | Run the `ensurepip` repair, then rerun setup. |
| Dependency cannot install | Interpreter version, package range and full pip error | Use Python 3.11/3.12 for a fresh setup; retain project version ranges. |
| GMR absent after project install | Which venv is running the command? | Use `$MOCAP_WS/venv/bin/python` for recorded scripts; require `runtime_ok: true`. |
| Prepared checkout but runtime fails | Bootstrap was prepare-only or interrupted | Run full bootstrap; read the import error in its check report. |
| Wrong GMR revision/patch failure | Existing external checkout belongs to another workflow | Bootstrap in a new external folder; preserve the old one. |
| `H0007` / missing Xsens license | Chosen physical capture PC, product and validity | Resolve native activation with Xsens Support. Never copy Sentinel storage to Linux. |
| No UDP traffic | Active preview/playback, destination, port, Ethernet link, route and scoped firewall | Correct the Windows destination/source filter; acceptance must pass. Ping alone is insufficient. |
| Only MXTP12/MXTP13 | Streamer is sending metadata/scaling | Enable body quaternion and joint streams and start actual preview/playback. |
| Unexpected source / no accepted frames | DHCP changed Windows IP or wrong adapter selected | Update the Windows IP in Setup → 03 Session (or rerun `new_session.sh`); rerun the stream check. |
| Bad segment count or joint pairing | Props/fingers, wrong preset, multiple actors, missing MXTP20 or an unsupported connection layout | Send 23 body segments and paired joint angles for one character; the decoder supports the observed Xsens 2026.9 28-record layout and legacy 22-record streams. Rerun acceptance; see the [commissioning record](live-xsens-commissioning-2026-10-14.md). |
| Rate/loss gate fails | Capture FPS, LAN congestion and machine load | Use the wired capture LAN; reduce load; keep the thresholds and repeat a full test. |
| `Address already in use` | `ss -lunp` and known Docker receivers | Stop the previous listener; verify the port is free; run one receiver. |
| Docker permission denied | Normal-user Docker access | Follow Docker's post-install access steps, log in again, verify `docker info`. |
| Docker cannot select GPU driver | NVIDIA Container Toolkit/runtime configuration | Register the runtime (see Docker, NVIDIA and display); retry. |
| Container `PermissionError` reading `/opt/mocap_humanoid/...` | Old image preserved owner-only source permissions | Rerun the live installer to rebuild. Do not run the viewer as root. |
| Xauthority/socket missing; viewer will not open | Local X11 session, `DISPLAY`, `XAUTHORITY`, `/tmp/.X11-unix` | Use the real desktop session; no `xhost +`; SSH headless is not supported. |
| Live startup times out | Receiver waiting but source not streaming required frames | Resume preview/playback and rerun after the port is free; do acceptance first. |
| Hands pass through the hips (live) | Arm abduction set too low | Use the default `--arm-abduction` (K1 22, G1 10, T1 22) or raise it a few degrees. |
| Feet sink or float (live) | No neutral calibration yet | Press N with the performer standing still, or use `--calibrate-after`. |
| Arms lag or cut through the body (live G1) | `--g1-hardware-envelope` is on | Leave it off in simulation; it previews real-robot speed limits. |
| Huge dropped-frame count or wrong heading after Xsens restart | Sender reset its frame counter or recalibrated | Relaunch the live viewer, reconfirm source/calibration and rerun acceptance. R alone is not a heading reset. |
| Motion too small/large | Export units, scale and measured height | Centimetres require `0.01`; fix units and rerun to a fresh output. |
| Arms move opposite to source or twist badly | Source calibration, preset, offsets and robot-specific arm mapping | Review the N-pose window; use robot-specific alignment; keep the preview label without complete offsets. |
| Wrong first pose or playback speed | Exporter bind frame, Frame Time and downsample settings | Select the real start frame and step; preserve true duration. |
| `.pkl` rejected by console / T1 NPZ wrong width | Console mode expects a different format/model | Use passive MP4 for either robot; only G1 qpos NPZ for `--simulation`. |
| Collision gate refuses output | Motion infeasible within drift/limit budget | Inspect the report, improve mapping/calibration or choose another interval. |
| Only `.partial` output | Render/retarget interrupted or failed | Retry with a fresh output name; require a completed file and successful exit. |
| Console live indicators seem unrelated | Demo uses synthetic status, playback is passive | The console is not connected to the live UDP viewer. |

To inspect traffic without a second UDP receiver (reveals network metadata; do not publish raw captures):

```bash
sudo tcpdump -ni '<capture-interface>' -c 20 'udp dst port 5555'
```

When asking for help, send the route, OS/Python versions, project/GMR revisions, sanitized doctor output, exact command and error, report path and whether Windows playback was active. Do not send activation keys, raw performer data or Xauthority contents.

## Shutdown and session evidence

If a terminal is lost, stop only the exact container you started (`No such container` means it already exited), then confirm UDP 5555 is free:

```bash
docker stop --timeout 5 gmr-xsens-live
docker stop --timeout 5 xsens-stream-5555
docker stop --timeout 5 gmr-bvh-loop
ss -lunp | rg ':5555\b'
```

Do not uninstall a license, delete source recordings or purge images as part of normal shutdown. For a live restart: reconfirm DHCP addresses, suit fit/calibration and display, update the session (Setup → 03, or `new_session.sh`), rerun the system check and stream check, then start one receiver.

Keep these together in `SESSION_DIR` (`session.env` already records the values used):

- Date/session ID, project Git revision, Windows/Xsens/firmware versions, Python and GPU/driver versions.
- Measured actor dimensions and calibration outcome; original `.mvn`, BVH/MVNX/actor provenance and source hashes, under your capture consent/access controls.
- Sender/destination/port, intended character and real acceptance JSON; whether ROS was used.
- Robot, IK config, GMR revision, runtime lock and calibration JSON/hash.
- Retarget interval, scale, output FPS, preview flags, correction reports, completed videos and review notes.
- Each failed, skipped or unverified step with its reason. Never record missing evidence as a pass.

The outcome can be **installed**, **real stream accepted**, **recorded retarget generated** and/or **simulation visually reviewed**, depending on the evidence collected. None grants physical robot authority; connecting a real robot is a separate, later stage outside this guide.

### Before handing this to a demo operator

- Deliver the complete reviewed revision, guide, Docker image identities and an approved recording fallback, visibly labelled as recorded playback.
- On the actual demonstration PCs, pass real-stream acceptance and inspect neutral stance, each arm, both arms, knee bends, steps and turns.
- Rehearse a 30-minute live session, three cold starts and stream-loss recovery with the operator who will run the demo. These are proposed release criteria, not completed results.
- Have that operator complete a full demonstration from the guide without developer help. Measure person-to-screen delay separately before making latency claims.

## Maintainer notes

| Reference | What it establishes |
| --- | --- |
| [Project README](../README.md) | Supported scope and repository map |
| [Installer](../tools/setup/install.sh) and [session helper](../tools/setup/new_session.sh) | Setup questions, defaults and session file |
| [Bootstrap tool](../tools/bootstrap_g1_workspace.py) | GMR pin, patches, environment and lock checks |
| [Acceptance wrapper](../tools/xsens/live_operator.sh) | Sequential stages, 5555 contract and gate settings |
| [Live viewer](../tools/xsens/live_g1_sim.py) and [live robots](../mocap_g1/live_robots.py) | G1/T1 layouts, screening, arm abduction, N calibration, freshness and latency |
| [Replayer](../tools/mxtp02_replayer.py) | Synthetic or real-BVH (`--xsens-bvh`) MXTP02 streams for testing without the suit |
| [Generic recorded runner](../tools/retarget/retarget_robot_recorded.py) | Robot-specific IK, offsets/N-pose modes, atomic outputs and metadata |
| [Generic renderer](../tools/retarget/render_robot_motion.py) | Motion validation and finite atomic video exports |

Screenshot slots W01–W05 and L01 remain placeholders until captured on the licensed Windows workstation or installed calibration runtime. Remove secrets/identity, put the image in `docs/engineer-guide-assets/`, and replace the matching `SCREENSHOT:` line with Markdown image syntax.

The interactive page is generated from the guide. Rebuild it after editing the guide or its template:

```bash
python3 tools/docs/build_engineer_guide.py
python3 tools/docs/build_engineer_guide.py --check
```

The HTML embeds its CSS, JavaScript and images and needs no network. Open it directly, or serve the checkout with `python3 -m http.server 8767 --bind 127.0.0.1` and visit `/docs/engineer-setup.html` for clipboard support and working source links.
