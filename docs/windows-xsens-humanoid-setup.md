# Xsens Humanoid on Windows

Use native Windows for Xsens capture when the available entitlement is a single **System
License**. Keep retargeting and ROS 2 ingestion on a separate Linux machine or inside the project
containers.

```text
Windows capture PC (licensed)                 Linux/GMR receiver (unlicensed)
source IPv4 confirmed per session             destination IPv4 detected per session
Xsens Humanoid + MVN Link
        |-- ROS preset, UDP 5555 ------------> acceptance -> ROS 2 -> GMR/MuJoCo
        `-- .mvn recording -> 3DS Max BVH ---> offline G1 retargeting
```

## Licence boundary

An Xsens product name without a `-D` (dongle) or `-N` (network) suffix is a System License: no
dongle, and documented by Xsens as tied to a specific PC.

- Choose the permanent Windows capture PC before activation.
- Do not activate the same key in Docker, WSL, a VM, or a second laptop.
- Do not put a product key, activation file, Sentinel storage, licensee identity, or screenshot of
  the licence email in Git, a Docker image, a command line, or shared logs.
- A copied Docker image does not carry the right to use the licence on another computer.
- Before replacing Windows, changing the capture PC, or attempting a rehost, contact Xsens Support
  and confirm that this entitlement permits the move. The Xsens downloads page provides the RUS
  utility for supported rehosting workflows.

If concurrent use on multiple laptops is required, ask Xsens for a network licence (`-N`), a
portable dongle licence (`-D`), or additional System License seats.

## 1. Prepare the Windows capture PC

Use a supported 64-bit Windows release and an NVIDIA GPU/driver that meet the current Xsens
Humanoid system requirements. Apply Windows updates, install the NVIDIA driver, and verify that the
PC has enough free space for the raw recordings before connecting the suit.

Download only from the official
[Xsens software and documentation page](https://www.xsens.com/support/software-documentation):

1. **Xsens Humanoid for Windows**
2. **Xsens License Manager for Windows**
3. The MVN Link or Xsens Link user manual that matches the delivered hardware

The Sentinel network-licence runtime is not required for this System License. Do not install a
dongle/network runtime merely to compensate for a failed System License activation.

## 2. Install and activate once

1. Install Xsens Humanoid and the Xsens License Manager natively in Windows.
2. Start the License Manager on the chosen physical Windows PC.
3. Select the on-screen System License/software-key activation workflow and enter the product key
   interactively. Do not paste it into PowerShell, Command Prompt, a script, or this repository.
4. Confirm that License Manager shows the expected Xsens Humanoid product, expiry, and one local
   protection key.
5. Close License Manager, start Xsens Humanoid, and confirm that the Sentinel `H0007` missing-key
   error is gone.

Licence validity is set by the dates in the Xsens order; delaying activation does not extend it.
Record a sanitized activation result for the project log, but omit the product key, protection-key
ID, email address, and customer identifiers.

## 3. Connect and validate the MVN Link kit

Follow the vendor manual for the router, Body Pack, sensors, suit placement, body dimensions, and
calibration. Keep the Windows capture PC and the Linux/GMR receiver on the same trusted LAN.

Before recording real takes:

1. Confirm that all expected body sensors are detected and charged.
2. Create the actor with measured body dimensions rather than estimates.
3. Complete the vendor calibration and save only sanitized calibration evidence.
4. Record a short neutral stance, arm raise, squat, walk, and turn.
5. Play the take back in Xsens Humanoid before exporting or streaming it.

Store raw `.mvn` takes outside Git. Back them up according to your data-handling policy.

## 4. Live stream from Windows to Linux

The UDP port and protocol are fixed; DHCP host addresses are confirmed per session:

| Role | Address |
|---|---|
| Windows Xsens source | Current Xsens-facing Ethernet IPv4 from `ipconfig` |
| Linux receiver/destination | Current Xsens-facing IPv4 from the dual-network checker |
| UDP destination port | `5555` |

In Xsens Humanoid, open **Options -> Network Streamer**, select the ROS preset, and set its
destination to the detected Linux Xsens-facing IPv4, UDP `5555`. The preset must emit both the 23-segment
**Position + Orientation (Quaternion)** body stream (MXTP02) and the 22-joint joint-angle stream
(MXTP20). Disable prop and finger segments in MXTP02 so the body packet retains the strict
23-segment layout required by the acceptance and GMR paths.

Xsens's ROS 2 guide mentions `9736`, while the pinned upstream ROS driver defaults to `9763`. Those
are reference defaults only. Do not use either for this route: the Windows destination and every
active Linux receiver must use `5555`.

Run `ipconfig` on Windows and the checker on Linux at the start of each session. Store the confirmed
values only in the current operator shell:

```bash
WINDOWS_XSENS_IP=...  # current Windows Xsens-facing Ethernet IPv4
LINUX_XSENS_IP=...    # current Linux Xsens-facing IPv4 reported by the checker
```

The Linux address is a Network Streamer destination setting; it is not encoded by the receiver
wrapper.

If UFW is installed, inspect it first with `sudo ufw status numbered`. Only when UFW is active and
no equivalent rule exists, add the scoped host-to-host rule:

```bash
sudo ufw allow from "$WINDOWS_XSENS_IP" to "$LINUX_XSENS_IP" port 5555 proto udp
```

### Build the two Linux images

From the repository root on the Linux receiver:

```bash
docker build -t xsens-ros2:jazzy docker/xsens-ros2
docker build -t gmr-live:bb1bbe4 -f docker/gmr-live/Dockerfile .
```

The GMR viewer requires NVIDIA Container Toolkit and a working host X11 session. Its launcher passes
the NVIDIA GPU and X11 socket into the container, drops Linux capabilities, runs as the invoking
UID/GID, and keeps optional recordings under the ignored `artifacts/xsens-live/` directory. CUDA is
not required for retargeting; the GPU accelerates MuJoCo/OpenGL rendering.

### Run the stages in order

Keep the Network Streamer running, but start only one Linux receiver at a time. Acceptance, ROS 2,
and GMR all bind UDP 5555 and cannot share it.

```bash
export XSENS_SOURCE_IP="$WINDOWS_XSENS_IP"
export XSENS_UDP_PORT=5555
tools/xsens/live_operator.sh acceptance
```

Wait for `PASS`, then run ROS 2:

```bash
tools/xsens/live_operator.sh ros2
```

1. `acceptance` records a 30-second aggregate report, requires paired MXTP02/MXTP20 frames, and
   rejects traffic whose source is not the explicit `XSENS_SOURCE_IP`.
2. Exit the acceptance listener, then run `ros2`. Confirm nonzero `/link_states`, `/joint_states`,
   `/tf`, `/com`, and `/diagnostics` traffic before stopping the ROS container. The ROS node does
   not filter source IP, so retain the scoped firewall rule or equivalent network isolation.
3. Run `gmr` only after ROS has exited. It filters for the explicit `XSENS_SOURCE_IP`, ignores the paired
   MXTP20 packets, keeps the newest valid MXTP02 frame, retargets to `unitree_g1`, and opens the
   MuJoCo viewer through host X11/NVIDIA.

Inspect ROS traffic from another terminal with, for example:

```bash
docker exec xsens-stream-5555 /entrypoint.sh ros2 topic echo --once /diagnostics
docker exec xsens-stream-5555 /entrypoint.sh ros2 topic echo --once /link_states
docker exec xsens-stream-5555 /entrypoint.sh ros2 topic hz /link_states
```

Stop ROS with `Ctrl+C`, verify `ss -lunp | rg ':5555\b'` prints nothing, then run GMR using the
measured performer height:

```bash
HUMAN_HEIGHT_M=1.75  # example only; replace with the measured height
ROBOT=unitree_g1     # or booster_t1_29dof
tools/xsens/live_operator.sh gmr --robot "$ROBOT" --human-height "$HUMAN_HEIGHT_M"
```

Once the robot moves, have the performer stand still, arms relaxed, and press N in the MuJoCo
window (or add `--calibrate-after 5`): this puts the robot's feet on the floor and, for the T1,
calibrates arm orientation.

Each stage runs in the foreground. If a terminal is lost, use
`docker stop --timeout 5 xsens-stream-5555` for ROS or
`docker stop --timeout 5 gmr-xsens-live` for GMR. Both launchers use `--rm`, so the stopped
container is removed while the reusable image remains installed. None of these receivers is
configured as a boot service. The current ROS wrapper does not mount persistent rosbag storage;
add a reviewed writable mount before recording real data.

The source IP has no silent default: `acceptance` and `gmr` require the current
`XSENS_SOURCE_IP`. The UDP port defaults to `5555`; the Linux destination IP remains a Windows
Network Streamer setting.

```bash
XSENS_SOURCE_IP="$WINDOWS_XSENS_IP" \
XSENS_UDP_PORT=5555 \
tools/xsens/live_operator.sh acceptance
```

### Dual-network Xsens route and checker

The Windows Xsens router and the normal Linux Wi-Fi router both use `192.168.1.0/24`, but they are
different networks. Their gateway MAC addresses distinguish them; obtain the Xsens router MAC from
its label/admin page rather than storing that site-specific identifier in Git. Linux can keep
Internet access over Wi-Fi while using Ethernet for Xsens, but
the overlapping networks require a specific host route; do not rely on automatic route priority.

Connect the Linux Ethernet adapter to the Xsens router while leaving Internet Wi-Fi on its own
adapter (find both names with `ip -br link` and `nmcli device status`; the helper has no
site-specific defaults and refuses to run without them). On Windows, run
`ipconfig` and confirm its current Xsens-facing Ethernet IPv4. From the repository root, set the
shell variables and run:

```bash
WINDOWS_XSENS_IP=...  # current Windows Ethernet IPv4
XSENS_ROUTER_MAC=...  # aa:bb:cc:dd:ee:ff from the router label/admin page
XSENS_WIFI_INTERFACE=...      # Internet adapter, e.g. from `ip -br link`
XSENS_ETHERNET_INTERFACE=...  # adapter cabled to the Xsens router
export XSENS_WIFI_INTERFACE XSENS_ETHERNET_INTERFACE
XSENS_SOURCE_IP="$WINDOWS_XSENS_IP" XSENS_GATEWAY_MAC="$XSENS_ROUTER_MAC" \
  tools/xsens/setup_xsens_dual_network.sh
```

The setup owns the dedicated `Xsens Ethernet` NetworkManager profile. Keep unrelated routes and DNS
settings in other profiles. It keeps the default route and DNS on Wi-Fi, makes Ethernet non-default,
adds a persistent `/32` route for the confirmed Windows address, activates Ethernet, verifies both routes, and then runs
the checker. The checker refuses the wrong router, detects the Ethernet address to enter as the
Windows Network Streamer destination, checks UDP 5555 ownership and firewall visibility, waits for
live preview/playback, and writes an aggregate acceptance report under ignored
`artifacts/xsens-live/`. Neither script changes firewall rules, bridges the networks, or enables
Internet sharing. Use `--configure-only` when MVN is not ready to stream yet.
If setup validation fails, the tool-owned profile is removed so partial persistent routing is not
left behind.

If no viewer opens, first verify `DISPLAY`, `XAUTHORITY`, the X11 socket, NVIDIA Container Toolkit,
and `docker image inspect gmr-live:bb1bbe4`. To record the GMR review, add `--record-video`; the
default path inside the ignored runtime directory is `g1-live.mp4` or `t1-live.mp4`.

If Windows and GMR are on the same physical machine, prefer the offline BVH workflow below. Docker
Desktop/WSL 2 networking adds a NAT boundary and is not the project's validated live path. Native
Windows remains responsible for Xsens hardware and licensing; never mount Sentinel or licence data
into either Linux container.

## 5. Offline Windows-to-GMR workflow

Keep the original `.mvn` as the capture master. GMR does not parse proprietary `.mvn` or `.mvnx`
files directly.

Therefore, transferring only a manually recorded `.mvn` to Linux is not enough. Before transfer,
open the take on the licensed Windows workstation and export its matching 3ds Max-preset BVH. The
`.mvn` remains valuable for re-export and Network Streamer replay, but the BVH is the offline GMR
input. The minimum recording package per take is the `.mvn`, a matching `.mvnx`, the 3DS Max BVH
export and the actor/body-dimension provenance.

1. Open or replay the take in Xsens Humanoid.
2. Export a matching `.mvnx` for archival/inspection and **BVH using the 3DS Max preset** for GMR.
   The 3DS Max layout is the only Xsens BVH layout supported by the pinned GMR importer.
3. Preserve the export units and note them in the job metadata. The current centimetre-based path
   uses a `0.01` scale.
4. Transfer the BVH to the external runtime/data workspace, not into Git.
5. Review and save the complete Xsens-to-G1 offsets calibration before producing deliverable
   output. Zero or missing offsets are only sufficient for structural smoke tests.
6. Retarget and render the take from the Setup console (setup guide steps 08–10), then visually
   review the source and robot motion together.

For a Windows-only workstation, install Docker Desktop with WSL 2 for the Linux GMR environment and
process exported BVH files offline. Native Windows remains responsible for Xsens hardware,
licensing, calibration, recording, and export; do not attempt to copy the Windows activation into
WSL or a container.

## 6. Acceptance record

Capture the following without licence secrets or personal data:

- Windows version, GPU model, NVIDIA driver, and Xsens Humanoid version
- hardware/firmware versions and detected sensor count
- calibration result and test-take names
- Network Streamer destination, packet rate/loss/order, and character ID
- original `.mvn` checksum and exported 3DS Max BVH checksum
- G1 retargeting configuration, offsets calibration revision, score report, and rendered review

The executable day-one checks are the Setup console's system check and 30-second stream check
([setup guide](engineer-setup-guide.md) steps 03 and 06).
