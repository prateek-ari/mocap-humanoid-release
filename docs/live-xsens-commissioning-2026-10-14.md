# Real Xsens reception and paired G1/T1 preview, 14 October 2026

The Windows Xsens stream now passes the Linux acceptance check, and the same
performer can drive the Unitree G1 and Booster T1 side by side in one MuJoCo
window. The [offline block diagram](live-xsens-g1-t1-diagram.html) explains the
working path, calibration and controls.

## What failed and why

The Windows Network Streamer initially targeted `127.0.0.1`, so its packets
stayed on Windows. Changing the destination to the Linux Ethernet address
delivered the data. The commissioning session used Windows `192.168.1.167`
and Linux `192.168.1.24`, UDP `5555`; these are observed session addresses,
not defaults. Linux-to-Windows ping did not receive replies, but Windows
answered Ethernet address-resolution requests and sent the UDP stream.

The next failure was in our MXTP20 decoder. It had been validated with
synthetic 22-record packets that used zero local point IDs and a zero body
count. Xsens Humanoid 2026.9 sent a 584-byte packet containing 28 angle records:
22 body-tree connections and six additional connections, with body count 23
and real local point IDs. The checker rejected these valid body-tree records.

The [Xsens streaming specification](https://www.xsens.com/hubfs/Downloads/Manuals/MVN_real-time_network_streaming_protocol_specification.pdf)
defines connection IDs as `256 * segment ID + local point ID` in sections
2.5.10 and 2.7.1. Section 2.4.1.7 retains the body segment count in non-body
packets. The decoder now preserves the point IDs and accepts the verified
22/28-record layouts while retaining framing, body-tree, finite-value and
body-only checks. The six-record suffix is an explicitly supported observed
2026.9 layout, not an assumption that arbitrary extra joints are safe.

Suit calibration remains necessary for meaningful motion. It did not explain
the receiver's packet-format rejection. The Windows **ROS 2 preset** remained
selected for the successful session; that preset name does not mean a ROS 2
node sits between Xsens and the direct GMR viewer.

## Why the paired preview uses one receiver

Starting two existing live launchers would make both bind UDP 5555. The new
`tools/xsens/live_compare_sim.py` instead reads each newest body frame once,
converts it, and copies it to independent G1 and T1 GMR retargeters. The
retargeters keep separate calibration, filter and joint-range screening state.
Both branches use the same source frame counter and timestamp.

One MuJoCo scene contains the prefixed robot models and one floor. Display-only
Y offsets put G1 on the left and T1 on the right without changing the poses
used by inverse kinematics or joint screening. The included G1 model has 29
hinge joints; the T1 asset has 27 despite its `booster_t1_29dof` registry key.

The first neutral calibration runs five seconds after the shared preview
starts, on fresh stream data. Stand still with arms down. It aligns both
robots' feet with the floor, derives T1 arm-orientation offsets, and keeps G1's
configured arm offsets. **N** repeats calibration, **Space** toggles waist/arm
tracking on both, and **R** rearms both after a stream interruption.

## Run it

Install the live workflow first, as described in the
[setup guide](engineer-setup-guide.md). Save a live session with current
addresses and performer height. From the repository root:

```bash
source .venv/bin/activate
source session.env
bash tools/xsens/live_operator.sh acceptance
```

After acceptance passes and exits:

```bash
bash docker/gmr-live/run-compare.sh --calibrate-after 5
```

The comparison launcher uses the existing GMR image and mounts the comparison
script and shared live-loop code read-only. It does not require rebuilding the
image for this preview. It uses the same `gmr-xsens-live` container name as the
single-robot launcher, so they cannot run together. Close the window or press
Ctrl+C to stop; if needed, stop only this project's receiver:

```bash
docker stop --timeout 5 gmr-xsens-live
```

The Setup console's **Start live simulation** button still launches the saved
single robot. Start the comparison with the terminal command above. No
comparison receiver is installed as a boot service.

## Recorded evidence and limits

The successful 30-second real-stream check recorded:

| Check | Result |
|---|---|
| Valid body frames | 7,198 at 239.9 Hz |
| Valid joint frames | 7,197 |
| Body/joint packet loss | 0% |
| Body/joint pairing | 99.986% |
| Malformed datagrams | 0 |

The paired viewer processed 4,782 matching frames per robot and ran at about
57 FPS. Both five-second neutral calibrations completed. It recorded zero
malformed or reordered body packets and one bounded startup-overload event
while the models/viewer were being initialized. Queued frames were discarded
and live tracking recovered; no subsequent frame failure was observed in
that run. Skipping source frames to display the latest pose at about 60 Hz is
different from packet loss measured by acceptance at 240 Hz.

This is a kinematic simulation preview with joint-position and stale-stream
screening. It does not establish balance, anatomical retargeting quality,
validated hardware velocity limits, or permission to drive a physical robot.
No robot SDK transport was opened. Formal source-calibration evidence and
visual review of neutral stance, arms, knees, steps and turns remain separate
quality checks.

Before publishing the branch, `make hardware-free-checks` passed with the
external model workspace selected and skipped tests forbidden: 801 tests and
834 subtests, plus lint, safety-module type checks, guide freshness, codec UDP
loopback, stream self-test and the mock arm bridge. A seven-second headless
check using real GMR and synthetic UDP exercised the final paired CLI, both
five-second calibrations and finite shutdown; it processed 415 frames per
robot with zero malformed packets or overload events. The final label/camera
follow behavior also has model-backed tests. This synthetic check supplements
the real-session evidence above; it does not replace source-motion review.

Raw packets, recordings, local session files and console logs stay in ignored
local directories. The branch contains sanitized regression inputs, source
code, portable documentation and the HTML diagram. Host dependency installation,
disk cleanup, and the user's home-directory AGENTS.md edits are machine setup
changes outside this repository.
