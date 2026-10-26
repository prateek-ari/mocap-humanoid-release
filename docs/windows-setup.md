# Windows: Xsens suit → MuJoCo simulation on one PC

> **Live simulation only? Use the app instead.** The [Mocap Humanoid installer and desktop
> app](windows-app.md) installs everything offline and runs the live K1/G1/T1 simulation from one
> window. This ZIP route stays for the recorded BVH route and for development.

Install and run the live suit → MuJoCo K1, G1 or T1 simulation (and the recorded BVH route) natively on
the **same Windows PC that runs Xsens Humanoid**. No Linux PC, Docker or WSL. After a one-click
install, everything is done in the **Setup console** in your browser.

> **Status (2026-10-13):** new and not yet validated on a real Xsens Windows PC. On a
> Windows test machine without a GPU or suit, the one-click install and its rerun, the Setup
> console, a console stream check of synthetic Xsens packets on `127.0.0.1:5555`, and starting
> and stopping a live job all pass. Not yet verified: the MuJoCo
> viewer window and MP4 recording on a real GPU, real Xsens Humanoid streaming, and capture
> performance with both programs on one PC. The first session on the capture PC is the
> acceptance test.

The Ubuntu route in the [engineer setup guide](engineer-setup-guide.md) stays the reference
setup. Use this guide when the Xsens PC is the only PC.

## What runs where

```text
Windows PC
  Xsens Humanoid ── Network Streamer ── UDP 127.0.0.1:5555 ──┐
                                                            │
  Setup console (browser, http://127.0.0.1:8784/)           │
    ├── Stream check  (tools/xsens/stream_acceptance.py) ◄──┤  one at a time:
    └── Live viewer   (tools/xsens/live_g1_sim.py, MuJoCo) ◄┘  both use UDP 5555
```

| Works on Windows | Not on Windows |
| --- | --- |
| Live suit → MuJoCo, Booster K1, Unitree G1 and Booster T1 | ROS 2 stage (`live_operator.sh ros2`) |
| Stream check, neutral calibration (N), MP4 recording | Physical robot control |
| Recorded route: BVH check, retarget, watch/record MP4 | `make` targets and the bash tools |
| Session values, report saving, system check | Docker images |

## Requirements

- Windows 10 (22H2) or Windows 11, 64-bit, with Xsens Humanoid installed and its licence active
  (see [Xsens Humanoid on Windows](windows-xsens-humanoid-setup.md)).
- A GPU with a current driver. NVIDIA is recommended; Intel and AMD also work for MuJoCo.
- About 5 GB of free disk space and Internet access during the install.
- **No administrator rights are needed.** Nothing is installed system-wide: Python, uv and (if
  missing) Git go into `%LOCALAPPDATA%\mocap-humanoid\`.

## 1 · Get the project

Put the project in a plain local folder such as `C:\Users\<you>\mocap-humanoid-release`. Avoid OneDrive
folders (Desktop or Documents are often synced), which can lock files during the install.

- **Download ZIP (no Git needed):** on GitHub open the repository, choose the branch, then
  **Code → Download ZIP**. Right-click the ZIP → **Extract All…** and pick the folder above.
- **Or clone it** if Git is installed:
  `git clone https://github.com/prateek-ari/mocap-humanoid-release.git`

## 2 · Install (one click)

In the project folder, double-click **`Install-Windows.cmd`**.

If Windows shows *"Windows protected your PC"* (because the file came from a download), click
**More info → Run anyway**. The installer then unblocks the rest of the project files itself.

The installer window shows each stage. It:

1. Downloads a pinned **uv** and **Python 3.13** (checked against fixed SHA-256 hashes).
2. Uses Git from `PATH`, or downloads a pinned portable **MinGit** if there is none.
3. Creates the project environment `.venv` in the project folder (Setup console and checks).
4. Creates the runtime in `C:\Users\<you>\mocap-runtime`: GMR at the pinned commit with this
   project's patches, plus MuJoCo, mink and the other pinned packages
   (`constraints/windows-runtime.txt`).
5. Runs the project checks and the stream self-test.
6. Adds **Mocap Setup** to the Start menu and the desktop, then opens the Setup console.

The first install takes several minutes. It ends with `PASS`. If a stage fails, fix the printed
cause and double-click `Install-Windows.cmd` again: finished stages are reused. The full log is in
`%LOCALAPPDATA%\mocap-humanoid\logs\`.

Python 3.13 is required on Windows: older versions have a 15.6 ms clock there, which is about one
whole frame of the 60 Hz live loop. The installer always uses its own Python 3.13, whatever else
is installed.

Command-line options (from PowerShell in the project folder), for scripted installs:

```powershell
.\Install-Windows.cmd -Unattended -NoOpen          # no questions, don't open the browser
.\Install-Windows.cmd -Workspace D:\mocap-runtime  # another runtime folder
.\Install-Windows.cmd -NoShortcuts                 # skip Start menu and desktop shortcuts
```

If you choose another runtime folder, set the same folder as **Runtime folder** in the Setup
console's Session step.

## 3 · Xsens Humanoid: stream to this PC

In Xsens Humanoid open **Options → Network Streamer** and add a stream:

| Setting | Value |
| --- | --- |
| Destination | `127.0.0.1` (this PC) |
| Port | `5555` |
| Protocol | UDP |
| Preset | ROS: Position + Orientation (Quaternion) **and** Joint Angles |
| Prop and finger segments | Off (the receiver needs the 23-segment body layout) |

Keep the stream enabled. Release notes for Xsens 2026.6 say a changed stream setting is not
always applied at once: disable and re-enable the stream after editing it.

## 4 · Use the Setup console

Open **Mocap Setup** from the Start menu or the desktop. It starts the console in the background
(no window) and opens `http://127.0.0.1:8784/`. On Windows the steps are numbered 1, 2, 3…

**Live route**

1. **Session.** Route **Live**, the robot (Booster K1, the default; Unitree G1 or Booster T1), the performer's measured
   height, and **Xsens Humanoid runs on: This PC**. Click **Run system check**: it checks Python,
   the runtime, an OpenGL test render and that UDP 5555 is free. Then **Save session**.
2. **Xsens: suit and streaming.** Calibrate the suit in Xsens Humanoid as usual and set up the
   stream from step 3 above. Nothing to click here.
3. **Check the stream.** Start streaming in Xsens, then click **Run 30-second check**. It must end
   with `PASS` (at least 300 body frames, 50 Hz, under 1 % loss, joint angles present).
4. **Live simulation.** Click **Start live simulation** and keep streaming. A MuJoCo window opens
   and the robot follows the performer. Have the performer stand still with arms relaxed and press
   **N** in the MuJoCo window (or tick **Auto-calibrate after** a few seconds) to put the feet on
   the floor and, for the T1, calibrate the arms. Tick **Record video** to save an MP4 into the
   session folder.
   **Stop** closes the viewer and finishes the video.
5. **Finish.** Click **Save report**.

**Recorded route** (BVH exported from Xsens Humanoid with the 3DS Max preset): Session (route
**Recorded**), **BVH files** (check), **Retarget**, **Watch and record**, **Finish**. Session folders
are created under `C:\Users\<you>\mocap-sessions\`.

Only one stream receiver can use UDP 5555 at a time, so the stream check and the live simulation
never run together. The console enforces this.

To stop the console itself (for example before updating the project), run in PowerShell from the
project folder:

```powershell
.\.venv\Scripts\python.exe tools\windows\console_launcher.py --stop
```

## 5 · Optional: Xsens on another PC

The console also supports a second PC running Xsens and streaming to this Windows PC over the LAN.
In the Session step choose **Xsens Humanoid runs on: Another PC**, enter that PC's IPv4 (from
`ipconfig` there) and choose this PC's IPv4. In Xsens set the Network Streamer destination to this
PC's IPv4, port 5555. When Windows Firewall asks, allow Python on **Private** networks (or add an
inbound UDP 5555 rule for that network). Loopback streaming on one PC needs no firewall change.

## Troubleshooting

| Symptom | Cause and fix |
| --- | --- |
| "Windows protected your PC" | The ZIP came from the Internet. **More info → Run anyway**. |
| Install stops at a download | Check Internet access, proxy or antivirus blocking `github.com`/`pypi.org`, then run the installer again. |
| Stream check: no UDP traffic | Is the stream enabled in Xsens? Destination `127.0.0.1`, port `5555`, UDP. Disable and re-enable the stream after edits. |
| Stream check: unexpected source | The console expects `127.0.0.1`. If Xsens streams to this PC's LAN IP instead, set the destination to `127.0.0.1`. |
| Stream check: no joint angles | Enable the Joint Angles datagram in the Network Streamer preset. |
| UDP 5555 busy | Another receiver is running (a second console, an old viewer). Stop it; `netstat -ano -p UDP \| findstr :5555` shows the process ID. |
| System check: OpenGL failed | Update the GPU driver from the vendor. Remote Desktop sessions often lack OpenGL: use the PC's own screen. |
| MuJoCo window is slow while capturing | Both programs share the GPU and CPU. Close other programs and keep the laptop on mains power with a high-performance power plan. If you ticked **Record video**, untick it (it is off by default). |
| The console page does not open | Open Mocap Setup again. The console log is `%LOCALAPPDATA%\mocap-humanoid\setup-console.log`. |

## Updating

Stop the console (above), replace the project folder with the new version (or `git pull`), and
double-click `Install-Windows.cmd` again. Session folders and the runtime are kept.

## Uninstall

Stop the console, then delete:

- the project folder,
- `C:\Users\<you>\mocap-runtime` (the runtime),
- `%LOCALAPPDATA%\mocap-humanoid` (uv, Python, MinGit, logs),
- the **Mocap Setup** shortcuts (desktop and Start menu).

Keep `C:\Users\<you>\mocap-sessions` if you need the recordings and reports.

## Limits and differences from the Linux route

- **One PC for capture and simulation.** Xsens Humanoid and MuJoCo share one CPU and GPU. This
  has not been measured yet; Xsens's own requirements are modest. Watch for dropped frames in Xsens
  during the first sessions. Video recording is off by default and only adds load when ticked.
- **No container isolation.** The Linux live viewer runs in a pinned Docker image; on Windows the
  same pins are installed into `mocap-runtime\venv` from `constraints/windows-runtime.txt`.
- **Upstream GMR is tested on Ubuntu only.** The project installs GMR without its optional
  dependencies (torch, SMPL-X, OpenCV) because the live and recorded paths do not use them.
- **No ROS 2 and no physical robot** on Windows. Keep the Linux setup for those.
- **Licensing:** the Xsens System License stays on this PC; the simulation is separate software
  and needs no Xsens licence.
