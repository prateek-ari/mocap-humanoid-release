# Mocap Humanoid for Windows: install and use

One installer, one app. Install **Mocap Humanoid** on the Windows PC that runs Xsens Humanoid,
open it from the desktop, and drive the simulated Unitree G1 or Booster T1 from the live suit. No
Python, PowerShell, ZIP files or browser tabs are involved.

> **Status (2026-10-14):** new. The installer passed automated tests on a clean Windows test
> machine: install and upgrade, firewall rule, shortcuts, the app window, the stream lamp, stream
> check and live-simulation controls with synthetic Xsens data, and uninstall. Not yet verified:
> the MuJoCo window on a real GPU, real Xsens Humanoid streaming, and capture performance with
> both programs on one PC. The first session on the capture PC is the acceptance test.

## What you need

- Windows 10 (22H2) or Windows 11, 64-bit, with Xsens Humanoid installed and licensed
  ([Xsens Humanoid on Windows](windows-xsens-humanoid-setup.md)).
- A GPU with a current driver (NVIDIA recommended; Intel and AMD also work for MuJoCo).
- About 1 GB of free disk space. No Internet connection is needed to install.

## 1 · Install

1. Download [Mocap Humanoid for Windows 0.1.0.5](https://files.catbox.moe/jyfv0s.zip) (104 MB zip) and right-click it →
   **Extract All**. SHA-256: zip `60fb981ffe5f752301e63eb28d3b1af41b9e6cff23e1243257caddc37c5da7c2`, installer `4d9e67f2765205d242ebf2210047d751d615555d49c721f4d36b18bcfd8b1eed`.
2. Double-click `MocapHumanoid-Setup-0.1.0.5.exe`. The installer is not code-signed yet, so Windows may show *"Windows protected
   your PC"*: click **More info → Run anyway**.
3. Allow the administrator prompt, click **Install**, then **Finish** (leave *Launch Mocap
   Humanoid* ticked).

The installer puts everything in `C:\Program Files\Mocap Humanoid`: its own Python, MuJoCo, GMR
with the G1 and T1 models, and the app. It also adds:

- **Mocap Humanoid** to the Start menu and the desktop;
- a Windows Firewall rule that lets the app receive Xsens on UDP 5555 from another PC on private
  networks (Xsens on the same PC needs no rule; the app then listens on `127.0.0.1` only);
- an entry in **Settings → Apps** to uninstall it.

Everything has default settings: robot G1, performer height 1.75 m, Xsens on this PC, port 5555,
sessions in `C:\Users\<you>\mocap-sessions`. Change them in the app.

*Install for me only* (offered when you are not an administrator) works too, but adds no firewall
rule: with Xsens on another PC, Windows asks once to allow the app; choose **Private networks**.

## 2 · Xsens Humanoid: stream to this PC (once)

In Xsens Humanoid open **Options → Network Streamer** and add a stream:

| Setting | Value |
| --- | --- |
| Destination | `127.0.0.1` (this PC) |
| Port | `5555` |
| Protocol | UDP |
| Preset | ROS: Position + Orientation (Quaternion) **and** Joint Angles |
| Prop and finger segments | Off |

Keep the stream enabled. After editing it, disable and re-enable it so the change applies. The app
shows the same steps under **How to stream from Xsens Humanoid**.

## 3 · Use the app

Open **Mocap Humanoid** from the desktop. One window shows everything:

- **Xsens stream** (left): the lamp turns green, **Receiving**, as soon as Xsens streams, with the
  rate (about 60 Hz), loss and joint angles. Amber means stale or below target; red means another
  program holds UDP 5555.
- **Session** (right): robot, performer height, where Xsens runs, **Record video**, and
  **Auto-calibrate after** *n* seconds (for a performer who cannot reach the PC).
- **Start**: a separate MuJoCo window opens with the robot following the performer. Then have the
  performer stand still with arms relaxed and click **Calibrate** (or press **N** in the MuJoCo
  window). **Tracking off/on** and **Rearm** match Space and R in the MuJoCo window.
- **Stop** closes the MuJoCo window and finishes the video. Recordings and logs go into the session
  folder under `mocap-sessions`.
- **Tools**: the 30-second stream check (PASS needs 50 Hz, under 1 % loss and joint angles), the
  system check (Python, runtime, OpenGL test render, UDP port), the sessions folder and the log of
  the last run.

Closing the window stops the simulation and the app. Opening the icon again while the app runs
brings its window to the front.

## Troubleshooting

| Symptom | Cause and fix |
| --- | --- |
| "Windows protected your PC" | The installer is not signed yet. **More info → Run anyway**. |
| Lamp stays **Waiting for Xsens** | Is the stream enabled? Destination `127.0.0.1`, port `5555`, UDP. Disable and re-enable it after edits. |
| Lamp **Port in use** | Another program receives on UDP 5555 (an old viewer, or the older *Mocap Setup* console). Close it. In PowerShell, `Get-NetUDPEndpoint -LocalPort 5555` shows its process ID. |
| Joint angles **No** | Enable the Joint Angles datagram in the Network Streamer preset. |
| MuJoCo window does not open, or the system check reports OpenGL failed | Update the GPU driver from the vendor. Use the PC's own screen: Remote Desktop sessions often lack OpenGL. |
| Simulation is slow while capturing | Both programs share the CPU and GPU. Close other programs, use mains power and a high-performance power plan, and leave **Record video** off. |
| The window does not appear | Open the icon again. Logs: `%LOCALAPPDATA%\MocapHumanoid\logs\` (`app.log`, `backend.log`). |
| The app was force-closed and now reports the port in use | Run `"C:\Program Files\Mocap Humanoid\python\python.exe" -I "C:\Program Files\Mocap Humanoid\project\tools\windows\app.py" --stop`, then open the app again. |

The window uses Microsoft Edge WebView2, which Windows 11 and updated Windows 10 include. Without it
the app opens in an Edge app window instead; everything works the same.

## Update and uninstall

- **Update:** run the newer `MocapHumanoid-Setup-<version>.exe`. It asks you to close the app if it
  is running, replaces the program and keeps your settings and sessions.
- **Uninstall:** **Settings → Apps → Mocap Humanoid → Uninstall**. It removes the program, the
  shortcuts and the firewall rule. Your settings (`%LOCALAPPDATA%\MocapHumanoid`) and recordings
  (`C:\Users\<you>\mocap-sessions`) stay; delete them by hand if you no longer need them.

## Not in this version

The recorded BVH route, ROS 2 and physical robot control are not part of the app. For the recorded
route on Windows use the [ZIP install and Setup console](windows-setup.md); for ROS 2 and robots use
the Ubuntu setup ([engineer setup guide](engineer-setup-guide.md)).
