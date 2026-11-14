# Mocap Humanoid

Move in an Xsens motion-capture suit and watch a simulated humanoid robot copy your movements on screen: the Booster K1 by default, or the Unitree G1 or Booster T1.

You can use it in two ways:

- **Live:** the robot follows the performer in real time.
- **Recorded:** turn a saved Xsens take into a robot video.

Everything happens in simulation. No real robot is connected. Testing a real Booster K1 is a separate, later step: see [Testing on the real Booster K1](#testing-on-the-real-booster-k1).

Choose your setup: **[Windows](#windows)** (one PC, Xsens and the simulation together, easiest) or **[Ubuntu](#ubuntu)** (a second PC runs the simulation).

## Windows

Run the live simulation on the same Windows PC as Xsens Humanoid. No Ubuntu PC, Docker or WSL.

### What you need

- Windows 10 or 11, 64-bit, with Xsens Humanoid (MVN), its licence, and the MVN Link suit.
- A graphics card with a current driver (NVIDIA recommended; Intel and AMD also work).

### Install

**[Download Mocap Humanoid for Windows 0.1.0.12](https://github.com/prateek-ari/mocap-humanoid-release/releases/download/windows-app-v0.1.0.12/MocapHumanoid-Setup-0.1.0.12.exe)** (122 MB, no Internet needed to install)

1. Double-click the downloaded `MocapHumanoid-Setup-0.1.0.12.exe`.
2. Windows may say *"Windows protected your PC"* because the installer is not signed yet: click **More info → Run anyway**. Allow the administrator prompt, click **Install**, then **Finish**.

Everything the app needs is installed into `C:\Program Files\Mocap Humanoid`, with a **Mocap Humanoid** icon on the desktop and in the Start menu. Uninstall it from **Settings → Apps**.

### Use it

1. In Xsens Humanoid open **Options → Network Streamer** and add a stream: destination `127.0.0.1`, port `5555`, UDP, preset *ROS: Position + Orientation (Quaternion) and Joint Angles*, prop and finger segments off.
2. Open **Mocap Humanoid** from the desktop. When its Xsens lamp turns green, choose the robot and the performer's height and press **Start**. The robot follows the performer in a separate window.
3. With the performer standing still, arms relaxed, click **Calibrate**. **Stop** ends the run; tick **Record video** before Start to save an MP4.

The app offers the Booster K1 (the default), Unitree G1 and Booster T1. Older builds offer only the G1 and T1: install 0.1.0.12 over them.

Details and troubleshooting: [Windows app guide](docs/windows-app.md). The app is new: it passed automated tests on a clean Windows machine but has not yet been used with a real suit.

**Recorded takes on Windows:** the app covers the live route. To turn saved Xsens takes into robot videos on Windows, use the [Windows one-PC setup](docs/windows-setup.md) (download the project and double-click `Install-Windows.cmd`).

## Ubuntu

Xsens Humanoid stays on its Windows PC and streams to an Ubuntu PC that runs the simulation.

### What you need

- A **Windows PC** with Xsens Humanoid (MVN), its licence, and the MVN Link suit with Body Pack and router.
- An **Ubuntu 22.04 or 24.04 PC** with an NVIDIA graphics card.
- An **Ethernet cable** from each PC to the Xsens router.

### Install

On the Ubuntu PC, open a terminal and paste this line:

```bash
sudo apt-get update && sudo apt-get install -y git && git clone https://github.com/prateek-ari/mocap-humanoid-release.git && cd mocap-humanoid-release && bash install-ubuntu.sh
```

Already downloaded the project? Run `bash install-ubuntu.sh` from its folder. To update an earlier install (for example, to get the Booster K1), run `git pull` in the project folder and then `bash install-ubuntu.sh` again: it rebuilds the live simulation.

The script checks the PC and installs only what is missing: the NVIDIA driver, Docker, the NVIDIA Container Toolkit and the project itself. If it installs the NVIDIA driver, it asks you to restart: run the same command again after the restart. Run it from a terminal on the PC's desktop, not over SSH. `bash install-ubuntu.sh --check` only reports what is missing.

### Use it

When the installer finishes, it opens the **Setup console** in your browser. After that, open it any time from your applications menu: **Mocap Setup**.

The Setup console walks you through every step with buttons: enter the session details, check the suit's stream, start the live simulation, or turn a recording into a video.

**G1 and T1 side by side:** to drive both robots from one suit in one window, load your saved session and run `bash docker/gmr-live/run-compare.sh --calibrate-after 5` from the project folder. Step 07 of the setup guide has the exact commands.

## Testing on the real Booster K1

When a Booster K1 is on site, test it step by step before it replays any motion. The script `tools/control/k1_commission.py` takes the robot through twelve small steps on a hoist: PC check, read-only probe and 3D mirror, zero check, takeover and release, hold, balance, command loss, joint signs, gains, and the acceptance record. Each step shows its plan before the robot moves, stops and releases the arms when a check fails, and saves its evidence. The acceptance record unlocks the K1 arm replay (`tools/control/k1_arm_bridge.py`). Every step can be practised first on a fake robot.

You need an Ubuntu PC wired to the robot (the Windows app cannot drive it), a hoist and two people. Install with `bash tools/setup/install.sh recorded --with-booster`, then follow the **[K1 hardware test guide](docs/k1-bringup-runbook.md)** word for word. The easiest way is the test window, `.venv/bin/python tools/control/k1_commission_gui.py`: the same steps as buttons, with a live log and a red alarm if a release is not confirmed. Nothing in this code has run on a real K1 yet.

## Guides

- **[Windows app guide](docs/windows-app.md)**: install and use the Windows app, and fix common problems.
- **[Setup guide](docs/engineer-setup.html)**: the Ubuntu setup, step by step, with screenshots of what to click. ([Text version](docs/engineer-setup-guide.md))
- **[Windows and Xsens setup](docs/windows-xsens-humanoid-setup.md)**: preparing the suit and streaming on the Windows PC.
- **[Windows one-PC setup](docs/windows-setup.md)**: the project's own Windows install, including recorded takes.
- **[Testing without a suit](docs/testing-without-suit.md)**: check the installation before the suit is connected.
- **[K1 hardware test guide](docs/k1-bringup-runbook.md)**: test a real Booster K1 step by step on a hoist.
- **[Engineering reference](docs/engineer-reference.md)**: troubleshooting and details, if something goes wrong.

Something not working on Ubuntu? See step 12 of the setup guide, **Fix common problems**. On Windows, see the troubleshooting table in the [Windows app guide](docs/windows-app.md).
