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

**Booster K1 live demo** (the robot copies the performer): open **K1 Live Demo** from the applications menu, or run `bash k1-demo.sh` in the project folder, and follow the four steps on the page.

Already downloaded the project? Run `bash install-ubuntu.sh` from its folder. To update an earlier install: stop the Setup console (its **Shut down** button), close the K1 test window, then open **Mocap Update** from the applications menu. It downloads the new version and runs the installer in a terminal, and it refuses to start while one of the project's programs runs. If there is no **Mocap Update** entry yet (an install from before it existed), run `git pull` in the project folder and then `bash install-ubuntu.sh` once: it rebuilds the live simulation and adds the entry.

The script checks the PC and installs only what is missing: the NVIDIA driver, Docker, the NVIDIA Container Toolkit and the project itself. If it installs the NVIDIA driver, it asks you to restart: run the same command again after the restart. Run it from a terminal on the PC's desktop, not over SSH. `bash install-ubuntu.sh --check` only reports what is missing.

At the end it asks once: **Also install the recorded route and the Booster K1 test tools?** Press Enter for yes. This adds the runtime for recorded takes (in `~/mocap-runtime`), the Booster SDK and Tk. You can also say no and install them later with a button in the Setup console (step 13).

### Use it

When the installer finishes, it opens the **Setup console** in your browser. After that, open it any time from your applications menu: **Mocap Setup**. To stop it, click **Shut down the console** in step 11, or right-click **Mocap Setup** in the menu or dock and choose **Stop the Setup console**.

The Setup console walks you through every step with buttons: enter the session details, allow the stream through the firewall, check the suit's stream, start the live simulation, or turn a recording into a video. When a step needs administrator rights (the firewall rule, installing Tk), the button asks for your password in a normal Ubuntu password window. If no password window appears, use the **Copy** button next to the command and run it in a terminal.

**G1 and T1 side by side:** to drive both robots from one suit in one window, click **Start G1 + T1 side by side** in step 07. Stop it with **Stop**.

## Testing on the real Booster K1

**Quick start (since 2026-11-15).** Live arms start from cold with one command or one window screen. The K1 test window opens on **Quick start**: the tool probes the robot (read-only), checks the robot and the suit stream automatically, shows a summary and starts only on your **Go**. From a terminal: `.venv/bin/python tools/control/k1_live_bridge.py --transport booster --net <PC address> --operator "<name>" --go`. The stage ladder and the acceptance record below no longer refuse a start; their problems are warnings in the summary. The protections that act while the arms move stay (joint limits, speed caps, contact check, stream-loss stop, release).

**Head (since 2026-11-15).** The K1 head follows the performer's head yaw and pitch through Booster's `RotateHead` (Booster keeps the head). It is on by default; `--no-head` or the window's **Head follows** tick box switches it off. It holds while the arms align, returns to straight after a normal end or STOP, and a head problem switches only the head off. Tested on the practice robot with a real Xsens capture; not yet on the real robot: turn the head slowly first and check each direction ([Head (K6)](docs/k1-bringup-runbook.md#head-k6)).

The twelve test steps below are now optional (**Advanced tests** in the window). The script `tools/control/k1_commission.py` takes the robot through twelve small steps on a hoist: PC check, read-only probe and 3D mirror, zero check, takeover and release, hold, balance, command loss, joint signs, gains, and the acceptance record. Each step shows its plan before the robot moves, stops and releases the arms when a check fails, and saves its evidence. The acceptance record unlocks the K1 arm replay (`tools/control/k1_arm_bridge.py`). Every step can be practised first on a fake robot.

After the hoist tests, the robot moves only through a **stage ladder** (`mocap_g1/stage_gate.py`): S0 the acceptance record, S0r replay on the hoist, S1 live arms on the hoist, S1r replay standing, S2 live arms standing. A stage the ladder does not allow yet is a warning before Go, not a lock. In **live arms** (test window step 13, `tools/control/k1_live_bridge.py`) the robot's arms follow the suit: tick **Send the arms to the K1 window** in step 07 of the Setup console, start the live simulation, then run step 13. The window shows a live stream status bar, the drills, **Evaluate stage** and a practice sender that replaces the suit on the fake robot.

You need an Ubuntu PC wired to the robot (the Windows app cannot drive it), a hoist and two people. Install the K1 test tools when the Ubuntu installer asks, or later with **Install K1 test tools** in step 13 · **Real K1** of the Setup console. Then follow the **[K1 hardware test guide](docs/k1-bringup-runbook.md)** word for word. Use the test window: open **K1 Hardware Tests** from the applications menu, or click **Open K1 test window** in step 13. It has the same steps (plus step 13 · Live arms) as buttons, with a live log and a red alarm if a release is not confirmed. Only one test window opens at a time, and it does not open while an install runs.

From a terminal, the same things still work: `bash tools/setup/install.sh recorded --with-booster` installs the tools, and `.venv/bin/python tools/control/k1_commission_gui.py` opens the window.

## Guides

- **[Windows app guide](docs/windows-app.md)**: install and use the Windows app, and fix common problems.
- **[Setup guide](docs/engineer-setup.html)**: the Ubuntu setup, step by step, with screenshots of what to click. ([Text version](docs/engineer-setup-guide.md))
- **[Windows and Xsens setup](docs/windows-xsens-humanoid-setup.md)**: preparing the suit and streaming on the Windows PC.
- **[Windows one-PC setup](docs/windows-setup.md)**: the project's own Windows install, including recorded takes.
- **[Testing without a suit](docs/testing-without-suit.md)**: check the installation before the suit is connected.
- **[K1 hardware test guide](docs/k1-bringup-runbook.md)**: the quick start for live arms on a real Booster K1, and the optional step-by-step tests on a hoist.
- **[Engineering reference](docs/engineer-reference.md)**: troubleshooting and details, if something goes wrong.

Something not working on Ubuntu? See step 12 of the setup guide, **Fix common problems**. On Windows, see the troubleshooting table in the [Windows app guide](docs/windows-app.md).
