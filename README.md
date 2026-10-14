# Mocap Humanoid

Move in an Xsens motion-capture suit and watch a simulated humanoid robot (Unitree G1 or Booster T1) copy your movements on screen.

You can use it in two ways:

- **Live:** the robot follows the performer in real time.
- **Recorded:** turn a saved Xsens take into a robot video.

Everything happens in simulation. No real robot is connected.

## What you need

- A **Windows PC** with Xsens Humanoid (MVN), its licence, and the MVN Link suit with Body Pack and router.
- An **Ubuntu 22.04 or 24.04 PC** with an NVIDIA graphics card.
- An **Ethernet cable** from each PC to the Xsens router.

## Install

On the Ubuntu PC, open a terminal and paste this line. Press **Enter** to accept each suggested answer:

```bash
sudo apt-get update && sudo apt-get install -y git && git clone https://github.com/prateek-ari/mocap-humanoid-release.git && cd mocap-humanoid-release && bash tools/setup/install.sh
```

For the live route, Docker and NVIDIA support must be installed first. The setup guide (step 02) explains how.

**Only have the Windows PC?** The simulation can also run on the same Windows PC as Xsens Humanoid, with no Ubuntu PC, Docker or WSL. Download the project, double-click `Install-Windows.cmd`, then open **Mocap Setup** from the Start menu. See the [Windows one-PC guide](docs/windows-setup.md). This route is new and not yet tested with a real suit.

## Use it

When the installer finishes, it opens the **Setup console** in your browser. After that, open it any time from your applications menu: **Mocap Setup**.

The Setup console walks you through every step with buttons: enter the session details, check the suit's stream, start the live simulation, or turn a recording into a video.

## Guides

- **[Setup guide](docs/engineer-setup.html)**: start here. Step by step, with screenshots of what to click. ([Text version](docs/engineer-setup-guide.md))
- **[Windows and Xsens setup](docs/windows-xsens-humanoid-setup.md)**: preparing the suit and streaming on the Windows PC.
- **[Windows one-PC setup](docs/windows-setup.md)**: run the simulation on the Xsens Windows PC itself, without an Ubuntu PC.
- **[Testing without a suit](docs/testing-without-suit.md)**: check the installation before the suit is connected.
- **[Engineering reference](docs/engineer-reference.md)**: troubleshooting and details, if something goes wrong.

Something not working? See step 12 of the setup guide, **Fix common problems**.
