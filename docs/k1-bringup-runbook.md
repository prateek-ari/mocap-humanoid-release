# Booster K1 hardware test guide (bring-up runbook)

This guide takes a new Booster K1 EDU from "just delivered" to the first arm replay, one small
step at a time. Each step has a script, `tools/control/k1_commission.py`, that moves the robot as
little as possible, checks the result, and saves the evidence. The last step writes the
**acceptance record**, the file that unlocks real replay in `tools/control/k1_arm_bridge.py`.

**Nothing in this code has run on a real K1 yet.** The steps are built so that the first real runs
are safe: the robot hangs in a hoist, every motion is small and slow, and every step stops and
releases the arms when something looks wrong.

**The easiest way to run the steps is the test window** (a GUI with buttons; see
[The test window](#the-test-window)). It runs the same script, so everything in this guide
applies to it. For a live session with the suit, follow
[Live suit to the real K1 in the UI](#live-suit-to-the-real-k1-in-the-ui).

## Stop rules (read first)

- The robot hangs in a **hoist** for every step. The hoist must take the robot's weight if it
  falls.
- A **second person holds the Booster remote** during every step that moves the robot and watches
  the arms. **LT+BACK puts the robot in damping mode.** Use it at the first unexpected motion, then
  press Ctrl-C in the terminal.
- Nobody and nothing within reach of the arms.
- The software release can be slow. The SDK sends one request at a time, so a release can wait
  behind another request: timeouts of about 9 s were seen with no robot answering. If the tool prints
  `Release NOT confirmed`, **stop the robot with the remote at once** (LT+BACK) and keep it on the
  hoist.
- Run one motion program at a time. The tools refuse a second one on the same PC, but they cannot
  see another PC: make sure no other computer sends commands to the robot.
- A step that ends in FAIL is information, not a reason to push on. Read the "What to check" line,
  fix the cause, and run the step again. Send the session folder to the developers when unsure.

## What you need

- The robot, its remote, a hoist, two people.
- An **Ubuntu 22.04 or 24.04 PC** (x86_64 or ARM) with a screen: the mirror (step 3) opens a 3D
  window. The Booster SDK has no Windows version, so the Windows desktop app cannot drive the robot.
- An Ethernet cable from the PC to the robot.
- A protractor or goniometer (for step 4), and a phone to film the motion steps.
- Run every command from the project folder (the checkout of this repository).

## Terms

| Term | Meaning |
| --- | --- |
| rad | the unit of joint angles: 1 rad = 57.3°, 0.1 rad ≈ 6° |
| hoist | a frame and strap that holds the robot up and catches it if it falls |
| damping mode | Booster's safe mode: the joints go soft and only resist fast motion (remote LT+BACK) |
| prepare / walking mode | Booster's standing modes, set with the remote; in walking mode Booster balances the robot |
| takeover / release | this PC takes the arms from Booster's controller (`UpperBodyCustomControl(true)`) and gives them back (`false`); Booster keeps the legs and owns the head (K6 moves it only through `RotateHead`) |
| head follows | the K1 head turns with the performer's head (yaw and pitch) through Booster's `RotateHead`; Booster keeps control of the head |
| kp / kd | how stiffly the arm motors hold their commanded angle (kp) and how much they resist speed (kd) |
| safety envelope | the layer in this software that limits every arm command's speed, acceleration and range |
| tracking error | the difference between the measured joint angle and the closest angle commanded in the last 0.5 s (the K1's arms follow about 0.1 s late; since 2026-11-15 a late but faithful arm is not an error) |
| p95 | 95 % of the samples are below this value |
| DDS | the network protocol the Booster SDK uses (UDP, including multicast) |
| session / attempt | a session is one robot's folder of evidence; each run of a step is one attempt in it |
| acceptance record | the file `record` writes; without it the real replay refuses to run |

## What to find out about the robot first

Check each item on the robot and in Booster's manual or app before day one. The defaults
come from Booster's SDK documentation (SDK for firmware 1.8.0); none has been checked on a real
K1 yet.

| Item | Expected (Booster default) | Where to find it | Used by |
| --- | --- | --- | --- |
| Robot IP address | `192.168.10.102` | Booster's manual or app | `preflight --robot-ip` (if different) |
| PC address on the robot network | `192.168.10.10/24`, same /24 as the robot | you set it on the PC | `--net` on preflight and probe |
| PC network port | the Ethernet port wired to the robot | `ip -4 addr` | `--net` (address or interface name) |
| DDS domain | `0` | Booster's manual | `probe --domain-id` (if not 0) |
| Firmware version | `1.8.0` | Booster's app; probe reads it | the session and the record |
| Serial number | — | Booster's app; probe reads it | the session and the record |
| Remote stop | LT+BACK = damping mode | Booster's manual for this remote | stop rules |
| How to change mode with the remote | damping, prepare, walking | Booster's manual | every motion step (`--mode`) |
| SDK access from an external PC | allowed over Ethernet | Booster's manual or support | everything |

Stop and ask the developers before any motion step if:

- the firmware is not 1.8.0 (the SDK bindings may not match it);
- the manual names a different emergency stop than LT+BACK;
- the robot needs a setting or a "developer mode" for the SDK that is not described here.

Write the actual values down; the developers need them with the session folder (see "What to
send to the developers" below).

## Before the robot arrives

1. **Install** this checkout with the Booster SDK bindings and the GMR runtime (for the K1 model).
   Easiest: say yes when `install-ubuntu.sh` asks about the K1 test tools, or click **Install K1
   test tools** in the Setup console, step **13 · Real K1** (it shows each part as FOUND or
   MISSING; **Install Tk** asks for your password if Tk is missing). From a terminal:
   `bash tools/setup/install.sh recorded --with-booster`
   This installs `booster_robotics_sdk_python==1.6.3`. The K1 model is then
   `~/mocap-runtime/GMR/assets/booster_k1/K1_serial.xml`; pass it with `--model` to `preflight`
   and `probe` (the session remembers it).
2. **Network.** Give the PC the static address `192.168.10.10/24` (gateway `192.168.10.1`) on the
   port wired to the robot. The robot is `192.168.10.102` (if not, use its address and a PC
   address in the same /24, and pass `--robot-ip` to `preflight`). Turn off Wi-Fi or make sure no
   other network uses `192.168.10.x`. The SDK talks over DDS (UDP multicast): if the PC firewall is
   on (`sudo ufw status`), allow the robot's network, for example
   `sudo ufw allow from 192.168.10.0/24`.
3. **Practise the whole ladder on the fake robot.** Every step works without a robot. Start with
   `.venv/bin/python tools/control/k1_commission.py probe --model <K1_serial.xml>`; it prints a
   session folder and the next command. Follow the "Next:" lines to the end (`record`). A practice
   session writes `k1-acceptance-PRACTICE.json`, which can never unlock a real robot.
4. **Firmware.** The SDK was written for robot firmware 1.8.0. Step 2 records the robot's version.
5. **A K1 recording** for the first replay: the recorded route with the K1 (Setup console, **Make
   preview motion**), then an approved calibration (`quality_approved` in the motion's metadata). A
   preview-grade recording runs on the fake robot only.

In the commands below, `T=".venv/bin/python tools/control/k1_commission.py"` and
`S=<the session folder that probe printed>`.

## Live suit to the real K1 in the UI

One page for a live session: the performer in the Xsens suit moves the real K1's arms and head.
Everything here is a button in the **Setup console** (menu: **Mocap Setup**) or the **K1 test
window**; no terminal is needed. People: the performer, an operator at the PC, and one person at
the robot with the remote (**LT+BACK = damping, the real stop**). Keep the robot held or on the
hoist.

1. **Software.** Setup console, step **03 · Session**, box **Software**: **Running version**
   (commit and date of this checkout) and **Last update** (the result line of the last
   **Mocap Update**: time, PASS or FAIL, old commit -> new commit). A failed update is shown in
   red with what to do. To update, see step 11 below.
2. **Session.** Step 03: robot **Booster K1**, the performer's height, **Windows Xsens IP** = the
   address of the PC that runs Xsens (MVN). **Save session**.
3. **Where the Xsens data comes from.** Start streaming in Xsens to this PC, port 5555. Steps
   **06** and **07** show one line about the sender:
   - green: data arrives from the address in the session. Good.
   - red, "arrives from A, but the session expects B": click **Use A**. It saves A as the Xsens
     source IP of the session. A live simulation that is already running keeps the old address:
     stop it and start it again. **Use** only changes the session; it does not fix network
     routing or a firewall.
   - red, "comes from 2 senders": two PCs stream to this PC. Stop the stream on all but one.
     There is no **Use** button then.
   - "No Xsens data" or "No new Xsens data for N s": Xsens does not stream to this PC (wrong
     destination address, cable, firewall: step 05 of the setup guide).
   - "Another program uses UDP port 5555": a live tool or stream check started in a terminal
     holds the port. The console listens on the port only while step 06 or 07 is open, and lets
     go of it 10 s after you leave those steps.
4. **Live simulation.** Step 07: tick **Send the arms to the K1 window**, click **Start live
   simulation**. With the arms ticked there is no **Auto-calibrate**: the K1 is only calibrated by
   your click. The lamps under the buttons show **Calibrated** and **Tracking on** (with the reason
   when it is off).
5. **Calibrate.** Click **Calibrate**. The console first shows what to tell the performer:
   stand up straight, arms down at the sides, look straight ahead, hold still. Say it, then click
   **Calibrate now** (or **Cancel**). The line under the buttons shows **Calibrating...**, then:
   - **Calibrated.** Good.
   - **Calibrated, but ...**: the calibration was taken, with a warning or a partial failure (for
     example "the performer is not standing (thigh 62° from vertical)"). Read it; usually: have the
     performer stand up, then Calibrate again. The pose check warns and never refuses.
   - **Calibration failed: ...**: the earlier calibration stays. Fix the cause, Calibrate again.
   - **No answer from the live simulation** (after 5 s): look at the **Calibrated** lamp, then
     Calibrate again if needed. A live tool older than this console does not report results.
   The K1 head takes its zero at Calibrate: a turned head gives a constant head offset for the
   whole run. A new Calibrate during a run stops the arms (a normal stop); click Check and Go
   again.
6. **K1 window.** Step 13: **Open K1 test window**. It opens on **Quick start**. Set
   **Transport** `booster` and **PC interface or IP** (the window remembers both), type your
   name. Tick or untick **Head follows**.
7. **Check and Go.** **Check robot and stream**: numbered lines, each with a lamp and, when red,
   what to do. The check repeats every 2 s (read-only) until all are green. **Go** (the warnings
   beside it do not block). The arms first move slowly to the performer's pose; the performer
   holds still until the status says following. One large line shows the phase, the seconds
   followed, head on or off (and why it is off) and lag holds.
8. **Stop and release.** **STOP** in the window (or the time limit) slows the arms, gives them back
   to Booster and turns the head back to straight. The remote's LT+BACK always works. At the end
   the window shows PASS, or the stop reason with what to do next.
9. **Recordings and their limits.** See [Recordings of a run](#recordings-of-a-run) for what is
   recorded on each live run (the incoming suit data and the robot's joint data), where it is,
   and what can be lost.
10. **Export.** In the K1 window, **Zip session for the developers** packs the session folder into
    one zip next to it and opens that folder. Send that file.
11. **Update.** Close the K1 test window and stop the Setup console (below), then open **Mocap
    Update** from the applications menu. It runs in a terminal: it refuses to start while the
    console, a live simulation or the K1 window runs (and names what to close), pulls the new
    release, runs the installer (type your password when asked), prints PASS or which step failed
    with the command to run again, and offers to open the Setup console. The result is the
    **Last update** line in step 03.
12. **Stopping the console for real.** Closing the browser tab does not stop it. Click **Shut down
    the console** in step 11 · Finish (a running job stops first; an open K1 window stays open),
    or right-click **Mocap Setup** in the menu and choose **Stop the Setup console**. From a
    terminal: `bash tools/setup/open_console.sh --stop`.

## The test window

Open **K1 Hardware Tests** from the applications menu, or click **Open K1 test window** in the
Setup console (step 13). From a terminal:

```bash
.venv/bin/python tools/control/k1_commission_gui.py
```

A window that opens on the **Quick start** (live arms after two automatic checks and your Go);
the thirteen steps are under **Advanced tests**. It needs a screen and Tk (**Install Tk** in step 13,
or `sudo apt install python3-tk`; `install.sh --with-booster --system-packages` installs it).
Only one test window opens at a time: a second launch (menu, console or terminal) says "already
open, or an install is running" and exits. The installer refuses to run while the window is open
("Close the K1 test window first"), because it changes the Python environment the window runs on.
Shutting down the Setup console does not close the window or stop a running step. It does not move the
robot by itself: each step runs the same `k1_commission.py` command as this guide (step 12 runs
`k1_arm_bridge.py`, step 13 and the quick start run `k1_live_bridge.py`), with the same checks,
limits and evidence.

- **Quick start (first screen).** The fields fill themselves: the window remembers the last
  transport, PC interface, K1 model, your name, stage, support and time limit
  (`~/.local/state/mocap-g1/k1-window.json`; a missing or bad file gives the defaults). On the
  first run the PC interface is the one PC address on the robot's network (when there is exactly
  one) and the K1 model is found without **Browse** (GMR, then `K1_serial.xml` in
  `$MOCAP_WS` or `~/mocap-runtime`). An empty field is named in plain words
  under the buttons.
  1. **Check robot and stream** fills a numbered checklist, one line per check, each with a lamp
     (green OK, red FIX, grey "..." = not checked yet), the result and, on a red line, **what to
     do**: PC on the robot's network · robot answers · robot standing in walking mode, not fallen,
     joints in range · live tool is sending · calibrated · tracking on · model matches · clock ·
     the arm gap between performer and robot (shown only; it does not block). These are the same two
     read-only auto-checks as on the command line.
  2. **The check repeats by itself** every 2 s until every line is green, or until you click
     **Stop checking**. Fix one line and watch its lamp change. The robot never moves while it
     checks. The first check probes the robot and makes the session; the repeats use that session
     (no new session each time). After a run, the next Check makes a new session.
  3. **Go** turns on when both checks pass. The warnings (stage-gate problems, no acceptance
     record) are listed beside it and do not block. A changed setting clears the Go and the check
     runs again. No Go, no motion.
  4. **While it runs**, one large line shows the phase (waiting for the arms to match, aligning,
     following, holding), the seconds followed of the time limit, whether the head follows (and
     why it is off) and the lag holds. **STOP** works as in the steps.
  5. **At the end**: PASS, or the stop reason with what to do next (for example "The performer
     moved faster than the arms can follow for 6 s: start again and move slowly at first"), and
     the attempt file.

  Practice: **Start practice sender** streams a slow pattern instead of the suit.

- **Top bar:** grey **PRACTICE** (fake robot) or red **REAL ROBOT**. The real stop is always the
  remote: LT+BACK.
- **Robot connection:** transport (`fake` to practise, `booster` for the robot), this PC's
  address on the robot network (**PC interface or IP**: a list of this PC's addresses; the one on
  the robot's network is chosen for you, and a grey line says when none is), the robot's IP, the
  DDS domain, the robot mode you set with the remote, and the K1 model (`K1_serial.xml`, found
  for you as in the quick start, else **Browse...**). Fill in **Attested serial**
  and **Firmware** only when the robot cannot report its serial (step 2).
- **Firewall:** the window shows whether Ubuntu's firewall (ufw) is on. If it is on or unknown,
  **Allow the robot network in the firewall** asks for your password in a normal Ubuntu dialog and
  allows the robot's network (`ufw allow from 192.168.10.0/24`). If no dialog can open, the log
  shows the command to run in a terminal instead.
- **Steps (left):** each step's evidence in the open session: green = valid, red = failed or no
  longer valid (with the reason), orange = interrupted (`INCOMPLETE`), grey = not run, blue = a file
  was saved (mirror ranges, the record; the record turns red "STALE" when a step changed after it).
  **2 · Probe** starts a new session; **Open session...** opens an earlier one.
- **Step panel (right):** what the step does, whether the robot moves, its settings (hold seconds,
  command-loss gap, gains level and kp/kd, your name and the gains run for the record, the
  recording for the replay), **Run step** and the red **STOP step**. STOP is Ctrl-C to the script:
  it stops and releases the arms. If anything looks wrong, use the remote first.
- **The tool asks:** questions appear as buttons. Before every takeover the window shows the plan
  and the checklist; **All of the above is true: start** starts the step, **Cancel** stops it with
  nothing sent to the robot. Yes/No questions get **Yes** and **No**; measurements and names get a
  text field and **Send**. Each answer is sent once.
- **Log (bottom):** the script's output, live: PASS in green, FAIL and errors in red. A yellow
  **What to check** box explains a failure. Each run's log is also saved in
  `$S/gui-logs/`. **Zip session for the developers** packs the whole session folder into
  `<session>.zip` next to it and opens that folder: send that file. **Open the guide** opens this
  guide. While the firewall or zip action runs, the steps wait.
- **Red alarm:** when a release is not confirmed, or a run ends abnormally (crash, killed, an
  interrupted attempt), a red bar says **stop the robot with the remote now**. Nothing else runs
  until you press **I have stopped the robot with the remote**.
- **The mirror (step 3)** runs beside the other steps; **Close mirror** stops it. Closing the
  window while a step runs asks first, stops the step and waits for its release. If the window
  itself crashes, the script gets Ctrl-C and releases the arms.
- **Step 12:** **Browse...** opens the Setup console's sessions folder (`~/mocap-sessions`), and the
  panel names the newest K1 preview there as a hint. Choose the recording yourself; choosing
  another file clears the tick. **Stage** picks `hoist` (default) or `standing`; the window passes
  the session, so each replay is an attempt there and a PASS goes to the stage ledger.
- **Step 13 · Live arms:** the arms follow the performer (see [K4 · live
  arms](#k4--live-arms-k1_live_bridgepy)). Choose **Stage** (`hoist`, or `standing` once the gate
  allows it), **Time limit** (seconds of following, default 120, max 600), **Stream port** (default
  47011, the live tool's `--robot-stream` port) and **Run**: **Following**, **Stream-loss drill** or
  **STOP drill**. Stage-gate problems (for example "S1r · K3 replay standing: no PASS for this
  robot") are warnings on the Go screen, not a lock; only a real refusal (an invalid acceptance
  record, a changed K1 model) keeps **Run step** grey.
  - While it runs, a coloured bar above the log shows the stream: state (waiting, aligning,
    following, lag-hold, stopping, released), packets per second, the age of the newest suit frame,
    the calibration epoch, tracking on/off, the arm joint furthest from its target, the seconds of
    following and the release (requested, confirmed or failed). Green = following, orange = holding
    or stopping, blue = waiting or aligning.
  - **Drills:** for the stream-loss drill, stop the stream while the arms follow (practice: **Stop
    practice sender**; real robot: stop the live simulation). For the STOP drill, press **STOP
    step** while the arms follow.
  - **Evaluate stage** (with your name) judges this session's attempts and writes the stage PASS to
    the ledger.
  - **Start practice sender** (practice only) streams a slow arm pattern instead of the suit, so
    you can rehearse the whole step. It stops with the step, or when you press it again.
- **Stage ladder** (button in the log bar and in step 13) lists S0, S0r, S1, S1r and S2 for the open
  session's robot: **passed** (date), **locked** (the first reason), **revoked** (why), or **no longer
  valid** (for example an evidence file changed). A practice session shows the PRACTICE ledger: its
  passes unlock nothing. Rows 12 and 13 in the step list show the same ledger state. The ladder
  updates after every run.

Practise the whole ladder in the window with transport `fake` before the robot arrives.

## Quick reference

| # | Step | Command | Robot moves | Motion time |
| --- | --- | --- | --- | --- |
| 1 | preflight | `$T preflight --transport booster --net 192.168.10.10 --model <K1_serial.xml>` | no | — |
| 2 | probe | `$T probe --transport booster --net 192.168.10.10 --model <K1_serial.xml>` | no | 10 s watch |
| 3 | mirror | `$T mirror --session $S` (second terminal, keep open) | no | — |
| 4 | zero-check | `$T zero-check --session $S` | no | 1 s sample |
| 5 | quick-release | `$T quick-release --session $S` | no (takeover and release only) | — |
| 6 | hold | `$T hold --session $S --seconds 10` | settling (3°) | 12 s |
| 7 | balance | `$T balance --session $S` | settling | 32 s |
| 8 | command-loss | `$T command-loss --session $S --gap 0.1` (then 0.25, 0.5, 1.0) | settling, then no commands | 2-3 s each |
| 9 | signs | `$T signs --session $S` | each arm joint 6° and back | 30 s |
| 10 | gains | `$T gains --session $S --level L1 --kp 20 --kd 1` (then L2, L3) | up to 17° per joint | 21-24 s each |
| 11 | record | `$T record --session $S --operator "<your name>"` | no | — |
| 12 | replay | `.venv/bin/python tools/control/k1_arm_bridge.py …` (see step 12) | the recording | `--duration 10` |
| 13 | live arms | `.venv/bin/python tools/control/k1_live_bridge.py …` (see [K4](#k4--live-arms-k1_live_bridgepy)) | the performer's arm motion | `--time-limit 120` |
| — | status | `$T status --session $S` | no | — |

Motion steps also take `--mode prepare` (default walking) and, for a robot that cannot report its
serial, `--attested-robot-id <serial>`. Each step prints the exact next command.

## The steps

Run them in this order. Each step refuses to run until the steps it depends on have passed in the
same session. `$T status --session $S` shows every step, whether its latest attempt is valid, and
why not.

### Step 1 · preflight (robot off or on; no motion)

```bash
$T preflight --transport booster --net 192.168.10.10 --model <K1_serial.xml>
```

Checks this PC: Linux, Python, SDK version, that `--net` is this PC's address or interface on the
robot's /24 network, that the robot answers `ping`, and that the K1 model is found. Add
`--robot-ip <address>` if the robot is not at `192.168.10.102`.
**PASS** when every line says PASS. **If it fails:** each line says what is missing.

### Step 2 · probe (robot on; read-only)

```bash
$T probe --transport booster --net 192.168.10.10 --model <K1_serial.xml>
```

Watches the robot for 10 s without commanding it, and **starts a session**: a folder under
`k1-commissioning/` named after the time and the robot's serial number. The session stores the
robot's serial and firmware, the SDK version and the K1 model; every later step checks that they
have not changed.

**PASS** when joint states arrive, the robot mode can be read, the fall-state topic is periodic
(at least once every 1.5 s; a K1 on firmware 1.8.1.2 sends it once a second, and motion steps then
treat a fall reading older than 3 periods as unknown), and no joint state is malformed. **If it fails:**

- No joint state: check the cable, `--net`, the PC firewall (DDS needs UDP multicast), the DDS
  domain (`--domain-id`), and that no other program talks to the robot. Also check the PC's route
  (`ip route get <robot IP>` must name the port wired to the robot), and that the robot's own DDS
  profile includes its Ethernet address: on one K1 (firmware 1.8.1.2) the motion services used
  `/opt/booster/config/dds_configs/single-board_configs/fastdds_profile_udp_only.xml`, which listed
  only internal addresses, so an external PC saw no DDS at all until `192.168.10.102` was added
  (back up the file, add the address, reboot the robot in damping). Ask Booster before you change it.
- Fall topic "event-or-missing": motion stays blocked (a remembered "ready" is no proof the robot
  stands). Send the probe output to the developers.
- The robot does not report its serial: read the serial and the firmware version in Booster's app
  and pass `--attested-robot-id <serial> --firmware <version>`. Later motion steps then need
  `--attested-robot-id <serial>` every time.

### Step 3 · mirror (keep it open; read-only)

```bash
$T mirror --session $S
```

Opens a 3D view of the K1 model posed from the robot's measured joints, and prints the arm angles
in degrees every second. Keep it open in a second terminal during all later steps: **the mirror
must always look like the robot.** If an arm in the mirror points somewhere else than the real arm,
stop: a joint sign, a zero offset or the joint order is wrong. On exit it saves the lowest and
highest angle seen for each joint (`mirror-*.json`), for example the real elbow-yaw stop if Booster
moves the arm there.

### Step 4 · zero-check (read-only; needs a protractor)

**The geometry.** In the K1 model, every joint at 0 is a T-pose: arms straight out to the sides.
The names of the arm joints do not all mean what they suggest:

| SDK joint | What it does | At rest (arms down) |
| --- | --- | --- |
| `Shoulder_Pitch` | swings the arm forward and back | about 0° |
| `Shoulder_Roll` | raises the arm sideways (0° = straight out) | about -75° left, +75° right |
| `Elbow_Pitch` | **twists the upper arm** about its own long axis | about 0° |
| `Elbow_Yaw` | **bends the elbow** (0° = straight; the range ends at 0) | 0° |

![K1 model: T-pose zero (left), arms down (middle), left elbow bent 52° (right)](engineer-guide-assets/k1-arm-poses.jpg)

Put the robot in walking mode, standing in the hoist, arms in Booster's own pose and still. Then:

```bash
$T zero-check --session $S
```

Measure on the robot and type four angles in degrees:

- **Upper arm from down** (each arm): the line from the shoulder joint (where the arm leaves the
  shoulder block) to the elbow joint, against the torso's long axis. 0° = the arm hangs straight
  down along the body.
- **Elbow bend** (each arm): the upper-arm line against the forearm line (elbow joint to the end
  of the forearm). 0° = straight arm.

Keep the robot still while the tool samples its joints for 1 s. It computes the same angles from
the K1 model at the measured joint values. **PASS** when all four agree within 5°.

What this proves, and what it does not: the upper-arm angle checks shoulder pitch and roll
together, and the elbow bend checks `Elbow_Yaw`. They cannot separate shoulder pitch from roll, and
they cannot see the upper-arm twist (`Elbow_Pitch`); those are checked by eye in the mirror (step 3)
and by step 9. The acceptance record states this scope. **If it fails:** a joint's zero offset or
sign differs from the model; send the attempt file to the developers before any motion step.

### Step 5 · quick-release (the first takeover; no motion)

```bash
$T quick-release --session $S
```

Every motion step first prints a checklist (and, for steps 6-10, the planned motion, joint by
joint, in degrees) and waits for you to type `yes`.

This step takes the arms over (`UpperBodyCustomControl(true)`), waits for one fresh joint state,
and releases them at once. It sends no arm command. **PASS** when the takeover and the release both
succeed with nothing published in between, and you confirm the robot is normal afterwards. The
attempt file records how long each request took. **If it fails** with `takeover_failed`: the robot
is probably not in walking mode; also try `--mode prepare` (runbook check: in which mode does the
takeover work?). Evidence from one mode never counts for the other: every later step and the record
take the same `--mode`.

### Step 6 · hold (settling only)

```bash
$T hold --session $S --seconds 10
```

Takes over, holds the arms where they are for 10 s (gains kp 20 / kd 1), then slows to rest and
releases. A resting elbow sits on its range end (0°); the safety envelope keeps 0.05 rad (3°) away
from every end, so the plan shows both elbow-yaw joints **settling 3° inward**. That is expected.

Right after the takeover, before the first command, the tool checks that the robot has not moved
since you confirmed the plan, re-checks the settling, and checks that the joint states are fresh;
otherwise it releases without sending anything. After settling it checks that the arms reached the
start pose. **PASS** when the hold completes, the release is confirmed, and you confirm the arms
stayed still and Booster took them back smoothly.

### Step 7 · balance (settling only, 30 s)

```bash
$T balance --session $S
```

The arms hold still for 30 s while you push the robot gently at the chest, several times, with the
robot still supported by the hoist. The tool stops at once if the robot mode or fall state changes.
Afterwards it asks whether the robot kept its balance and how it was supported. This is
**hoist-supported** evidence only; free-standing balance is a later test with a spotter.

### Step 8 · command-loss (commands stop for a short gap)

```bash
$T command-loss --session $S --gap 0.1
$T command-loss --session $S --gap 0.25
$T command-loss --session $S --gap 0.5
$T command-loss --session $S --gap 1.0
```

Answers the question: what do the arms do if this PC stops sending commands? After settling, the
tool stops publishing for the gap while it keeps checking the joint states, the robot mode, the fall
state, contact with the body, and how far each arm drifts. Then it releases the arms (it never
resumes commanding after a gap). It stops early and releases if an arm drifts more than 0.1 rad
(6°). Each gap needs the one before it; the record needs 0.5 s or longer.

**PASS** when the gap ran to its end and you accept the printed behaviour ("max arm drift left …,
right …; then released in … s"). **If it fails** with `command_gap:drift`, the robot does not hold
the arms when commands stop; stop here and send the log to the developers.

### Step 9 · signs (each arm joint 0.1 rad and back)

```bash
$T signs --session $S
```

Moves each of the 8 arm joints by 0.1 rad (6°), one at a time, and back. The order is printed
first. Watch the robot and the mirror together. The tool checks that each joint moved in the
commanded direction by 0.05-0.15 rad; afterwards it asks, joint by joint, whether the robot moved
the same way as the mirror. **PASS** when every joint is right both ways. **If it fails:** a sign,
an offset or the joint order differs between the robot and the model; do not continue.

### Step 10 · gains (tracking quality)

```bash
$T gains --session $S --level L1 --kp 20 --kd 1
$T gains --session $S --level L2 --kp 20 --kd 1
$T gains --session $S --level L3 --kp 20 --kd 1
```

Moves each arm joint alone, out and back, then all eight together, and measures how closely the
arms follow. The levels are fixed:

| Level | Move | Speed | Acceleration | Needs |
| --- | --- | --- | --- | --- |
| L1 | 0.10 rad | 0.25 rad/s | 1.0 rad/s² | valid signs and balance |
| L2 | 0.20 rad | 0.45 rad/s | 1.5 rad/s² | a valid L1 with the same kp/kd |
| L3 | 0.30 rad | 0.60 rad/s | 2.0 rad/s² | a valid L2 with the same kp/kd |

Before the takeover the tool simulates the whole motion and refuses a plan that leaves a joint's
safe range, comes close to the body, or cannot reach the level's speed. **PASS** when no check
stops the run, every joint's tracking error stays below 0.2 rad (p95 below 0.1 rad), and every
joint reached the level's speed and acceleration in both phases.

If the arms lag, try a stiffer pair, starting again at L1: `--kp 30 --kd 1.5`, then `--kp 40 --kd 2`.
The ceiling, kp 60 / kd 3, is the value in Booster's own example, not a tested safe value. A pair is
approved exactly as tested: the replay later refuses any other kp/kd, and its speed and
acceleration are capped at what the approved run demonstrated (the lowest over all joints).

### Step 11 · record (the acceptance record)

```bash
$T record --session $S --operator "<your name>"
```

Checks that every step is valid in this session, lists the passed gains runs and asks which one to
approve, asks two questions for later phases (does Booster keep walking while the arms are custom;
which controller owns the head joints), shows the whole record including the zero-check scope, and
writes it when you type your name. It stores, for each field, the attempt and its SHA-256, and
writes `$S/k1-acceptance.json`. It also registers S0 (`acceptance`) in the robot's **stage
ledger**, `<root>/<serial>/stages.json` (`<root>` is `$K1_COMMISSIONING_ROOT`, else the repository's
`k1-commissioning/`), with the record and every attempt it cites as evidence; a practice record goes
to `<root>/PRACTICE/stages.json`, which unlocks nothing. It then prints the first replay command.

### Step 12 · first real replay (stage `replay@hoist`)

The recording: a K1 retarget from the Setup console (or `retarget_robot_recorded.py --robot
booster_k1 --arm-mode config`) is clamped to the official K1 joint limits and marked
`arm_replay_eligible`; the real replay accepts it for the arms. Other preview recordings are
refused. This release has no automatic qualification for recordings, so check the recording
yourself: use a short, slow K1 recording with an approved calibration (see "Before the robot
arrives", item 5), watch its whole preview motion first (Setup console, or
`tools/retarget/render_robot_motion.py`), and do not use a recording where an arm passes through
the body. Then, on the hoist:

```bash
.venv/bin/python tools/control/k1_arm_bridge.py <motion.pkl> --transport booster \
  --net 192.168.10.10 --session $S --stage hoist \
  --model <K1_serial.xml> --duration 10 \
  --yes-i-have-read-the-unverified-section
```

The replay is a gated stage. It refuses unless the stage ledger holds a valid S0 for this robot,
firmware and SDK bindings (evidence unchanged, not revoked, at most 30 days old) and
`$S/k1-acceptance.json` is the record registered there; `--stage standing` needs live arms on the
hoist. It takes kp/kd, speed and acceleration from the record. `--model` must be the model the
recording was retargeted on. Each run writes an attempt and its tick log into `$S/attempts/`.
**PASS** (written to the ledger, with the exact settings it ran with): at least 10 s played to the
end, no fault, release confirmed, tracking p95 below 0.15 rad. A release that is not confirmed, or
a fall, **revokes** S0: redo S0 before the next replay. Ctrl-C stops cleanly (slows the arms to
rest and releases them) and counts as a FAIL.

## When a step fails

The tool prints the reason and a "What to check" line. The common reasons:

| Reason | What it means | What to check |
| --- | --- | --- |
| `state_stale` | joint states stopped arriving | cable, `--net`, PC firewall, another program on the robot |
| `mode_not_allowed:N` | the robot left the takeover mode | set the mode with the remote, or match `--mode` |
| `fall_state:N` | the robot reports a fall | stop; check the robot in the hoist |
| `tracking_error:<joint>` | an arm is more than 0.3 rad from its command | an obstacle; the mirror; joint signs; gains too low |
| `post_takeover:pose_changed` | the robot moved during the takeover | rerun with the robot still |
| `not_settled` | the arms did not reach the start pose | an obstacle; gains |
| `takeover_failed` | `UpperBodyCustomControl(true)` failed | the robot mode (step 5) |
| `command_gap:drift` | the arms moved > 0.1 rad with no commands | stop; send the log |
| `real_time_lost` / `loop_overrun` | this PC fell behind | close other programs |
| `release not confirmed` | the release request did not succeed | **stop with the remote now** |

Every attempt is saved as `$S/attempts/NNN.json` (settings, measurements, answers, the reason, the
request timings) with a tick log `NNN.jsonl` for motion steps. An attempt that was interrupted stays
`INCOMPLETE` and never counts. When you rerun a step, everything that depended on its earlier run
must be rerun too; `status` shows what.

### What to send to the developers

- The whole session folder (`k1-commissioning/<session>/`), zipped. It holds the probe report, every
  attempt with its tick log, the mirror ranges, the record and the test window's logs (`gui-logs/`).
- The values from "What to find out about the robot first" (IP, domain, firmware, serial, remote stop).
- The output of `$T status --session $S`.
- A phone video of any step that looked wrong, with the mirror visible if possible.

## Reference

### What the bridge does

- It calls `UpperBodyCustomControl(true)` (Booster SDK) so the K1's arms follow this PC. Booster's
  controller is expected to keep the head and legs (step 7 checks the balance).
- It publishes the SDK's `ArmController` frame on `rt/joint_ctrl`. All 22 motors use `mode 0x0A`,
  `dq 0`, `tau 0`. The 8 arm motors (indices 2-9) get a position and kp/kd. The head and legs get
  `q 0, kp 0, kd 0`. `weight` and `cmd_type` keep the binding defaults.
- The safety envelope limits arm speed and acceleration and keeps 0.05 rad inside every joint
  range. It starts from a joint state measured after the takeover; an arm joint up to 0.02 rad past
  its range end (a straight elbow rests on 0.0) counts as the range end, further out refuses. Head
  and arm positions come from `motor_state_parallel`, leg positions from `motor_state_serial` (the
  parallel ankle entries are crank motors). The arm-versus-body contact check uses the measured leg
  pose.
- A normal end slows the arms to rest, then calls `UpperBodyCustomControl(false)`. A fault stops
  publishing and releases at once. Faults: a joint state older than 0.1 s, robot mode outside the
  allowed mode or not read for 1.5 s, a fall state other than ready, a tracking error over 0.3 rad,
  a lost command source, a publish error, lost real time, and any unexpected error.
- Position control does not enforce the motors' effort limits.

### Commissioning limits (fixed in the tool)

| Step | Motion from the settled pose | kp / kd | Speed / acceleration |
| --- | --- | --- | --- |
| quick-release | none, no command | — | — |
| hold, balance, command-loss | settling only (≤ 0.10 rad per joint, inward) | 20 / 1 | 0.3 rad/s / 1.0 rad/s² |
| signs | 0.10 rad, one joint at a time | 20 / 1 | 0.3 / 1.0 |
| gains L1 / L2 / L3 | 0.10 / 0.20 / 0.30 rad | your pair, ≤ 60 / 3 | 0.25 / 1.0, 0.45 / 1.5, 0.60 / 2.0 |

Hold ≤ 60 s; command gaps 0.1 / 0.25 / 0.5 / 1.0 s with a 0.1 rad drift stop.

### The acceptance record (schema 2)

`record` writes it; do not write it by hand. The replay refuses it unless every field is present,
the connected robot, firmware and SDK bindings match, and the requested gains are exactly the
approved pair.

```json
{
  "schema": 2,
  "practice": false,
  "robot": "booster_k1",
  "command_contract": "k1-upper-body/1",
  "date": "YYYY-MM-DD",
  "operator": "name",
  "firmware": "the robot_info version",
  "sdk_binding_version": "1.6.3",
  "robot_identity": {"serial_number": "from robot_info", "model": "K1"},
  "identity_source": "queried",
  "takeover_mode": "walking",
  "balance_during_upper_body": true,
  "balance_evidence": "hoist-supported",
  "release_verified": true,
  "rpc_ordering_verified": true,
  "signs_and_zero_offsets_verified": true,
  "zero_offsets": {"measured": ["..."], "visual": ["..."]},
  "fall_topic": {"kind": "periodic", "period_s": 0.1},
  "low_state_period_s": 0.002,
  "command_loss": {"behaviour": "what the arms did", "acceptable": true, "gap_s": 1.0},
  "approved": {"kp": 20, "kd": 1, "speed_max_rad_s": 0.6, "accel_max_rad_s2": 2.0},
  "notes": {"walking_with_custom_arms": "...", "head_owner": "..."},
  "evidence": {"probe": {"attempt": "001", "sha256": "..."}}
}
```

- A record from the fake robot has `"practice": true` and is always refused.
- Schema 1 records (written by hand before 2026-11-09) are refused; run the steps instead.
- Changing the bridge's frame or lifecycle bumps `command_contract`; the steps must be redone.

## Findings on the first real K1 (2026-11-15)

The first real run (K1 Education, firmware 1.8.1.2, SDK 1.6.3, robot held by a person instead of a
hoist at the owner's decision) passed S0 and S0r, and live arms followed the suit for about 35 s.
What the robot taught us, and what the tools now do about it:

- **Takeover settle.** For about 1-1.5 s after `UpperBodyCustomControl(true)` the K1 ignores arm
  commands and refuses the release (code 400). The tools hold the takeover pose for 1.5 s before
  any motion (commissioning `SETTLE_S`, replay and live arms `TAKEOVER_SETTLE_S`); quick-release
  waits 0.5 s; the release is retried 4 times, 0.5 s apart. Every attempt file records each
  takeover/release call's error text.
- **Stiffness.** kp 20 / kd 1 could not follow multi-joint motion (the upper-arm twist lagged
  0.3 rad). kp 40 / kd 2 tracks within 0.04-0.06 rad p95.
- **Speed.** Gains level **L4** (0.45 rad at 1.0 rad/s, 3.0 rad/s^2) exists for live arms; at
  0.6 rad/s a recorded performer outran the arms within 35 s. At 1.0 rad/s the right elbow still
  hit the cap on fast moves (lag stop).
- **Command loss.** With commands stopped for up to 1 s the arms drift only about 2 degrees.
- **Why live arms differ from the suit (offline analysis).** The robot tracks our commands within
  about 1 degree RMS with about 80 ms delay (kp 40). The gap is upstream: a recorded performer
  (Session-002 on K1) exceeds 1 rad/s 7 % and 3 rad/s^2 16 % of the time, and the live tool's
  One-Euro filter (2 Hz, beta 0.6) adds about 60-80 ms. Hence gains levels **L5** (0.75 rad at
  2.0 rad/s, 8 rad/s^2) and **L6** (1.0 rad at 3.0 rad/s, 12 rad/s^2; covers ~99 % of that take;
  the arm motors are rated 33.5 rad/s, 14 Nm), lag policy 0.5 rad / 0.5 s / 6 s (was 0.35 / 0.5 / 3),
  and for the robot run the live tool with `--filter-min-cutoff 3 --filter-beta 1.5`.
  Each live-arms log row now also records `target` (the performer's arm targets before the ramp,
  lag policy and envelope), so target -> command -> measured can be compared offline.
- **Takeover pose check.** `POSE_MATCH_RAD` is 0.10 rad (was 0.05): a hand-held robot sways.
- **Signs** move 0.2 rad (0.1 was below the hold error at kp 20; 0.3 hit the torso in the contact
  check).
- **New steps.** `tpose` (demo: arms out to the T-pose and back) and `sysid` (measurements for
  offline tuning: `--test sweep|steps|sine|poses`, one `--joint` at a time, at 3.0 rad/s and 12 rad/s^2).
- **Flight recorder.** `tools/control/k1_flight_recorder.py [out_dir] [net]` saves every LowState
  (about 500 Hz: all 22 motors' q, dq, ddq, tau_est, temperature, lost, and the IMU) to `.npz`
  chunks, read-only. Its `t_monotonic` matches the live bridge log's `t`.

## After day one

The stage ladder stays as a record of what passed: S0 (steps 1-11, the acceptance record), S0r
replay on the hoist (step 12), S1 live arms on the hoist (step 13), S1r replay standing, S2 live
arms standing. Since 2026-11-15 a stage that is not passed yet is a warning before Go, not a lock
(see the quick start in [K4 · live arms](#k4--live-arms-k1_live_bridgepy)). Keep the robot held
or on the hoist, and use a spotter when it stands free. Head tracking is in this release
([Head (K6)](#head-k6)); walking and whole-body tracking on the K1 are later work.

## K4 · live arms (`k1_live_bridge.py`)

The live tool sends each accepted, filtered K1 arm reference over UDP on this PC; the bridge
follows it behind the same envelope, motion lock and contact check as replay.

**Quick start (owner decision 2026-11-15).** One command from cold: without `--session` the bridge
runs the probe (read-only, a new session), auto-check 1 (robot: walking, not fallen, fresh joint
state, arm and head joints in range) and auto-check 2 (stream: fresh, calibrated, tracking on,
model, clock; the arm gap is shown), prints a summary and starts on your Go:
`--go`, else it asks once. No Go, no motion. Stage-gate problems and a missing acceptance record
are warnings in the summary; without a record the run uses attempt 049's settings (kp 40, kd 2,
2 rad/s, 8 rad/s^2, 50 Hz). A record that is given must still be valid. The attempt file stores
`owner_go` (operator, time, the warnings shown, the settings). Replay (`k1_arm_bridge.py`) works the
same way: warnings plus Go; a run that started with warnings writes no ledger PASS.

```bash
# 1. the live simulation with the stream on: in the Setup console, step 07, tick
#    "Send the arms to the K1 window"; or from a terminal (Docker, --network host):
source session.env
bash tools/xsens/live_operator.sh gmr --robot booster_k1 --robot-stream udp://127.0.0.1:47011
# 2. the bridge from cold (fake = practice robot), checks, summary, Go
.venv/bin/python tools/control/k1_live_bridge.py --transport booster --net <PC address> \
    --operator "<name>" --go [--acceptance-record <k1-acceptance.json>] [--no-head]
#    or in an existing session (no probe)
.venv/bin/python tools/control/k1_live_bridge.py --session <DIR> --stage hoist
# practice without the suit: a slow pattern instead of the live tool
.venv/bin/python tools/control/k1_live_synth.py --port 47011 --seconds 120
```

- **Stream clock (no operator step).** The live tool and the bridge must read one clock: both
  refuse to run in a time namespace with a monotonic offset (`/proc/self/timens_offsets`). The
  delay between the suit sample and this PC counts as age against the same 0.1 s bound at the
  start and during the run (attempt 046's 26 ms under load now runs). A packet stamped more than
  2 ms ahead of this PC's clock is a clock mismatch: refused before the takeover, a normal stop
  after it. The stream is also stale when the suit sample counter has not advanced for 0.1 s on
  this PC's clock, whatever the sender's time says.

- **One run = one takeover/release cycle.** The arms take over only when the stream is fresh,
  Calibrate was done and Tracking is on for 1 s. Then the robot's arms move to the performer's
  arm pose at 0.2 rad/s or less, from any gap, then follow. The performer holds still until the
  status says following. Until 2026-11-16 the start refused when the performer's arms were more
  than 0.4 rad from the robot's; a performer's natural rest pose can be further away than that
  (0.74 rad on the first live start on a real K1), so that limit was removed.
- **Stops.** STOP (Ctrl-C) or the time limit (default 120 s, max 600 s): the arms slow down, then go
  back to Booster. A new Calibrate, a restarted live tool or Tracking off also stops: start again.
  A performer more than 1.0 rad ahead for 1.0 s makes the arms hold, then re-align at up to
  1.0 rad/s; after 6 s of that they stop. Stream loss: hold at 0.1 s, release at 0.1 s + braking time + 0.1 s (the bridge
  refuses speed and acceleration caps that make this longer than 1.5 s).
- **Hoist.** With `--strap-not-clear` (window: untick **Strap clear of both arms**), shoulder
  pitch and roll stay within 0.6 rad of rest. The default is the strap clear, as in attempt 049.
- **Drills** are separate runs: `--drill stream-loss` (stop the live tool while the arms follow)
  and `--drill stop` (press STOP while the arms follow).
- **Stage PASS:** `--evaluate --operator "<name>"` checks the session (600 s of following in runs
  ended by STOP or the time limit, 5 cycles, every arm joint over half its range and at 80 % of the
  speed cap, tracking p95 under 0.15 rad, no unconfirmed release, contact or stale-state fault, both
  drills PASS) and writes the ledger entry. After a firmware or SDK change, 120 s and the drills
  requalify a stage that passed before.
- **In the test window** this is step 13 (see [The test window](#the-test-window)).

### Head (K6)

The K1 head follows the performer's head yaw (left, right) and pitch (up, down). Roll is ignored:
the K1 head has no roll.

- **How.** The live tool sends the two angles with the arm reference. The bridge sends them to
  Booster's `RotateHead(pitch, yaw)` at 50 Hz. Booster keeps control of the head: our arm command
  keeps the head gain at 0. Attempt 055 showed that `RotateHead` works while the arms are custom.
  The head has its own command thread and its own connection, so a slow head call never delays an
  arm command or the arm release.
- **On by default.** Switch it off with `--no-head`, or untick **Head follows** in the window. Then
  the bridge sends no head command at all.
- **Zero.** Calibrate with the performer's head straight in the calibration pose. That head
  position becomes the head's zero.
- **Limits.** Yaw ±0.95 rad, pitch −0.264…0.744 rad (positive pitch = looking down), speed 2 rad/s.
  The start ramps at 0.5 rad/s from the robot's real head position, so the head never jumps.
- **The head holds** (stays where it is) while the arms align, while the arms hold for lag, and
  when the stream is stale (no new packet for 0.1 s).
- **At the end.** After a normal end or STOP, the head returns to straight (one 1 s move, after
  the arms are released). The real K1 refuses this move (code 400) right after the arm release,
  so the bridge sends it again every 0.3 s, at most 10 times. After a fall or a fault there is
  no head command at all.
- **A head problem is not an arm stop.** If head commands fail for 0.5 s, only the head switches
  off for this run: the status shows `head off` and the arms continue.
- **First runs on the real robot (2026-11-16).** Robot on the hoist, the 2026-11-15 Xsens capture
  played back as the performer, arms and head: 60 s PASS, then 30 s PASS. In the 60 s run: 2916
  head commands, 0 failures; the head turned -45° to +42° and looked down up to 12°; command
  against measured, 95th percentile: 1.4° (turn), 0.3° (up/down). At the end the robot refused
  the head move (code 400) for about 1.2 s after the arm release; sent again, it was accepted
  on the fifth try. The joint data of the 30 s run shows that the head was almost straight
  before that: it went from 4.8° down to 0.6° in the 0.5 s after the arm release, with no head
  move accepted yet. So the robot seems to bring the head back by itself when the arms are
  released; the repeated move finishes it and the log ends with "confirmed". Whether the robot
  also brings a turned head (left/right) back by itself is not measured: the head was at 0°
  turn at the end of both head runs.
- **Direction check with a live performer (not yet done).** Hold the robot as for the arms, then turn the
  performer's head **slowly** left, right, up and down. Confirm that the robot's head looks the
  same way each time (performer looks left: the robot looks to its left; performer looks down: the
  robot looks down). If a direction is wrong, press STOP and run again with `--no-head` (or
  untick **Head follows**).

## Recordings of a run

Every live run from the K1 window leaves these recordings. Nobody has to start a recorder by hand.

| What | Where | Made by | When |
| --- | --- | --- | --- |
| **Suit data**: every UDP datagram the live tool receives, raw, with its receive time and sender | `artifacts/xsens-live/suit/<capture id>/` in the checkout (`/data/suit/` inside Docker) | the live tool, whenever it streams to the K1 (`--robot-stream`) | from the live tool's start to its end |
| **Robot data**: every joint state (about 500 Hz: position, speed, torque estimate, temperature, IMU); read-only | `<session>/robot-data/<time>/lowstate-*.npz` | the K1 window (`tools/control/k1_flight_recorder.py`) | real robot only: Quick start (from the moment both checks pass) and step 13 |
| **Bridge data**: commands and measured joints per tick, the attempt file | `<session>/attempts/` | the K1 live bridge | every attempt |

**How a run finds its suit data.** At Go the bridge looks in `artifacts/xsens-live/suit/` (option
`--suit-root`) for the capture whose `capture.json` has the stream's session id, and writes the
folder, the capture id and the Go time into the attempt file (`suit_recording`). At the end it adds
the release time. `not found` or `ambiguous` is a warning on the Go screen and in the attempt file;
it never stops the run. An attempt that was cut off (the bridge was killed) keeps its Go time and
`release_monotonic: "unknown"`.

**Zip session for the developers** adds `suit/<attempt>/` to the zip: the whole 10 s chunk files
from Go − 60 s to the release + 5 s (to the capture's end when the release is unknown),
`capture.json`, and `suit-recording.json`:

- `complete: true`: every datagram from Go − 60 s (or from the capture's first one) to release + 5 s
  is in the zip, with no gap in the sequence numbers.
- `complete: false`: `gaps`, `missing_chunks` or `unreadable` say what is missing.
- `complete: "unknown"`: the release time is unknown, or no datagram after release + 5 s arrived
  within 10 s.
- `missing_pre_roll_s`: the capture started later than Go − 60 s (not an error).

**Messages of the live tool, and what to do.**

| Message | Meaning | What to do |
| --- | --- | --- |
| `SUIT RECORDING to <folder>` | it records | nothing |
| `SUIT RECORDING INCOMPLETE: the disk does not keep up` | the disk is too slow; datagrams are lost from the recording (the robot run is not affected) | close other programs that write to the disk; tell the developers |
| `SUIT RECORDING recovered: ...` | it records again; the gap stays in the recording | nothing |
| `SUIT RECORDING OFF: only N GB free in <folder>. Delete old capture folders there.` | less than 10 GB free at the start, or less than 2 GB while it recorded | zip the sessions you still need, then delete old capture folders in that folder |
| `SUIT RECORDING INCOMPLETE: cannot write ...` | a disk error; it stopped recording | check the disk; tell the developers |

**Disk.** About 30 MB per minute (1.8 GB per hour). Nothing is ever deleted automatically: delete
old capture folders in `artifacts/xsens-live/suit/` yourself, after the sessions that use them are
zipped.

**Limits.** A crash of the live tool can lose up to 4096 datagrams that were waiting for the disk,
plus the one being written. A power loss or an operating-system crash can also lose what the
operating system had not written yet, and the last chunk can end in a part of a datagram (the
reader skips it). A gap in the sequence numbers proves a loss only when a later datagram survived;
a lost end cannot be seen. The 3 s age limit only triggers the warning.

**Not covered.** A bridge started from a terminal does not start the robot data recorder: start
`tools/control/k1_flight_recorder.py <folder> <PC interface> <DDS domain>` yourself. The practice
robot has no robot data.

**Play suit data back** (for example into the live tool on another PC):
`python tools/xsens/play_suit_recording.py artifacts/xsens-live/suit/<capture id> --to 127.0.0.1:5555`.
The datagrams then come from this PC: start the live tool with `--source-ip 127.0.0.1`.

**Calibrate pose check.** When either thigh is more than 35° from vertical at Calibrate, the live
tool says `the performer is not standing (thigh N° from vertical). Stand up, arms down, look
straight ahead, then Calibrate again.` It is a warning: the calibration is still applied.
`pose check unavailable` means the suit sent no thigh data. With `--control-stdin`
(the Setup console) the live tool answers each request with one
`STATUS {"calibration_result": {...}}` line, and the recording state is the `suit_recording`
field of its `STATUS` lines.
