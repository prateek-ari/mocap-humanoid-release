# Unitree G1 EDU hardware bring-up runbook

This is the robot-side counterpart to the Xsens commissioning procedure in
[the setup guide](engineer-setup-guide.md) and [the Windows runbook](windows-xsens-humanoid-setup.md):
same standard (gate table, pass conditions, hard-stop column, evidence retained per gate). Keep the
G1 and Xsens bring-ups independent: the suit must pass its own gates (stream acceptance, calibration)
before it is ever coupled to the robot, and the robot must pass every gate below before it accepts
*any* motion clip — recorded, replayed, or live.

This runbook assumes hardware has not yet arrived. Nothing here can be executed or
rehearsed against real hardware from this repository today; it is the procedure the first bring-up
session must follow, written against what the codebase already implements and tests
hardware-free (`mocap_g1/safety.py`'s `ArmSafetyEnvelope`, `mocap_g1/schema.py`'s canonical joint
list, `make hardware-free-checks`) plus the tooling currently in progress.

Steps marked **(two people)** require a second person for the entire step, not just standby —
one operator at the controls/keyboard, one dedicated spotter with a clear, unobstructed path to the
robot and to the physical/software stop. Never run a step marked **(two people)** solo.

Store photographs, serial numbers, firmware screenshots, measured joint data, and session logs
outside Git or under the ignored `artifacts/` directory, as for the Xsens session evidence.

## Acceptance gates

| Gate | Pass condition | Hard stop |
|---|---|---|
| SKU / inventory | Delivered unit is confirmed EDU (SDK-entitled), 29-DOF configuration with 3-DOF wrists (roll+pitch+yaw) on both arms matching `mocap_g1.schema.G1_29DOF_JOINTS`; compute option, end effector, battery/spares, and SDK entitlement match the written quote | Base (non-EDU) unit, 23-DOF/1-DOF-wrist variant, missing SDK entitlement, or any inventory item not matching the quote |
| Network / DDS bring-up | Host reaches the robot's DDS domain; official high-level state/loco examples run and receive live topics | No DDS discovery, wrong domain/interface, or example receives no traffic |
| Firmware / MotionSwitcher | Firmware build matches the vendor-confirmed version; MotionSwitcher confirms low-level control mode selected and the factory sport/balance controller is released (not concurrently active) | Unconfirmed/mismatched firmware, MotionSwitcher fails to release the factory controller, or any dual-control state where both the factory controller and `rt/arm_sdk` could command motors at once |
| `rt/lowstate` sanity | `rt/lowstate` (joint positions/velocities, `rt/secondary_imu`) and motor temperatures read at the documented rate with plausible, non-zero, non-stale values over a sustained window | Missing/stale/zero-valued fields, implausible encoder jumps, or a temperature warning |
| Damping-mode reflex drill | Operator engages damping mode / the software stop path within the abort table's response-time budget (§5), robot visibly stops commanding further motion, and a stalled control loop independently trips the coded `ArmSafetyEnvelope` fail-closed path | Any trigger condition without a confirmed stop within budget, or the robot continuing to move after the stop signal |
| Harness / clear-area | Physical harness/gantry rated and attached (or supervised free-standing only once explicitly authorized by an earlier phase's exit criteria), clear-area radius enforced, spotter positioned with an unobstructed stop path | No harness for any test where free-standing has not yet been authorized, obstructed stop path, or bystanders inside the clear-area radius |
| Joint zeroing & sign/index verification | Every one of the 17 `G1_ARM_SDK_JOINTS` (waist + both arms) individually commanded to a known small angle, externally measured, and confirmed to match both sign and index order predicted by `mocap_g1.schema.G1_29DOF_JOINTS` and the model FK | Any joint whose measured motion direction, magnitude, or index disagrees with the model — see the mandatory note in §7 |
| Tiny-amplitude arm test | Official arm-SDK example (or the arm bridge CLI once available) tracks a tiny-amplitude reference within the configured tracking/torque bounds, with no envelope violation, self-collision, or unexpected joint motion | Tracking error, torque saturation, or joint-limit/self-collision violation beyond the abort thresholds in §8; any motion on an unmapped joint |
| Per-session operator preflight | Every item in the §9 checklist is confirmed and logged immediately before *each* session, not just the first | Any unchecked or stale (>1 session old) preflight item |

## 1. Prepare before the session **(two people)**

- Confirm both operator and spotter are present, briefed on the abort table (§5), and know the
  physical/software stop locations before power-up.
- Verify `make hardware-free-checks` passes on the current commit (this is a source-level gate, not
  a substitute for anything below — it proves the safety envelope and joint schema are internally
  consistent, not that the robot is safe).
- Confirm the clear-area radius, harness/gantry rating, and that no unrelated personnel or equipment
  are inside the radius for the duration of the session.
- Have a stopwatch, goniometer/protractor (or equivalent external angle reference), and the session
  log ready.

## 2. SKU / inventory verification

1. Record the unit's serial number, delivered SKU code, and compute-option/end-effector variant from
   the packing list and the physical unit's labeling.
2. Confirm EDU entitlement (SDK access) in writing against the purchase quote — a base G1 cannot
   run this project's control path at all.
3. Confirm the 29-DOF configuration: count and identify every joint against
   `mocap_g1.schema.G1_29DOF_JOINTS` (29 entries: 12 leg, 3 waist — yaw/roll/pitch — and 7 per arm —
   shoulder pitch/roll/yaw, elbow, wrist roll/pitch/yaw). Specifically confirm **3-DOF wrists**
   (roll, pitch, *and* yaw) on both arms; a 1-DOF-wrist or 23-DOF unit does not match this project's
   target and cannot preserve wrist orientation from the retargeted Xsens capture.
4. Photograph and record condition of the robot, battery/spares, charger, harness/gantry hardware,
   and any bundled end effector.
5. Compare every item against the written quote; log discrepancies before proceeding.

## 3. Network and DDS bring-up

1. Connect the robot to the designated network segment per the vendor's documented topology.
2. Confirm the host reaches the robot's DDS domain (interface, domain ID, and any required network
   interface name match the vendor documentation).
3. Run the official high-level state/loco example and confirm it receives live state — this proves
   the DDS path end-to-end without touching low-level joint control.
4. Record the interface, domain ID, and observed topic rate for the session log.

## 4. Firmware and MotionSwitcher mode check

1. Read and record the robot's firmware/build version; compare against the vendor-confirmed version
   for this unit. Do not proceed on an unconfirmed or unexpectedly different build.
2. Confirm the factory motion service (sport/balance/AI mode) is active and healthy in its own right
   before attempting to release it — a service that is already unhealthy should not be trusted to
   release cleanly.
3. Use the official MotionSwitcher service to select low-level control mode and confirm the factory
   controller has released the motors. Read back MotionSwitcher's own status rather than assuming
   the request succeeded.
4. **Hard stop if MotionSwitcher cannot confirm release**, or if there is any indication the factory
   controller and `rt/arm_sdk` could both be issuing commands concurrently — this is the single most
   dangerous mode-confusion failure in this bring-up and must never be worked around.

## 5. `rt/lowstate` sanity gate

1. Subscribe to `rt/lowstate` and `rt/secondary_imu` and confirm every joint's position/velocity and
   the IMU update at the documented rate, with no stale-timestamp or dropped-packet warnings, over a
   sustained window (recommend at least 30 s, matching the Xsens runbook's own 30 s stream-acceptance
   window for consistency).
2. Confirm every reported joint position is a plausible value for the robot's current physical pose
   (compare a handful of joints visually against the readback) and that no motor temperature reading
   is already elevated at idle.
3. Do not proceed past this gate on any missing, stale, zero-valued, or implausible field — a
   feedback path that cannot be trusted at idle cannot be trusted once commands are being sent.

## 6. Damping-mode reflex drill **(two people)**, with written abort table

This drill exercises the *human* reflex path (operator/spotter reaction to an unexpected event) and
must be measured with a stopwatch, separately from the *software* fail-closed path
(`mocap_g1.safety.ArmSafetyEnvelope`, already covered hardware-free by
`tools/control/replay_g1_faults.py` / `make arm-safety-check` — see the coded defaults below, which
this drill's response budget is deliberately set inside of, not against).

Coded envelope defaults for reference (`mocap_g1/safety.py`, `SafetyConfig`): a stale command source
(`stale_timeout_s = 0.10` s) is already zero-blended out automatically, and a source silent for
`hard_timeout_s = 0.50` s or a stalled control loop latches a hard timeout that requires a fresh
measured-state reset before any further command is accepted. The human reflex path below must not
rely on those numbers alone — it is the operator's job to independently stop the robot, not to wait
out the software timeout.

Written abort table (fill in and post at the operator station before every session; the times below
are the required maximum operator response, not a target to aim for):

| Trigger condition | Required operator action | Max response time | Verified by |
|---|---|---|---|
| Spotter calls "stop" | Engage damping mode / software stop path immediately | 2.0 s | Stopwatch, spotter confirms robot stops commanding motion |
| Unexpected joint motion outside the commanded reference | Engage damping mode / software stop path immediately | 2.0 s | Stopwatch, operator log |
| Operator loses visual on the robot | Engage damping mode / software stop path immediately | 1.0 s | Stopwatch |
| Command source disconnected (network pull, process kill) | Confirm the envelope's own stale/hard-timeout fail-closed behavior engages *and* manually engage damping mode regardless | 2.0 s (human) / independently verify the `hard_timeout_s = 0.50 s` coded bound also tripped | Stopwatch + session log cross-check against `rt/lowstate` |
| Any motor temperature or fault flag | Engage damping mode / software stop path immediately, do not attempt to clear the fault before stopping | 2.0 s | Stopwatch |

Run every row above at least once per session before any coupled motion test, with the spotter
timing the operator and logging pass/fail against the budget. A single missed row is a hard stop for
the rest of the session.

## 7. Harness / clear-area procedure **(two people)**

1. Attach the rated harness/gantry per the vendor procedure; confirm load rating and attachment
   points before the first powered motion. Do not proceed to free-standing tests until an earlier
   phase's exit criteria have explicitly authorized it (gantry → assisted → free-standing, in that
   order, never skipped).
2. Mark and enforce a clear-area radius around the robot; the spotter's job for the remainder of the
   session is to keep that radius clear and to hold an unobstructed path to the physical/software
   stop at all times, not to operate any other equipment.
3. Confirm the spotter and operator have rehearsed the abort table (§6) inside this same clear-area
   configuration before any joint is commanded.

## 8. Joint zeroing & sign/index verification (mandatory before any motion clip)

**Do this before running any recorded or live motion clip on hardware, every time the arm bridge or
its joint mapping changes, and again at the start of a new hardware session as part of §9.** A
mirrored sign (e.g. commanding `+`, robot moves `-`) or a swapped index between two joints of similar
range can pass every other gate in this document and still command a physically wrong, potentially
unsafe motion the moment a real clip is played — the safety envelope in `mocap_g1/safety.py` clamps
position/velocity/acceleration per joint, but it has no way to know a *labeled* joint is actually the
*wrong* joint. This gate exists specifically to close that hole; it is not optional.

Procedure, once per joint, for all 17 entries of `mocap_g1.schema.G1_ARM_SDK_JOINTS` (waist_yaw,
waist_roll, waist_pitch, then both arms' shoulder_pitch, shoulder_roll, shoulder_yaw, elbow,
wrist_roll, wrist_pitch, wrist_yaw — the same order the controller screen in
`mocap_g1/controller_screen.py` already controls as a block, columns 12+ of the canonical 29-DOF
`qpos`):

1. With the robot damped and the operator/spotter both ready, command *one* joint at a time to a
   small, known positive angle (recommend 0.05–0.10 rad, well inside `SafetyConfig`'s
   `position_margin_rad`/`velocity_limit_fraction` defaults) via the arm bridge CLI (generic
   reference — see `tools/control/g1_arm_bridge.py`, in progress; do not assume specific flags beyond
   what that tool documents when it lands). Every other joint's target stays at the current measured
   position.
2. Externally measure the resulting joint angle (goniometer/protractor against a fixed reference, or
   an equivalent external measurement) — do not trust only the commanded value.
3. Independently read `rt/lowstate`'s reported angle for that joint.
4. Compute the expected direction and approximate magnitude from the model's forward kinematics for
   that same commanded delta.
5. Record all four values (commanded, externally measured, `rt/lowstate`-reported, model-expected) in
   a table row for that joint. The joint passes only if sign and index both agree: the *external*
   measurement moves in the *model-expected* direction, `rt/lowstate` agrees with the external
   measurement within the expected tolerance, and no *other* joint moved.
6. Return the joint to its starting position before moving to the next joint. Never test two joints
   simultaneously in this procedure — a simultaneous test cannot distinguish a swapped index from two
   independently-correct joints.
7. Any single failing joint is a hard stop for the whole session: do not proceed to §9's tiny-amplitude
   test, and do not run any motion clip, until the mapping is corrected and every joint re-passes.

## 9. Tiny-amplitude arm test with abort criteria

1. With every joint from §8 passed, run the official arm-SDK example (or the arm bridge CLI, once
   available) at tiny amplitude — the smallest reference motion that is still externally observable,
   well below any clip intended for later playback.
2. Ramp the SDK blend/weight from 0→1 slowly and hold; confirm no unexpected motion during the ramp
   itself (ramp the SDK blend/weight from 0→1 and back out smoothly).
3. Abort immediately (engage damping mode per §6's table) on any of:
   - tracking error or torque saturation beyond the bounds the controller screen already checks
     hardware-free (`mocap_g1/controller_screen.py`'s `tracking_p95_limit_rad` /
     `tracking_max_limit_rad` / `saturation_fraction_limit` — treat those simulator-side thresholds
     as the hardware ceiling too, not a looser one),
   - any motion on a joint that was not commanded,
   - any self-collision or joint-limit indication,
   - any fault flag or unexpected `rt/lowstate` discontinuity.
4. Ramp the blend weight back to 0 and confirm the robot holds its final commanded pose (not the
   factory controller's own pose) before releasing control.
5. Log the full session (commanded reference, `rt/lowstate` readback, any abort triggered) to
   `artifacts/` before ending the session.

## 10. Per-session operator preflight checklist

Confirm and log every item immediately before *each* session (not just the first):

- [ ] Firmware version unchanged since last confirmed (§4), or re-verified if changed
- [ ] MotionSwitcher confirms low-level mode selected, factory controller released (§4)
- [ ] `rt/lowstate` / `rt/secondary_imu` sanity gate re-run and passing (§5)
- [ ] Damping-mode reflex drill re-run this session, every abort-table row passed within budget (§6)
- [ ] Harness/gantry attached and rated for today's test tier; clear-area radius enforced,
      spotter present with unobstructed stop path (§7)
- [ ] Joint zeroing & sign/index verification re-run if the arm bridge, its joint mapping, or the
      robot's calibration changed since the last pass (§8) — otherwise confirm the last pass is still
      on file and unchanged
- [ ] Operator and spotter have both reviewed the abort table (§6) out loud before power-up
- [ ] `make hardware-free-checks` passes on the commit in use today

## Evidence to retain

- SKU/serial/inventory record and condition photographs
- firmware version and MotionSwitcher confirmation logs
- network/DDS configuration and observed topic rates
- `rt/lowstate` sanity window logs
- damping-mode reflex drill timings against the written abort table, per session
- harness/gantry rating record and clear-area configuration
- joint zeroing & sign/index verification table (all 17 joints, all four measured values), re-run
  date, and who ran it
- tiny-amplitude test logs (commanded reference, readback, any abort) and any resulting video
- per-session preflight checklist, signed by operator and spotter
