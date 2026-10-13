# Standing-balance screen, torque feasibility, and verified controller gains

**Date:** 2026-08-16
**Scope:** `mocap_g1/balance_screen.py`, `tools/control/screen_g1_balance.py`,
`eval/metrics.py` (torque feasibility), `mocap_g1/controller_screen.py` (gains).
**Status:** hardware-free. No G1 has been touched; every number below comes
from MuJoCo or from vendor files.

---

## 1. Why this exists

The project's top physical risk is *"arm motion destabilizes the standing robot."*
Until today that risk had **zero simulation evidence**. The only controller
evidence in the repo was `mocap_g1/controller_screen.py`, which deletes the
pelvis free joint — it welds the robot to the world, so by construction it
cannot tip over and cannot say anything about balance.

`mocap_g1/balance_screen.py` is the floating-base companion. Same MJCF, same
recorded NPZ format, pelvis free joint **retained**, robot standing on the
model's ground plane, real `mj_step` physics.

It is a **screen, not balance validation.** See §6 before quoting any result.

---

## 2. What the screen does

1. Loads `g1_mocap_29dof.xml` through `MjSpec` with the pelvis free joint
   intact, sets the integrator to `implicitfast`, and clamps every actuator's
   `ctrlrange` to `min(MJCF actuatorfrcrange, vendor URDF effort)` ×
   `torque_limit_fraction`.
2. Places the robot in a standing stance and drops the pelvis so the lowest
   sole point sits exactly on the floor. **The recorded root trajectory is
   ignored** — this screen asks "does the arm motion topple a standing robot",
   not "does the recorded root pose make sense".
3. Holds the twelve leg joints and three waist joints at that stance with
   stiff joint-space PD; drives the fourteen arm joints along the reference
   with Unitree's published `arm_sdk` gains.
4. Settles for `settle_seconds`, then steps the reference and records
   per-frame balance metrics.

Control law, every physics step, for all 29 joints:

```
tau = kp * (q_target - q) - kd * dq          # clipped to the actuator cap
```

**No gravity/bias feedforward.** This matches Unitree's `MotorCmd` semantics
(the arm_sdk examples send `dq_des = 0` and `tau_ff = 0`), and it is also the
only thing that works: feeding `data.qfrc_bias` forward — which is computed for
an *unsupported* floating base — destabilised the stance hold outright and the
model toppled within 300 ms in every gain combination tried.

### Stance

`hip_pitch = -0.10`, `knee = +0.30`, `ankle_pitch = -0.20`, everything else 0.
These are Unitree's published G1 `default_angles` from
[`unitree_rl_gym/deploy/deploy_real/configs/g1.yaml`](https://github.com/unitreerobotics/unitree_rl_gym/blob/main/deploy/deploy_real/configs/g1.yaml).
The MJCF ships **no keyframe**, so a documented default was required; using the
vendor's own deployment default is the least arbitrary choice available. All
three angles are CLI-configurable.

### Gains

| Group | kp | kd | Provenance |
|---|---|---|---|
| Arms (14 joints) | 60 | 1.5 | **Verified published** — Unitree `arm_sdk` examples |
| Hip / knee | 300 | 18 | Screening surrogate, **not** published |
| Ankle pitch + roll | 600 | 36 | Screening surrogate, **not** published |
| Waist | 300 | 18 | Screening surrogate, **not** published |

The report echoes this split explicitly
(`gains.arm_gain_verification_status = "verified_published"` vs
`gains.stance_gain_verification_status = "screening_surrogate_not_published"`)
so nobody can mistake the surrogate half for vendor data.

**Why the leg gains are ~15× Unitree's published deployment values.** A
standing humanoid is an inverted pendulum. Linearised about the ankle its
*destabilising* gravitational stiffness is `m·g·h_com` ≈ 34.4 kg × 9.81 × 0.70 m
≈ **236 N·m/rad**. A joint-space ankle PD only resists tipping through that
same joint, so the two ankles' combined stiffness has to exceed 236 N·m/rad or
the model simply falls over — and Unitree's published deployment ankle gain is
40 N·m/rad per ankle (80 total). Measured, at `stance_damping_ratio = 0.06`:

| ankle kp (each) | Result on a motionless 2 s hold |
|---|---|
| 40 (published deployment value) | topples in < 0.5 s |
| 100 / 200 | topples |
| 400 | holds, settled tilt 0.077 rad |
| **600 (default)** | holds, settled tilt 0.040 rad |
| 800 | holds, settled tilt 0.026 rad |

600 is the smallest round value that holds with the tilt comfortably inside
the attitude gate. This gain is standing in for the factory whole-body
controller's authority, not modelling a G1 motor.

### Timestep

`timestep_s` defaults to **1 ms** and `validate()` refuses anything larger.
2 ms (the MJCF default, and what `controller_screen.py` uses) is *not*
numerically stable with these stance gains — the ankle PD diverges and the
model topples on a motionless hold, which would make every run a false
BLOCKED. Measured: 2 ms topples, 1 ms and 0.5 ms both hold with identical
settled values.

---

## 3. Gates and default thresholds

All thresholds are CLI-configurable. Reference values below are from a
motionless hold with the defaults (settled pelvis tilt 0.040 rad, pelvis
z 0.7833 m).

### Gate 1 — CoM ground-projection margin (`com_margin_min_m`, default 0.02 m)

The support polygon is the convex hull of the sole contact points currently
within `contact_height_m` (1 cm) of the floor. The G1 MJCF gives each foot
four 5 mm contact spheres at `x ∈ {-0.05, +0.12}`, `y = ±0.025/±0.03` in the
ankle-roll frame, so a flat double stance yields an eight-point hull. The
metric is the signed distance from `data.subtree_com[0]`'s ground projection
to that hull (positive inside).

*Rationale for 0.02 m.* The nominal standing margin is **+0.051 m**, and the
binding direction is the toe edge — the G1's CoM in this stance sits ~0.09 m
ahead of the ankle axis with only ~0.125 m of foot in front of it. A 0.02 m
floor keeps roughly 40 % of the nominal margin in reserve, which covers the
foot-placement and inertial-model error a visualisation MJCF carries, while
still leaving a wide band before the physical edge. Two structural failures
share this gate: `support_polygon_lost` (fewer than `min_support_points` = 3
sole points on the ground — a foot has lifted) and `com_margin_unmeasurable`.

*Honesty note.* Outside the polygon the returned value is the minimum signed
distance to the hull's edge *lines*, which can understate the magnitude near a
vertex. The sign — which is what the gate uses — is always correct.

### Gate 2 — Ankle torque headroom (`ankle_torque_utilization_max`, default 0.70)

Realised `actuator_force` on the four ankle joints against their applied cap.
The cap is `min(MJCF actuatorfrcrange, vendor URDF effort)` — **35 N·m**, not
the MJCF's more permissive 50 N·m. The vendor URDF is the number quoted
externally, so the screen grades against it.

*Rationale for 0.70.* A motionless hold already consumes **0.35** of the
35 N·m budget (~12 N·m per ankle, which is just `m·g·d` for a CoM ~0.09 m
ahead of the ankle — this is real G1 geometry, not a modelling artifact).
0.70 is exactly 2× that static draw: it allows the arm motion to double the
ankle's static workload before complaining, and still reserves 30 % for the
balance authority, joint friction and thermal derating that a rigid ideal
torque source does not model. Above 0.70 the remaining margin is too thin to
claim anything about a real machine.

### Gate 3 — Pelvis attitude

| Sub-check | Default | Nominal | Rationale |
|---|---|---|---|
| `pelvis_tilt_max_rad` | 0.15 (8.6°) | 0.040 | 3.7× the static lean of the stiff-PD stance; far below any plausible recovery limit, and clearly separated from the ≥ 0.5 rad seen in a fall |
| `pelvis_angular_speed_max_rad_s` | 1.0 | 0.08 | ~12× nominal, ~5–8× below the 5–9 rad/s a toppling run produces — a wide, unambiguous band |
| `pelvis_height_drop_max_m` | 0.05 | 0.0001 | a 5 cm squat is far beyond what a held stance should ever do; catches leg collapse that tilt alone misses |

A hard abort (`abort_tilt_rad` 0.60, `abort_height_drop_m` 0.20) stops the sim
once the robot is unambiguously falling, so a bad clip costs a fraction of a
second instead of a full run. `events.aborted_step` records where.

### Measured separation

Symmetric shoulder-pitch + elbow swing on a 1 s clip, defaults throughout:

| Motion | Verdict | CoM margin (m) | Ankle util | Tilt (rad) | ω (rad/s) |
|---|---|---|---|---|---|
| static hold | PASS | +0.051 | 0.35 | 0.040 | 0.08 |
| 0.25 Hz / 0.15 rad | PASS | +0.051 | 0.35 | 0.040 | 0.07 |
| 0.50 Hz / 0.30 rad | PASS | +0.051 | 0.37 | 0.040 | 0.15 |
| 1.50 Hz / 1.00 rad | **BLOCKED** | −0.241 | 1.00 | 0.602 | 5.12 |
| 2.50 Hz / 1.40 rad | **BLOCKED** | −0.250 | 1.00 | 0.511 | 8.64 |

The screen is quiet on gentle upper-body work and unambiguous on a swing that
throws the CoM off the feet. Nothing sits in a grey zone at these thresholds.

---

## 4. Running it

```bash
python3 tools/control/screen_g1_balance.py path/to/clip.npz \
    --model "$MOCAP_WS/GMR/assets/unitree_g1/g1_mocap_29dof.xml" \
    --output artifacts/balance/clip.json
```

Input is the same preview-cache NPZ the other `mocap_g1` tools consume
(`qpos [T,36]`, `fps`, JSON `metadata`), read through
`mocap_g1.conditioning.load_qpos_npz`. `--model` defaults to
`$MOCAP_WS/GMR/assets/unitree_g1/g1_mocap_29dof.xml` when `MOCAP_WS` is set.

Exit code 0 = PASS, 1 = BLOCKED, 2 = error. The tool refuses to overwrite an
existing report. Useful flags:

```
--start-seconds / --end-seconds     clip a window out of a long take
--settle-seconds                    stance settle before metrics (default 1.0)
--speed-factor                      replay the reference faster/slower
--com-margin-min-m                  gate 1 threshold
--ankle-utilization-max             gate 2 threshold
--pelvis-tilt-max-rad               gate 3 tilt threshold
--pelvis-angular-speed-max-rad-s    gate 3 angular-speed threshold
--ankle-kp / --leg-kp / --waist-kp  stance surrogate stiffness
--arm-kp / --arm-kd                 arm gains (default = published arm_sdk)
--effort-urdf                       effort-limit source (default: vendored official URDF)
```

Report shape follows `controller_screen.py`: `balance_screen.{status,reasons}`,
a `hardware_candidate` verdict that stays BLOCKED on preview calibration, a
full `configuration` echo, per-gate worst values under `gates`, and a
`limitations` list.

---

## 5. Companion changes

### 5.1 Torque/effort feasibility in `eval/metrics.py` (additive)

`parse_urdf_limits` now also reads `<limit effort=...>` (a new key on the
returned dict; `lower`/`upper`/`velocity` are untouched), `JointInfo` gains
`effort_limit` and `dof_adr`, and a new `torque_feasibility()` metric is wired
into `score_motion_file` under the new `"torque_feasibility"` key. **No
existing metric name, gate semantic, or output field changed** — verified by
running a clip end-to-end and diffing the top-level key set.

> Update 2026-10-12: `torque_feasibility` is **informational only**. No gate
> consumes it; the score report marks it `gated: false` with a note.

Method: `mj_inverse` per reference frame, with `qvel` from
`mj_differentiatePos` (quaternion-correct for the free root) and `qacc` as a
central difference of that `qvel`. Contact is disabled for the pass and
restored afterwards.

The G1's effort limits are wildly asymmetric, and the report is built around
that fact:

| Joints | Effort |
|---|---|
| knee | 139 N·m |
| hip, waist_yaw | 88 N·m |
| ankle pitch/roll, waist roll/pitch | 35 N·m |
| shoulder, elbow, wrist_roll | 25 N·m |
| **wrist_pitch, wrist_yaw** | **5 N·m** |

Wrist pitch/yaw at 5 N·m is a **5× tighter budget than the shoulders** — so a
wrist-heavy mocap take is the likeliest first saturation on hardware, and the
report surfaces `worst_joint` ranked by *utilisation* (fraction of that
joint's own limit) precisely so a 4.6 N·m wrist outranks a 40 N·m knee.

A frame passes when every gated joint stays within `effort_frac` × effort
(default **0.80** — 20 % reserve for balance-controller effort, friction and
thermal derating that a rigid-body estimate does not model). `gate_joints`
defaults to `"upper_body"` (waist + arms = the arm_sdk-driven set, 17 joints);
`"all"` grades the legs too.

Structural limitations, in the report's own `limitations` list:

* **No ground reaction force** (contact disabled). Leg torques are therefore
  only meaningful in flight, which is exactly why they are reported but not
  gated by default. `root_residual_wrench` records how unsupported the base
  was, so the reader can see the size of the omission.
* Rigid-body only: no rotor inertia, gearbox friction, or motor speed/torque
  derating.
* `qacc` is a double finite difference and amplifies reference jitter — read
  it alongside `jerk` / `discontinuity_frames`.

### 5.2 Verified controller gains in `mocap_g1/controller_screen.py`

The assumed `kp = 10.0`, `kd = 0.2` are replaced with Unitree's **published**
`arm_sdk` example gains, `kp = 60.0`, `kd = 1.5`. The same pair appears
verbatim in all four official G1 arm_sdk examples and is applied to every arm
*and waist* joint the SDK drives — exactly the joint set this screen controls:

* <https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/high_level/g1_arm7_sdk_dds_example.cpp>
* <https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/high_level/g1_arm5_sdk_dds_example.cpp>
* <https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/g1/high_level/g1_arm7_sdk_dds_example.py>
* <https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/g1/high_level/g1_arm5_sdk_dds_example.py>

Gains stay config-driven. The old pair is preserved as `LEGACY_ASSUMED_KP` /
`LEGACY_ASSUMED_KD`, and every report now carries an additive
`controller_gains` block recording `verification_status` — one of
`verified_published`, `legacy_assumed`, or `operator_override` — plus the
source URLs.

**Side effect that had to be fixed: 6× the stiffness broke the integrator.**
kp 10 → 60 raises the wrist PD natural frequency `sqrt(kp/M_ii)` by √6 ≈ 2.5×,
to ω = 405 rad/s on the worst joint (`wrist_roll`, `M_ii` = 3.664e-4 kg·m²).
A single 2 ms explicit step no longer integrates it:

| physics dt | ω·dt | wrist_roll peak tracking error |
|---|---|---|
| 2.0 ms | 0.81 | 0.617 rad — divergent |
| 1.0 ms | 0.40 | 0.240 rad — oscillating |
| 0.5 ms | 0.20 | 0.012 rad — clean |

Left alone the screen would have reported `tracking_max_exceeded` and blamed
the *motion* for an integrator failure. `timestep_s` is now the **control and
metric period**, and physics is integrated in `physics_substeps` equal
sub-steps inside it, chosen automatically so `ω·dt ≤ pd_stability_product_max`
(default 0.25, the smallest round cap admitting the verified-good 0.20 point).
At the default 2 ms control period this resolves to 4 sub-steps of 0.5 ms.
Metric cadence, array shapes and every existing report field are unchanged;
the resolved values are echoed in a new additive `numerics` block. Pin
`physics_substeps=1` with the legacy gains to reproduce pre-2026-08-16 reports
exactly.

**Follow-up owed by whoever owns `tools/control/qualify_g1_upper_body.py`:**
that CLI still hard-codes `--kp 10.0 --kd 0.2` as argparse defaults, so it
overrides the new module defaults and will report
`verification_status: legacy_assumed`. Changing those two defaults to 60.0 /
1.5 is the whole fix; it was out of scope for this change.

---

## 6. Limitations — read before quoting any result

1. **This is not balance validation.** Legs and waist are held by stiff
   joint-space PD, not the factory whole-body balance controller. PASS means
   "this clip did not topple the surrogate", nothing more.
2. **The stance gains are invented.** They are ~15× Unitree's published
   deployment values, chosen only because a joint PD below the
   inverted-pendulum stiffness `m·g·h` cannot hold a standing pose at all.
   They are a stand-in for the missing balance controller.
3. **The screen is conservative in the sagittal plane.** It models no recovery
   step, no hip strategy, no CoM/ZMP feedback and no arm-swing counterbalance.
   A real G1 may well survive motions this screen blocks. Treat a BLOCKED
   verdict as "needs work before hardware", not as proof of a fall.
4. **The screen is optimistic about actuation.** Actuators are ideal torque
   sources: no transport delay, no 50 Hz command quantisation (arm_sdk
   publishes at `control_dt = 0.02`), no motor thermal derating, no gearbox
   backlash or compliance.
5. **Ground and root are idealised.** Flat, high-friction floor; the recorded
   root trajectory is ignored and the robot always starts from a nominal
   standing pose.
6. **Inertia comes from a visualisation MJCF.** `g1_mocap_29dof.xml` is GMR's
   retargeting model, not a vendor dynamics model. Absolute torque and CoM
   numbers inherit whatever error that carries.
7. **The eval torque gate ignores ground reaction forces**, so its leg numbers
   are flight-phase only (§5.1).
8. Everything here is hardware-free. Nothing has been confirmed on a G1.
