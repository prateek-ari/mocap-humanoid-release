# G1 velocity-limit source decision

Date: 2026-08-04. This file documents the limit source used by `eval/metrics.py`.

## Background

MuJoCo MJCF has no joint-velocity-limit field, so Joint Feasibility reads velocity from a URDF by
joint name. Two G1 URDFs were available:

- **GMR companion:** `$MOCAP_WS/GMR/assets/unitree_g1/g1_custom_collision_29dof.urdf`
- **Official Unitree:** `eval/assets/g1_29dof_official.urdf`, vendored from
  `unitree_ros/robots/g1_description/g1_29dof.urdf`

Position limits match on all 29 joints. Velocity and effort disagree on eight joints.

## Eight-joint disagreement

| Joint | GMR companion (rad/s) | Official (rad/s) |
|---|---:|---:|
| `left_hip_roll_joint` | 20.0 | 32.0 |
| `right_hip_roll_joint` | 20.0 | 32.0 |
| `left_ankle_pitch_joint` | 37.0 | 30.0 |
| `right_ankle_pitch_joint` | 37.0 | 30.0 |
| `left_ankle_roll_joint` | 37.0 | 30.0 |
| `right_ankle_roll_joint` | 37.0 | 30.0 |
| `waist_roll_joint` | 37.0 | 30.0 |
| `waist_pitch_joint` | 37.0 | 30.0 |

The effort values diverge on the same joints, indicating different actuator assumptions rather than
an isolated typo.

## Decision

**The official Unitree URDF is the default for both position and velocity feasibility.**

Rationale:

1. Published claims should trace to the OEM mechanical specification.
2. One official source for both halves of Joint Feasibility avoids a hidden two-URDF asymmetry.
3. The vendored file was rechecked against the upstream Unitree file at decision time.

`LEGACY_COMPANION_URDF` remains available only to reproduce older score runs. Pass it explicitly via
`eval/score_motion.py --urdf`; it is never selected by default.

## Effect

- Hip-roll velocity becomes less restrictive: 20 → 32 rad/s.
- Ankle pitch/roll and waist roll/pitch become more restrictive: 37 → 30 rad/s.
- Every score JSON records the resolved URDF source and matched-joint count.

## Verification

```bash
curl -sS -o /tmp/g1_official.urdf \
  https://raw.githubusercontent.com/unitreerobotics/unitree_ros/master/robots/g1_description/g1_29dof.urdf
tr -d '\r' < /tmp/g1_official.urdf > /tmp/g1_official_lf.urdf
diff /tmp/g1_official_lf.urdf <(tail -n +2 eval/assets/g1_29dof_official.urdf)
```

The vendored file carries one leading provenance comment, hence `tail -n +2`.
