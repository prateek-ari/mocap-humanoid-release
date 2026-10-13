# Xsens MVN (live) → Booster T1 29-DoF mapping

Live IK config for `tools/xsens/live_g1_sim.py --robot booster_t1_29dof`. GMR ships no `xsens_mvn`
config for the T1, so the live tool installs this one into GMR's in-process registry.

It is the reviewed recorded mapping, [`bvh_xsens_to_booster_t1_29dof.json`](bvh_xsens_to_booster_t1_29dof.md),
with the source bodies renamed to the GMR `xsens_mvn` segments produced from MXTP02
(`Pelvis`, `Chest`, `*_UpperLeg`, `*_LowerLeg`, `*_Foot`, `*_UpperArm`, `*_Forearm`, `*_Hand`).
Task weights, the 0.6 lower-body/torso and 0.7 arm scales, and the 1.8 m `human_height_assumption`
are unchanged: MVN segment origins and the 3DS Max BVH joints are the same anatomical joint centres.

All rotation offsets are identity. For the pelvis, legs and torso this follows the recorded
derivation (GMR's G1 Xsens configs use identity offsets for those segments in both conventions).
The arm offsets are fallbacks only: the live tool starts the T1 arms position-only and the
operator's neutral-pose calibration (N key or `--calibrate-after`) derives the arm orientation
offsets from the stream, as `--arm-mode npose` does for recordings.

The live tool also rotates the T1 arm targets 22° outward about the shoulders so hanging hands clear
the T1 hip housings (see `mocap_g1/live_arm_clearance.py`), and screens commands by the MJCF joint
ranges only: the T1 is simulation-only in this project.

Measured on a recorded MVN take replayed as MXTP02 (840 frames): root height 0.685 m at the neutral
pose, knees near neutral, no joint-range violations, no hand/arm overlap with the hips or torso,
elbow-bend error 3.3°, every elbow bending the performer's way. Real MVN stream acceptance remains
outstanding. `tests/test_live_robots.py` locks the naming and identity offsets.
