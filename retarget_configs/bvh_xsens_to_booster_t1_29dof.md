# Xsens 3DSM → Booster T1 29-DoF mapping

This is the project configuration for the GMR `load_xsens_file(..., bvh_format="3DSM")` path. It uses the source body names emitted by that loader: `Hips`, `Chest4`, the hip/knee/shoulder/elbow/wrist bodies, and the loader-created `LeftFootMod`/`RightFootMod` ankle aliases.

Target frames, task weights, and position offsets come from upstream GMR's
`bvh_lafan1_to_t1_29dof.json`. The T1 body frames are `Waist`, `Hip_Yaw_*`, `Shank_*`,
`*_foot_link`, `Trunk`, `AL2/4/5`, and `AR2/4/5`.

The quaternion offsets cannot be copied from LAFAN. GMR postmultiplies the source orientation by
the configured offset, and its Xsens body frames differ from its LAFAN body frames. The conversion
uses the working G1 mappings as the shared target convention:

```text
T1_Xsens_offset = G1_Xsens_offset * inverse(G1_LAFAN_offset) * T1_LAFAN_offset
```

This produces identity offsets for the pelvis, hips, knees, feet, and torso. Arm fallbacks are also
identity. Arm orientation is robot-chain-specific, so `--arm-mode npose` replaces those fallbacks
with offsets derived directly from the reviewed neutral-pose recording. Copying the G1 arm result
or composing it through LAFAN produces the wrong T1 elbow branch.

An earlier draft copied the LAFAN quaternions verbatim. On a neutral frame of a recorded take, that draft
tilted the root and folded both legs even though the parsed Xsens lower-body orientations were
identity. The corrected frame-0 solve raises the T1 root from about 0.41 m to 0.69 m and returns
both knees near neutral. `tests/test_booster_t1_xsens_config.py` locks this convention.

The 0.6 lower-body/torso and 0.7 arm scales are also retained from the upstream T1 configuration, with the existing `human_height_assumption` of 1.8 m. `ground_height` remains 0.0 and there is deliberately no per-robot ground offset: tune one against a visually inspected stance only if recorded-data playback shows a sole-height residual.

This configuration assumes the default GMR Xsens parser coordinate conversion (`axis_order="zxy"`, `scale=0.01`) and its post-processing offsets. It is suitable for the recorded-BVH demonstration. Live MVN uses [`xsens_mvn_to_booster_t1_29dof.json`](xsens_mvn_to_booster_t1_29dof.md), derived from this config.

The recorded runner must retarget every source frame before downsampling because GMR carries the
previous IK result into the next solve. The final comparison processes 1,200 source poses at
239.9808 Hz and only then keeps every fourth robot pose. Decimating the Xsens poses first is a
rejected path because it can change IK branches.

The config is an IK mapping, not a collision guarantee. The final demonstration applies
`tools/retarget/project_robot_arm_collisions.py` afterward. That pass can change only the 14 arm
hinges, requires 3 mm clearance from distal arm/hand proxies to torso and leg proxies, and fails if
root/lower-body values change or the tracked-frame drift ceiling is exceeded. See the arm/body
clearance section of `docs/engineer-reference.md`; the result is a simulation preview, not hardware
evidence.
