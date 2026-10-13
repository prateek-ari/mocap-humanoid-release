# G1 `ground_offset` calibration

Date: 2026-08-03; final recalibration 2026-08-04. Applies to GMR with
`gmr-issue-180.patch` followed by `gmr-ground-offset-config.patch`.

**Current G1 value: `ground_offset = 0.0454 m`.**

## Problem

GMR grounds a human ankle-centre target, not the robot's lowest sole point. The upstream issue-#180
path also used a hard-coded value instead of the IK config's `ground_offset`. With zero offset, the
tracked ankle approaches world z=0 while the sole geometry ends several centimetres below the
floor. With an overly large constant, flat stance floats.

The repository patches do two things:

1. route the configured `ground_offset` into the grounding function;
2. keep ungrounded (`offset_to_ground=False`) runs unchanged.

## Geometric starting point

The G1 mocap MJCF foot uses four collision spheres of radius 0.005 m centred at z=-0.030 m in each
ankle-roll body. The flat-sole depth is therefore **0.035 m** below the tracked ankle frame.

Geometry alone under-corrected the solved motion because stance pitch and IK residual also affect
the lowest point. The first six-clip calibration used the suite median and selected 0.051 m. The
final recalibration below measures stance only and supersedes that value.

## Final stance-median method

1. Run the six G1 clips with `offset_to_ground=True` at the current offset.
2. For each foot, compute the lowest collision-geometry point through MuJoCo FK.
3. Select stance samples with the same `anchored` policy used for grading:
   sole z < 3 cm and |vertical speed| < 5 cm/s.
4. Pool left/right stance samples per clip and take the median.
5. Take the median of the six per-clip medians as the residual.
6. Set `new_offset = current_offset - residual`, rerun the suite, and remeasure.

This selects an offset from geometry in metres; it does not search over NF/NP/NS scores or gate
verdicts.

Reproduce the measurement with:

```bash
python tools/checks/ground_offset_residual.py \
  --dir "$MOCAP_WS/out/g1_baseline/patched"
```

## Measurement at 0.051 m

| Clip | stance L | stance R | median L (m) | median R (m) | pooled median (m) |
|---|---:|---:|---:|---:|---:|
| `dance1_subject2` | 1149 | 1325 | +0.0074 | +0.0009 | +0.0042 |
| `walk3_subject1` | 4031 | 3986 | +0.0044 | +0.0094 | +0.0065 |
| `run1_subject5` | 1250 | 1215 | +0.0059 | +0.0038 | +0.0048 |
| `fight1_subject3` | 2419 | 2521 | +0.0041 | +0.0029 | +0.0034 |
| `obstacles1_subject1` | 1996 | 2014 | +0.0053 | +0.0078 | +0.0065 |
| `fallAndGetUp3_subject1` | 1487 | 1514 | +0.0059 | +0.0092 | +0.0075 |

Cross-clip median residual: **+0.0056 m**. New offset: `0.051 - 0.0056 = 0.0454 m`.

## Verification at 0.0454 m

| Clip | stance L | stance R | median L (m) | median R (m) | pooled median (m) |
|---|---:|---:|---:|---:|---:|
| `dance1_subject2` | 1151 | 1335 | +0.0019 | -0.0046 | -0.0013 |
| `walk3_subject1` | 4063 | 4020 | -0.0011 | +0.0038 | +0.0009 |
| `run1_subject5` | 1257 | 1224 | +0.0004 | -0.0017 | -0.0007 |
| `fight1_subject3` | 2437 | 2559 | -0.0013 | -0.0025 | -0.0021 |
| `obstacles1_subject1` | 2028 | 2032 | -0.0002 | +0.0022 | +0.0010 |
| `fallAndGetUp3_subject1` | 1501 | 1528 | +0.0004 | +0.0037 | +0.0020 |

The final cross-clip median is **+0.0001 m**; every clip's pooled median lies within ±2.1 mm.
`walk3_subject1` shifted by a constant -0.0056000 m on every frame (standard deviation of the
per-frame shift `3.2e-10 m`), confirming that the configuration change is a rigid translation.

## Limitation

No constant offset fixes a pitched rigid foot. At heel-strike/toe-off, a sole corner can sit several
centimetres below the ankle-height plane even when the flat-stance plateau is correct. Raising the
constant enough to eliminate those corners would make normal stance visibly float.

The remaining dynamic-clip failures therefore need an orientation/contact-aware IK target or a
tracking controller that reasons about contacts. `tools/retarget/postprocess_grounding.py` remains a
preview tool, not independent grading evidence.

## Files

- `bvh_lafan1_to_g1.json` — carries `ground_offset = 0.0454`.
- `gmr-issue-180.patch` — replaces the hard-coded grounding value.
- `gmr-ground-offset-config.patch` — reads the per-config value.
- `tools/checks/ground_offset_residual.py` — measures the anchored stance residual.
