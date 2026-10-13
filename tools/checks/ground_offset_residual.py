#!/usr/bin/env python3
"""Ground-offset residual check for retarget_configs/ground-offset-calibration.md
(T1, 2026-08-04 recalibration).

Measures the per-clip MEDIAN stance-phase lowest-sole-z on an already-retargeted
patched suite, where "stance" = frames the eval harness's own ANCHORED contact
policy (eval/metrics.py: height < 3cm AND |vz| < 5cm/s) calls a contact frame
for that foot. This reuses metrics.build_robot / run_fk_pass / infer_contact
directly so "stance" here means exactly what score_motion.py's
--contact-policy anchored means -- no separate contact definition to drift
out of sync.

A constant `ground_offset` shifts the whole GMR output vertically by exactly
that amount (verified below and in ground-offset-calibration.md), so the
median stance sole-z on the CURRENT suite is directly usable as the residual
offset error: new_offset = current_offset - residual. This script only
MEASURES the residual; it does not choose an offset by any score/gate metric
(NF/NP/Non-Skating are never read here) -- see the calibration doc's
"not gate-score tuning" note.

Usage:
    VENVPY tools/checks/ground_offset_residual.py \
        --dir WS/out/g1_baseline/patched
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "eval"))
import metrics  # noqa: E402

CLIPS = ["dance1_subject2", "walk3_subject1", "run1_subject5",
         "fight1_subject3", "obstacles1_subject1", "fallAndGetUp3_subject1"]


def clip_pkl_path(dir_: Path, clip: str) -> Path:
    return dir_ / f"{clip}.pkl"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", required=True, help="patched-suite pkl directory")
    ap.add_argument("--fixed-height", type=float, default=0.03,
                    help="anchored-policy contact height threshold (m)")
    ap.add_argument("--anchor-vz-max", type=float, default=0.05,
                    help="anchored-policy |vz| gate (m/s)")
    ap.add_argument("--quat-order", choices=["xyzw", "wxyz"], default=None,
                    help="LEGACY override for pkls WITHOUT a root_quat_order key "
                         "(GMR-native suites are xyzw); default requires the key")
    args = ap.parse_args()

    robot = "unitree_g1"
    rm = metrics.build_robot(robot)
    d = Path(args.dir)

    per_clip_medians = []
    print(f"robot={robot} dir={d} policy=anchored "
          f"(height<{args.fixed_height}m AND |vz|<{args.anchor_vz_max}m/s)")
    print(f"{'clip':<24}{'n_l':>7}{'n_r':>7}{'med_l_m':>10}{'med_r_m':>10}"
          f"{'pooled_med_m':>14}{'pooled_mean_m':>15}")
    for clip in CLIPS:
        p = clip_pkl_path(d, clip)
        motion = metrics.load_gmr_pkl(p, quat_order=args.quat_order)
        fk = metrics.run_fk_pass(rm, motion)
        pooled = []
        side_med, side_n = {}, {}
        for s in sorted(fk["foot_low"]):
            h = fk["foot_low"][s]  # ground_z = 0.0
            mask, _ = metrics.infer_contact(
                h, motion["fps"], policy="anchored",
                fixed_height=args.fixed_height, anchor_vz_max=args.anchor_vz_max)
            samples = h[mask]
            side_med[s] = float(np.median(samples)) if samples.size else float("nan")
            side_n[s] = int(mask.sum())
            pooled.append(samples)
        pooled = np.concatenate(pooled) if pooled else np.array([])
        pooled_med = float(np.median(pooled)) if pooled.size else float("nan")
        pooled_mean = float(np.mean(pooled)) if pooled.size else float("nan")
        per_clip_medians.append(pooled_med)
        print(f"{clip:<24}{side_n['left']:>7}{side_n['right']:>7}"
              f"{side_med['left']:>10.4f}{side_med['right']:>10.4f}"
              f"{pooled_med:>14.4f}{pooled_mean:>15.4f}")

    residual = float(np.median(per_clip_medians))
    print(f"\ncross-clip median of per-clip pooled medians (= offset residual): "
          f"{residual:+.4f} m")
    print(f"per-clip pooled medians (m): {[round(x, 4) for x in per_clip_medians]}")


if __name__ == "__main__":
    main()
