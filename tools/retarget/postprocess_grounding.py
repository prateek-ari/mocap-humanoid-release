#!/usr/bin/env python3
"""Orientation-aware sole grounding for G1 preview motion.

This is a post-IK visualization pass, not an independent quality metric.
Because the pass explicitly moves inferred stance contacts toward z=0, its
own Non-Penetration result is circular and must not be quoted as evidence of
physical plausibility. The unmodified retarget output remains the scoring
source of truth.

For each frame the pass evaluates the lowest point of the G1 foot collision
geometry, infers stance using height and vertical speed, median-filters the
required root-height correction, and fills flight gaps with a shape-preserving
PCHIP interpolant. Only ``root_pos[:, 2]`` changes. Root rotation, joint
positions, frame rate, and all other pkl keys pass through unchanged.

The calibrated constant offset and its measurement procedure are documented
in ``retarget_configs/ground-offset-calibration.md``. This optional pass is
only for cleaner visual previews where a per-frame sole correction is useful.
"""

from __future__ import annotations

import argparse
import pickle
import sys
from pathlib import Path

import numpy as np
from scipy.interpolate import PchipInterpolator

import mujoco as mj

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "eval"))
sys.path.insert(0, str(REPO_ROOT))
import metrics as eval_metrics  # noqa: E402  (resolve_robot_xml, resolve_foot_bodies)
from mocap_g1.contact import (  # noqa: E402  (the one shared sole/contact implementation)
    central_diff as _shared_central_diff,
    contact_candidates,
    foot_contact_geoms,
    geom_lowest_z,
)
from mocap_g1.gmr_pkl import (  # noqa: E402
    declare_gmr_root_quat_order,
    root_rot_as,
    xyzw_to_wxyz,
)


# ---------------------------------------------------------------------------
# I/O
# ---------------------------------------------------------------------------


def load_motion_pkl(path: str | Path) -> dict:
    with open(path, "rb") as f:
        d = pickle.load(f)
    for key in ("fps", "root_pos", "root_rot", "dof_pos"):
        if key not in d:
            raise ValueError(f"{path}: missing key {key!r}; not a GMR motion pkl")
    return d


def save_motion_pkl(path: str | Path, motion: dict) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "wb") as f:
        pickle.dump(motion, f)


# ---------------------------------------------------------------------------
# FK helpers
# ---------------------------------------------------------------------------


def resolve_foot_geoms(model: mj.MjModel) -> dict[str, list[int]]:
    """{side: [collision geom ids]}, mirroring eval/metrics.py build_robot's
    geom selection (contype/conaffinity != 0; fall back to any geom if a
    body has none, e.g. visual-only test models)."""
    foot_bodies, _notes = eval_metrics.resolve_foot_bodies(model)
    out = {}
    for side, b in foot_bodies.items():
        gids, _visual_fallback = foot_contact_geoms(model, b)
        if not gids:
            raise RuntimeError(f"{side} foot body has no geoms at all; cannot measure height")
        out[side] = gids
    return out


def foot_lowest_z_per_frame(model: mj.MjModel, data: mj.MjData,
                            foot_geoms: dict[str, list[int]],
                            root_pos: np.ndarray, root_rot_wxyz: np.ndarray,
                            dof_pos: np.ndarray) -> dict[str, np.ndarray]:
    """One mj_forward per frame; per side, the lowest point of its collision
    geoms (orientation-aware: a pitched box/capsule corner counts)."""
    T = dof_pos.shape[0]
    mesh_cache: dict[int, np.ndarray] = {}
    out = {s: np.zeros(T) for s in foot_geoms}
    for f in range(T):
        data.qpos[0:3] = root_pos[f]
        data.qpos[3:7] = root_rot_wxyz[f]
        data.qpos[7:] = dof_pos[f]
        data.qvel[:] = 0
        mj.mj_forward(model, data)
        for s, gids in foot_geoms.items():
            out[s][f] = min(geom_lowest_z(model, data, g, mesh_cache, mj) for g in gids)
    return out


def central_diff(x: np.ndarray, fps: float) -> np.ndarray:
    """Central difference along axis 0, one-sided at the ends (the shared
    mocap_g1.contact implementation, also used by eval/metrics.py)."""
    return _shared_central_diff(x, fps)


# ---------------------------------------------------------------------------
# Grounding correction
# ---------------------------------------------------------------------------


def infer_foot_contact(height: np.ndarray, fps: float, percentile: float,
                       band: float, vz_max: float) -> tuple[np.ndarray, float]:
    """Same shape as eval/metrics.py infer_contact: adaptive per-foot
    threshold (percentile + band) AND low vertical speed, via the shared
    ``mocap_g1.contact.contact_candidates`` predicate.

    Unlike the scoring policy, the threshold is intentionally NOT clamped at
    z=0: this pass exists to find the stance frames of an ungrounded or
    penetrating clip and move them TO z=0, so a negative percentile band is
    the signal it corrects rather than a denominator pathology.
    """
    vz = central_diff(height, fps)
    thresh = float(np.percentile(height, percentile)) + band
    mask = contact_candidates(height, vz, height_max=thresh, vz_max=vz_max)
    return mask, thresh


def compute_grounding_shift(foot_low: dict[str, np.ndarray], fps: float,
                            percentile: float, band: float, vz_max: float,
                            smooth_window: int) -> tuple[np.ndarray, dict]:
    """Return (shift (T,), stats). See module docstring steps 2-4."""
    sides = sorted(foot_low)
    T = len(foot_low[sides[0]])
    masks, threshs = {}, {}
    for s in sides:
        masks[s], threshs[s] = infer_foot_contact(foot_low[s], fps, percentile, band, vz_max)

    # Per-frame target: -min height among feet CURRENTLY in contact only.
    target = np.full(T, np.nan)
    any_contact = np.zeros(T, dtype=bool)
    for f in range(T):
        lows = [foot_low[s][f] for s in sides if masks[s][f]]
        if lows:
            target[f] = -min(lows)
            any_contact[f] = True
    contact_idx = np.where(any_contact)[0]

    if len(contact_idx) == 0:
        return np.zeros(T), {
            "contact_frames": 0, "contact_frac": 0.0,
            "contact_thresholds_m": {s: round(float(t), 4) for s, t in threshs.items()},
            "warning": "no contact frames detected by the adaptive threshold; shift left at 0",
        }

    # Low-pass: median over OTHER CONTACT frames within +/- (window//2)
    # frame-indices (flight frames in between are not counted).
    vals = target[contact_idx]
    half = max(0, smooth_window // 2)
    lo_pos = np.searchsorted(contact_idx, contact_idx - half, side="left")
    hi_pos = np.searchsorted(contact_idx, contact_idx + half, side="right")
    smoothed = np.array([np.median(vals[a:b]) for a, b in zip(lo_pos, hi_pos)])

    # Flight frames: C1 (monotone-cubic / PCHIP) interpolation across the
    # gap -- see module docstring step 4 for why this replaced np.interp's
    # piecewise-LINEAR fill. Query points are clamped into [first, last]
    # contact-frame index before evaluation, reproducing np.interp's old
    # "clamp to the boundary value before the first / after the last
    # contact frame" behavior exactly (PCHIP evaluated AT a knot returns
    # that knot's value, so clamping the query is equivalent to clamping
    # the output).
    if len(contact_idx) >= 2:
        pchip = PchipInterpolator(contact_idx, smoothed, extrapolate=False)
        query = np.clip(np.arange(T), contact_idx[0], contact_idx[-1])
        shift = pchip(query)
    else:
        # Exactly one contact knot (the zero-knot case returned above
        # already): no slope to speak of, and PchipInterpolator requires
        # >=2 points. np.interp's behavior for len(xp)==1 is a constant
        # fill; reproduce that directly.
        shift = np.full(T, smoothed[0])

    stats = {
        "contact_frames": int(len(contact_idx)),
        "contact_frac": round(len(contact_idx) / T, 4),
        "contact_thresholds_m": {s: round(float(t), 4) for s, t in threshs.items()},
    }
    return shift, stats


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Orientation-aware post-IK sole grounding for a GMR motion .pkl. "
                     "NON-GRADED OUTPUT: preview/visualization use only -- do not quote "
                     "this tool's own Non-Penetration score as physical-plausibility "
                     "evidence (near-circular; see module docstring header).")
    ap.add_argument("--input", "-i", required=True, help="input GMR .pkl")
    ap.add_argument("--robot", default="unitree_g1",
                    help="GMR robot name (default: unitree_g1)")
    ap.add_argument("--output", "-o", required=True, help="output .pkl path")
    ap.add_argument("--mjcf", default=None, help="override MJCF path (else resolved from --robot)")
    ap.add_argument("--report", action="store_true",
                    help="print per-clip before/after min-sole-z and correction stats")

    ap.add_argument("--contact-percentile", type=float, default=5.0,
                    help="adaptive per-foot contact threshold = percentile(height) + band")
    ap.add_argument("--contact-band", type=float, default=0.03)
    ap.add_argument("--contact-vz-max", type=float, default=0.4,
                    help="max |vertical speed| (m/s) of the foot's lowest point to count as "
                         "contact (looser than eval/metrics.py's 0.2 scoring default -- see "
                         "module docstring)")
    ap.add_argument("--quat-order", default=None, choices=["xyzw", "wxyz"],
                    help="LEGACY override: root_rot order of an input pkl WITHOUT a "
                         "root_quat_order key (default: require the key); the output "
                         "is always written as xyzw with the key stamped")
    ap.add_argument("--smooth-window", type=int, default=7,
                    help="frame-index window for the median low-pass of the per-frame "
                         "grounding correction over contact frames")

    args = ap.parse_args()

    motion = load_motion_pkl(args.input)
    fps = motion["fps"]
    root_pos = np.asarray(motion["root_pos"], dtype=float).copy()
    root_rot_xyzw = root_rot_as(motion, "xyzw", legacy_order=args.quat_order,
                                source=args.input)
    root_rot_wxyz = xyzw_to_wxyz(root_rot_xyzw)
    dof_pos = np.asarray(motion["dof_pos"], dtype=float).copy()
    T = dof_pos.shape[0]
    if root_pos.shape[0] != T or root_rot_xyzw.shape[0] != T:
        raise ValueError(
            f"{args.input}: frame count mismatch root_pos={root_pos.shape[0]} "
            f"root_rot={root_rot_xyzw.shape[0]} dof_pos={T}")

    xml_path = eval_metrics.resolve_robot_xml(args.robot, args.mjcf)
    model = mj.MjModel.from_xml_path(str(xml_path))
    data = mj.MjData(model)
    if model.nq != 7 + dof_pos.shape[1]:
        raise RuntimeError(
            f"{args.input}: dof_pos has {dof_pos.shape[1]} columns but model "
            f"{xml_path.name} expects nq-7={model.nq - 7}")
    foot_geoms = resolve_foot_geoms(model)

    foot_low_before = foot_lowest_z_per_frame(model, data, foot_geoms, root_pos, root_rot_wxyz, dof_pos)
    combined_before = np.minimum(foot_low_before["left"], foot_low_before["right"])

    shift, contact_stats = compute_grounding_shift(
        foot_low_before, fps, args.contact_percentile, args.contact_band,
        args.contact_vz_max, args.smooth_window)

    root_pos[:, 2] += shift

    # Independent second FK pass on the corrected motion (not just shift
    # added analytically) -- see module docstring for why the two must agree.
    foot_low_after = foot_lowest_z_per_frame(model, data, foot_geoms, root_pos, root_rot_wxyz, dof_pos)
    combined_after = np.minimum(foot_low_after["left"], foot_low_after["right"])
    analytic_after = combined_before + shift
    max_fk_shift_disagree = float(np.abs(combined_after - analytic_after).max())
    if max_fk_shift_disagree > 1e-6:
        print(f"WARNING: FK-recomputed vs analytically-shifted sole z disagree by up to "
              f"{max_fk_shift_disagree:.2e} m (expected ~0; rigid root-z translation)",
              file=sys.stderr)

    out = dict(motion)
    out["root_pos"] = root_pos
    out["root_rot"] = root_rot_xyzw
    out["dof_pos"] = dof_pos
    declare_gmr_root_quat_order(out)
    save_motion_pkl(args.output, out)

    if args.report:
        print(f"=== {Path(args.input).name} -> {Path(args.output).name}  robot={args.robot} ===")
        print(f"  frames={T} fps={fps}")
        print(f"  min sole z (m):  before={combined_before.min():+.4f}  after={combined_after.min():+.4f}")
        print(f"  max |correction| (m): {np.abs(shift).max():.4f}   "
              f"mean |correction| (m): {np.abs(shift).mean():.4f}")
        print(f"  contact frames: {contact_stats.get('contact_frames')} "
              f"({100 * contact_stats.get('contact_frac', 0.0):.1f}% of clip)  "
              f"thresholds(m)={contact_stats.get('contact_thresholds_m')}")
        if contact_stats.get("warning"):
            print(f"  WARNING: {contact_stats['warning']}")
        print(f"  saved -> {args.output}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
