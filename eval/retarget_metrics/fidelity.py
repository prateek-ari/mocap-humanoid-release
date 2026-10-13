"""Motion fidelity against GMR's own scaled human reference (needs GMR + BVH).

Split out of ``eval/metrics.py`` (still the public facade).
"""

from __future__ import annotations

import contextlib
import io

import numpy as np

from .robot import RobotModel

# ---------------------------------------------------------------------------
# Motion fidelity (needs source BVH + GMR)
# ---------------------------------------------------------------------------


def build_fidelity_reference(robot: str, bvh_file: str, src_format: str = "lafan1"):
    """Re-run GMR's own loading/scaling for the source BVH and return
    (frames, ret, links) where links maps
    robot_link -> (human_body, pos_w, rot_w); pos_w/rot_w are the MAXIMUM
    GMR IK weights for that link across the used ik_match tables (D3: axes
    with zero weight everywhere were never solved for and must not gate
    fidelity in "weighted" mode).

    Raises KeyError if the robot is not registered in GMR's BVH config dict."""
    from general_motion_retargeting import GeneralMotionRetargeting as GMRClass
    from general_motion_retargeting.utils.lafan1 import load_bvh_file

    frames, human_height = load_bvh_file(bvh_file, format=src_format)
    with contextlib.redirect_stdout(io.StringIO()):
        ret = GMRClass(
            src_human=f"bvh_{src_format}",
            tgt_robot=robot,
            actual_human_height=human_height,
            verbose=False,
        )

    links: dict[str, tuple[str, float, float]] = {}
    for table, used in ((ret.ik_match_table1, ret.use_ik_match_table1),
                        (ret.ik_match_table2, ret.use_ik_match_table2)):
        if not used:
            continue
        for robot_link, entry in table.items():
            human_body, pos_w, rot_w = entry[0], float(entry[1]), float(entry[2])
            if pos_w != 0 or rot_w != 0:
                prev = links.get(robot_link)
                if prev is None:
                    links[robot_link] = (human_body, pos_w, rot_w)
                else:
                    links[robot_link] = (prev[0], max(prev[1], pos_w), max(prev[2], rot_w))
    return frames, ret, links


def motion_fidelity(rm: RobotModel, fk: dict, frames, ret, fidelity_links,
                    pos_thresh: float = 0.10, ori_thresh_deg: float = 10.0,
                    gating: str = "weighted",
                    grounded_reference: bool = False) -> dict:
    """PHUMA Motion Fidelity: % frames where matched links have position
    error < pos_thresh AND orientation error < ori_thresh vs GMR's scaled
    human reference (the exact IK targets GMR itself solved for).

    gating (D3):

    - ``"weighted"`` (default): a link's position error gates a frame only if
      its GMR IK pos weight != 0, and its orientation error only if its rot
      weight != 0 (weights from the ik_config via ``build_fidelity_reference``).
      Axes GMR never solved for cannot fail the frame.
    - ``"strict"``: every matched link gates on both axes regardless of weight
      (pre-D3 behavior, kept for PHUMA comparability).

    grounded_reference (D2): apply GMR's ``offset_to_ground`` grounding to the
    reference, replicating what GMR solved when the motion was produced with
    ``retarget(frame, offset_to_ground=True)`` (the "patched" suites). Without
    this the reference differs from the solved targets by a vertical shift.

    Two position variants are reported:

    - ``motion_fidelity_pct`` (strict, per the PHUMA definition): raw world
      position error per matched link. CAVEAT: GMR's ik_config pairs some
      robot link ORIGINS (hip_yaw, torso, shoulder_yaw) with human joint
      centers that are anatomically offset by a constant ~20 cm, and gives
      those pairs near-zero position weight - so the strict number saturates
      at ~0 for GMR output regardless of motion quality.
    - ``motion_fidelity_debiased_pct``: per-link constant offset (the median
      of the error expressed in the target's local frame) is subtracted
      before thresholding. This removes the skeleton-morphology bias and
      measures actual tracking; it is the number to compare across retargets.
    """
    from scipy.spatial.transform import Rotation as R

    fid_pos, fid_quat = fk["fid_pos"], fk["fid_quat"]
    T = min(fid_pos.shape[0], len(frames))
    L = len(fidelity_links)
    perr = np.zeros((T, L))
    aerr = np.zeros((T, L))
    e_local = np.zeros((T, L, 3))
    for f in range(T):
        # replicates GMR scale/offset pipeline (+ grounding if requested)
        if grounded_reference:
            ret.update_targets(frames[f], offset_to_ground=True)
        else:
            ret.update_targets(frames[f])
        sh = ret.scaled_human_data
        for k, (_, _, human_body, _, _) in enumerate(fidelity_links):
            tgt_pos, tgt_quat = sh[human_body]
            dvec = fid_pos[f, k] - np.asarray(tgt_pos)
            perr[f, k] = np.linalg.norm(dvec)
            e_local[f, k] = R.from_quat(np.asarray(tgt_quat), scalar_first=True).inv().apply(dvec)
            d = abs(float(np.dot(fid_quat[f, k], np.asarray(tgt_quat))))
            aerr[f, k] = np.degrees(2.0 * np.arccos(min(1.0, d)))
    offsets = np.median(e_local, axis=0)  # (L,3) structural offset per link
    resid = np.linalg.norm(e_local - offsets[None], axis=2)  # (T,L)

    pos_w = np.array([lk[3] for lk in fidelity_links])
    rot_w = np.array([lk[4] for lk in fidelity_links])
    if gating == "weighted":
        pos_gate = pos_w > 0
        rot_gate = rot_w > 0
    elif gating == "strict":
        pos_gate = np.ones(L, dtype=bool)
        rot_gate = np.ones(L, dtype=bool)
    else:
        raise ValueError(f"fidelity gating {gating!r} not in (weighted, strict)")

    # .all over zero gated columns is vacuously True (correct: nothing to gate)
    pos_ok = (perr[:, pos_gate] < pos_thresh).all(axis=1)
    ori_ok = (aerr[:, rot_gate] < ori_thresh_deg).all(axis=1)
    pos_ok_db = (resid[:, pos_gate] < pos_thresh).all(axis=1)
    names = [lk[0] for lk in fidelity_links]
    return {
        "motion_fidelity_pct": round(100.0 * float((pos_ok & ori_ok).mean()), 2),
        "motion_fidelity_debiased_pct": round(100.0 * float((pos_ok_db & ori_ok).mean()), 2),
        "position_only_pct": round(100.0 * float(pos_ok.mean()), 2),
        "position_debiased_only_pct": round(100.0 * float(pos_ok_db.mean()), 2),
        "orientation_only_pct": round(100.0 * float(ori_ok.mean()), 2),
        "gating": gating,
        "pos_gated_links": int(pos_gate.sum()),
        "rot_gated_links": int(rot_gate.sum()),
        "total_links": L,
        "reference_grounded": grounded_reference,
        "mean_link_pos_err_m": round(float(perr.mean()), 4),
        "mean_link_pos_err_debiased_m": round(float(resid.mean()), 4),
        "mean_link_ori_err_deg": round(float(aerr.mean()), 2),
        "per_link": {
            nm: {
                "mean_pos_err_m": round(float(perr[:, k].mean()), 4),
                "structural_offset_m": round(float(np.linalg.norm(offsets[k])), 4),
                "mean_pos_err_debiased_m": round(float(resid[:, k].mean()), 4),
                "mean_ori_err_deg": round(float(aerr[:, k].mean()), 2),
                "pos_gated": bool(pos_gate[k]),
                "rot_gated": bool(rot_gate[k]),
            }
            for k, nm in enumerate(names)
        },
        "frames_compared": int(T),
        "thresholds": {"pos_m": pos_thresh, "ori_deg": ori_thresh_deg},
    }


