"""``score_motion_file``: one pkl -> the full metric dict.

Split out of ``eval/metrics.py`` (still the public facade).
"""

from __future__ import annotations

from pathlib import Path

import mujoco as mj

from .contact import foot_contact_metrics, run_fk_pass
from .fidelity import build_fidelity_reference, motion_fidelity
from .io import load_gmr_pkl
from .kinematics import (
    jerk_metrics,
    joint_feasibility,
    root_motion_metrics,
    torque_feasibility,
)
from .robot import RobotModel, dof_column_joint_names

# ---------------------------------------------------------------------------
# Top-level scoring
# ---------------------------------------------------------------------------

#: Decision 2026-10-12 (D2): torque_feasibility is INFORMATIONAL.  It is
#: computed and reported, but no gate consumes it -- not gate v2
#: (``evaluate_gate_v2``), not the qualification driver, not any screen -- so
#: a low ``torque_feasibility_pct`` never fails a clip.  The report says so
#: explicitly (block-level ``gated: false`` + a note) so nobody reads the
#: number as a verdict.  Per-joint ``gated`` flags inside the block only
#: select which joints count toward ``torque_feasibility_pct``.
TORQUE_FEASIBILITY_GATED = False
TORQUE_FEASIBILITY_NOTE = (
    "torque_feasibility is informational and not gated: no gate (gate v2, the "
    "qualification driver or any screen) consumes it, so it never fails a clip; "
    "its per-joint 'gated' flags only select the joints counted in "
    "torque_feasibility_pct"
)


def _informational_torque_block(block: dict) -> dict:
    """Label a torque_feasibility result as reported-but-not-gated."""
    labelled = dict(block)
    labelled["gated"] = TORQUE_FEASIBILITY_GATED
    labelled["gate_note"] = TORQUE_FEASIBILITY_NOTE
    return labelled


def score_motion_file(pkl_path: str | Path, rm: RobotModel, *,
                      bvh_path: str | Path | None = None,
                      src_format: str = "lafan1",
                      quat_order: str | None = None,
                      ground_z: float = 0.0,
                      contact_mode: str = "adaptive",
                      contact_policy: str | None = None,
                      contact_height: float = 0.03,
                      contact_percentile: float = 5.0,
                      contact_band: float = 0.03,
                      contact_vz_max: float = 0.2,
                      contact_anchor_vz_max: float = 0.05,
                      float_thresh: float = 0.01,
                      pen_thresh: float = 0.01,
                      skate_thresh: float = 0.10,
                      limit_frac: float = 0.98,
                      pos_fid_thresh: float = 0.10,
                      ori_fid_thresh_deg: float = 10.0,
                      fidelity_gating: str = "weighted",
                      grounded_reference: bool = False,
                      ref_ground_offset: float = 0.0,
                      disc_jump_thresh: float = 1.0,
                      root_accel_count_thresh_mps2: float = 60.0,
                      effort_frac: float = 0.80,
                      torque_gate_joints: str = "upper_body") -> dict:
    motion = load_gmr_pkl(pkl_path, quat_order=quat_order)
    T = motion["dof_pos"].shape[0]
    fps = motion["fps"]

    fidelity_setup = None
    fidelity_error = None
    if bvh_path is not None:
        try:
            frames, ret, link_map = build_fidelity_reference(rm.robot, str(bvh_path), src_format)
            if grounded_reference and ref_ground_offset != 0.0:
                ret.set_ground_offset(ref_ground_offset)  # match the suite's driver
            fidelity_links = []
            for robot_link, (human_body, pos_w, rot_w) in link_map.items():
                bid = mj.mj_name2id(rm.model, mj.mjtObj.mjOBJ_BODY, robot_link)
                if bid < 0:
                    raise KeyError(f"ik_config link {robot_link!r} not in MJCF {rm.xml_path.name}")
                fidelity_links.append((robot_link, bid, human_body, pos_w, rot_w))
            fidelity_setup = (frames, ret, fidelity_links)
        except Exception as e:  # keep the other 4 metrics usable
            fidelity_error = f"{type(e).__name__}: {e}"

    fk = run_fk_pass(rm, motion,
                     fidelity_links=fidelity_setup[2] if fidelity_setup else None)

    result = {
        "file": str(pkl_path),
        "robot": rm.robot,
        "mjcf": str(rm.xml_path),
        "frames": T,
        "fps": fps,
        "duration_s": round(T / fps, 2),
        "joint_feasibility": joint_feasibility(rm, motion, limit_frac),
        "foot_contact": foot_contact_metrics(
            fk, fps, ground_z, contact_mode, contact_height, contact_percentile,
            contact_band, contact_vz_max, float_thresh, pen_thresh, skate_thresh,
            contact_policy=contact_policy, anchor_vz_max=contact_anchor_vz_max,
            grounded_reference=grounded_reference),
        "jerk": jerk_metrics(motion, disc_jump_thresh),
        "root_motion": root_motion_metrics(motion, count_thresh_mps2=root_accel_count_thresh_mps2),
        # Additive (2026-08-16): torque/effort feasibility.  New key only --
        # no existing metric name, gate or field is touched.  Runs AFTER the
        # FK pass because it borrows rm.data and restores it on the way out.
        # Informational only (D2, 2026-10-12): labelled ``gated: false``.
        "torque_feasibility": _informational_torque_block(torque_feasibility(
            rm, motion, effort_frac=effort_frac, gate_joints=torque_gate_joints)),
        "self_collision": {
            "frames_with_self_collision": int((fk["ncon_self"] > 0).sum()),
            "pct_frames": round(100.0 * float((fk["ncon_self"] > 0).mean()), 2),
            "max_contacts_in_frame": int(fk["ncon_self"].max()),
            "top_pairs": {f"{a} | {b}": n for (a, b), n in
                          sorted(fk["self_pairs"].items(), key=lambda kv: -kv[1])[:8]},
        },
        "notes": [*rm.notes, TORQUE_FEASIBILITY_NOTE],
    }
    # run_fk_pass already refused a pkl whose declared joint_names disagree
    # with the MJCF; here we only record which case applied.
    if motion.get("joint_names") is not None:
        result["joint_order"] = {"status": "verified", "joint_names": dof_column_joint_names(rm)}
    else:
        result["joint_order"] = {
            "status": "unverified_legacy_pkl",
            "joint_names": dof_column_joint_names(rm),
        }
        result["notes"].append(
            "WARNING: legacy pkl without joint_names; dof_pos columns ASSUMED to follow "
            f"{rm.xml_path.name}'s hinge order (unverified)")
    if fidelity_setup is not None:
        frames, ret, fidelity_links = fidelity_setup
        result["motion_fidelity"] = motion_fidelity(
            rm, fk, frames, ret, fidelity_links, pos_fid_thresh, ori_fid_thresh_deg,
            gating=fidelity_gating, grounded_reference=grounded_reference)
    else:
        result["motion_fidelity"] = {
            "status": "not_evaluated",
            "reason": fidelity_error or "no source BVH provided (--bvh/--bvh-dir)",
        }
    return result


