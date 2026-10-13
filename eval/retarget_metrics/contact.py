"""Foot geometry, the per-frame FK pass, contact inference and foot metrics.

Split out of ``eval/metrics.py`` (still the public facade; its module
docstring documents the contact policies and the round-2 pathology guard).
"""

from __future__ import annotations

import numpy as np

import mujoco as mj

from mocap_g1 import contact as _shared_contact
from mocap_g1.contact import (
    METRIC_ADAPTIVE_BAND_M,
    METRIC_ADAPTIVE_VZ_MAX_M_S,
    METRIC_ANCHOR_VZ_MAX_M_S,
    METRIC_CONTACT_HEIGHT_M,
    contact_candidates,
)

from .robot import RobotModel, verify_dof_joint_order

#: Per-foot contact share below which the denominator is flagged as the
#: round-2 pathology (warning in ``foot_contact_metrics``) and below which
#: gate v2's ``contact_coverage`` precondition fails (``GATE_V2_MIN_FOOT_CONTACT_FRAC``).
SPARSE_FOOT_CONTACT_FRAC = 0.02

# ---------------------------------------------------------------------------
# Geometry: lowest point of a geom (world z)
# ---------------------------------------------------------------------------


def _mesh_verts(model: mj.MjModel, gid: int, cache: dict[int, np.ndarray]) -> np.ndarray:
    """Mesh vertices of geom ``gid`` (cache keyed by mesh id, as in the shared copy)."""
    mesh_id = int(model.geom_dataid[gid])
    if mesh_id not in cache:
        adr, num = model.mesh_vertadr[mesh_id], model.mesh_vertnum[mesh_id]
        cache[mesh_id] = np.array(model.mesh_vert[adr:adr + num], dtype=float)
    return cache[mesh_id]


def geom_lowest_z(model: mj.MjModel, data: mj.MjData, gid: int,
                  mesh_cache: dict[int, np.ndarray]) -> float:
    """World z of the lowest point of geom `gid` at the current data state.

    Thin wrapper over the single shared implementation in
    ``mocap_g1.contact`` (also used by contact refinement and the grounding
    preview pass), so all three measure the same sole.
    """
    return _shared_contact.geom_lowest_z(model, data, gid, mesh_cache, mj)


# ---------------------------------------------------------------------------
# Frame pass (FK + contacts)
# ---------------------------------------------------------------------------


def run_fk_pass(rm: RobotModel, motion: dict,
                fidelity_links: list[tuple[str, int, str, float, float]] | None = None):
    """One mj_forward per frame. Returns dict of per-frame arrays.

    fidelity_links: list of (robot_body_name, body_id, human_body_name,
    pos_weight, rot_weight) whose world poses are recorded for Motion
    Fidelity.
    """
    model, data = rm.model, rm.data
    T = motion["dof_pos"].shape[0]
    nD = motion["dof_pos"].shape[1]
    if model.nq != 7 + nD:
        raise RuntimeError(
            f"dof_pos has {nD} columns but model {rm.xml_path.name} expects nq-7={model.nq - 7}"
        )
    # Column-order check (T3-3): a pkl that declares joint_names must match
    # this MJCF's hinge order exactly; a legacy pkl without them is flagged
    # as unverified by score_motion_file.
    verify_dof_joint_order(rm, motion.get("joint_names"))
    sides = sorted(rm.foot_bodies)  # ["left", "right"]
    foot_pos = {s: np.zeros((T, 3)) for s in sides}
    foot_low = {s: np.zeros(T) for s in sides}
    ncon_self = np.zeros(T, dtype=int)
    self_pairs: dict[tuple[str, str], int] = {}
    fid_pos = fid_quat = None
    if fidelity_links:
        fid_pos = np.zeros((T, len(fidelity_links), 3))
        fid_quat = np.zeros((T, len(fidelity_links), 4))
    mesh_cache: dict[int, np.ndarray] = {}

    for f in range(T):
        data.qpos[0:3] = motion["root_pos"][f]
        data.qpos[3:7] = motion["root_rot_wxyz"][f]
        data.qpos[7:] = motion["dof_pos"][f]
        data.qvel[:] = 0
        mj.mj_forward(model, data)

        for s in sides:
            b = rm.foot_bodies[s]
            foot_pos[s][f] = data.xpos[b]
            foot_low[s][f] = min(
                geom_lowest_z(model, data, g, mesh_cache) for g in rm.foot_geoms[s]
            )
        # self-collision: penetrating contacts between two non-world,
        # non-end-effector robot bodies
        cnt = 0
        for i in range(data.ncon):
            con = data.contact[i]
            if con.dist >= 0:
                continue
            b1 = model.geom_bodyid[con.geom1]
            b2 = model.geom_bodyid[con.geom2]
            if b1 == 0 or b2 == 0:
                continue  # ground / world contacts
            if b1 in rm.ee_bodies or b2 in rm.ee_bodies:
                continue
            cnt += 1
            n1 = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, b1)
            n2 = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, b2)
            key = tuple(sorted((n1, n2)))
            self_pairs[key] = self_pairs.get(key, 0) + 1
        ncon_self[f] = cnt

        if fidelity_links:
            for k, link in enumerate(fidelity_links):
                bid = link[1]
                fid_pos[f, k] = data.xpos[bid]
                fid_quat[f, k] = data.xquat[bid]

    return {
        "foot_pos": foot_pos,
        "foot_low": foot_low,
        "ncon_self": ncon_self,
        "self_pairs": self_pairs,
        "fid_pos": fid_pos,
        "fid_quat": fid_quat,
    }


# ---------------------------------------------------------------------------
# Contact inference + foot metrics
# ---------------------------------------------------------------------------


def _central_diff(x: np.ndarray, fps: float) -> np.ndarray:
    """Central difference along axis 0, one-sided at the ends (shared copy)."""
    return _shared_contact.central_diff(x, fps)


CONTACT_POLICIES = ("adaptive", "fixed", "anchored")


def resolve_contact_policy(contact_policy: str | None, contact_mode: str,
                           grounded_reference: bool) -> str:
    """Resolve the effective policy name from the (contact_policy,
    contact_mode) pair. ``contact_mode`` is the pre-existing keyword
    (score_motion.py's ``--contact-mode`` CLI flag, choices adaptive/fixed
    only today); ``contact_policy`` is the superset-of-three-modes name used
    by direct callers, with ``"auto"`` as a convenience sentinel that picks
    "anchored" for grounded suites and "adaptive" otherwise (see module
    docstring, "Suite-type default"). ``contact_policy`` wins if both are
    given and non-default; falling back to ``contact_mode`` keeps
    score_motion.py's existing calls (which always pass ``contact_mode``
    explicitly) working unchanged."""
    chosen = contact_policy if contact_policy is not None else contact_mode
    if chosen == "auto":
        return "anchored" if grounded_reference else "adaptive"
    if chosen not in CONTACT_POLICIES:
        raise ValueError(
            f"contact policy {chosen!r} not in {CONTACT_POLICIES} (or 'auto')")
    return chosen


def infer_contact(height: np.ndarray, fps: float, policy: str = "adaptive",
                  fixed_height: float = METRIC_CONTACT_HEIGHT_M, percentile: float = 5.0,
                  band: float = METRIC_ADAPTIVE_BAND_M,
                  vz_max: float = METRIC_ADAPTIVE_VZ_MAX_M_S,
                  anchor_vz_max: float = METRIC_ANCHOR_VZ_MAX_M_S):
    """Per-foot contact mask from the robot foot trajectory (see module
    docstring, "Contact inference", for the semantics and validity of each
    policy and the round-2-pathology guard applied here).

    policy:
      - "adaptive": threshold = percentile(height, percentile) + band;
        contact = height < threshold AND |vz| < vz_max.
      - "fixed": threshold = fixed_height (world z=0-relative); contact =
        height < threshold (height-only, no velocity gate).
      - "anchored": threshold = fixed_height; contact = height < threshold
        AND |vz| < anchor_vz_max (a SMALL threshold vs. vz_max -- this is
        what makes it "anchored"/principled rather than a bare height test).

    Returns (mask, threshold_m). The threshold is always clamped to
    ``max(threshold, 0.0)`` -- world z=0 (true ground) is the floor for every
    policy, so a threshold can never go negative and concentrate the contact
    denominator on penetration-only frames (the round-2 pathology).
    """
    vz = _central_diff(height, fps)
    # The predicate itself is the shared mocap_g1.contact.contact_candidates;
    # only the threshold selection is policy-specific.
    if policy == "adaptive":
        thresh = max(float(np.percentile(height, percentile)) + band, 0.0)
        mask = contact_candidates(height, vz, height_max=thresh, vz_max=vz_max)
    elif policy == "fixed":
        thresh = max(fixed_height, 0.0)
        mask = contact_candidates(height, None, height_max=thresh, vz_max=None)
    elif policy == "anchored":
        thresh = max(fixed_height, 0.0)
        mask = contact_candidates(height, vz, height_max=thresh, vz_max=anchor_vz_max)
    else:
        raise ValueError(f"contact policy {policy!r} not in {CONTACT_POLICIES}")
    return mask, thresh


def foot_contact_metrics(fk: dict, fps: float, ground_z: float = 0.0,
                         contact_mode: str = "adaptive", fixed_height: float = 0.03,
                         percentile: float = 5.0, band: float = 0.03,
                         vz_max: float = 0.2, float_thresh: float = 0.01,
                         pen_thresh: float = 0.01, skate_thresh: float = 0.10,
                         contact_policy: str | None = None,
                         anchor_vz_max: float = 0.05,
                         grounded_reference: bool = False) -> dict:
    """Non-Floating / Non-Penetration / Non-Skating.

    A frame enters the denominator if at least one foot is in contact; it
    passes a metric if EVERY foot in contact on that frame satisfies the
    criterion. Pooled per-(foot,frame) sample rates are also reported.

    ``contact_policy`` (superset of ``contact_mode``; see module docstring
    "Contact inference") selects one of "adaptive" / "fixed" / "anchored" /
    "auto"; ``contact_mode`` is the pre-existing keyword kept for
    score_motion.py CLI back-compat and used when ``contact_policy`` is None.
    NOTE (comparability): Non-Floating/Non-Penetration/Non-Skating are only
    comparable across runs that used the SAME resolved policy -- each policy
    selects a different contact-frame denominator (see module docstring).
    """
    policy = resolve_contact_policy(contact_policy, contact_mode, grounded_reference)
    sides = sorted(fk["foot_low"])
    T = len(fk["ncon_self"])
    masks, heights, hspeeds, thresholds = {}, {}, {}, {}
    for s in sides:
        h = fk["foot_low"][s] - ground_z
        heights[s] = h
        # Non-Skating measures the foot BODY ORIGIN's horizontal speed (PHUMA
        # comparability with every historical score).  This intentionally
        # differs from mocap_g1.contact_refine, which measures the contact
        # material point's slip because it must drive an exact rigid repair.
        vel = _central_diff(fk["foot_pos"][s], fps)
        hspeeds[s] = np.linalg.norm(vel[:, :2], axis=1)
        masks[s], thresholds[s] = infer_contact(
            h, fps, policy, fixed_height, percentile, band, vz_max, anchor_vz_max)

    any_contact = np.zeros(T, dtype=bool)
    ok_float = np.ones(T, dtype=bool)
    ok_pen = np.ones(T, dtype=bool)
    ok_skate = np.ones(T, dtype=bool)
    pooled = {"float": [0, 0], "pen": [0, 0], "skate": [0, 0]}  # [pass, total]
    for s in sides:
        m = masks[s]
        any_contact |= m
        f_ok = heights[s] <= float_thresh
        p_ok = heights[s] >= -pen_thresh
        s_ok = hspeeds[s] < skate_thresh
        ok_float &= ~m | f_ok
        ok_pen &= ~m | p_ok
        ok_skate &= ~m | s_ok
        pooled["float"] = [pooled["float"][0] + int((m & f_ok).sum()), pooled["float"][1] + int(m.sum())]
        pooled["pen"] = [pooled["pen"][0] + int((m & p_ok).sum()), pooled["pen"][1] + int(m.sum())]
        pooled["skate"] = [pooled["skate"][0] + int((m & s_ok).sum()), pooled["skate"][1] + int(m.sum())]

    n_contact = int(any_contact.sum())
    def pct(ok):
        return round(100.0 * float(ok[any_contact].mean()), 2) if n_contact else None

    # Per-foot contact-frame counts (round-2 guard): logged unconditionally so
    # a concentrated denominator -- e.g. one foot contributing only a handful
    # of frames, all near a penetration dip -- is visible in the output JSON
    # without re-deriving it from the pkl. See module docstring, "Round-2
    # pathology and its guard".
    contact_frames_by_foot = {s: int(masks[s].sum()) for s in sides}
    contact_frac_by_foot = {
        s: (round(n / T, 4) if T else None) for s, n in contact_frames_by_foot.items()
    }

    out = {
        "contact_mode": contact_mode,  # kept for score_motion.py back-compat
        "contact_policy": policy,  # resolved policy actually used (== contact_mode unless overridden)
        "contact_thresholds_m": {s: round(float(t), 4) for s, t in thresholds.items()},
        "contact_frames": n_contact,
        "contact_frac": round(n_contact / T, 4) if T else None,
        "contact_frames_by_foot": contact_frames_by_foot,
        "contact_frac_by_foot": contact_frac_by_foot,
        "non_floating_pct": pct(ok_float),
        "non_penetration_pct": pct(ok_pen),
        "non_skating_pct": pct(ok_skate),
        "pooled_sample_pct": {
            k: (round(100.0 * a / b, 2) if b else None) for k, (a, b) in pooled.items()
        },
        "foot_height_stats_m": {
            s: {
                "min": round(float(h.min()), 4),
                "p05": round(float(np.percentile(h, 5)), 4),
                "mean_in_contact": (round(float(h[masks[s]].mean()), 4) if masks[s].any() else None),
            }
            for s, h in heights.items()
        },
    }
    if n_contact == 0:
        out["warning"] = "no contact frames inferred; foot metrics undefined"
    elif min(contact_frames_by_foot.values()) < SPARSE_FOOT_CONTACT_FRAC * T:
        # heuristic: a foot with <2% of frames in contact on a multi-second
        # clip is the round-2 pathology's signature (denominator concentrated
        # on a handful of dip/penetration frames rather than genuine stance).
        out["warning"] = (
            "contact_frames_by_foot is highly imbalanced/sparse "
            f"({contact_frames_by_foot}, {T} total frames) -- check for the "
            "round-2 negative-threshold pathology (adaptive policy) or an "
            "ungrounded pkl scored with a fixed/anchored policy"
        )
    return out


