"""Joint feasibility, torque feasibility, jerk and root-translation metrics.

Split out of ``eval/metrics.py`` (still the public facade).
"""

from __future__ import annotations

import re

import numpy as np

import mujoco as mj

from .contact import _central_diff
from .robot import RobotModel

# ---------------------------------------------------------------------------
# Joint feasibility
# ---------------------------------------------------------------------------


#: Numerical allowance for the HARD position-limit count below: a sample is a
#: hard violation only when it is outside [lo, hi] by more than this.  Twelve
#: orders below any joint range; it only absorbs float round-off of an IK or
#: clamp that lands exactly on a limit.
POSITION_HARD_LIMIT_TOL_RAD = 1e-9


def joint_feasibility(rm: RobotModel, motion: dict, limit_frac: float = 0.98,
                      hard_limit_tol: float = POSITION_HARD_LIMIT_TOL_RAD) -> dict:
    """PHUMA Joint Feasibility plus a HARD mechanical position-limit count.

    ``joint_feasibility_pct`` / ``position_only_pct`` are the PHUMA
    percentages at ``limit_frac`` of each range (unchanged).  The additive
    ``position_hard_violation_frames`` counts frames where ANY limited joint
    lies outside its full [lo, hi] (``limit_frac`` = 1, ``hard_limit_tol``
    allowance); gate v2's ``joint_position_limits`` check requires it to be
    0, so a conditioned / contact-refined output that leaves the mechanical
    range fails closed instead of being reported as a percentage only.
    """
    q = motion["dof_pos"]
    fps = motion["fps"]
    qd = _central_diff(q, fps)
    T = q.shape[0]
    pos_ok = np.ones(T, dtype=bool)
    vel_ok = np.ones(T, dtype=bool)
    hard_ok = np.ones(T, dtype=bool)
    hard_per_joint: dict[str, dict] = {}
    per_joint = {}
    vel_checked = any(ji.vel_limit is not None for ji in rm.joints)
    for ji in rm.joints:
        col = q[:, ji.dof_col]
        if ji.limited:
            mid = 0.5 * (ji.lo + ji.hi)
            half = 0.5 * (ji.hi - ji.lo)
            ok_p = np.abs(col - mid) <= limit_frac * half
            excess = np.maximum(col - ji.hi, ji.lo - col)
            ok_hard = excess <= hard_limit_tol
            if not ok_hard.all():
                worst = int(np.argmax(excess))
                hard_per_joint[ji.name] = {
                    "frames": int((~ok_hard).sum()),
                    "max_excess_rad": round(float(excess[worst]), 6),
                    "worst_frame": worst,
                    "limits_rad": [ji.lo, ji.hi],
                    "limit_source": ji.pos_source,
                }
            hard_ok &= ok_hard
        else:
            ok_p = np.ones(T, dtype=bool)
        if ji.vel_limit is not None:
            ok_v = np.abs(qd[:, ji.dof_col]) <= limit_frac * ji.vel_limit
        else:
            ok_v = np.ones(T, dtype=bool)
        pos_ok &= ok_p
        vel_ok &= ok_v
        viol = int((~ok_p).sum() + (~ok_v).sum())
        if viol:
            per_joint[ji.name] = {
                "pos_violation_frames": int((~ok_p).sum()),
                "vel_violation_frames": int((~ok_v).sum()),
            }
    both = pos_ok & vel_ok
    src_counts: dict[str, int] = {}
    for ji in rm.joints:
        src_counts[ji.pos_source] = src_counts.get(ji.pos_source, 0) + 1
    return {
        "joint_feasibility_pct": round(100.0 * float(both.mean()), 2),
        "position_only_pct": round(100.0 * float(pos_ok.mean()), 2),
        "velocity_only_pct": (round(100.0 * float(vel_ok.mean()), 2) if vel_checked else None),
        "velocity_checked": vel_checked,
        "position_limit_sources": src_counts,  # D1: expect all "urdf" for known robots
        "limit_frac": limit_frac,
        "position_hard_violation_frames": int((~hard_ok).sum()),
        "position_hard_limit_tol_rad": hard_limit_tol,
        "position_hard_violations": dict(sorted(
            hard_per_joint.items(), key=lambda kv: -kv[1]["max_excess_rad"])[:10]),
        "violating_joints": dict(sorted(
            per_joint.items(),
            key=lambda kv: -(kv[1]["pos_violation_frames"] + kv[1]["vel_violation_frames"]),
        )[:10]),
    }


# ---------------------------------------------------------------------------
# Torque / effort feasibility (2026-08-16)
# ---------------------------------------------------------------------------
#
# Joint Feasibility above asks whether the reference is reachable in POSITION
# and VELOCITY.  It says nothing about whether a G1 motor can produce the
# torque the motion demands, and the G1's effort limits are wildly asymmetric:
# 5 N*m on wrist pitch/yaw against 25 N*m on shoulders/elbow and 139 N*m on the
# knee.  A wrist-heavy mocap take is therefore the likeliest first saturation
# on hardware, and nothing in the harness could see it.
#
# Method: rigid-body inverse dynamics (``mj_inverse``) per reference frame.
# qvel comes from ``mj_differentiatePos`` (quaternion-correct for the free
# root); qacc is a central difference of that qvel in the tangent space.
# CONTACT IS DISABLED for the pass, which makes the semantics crisp and the
# result reproducible, at the cost of a hard limitation: with no ground
# reaction force the LEG torques are only meaningful during flight, so the
# gate defaults to the upper body (waist + arms), which is exactly the joint
# set the arm_sdk drives and the set whose limits are tight.  Every joint's
# numbers are still reported.

#: Joints the torque gate grades by default: the arm_sdk-driven set.
UPPER_BODY_PATTERN = r"shoulder|elbow|wrist|waist"
TORQUE_GATE_JOINT_SETS = ("upper_body", "all")


def _finite_difference_state(model: "mj.MjModel", qpos: np.ndarray, fps: float):
    """(qvel, qacc) in tangent space for a [T, nq] reference at ``fps``."""
    T = qpos.shape[0]
    dt = 1.0 / float(fps)
    qvel = np.zeros((T, model.nv))
    for f in range(T):
        lo = max(0, f - 1)
        hi = min(T - 1, f + 1)
        span = (hi - lo) * dt
        if span <= 0.0:
            continue
        v = np.zeros(model.nv)
        mj.mj_differentiatePos(model, v, span, qpos[lo], qpos[hi])
        qvel[f] = v
    qacc = np.zeros((T, model.nv))
    if T >= 3:
        qacc[1:-1] = (qvel[2:] - qvel[:-2]) * (0.5 / dt)
        qacc[0] = (qvel[1] - qvel[0]) / dt
        qacc[-1] = (qvel[-1] - qvel[-2]) / dt
    return qvel, qacc


def torque_feasibility(rm: RobotModel, motion: dict, effort_frac: float = 0.80,
                       gate_joints: str = "upper_body",
                       report_top_n: int = 10) -> dict:
    """Inverse-dynamics torque estimate per frame, checked against URDF effort.

    INFORMATIONAL: no gate consumes this result (decision 2026-10-12);
    ``score_motion_file`` labels the block ``gated: false``.  Here "gated"
    joints only means the joints counted in ``torque_feasibility_pct``.

    ``effort_frac`` is the usable fraction of the vendor effort limit (0.80 =
    keep 20 % in reserve for the balance controller, friction and thermal
    derating that this rigid-body estimate does not model).  A frame counts as
    feasible when EVERY counted joint stays within ``effort_frac * effort``.

    Known limitations, all structural rather than incidental:
      * no ground reaction force (contact disabled) -- leg torques are valid
        only in flight, which is why they are reported but not gated by
        default;
      * rigid-body dynamics only: no rotor inertia, gearbox friction, belt
        compliance or motor speed/torque derating;
      * qacc is a finite difference of a finite difference, so it amplifies
        reference jitter -- read this alongside ``jerk`` /
        ``discontinuity_frames``.
    """
    if gate_joints not in TORQUE_GATE_JOINT_SETS:
        raise ValueError(f"gate_joints must be one of {TORQUE_GATE_JOINT_SETS}")
    if not 0.0 < effort_frac <= 1.0:
        raise ValueError("effort_frac must be in (0, 1]")

    model, data = rm.model, rm.data
    T = motion["dof_pos"].shape[0]
    with_effort = [ji for ji in rm.joints if ji.effort_limit is not None]
    if T < 3:
        return {"status": "not_evaluated", "reason": "need at least 3 frames"}
    if not with_effort:
        return {
            "status": "not_evaluated",
            "reason": "no joint effort limits available (mechanical URDF missing "
                      "<limit effort=...>)",
            "effort_limits_matched": 0,
            "joints_total": len(rm.joints),
        }

    qpos = np.zeros((T, model.nq))
    qpos[:, 0:3] = motion["root_pos"]
    qpos[:, 3:7] = motion["root_rot_wxyz"]
    qpos[:, 7:] = motion["dof_pos"]
    qvel, qacc = _finite_difference_state(model, qpos, motion["fps"])

    tau = np.zeros((T, model.nv))
    saved_disableflags = int(model.opt.disableflags)
    try:
        model.opt.disableflags = saved_disableflags | int(mj.mjtDisableBit.mjDSBL_CONTACT)
        for f in range(T):
            data.qpos[:] = qpos[f]
            data.qvel[:] = qvel[f]
            data.qacc[:] = qacc[f]
            mj.mj_inverse(model, data)
            tau[f] = data.qfrc_inverse
    finally:
        model.opt.disableflags = saved_disableflags
        data.qacc[:] = 0
        data.qvel[:] = 0

    gated = [
        ji for ji in with_effort
        if gate_joints == "all" or re.search(UPPER_BODY_PATTERN, ji.name, re.I)
    ]
    if not gated:
        return {
            "status": "not_evaluated",
            "reason": f"no joints matched gate set {gate_joints!r}",
            "effort_limits_matched": len(with_effort),
        }

    per_joint: dict[str, dict] = {}
    frame_ok = np.ones(T, dtype=bool)
    worst = {"joint": None, "max_utilization": -1.0}
    for ji in with_effort:
        column = np.abs(tau[:, ji.dof_adr])
        util = column / ji.effort_limit
        budget = effort_frac * ji.effort_limit
        over = column > budget
        is_gated = ji in gated
        if is_gated:
            frame_ok &= ~over
        peak = int(np.argmax(util))
        per_joint[ji.name] = {
            "effort_limit_nm": ji.effort_limit,
            "max_abs_torque_nm": round(float(column.max()), 3),
            "p99_abs_torque_nm": round(float(np.percentile(column, 99)), 3),
            "max_utilization": round(float(util[peak]), 4),
            "peak_frame": peak,
            "frames_over_budget": int(over.sum()),
            "gated": is_gated,
        }
        if is_gated and float(util[peak]) > worst["max_utilization"]:
            worst = {
                "joint": ji.name,
                "max_utilization": round(float(util[peak]), 4),
                "max_abs_torque_nm": round(float(column[peak]), 3),
                "effort_limit_nm": ji.effort_limit,
                "budget_nm": round(float(budget), 3),
                "peak_frame": peak,
                "peak_time_s": round(peak / float(motion["fps"]), 3),
            }

    root_force = np.linalg.norm(tau[:, 0:3], axis=1)
    root_torque = np.linalg.norm(tau[:, 3:6], axis=1)
    ranked = sorted(per_joint.items(), key=lambda kv: -kv[1]["max_utilization"])
    violating = {
        name: stats for name, stats in ranked
        if stats["gated"] and stats["frames_over_budget"]
    }
    return {
        "status": "evaluated",
        "torque_feasibility_pct": round(100.0 * float(frame_ok.mean()), 2),
        "effort_frac": effort_frac,
        "gate_joint_set": gate_joints,
        "gated_joints": len(gated),
        "effort_limits_matched": len(with_effort),
        "joints_total": len(rm.joints),
        "worst_joint": worst if worst["joint"] else None,
        "violating_joints": dict(list(violating.items())[:report_top_n]),
        "per_joint_top": dict(ranked[:report_top_n]),
        "root_residual_wrench": {
            "max_force_n": round(float(root_force.max()), 2),
            "max_torque_nm": round(float(root_torque.max()), 2),
            "note": "non-zero because contact is disabled; the free base is unsupported",
        },
        "method": "mj_inverse, contact disabled, qvel via mj_differentiatePos, "
                  "qacc via central difference of qvel",
        "limitations": [
            "no ground reaction force: leg torques are valid only in flight and are "
            "not gated unless gate_joints='all'",
            "rigid-body only: no rotor inertia, gearbox friction or motor derating",
            "qacc is a double finite difference and amplifies reference jitter",
        ],
    }


# ---------------------------------------------------------------------------
# Jerk / discontinuities
# ---------------------------------------------------------------------------


def jerk_metrics(motion: dict, disc_jump_thresh: float = 1.0) -> dict:
    """|third difference| of joint positions (reported in rad/s^3) and a
    discontinuity count: frames where any joint's |second difference| exceeds
    ``disc_jump_thresh`` rad/frame^2 (a one-frame jump detector for
    Fig.3(c)-style waist snaps in Retargeting Matters). Default 1.0 rad/frame^2
    = a one-frame velocity snap of fps rad/s (30 rad/s at 30 fps), far beyond
    actuator capability; GMR IK output has median max-joint |d2q| ~0.2 on
    LAFAN1 dance, so smaller defaults would flag ordinary IK jitter."""
    q = motion["dof_pos"]
    fps = motion["fps"]
    if q.shape[0] < 4:
        return {"warning": "too few frames for jerk"}
    d3 = np.abs(np.diff(q, n=3, axis=0)) * fps ** 3  # rad/s^3
    d2 = np.abs(np.diff(q, n=2, axis=0))  # rad/frame^2
    disc_frames = np.where(d2.max(axis=1) > disc_jump_thresh)[0]
    worst = np.unravel_index(np.argmax(d3), d3.shape)
    return {
        "jerk_max_rad_s3": round(float(d3.max()), 1),
        "jerk_p99_rad_s3": round(float(np.percentile(d3, 99)), 1),
        "discontinuity_frames": int(len(disc_frames)),
        "disc_jump_thresh_rad_frame2": disc_jump_thresh,
        "worst_jerk": {"frame": int(worst[0]), "dof_col": int(worst[1])},
    }


# ---------------------------------------------------------------------------
# Root-translation smoothness (T2, 2026-08-04: gate v2 support)
# ---------------------------------------------------------------------------
#
# ``jerk_metrics`` above reads ``dof_pos`` only -- ``root_pos`` never appears
# in it, so it is structurally blind to anything that edits the free-root
# translation without touching joint angles. That is precisely the shape of
# an earlier grounding-pass defect: its piecewise-linear flight interpolation
# injected base-z acceleration spikes up to 127 m/s^2 (14g) on one clip while
# the harness's joint-only jerk block reported identical numbers. The preview
# pass now uses PCHIP, and this section closes the underlying measurement blind spot
# that blind spot and, via ``static_frame_frac``, gives gate v2 a way to
# detect a degenerate (frozen/motionless) motion that would otherwise satisfy
# Non-Skating vacuously (section 5.4 item 2b: a frozen single-frame pose
# scores 100/100/100).


def root_motion_metrics(motion: dict, count_thresh_mps2: float = 60.0) -> dict:
    """Root-translation linear acceleration and jerk from ``root_pos``,
    finite-differenced at the clip's own fps (no resampling).

    Method matches ``jerk_metrics``'s dof-space computation for consistency
    and, importantly, for reproducing the audited 127 m/s^2 figure above
    exactly: accel is the direct 3-point second derivative
    (``np.diff(root_pos, n=2, axis=0) * fps**2``, m/s^2), jerk the direct
    4-point third derivative (``np.diff(root_pos, n=3, axis=0) * fps**3``,
    m/s^3). This is NOT the same as re-differencing a central-difference
    velocity (``_central_diff`` applied twice): that alternative was tried
    first and under-reports the same audited spike by roughly 2x (39-49
    m/s^2 instead of 78-127) because chaining two averaging operators
    low-pass-filters a knot that is only 1-2 frames wide -- the direct
    ``np.diff`` formula is what was cross-checked against the round-3 V5
    figures and is used here for that reason.

    Units: accel in m/s^2, jerk in m/s^3 (documented in the returned dict's
    ``units`` key too, so a consumer reading only the JSON still has it).
    Reports max and p99 of |value| per world axis (x, y, z) plus the 3D
    vector magnitude, for both accel and jerk -- p99 rather than mean
    because a single interpolation knot (or a block-shuffle splice) is a
    handful of frames out of a multi-thousand-frame clip; a mean would
    dilute it to nothing, exactly the "NP is dilution-limited" pathology
    V5 flagged for the contact metrics (section 5.4 item 2c).

    2026-08-04 (V6 hardening, see ``evaluate_gate_v2`` "Gate v2" section for
    the full derivation): also reports two statistics p99 cannot substitute
    for, ``frames_over_thresh`` (count/frac of frames with accel magnitude
    strictly above ``count_thresh_mps2``, default 60 m/s^2 to match the
    existing p99 bound's anchor) and the per-frame displacement medians used
    by ``evaluate_gate_v2``'s hardened ``not_degenerate`` check
    (``root_disp_median_m``, ``dof_disp_median_rad`` -- see that function's
    docstring for why ``static_frame_frac`` alone is not sufficient).

    Also reports ``static_frame_frac``: the fraction of frames whose
    (root_pos, root_rot, dof_pos) triplet is numerically identical
    (``np.isclose``, atol=1e-12) to the immediately preceding frame. A
    motion frozen at one pose for its whole duration has this at ~1.0 (and
    zero foot velocity every frame, which is exactly why Non-Skating cannot
    be trusted for it -- see ``evaluate_gate_v2``'s ``not_degenerate``
    check below). Genuine mocap-derived retargets measured at T2 time have
    this at 0.0 on every honest clip (IK output has no exactly-repeated
    frames); a real robot briefly holding a pose would show a *small*
    nonzero fraction, not ~1.0. CAVEAT (V6, confirmed defect, fixed here):
    this statistic alone is trivially defeated by adding imperceptible
    (e.g. 1 micrometre std) Gaussian dither to an otherwise-frozen pose --
    ``static_frame_frac`` drops to ~0.0 even though the motion is still
    physically degenerate. ``root_disp_median_m`` / ``dof_disp_median_rad``
    close that hole; ``static_frame_frac`` is retained (not removed) because
    it still gives a precise, unambiguous reason string for the literal
    bit-identical case and costs nothing extra to compute.
    """
    pos = np.asarray(motion["root_pos"], dtype=float)
    fps = float(motion["fps"])
    T = pos.shape[0]

    if "root_rot_wxyz" in motion:
        rot = np.asarray(motion["root_rot_wxyz"], dtype=float)
    else:
        rot = np.asarray(motion.get("root_rot", np.zeros((T, 4))), dtype=float)
    dof = np.asarray(motion["dof_pos"], dtype=float)

    if T >= 2:
        same = (
            np.all(np.isclose(pos[1:], pos[:-1], atol=1e-12), axis=1)
            & np.all(np.isclose(rot[1:], rot[:-1], atol=1e-12), axis=1)
            & np.all(np.isclose(dof[1:], dof[:-1], atol=1e-12), axis=1)
        )
        static_frame_frac = round(float(same.mean()), 4)
        root_disp = np.linalg.norm(np.diff(pos, axis=0), axis=1)  # m/frame
        dof_disp = np.linalg.norm(np.diff(dof, axis=0), axis=1)   # rad/frame (L2 over joints)
        root_disp_median_m = round(float(np.median(root_disp)), 8)
        dof_disp_median_rad = round(float(np.median(dof_disp)), 8)
    else:
        static_frame_frac = None
        root_disp_median_m = None
        dof_disp_median_rad = None

    if T < 4:
        return {
            "warning": "too few frames for root accel/jerk (need >= 4 frames)",
            "static_frame_frac": static_frame_frac,
            "root_disp_median_m": root_disp_median_m,
            "dof_disp_median_rad": dof_disp_median_rad,
            "frames": T,
        }

    d2 = np.diff(pos, n=2, axis=0) * fps ** 2  # m/s^2, len T-2
    d3 = np.diff(pos, n=3, axis=0) * fps ** 3  # m/s^3, len T-3

    def axis_stats(arr: np.ndarray) -> dict:
        mag = np.linalg.norm(arr, axis=1)
        mx: dict[str, float] = {}
        p99: dict[str, float] = {}
        for i, ax in enumerate("xyz"):
            a = np.abs(arr[:, i])
            mx[ax] = round(float(a.max()), 4)
            p99[ax] = round(float(np.percentile(a, 99)), 4)
        mx["mag"] = round(float(mag.max()), 4)
        p99["mag"] = round(float(np.percentile(mag, 99)), 4)
        return {"max": mx, "p99": p99}

    accel_mag = np.linalg.norm(d2, axis=1)
    n_over = int((accel_mag > count_thresh_mps2).sum())
    frames_over_thresh = {
        "threshold_mps2": count_thresh_mps2,
        "count": n_over,
        "frac": round(n_over / len(accel_mag), 6) if len(accel_mag) else None,
    }

    return {
        "units": {"accel": "m/s^2", "jerk": "m/s^3",
                  "axes": "world-frame x,y,z; mag = 3D vector norm"},
        "accel": axis_stats(d2),
        "jerk": axis_stats(d3),
        "frames_over_thresh": frames_over_thresh,
        "static_frame_frac": static_frame_frac,
        "root_disp_median_m": root_disp_median_m,
        "dof_disp_median_rad": dof_disp_median_rad,
        "frames": T,
        "fps": fps,
    }


def root_accel_paired_delta(motion: dict, baseline_motion: dict,
                            delta_max_thresh_mps2: float = 5.0) -> dict:
    """Paired honest-vs-processed root-accel delta (V6 hardening,
    ``evaluate_gate_v2`` "Gate v2" section item (e)/known-limitations).

    ``motion`` and ``baseline_motion`` must be the SAME clip's ``root_pos``
    before/after some root-editing post-process (e.g.
    ``tools/retarget/postprocess_grounding.py``); this function is a
    diagnostic UTILITY, not part of the default (single-pkl) gate v2 grade
    -- gate v2 has no inherent access to "the pre-post-process version" of
    whatever it is scoring, so this is opt-in (see ``evaluate_gate_v2``'s
    ``baseline_root_motion`` parameter) and requires the caller to supply a
    known-honest baseline explicitly.

    Rationale for delta over an absolute population bound: on the available
    honest and preview-grounded G1 clips at V6-hardening time,
    honest-suite root-accel max/count-over-threshold statistics were found
    to fully DOMINATE the grounding-pass artifact's own statistics (honest
    max range 22.18-188.61 m/s^2 vs. the artifact's worst observed max of
    127.15 m/s^2; honest worst frames-over-60 count is 12, tied with the
    grounded suite's own worst count of 12) -- see the "Gate v2" section's
    module comment for the full G1 table. NEITHER an
    absolute max bound NOR an absolute count-above-threshold bound,
    calibrated honestly from that population, can flag a single one of the
    currently-available post-processed clips without also failing
    genuine honest clips (`fight1_subject3` reaches 188 m/s^2 from ordinary
    fast-combat root motion, no
    post-processing involved). A PAIRED delta against the clip's own
    pre-post-process twin sidesteps the cross-clip population overlap
    entirely: it asks "did THIS clip's root-accel get worse", not "is this
    clip's root-accel unusually large in general".

    ``delta_max_thresh_mps2`` default (5.0) is not from the honest
    population either -- it sits in a clean, empirically-observed gap in
    G1 patched-vs-grounded max-accel deltas measured at V6-hardening time.
    It flags ``obstacles1_subject1`` (+48.77) and ``walk3_subject1`` (+8.88);
    the other four clips show a negligible-or-negative delta and are not
    flagged. This
    matches ``postprocess_grounding.py``'s own T4 changelog account of
    "genuine wins" (accel DECREASING) on some clips, so treating every
    grounded clip as equally suspect on this axis would be dishonest, not
    just imprecise.
    """
    pos = np.asarray(motion["root_pos"], dtype=float)
    base_pos = np.asarray(baseline_motion["root_pos"], dtype=float)
    fps = float(motion["fps"])
    base_fps = float(baseline_motion["fps"])
    if pos.shape[0] != base_pos.shape[0]:
        raise ValueError(
            f"paired delta requires matching frame counts: {pos.shape[0]} vs "
            f"{base_pos.shape[0]} (candidate vs baseline) -- not the same clip?")
    if fps != base_fps:
        raise ValueError(f"paired delta requires matching fps: {fps} vs {base_fps}")

    def mag_stats(p):
        d2 = np.diff(p, n=2, axis=0) * fps ** 2
        mag = np.linalg.norm(d2, axis=1)
        return mag

    mag = mag_stats(pos)
    base_mag = mag_stats(base_pos)
    delta_max = round(float(mag.max() - base_mag.max()), 4)
    delta_p99 = round(float(np.percentile(mag, 99) - np.percentile(base_mag, 99)), 4)
    flagged = delta_max > delta_max_thresh_mps2
    return {
        "candidate_max_mag": round(float(mag.max()), 4),
        "baseline_max_mag": round(float(base_mag.max()), 4),
        "delta_max_mag": delta_max,
        "candidate_p99_mag": round(float(np.percentile(mag, 99)), 4),
        "baseline_p99_mag": round(float(np.percentile(base_mag, 99)), 4),
        "delta_p99_mag": delta_p99,
        "delta_max_thresh_mps2": delta_max_thresh_mps2,
        "flagged": flagged,
        "note": ("delta_max_mag > threshold: root-accel got measurably worse "
                 "vs. the supplied baseline" if flagged else
                 "delta within the honest-noise-like band; not flagged on this axis"),
    }


