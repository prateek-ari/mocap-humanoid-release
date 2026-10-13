"""Gate v2: the anti-gaming PASS/FAIL verdict over a ``score_motion_file`` dict.

Split out of ``eval/metrics.py`` (still the public facade).
"""

from __future__ import annotations

import math

from .contact import SPARSE_FOOT_CONTACT_FRAC

# ---------------------------------------------------------------------------
# Gate v2 (T2, 2026-08-04; hardened 2026-08-04 post-V6)
# ---------------------------------------------------------------------------
#
# Round-3's V5 audit found the round-2 "NF>80 AND NP>80" gate is gameable: a
# frozen single-frame pose scores 100/100/100 and passes outright, and
# block-shuffled garbage dof_pos scores Non-Floating 71.09 -- statistically
# indistinguishable from an honest grounded retarget's 70.71 -- with only
# Non-Skating (not in the round-2 gate at all) telling them apart. Gate v2
# closes both gaps: it adds Non-Skating as a real criterion, adds the
# root-translation smoothness metric above as a fourth criterion so a
# root-only edit (e.g. a future grounding-pass revision) cannot be tuned
# against a blind spot again, and adds anti-gaming PRECONDITIONS that fail
# the gate outright -- before the numeric criteria are even trusted -- when
# the numbers would otherwise be meaningless rather than merely bad:
#
#   (0) contact_policy must be resolved to "anchored". Non-Floating /
#       Non-Penetration / Non-Skating are percentages of a policy-specific
#       contact-frame denominator (module docstring, "Non-Skating ...
#       comparability caveat" above); ns_min=60 was calibrated against
#       anchored-policy honest suites specifically (the current G1 mean is
#       64.61), so grading a
#       "fixed" or "adaptive" run against it
#       would be a silent unit error, not a stricter check.
#   (P) the motion must not be degenerate/static -- see "not_degenerate
#       hardening" below.
#
# ns_min=60 (not 80, matching (a)/(b)): the round-3 anchored-policy honest
# G1 suite mean (64.61, cited above) sits between 60 and 80 -- using an
# 80-floor would fail the honest baseline itself before any
# gaming defense is even relevant, so 60 is the highest round number that
# is still an absolute floor (V5's gaming garbage scored Non-Skating 4.04,
# 15x below it) without also failing today's honest suites. This is
# documented here as the threshold's justification and should be revisited
# once more than 6 honest clips have been scored.
#
# root_accel_p99_max=60.0 m/s^2 (T2 derivation): root_motion_metrics'
# accel-magnitude p99 was computed across the 12 honest G1 clips available
# in the current corpus (6 unpatched + 6 config-grounded "patched" clips,
# with no post-processing). This statistic is root_pos-only and does not
# depend on contact policy. Observed range: 10.60-42.11 m/s^2 (worst: patched
# run1_subject5, a fast locomotion clip with genuine hard footfalls). The
# bound is set at 60 m/s^2 (~1.4x the single worst honest p99) -- comfortably
# above every honest clip with margin,
# while still well below what a block-shuffled/frozen-pose gaming attempt
# produces once root_pos itself is corrupted (see
# tools/checks/gate_v2_selftest.py and the T2 changelog entry below for the
# measured numbers).
#
# --- not_degenerate hardening (V6 CONFIRMED-DEFECT-HIGH #2, fixed here) ---
#
# V6 showed the original precondition (``static_frame_frac < 0.99``, where
# ``static_frame_frac`` is the fraction of frames EXACTLY bit-identical to
# the previous frame, ``np.isclose(atol=1e-12)``) is defeated by 1
# micrometre: a frozen pose plus N(0, 1e-6) Gaussian dither on
# root_pos/dof_pos drops static_frame_frac to ~0.0 (no two frames are
# bit-identical anymore) and passes the original gate, even though the
# motion is still physically frozen. Fix: ``not_degenerate`` now ALSO
# requires genuine motion-MAGNITUDE, not just non-repetition:
#
#   root_disp_median_m   >= GATE_V2_ROOT_DISP_MEDIAN_MIN_M   (0.001 m)
#   dof_disp_median_rad  >= GATE_V2_DOF_DISP_MEDIAN_MIN_RAD  (0.02 rad)
#
# (median per-frame L2 displacement of root_pos / dof_pos respectively, from
# ``root_motion_metrics``; both required -- an attacker who dithers only one
# channel and leaves the other frozen must still fail). Thresholds are
# conservative relative to the 12 honest G1 clips (6 unpatched + 6 patched),
# all of which clear them. Recalibrate these values when the G1 motion corpus
# grows. Against
# the adversarial case: dither std sigma produces an expected per-frame L2
# displacement of roughly sigma*2.26 (root, 3 dims) / sigma*7.1 (dof, G1's
# 29 dims) by chi-distribution scaling, so root_disp_median's own breakeven
# is around sigma~4.4e-4 -- but dof_disp_median (needing the larger sigma of
# the two, since BOTH must clear) is the actual binding constraint at
# sigma~0.02/7.1~2.8e-3; the 1um (sigma=1e-6) case from V6's report is
# ~2,800x below that. ``tools/checks/gate_v2_selftest.py`` bisects for the
# EXACT empirical crossover (not just this estimate) and confirmed it at
# sigma~2.65e-3, matching the hand estimate to within 6%. IMPORTANT: even
# past that crossover (``not_degenerate`` itself starts passing), the
# OVERALL gate stays correctly rejecting up to at least sigma=1e-2 (1 cm)
# in the self-test's own run -- once dither is large enough to defeat
# ``not_degenerate``, the same independent per-frame noise reads as
# high-frequency jitter and instead fails Non-Skating / Non-Floating /
# root-accel-p99 (a different, still-correct layer of defense; see that
# script's own sigma=1e-2 output). ``static_frame_frac >= static_frac_max``
# is KEPT as an additional (not replaced) precondition: it is a free, exact
# check for the literal bit-identical case and gives a more specific reason
# string when that's what actually happened.
#
# --- root-accel hardening (V6 CONFIRMED-DEFECT-HIGH #3) ---
#
# V6 showed ``root_accel_p99_mag<=60`` cannot detect the grounding-pass
# artifact it was added for: all six post-processed G1 clips pass it, and the
# observed p99 values overlap the honest range, while the audited clip's MAX accel goes
# 78.38->127.15 m/s^2 (frames over 60 m/s^2: 3->9 of 5388, 0.167%) --
# invisible to any percentile below p99.83.
#
# Two population-level criteria were added per the task brief
# ("add a max-magnitude bound and/or count-of-frames-above-threshold"):
#
#   root_accel_max_mag         <= GATE_V2_ROOT_ACCEL_MAX_MPS2        (235 m/s^2)
#   root_accel_frames_over_thresh <= GATE_V2_ROOT_ACCEL_MAX_FRAMES_OVER (15 frames > 60 m/s^2)
#
# HONEST CAVEAT, re-derived directly from data (this correction supersedes
# an earlier "~78" estimate of the honest max ceiling that undercounted the
# honest population): re-measuring root-accel MAX and frames-over-60 across
# the 12 honest G1 clips used for the p99 bound gives a maximum of
# 188.61 m/s^2 (`fight1_subject3`, 188.38 unpatched / 188.61 patched) -- a
# fast-combat clip with a genuine, reproducible hard root-accel transient,
# nothing to do with any post-process. The
# grounding-pass artifact (max 127.15, frames-over-60 up to 9) is smaller on
# BOTH statistics than these honest outliers. CONSEQUENCE: calibrated
# honestly (with margin over the true honest ceiling, not the incorrect
# "~78" estimate), NEITHER of these two population-level criteria flips ANY
# of the six currently-available grounded clips to FAIL -- they pass on both
# the 12 honest clips and the six grounded clips (0/6 newly flagged). This
# is not a calibration slip to be tightened away: `fight1_subject3` is
# demonstrably honest (present, near-identically, in both the unpatched
# source and the patched/offset_to_ground variants, neither of which
# touches root_pos post-hoc), so any bound tight enough to catch the
# 127.15 m/s^2 artifact would also fail that genuine clip. Kept anyway as a
# real (if currently inactive on this data) ceiling against a WORSE future
# artifact, and because the task brief specifically asked for this
# population-level form to be added and validated -- the validation result,
# reported honestly, is that population-level thresholds cannot do this job
# on the data available. See ``root_accel_paired_delta`` (function above)
# for the mechanism that catches a real subset (2/6) of the G1 grounded
# clips, opt-in via ``baseline_root_motion`` below. The
# grounding pass remains NON-GRADED / preview-only -- no criterion here
# certifies it as physically plausible, only as internally self-consistent.

GATE_V2_NF_MIN = 80.0
GATE_V2_NP_MIN = 80.0
GATE_V2_NS_MIN = 60.0
GATE_V2_ROOT_ACCEL_P99_MAX_MPS2 = 60.0
GATE_V2_STATIC_FRAC_MAX = 0.99
GATE_V2_ROOT_DISP_MEDIAN_MIN_M = 0.001       # 1 mm/frame median root translation
GATE_V2_DOF_DISP_MEDIAN_MIN_RAD = 0.02       # ~1.1deg L2/frame median joint motion
GATE_V2_ROOT_ACCEL_MAX_MPS2 = 235.0          # ~1.25x the honest G1 max (188.61)
GATE_V2_ROOT_ACCEL_COUNT_THRESH_MPS2 = 60.0  # per-frame threshold for the count stat
GATE_V2_ROOT_ACCEL_MAX_FRAMES_OVER = 15      # conservative G1 corpus ceiling
GATE_V2_PAIRED_DELTA_MAX_MPS2 = 5.0          # see root_accel_paired_delta
#
# --- contact coverage precondition (2026-10-10 review fix) ---
#
# NF/NP/NS are percentages of the frames in which at least one foot is in
# contact (``foot_contact_metrics``' ``pct()``), so a clip with ONE inferred
# contact frame scores 100.0 on all three from a single sample.  The gate
# therefore refuses to trust them unless the denominator is a real share of
# the clip:
#
#   contact_frac                >= GATE_V2_MIN_CONTACT_FRAC       (5 % of frames)
#   min(contact_frac_by_foot)   >= GATE_V2_MIN_FOOT_CONTACT_FRAC  (2 % per foot)
#
# The per-foot floor is the existing ``foot_contact_metrics`` round-2
# pathology heuristic (``SPARSE_FOOT_CONTACT_FRAC``), promoted from a warning
# to a precondition so one foot's handful of dip frames cannot carry NP.
# Both are deliberately loose: honest anchored-policy locomotion has each foot
# planted for a large fraction of the gait cycle (a walking stance phase is
# ~60 % per foot before the 5 cm/s anchor gate trims transients), so they only
# reject near-airborne / ungrounded / degenerate denominators.  They were NOT
# re-derived from the LAFAN1 corpus (not available to this change); recalibrate
# with the other floors when the honest corpus is rescored.  Known limitation:
# a long genuine single-leg stand would fail the per-foot floor.
GATE_V2_MIN_CONTACT_FRAC = 0.05
GATE_V2_MIN_FOOT_CONTACT_FRAC = SPARSE_FOOT_CONTACT_FRAC
#
# --- joint position limits (2026-10-10 review fix) ---
#
# Conditioning (FIR + time scaling) and contact refinement (ankle repairs)
# rewrite joint angles after the raw-input window audit has run, and PHUMA's
# Joint Feasibility is only a percentage.  ``joint_position_limits`` grades
# the scored trajectory itself: it FAILS if any frame has any joint outside
# its mechanical [lo, hi] (``joint_feasibility``'s
# ``position_hard_violation_frames``; tolerance POSITION_HARD_LIMIT_TOL_RAD).
# Missing data fails closed.
GATE_V2_MAX_POSITION_LIMIT_VIOLATION_FRAMES = 0


def _finite(value: object) -> float | None:
    """``value`` as a float if it is a finite real number, else None.

    Every comparison below goes through this so a NaN/inf statistic can never
    satisfy a precondition by comparing False (``nan < floor`` is False, which
    previously let a NaN displacement median pass ``not_degenerate``).
    """
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def evaluate_gate_v2(result: dict, *,
                     nf_min: float = GATE_V2_NF_MIN,
                     np_min: float = GATE_V2_NP_MIN,
                     ns_min: float = GATE_V2_NS_MIN,
                     root_accel_p99_max: float = GATE_V2_ROOT_ACCEL_P99_MAX_MPS2,
                     static_frac_max: float = GATE_V2_STATIC_FRAC_MAX,
                     root_disp_min_m: float = GATE_V2_ROOT_DISP_MEDIAN_MIN_M,
                     dof_disp_min_rad: float = GATE_V2_DOF_DISP_MEDIAN_MIN_RAD,
                     root_accel_max_mps2: float = GATE_V2_ROOT_ACCEL_MAX_MPS2,
                     max_frames_over_thresh: int = GATE_V2_ROOT_ACCEL_MAX_FRAMES_OVER,
                     baseline_root_motion: dict | None = None,
                     paired_delta_max_mps2: float = GATE_V2_PAIRED_DELTA_MAX_MPS2,
                     min_contact_frac: float = GATE_V2_MIN_CONTACT_FRAC,
                     min_foot_contact_frac: float = GATE_V2_MIN_FOOT_CONTACT_FRAC,
                     max_position_limit_violation_frames: int = (
                         GATE_V2_MAX_POSITION_LIMIT_VIOLATION_FRAMES),
                     ) -> dict:
    """Gate v2: score PASS requires ALL of

      (a) Non-Floating    > nf_min    (default 80)
      (b) Non-Penetration > np_min    (default 80)
      (c) Non-Skating     > ns_min    (default 60) -- ONLY valid under
          contact_policy == "anchored" (precondition 0 below)
      (d) root-accel-magnitude p99   <= root_accel_p99_max      (default 60 m/s^2)
      (e) root-accel-magnitude max   <= root_accel_max_mps2     (default 235 m/s^2)
      (f) frames w/ accel > 60 m/s^2 <= max_frames_over_thresh  (default 15 frames)
      (g) ``joint_position_limits``: frames with any joint outside its
          mechanical [lo, hi] <= max_position_limit_violation_frames
          (default 0; from ``joint_feasibility``'s
          ``position_hard_violation_frames``, missing -> FAIL)

    plus preconditions that fail the gate OUTRIGHT (independent of, and
    checked before trusting, (a)-(f) -- see the module-level comment above
    this function for the full rationale, honest-suite derivation, and
    known limitations of every threshold):

      (0) ``result["foot_contact"]["contact_policy"] == "anchored"``.
      (C) ``contact_coverage``: ``foot_contact.contact_frac >=
          min_contact_frac`` (default 0.05) AND every foot's
          ``contact_frac_by_foot >= min_foot_contact_frac`` (default 0.02) --
          NF/NP/NS are percentages of contact frames, so a near-empty
          denominator (1 contact frame -> 100 %) must not be trusted.
      (P) ``not_degenerate``: the motion must show genuine dynamics, not
          just non-repetition -- FAILS if ANY of:
            - ``static_frame_frac >= static_frac_max`` (literal bit-identical
              frames, ``np.isclose`` exact-equality -- the original round-3
              V5 case);
            - ``root_disp_median_m < root_disp_min_m`` (median per-frame
              root_pos displacement too small -- catches dithered-frozen
              gaming, V6 CONFIRMED-DEFECT-HIGH #2);
            - ``dof_disp_median_rad < dof_disp_min_rad`` (same, for dof_pos);
            - any of the three statistics is missing or non-finite (fewer
              than 2 frames, or a NaN that would otherwise compare False and
              slip through -- both previously PASSED the precondition).

    ``baseline_root_motion`` (optional, default None): a ``root_motion_
    metrics()``-shaped dict for a KNOWN-HONEST counterpart of the SAME clip
    (e.g. the pre-post-process "patched" pkl, for a "grounded" candidate).
    When supplied, adds a seventh, OPT-IN check, ``root_accel_paired_delta``
    (see that function's docstring for why this exists and what it catches
    that (e)/(f) cannot): FAILS if the candidate's root-accel max exceeds
    the baseline's by more than ``paired_delta_max_mps2`` (default 5 m/s^2),
    and also FAILS (closed) when the delta cannot be computed because either
    side lacks a finite root-accel max -- a supplied-but-unusable baseline is
    not evidence that nothing got worse.
    Omitted from ``checks``/the pass computation entirely when no baseline
    is given -- default (single-pkl) grading is unchanged.

    Takes the dict returned by ``score_motion_file`` (must have been run
    with ``root_motion_metrics`` available, i.e. any T2-or-later harness;
    always true for a fresh ``score_motion_file`` call). Every criterion
    (preconditions included) is graded independently and returned in
    ``checks`` with its ``pass``/``value``/``requirement``; top-level
    ``pass`` is the AND of all active checks. ``reasons`` lists a
    human-readable sentence for every failing check, preconditions first
    (score_motion.py's ``--gate v2`` prints both ``checks`` and
    ``reasons``).

    KNOWN LIMITATIONS (documented, not silently patched around -- see the
    module comment above for the full evidence):

      - Block-shuffle anti-gaming rejection via Non-Skating SPECIFICALLY is
        only robust for block size <= 5 frames (V6, confirmed). At block
        size >= 10, Non-Skating does not reject every current G1 clip
        (seed=42, measured via ``tools/checks/gate_v2_selftest.py``). Rejection
        instead comes from root_accel_p99_mag (block boundaries are frequent
        enough at block=10 that inter-block position jumps push p99 into
        the thousands of m/s^2) and/or non_penetration. The overall gate
        v2 verdict still correctly fails the current block=10 tests, but the
        mechanism is weaker/less universal than block<=5's: root_accel_p99
        and non_penetration are not anti-shuffle-specific criteria (an
        honest clip that is already marginal on one of them, e.g.
        `fight1_subject3`'s honest non_penetration shortfall at round 4,
        could in principle happen to "reject" a shuffle for reasons
        unrelated to the shuffle itself, or a differently-constructed
        shuffle at large block size could avoid tripping any of them). Not
        fixed here per the task's scope note (documented, not chased).
      - The two population-level root-accel criteria added in (e)/(f) above
        are, on the data available today, INACTIVE against the one known
        real-world artifact (the grounding post-process): the honest suite
        itself contains larger max/count spikes (`fight1_subject3`) than
        the artifact produces, so no honestly-calibrated absolute bound can
        separate them (see the module comment's "root-accel hardening"
        section for the numbers). Use ``root_accel_paired_delta`` with an
        explicit baseline for a per-clip mechanism that does catch a real
        subset (2/6 on current G1 data).
      - ``not_degenerate``'s displacement-median thresholds, like every
        threshold in this module, must be revisited as the honest G1 corpus
        grows and introduces slower genuine motion.
    """
    fc = result.get("foot_contact") or {}
    rmo = result.get("root_motion") or {}

    policy = fc.get("contact_policy")
    policy_ok = policy == "anchored"

    static_frac = rmo.get("static_frame_frac")
    root_disp = rmo.get("root_disp_median_m")
    dof_disp = rmo.get("dof_disp_median_rad")

    def _gt(v, b):
        v = _finite(v)
        return v is not None and v > b

    def _le(v, b):
        v = _finite(v)
        return v is not None and v <= b

    def _lt(v, b):
        v = _finite(v)
        return v is not None and v < b

    def _ge(v, b):
        v = _finite(v)
        return v is not None and v >= b

    missing_stats = any(_finite(v) is None for v in (static_frac, root_disp, dof_disp))
    frozen = _ge(static_frac, static_frac_max)
    root_disp_low = _lt(root_disp, root_disp_min_m)
    dof_disp_low = _lt(dof_disp, dof_disp_min_rad)
    degenerate = missing_stats or frozen or root_disp_low or dof_disp_low

    contact_frac = fc.get("contact_frac")
    by_foot = fc.get("contact_frac_by_foot")
    foot_fracs = (
        {str(side): value for side, value in by_foot.items()}
        if isinstance(by_foot, dict) and by_foot else None
    )
    coverage_ok = (
        _ge(contact_frac, min_contact_frac)
        and foot_fracs is not None
        and all(_ge(value, min_foot_contact_frac) for value in foot_fracs.values())
    )

    jf = result.get("joint_feasibility") or {}
    limit_frames = jf.get("position_hard_violation_frames")
    limits_ok = (
        isinstance(limit_frames, int) and not isinstance(limit_frames, bool)
        and limit_frames <= max_position_limit_violation_frames
    )

    nf = fc.get("non_floating_pct")
    npn = fc.get("non_penetration_pct")
    ns = fc.get("non_skating_pct")
    accel_p99 = ((rmo.get("accel") or {}).get("p99") or {}).get("mag")
    accel_max = ((rmo.get("accel") or {}).get("max") or {}).get("mag")
    frames_over = (rmo.get("frames_over_thresh") or {}).get("count")
    frames_over_at = (rmo.get("frames_over_thresh") or {}).get(
        "threshold_mps2", GATE_V2_ROOT_ACCEL_COUNT_THRESH_MPS2)

    checks = {
        "contact_policy_anchored": {
            "pass": policy_ok, "value": policy, "requirement": "== 'anchored'"},
        "contact_coverage": {
            "pass": bool(coverage_ok),
            "value": {"contact_frac": contact_frac, "contact_frac_by_foot": foot_fracs},
            "requirement": (f"contact_frac >= {min_contact_frac} AND every foot's "
                            f"contact_frac_by_foot >= {min_foot_contact_frac}")},
        "not_degenerate": {
            "pass": not degenerate,
            "value": {"static_frame_frac": static_frac, "root_disp_median_m": root_disp,
                      "dof_disp_median_rad": dof_disp},
            "requirement": (f"static_frame_frac < {static_frac_max} AND "
                            f"root_disp_median_m >= {root_disp_min_m} AND "
                            f"dof_disp_median_rad >= {dof_disp_min_rad}")},
        "non_floating": {
            "pass": _gt(nf, nf_min), "value": nf, "requirement": f"> {nf_min}"},
        "non_penetration": {
            "pass": _gt(npn, np_min), "value": npn, "requirement": f"> {np_min}"},
        "non_skating": {
            "pass": _gt(ns, ns_min), "value": ns, "requirement": f"> {ns_min}"},
        "root_accel_p99_mag": {
            "pass": _le(accel_p99, root_accel_p99_max), "value": accel_p99,
            "requirement": f"<= {root_accel_p99_max} m/s^2"},
        "root_accel_max_mag": {
            "pass": _le(accel_max, root_accel_max_mps2), "value": accel_max,
            "requirement": f"<= {root_accel_max_mps2} m/s^2"},
        "root_accel_frames_over_thresh": {
            "pass": _le(frames_over, max_frames_over_thresh),
            "value": frames_over,
            "requirement": f"<= {max_frames_over_thresh} frames with accel > {frames_over_at} m/s^2"},
        "joint_position_limits": {
            "pass": bool(limits_ok), "value": limit_frames,
            "requirement": (f"<= {max_position_limit_violation_frames} frames with any joint "
                            f"outside its mechanical position range")},
    }

    if baseline_root_motion is not None:
        cand_max = _finite(accel_max)
        base_max = _finite(
            ((baseline_root_motion.get("accel") or {}).get("max") or {}).get("mag"))
        delta_max = (round(cand_max - base_max, 4)
                     if cand_max is not None and base_max is not None else None)
        # Fail closed: a baseline was explicitly supplied, so "no delta could
        # be computed" (e.g. a too-short baseline with no accel block) must
        # not read as "did not get worse".
        checks["root_accel_paired_delta"] = {
            "pass": delta_max is not None and delta_max <= paired_delta_max_mps2,
            "value": delta_max,
            "requirement": f"<= +{paired_delta_max_mps2} m/s^2 vs supplied baseline"}

    reasons = []
    if not policy_ok:
        reasons.append(
            f"contact_policy={policy!r} (gate v2 requires 'anchored' -- Non-Floating/"
            f"Non-Penetration/Non-Skating percentages are only comparable within one "
            f"contact policy, see module docstring)")
    if not coverage_ok:
        reasons.append(
            f"insufficient contact coverage: contact_frac={contact_frac}, "
            f"contact_frac_by_foot={foot_fracs} (requires >= {min_contact_frac} overall "
            f"and >= {min_foot_contact_frac} per foot) -- NF/NP/NS are percentages of "
            f"contact frames and are not evidence on a near-empty denominator")
    if degenerate:
        if missing_stats:
            reasons.append(
                "no contact dynamics: static_frame_frac/root/dof displacement medians "
                "are missing or non-finite (fewer than 2 frames, or NaN input)")
        if frozen:
            pct = f"{static_frac:.1%}" if static_frac is not None else "n/a"
            reasons.append(
                f"no contact dynamics: {pct} of frames are static (root_pos+root_rot+"
                f"dof_pos numerically unchanged from the previous frame) -- Non-Skating "
                f"is vacuously satisfied by a motionless pose and NF/NP/NS are not "
                f"evidence of usable motion here")
        if root_disp_low:
            reasons.append(
                f"no contact dynamics: median per-frame root displacement "
                f"{root_disp} m < {root_disp_min_m} m -- motion is root-frozen (or "
                f"defeated only by imperceptible dither), Non-Skating cannot be trusted")
        if dof_disp_low:
            reasons.append(
                f"no contact dynamics: median per-frame dof displacement "
                f"{dof_disp} rad < {dof_disp_min_rad} rad -- motion is joint-frozen "
                f"(or defeated only by imperceptible dither), Non-Skating cannot be trusted")
    for name in ("non_floating", "non_penetration", "non_skating", "root_accel_p99_mag",
                 "root_accel_max_mag", "root_accel_frames_over_thresh",
                 "joint_position_limits", "root_accel_paired_delta"):
        c = checks.get(name)
        if c is not None and not c["pass"]:
            reasons.append(f"{name} failed: value={c['value']} (requires {c['requirement']})")

    return {
        "gate": "v2",
        "pass": all(c["pass"] for c in checks.values()),
        "checks": checks,
        "reasons": reasons,
        "thresholds": {
            "nf_min": nf_min, "np_min": np_min, "ns_min": ns_min,
            "root_accel_p99_max_mps2": root_accel_p99_max,
            "static_frac_max": static_frac_max,
            "root_disp_min_m": root_disp_min_m,
            "dof_disp_min_rad": dof_disp_min_rad,
            "root_accel_max_mps2": root_accel_max_mps2,
            "max_frames_over_thresh": max_frames_over_thresh,
            "paired_delta_max_mps2": (paired_delta_max_mps2
                                      if baseline_root_motion is not None else None),
            "min_contact_frac": min_contact_frac,
            "min_foot_contact_frac": min_foot_contact_frac,
            "max_position_limit_violation_frames": max_position_limit_violation_frames,
        },
    }
