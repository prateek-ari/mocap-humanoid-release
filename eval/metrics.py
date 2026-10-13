"""Retarget-quality metrics for GMR-style robot motions.

Implements the PHUMA metric suite (arXiv:2510.26236; thresholds follow that
paper) plus jerk /
discontinuity and self-collision diagnostics. Every PHUMA metric is the
percentage of frames satisfying a physical criterion:

- Joint Feasibility: joint positions AND velocities within ``limit_frac``
  (default 0.98) of mechanical limits. Position limits come from a MECHANICAL
  URDF (``MECHANICAL_URDF`` / ``--pos-urdf``), NOT the MJCF ``jnt_range``:
  GMR's IK MJCFs narrow ranges for retargeting (g1_mocap_29dof.xml narrows
  15/29 G1 joints) and must not define feasibility; joints absent from the
  URDF fall back to MJCF. MuJoCo MJCF has no joint-velocity-limit field, so
  velocity limits are read from a companion URDF (``<limit velocity=...>``)
  matched by joint name; if none is available the velocity half of the check
  is skipped and flagged in the output. Both source files are logged in
  ``notes``.
- Non-Floating: during contact, foot lowest point within ``float_thresh``
  (1 cm) ABOVE the ground plane (i.e. height <= +1 cm).
- Non-Penetration: during contact, foot lowest point within ``pen_thresh``
  (1 cm) BELOW the ground plane (i.e. height >= -1 cm).
- Non-Skating: during contact, horizontal foot speed < ``skate_thresh``
  (10 cm/s).
- Motion Fidelity: per-link position error < 10 cm AND per-link orientation
  error < 10 deg vs the scaled human reference, evaluated at the link
  correspondences of the GMR ik_config match tables (requires the source BVH;
  see ``motion_fidelity``). Gating (``--fidelity-gating``): "weighted"
  (default) gates each axis only on links whose GMR IK weight for that axis
  is nonzero; "strict" gates every matched link on both axes (PHUMA
  comparability). For motions produced with GMR grounding
  (``retarget(..., offset_to_ground=True)``), pass ``--grounded-reference``
  (and the suite's ``--ref-ground-offset`` if nonzero) so the reference
  matches what GMR actually solved.

Contact inference (LAFAN1 ships no contact labels)
--------------------------------------------------
Contact is inferred from the ROBOT foot trajectory, not the human reference:
a foot is "in contact" on a frame when a policy-specific height/velocity test
passes. Three policies are available via ``contact_policy`` (the pre-existing
``contact_mode`` keyword is kept as an exact synonym, so score_motion.py's
current ``--contact-mode adaptive|fixed`` CLI keeps working unchanged):

- ``"adaptive"`` (DEFAULT; use for UNGROUNDED / legacy pkls). Per-foot
  threshold = ``percentile(height, contact_percentile) + contact_band``
  (defaults p5 + 3 cm), contact = ``height < threshold AND |vz| < contact_vz_max``.
  Self-contained: works on a bare GMR ``.pkl`` (robot name is the only other
  input; the human BVH is only needed for Motion Fidelity) with no assumption
  about where world z=0 sits. Robust to GMR issue #180: GMR's hard-coded
  ground offset biases the whole trajectory upward by a roughly constant
  amount, so contact phases remain the *local minima* of foot height even
  when the absolute height is wrong -- a fixed world-frame threshold would
  find zero contact frames on a floating motion and make
  Non-Floating/Non-Penetration/Non-Skating degenerate. GUARDED against the
  round-2 pathology below: the resolved threshold is clamped to
  ``max(threshold, 0.0)``.
- ``"fixed"`` (GROUNDED pkls only). Threshold = ``contact_height`` (default
  3 cm) measured from world z=0, height-only (no velocity gate). Requires the
  pkl to actually be grounded (produced with ``offset_to_ground=True``, or a
  correct ``--ground-z``) -- on an ungrounded pkl this finds ~0 contact
  frames (the whole trajectory floats above 3 cm), same degenerate failure
  mode as a naive fixed threshold under "adaptive" above.
- ``"anchored"`` (GROUNDED pkls; the PRINCIPLED policy, use as the default
  for grounded suites). z=0-anchored two-part test: contact when lowest sole
  z < ``contact_height`` (3 cm) AND |vertical speed| <
  ``contact_anchor_vz_max`` -- a SMALL threshold (default 5 cm/s, well below
  adaptive/fixed's 20 cm/s ``contact_vz_max``). The velocity gate is what
  makes this "principled" versus plain "fixed": a height-only test also
  counts a foot that is merely swinging *through* the 3 cm band near
  heel-strike/toe-off (fast, not planted), which inflates the contact
  denominator with transient frames and corrupts Non-Skating (a fast foot
  passing through the band could pass the skate check by luck, or fail it
  for a frame that was never really "in contact"). Requiring near-zero
  vertical speed restricts the denominator to genuine stance frames.

Suite-type default: ``score_motion_file``'s explicit default stays
"adaptive" for CLI back-compat (score_motion.py always passes an explicit
``--contact-mode`` value, currently choices ``adaptive``/``fixed`` only --
wiring "anchored" and "auto" as ``--contact-mode``/new ``--contact-policy``
CLI choices is a score_motion.py follow-up, out of scope for this module).
Direct callers (batch drivers, notebooks) should: pick ``"adaptive"`` for
ungrounded/legacy pkls, ``"anchored"`` for grounded ("patched",
``offset_to_ground=True``) pkls, or pass ``contact_policy="auto"`` to get
that same selection automatically from the ``grounded_reference`` flag
already threaded through ``score_motion_file`` for Motion Fidelity.

Round-2 pathology and its guard
--------------------------------
The adaptive per-foot threshold ``percentile(height, 5) + 3cm`` can go
NEGATIVE on clips with genuine, deep sole-corner penetration: GMR grounds the
human ANKLE CENTER and ignores sole extent/pitch, so at heel-strike/toe-off
the robot sole CORNER can dip 3-10 cm below the nominal contact height even
on a well-grounded retarget. When the 5th-percentile foot height is already
below -3 cm, the adaptive threshold goes negative and the "in contact"
denominator collapses onto exactly those deepest-penetration frames -- Non-
Penetration (height >= -1 cm) then fails on ~all of them BY CONSTRUCTION,
scoring 0.0 regardless of overall retarget quality. Observed on the round-2
patched sweep: fallAndGetUp3_subject1, run1_subject5, and fight1_subject3 all
scored adaptive-mode Non-Penetration 0.0 with thresholds of roughly -1 to
-2 cm.
Every policy therefore clamps its resolved threshold to ``max(threshold,
0.0)`` -- world z=0 (true ground) is the most permissive floor that still
means "at or below ground," rather than letting the band chase penetration
downward and shrink the denominator to the pathological frames.
``foot_contact_metrics`` additionally logs ``contact_frames_by_foot`` (raw
per-foot count) and ``contact_frac_by_foot`` (fraction of the clip) in its
output JSON so a concentrated denominator -- the signature of this
pathology -- is visible without re-deriving it from the pkl.

Non-Skating (and Non-Floating / Non-Penetration) comparability caveat
-----------------------------------------------------------------------
Non-Floating / Non-Penetration / Non-Skating are percentages OF THE CONTACT
DENOMINATOR, and each policy selects a different set of "in contact" frames
(different size, different frames). Comparing any of these three metrics
ACROSS runs that used different ``contact_policy`` values is NOT meaningful
-- e.g. adaptive Non-Skating=95 vs anchored Non-Skating=80 on the same clip
does not mean the anchored contact frames "skate more," it means the two
policies picked different (and differently sized) frame sets with different
composition. Only compare a metric across runs that used the SAME
``contact_policy``; ``foot_contact_metrics`` records the policy used as both
``contact_mode`` and ``contact_policy`` in its output specifically so this is
auditable downstream.

All thresholds are exposed as CLI flags in score_motion.py (adaptive/fixed
only at present; see "Suite-type default" above).

Input format: GMR .pkl as written by GMR scripts/bvh_to_robot.py:
``{"fps": int, "root_pos": (T,3), "root_rot": (T,4) xyzw, "dof_pos": (T,D)}``
where dof_pos follows MJCF qpos[7:] order for the robot given by GMR's
ROBOT_XML_DICT (single free root joint assumed and validated).

Changelog
---------
2026-08-03  Audit fixes D1-D3:
  D1: Joint Feasibility position limits now come from a mechanical URDF
      (``MECHANICAL_URDF`` / ``--pos-urdf``) instead of the possibly
      IK-narrowed MJCF ``jnt_range``. G1 source: eval/assets/
      g1_29dof_official.urdf, vendored unmodified from unitreerobotics/
      unitree_ros g1_description (position limits verified identical to
      GMR's g1_custom_collision_29dof.urdf on all 29 joints). Velocity
      limits keep their previous source (``DEFAULT_URDF``); see
      ``retarget_configs/velocity-limit-sources.md`` for the source decision.
  D2: ``--grounded-reference`` (+ ``--ref-ground-offset``) applies GMR's
      ``offset_to_ground`` grounding to the Motion Fidelity reference so it
      matches the IK targets GMR solved for grounded ("patched") suites.
  D3: ``--fidelity-gating=weighted|strict`` (weighted default): fidelity no
      longer gates frames on axes with zero GMR IK weight; strict preserves
      pre-D3/PHUMA behavior.
2026-08-04  D5 (T4): velocity-limit source decision. The G1 companion URDF
      (GMR's ``g1_custom_collision_29dof.urdf``) disagrees with the official
      vendor URDF on 8 joints' ``<limit velocity=...>`` (ankle pitch/roll x2,
      hip_roll x2, waist pitch/roll: see retarget_configs/
      velocity-limit-sources.md for the full table). The official URDF is
      the vendor's word and is what gets quoted externally, so it is now the
      ``DEFAULT_URDF`` default for G1 (same vendored file already used for
      position limits, ``eval/assets/g1_29dof_official.urdf`` /
      ``MECHANICAL_URDF``); the companion values remain reachable via
      ``--urdf`` pointed at ``LEGACY_COMPANION_URDF`` for A/B comparison
      against pre-D5 scores.
2026-08-04  T2: contact-inference policy overhaul, adding ``"anchored"`` as a
      third ``contact_policy`` (``infer_contact``/``foot_contact_metrics``/
      ``score_motion_file`` all accept the new ``contact_policy`` keyword;
      the pre-existing ``contact_mode`` keyword is unchanged/still accepted
      for score_motion.py CLI back-compat -- see module docstring, "Contact
      inference"). Two behavior changes to be aware of when comparing against
      pre-T2 scores:
      (a) GUARD: every policy's resolved threshold is now clamped to
          ``max(threshold, 0.0)``. This fixes the round-2 gate failure where
          the adaptive threshold (``percentile(height,5)+3cm``) went negative
          on clips with deep sole-corner penetration and collapsed the
          contact denominator onto exactly those frames, scoring
          Non-Penetration 0.0 by construction (fallAndGetUp3_subject1,
          run1_subject5, and fight1_subject3). Re-scored with the
          guard (patched suites, mean of 6 clips, contact_policy="adaptive",
          otherwise-default thresholds): G1 Non-Floating/Non-Penetration
          100.0/57.41 (was 100.0/35.77).
      (b) SEMANTICS: "fixed" is now a bare height test (no velocity gate) --
          previously "fixed" applied the same ``|vz| < contact_vz_max`` (0.2
          m/s) gate as "adaptive". The new "anchored" policy is what carries
          a velocity gate forward, with a much smaller default
          (``contact_anchor_vz_max`` = 0.05 m/s) specifically so it only
          admits genuinely-planted stance frames; it is the recommended
          default for grounded suites (G1 mean across 6 clips: 71.44/85.20).
          Pre-T2 "fixed-mode sensitivity" numbers quoted elsewhere used the
          OLD vz-gated "fixed" semantics and are not
          directly comparable to either the new "fixed" or "anchored"; treat
          "anchored" as superseding that sensitivity run.
      Per-foot contact-frame counts are now always logged
      (``contact_frames_by_foot`` / ``contact_frac_by_foot`` in
      ``foot_contact_metrics``'s output), with a heuristic warning when one
      foot contributes <2% of frames to the denominator (the pathology's
      signature). Non-Skating/Non-Floating/Non-Penetration comparisons are
      only valid within a single ``contact_policy`` -- see module docstring.
2026-08-04  T2: gate v2 + root-translation smoothness metric (a SEPARATE
      work item from the "T2" changelog entry above, which is contact-policy
      work from a different task in the same day's round; kept as-is rather
      than renumbered to avoid rewriting history other agents may already
      reference). Two additions, both new sections/functions -- contact
      inference and the limit resolver above are unchanged by this entry:
      (a) ``root_motion_metrics`` (new "Root-translation smoothness"
          section, right after ``jerk_metrics``): linear acceleration and
          jerk of ``root_pos`` via direct finite differences at the clip's
          own fps (``np.diff(root_pos, n=2)*fps**2`` / ``n=3)*fps**3``,
          m/s^2 / m/s^3), max + p99 per axis and 3D magnitude, plus
          ``static_frame_frac`` (fraction of frames identical to the
          previous one). Closes the gap ``jerk_metrics`` always had --
          it reads ``dof_pos`` only. The round-3 grounding post-process
          injects base-z accel spikes up to
          127 m/s^2 that the pre-T2 harness could not see. Wired into
          ``score_motion_file``'s output as the ``"root_motion"`` key.
      (b) ``evaluate_gate_v2`` (new "Gate v2" section, end of file) + the
          ``--gate v2`` flag on score_motion.py: PASS requires NF>80,
          NP>80, NS>60 (all under ``contact_policy=="anchored"`` -- refused
          otherwise, since NF/NP/NS are policy-specific denominators) AND
          root-accel-magnitude p99 <= 60 m/s^2 (bound derived from the 24
          honest clips available at T2 time, observed range 9.34-42.11;
          see the "Gate v2" section's module-level comment for the full
          derivation and the ns_min=60 rationale), plus a ``not_degenerate``
          precondition (``static_frame_frac < 0.99``) that fails the gate
          outright with reason "no contact dynamics" on a frozen/static
          pose -- closing round-3 V5's finding that the old NF>80-AND-NP>80
          gate is satisfied vacuously by a motionless pose (100/100/100)
          and cannot separate an honest grounded retarget from
          block-shuffled garbage on Non-Floating alone (gap G11). Self-test
          in ``tools/checks/gate_v2_selftest.py``: a synthesized frozen-pose
          pkl and a synthesized block-shuffled pkl (root_pos + root_rot +
          dof_pos cut into fixed blocks, block order permuted with a fixed
          seed) both FAIL gate v2 with the expected reasons (frozen: "no
          contact dynamics"; shuffled: fails both non_skating AND
          root_accel_p99_mag); one honest UNPATCHED clip's root-accel p99
          is confirmed well under the 60 m/s^2 bound (not spuriously
          failed). Exact numbers are in that script's own run output, not
          duplicated here to avoid a second place to keep in sync.
2026-08-04  Gate v2 hardening (post-round-4 V6 audit).
      Two defects fixed, both in ``root_motion_metrics`` / ``evaluate_gate_v2``
      (the "Gate v2" section at the end of this file has the full derivation):
      (a) ``not_degenerate`` was defeated by 1 micrometre of Gaussian dither
          on a frozen pose (``static_frame_frac`` uses exact-equality
          ``np.isclose``, so any nonzero per-frame noise drops it to ~0 and
          passes the precondition vacuously). Fixed by requiring genuine
          motion MAGNITUDE, not just non-repetition: two new
          ``root_motion_metrics`` fields, ``root_disp_median_m`` and
          ``dof_disp_median_rad`` (median per-frame L2 displacement of
          root_pos / dof_pos), each with a conservative G1-corpus floor;
          ``static_frame_frac`` is kept as an additional, not replacement,
          check. (b) ``root_accel_p99_mag<=60`` could not detect the
          grounding-pass artifact (all six post-processed G1 clips passed it;
          honest/processed p99 ranges fully overlap). Added
          ``root_accel_max_mag`` and ``root_accel_frames_over_thresh`` as
          population-level criteria per the task brief -- HONESTLY
          re-deriving the honest-suite max range in the process found it is
          22.18-188.61 m/s^2 (not the ~78 previously assumed), which makes
          both new population-level criteria structurally unable to flag
          the one known artifact (max 127.15) without also failing a
          genuine honest G1 clip (`fight1_subject3`, max 188+, no
          post-processing involved) -- documented as a known limitation,
          not silently worked around. Added ``root_accel_paired_delta``
          (new function) as an opt-in, per-clip PAIRED comparison against
          an explicit known-honest baseline of the same clip; this DOES
          catch a real subset (2/6 currently-available grounded G1 clips)
          using a threshold (5 m/s^2 max-accel increase) chosen from a
          clean empirical gap in the observed deltas, not from the honest
          population. Self-test battery re-run and extended
          (``tools/checks/gate_v2_selftest.py``): frozen pose, frozen + 1um/
          1e-4/1e-2 m dither (all FAIL), block-shuffle at block=5 (FAILS)
          and block=10 (documented limitation -- V6 found Non-Skating
          alone stops rejecting at block>=10; see that script's own output
          and the "known limitations" section of ``evaluate_gate_v2``'s
          docstring for the honest per-criterion result at block=10).
2026-10-10  Review fixes (module split + fail-closed gate v2).
      (a) LAYOUT: the implementation moved into ``eval/retarget_metrics/``
          (io, robot, contact, kinematics, fidelity, scoring, gate); this file
          is now a backwards-compatible re-export facade, so ``import
          metrics`` (eval/ on sys.path) and ``from eval.metrics import ...``
          both keep working unchanged.  The foot-geom selection, geom lowest
          point, central difference and contact predicate are the shared
          ``mocap_g1.contact`` implementations (also used by
          mocap_g1.contact_refine and tools/retarget/postprocess_grounding.py);
          the threshold VALUES are unchanged and stay policy-specific.
      (b) ``load_gmr_pkl`` now fails closed on non-finite arrays, shape /
          frame-count mismatches, non-positive fps and non-unit root
          quaternions, and enforces the explicit ``root_quat_order`` pkl key
          (``mocap_g1.gmr_pkl``).  ``quat_order`` defaults to None (key
          required); passing it is an explicit LEGACY override for keyless
          pre-contract pkls (score_motion.py ``--quat-order``).
      (c) gate v2 VERDICT CHANGES (each fails closed): new precondition
          ``contact_coverage`` (contact_frac >= 0.05 and >= 0.02 per foot --
          a 1-contact-frame clip used to score NF/NP/NS = 100);
          new criterion ``joint_position_limits`` (no frame outside the
          mechanical [lo, hi], from ``joint_feasibility``'s new
          ``position_hard_violation_frames``); ``root_accel_paired_delta``
          now FAILS when a baseline is supplied but the delta cannot be
          computed (it used to pass); NaN statistics can no longer satisfy
          ``not_degenerate`` (``nan < floor`` compared False).
"""

# ---------------------------------------------------------------------------
# Backwards-compatible facade: every name below used to be defined here.
# ---------------------------------------------------------------------------

from __future__ import annotations

import sys
from pathlib import Path

if __package__:  # imported as ``eval.metrics``
    from .retarget_metrics import contact as _contact
    from .retarget_metrics import fidelity as _fidelity
    from .retarget_metrics import gate as _gate
    from .retarget_metrics import io as _io
    from .retarget_metrics import kinematics as _kinematics
    from .retarget_metrics import robot as _robot
    from .retarget_metrics import scoring as _scoring
else:  # imported as top-level ``metrics`` with eval/ on sys.path
    _EVAL_DIR = str(Path(__file__).resolve().parent)
    if _EVAL_DIR not in sys.path:
        sys.path.insert(0, _EVAL_DIR)
    from retarget_metrics import contact as _contact  # noqa: E402
    from retarget_metrics import fidelity as _fidelity  # noqa: E402
    from retarget_metrics import gate as _gate  # noqa: E402
    from retarget_metrics import io as _io  # noqa: E402
    from retarget_metrics import kinematics as _kinematics  # noqa: E402
    from retarget_metrics import robot as _robot  # noqa: E402
    from retarget_metrics import scoring as _scoring  # noqa: E402

import mujoco as mj  # noqa: E402,F401  (historically importable as metrics.mj / .np)
import numpy as np  # noqa: E402,F401

_EXPORTS = {
    _io: (
        "MOCAP_WS", "DEFAULT_URDF", "LEGACY_COMPANION_URDF", "MECHANICAL_URDF",
        "GMR_ROOT_QUAT_ORDER", "ROOT_QUAT_ORDER_KEY", "UNIT_QUATERNION_TOLERANCE",
        "RootQuatOrderError", "resolve_robot_xml", "load_gmr_pkl", "parse_urdf_limits",
    ),
    _robot: (
        "FOOT_PATTERNS", "HAND_PATTERNS", "JointInfo", "RobotModel", "_body_children",
        "_subtree", "_body_depth", "resolve_foot_bodies", "build_robot",
    ),
    _contact: (
        "SPARSE_FOOT_CONTACT_FRAC", "_mesh_verts", "geom_lowest_z", "run_fk_pass",
        "_central_diff", "CONTACT_POLICIES", "resolve_contact_policy", "infer_contact",
        "foot_contact_metrics",
    ),
    _kinematics: (
        "POSITION_HARD_LIMIT_TOL_RAD", "joint_feasibility", "UPPER_BODY_PATTERN",
        "TORQUE_GATE_JOINT_SETS", "_finite_difference_state", "torque_feasibility",
        "jerk_metrics", "root_motion_metrics", "root_accel_paired_delta",
    ),
    _fidelity: ("build_fidelity_reference", "motion_fidelity"),
    _scoring: ("score_motion_file",),
    _gate: (
        "GATE_V2_NF_MIN", "GATE_V2_NP_MIN", "GATE_V2_NS_MIN",
        "GATE_V2_ROOT_ACCEL_P99_MAX_MPS2", "GATE_V2_STATIC_FRAC_MAX",
        "GATE_V2_ROOT_DISP_MEDIAN_MIN_M", "GATE_V2_DOF_DISP_MEDIAN_MIN_RAD",
        "GATE_V2_ROOT_ACCEL_MAX_MPS2", "GATE_V2_ROOT_ACCEL_COUNT_THRESH_MPS2",
        "GATE_V2_ROOT_ACCEL_MAX_FRAMES_OVER", "GATE_V2_PAIRED_DELTA_MAX_MPS2",
        "GATE_V2_MIN_CONTACT_FRAC", "GATE_V2_MIN_FOOT_CONTACT_FRAC",
        "GATE_V2_MAX_POSITION_LIMIT_VIOLATION_FRAMES", "_finite", "evaluate_gate_v2",
    ),
}
for _module, _names in _EXPORTS.items():
    for _name in _names:
        globals()[_name] = getattr(_module, _name)
del _module, _names, _name

__all__ = [name for names in _EXPORTS.values() for name in names if not name.startswith("_")]
