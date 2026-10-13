#!/usr/bin/env python
"""Score GMR retargeted motions (.pkl) with the PHUMA-style metric suite.

Examples
--------
Score one motion (foot metrics, feasibility, jerk, self-collision)::

    python eval/score_motion.py out/dance1_subject2_g1.pkl --robot unitree_g1

Add Motion Fidelity by pointing at the source BVH::

    python eval/score_motion.py out/dance1_subject2_g1.pkl --robot unitree_g1 \
        --bvh data/lafan1/dance1_subject2.bvh

Score a directory of pkls, matching BVHs by stem from a directory::

    python eval/score_motion.py out/ --robot unitree_g1 --bvh-dir data/lafan1 \
        --out out/scores.json

BVH stem matching: for pkl ``foo_bar_g1.pkl`` we try ``foo_bar_g1.bvh``,
then progressively strip trailing ``_token`` pieces (``foo_bar.bvh``, ...)
until a file in --bvh-dir matches.

Run with the workspace venv python (GMR + mujoco installed).

Exit status: 0 only when EVERY input was scored; 1 when any input failed to
score (the JSON with the partial results and a ``failures`` map is still
written) or the payload is not strict JSON; 2 for usage errors.  A gate v2
FAIL is a verdict, not an error, and does not change the exit status.

Quaternion order: pkls must carry ``root_quat_order: "xyzw"`` (every writer
in this repo stamps it; see mocap_g1/gmr_pkl.py).  ``--quat-order`` is only an
explicit override for LEGACY keyless pkls whose provenance you know.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import metrics  # noqa: E402

from mocap_g1.io_utils import sha256_file, write_json_atomic  # noqa: E402


def _file_provenance(path: Path | None) -> dict | None:
    if path is None:
        return None
    return {"path": str(path), "sha256": sha256_file(path)}


def find_bvh(pkl: Path, bvh_dir: Path) -> Path | None:
    stem = pkl.stem
    while True:
        cand = bvh_dir / f"{stem}.bvh"
        if cand.is_file():
            return cand
        if "_" not in stem:
            return None
        stem = stem.rsplit("_", 1)[0]


def fmt(v):
    return "n/a" if v is None else v


def print_report(res: dict) -> None:
    jf = res["joint_feasibility"]
    fc = res["foot_contact"]
    jk = res["jerk"]
    sc = res["self_collision"]
    mf = res["motion_fidelity"]
    name = Path(res["file"]).name
    print(f"\n=== {name}  [{res['robot']}] ===")
    print(f"  frames/fps/duration : {res['frames']} @ {res['fps']} fps = {res['duration_s']} s")
    print("  --- PHUMA metrics (% frames passing) ---")
    if "motion_fidelity_pct" in mf:
        tags = [mf.get("gating", "strict") + " gating"]
        if mf.get("reference_grounded"):
            tags.append("grounded ref")
        print(f"  Motion Fidelity     : {mf['motion_fidelity_pct']:>6}   "
              f"(debiased {mf['motion_fidelity_debiased_pct']}; pos-only {mf['position_only_pct']}, "
              f"ori-only {mf['orientation_only_pct']}, "
              f"mean err {mf['mean_link_pos_err_m']} m / {mf['mean_link_ori_err_deg']} deg) "
              f"[{', '.join(tags)}]")
    else:
        print(f"  Motion Fidelity     : {mf['status']} ({mf['reason']})")
    vel_note = "" if jf["velocity_checked"] else "  [velocity limits UNAVAILABLE - positions only]"
    print(f"  Joint Feasibility   : {jf['joint_feasibility_pct']:>6}   "
          f"(pos-only {jf['position_only_pct']}, vel-only {fmt(jf['velocity_only_pct'])}){vel_note}")
    print(f"  Non-Floating        : {fmt(fc['non_floating_pct']):>6}")
    print(f"  Non-Penetration     : {fmt(fc['non_penetration_pct']):>6}")
    print(f"  Non-Skating         : {fmt(fc['non_skating_pct']):>6}")
    print("  --- contact inference ---")
    print(f"  mode={fc['contact_mode']}  thresholds(m)={fc['contact_thresholds_m']}  "
          f"contact frames={fc['contact_frames']} ({round(100 * fc['contact_frac'], 1)}% of clip)")
    print(f"  foot height stats(m): {fc['foot_height_stats_m']}")
    print("  --- smoothness / collisions ---")
    print(f"  jerk max / p99      : {jk.get('jerk_max_rad_s3')} / {jk.get('jerk_p99_rad_s3')} rad/s^3   "
          f"discontinuity frames: {jk.get('discontinuity_frames')}")
    print(f"  self-collision      : {sc['frames_with_self_collision']} frames "
          f"({sc['pct_frames']}%), worst pairs: {list(sc['top_pairs'])[:3]}")
    rm_ = res.get("root_motion", {})
    if "accel" in rm_:
        print(f"  root accel max/p99  : {rm_['accel']['max']['mag']} / {rm_['accel']['p99']['mag']} "
              f"m/s^2 (mag)   jerk max/p99: {rm_['jerk']['max']['mag']} / {rm_['jerk']['p99']['mag']} m/s^3   "
              f"static frame frac: {rm_.get('static_frame_frac')}")
    elif rm_.get("warning"):
        print(f"  root motion         : {rm_['warning']}")
    if jf["violating_joints"]:
        print(f"  limit violators     : {list(jf['violating_joints'])[:5]}")
    for n in res.get("notes", []):
        print(f"  note: {n}")
    if "gate_v2" in res:
        print_gate_v2(res["gate_v2"])


def print_gate_v2(gate: dict) -> None:
    verdict = "PASS" if gate["pass"] else "FAIL"
    print(f"  --- gate v2: {verdict} ---")
    order = ["contact_policy_anchored", "contact_coverage", "not_degenerate",
             "non_floating", "non_penetration", "non_skating", "root_accel_p99_mag",
             "root_accel_max_mag", "root_accel_frames_over_thresh",
             "joint_position_limits", "root_accel_paired_delta"]
    for name in order:
        c = gate["checks"].get(name)
        if c is None:
            continue  # root_accel_paired_delta only present when a baseline was supplied
        mark = "PASS" if c["pass"] else "FAIL"
        print(f"    [{mark}] {name:28} value={fmt(c['value'])}   requirement: {c['requirement']}")
    for r in gate["reasons"]:
        print(f"    reason: {r}")


def summary_row(res: dict) -> dict:
    mf = res["motion_fidelity"]
    rmo = res.get("root_motion", {})
    row = {
        "file": Path(res["file"]).name,
        "frames": res["frames"],
        "motion_fidelity": mf.get("motion_fidelity_pct"),
        "motion_fidelity_debiased": mf.get("motion_fidelity_debiased_pct"),
        "joint_feasibility": res["joint_feasibility"]["joint_feasibility_pct"],
        "non_floating": res["foot_contact"]["non_floating_pct"],
        "non_penetration": res["foot_contact"]["non_penetration_pct"],
        "non_skating": res["foot_contact"]["non_skating_pct"],
        "jerk_p99": res["jerk"].get("jerk_p99_rad_s3"),
        "root_accel_p99": (rmo.get("accel") or {}).get("p99", {}).get("mag"),
        "selfcol_pct": res["self_collision"]["pct_frames"],
    }
    if "gate_v2" in res:
        row["gate_v2_pass"] = "PASS" if res["gate_v2"]["pass"] else "FAIL"
    return row


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", help="GMR .pkl file or a directory of .pkl files")
    ap.add_argument("--robot", default="unitree_g1",
                    help="GMR robot name (default: unitree_g1)")
    ap.add_argument("--mjcf", default=None, help="explicit MJCF path (overrides --robot lookup)")
    ap.add_argument("--urdf", default=None,
                    help="URDF used only for joint velocity limits (auto for known robots: "
                         "official vendor URDF as of D5; pass metrics.LEGACY_COMPANION_URDF "
                         "[GMR's g1_custom_collision_29dof.urdf] here to reproduce pre-D5 "
                         "companion-sourced scores -- see retarget_configs/"
                         "velocity-limit-sources.md)")
    ap.add_argument("--pos-urdf", default=None,
                    help="mechanical URDF for joint POSITION limits (auto for known "
                         "robots; MJCF jnt_range may be IK-narrowed)")
    ap.add_argument("--bvh", default=None, help="source BVH for Motion Fidelity (single pkl)")
    ap.add_argument("--bvh-dir", default=None, help="directory of source BVHs (matched by stem)")
    ap.add_argument("--src-format", default="lafan1", choices=["lafan1", "nokov"],
                    help="BVH flavor for GMR's loader")
    ap.add_argument("--quat-order", default=None, choices=["xyzw", "wxyz"],
                    help="LEGACY override: root_rot order for pkls WITHOUT a "
                         "root_quat_order key (GMR's own bvh_to_robot.py saves xyzw). "
                         "Default: require the key and refuse unlabeled pkls; a value "
                         "that conflicts with a present key is refused too")
    ap.add_argument("--foot-bodies", default=None,
                    help="comma-separated left,right foot body names (else auto by pattern)")
    ap.add_argument("--out", default=None, help="JSON output path (default <input>.metrics.json)")
    ap.add_argument("--json-only", action="store_true", help="suppress the readable table")
    # contact inference
    ap.add_argument("--contact-mode", default="adaptive", choices=["adaptive", "fixed"],
                    help="pre-existing 2-choice flag, kept for back-compat; prefer "
                         "--contact-policy (superset of 3 policies + 'auto')")
    ap.add_argument("--contact-policy", default=None,
                    choices=["adaptive", "fixed", "anchored", "auto"],
                    help="metrics.py's full contact_policy (see eval/metrics.py module "
                         "docstring, 'Contact inference'): adaptive (ungrounded/legacy "
                         "pkls), fixed (grounded, height-only), anchored (grounded, "
                         "height + small vz gate -- the recommended default for grounded "
                         "suites), auto (anchored if --grounded-reference else adaptive). "
                         "Overrides --contact-mode when given.")
    ap.add_argument("--contact-height", type=float, default=0.03,
                    help="fixed/anchored-mode contact height threshold (m)")
    ap.add_argument("--contact-percentile", type=float, default=5.0,
                    help="adaptive-mode height percentile anchoring the threshold")
    ap.add_argument("--contact-band", type=float, default=0.03,
                    help="adaptive-mode band (m) above the percentile height")
    ap.add_argument("--contact-vz-max", type=float, default=0.2,
                    help="adaptive-mode max |vertical velocity| (m/s) for contact")
    ap.add_argument("--contact-anchor-vz-max", type=float, default=0.05,
                    help="anchored-mode max |vertical velocity| (m/s) for contact "
                         "(small vs --contact-vz-max: this is what makes 'anchored' "
                         "principled rather than a bare height test)")
    ap.add_argument("--ground-z", type=float, default=0.0, help="ground plane height (m)")
    # metric thresholds (PHUMA defaults)
    ap.add_argument("--float-thresh", type=float, default=0.01)
    ap.add_argument("--pen-thresh", type=float, default=0.01)
    ap.add_argument("--skate-thresh", type=float, default=0.10)
    ap.add_argument("--limit-frac", type=float, default=0.98)
    ap.add_argument("--pos-fid-thresh", type=float, default=0.10)
    ap.add_argument("--ori-fid-thresh-deg", type=float, default=10.0)
    ap.add_argument("--fidelity-gating", default="weighted", choices=["weighted", "strict"],
                    help="weighted: gate fidelity only on axes with nonzero GMR IK "
                         "weight; strict: gate all matched links on both axes (PHUMA)")
    ap.add_argument("--grounded-reference", action="store_true",
                    help="apply GMR offset_to_ground grounding to the fidelity "
                         "reference (use for suites retargeted with offset_to_ground=True)")
    ap.add_argument("--ref-ground-offset", type=float, default=0.0,
                    help="ground_offset the suite's driver passed to GMR "
                         "set_ground_offset (only with --grounded-reference)")
    ap.add_argument("--disc-jump-thresh", type=float, default=1.0,
                    help="discontinuity detector threshold: |d2q| rad/frame^2")
    # gate v2 (T2)
    ap.add_argument("--gate", default=None, choices=["v2"],
                    help="apply metrics.evaluate_gate_v2 to each scored motion and print "
                         "a per-criterion PASS/FAIL verdict (see eval/metrics.py, 'Gate v2' "
                         "section, for the full definition and threshold rationale). "
                         "Requires --contact-policy anchored to have any chance of passing "
                         "-- NS/NF/NP are only comparable within one contact policy, and "
                         "ns_min was calibrated against the anchored suite.")
    ap.add_argument("--gate-nf-min", type=float, default=metrics.GATE_V2_NF_MIN)
    ap.add_argument("--gate-np-min", type=float, default=metrics.GATE_V2_NP_MIN)
    ap.add_argument("--gate-ns-min", type=float, default=metrics.GATE_V2_NS_MIN)
    ap.add_argument("--gate-root-accel-p99-max", type=float,
                    default=metrics.GATE_V2_ROOT_ACCEL_P99_MAX_MPS2,
                    help="gate v2 criterion (d): root-accel-magnitude p99 bound, m/s^2")
    ap.add_argument("--gate-static-frac-max", type=float,
                    default=metrics.GATE_V2_STATIC_FRAC_MAX,
                    help="gate v2 'not_degenerate' precondition: max allowed fraction of "
                         "frames identical to the previous frame before a motion is "
                         "treated as frozen/static ('no contact dynamics')")
    ap.add_argument("--gate-root-disp-min-m", type=float,
                    default=metrics.GATE_V2_ROOT_DISP_MEDIAN_MIN_M,
                    help="gate v2 'not_degenerate' precondition (V6 hardening): min "
                         "median per-frame root_pos displacement, m -- closes the "
                         "dithered-frozen-pose gaming hole in --gate-static-frac-max alone")
    ap.add_argument("--gate-dof-disp-min-rad", type=float,
                    default=metrics.GATE_V2_DOF_DISP_MEDIAN_MIN_RAD,
                    help="gate v2 'not_degenerate' precondition (V6 hardening): min "
                         "median per-frame dof_pos L2 displacement, rad")
    ap.add_argument("--gate-root-accel-max-mps2", type=float,
                    default=metrics.GATE_V2_ROOT_ACCEL_MAX_MPS2,
                    help="gate v2 criterion (e), V6 hardening: root-accel-magnitude MAX "
                         "bound, m/s^2 (supplements p99; see eval/metrics.py 'Gate v2' "
                         "section for why this is currently inactive against the known "
                         "grounding-pass artifact on the available honest-suite data)")
    ap.add_argument("--gate-max-frames-over-thresh", type=int,
                    default=metrics.GATE_V2_ROOT_ACCEL_MAX_FRAMES_OVER,
                    help="gate v2 criterion (f), V6 hardening: max allowed count of "
                         "frames with root-accel magnitude above "
                         "--gate-root-accel-count-thresh-mps2")
    ap.add_argument("--gate-root-accel-count-thresh-mps2", type=float,
                    default=metrics.GATE_V2_ROOT_ACCEL_COUNT_THRESH_MPS2,
                    help="per-frame accel threshold (m/s^2) used by the "
                         "root_accel_frames_over_thresh count, and by root_motion's "
                         "reported frames_over_thresh")
    ap.add_argument("--baseline-pkl", default=None,
                    help="OPTIONAL: a known-honest counterpart pkl of the SAME clip "
                         "(e.g. the pre-grounding-pass 'patched' pkl for a 'grounded' "
                         "candidate). When given, adds the opt-in "
                         "root_accel_paired_delta gate v2 check (see "
                         "metrics.root_accel_paired_delta / evaluate_gate_v2's "
                         "baseline_root_motion parameter) -- catches a per-clip root-accel "
                         "regression that the population-level max/count-over-threshold "
                         "criteria cannot (see eval/metrics.py 'Gate v2' section)")
    ap.add_argument("--gate-paired-delta-max-mps2", type=float,
                    default=metrics.GATE_V2_PAIRED_DELTA_MAX_MPS2,
                    help="only used with --baseline-pkl: max allowed increase in "
                         "root-accel max magnitude vs the baseline, m/s^2 (the check "
                         "FAILS when the delta cannot be computed)")
    ap.add_argument("--gate-min-contact-frac", type=float,
                    default=metrics.GATE_V2_MIN_CONTACT_FRAC,
                    help="gate v2 'contact_coverage' precondition: min fraction of "
                         "frames with any foot in contact (NF/NP/NS denominator)")
    ap.add_argument("--gate-min-foot-contact-frac", type=float,
                    default=metrics.GATE_V2_MIN_FOOT_CONTACT_FRAC,
                    help="gate v2 'contact_coverage' precondition: min per-foot "
                         "contact fraction")
    ap.add_argument("--gate-max-position-limit-violation-frames", type=int,
                    default=metrics.GATE_V2_MAX_POSITION_LIMIT_VIOLATION_FRAMES,
                    help="gate v2 'joint_position_limits' criterion: max frames with "
                         "any joint outside its mechanical position range")
    args = ap.parse_args()

    inp = Path(args.input)
    if inp.is_dir():
        pkls = sorted(inp.glob("*.pkl"))
        if not pkls:
            print(f"error: no .pkl files in {inp}", file=sys.stderr)
            return 2
    elif inp.is_file():
        pkls = [inp]
    else:
        print(f"error: {inp} not found", file=sys.stderr)
        return 2

    foot_override = args.foot_bodies.split(",") if args.foot_bodies else None
    rm = metrics.build_robot(args.robot, args.mjcf, args.urdf, foot_override,
                             pos_urdf=args.pos_urdf)

    bvh_dir = Path(args.bvh_dir) if args.bvh_dir else None
    results, failures = {}, {}
    for pkl in pkls:
        bvh = args.bvh
        if bvh is None and bvh_dir is not None:
            found = find_bvh(pkl, bvh_dir)
            bvh = str(found) if found else None
        try:
            res = metrics.score_motion_file(
                pkl, rm,
                bvh_path=bvh, src_format=args.src_format, quat_order=args.quat_order,
                ground_z=args.ground_z, contact_mode=args.contact_mode,
                contact_policy=args.contact_policy,
                contact_height=args.contact_height,
                contact_percentile=args.contact_percentile,
                contact_band=args.contact_band, contact_vz_max=args.contact_vz_max,
                contact_anchor_vz_max=args.contact_anchor_vz_max,
                float_thresh=args.float_thresh, pen_thresh=args.pen_thresh,
                skate_thresh=args.skate_thresh, limit_frac=args.limit_frac,
                pos_fid_thresh=args.pos_fid_thresh,
                ori_fid_thresh_deg=args.ori_fid_thresh_deg,
                fidelity_gating=args.fidelity_gating,
                grounded_reference=args.grounded_reference,
                ref_ground_offset=args.ref_ground_offset,
                disc_jump_thresh=args.disc_jump_thresh,
                root_accel_count_thresh_mps2=args.gate_root_accel_count_thresh_mps2)
        except Exception as e:
            failures[pkl.stem] = f"{type(e).__name__}: {e}"
            print(f"FAILED {pkl.name}: {failures[pkl.stem]}", file=sys.stderr)
            continue
        if args.gate == "v2":
            baseline_root_motion = None
            if args.baseline_pkl:
                try:
                    baseline_motion = metrics.load_gmr_pkl(
                        args.baseline_pkl, quat_order=args.quat_order)
                except Exception as e:
                    failures[pkl.stem] = f"baseline {type(e).__name__}: {e}"
                    print(f"FAILED {pkl.name}: {failures[pkl.stem]}", file=sys.stderr)
                    continue
                baseline_root_motion = metrics.root_motion_metrics(
                    baseline_motion, count_thresh_mps2=args.gate_root_accel_count_thresh_mps2)
            res["gate_v2"] = metrics.evaluate_gate_v2(
                res, nf_min=args.gate_nf_min, np_min=args.gate_np_min,
                ns_min=args.gate_ns_min, root_accel_p99_max=args.gate_root_accel_p99_max,
                static_frac_max=args.gate_static_frac_max,
                root_disp_min_m=args.gate_root_disp_min_m,
                dof_disp_min_rad=args.gate_dof_disp_min_rad,
                root_accel_max_mps2=args.gate_root_accel_max_mps2,
                max_frames_over_thresh=args.gate_max_frames_over_thresh,
                baseline_root_motion=baseline_root_motion,
                paired_delta_max_mps2=args.gate_paired_delta_max_mps2,
                min_contact_frac=args.gate_min_contact_frac,
                min_foot_contact_frac=args.gate_min_foot_contact_frac,
                max_position_limit_violation_frames=(
                    args.gate_max_position_limit_violation_frames))
        results[pkl.stem] = res
        if not args.json_only:
            print_report(res)

    if not results:
        print("error: every input failed", file=sys.stderr)
        return 1

    payload = {
        "robot": args.robot,
        "mjcf": str(rm.xml_path),
        # Exact files every motion in this report was graded against.
        "provenance": {
            "mjcf": _file_provenance(Path(rm.xml_path).resolve()),
            "position_limits_urdf": _file_provenance(rm.pos_limits_urdf),
            "velocity_limits_urdf": _file_provenance(rm.vel_limits_urdf),
        },
        "config": {k: v for k, v in vars(args).items() if k not in ("input", "out")},
        "motions": results,
        "failures": failures,
    }
    if len(results) > 1:
        rows = [summary_row(r) for r in results.values()]
        agg = {}
        for key in ("motion_fidelity", "motion_fidelity_debiased", "joint_feasibility",
                    "non_floating", "non_penetration", "non_skating", "root_accel_p99",
                    "selfcol_pct"):
            vals = [r[key] for r in rows if r[key] is not None]
            agg[f"mean_{key}"] = round(sum(vals) / len(vals), 2) if vals else None
        if args.gate == "v2":
            gate_flags = [r["gate_v2_pass"] for r in rows if "gate_v2_pass" in r]
            agg["gate_v2_pass_count"] = sum(1 for g in gate_flags if g == "PASS")
            agg["gate_v2_total"] = len(gate_flags)
        payload["aggregate"] = agg
        if not args.json_only:
            print("\n=== summary ===")
            hdr = ["file", "frames", "motion_fidelity", "motion_fidelity_debiased",
                   "joint_feasibility", "non_floating", "non_penetration", "non_skating",
                   "jerk_p99", "root_accel_p99", "selfcol_pct"]
            if args.gate == "v2":
                hdr = hdr + ["gate_v2_pass"]
            print("  " + " | ".join(f"{h:>16}" for h in hdr))
            for r in rows:
                print("  " + " | ".join(f"{fmt(r.get(h)):>16}" for h in hdr))
            print(f"  aggregate: {agg}")

    out = Path(args.out) if args.out else (
        inp / "scores.json" if inp.is_dir() else inp.with_suffix(".metrics.json"))
    try:
        # Strict JSON (a NaN/inf anywhere is an error, never a silent "NaN"
        # token), serialized before the target is touched, then written via a
        # same-directory temp file + os.replace so a reader never sees a
        # partial report.
        write_json_atomic(out, payload, overwrite=True)
    except ValueError as e:
        print(f"error: score payload is not strict JSON ({e}); nothing written",
              file=sys.stderr)
        return 1
    print(f"\nJSON written to {out}")
    if failures:
        print(f"error: {len(failures)} of {len(pkls)} input(s) failed to score: "
              f"{sorted(failures)}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
