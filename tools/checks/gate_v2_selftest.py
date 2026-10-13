#!/usr/bin/env python3
"""Gate v2 anti-gaming self-test (T2; hardened 2026-08-04 post-V6 audit).

Verifies the properties the hardened gate v2 is asked to demonstrate:

  1. A frozen-pose pkl (one honest frame repeated for the whole clip) FAILS
     gate v2, with a reason mentioning "no contact dynamics" (the
     ``not_degenerate`` precondition in ``metrics.evaluate_gate_v2``).
  2. A frozen pose PLUS imperceptible Gaussian dither on root_pos/dof_pos
     (std sweep: 1e-6 [the exact V6 CONFIRMED-DEFECT-HIGH #2 case], 1e-4,
     1e-2 m/rad) still FAILS gate v2 -- this is the hardening this script
     was extended to verify: the pre-hardening ``static_frame_frac``-only
     precondition PASSED at 1e-6 and even 1e-4 (V6's finding); the new
     ``root_disp_median_m`` / ``dof_disp_median_rad`` floors close that
     hole. Reports the margin at each sigma and (via a bisection search)
     the actual crossover sigma where the hardened check would eventually
     start passing, so the margin is stated honestly rather than assumed.
  3. A block-shuffled pkl (root_pos + root_rot + dof_pos of one honest clip
     cut into fixed-size blocks and the block ORDER permuted with a fixed
     seed -- same permutation applied to all three arrays, so each frame
     stays internally consistent, only the timeline is destroyed) is run at
     BOTH block=5 and block=10, per the V6 finding that block-shuffle
     rejection via Non-Skating alone is only robust at block<=5: block=10
     is reported HONESTLY (pass or fail, and which criteria), not tuned to
     force a fail.
  4. An honest, UNGROUNDED (unpatched) clip's root-accel-p99/max are NOT
     spuriously flagged by the new bounds (GATE_V2_ROOT_ACCEL_P99_MAX_MPS2 /
     GATE_V2_ROOT_ACCEL_MAX_MPS2) -- checked directly from
     ``root_motion_metrics`` since these statistics are contact-policy-
     independent (root_pos-only), so this is true regardless of what
     contact policy would be "correct" to grade an ungrounded pkl under.

2026-10-10: gate v2 gained a ``contact_coverage`` precondition, a
``joint_position_limits`` criterion and fail-closed NaN / paired-delta
handling. These only ADD failure modes, so every expectation above is
unchanged (the frozen/dithered/shuffled cases still fail for the reasons
asserted; the honest-unpatched root-accel sanity check does not involve
them). Synthesized pkls now carry ``root_quat_order: "xyzw"``; pass
``--quat-order xyzw`` when the honest/unpatched inputs are keyless legacy
GMR-native pkls.

Does not modify GMR, eval/, or any file under WS/out/ -- synthesized pkls
are written to this script's own --outdir (scratchpad by default) and the
honest source pkl is only read.

Usage::

    VENVPY score_motion.py's venv python
    tools/checks/gate_v2_selftest.py --robot unitree_g1 \\
        --honest-pkl $WS/out/g1_baseline/patched/dance1_subject2.pkl \\
        --unpatched-pkl $WS/out/g1_baseline/unpatched/dance1_subject2.pkl
"""

from __future__ import annotations

import argparse
import pickle
import sys
import tempfile
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "eval"))
import metrics  # noqa: E402
from mocap_g1.gmr_pkl import declare_gmr_root_quat_order, root_rot_as  # noqa: E402


def load_pkl(path: str | Path, legacy_quat_order: str | None = None) -> dict:
    """Load an honest pkl with root_rot normalized to xyzw and the order key
    stamped, so every synthesized derivative inherits an explicit order."""
    with open(path, "rb") as f:
        motion = pickle.load(f)
    motion = dict(motion)
    motion["root_rot"] = root_rot_as(motion, "xyzw", legacy_order=legacy_quat_order,
                                     source=path)
    return declare_gmr_root_quat_order(motion)


def save_pkl(path: str | Path, motion: dict) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "wb") as f:
        pickle.dump(motion, f)


def make_frozen_pose(honest: dict, frame: int = 0) -> dict:
    """Repeat ``honest``'s frame ``frame`` for the whole clip length."""
    T = len(honest["dof_pos"])
    return declare_gmr_root_quat_order({
        "fps": honest["fps"],
        "root_pos": np.tile(np.asarray(honest["root_pos"])[frame], (T, 1)),
        "root_rot": np.tile(np.asarray(honest["root_rot"])[frame], (T, 1)),
        "dof_pos": np.tile(np.asarray(honest["dof_pos"])[frame], (T, 1)),
    })


def make_dithered_frozen_pose(honest: dict, sigma: float, frame: int = 0,
                              seed: int = 7) -> dict:
    """Frozen pose (see ``make_frozen_pose``) plus iid N(0, sigma^2) dither
    on root_pos and dof_pos (root_rot left exactly frozen, matching V6's
    reported construction -- "N(0, 1e-6 m) Gaussian dither on
    root_pos/dof_pos"). dof_pos is in radians, root_pos in metres; sigma is
    applied in each array's own units, unnormalized, exactly as V6 did.
    """
    frozen = make_frozen_pose(honest, frame)
    rng = np.random.default_rng(seed)
    frozen["root_pos"] = frozen["root_pos"] + rng.normal(0, sigma, frozen["root_pos"].shape)
    frozen["dof_pos"] = frozen["dof_pos"] + rng.normal(0, sigma, frozen["dof_pos"].shape)
    return frozen


def make_block_shuffled(honest: dict, block: int = 3, seed: int = 42) -> dict:
    """Cut into fixed-size blocks of ``block`` frames and permute block
    ORDER (fixed seed, reproducible). The same permutation is applied to
    root_pos, root_rot AND dof_pos so each individual frame stays an
    internally-consistent (root, pose) pair -- only inter-block continuity
    is destroyed, which is exactly what makes this "block-shuffled garbage"
    rather than "per-frame white noise": every single frame, taken alone,
    is a real pose the honest clip actually visited.
    """
    root_pos = np.asarray(honest["root_pos"])
    root_rot = np.asarray(honest["root_rot"])
    dof_pos = np.asarray(honest["dof_pos"])
    T = len(dof_pos)
    n_blocks = T // block
    rest = T - n_blocks * block
    rng = np.random.default_rng(seed)
    order = rng.permutation(n_blocks)

    def shuffle(arr):
        head = arr[: n_blocks * block].reshape(n_blocks, block, *arr.shape[1:])
        head = head[order].reshape(n_blocks * block, *arr.shape[1:])
        if rest:
            return np.concatenate([head, arr[n_blocks * block :]], axis=0)
        return head

    return declare_gmr_root_quat_order({
        "fps": honest["fps"],
        "root_pos": shuffle(root_pos),
        "root_rot": shuffle(root_rot),
        "dof_pos": shuffle(dof_pos),
    })


def score(rm: metrics.RobotModel, pkl_path: Path, *, contact_policy: str,
          quat_order: str | None = None) -> dict:
    return metrics.score_motion_file(
        pkl_path, rm, contact_policy=contact_policy, grounded_reference=False,
        quat_order=quat_order,
    )


def find_dither_crossover(rm: metrics.RobotModel, honest: dict, outdir: Path,
                          lo: float = 1e-4, hi: float = 1e-2, iters: int = 16) -> float:
    """Bisect for the smallest dither sigma at which ``not_degenerate`` FLIPS
    to passing (i.e. the gaming hole re-opens), reporting it honestly rather
    than asserting a fixed number. Assumes ``lo`` fails and ``hi`` passes;
    returns ``float('nan')`` if that assumption doesn't hold (both same
    verdict) so the caller can report that instead of a bogus number.
    """
    def not_degenerate_at(sigma: float) -> bool:
        motion = make_dithered_frozen_pose(honest, sigma, seed=123)
        p = outdir / "crossover_probe.pkl"
        save_pkl(p, motion)
        res = score(rm, p, contact_policy="anchored")
        gate = metrics.evaluate_gate_v2(res)
        return gate["checks"]["not_degenerate"]["pass"]

    lo_pass, hi_pass = not_degenerate_at(lo), not_degenerate_at(hi)
    if lo_pass == hi_pass:
        return float("nan")
    for _ in range(iters):
        mid = (lo * hi) ** 0.5  # geometric bisection, spans decades
        if not_degenerate_at(mid):
            hi = mid
        else:
            lo = mid
    return hi


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", default="unitree_g1")
    ap.add_argument("--honest-pkl", required=True,
                    help="one HONEST, GROUNDED (anchored-appropriate) clip pkl "
                         "to synthesize the frozen-pose, dithered-frozen, and "
                         "block-shuffled cases from")
    ap.add_argument("--unpatched-pkl", required=True,
                    help="one HONEST, UNGROUNDED (unpatched) clip pkl, to confirm "
                         "the root-accel-p99/max bounds are not spuriously tripped")
    ap.add_argument("--block", type=int, default=3,
                    help="block size for a preliminary shuffle case, kept for "
                         "back-compat with earlier runs; the required battery "
                         "(block=5, block=10) is run unconditionally below "
                         "regardless of this flag")
    ap.add_argument("--dither-sigmas", type=float, nargs="+",
                    default=[1e-6, 1e-4, 1e-2],
                    help="Gaussian dither std values (same units as "
                         "root_pos/dof_pos, unnormalized) to test against "
                         "the hardened not_degenerate check")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--quat-order", choices=["xyzw", "wxyz"], default=None,
                    help="LEGACY override for honest/unpatched pkls WITHOUT a "
                         "root_quat_order key (the GMR-native baseline suites are "
                         "xyzw); default requires the key")
    ap.add_argument("--skip-crossover-search", action="store_true",
                    help="skip the bisection search for the dither crossover "
                         "sigma (saves ~16 extra scoring passes)")
    ap.add_argument(
        "--outdir",
        default=str(Path(tempfile.gettempdir()) / "mocap-g1-gate-v2-selftest"),
    )
    args = ap.parse_args()

    outdir = Path(args.outdir)
    rm = metrics.build_robot(args.robot)

    honest = load_pkl(args.honest_pkl, args.quat_order)
    frozen = make_frozen_pose(honest)

    ok = True

    # -----------------------------------------------------------------
    print("=" * 78)
    print(f"[1] FROZEN POSE  ({args.honest_pkl} frame 0, repeated)")
    frozen_path = outdir / "frozen_pose.pkl"
    save_pkl(frozen_path, frozen)
    res_frozen = score(rm, frozen_path, contact_policy="anchored")
    gate_frozen = metrics.evaluate_gate_v2(res_frozen)
    print(f"  static_frame_frac = {res_frozen['root_motion']['static_frame_frac']}  "
          f"root_disp_median_m = {res_frozen['root_motion']['root_disp_median_m']}  "
          f"dof_disp_median_rad = {res_frozen['root_motion']['dof_disp_median_rad']}")
    print(f"  gate v2 pass = {gate_frozen['pass']}")
    for r in gate_frozen["reasons"]:
        print(f"    reason: {r}")
    cond1 = (not gate_frozen["pass"]) and any(
        "no contact dynamics" in r for r in gate_frozen["reasons"])
    print(f"  [{'OK' if cond1 else 'FAIL'}] expected: FAIL gate v2 with a "
          f"'no contact dynamics' reason")
    ok &= cond1

    # -----------------------------------------------------------------
    print("=" * 78)
    print(f"[2] FROZEN + DITHER sweep (sigmas={args.dither_sigmas}, V6 CONFIRMED-DEFECT-HIGH #2)")
    for sigma in args.dither_sigmas:
        dithered = make_dithered_frozen_pose(honest, sigma)
        p = outdir / f"dithered_sigma_{sigma:.0e}.pkl"
        save_pkl(p, dithered)
        res_d = score(rm, p, contact_policy="anchored")
        gate_d = metrics.evaluate_gate_v2(res_d)
        rd = res_d["root_motion"]
        nd_pass = gate_d["checks"]["not_degenerate"]["pass"]
        print(f"  sigma={sigma:.0e}: static_frame_frac={rd['static_frame_frac']}  "
              f"root_disp_med={rd['root_disp_median_m']:.8f} (floor {metrics.GATE_V2_ROOT_DISP_MEDIAN_MIN_M})  "
              f"dof_disp_med={rd['dof_disp_median_rad']:.8f} (floor {metrics.GATE_V2_DOF_DISP_MEDIAN_MIN_RAD})  "
              f"gate_pass={gate_d['pass']}  not_degenerate_pass={nd_pass}")
        # The REQUIRED invariant is that the OVERALL gate still rejects the
        # pkl, not that not_degenerate specifically is what catches it: once
        # sigma is large enough that not_degenerate stops firing (see the
        # crossover search below), the dither is no longer "imperceptible"
        # and independent per-frame noise on a "frozen" pose instead reads
        # as high-frequency jitter that fails Non-Skating / root-accel
        # criteria instead -- a different, still-correct layer of defense.
        cond = not gate_d["pass"]
        failing = [n for n, c in gate_d["checks"].items() if not c["pass"]]
        tag = "not_degenerate" if not nd_pass else f"other criteria {failing}"
        print(f"    [{'OK' if cond else 'FAIL'}] expected: gate v2 overall FAILS at this "
              f"sigma (caught by: {tag})")
        ok &= cond

    if not args.skip_crossover_search:
        crossover = find_dither_crossover(rm, honest, outdir)
        if crossover != crossover:  # NaN
            print("  crossover search: lo/hi bracket did not straddle a flip in "
                  "[1e-4, 1e-2] -- not_degenerate holds across that whole range "
                  "(consistent with the 1e-6/1e-4/1e-2 sweep above all failing)")
        else:
            print(f"  crossover search (bisection, 1e-4..1e-2): not_degenerate first "
                  f"starts PASSING near sigma~{crossover:.2e} -- "
                  f"{crossover/max(args.dither_sigmas):.1f}x above the largest swept "
                  f"sigma ({max(args.dither_sigmas):.0e}), "
                  f"{crossover/1e-6:.0f}x above the V6-reported 1um case")

    # -----------------------------------------------------------------
    for block in (5, 10):
        print("=" * 78)
        print(f"[3] BLOCK-SHUFFLED (block={block}, seed={args.seed})")
        shuffled = make_block_shuffled(honest, block=block, seed=args.seed)
        shuffled_path = outdir / f"block_shuffled_b{block}.pkl"
        save_pkl(shuffled_path, shuffled)
        res_shuf = score(rm, shuffled_path, contact_policy="anchored")
        gate_shuf = metrics.evaluate_gate_v2(res_shuf)
        print(f"  static_frame_frac = {res_shuf['root_motion']['static_frame_frac']}")
        print(f"  foot_contact: NF={res_shuf['foot_contact']['non_floating_pct']} "
              f"NP={res_shuf['foot_contact']['non_penetration_pct']} "
              f"NS={res_shuf['foot_contact']['non_skating_pct']} "
              f"contact_frames={res_shuf['foot_contact']['contact_frames']}")
        print(f"  root accel p99 mag = {res_shuf['root_motion']['accel']['p99']['mag']} m/s^2 "
              f"(max = {res_shuf['root_motion']['accel']['max']['mag']})")
        print(f"  gate v2 pass = {gate_shuf['pass']}")
        failing = [name for name, c in gate_shuf["checks"].items() if not c["pass"]]
        print(f"  failing checks: {failing}")
        for r in gate_shuf["reasons"]:
            print(f"    reason: {r}")
        if block == 5:
            ns_fails = not gate_shuf["checks"]["non_skating"]["pass"]
            cond_b5 = (not gate_shuf["pass"]) and ns_fails
            print(f"  [{'OK' if cond_b5 else 'FAIL'}] expected (block=5, robust range per V6): "
                  f"FAIL gate v2, failing non_skating")
            ok &= cond_b5
        else:
            # block=10: V6 found Non-Skating alone stops reliably rejecting here.
            # Report honestly -- do NOT force a fail by silently tightening
            # other bounds; this is a documented known limitation, not a bug
            # to hide.
            if gate_shuf["pass"]:
                print("  [KNOWN LIMITATION, as documented] block=10 PASSES gate v2 -- "
                      "matches V6's finding that Non-Skating rejection is not robust "
                      "at block>=10 and no other criterion here happens to catch this "
                      "particular shuffle either. This is recorded in "
                      "evaluate_gate_v2's docstring 'known limitations' section, not "
                      "silently patched around.")
            else:
                print(f"  block=10 FAILS gate v2 on this clip/seed (checks: {failing}) -- "
                      f"note V6 found this is NOT universal across clips/seeds at "
                      f"block>=10; do not generalize this one result.")

    # -----------------------------------------------------------------
    print("=" * 78)
    print(f"[4] HONEST UNPATCHED CLIP root-accel bound sanity  ({args.unpatched_pkl})")
    motion_up = metrics.load_gmr_pkl(args.unpatched_pkl, quat_order=args.quat_order)
    rmo_up = metrics.root_motion_metrics(motion_up)
    accel_p99 = rmo_up["accel"]["p99"]["mag"]
    accel_max = rmo_up["accel"]["max"]["mag"]
    n_over = rmo_up["frames_over_thresh"]["count"]
    p99_bound = metrics.GATE_V2_ROOT_ACCEL_P99_MAX_MPS2
    max_bound = metrics.GATE_V2_ROOT_ACCEL_MAX_MPS2
    frames_bound = metrics.GATE_V2_ROOT_ACCEL_MAX_FRAMES_OVER
    cond4a = accel_p99 <= p99_bound
    cond4b = accel_max <= max_bound
    cond4c = n_over <= frames_bound
    print(f"  root accel p99 mag = {accel_p99} m/s^2  (bound = {p99_bound})  [{'OK' if cond4a else 'FAIL'}]")
    print(f"  root accel max mag = {accel_max} m/s^2  (bound = {max_bound})  [{'OK' if cond4b else 'FAIL'}]")
    print(f"  frames_over_60     = {n_over}  (bound = {frames_bound})  [{'OK' if cond4c else 'FAIL'}]")
    print(f"  [{'OK' if (cond4a and cond4b and cond4c) else 'FAIL'}] expected: NOT spuriously failed on any of the three")
    ok &= cond4a & cond4b & cond4c
    # Also show the full gate for context (expected to fail on the
    # contact_policy precondition since this clip is ungrounded/not
    # anchored-appropriate -- NOT a root-accel false positive).
    res_up_anchored = score(rm, Path(args.unpatched_pkl), contact_policy="anchored",
                            quat_order=args.quat_order)
    gate_up = metrics.evaluate_gate_v2(res_up_anchored)
    print(f"  (context) full gate v2 under --contact-policy anchored on this "
          f"same ungrounded clip: pass={gate_up['pass']}; "
          f"root_accel checks pass="
          f"{gate_up['checks']['root_accel_p99_mag']['pass']}/"
          f"{gate_up['checks']['root_accel_max_mag']['pass']}/"
          f"{gate_up['checks']['root_accel_frames_over_thresh']['pass']} "
          f"(other failures here are the EXPECTED ungrounded/anchored mismatch, "
          f"not these criteria)")

    print("=" * 78)
    print(f"SELF-TEST {'PASSED' if ok else 'FAILED'}")
    print("(block=10 is graded as a documented known limitation, not a hard "
          "pass/fail requirement -- see [3] above; it does not affect this verdict)")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
