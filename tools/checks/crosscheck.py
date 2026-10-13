#!/usr/bin/env python3
"""Compare two G1 motion pickles and report per-channel numeric drift."""

from __future__ import annotations

import argparse
import pickle
from pathlib import Path

import numpy as np


def load(path: Path) -> dict:
    with path.open("rb") as handle:
        return pickle.load(handle)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference", type=Path)
    parser.add_argument("candidate", type=Path)
    parser.add_argument("--atol", type=float, default=0.0)
    args = parser.parse_args()

    reference = load(args.reference)
    candidate = load(args.candidate)
    print("frames ref/new:", reference["dof_pos"].shape[0], candidate["dof_pos"].shape[0])
    print("dof dims ref/new:", reference["dof_pos"].shape[1], candidate["dof_pos"].shape[1])
    print("fps ref/new:", reference["fps"], candidate["fps"])
    passed = reference["fps"] == candidate["fps"]
    # root_rot values are only comparable when both files store the same
    # quaternion order (see mocap_g1/gmr_pkl.py); a keyless legacy file is
    # reported as "undeclared" rather than assumed.
    orders = [pkl.get("root_quat_order", "undeclared") for pkl in (reference, candidate)]
    print("root_quat_order ref/new:", *orders)
    if orders[0] != orders[1]:
        print("root_rot: ORDER MISMATCH -- quaternion values are not comparable")
        passed = False

    for key in ("root_pos", "root_rot", "dof_pos"):
        a = np.asarray(reference[key])
        b = np.asarray(candidate[key])
        if a.shape != b.shape:
            print(f"{key}: SHAPE MISMATCH {a.shape} vs {b.shape}")
            passed = False
            continue
        difference = np.abs(a - b)
        maximum = float(difference.max()) if difference.size else 0.0
        print(
            f"{key}: max|diff|={maximum:.6e} mean|diff|={difference.mean():.6e} "
            f"ref_mean={a.mean():.4f} new_mean={b.mean():.4f} "
            f"ref_std={a.std():.4f} new_std={b.std():.4f}"
        )
        passed &= maximum <= args.atol

    print("PASS" if passed else "FAIL")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
