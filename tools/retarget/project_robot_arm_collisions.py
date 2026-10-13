#!/usr/bin/env python3
"""Constrain humanoid arm penetrations while preserving the GMR pose.

The input remains the Xsens/GMR reference.  Only arm joints are optimized,
and the objective penalizes joint, Cartesian-position, Cartesian-orientation,
and temporal drift from that reference while enforcing signed proxy distance:

* distal arm or hand against torso or legs.

Deliberate hand-to-hand motion is retained.  The supported MJCFs use broad
hand proxies that are not accurate enough to rewrite crossed or clasped hands.

The command fails closed if any declared proxy pair misses the requested
clearance, a joint limit is exceeded, the tracking-drift ceiling is exceeded,
or any root, torso, or leg value changes.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
from pathlib import Path
import pickle
import sys
from typing import Any, Iterable

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from mocap_g1.arm_collision_profiles import (  # noqa: E402,F401
    G1_TARGET_BODIES,
    ROBOT_PROFILES,
    T1_TARGET_BODIES,
    ArmProfileError,
    GeometryPair,
    RobotArmProfile,
    geometry_pairs,
)
from mocap_g1.gmr_pkl import declare_gmr_root_quat_order, wxyz_to_xyzw  # noqa: E402
from mocap_g1.io_utils import sha256_file  # noqa: E402
from tools.retarget.render_robot_motion import (  # noqa: E402
    RenderError,
    validate_motion_payload,
)


class ProjectionError(RuntimeError):
    """Raised when bounded arm projection cannot produce an auditable result."""


def _load_payload(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise ProjectionError(f"motion pickle does not exist: {path}")
    try:
        with path.open("rb") as handle:
            payload = pickle.load(handle)
    except Exception as exc:
        raise ProjectionError(f"cannot read motion pickle {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise ProjectionError("motion pickle must contain a dictionary")
    try:
        validate_motion_payload(payload)
    except RenderError as exc:
        raise ProjectionError(str(exc)) from exc
    return payload


def _load_model(gmr_root: Path, robot: str) -> tuple[Any, Any, Path]:
    root = gmr_root.expanduser().resolve()
    if not root.is_dir():
        raise ProjectionError(f"GMR root does not exist: {root}")
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    try:
        import mujoco
        from general_motion_retargeting import ROBOT_XML_DICT
    except ImportError as exc:
        raise ProjectionError(f"cannot load GMR/MuJoCo runtime: {exc}") from exc
    if robot not in ROBOT_PROFILES:
        known = ", ".join(sorted(ROBOT_PROFILES))
        raise ProjectionError(f"no arm collision profile for {robot!r}; available: {known}")
    if robot not in ROBOT_XML_DICT:
        raise ProjectionError(f"GMR does not register robot {robot!r}")
    model_path = Path(ROBOT_XML_DICT[robot]).resolve()
    try:
        model = mujoco.MjModel.from_xml_path(str(model_path))
    except Exception as exc:
        raise ProjectionError(f"cannot load robot MJCF {model_path}: {exc}") from exc
    return mujoco, model, model_path


def _joint_qpos_addresses(mujoco: Any, model: Any, names: Iterable[str]) -> np.ndarray:
    addresses: list[int] = []
    for name in names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise ProjectionError(f"robot MJCF lacks arm joint {name!r}")
        if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
            raise ProjectionError(f"arm joint {name!r} is not a hinge")
        addresses.append(int(model.jnt_qposadr[joint_id]))
    return np.asarray(addresses, dtype=int)


def _geometry_pairs(mujoco: Any, model: Any, profile: RobotArmProfile) -> tuple[GeometryPair, ...]:
    """Enumerate enabled collision proxies used by the declared arm gate."""

    try:
        return geometry_pairs(mujoco, model, profile)
    except ArmProfileError as exc:
        raise ProjectionError(str(exc)) from exc


def _targeted_distances(
    mujoco: Any,
    model: Any,
    data: Any,
    qpos: np.ndarray,
    geometry_pairs: tuple[GeometryPair, ...],
    clearance_m: float,
) -> list[tuple[str, str, float, str]]:
    """Return declared proxy pairs whose signed distance is below clearance."""

    data.qpos[:] = qpos
    mujoco.mj_forward(model, data)
    failures: list[tuple[str, str, float, str]] = []
    fromto = np.zeros(6, dtype=np.float64)
    distance_limit = max(0.05, clearance_m + 1e-6)
    for pair in geometry_pairs:
        distance = float(
            mujoco.mj_geomDistance(
                model,
                data,
                pair.geom1,
                pair.geom2,
                distance_limit,
                fromto,
            )
        )
        if distance < clearance_m:
            failures.append((pair.body1, pair.body2, distance, pair.category))
    return failures


def _distance_vector(
    mujoco: Any,
    model: Any,
    data: Any,
    qpos: np.ndarray,
    geometry_pairs: tuple[GeometryPair, ...],
) -> np.ndarray:
    """Return signed distance for every declared collision-proxy pair."""

    data.qpos[:] = qpos
    mujoco.mj_forward(model, data)
    fromto = np.zeros(6, dtype=np.float64)
    return np.asarray(
        [
            mujoco.mj_geomDistance(
                model, data, pair.geom1, pair.geom2, 0.05, fromto
            )
            for pair in geometry_pairs
        ],
        dtype=np.float64,
    )


def _audit_contacts(
    mujoco: Any,
    model: Any,
    qpos: np.ndarray,
    geometry_pairs: tuple[GeometryPair, ...],
    clearance_m: float,
) -> dict[str, Any]:
    data = mujoco.MjData(model)
    pair_stats: dict[tuple[str, str, str], dict[str, Any]] = {}
    collision_frames = 0
    distal_body_frames = 0
    arm_arm_frames = 0
    minimum_distance = np.inf
    for frame_index, row in enumerate(qpos):
        distances = _distance_vector(mujoco, model, data, row, geometry_pairs)
        minimum_distance = min(minimum_distance, float(np.min(distances)))
        contacts = [
            (pair.body1, pair.body2, float(distance), pair.category)
            for pair, distance in zip(geometry_pairs, distances, strict=True)
            if distance < clearance_m
        ]
        if not contacts:
            continue
        collision_frames += 1
        categories = {contact[3] for contact in contacts}
        distal_body_frames += "distal_to_body" in categories
        arm_arm_frames += "arm_arm" in categories
        minimum_distance = min(minimum_distance, *(contact[2] for contact in contacts))
        seen: set[tuple[str, str, str]] = set()
        for body1, body2, distance, category in contacts:
            key = (body1, body2, category)
            stats = pair_stats.setdefault(
                key,
                {
                    "body1": body1,
                    "body2": body2,
                    "category": category,
                    "contact_count": 0,
                    "frame_count": 0,
                    "first_frame": frame_index,
                    "minimum_distance_m": distance,
                },
            )
            stats["contact_count"] += 1
            stats["minimum_distance_m"] = min(stats["minimum_distance_m"], distance)
            if key not in seen:
                stats["frame_count"] += 1
                seen.add(key)
    pairs = sorted(
        pair_stats.values(),
        key=lambda value: (-value["frame_count"], value["body1"], value["body2"]),
    )
    return {
        "frames": int(qpos.shape[0]),
        "collision_frames": collision_frames,
        "distal_to_body_frames": distal_body_frames,
        "arm_arm_frames": arm_arm_frames,
        "minimum_distance_m": float(minimum_distance),
        "pairs": pairs,
    }


def _project(
    mujoco: Any,
    model: Any,
    qpos: np.ndarray,
    profile: RobotArmProfile,
    left_addresses: np.ndarray,
    right_addresses: np.ndarray,
    geometry_pairs: tuple[GeometryPair, ...],
    *,
    clearance_m: float,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Run arm-only nonlinear constrained optimization on offending frames."""

    try:
        from scipy.optimize import minimize
    except ImportError as exc:
        raise ProjectionError(f"constrained arm projection requires SciPy: {exc}") from exc

    arm_addresses = np.concatenate((left_addresses, right_addresses))
    joint_ids: list[int] = []
    for address in arm_addresses:
        matches = np.flatnonzero(model.jnt_qposadr == address)
        if matches.size != 1:
            raise ProjectionError(f"cannot resolve unique joint for qpos address {address}")
        joint_ids.append(int(matches[0]))
    bounds = [tuple(float(value) for value in model.jnt_range[joint_id]) for joint_id in joint_ids]
    joint_ranges = np.asarray([upper - lower for lower, upper in bounds], dtype=np.float64)
    if np.any(joint_ranges <= 0):
        raise ProjectionError("arm profile contains a joint with an invalid range")

    tracked_ids = np.asarray(
        [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) for name in profile.tracked_bodies],
        dtype=int,
    )
    if np.any(tracked_ids < 0):
        raise ProjectionError("robot MJCF lacks a configured arm tracking body")
    position_weights = np.asarray(profile.position_weights, dtype=np.float64)
    orientation_weights = np.asarray(profile.orientation_weights, dtype=np.float64)
    if not len(tracked_ids) == len(position_weights) == len(orientation_weights):
        raise ProjectionError("arm tracking bodies and objective weights differ in length")

    audit_data = mujoco.MjData(model)
    objective_data = mujoco.MjData(model)
    constraint_data = mujoco.MjData(model)
    projected = qpos.copy()
    attempted = 0
    optimizer_non_success = 0
    optimizer_iterations = 0
    feasible_seed_fallbacks = 0
    previous_correction = np.zeros(len(arm_addresses), dtype=np.float64)
    previous_was_projected = False
    solver_clearance = clearance_m + 1e-4

    for frame_index, reference in enumerate(qpos):
        violations = _targeted_distances(
            mujoco, model, audit_data, reference, geometry_pairs, clearance_m
        )
        if not violations:
            previous_correction.fill(0.0)
            previous_was_projected = False
            continue
        attempted += 1
        objective_data.qpos[:] = reference
        mujoco.mj_forward(model, objective_data)
        reference_positions = objective_data.xpos[tracked_ids].copy()
        reference_quaternions = objective_data.xquat[tracked_ids].copy()
        reference_arms = reference[arm_addresses].copy()

        def set_arms(data: Any, values: np.ndarray) -> None:
            data.qpos[:] = reference
            data.qpos[arm_addresses] = values
            mujoco.mj_forward(model, data)

        def objective(values: np.ndarray) -> float:
            set_arms(objective_data, values)
            position_error = np.linalg.norm(
                objective_data.xpos[tracked_ids] - reference_positions, axis=1
            ) / 0.02
            dots = np.abs(
                np.sum(
                    objective_data.xquat[tracked_ids] * reference_quaternions,
                    axis=1,
                )
            )
            orientation_error = (
                2.0 * np.arccos(np.clip(dots, 0.0, 1.0)) / np.deg2rad(10.0)
            )
            joint_error = (values - reference_arms) / joint_ranges
            temporal_error = (
                (values - reference_arms - previous_correction) / joint_ranges
            )
            temporal_weight = 0.05 if previous_was_projected else 0.0
            return float(
                np.dot(position_weights, position_error * position_error)
                + np.dot(orientation_weights, orientation_error * orientation_error)
                + 0.1 * np.dot(joint_error, joint_error)
                + temporal_weight * np.dot(temporal_error, temporal_error)
            )

        def constraint(values: np.ndarray) -> np.ndarray:
            candidate = reference.copy()
            candidate[arm_addresses] = values
            return (
                _distance_vector(
                    mujoco, model, constraint_data, candidate, geometry_pairs
                )
                - solver_clearance
            )

        initial_values = (
            reference_arms + previous_correction
            if previous_was_projected
            else reference_arms
        )
        initial_values = np.clip(
            initial_values,
            np.asarray([lower for lower, _upper in bounds]),
            np.asarray([upper for _lower, upper in bounds]),
        )
        result = minimize(
            objective,
            initial_values,
            method="SLSQP",
            bounds=bounds,
            constraints={"type": "ineq", "fun": constraint},
            options={"ftol": 1e-9, "maxiter": 600, "disp": False},
        )
        candidate = reference.copy()
        candidate[arm_addresses] = result.x
        verified_distances = _distance_vector(
            mujoco, model, audit_data, candidate, geometry_pairs
        )
        if not np.isfinite(result.x).all() or np.min(verified_distances) < clearance_m:
            neutral_arms = model.qpos0[arm_addresses]
            levels = np.linspace(0.0, 1.0, 51)
            squared_ranges = (
                (reference_arms[: len(left_addresses)] - neutral_arms[: len(left_addresses)]) ** 2,
                (reference_arms[len(left_addresses) :] - neutral_arms[len(left_addresses) :]) ** 2,
            )
            retreat_candidates = sorted(
                (
                    (
                        left,
                        right,
                        left * left * float(np.sum(squared_ranges[0]))
                        + right * right * float(np.sum(squared_ranges[1])),
                    )
                    for left in levels
                    for right in levels
                ),
                key=lambda value: (value[2], value[0] + value[1]),
            )
            feasible_seed = None
            for left, right, _cost in retreat_candidates:
                seed = reference_arms.copy()
                seed[: len(left_addresses)] = (
                    (1.0 - left) * reference_arms[: len(left_addresses)]
                    + left * neutral_arms[: len(left_addresses)]
                )
                seed[len(left_addresses) :] = (
                    (1.0 - right) * reference_arms[len(left_addresses) :]
                    + right * neutral_arms[len(left_addresses) :]
                )
                if np.min(constraint(seed)) >= 0.0:
                    feasible_seed = seed
                    break
            if feasible_seed is None:
                raise ProjectionError(
                    f"frame {frame_index}: no feasible neutral-retreat seed exists"
                )
            retry = minimize(
                objective,
                feasible_seed,
                method="SLSQP",
                bounds=bounds,
                constraints={"type": "ineq", "fun": constraint},
                options={"ftol": 1e-9, "maxiter": 600, "disp": False},
            )
            retry_candidate = reference.copy()
            retry_candidate[arm_addresses] = retry.x
            retry_distances = _distance_vector(
                mujoco, model, audit_data, retry_candidate, geometry_pairs
            )
            seed_candidate = reference.copy()
            seed_candidate[arm_addresses] = feasible_seed
            seed_distances = _distance_vector(
                mujoco, model, audit_data, seed_candidate, geometry_pairs
            )
            feasible_options = []
            if np.isfinite(result.x).all() and np.min(verified_distances) >= clearance_m:
                feasible_options.append((objective(result.x), result, candidate, verified_distances))
            if np.isfinite(retry.x).all() and np.min(retry_distances) >= clearance_m:
                feasible_options.append((objective(retry.x), retry, retry_candidate, retry_distances))
            if np.min(seed_distances) >= clearance_m:
                feasible_options.append((objective(feasible_seed), None, seed_candidate, seed_distances))
            if feasible_options:
                _cost, selected_result, candidate, verified_distances = min(
                    feasible_options, key=lambda option: option[0]
                )
                if selected_result is None:
                    feasible_seed_fallbacks += 1
                    result.x = feasible_seed
                    result.success = False
                else:
                    result = selected_result
        if not np.isfinite(result.x).all() or np.min(verified_distances) < clearance_m:
            raise ProjectionError(
                f"frame {frame_index}: constrained solve reached only "
                f"{float(np.min(verified_distances)) * 1000:.3f} mm clearance"
            )
        projected[frame_index] = candidate
        previous_correction = result.x - reference_arms
        previous_was_projected = True
        optimizer_non_success += not bool(result.success)
        optimizer_iterations += int(result.nit)

    return projected, {
        "attempted_frames": attempted,
        "optimizer_non_success_but_verified_frames": optimizer_non_success,
        "optimizer_iterations": optimizer_iterations,
        "feasible_seed_fallback_frames": feasible_seed_fallbacks,
        "solver_clearance_m": solver_clearance,
    }


def _tracking_drift(
    mujoco: Any,
    model: Any,
    reference: np.ndarray,
    projected: np.ndarray,
    tracked_bodies: tuple[str, ...],
) -> tuple[np.ndarray, np.ndarray]:
    """Return per-frame maximum tracked-body position and orientation drift."""

    body_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) for name in tracked_bodies]
    reference_data = mujoco.MjData(model)
    projected_data = mujoco.MjData(model)
    position = np.zeros(reference.shape[0], dtype=np.float64)
    orientation = np.zeros(reference.shape[0], dtype=np.float64)
    for frame_index, (source, result) in enumerate(zip(reference, projected, strict=True)):
        reference_data.qpos[:] = source
        projected_data.qpos[:] = result
        mujoco.mj_forward(model, reference_data)
        mujoco.mj_forward(model, projected_data)
        position[frame_index] = max(
            np.linalg.norm(reference_data.xpos[body_id] - projected_data.xpos[body_id])
            for body_id in body_ids
        )
        orientation[frame_index] = max(
            2.0
            * np.arccos(
                np.clip(
                    abs(
                        np.dot(
                            reference_data.xquat[body_id],
                            projected_data.xquat[body_id],
                        )
                    ),
                    0.0,
                    1.0,
                )
            )
            for body_id in body_ids
        )
    return position, np.rad2deg(orientation)


def _atomic_pickle(path: Path, value: Any, *, overwrite: bool) -> None:
    partial = path.with_name(f".{path.name}.partial")
    existing = [str(candidate) for candidate in (path, partial) if candidate.exists()]
    if existing and not overwrite:
        raise ProjectionError(
            "refusing to overwrite output; pass --overwrite or choose another path: "
            + ", ".join(existing)
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    if overwrite:
        partial.unlink(missing_ok=True)
    try:
        with partial.open("wb") as handle:
            pickle.dump(value, handle, protocol=pickle.HIGHEST_PROTOCOL)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(partial, path)
    except Exception:
        partial.unlink(missing_ok=True)
        raise


def _atomic_json(path: Path, value: Any, *, overwrite: bool) -> None:
    partial = path.with_name(f".{path.name}.partial")
    existing = [str(candidate) for candidate in (path, partial) if candidate.exists()]
    if existing and not overwrite:
        raise ProjectionError(
            "refusing to overwrite report; pass --overwrite or choose another path: "
            + ", ".join(existing)
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    if overwrite:
        partial.unlink(missing_ok=True)
    try:
        with partial.open("w", encoding="utf-8") as handle:
            json.dump(value, handle, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(partial, path)
    except Exception:
        partial.unlink(missing_ok=True)
        raise


def run(args: argparse.Namespace) -> dict[str, Any]:
    motion_path = args.motion.expanduser().resolve()
    output_path = args.output.expanduser().resolve()
    report_path = args.report.expanduser().resolve()
    for path, label in ((output_path, "output"), (report_path, "report")):
        partial = path.with_name(f".{path.name}.partial")
        existing = [str(candidate) for candidate in (path, partial) if candidate.exists()]
        if existing and not args.overwrite:
            raise ProjectionError(
                f"refusing to overwrite {label}; pass --overwrite or choose another path: "
                + ", ".join(existing)
            )
    if output_path == motion_path:
        raise ProjectionError("output must differ from the immutable input motion")
    if output_path == report_path or motion_path == report_path:
        raise ProjectionError("motion input, motion output, and JSON report must be different files")
    if args.clearance_m < 0 or not np.isfinite(args.clearance_m):
        raise ProjectionError("clearance must be non-negative and finite")
    if args.maximum_position_drift_m <= 0 or not np.isfinite(args.maximum_position_drift_m):
        raise ProjectionError("maximum position drift must be positive and finite")
    if (
        args.maximum_orientation_drift_deg <= 0
        or not np.isfinite(args.maximum_orientation_drift_deg)
    ):
        raise ProjectionError("maximum orientation drift must be positive and finite")

    payload = _load_payload(motion_path)
    motion = validate_motion_payload(payload)
    mujoco, model, model_path = _load_model(args.gmr_root, args.robot)
    if motion.dof_pos.shape[1] != model.nq - 7:
        raise ProjectionError(
            f"motion has {motion.dof_pos.shape[1]} joints but {args.robot!r} expects {model.nq - 7}"
        )
    metadata = payload.get("metadata")
    if isinstance(metadata, dict) and metadata.get("robot") not in (None, args.robot):
        raise ProjectionError(
            f"motion metadata declares robot {metadata.get('robot')!r}, not {args.robot!r}"
        )

    profile = ROBOT_PROFILES[args.robot]
    geometry_pairs = _geometry_pairs(mujoco, model, profile)
    left_addresses = _joint_qpos_addresses(mujoco, model, profile.left_joint_names)
    right_addresses = _joint_qpos_addresses(mujoco, model, profile.right_joint_names)
    root_wxyz = motion.root_rot_wxyz
    qpos = np.ascontiguousarray(
        np.concatenate((motion.root_pos, root_wxyz, motion.dof_pos), axis=1)
    )
    before = _audit_contacts(
        mujoco, model, qpos, geometry_pairs, args.clearance_m
    )
    projected, solver_summary = _project(
        mujoco,
        model,
        qpos,
        profile,
        left_addresses,
        right_addresses,
        geometry_pairs,
        clearance_m=args.clearance_m,
    )
    after = _audit_contacts(
        mujoco, model, projected, geometry_pairs, args.clearance_m
    )
    if after["collision_frames"] != 0:
        raise ProjectionError("targeted collision audit did not reach zero frames")

    all_arm_addresses = np.concatenate((left_addresses, right_addresses))
    arm_lower = []
    arm_upper = []
    for address in all_arm_addresses:
        joint_ids = np.flatnonzero(model.jnt_qposadr == address)
        if joint_ids.size != 1:
            raise ProjectionError(f"cannot resolve arm limit for qpos address {address}")
        lower, upper = model.jnt_range[int(joint_ids[0])]
        arm_lower.append(float(lower))
        arm_upper.append(float(upper))
    arm_lower_array = np.asarray(arm_lower)
    arm_upper_array = np.asarray(arm_upper)
    limit_violation = max(
        float(np.max(arm_lower_array - projected[:, all_arm_addresses])),
        float(np.max(projected[:, all_arm_addresses] - arm_upper_array)),
        0.0,
    )
    if limit_violation > 1e-6:
        raise ProjectionError(
            f"projected arm exceeds an MJCF joint limit by {limit_violation:.6g} rad"
        )
    non_arm_addresses = np.asarray(
        [address for address in range(model.nq) if address not in set(all_arm_addresses)],
        dtype=int,
    )
    if not np.array_equal(projected[:, non_arm_addresses], qpos[:, non_arm_addresses]):
        raise ProjectionError("arm projection changed root, torso, or leg qpos")
    position_drift, orientation_drift_deg = _tracking_drift(
        mujoco, model, qpos, projected, profile.tracked_bodies
    )
    if float(np.max(position_drift)) > args.maximum_position_drift_m:
        raise ProjectionError(
            "tracking position drift exceeds ceiling: "
            f"{float(np.max(position_drift)) * 1000:.2f} mm > "
            f"{args.maximum_position_drift_m * 1000:.2f} mm"
        )
    if float(np.max(orientation_drift_deg)) > args.maximum_orientation_drift_deg:
        raise ProjectionError(
            "tracking orientation drift exceeds ceiling: "
            f"{float(np.max(orientation_drift_deg)):.2f} deg > "
            f"{args.maximum_orientation_drift_deg:.2f} deg"
        )
    arm_delta = np.abs(projected[:, all_arm_addresses] - qpos[:, all_arm_addresses])
    nonzero_delta_deg = np.rad2deg(arm_delta[arm_delta > 0])
    original_velocity = np.max(
        np.abs(np.diff(qpos[:, all_arm_addresses], axis=0)), axis=1
    ) * motion.fps
    projected_velocity = np.max(
        np.abs(np.diff(projected[:, all_arm_addresses], axis=0)), axis=1
    ) * motion.fps
    original_acceleration = np.max(
        np.abs(np.diff(qpos[:, all_arm_addresses], n=2, axis=0)), axis=1
    ) * motion.fps * motion.fps
    projected_acceleration = np.max(
        np.abs(np.diff(projected[:, all_arm_addresses], n=2, axis=0)), axis=1
    ) * motion.fps * motion.fps
    manual_review = np.flatnonzero(
        (position_drift > 0.02) | (orientation_drift_deg > 10.0)
    )
    source_sha256 = sha256_file(motion_path)
    projection_summary = {
        "method": "arm_only_slsqp_proxy_clearance_v1",
        "robot": args.robot,
        "source_motion_path": str(motion_path),
        "source_motion_sha256": source_sha256,
        "model_path": str(model_path),
        "model_sha256": sha256_file(model_path),
        "required_proxy_clearance_m": float(args.clearance_m),
        "declared_geometry_pairs": len(geometry_pairs),
        "modified_frames": int(np.count_nonzero(np.any(arm_delta > 0, axis=1))),
        "solver": solver_summary,
        "arm_joint_delta_degrees": {
            "median_nonzero": float(np.median(nonzero_delta_deg)) if nonzero_delta_deg.size else 0.0,
            "p95_nonzero": float(np.percentile(nonzero_delta_deg, 95)) if nonzero_delta_deg.size else 0.0,
            "maximum": float(np.max(nonzero_delta_deg)) if nonzero_delta_deg.size else 0.0,
        },
        "maximum_arm_joint_velocity_rad_s": {
            "input": float(np.max(original_velocity)) if original_velocity.size else 0.0,
            "output": float(np.max(projected_velocity)) if projected_velocity.size else 0.0,
            "input_p99": float(np.percentile(original_velocity, 99)) if original_velocity.size else 0.0,
            "output_p99": float(np.percentile(projected_velocity, 99)) if projected_velocity.size else 0.0,
        },
        "maximum_arm_joint_acceleration_rad_s2": {
            "input": float(np.max(original_acceleration)) if original_acceleration.size else 0.0,
            "output": float(np.max(projected_acceleration)) if projected_acceleration.size else 0.0,
            "input_p99": float(np.percentile(original_acceleration, 99)) if original_acceleration.size else 0.0,
            "output_p99": float(np.percentile(projected_acceleration, 99)) if projected_acceleration.size else 0.0,
        },
        "tracked_body_drift": {
            "position_m": {
                "p95": float(np.percentile(position_drift, 95)),
                "maximum": float(np.max(position_drift)),
                "hard_ceiling": float(args.maximum_position_drift_m),
            },
            "orientation_degrees": {
                "p95": float(np.percentile(orientation_drift_deg, 95)),
                "maximum": float(np.max(orientation_drift_deg)),
                "hard_ceiling": float(args.maximum_orientation_drift_deg),
            },
            "manual_review_thresholds": {"position_m": 0.02, "orientation_degrees": 10.0},
            "manual_review_frame_count": int(manual_review.size),
            "manual_review_frames": manual_review.tolist(),
        },
        "root_torso_leg_qpos_preserved_bit_exact": True,
        "maximum_arm_joint_limit_violation_rad": limit_violation,
        "before": before,
        "after": after,
    }

    result = copy.deepcopy(payload)
    result["root_pos"] = np.ascontiguousarray(projected[:, :3])
    result["root_rot"] = np.ascontiguousarray(wxyz_to_xyzw(projected[:, 3:7]))
    result["dof_pos"] = np.ascontiguousarray(projected[:, 7:])
    declare_gmr_root_quat_order(result)
    result_metadata = result.setdefault("metadata", {})
    postprocess = result_metadata.setdefault("postprocess", {})
    postprocess["arm_collision_projection"] = projection_summary

    _atomic_pickle(output_path, result, overwrite=args.overwrite)
    report = {
        "schema_version": 1,
        "status": "PASS",
        "output_motion_path": str(output_path),
        "output_motion_sha256": sha256_file(output_path),
        **projection_summary,
    }
    _atomic_json(report_path, report, overwrite=args.overwrite)
    return report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gmr-root", required=True, type=Path)
    parser.add_argument("--robot", required=True, choices=sorted(ROBOT_PROFILES))
    parser.add_argument("--motion", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--report", required=True, type=Path)
    parser.add_argument("--clearance-m", type=float, default=0.003)
    parser.add_argument("--maximum-position-drift-m", type=float, default=0.065)
    parser.add_argument("--maximum-orientation-drift-deg", type=float, default=35.0)
    parser.add_argument("--overwrite", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        report = run(args)
    except (ProjectionError, OSError, RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    print(
        "PASS: targeted arm collisions "
        f"{report['before']['collision_frames']} -> {report['after']['collision_frames']} frame(s); "
        f"modified {report['modified_frames']}/{report['before']['frames']} frame(s)"
    )
    print(f"motion: {report['output_motion_path']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
