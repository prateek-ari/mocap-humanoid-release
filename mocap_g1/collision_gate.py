"""Fail-closed fixed-base self-collision gate for G1 joint references.

The gate either accepts an already collision-free pose or tries a small set of
deterministic retreats toward the neutral pose encoded by the supplied MJCF.
It never estimates a neutral pose, offsets, or calibration from the recording.
Every retreat is bounded relative to the original frame.  A trajectory is only
returned when every frame is nonpenetrating and the correction itself is
temporally continuous; otherwise the whole derivative is rejected.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from .controller_screen import (
    ControllerScreenConfig,
    ControllerScreenError,
    _build_fixed_model,
    _current_self_collisions,
    _require_mujoco,
    _validate_reference,
    sha256_file,
)
from .io_utils import array_sha256 as _array_sha256, sha256_text, write_qpos_npz_atomic
from .schema import G1_29DOF_JOINTS


COLLISION_GATE_VERSION = 1
ARM_JOINTS = G1_29DOF_JOINTS[15:]


class CollisionGateError(RuntimeError):
    """Raised when collision gating cannot produce a trustworthy result."""


@dataclass(frozen=True)
class CollisionGateConfig:
    """Conservative correction limits for fixed-base reference poses."""

    max_per_joint_deviation_rad: float = 0.25
    max_total_deviation_rad: float = 0.35
    max_correction_step_rad: float = 0.05
    max_correction_velocity_rad_s: float = 1.0
    max_correction_acceleration_rad_s2: float = 4.0
    search_steps: int = 24
    refinement_steps: int = 18
    global_alpha_guard_fraction: float = 0.02
    position_tolerance_rad: float = 1e-6
    projectable_joints: tuple[str, ...] = ARM_JOINTS
    projection_mode: str = "per_frame_then_global"

    def validate(self) -> None:
        for name in (
            "max_per_joint_deviation_rad",
            "max_total_deviation_rad",
            "max_correction_step_rad",
            "max_correction_velocity_rad_s",
            "max_correction_acceleration_rad_s2",
            "position_tolerance_rad",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise CollisionGateError(f"{name} must be finite and positive")
        if (
            not math.isfinite(float(self.global_alpha_guard_fraction))
            or self.global_alpha_guard_fraction < 0.0
        ):
            raise CollisionGateError(
                "global_alpha_guard_fraction must be finite and non-negative"
            )
        for name in ("search_steps", "refinement_steps"):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or value < 1:
                raise CollisionGateError(f"{name} must be a positive integer")
        if not self.projectable_joints:
            raise CollisionGateError("projectable_joints cannot be empty")
        unknown = [name for name in self.projectable_joints if name not in G1_29DOF_JOINTS]
        if unknown:
            raise CollisionGateError(f"unknown projectable joints: {unknown}")
        if len(set(self.projectable_joints)) != len(self.projectable_joints):
            raise CollisionGateError("projectable_joints cannot contain duplicates")
        if self.projection_mode not in {
            "per_frame_then_global",
            "per_frame_only",
            "global_only",
        }:
            raise CollisionGateError(
                "projection_mode must be per_frame_then_global, per_frame_only, or global_only"
            )


@dataclass(frozen=True)
class CollisionGateResult:
    """Collision-gate output; qpos is absent whenever the gate rejects."""

    qpos: np.ndarray | None
    fps: float
    report: dict[str, Any]


def _pair_records(pairs: Mapping[tuple[str, str], float]) -> list[dict[str, Any]]:
    return [
        {
            "body_a": pair[0],
            "body_b": pair[1],
            "contact_distance_m": float(pairs[pair]),
        }
        for pair in sorted(pairs)
    ]


def _projection_groups(projectable: tuple[str, ...]) -> list[tuple[str, tuple[int, ...]]]:
    """Return deterministic, named retreat directions from narrow to broad."""
    allowed = set(projectable)
    index = {name: position for position, name in enumerate(G1_29DOF_JOINTS)}

    raw: list[tuple[str, tuple[str, ...]]] = []
    raw.extend((f"single:{name}", (name,)) for name in projectable)

    for role in (
        "shoulder_pitch_joint",
        "shoulder_roll_joint",
        "shoulder_yaw_joint",
        "elbow_joint",
        "wrist_roll_joint",
        "wrist_pitch_joint",
        "wrist_yaw_joint",
    ):
        raw.append((f"bilateral:{role}", (f"left_{role}", f"right_{role}")))

    for side in ("left", "right"):
        shoulder = tuple(f"{side}_{role}_joint" for role in (
            "shoulder_pitch", "shoulder_roll", "shoulder_yaw"
        ))
        raw.append((f"{side}:shoulder", shoulder))
        raw.append((f"{side}:upper_arm", shoulder + (f"{side}_elbow_joint",)))
        raw.append((f"{side}:arm", tuple(name for name in ARM_JOINTS if name.startswith(side))))

    raw.extend(
        (
            ("bilateral:shoulders", tuple(
                name for name in ARM_JOINTS
                if "shoulder_" in name
            )),
            ("bilateral:upper_arms", tuple(
                name for name in ARM_JOINTS
                if "shoulder_" in name or "elbow_" in name
            )),
            ("all_projectable", projectable),
        )
    )

    result: list[tuple[str, tuple[int, ...]]] = []
    seen: set[tuple[int, ...]] = set()
    for label, names in raw:
        if not names or any(name not in allowed for name in names):
            continue
        columns = tuple(index[name] for name in names)
        if columns not in seen:
            seen.add(columns)
            result.append((label, columns))
    return result


def _max_alpha(
    original: np.ndarray,
    neutral: np.ndarray,
    columns: tuple[int, ...],
    config: CollisionGateConfig,
) -> float:
    delta = neutral[list(columns)] - original[list(columns)]
    maximum = float(np.max(np.abs(delta)))
    norm = float(np.linalg.norm(delta))
    alpha = 1.0
    if maximum > 0.0:
        alpha = min(alpha, config.max_per_joint_deviation_rad / maximum)
    if norm > 0.0:
        alpha = min(alpha, config.max_total_deviation_rad / norm)
    return max(0.0, alpha)


def _candidate(
    original: np.ndarray,
    neutral: np.ndarray,
    columns: tuple[int, ...],
    alpha: float,
) -> np.ndarray:
    result = original.copy()
    selected = list(columns)
    result[selected] = (1.0 - alpha) * original[selected] + alpha * neutral[selected]
    return result


def _find_safe_retreat(
    original: np.ndarray,
    neutral: np.ndarray,
    columns: tuple[int, ...],
    config: CollisionGateConfig,
    collision_pairs: Any,
) -> tuple[np.ndarray, float] | None:
    """Find the first collision-free point along one bounded retreat ray."""
    upper = _max_alpha(original, neutral, columns, config)
    if upper <= 0.0:
        return None

    unsafe_alpha = 0.0
    safe_alpha: float | None = None
    for step in range(1, config.search_steps + 1):
        alpha = upper * step / config.search_steps
        trial = _candidate(original, neutral, columns, alpha)
        if not collision_pairs(trial):
            safe_alpha = alpha
            break
        unsafe_alpha = alpha
    if safe_alpha is None:
        return None

    for _ in range(config.refinement_steps):
        midpoint = 0.5 * (unsafe_alpha + safe_alpha)
        trial = _candidate(original, neutral, columns, midpoint)
        if collision_pairs(trial):
            unsafe_alpha = midpoint
        else:
            safe_alpha = midpoint

    # Stay just inside the observed safe interval to avoid returning a pose on
    # a floating-point contact boundary.  This is an angular numerical guard,
    # not a claim of geometric clearance.
    guarded_alpha = min(upper, safe_alpha + max(1e-8, upper * 1e-7))
    result = _candidate(original, neutral, columns, guarded_alpha)
    if collision_pairs(result):
        result = _candidate(original, neutral, columns, safe_alpha)
        guarded_alpha = safe_alpha
    if collision_pairs(result):
        return None
    return result, guarded_alpha


def _group_sides(columns: tuple[int, ...]) -> set[str]:
    sides: set[str] = set()
    for column in columns:
        name = G1_29DOF_JOINTS[column]
        if name.startswith("left_"):
            sides.add("left")
        elif name.startswith("right_"):
            sides.add("right")
    return sides


def _collision_sides(
    pairs_by_frame: list[Mapping[tuple[str, str], float]],
) -> set[str]:
    sides: set[str] = set()
    for pairs in pairs_by_frame:
        for bodies in pairs:
            for body in bodies:
                if body.startswith("left_"):
                    sides.add("left")
                elif body.startswith("right_"):
                    sides.add("right")
    return sides


def _global_max_alpha(
    original: np.ndarray,
    neutral: np.ndarray,
    columns: tuple[int, ...],
    config: CollisionGateConfig,
) -> float:
    selected = list(columns)
    delta = neutral[selected][None, :] - original[:, selected]
    maximum = float(np.max(np.abs(delta)))
    maximum_norm = float(np.max(np.linalg.norm(delta, axis=1)))
    alpha = 1.0
    if maximum > 0.0:
        alpha = min(alpha, config.max_per_joint_deviation_rad / maximum)
    if maximum_norm > 0.0:
        alpha = min(alpha, config.max_total_deviation_rad / maximum_norm)
    return max(0.0, alpha)


def _global_candidate(
    original: np.ndarray,
    neutral: np.ndarray,
    columns: tuple[int, ...],
    alpha: float,
) -> np.ndarray:
    result = original.copy()
    selected = list(columns)
    result[:, selected] = (
        (1.0 - alpha) * original[:, selected]
        + alpha * neutral[selected][None, :]
    )
    return result


def _find_global_safe_retreat(
    original: np.ndarray,
    neutral: np.ndarray,
    groups: list[tuple[str, tuple[int, ...]]],
    required_sides: set[str],
    config: CollisionGateConfig,
    collision_pairs: Any,
) -> tuple[np.ndarray, float, str, tuple[int, ...], float] | None:
    """Find one group and interpolation fraction safe for every frame."""

    def trajectory_is_safe(candidate: np.ndarray) -> bool:
        return all(not collision_pairs(frame) for frame in candidate)

    for label, columns in groups:
        # Arm joints cannot move the opposite arm.  This avoids testing a
        # single-sided direction when the recording contains collisions on
        # both sides, without changing the set of poses that could resolve all
        # observed arm/torso contacts.
        if required_sides and not required_sides.issubset(_group_sides(columns)):
            continue
        upper = _global_max_alpha(original, neutral, columns, config)
        if upper <= 0.0:
            continue

        unsafe_alpha = 0.0
        safe_alpha: float | None = None
        for step in range(1, config.search_steps + 1):
            alpha = upper * step / config.search_steps
            trial = _global_candidate(original, neutral, columns, alpha)
            if trajectory_is_safe(trial):
                safe_alpha = alpha
                break
            unsafe_alpha = alpha
        if safe_alpha is None:
            continue

        for _ in range(config.refinement_steps):
            midpoint = 0.5 * (unsafe_alpha + safe_alpha)
            trial = _global_candidate(original, neutral, columns, midpoint)
            if trajectory_is_safe(trial):
                safe_alpha = midpoint
            else:
                unsafe_alpha = midpoint

        guarded_alpha = safe_alpha + config.global_alpha_guard_fraction
        if guarded_alpha > upper + 1e-12:
            # The requested guard is part of the conservative acceptance
            # contract; do not silently consume more than either deviation
            # budget permits.
            continue
        guarded_alpha = min(upper, guarded_alpha)
        result = _global_candidate(original, neutral, columns, guarded_alpha)
        if not trajectory_is_safe(result):
            continue
        return (
            result,
            guarded_alpha,
            label,
            columns,
            guarded_alpha - safe_alpha,
        )
    return None


def _correction_dynamics(
    corrections: np.ndarray,
    rate: float,
    config: CollisionGateConfig,
) -> tuple[dict[str, float], list[dict[str, Any]]]:
    correction_steps = np.diff(corrections, axis=0)
    max_step = float(np.max(np.abs(correction_steps))) if len(correction_steps) else 0.0
    velocity = np.zeros_like(corrections)
    velocity[1:] = correction_steps * rate
    acceleration = np.zeros_like(corrections)
    acceleration[1:] = np.diff(velocity, axis=0) * rate
    max_velocity = float(np.max(np.abs(velocity)))
    max_acceleration = float(np.max(np.abs(acceleration)))
    rejections: list[dict[str, Any]] = []

    for frame, joint in np.argwhere(
        np.abs(correction_steps) > config.max_correction_step_rad + 1e-12
    ):
        rejections.append(
            {
                "frame": int(frame + 1),
                "time_s": float((frame + 1) / rate),
                "reason": "correction_step_limit_exceeded",
                "joint": G1_29DOF_JOINTS[int(joint)],
                "correction_step_rad": float(correction_steps[frame, joint]),
            }
        )
    for frame, joint in np.argwhere(
        np.abs(velocity) > config.max_correction_velocity_rad_s + 1e-12
    ):
        rejections.append(
            {
                "frame": int(frame),
                "time_s": float(frame / rate),
                "reason": "correction_velocity_limit_exceeded",
                "joint": G1_29DOF_JOINTS[int(joint)],
                "correction_velocity_rad_s": float(velocity[frame, joint]),
            }
        )
    for frame, joint in np.argwhere(
        np.abs(acceleration) > config.max_correction_acceleration_rad_s2 + 1e-12
    ):
        rejections.append(
            {
                "frame": int(frame),
                "time_s": float(frame / rate),
                "reason": "correction_acceleration_limit_exceeded",
                "joint": G1_29DOF_JOINTS[int(joint)],
                "correction_acceleration_rad_s2": float(acceleration[frame, joint]),
            }
        )
    return {
        "max_correction_step_rad": max_step,
        "max_correction_velocity_rad_s": max_velocity,
        "max_correction_acceleration_rad_s2": max_acceleration,
    }, rejections


def gate_reference_self_collisions(
    qpos: Any,
    fps: Any,
    *,
    model_path: str | Path,
    config: CollisionGateConfig | None = None,
    source_sha256: str | None = None,
    source_metadata: Mapping[str, Any] | None = None,
) -> CollisionGateResult:
    """Accept or conservatively project a complete fixed-base G1 reference.

    A returned ``qpos`` is proof that every output frame was revalidated as
    nonpenetrating under the supplied model.  If any frame cannot be corrected
    within the configured budgets, ``qpos`` is ``None`` and status is REJECT.
    """
    settings = config or CollisionGateConfig()
    settings.validate()
    try:
        reference, rate = _validate_reference(qpos, fps)
    except ControllerScreenError as exc:
        raise CollisionGateError(str(exc)) from exc
    source = reference.copy()
    input_hash_before = _array_sha256(source)

    model_file = Path(model_path).resolve()
    try:
        model, derived_xml, _actuators, qadr, dadr = _build_fixed_model(
            model_file, ControllerScreenConfig()
        )
    except ControllerScreenError as exc:
        raise CollisionGateError(str(exc)) from exc
    mujoco = _require_mujoco()
    data = mujoco.MjData(model)
    neutral = np.asarray(model.qpos0[qadr], dtype=np.float64).copy()
    joint_ranges = np.asarray(model.jnt_range, dtype=np.float64)
    joint_ids = np.asarray(
        [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
         for name in G1_29DOF_JOINTS],
        dtype=np.int32,
    )
    ranges = joint_ranges[joint_ids]
    groups = _projection_groups(settings.projectable_joints)

    def pairs_for(joints: np.ndarray) -> dict[tuple[str, str], float]:
        data.qpos[qadr] = joints
        data.qvel[dadr] = 0.0
        mujoco.mj_forward(model, data)
        return _current_self_collisions(model, data, mujoco)

    output = source.copy()
    corrections = np.zeros((len(source), len(G1_29DOF_JOINTS)), dtype=np.float64)
    projected_records: list[dict[str, Any]] = []
    rejected_records: list[dict[str, Any]] = []
    input_limit_violation = False
    input_pairs_by_frame: list[dict[tuple[str, str], float]] = [
        {} for _ in range(len(source))
    ]
    input_collision_frames = 0
    for frame, pose in enumerate(source):
        joints = pose[7:]
        outside = (
            joints < ranges[:, 0] - settings.position_tolerance_rad
        ) | (
            joints > ranges[:, 1] + settings.position_tolerance_rad
        )
        if np.any(outside):
            input_limit_violation = True
            rejected_records.append(
                {
                    "frame": frame,
                    "time_s": frame / rate,
                    "reason": "input_joint_position_limit_violation",
                    "joints": [
                        G1_29DOF_JOINTS[index]
                        for index in np.flatnonzero(outside)
                    ],
                }
            )
            continue

        before_pairs = pairs_for(joints)
        input_pairs_by_frame[frame] = before_pairs
        if not before_pairs:
            continue
        input_collision_frames += 1

        if settings.projection_mode == "global_only":
            continue

        accepted: tuple[np.ndarray, float, str, tuple[int, ...]] | None = None
        # Always retain narrow-to-broad order.  Reusing the previous frame's
        # wider group is faster, but can modify an uninvolved limb and can make
        # that correction disappear abruptly when the strategy later changes.
        for label, columns in groups:
            found = _find_safe_retreat(joints, neutral, columns, settings, pairs_for)
            if found is None:
                continue
            candidate, alpha = found
            deviation = candidate - joints
            max_deviation = float(np.max(np.abs(deviation)))
            total_deviation = float(np.linalg.norm(deviation))
            if (
                max_deviation <= settings.max_per_joint_deviation_rad + 1e-12
                and total_deviation <= settings.max_total_deviation_rad + 1e-12
            ):
                accepted = candidate, alpha, label, columns
                break

        if accepted is None:
            rejected_records.append(
                {
                    "frame": frame,
                    "time_s": frame / rate,
                    "reason": "no_nonpenetrating_pose_within_deviation_budgets",
                    "input_collision_pairs": _pair_records(before_pairs),
                }
            )
            continue

        candidate, alpha, label, columns = accepted
        output[frame, 7:] = candidate
        corrections[frame] = candidate - joints
        projected_records.append(
            {
                "frame": frame,
                "time_s": frame / rate,
                "strategy": label,
                "neutral_interpolation_fraction": alpha,
                "changed_joints": [G1_29DOF_JOINTS[index] for index in columns],
                "max_per_joint_deviation_rad": float(np.max(np.abs(corrections[frame]))),
                "total_deviation_rad": float(np.linalg.norm(corrections[frame])),
                "input_collision_pairs": _pair_records(before_pairs),
            }
        )

    per_frame_dynamics, per_frame_dynamics_rejections = _correction_dynamics(
        corrections, rate, settings
    )
    global_projection: dict[str, Any] | None = None
    attempt_global = (
        settings.projection_mode == "global_only"
        or (
            settings.projection_mode == "per_frame_then_global"
            and bool(rejected_records or per_frame_dynamics_rejections)
        )
    )
    if attempt_global and input_collision_frames and not input_limit_violation:
        found_global = _find_global_safe_retreat(
            source[:, 7:],
            neutral,
            groups,
            _collision_sides(input_pairs_by_frame),
            settings,
            pairs_for,
        )
        if found_global is not None:
            global_joints, alpha, label, columns, applied_guard = found_global
            output = source.copy()
            output[:, 7:] = global_joints
            corrections = global_joints - source[:, 7:]
            rejected_records = []
            projected_records = []
            for frame, correction in enumerate(corrections):
                if not np.any(correction):
                    continue
                projected_records.append(
                    {
                        "frame": frame,
                        "time_s": frame / rate,
                        "strategy": f"global:{label}",
                        "neutral_interpolation_fraction": alpha,
                        "changed_joints": [
                            G1_29DOF_JOINTS[index] for index in columns
                        ],
                        "max_per_joint_deviation_rad": float(
                            np.max(np.abs(correction))
                        ),
                        "total_deviation_rad": float(np.linalg.norm(correction)),
                        "input_collision_pairs": _pair_records(
                            input_pairs_by_frame[frame]
                        ),
                    }
                )
            global_projection = {
                "strategy": label,
                "neutral_interpolation_fraction": alpha,
                "configured_alpha_guard_fraction": (
                    settings.global_alpha_guard_fraction
                ),
                "applied_alpha_guard_fraction": applied_guard,
                "changed_joints": [G1_29DOF_JOINTS[index] for index in columns],
                "applied_to_all_frames": True,
                "selection": (
                    "first narrow-to-broad globally safe group, then minimum alpha "
                    "plus configured guard"
                ),
            }
        else:
            if settings.projection_mode == "global_only":
                rejected_records.append(
                    {
                        "reason": "no_globally_coherent_nonpenetrating_trajectory",
                        "frames": len(source),
                    }
                )
            else:
                rejected_records.extend(per_frame_dynamics_rejections)
                rejected_records.append(
                    {
                        "reason": "global_coherent_fallback_failed",
                        "frames": len(source),
                    }
                )
    elif settings.projection_mode != "global_only":
        rejected_records.extend(per_frame_dynamics_rejections)

    dynamics, dynamics_rejections = _correction_dynamics(corrections, rate, settings)
    if global_projection is not None:
        rejected_records.extend(dynamics_rejections)

    # Independent final pass: never trust an optimizer/search result without
    # validating the exact array that would be returned.
    final_collision_records: list[dict[str, Any]] = []
    for frame, pose in enumerate(output):
        pairs = pairs_for(pose[7:])
        if pairs:
            final_collision_records.append(
                {
                    "frame": frame,
                    "time_s": frame / rate,
                    "collision_pairs": _pair_records(pairs),
                }
            )
    if final_collision_records:
        rejected_records.extend(
            {
                "frame": item["frame"],
                "time_s": item["time_s"],
                "reason": "final_validation_self_collision",
                "collision_pairs": item["collision_pairs"],
            }
            for item in final_collision_records
        )

    input_hash_after = _array_sha256(source)
    if input_hash_after != input_hash_before or not np.array_equal(source, reference):
        raise CollisionGateError("input reference was mutated during collision gating")

    status = "REJECT" if rejected_records else "PASS"
    max_per_joint = float(np.max(np.abs(corrections)))
    total_by_frame = np.linalg.norm(corrections, axis=1)
    changed_frames = int(np.count_nonzero(np.any(corrections != 0.0, axis=1)))
    metadata = dict(source_metadata or {})
    report: dict[str, Any] = {
        "schema_version": COLLISION_GATE_VERSION,
        "purpose": "fixed_base_collision_projection_not_calibration",
        "status": status,
        "hardware_candidate": {
            "status": (
                "BLOCKED"
                if metadata.get("offsets_key") == "zero-preview" or status != "PASS"
                else "COLLISION_GATE_ONLY"
            ),
            "reasons": (
                (
                    ["preview_calibration"]
                    if metadata.get("offsets_key") == "zero-preview"
                    else []
                )
                + (["collision_gate_rejected"] if status != "PASS" else [])
            ),
        },
        "summary": {
            "frames": len(source),
            "input_self_collision_frames": input_collision_frames,
            "unchanged_frames": len(source) - changed_frames,
            "projected_frames": changed_frames,
            "rejected_events": len(rejected_records),
            "final_self_collision_frames": len(final_collision_records),
            "max_per_joint_deviation_rad": max_per_joint,
            "max_total_deviation_rad": float(np.max(total_by_frame)),
            **dynamics,
        },
        "input": {
            "frames": len(source),
            "fps": rate,
            "duration_s": (len(source) - 1) / rate,
            "source_sha256": source_sha256,
            "qpos_in_memory_sha256_before": input_hash_before,
            "qpos_in_memory_sha256_after": input_hash_after,
            "preserved": input_hash_before == input_hash_after,
            "offsets_key": metadata.get("offsets_key"),
        },
        "configuration": asdict(settings),
        "method": {
            "neutral_source": "supplied_MJCF_qpos0",
            "calibration_estimation": "none",
            "projection_mode": settings.projection_mode,
            "search": (
                "deterministic bounded interpolation toward MJCF qpos0 over "
                "named joint groups"
            ),
            "penetration_acceptance": "no MuJoCo self contact with distance_m < 0",
            "geometric_clearance_claimed": False,
            "whole_trajectory_policy": (
                "reject_without_qpos_if_any_frame_or_correction_dynamics_check_fails"
            ),
        },
        "model": {
            "source_path": str(model_file),
            "source_sha256": sha256_file(model_file),
            "derived_fixed_base_xml_sha256": sha256_text(derived_xml),
        },
        "global_projection": global_projection,
        "projected_frames": projected_records,
        "rejections": rejected_records,
        "limitations": [
            (
                "nonpenetration is checked in the supplied fixed-base MJCF and does "
                "not prove real-robot clearance"
            ),
            (
                "the deterministic retreat can reject a pose even if a different "
                "bounded optimizer could find a solution"
            ),
            (
                "correction dynamics are bounded, but full output velocity/acceleration "
                "still requires a separate trajectory audit"
            ),
            (
                "no balance, actuator tracking, torque, thermal, or communication "
                "behavior is validated"
            ),
            (
                "the result inherits source calibration quality and never estimates "
                "calibration from motion"
            ),
        ],
    }
    return CollisionGateResult(qpos=None if status != "PASS" else output, fps=rate, report=report)


def write_projected_npz(
    path: str | Path,
    result: CollisionGateResult,
    source_metadata: Mapping[str, Any],
) -> None:
    """Atomically create an accepted derivative, refusing overwrite and rejects."""
    if result.qpos is None or result.report.get("status") != "PASS":
        raise CollisionGateError("cannot write a rejected collision-gate result")
    target = Path(path)
    if target.suffix.lower() != ".npz":
        raise CollisionGateError("projected output path must end in .npz")
    metadata = dict(source_metadata)
    metadata["collision_projection"] = {
        "schema_version": COLLISION_GATE_VERSION,
        "purpose": "fixed_base_collision_projection_not_calibration",
        "source_sha256": result.report["input"]["source_sha256"],
        "model_source_sha256": result.report["model"]["source_sha256"],
        "configuration": result.report["configuration"],
        "summary": result.report["summary"],
    }
    write_qpos_npz_atomic(
        target,
        qpos=result.qpos,
        fps=result.fps,
        metadata=metadata,
        exists_error=CollisionGateError,
    )
