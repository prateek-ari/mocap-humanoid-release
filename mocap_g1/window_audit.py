"""Fail-closed, fixed-window kinematic audit for retargeted G1 recordings.

The audit is deliberately read-only.  It accepts either the canonical motion
arrays (``root_pos``, ``root_rot``, ``dof_pos``, ``fps``) or the compact cache
form used by :mod:`tools.xsens.loop_bvh_g1` (``qpos``, ``fps``).  A compact
``qpos`` row is ``root_pos[3] + root_rot[4] + dof_pos[29]``.

Velocity and acceleration are differentiated over the complete recording and
then assigned to a window by the derivative's ending frame.  Consequently, a
jump across a window boundary is not hidden by windowing.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from .io_utils import sha256_file
from .schema import G1_29DOF_JOINTS, JointLimit


SCHEMA_VERSION = 1
QPOS_WIDTH = 3 + 4 + len(G1_29DOF_JOINTS)


class WindowAuditError(RuntimeError):
    """Raised when an audit input or configuration cannot be trusted."""


@dataclass(frozen=True)
class MotionArrays:
    """Validated canonical arrays loaded from a recording."""

    root_pos: np.ndarray
    root_rot: np.ndarray
    dof_pos: np.ndarray
    fps: float
    input_layout: str


@dataclass(frozen=True)
class AuditConfig:
    """Configuration for :func:`audit_motion`."""

    window_seconds: float = 10.0
    discontinuity_rad: float = 0.25
    limit_fraction: float = 1.0
    position_tolerance_rad: float = 1e-6
    candidate_count: int = 5

    def validate(self) -> None:
        for name, value in (
            ("window_seconds", self.window_seconds),
            ("discontinuity_rad", self.discontinuity_rad),
            ("limit_fraction", self.limit_fraction),
        ):
            if isinstance(value, bool) or not math.isfinite(float(value)) or value <= 0:
                raise WindowAuditError(f"{name} must be a finite positive number")
        if self.limit_fraction > 1.0:
            raise WindowAuditError("limit_fraction must be <= 1.0")
        if (
            isinstance(self.position_tolerance_rad, bool)
            or not math.isfinite(float(self.position_tolerance_rad))
            or self.position_tolerance_rad < 0
        ):
            raise WindowAuditError("position_tolerance_rad must be a finite non-negative number")
        if (
            isinstance(self.candidate_count, bool)
            or not isinstance(self.candidate_count, int)
            or self.candidate_count < 0
        ):
            raise WindowAuditError("candidate_count must be a non-negative integer")


def _numeric_array(value: Any, *, name: str, shape_tail: tuple[int, ...]) -> np.ndarray:
    raw = np.asarray(value)
    if raw.dtype.kind not in "iuf":
        raise WindowAuditError(f"{name} must be a real numeric array, got dtype {raw.dtype}")
    array = np.asarray(raw, dtype=np.float64)
    if array.ndim != 1 + len(shape_tail) or array.shape[1:] != shape_tail:
        expected = "T," + ",".join(str(size) for size in shape_tail)
        raise WindowAuditError(f"{name} must have shape [{expected}], got {list(array.shape)}")
    if not np.isfinite(array).all():
        raise WindowAuditError(f"{name} contains non-finite values")
    return array


def _scalar_fps(value: Any) -> float:
    raw = np.asarray(value)
    if raw.shape != () or raw.dtype.kind not in "iuf":
        raise WindowAuditError("fps must be one real numeric scalar")
    fps = float(raw.item())
    if not math.isfinite(fps) or fps <= 0:
        raise WindowAuditError("fps must be finite and positive")
    return fps


def _validate_motion(
    root_pos: Any,
    root_rot: Any,
    dof_pos: Any,
    fps: Any,
    *,
    input_layout: str,
) -> MotionArrays:
    position = _numeric_array(root_pos, name="root_pos", shape_tail=(3,))
    rotation = _numeric_array(root_rot, name="root_rot", shape_tail=(4,))
    joints = _numeric_array(
        dof_pos, name="dof_pos", shape_tail=(len(G1_29DOF_JOINTS),)
    )
    rate = _scalar_fps(fps)
    frames = position.shape[0]
    if frames < 3:
        raise WindowAuditError("motion must contain at least 3 frames to audit acceleration")
    if rotation.shape[0] != frames or joints.shape[0] != frames:
        raise WindowAuditError(
            "root_pos, root_rot, and dof_pos must have the same frame count"
        )
    quaternion_error = np.abs(np.linalg.norm(rotation, axis=1) - 1.0)
    if float(np.max(quaternion_error)) > 1e-3:
        bad_frame = int(np.flatnonzero(quaternion_error > 1e-3)[0])
        raise WindowAuditError(
            f"root_rot contains a non-unit quaternion at frame {bad_frame}"
        )
    return MotionArrays(position, rotation, joints, rate, input_layout)


def load_npz_motion(path: str | Path) -> MotionArrays:
    """Safely load and validate a canonical or compact G1 NPZ recording.

    Pickled/object arrays are never enabled.  If both layouts are present they
    must describe exactly the same motion, preventing an ambiguous audit.
    """
    source = Path(path)
    if not source.is_file():
        raise WindowAuditError(f"recording is not a readable file: {source}")
    if source.suffix.lower() != ".npz":
        raise WindowAuditError(f"recording must be an .npz file: {source}")
    try:
        with np.load(source, allow_pickle=False) as archive:
            names = set(archive.files)
            canonical_names = {"root_pos", "root_rot", "dof_pos"}
            has_any_canonical = bool(names & canonical_names)
            has_canonical = canonical_names <= names
            has_qpos = "qpos" in names
            if "fps" not in names:
                raise WindowAuditError("recording lacks required array 'fps'")
            if has_any_canonical and not has_canonical:
                missing = sorted(canonical_names - names)
                raise WindowAuditError(
                    "recording has an incomplete canonical layout; missing " + ", ".join(missing)
                )
            if not has_canonical and not has_qpos:
                raise WindowAuditError(
                    "recording must contain root_pos/root_rot/dof_pos or qpos"
                )

            fps = archive["fps"]
            canonical: MotionArrays | None = None
            compact: MotionArrays | None = None
            if has_canonical:
                canonical = _validate_motion(
                    archive["root_pos"],
                    archive["root_rot"],
                    archive["dof_pos"],
                    fps,
                    input_layout="canonical",
                )
            if has_qpos:
                qpos = _numeric_array(archive["qpos"], name="qpos", shape_tail=(QPOS_WIDTH,))
                compact = _validate_motion(
                    qpos[:, :3],
                    qpos[:, 3:7],
                    qpos[:, 7:],
                    fps,
                    input_layout="qpos",
                )
    except WindowAuditError:
        raise
    except (OSError, ValueError, KeyError) as exc:
        raise WindowAuditError(f"cannot read recording {source}: {exc}") from exc

    if canonical is not None and compact is not None:
        for name in ("root_pos", "root_rot", "dof_pos"):
            if not np.array_equal(getattr(canonical, name), getattr(compact, name)):
                raise WindowAuditError(
                    f"canonical arrays and qpos disagree for {name}"
                )
        return MotionArrays(
            canonical.root_pos,
            canonical.root_rot,
            canonical.dof_pos,
            canonical.fps,
            "canonical+qpos",
        )
    # The layout checks above guarantee one of these is populated.
    return canonical if canonical is not None else compact  # type: ignore[return-value]


def _round(value: float, digits: int = 9) -> float:
    return round(float(value), digits)


def _time(frame: int, fps: float) -> float:
    return _round(frame / fps, 6)


def _derivative_stats(
    values: np.ndarray,
    frames: np.ndarray,
    fps: float,
) -> dict[str, Any]:
    if values.size == 0:
        return {
            "max_abs": None,
            "p99_abs": None,
            "max_frame": None,
            "max_time_s": None,
            "max_joint": None,
            "by_joint": {
                name: {"max_abs": None, "p99_abs": None}
                for name in G1_29DOF_JOINTS
            },
        }
    absolute = np.abs(values)
    flat_index = int(np.argmax(absolute))
    row, column = np.unravel_index(flat_index, absolute.shape)
    by_joint = {
        name: {
            "max_abs": _round(np.max(absolute[:, joint_index])),
            "p99_abs": _round(np.percentile(absolute[:, joint_index], 99)),
        }
        for joint_index, name in enumerate(G1_29DOF_JOINTS)
    }
    frame = int(frames[row])
    return {
        "max_abs": _round(absolute[row, column]),
        "p99_abs": _round(np.percentile(absolute, 99)),
        "max_frame": frame,
        "max_time_s": _time(frame, fps),
        "max_joint": G1_29DOF_JOINTS[column],
        "by_joint": by_joint,
    }


def _limit_violations(
    values: np.ndarray,
    frames: np.ndarray,
    fps: float,
    lows: np.ndarray,
    highs: np.ndarray,
    *,
    kind: str,
    comparison_tolerance: float = 0.0,
    official_lows: np.ndarray | None = None,
    official_highs: np.ndarray | None = None,
) -> list[dict[str, Any]]:
    if values.size == 0:
        return []
    mask = (values < lows - comparison_tolerance) | (
        values > highs + comparison_tolerance
    )
    violations: list[dict[str, Any]] = []
    for joint_index, name in enumerate(G1_29DOF_JOINTS):
        rows = np.flatnonzero(mask[:, joint_index])
        if not len(rows):
            continue
        observed = values[rows, joint_index]
        sample_frames = [int(frames[row]) for row in rows]
        excess = np.maximum(lows[joint_index] - observed, observed - highs[joint_index])
        entry: dict[str, Any] = {
            "joint": name,
            "count": len(sample_frames),
            "frames": sample_frames,
            "timestamps_s": [_time(frame, fps) for frame in sample_frames],
            "min_observed": _round(np.min(observed)),
            "max_observed": _round(np.max(observed)),
            "max_excess": _round(np.max(excess)),
            "applied_lower": _round(lows[joint_index]),
            "applied_upper": _round(highs[joint_index]),
        }
        if official_lows is not None and official_highs is not None:
            entry["official_lower"] = _round(official_lows[joint_index])
            entry["official_upper"] = _round(official_highs[joint_index])
        entry["units"] = "rad" if kind == "position" else "rad/s"
        violations.append(entry)
    return violations


def _discontinuities(
    deltas: np.ndarray,
    frames: np.ndarray,
    fps: float,
    threshold: float,
) -> list[dict[str, Any]]:
    absolute = np.abs(deltas)
    rows = np.flatnonzero(np.any(absolute > threshold, axis=1))
    events: list[dict[str, Any]] = []
    for row in rows:
        joint_indices = np.flatnonzero(absolute[row] > threshold)
        frame = int(frames[row])
        events.append(
            {
                "frame": frame,
                "timestamp_s": _time(frame, fps),
                "max_delta_rad": _round(np.max(absolute[row, joint_indices])),
                "joints": [
                    {
                        "joint": G1_29DOF_JOINTS[index],
                        "delta_rad": _round(deltas[row, index]),
                    }
                    for index in joint_indices
                ],
            }
        )
    return events


def _violation_count(entries: list[dict[str, Any]]) -> int:
    return sum(int(entry["count"]) for entry in entries)


def _velocity_utilization(values: np.ndarray, applied_velocity: np.ndarray) -> float:
    if values.size == 0:
        return 0.0
    return float(np.max(np.abs(values) / applied_velocity[np.newaxis, :]))


# AuditConfig is frozen, so one shared default instance is safe.
DEFAULT_AUDIT_CONFIG = AuditConfig()


def audit_motion(
    motion: MotionArrays,
    limits: Mapping[str, JointLimit],
    config: AuditConfig = DEFAULT_AUDIT_CONFIG,
    *,
    source_path: str | Path | None = None,
    source_sha256: str | None = None,
) -> dict[str, Any]:
    """Audit a validated G1 motion and return a JSON-serializable report."""
    config.validate()
    missing = [name for name in G1_29DOF_JOINTS if name not in limits]
    if missing:
        raise WindowAuditError("joint limits are incomplete: " + ", ".join(missing))

    frames_total = motion.dof_pos.shape[0]
    fps = motion.fps
    window_frames = int(round(config.window_seconds * fps))
    if window_frames < 1:
        raise WindowAuditError(
            "window_seconds is shorter than one frame at the recording fps"
        )

    official_lower = np.asarray([limits[name].lower for name in G1_29DOF_JOINTS])
    official_upper = np.asarray([limits[name].upper for name in G1_29DOF_JOINTS])
    official_velocity = np.asarray([limits[name].velocity for name in G1_29DOF_JOINTS])
    if not (
        np.isfinite(official_lower).all()
        and np.isfinite(official_upper).all()
        and np.isfinite(official_velocity).all()
        and np.all(official_lower < official_upper)
        and np.all(official_velocity > 0)
    ):
        raise WindowAuditError("joint limits must be finite with lower < upper and velocity > 0")

    midpoint = (official_lower + official_upper) / 2.0
    half_range = (official_upper - official_lower) / 2.0
    applied_lower = midpoint - config.limit_fraction * half_range
    applied_upper = midpoint + config.limit_fraction * half_range
    applied_velocity = official_velocity * config.limit_fraction

    joint_frames = np.arange(frames_total, dtype=np.int64)
    velocity_frames = np.arange(1, frames_total, dtype=np.int64)
    acceleration_frames = np.arange(2, frames_total, dtype=np.int64)
    delta = np.diff(motion.dof_pos, axis=0)
    velocity = delta * fps
    acceleration = np.diff(motion.dof_pos, n=2, axis=0) * fps**2

    windows: list[dict[str, Any]] = []
    for window_index, start in enumerate(range(0, frames_total, window_frames)):
        end = min(start + window_frames, frames_total)
        position_selector = (joint_frames >= start) & (joint_frames < end)
        velocity_selector = (velocity_frames >= start) & (velocity_frames < end)
        acceleration_selector = (
            (acceleration_frames >= start) & (acceleration_frames < end)
        )
        window_position = motion.dof_pos[position_selector]
        window_velocity = velocity[velocity_selector]
        window_acceleration = acceleration[acceleration_selector]
        window_velocity_frames = velocity_frames[velocity_selector]
        window_acceleration_frames = acceleration_frames[acceleration_selector]

        position_violations = _limit_violations(
            window_position,
            joint_frames[position_selector],
            fps,
            applied_lower,
            applied_upper,
            kind="position",
            comparison_tolerance=config.position_tolerance_rad,
            official_lows=official_lower,
            official_highs=official_upper,
        )
        velocity_violations = _limit_violations(
            window_velocity,
            window_velocity_frames,
            fps,
            -applied_velocity,
            applied_velocity,
            kind="velocity",
            official_lows=-official_velocity,
            official_highs=official_velocity,
        )
        discontinuities = _discontinuities(
            delta[velocity_selector],
            window_velocity_frames,
            fps,
            config.discontinuity_rad,
        )
        reasons = []
        if position_violations:
            reasons.append("position_limit_violations")
        if velocity_violations:
            reasons.append("velocity_limit_violations")
        if discontinuities:
            reasons.append("joint_discontinuities")
        windows.append(
            {
                "window_index": window_index,
                "start_frame": start,
                "end_frame_exclusive": end,
                "start_time_s": _time(start, fps),
                "end_time_s": _time(end, fps),
                "duration_s": _round((end - start) / fps, 6),
                "frame_count": end - start,
                "is_full_window": end - start == window_frames,
                "status": "FAIL" if reasons else "PASS",
                "reasons": reasons,
                "joint_speed_rad_s": _derivative_stats(
                    window_velocity, window_velocity_frames, fps
                ),
                "joint_acceleration_rad_s2": _derivative_stats(
                    window_acceleration, window_acceleration_frames, fps
                ),
                "violations": {
                    "position_by_joint": position_violations,
                    "velocity_by_joint": velocity_violations,
                    "position_sample_count": _violation_count(position_violations),
                    "velocity_sample_count": _violation_count(velocity_violations),
                },
                "discontinuities": discontinuities,
                "max_velocity_limit_ratio": _round(
                    _velocity_utilization(window_velocity, applied_velocity)
                ),
            }
        )

    all_position_violations = sum(
        int(window["violations"]["position_sample_count"]) for window in windows
    )
    all_velocity_violations = sum(
        int(window["violations"]["velocity_sample_count"]) for window in windows
    )
    all_discontinuities = sum(len(window["discontinuities"]) for window in windows)
    failed_windows = sum(window["status"] == "FAIL" for window in windows)
    reasons = []
    if all_position_violations:
        reasons.append("position_limit_violations")
    if all_velocity_violations:
        reasons.append("velocity_limit_violations")
    if all_discontinuities:
        reasons.append("joint_discontinuities")

    candidates = []
    for window in windows:
        if window["status"] != "PASS" or not window["is_full_window"]:
            continue
        acceleration_stats = window["joint_acceleration_rad_s2"]
        candidates.append(
            {
                "window_index": window["window_index"],
                "start_time_s": window["start_time_s"],
                "end_time_s": window["end_time_s"],
                "max_velocity_limit_ratio": window["max_velocity_limit_ratio"],
                "p99_joint_acceleration_rad_s2": acceleration_stats["p99_abs"],
                "max_joint_acceleration_rad_s2": acceleration_stats["max_abs"],
            }
        )
    candidates.sort(
        key=lambda candidate: (
            candidate["max_velocity_limit_ratio"],
            candidate["p99_joint_acceleration_rad_s2"]
            if candidate["p99_joint_acceleration_rad_s2"] is not None
            else math.inf,
            candidate["max_joint_acceleration_rad_s2"]
            if candidate["max_joint_acceleration_rad_s2"] is not None
            else math.inf,
            candidate["window_index"],
        )
    )
    candidates = candidates[: config.candidate_count]
    for rank, candidate in enumerate(candidates, start=1):
        candidate["rank"] = rank

    report: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "source": {
            "path": str(Path(source_path).resolve()) if source_path is not None else None,
            "sha256": source_sha256,
            "input_layout": motion.input_layout,
        },
        "motion": {
            "robot": "unitree_g1",
            "frames": frames_total,
            "fps": _round(fps),
            "duration_s": _round(frames_total / fps, 6),
            "joint_count": len(G1_29DOF_JOINTS),
            "joint_names": list(G1_29DOF_JOINTS),
        },
        "configuration": {
            "window_seconds_requested": _round(config.window_seconds, 6),
            "window_frames": window_frames,
            "window_seconds_effective": _round(window_frames / fps, 6),
            "discontinuity_threshold_rad_per_frame": _round(
                config.discontinuity_rad
            ),
            "position_comparison_tolerance_rad": _round(
                config.position_tolerance_rad
            ),
            "limit_fraction": _round(config.limit_fraction),
            "limits": "official G1 29-DoF URDF position and velocity limits",
            "derivative_assignment": "ending_frame",
        },
        "summary": {
            "status": "FAIL" if reasons else "PASS",
            "reasons": reasons,
            "window_count": len(windows),
            "failed_window_count": failed_windows,
            "passed_window_count": len(windows) - failed_windows,
            "position_violation_sample_count": all_position_violations,
            "velocity_violation_sample_count": all_velocity_violations,
            "discontinuity_frame_count": all_discontinuities,
        },
        "candidate_safe_windows": {
            "qualification": (
                "Full-length kinematic candidates only: no applied position/velocity-"
                "limit violations or configured joint discontinuities. This is not "
                "a hardware-safety certification."
            ),
            "ranking": "lowest velocity-limit ratio, then p99/max joint acceleration",
            "windows": candidates,
        },
        "windows": windows,
    }
    return report


def audit_npz(
    path: str | Path,
    limits: Mapping[str, JointLimit],
    config: AuditConfig = DEFAULT_AUDIT_CONFIG,
) -> dict[str, Any]:
    """Load and audit an NPZ while recording its immutable source fingerprint."""
    source = Path(path).resolve()
    fingerprint_before = sha256_file(source)
    motion = load_npz_motion(source)
    report = audit_motion(
        motion,
        limits,
        config,
        source_path=source,
        source_sha256=fingerprint_before,
    )
    fingerprint_after = sha256_file(source)
    if fingerprint_after != fingerprint_before:
        raise WindowAuditError("source recording changed while it was being audited")
    return report
