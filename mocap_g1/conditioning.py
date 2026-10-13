"""Offline conditioning for retargeted G1 trajectories.

This module makes an already-retargeted trajectory smoother and slower.  It
does not estimate calibration and must not be used as evidence that an Xsens
recording is calibrated correctly.

When ``position_limits`` are supplied, conditioning never pushes an in-range
sample out of range: zero-phase FIR ringing (Gibbs overshoot of the windowed
sinc) that would carry a joint past its mechanical limit is clamped back to
the limit before time scaling, and the PCHIP resampler is monotone between
samples so it cannot re-introduce an overshoot.  Samples whose SOURCE value is
already outside the limit are deliberately left as they are (and reported):
silently clamping a bad retarget would hide it from the downstream
position-limit gate (eval gate v2 ``joint_position_limits``).

The supported array contract is MuJoCo G1 ``qpos``: root XYZ, a unit root
quaternion in WXYZ order, then the canonical 29 G1 joints.  Conditioning is
deliberately offline and zero-phase: the FIR uses future samples, so this is
not a live-controller filter.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

from .io_utils import write_json_atomic, write_qpos_npz_atomic
from .schema import G1_29DOF_JOINTS


QPOS_WIDTH = 7 + len(G1_29DOF_JOINTS)
CONDITIONING_VERSION = 1


class MotionConditioningError(ValueError):
    """Raised when conditioning cannot produce a validated trajectory."""


@dataclass(frozen=True)
class ConditioningConfig:
    """Configuration for conservative, offline trajectory conditioning."""

    decimation: int = 1
    lowpass_cutoff_hz: float | None = None
    filter_taps: int = 31
    spike_threshold_rad: float = 0.20
    max_time_scale: float = 20.0
    limit_margin: float = 1.002
    max_output_frames: int = 2_000_000


@dataclass(frozen=True)
class ConditionedMotion:
    qpos: np.ndarray
    fps: float
    report: dict[str, Any]


def _positive_finite(value: Any, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise MotionConditioningError(f"{name} must be a positive finite number") from exc
    if not math.isfinite(number) or number <= 0.0:
        raise MotionConditioningError(f"{name} must be a positive finite number")
    return number


def _validate_config(config: ConditioningConfig, input_fps: float) -> tuple[float, float]:
    if isinstance(config.decimation, bool) or not isinstance(config.decimation, int):
        raise MotionConditioningError("decimation must be a positive integer")
    if config.decimation < 1:
        raise MotionConditioningError("decimation must be a positive integer")
    if isinstance(config.filter_taps, bool) or not isinstance(config.filter_taps, int):
        raise MotionConditioningError("filter_taps must be an odd integer >= 3")
    if config.filter_taps < 3 or config.filter_taps % 2 != 1:
        raise MotionConditioningError("filter_taps must be an odd integer >= 3")
    spike_threshold = _positive_finite(config.spike_threshold_rad, "spike_threshold_rad")
    _positive_finite(config.max_time_scale, "max_time_scale")
    _positive_finite(config.limit_margin, "limit_margin")
    if config.limit_margin < 1.0:
        raise MotionConditioningError("limit_margin must be >= 1.0")
    if (
        isinstance(config.max_output_frames, bool)
        or not isinstance(config.max_output_frames, int)
        or config.max_output_frames < 2
    ):
        raise MotionConditioningError("max_output_frames must be an integer >= 2")

    output_fps = input_fps / config.decimation
    cutoff = (
        0.20 * output_fps
        if config.lowpass_cutoff_hz is None
        else _positive_finite(config.lowpass_cutoff_hz, "lowpass_cutoff_hz")
    )
    # The cutoff is checked against the *post-decimation* Nyquist frequency,
    # which guarantees that the FIR is a real anti-alias stage when N > 1.
    if cutoff >= 0.5 * output_fps:
        raise MotionConditioningError(
            f"lowpass_cutoff_hz={cutoff:g} must be below post-decimation Nyquist "
            f"{0.5 * output_fps:g} Hz"
        )
    if output_fps <= 0.0 or not math.isfinite(output_fps):
        raise MotionConditioningError("decimation produces an invalid output FPS")
    return cutoff, spike_threshold


def _validate_qpos(qpos: Any, fps: Any) -> tuple[np.ndarray, float]:
    rate = _positive_finite(fps, "fps")
    try:
        array = np.asarray(qpos, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise MotionConditioningError("qpos must be a numeric array") from exc
    if array.ndim != 2 or array.shape[1] != QPOS_WIDTH:
        raise MotionConditioningError(f"qpos must have shape [T,{QPOS_WIDTH}]")
    if array.shape[0] < 3:
        raise MotionConditioningError("qpos must contain at least 3 frames")
    if not np.isfinite(array).all():
        raise MotionConditioningError("qpos contains non-finite values")
    norms = np.linalg.norm(array[:, 3:7], axis=1)
    if np.any(norms < 1e-12) or np.max(np.abs(norms - 1.0)) > 1e-3:
        raise MotionConditioningError("qpos root quaternions are not unit length")
    validated = array.copy()
    validated[:, 3:7] /= norms[:, None]
    return validated, rate


def _validate_caps(
    caps: Mapping[str, float], joint_names: Sequence[str], label: str
) -> np.ndarray:
    expected = tuple(joint_names)
    missing = [name for name in expected if name not in caps]
    unknown = sorted(set(caps) - set(expected))
    if missing or unknown:
        raise MotionConditioningError(
            f"{label} must name exactly the trajectory joints; "
            f"missing={missing}, unknown={unknown}"
        )
    return np.asarray(
        [_positive_finite(caps[name], f"{label}[{name!r}]") for name in expected],
        dtype=np.float64,
    )


def _validate_position_limits(
    limits: Mapping[str, Sequence[float]], joint_names: Sequence[str]
) -> tuple[np.ndarray, np.ndarray]:
    expected = tuple(joint_names)
    missing = [name for name in expected if name not in limits]
    unknown = sorted(set(limits) - set(expected))
    if missing or unknown:
        raise MotionConditioningError(
            f"position_limits must name exactly the trajectory joints; "
            f"missing={missing}, unknown={unknown}"
        )
    lower = np.empty(len(expected))
    upper = np.empty(len(expected))
    for index, name in enumerate(expected):
        try:
            low, high = (float(value) for value in limits[name])
        except (TypeError, ValueError) as exc:
            raise MotionConditioningError(
                f"position_limits[{name!r}] must be a (lower, upper) pair"
            ) from exc
        if not (math.isfinite(low) and math.isfinite(high) and low < high):
            raise MotionConditioningError(
                f"position_limits[{name!r}] must be finite with lower < upper"
            )
        lower[index], upper[index] = low, high
    return lower, upper


def _position_violation_summary(
    joints: np.ndarray, lower: np.ndarray, upper: np.ndarray, joint_names: Sequence[str]
) -> dict[str, Any]:
    excess = np.maximum(joints - upper[None, :], lower[None, :] - joints)
    mask = excess > 0.0
    per_joint = {
        name: {
            "samples_outside_limit": int(np.count_nonzero(mask[:, index])),
            "max_excess_rad": float(np.max(excess[:, index])),
        }
        for index, name in enumerate(joint_names)
        if np.any(mask[:, index])
    }
    return {
        "samples_outside_limit": int(np.count_nonzero(mask)),
        "frames_outside_limit": int(np.count_nonzero(np.any(mask, axis=1))),
        "max_excess_rad": float(max(0.0, float(np.max(excess)))) if excess.size else 0.0,
        "joints": per_joint,
    }


def _align_quaternion_signs(quaternions: np.ndarray) -> np.ndarray:
    result = np.asarray(quaternions, dtype=np.float64).copy()
    for index in range(1, len(result)):
        if float(np.dot(result[index - 1], result[index])) < 0.0:
            result[index] *= -1.0
    return result


def _fir_kernel(fps: float, cutoff_hz: float, taps: int) -> np.ndarray:
    offset = np.arange(taps, dtype=np.float64) - taps // 2
    normalized_cutoff = cutoff_hz / fps
    kernel = 2.0 * normalized_cutoff * np.sinc(2.0 * normalized_cutoff * offset)
    kernel *= np.hamming(taps)
    kernel /= np.sum(kernel)
    return kernel


def _zero_phase_fir(values: np.ndarray, kernel: np.ndarray) -> np.ndarray:
    half = len(kernel) // 2
    padded = np.pad(values, ((half, half), (0, 0)), mode="edge")
    return np.stack(
        [np.convolve(padded[:, column], kernel, mode="valid") for column in range(values.shape[1])],
        axis=1,
    )


def _replace_one_frame_joint_spikes(
    joints: np.ndarray, threshold_rad: float
) -> tuple[np.ndarray, np.ndarray]:
    """Replace only isolated A-B-A excursions, independently per joint."""
    result = joints.copy()
    previous = joints[1:-1] - joints[:-2]
    following = joints[1:-1] - joints[2:]
    midpoint = 0.5 * (joints[:-2] + joints[2:])
    deviation = joints[1:-1] - midpoint
    mask = (
        (np.abs(previous) > threshold_rad)
        & (np.abs(following) > threshold_rad)
        & (previous * following > 0.0)
        & (np.abs(deviation) > threshold_rad)
    )
    result[1:-1][mask] = midpoint[mask]
    return result, mask


def _normalize_quaternions(quaternions: np.ndarray) -> np.ndarray:
    norms = np.linalg.norm(quaternions, axis=1)
    if np.any(norms < 1e-12) or not np.isfinite(norms).all():
        raise MotionConditioningError("quaternion filtering produced an invalid orientation")
    return _align_quaternion_signs(quaternions / norms[:, None])


def _slerp_pair(first: np.ndarray, second: np.ndarray, fraction: float) -> np.ndarray:
    dot = float(np.dot(first, second))
    if dot < 0.0:
        second = -second
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        value = first + fraction * (second - first)
        return value / np.linalg.norm(value)
    angle = math.acos(dot)
    sine = math.sin(angle)
    return (
        math.sin((1.0 - fraction) * angle) / sine * first
        + math.sin(fraction * angle) / sine * second
    )


def _pchip_slopes(values: np.ndarray) -> np.ndarray:
    """Return monotonicity-preserving C1 slopes for uniformly spaced samples."""
    delta = np.diff(values, axis=0)
    slopes = np.empty_like(values)
    slopes[0] = delta[0]
    slopes[-1] = delta[-1]
    previous = delta[:-1]
    following = delta[1:]
    same_direction = previous * following > 0.0
    safe_previous = np.where(same_direction, previous, 1.0)
    safe_following = np.where(same_direction, following, 1.0)
    harmonic = 2.0 / (1.0 / safe_previous + 1.0 / safe_following)
    slopes[1:-1] = np.where(same_direction, harmonic, 0.0)
    return slopes


def _resample_qpos(
    qpos: np.ndarray,
    source_indices: np.ndarray,
    *,
    smooth_linear_columns: bool = True,
) -> np.ndarray:
    lower = np.floor(source_indices).astype(np.int64)
    upper = np.minimum(lower + 1, len(qpos) - 1)
    fraction = source_indices - lower
    output = np.empty((len(source_indices), QPOS_WIDTH), dtype=np.float64)
    linear_columns = np.r_[0:3, 7:QPOS_WIDTH]
    first = qpos[lower][:, linear_columns]
    second = qpos[upper][:, linear_columns]
    if smooth_linear_columns:
        slopes = _pchip_slopes(qpos[:, linear_columns])
        alpha = fraction[:, None]
        alpha_squared = alpha * alpha
        alpha_cubed = alpha_squared * alpha
        output[:, linear_columns] = (
            (2.0 * alpha_cubed - 3.0 * alpha_squared + 1.0) * first
            + (alpha_cubed - 2.0 * alpha_squared + alpha) * slopes[lower]
            + (-2.0 * alpha_cubed + 3.0 * alpha_squared) * second
            + (alpha_cubed - alpha_squared) * slopes[upper]
        )
    else:
        output[:, linear_columns] = (
            first * (1.0 - fraction[:, None]) + second * fraction[:, None]
        )
    output[:, 3:7] = np.stack(
        [
            _slerp_pair(qpos[lo, 3:7], qpos[hi, 3:7], float(alpha))
            for lo, hi, alpha in zip(lower, upper, fraction)
        ],
        axis=0,
    )
    output[:, 3:7] = _normalize_quaternions(output[:, 3:7])
    return output


def _limit_metrics(
    joints: np.ndarray,
    fps: float,
    velocity_caps: np.ndarray,
    acceleration_caps: np.ndarray,
    joint_names: Sequence[str],
) -> dict[str, Any]:
    velocity = np.abs(np.diff(joints, axis=0)) * fps
    acceleration = np.abs(np.diff(joints, n=2, axis=0)) * fps**2

    def summarize(values: np.ndarray, caps: np.ndarray) -> dict[str, Any]:
        tolerance_caps = caps * (1.0 + 1e-9)
        mask = values > tolerance_caps[None, :]
        maxima = np.max(values, axis=0) if len(values) else np.zeros_like(caps)
        per_joint: dict[str, Any] = {}
        for column, name in enumerate(joint_names):
            count = int(np.count_nonzero(mask[:, column]))
            if count:
                per_joint[name] = {
                    "cap": float(caps[column]),
                    "max": float(maxima[column]),
                    "max_ratio": float(maxima[column] / caps[column]),
                    "samples_over_cap": count,
                }
        return {
            "samples_over_cap": int(np.count_nonzero(mask)),
            "frames_with_violation": int(np.count_nonzero(np.any(mask, axis=1))),
            "max_ratio": float(np.max(maxima / caps)) if len(maxima) else 0.0,
            "joints": per_joint,
        }

    return {
        "velocity_rad_s": summarize(velocity, velocity_caps),
        "acceleration_rad_s2": summarize(acceleration, acceleration_caps),
    }


def _required_scale(metrics: Mapping[str, Any]) -> float:
    velocity_ratio = float(metrics["velocity_rad_s"]["max_ratio"])
    acceleration_ratio = float(metrics["acceleration_rad_s2"]["max_ratio"])
    return max(1.0, velocity_ratio, math.sqrt(acceleration_ratio))


def _change_metrics(output: np.ndarray, raw_at_same_phase: np.ndarray) -> dict[str, float]:
    joint_change = output[:, 7:] - raw_at_same_phase[:, 7:]
    root_change = output[:, :3] - raw_at_same_phase[:, :3]
    dots = np.abs(np.sum(output[:, 3:7] * raw_at_same_phase[:, 3:7], axis=1))
    angles = 2.0 * np.arccos(np.clip(dots, -1.0, 1.0))
    return {
        "joint_rms_rad": float(np.sqrt(np.mean(np.square(joint_change)))),
        "joint_max_abs_rad": float(np.max(np.abs(joint_change))),
        "root_position_rms_m": float(np.sqrt(np.mean(np.square(root_change)))),
        "root_position_max_abs_m": float(np.max(np.abs(root_change))),
        "root_orientation_rms_rad": float(np.sqrt(np.mean(np.square(angles)))),
        "root_orientation_max_rad": float(np.max(angles)),
    }


def condition_g1_motion(
    qpos: Any,
    fps: Any,
    *,
    velocity_caps: Mapping[str, float],
    acceleration_caps: Mapping[str, float],
    config: ConditioningConfig | None = None,
    joint_names: Sequence[str] = G1_29DOF_JOINTS,
    position_limits: Mapping[str, Sequence[float]] | None = None,
) -> ConditionedMotion:
    """Condition a G1 qpos trajectory without modifying the caller's input.

    Velocity and acceleration limits are mappings keyed by joint name.  The
    function rejects incomplete or extra mappings so a column-order mistake
    cannot silently apply a cap to the wrong actuator.  ``position_limits``
    (optional, ``{joint: (lower, upper)}`` in rad, same strictness) enables
    the filter-overshoot clamp described in the module docstring; if the
    output still has a violation that the source did not, conditioning
    raises instead of returning it.
    """
    settings = config or ConditioningConfig()
    source, input_fps = _validate_qpos(qpos, fps)
    if tuple(joint_names) != G1_29DOF_JOINTS:
        raise MotionConditioningError(
            "joint_names must match the canonical G1 29-DoF qpos order"
        )
    cutoff, spike_threshold = _validate_config(settings, input_fps)
    velocity = _validate_caps(velocity_caps, joint_names, "velocity_caps")
    acceleration = _validate_caps(acceleration_caps, joint_names, "acceleration_caps")
    position_bounds = (
        None if position_limits is None
        else _validate_position_limits(position_limits, joint_names)
    )

    working = source.copy()
    cleaned_joints, spike_mask = _replace_one_frame_joint_spikes(
        working[:, 7:], spike_threshold
    )
    working[:, 7:] = cleaned_joints

    kernel = _fir_kernel(input_fps, cutoff, settings.filter_taps)
    working[:, :3] = _zero_phase_fir(working[:, :3], kernel)
    aligned_root = _align_quaternion_signs(working[:, 3:7])
    working[:, 3:7] = _normalize_quaternions(_zero_phase_fir(aligned_root, kernel))
    working[:, 7:] = _zero_phase_fir(working[:, 7:], kernel)

    factor = settings.decimation
    base = working[::factor].copy()
    output_fps = input_fps / factor
    if len(base) < 3:
        raise MotionConditioningError(
            "decimation leaves fewer than 3 frames; choose a smaller factor"
        )
    position_report: dict[str, Any] | None = None
    if position_bounds is not None:
        lower, upper = position_bounds
        # Pre-filter (spike-cleaned) value of each kept sample: an excursion
        # is filter overshoot only if this was inside the limits.
        source_joints = cleaned_joints[::factor]
        source_inside = (source_joints >= lower[None, :]) & (source_joints <= upper[None, :])
        overshoot = source_inside & (
            (base[:, 7:] > upper[None, :]) | (base[:, 7:] < lower[None, :])
        )
        clamped = np.clip(base[:, 7:], lower[None, :], upper[None, :])
        base[:, 7:] = np.where(overshoot, clamped, base[:, 7:])
        position_report = {
            "limits_rad": {
                name: [float(lower[index]), float(upper[index])]
                for index, name in enumerate(joint_names)
            },
            "rule": "clamp_filter_overshoot_only; source violations are kept and reported",
            "filter_overshoot_samples_clamped": int(np.count_nonzero(overshoot)),
            "source_violations": _position_violation_summary(
                source[:, 7:], lower, upper, joint_names
            ),
        }
    before_scaling = _limit_metrics(
        base[:, 7:], output_fps, velocity, acceleration, joint_names
    )
    requested_scale = _required_scale(before_scaling) * settings.limit_margin
    if requested_scale > settings.max_time_scale:
        raise MotionConditioningError(
            f"required time scale {requested_scale:.6g} exceeds max_time_scale "
            f"{settings.max_time_scale:g}"
        )

    # Recompute from the same base trajectory each pass.  Checking the actual
    # sampled output (rather than trusting only 1/s and 1/s^2 estimates) keeps
    # this fail-closed around interpolation/sample-boundary effects.
    scale = max(1.0, requested_scale)
    output = base
    source_indices = np.arange(len(base), dtype=np.float64)
    remaining: dict[str, Any] | None = None
    for _attempt in range(12):
        frames = int(math.ceil((len(base) - 1) * scale)) + 1
        if frames > settings.max_output_frames:
            raise MotionConditioningError(
                f"time scaling would create {frames} frames, above max_output_frames "
                f"{settings.max_output_frames}"
            )
        source_indices = np.linspace(0.0, len(base) - 1, frames)
        output = _resample_qpos(base, source_indices)
        remaining = _limit_metrics(
            output[:, 7:], output_fps, velocity, acceleration, joint_names
        )
        if (
            remaining["velocity_rad_s"]["samples_over_cap"] == 0
            and remaining["acceleration_rad_s2"]["samples_over_cap"] == 0
        ):
            break
        next_scale = scale * _required_scale(remaining) * settings.limit_margin
        if scale >= settings.max_time_scale:
            raise MotionConditioningError(
                f"sampled output still violates caps and requires time scale > "
                f"{settings.max_time_scale:g}"
            )
        # Test the configured ceiling once before failing; an estimate just
        # above it is not proof that the sampled trajectory fails at it.
        scale = min(next_scale, settings.max_time_scale)
    else:
        raise MotionConditioningError("could not satisfy motion caps after 12 scaling passes")

    assert remaining is not None
    if position_report is not None:
        lower, upper = position_bounds
        output_violations = _position_violation_summary(
            output[:, 7:], lower, upper, joint_names
        )
        position_report["output_violations"] = output_violations
        if (
            output_violations["samples_outside_limit"]
            and not position_report["source_violations"]["samples_outside_limit"]
        ):
            # Fail closed: the source was entirely in range, so any violation
            # here was introduced by conditioning itself.
            raise MotionConditioningError(
                "conditioning introduced joint position-limit violations "
                f"({output_violations['samples_outside_limit']} samples) into an "
                "in-range source"
            )
    actual_scale = (len(output) - 1) / (len(base) - 1)
    raw_source_indices = source_indices * factor
    raw_at_same_phase = _resample_qpos(
        source, raw_source_indices, smooth_linear_columns=False
    )
    spike_points = np.argwhere(spike_mask)
    spike_by_joint = {
        name: int(np.count_nonzero(spike_mask[:, index]))
        for index, name in enumerate(joint_names)
        if np.any(spike_mask[:, index])
    }
    input_duration = (len(source) - 1) / input_fps
    base_duration = (len(base) - 1) / output_fps
    output_duration = (len(output) - 1) / output_fps
    report: dict[str, Any] = {
        "schema_version": CONDITIONING_VERSION,
        "purpose": "offline_motion_conditioning_not_calibration",
        "input": {
            "frames": len(source),
            "fps": input_fps,
            "duration_s": input_duration,
            "quaternion_order": "wxyz",
            "joint_names": list(joint_names),
        },
        "config": asdict(settings),
        "filter": {
            "type": "windowed_sinc_fir",
            "cutoff_hz": cutoff,
            "taps": settings.filter_taps,
            "application": "offline_centered_zero_phase",
            "introduced_phase_delay_s": 0.0,
            "causal_equivalent_group_delay_s": (settings.filter_taps // 2) / input_fps,
        },
        "spikes": {
            "rule": "isolated_A_B_A_excursion_replaced_by_neighbor_midpoint",
            "threshold_rad": spike_threshold,
            "joint_samples_replaced": int(len(spike_points)),
            "frames_affected": int(len(np.unique(spike_points[:, 0]))) if len(spike_points) else 0,
            "by_joint": spike_by_joint,
        },
        "decimation": {
            "factor": factor,
            "output_fps": output_fps,
            "frames_before_time_scaling": len(base),
            "tail_discarded_s": max(0.0, input_duration - base_duration),
        },
        "limits": {
            "velocity_caps_rad_s": {
                name: float(velocity[index]) for index, name in enumerate(joint_names)
            },
            "acceleration_caps_rad_s2": {
                name: float(acceleration[index]) for index, name in enumerate(joint_names)
            },
            "before_time_scaling": before_scaling,
            "remaining_violations": remaining,
            "position": position_report,
        },
        "time_scaling": {
            "method": "global_uniform",
            "factor": actual_scale,
            "requested_factor_before_frame_rounding": requested_scale,
        },
        "output": {
            "frames": len(output),
            "fps": output_fps,
            "duration_s": output_duration,
            "duration_change_s": output_duration - input_duration,
        },
        "introduced_change": _change_metrics(output, raw_at_same_phase),
    }
    return ConditionedMotion(qpos=output, fps=output_fps, report=report)


def load_qpos_npz(path: str | Path) -> tuple[np.ndarray, float, dict[str, Any]]:
    """Load and strictly validate a preview-cache NPZ."""
    source = Path(path)
    if not source.is_file():
        raise MotionConditioningError(f"input NPZ does not exist: {source}")
    try:
        with np.load(source, allow_pickle=False) as archive:
            if set(archive.files) != {"metadata", "qpos", "fps"}:
                raise MotionConditioningError(
                    "input NPZ must contain exactly metadata, qpos, and fps"
                )
            raw_metadata = archive["metadata"]
            if raw_metadata.shape != ():
                raise MotionConditioningError("metadata must be a scalar JSON string")
            metadata = json.loads(str(raw_metadata.item()))
            qpos = np.asarray(archive["qpos"], dtype=np.float64)
            fps_array = archive["fps"]
            if fps_array.shape != ():
                raise MotionConditioningError("fps must be a scalar")
            fps = float(fps_array.item())
    except MotionConditioningError:
        raise
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        raise MotionConditioningError(f"cannot read input NPZ {source}: {exc}") from exc
    if not isinstance(metadata, dict):
        raise MotionConditioningError("metadata JSON must be an object")
    if metadata.get("robot") != "unitree_g1":
        raise MotionConditioningError("metadata.robot must be 'unitree_g1'")
    if metadata.get("root_quaternion_order") != "wxyz":
        raise MotionConditioningError("metadata.root_quaternion_order must be 'wxyz'")
    validated, rate = _validate_qpos(qpos, fps)
    return validated, rate, metadata


def write_conditioned_npz(
    path: str | Path,
    conditioned: ConditionedMotion,
    source_metadata: Mapping[str, Any],
) -> None:
    """Atomically create a conditioned NPZ; refuse to overwrite any file."""
    target = Path(path)
    if target.suffix.lower() != ".npz":
        raise MotionConditioningError("conditioned output path must end in .npz")
    metadata = dict(source_metadata)
    metadata["conditioning"] = {
        "schema_version": CONDITIONING_VERSION,
        "purpose": "offline_motion_conditioning_not_calibration",
        "config": conditioned.report["config"],
        "time_scale": conditioned.report["time_scaling"]["factor"],
    }
    if "source_selection" in conditioned.report:
        metadata["conditioning"]["source_selection"] = conditioned.report[
            "source_selection"
        ]
    write_qpos_npz_atomic(
        target,
        qpos=conditioned.qpos,
        fps=conditioned.fps,
        metadata=metadata,
        exists_error=MotionConditioningError,
    )


def write_json_report(path: str | Path, report: Mapping[str, Any]) -> None:
    """Atomically create a JSON report; refuse to overwrite any file."""
    # Strict JSON (allow_nan=False) via the shared atomic writer.
    write_json_atomic(path, report, exists_error=MotionConditioningError)
