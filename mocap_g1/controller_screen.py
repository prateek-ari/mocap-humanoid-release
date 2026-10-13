"""Fixed-base MuJoCo screen for recorded G1 upper-body references.

This is deliberately narrower than a balance or hardware-controller test.  It
removes the visualization model's floating pelvis, holds the twelve leg joints
at the first selected pose, and drives waist/arm joints through bounded PD
torque plus MuJoCo bias compensation.  References are never assigned directly
to simulated joint positions.

Balance is out of scope here by construction (the pelvis is welded to the
world).  ``mocap_g1.balance_screen`` is the floating-base companion that does
look at standing stability.

Controller gains (2026-08-16)
-----------------------------
``kp``/``kd`` used to be the ASSUMED values 10.0 / 0.2, which were never
traceable to anything Unitree published.  They are now Unitree's published
``arm_sdk`` example gains, ``kp = 60.0`` and ``kd = 1.5`` -- the same pair
appears verbatim in all four official G1 arm_sdk examples and is applied to
every arm AND waist joint the SDK drives, i.e. exactly the joint set this
screen controls:

* https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/high_level/g1_arm7_sdk_dds_example.cpp
* https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/high_level/g1_arm5_sdk_dds_example.cpp
* https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/g1/high_level/g1_arm7_sdk_dds_example.py
* https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/g1/high_level/g1_arm5_sdk_dds_example.py

Units are N*m/rad and N*m*s/rad, closed by the onboard motor controller;
the examples send ``dq_des = 0`` and ``tau_ff = 0``.  ``kp``/``kd`` remain
config-driven, and the legacy pair is preserved as ``LEGACY_ASSUMED_KP`` /
``LEGACY_ASSUMED_KD`` for A/B comparison against pre-2026-08-16 reports.
Every report now carries a ``controller_gains`` block recording which pair
was used and whether it is verified.

Joint position limits (2026-10-12, report schema v2)
----------------------------------------------------
The GMR mocap MJCF narrows 15/29 G1 joint ranges for IK (e.g.
``left_shoulder_roll`` [-0.6, 2.25] vs the vendor URDF's [-1.59, 2.25]).
Conditioning (stage 2) and gate v2 (stage 7) grade against the mechanical
URDF, so the fixed-base model here overwrites every hinge ``range`` with the
URDF limits before compiling (``limits_urdf_path``; default: the vendored
official URDF) -- one limit source for the whole chain.  PD targets are
clamped to those limits: a reference sample outside them is reported as
``joint_position_limit_violation`` (``events.reference_joint_position_violations``),
not as a tracking error caused by the simulated joint stopping at a limit.
Every report records the limit file and its sha256 under ``joint_limits``.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import asdict, dataclass
import math
from pathlib import Path
from typing import Any

import numpy as np

import xml.etree.ElementTree as ET

from .assets import official_urdf_path
from .io_utils import sha256_file, sha256_text as _sha256_text
from .schema import (
    G1_29DOF_JOINTS,
    G1_ARM_SDK_JOINTS,
    JointLimit,
    load_official_joint_limits,
)


class ControllerScreenError(RuntimeError):
    """Raised when the screen cannot produce trustworthy metrics."""


#: Report schema.  v2 (2026-10-12): URDF joint limits replace the MJCF ranges,
#: PD targets are clamped to them, and ``joint_limits`` provenance is added.
SCHEMA_VERSION = 2


def resolve_urdf_joint_limits(
    limits_urdf: str | Path | None,
    error: type[Exception] = ControllerScreenError,
) -> tuple[dict[str, JointLimit], dict[str, Any]]:
    """Load the 29 G1 position limits from the mechanical URDF.

    ``None`` selects the vendored official URDF.  Returns the limits and a
    provenance record (``source``/``path``/``sha256``) for reports.  Any
    unreadable, incomplete or non-finite limit table raises ``error``.
    """
    path = Path(limits_urdf) if limits_urdf is not None else official_urdf_path()
    try:
        limits = load_official_joint_limits(path)
    except (OSError, ValueError, ET.ParseError) as exc:
        raise error(f"cannot load joint position limits from {path}: {exc}") from exc
    for name, limit in limits.items():
        if not (math.isfinite(limit.lower) and math.isfinite(limit.upper)
                and limit.lower < limit.upper):
            raise error(f"{path}: joint {name} has an invalid position range")
    provenance = {
        "source": "urdf",
        "path": str(path.resolve()),
        "sha256": sha256_file(path),
    }
    return limits, provenance


def urdf_limit_arrays(limits: dict[str, JointLimit]) -> tuple[np.ndarray, np.ndarray]:
    """(lower, upper) arrays in ``G1_29DOF_JOINTS`` column order."""
    lower = np.asarray([limits[name].lower for name in G1_29DOF_JOINTS], dtype=np.float64)
    upper = np.asarray([limits[name].upper for name in G1_29DOF_JOINTS], dtype=np.float64)
    return lower, upper


#: Unitree's published G1 ``arm_sdk`` example gains (see the module docstring
#: for the four source files these are quoted from).  N*m/rad, N*m*s/rad.
ARM_SDK_KP = 60.0
ARM_SDK_KD = 1.5

#: The pre-2026-08-16 assumed gains, kept so old reports can be reproduced.
LEGACY_ASSUMED_KP = 10.0
LEGACY_ASSUMED_KD = 0.2

ARM_SDK_GAIN_SOURCES = (
    "https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/high_level/g1_arm7_sdk_dds_example.cpp",
    "https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/high_level/g1_arm5_sdk_dds_example.cpp",
    "https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/g1/high_level/g1_arm7_sdk_dds_example.py",
    "https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/g1/high_level/g1_arm5_sdk_dds_example.py",
)


def classify_gains(kp: float, kd: float) -> dict[str, Any]:
    """Describe where a (kp, kd) pair came from, for the report."""
    if (kp, kd) == (ARM_SDK_KP, ARM_SDK_KD):
        return {
            "verification_status": "verified_published",
            "source": "unitree_sdk2 / unitree_sdk2_python G1 arm_sdk DDS examples",
            "source_urls": list(ARM_SDK_GAIN_SOURCES),
            "verified_on": "2026-08-16",
        }
    if (kp, kd) == (LEGACY_ASSUMED_KP, LEGACY_ASSUMED_KD):
        return {
            "verification_status": "legacy_assumed",
            "source": "pre-2026-08-16 screening assumption, not traceable to Unitree",
            "source_urls": [],
            "verified_on": None,
        }
    return {
        "verification_status": "operator_override",
        "source": "caller-supplied gains; provenance unknown to this module",
        "source_urls": [],
        "verified_on": None,
    }


@dataclass(frozen=True)
class ControllerScreenConfig:
    timestep_s: float = 0.002
    kp: float = ARM_SDK_KP
    kd: float = ARM_SDK_KD
    torque_limit_fraction: float = 0.50
    speed_factor: float = 1.0
    settle_seconds: float = 0.50
    tracking_p95_limit_rad: float = 0.10
    tracking_max_limit_rad: float = 0.25
    saturation_fraction_limit: float = 0.01
    position_tolerance_rad: float = 1e-6
    # ``timestep_s`` is the CONTROL/metric period.  Physics is integrated in
    # ``physics_substeps`` equal sub-steps inside it (0 = choose automatically
    # from ``pd_stability_product_max``).  Raising kp from the old assumed 10
    # to Unitree's published 60 quadrupled the wrist PD natural frequency, and
    # a single 2 ms explicit step no longer integrates it: the wrist-roll
    # joint oscillates to 0.62 rad of pure numerical error and the screen
    # would blame the motion for an integrator failure.  Sub-stepping keeps
    # the reported cadence unchanged while integrating stably.
    physics_substeps: int = 0
    # omega*dt cap, where omega = sqrt(kp / M_ii) at the initial pose.
    # Calibrated on g1_mocap_29dof.xml at kp=60 (worst joint wrist_roll,
    # M_ii = 3.664e-4 kg*m^2, omega = 404.7 rad/s): omega*dt = 0.20 tracks to
    # 0.012 rad, 0.40 oscillates at 0.24 rad, 0.81 diverges to 0.62 rad.  0.25
    # is the smallest round cap that keeps the verified-good point admissible.
    pd_stability_product_max: float = 0.25
    # Mechanical URDF whose position limits replace the MJCF's IK-narrowed
    # joint ranges (None = the vendored official URDF).  See the module
    # docstring, "Joint position limits".
    limits_urdf_path: str | None = None

    def validate(self) -> None:
        for name in (
            "timestep_s",
            "kp",
            "kd",
            "torque_limit_fraction",
            "speed_factor",
            "tracking_p95_limit_rad",
            "tracking_max_limit_rad",
            "saturation_fraction_limit",
            "position_tolerance_rad",
            "pd_stability_product_max",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ControllerScreenError(f"{name} must be finite and positive")
        if int(self.physics_substeps) < 0:
            raise ControllerScreenError("physics_substeps must be non-negative (0 = automatic)")
        if self.torque_limit_fraction > 1.0:
            raise ControllerScreenError("torque_limit_fraction must be <= 1")
        if self.saturation_fraction_limit > 1.0:
            raise ControllerScreenError("saturation_fraction_limit must be <= 1")
        if not math.isfinite(float(self.settle_seconds)) or self.settle_seconds < 0.0:
            raise ControllerScreenError("settle_seconds must be finite and non-negative")


def _require_mujoco() -> Any:
    try:
        import mujoco
    except ImportError as exc:  # pragma: no cover - depends on optional runtime
        raise ControllerScreenError(
            "MuJoCo is required; install the project's 'runtime' optional dependencies"
        ) from exc
    return mujoco


def _build_fixed_model(
    model_path: Path,
    config: ControllerScreenConfig,
) -> tuple[Any, str, np.ndarray, np.ndarray, np.ndarray]:
    mujoco = _require_mujoco()
    if not model_path.is_file():
        raise ControllerScreenError(f"MuJoCo model does not exist: {model_path}")
    try:
        spec = mujoco.MjSpec.from_file(str(model_path))
    except Exception as exc:
        raise ControllerScreenError(f"cannot parse MuJoCo model {model_path}: {exc}") from exc

    free_joints = [
        joint for joint in spec.joints if joint.type == mujoco.mjtJoint.mjJNT_FREE
    ]
    if len(free_joints) != 1 or free_joints[0].name != "pelvis":
        raise ControllerScreenError("model must contain exactly one free joint named 'pelvis'")
    spec.delete(free_joints[0])
    spec.option.timestep = config.timestep_s
    # The wrist links have very small inertias.  Explicit Euler can become
    # numerically unstable under even moderate PD damping, which would turn a
    # controller screen into an integrator screen.
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST

    missing_joints = [name for name in G1_29DOF_JOINTS if spec.joint(name) is None]
    missing_actuators = [name for name in G1_29DOF_JOINTS if spec.actuator(name) is None]
    if missing_joints or missing_actuators:
        raise ControllerScreenError(
            f"model lacks canonical joints={missing_joints}, actuators={missing_actuators}"
        )

    force_limits = np.empty((len(G1_29DOF_JOINTS), 2), dtype=np.float64)
    for index, name in enumerate(G1_29DOF_JOINTS):
        joint = spec.joint(name)
        actuator = spec.actuator(name)
        force_range = np.asarray(joint.actfrcrange, dtype=np.float64)
        if force_range.shape != (2,) or not np.isfinite(force_range).all():
            raise ControllerScreenError(f"{name} lacks a finite actuator force range")
        if not force_range[0] < 0.0 < force_range[1]:
            raise ControllerScreenError(f"{name} actuator force range must straddle zero")
        applied = force_range * config.torque_limit_fraction
        actuator.ctrlrange = applied
        actuator.ctrllimited = True
        force_limits[index] = applied

    # One limit source for the whole chain: the mechanical URDF, never the
    # MJCF's IK-narrowed ranges (module docstring, "Joint position limits").
    limits, _provenance = resolve_urdf_joint_limits(config.limits_urdf_path)
    for name in G1_29DOF_JOINTS:
        joint = spec.joint(name)
        joint.range = [limits[name].lower, limits[name].upper]
        joint.limited = mujoco.mjtLimited.mjLIMITED_TRUE

    derived_xml = spec.to_xml()
    try:
        model = spec.compile()
    except Exception as exc:
        raise ControllerScreenError(f"cannot compile fixed-base MuJoCo model: {exc}") from exc
    if model.nq != len(G1_29DOF_JOINTS) or model.nv != len(G1_29DOF_JOINTS):
        raise ControllerScreenError(
            f"fixed model must have 29 qpos/qvel, got nq={model.nq}, nv={model.nv}"
        )

    joint_ids = np.asarray(
        [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in G1_29DOF_JOINTS],
        dtype=np.int32,
    )
    actuator_ids = np.asarray(
        [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in G1_29DOF_JOINTS],
        dtype=np.int32,
    )
    if np.any(joint_ids < 0) or np.any(actuator_ids < 0):
        raise ControllerScreenError("compiled model lost canonical joint/actuator names")
    qpos_addresses = model.jnt_qposadr[joint_ids]
    dof_addresses = model.jnt_dofadr[joint_ids]
    if len(set(map(int, qpos_addresses))) != len(qpos_addresses):
        raise ControllerScreenError("canonical joints do not map to distinct qpos addresses")
    return model, derived_xml, actuator_ids, qpos_addresses, dof_addresses


def validate_reference_qpos(
    qpos: Any, fps: Any, error: type[Exception] = ControllerScreenError
) -> tuple[np.ndarray, float]:
    """Validate a floating-base [T>=3,36] qpos reference and its frame rate.

    Shared by every screen that drives a recorded reference (this module,
    collision_gate, contact_refine, balance_screen); ``error`` is the
    caller's own exception type.  Requires finite values, unit root
    quaternions (|norm-1| <= 1e-3) and a finite positive fps.
    """
    values = np.asarray(qpos, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 36 or len(values) < 3:
        raise error("reference qpos must have shape [T>=3,36]")
    if not np.isfinite(values).all():
        raise error("reference qpos contains non-finite values")
    norms = np.linalg.norm(values[:, 3:7], axis=1)
    if np.max(np.abs(norms - 1.0)) > 1e-3:
        raise error("reference contains non-unit root quaternions")
    rate = float(fps)
    if not math.isfinite(rate) or rate <= 0.0:
        raise error("reference fps must be finite and positive")
    return values, rate


def _validate_reference(qpos: Any, fps: Any) -> tuple[np.ndarray, float]:
    return validate_reference_qpos(qpos, fps, ControllerScreenError)


def _per_joint_stats(values: np.ndarray) -> dict[str, dict[str, float]]:
    absolute = np.abs(values)
    return {
        name: {
            "rms_rad": float(np.sqrt(np.mean(np.square(values[:, column])))),
            "p95_abs_rad": float(np.percentile(absolute[:, column], 95)),
            "max_abs_rad": float(np.max(absolute[:, column])),
        }
        for column, name in enumerate(G1_ARM_SDK_JOINTS)
    }


def _current_self_collisions(model: Any, data: Any, mujoco: Any) -> dict[tuple[str, str], float]:
    pairs: dict[tuple[str, str], float] = {}
    for contact_index in range(data.ncon):
        contact = data.contact[contact_index]
        if contact.dist >= 0.0:
            continue
        body1 = int(model.geom_bodyid[contact.geom1])
        body2 = int(model.geom_bodyid[contact.geom2])
        if body1 == 0 or body2 == 0 or body1 == body2:
            continue
        name1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body1)
        name2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body2)
        if name1 is None or name2 is None:
            raise ControllerScreenError("self-collision contact references an unnamed body")
        pair = tuple(sorted((name1, name2)))
        pairs[pair] = min(pairs.get(pair, math.inf), float(contact.dist))
    return pairs


def scan_reference_self_collisions(
    qpos: Any,
    fps: Any,
    *,
    model_path: str | Path,
    window_seconds: float = 10.0,
    config: ControllerScreenConfig | None = None,
    source_sha256: str | None = None,
    source_metadata: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Check every kinematic reference frame against the full fixed-base MJCF."""
    settings = config or ControllerScreenConfig()
    settings.validate()
    reference, rate = _validate_reference(qpos, fps)
    if not math.isfinite(float(window_seconds)) or window_seconds <= 0.0:
        raise ControllerScreenError("window_seconds must be finite and positive")
    model_file = Path(model_path).resolve()
    model, derived_xml, _actuators, qadr, dadr = _build_fixed_model(model_file, settings)
    mujoco = _require_mujoco()
    data = mujoco.MjData(model)

    collision_frames = np.zeros(len(reference), dtype=bool)
    pair_frames: dict[tuple[str, str], int] = defaultdict(int)
    pair_min_distance: dict[tuple[str, str], float] = {}
    for frame, pose in enumerate(reference):
        data.qpos[qadr] = pose[7:]
        data.qvel[dadr] = 0.0
        mujoco.mj_forward(model, data)
        pairs = _current_self_collisions(model, data, mujoco)
        collision_frames[frame] = bool(pairs)
        for pair, distance in pairs.items():
            pair_frames[pair] += 1
            pair_min_distance[pair] = min(
                pair_min_distance.get(pair, math.inf), distance
            )

    window_frames = max(1, int(round(window_seconds * rate)))
    windows = []
    for index, start in enumerate(range(0, len(reference), window_frames)):
        end = min(start + window_frames, len(reference))
        count = int(np.count_nonzero(collision_frames[start:end]))
        windows.append(
            {
                "window_index": index,
                "start_frame": start,
                "end_frame_exclusive": end,
                "start_time_s": start / rate,
                "end_time_s": end / rate,
                "frame_count": end - start,
                "self_collision_frames": count,
                "self_collision_fraction": count / (end - start),
                "status": "FAIL" if count else "PASS",
            }
        )

    longest_start = longest_end = current_start = 0
    in_run = False
    for index, collided in enumerate(collision_frames):
        if not collided and not in_run:
            current_start = index
            in_run = True
        if in_run and (collided or index == len(collision_frames) - 1):
            current_end = index if collided else index + 1
            if current_end - current_start > longest_end - longest_start:
                longest_start, longest_end = current_start, current_end
            in_run = False

    count = int(np.count_nonzero(collision_frames))
    metadata = source_metadata or {}
    _limits, limit_provenance = resolve_urdf_joint_limits(settings.limits_urdf_path)
    return {
        "schema_version": SCHEMA_VERSION,
        "purpose": "fixed_base_kinematic_self_collision_audit_not_calibration",
        "joint_limits": limit_provenance,
        "summary": {
            "status": "FAIL" if count else "PASS",
            "frames": len(reference),
            "self_collision_frames": count,
            "self_collision_fraction": count / len(reference),
            "collision_free_frames": len(reference) - count,
            "longest_collision_free_run": {
                "start_frame": longest_start,
                "end_frame_exclusive": longest_end,
                "start_time_s": longest_start / rate,
                "end_time_s": longest_end / rate,
                "duration_s": (longest_end - longest_start) / rate,
            },
        },
        "input": {
            "fps": rate,
            "duration_s": len(reference) / rate,
            "source_sha256": source_sha256,
            "offsets_key": metadata.get("offsets_key"),
        },
        "configuration": {
            "window_seconds_requested": window_seconds,
            "window_frames": window_frames,
            "root_handling": "ignored_for_self_collision; pelvis fixed at MJCF pose",
        },
        "model": {
            "source_path": str(model_file),
            "source_sha256": sha256_file(model_file),
            "derived_fixed_base_xml_sha256": _sha256_text(derived_xml),
        },
        "collision_pairs": [
            {
                "body_a": pair[0],
                "body_b": pair[1],
                "frames": pair_frames[pair],
                "minimum_contact_distance_m": pair_min_distance[pair],
            }
            for pair in sorted(pair_frames, key=lambda item: (-pair_frames[item], item))
        ],
        "windows": windows,
        "limitations": [
            "this is a kinematic geometry audit; actuator tracking and balance are not evaluated",
            "the pelvis is fixed and world/ground contacts are excluded",
            "the result inherits the reference calibration quality",
        ],
    }


def run_upper_body_screen(
    qpos: Any,
    fps: Any,
    *,
    model_path: str | Path,
    config: ControllerScreenConfig | None = None,
    source_sha256: str | None = None,
    source_metadata: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Track a recorded upper-body reference through bounded MuJoCo torque control."""
    settings = config or ControllerScreenConfig()
    settings.validate()
    reference, rate = _validate_reference(qpos, fps)
    model_file = Path(model_path).resolve()
    model, derived_xml, actuator_ids, qadr, dadr = _build_fixed_model(model_file, settings)
    mujoco = _require_mujoco()
    data = mujoco.MjData(model)

    # Reference vs the URDF limits the model was just compiled with.  Samples
    # outside them are a joint-limit finding about the REFERENCE; the PD
    # target is clamped to the limit so the simulated joint stopping there is
    # not misreported as a tracking failure.
    limits, limit_provenance = resolve_urdf_joint_limits(settings.limits_urdf_path)
    limit_lower, limit_upper = urdf_limit_arrays(limits)
    joints = reference[:, 7:]
    reference_outside = (
        (joints < limit_lower - settings.position_tolerance_rad)
        | (joints > limit_upper + settings.position_tolerance_rad)
    )
    reference_excess = np.maximum(
        np.maximum(limit_lower - joints, joints - limit_upper), 0.0
    ).max(axis=0)
    reference_violation_frames = int(np.count_nonzero(np.any(reference_outside, axis=1)))
    reference = reference.copy()
    reference[:, 7:] = np.clip(joints, limit_lower, limit_upper)

    initial = reference[0, 7:].copy()
    data.qpos[qadr] = initial
    data.qvel[dadr] = 0.0
    mujoco.mj_forward(model, data)

    initial_collision_pairs = _current_self_collisions(model, data, mujoco)

    # Resolve the physics sub-step count from the PD natural frequency of the
    # stiffest-to-integrate joint (see ControllerScreenConfig.physics_substeps).
    inertia = np.zeros((model.nv, model.nv), dtype=np.float64)
    mujoco.mj_fullM(model, data, inertia)
    joint_inertia = np.diag(inertia)[np.asarray(dadr, dtype=np.int64)]
    if not np.isfinite(joint_inertia).all() or np.any(joint_inertia <= 0.0):
        raise ControllerScreenError("model has a non-positive joint-space inertia diagonal")
    natural_frequencies = np.sqrt(settings.kp / joint_inertia)
    worst_frequency_index = int(np.argmax(natural_frequencies))
    worst_frequency = float(natural_frequencies[worst_frequency_index])
    if settings.physics_substeps:
        substeps = int(settings.physics_substeps)
    else:
        substeps = max(
            1,
            int(
                math.ceil(
                    settings.timestep_s * worst_frequency / settings.pd_stability_product_max
                )
            ),
        )
    physics_timestep = settings.timestep_s / substeps
    model.opt.timestep = physics_timestep

    controlled_columns = np.arange(12, len(G1_29DOF_JOINTS), dtype=np.int32)
    applied_lower = model.actuator_ctrlrange[actuator_ids, 0]
    applied_upper = model.actuator_ctrlrange[actuator_ids, 1]
    joint_ranges = model.jnt_range[
        np.asarray([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in G1_29DOF_JOINTS])
    ]

    def control(target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        position = data.qpos[qadr]
        velocity = data.qvel[dadr]
        requested = settings.kp * (target - position) - settings.kd * velocity
        requested += data.qfrc_bias[dadr]
        clipped = np.clip(requested, applied_lower, applied_upper)
        data.ctrl[actuator_ids] = clipped
        return requested, clipped

    # One control period = `substeps` physics steps, with the PD law
    # recomputed on each of them (a real motor controller closes its loop
    # faster than the reference is resampled).
    settle_steps = int(math.ceil(settings.settle_seconds / settings.timestep_s))
    for _ in range(settle_steps * substeps):
        control(initial)
        mujoco.mj_step(model, data)

    duration = (len(reference) - 1) / rate / settings.speed_factor
    steps = int(math.ceil(duration / settings.timestep_s)) + 1
    errors = np.empty((steps, len(controlled_columns)), dtype=np.float64)
    torque_ratios = np.empty((steps, len(controlled_columns)), dtype=np.float64)
    saturated = np.zeros((steps, len(controlled_columns)), dtype=bool)
    position_violation_steps = 0
    position_violation_samples = np.zeros(len(G1_29DOF_JOINTS), dtype=np.int64)
    position_max_excess = np.zeros(len(G1_29DOF_JOINTS), dtype=np.float64)
    self_collision_steps = 0
    collision_pair_steps: dict[tuple[str, str], int] = defaultdict(int)
    collision_pair_min_distance: dict[tuple[str, str], float] = {}

    for step in range(steps):
        source_index = min(
            step * settings.timestep_s * rate * settings.speed_factor,
            len(reference) - 1,
        )
        lower = int(math.floor(source_index))
        upper = min(lower + 1, len(reference) - 1)
        alpha = source_index - lower
        moving = (1.0 - alpha) * reference[lower, 7:] + alpha * reference[upper, 7:]
        target = initial.copy()
        target[controlled_columns] = moving[controlled_columns]
        requested, clipped = control(target)

        actual = data.qpos[qadr]
        errors[step] = target[controlled_columns] - actual[controlled_columns]
        caps = np.maximum(np.abs(applied_lower), np.abs(applied_upper))
        torque_ratios[step] = np.abs(clipped[controlled_columns]) / caps[controlled_columns]
        saturated[step] = (
            (requested[controlled_columns] < applied_lower[controlled_columns] - 1e-12)
            | (requested[controlled_columns] > applied_upper[controlled_columns] + 1e-12)
        )
        outside = (
            actual < joint_ranges[:, 0] - settings.position_tolerance_rad
        ) | (
            actual > joint_ranges[:, 1] + settings.position_tolerance_rad
        )
        position_violation_steps += int(np.any(outside))
        position_violation_samples += outside
        excess = np.maximum(
            np.maximum(joint_ranges[:, 0] - actual, actual - joint_ranges[:, 1]),
            0.0,
        )
        position_max_excess = np.maximum(position_max_excess, excess)

        step_pairs = _current_self_collisions(model, data, mujoco)
        self_collision_steps += int(bool(step_pairs))
        for pair, distance in step_pairs.items():
            collision_pair_steps[pair] += 1
            collision_pair_min_distance[pair] = min(
                collision_pair_min_distance.get(pair, math.inf), distance
            )
        mujoco.mj_step(model, data)
        for _ in range(substeps - 1):
            control(target)
            mujoco.mj_step(model, data)

    per_joint = _per_joint_stats(errors)
    absolute_error = np.abs(errors)
    p95 = float(np.percentile(absolute_error, 95))
    maximum = float(np.max(absolute_error))
    saturation_fraction = float(np.mean(saturated))
    reasons: list[str] = []
    if p95 > settings.tracking_p95_limit_rad:
        reasons.append("tracking_p95_exceeded")
    if maximum > settings.tracking_max_limit_rad:
        reasons.append("tracking_max_exceeded")
    if saturation_fraction > settings.saturation_fraction_limit:
        reasons.append("torque_saturation_exceeded")
    if position_violation_steps or reference_violation_frames:
        reasons.append("joint_position_limit_violation")
    if initial_collision_pairs or self_collision_steps:
        reasons.append("self_collision")

    metadata = source_metadata or {}
    preview_calibration = metadata.get("offsets_key") == "zero-preview"
    return {
        "schema_version": SCHEMA_VERSION,
        "purpose": "fixed_base_upper_body_controller_screen_not_balance_validation",
        "joint_limits": {
            **limit_provenance,
            "applied_to": "fixed-base model joint ranges and PD target clamp",
        },
        "controller_screen": {
            "status": "FAIL" if reasons else "PASS",
            "reasons": reasons,
        },
        "hardware_candidate": {
            "status": "BLOCKED" if preview_calibration or reasons else "CONTROLLER_SCREEN_ONLY",
            "reasons": (["preview_calibration"] if preview_calibration else []) + reasons,
        },
        "input": {
            "frames": len(reference),
            "fps": rate,
            "duration_s": (len(reference) - 1) / rate,
            "source_sha256": source_sha256,
            "offsets_key": metadata.get("offsets_key"),
        },
        "configuration": asdict(settings),
        "controller_gains": {
            "kp": settings.kp,
            "kd": settings.kd,
            "feedforward": "MuJoCo qfrc_bias (gravity/Coriolis compensation)",
            **classify_gains(settings.kp, settings.kd),
        },
        "numerics": {
            "control_period_s": settings.timestep_s,
            "physics_substeps": substeps,
            "physics_timestep_s": physics_timestep,
            "physics_substeps_mode": "explicit" if settings.physics_substeps else "automatic",
            "pd_natural_frequency_max_rad_s": worst_frequency,
            "pd_natural_frequency_worst_joint": G1_29DOF_JOINTS[worst_frequency_index],
            "pd_stability_product": worst_frequency * physics_timestep,
            "pd_stability_product_max": settings.pd_stability_product_max,
        },
        "model": {
            "source_path": str(model_file),
            "source_sha256": sha256_file(model_file),
            "derived_fixed_base_xml_sha256": _sha256_text(derived_xml),
            "base": "pelvis_freejoint_removed",
            "legs": "bounded_PD_hold_at_first_selected_reference",
            "controlled_joints": list(G1_ARM_SDK_JOINTS),
        },
        "tracking": {
            "pooled_rms_rad": float(np.sqrt(np.mean(np.square(errors)))),
            "pooled_p95_abs_rad": p95,
            "pooled_max_abs_rad": maximum,
            "per_joint": per_joint,
        },
        "actuation": {
            "torque_saturation_sample_fraction": saturation_fraction,
            "steps_with_any_saturation": int(np.count_nonzero(np.any(saturated, axis=1))),
            "max_applied_torque_limit_ratio": float(np.max(torque_ratios)),
            "p99_applied_torque_limit_ratio": float(np.percentile(torque_ratios, 99)),
            "applied_ctrlrange_by_joint": {
                name: [float(applied_lower[index]), float(applied_upper[index])]
                for index, name in enumerate(G1_29DOF_JOINTS)
            },
        },
        "events": {
            "simulation_steps": steps,
            "reference_joint_position_violation_frames": reference_violation_frames,
            "reference_joint_position_violations": [
                {
                    "joint": name,
                    "frames": int(np.count_nonzero(reference_outside[:, index])),
                    "max_excess_rad": float(reference_excess[index]),
                }
                for index, name in enumerate(G1_29DOF_JOINTS)
                if np.any(reference_outside[:, index])
            ],
            "joint_position_violation_steps": position_violation_steps,
            "joint_position_violations": [
                {
                    "joint": name,
                    "samples": int(position_violation_samples[index]),
                    "max_excess_rad": float(position_max_excess[index]),
                }
                for index, name in enumerate(G1_29DOF_JOINTS)
                if position_violation_samples[index]
            ],
            "initial_pose_self_collision_pairs": [
                {
                    "body_a": pair[0],
                    "body_b": pair[1],
                    "contact_distance_m": initial_collision_pairs[pair],
                }
                for pair in sorted(initial_collision_pairs)
            ],
            "self_collision_steps": self_collision_steps,
            "self_collision_pairs": [
                {
                    "body_a": pair[0],
                    "body_b": pair[1],
                    "steps": collision_pair_steps[pair],
                    "minimum_contact_distance_m": collision_pair_min_distance[pair],
                }
                for pair in sorted(collision_pair_steps)
            ],
        },
        "limitations": [
            "pelvis is fixed; balance, falls and stepping are not evaluated -- see mocap_g1.balance_screen",
            "legs hold the first pose through simulated PD rather than a factory balance controller",
            "PD gains are Unitree's published arm_sdk example values, but this simulation adds "
            "qfrc_bias feedforward and a torque fraction the SDK does not; both remain screening assumptions",
            "thermal, battery, communication and real motor dynamics are not modeled",
        ],
    }
