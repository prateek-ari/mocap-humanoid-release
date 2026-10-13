"""Floating-base standing-balance SCREEN for recorded G1 arm references.

What this is
------------
``mocap_g1.controller_screen`` deletes the pelvis free joint and therefore
cannot say anything about balance: the project's top physical risk ("arm motion
destabilizes the standing robot") had zero simulation evidence.  This module
closes that gap the cheapest honest way: it loads the SAME MJCF with the
pelvis free joint INTACT, plants the robot on the model's ground plane in a
documented standing stance, holds legs and waist there with stiff joint-space
PD, drives the fourteen arm joints along the recorded reference with Unitree's
published ``arm_sdk`` gains, and runs real ``mj_step`` physics.

What this is NOT
----------------
This is NOT balance validation and NOT a hardware go/no-go.  A stiff
joint-space PD is a crude surrogate for the factory standing controller:

* The factory controller closes a loop on CoM / ZMP / base attitude and can
  take a recovery step.  A joint PD has no such authority -- it can only
  resist motion that shows up as a JOINT angle error.  A standing humanoid is
  an inverted pendulum whose gravitational stiffness about the ankle is
  ``m*g*h_com`` ~ 236 N*m/rad for this model (34.4 kg, CoM ~0.70 m); the
  ankle-pitch stiffness sum has to exceed that or the model simply topples,
  which is why ``ankle_kp`` defaults to 600 N*m/rad per ankle -- roughly 15x
  Unitree's published deployment value (40 N*m/rad in
  ``unitree_rl_gym/deploy/deploy_real/configs/g1.yaml``).  That gain is a
  SCREENING SURROGATE for the missing balance controller, not a claim about
  G1 firmware.
* Consequence: the screen is CONSERVATIVE in the sagittal plane (it has no
  hip/stepping recovery strategy, so it will fail motions a real G1 may well
  survive) and OPTIMISTIC about actuator dynamics (ideal torque sources, no
  transport delay, no 50 Hz command quantisation, no motor thermal model).
  A FAIL verdict means "do not put this on hardware without more work";
  a PASS verdict means only "this clip did not topple the crude surrogate".

Status vocabulary (report schema v2, 2026-10-12): ``balance_screen.status``
is PASS or FAIL -- FAIL means a physical gate was violated.  (Schema v1
reported BLOCKED for that case, which collided with the chain-wide meaning of
BLOCKED = "could not check".)  Anything that prevents a trustworthy verdict
raises ``BalanceScreenError`` instead, which the CLI turns into exit 2.
``hardware_candidate.status`` keeps its own BLOCKED/BALANCE_SCREEN_ONLY
convention, as in ``mocap_g1.controller_screen``.

Joint position limits: as in the controller screen, every hinge ``range`` is
overwritten with the mechanical URDF limits (``limits_urdf_path``; default
the vendored official URDF) and arm PD targets are clamped to them, so the
whole chain uses one limit source; the report records it under
``joint_limits``.

Gates (all thresholds configurable; rationale in
docs/balance-screen-2026-08-16.md):

1. CoM ground-projection margin inside the support polygon.  The polygon is
   the convex hull of the sole contact points that are within
   ``contact_height_m`` of the floor; the margin is the signed distance from
   the world CoM's ground projection to the hull boundary (positive inside).
2. Ankle torque headroom: realised ankle pitch/roll torque against the
   vendor URDF ``effort`` limit (35 N*m -- note the MJCF ``actuatorfrcrange``
   is a more permissive 50 N*m; the screen uses the vendor number).
3. Pelvis attitude: tilt angle away from vertical and base angular speed.

Plus two structural failures: loss of the support polygon (fewer than three
sole points on the ground) and pelvis height collapse.

The recorded ROOT trajectory is deliberately ignored -- the robot is placed
in a nominal standing pose on the ground.  Only the arm columns of the
reference's ``qpos[7:]`` block are used as PD targets.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np

from .assets import official_urdf_path
from .contact import geom_lowest_z
from .controller_screen import (
    resolve_urdf_joint_limits,
    sha256_file,
    urdf_limit_arrays,
    validate_reference_qpos,
)
from .schema import G1_29DOF_JOINTS


class BalanceScreenError(RuntimeError):
    """Raised when the balance screen cannot produce trustworthy metrics."""


#: Report schema; v2 = PASS/FAIL status vocabulary + URDF joint limits.
SCHEMA_VERSION = 2


#: Column indices into ``G1_29DOF_JOINTS`` for the joint groups the screen
#: treats differently.  Legs and waist are held; arms follow the reference.
LEG_COLUMNS = tuple(range(0, 12))
WAIST_COLUMNS = tuple(range(12, 15))
ARM_COLUMNS = tuple(range(15, 29))
ANKLE_JOINTS = (
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
)
ANKLE_ROLL_JOINTS = ("left_ankle_roll_joint", "right_ankle_roll_joint")

#: Vendored official URDF, used only for its ``effort`` limits.
DEFAULT_EFFORT_URDF = official_urdf_path()

#: Provenance for the arm gains and the stance pose, echoed into every report.
ARM_GAIN_SOURCES = (
    "https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/high_level/g1_arm7_sdk_dds_example.cpp",
    "https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/g1/high_level/g1_arm7_sdk_dds_example.py",
)
STANCE_SOURCE = (
    "https://github.com/unitreerobotics/unitree_rl_gym/blob/main/deploy/deploy_real/configs/g1.yaml"
)


@dataclass(frozen=True)
class BalanceScreenConfig:
    """Screen parameters.  Threshold rationale: docs/balance-screen-2026-08-16.md."""

    # --- integration -------------------------------------------------------
    # 0.002 s (the MJCF default and controller_screen's value) is NOT stable
    # with the stiff stance gains below -- the ankle PD blows up and the model
    # topples even on a motionless hold.  1 ms is the largest verified-stable
    # step for the default gains.
    timestep_s: float = 0.001
    settle_seconds: float = 1.0
    speed_factor: float = 1.0

    # --- stance ------------------------------------------------------------
    # Unitree's published G1 standing default_angles
    # (unitree_rl_gym/deploy/deploy_real/configs/g1.yaml): hip_pitch -0.1,
    # knee 0.3, ankle_pitch -0.2, every other joint 0.
    stance_hip_pitch_rad: float = -0.10
    stance_knee_rad: float = 0.30
    stance_ankle_pitch_rad: float = -0.20

    # --- gains -------------------------------------------------------------
    # Arms: Unitree's published arm_sdk example gains (see ARM_GAIN_SOURCES).
    arm_kp: float = 60.0
    arm_kd: float = 1.5
    # Legs/waist: screening surrogate for the factory balance controller, NOT
    # published values.  See the module docstring.
    leg_kp: float = 300.0
    ankle_kp: float = 600.0
    waist_kp: float = 300.0
    stance_damping_ratio: float = 0.06

    # --- actuation ---------------------------------------------------------
    torque_limit_fraction: float = 1.0
    effort_urdf_path: str | None = None
    # Mechanical URDF whose position limits replace the MJCF joint ranges
    # (None = the vendored official URDF).
    limits_urdf_path: str | None = None

    # --- gates -------------------------------------------------------------
    com_margin_min_m: float = 0.02
    ankle_torque_utilization_max: float = 0.70
    pelvis_tilt_max_rad: float = 0.15
    pelvis_angular_speed_max_rad_s: float = 1.0
    pelvis_height_drop_max_m: float = 0.05
    min_support_points: int = 3

    # --- geometry / bookkeeping -------------------------------------------
    contact_height_m: float = 0.01
    abort_tilt_rad: float = 0.60
    abort_height_drop_m: float = 0.20

    def validate(self) -> None:
        positive = (
            "timestep_s",
            "speed_factor",
            "arm_kp",
            "leg_kp",
            "ankle_kp",
            "waist_kp",
            "stance_damping_ratio",
            "torque_limit_fraction",
            "com_margin_min_m",
            "ankle_torque_utilization_max",
            "pelvis_tilt_max_rad",
            "pelvis_angular_speed_max_rad_s",
            "pelvis_height_drop_max_m",
            "contact_height_m",
            "abort_tilt_rad",
            "abort_height_drop_m",
        )
        for name in positive:
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise BalanceScreenError(f"{name} must be finite and positive")
        for name in ("arm_kd", "settle_seconds"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0.0:
                raise BalanceScreenError(f"{name} must be finite and non-negative")
        for name in (
            "stance_hip_pitch_rad",
            "stance_knee_rad",
            "stance_ankle_pitch_rad",
        ):
            if not math.isfinite(float(getattr(self, name))):
                raise BalanceScreenError(f"{name} must be finite")
        if self.torque_limit_fraction > 1.0:
            raise BalanceScreenError("torque_limit_fraction must be <= 1")
        if self.ankle_torque_utilization_max > 1.0:
            raise BalanceScreenError("ankle_torque_utilization_max must be <= 1")
        if int(self.min_support_points) < 3:
            raise BalanceScreenError("min_support_points must be at least 3")
        if self.timestep_s > 0.001 + 1e-12:
            raise BalanceScreenError(
                "timestep_s above 1 ms is not numerically stable with the stiff "
                "stance gains; lower it, or lower ankle_kp deliberately"
            )


def _require_mujoco() -> Any:
    try:
        import mujoco
    except ImportError as exc:  # pragma: no cover - depends on optional runtime
        raise BalanceScreenError(
            "MuJoCo is required; install the project's 'runtime' optional dependencies"
        ) from exc
    return mujoco


def parse_urdf_effort_limits(urdf_path: str | Path) -> dict[str, float]:
    """joint name -> ``<limit effort=...>`` in N*m, for non-fixed joints."""
    root = ET.parse(str(urdf_path)).getroot()
    efforts: dict[str, float] = {}
    for joint in root.iter("joint"):
        if joint.get("type") in ("fixed", None):
            continue
        limit = joint.find("limit")
        if limit is None or limit.get("effort") is None:
            continue
        name = joint.get("name")
        if name:
            efforts[name] = float(limit.get("effort"))
    return efforts


def _validate_reference(qpos: Any, fps: Any) -> tuple[np.ndarray, float]:
    # The shared validator, so balance_screen also refuses non-unit root
    # quaternions (its former private copy had dropped that check).
    return validate_reference_qpos(qpos, fps, BalanceScreenError)


# ---------------------------------------------------------------------------
# Support polygon geometry
# ---------------------------------------------------------------------------


def convex_hull(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Counter-clockwise monotone-chain hull of a small 2-D point set."""
    unique = sorted(set(points))
    if len(unique) < 3:
        return unique

    def _cross(origin, first, second) -> float:
        return (first[0] - origin[0]) * (second[1] - origin[1]) - (
            first[1] - origin[1]
        ) * (second[0] - origin[0])

    def _half(sequence):
        chain: list[tuple[float, float]] = []
        for point in sequence:
            while len(chain) >= 2 and _cross(chain[-2], chain[-1], point) <= 0.0:
                chain.pop()
            chain.append(point)
        return chain

    hull = _half(unique)[:-1] + _half(unique[::-1])[:-1]
    return hull if len(hull) >= 3 else []


def support_margin(
    point: tuple[float, float], hull: list[tuple[float, float]]
) -> float | None:
    """Signed distance from ``point`` to a CCW convex ``hull`` (+ = inside).

    Computed as the minimum signed distance to the hull's EDGE LINES.  Inside
    the polygon that is exactly the distance to the boundary.  Outside it is
    correctly signed but can understate the magnitude near a vertex (distance
    to a line never exceeds distance to the segment), which errs toward
    reporting a smaller violation -- the sign, which is what the gate uses, is
    always right.  ``None`` when the polygon is degenerate.
    """
    if len(hull) < 3:
        return None
    best = math.inf
    count = len(hull)
    for index in range(count):
        ax, ay = hull[index]
        bx, by = hull[(index + 1) % count]
        edge_x, edge_y = bx - ax, by - ay
        length = math.hypot(edge_x, edge_y)
        if length < 1e-12:
            continue
        best = min(best, (edge_x * (point[1] - ay) - edge_y * (point[0] - ax)) / length)
    return None if math.isinf(best) else float(best)


# ---------------------------------------------------------------------------
# Model construction
# ---------------------------------------------------------------------------


@dataclass
class _Rig:
    model: Any
    actuator_ids: np.ndarray
    qpos_addresses: np.ndarray
    dof_addresses: np.ndarray
    sole_geoms: list[int]
    torque_caps: np.ndarray  # applied |cap| per canonical joint, N*m
    mjcf_caps: np.ndarray
    effort_source: str
    nominal_pelvis_z: float


def _build_floating_model(model_path: Path, config: BalanceScreenConfig) -> _Rig:
    mujoco = _require_mujoco()
    if not model_path.is_file():
        raise BalanceScreenError(f"MuJoCo model does not exist: {model_path}")
    try:
        spec = mujoco.MjSpec.from_file(str(model_path))
    except Exception as exc:
        raise BalanceScreenError(f"cannot parse MuJoCo model {model_path}: {exc}") from exc

    free_joints = [joint for joint in spec.joints if joint.type == mujoco.mjtJoint.mjJNT_FREE]
    if len(free_joints) != 1 or free_joints[0].name != "pelvis":
        raise BalanceScreenError("model must contain exactly one free joint named 'pelvis'")
    spec.option.timestep = config.timestep_s
    # Same reason as controller_screen: explicit Euler is unusable at the
    # damping levels a stiff stance hold needs.
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST

    missing_joints = [name for name in G1_29DOF_JOINTS if spec.joint(name) is None]
    missing_actuators = [name for name in G1_29DOF_JOINTS if spec.actuator(name) is None]
    if missing_joints or missing_actuators:
        raise BalanceScreenError(
            f"model lacks canonical joints={missing_joints}, actuators={missing_actuators}"
        )

    urdf_path = Path(config.effort_urdf_path) if config.effort_urdf_path else DEFAULT_EFFORT_URDF
    urdf_efforts: dict[str, float] = {}
    effort_source = "mjcf_actuatorfrcrange"
    if urdf_path.is_file():
        try:
            parsed = parse_urdf_effort_limits(urdf_path)
        except ET.ParseError as exc:
            raise BalanceScreenError(f"cannot parse effort URDF {urdf_path}: {exc}") from exc
        if all(name in parsed for name in G1_29DOF_JOINTS):
            urdf_efforts = parsed
            effort_source = f"min(mjcf_actuatorfrcrange, urdf_effort:{urdf_path.name})"
    elif config.effort_urdf_path is not None:
        raise BalanceScreenError(f"effort URDF does not exist: {urdf_path}")

    mjcf_caps = np.empty(len(G1_29DOF_JOINTS), dtype=np.float64)
    applied = np.empty(len(G1_29DOF_JOINTS), dtype=np.float64)
    for index, name in enumerate(G1_29DOF_JOINTS):
        joint = spec.joint(name)
        actuator = spec.actuator(name)
        force_range = np.asarray(joint.actfrcrange, dtype=np.float64)
        if force_range.shape != (2,) or not np.isfinite(force_range).all():
            raise BalanceScreenError(f"{name} lacks a finite actuator force range")
        if not force_range[0] < 0.0 < force_range[1]:
            raise BalanceScreenError(f"{name} actuator force range must straddle zero")
        mjcf_caps[index] = float(min(abs(force_range[0]), abs(force_range[1])))
        # The vendor URDF is the authoritative number and is TIGHTER than the
        # MJCF on the ankles (35 vs 50 N*m); take the smaller of the two.
        cap = mjcf_caps[index]
        if urdf_efforts:
            cap = min(cap, urdf_efforts[name])
        cap *= config.torque_limit_fraction
        if cap <= 0.0:
            raise BalanceScreenError(f"{name} resolved to a non-positive torque cap")
        applied[index] = cap
        actuator.ctrlrange = np.array([-cap, cap], dtype=np.float64)
        actuator.ctrllimited = True

    # One limit source for the chain: URDF position limits, not the MJCF's
    # IK-narrowed ranges (see the module docstring).
    position_limits, _provenance = resolve_urdf_joint_limits(
        config.limits_urdf_path, BalanceScreenError
    )
    for name in G1_29DOF_JOINTS:
        joint = spec.joint(name)
        joint.range = [position_limits[name].lower, position_limits[name].upper]
        joint.limited = mujoco.mjtLimited.mjLIMITED_TRUE

    try:
        model = spec.compile()
    except Exception as exc:
        raise BalanceScreenError(f"cannot compile floating-base MuJoCo model: {exc}") from exc
    if model.nq != len(G1_29DOF_JOINTS) + 7 or model.nv != len(G1_29DOF_JOINTS) + 6:
        raise BalanceScreenError(
            f"floating model must have 36 qpos / 35 qvel, got nq={model.nq}, nv={model.nv}"
        )
    if model.jnt_type[0] != mujoco.mjtJoint.mjJNT_FREE or model.jnt_qposadr[0] != 0:
        raise BalanceScreenError("floating model must start with the pelvis free joint")

    joint_ids = np.asarray(
        [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in G1_29DOF_JOINTS],
        dtype=np.int32,
    )
    actuator_ids = np.asarray(
        [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in G1_29DOF_JOINTS],
        dtype=np.int32,
    )
    if np.any(joint_ids < 0) or np.any(actuator_ids < 0):
        raise BalanceScreenError("compiled model lost canonical joint/actuator names")

    # Sole geoms: the contact-enabled geoms on whichever body each ankle-roll
    # joint drives.  Derived from the joint, never from body-name guessing.
    sole_geoms: list[int] = []
    for name in ANKLE_ROLL_JOINTS:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        body_id = int(model.jnt_bodyid[joint_id])
        geoms = [
            geom
            for geom in range(model.ngeom)
            if int(model.geom_bodyid[geom]) == body_id
            and (model.geom_contype[geom] or model.geom_conaffinity[geom])
        ]
        if not geoms:
            raise BalanceScreenError(
                f"{name}'s body carries no contact geoms; cannot form a support polygon"
            )
        sole_geoms.extend(geoms)
    if len(sole_geoms) < int(config.min_support_points):
        raise BalanceScreenError("model has too few sole contact geoms for a support polygon")

    return _Rig(
        model=model,
        actuator_ids=actuator_ids,
        qpos_addresses=model.jnt_qposadr[joint_ids],
        dof_addresses=model.jnt_dofadr[joint_ids],
        sole_geoms=sole_geoms,
        torque_caps=applied,
        mjcf_caps=mjcf_caps,
        effort_source=effort_source,
        nominal_pelvis_z=float(model.body_pos[int(model.jnt_bodyid[0])][2]),
    )


def _pelvis_tilt_rad(data: Any, mujoco: Any) -> float:
    matrix = np.zeros(9, dtype=np.float64)
    mujoco.mju_quat2Mat(matrix, data.qpos[3:7])
    return math.acos(float(np.clip(matrix.reshape(3, 3)[2, 2], -1.0, 1.0)))


def build_stance(config: BalanceScreenConfig) -> np.ndarray:
    """The 29-vector standing pose the legs and waist are held at."""
    stance = np.zeros(len(G1_29DOF_JOINTS), dtype=np.float64)
    names = list(G1_29DOF_JOINTS)
    for side in ("left", "right"):
        stance[names.index(f"{side}_hip_pitch_joint")] = config.stance_hip_pitch_rad
        stance[names.index(f"{side}_knee_joint")] = config.stance_knee_rad
        stance[names.index(f"{side}_ankle_pitch_joint")] = config.stance_ankle_pitch_rad
    return stance


def gain_vectors(config: BalanceScreenConfig) -> tuple[np.ndarray, np.ndarray]:
    """Per-joint (kp, kd) in canonical G1 order."""
    names = list(G1_29DOF_JOINTS)
    kp = np.empty(len(names), dtype=np.float64)
    kp[list(LEG_COLUMNS)] = config.leg_kp
    kp[list(WAIST_COLUMNS)] = config.waist_kp
    kp[list(ARM_COLUMNS)] = config.arm_kp
    for name in ANKLE_JOINTS:
        kp[names.index(name)] = config.ankle_kp
    kd = config.stance_damping_ratio * kp
    kd[list(ARM_COLUMNS)] = config.arm_kd
    return kp, kd


def run_balance_screen(
    qpos: Any,
    fps: Any,
    *,
    model_path: str | Path,
    config: BalanceScreenConfig | None = None,
    source_sha256: str | None = None,
    source_metadata: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Drive a recorded arm reference on a floating-base standing G1.

    See the module docstring: this is a SCREEN, not balance validation.
    """
    settings = config or BalanceScreenConfig()
    settings.validate()
    reference, rate = _validate_reference(qpos, fps)
    model_file = Path(model_path).resolve()
    rig = _build_floating_model(model_file, settings)
    mujoco = _require_mujoco()
    position_limits, limit_provenance = resolve_urdf_joint_limits(
        settings.limits_urdf_path, BalanceScreenError
    )
    limit_lower, limit_upper = urdf_limit_arrays(position_limits)
    reference_joints = reference[:, 7:]
    reference_limit_violation_frames = int(np.count_nonzero(np.any(
        (reference_joints[:, ARM_COLUMNS] < limit_lower[list(ARM_COLUMNS)])
        | (reference_joints[:, ARM_COLUMNS] > limit_upper[list(ARM_COLUMNS)]),
        axis=1,
    )))
    reference = reference.copy()
    reference[:, 7:] = np.clip(reference_joints, limit_lower, limit_upper)
    model = rig.model
    data = mujoco.MjData(model)

    names = list(G1_29DOF_JOINTS)
    stance = build_stance(settings)
    arm_columns = np.asarray(ARM_COLUMNS, dtype=np.int32)
    ankle_columns = np.asarray([names.index(name) for name in ANKLE_JOINTS], dtype=np.int32)
    kp, kd = gain_vectors(settings)
    caps = rig.torque_caps
    ankle_caps = caps[ankle_columns]
    sole_ids = rig.sole_geoms
    # Lowest sole point via the shared contact.py implementation (exact for
    # the G1's sphere sole geoms; orientation-aware for any other type).
    mesh_cache: dict[int, np.ndarray] = {}

    # --- place the robot on the ground in the stance pose ------------------
    initial = stance.copy()
    initial[arm_columns] = reference[0, 7:][arm_columns]
    data.qpos[:] = 0.0
    data.qpos[3] = 1.0
    data.qpos[2] = rig.nominal_pelvis_z
    data.qpos[rig.qpos_addresses] = initial
    mujoco.mj_forward(model, data)
    lowest = min(geom_lowest_z(model, data, geom, mesh_cache, mujoco) for geom in sole_ids)
    data.qpos[2] = rig.nominal_pelvis_z - lowest
    mujoco.mj_forward(model, data)
    placement_pelvis_z = float(data.qpos[2])

    def actuate(target: np.ndarray) -> None:
        # Pure PD, no gravity/bias feedforward.  This matches the Unitree
        # MotorCmd semantics the arm_sdk examples use (tau_ff = 0), and adding
        # data.qfrc_bias -- which is computed for an UNSUPPORTED floating base
        # -- was measured to destabilise the stance hold outright.
        error = target - data.qpos[rig.qpos_addresses]
        torque = kp * error - kd * data.qvel[rig.dof_addresses]
        data.ctrl[rig.actuator_ids] = np.clip(torque, -caps, caps)

    settle_steps = int(math.ceil(settings.settle_seconds / settings.timestep_s))
    for _ in range(settle_steps):
        actuate(initial)
        mujoco.mj_step(model, data)
    settled_pelvis_z = float(data.qpos[2])
    settled_tilt = _pelvis_tilt_rad(data, mujoco)

    duration = (len(reference) - 1) / rate / settings.speed_factor
    steps = int(math.ceil(duration / settings.timestep_s)) + 1

    worst_margin = math.inf
    worst_margin_step = -1
    degenerate_steps = 0
    min_support_points_seen = len(sole_ids)
    worst_ankle_ratio = 0.0
    worst_ankle_joint: str | None = None
    worst_ankle_step = -1
    per_ankle_max = np.zeros(len(ankle_columns), dtype=np.float64)
    max_tilt = 0.0
    max_tilt_step = -1
    max_angular_speed = 0.0
    max_angular_speed_step = -1
    min_pelvis_z = settled_pelvis_z
    max_arm_error = 0.0
    aborted_step: int | None = None

    for step in range(steps):
        source_index = min(
            step * settings.timestep_s * rate * settings.speed_factor,
            len(reference) - 1,
        )
        lower = int(math.floor(source_index))
        upper = min(lower + 1, len(reference) - 1)
        alpha = source_index - lower
        blended = (1.0 - alpha) * reference[lower, 7:] + alpha * reference[upper, 7:]
        target = stance.copy()
        target[arm_columns] = blended[arm_columns]
        actuate(target)
        mujoco.mj_step(model, data)

        points: list[tuple[float, float]] = []
        for geom in sole_ids:
            if geom_lowest_z(model, data, geom, mesh_cache, mujoco) < settings.contact_height_m:
                position = data.geom_xpos[geom]
                points.append((float(position[0]), float(position[1])))
        min_support_points_seen = min(min_support_points_seen, len(points))
        com = data.subtree_com[0]
        margin = (
            support_margin((float(com[0]), float(com[1])), convex_hull(points))
            if len(points) >= int(settings.min_support_points)
            else None
        )
        if margin is None:
            degenerate_steps += 1
        elif margin < worst_margin:
            worst_margin = margin
            worst_margin_step = step

        ankle_torque = np.abs(data.actuator_force[rig.actuator_ids[ankle_columns]])
        per_ankle_max = np.maximum(per_ankle_max, ankle_torque)
        ratios = ankle_torque / ankle_caps
        peak = int(np.argmax(ratios))
        if float(ratios[peak]) > worst_ankle_ratio:
            worst_ankle_ratio = float(ratios[peak])
            worst_ankle_joint = ANKLE_JOINTS[peak]
            worst_ankle_step = step

        tilt = _pelvis_tilt_rad(data, mujoco)
        if tilt > max_tilt:
            max_tilt, max_tilt_step = tilt, step
        angular_speed = float(np.linalg.norm(data.qvel[3:6]))
        if angular_speed > max_angular_speed:
            max_angular_speed, max_angular_speed_step = angular_speed, step
        min_pelvis_z = min(min_pelvis_z, float(data.qpos[2]))
        max_arm_error = max(
            max_arm_error,
            float(
                np.max(
                    np.abs(
                        target[arm_columns]
                        - data.qpos[rig.qpos_addresses][arm_columns]
                    )
                )
            ),
        )

        if (
            tilt > settings.abort_tilt_rad
            or settled_pelvis_z - float(data.qpos[2]) > settings.abort_height_drop_m
        ):
            aborted_step = step
            break

    executed = steps if aborted_step is None else aborted_step + 1
    height_drop = settled_pelvis_z - min_pelvis_z
    finite_margin = math.isfinite(worst_margin)

    reasons: list[str] = []
    if degenerate_steps:
        reasons.append("support_polygon_lost")
    if not finite_margin:
        reasons.append("com_margin_unmeasurable")
    elif worst_margin < settings.com_margin_min_m:
        reasons.append("com_support_margin_exceeded")
    if worst_ankle_ratio > settings.ankle_torque_utilization_max:
        reasons.append("ankle_torque_headroom_exceeded")
    if max_tilt > settings.pelvis_tilt_max_rad:
        reasons.append("pelvis_tilt_exceeded")
    if max_angular_speed > settings.pelvis_angular_speed_max_rad_s:
        reasons.append("pelvis_angular_speed_exceeded")
    if height_drop > settings.pelvis_height_drop_max_m:
        reasons.append("pelvis_height_collapse")
    if aborted_step is not None:
        reasons.append("fall_abort")

    metadata = source_metadata or {}
    preview_calibration = metadata.get("offsets_key") == "zero-preview"
    com_status = (
        "FAIL"
        if (
            not finite_margin
            or degenerate_steps
            or worst_margin < settings.com_margin_min_m
        )
        else "PASS"
    )
    attitude_status = (
        "FAIL"
        if (
            max_tilt > settings.pelvis_tilt_max_rad
            or max_angular_speed > settings.pelvis_angular_speed_max_rad_s
            or height_drop > settings.pelvis_height_drop_max_m
        )
        else "PASS"
    )
    return {
        "schema_version": SCHEMA_VERSION,
        "purpose": "floating_base_stiff_pd_standing_balance_screen_not_balance_validation",
        "balance_screen": {
            "status": "FAIL" if reasons else "PASS",
            "reasons": reasons,
        },
        "joint_limits": {
            **limit_provenance,
            "applied_to": "floating-base model joint ranges and arm PD target clamp",
            "reference_arm_frames_outside_limits": reference_limit_violation_frames,
        },
        "hardware_candidate": {
            "status": "BLOCKED" if preview_calibration or reasons else "BALANCE_SCREEN_ONLY",
            "reasons": (["preview_calibration"] if preview_calibration else []) + reasons,
        },
        "input": {
            "frames": len(reference),
            "fps": rate,
            "duration_s": (len(reference) - 1) / rate,
            "source_sha256": source_sha256,
            "offsets_key": metadata.get("offsets_key"),
            "root_trajectory": "ignored; the robot is placed in a nominal standing pose",
        },
        "configuration": asdict(settings),
        "gains": {
            "arm_kp": settings.arm_kp,
            "arm_kd": settings.arm_kd,
            "arm_gain_verification_status": "verified_published",
            "arm_gain_sources": list(ARM_GAIN_SOURCES),
            "stance_kp_by_group": {
                "leg": settings.leg_kp,
                "ankle": settings.ankle_kp,
                "waist": settings.waist_kp,
            },
            "stance_kd_ratio": settings.stance_damping_ratio,
            "stance_gain_verification_status": "screening_surrogate_not_published",
            "feedforward": "none (tau_ff = 0, matching Unitree MotorCmd semantics)",
        },
        "model": {
            "source_path": str(model_file),
            "source_sha256": sha256_file(model_file),
            "base": "pelvis_freejoint_retained",
            "legs_waist": "stiff joint-space PD hold at the configured standing stance",
            "arms": "PD tracking of the reference arm columns",
            "controlled_arm_joints": [names[index] for index in ARM_COLUMNS],
            "torque_limit_source": rig.effort_source,
            "applied_torque_caps_nm": {
                name: float(caps[index]) for index, name in enumerate(names)
            },
        },
        "stance": {
            "source": STANCE_SOURCE,
            "hip_pitch_rad": settings.stance_hip_pitch_rad,
            "knee_rad": settings.stance_knee_rad,
            "ankle_pitch_rad": settings.stance_ankle_pitch_rad,
            "placement_pelvis_z_m": placement_pelvis_z,
            "settled_pelvis_z_m": settled_pelvis_z,
            "settled_pelvis_tilt_rad": settled_tilt,
            "settle_steps": settle_steps,
        },
        "gates": {
            "com_support_margin": {
                "threshold_min_m": settings.com_margin_min_m,
                "worst_margin_m": float(worst_margin) if finite_margin else None,
                "worst_margin_step": worst_margin_step if finite_margin else None,
                "worst_margin_time_s": (
                    worst_margin_step * settings.timestep_s if finite_margin else None
                ),
                "support_polygon_degenerate_steps": degenerate_steps,
                "min_support_points_observed": min_support_points_seen,
                "sole_contact_geoms": len(sole_ids),
                "status": com_status,
            },
            "ankle_torque_headroom": {
                "threshold_max_utilization": settings.ankle_torque_utilization_max,
                "worst_utilization": worst_ankle_ratio,
                "worst_joint": worst_ankle_joint,
                "worst_step": worst_ankle_step,
                "limit_source": rig.effort_source,
                "per_joint_max_torque_nm": {
                    name: float(per_ankle_max[index])
                    for index, name in enumerate(ANKLE_JOINTS)
                },
                "per_joint_limit_nm": {
                    name: float(ankle_caps[index])
                    for index, name in enumerate(ANKLE_JOINTS)
                },
                "mjcf_actuatorfrcrange_nm": {
                    name: float(rig.mjcf_caps[names.index(name)]) for name in ANKLE_JOINTS
                },
                "status": (
                    "FAIL"
                    if worst_ankle_ratio > settings.ankle_torque_utilization_max
                    else "PASS"
                ),
            },
            "pelvis_attitude": {
                "tilt_threshold_max_rad": settings.pelvis_tilt_max_rad,
                "max_tilt_rad": max_tilt,
                "max_tilt_step": max_tilt_step,
                "angular_speed_threshold_max_rad_s": settings.pelvis_angular_speed_max_rad_s,
                "max_angular_speed_rad_s": max_angular_speed,
                "max_angular_speed_step": max_angular_speed_step,
                "height_drop_threshold_max_m": settings.pelvis_height_drop_max_m,
                "max_height_drop_m": height_drop,
                "status": attitude_status,
            },
        },
        "events": {
            "planned_steps": steps,
            "executed_steps": executed,
            "aborted_step": aborted_step,
            "aborted_time_s": None if aborted_step is None else aborted_step * settings.timestep_s,
            "max_arm_tracking_error_rad": max_arm_error,
        },
        "limitations": [
            "legs and waist are held by stiff joint-space PD, not the factory whole-body balance controller",
            "stance gains are a screening surrogate far above Unitree's published deployment values; below the inverted-pendulum stiffness m*g*h a joint PD cannot hold a standing pose at all",
            "no recovery step, hip strategy, CoM/ZMP feedback or arm-swing counterbalance is modelled, so the screen is conservative in the sagittal plane",
            "actuators are ideal torque sources: no transport delay, no 50 Hz command quantisation, no motor thermal derating or gearbox backlash",
            "the recorded root trajectory is ignored; the robot always starts from a nominal standing pose on flat, high-friction ground",
            "PASS means this clip did not topple the surrogate, not that the real G1 will stay upright",
        ],
    }
