"""Fail-closed world-contact refinement for retargeted G1 trajectories.

PhySINK-style contact-consistent post-pass applied directly to a conditioned
floating-base G1 ``qpos`` trajectory (root XYZ + WXYZ quaternion + canonical
29 joints).  The pass repairs two measured artifacts of kinematic retargeting
that grounding-only post-processes cannot fix:

  1. pitched-foot ground penetration -- a foot corner dips below the world
     ground plane because ankle limits cannot reproduce the source foot pitch;
     repaired per frame with a deterministic bounded search (ankle pitch ->
     ankle roll -> root-z lift), each candidate verified by forward kinematics.
  2. single-stance foot skating -- the sole's contact material point slides
     horizontally while its foot is detected in stance; repaired by shifting
     the root horizontally by the negative of that point's frame-to-frame
     displacement (rigid compensation), clamped per step and per segment.
     Skating is measured on the MATERIAL point (the previous frame's lowest
     sole point carried rigidly with the foot), not by differencing the
     argmin lowest point: on a flat foot the argmin flips between corners
     under sub-millimetre pitch/roll dither and would read as m/s skating
     (see ``mocap_g1.contact.material_point_slip_xy``).

Honesty boundaries (mirroring mocap_g1.collision_gate):

  * Every repair is bounded by explicit budgets and verified by an
    independent second FK pass over the exact returned array.  If any frame
    cannot be repaired within the budgets -- or any correction violates the
    dynamics bounds -- the whole derivative is rejected and ``qpos`` is
    ``None``.  Thresholds are never silently loosened.
  * The result inherits the source recording's calibration quality and never
    estimates calibration, offsets, or a ground height from the data; the
    ground plane is fixed at z=0 in the supplied MJCF's world frame.
  * Contact inference is purely kinematic (sole height + vertical speed with
    hysteresis).  It is not force sensing and does not prove real-robot
    contact stability, balance, or actuation feasibility.
  * Double-stance skating is detected and reported but intentionally NOT
    repaired in this version: pinning one planted foot necessarily slides the
    other.  Such frames are listed as rejections.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from numbers import Real
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from .contact import (
    REFINE_CONTACT_HEIGHT_BAND_M,
    REFINE_CONTACT_VZ_MAX_M_S,
    central_diff,
    contact_candidates,
    foot_contact_geoms,
    geom_lowest_point,
    lowest_point_of_geoms,
    material_point_slip_xy,
)
from .controller_screen import (
    ControllerScreenError,
    _require_mujoco,
    _validate_reference,
    resolve_urdf_joint_limits,
)
from .schema import JointLimit
from .io_utils import array_sha256, sha256_file, write_qpos_npz_atomic
from .schema import G1_29DOF_JOINTS


#: v3 (2026-10-12): ankle correction bounds come from the mechanical URDF
#: (``limits_urdf_path``), not the MJCF's IK-narrowed ranges; the report
#: records the limit file + sha256 under ``joint_limits``.
CONTACT_REFINE_VERSION = 3
QPOS_WIDTH = 7 + len(G1_29DOF_JOINTS)
ANKLE_ROLE_COLUMNS = {
    ("left", "ankle_pitch"): "left_ankle_pitch_joint",
    ("left", "ankle_roll"): "left_ankle_roll_joint",
    ("right", "ankle_pitch"): "right_ankle_pitch_joint",
    ("right", "ankle_roll"): "right_ankle_roll_joint",
}


class ContactRefineError(RuntimeError):
    """Raised when contact refinement cannot produce a trustworthy result."""


@dataclass(frozen=True)
class ContactRefineConfig:
    """Conservative, explicit budgets for world-contact repairs."""

    # --- contact inference (kinematic, world-anchored, with hysteresis) ---
    # Named shared constants; deliberately looser in vz than the eval
    # "anchored" scoring policy -- see mocap_g1/contact.py for why.
    contact_height_band_m: float = REFINE_CONTACT_HEIGHT_BAND_M
    contact_vz_max_m_s: float = REFINE_CONTACT_VZ_MAX_M_S
    hysteresis_enter_frames: int = 2
    hysteresis_exit_frames: int = 3
    # --- penetration repair ---
    penetration_tolerance_m: float = 0.0005
    target_clearance_m: float = 0.002
    max_ankle_correction_rad: float = 0.30
    max_root_z_lift_m: float = 0.05
    search_steps: int = 16
    refinement_steps: int = 12
    # --- skating repair (single stance only) ---
    skate_repair_enabled: bool = True
    skate_speed_threshold_m_s: float = 0.03
    max_root_shift_step_m: float = 0.01
    max_segment_pinning_displacement_m: float = 0.08
    skate_taper_frames: int = 8
    # --- correction coherence budgets (whole trajectory) ---
    max_correction_velocity_rad_s: float = 1.5
    max_correction_acceleration_rad_s2: float = 6.0
    max_correction_velocity_m_s: float = 0.50
    max_correction_acceleration_m_s2: float = 2.00
    max_total_joint_deviation_rad: float = 0.35
    # --- joint limits (one source for the chain: the mechanical URDF) ---
    limits_urdf_path: str | None = None

    def validate(self) -> None:
        positive_floats = (
            "contact_height_band_m",
            "contact_vz_max_m_s",
            "penetration_tolerance_m",
            "target_clearance_m",
            "max_ankle_correction_rad",
            "max_root_z_lift_m",
            "skate_speed_threshold_m_s",
            "max_root_shift_step_m",
            "max_segment_pinning_displacement_m",
            "max_correction_velocity_rad_s",
            "max_correction_acceleration_rad_s2",
            "max_correction_velocity_m_s",
            "max_correction_acceleration_m_s2",
            "max_total_joint_deviation_rad",
        )
        for name in positive_floats:
            value = getattr(self, name)
            if (
                isinstance(value, bool)
                or not isinstance(value, Real)
                or not math.isfinite(float(value))
                or value <= 0.0
            ):
                raise ContactRefineError(f"{name} must be finite and positive")
        for name in ("hysteresis_enter_frames", "hysteresis_exit_frames", "skate_taper_frames"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ContactRefineError(f"{name} must be a positive integer")
        for name in ("search_steps", "refinement_steps"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ContactRefineError(f"{name} must be a positive integer")
        if self.target_clearance_m <= self.penetration_tolerance_m:
            raise ContactRefineError(
                "target_clearance_m must exceed penetration_tolerance_m so a "
                "repaired frame is not itself flagged as penetrating"
            )
        if not isinstance(self.skate_repair_enabled, bool):
            raise ContactRefineError("skate_repair_enabled must be a boolean")


@dataclass(frozen=True)
class ContactRefineResult:
    """Contact-refinement output; ``qpos`` is absent whenever rejected."""

    qpos: np.ndarray | None
    fps: float
    report: dict[str, Any]


# ---------------------------------------------------------------------------
# Foot geometry resolution + sole measurement (local, eval-free)
# ---------------------------------------------------------------------------


def _resolve_feet(model: Any, mujoco: Any) -> dict[str, dict[str, Any]]:
    """Resolve {side: {body_id, geoms, sphere_geoms}} for the two feet.

    Prefers the G1 convention ``{side}_ankle_roll_link``; falls back to the
    common ``{side}_foot`` naming used by minimal test models.
    """
    names = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body): body
        for body in range(1, model.nbody)
    }
    feet: dict[str, dict[str, Any]] = {}
    for side in ("left", "right"):
        body = None
        for candidate in (f"{side}_ankle_roll_link", f"{side}_foot", f"{side}_foot_link"):
            if candidate in names:
                body = names[candidate]
                break
        if body is None:
            raise ContactRefineError(
                f"no foot body resolved for side {side!r}; expected e.g. "
                f"'{side}_ankle_roll_link'"
            )
        geoms, _visual_fallback = foot_contact_geoms(model, body)
        if not geoms:
            raise ContactRefineError(
                f"foot body for side {side!r} has no geoms; cannot measure sole height"
            )
        sphere_geoms = [
            geom
            for geom in geoms
            if model.geom_type[geom] == mujoco.mjtGeom.mjGEOM_SPHERE
        ]
        geom_index = np.asarray(geoms, dtype=np.intp)
        feet[side] = {
            "body": int(body),
            "geoms": geoms,
            "sphere_geoms": sphere_geoms,
            # Invariant geometry cached once per model (performance only):
            # when every contact geom is a sphere -- the G1 sole layout --
            # the lowest point is evaluated vectorised by _side_lowest_point.
            "geom_index": geom_index,
            "sphere_radii": (
                np.array(model.geom_size[geom_index, 0], dtype=np.float64)
                if len(sphere_geoms) == len(geoms)
                else None
            ),
        }
    return feet


def _fk(model: Any, data: Any, mujoco: Any, row: np.ndarray) -> None:
    """Positional forward kinematics of one qpos row into ``data``.

    ``mj_kinematics`` is the first step of ``mj_forward`` and the only one
    that writes the body/geom world poses (``xpos``, ``xmat``,
    ``geom_xpos``, ``geom_xmat``) this module reads, so the values are
    bit-identical to a full ``mj_forward`` (asserted by
    tests/test_contact_refine.py) while skipping collision detection,
    constraint assembly and dynamics -- ~98% of the per-call cost on the G1
    MJCF, whose penetrating sole spheres make collision dominate.
    """
    data.qpos[:] = row
    mujoco.mj_kinematics(model, data)


def _side_lowest_point(model: Any, data: Any, mujoco: Any, info: Mapping[str, Any],
                       mesh_cache: dict[int, np.ndarray]) -> tuple[float, np.ndarray]:
    """Lowest (z, xyz) over one foot's contact geoms at the current FK state.

    All-sphere feet take a vectorised path that performs exactly the shared
    helper's float operations (``center_z - radius``; x/y = sphere centre;
    first minimum wins ties), so the result is bit-identical to
    ``mocap_g1.contact.lowest_point_of_geoms``.  Any other geom type, or a
    non-finite value, defers to the shared helper.
    """
    radii = info.get("sphere_radii")
    if radii is not None:
        index = info["geom_index"]
        centers = data.geom_xpos
        heights = centers[index, 2] - radii
        best = int(heights.argmin())
        low_z = float(heights[best])
        if math.isfinite(low_z):
            point = np.array(centers[index[best]], dtype=np.float64)
            point[2] = low_z
            return low_z, point
    return lowest_point_of_geoms(model, data, info["geoms"], mesh_cache, mujoco)


def _side_lowest_z(model: Any, data: Any, mujoco: Any, info: Mapping[str, Any],
                   mesh_cache: dict[int, np.ndarray]) -> float:
    """``_side_lowest_point(...)[0]`` without building the point (hot path)."""
    radii = info.get("sphere_radii")
    if radii is not None:
        low_z = float((data.geom_xpos[info["geom_index"], 2] - radii).min())
        if math.isfinite(low_z):
            return low_z
    return _side_lowest_point(model, data, mujoco, info, mesh_cache)[0]


def _subtree_bodies(model: Any, root_body: int) -> set[int]:
    """``root_body`` and every body below it in the kinematic tree."""
    members = {int(root_body)}
    for body in range(int(root_body) + 1, model.nbody):
        if int(model.body_parentid[body]) in members:
            members.add(body)
    return members


def _geoms_independent_of_columns(model: Any, mujoco: Any,
                                  feet: Mapping[str, Mapping[str, Any]],
                                  columns: list[int]) -> dict[int, list[int]]:
    """{canonical column: foot contact geoms whose pose that joint cannot move}.

    A body's world pose depends only on the joints of its ancestors, so a
    geom outside the joint's subtree has a bit-identical FK pose for every
    value of that joint.  Used to skip ankle searches that are provably
    infeasible (see ``_repair_penetration_frame``).
    """
    out: dict[int, list[int]] = {}
    for column in columns:
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, G1_29DOF_JOINTS[column])
        moved = _subtree_bodies(model, int(model.jnt_bodyid[joint]))
        out[column] = [
            geom
            for info in feet.values()
            for geom in info["geoms"]
            if int(model.geom_bodyid[geom]) not in moved
        ]
    return out


def _validate_model_layout(model: Any, mujoco: Any, model_file: Path) -> None:
    """Require the exact free-root plus canonical-joint qpos contract.

    Contact refinement writes canonical NPZ columns directly into ``data.qpos``.
    Matching ``nq`` alone is not enough: a renamed or reordered joint would make
    FK interpret the recording as another pose and could make an ankle repair
    modify the wrong joint.
    """
    if model.nq != QPOS_WIDTH:
        raise ContactRefineError(
            f"model {model_file.name} must have nq={QPOS_WIDTH} "
            f"(free pelvis + 29 joints), got {model.nq}"
        )
    free_joints = [
        joint
        for joint in range(model.njnt)
        if model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_FREE
    ]
    if len(free_joints) != 1 or int(model.jnt_qposadr[free_joints[0]]) != 0:
        raise ContactRefineError(
            f"model {model_file.name} must contain exactly one free root at qpos address 0"
        )

    missing: list[str] = []
    wrong_type: list[str] = []
    wrong_address: list[str] = []
    for column, name in enumerate(G1_29DOF_JOINTS):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            missing.append(name)
            continue
        if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
            wrong_type.append(name)
        expected = 7 + column
        actual = int(model.jnt_qposadr[joint_id])
        if actual != expected:
            wrong_address.append(f"{name}:{actual}!={expected}")
    if missing or wrong_type or wrong_address:
        raise ContactRefineError(
            f"model {model_file.name} does not match canonical G1 qpos layout: "
            f"missing={missing}, non_hinge={wrong_type}, wrong_addresses={wrong_address}"
        )


def _geom_lowest_point(
    model: Any, data: Any, mujoco: Any, geom: int, mesh_cache: dict[int, np.ndarray]
) -> tuple[float, np.ndarray]:
    """World (z, xyz) of a geom's lowest point (shared mocap_g1.contact copy)."""
    return geom_lowest_point(model, data, geom, mesh_cache, mujoco)


def _sole_state(model: Any, data: Any, mujoco: Any, feet: Mapping[str, Mapping[str, Any]],
                mesh_cache: dict[int, np.ndarray]) -> dict[str, tuple[float, np.ndarray]]:
    """Current-FK {side: (lowest sole z, lowest point world xyz)} per foot."""
    return {
        side: _side_lowest_point(model, data, mujoco, info, mesh_cache)
        for side, info in feet.items()
    }


def _fk_sole_pass(model: Any, data: Any, mujoco: Any, feet: Mapping[str, Mapping[str, Any]],
                  qpos_rows: np.ndarray) -> dict[str, dict[str, np.ndarray]]:
    """One FK (``_fk``) per frame; per side:

    * ``z`` (T,): lowest sole z -- used for penetration and stance height;
    * ``xy`` (T,2): that lowest point's world xy (reported only);
    * ``slip_xy`` (T,2): frame-to-frame horizontal displacement of the
      contact MATERIAL point (row 0 is zero) -- the skating measurement.
    """
    total = len(qpos_rows)
    mesh_cache: dict[int, np.ndarray] = {}
    out: dict[str, dict[str, np.ndarray]] = {
        side: {"z": np.zeros(total), "xy": np.zeros((total, 2))} for side in feet
    }
    points = {side: np.zeros((total, 3)) for side in feet}
    body_pos = {side: np.zeros((total, 3)) for side in feet}
    body_mat = {side: np.zeros((total, 3, 3)) for side in feet}
    for frame in range(total):
        _fk(model, data, mujoco, qpos_rows[frame])
        state = _sole_state(model, data, mujoco, feet, mesh_cache)
        for side, (low_z, point) in state.items():
            out[side]["z"][frame] = low_z
            out[side]["xy"][frame] = point[:2]
            points[side][frame] = point
            body = feet[side]["body"]
            body_pos[side][frame] = data.xpos[body]
            body_mat[side][frame] = data.xmat[body].reshape(3, 3)
    for side in feet:
        out[side]["slip_xy"] = material_point_slip_xy(
            points[side], body_pos[side], body_mat[side]
        )
    return out


def _slip_speeds(sole_pass: Mapping[str, np.ndarray], rate: float,
                 start: int = 0, end: int | None = None) -> np.ndarray:
    """Material-point slip speeds (m/s) for frames ``start+1 .. end``."""
    stop = len(sole_pass["slip_xy"]) if end is None else end + 1
    return np.linalg.norm(sole_pass["slip_xy"][start + 1:stop], axis=1) * rate


# ---------------------------------------------------------------------------
# Stage A: hysteresis contact inference (pure, model-free)
# ---------------------------------------------------------------------------


def infer_contact_states(sole_height: np.ndarray, fps: float,
                         config: ContactRefineConfig,
                         raw_candidate: np.ndarray | None = None
                         ) -> tuple[np.ndarray, float]:
    """Per-frame stance booleans with enter/exit hysteresis debouncing.

    Deliberately world-anchored, unlike the adaptive percentile thresholds in
    eval/metrics.py and the grounding preview tool: a foot counts as a stance
    candidate only when its lowest point is within ``contact_height_band_m``
    of the MJCF world ground plane (z=0) AND its vertical speed is small.
    An adaptive threshold would mark any constant-height airborne foot as
    stance, which would let the skating repair pin feet that never touch the
    ground.  Entering stance requires ``hysteresis_enter_frames`` consecutive
    candidates; leaving requires ``hysteresis_exit_frames`` consecutive
    non-candidates.  In between, the previous state is held.

    ``raw_candidate`` overrides the height/speed computation so the debounce
    semantics can be exercised directly in unit tests; production callers
    leave it as ``None``.
    """
    height = np.asarray(sole_height, dtype=np.float64)
    threshold = config.contact_height_band_m
    if raw_candidate is None:
        raw_candidate = contact_candidates(
            height, central_diff(height, fps),
            height_max=threshold, vz_max=config.contact_vz_max_m_s,
        )
    else:
        raw_candidate = np.asarray(raw_candidate, dtype=bool)
    states = np.zeros(len(height), dtype=bool)
    entered = run_length = 0
    exited = exit_run = 0
    for frame in range(len(height)):
        if not entered:
            run_length = run_length + 1 if raw_candidate[frame] else 0
            if run_length >= config.hysteresis_enter_frames:
                entered = True
                exited = False
                exit_run = 0
        elif not exited:
            exit_run = 0 if raw_candidate[frame] else exit_run + 1
            if exit_run >= config.hysteresis_exit_frames:
                exited = True
                entered = False
                run_length = 0
        states[frame] = entered and not exited
    return states, threshold


# ---------------------------------------------------------------------------
# Stages B/C: bounded deterministic repairs
# ---------------------------------------------------------------------------


def _ankle_bounds(limits: Mapping[str, JointLimit], column: int, value: float,
                  config: ContactRefineConfig) -> tuple[float, float]:
    """Feasible correction interval for one ankle column: the mechanical URDF
    position range intersected with the symmetric per-repair budget around
    the input value."""
    name = G1_29DOF_JOINTS[column]
    if name not in limits:
        raise ContactRefineError(f"joint limits lack canonical joint {name!r}")
    low = max(float(limits[name].lower), value - config.max_ankle_correction_rad)
    high = min(float(limits[name].upper), value + config.max_ankle_correction_rad)
    return (low - value, high - value)


def _frame_predicate(model: Any, data: Any, mujoco: Any, feet: Any, row: np.ndarray,
                     columns: tuple[tuple[int, float], ...],
                     target: float, mesh_cache: dict[int, np.ndarray]) -> bool:
    """True when the trial row (with deltas applied) clears ``target`` everywhere."""
    trial = row.copy()
    for column, delta in columns:
        if column >= 0:
            trial[7 + column] += delta
        else:  # column == -1 encodes a root-z lift
            trial[2] += delta
    _fk(model, data, mujoco, trial)
    # FK around an exactly tangent sphere can differ by a few ulps after an
    # otherwise exact root translation. This numerical allowance is twelve
    # orders of magnitude below the millimetre-scale safety thresholds.
    return min(
        _side_lowest_z(model, data, mujoco, info, mesh_cache) for info in feet.values()
    ) >= target - 1e-12


def _bounded_search(predicate: Any, column: int, bounds: tuple[float, float],
                    config: ContactRefineConfig) -> float | None:
    """Smallest-magnitude feasible delta, searched outward from zero.

    Two rays are scanned independently (negative then positive), each with a
    coarse grid and bisection refinement toward the ray origin; the feasible
    candidate with smaller absolute value wins.  Deterministic, and no
    monotonicity across zero is assumed.
    """
    low, high = bounds
    best: float | None = None

    def scan_ray(origin_sign_zero_to: float) -> float | None:
        """Scan [0 -> bound] on one side; return refined smallest feasible."""
        if origin_sign_zero_to == 0.0:
            return None
        grid = [origin_sign_zero_to * step / config.search_steps
                for step in range(1, config.search_steps + 1)]
        first_feasible = None
        last_infeasible = 0.0
        for delta in grid:
            if predicate(column, delta):
                first_feasible = delta
                break
            last_infeasible = delta
        if first_feasible is None:
            return None
        refined = first_feasible
        inner_infeasible = last_infeasible
        for _ in range(config.refinement_steps):
            midpoint = 0.5 * (inner_infeasible + refined)
            if predicate(column, midpoint):
                refined = midpoint
            else:
                inner_infeasible = midpoint
        return refined

    for ray in (low, high):
        found = scan_ray(ray)
        if found is not None and (best is None or abs(found) < abs(best)):
            best = found
    return best


def _repair_penetration_frame(model: Any, data: Any, mujoco: Any, feet: Any,
                              working_row: np.ndarray, column_of: Mapping[str, int],
                              config: ContactRefineConfig,
                              mesh_cache: dict[int, np.ndarray],
                              limits: Mapping[str, JointLimit],
                              independent_geoms: Mapping[int, list[int]] | None = None,
                              ) -> tuple[list[dict[str, Any]], bool]:
    """Repair one frame; returns (records, success).  A non-penetrating frame
    is trivially successful and left untouched.

    ``independent_geoms`` ({column: foot geoms that joint cannot move}, from
    ``_geoms_independent_of_columns``) enables an exact shortcut: when such a
    geom is already below the clearance target, every candidate of that
    column's search fails the predicate, so the search is skipped -- it would
    have returned ``None`` after evaluating every grid point.  ``None``
    disables the shortcut (the result is identical either way).
    """
    target = config.target_clearance_m
    _fk(model, data, mujoco, working_row)
    state = _sole_state(model, data, mujoco, feet, mesh_cache)
    penetrating = sorted(
        (side for side, (low_z, _) in state.items()
         if low_z < -config.penetration_tolerance_m),
        key=lambda side: state[side][0],
    )
    if not penetrating:
        return [], True
    applied: list[dict[str, Any]] = []

    def predicate(column: int, delta: float) -> bool:
        return _frame_predicate(model, data, mujoco, feet, working_row,
                                ((column, delta),), target, mesh_cache)

    def per_geom_low_z() -> dict[int, float]:
        # Snapshot of every foot geom's lowest z for the CURRENT working_row
        # (``data`` must hold its FK); the same helper and float ops as the
        # predicate, so the comparison below is exact.
        return {
            geom: geom_lowest_point(model, data, geom, mesh_cache, mujoco)[0]
            for info in feet.values()
            for geom in info["geoms"]
        }

    geom_low = per_geom_low_z() if independent_geoms is not None else {}
    for side in penetrating:
        for role in ("ankle_pitch", "ankle_roll"):
            name = ANKLE_ROLE_COLUMNS[(side, role)]
            column = column_of[name]
            bounds = _ankle_bounds(limits, column, working_row[7 + column], config)
            fixed = independent_geoms.get(column, []) if independent_geoms is not None else []
            if fixed and min(geom_low[geom] for geom in fixed) < target - 1e-12:
                continue  # provably infeasible: identical to a None search
            delta = _bounded_search(predicate, column, bounds, config)
            if delta is None:
                continue
            working_row[7 + column] += delta
            applied.append({
                "strategy": f"{side}:{role}",
                "joint": name,
                "delta_rad": float(delta),
            })
            _fk(model, data, mujoco, working_row)
            state = _sole_state(model, data, mujoco, feet, mesh_cache)
            if independent_geoms is not None:
                geom_low = per_geom_low_z()
            if all(low_z >= target for low_z, _ in state.values()):
                return applied, True

    # Root-z fallback: smallest lift on a coarse grid that clears both feet.
    # Every candidate, including the top of the grid, stays inside the stated
    # budget. _frame_predicate carries the ulp-scale boundary allowance.
    deficit = target - min(low_z for low_z, _ in state.values())
    chosen_lift = None
    if 0.0 < deficit <= config.max_root_z_lift_m:
        top = min(deficit, config.max_root_z_lift_m)
        grid_lifts = [top * step / config.search_steps
                      for step in range(1, config.search_steps + 1)]
        chosen_lift = next(
            (lift for lift in grid_lifts
             if _frame_predicate(model, data, mujoco, feet, working_row,
                                 ((-1, lift),), target, mesh_cache)),
            None,
        )
    if chosen_lift is not None:
        if chosen_lift > config.max_root_z_lift_m:
            raise ContactRefineError("internal error: root lift exceeded configured budget")
        working_row[2] += chosen_lift
        applied.append({
            "strategy": "root_z_lift",
            "joint": None,
            "delta_rad": 0.0,
            "lift_m": float(chosen_lift),
        })
        return applied, True

    return applied, False


def _contiguous_segments(mask: np.ndarray) -> list[tuple[int, int]]:
    """[(start, end_inclusive)] runs of True in a boolean array."""
    segments: list[tuple[int, int]] = []
    start = None
    for index, flag in enumerate(mask):
        if flag and start is None:
            start = index
        elif not flag and start is not None:
            segments.append((start, index - 1))
            start = None
    if start is not None:
        segments.append((start, len(mask) - 1))
    return segments


def _repair_skating_segment(working: np.ndarray, side: str,
                            start: int, end: int, sole_points: Mapping[str, np.ndarray],
                            contact_free: np.ndarray, rate: float,
                            config: ContactRefineConfig,
                            ) -> dict[str, Any]:
    """Plan and apply root-xy pinning for one single-stance segment.

    Because a root translation moves every body rigidly, the contact material
    point's frame-to-frame displacement measured on the pre-shift pass is
    exact, and
    applying ``-displacement`` to the root at each frame cancels it. The
    correction step ramps within the configured velocity and acceleration
    bounds. If that cannot reduce the residual below the skating threshold,
    final validation rejects the clip.

    A nonzero terminal offset is returned to zero only across an immediately
    following run of frames where neither foot is in contact. If no
    contact-free, dynamics-safe taper exists, this function leaves ``working``
    untouched and reports a failed repair attempt.
    """
    total = len(working)
    offsets = np.zeros((total, 2))
    applied_steps = 0
    velocity_step_cap = min(
        config.max_root_shift_step_m,
        config.max_correction_velocity_m_s / rate,
    )
    acceleration_step_cap = config.max_correction_acceleration_m_s2 / (rate * rate)

    for frame in range(max(start, 1), end + 1):
        displacement = sole_points[side]["slip_xy"][frame]
        source_speed = float(np.linalg.norm(displacement)) * rate
        desired = (
            -displacement
            if source_speed > config.skate_speed_threshold_m_s
            else np.zeros(2, dtype=np.float64)
        )
        desired_norm = float(np.linalg.norm(desired))
        if desired_norm > velocity_step_cap:
            desired *= velocity_step_cap / desired_norm

        previous_step = (
            offsets[frame - 1] - offsets[frame - 2]
            if frame >= 2
            else offsets[frame - 1]
        )
        step_change = np.clip(
            desired - previous_step,
            -acceleration_step_cap,
            acceleration_step_cap,
        )
        step = previous_step + step_change
        step_norm = float(np.linalg.norm(step))
        if step_norm > velocity_step_cap:
            step *= velocity_step_cap / step_norm

        projected = offsets[frame - 1] + step
        projected_norm = float(np.linalg.norm(projected))
        cap = config.max_segment_pinning_displacement_m
        if projected_norm > cap:
            room = max(0.0, cap - float(np.linalg.norm(offsets[frame - 1])))
            if room <= 1e-12:
                offsets[frame] = offsets[frame - 1]
                continue
            step = step * (room / max(float(np.linalg.norm(step)), 1e-12))
            projected = offsets[frame - 1] + step
        offsets[frame] = projected
        if float(np.linalg.norm(step)) > 1e-15:
            applied_steps += 1

    def within_dynamics(candidate: np.ndarray) -> bool:
        if len(candidate) > 1:
            step_norms = np.linalg.norm(np.diff(candidate, axis=0), axis=1)
            if np.any(step_norms > config.max_root_shift_step_m + 1e-12):
                return False
        _summary, violations = _correction_dynamics(candidate, rate, config, "m")
        return not violations

    taper_end: int | None = None
    terminal_offset = float(np.linalg.norm(offsets[end]))
    if terminal_offset > 0.0 and end < total - 1:
        available = 0
        for frame in range(end + 1, min(total, end + 1 + config.skate_taper_frames)):
            if not contact_free[frame]:
                break
            available += 1

        initial_step = (
            offsets[end] - offsets[end - 1] if end > 0 else offsets[end].copy()
        )
        tapered: np.ndarray | None = None
        for span in range(1, available + 1):
            candidate = offsets.copy()
            for index in range(1, span + 1):
                fraction = index / span
                h00 = 2.0 * fraction ** 3 - 3.0 * fraction ** 2 + 1.0
                h10 = fraction ** 3 - 2.0 * fraction ** 2 + fraction
                candidate[end + index] = (
                    h00 * offsets[end] + h10 * (span * initial_step)
                )
            if within_dynamics(candidate):
                tapered = candidate
                taper_end = end + span
                break
        if tapered is None:
            return {
                "side": side,
                "start_frame": int(start),
                "end_frame": int(end),
                "applied": False,
                "failure": "no_contact_free_dynamics_safe_taper",
                "available_contact_free_taper_frames": int(available),
                "applied_steps": 0,
                "net_offset_xy_m": [0.0, 0.0],
                "tapered_to_zero_by_frame": None,
            }
        offsets = tapered

    if not within_dynamics(offsets):
        return {
            "side": side,
            "start_frame": int(start),
            "end_frame": int(end),
            "applied": False,
            "failure": "pinning_correction_dynamics_exceeded",
            "available_contact_free_taper_frames": 0,
            "applied_steps": 0,
            "net_offset_xy_m": [0.0, 0.0],
            "tapered_to_zero_by_frame": None,
        }

    working[:, 0:2] += offsets
    return {
        "side": side,
        "start_frame": int(start),
        "end_frame": int(end),
        "applied": True,
        "applied_steps": int(applied_steps),
        "net_offset_xy_m": [float(offsets[end][0]), float(offsets[end][1])],
        "tapered_to_zero_by_frame": int(taper_end) if taper_end is not None else None,
    }


# ---------------------------------------------------------------------------
# Correction coherence
# ---------------------------------------------------------------------------


def _correction_dynamics(corrections: np.ndarray, rate: float,
                         config: ContactRefineConfig, unit: str,
                         label_map: Any = None) -> tuple[dict[str, float], list[dict[str, Any]]]:
    """Step/velocity/acceleration summaries plus bound-violation records."""
    steps = np.diff(corrections, axis=0)
    velocity = np.zeros_like(corrections)
    velocity[1:] = steps * rate
    acceleration = np.zeros_like(corrections)
    acceleration[1:] = np.diff(velocity, axis=0) * rate
    vel_bound = (config.max_correction_velocity_m_s if unit == "m"
                 else config.max_correction_velocity_rad_s)
    acc_bound = (config.max_correction_acceleration_m_s2 if unit == "m"
                 else config.max_correction_acceleration_rad_s2)
    rejections: list[dict[str, Any]] = []

    def label(index: int) -> Any:
        if label_map is None:
            return int(index)
        return label_map[index]

    for frame, axis in np.argwhere(np.abs(velocity) > vel_bound + 1e-12):
        rejections.append({
            "frame": int(frame),
            "time_s": float(frame / rate),
            "reason": f"correction_velocity_limit_exceeded_{unit}",
            "axis_or_joint": label(int(axis)),
            "value": float(velocity[frame, axis]),
        })
    for frame, axis in np.argwhere(np.abs(acceleration) > acc_bound + 1e-12):
        rejections.append({
            "frame": int(frame),
            "time_s": float(frame / rate),
            "reason": f"correction_acceleration_limit_exceeded_{unit}",
            "axis_or_joint": label(int(axis)),
            "value": float(acceleration[frame, axis]),
        })
    return {
        "max_step": float(np.max(np.abs(steps))) if len(steps) else 0.0,
        "max_velocity": float(np.max(np.abs(velocity))),
        "max_acceleration": float(np.max(np.abs(acceleration))),
    }, rejections


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------


def refine_reference_contacts(
    qpos: Any,
    fps: Any,
    *,
    model_path: str | Path,
    config: ContactRefineConfig | None = None,
    source_sha256: str | None = None,
    source_metadata: Mapping[str, Any] | None = None,
) -> ContactRefineResult:
    """Accept or conservatively repair world-contact artifacts in a reference.

    A returned ``qpos`` is proof that every output frame was revalidated by an
    independent FK pass: no sole point penetrates beyond tolerance and no
    repairable skating residual remains.  Otherwise ``qpos`` is ``None`` and
    the report explains which frames failed and why.
    """
    settings = config or ContactRefineConfig()
    settings.validate()
    try:
        reference, rate = _validate_reference(qpos, fps)
    except ControllerScreenError as exc:
        raise ContactRefineError(str(exc)) from exc
    source = reference.copy()
    hash_before = array_sha256(source)

    mujoco = _require_mujoco()
    model_file = Path(model_path).resolve()
    if not model_file.is_file():
        raise ContactRefineError(f"MuJoCo model does not exist: {model_file}")
    model = mujoco.MjModel.from_xml_path(str(model_file))
    data = mujoco.MjData(model)
    _validate_model_layout(model, mujoco, model_file)
    feet = _resolve_feet(model, mujoco)
    joint_limits, limit_provenance = resolve_urdf_joint_limits(
        settings.limits_urdf_path, ContactRefineError
    )
    column_of = {name: index for index, name in enumerate(G1_29DOF_JOINTS)}

    # ---- stage A on the input: honest detection snapshot ----
    input_pass = _fk_sole_pass(model, data, mujoco, feet, source)
    input_contacts = {}
    for side, arrays in input_pass.items():
        states, threshold = infer_contact_states(arrays["z"], rate, settings)
        input_contacts[side] = {
            "stance_frames": int(np.count_nonzero(states)),
            "threshold_m": round(threshold, 4),
        }

    # ---- stage B: per-frame penetration repair ----
    working = source.copy()
    mesh_cache: dict[int, np.ndarray] = {}
    independent_geoms = _geoms_independent_of_columns(
        model, mujoco, feet, sorted(column_of[name] for name in ANKLE_ROLE_COLUMNS.values())
    )
    penetration_records: list[dict[str, Any]] = []
    penetration_failures = 0
    for frame in range(len(source)):
        row = working[frame]
        records, success = _repair_penetration_frame(
            model, data, mujoco, feet, row, column_of, settings, mesh_cache, joint_limits,
            independent_geoms,
        )
        if records:
            penetration_records.append({"frame": frame, "time_s": float(frame / rate),
                                        "repairs": records})
        if not success:
            penetration_failures += 1

    # ---- stage C: single-stance skating repair ----
    repaired_pass = _fk_sole_pass(model, data, mujoco, feet, working)
    repaired_states = {
        side: infer_contact_states(arrays["z"], rate, settings)[0]
        for side, arrays in repaired_pass.items()
    }
    skating_records: list[dict[str, Any]] = []
    skating_failures: list[dict[str, Any]] = []
    double_stance_skate_frames: set[int] = set()
    both_stance = repaired_states["left"] & repaired_states["right"]
    for side in sorted(feet):
        speeds = _slip_speeds(repaired_pass[side], rate)
        moving = np.zeros(len(speeds) + 1, dtype=bool)
        moving[1:] = speeds > settings.skate_speed_threshold_m_s
        double_stance_skate_frames.update(
            int(f) for f in np.flatnonzero(both_stance & moving)
        )
    if settings.skate_repair_enabled:
        contact_free = ~(repaired_states["left"] | repaired_states["right"])
        for side in sorted(feet):
            other = "right" if side == "left" else "left"
            single_stance = repaired_states[side] & ~repaired_states[other]
            for start, end in _contiguous_segments(single_stance):
                speeds = _slip_speeds(repaired_pass[side], rate, start, end)
                if float(np.max(speeds, initial=0.0)) <= settings.skate_speed_threshold_m_s:
                    continue
                summary = _repair_skating_segment(
                    working, side, start, end, repaired_pass, contact_free, rate, settings,
                )
                summary["pre_repair_max_speed_m_s"] = round(float(np.max(speeds)), 4)
                if summary["applied"]:
                    skating_records.append(summary)
                else:
                    skating_failures.append(summary)

    # ---- correction coherence ----
    root_corrections = working[:, 0:3] - source[:, 0:3]
    joint_corrections = working[:, 7:] - source[:, 7:]
    root_dynamics, root_violations = _correction_dynamics(
        root_corrections, rate, settings, "m"
    )
    joint_dynamics, joint_violations = _correction_dynamics(
        joint_corrections, rate, settings, "rad", list(G1_29DOF_JOINTS)
    )
    max_total_joint = float(np.max(np.abs(joint_corrections)))
    deviation_rejections: list[dict[str, Any]] = []
    if max_total_joint > settings.max_total_joint_deviation_rad + 1e-12:
        deviation_rejections.append({
            "reason": "total_joint_deviation_budget_exceeded",
            "max_per_frame_abs_rad": max_total_joint,
            "budget_rad": settings.max_total_joint_deviation_rad,
        })

    # ---- independent final verification pass ----
    # A fresh MjData so the verification shares no FK state with the search.
    final_pass = _fk_sole_pass(model, mujoco.MjData(model), mujoco, feet, working)
    final_states = {
        side: infer_contact_states(final_pass[side]["z"], rate, settings)[0]
        for side in sorted(feet)
    }
    residual_penetrations: list[dict[str, Any]] = []
    for frame in range(len(working)):
        worst = min(final_pass[side]["z"][frame] for side in final_pass)
        if worst < -settings.penetration_tolerance_m:
            residual_penetrations.append({
                "frame": int(frame),
                "time_s": float(frame / rate),
                "min_sole_z_m": float(worst),
            })
    final_contact_frames = int(np.count_nonzero(
        final_states["left"] | final_states["right"]
    ))
    residual_skates: list[dict[str, Any]] = []
    final_double_stance_skates: dict[int, dict[str, Any]] = {}
    final_both_stance = final_states["left"] & final_states["right"]
    for side in sorted(feet):
        other = "right" if side == "left" else "left"
        single_stance = final_states[side] & ~final_states[other]
        speeds = _slip_speeds(final_pass[side], rate)
        frame_speeds = np.zeros(len(working))
        frame_speeds[1:] = speeds
        for frame in np.flatnonzero(single_stance & (frame_speeds > settings.skate_speed_threshold_m_s)):
            residual_skates.append({
                "frame": int(frame),
                "time_s": float(frame / rate),
                "side": side,
                "speed_m_s": round(float(frame_speeds[frame]), 4),
            })
        for frame in np.flatnonzero(
            final_both_stance & (frame_speeds > settings.skate_speed_threshold_m_s)
        ):
            record = final_double_stance_skates.setdefault(
                int(frame),
                {
                    "frame": int(frame),
                    "time_s": float(frame / rate),
                    "speeds_m_s": {},
                },
            )
            record["speeds_m_s"][side] = round(float(frame_speeds[frame]), 4)

    rejections: list[dict[str, Any]] = []
    if penetration_failures:
        rejections.append({
            "reason": "penetration_unrepairable_within_budgets",
            "frames": penetration_failures,
        })
    if double_stance_skate_frames:
        sorted_double_stance = sorted(double_stance_skate_frames)
        rejections.append({
            "reason": "double_stance_skating_unrepaired_in_this_version",
            "count": len(sorted_double_stance),
            "examples": sorted_double_stance[:16],
        })
    if skating_failures:
        rejections.append({
            "reason": "skating_repair_unavailable_within_budgets",
            "count": len(skating_failures),
            "examples": skating_failures[:16],
        })
    rejections.extend(root_violations)
    rejections.extend(joint_violations)
    rejections.extend(deviation_rejections)
    if final_contact_frames == 0:
        rejections.append({
            "reason": "no_world_contact_observed",
            "contact_height_band_m": settings.contact_height_band_m,
        })
    if residual_penetrations:
        rejections.append({
            "reason": "final_validation_residual_penetration",
            "frames": [item["frame"] for item in residual_penetrations][:64],
            "count": len(residual_penetrations),
        })
    if residual_skates:
        rejections.append({
            "reason": "final_validation_residual_skating",
            "count": len(residual_skates),
            "examples": residual_skates[:16],
        })
    if final_double_stance_skates:
        final_double_examples = [
            final_double_stance_skates[frame]
            for frame in sorted(final_double_stance_skates)
        ]
        rejections.append({
            "reason": "final_validation_double_stance_skating",
            "count": len(final_double_examples),
            "examples": final_double_examples[:16],
        })

    hash_after = array_sha256(source)
    if hash_after != hash_before or not np.array_equal(source, reference):
        raise ContactRefineError("input reference was mutated during contact refinement")

    status = "REJECT" if rejections else "PASS"
    metadata = dict(source_metadata or {})
    report: dict[str, Any] = {
        "schema_version": CONTACT_REFINE_VERSION,
        "purpose": "world_contact_refinement_not_calibration",
        "status": status,
        "joint_limits": {
            **limit_provenance,
            "applied_to": "ankle pitch/roll correction bounds",
        },
        "hardware_candidate": {
            "status": (
                "BLOCKED"
                if metadata.get("offsets_key") == "zero-preview" or status != "PASS"
                else "CONTACT_REFINE_ONLY"
            ),
            "reasons": (
                (["preview_calibration"]
                 if metadata.get("offsets_key") == "zero-preview"
                 else [])
                + (["contact_refine_rejected"] if status != "PASS" else [])
            ),
        },
        "summary": {
            "frames": len(source),
            "input_stance_frames": {
                side: info["stance_frames"] for side, info in input_contacts.items()
            },
            "contact_thresholds_m": {
                side: info["threshold_m"] for side, info in input_contacts.items()
            },
            "penetration_repaired_frames": len(penetration_records),
            "penetration_failed_frames": penetration_failures,
            "skating_segments_repaired": len(skating_records),
            "skating_segments_unrepaired": len(skating_failures),
            "double_stance_skate_frames": len(double_stance_skate_frames),
            "final_contact_frames": final_contact_frames,
            "residual_penetration_frames": len(residual_penetrations),
            "residual_skating_frames": len(residual_skates),
            "residual_double_stance_skating_frames": len(final_double_stance_skates),
            "root_correction_dynamics": root_dynamics,
            "joint_correction_dynamics": joint_dynamics,
            "max_total_joint_deviation_rad": max_total_joint,
        },
        "input": {
            "frames": len(source),
            "fps": rate,
            "duration_s": (len(source) - 1) / rate,
            "source_sha256": source_sha256,
            "qpos_in_memory_sha256_before": hash_before,
            "qpos_in_memory_sha256_after": hash_after,
            "preserved": hash_before == hash_after,
            "offsets_key": metadata.get("offsets_key"),
        },
        "configuration": asdict(settings),
        "method": {
            "ground_plane": "world z=0 in the supplied MJCF frame; never estimated",
            "contact_inference": "kinematic sole height + vertical speed with hysteresis",
            "skating_measurement": (
                "horizontal slip of the contact material point (previous frame's "
                "lowest sole point carried rigidly with the foot body)"
            ),
            "penetration_search": (
                "per-frame bounded coarse scan + bisection over ankle pitch, ankle roll, "
                "then root-z lift; every candidate FK-verified against both feet"
            ),
            "skating_repair": (
                "single-stance root-xy pinning via negative material-point slip; "
                "velocity/acceleration bounded with contact-free taper only; double stance "
                "reported unrepaired"
            ),
            "calibration_estimation": "none",
            "geometric_clearance_claimed": False,
            "whole_trajectory_policy": (
                "reject_without_qpos_if_any_frame_or_correction_dynamics_check_fails"
            ),
        },
        "model": {
            "source_path": str(model_file),
            "source_sha256": sha256_file(model_file),
        },
        "penetration_repairs": penetration_records,
        "skating_repairs": skating_records,
        "skating_repair_failures": skating_failures,
        "rejections": rejections,
        "limitations": [
            "nonpenetration and skating are evaluated in the supplied MJCF world and do "
            "not prove real-robot clearance or contact stability",
            "double-stance skating is detected but deliberately not repaired in this version",
            "PASS requires at least one kinematically observed world-contact frame but does "
            "not prove that contact coverage is sufficient for balance or tracking",
            "lowest-point x/y are exact for sphere/capsule geoms and approximate (geom "
            "origin) for box/mesh geoms",
            "the result inherits source calibration quality and never estimates calibration",
            "no balance, torque, thermal, or communication behavior is validated",
        ],
    }
    return ContactRefineResult(
        qpos=None if status != "PASS" else working, fps=rate, report=report
    )


def write_refined_npz(path: str | Path, result: ContactRefineResult,
                      source_metadata: Mapping[str, Any]) -> None:
    """Atomically create an accepted derivative, refusing overwrite and rejects."""
    if result.qpos is None or result.report.get("status") != "PASS":
        raise ContactRefineError("cannot write a rejected contact-refinement result")
    target = Path(path)
    if target.suffix.lower() != ".npz":
        raise ContactRefineError("refined output path must end in .npz")
    metadata = dict(source_metadata)
    metadata["contact_refinement"] = {
        "schema_version": CONTACT_REFINE_VERSION,
        "purpose": "world_contact_refinement_not_calibration",
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
        exists_error=ContactRefineError,
    )
