"""Shared kinematic foot-contact primitives (purely kinematic, no force sensing).

Single source of truth for the three consumers that previously carried their
own copies:

  * ``mocap_g1.contact_refine``            -- the fail-closed repair pass;
  * ``eval/metrics.py`` (``eval/retarget_metrics``) -- PHUMA scoring;
  * ``tools/retarget/postprocess_grounding.py`` -- the preview grounding pass.

What is shared: the foot collision-geom selection, the orientation-aware
lowest point of a geom, the central difference used for vertical speed, the
"height band AND small vertical speed" contact predicate, and the
material-point slip used to measure horizontal skating.

What is deliberately NOT shared: the threshold VALUES.  Each consumer answers
a different question, so it passes its own named thresholds (see the
constants below and the comments next to each) instead of silently inheriting
another stage's tuning.

MuJoCo is imported lazily (callers may pass the module explicitly) so that
importing this module never requires the runtime extra.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np


# --- named per-consumer contact thresholds ---------------------------------
#
# contact_refine (repair): a 2 cm band around the world plane with a LOOSE
# 0.4 m/s vertical-speed gate.  The repair must see the whole stance phase
# including heel-strike settling, otherwise a skating segment is split into
# fragments too short to pin; it is fail-closed because every repair is
# re-verified by an independent final FK pass.
REFINE_CONTACT_HEIGHT_BAND_M = 0.02
REFINE_CONTACT_VZ_MAX_M_S = 0.4
#
# eval "anchored" policy (scoring): a 3 cm band with a TIGHT 0.05 m/s gate.
# The scoring denominator must contain only genuinely planted frames; a foot
# swinging through the band near heel-strike/toe-off would otherwise inflate
# it with transient frames (see eval/metrics.py, "Contact inference").
METRIC_CONTACT_HEIGHT_M = 0.03
METRIC_ANCHOR_VZ_MAX_M_S = 0.05
# eval "adaptive" policy and the preview grounding pass use a percentile
# threshold plus this band and a 0.2 m/s gate (the grounding CLI exposes its
# own looser default; see that tool's --vz-max help text).
METRIC_ADAPTIVE_BAND_M = 0.03
METRIC_ADAPTIVE_VZ_MAX_M_S = 0.2


def central_diff(values: np.ndarray, rate: float) -> np.ndarray:
    """Central difference along axis 0, one-sided at the ends."""
    values = np.asarray(values, dtype=np.float64)
    velocity = np.empty_like(values)
    if len(values) < 2:
        velocity[:] = 0.0
        return velocity
    velocity[1:-1] = (values[2:] - values[:-2]) * (rate / 2.0)
    velocity[0] = (values[1] - values[0]) * rate
    velocity[-1] = (values[-1] - values[-2]) * rate
    return velocity


def foot_contact_geoms(model: Any, body: int) -> tuple[list[int], bool]:
    """Collision geoms of ``body`` (contype/conaffinity != 0).

    Falls back to every geom of the body for visual-only test models.
    Returns ``(geoms, used_visual_fallback)``; ``geoms`` is empty only when
    the body owns no geoms at all.
    """
    geoms = [
        geom
        for geom in range(model.ngeom)
        if model.geom_bodyid[geom] == body
        and (model.geom_contype[geom] != 0 or model.geom_conaffinity[geom] != 0)
    ]
    if geoms:
        return geoms, False
    return [geom for geom in range(model.ngeom) if model.geom_bodyid[geom] == body], True


def _mujoco(module: Any) -> Any:
    if module is not None:
        return module
    import mujoco  # noqa: PLC0415 -- deliberately lazy

    return mujoco


def geom_lowest_point(model: Any, data: Any, geom: int, mesh_cache: dict[int, np.ndarray],
                      mujoco: Any = None) -> tuple[float, np.ndarray]:
    """World (z, xyz) of a geom's lowest point at the current FK state.

    Exact for spheres and capsules.  For boxes/cylinders/ellipsoids/meshes the
    z is exact (orientation-aware) but the returned x/y are the geom origin's
    (documented approximation).  Other types fall back to the bounding
    sphere.  ``mesh_cache`` is keyed by mesh id.
    """
    types = _mujoco(mujoco).mjtGeom
    center = np.asarray(data.geom_xpos[geom], dtype=np.float64)
    rotation = np.asarray(data.geom_xmat[geom], dtype=np.float64).reshape(3, 3)
    size = model.geom_size[geom]
    geom_type = model.geom_type[geom]
    if geom_type == types.mjGEOM_SPHERE:
        radius = float(size[0])
        return float(center[2] - radius), center - np.array([0.0, 0.0, radius])
    if geom_type == types.mjGEOM_CAPSULE:
        axis = rotation[:, 2] * float(size[1])
        end_a = center + axis
        end_b = center - axis
        lower = end_a if end_a[2] <= end_b[2] else end_b
        radius = float(size[0])
        return float(lower[2] - radius), lower - np.array([0.0, 0.0, radius])
    if geom_type == types.mjGEOM_BOX:
        lowest_z = center[2] - float(np.abs(rotation[2, :]) @ size[:3])
    elif geom_type == types.mjGEOM_CYLINDER:
        c = abs(rotation[2, 2])
        s = math.sqrt(max(0.0, 1.0 - c * c))
        lowest_z = center[2] - (float(size[1]) * c + float(size[0]) * s)
    elif geom_type == types.mjGEOM_ELLIPSOID:
        lowest_z = center[2] - float(math.sqrt(float(np.sum((rotation[2, :] * size[:3]) ** 2))))
    elif geom_type == types.mjGEOM_MESH:
        mesh_id = int(model.geom_dataid[geom])
        if mesh_id not in mesh_cache:
            address, count = model.mesh_vertadr[mesh_id], model.mesh_vertnum[mesh_id]
            mesh_cache[mesh_id] = np.array(
                model.mesh_vert[address:address + count], dtype=np.float64
            )
        verts = mesh_cache[mesh_id]
        lowest_z = center[2] + float((rotation[2, :] @ verts.T).min())
    else:
        lowest_z = center[2] - float(model.geom_rbound[geom])
    return float(lowest_z), np.array([center[0], center[1], lowest_z])


def geom_lowest_z(model: Any, data: Any, geom: int, mesh_cache: dict[int, np.ndarray],
                  mujoco: Any = None) -> float:
    """World z of the lowest point of ``geom`` at the current FK state."""
    return geom_lowest_point(model, data, geom, mesh_cache, mujoco)[0]


def lowest_point_of_geoms(model: Any, data: Any, geoms: list[int],
                          mesh_cache: dict[int, np.ndarray],
                          mujoco: Any = None) -> tuple[float, np.ndarray]:
    """Lowest (z, xyz) over ``geoms``; ties keep the first geom."""
    best_z = math.inf
    best_point: np.ndarray | None = None
    for geom in geoms:
        low_z, point = geom_lowest_point(model, data, geom, mesh_cache, mujoco)
        if low_z < best_z:
            best_z, best_point = low_z, point
    if best_point is None:
        raise ValueError("cannot measure a lowest point over an empty geom list")
    return best_z, best_point


def contact_candidates(height: np.ndarray, vertical_speed: np.ndarray | None, *,
                       height_max: float, vz_max: float | None) -> np.ndarray:
    """Raw per-frame contact predicate: ``height < height_max`` AND, when
    ``vz_max`` is given, ``|vertical_speed| < vz_max``.

    The single definition every consumer applies; hysteresis/debouncing and
    threshold selection (fixed, percentile-adaptive, ...) stay with callers.
    """
    mask = np.asarray(height, dtype=np.float64) < height_max
    if vz_max is not None:
        if vertical_speed is None:
            raise ValueError("vertical_speed is required when vz_max is given")
        mask &= np.abs(np.asarray(vertical_speed, dtype=np.float64)) < vz_max
    return mask


def material_point_slip_xy(points: np.ndarray, body_pos: np.ndarray,
                           body_mat: np.ndarray) -> np.ndarray:
    """Per-frame horizontal displacement of the foot's contact MATERIAL point.

    For each frame ``f >= 1``, the lowest point at ``f-1`` is expressed in the
    foot body's frame at ``f-1`` and carried rigidly to the body pose at
    ``f``; the result is that point's horizontal displacement (row 0 is 0).

    This is the quantity skating actually is -- the sliding of the patch of
    sole that touches the ground -- and, unlike differencing the argmin
    lowest point frame-to-frame, it does not jump when a sub-millimetre
    pitch/roll dither flips WHICH corner of a flat foot is lowest.  A foot
    pivoting about its planted toe or heel correctly reads ~0 as well.  A
    pure root translation shifts it exactly by the translation, so the
    contact-refinement rigid compensation remains exact.

    ``points`` (T,3) world lowest points, ``body_pos`` (T,3) world body
    origins, ``body_mat`` (T,3,3) world body rotations.
    """
    points = np.asarray(points, dtype=np.float64)
    body_pos = np.asarray(body_pos, dtype=np.float64)
    body_mat = np.asarray(body_mat, dtype=np.float64)
    slip = np.zeros((len(points), 2), dtype=np.float64)
    if len(points) < 2:
        return slip
    local = np.einsum("tji,tj->ti", body_mat[:-1], points[:-1] - body_pos[:-1])
    carried = body_pos[1:] + np.einsum("tij,tj->ti", body_mat[1:], local)
    slip[1:] = carried[:, :2] - points[:-1, :2]
    return slip
