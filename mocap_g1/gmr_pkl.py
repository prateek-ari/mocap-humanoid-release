"""GMR motion-pkl root-quaternion contract shared by every writer and loader.

GMR's own writer (``scripts/bvh_to_robot.py``: ``qpos[3:7][[1, 2, 3, 0]]``)
and loader (``general_motion_retargeting/data_loader.py``: "from xyzw to
wxyz") store ``root_rot`` in scalar-last **xyzw** order, while MuJoCo
``qpos`` -- and every NPZ in this repo -- is scalar-first wxyz.  A silent
order mix-up does not crash anything: it just scores, renders or exports a
robot rotated by a garbage quaternion.  So the contract is explicit:

  * every pkl writer in this repo stores ``root_rot`` as xyzw AND an explicit
    ``root_quat_order: "xyzw"`` key (``declare_gmr_root_quat_order``);
  * every loader calls ``resolve_root_quat_order``, which refuses a pkl with
    no key unless the caller passes an explicit ``legacy_order`` (only for
    pre-contract files whose provenance is known), and refuses any declared
    order other than xyzw or one that conflicts with the override.
"""

from __future__ import annotations

import math
from typing import Any, Mapping, MutableMapping

import numpy as np


GMR_ROOT_QUAT_ORDER = "xyzw"
ROOT_QUAT_ORDER_KEY = "root_quat_order"
QUAT_ORDERS = ("xyzw", "wxyz")
#: Same tolerance the dataset loader and conditioning already apply.
UNIT_QUATERNION_TOLERANCE = 1e-3


class RootQuatOrderError(ValueError):
    """Raised when a pkl's root quaternion order is missing or inconsistent."""


def declare_gmr_root_quat_order(motion: MutableMapping[str, Any]) -> MutableMapping[str, Any]:
    """Stamp the explicit order key on a pkl payload whose root_rot is xyzw."""
    motion[ROOT_QUAT_ORDER_KEY] = GMR_ROOT_QUAT_ORDER
    return motion


def declared_root_quat_order(motion: Mapping[str, Any], *,
                             source: object = "motion") -> str | None:
    """The order a pkl explicitly declares, or None.

    The canonical declaration is the top-level ``root_quat_order`` key.
    ``metadata.layout.root_rot`` (written by retarget_robot_recorded.py before
    the key existed) is accepted as an equally explicit declaration; if both
    are present they must agree.
    """
    declarations = []
    if motion.get(ROOT_QUAT_ORDER_KEY) is not None:
        declarations.append(motion[ROOT_QUAT_ORDER_KEY])
    metadata = motion.get("metadata")
    layout = metadata.get("layout") if isinstance(metadata, Mapping) else None
    if isinstance(layout, Mapping) and layout.get("root_rot") is not None:
        declarations.append(layout["root_rot"])
    if not declarations:
        return None
    if any(value != declarations[0] for value in declarations[1:]):
        raise RootQuatOrderError(
            f"{source}: conflicting root quaternion order declarations {declarations!r}"
        )
    return declarations[0]


def resolve_root_quat_order(motion: Mapping[str, Any], *, legacy_order: str | None = None,
                            source: object = "motion") -> str:
    """Return the order ``root_rot`` is stored in, or refuse.

    ``legacy_order`` is an explicit caller override for pre-contract files
    that carry no ``root_quat_order`` key; it is never applied silently.
    """
    if legacy_order is not None and legacy_order not in QUAT_ORDERS:
        raise RootQuatOrderError(
            f"legacy quaternion order must be one of {QUAT_ORDERS}, got {legacy_order!r}"
        )
    declared = declared_root_quat_order(motion, source=source)
    if declared is None:
        if legacy_order is None:
            raise RootQuatOrderError(
                f"{source}: GMR pkl has no {ROOT_QUAT_ORDER_KEY!r} key, so its root_rot "
                f"order is unknown; re-export it with a current writer, or pass an "
                f"explicit legacy order (e.g. --quat-order xyzw for a GMR-native "
                f"bvh_to_robot.py/retarget_headless.py file) if you know its provenance"
            )
        return legacy_order
    if declared != GMR_ROOT_QUAT_ORDER:
        raise RootQuatOrderError(
            f"{source}: declares {ROOT_QUAT_ORDER_KEY}={declared!r}; GMR pkl root_rot "
            f"must be {GMR_ROOT_QUAT_ORDER!r}"
        )
    if legacy_order is not None and legacy_order != declared:
        raise RootQuatOrderError(
            f"{source}: explicit quaternion order {legacy_order!r} conflicts with the "
            f"declared {ROOT_QUAT_ORDER_KEY}={declared!r}"
        )
    return declared


def validate_unit_quaternions(quaternions: Any, *, source: object = "root_rot",
                              tolerance: float = UNIT_QUATERNION_TOLERANCE) -> np.ndarray:
    """Require a finite (T,4) array of unit quaternions; return it as float64."""
    values = np.asarray(quaternions, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 4:
        raise ValueError(f"{source}: must have shape (T, 4), got {values.shape}")
    if not np.isfinite(values).all():
        raise ValueError(f"{source}: contains non-finite values")
    if len(values):
        error = np.abs(np.linalg.norm(values, axis=1) - 1.0)
        worst = int(np.argmax(error))
        if not math.isfinite(float(error[worst])) or error[worst] > tolerance:
            raise ValueError(
                f"{source}: non-unit quaternion at frame {worst} "
                f"(|norm-1|={float(error[worst]):.3g} > {tolerance:g})"
            )
    return values


#: Index permutations between scalar-first (MuJoCo qpos / NPZ) and
#: scalar-last (GMR pkl / SciPy default) quaternions, applied on the last axis.
_WXYZ_TO_XYZW = [1, 2, 3, 0]
_XYZW_TO_WXYZ = [3, 0, 1, 2]


def wxyz_to_xyzw(quaternions: Any) -> np.ndarray:
    """Scalar-first -> scalar-last on the last axis (a copy; no validation)."""
    return np.asarray(quaternions)[..., _WXYZ_TO_XYZW]


def xyzw_to_wxyz(quaternions: Any) -> np.ndarray:
    """Scalar-last -> scalar-first on the last axis (a copy; no validation)."""
    return np.asarray(quaternions)[..., _XYZW_TO_WXYZ]


def root_rot_as(motion: Mapping[str, Any], order: str, *, legacy_order: str | None = None,
                source: object = "motion") -> np.ndarray:
    """Validated ``root_rot`` converted to ``order`` ("xyzw" or "wxyz")."""
    if order not in QUAT_ORDERS:
        raise RootQuatOrderError(f"target order must be one of {QUAT_ORDERS}, got {order!r}")
    stored = resolve_root_quat_order(motion, legacy_order=legacy_order, source=source)
    values = validate_unit_quaternions(motion["root_rot"], source=f"{source}: root_rot")
    if stored == order:
        return values
    return xyzw_to_wxyz(values) if order == "wxyz" else wxyz_to_xyzw(values)
