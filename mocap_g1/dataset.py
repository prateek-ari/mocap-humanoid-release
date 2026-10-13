"""Offsets-calibration validation shared by the recorded retargeting tools."""

from __future__ import annotations

import math
from pathlib import Path
import re
from typing import Any


class DatasetError(RuntimeError):
    """Fail-closed error for invalid inputs or incomplete processing."""


def validate_offsets_for_bvh(offsets: dict[str, Any], bvh_path: str | Path) -> int:
    """Require explicit finite XYZ offsets for every articulated BVH joint."""
    path = Path(bvh_path)
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        hierarchy = handle.read(2 * 1024 * 1024).split("MOTION", 1)[0]
    joint_names = re.findall(r"^\s*(?:ROOT|JOINT)\s+([^\s{]+)", hierarchy, re.MULTILINE)
    if not joint_names:
        raise DatasetError(f"{path}: cannot find ROOT/JOINT names for offset validation")
    missing = [name for name in joint_names if name not in offsets]
    if missing:
        raise DatasetError(
            f"offsets calibration lacks {len(missing)} BVH joints: {', '.join(missing[:8])}"
            + (" ..." if len(missing) > 8 else "")
        )
    for name in joint_names:
        channels = offsets[name]
        if not isinstance(channels, dict):
            raise DatasetError(f"offsets calibration joint {name!r} must be an object")
        for axis in ("X", "Y", "Z"):
            value = channels.get(axis)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise DatasetError(f"offsets calibration {name}.{axis} must be numeric")
            if not math.isfinite(float(value)):
                raise DatasetError(f"offsets calibration {name}.{axis} must be finite")
    return len(joint_names)
