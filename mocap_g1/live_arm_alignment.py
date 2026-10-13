"""Live neutral-pose arm alignment for a GMR retargeter.

Live MVN arm segment frames have no validated robot offsets in this project,
and copying another robot's arm offsets picks the wrong elbow branch.  This
module applies the method used for recorded T1 calibration
(``tools/retarget/retarget_robot_recorded.py --arm-mode npose``) to the live
stream: with the performer standing still, arms down, solve the arm *positions*
only, read each robot arm frame's orientation, and store

    offset = inverse(human segment orientation) * robot frame orientation

so GMR's ``human_rotation * offset`` reproduces the robot frame for that pose.
Orientation tracking is then enabled with the configured task costs.

GMR is duck-typed (``human_body_to_task1/2``, ``rot_offsets1/2``, ``retarget``,
``model``, ``configuration``) so the logic is testable without GMR.
"""

from __future__ import annotations

from typing import Any, Iterable

import numpy as np

#: Repeated position-only solves on the same neutral frame; GMR's IK is
#: incremental, so a few passes let the arms settle from the current pose.
CALIBRATION_SOLVES = 8


class ArmAlignmentError(RuntimeError):
    pass


def _quat_multiply_wxyz(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.asarray(
        (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ),
        dtype=np.float64,
    )


def _unit(quaternion: Any, *, label: str) -> np.ndarray:
    value = np.asarray(quaternion, dtype=np.float64)
    norm = float(np.linalg.norm(value))
    if value.shape != (4,) or not np.isfinite(value).all() or norm <= 1e-8:
        raise ArmAlignmentError(f"{label} is not a usable quaternion")
    return value / norm


def offset_wxyz(human_wxyz: Any, robot_wxyz: Any) -> np.ndarray:
    """``inverse(human) * robot`` for unit quaternions in w, x, y, z order."""
    human = _unit(human_wxyz, label="human orientation")
    robot = _unit(robot_wxyz, label="robot orientation")
    conjugate = human * np.asarray((1.0, -1.0, -1.0, -1.0))
    result = _quat_multiply_wxyz(conjugate, robot)
    return result if result[0] >= 0.0 else -result


class LiveArmAlignment:
    """Switch a live retargeter between position-only and calibrated arms."""

    def __init__(self, retargeter: Any, arm_bodies: Iterable[str]) -> None:
        self.retargeter = retargeter
        self.arm_bodies = tuple(arm_bodies)
        self._tasks: list[tuple[str, Any, Any]] = []
        for table in ("human_body_to_task1", "human_body_to_task2"):
            for body, task in getattr(retargeter, table).items():
                if body in self.arm_bodies:
                    self._tasks.append((body, task, np.asarray(task.orientation_cost).copy()))
        if not self._tasks:
            raise ArmAlignmentError("the IK config has no tasks for the live arm segments")
        self.mode = "config"
        self.calibrations = 0

    def use_position_only(self) -> None:
        """Track arm positions only; orientations come from the IK solution."""
        for _body, task, _cost in self._tasks:
            task.set_orientation_cost(0.0)
        self.mode = "position-only"

    def _restore_orientation(self) -> None:
        for _body, task, cost in self._tasks:
            task.set_orientation_cost(cost)

    def calibrate(self, frame: dict[str, tuple[Any, Any]]) -> dict[str, list[float]]:
        """Derive arm offsets from one neutral-pose frame and enable them.

        Returns the derived offsets keyed by human segment.  On failure the
        previous mode and offsets are restored.
        """
        retargeter = self.retargeter
        missing = [body for body in self.arm_bodies if body not in frame]
        if missing:
            raise ArmAlignmentError(f"neutral frame lacks arm segments: {', '.join(missing)}")
        previous_mode = self.mode
        previous_offsets = {
            name: dict(getattr(retargeter, name)) for name in ("rot_offsets1", "rot_offsets2")
        }
        self.use_position_only()
        try:
            for _ in range(CALIBRATION_SOLVES):
                retargeter.retarget(
                    {name: (np.array(pose[0]), np.array(pose[1])) for name, pose in frame.items()},
                    offset_to_ground=False,
                )
            data = retargeter.configuration.data
            derived: dict[str, list[float]] = {}
            for body, task, _cost in self._tasks:
                robot_quat = data.xquat[retargeter.model.body(task.frame_name).id]
                offset = offset_wxyz(frame[body][1], robot_quat)
                for name in ("rot_offsets1", "rot_offsets2"):
                    table = getattr(retargeter, name)
                    if body in table:
                        table[body] = type(table[body]).from_quat(offset, scalar_first=True)
                derived[body] = [float(value) for value in offset]
        except Exception:
            for name, offsets in previous_offsets.items():
                getattr(retargeter, name).clear()
                getattr(retargeter, name).update(offsets)
            if previous_mode == "position-only":
                self.use_position_only()
            else:
                self._restore_orientation()
                self.mode = previous_mode
            raise
        self._restore_orientation()
        self.mode = "calibrated"
        self.calibrations += 1
        return derived


def floor_offset_m(retargeter: Any, feet: Iterable[tuple[str, float]]) -> float:
    """Root height correction that puts the lower sole on z=0 in the current IK pose.

    Call right after a ``retarget`` of a standing neutral frame.  The live
    stream is not grounded (GMR runs with ``offset_to_ground=False``), so the
    scaled pelvis height can leave the feet above or below the floor.
    """
    data = retargeter.configuration.data
    model = retargeter.model
    soles = [float(data.xpos[model.body(body).id][2]) - depth for body, depth in feet]
    if not soles or not all(np.isfinite(soles)):
        raise ArmAlignmentError("cannot measure the robot feet for floor calibration")
    return -min(soles)
