"""Fixed arm abduction so the robot's hanging arms clear its hips.

GMR's upstream ``xsens_mvn_to_g1.json`` drives the arms mainly by segment
orientation (upper-arm orientation weight 100; hand position is a light
correction), so a robot arm hangs at the performer's arm angle from the
robot's own shoulder.  The G1 hip housings stand out further than its
shoulders: arms hanging straight down, as a performer's do, pass through them.
Moving only hand *position* targets cannot fix this (widening hand targets 1.6x
removed 11% of the overlaps), and IK collision limits made the robot vibrate.

Instead every arm target (positions and orientations of upper arm, forearm and
hand) is rotated rigidly outward about the shoulder, around the chest's forward
axis.  On a recorded MVN take (840 frames, visual hand meshes included) a 10 degree
G1 abduction removed every hand/forearm overlap with the hips (429 -> 0), and
22 degrees did the same for the Booster T1 (561 -> 0, standing clearance
2.2 cm).  Standing jitter and elbow-bend error were unchanged and every elbow
kept bending the performer's way.  It is a constant offset: nothing switches
on or off between frames.

Robots whose arm orientation is calibrated from a neutral pose (T1) must
calibrate against the same abducted human rotations, otherwise the tilt is
applied twice; see :func:`abduct_frame_rotations`.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np

ARM_SEGMENTS = ("UpperArm", "Forearm", "Hand")


def _quat_matrix(wxyz) -> np.ndarray:
    w, x, y, z = np.asarray(wxyz, dtype=np.float64) / np.linalg.norm(wxyz)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def _quat_multiply(a, b) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def abduct_arms(scaled: dict, degrees: float, *, chest: str = "Chest") -> dict[str, tuple]:
    """Return arm targets rotated outward about each shoulder (left +, right -)."""
    forward = _quat_matrix(scaled[chest][1])[:, 0]
    moved: dict[str, tuple] = {}
    for side, sign in (("Left", 1.0), ("Right", -1.0)):
        angle = math.radians(degrees) * sign
        half = angle / 2.0
        rotation = np.array([math.cos(half), *(math.sin(half) * forward)])
        matrix = _quat_matrix(rotation)
        shoulder = np.asarray(scaled[f"{side}_UpperArm"][0], dtype=np.float64)
        for segment in ARM_SEGMENTS:
            name = f"{side}_{segment}"
            position, orientation = scaled[name]
            moved[name] = (
                shoulder + matrix @ (np.asarray(position, dtype=np.float64) - shoulder),
                _quat_multiply(rotation, np.asarray(orientation, dtype=np.float64)),
            )
    return moved


def abduct_frame_rotations(frame: dict, degrees: float) -> dict:
    """Copy of a GMR human frame with only the arm *rotations* abducted.

    Used for neutral-pose arm calibration: the arm positions are abducted by
    :class:`ArmAbduction` during the solve, so the derived orientation offsets
    must see the same abduction on the human side.
    """
    result = {name: (np.array(pose[0]), np.array(pose[1])) for name, pose in frame.items()}
    if degrees:
        for name, (_position, orientation) in abduct_arms(result, degrees).items():
            result[name] = (result[name][0], orientation)
    return result


class ArmAbduction:
    """Wraps a GMR retargeter's ``update_targets`` to abduct the arm targets."""

    def __init__(self, retargeter: Any, degrees: float) -> None:
        self.retargeter = retargeter
        self.degrees = float(degrees)
        self._update_targets = retargeter.update_targets
        retargeter.update_targets = self.update_targets

    def update_targets(self, human_data, offset_to_ground=False):
        self._update_targets(human_data, offset_to_ground)
        import mink

        retargeter = self.retargeter
        scaled = retargeter.scaled_human_data
        for name, (position, orientation) in abduct_arms(scaled, self.degrees).items():
            scaled[name] = (position, orientation)
            target = mink.SE3.from_rotation_and_translation(mink.SO3(orientation), position)
            for table in (retargeter.human_body_to_task1, retargeter.human_body_to_task2):
                task = table.get(name)
                if task is not None:
                    task.set_target(target)
