"""Canonical G1 29-DoF names and official-URDF limit loading."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET


G1_29DOF_JOINTS = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)

G1_ARM_SDK_JOINTS = G1_29DOF_JOINTS[12:]


@dataclass(frozen=True)
class JointLimit:
    lower: float
    upper: float
    velocity: float


def load_official_joint_limits(urdf_path: str | Path) -> dict[str, JointLimit]:
    """Load all 29 position/velocity limits and reject incomplete URDFs."""
    root = ET.parse(str(urdf_path)).getroot()
    raw: dict[str, JointLimit] = {}
    for joint in root.iter("joint"):
        name = joint.get("name")
        limit = joint.find("limit")
        if name not in G1_29DOF_JOINTS or limit is None:
            continue
        try:
            raw[name] = JointLimit(
                lower=float(limit.attrib["lower"]),
                upper=float(limit.attrib["upper"]),
                velocity=float(limit.attrib["velocity"]),
            )
        except KeyError as exc:
            raise ValueError(f"{urdf_path}: joint {name} lacks {exc.args[0]} limit") from exc

    missing = [name for name in G1_29DOF_JOINTS if name not in raw]
    if missing:
        raise ValueError(f"{urdf_path}: missing G1 joint limits: {', '.join(missing)}")
    return {name: raw[name] for name in G1_29DOF_JOINTS}
