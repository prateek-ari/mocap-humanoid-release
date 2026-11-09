"""Arm-versus-body collision proxies for the supported humanoid MJCFs.

Shared by the offline arm projection (``tools/retarget/project_robot_arm_collisions.py``)
and the live preview, which adds the same pairs to the IK as a collision limit.
Only distal arm/hand geometry against torso and leg geometry is declared:
shoulder-chain contacts with the torso are structural in these models.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


class ArmProfileError(ValueError):
    """Raised when a robot model does not resolve to any declared proxy pair."""


@dataclass(frozen=True)
class RobotArmProfile:
    """Names that define one robot's arm-only collision gate."""

    left_joint_names: tuple[str, ...]
    right_joint_names: tuple[str, ...]
    left_arm_bodies: frozenset[str]
    right_arm_bodies: frozenset[str]
    left_distal_bodies: frozenset[str]
    right_distal_bodies: frozenset[str]
    target_bodies: frozenset[str]
    tracked_bodies: tuple[str, ...]
    position_weights: tuple[float, ...]
    orientation_weights: tuple[float, ...]


@dataclass(frozen=True)
class GeometryPair:
    """One collidable MuJoCo proxy pair included in the arm gate."""

    geom1: int
    geom2: int
    body1: str
    body2: str
    category: str


T1_TARGET_BODIES = frozenset(
    {"Trunk", "Waist", "left_foot_link", "right_foot_link", "left_toe_link", "right_toe_link"}
    | {
        f"{segment}_{side}"
        for segment in ("Hip_Pitch", "Hip_Roll", "Hip_Yaw", "Shank", "Ankle_Cross")
        for side in ("Left", "Right")
    }
)
G1_TARGET_BODIES = frozenset(
    {"pelvis", "pelvis_contour_link", "waist_yaw_link", "waist_roll_link", "torso_link"}
    | {
        f"{side}_{segment}_link"
        for side in ("left", "right")
        for segment in (
            "hip_pitch",
            "hip_roll",
            "hip_yaw",
            "knee",
            "ankle_pitch",
            "ankle_roll",
            "toe",
        )
    }
)


#: The Booster K1 (GMR ``K1_serial.xml``): the 23-DOF T1's arm chain under other
#: names, and no waist, so the trunk is the root body.
K1_ARM_PROFILE = RobotArmProfile(
    left_joint_names=(
        "ALeft_Shoulder_Pitch", "Left_Shoulder_Roll", "Left_Elbow_Pitch", "Left_Elbow_Yaw",
    ),
    right_joint_names=(
        "ARight_Shoulder_Pitch", "Right_Shoulder_Roll", "Right_Elbow_Pitch", "Right_Elbow_Yaw",
    ),
    left_arm_bodies=frozenset({"Left_Arm_1", "Left_Arm_2", "Left_Arm_3", "left_hand_link"}),
    right_arm_bodies=frozenset({"Right_Arm_1", "Right_Arm_2", "Right_Arm_3", "right_hand_link"}),
    left_distal_bodies=frozenset({"Left_Arm_3", "left_hand_link"}),
    right_distal_bodies=frozenset({"Right_Arm_3", "right_hand_link"}),
    target_bodies=frozenset(
        {"Trunk", "left_foot_link", "right_foot_link"}
        | {
            f"{side}_{segment}"
            for segment in ("Hip_Pitch", "Hip_Roll", "Hip_Yaw", "Shank", "Ankle_Cross")
            for side in ("Left", "Right")
        }
    ),
    tracked_bodies=(
        "Left_Arm_2", "Left_Arm_3", "left_hand_link", "Right_Arm_2", "Right_Arm_3", "right_hand_link",
    ),
    position_weights=(5.0, 10.0, 10.0, 5.0, 10.0, 10.0),
    orientation_weights=(20.0, 15.0, 15.0, 20.0, 15.0, 15.0),
)


ROBOT_PROFILES = {
    "booster_t1_29dof": RobotArmProfile(
        left_joint_names=(
            "Left_Shoulder_Pitch",
            "Left_Shoulder_Roll",
            "Left_Elbow_Pitch",
            "Left_Elbow_Yaw",
            "Left_Wrist_Pitch",
            "Left_Wrist_Yaw",
            "Left_Hand_Roll",
        ),
        right_joint_names=(
            "Right_Shoulder_Pitch",
            "Right_Shoulder_Roll",
            "Right_Elbow_Pitch",
            "Right_Elbow_Yaw",
            "Right_Wrist_Pitch",
            "Right_Wrist_Yaw",
            "Right_Hand_Roll",
        ),
        left_arm_bodies=frozenset({*(f"AL{i}" for i in range(1, 7)), "left_hand_link"}),
        right_arm_bodies=frozenset({*(f"AR{i}" for i in range(1, 7)), "right_hand_link"}),
        left_distal_bodies=frozenset({*(f"AL{i}" for i in range(3, 7)), "left_hand_link"}),
        right_distal_bodies=frozenset({*(f"AR{i}" for i in range(3, 7)), "right_hand_link"}),
        target_bodies=T1_TARGET_BODIES,
        tracked_bodies=("AL2", "AL4", "AL5", "AR2", "AR4", "AR5"),
        position_weights=(5.0, 10.0, 10.0, 5.0, 10.0, 10.0),
        orientation_weights=(20.0, 15.0, 15.0, 20.0, 15.0, 15.0),
    ),
    "unitree_g1": RobotArmProfile(
        left_joint_names=(
            "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint",
            "left_elbow_joint",
            "left_wrist_roll_joint",
            "left_wrist_pitch_joint",
            "left_wrist_yaw_joint",
        ),
        right_joint_names=(
            "right_shoulder_pitch_joint",
            "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",
            "right_elbow_joint",
            "right_wrist_roll_joint",
            "right_wrist_pitch_joint",
            "right_wrist_yaw_joint",
        ),
        left_arm_bodies=frozenset(
            {
                "left_shoulder_pitch_link",
                "left_shoulder_roll_link",
                "left_shoulder_yaw_link",
                "left_elbow_link",
                "left_wrist_roll_link",
                "left_wrist_pitch_link",
                "left_wrist_yaw_link",
                "left_rubber_hand",
            }
        ),
        right_arm_bodies=frozenset(
            {
                "right_shoulder_pitch_link",
                "right_shoulder_roll_link",
                "right_shoulder_yaw_link",
                "right_elbow_link",
                "right_wrist_roll_link",
                "right_wrist_pitch_link",
                "right_wrist_yaw_link",
                "right_rubber_hand",
            }
        ),
        # Shoulder-chain contacts with the torso are structural in the G1
        # model.  The gate starts at the elbow and includes every distal link.
        left_distal_bodies=frozenset(
            {
                "left_elbow_link",
                "left_wrist_roll_link",
                "left_wrist_pitch_link",
                "left_wrist_yaw_link",
                "left_rubber_hand",
            }
        ),
        right_distal_bodies=frozenset(
            {
                "right_elbow_link",
                "right_wrist_roll_link",
                "right_wrist_pitch_link",
                "right_wrist_yaw_link",
                "right_rubber_hand",
            }
        ),
        target_bodies=G1_TARGET_BODIES,
        tracked_bodies=(
            "left_shoulder_yaw_link",
            "left_elbow_link",
            "left_wrist_yaw_link",
            "right_shoulder_yaw_link",
            "right_elbow_link",
            "right_wrist_yaw_link",
        ),
        position_weights=(5.0, 10.0, 10.0, 5.0, 10.0, 10.0),
        orientation_weights=(20.0, 15.0, 15.0, 20.0, 15.0, 15.0),
    ),
}


def body_name(mujoco: Any, model: Any, geom_id: int) -> str:
    body_id = int(model.geom_bodyid[geom_id])
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or f"body_{body_id}"


def geometry_pairs(mujoco: Any, model: Any, profile: RobotArmProfile) -> tuple[GeometryPair, ...]:
    """Enumerate enabled collision proxies used by the declared arm gate."""

    pairs: list[GeometryPair] = []
    for geom1 in range(1, model.ngeom):  # geom zero is the floor in both supported MJCFs.
        for geom2 in range(geom1 + 1, model.ngeom):
            enabled = (
                int(model.geom_contype[geom1]) & int(model.geom_conaffinity[geom2])
            ) or (
                int(model.geom_contype[geom2]) & int(model.geom_conaffinity[geom1])
            )
            if not enabled:
                continue
            body1 = body_name(mujoco, model, geom1)
            body2 = body_name(mujoco, model, geom2)
            distal_target = any(
                (body1 in distal and body2 in profile.target_bodies)
                or (body2 in distal and body1 in profile.target_bodies)
                for distal in (profile.left_distal_bodies, profile.right_distal_bodies)
            )
            if not distal_target:
                continue
            first_body, second_body = sorted((body1, body2))
            pairs.append(
                GeometryPair(
                    geom1=geom1,
                    geom2=geom2,
                    body1=first_body,
                    body2=second_body,
                    category="distal_to_body",
                )
            )
    if not pairs:
        raise ArmProfileError("arm collision profile resolved to zero enabled geometry pairs")
    return tuple(pairs)
