"""Robots the live Xsens MuJoCo preview can drive.

Each entry fixes the simulator joint layout the live loop expects from GMR, so
the filter, upsampler and command screening can be sized before GMR loads.
The live tool verifies the layout against the loaded MuJoCo model at startup.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from .schema import G1_29DOF_JOINTS, G1_ARM_SDK_JOINTS

REPO_ROOT = Path(__file__).resolve().parents[1]

#: Hinge joints of GMR's ``assets/booster_t1_29dof/t1_mocap.xml`` (pinned GMR
#: revision) in qpos order, with their MJCF position ranges in radians.  The
#: mocap asset has 27 hinges despite the ``29dof`` registry key; MJCF carries no
#: velocity limits.
T1_MOCAP_JOINT_RANGES: tuple[tuple[str, float, float], ...] = (
    ("Left_Shoulder_Pitch", -3.31, 1.22),
    ("Left_Shoulder_Roll", -1.74, 1.57),
    ("Left_Elbow_Pitch", -2.27, 2.27),
    ("Left_Elbow_Yaw", -2.27, 0.0),
    ("Left_Wrist_Pitch", -2.27, 2.27),
    ("Left_Wrist_Yaw", -2.27, 2.27),
    ("Left_Hand_Roll", -2.27, 2.27),
    ("Right_Shoulder_Pitch", -3.31, 1.22),
    ("Right_Shoulder_Roll", -1.57, 1.74),
    ("Right_Elbow_Pitch", -2.27, 2.27),
    ("Right_Elbow_Yaw", 0.0, 2.27),
    ("Right_Wrist_Pitch", -2.27, 2.27),
    ("Right_Wrist_Yaw", -2.27, 2.27),
    ("Right_Hand_Roll", -2.27, 2.27),
    ("Waist", -1.57, 1.57),
    ("Left_Hip_Pitch", -1.8, 1.57),
    ("Left_Hip_Roll", -0.3, 1.57),
    ("Left_Hip_Yaw", -1.0, 1.0),
    ("Left_Knee_Pitch", 0.0, 2.34),
    ("Left_Ankle_Pitch", -0.87, 0.35),
    ("Left_Ankle_Roll", -0.44, 0.44),
    ("Right_Hip_Pitch", -1.8, 1.57),
    ("Right_Hip_Roll", -1.57, 0.3),
    ("Right_Hip_Yaw", -1.0, 1.0),
    ("Right_Knee_Pitch", 0.0, 2.34),
    ("Right_Ankle_Pitch", -0.87, 0.35),
    ("Right_Ankle_Roll", -0.44, 0.44),
)
T1_MOCAP_JOINTS = tuple(name for name, _lower, _upper in T1_MOCAP_JOINT_RANGES)
#: Waist and arm hinges: the joints Space enables/disables, as on the G1.
T1_UPPER_BODY_JOINTS = tuple(
    name for name in T1_MOCAP_JOINTS if "Hip" not in name and "Knee" not in name
    and "Ankle" not in name
)

#: Arm segments of the GMR ``xsens_mvn`` source (MXTP02 converted names).
MVN_ARM_BODIES = (
    "Left_UpperArm",
    "Left_Forearm",
    "Left_Hand",
    "Right_UpperArm",
    "Right_Forearm",
    "Right_Hand",
)

ARM_ALIGNMENTS = ("config", "position-only")


@dataclass(frozen=True)
class LiveRobot:
    key: str
    label: str
    nq: int
    dof_joints: tuple[str, ...]
    upper_body_joints: tuple[str, ...]
    #: Project IK config installed into GMR for ``xsens_mvn``; ``None`` uses
    #: GMR's own registry entry.
    ik_config: Path | None
    #: Arm orientation at startup.  With "position-only", N in a neutral pose
    #: also calibrates arm orientation; with "config", N calibrates floor only.
    default_arm_alignment: str
    #: Foot bodies and the depth of their lowest collision geometry below the
    #: body origin at the zero pose (metres), for the N-key floor calibration.
    feet: tuple[tuple[str, float], ...]
    #: Outward arm rotation about the shoulder so hanging arms clear the hips
    #: (see mocap_g1/live_arm_clearance.py).
    arm_abduction_deg: float


LIVE_ROBOTS: dict[str, LiveRobot] = {
    "unitree_g1": LiveRobot(
        key="unitree_g1",
        label="G1",
        nq=7 + len(G1_29DOF_JOINTS),
        dof_joints=G1_29DOF_JOINTS,
        upper_body_joints=G1_ARM_SDK_JOINTS,
        ik_config=None,
        # GMR's xsens_mvn_to_g1.json arm offsets were tuned by the Xsens author
        # on a live MVN stream.  On a recorded MVN take they keep every elbow
        # bending the performer's way; position-only arms reversed some elbows
        # and a neutral-pose arm calibration captured an arbitrary upper-arm
        # twist (the straight-arm twist is unobservable from positions).
        default_arm_alignment="config",
        feet=(("left_ankle_roll_link", 0.052), ("right_ankle_roll_link", 0.052)),
        arm_abduction_deg=10.0,
    ),
    "booster_t1_29dof": LiveRobot(
        key="booster_t1_29dof",
        label="T1",
        nq=7 + len(T1_MOCAP_JOINTS),
        dof_joints=T1_MOCAP_JOINTS,
        upper_body_joints=T1_UPPER_BODY_JOINTS,
        ik_config=REPO_ROOT / "retarget_configs/xsens_mvn_to_booster_t1_29dof.json",
        # The T1 arm frames have no validated live MVN offsets; track arm
        # positions until a neutral-pose calibration supplies orientations.
        default_arm_alignment="position-only",
        feet=(("left_foot_link", 0.041), ("right_foot_link", 0.041)),
        # Standing hands sat 7.6 cm inside T1's hips; 22 deg leaves 2.2 cm and
        # no contact on the measured take (18 deg: 0.5 cm).
        arm_abduction_deg=22.0,
    ),
}
