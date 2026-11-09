"""Robots the live Xsens MuJoCo preview can drive.

Each entry fixes the simulator joint layout the live loop expects from GMR, so
the filter, upsampler and command screening can be sized before GMR loads.
The live tool verifies the layout against the loaded MuJoCo model at startup.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from .arm_collision_profiles import K1_ARM_PROFILE, RobotArmProfile
from .schema import G1_29DOF_JOINTS, G1_ARM_SDK_JOINTS, JointLimit, load_official_joint_limits

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

#: Hinge joints of GMR's ``assets/booster_k1/K1_serial.xml`` (the 22-DOF Booster
#: K1) in qpos order, which is also the Booster SDK's ``JointIndexK1`` motor order,
#: with their position ranges (rad): the intersection of Booster's official ranges
#: (booster_assets ``robots/K1/K1_22dof.urdf`` @12a9751, BSD-3-Clause) and GMR's
#: model, so the simulator stays inside both.
K1_JOINT_RANGES: tuple[tuple[str, float, float], ...] = (
    ("AAHead_yaw", -1.0, 1.0),
    ("Head_pitch", -0.314, 0.794),
    ("ALeft_Shoulder_Pitch", -2.932, 1.196),
    ("Left_Shoulder_Roll", -1.642, 1.57),
    ("Left_Elbow_Pitch", -1.885, 1.885),
    ("Left_Elbow_Yaw", -2.242, 0.0),
    ("ARight_Shoulder_Pitch", -2.932, 1.196),
    ("Right_Shoulder_Roll", -1.57, 1.629),
    ("Right_Elbow_Pitch", -1.885, 1.885),
    ("Right_Elbow_Yaw", 0.0, 2.242),
    ("Left_Hip_Pitch", -2.958, 2.21),
    ("Left_Hip_Roll", -0.375, 1.536),
    ("Left_Hip_Yaw", -1.0, 1.0),
    ("Left_Knee_Pitch", 0.0, 2.23),
    ("Left_Ankle_Pitch", -0.87, 0.345),
    ("Left_Ankle_Roll", -0.345, 0.345),
    ("Right_Hip_Pitch", -2.958, 2.21),
    ("Right_Hip_Roll", -1.536, 0.375),
    ("Right_Hip_Yaw", -1.0, 1.0),
    ("Right_Knee_Pitch", 0.0, 2.23),
    ("Right_Ankle_Pitch", -0.87, 0.345),
    ("Right_Ankle_Roll", -0.345, 0.345),
)
K1_JOINTS = tuple(name for name, _lower, _upper in K1_JOINT_RANGES)
#: The arms; the K1 has no waist, and its head is not driven by the live IK.
K1_UPPER_BODY_JOINTS = K1_JOINTS[2:10]
#: Official velocity (rad/s) and effort (N*m) limits per joint group.
_K1_LEG_LIMITS = {"Hip_Pitch": (14.66, 68.0), "Hip_Roll": (12.57, 43.0),
                  "Hip_Yaw": (17.59, 38.3), "Knee_Pitch": (12.57, 112.0),
                  "Ankle_Pitch": (17.59, 38.3), "Ankle_Roll": (17.59, 38.3)}
K1_VELOCITY_LIMITS: tuple[tuple[str, float], ...] = (
    ("AAHead_yaw", 7.85), ("Head_pitch", 7.85),
    *((name, 33.51) for name in K1_UPPER_BODY_JOINTS),
    *((f"{side}_{joint}", velocity) for side in ("Left", "Right")
      for joint, (velocity, _effort) in _K1_LEG_LIMITS.items()),
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
    #: Position ranges of ``dof_joints`` for the simulation bounds; ``None`` (the G1)
    #: reads them from the official URDF.
    joint_ranges: tuple[tuple[str, float, float], ...] | None = None
    #: URDF with lower, upper and velocity limits for every joint.
    limits_urdf: Path | None = None
    #: Upper-body velocity limits (rad/s) when no URDF supplies them.
    velocity_limits: tuple[tuple[str, float], ...] | None = None
    #: Arm-versus-body collision pairs for the K1 hardware envelope's contact
    #: check; ``None`` runs the envelope without one.
    arm_collision: RobotArmProfile | None = None


LIVE_ROBOTS: dict[str, LiveRobot] = {
    "booster_k1": LiveRobot(
        key="booster_k1",
        label="K1",
        nq=7 + len(K1_JOINTS),
        dof_joints=K1_JOINTS,
        upper_body_joints=K1_UPPER_BODY_JOINTS,
        ik_config=REPO_ROOT / "retarget_configs/xsens_mvn_to_booster_k1.json",
        # No wrist, so no hand target: with position-only arms nothing points the
        # forearm. The world-aligned zero-pose frames take the T1 config's offsets.
        default_arm_alignment="config",
        # Collision box bottom 0.024 m below the foot body origin at the zero pose.
        feet=(("left_foot_link", 0.024), ("right_foot_link", 0.024)),
        arm_abduction_deg=22.0,
        joint_ranges=K1_JOINT_RANGES,
        velocity_limits=K1_VELOCITY_LIMITS,
        arm_collision=K1_ARM_PROFILE,
    ),
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
        joint_ranges=T1_MOCAP_JOINT_RANGES,
    ),
}


def robot_joint_ranges(
    robot: LiveRobot, limits_urdf: Path | None = None
) -> tuple[tuple[str, float, float], ...]:
    """Position ranges of every joint, from the profile or its limits URDF."""
    urdf = limits_urdf or robot.limits_urdf
    if urdf is None:
        if robot.joint_ranges is None:
            raise ValueError(f"{robot.label} profile has neither joint ranges nor a limits URDF")
        return robot.joint_ranges
    limits = load_official_joint_limits(urdf, robot.dof_joints)
    return tuple((name, limits[name].lower, limits[name].upper) for name in robot.dof_joints)


def robot_envelope_limits(
    robot: LiveRobot, limits_urdf: Path | None = None
) -> dict[str, JointLimit]:
    """Limits of the upper-body joints the hardware envelope guards.

    Raises ``ValueError`` naming the gap when the profile has no velocity
    limits: the envelope cannot run without them.
    """
    urdf = limits_urdf or robot.limits_urdf
    if urdf is not None:
        # Load every joint so an incomplete URDF is rejected, not just its guarded part.
        limits = load_official_joint_limits(urdf, robot.dof_joints)
        return {name: limits[name] for name in robot.upper_body_joints}
    if robot.velocity_limits is None:
        raise ValueError(
            f"{robot.label} has no joint velocity limits, so it cannot use the hardware envelope"
        )
    velocity = dict(robot.velocity_limits)
    missing = [name for name in robot.upper_body_joints if name not in velocity]
    if missing:
        raise ValueError(f"{robot.label} lacks velocity limits for: {', '.join(missing)}")
    ranges = {name: (lower, upper) for name, lower, upper in robot_joint_ranges(robot)}
    return {
        name: JointLimit(lower=ranges[name][0], upper=ranges[name][1], velocity=velocity[name])
        for name in robot.upper_body_joints
    }
