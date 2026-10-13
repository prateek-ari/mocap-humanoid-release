from __future__ import annotations

from tools.retarget.project_robot_arm_collisions import ROBOT_PROFILES


def test_profiles_keep_arm_chains_and_body_targets_disjoint() -> None:
    for profile in ROBOT_PROFILES.values():
        assert profile.left_arm_bodies.isdisjoint(profile.right_arm_bodies)
        assert profile.left_distal_bodies <= profile.left_arm_bodies
        assert profile.right_distal_bodies <= profile.right_arm_bodies
        assert profile.left_arm_bodies.isdisjoint(profile.target_bodies)
        assert profile.right_arm_bodies.isdisjoint(profile.target_bodies)
        assert len(profile.left_joint_names) == len(profile.right_joint_names) == 7
        assert len(profile.tracked_bodies) == len(profile.position_weights)
        assert len(profile.tracked_bodies) == len(profile.orientation_weights)
