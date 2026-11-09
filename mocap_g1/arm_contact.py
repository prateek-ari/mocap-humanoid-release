"""Arm-versus-body penetration of a robot pose, for the envelope's contact check.

Uses the robot profile's arm collision pairs (mocap_g1/arm_collision_profiles.py):
each arm's distal links against the torso, pelvis and legs.  Distances are
fixed-base, so the floating root does not matter.  A bounding-sphere test skips
pairs that cannot touch, which keeps a pose check to a fraction of a millisecond.
"""

from __future__ import annotations

from typing import Any, Mapping, Sequence

import numpy as np

from .arm_collision_profiles import RobotArmProfile, geometry_pairs


class ArmContact:
    """Deepest penetration of each arm (``"left"``, ``"right"``) at a pose.

    :meth:`at` binds the joints the envelope does not guard (the legs) for one
    step; :meth:`depths` then takes the guarded joints by name, as
    :class:`mocap_g1.safety.ContactCheck` requires.
    """

    def __init__(
        self,
        mujoco: Any,
        model: Any,
        profile: RobotArmProfile,
        dof_joints: Sequence[str],
    ) -> None:
        self._mujoco = mujoco
        self._model = model
        self._data = mujoco.MjData(model)
        self._qpos = np.array(model.qpos0, dtype=np.float64)
        self._index = {name: 7 + i for i, name in enumerate(dof_joints)}
        self.groups = {
            "left": tuple(name for name in profile.left_joint_names if name in self._index),
            "right": tuple(name for name in profile.right_joint_names if name in self._index),
        }
        pairs = geometry_pairs(mujoco, model, profile)
        self._pairs: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
        for group, distal in (
            ("left", profile.left_distal_bodies),
            ("right", profile.right_distal_bodies),
        ):
            mine = [pair for pair in pairs if pair.body1 in distal or pair.body2 in distal]
            first = np.asarray([pair.geom1 for pair in mine], dtype=int)
            second = np.asarray([pair.geom2 for pair in mine], dtype=int)
            reach = model.geom_rbound[first] + model.geom_rbound[second]
            self._pairs[group] = (first, second, reach)
        self.pair_count = len(pairs)
        self._fromto = np.zeros(6, dtype=np.float64)
        # Upper bound on how far any point of a group's pairs moves per radian
        # of each joint: the joint anchor to the geom's far side along the
        # kinematic chain (fixed link lengths, so it holds in every pose).
        # Doubled because both geoms of a pair may hang below the same joint.
        self._levers: dict[str, dict[str, float]] = {}
        for group, (first, second, _reach) in self._pairs.items():
            geoms = set(first.tolist()) | set(second.tolist())
            self._levers[group] = {
                name: 2.0 * max((self._chain_reach(joint, geom) for geom in geoms), default=0.0)
                for name, joint in self._joint_ids(mujoco, model, dof_joints).items()
            }

    @staticmethod
    def _joint_ids(mujoco: Any, model: Any, dof_joints: Sequence[str]) -> dict[str, int]:
        ids = {name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
               for name in dof_joints}
        missing = sorted(name for name, joint in ids.items() if joint < 0)
        if missing:  # a missing joint would get no lever and break the early exit
            raise ValueError(f"robot model lacks joints: {', '.join(missing)}")
        return ids

    def _chain_reach(self, joint: int, geom: int) -> float:
        """Joint anchor to the geom's bounding sphere along the kinematic
        chain, or zero when the joint does not move the geom."""
        model = self._model
        body, anchor_body = int(model.geom_bodyid[geom]), int(model.jnt_bodyid[joint])
        length = float(np.linalg.norm(model.geom_pos[geom]) + model.geom_rbound[geom])
        while body != anchor_body:
            if body == 0:
                return 0.0
            length += float(np.linalg.norm(model.body_pos[body]))
            body = int(model.body_parentid[body])
        return length + float(np.linalg.norm(model.jnt_pos[joint]))

    def clear_of(self, positions: Mapping[str, float], travel: Mapping[str, float]) -> bool:
        """Whether no pair can touch while each named joint moves at most
        ``travel`` radians from ``positions`` (the bound legs stay put): every
        pair's distance exceeds how far its geoms can move.  Bounding spheres
        settle most pairs; the rest are measured."""
        self._pose(positions)
        xpos = self._data.geom_xpos
        for group, (first, second, reach) in self._pairs.items():
            levers = self._levers[group]
            movement = sum(levers.get(name, 0.0) * abs(value) for name, value in travel.items())
            gaps = np.linalg.norm(xpos[first] - xpos[second], axis=1) - reach
            for index in np.flatnonzero(gaps <= movement):
                distance = self._mujoco.mj_geomDistance(
                    self._model, self._data, int(first[index]), int(second[index]),
                    movement + 1e-3, self._fromto,
                )
                if distance <= movement:
                    return False
        return True

    def _pose(self, positions: Mapping[str, float]) -> None:
        for name, value in positions.items():
            self._qpos[self._index[name]] = value
        self._data.qpos[:] = self._qpos
        self._mujoco.mj_kinematics(self._model, self._data)

    def at(self, dof_pos: np.ndarray) -> ArmContact:
        """Use ``dof_pos`` for every joint not passed to :meth:`depths`."""
        self._qpos[7:] = dof_pos
        return self

    def depths(self, positions: Mapping[str, float]) -> dict[str, float]:
        self._pose(positions)
        xpos = self._data.geom_xpos
        result = {}
        for group, (first, second, reach) in self._pairs.items():
            gaps = np.linalg.norm(xpos[first] - xpos[second], axis=1) - reach
            deepest = 0.0
            for index in np.flatnonzero(gaps < 0.0):  # bounding spheres overlap
                distance = self._mujoco.mj_geomDistance(
                    self._model, self._data, int(first[index]), int(second[index]), 0.0,
                    self._fromto,
                )
                deepest = max(deepest, -float(distance))
            result[group] = deepest
        return result

    def pose_depth(self, dof_pos: np.ndarray) -> float:
        """Deepest penetration of either arm at a whole DOF vector (for audits)."""
        return max(self.at(dof_pos).depths({}).values())
