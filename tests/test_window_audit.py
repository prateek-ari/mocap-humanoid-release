from __future__ import annotations

import hashlib
from pathlib import Path
import tempfile
import unittest

import numpy as np

from mocap_g1.schema import G1_29DOF_JOINTS, JointLimit
from mocap_g1.window_audit import (
    AuditConfig,
    WindowAuditError,
    audit_npz,
    load_npz_motion,
)


def make_limits(*, velocity: float = 1.0) -> dict[str, JointLimit]:
    return {
        name: JointLimit(lower=-10.0, upper=10.0, velocity=velocity)
        for name in G1_29DOF_JOINTS
    }


def qpos(frames: int) -> np.ndarray:
    values = np.zeros((frames, 36), dtype=np.float64)
    values[:, 3] = 1.0
    return values


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class WindowAuditTests(unittest.TestCase):
    def test_candidate_count_requires_an_integer(self):
        with self.assertRaisesRegex(WindowAuditError, "non-negative integer"):
            AuditConfig(candidate_count=2.5).validate()
        with self.assertRaisesRegex(WindowAuditError, "non-negative integer"):
            AuditConfig(candidate_count=True).validate()

    def test_qpos_load_and_source_preservation(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "motion.npz"
            values = qpos(6)
            np.savez_compressed(path, qpos=values, fps=np.asarray(2.0))
            before = digest(path)

            motion = load_npz_motion(path)
            report = audit_npz(
                path,
                make_limits(velocity=100.0),
                AuditConfig(window_seconds=1.0, discontinuity_rad=0.5),
            )

            self.assertEqual(motion.input_layout, "qpos")
            self.assertEqual(motion.dof_pos.shape, (6, 29))
            self.assertEqual(report["summary"]["status"], "PASS")
            self.assertEqual(report["source"]["sha256"], before)
            self.assertEqual(digest(path), before)

    def test_boundary_jump_is_assigned_to_next_window(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "motion.npz"
            values = qpos(6)
            # At 2 fps and 1 s windows, frame 2 starts window 1.  Its delta
            # crosses the boundary from frame 1 and must remain visible.
            values[2:, 7] = 1.0
            np.savez(path, qpos=values, fps=np.asarray(2.0))

            report = audit_npz(
                path,
                make_limits(velocity=1.0),
                AuditConfig(window_seconds=1.0, discontinuity_rad=0.5),
            )

            self.assertEqual(report["summary"]["status"], "FAIL")
            self.assertEqual(report["summary"]["velocity_violation_sample_count"], 1)
            self.assertEqual(report["summary"]["discontinuity_frame_count"], 1)
            first, second = report["windows"][:2]
            self.assertEqual(first["status"], "PASS")
            self.assertEqual(second["status"], "FAIL")
            self.assertEqual(second["discontinuities"][0]["frame"], 2)
            self.assertEqual(second["discontinuities"][0]["timestamp_s"], 1.0)
            violation = second["violations"]["velocity_by_joint"][0]
            self.assertEqual(violation["joint"], G1_29DOF_JOINTS[0])
            self.assertEqual(violation["frames"], [2])
            self.assertEqual(second["joint_speed_rad_s"]["max_abs"], 2.0)
            self.assertEqual(second["joint_acceleration_rad_s2"]["max_abs"], 4.0)

    def test_position_violation_is_reported_by_joint(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "motion.npz"
            values = qpos(4)
            values[1, 7 + 5] = 11.0
            np.savez(path, qpos=values, fps=np.asarray(1.0))

            report = audit_npz(
                path,
                make_limits(velocity=100.0),
                AuditConfig(window_seconds=4.0, discontinuity_rad=100.0),
            )

            violations = report["windows"][0]["violations"]["position_by_joint"]
            self.assertEqual(len(violations), 1)
            self.assertEqual(violations[0]["joint"], G1_29DOF_JOINTS[5])
            self.assertEqual(violations[0]["frames"], [1])
            self.assertEqual(violations[0]["max_excess"], 1.0)

    def test_position_comparison_tolerance_ignores_solver_noise_only(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            within = root / "within.npz"
            values = qpos(3)
            values[1, 7] = -10.0 - 0.5e-6
            np.savez(within, qpos=values, fps=np.asarray(1.0))
            report = audit_npz(
                within,
                make_limits(velocity=100.0),
                AuditConfig(window_seconds=3.0, discontinuity_rad=100.0),
            )
            self.assertEqual(report["summary"]["position_violation_sample_count"], 0)
            self.assertEqual(
                report["configuration"]["position_comparison_tolerance_rad"],
                1e-6,
            )

            outside = root / "outside.npz"
            values[1, 7] = -10.0 - 2e-6
            np.savez(outside, qpos=values, fps=np.asarray(1.0))
            report = audit_npz(
                outside,
                make_limits(velocity=100.0),
                AuditConfig(window_seconds=3.0, discontinuity_rad=100.0),
            )
            self.assertEqual(report["summary"]["position_violation_sample_count"], 1)

    def test_canonical_layout_and_candidates_are_deterministic(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "motion.npz"
            values = qpos(6)
            np.savez(
                path,
                root_pos=values[:, :3],
                root_rot=values[:, 3:7],
                dof_pos=values[:, 7:],
                fps=np.asarray(2.0),
            )
            report = audit_npz(
                path,
                make_limits(velocity=100.0),
                AuditConfig(window_seconds=1.0, candidate_count=2),
            )
            self.assertEqual(report["source"]["input_layout"], "canonical")
            self.assertEqual(
                [candidate["window_index"] for candidate in report["candidate_safe_windows"]["windows"]],
                [1, 2],
            )
            self.assertEqual(
                [candidate["rank"] for candidate in report["candidate_safe_windows"]["windows"]],
                [1, 2],
            )

    def test_nonfinite_and_nonunit_inputs_fail_closed(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            nonfinite = root / "nonfinite.npz"
            values = qpos(3)
            values[1, 7] = np.nan
            np.savez(nonfinite, qpos=values, fps=np.asarray(60.0))
            with self.assertRaisesRegex(WindowAuditError, "non-finite"):
                load_npz_motion(nonfinite)

            bad_rotation = root / "bad-rotation.npz"
            values = qpos(3)
            values[1, 3:7] = 0.0
            np.savez(bad_rotation, qpos=values, fps=np.asarray(60.0))
            with self.assertRaisesRegex(WindowAuditError, "non-unit quaternion"):
                load_npz_motion(bad_rotation)

    def test_incomplete_or_conflicting_layout_fails_closed(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            incomplete = root / "incomplete.npz"
            np.savez(
                incomplete,
                root_pos=np.zeros((3, 3)),
                root_rot=np.tile([1.0, 0.0, 0.0, 0.0], (3, 1)),
                fps=np.asarray(60.0),
            )
            with self.assertRaisesRegex(WindowAuditError, "incomplete canonical"):
                load_npz_motion(incomplete)

            conflict = root / "conflict.npz"
            values = qpos(3)
            dof = values[:, 7:].copy()
            dof[0, 0] = 1.0
            np.savez(
                conflict,
                qpos=values,
                root_pos=values[:, :3],
                root_rot=values[:, 3:7],
                dof_pos=dof,
                fps=np.asarray(60.0),
            )
            with self.assertRaisesRegex(WindowAuditError, "disagree"):
                load_npz_motion(conflict)


if __name__ == "__main__":
    unittest.main()
