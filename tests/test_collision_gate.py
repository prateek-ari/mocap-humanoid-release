from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

from mocap_g1.collision_gate import (
    CollisionGateConfig,
    CollisionGateError,
    _find_global_safe_retreat,
    gate_reference_self_collisions,
)
from conftest import G1_MJCF as MODEL, requires_g1_mjcf


REPO_ROOT = Path(__file__).resolve().parents[1]


def reference(frames: int = 6, *, colliding: bool = False) -> np.ndarray:
    values = np.zeros((frames, 36), dtype=np.float64)
    values[:, 3] = 1.0
    if colliding:
        values[:, 7 + 22] = 0.30
        values[:, 7 + 23] = 0.25
        values[:, 7 + 25] = 0.45
    return values


class CollisionGateTests(unittest.TestCase):
    def test_config_validation(self):
        with self.assertRaisesRegex(CollisionGateError, "max_total"):
            CollisionGateConfig(max_total_deviation_rad=0.0).validate()
        with self.assertRaisesRegex(CollisionGateError, "max_correction_acceleration"):
            CollisionGateConfig(max_correction_acceleration_rad_s2=0.0).validate()
        with self.assertRaisesRegex(CollisionGateError, "global_alpha_guard"):
            CollisionGateConfig(global_alpha_guard_fraction=-0.1).validate()
        with self.assertRaisesRegex(CollisionGateError, "unknown"):
            CollisionGateConfig(projectable_joints=("not_a_joint",)).validate()
        with self.assertRaisesRegex(CollisionGateError, "projection_mode"):
            CollisionGateConfig(projection_mode="unknown").validate()

    def test_global_search_never_drops_an_unsafe_requested_guard(self):
        original = np.zeros((3, 29), dtype=np.float64)
        neutral = np.ones(29, dtype=np.float64)
        collision = {("body_a", "body_b"): -0.001}

        def pairs(joints: np.ndarray):
            # The unguarded boundary near 0.5 is safe, but adding the requested
            # 0.1 guard crosses into a second unsafe region.
            value = float(joints[0])
            return collision if value < 0.5 or value > 0.55 else {}

        result = _find_global_safe_retreat(
            original,
            neutral,
            [("single:test", (0,))],
            set(),
            CollisionGateConfig(
                max_per_joint_deviation_rad=1.0,
                max_total_deviation_rad=1.0,
                global_alpha_guard_fraction=0.1,
            ),
            pairs,
        )
        self.assertIsNone(result)

    @requires_g1_mjcf
    def test_collision_free_reference_is_bitwise_preserved(self):
        source = reference()
        before = source.copy()
        result = gate_reference_self_collisions(
            source, 60.0, model_path=MODEL,
            source_metadata={"offsets_key": "zero-preview"},
        )
        self.assertEqual(result.report["status"], "PASS")
        self.assertEqual(result.report["summary"]["projected_frames"], 0)
        self.assertTrue(np.array_equal(result.qpos, source))
        self.assertTrue(np.array_equal(source, before))
        self.assertEqual(result.report["hardware_candidate"]["status"], "BLOCKED")
        self.assertEqual(result.report["method"]["calibration_estimation"], "none")

    @requires_g1_mjcf
    def test_collision_is_projected_within_budgets(self):
        source = reference(colliding=True)
        before = source.copy()
        result = gate_reference_self_collisions(
            source,
            60.0,
            model_path=MODEL,
            config=CollisionGateConfig(
                max_per_joint_deviation_rad=0.25,
                max_total_deviation_rad=0.30,
            ),
        )
        self.assertEqual(result.report["status"], "PASS")
        self.assertEqual(result.report["summary"]["projected_frames"], len(source))
        self.assertEqual(result.report["summary"]["final_self_collision_frames"], 0)
        self.assertLessEqual(result.report["summary"]["max_per_joint_deviation_rad"], 0.25)
        self.assertLessEqual(result.report["summary"]["max_total_deviation_rad"], 0.30)
        self.assertTrue(np.array_equal(source, before))
        self.assertTrue(np.array_equal(result.qpos[:, :7], source[:, :7]))

    @requires_g1_mjcf
    def test_strict_budget_rejects_and_returns_no_trajectory(self):
        result = gate_reference_self_collisions(
            reference(colliding=True),
            60.0,
            model_path=MODEL,
            config=CollisionGateConfig(
                max_per_joint_deviation_rad=0.02,
                max_total_deviation_rad=0.03,
            ),
        )
        self.assertEqual(result.report["status"], "REJECT")
        self.assertIsNone(result.qpos)
        self.assertTrue(any(
            item["reason"] == "no_nonpenetrating_pose_within_deviation_budgets"
            for item in result.report["rejections"]
        ))

    @requires_g1_mjcf
    def test_abrupt_correction_rejects_on_dynamics(self):
        source = reference(frames=8)
        source[4:] = reference(frames=4, colliding=True)
        result = gate_reference_self_collisions(
            source,
            60.0,
            model_path=MODEL,
            config=CollisionGateConfig(
                max_per_joint_deviation_rad=0.25,
                max_total_deviation_rad=0.30,
            ),
        )
        self.assertEqual(result.report["status"], "REJECT")
        self.assertIsNone(result.qpos)
        reasons = {item["reason"] for item in result.report["rejections"]}
        self.assertIn("correction_velocity_limit_exceeded", reasons)
        self.assertIn("correction_acceleration_limit_exceeded", reasons)

    @requires_g1_mjcf
    def test_explicit_global_mode_uses_one_group_and_alpha(self):
        source = reference(colliding=True)
        result = gate_reference_self_collisions(
            source,
            60.0,
            model_path=MODEL,
            config=CollisionGateConfig(
                max_per_joint_deviation_rad=0.25,
                max_total_deviation_rad=0.30,
                projection_mode="global_only",
            ),
        )
        self.assertEqual(result.report["status"], "PASS")
        global_projection = result.report["global_projection"]
        self.assertIsNotNone(global_projection)
        self.assertTrue(global_projection["applied_to_all_frames"])
        strategies = {item["strategy"] for item in result.report["projected_frames"]}
        alphas = {
            item["neutral_interpolation_fraction"]
            for item in result.report["projected_frames"]
        }
        self.assertEqual(len(strategies), 1)
        self.assertEqual(len(alphas), 1)
        self.assertEqual(result.report["summary"]["final_self_collision_frames"], 0)

    @requires_g1_mjcf
    def test_global_fallback_replaces_incoherent_per_frame_corrections(self):
        source = reference(frames=12)
        source[:, 7 + 15] = 0.32
        source[:, 7 + 16] = -0.04
        source[:, 7 + 17] = np.linspace(0.10, -0.20, len(source))
        source[:, 7 + 18] = 0.47
        source[:, 7 + 22] = 0.30
        source[:, 7 + 23] = 0.25
        source[:, 7 + 25] = 0.45

        per_frame = gate_reference_self_collisions(
            source,
            60.0,
            model_path=MODEL,
            config=CollisionGateConfig(
                max_total_deviation_rad=0.30,
                projection_mode="per_frame_only",
            ),
        )
        self.assertEqual(per_frame.report["status"], "REJECT")

        fallback = gate_reference_self_collisions(
            source,
            60.0,
            model_path=MODEL,
            config=CollisionGateConfig(max_total_deviation_rad=0.30),
        )
        self.assertEqual(fallback.report["status"], "PASS")
        self.assertIsNotNone(fallback.report["global_projection"])
        self.assertEqual(
            fallback.report["global_projection"]["strategy"],
            "bilateral:shoulder_roll_joint",
        )
        self.assertEqual(
            fallback.report["summary"]["max_correction_acceleration_rad_s2"],
            0.0,
        )

    @requires_g1_mjcf
    def test_cli_writes_derivative_and_provenance_without_touching_source(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.npz"
            output = root / "projected.npz"
            report = root / "report.json"
            metadata = {
                "robot": "unitree_g1",
                "root_quaternion_order": "wxyz",
                "offsets_key": "zero-preview",
            }
            np.savez_compressed(
                source,
                metadata=np.asarray(json.dumps(metadata, sort_keys=True)),
                qpos=reference(colliding=True),
                fps=np.asarray(60.0),
            )
            before = source.read_bytes()
            completed = subprocess.run(
                [
                    sys.executable,
                    str(REPO_ROOT / "tools/control/project_g1_self_collisions.py"),
                    str(source),
                    "--model", str(MODEL),
                    "--output", str(output),
                    "--report", str(report),
                    "--max-total-rad", "0.30",
                ],
                cwd=REPO_ROOT,
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertEqual(source.read_bytes(), before)
            self.assertTrue(output.is_file())
            payload = json.loads(report.read_text())
            self.assertEqual(payload["status"], "PASS")
            self.assertTrue(payload["input"]["source_file_preserved"])
            self.assertTrue(payload["output"]["written"])
            with np.load(output, allow_pickle=False) as archive:
                derived_metadata = json.loads(str(archive["metadata"].item()))
            self.assertEqual(
                derived_metadata["collision_projection"]["purpose"],
                "fixed_base_collision_projection_not_calibration",
            )


if __name__ == "__main__":
    unittest.main()
