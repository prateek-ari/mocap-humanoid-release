from __future__ import annotations

import json
import hashlib
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

from mocap_g1.conditioning import (
    ConditioningConfig,
    MotionConditioningError,
    condition_g1_motion,
)
from mocap_g1.schema import G1_29DOF_JOINTS


REPO_ROOT = Path(__file__).resolve().parents[1]


def caps(value: float) -> dict[str, float]:
    return {name: value for name in G1_29DOF_JOINTS}


def trajectory(frames: int = 121, fps: float = 60.0) -> np.ndarray:
    time = np.arange(frames) / fps
    qpos = np.zeros((frames, 36), dtype=np.float64)
    qpos[:, 0] = 0.1 * time
    angle = 0.25 * time
    qpos[:, 3] = np.cos(angle / 2.0)
    qpos[:, 6] = np.sin(angle / 2.0)
    qpos[:, 7:] = 0.1 * np.sin(2.0 * np.pi * time[:, None] + np.arange(29)[None, :] * 0.01)
    return qpos


class ConditioningTests(unittest.TestCase):
    def test_rejects_invalid_arrays_and_cap_maps(self):
        qpos = trajectory()
        invalid = qpos.copy()
        invalid[3, 7] = np.nan
        with self.assertRaisesRegex(MotionConditioningError, "non-finite"):
            condition_g1_motion(
                invalid,
                60.0,
                velocity_caps=caps(1.0),
                acceleration_caps=caps(4.0),
            )
        invalid = qpos.copy()
        invalid[0, 3:7] *= 0.5
        with self.assertRaisesRegex(MotionConditioningError, "unit length"):
            condition_g1_motion(
                invalid,
                60.0,
                velocity_caps=caps(1.0),
                acceleration_caps=caps(4.0),
            )
        incomplete = caps(1.0)
        incomplete.pop(G1_29DOF_JOINTS[0])
        with self.assertRaisesRegex(MotionConditioningError, "name exactly"):
            condition_g1_motion(
                qpos,
                60.0,
                velocity_caps=incomplete,
                acceleration_caps=caps(4.0),
            )

    def test_spike_filter_decimation_and_quaternion_validity(self):
        original = trajectory()
        original[60, 12] += 1.0
        untouched = original.copy()
        result = condition_g1_motion(
            original,
            60.0,
            velocity_caps=caps(5.0),
            acceleration_caps=caps(100.0),
            config=ConditioningConfig(
                decimation=2,
                lowpass_cutoff_hz=10.0,
                filter_taps=9,
                spike_threshold_rad=0.2,
            ),
        )
        np.testing.assert_array_equal(original, untouched)
        self.assertEqual(result.fps, 30.0)
        self.assertGreater(result.report["spikes"]["joint_samples_replaced"], 0)
        self.assertEqual(result.report["filter"]["introduced_phase_delay_s"], 0.0)
        np.testing.assert_allclose(np.linalg.norm(result.qpos[:, 3:7], axis=1), 1.0, atol=1e-12)
        self.assertTrue(np.isfinite(result.qpos).all())

    def test_global_time_scaling_satisfies_named_caps(self):
        fps = 20.0
        qpos = trajectory(frames=21, fps=fps)
        qpos[:, 7] = np.linspace(0.0, 2.0, len(qpos))
        # Give it an acceleration corner as well as an excessive velocity.
        qpos[10:, 7] += np.linspace(0.0, 2.0, len(qpos) - 10)
        result = condition_g1_motion(
            qpos,
            fps,
            velocity_caps=caps(0.8),
            acceleration_caps=caps(2.0),
            config=ConditioningConfig(
                lowpass_cutoff_hz=6.0,
                filter_taps=5,
                spike_threshold_rad=10.0,
                max_time_scale=20.0,
            ),
        )
        self.assertGreater(result.report["time_scaling"]["factor"], 1.0)
        remaining = result.report["limits"]["remaining_violations"]
        self.assertEqual(remaining["velocity_rad_s"]["samples_over_cap"], 0)
        self.assertEqual(remaining["acceleration_rad_s2"]["samples_over_cap"], 0)
        self.assertEqual(remaining["velocity_rad_s"]["joints"], {})
        self.assertGreater(result.report["output"]["duration_change_s"], 0.0)
        self.assertIn(G1_29DOF_JOINTS[0], result.report["limits"]["velocity_caps_rad_s"])

    def _limits(self, upper: float = 1.0) -> dict[str, tuple[float, float]]:
        return {name: (-upper, upper) for name in G1_29DOF_JOINTS}

    def _step_to_limit(self, frames: int = 121) -> np.ndarray:
        """Every joint steps from mid-range to exactly its upper limit."""
        qpos = trajectory(frames=frames)
        qpos[:, 7:] = 0.0
        qpos[frames // 2:, 7:] = 1.0
        return qpos

    def test_conditioning_preserves_joint_position_limits(self):
        qpos = self._step_to_limit()
        settings = ConditioningConfig(lowpass_cutoff_hz=6.0, filter_taps=31)
        generous = dict(velocity_caps=caps(1e6), acceleration_caps=caps(1e9))
        # Fixture guard: without limits the zero-phase FIR's Gibbs ringing
        # carries the in-range step past the limit.
        unconstrained = condition_g1_motion(qpos, 60.0, config=settings, **generous)
        self.assertGreater(float(np.max(unconstrained.qpos[:, 7:])), 1.0 + 1e-3)
        self.assertIsNone(unconstrained.report["limits"]["position"])

        result = condition_g1_motion(
            qpos, 60.0, config=settings, position_limits=self._limits(), **generous
        )
        joints = result.qpos[:, 7:]
        self.assertLessEqual(float(np.max(joints)), 1.0)
        self.assertGreaterEqual(float(np.min(joints)), -1.0)
        position = result.report["limits"]["position"]
        self.assertGreater(position["filter_overshoot_samples_clamped"], 0)
        self.assertEqual(position["source_violations"]["samples_outside_limit"], 0)
        self.assertEqual(position["output_violations"]["samples_outside_limit"], 0)
        json.dumps(result.report, allow_nan=False)  # strict-JSON report

    def test_time_scaled_output_stays_inside_limits(self):
        qpos = self._step_to_limit(frames=61)
        result = condition_g1_motion(
            qpos, 60.0, velocity_caps=caps(2.0), acceleration_caps=caps(20.0),
            config=ConditioningConfig(lowpass_cutoff_hz=6.0, filter_taps=15),
            position_limits=self._limits(),
        )
        self.assertGreater(result.report["time_scaling"]["factor"], 1.0)
        self.assertLessEqual(float(np.max(result.qpos[:, 7:])), 1.0)

    def test_source_violations_are_reported_not_hidden(self):
        qpos = self._step_to_limit()
        qpos[70:80, 7] = 1.2  # the SOURCE leaves range on one joint
        result = condition_g1_motion(
            qpos, 60.0, velocity_caps=caps(1e6), acceleration_caps=caps(1e9),
            config=ConditioningConfig(lowpass_cutoff_hz=6.0),
            position_limits=self._limits(),
        )
        position = result.report["limits"]["position"]
        self.assertEqual(
            set(position["source_violations"]["joints"]), {G1_29DOF_JOINTS[0]}
        )
        # Left for the downstream gate to fail, not silently clamped away.
        self.assertGreater(float(np.max(result.qpos[:, 7])), 1.0)
        self.assertIn(G1_29DOF_JOINTS[0], position["output_violations"]["joints"])

    def test_position_limit_map_is_validated(self):
        limits = self._limits()
        limits.pop(G1_29DOF_JOINTS[3])
        with self.assertRaisesRegex(MotionConditioningError, "position_limits must name"):
            condition_g1_motion(
                trajectory(), 60.0, velocity_caps=caps(5.0), acceleration_caps=caps(100.0),
                position_limits=limits,
            )
        limits = self._limits()
        limits[G1_29DOF_JOINTS[0]] = (1.0, -1.0)
        with self.assertRaisesRegex(MotionConditioningError, "lower < upper"):
            condition_g1_motion(
                trajectory(), 60.0, velocity_caps=caps(5.0), acceleration_caps=caps(100.0),
                position_limits=limits,
            )

    def test_cli_writes_distinct_npz_and_report(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source.npz"
            output = root / "conditioned.npz"
            metadata = {
                "robot": "unitree_g1",
                "root_quaternion_order": "wxyz",
                "offsets_key": "zero-preview",
            }
            np.savez_compressed(
                source,
                metadata=np.asarray(json.dumps(metadata)),
                qpos=trajectory(frames=31),
                fps=np.asarray(60.0),
            )
            original_bytes = source.read_bytes()
            command = [
                sys.executable,
                str(REPO_ROOT / "tools/condition_g1_motion.py"),
                str(source),
                str(output),
                "--start-seconds",
                "0.1",
                "--end-seconds",
                "0.4",
                "--filter-taps",
                "5",
            ]
            completed = subprocess.run(command, cwd=REPO_ROOT, capture_output=True, text=True)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertEqual(source.read_bytes(), original_bytes)
            report_path = output.with_suffix(".report.json")
            self.assertTrue(output.is_file())
            self.assertTrue(report_path.is_file())
            report = json.loads(report_path.read_text(encoding="utf-8"))
            self.assertEqual(report["purpose"], "offline_motion_conditioning_not_calibration")
            self.assertEqual(
                report["provenance"]["output_sha256"],
                hashlib.sha256(output.read_bytes()).hexdigest(),
            )
            self.assertEqual(report["source_selection"]["start_frame_inclusive"], 6)
            self.assertEqual(report["source_selection"]["end_frame_exclusive"], 24)
            with np.load(output, allow_pickle=False) as archive:
                conditioned_metadata = json.loads(str(archive["metadata"].item()))
                self.assertIn("conditioning", conditioned_metadata)
                self.assertEqual(
                    conditioned_metadata["conditioning"]["source_selection"][
                        "start_frame_inclusive"
                    ],
                    6,
                )
                self.assertEqual(archive["qpos"].shape[1], 36)
            repeated = subprocess.run(command, cwd=REPO_ROOT, capture_output=True, text=True)
            self.assertEqual(repeated.returncode, 2)
            self.assertIn("refusing to overwrite", repeated.stderr)


if __name__ == "__main__":
    unittest.main()
