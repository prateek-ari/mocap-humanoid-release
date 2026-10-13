"""Subprocess smoke tests for the CLIs added in the recording-hardening tranche.

These tests exercise process-level contracts only: `--help` exits cleanly,
invalid arguments fail closed with a non-zero exit code, and a small
synthetic NPZ fixture produces a schema-shaped JSON/NPZ artifact. They
deliberately avoid asserting specific PASS/BLOCKED verdicts or exact flag
inventories, since controller-screen gains and `live_g1_sim.py` flags are
under active development elsewhere.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
from conftest import G1_MJCF as MODEL, requires_g1_mjcf


REPO_ROOT = Path(__file__).resolve().parents[1]
PYTHON = sys.executable



def make_qpos(frames: int = 121, fps: float = 60.0, joint_amplitude: float = 0.05) -> np.ndarray:
    """A small, well-inside-limits G1 qpos trajectory: identity root pose plus
    a gentle waist/elbow sinusoid."""
    qpos = np.zeros((frames, 36), dtype=np.float64)
    qpos[:, 3] = 1.0  # identity root quaternion, wxyz
    if joint_amplitude:
        time = np.arange(frames) / fps
        qpos[:, 7 + 15] = joint_amplitude * np.sin(2.0 * np.pi * 0.5 * time)
        qpos[:, 7 + 22] = joint_amplitude * np.sin(2.0 * np.pi * 0.5 * time)
    return qpos


def write_fixture_npz(
    path: Path, frames: int = 121, fps: float = 60.0, joint_amplitude: float = 0.05
) -> Path:
    """Write a minimal {metadata, qpos, fps} NPZ accepted by every CLI here."""
    metadata = {
        "robot": "unitree_g1",
        "root_quaternion_order": "wxyz",
        "offsets_key": "zero-preview",
    }
    np.savez_compressed(
        path,
        metadata=np.asarray(json.dumps(metadata)),
        qpos=make_qpos(frames=frames, fps=fps, joint_amplitude=joint_amplitude),
        fps=np.asarray(fps),
    )
    return path


def run_cli(script: Path, *args: str, env: dict[str, str] | None = None) -> subprocess.CompletedProcess:
    return subprocess.run(
        [PYTHON, str(script), *args],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        env=env,
    )


def env_without_mocap_ws() -> dict[str, str]:
    """A subprocess environment with MOCAP_WS unset, so --model becomes required
    deterministically regardless of the host's own environment."""
    env = dict(os.environ)
    env.pop("MOCAP_WS", None)
    return env


class AuditG1RecordingSmokeTest(unittest.TestCase):
    SCRIPT = REPO_ROOT / "tools/xsens/audit_g1_recording.py"

    def test_help_exits_zero(self):
        result = run_cli(self.SCRIPT, "--help")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("usage", result.stdout.lower())

    def test_missing_input_fails_closed_with_report(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            output = tmp_path / "audit.json"
            result = run_cli(
                self.SCRIPT,
                str(tmp_path / "does-not-exist.npz"),
                "--output",
                str(output),
            )
            self.assertEqual(result.returncode, 2)
            self.assertTrue(output.is_file())
            payload = json.loads(output.read_text())
            self.assertIn("error", payload)
            self.assertEqual(payload["summary"]["status"], "FAIL")

    def test_real_recording_writes_schema_shaped_report(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            recording = write_fixture_npz(tmp_path / "recording.npz", frames=121, fps=60.0)
            output = tmp_path / "audit.json"
            result = run_cli(
                self.SCRIPT,
                str(recording),
                "--output",
                str(output),
                "--window-seconds",
                "1.0",
                "--candidate-count",
                "1",
            )
            self.assertIn(result.returncode, (0, 1), result.stderr)
            self.assertTrue(output.is_file())
            payload = json.loads(output.read_text())
            self.assertIn("schema_version", payload)
            self.assertIn(payload["summary"]["status"], ("PASS", "FAIL"))
            self.assertIn("windows", payload)
            # The source recording must never be modified by an audit.
            with np.load(recording, allow_pickle=False) as archive:
                self.assertEqual(set(archive.files), {"metadata", "qpos", "fps"})


class AuditG1SelfCollisionsSmokeTest(unittest.TestCase):
    SCRIPT = REPO_ROOT / "tools/control/audit_g1_self_collisions.py"

    def test_help_exits_zero(self):
        result = run_cli(self.SCRIPT, "--help")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("usage", result.stdout.lower())

    def test_missing_model_argument_fails_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            recording = write_fixture_npz(tmp_path / "recording.npz", frames=6)
            output = tmp_path / "collision-audit.json"
            result = run_cli(
                self.SCRIPT,
                str(recording),
                "--output",
                str(output),
                env=env_without_mocap_ws(),
            )
            self.assertEqual(result.returncode, 2)
            self.assertFalse(output.exists())

    def test_missing_input_fails_closed_without_writing_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            output = tmp_path / "collision-audit.json"
            result = run_cli(
                self.SCRIPT,
                str(tmp_path / "does-not-exist.npz"),
                "--output",
                str(output),
                "--model",
                str(MODEL),
            )
            self.assertEqual(result.returncode, 2)
            self.assertIn("ERROR", result.stderr)
            self.assertFalse(output.exists())

    @requires_g1_mjcf
    def test_real_recording_writes_schema_shaped_report(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            recording = write_fixture_npz(
                tmp_path / "recording.npz", frames=6, fps=60.0, joint_amplitude=0.0
            )
            output = tmp_path / "collision-audit.json"
            result = run_cli(
                self.SCRIPT,
                str(recording),
                "--output",
                str(output),
                "--model",
                str(MODEL),
                "--window-seconds",
                "0.05",
            )
            self.assertIn(result.returncode, (0, 1), result.stderr)
            self.assertTrue(output.is_file())
            payload = json.loads(output.read_text())
            self.assertIn(payload["summary"]["status"], ("PASS", "FAIL"))
            self.assertIn("self_collision_frames", payload["summary"])
            self.assertIn("frames", payload["summary"])


class QualifyG1UpperBodySmokeTest(unittest.TestCase):
    SCRIPT = REPO_ROOT / "tools/control/qualify_g1_upper_body.py"

    def test_help_exits_zero(self):
        result = run_cli(self.SCRIPT, "--help")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("usage", result.stdout.lower())

    def test_missing_model_argument_fails_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            recording = write_fixture_npz(tmp_path / "recording.npz", frames=31)
            output = tmp_path / "controller-screen.json"
            result = run_cli(
                self.SCRIPT,
                str(recording),
                "--output",
                str(output),
                env=env_without_mocap_ws(),
            )
            self.assertEqual(result.returncode, 2)
            self.assertFalse(output.exists())

    def test_missing_input_fails_closed_without_writing_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            output = tmp_path / "controller-screen.json"
            result = run_cli(
                self.SCRIPT,
                str(tmp_path / "does-not-exist.npz"),
                "--output",
                str(output),
                "--model",
                str(MODEL),
            )
            self.assertEqual(result.returncode, 2)
            self.assertIn("ERROR", result.stderr)
            self.assertFalse(output.exists())

    @requires_g1_mjcf
    def test_real_recording_writes_schema_shaped_report(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            recording = write_fixture_npz(
                tmp_path / "recording.npz", frames=31, fps=60.0, joint_amplitude=0.05
            )
            output = tmp_path / "controller-screen.json"
            result = run_cli(
                self.SCRIPT,
                str(recording),
                "--model",
                str(MODEL),
                "--output",
                str(output),
                "--settle-seconds",
                "0.02",
                "--tracking-p95-limit-rad",
                "1.0",
                "--tracking-max-limit-rad",
                "2.0",
                "--saturation-fraction-limit",
                "1.0",
            )
            self.assertIn(result.returncode, (0, 1), result.stderr)
            self.assertTrue(output.is_file())
            payload = json.loads(output.read_text())
            self.assertIn(payload["controller_screen"]["status"], ("PASS", "FAIL"))
            self.assertIn("hardware_candidate", payload)
            self.assertIn("pooled_p95_abs_rad", payload["tracking"])
            self.assertIn("source_selection", payload)


class ReplayG1FaultsSmokeTest(unittest.TestCase):
    SCRIPT = REPO_ROOT / "tools/control/replay_g1_faults.py"

    def test_help_exits_zero(self):
        result = run_cli(self.SCRIPT, "--help")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("usage", result.stdout.lower())

    def test_missing_input_fails_closed_without_writing_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            output = tmp_path / "fault-replay.json"
            result = run_cli(
                self.SCRIPT,
                str(tmp_path / "does-not-exist.npz"),
                "--output",
                str(output),
            )
            self.assertEqual(result.returncode, 2)
            self.assertIn("ERROR", result.stderr)
            self.assertFalse(output.exists())

    def test_invalid_hard_timeout_fails_closed(self):
        # hard-timeout must exceed stale-timeout; this is a deterministic
        # configuration error unrelated to any concurrent gain tuning.
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            recording = write_fixture_npz(tmp_path / "recording.npz", frames=301, fps=60.0)
            output = tmp_path / "fault-replay.json"
            result = run_cli(
                self.SCRIPT,
                str(recording),
                "--output",
                str(output),
                "--stale-timeout",
                "0.10",
                "--hard-timeout",
                "0.05",
            )
            self.assertEqual(result.returncode, 2)
            self.assertIn("ERROR", result.stderr)
            self.assertFalse(output.exists())

    def test_real_recording_writes_schema_shaped_report(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            recording = write_fixture_npz(
                tmp_path / "recording.npz", frames=301, fps=60.0, joint_amplitude=0.05
            )
            output = tmp_path / "fault-replay.json"
            before = recording.read_bytes()
            result = run_cli(
                self.SCRIPT,
                str(recording),
                "--output",
                str(output),
                "--control-rate",
                "60.0",
            )
            self.assertIn(result.returncode, (0, 1), result.stderr)
            self.assertTrue(output.is_file())
            self.assertEqual(recording.read_bytes(), before)
            payload = json.loads(output.read_text())
            self.assertIn(payload["status"], ("PASS", "FAIL"))
            self.assertIn("transport", payload)
            self.assertIn("control", payload)
            self.assertIn("fault_response", payload)
            self.assertIn("safety_compliance", payload)
            self.assertIn("provenance", payload)

            # A second run against the same (untouched) output path must
            # refuse to overwrite an existing report.
            repeated = run_cli(
                self.SCRIPT,
                str(recording),
                "--output",
                str(output),
                "--control-rate",
                "60.0",
            )
            self.assertEqual(repeated.returncode, 2)
            self.assertIn("ERROR", repeated.stderr)


class LiveG1SimSmokeTest(unittest.TestCase):
    """live_g1_sim.py opens a MuJoCo viewer once a frame arrives, so these
    tests only cover --help and argument validation that fails closed before
    any receiver/viewer/GMR runtime is touched."""

    SCRIPT = REPO_ROOT / "tools/xsens/live_g1_sim.py"

    def test_help_exits_zero(self):
        result = run_cli(self.SCRIPT, "--help")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("usage", result.stdout.lower())

    def test_invalid_port_fails_closed(self):
        result = run_cli(self.SCRIPT, "--port", "70000")
        self.assertEqual(result.returncode, 2)
        self.assertIn("--port", result.stderr)

    def test_invalid_timeout_ordering_fails_closed(self):
        result = run_cli(
            self.SCRIPT,
            "--stale-timeout",
            "0.50",
            "--hard-timeout",
            "0.10",
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn("--hard-timeout", result.stderr)

    def test_missing_limits_urdf_fails_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            missing = Path(tmp) / "does-not-exist.urdf"
            result = run_cli(self.SCRIPT, "--limits-urdf", str(missing))
            self.assertEqual(result.returncode, 2)
            self.assertIn("--limits-urdf", result.stderr)

    def test_parser_defaults_are_pinned(self):
        # build_parser() was extracted from main(); every default is pinned so
        # the extraction (and any later one) cannot silently change the CLI.
        from tools.xsens.live_g1_sim import DEFAULT_LIMITS_URDF, build_parser

        self.assertEqual(
            vars(build_parser().parse_args([])),
            {
                "acceleration_limit": 4.0,
                "arm_abduction": None,
                "arm_alignment": None,
                "bind": "0.0.0.0",
                "blend_rate": 1.0,
                "calibrate_after": None,
                "command_interpolation": "interpolate",
                "command_rate": 250.0,
                "control_rate": 60.0,
                "duration": 0.0,
                "filter": True,
                "filter_beta": 0.6,
                "filter_cutoff": 4.0,
                "filter_derivative_cutoff": 1.0,
                "filter_min_cutoff": 2.0,
                "filter_type": "one_euro",
                "g1_hardware_envelope": False,
                "hard_timeout": 0.5,
                "human_height": None,
                "limits_urdf": DEFAULT_LIMITS_URDF,
                "port": 5555,
                "position_margin": 0.05,
                "rate_limit": True,
                "record_video": False,
                "robot": "unitree_g1",
                "source_ip": None,
                "stale_timeout": 0.1,
                "start_disabled": False,
                "startup_timeout": 60.0,
                "velocity_limit_fraction": 0.25,
                # Resolved per robot in main(): artifacts/xsens-live/<robot>-live.mp4.
                "video_path": None,
            },
        )
        self.assertEqual(
            DEFAULT_LIMITS_URDF.resolve(),
            (REPO_ROOT / "eval/assets/g1_29dof_official.urdf").resolve(),
        )


if __name__ == "__main__":
    unittest.main()
