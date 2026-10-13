"""Exercise installer control flow without installing packages or building images."""

from __future__ import annotations

import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "tools/setup/install.sh"


class InstallSetupTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name) / "checkout space"
        (self.root / "tools/setup").mkdir(parents=True)
        shutil.copy2(SCRIPT, self.root / "tools/setup/install.sh")
        self.script = self.root / "tools/setup/install.sh"

    def invoke(self, *args: str, env: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            ["/bin/bash", str(self.script), *args],
            cwd="/",
            env=env,
            text=True,
            capture_output=True,
            check=False,
        )

    def test_core_dry_run_from_any_directory_executes_nothing(self) -> None:
        # An empty PATH makes any accidental external command fail.
        result = self.invoke("core", "--dry-run", env={"PATH": "/nonexistent"})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("-m venv", result.stdout)
        self.assertIn("[runtime,operator]", result.stdout.replace("\\", ""))
        self.assertIn("--self-test --require-joint-angles", result.stdout)
        self.assertIn("DRY RUN: NOT VERIFIED", result.stdout)
        self.assertFalse((self.root / ".venv").exists())

    def test_recorded_dry_run_quotes_workspace_and_orders_checks(self) -> None:
        workspace = Path(self.temporary.name) / "external space" / "GMR runtime"
        result = self.invoke(
            "recorded", "--workspace", str(workspace), "--robot", "booster_t1_29dof",
            "--dry-run", env={"PATH": "/nonexistent"},
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        commands = [shlex.split(line[2:]) for line in result.stdout.splitlines() if line.startswith("+ ")]
        self.assertEqual([cmd[-1] for cmd in commands[-3:]],
                         [str(workspace), str(workspace), "booster_t1_29dof"])
        self.assertIn("bootstrap", commands[-3])
        self.assertIn("check", commands[-2])
        self.assertIn("recorded", commands[-1])
        self.assertEqual(commands[-1][commands[-1].index("--workspace") + 1], str(workspace))
        self.assertFalse(workspace.exists())

    def test_live_dry_run_covers_preflight_gpu_and_optional_ros(self) -> None:
        result = self.invoke("live", "--with-ros2", "--dry-run", env={"PATH": "/nonexistent"})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("docker info", result.stdout)
        self.assertIn("nvidia-smi -L", result.stdout)
        self.assertIn("docker build -t gmr-live:bb1bbe4", result.stdout)
        self.assertIn("docker run --rm --gpus all", result.stdout)
        self.assertIn("docker build -t xsens-ros2:jazzy", result.stdout)
        self.assertLess(result.stdout.index("docker info"), result.stdout.index("docker build"))
        self.assertFalse((self.root / ".venv").exists())

    def test_invalid_options_and_workspace_fail_before_writes(self) -> None:
        invalid = [
            (),  # no profile and no terminal: refuse instead of asking
            ("--dry-run",),
            ("recorded", "--workspace", str(self.root / "runtime"), "--dry-run"),
            ("recorded", "--workspace", "/tmp", "--dry-run"),
            ("recorded", "--workspace", "relative/path", "--dry-run"),
            ("core", "--workspace", "/tmp/external/runtime", "--dry-run"),
            ("core", "--robot", "booster_t1_29dof", "--dry-run"),
            ("recorded", "--workspace", "/tmp/external/runtime", "--robot", "bad", "--dry-run"),
            ("recorded", "--workspace", "/tmp/external/runtime", "--with-ros2", "--dry-run"),
            ("live", "--unexpected", "--dry-run"),
        ]
        for args in invalid:
            with self.subTest(args=args):
                result = self.invoke(*args, env={"PATH": "/nonexistent"})
                self.assertNotEqual(result.returncode, 0)
                self.assertNotIn("-m pip install", result.stdout)
        alias = Path(self.temporary.name) / "outside-alias"
        alias.symlink_to(self.root, target_is_directory=True)
        result = self.invoke("recorded", "--workspace", str(alias / "runtime"), "--dry-run",
                             env={"PATH": "/nonexistent"})
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("contains a symlink", result.stderr)
        self.assertFalse((self.root / ".venv").exists())

    def test_recorded_workspace_defaults_to_home_runtime(self) -> None:
        home = Path(self.temporary.name) / "home dir"
        home.mkdir()
        result = self.invoke("recorded", "--dry-run", env={"PATH": "/nonexistent", "HOME": str(home)})
        self.assertEqual(result.returncode, 0, result.stderr)
        commands = [shlex.split(line[2:]) for line in result.stdout.splitlines() if line.startswith("+ ")]
        self.assertEqual(commands[-1][commands[-1].index("--workspace") + 1], str(home / "mocap-runtime"))
        self.assertEqual(commands[-1][-1], "unitree_g1")
        # Without HOME there is no default to fall back on.
        result = self.invoke("recorded", "--dry-run", env={"PATH": "/nonexistent"})
        self.assertNotEqual(result.returncode, 0)

    def _fake_bin(self, name: str, content: str) -> Path:
        bin_dir = Path(self.temporary.name) / "fake-bin"
        bin_dir.mkdir(exist_ok=True)
        target = bin_dir / name
        target.write_text("#!/bin/sh\n" + content, encoding="utf-8")
        target.chmod(0o755)
        return bin_dir

    def test_failed_pip_stops_before_doctor(self) -> None:
        log = Path(self.temporary.name) / "commands.log"
        (self.root / ".venv/bin").mkdir(parents=True)
        venv_python = self.root / ".venv/bin/python"
        venv_python.write_text(
            '#!/bin/sh\ncase "$1" in -c) exit 0;; esac\n'
            'printf "%s\\n" "$*" >> "$INSTALL_LOG"\nexit 37\n', encoding="utf-8"
        )
        venv_python.chmod(0o755)
        bin_dir = self._fake_bin("git", "echo abc123\n")
        env = dict(os.environ, INSTALL_LOG=str(log), PATH=f"{bin_dir}:{os.environ['PATH']}")
        result = self.invoke("core", "--python", sys.executable, env=env)
        self.assertEqual(result.returncode, 37, result.stderr)
        self.assertIn("FAIL: project environment stage stopped", result.stderr)
        self.assertEqual(log.read_text().splitlines(),
                         [f"-m pip install -e {self.root}[runtime,operator]"])

    def test_incompatible_existing_venv_is_preserved(self) -> None:
        (self.root / ".venv/bin").mkdir(parents=True)
        venv_python = self.root / ".venv/bin/python"
        venv_python.write_text("#!/bin/sh\nexit 1\n", encoding="utf-8")
        venv_python.chmod(0o755)
        bin_dir = self._fake_bin("git", "echo abc123\n")
        env = dict(os.environ, PATH=f"{bin_dir}:{os.environ['PATH']}")
        result = self.invoke("core", "--python", sys.executable, env=env)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("move .venv aside", result.stderr)
        self.assertNotIn("-m pip install", result.stdout)
        self.assertTrue(venv_python.exists())

    def test_live_daemon_failure_prevents_build(self) -> None:
        (self.root / ".venv/bin").mkdir(parents=True)
        venv_python = self.root / ".venv/bin/python"
        venv_python.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
        venv_python.chmod(0o755)
        bin_dir = self._fake_bin("git", "echo abc123\n")
        self._fake_bin("docker", 'echo "$*" >> "$INSTALL_LOG"\nexit 1\n')
        self._fake_bin("nvidia-smi", "exit 0\n")
        log = Path(self.temporary.name) / "docker.log"
        env = dict(os.environ, INSTALL_LOG=str(log), PATH=f"{bin_dir}:{os.environ['PATH']}")
        result = self.invoke("live", "--python", sys.executable, env=env)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Docker daemon is unavailable", result.stderr)
        self.assertEqual(log.read_text().splitlines(), ["info --format {{json .Runtimes}}"])


if __name__ == "__main__":
    unittest.main()
