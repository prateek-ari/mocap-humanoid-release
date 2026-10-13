"""Read-only setup doctor checks, with host probes replaced by fakes."""

from __future__ import annotations

from contextlib import redirect_stdout
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock


SCRIPT = Path(__file__).resolve().parents[1] / "tools/checks/setup_doctor.py"
spec = importlib.util.spec_from_file_location("setup_doctor", SCRIPT)
assert spec and spec.loader
doctor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(doctor)


def by_name(report: dict) -> dict[str, dict[str, str]]:
    return {item["name"]: item for item in report["checks"]}


class SetupDoctorTests(unittest.TestCase):
    def test_core_checks_numpy_version_and_does_not_probe_host(self):
        with mock.patch.object(doctor, "_module", return_value=type("Numpy", (), {"__version__": "2.3.0"})()), \
             mock.patch.object(doctor, "_command") as command:
            report = doctor.diagnose("core", env={})
        self.assertFalse(report["ok"])
        self.assertEqual(by_name(report)["numpy"]["status"], "block")
        command.assert_not_called()

    def test_recorded_workspace_has_concrete_paths_and_verifier(self):
        with tempfile.TemporaryDirectory() as temp:
            workspace = Path(temp)
            gmr = workspace / "GMR"
            (gmr / ".git").mkdir(parents=True)
            (gmr / "assets/unitree_g1").mkdir(parents=True)
            (gmr / "assets/booster_t1_29dof").mkdir(parents=True)
            (gmr / "assets/unitree_g1/g1_mocap_29dof.xml").touch()
            (gmr / "assets/booster_t1_29dof/t1_mocap.xml").touch()
            (workspace / "venv/bin").mkdir(parents=True)
            (workspace / "venv/bin/python").touch()
            (workspace / "runtime-lock.txt").touch()
            def command(args: list[str]) -> tuple[int, str]:
                return ((0, doctor.PINNED_GMR) if args[0] == "git" else
                        (0, '{"numpy":"2.2.6","scipy":"1.15.3","mujoco":"3.11.0"}'))

            with mock.patch.object(doctor, "_python_checks", return_value=[]), \
                 mock.patch.object(doctor, "_optional_modules", return_value=[]), \
                 mock.patch.object(doctor, "_command", side_effect=command):
                report = doctor.diagnose("recorded", workspace=workspace, env={})
            self.assertTrue(report["ok"])
            self.assertEqual(by_name(report)["venv_imports"]["status"], "pass")
            self.assertEqual(by_name(report)["booster_model"]["status"], "pass")
            self.assertIn("bootstrap_g1_workspace.py check", report["full_verification"])
            (gmr / "assets/booster_t1_29dof/t1_mocap.xml").unlink()
            with mock.patch.object(doctor, "_python_checks", return_value=[]), \
                 mock.patch.object(doctor, "_command", side_effect=command):
                warning_report = doctor.diagnose("recorded", workspace=workspace, env={})
            self.assertTrue(warning_report["ok"])
            self.assertEqual(by_name(warning_report)["booster_model"]["status"], "warn")
            with mock.patch.object(doctor, "_python_checks", return_value=[]), \
                 mock.patch.object(doctor, "_command", side_effect=command):
                booster_report = doctor.diagnose("recorded", workspace=workspace,
                                                robot="booster_t1_29dof", env={})
            self.assertFalse(booster_report["ok"])
            self.assertEqual(by_name(booster_report)["booster_model"]["status"], "block")

    def test_recorded_missing_workspace_is_blocker(self):
        with mock.patch.object(doctor, "_python_checks", return_value=[]), \
             mock.patch.object(doctor, "_optional_modules", return_value=[]):
            report = doctor.diagnose("recorded", env={})
        self.assertFalse(report["ok"])
        self.assertIn("--workspace", by_name(report)["workspace"]["fix"])

    def test_live_reports_occupied_udp_and_missing_source(self):
        def which(name: str) -> str | None:
            return {"ss": "/usr/bin/ss", "docker": "/usr/bin/docker",
                    "nvidia-smi": "/usr/bin/nvidia-smi"}.get(name)

        def command(args: list[str]) -> tuple[int, str]:
            if args[0] == "ss":
                return 0, "udp UNCONN 0 0 0.0.0.0:5555 0.0.0.0:*"
            if args[1] == "info":
                return 0, '{"nvidia":{}}'
            return 0, "GPU 0"

        with tempfile.NamedTemporaryFile() as auth, \
             mock.patch.object(doctor, "_python_checks", return_value=[]), \
             mock.patch.object(doctor.shutil, "which", side_effect=which), \
             mock.patch.object(doctor, "_command", side_effect=command), \
             mock.patch.object(Path, "is_socket", return_value=True):
            report = doctor.diagnose("live", env={"XAUTHORITY": auth.name})
        checks = by_name(report)
        self.assertFalse(report["ok"])
        self.assertEqual(checks["udp_port"]["status"], "block")
        self.assertEqual(checks["source_ipv4"]["status"], "block")
        self.assertEqual(checks["nvidia_runtime"]["status"], "pass")

    def test_udp_probe_failure_is_explicit_unknown(self):
        with mock.patch.object(doctor.shutil, "which", return_value="/usr/bin/ss"), \
             mock.patch.object(doctor, "_command", return_value=(None, "permission denied")):
            check = doctor._udp_check(5555)
        self.assertEqual(check["status"], "unknown")
        self.assertIn("permission denied", check["detail"])

    def test_json_exit_status_tracks_blockers(self):
        report = {"profile": "core", "ok": False,
                  "checks": [doctor._check("numpy", "block", "missing", "install") ]}
        out = io.StringIO()
        with mock.patch.object(doctor, "diagnose", return_value=report), redirect_stdout(out):
            status = doctor.main(["core", "--json"])
        self.assertEqual(status, 1)
        self.assertEqual(json.loads(out.getvalue()), report)


if __name__ == "__main__":
    unittest.main()
