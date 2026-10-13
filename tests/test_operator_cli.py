"""Process-level contracts for the operator-console launcher."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "tools" / "operator_console.py"
PYTHON = sys.executable


class OperatorConsoleCliTest(unittest.TestCase):
    def test_help_does_not_require_optional_web_dependencies(self):
        result = subprocess.run(
            [PYTHON, "-S", str(SCRIPT), "--help"],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            check=False,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--host", result.stdout)
        self.assertIn("--port", result.stdout)
        self.assertIn("--data-root", result.stdout)
        self.assertIn("--demo", result.stdout)
        self.assertIn("--playback", result.stdout)
        self.assertIn("--simulation", result.stdout)
        self.assertIn("--qualification-root", result.stdout)
        self.assertIn("--open", result.stdout)

    def test_missing_optional_dependencies_has_install_hint_and_no_state_write(self):
        with tempfile.TemporaryDirectory() as tmp:
            env = dict(os.environ)
            state_root = Path(tmp) / "state-home"
            env["XDG_STATE_HOME"] = str(state_root)
            self.assertFalse(state_root.exists())

            result = subprocess.run(
                [PYTHON, "-S", str(SCRIPT)],
                cwd=REPO_ROOT,
                env=env,
                capture_output=True,
                text=True,
                check=False,
            )

            self.assertEqual(result.returncode, 2)
            self.assertIn("mocap-g1[operator]", result.stderr)
            self.assertFalse(state_root.exists())

    def test_defaults_are_local_and_robot_control_is_not_an_option(self):
        from mocap_g1.operator.cli import build_parser

        args = build_parser().parse_args([])
        self.assertEqual(args.host, "127.0.0.1")
        self.assertEqual(args.port, 8766)
        self.assertFalse(args.demo)
        self.assertIsNone(args.playback)
        self.assertIsNone(args.simulation)
        self.assertIsNone(args.qualification_root)
        self.assertFalse(args.open_browser)
        option_names = {
            option
            for action in build_parser()._actions
            for option in action.option_strings
        }
        self.assertFalse({"--robot", "--arm", "--control"} & option_names)

    def test_demo_and_playback_are_mutually_exclusive(self):
        from mocap_g1.operator.cli import build_parser

        with self.assertRaises(SystemExit):
            build_parser().parse_args(["--demo", "--playback", "review.mp4"])

    def test_missing_or_non_mp4_playback_fails_before_runtime_load(self):
        from mocap_g1.operator import cli as operator_console

        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(
            operator_console, "_load_runtime"
        ) as load_runtime, mock.patch("sys.stderr") as stderr:
            missing = operator_console.main(
                ["--data-root", tmp, "--playback", str(Path(tmp) / "missing.mp4")]
            )
            wrong = Path(tmp) / "review.txt"
            wrong.write_bytes(b"not a video")
            wrong_type = operator_console.main(
                ["--data-root", tmp, "--playback", str(wrong)]
            )
            directory = Path(tmp) / "review.mp4"
            directory.mkdir()
            not_regular = operator_console.main(
                ["--data-root", tmp, "--playback", str(directory)]
            )
            corrupt = Path(tmp) / "corrupt.mp4"
            corrupt.write_bytes(b"not-an-mp4")
            corrupt_file = operator_console.main(
                ["--data-root", tmp, "--playback", str(corrupt)]
            )
            empty = Path(tmp) / "empty.mp4"
            empty.write_bytes(b"")
            empty_file = operator_console.main(
                ["--data-root", tmp, "--playback", str(empty)]
            )

        self.assertEqual(missing, 2)
        self.assertEqual(wrong_type, 2)
        self.assertEqual(not_regular, 2)
        self.assertEqual(corrupt_file, 2)
        self.assertEqual(empty_file, 2)
        load_runtime.assert_not_called()
        output = "".join(call.args[0] for call in stderr.write.call_args_list)
        self.assertIn("playback", output)
        self.assertIn(".mp4", output)

    def test_playback_is_resolved_and_passed_to_app_factory(self):
        from mocap_g1.operator import cli as operator_console

        fake_uvicorn = mock.Mock()
        fake_factory = mock.Mock(return_value=object())
        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(
            operator_console,
            "_load_runtime",
            return_value=(fake_uvicorn, fake_factory),
        ):
            video = Path(tmp) / "recorded review.mp4"
            video.write_bytes(b"\x00\x00\x00\x18ftypmp42recorded-review")
            playback_data_root = Path(tmp) / "production-state-must-not-exist"
            result = operator_console.main(
                ["--data-root", str(playback_data_root), "--playback", str(video)]
            )

        self.assertEqual(result, 0)
        self.assertFalse(playback_data_root.exists())
        self.assertIsNone(fake_factory.call_args.kwargs["data_root"])
        self.assertEqual(fake_factory.call_args.kwargs["playback"], video.resolve())
        self.assertFalse(fake_factory.call_args.kwargs["demo"])

    def test_browser_is_opened_only_with_explicit_flag(self):
        from mocap_g1.operator import cli as operator_console

        fake_uvicorn = mock.Mock()
        fake_factory = mock.Mock(return_value=object())
        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(
            operator_console,
            "_load_runtime",
            return_value=(fake_uvicorn, fake_factory),
        ), mock.patch.object(operator_console, "_schedule_browser") as open_browser:
            result = operator_console.main(["--data-root", tmp])
            self.assertEqual(result, 0)
            open_browser.assert_not_called()

            result = operator_console.main(["--data-root", tmp, "--open"])
            self.assertEqual(result, 0)
            open_browser.assert_called_once_with("http://127.0.0.1:8766/")

        self.assertEqual(fake_uvicorn.run.call_count, 2)

    def test_factory_missing_fastapi_reports_install_hint(self):
        from mocap_g1.operator import cli as operator_console

        def missing_fastapi(**_kwargs):
            try:
                raise ImportError("No module named 'fastapi'")
            except ImportError as exc:
                raise RuntimeError("FastAPI is required") from exc

        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(
            operator_console,
            "_load_runtime",
            return_value=(mock.Mock(), missing_fastapi),
        ), mock.patch("sys.stderr") as stderr:
            result = operator_console.main(["--data-root", tmp])

        self.assertEqual(result, 2)
        output = "".join(call.args[0] for call in stderr.write.call_args_list)
        self.assertIn("mocap-g1[operator]", output)

    def test_checkout_shim_delegates_to_packaged_entry_point(self):
        from mocap_g1.operator import cli
        from tools import operator_console as shim

        self.assertIs(shim.main, cli.main)

    def test_identity_proxy_options_reach_app_factory_from_flags_or_env(self):
        from mocap_g1.operator import cli as operator_console

        fake_factory = mock.Mock(return_value=object())
        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(
            operator_console,
            "_load_runtime",
            return_value=(mock.Mock(), fake_factory),
        ):
            with mock.patch.dict(os.environ, {}, clear=False):
                os.environ.pop("MOCAP_G1_IDENTITY_HEADER", None)
                os.environ.pop("MOCAP_G1_TRUSTED_PROXIES", None)
                self.assertEqual(operator_console.main(["--data-root", tmp]), 0)
                kwargs = fake_factory.call_args.kwargs
                self.assertIsNone(kwargs["identity_header"])
                self.assertEqual(kwargs["trusted_proxies"], ())

            with mock.patch.dict(
                os.environ,
                {
                    "MOCAP_G1_IDENTITY_HEADER": "X-Env-User",
                    "MOCAP_G1_TRUSTED_PROXIES": "127.0.0.1, ::1",
                },
            ):
                self.assertEqual(operator_console.main(["--data-root", tmp]), 0)
                kwargs = fake_factory.call_args.kwargs
                self.assertEqual(kwargs["identity_header"], "X-Env-User")
                self.assertEqual(kwargs["trusted_proxies"], ("127.0.0.1", "::1"))

                self.assertEqual(
                    operator_console.main(
                        [
                            "--data-root",
                            tmp,
                            "--identity-header",
                            "X-Flag-User",
                            "--trusted-proxy",
                            "10.0.0.5",
                        ]
                    ),
                    0,
                )
                kwargs = fake_factory.call_args.kwargs
                self.assertEqual(kwargs["identity_header"], "X-Flag-User")
                self.assertEqual(kwargs["trusted_proxies"], ("10.0.0.5",))

    def test_project_exposes_operator_extra_and_console_entry_point(self):
        project = (REPO_ROOT / "pyproject.toml").read_text()
        self.assertIn("operator = [", project)
        self.assertIn('"fastapi>=0.115,<1"', project)
        self.assertIn(
            'mocap-g1-console = "mocap_g1.operator.cli:main"',
            project,
        )
        self.assertIn('"mocap_g1.operator" = ["static/*", "static/**/*"]', project)


if __name__ == "__main__":
    unittest.main()
