"""T2-5: the console is never exposed beyond loopback without an identity proxy."""

from __future__ import annotations

import contextlib
import io
import os
import tempfile
import unittest
from unittest import mock

from mocap_g1.operator import cli as operator_console
from mocap_g1.operator.app import (
    NetworkExposureError,
    require_identity_proxy_for_network_access,
)

_PROXY_ENV = ("MOCAP_G1_IDENTITY_HEADER", "MOCAP_G1_TRUSTED_PROXIES")


class NetworkExposurePolicyTests(unittest.TestCase):
    def check(self, *, bind_host="127.0.0.1", allowed_hosts=(), header=None, proxies=()):
        require_identity_proxy_for_network_access(
            bind_host=bind_host,
            port=8766,
            allowed_hosts=allowed_hosts,
            identity_header=header,
            trusted_proxies=proxies,
        )

    def test_loopback_binds_and_hosts_need_no_proxy(self):
        for bind in ("127.0.0.1", "::1", "[::1]", "localhost", "LOCALHOST", "127.0.0.2"):
            with self.subTest(bind=bind):
                self.check(bind_host=bind)
        self.check(
            allowed_hosts=(
                "127.0.0.1:5173",
                "http://localhost:5173",
                "[::1]:9000",
                "http://[::ffff:127.0.0.1]:8766",
            )
        )

    def test_non_loopback_exposure_without_proxy_is_refused(self):
        cases = (
            {"bind_host": "0.0.0.0"},
            {"bind_host": "::"},
            {"bind_host": "192.168.50.20"},
            {"bind_host": "lab-ws"},
            {"allowed_hosts": ("https://console.example",)},
            {"allowed_hosts": ("lab-ws",)},
            {"allowed_hosts": ("127.0.0.1.evil.example",)},
            # One half of the proxy configuration is not enough.
            {"bind_host": "0.0.0.0", "header": "X-Authenticated-User"},
            {"bind_host": "0.0.0.0", "proxies": ("127.0.0.1",)},
        )
        for kwargs in cases:
            with self.subTest(**{k: str(v) for k, v in kwargs.items()}):
                with self.assertRaises(NetworkExposureError) as raised:
                    self.check(**kwargs)
                self.assertIn("--identity-header", str(raised.exception))
                self.assertIsInstance(raised.exception, ValueError)

    def test_non_loopback_exposure_with_identity_proxy_is_accepted(self):
        self.check(
            bind_host="0.0.0.0",
            allowed_hosts=("https://console.example",),
            header="X-Authenticated-User",
            proxies=("10.0.0.5",),
        )

    def test_create_app_enforces_the_policy(self):
        from mocap_g1.operator.app import create_app

        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(NetworkExposureError):
                create_app(data_root=directory, demo=True, bind_host="0.0.0.0")
            with self.assertRaises(NetworkExposureError):
                create_app(
                    data_root=directory,
                    demo=True,
                    allowed_hosts=("https://console.example",),
                )


class CliNetworkExposureTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        environ = {k: v for k, v in os.environ.items() if k not in _PROXY_ENV}
        patcher = mock.patch.dict(os.environ, environ, clear=True)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.uvicorn = mock.Mock()
        self.factory = mock.Mock(return_value=object())
        patcher = mock.patch.object(
            operator_console,
            "_load_runtime",
            return_value=(self.uvicorn, self.factory),
        )
        patcher.start()
        self.addCleanup(patcher.stop)

    def run_cli(self, *argv: str) -> tuple[int, str]:
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            result = operator_console.main(["--data-root", self.temporary.name, *argv])
        return result, stderr.getvalue()

    def test_cli_refuses_lan_bind_without_identity_proxy(self):
        for argv in (
            ("--host", "0.0.0.0"),
            ("--host", "192.168.50.20"),
            ("--allowed-host", "https://console.example"),
            ("--host", "0.0.0.0", "--identity-header", "X-Authenticated-User"),
        ):
            with self.subTest(argv=argv):
                result, stderr = self.run_cli(*argv)
                self.assertEqual(result, 2)
                self.assertIn("Refusing to expose the operator console", stderr)
        self.factory.assert_not_called()
        self.uvicorn.run.assert_not_called()

    def test_cli_accepts_lan_with_identity_header_and_trusted_proxy(self):
        result, stderr = self.run_cli(
            "--host", "192.168.50.20",
            "--allowed-host", "https://console.example",
            "--identity-header", "X-Authenticated-User",
            "--trusted-proxy", "127.0.0.1",
        )
        self.assertEqual(result, 0, stderr)
        self.uvicorn.run.assert_called_once()
        self.assertEqual(self.uvicorn.run.call_args.kwargs["host"], "192.168.50.20")

    def test_cli_accepts_identity_proxy_from_environment(self):
        with mock.patch.dict(
            os.environ,
            {
                "MOCAP_G1_IDENTITY_HEADER": "X-Authenticated-User",
                "MOCAP_G1_TRUSTED_PROXIES": "127.0.0.1",
            },
        ):
            result, stderr = self.run_cli("--allowed-host", "https://console.example")
        self.assertEqual(result, 0, stderr)

    def test_cli_accepts_loopback_without_proxy(self):
        for argv in (
            (),
            ("--host", "::1"),
            ("--host", "localhost"),
            ("--allowed-host", "127.0.0.1:5173"),
        ):
            with self.subTest(argv=argv):
                result, stderr = self.run_cli(*argv)
                self.assertEqual(result, 0, stderr)


if __name__ == "__main__":
    unittest.main()
