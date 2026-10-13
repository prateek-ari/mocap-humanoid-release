"""Regression tests for watchdog, startup, attribution, and restart handling."""

from __future__ import annotations

import asyncio
from importlib.util import find_spec
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest import mock

from mocap_g1.operator import SessionController
from mocap_g1.operator.runtime import PassiveRuntime


TESTSERVER_HOSTS = ("http://testserver",)
# "testserver" is not a loopback name, so the app requires an identity proxy
# to trust it.  The TestClient peer ("testclient") is never inside that proxy
# network, so audit attribution stays "local-console".
TESTSERVER_APP = {
    "allowed_hosts": TESTSERVER_HOSTS,
    "identity_header": "X-Test-Identity",
    "trusted_proxies": ("192.0.2.1",),
}
_API_TESTS_AVAILABLE = find_spec("fastapi") is not None and find_spec("httpx") is not None
DEMO_TO_TRACKING = (
    "check",
    "mocap-accept",
    "start-simulation",
    "connect-robot",
    "confirm-preflight",
    "arm",
    "start-tracking",
)


def wait_for(predicate, timeout: float = 2.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return predicate()


class UndeclaredSimulatedRuntime:
    """A simulated adapter that never declares ``allows_hardware_bypass``."""

    is_simulated = True


class TruthyStringBypassRuntime(UndeclaredSimulatedRuntime):
    allows_hardware_bypass = "yes"


class HardwareBypassDefaultTests(unittest.TestCase):
    def test_simulated_runtime_without_declaration_does_not_bypass_capability(self):
        controller = SessionController(runtime=UndeclaredSimulatedRuntime())

        self.assertFalse(controller._demo)
        controls = controller.status_snapshot()["controls"]
        for action in ("connect-robot", "arm", "start-tracking"):
            with self.subTest(action=action):
                self.assertFalse(controls[action]["enabled"])
                self.assertEqual(
                    controls[action]["reason"],
                    controller.capability_lock.reason,
                )

    def test_truthy_non_boolean_declaration_does_not_bypass_capability(self):
        controller = SessionController(runtime=TruthyStringBypassRuntime())
        self.assertFalse(controller._demo)

    def test_shipped_runtimes_declare_hardware_bypass_explicitly(self):
        from mocap_g1.operator.runtime import DemoRuntime
        from mocap_g1.operator.simulation import RecordedSimulationRuntime

        for runtime, expected in (
            (PassiveRuntime, False),
            (DemoRuntime, True),
            (RecordedSimulationRuntime, False),
        ):
            with self.subTest(runtime=runtime.__name__):
                self.assertIn("allows_hardware_bypass", vars(runtime))
                self.assertIs(runtime.allows_hardware_bypass, expected)


class CapabilityKeyCredentialTests(unittest.TestCase):
    def test_systemd_credential_takes_precedence_over_environment(self):
        from mocap_g1.operator.app import CAPABILITY_KEY_CREDENTIAL, load_capability_key

        with tempfile.TemporaryDirectory() as credentials:
            (Path(credentials) / CAPABILITY_KEY_CREDENTIAL).write_bytes(b"from-credential\n")
            key = load_capability_key(
                {
                    "CREDENTIALS_DIRECTORY": credentials,
                    "MOCAP_G1_CAPABILITY_KEY": "from-environment",
                }
            )
        self.assertEqual(key, b"from-credential")

    def test_environment_is_used_when_no_credential_is_loaded(self):
        from mocap_g1.operator.app import load_capability_key

        with tempfile.TemporaryDirectory() as credentials:
            self.assertEqual(
                load_capability_key(
                    {
                        "CREDENTIALS_DIRECTORY": credentials,
                        "MOCAP_G1_CAPABILITY_KEY": "from-environment",
                    }
                ),
                "from-environment",
            )
        self.assertEqual(
            load_capability_key({"MOCAP_G1_CAPABILITY_KEY": "from-environment"}),
            "from-environment",
        )
        self.assertIsNone(load_capability_key({}))

    def test_empty_credential_fails_closed_without_environment_fallback(self):
        from mocap_g1.operator.app import CAPABILITY_KEY_CREDENTIAL, load_capability_key

        with tempfile.TemporaryDirectory() as credentials:
            (Path(credentials) / CAPABILITY_KEY_CREDENTIAL).write_bytes(b"\n")
            self.assertIsNone(
                load_capability_key(
                    {
                        "CREDENTIALS_DIRECTORY": credentials,
                        "MOCAP_G1_CAPABILITY_KEY": "from-environment",
                    }
                )
            )


@unittest.skipUnless(_API_TESTS_AVAILABLE, "FastAPI/httpx test dependencies are not installed")
class WatchdogResilienceTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)

    def app(self, **kwargs):
        from mocap_g1.operator.app import create_app

        kwargs.setdefault("data_root", self.temporary.name)
        kwargs.setdefault("demo", True)
        return create_app(**TESTSERVER_APP, **kwargs)

    def test_failing_watchdog_tick_is_logged_and_the_loop_keeps_running(self):
        from fastapi.testclient import TestClient

        app = self.app()
        with mock.patch.object(
            app.state.controller,
            "watchdog_tick",
            side_effect=RuntimeError("tick exploded"),
        ) as watchdog_tick, self.assertLogs(
            "mocap_g1.operator.app", level="ERROR"
        ) as logs:
            with TestClient(app) as client:
                self.assertTrue(wait_for(lambda: watchdog_tick.call_count >= 3))
                self.assertFalse(app.state.watchdog_task.done())
                # An idle session is not faulted merely because a tick failed.
                self.assertEqual(client.get("/api/v1/status").json()["state"], "idle")
        self.assertTrue(any("watchdog tick failed" in line for line in logs.output))

    def test_failing_watchdog_tick_withdraws_active_authority(self):
        from fastapi.testclient import TestClient

        app = self.app()
        with TestClient(app) as client:
            for action in DEMO_TO_TRACKING:
                response = client.post(f"/api/v1/actions/{action}")
                self.assertEqual(response.status_code, 200, response.text)
            self.assertEqual(response.json()["state"], "tracking")
            with mock.patch.object(
                app.state.controller,
                "watchdog_tick",
                side_effect=RuntimeError("tick exploded"),
            ), self.assertLogs("mocap_g1.operator.app", level="ERROR"):
                self.assertTrue(
                    wait_for(lambda: app.state.controller.state.value == "fault_latched")
                )
            status = client.get("/api/v1/status").json()
        self.assertFalse(status["health"]["command_authority"])
        self.assertEqual(status["active_fault"]["code"], "watchdog_tick_failed")
        self.assertTrue(status["active_fault"]["recoverable"])

    def test_unexpected_watchdog_stop_latches_a_nonrecoverable_fault(self):
        from fastapi.testclient import TestClient

        app = self.app()
        with self.assertLogs("mocap_g1.operator.app", level="CRITICAL"):
            with TestClient(app) as client:
                task = app.state.watchdog_task
                task.get_loop().call_soon_threadsafe(task.cancel)
                self.assertTrue(wait_for(task.done))
                self.assertTrue(
                    wait_for(lambda: app.state.controller.state.value == "fault_latched")
                )
                status = client.get("/api/v1/status").json()
        self.assertEqual(status["active_fault"]["code"], "watchdog_stopped")
        self.assertFalse(status["active_fault"]["recoverable"])
        self.assertFalse(status["controls"]["check"]["enabled"])
        self.assertTrue(status["controls"]["disengage"]["enabled"])

    def test_normal_shutdown_does_not_latch_a_watchdog_fault(self):
        from fastapi.testclient import TestClient

        app = self.app()
        with TestClient(app):
            pass
        self.assertEqual(app.state.controller.state.value, "idle")


@unittest.skipUnless(_API_TESTS_AVAILABLE, "FastAPI/httpx test dependencies are not installed")
class InvalidPersistedConfigTests(unittest.TestCase):
    def valid_config(self) -> dict:
        return {
            "schema_version": 1,
            "topology": "all_in_one",
            "runtime_mode": "real",
            "mocap": {"listen_address": "0.0.0.0", "port": 5555, "source_ip": None},
            "robot": {"interface": None},
            "privacy": {"record_raw_mocap": False},
        }

    def test_parseable_but_invalid_config_starts_in_setup_required(self):
        from fastapi.testclient import TestClient

        from mocap_g1.operator.app import create_app

        port_zero = self.valid_config()
        port_zero["mocap"]["port"] = 0
        unknown_key = self.valid_config()
        unknown_key["surprise"] = True
        null_mocap = self.valid_config()
        null_mocap["mocap"] = None
        for label, payload in (
            ("port_zero", port_zero),
            ("unknown_key", unknown_key),
            ("null_mocap", null_mocap),
        ):
            with self.subTest(label), tempfile.TemporaryDirectory() as root:
                config_path = Path(root) / "config.json"
                config_path.write_text(json.dumps(payload), encoding="utf-8")

                app = create_app(data_root=root, **TESTSERVER_APP)
                with TestClient(app) as client:
                    status = client.get("/api/v1/status").json()
                    self.assertEqual(status["state"], "setup_required")
                    self.assertEqual(client.get("/api/v1/config").json()["mocap"]["port"], 5555)
                    rejected = [
                        record
                        for record in client.get("/api/v1/audit").json()["records"]
                        if record["category"] == "configuration"
                        and record["action"] == "load"
                    ]
                    self.assertEqual(rejected[-1]["outcome"], "rejected")
                    self.assertEqual(
                        rejected[-1]["details"]["code"], "invalid_configuration"
                    )

                    response = client.put(
                        "/api/v1/config", json={"mocap": {"port": 5556}}
                    )
                    self.assertEqual(response.status_code, 200, response.text)
                    self.assertEqual(client.get("/api/v1/status").json()["state"], "idle")
                self.assertEqual(
                    json.loads(config_path.read_text(encoding="utf-8"))["mocap"]["port"],
                    5556,
                )


@unittest.skipUnless(_API_TESTS_AVAILABLE, "FastAPI/httpx test dependencies are not installed")
class TrustedIdentityTests(unittest.TestCase):
    HEADER = "X-Authenticated-User"

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)

    def client(self, peer: str, **kwargs):
        from fastapi.testclient import TestClient

        from mocap_g1.operator.app import create_app

        kwargs.setdefault("data_root", self.temporary.name)
        kwargs.setdefault("demo", True)
        client = TestClient(
            create_app(**kwargs),
            base_url="http://127.0.0.1:8766",
            client=(peer, 40000),
        )
        self.addCleanup(client.close)
        return client

    def last(self, client, category: str, action: str) -> dict:
        records = client.get("/api/v1/audit").json()["records"]
        return [
            record
            for record in records
            if record["category"] == category and record["action"] == action
        ][-1]

    def test_identity_header_is_trusted_only_from_configured_proxy(self):
        proxied = self.client(
            "127.0.0.1",
            identity_header=self.HEADER,
            trusted_proxies=("127.0.0.1/32",),
        )
        response = proxied.post(
            "/api/v1/actions/check",
            json={"operator": "typed-name"},
            headers={self.HEADER: "alice@example.test"},
        )
        self.assertEqual(response.status_code, 200, response.text)
        record = self.last(proxied, "control", "check")
        self.assertEqual(record["actor"], "alice@example.test")
        self.assertEqual(record["details"]["claimed_operator"], "typed-name")

        response = proxied.put(
            "/api/v1/config",
            json={"mocap": {"port": 5557}},
            headers={self.HEADER: "alice@example.test", "x-operator-name": "typed-name"},
        )
        self.assertEqual(response.status_code, 409, response.text)
        self.assertEqual(response.json()["error"]["code"], "configuration_locked")
        record = self.last(proxied, "control", "config")
        self.assertEqual(record["actor"], "alice@example.test")
        self.assertEqual(record["details"]["claimed_operator"], "typed-name")

    def test_identity_header_from_untrusted_peer_is_ignored(self):
        direct = self.client(
            "203.0.113.9",
            identity_header=self.HEADER,
            trusted_proxies=("127.0.0.1",),
        )
        response = direct.post(
            "/api/v1/actions/check",
            json={"operator": "mallory"},
            headers={self.HEADER: "alice@example.test"},
        )
        self.assertEqual(response.status_code, 200, response.text)
        record = self.last(direct, "control", "check")
        self.assertEqual(record["actor"], "local-console")
        self.assertEqual(record["details"]["claimed_operator"], "mallory")

    def test_without_proxy_configuration_actor_is_the_local_console(self):
        client = self.client("127.0.0.1")
        response = client.put(
            "/api/v1/config",
            json={"topology": "split", "mocap": {"source_ip": "192.168.50.10"}},
            headers={self.HEADER: "alice@example.test", "x-operator-name": "bob"},
        )
        self.assertEqual(response.status_code, 200, response.text)
        record = self.last(client, "configuration", "update")
        self.assertEqual(record["actor"], "local-console")
        self.assertEqual(record["details"]["claimed_operator"], "bob")

        client.post("/api/v1/actions/check")
        record = self.last(client, "control", "check")
        self.assertEqual(record["actor"], "local-console")
        self.assertNotIn("claimed_operator", record["details"])

    def test_identity_configuration_must_be_complete_and_valid(self):
        from mocap_g1.operator.app import create_app

        for kwargs in (
            {"identity_header": self.HEADER},
            {"trusted_proxies": ("127.0.0.1",)},
            {"identity_header": "bad header", "trusted_proxies": ("127.0.0.1",)},
            {"identity_header": "Host", "trusted_proxies": ("127.0.0.1",)},
            {"identity_header": self.HEADER, "trusted_proxies": ("not-an-ip",)},
        ):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                create_app(data_root=self.temporary.name, demo=True, **kwargs)


@unittest.skipUnless(_API_TESTS_AVAILABLE, "FastAPI/httpx test dependencies are not installed")
class RestartDetectionTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)

    def client(self):
        from fastapi.testclient import TestClient

        from mocap_g1.operator.app import create_app

        client = TestClient(
            create_app(
                data_root=self.temporary.name, demo=True, **TESTSERVER_APP
            )
        )
        self.addCleanup(client.close)
        return client

    def test_each_process_instance_has_a_distinct_id_on_status_and_events(self):
        first = self.client()
        first_status = first.get("/api/v1/status").json()
        self.assertRegex(first_status["instance_id"], r"^[0-9a-f]{32}$")
        first.post("/api/v1/actions/check")
        first_audit_sequence = first.get("/api/v1/audit").json()["last_sequence"]

        # Same data root: durable audit numbering continues across a restart
        # while the in-memory event counter and instance id start over.
        second = self.client()
        second_status = second.get("/api/v1/status").json()
        self.assertNotEqual(second_status["instance_id"], first_status["instance_id"])
        self.assertLess(
            second_status["last_event_sequence"], first_audit_sequence
        )

        with second.websocket_connect("/api/v1/events?after_sequence=500") as socket:
            snapshot = socket.receive_json()
            self.assertEqual(snapshot["type"], "snapshot")
            self.assertEqual(snapshot["instance_id"], second_status["instance_id"])
            self.assertEqual(snapshot["status"]["instance_id"], second_status["instance_id"])
            # A stale cursor is clamped so new events are not silently skipped.
            self.assertEqual(snapshot["sequence"], second_status["last_event_sequence"])
            second.post("/api/v1/actions/check")
            event = socket.receive_json()
            while event["type"] == "snapshot":
                event = socket.receive_json()
        self.assertEqual(event["instance_id"], second_status["instance_id"])
        self.assertEqual(event["sequence"], second_status["last_event_sequence"] + 1)
        self.assertGreater(event["audit_sequence"], first_audit_sequence)
        self.assertNotEqual(event["audit_sequence"], event["sequence"])


@unittest.skipUnless(_API_TESTS_AVAILABLE, "FastAPI/httpx test dependencies are not installed")
class EventLoopOffloadTests(unittest.TestCase):
    def test_blocking_controller_and_diagnostics_work_runs_off_the_event_loop(self):
        from fastapi.testclient import TestClient

        from mocap_g1.operator import app as app_module

        def on_event_loop() -> bool:
            try:
                asyncio.get_running_loop()
            except RuntimeError:
                return False
            return True

        observed: dict[str, list[bool]] = {}

        def recording(name, function):
            def wrapper(*args, **kwargs):
                observed.setdefault(name, []).append(on_event_loop())
                return function(*args, **kwargs)

            return wrapper

        with tempfile.TemporaryDirectory() as root:
            application = app_module.create_app(
                data_root=root, demo=True, **TESTSERVER_APP
            )
            controller = application.state.controller
            with mock.patch.object(
                controller, "update_config", recording("update_config", controller.update_config)
            ), mock.patch.object(
                controller, "perform", recording("perform", controller.perform)
            ), mock.patch.object(
                controller,
                "watchdog_tick",
                recording("watchdog_tick", controller.watchdog_tick),
            ), mock.patch.object(
                app_module, "redact", recording("diagnostics_redact", app_module.redact)
            ), TestClient(application) as client:
                self.assertEqual(
                    client.put("/api/v1/config", json={"mocap": {"port": 5560}}).status_code,
                    200,
                )
                self.assertEqual(client.post("/api/v1/actions/check").status_code, 200)
                self.assertEqual(client.get("/api/v1/diagnostics").status_code, 200)
                self.assertTrue(wait_for(lambda: "watchdog_tick" in observed))

        for name in ("update_config", "perform", "watchdog_tick", "diagnostics_redact"):
            with self.subTest(name=name):
                self.assertTrue(observed.get(name))
                self.assertFalse(any(observed[name]))


if __name__ == "__main__":
    unittest.main()
