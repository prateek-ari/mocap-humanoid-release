from __future__ import annotations

from importlib.util import find_spec
import json
from pathlib import Path
import tempfile
import unittest


# FastAPI's TestClient sends ``Host: testserver``; trust it explicitly rather
# than widening the production default allowlist.
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


@unittest.skipUnless(_API_TESTS_AVAILABLE, "FastAPI/httpx test dependencies are not installed")
class OperatorApiTests(unittest.TestCase):
    def setUp(self):
        from fastapi.testclient import TestClient

        from mocap_g1.operator.app import create_app

        self.temporary = tempfile.TemporaryDirectory()
        self.client = TestClient(create_app(data_root=self.temporary.name, demo=True, **TESTSERVER_APP))

    def tearDown(self):
        if hasattr(self, "client"):
            self.client.close()
        self.temporary.cleanup()

    def test_status_uses_canonical_v1_schema_and_is_locked_by_default(self):
        response = self.client.get("/api/v1/status")
        self.assertEqual(response.status_code, 200)
        status = response.json()
        self.assertEqual(status["schema_version"], 1)
        self.assertEqual(status["presentation_mode"], "demo")
        self.assertEqual(status["state"], "idle")
        self.assertEqual(status["topology"], "all_in_one")
        self.assertEqual(status["runtime_mode"], "fake")
        self.assertFalse(status["capability_lock"]["whole_body_unlocked"])
        self.assertIn("command_authority", status["health"])
        self.assertIn("disengage", status["controls"])

    def test_browser_favicon_fallback_serves_the_authored_svg(self):
        response = self.client.get("/favicon.ico")

        self.assertEqual(response.status_code, 200, response.text)
        self.assertEqual(response.headers["content-type"], "image/svg+xml")
        self.assertIn(b"<svg", response.content)

    def test_live_status_has_canonical_presentation_mode(self):
        from fastapi.testclient import TestClient

        from mocap_g1.operator.app import create_app

        with tempfile.TemporaryDirectory() as temporary, TestClient(
            create_app(data_root=temporary, **TESTSERVER_APP)
        ) as client:
            status = client.get("/api/v1/status").json()
        self.assertEqual(status["presentation_mode"], "live")

    def test_lifespan_runs_and_cancels_watchdog_without_http_traffic(self):
        from threading import Event
        from unittest.mock import patch

        from fastapi.testclient import TestClient

        from mocap_g1.operator.app import create_app

        tick_seen = Event()
        with tempfile.TemporaryDirectory() as temporary:
            app = create_app(data_root=temporary, demo=True, **TESTSERVER_APP)
            with patch.object(
                app.state.controller,
                "watchdog_tick",
                side_effect=tick_seen.set,
            ) as watchdog_tick:
                with TestClient(app):
                    self.assertTrue(tick_seen.wait(timeout=1.0))
                    task = app.state.watchdog_task
                    self.assertIsNotNone(task)
                    self.assertFalse(task.done())

                self.assertGreaterEqual(watchdog_tick.call_count, 1)
                self.assertTrue(task.done())
                self.assertTrue(task.cancelled())
                self.assertIsNone(app.state.watchdog_task)

    def test_demo_action_flow_and_immediate_disengage(self):
        expected = {
            "check": "checking",
            "mocap-accept": "mocap_accepted",
            "start-simulation": "sim_validated",
            "connect-robot": "robot_connected",
            "confirm-preflight": "preflight_passed",
            "arm": "armed_disabled",
            "start-tracking": "tracking",
            "disengage": "armed_disabled",
        }
        for action, state in expected.items():
            response = self.client.post(f"/api/v1/actions/{action}")
            self.assertEqual(response.status_code, 200, response.text)
            self.assertEqual(response.json()["state"], state)
            self.assertEqual(response.json()["presentation_mode"], "demo")
        self.assertFalse(response.json()["health"]["command_authority"])

    def test_config_and_audit_contracts(self):
        response = self.client.put(
            "/api/v1/config",
            json={"topology": "split", "mocap": {"source_ip": "192.168.50.10"}},
            headers={"x-operator-name": "engineer-a"},
        )
        self.assertEqual(response.status_code, 200, response.text)
        self.assertEqual(response.json()["topology"], "split")
        self.assertEqual(
            self.client.get("/api/v1/config").json()["mocap"]["source_ip"],
            "192.168.50.10",
        )
        audit = self.client.get("/api/v1/audit").json()
        self.assertIn("records", audit)
        self.assertGreaterEqual(audit["last_sequence"], 1)
        # A browser-supplied name is evidence, not identity: without a
        # configured trusted proxy the actor is the local console.
        updates = [
            record
            for record in audit["records"]
            if record["category"] == "configuration" and record["action"] == "update"
        ]
        self.assertEqual(updates[-1]["actor"], "local-console")
        self.assertEqual(updates[-1]["details"]["claimed_operator"], "engineer-a")
        self.assertFalse(
            any(record["actor"] == "engineer-a" for record in audit["records"])
        )

    def test_same_origin_mutations_and_clients_without_origin_are_allowed(self):
        response = self.client.put(
            "/api/v1/config",
            json={"topology": "split", "mocap": {"source_ip": "192.168.50.10"}},
            headers={"origin": "http://testserver:80"},
        )
        self.assertEqual(response.status_code, 200, response.text)
        self.assertEqual(response.json()["topology"], "split")

        response = self.client.post(
            "/api/v1/actions/check",
            headers={"origin": "http://testserver"},
        )
        self.assertEqual(response.status_code, 200, response.text)
        self.assertEqual(response.json()["state"], "checking")

        response = self.client.post("/api/v1/actions/mocap-accept")
        self.assertEqual(response.status_code, 200, response.text)
        self.assertEqual(response.json()["state"], "mocap_accepted")

    def test_cross_origin_mutations_are_rejected_without_state_change(self):
        initial_config = self.client.get("/api/v1/config").json()
        for origin in (
            "https://attacker.example",
            "null",
            "http://testserver.attacker.example",
            "http://testserver/path",
        ):
            with self.subTest(origin=origin):
                response = self.client.put(
                    "/api/v1/config",
                    json={"topology": "split"},
                    headers={"origin": origin},
                )
                self.assertEqual(response.status_code, 403, response.text)
                self.assertEqual(
                    response.json()["detail"],
                    "Cross-origin request is not allowed",
                )

        response = self.client.post(
            "/api/v1/actions/check",
            headers={"origin": "https://attacker.example"},
        )
        self.assertEqual(response.status_code, 403, response.text)
        self.assertEqual(self.client.get("/api/v1/status").json()["state"], "idle")
        self.assertEqual(self.client.get("/api/v1/config").json(), initial_config)

    def test_mutation_bodies_require_json_and_enforce_size_limit(self):
        response = self.client.post(
            "/api/v1/actions/check",
            content='{"operator":"browser-form"}',
            headers={"content-type": "text/plain"},
        )
        self.assertEqual(response.status_code, 415, response.text)
        self.assertEqual(
            response.json()["detail"],
            "Content-Type must be application/json",
        )

        response = self.client.post(
            "/api/v1/actions/check",
            json={"metadata": {"padding": "x" * (64 * 1024)}},
        )
        self.assertEqual(response.status_code, 413, response.text)
        self.assertEqual(response.json()["detail"], "Request body is too large")
        self.assertEqual(self.client.get("/api/v1/status").json()["state"], "idle")

        for constant in ("NaN", "Infinity", "-Infinity"):
            with self.subTest(constant=constant):
                response = self.client.post(
                    "/api/v1/actions/check",
                    content=f'{{"metadata":{{"value":{constant}}}}}',
                    headers={"content-type": "application/json"},
                )
                self.assertEqual(response.status_code, 400, response.text)
                self.assertEqual(response.json()["detail"], "Request body must be JSON")
        self.assertEqual(self.client.get("/api/v1/status").json()["state"], "idle")

    def test_invalid_action_order_has_stable_error_schema(self):
        response = self.client.post("/api/v1/actions/start-tracking")
        self.assertEqual(response.status_code, 409)
        self.assertEqual(response.json()["error"]["code"], "invalid_action")
        self.assertIn("message", response.json()["error"])

    def test_event_socket_observes_client_disconnect_and_can_reconnect(self):
        with self.client.websocket_connect("/api/v1/events") as websocket:
            snapshot = websocket.receive_json()
            self.assertEqual(snapshot["type"], "snapshot")
            self.assertEqual(snapshot["status"]["schema_version"], 1)
        # Closing the first context delivers websocket.disconnect to the app.
        # A fresh socket proves the prior connection task exited cleanly.
        with self.client.websocket_connect("/api/v1/events") as websocket:
            self.assertEqual(websocket.receive_json()["type"], "snapshot")

    def test_event_socket_rejects_cross_origin_browser_handshake(self):
        from starlette.websockets import WebSocketDisconnect

        with self.assertRaises(WebSocketDisconnect) as raised:
            with self.client.websocket_connect(
                "/api/v1/events",
                headers={"origin": "https://attacker.example"},
            ):
                pass
        self.assertEqual(raised.exception.code, 1008)

    def test_diagnostics_is_attachment_with_redacted_canonical_schema(self):
        secret = "diagnostics-must-not-leak-this-token"
        raw_motion = "raw-body-motion-must-not-leak"
        response = self.client.post(
            "/api/v1/actions/check",
            json={
                "operator": "engineer-a",
                "metadata": {"token": secret, "raw_mocap": raw_motion},
            },
        )
        self.assertEqual(response.status_code, 200, response.text)
        response = self.client.get("/api/v1/diagnostics")
        self.assertEqual(response.status_code, 200)
        disposition = response.headers["content-disposition"]
        self.assertTrue(disposition.startswith("attachment;"))
        self.assertIn("mocap-g1-diagnostics-", disposition)
        diagnostics = response.json()
        self.assertEqual(diagnostics["schema_version"], 1)
        from mocap_g1.operator._version import service_version

        self.assertEqual(diagnostics["service_version"], service_version())
        self.assertNotEqual(diagnostics["service_version"], "1.0.0")
        self.assertTrue(diagnostics["generated_at"].endswith("Z"))
        self.assertIn("state", diagnostics["status"])
        self.assertEqual(diagnostics["config"]["schema_version"], 1)
        self.assertIn("python", diagnostics["system_probe"])
        self.assertIsInstance(diagnostics["audit"], list)
        self.assertNotIn(secret, response.text)
        self.assertNotIn(raw_motion, response.text)
        matching = [
            record
            for record in diagnostics["audit"]
            if record["action"] == "check"
            and record["actor"] == "local-console"
            and record["details"].get("claimed_operator") == "engineer-a"
        ]
        self.assertEqual(
            matching[-1]["details"]["request"]["metadata"]["token"],
            "[REDACTED]",
        )

    def test_startup_and_shutdown_clear_persisted_raw_mocap_consent(self):
        self.client.close()
        config_path = Path(self.temporary.name) / "config.json"
        config_path.write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "topology": "all_in_one",
                    "runtime_mode": "fake",
                    "mocap": {
                        "listen_address": "0.0.0.0",
                        "port": 5555,
                        "source_ip": None,
                    },
                    "robot": {"interface": None},
                    "privacy": {"record_raw_mocap": True},
                }
            ),
            encoding="utf-8",
        )
        from fastapi.testclient import TestClient

        from mocap_g1.operator.app import create_app

        self.client = TestClient(create_app(data_root=self.temporary.name, demo=True, **TESTSERVER_APP))
        self.assertFalse(
            self.client.get("/api/v1/config").json()["privacy"]["record_raw_mocap"]
        )
        self.assertFalse(
            json.loads(config_path.read_text(encoding="utf-8"))["privacy"][
                "record_raw_mocap"
            ]
        )
        response = self.client.put(
            "/api/v1/config", json={"privacy": {"record_raw_mocap": True}}
        )
        self.assertTrue(response.json()["privacy"]["record_raw_mocap"])
        self.assertEqual(
            self.client.post("/api/v1/actions/shutdown").status_code, 200
        )
        self.assertFalse(
            json.loads(config_path.read_text(encoding="utf-8"))["privacy"][
                "record_raw_mocap"
            ]
        )

    def test_recorded_playback_serves_only_configured_video_without_state_change(self):
        self.client.close()
        video = Path(self.temporary.name) / "client recording.mp4"
        video_bytes = b"\x00\x00\x00\x18ftypmp42recorded-review"
        video.write_bytes(video_bytes)
        (Path(self.temporary.name) / "config.json").write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "topology": "all_in_one",
                    "runtime_mode": "fake",
                    "mocap": {
                        "listen_address": "0.0.0.0",
                        "port": 5555,
                        "source_ip": None,
                    },
                    "robot": {"interface": None},
                    "privacy": {"record_raw_mocap": False},
                }
            ),
            encoding="utf-8",
        )
        persisted_audit = Path(self.temporary.name) / "audit.jsonl"
        persisted_audit.write_text(
            json.dumps(
                {
                    "sequence": 99,
                    "timestamp": 1.0,
                    "category": "production",
                    "action": "must_not_load",
                    "outcome": "accepted",
                    "state": "tracking",
                    "actor": "production",
                    "details": {},
                },
                sort_keys=True,
                separators=(",", ":"),
            )
            + "\n",
            encoding="utf-8",
        )
        persisted_manifest = Path(self.temporary.name) / "capability_manifest.json"
        persisted_manifest.write_text(
            json.dumps({"capabilities": ["whole_body_hardware"], "signature": "ignored"}),
            encoding="utf-8",
        )
        persisted_before = {
            path: path.read_bytes()
            for path in (
                Path(self.temporary.name) / "config.json",
                persisted_audit,
                persisted_manifest,
            )
        }

        from fastapi.testclient import TestClient

        from mocap_g1.operator.app import create_app

        self.client = TestClient(
            create_app(data_root=self.temporary.name, playback=video, **TESTSERVER_APP)
        )
        self.assertIsNone(self.client.app.state.data_root)
        status_before = self.client.get("/api/v1/status").json()
        config_before = self.client.get("/api/v1/config").json()
        audit_before = self.client.get("/api/v1/audit").json()

        manifest_response = self.client.get("/api/v1/playback")
        self.assertEqual(manifest_response.status_code, 200)
        manifest = manifest_response.json()
        self.assertEqual(manifest["filename"], video.name)
        self.assertEqual(manifest["presentation_mode"], "recorded_playback")
        self.assertNotIn(str(video.parent), manifest_response.text)
        self.assertEqual(manifest["video_url"], "/api/v1/playback/video")
        self.assertEqual(manifest["review_mode"], "simulation_preview")
        self.assertEqual(manifest["hardware_status"], "blocked")
        self.assertFalse(manifest["command_authority"])

        video_response = self.client.get(manifest["video_url"])
        self.assertEqual(video_response.status_code, 200)
        self.assertEqual(video_response.headers["content-type"], "video/mp4")
        self.assertIn("inline", video_response.headers["content-disposition"])
        self.assertEqual(video_response.headers["cache-control"], "private, no-store")
        self.assertEqual(
            video_response.headers["cross-origin-resource-policy"], "same-origin"
        )
        self.assertEqual(video_response.content, video_bytes)

        self.assertEqual(self.client.get("/api/v1/status").json(), status_before)
        self.assertEqual(self.client.get("/api/v1/audit").json(), audit_before)
        self.assertEqual(status_before["state"], "idle")
        self.assertEqual(status_before["presentation_mode"], "recorded_playback")
        self.assertEqual(status_before["runtime_mode"], "real")
        self.assertFalse(status_before["health"]["command_authority"])
        self.assertFalse(status_before["capability_lock"]["whole_body_unlocked"])
        self.assertEqual(
            status_before["capability_lock"]["reason"],
            "Recorded playback mode never grants hardware authority",
        )
        self.assertEqual(config_before["runtime_mode"], "real")
        self.assertEqual(audit_before["last_sequence"], 1)
        self.assertFalse(
            any(record["action"] == "must_not_load" for record in audit_before["records"])
        )
        self.assertTrue(
            all(not control["enabled"] for control in status_before["controls"].values())
        )
        with self.client.websocket_connect("/api/v1/events") as websocket:
            self.assertEqual(websocket.receive_json()["status"], status_before)
        diagnostics = self.client.get("/api/v1/diagnostics").json()
        self.assertEqual(diagnostics["status"], status_before)
        self.assertEqual(diagnostics["config"], config_before)
        self.assertEqual(diagnostics["audit"], audit_before["records"])

        expected_error = {
            "error": {
                "code": "playback_read_only",
                "message": (
                    "Recorded playback is read-only; operator actions and "
                    "configuration changes are disabled"
                ),
            }
        }
        for action in ("check", "not-a-real-action"):
            rejected = self.client.post(
                f"/api/v1/actions/{action}", json={"operator": "reviewer"}
            )
            self.assertEqual(rejected.status_code, 423)
            self.assertEqual(rejected.json(), expected_error)
        rejected_config = self.client.put(
            "/api/v1/config",
            json={"runtime_mode": "fake", "privacy": {"record_raw_mocap": True}},
        )
        self.assertEqual(rejected_config.status_code, 423)
        self.assertEqual(rejected_config.json(), expected_error)

        self.assertEqual(self.client.get("/api/v1/status").json(), status_before)
        self.assertEqual(self.client.get("/api/v1/config").json(), config_before)
        self.assertEqual(self.client.get("/api/v1/audit").json(), audit_before)
        for path, content in persisted_before.items():
            self.assertEqual(path.read_bytes(), content)

    def test_playback_endpoints_are_absent_without_configured_media(self):
        self.assertEqual(self.client.get("/api/v1/playback").status_code, 404)
        self.assertEqual(self.client.get("/api/v1/playback/video").status_code, 404)


if __name__ == "__main__":
    unittest.main()
