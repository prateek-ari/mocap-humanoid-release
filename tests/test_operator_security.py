from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import stat
import tempfile
import unittest
from unittest import mock

from mocap_g1.operator import AuditLog, CapabilityManifestVerifier, sign_manifest
from mocap_g1.operator.audit import redact
from mocap_g1.operator.capabilities import WHOLE_BODY_CAPABILITY


class CapabilityManifestTests(unittest.TestCase):
    def manifest(self):
        return {
            "manifest_id": "qualification-001",
            "expires_at": "2035-01-01T00:00:00Z",
            "capabilities": [WHOLE_BODY_CAPABILITY],
        }

    def test_missing_key_or_manifest_fails_closed(self):
        self.assertFalse(CapabilityManifestVerifier(None).verify(self.manifest()).whole_body_unlocked)
        self.assertFalse(CapabilityManifestVerifier("key").verify(None).whole_body_unlocked)

    def test_valid_signed_manifest_unlocks_whole_body(self):
        signed = sign_manifest(self.manifest(), "test-secret")
        result = CapabilityManifestVerifier("test-secret").verify(signed)
        self.assertTrue(result.manifest_verified)
        self.assertTrue(result.whole_body_unlocked)
        self.assertEqual(result.manifest_id, "qualification-001")

    def test_tampering_invalidates_signature(self):
        signed = sign_manifest(self.manifest(), "test-secret")
        signed["capabilities"] = []
        result = CapabilityManifestVerifier("test-secret").verify(signed)
        self.assertFalse(result.manifest_verified)
        self.assertFalse(result.whole_body_unlocked)
        self.assertIn("invalid", result.reason)

    def test_expired_manifest_remains_locked(self):
        now = datetime(2036, 1, 1, tzinfo=timezone.utc)
        signed = sign_manifest(self.manifest(), "test-secret")
        result = CapabilityManifestVerifier("test-secret").verify(signed, now=now)
        self.assertFalse(result.whole_body_unlocked)
        self.assertIn("expired", result.reason)

    def test_verified_manifest_without_capability_stays_locked(self):
        manifest = self.manifest()
        manifest["capabilities"] = ["simulation_only"]
        signed = sign_manifest(manifest, "test-secret")
        result = CapabilityManifestVerifier("test-secret").verify(signed)
        self.assertTrue(result.manifest_verified)
        self.assertFalse(result.whole_body_unlocked)

    def test_signing_does_not_mutate_input(self):
        manifest = self.manifest()
        signed = sign_manifest(manifest, "secret")
        self.assertNotIn("signature", manifest)
        self.assertIn("signature", signed)


class AuditLogTests(unittest.TestCase):
    def test_recursive_redaction_does_not_mutate_input(self):
        value = {
            "token": "abc",
            "nested": {"password": "pw", "safe": 12},
            "items": [{"Authorization": "Bearer xyz"}],
        }
        clean = redact(value)
        self.assertEqual(clean["token"], "[REDACTED]")
        self.assertEqual(clean["nested"]["password"], "[REDACTED]")
        self.assertEqual(clean["items"][0]["Authorization"], "[REDACTED]")
        self.assertEqual(value["token"], "abc")

    def test_jsonl_is_append_only_and_sequence_survives_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "audit.jsonl"
            first = AuditLog(path)
            first.append(
                category="control",
                action="check",
                outcome="accepted",
                state="checking",
                details={"api_key": "do-not-log"},
            )
            before = path.read_text(encoding="utf-8")
            second = AuditLog(path)
            record = second.append(
                category="control",
                action="shutdown",
                outcome="accepted",
                state="idle",
            )
            after = path.read_text(encoding="utf-8")
            self.assertTrue(after.startswith(before))
            self.assertEqual(record["sequence"], 2)
            lines = [json.loads(line) for line in after.splitlines()]
            self.assertEqual(lines[0]["details"]["api_key"], "[REDACTED]")
            self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o600)

    def test_legacy_records_are_redacted_when_loaded(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "audit.jsonl"
            path.write_text(
                json.dumps({
                    "sequence": 1,
                    "timestamp": 1.0,
                    "category": "control",
                    "action": "check",
                    "outcome": "accepted",
                    "state": "checking",
                    "actor": "legacy",
                    "details": {"token": "legacy-secret", "safe": 12},
                }) + "\n",
                encoding="utf-8",
            )

            record = AuditLog(path).read()[0]

            self.assertEqual(record["details"]["token"], "[REDACTED]")
            self.assertEqual(record["details"]["safe"], 12)
            self.assertIn("legacy-secret", path.read_text(encoding="utf-8"))

    def test_existing_sequence_requires_a_positive_json_integer(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "audit.jsonl"
            records = [
                {"sequence": True, "details": {}},
                {"sequence": 1.5, "details": {}},
                {"sequence": "1", "details": {}},
                {"sequence": 0, "details": {}},
                {"sequence": -1, "details": {}},
                {"sequence": 1, "details": {"marker": "valid"}},
            ]
            path.write_text(
                "".join(json.dumps(record) + "\n" for record in records),
                encoding="utf-8",
            )

            audit = AuditLog(path)

            loaded = audit.read()
            self.assertEqual([record["sequence"] for record in loaded], [1])
            self.assertEqual(loaded[0]["details"]["marker"], "valid")
            quarantine = path.with_suffix(".corrupt.log")
            quarantined = [
                json.loads(line)
                for line in quarantine.read_text(encoding="utf-8").splitlines()
            ]
            self.assertEqual(len(quarantined), 5)
            self.assertTrue(
                all(record["error_type"] == "ValueError" for record in quarantined)
            )

    def test_read_supports_cursor_and_validates_limit(self):
        audit = AuditLog()
        for index in range(5):
            audit.append(
                category="test",
                action=str(index),
                outcome="accepted",
                state="idle",
            )
        self.assertEqual([r["sequence"] for r in audit.read(after_sequence=3)], [4, 5])
        self.assertEqual([r["sequence"] for r in audit.read(limit=2)], [1, 2])
        self.assertEqual(
            [r["sequence"] for r in audit.read(limit=2, newest=True)], [4, 5]
        )
        with self.assertRaisesRegex(ValueError, "between"):
            audit.read(limit=0)

    def test_invalid_existing_record_is_quarantined_without_copying_secrets(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "audit.jsonl"
            secret = "not-json bearer-token=do-not-copy"
            path.write_text(
                '{"sequence": 1, "category": "test", "action": "a", '
                '"outcome": "accepted", "state": "idle"}\n'
                f"{secret}\n",
                encoding="utf-8",
            )
            audit = AuditLog(path)
            self.assertEqual(len(audit.read()), 1)
            quarantine = Path(directory) / "audit.corrupt.log"
            self.assertTrue(quarantine.is_file())
            payload = json.loads(quarantine.read_text(encoding="utf-8"))
            self.assertNotIn(secret, quarantine.read_text(encoding="utf-8"))
            self.assertEqual(payload["content_bytes"], len(secret.encode("utf-8")))
            self.assertEqual(len(payload["content_sha256"]), 64)
            self.assertEqual(stat.S_IMODE(quarantine.stat().st_mode), 0o600)

            # The malformed source line remains for forensic integrity, but a
            # restart must not append the same quarantine record again.
            before = quarantine.read_bytes()
            AuditLog(path)
            self.assertEqual(quarantine.read_bytes(), before)

    def test_invalid_utf8_is_quarantined_without_crashing_or_copying_bytes(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "audit.jsonl"
            valid = json.dumps({"sequence": 1, "details": {}}).encode("utf-8")
            corrupt = b"private-token=must-not-copy\xff\xfe"
            original = valid + b"\n" + corrupt + b"\n"
            path.write_bytes(original)

            audit = AuditLog(path)

            self.assertEqual([record["sequence"] for record in audit.read()], [1])
            self.assertEqual(path.read_bytes(), original)
            quarantine = path.with_suffix(".corrupt.log")
            quarantine_bytes = quarantine.read_bytes()
            self.assertNotIn(b"private-token", quarantine_bytes)
            payload = json.loads(quarantine_bytes)
            self.assertEqual(payload["error_type"], "UnicodeDecodeError")
            self.assertEqual(payload["content_bytes"], len(corrupt))
            self.assertEqual(
                payload["content_sha256"],
                hashlib.sha256(corrupt).hexdigest(),
            )

            before = quarantine.read_bytes()
            AuditLog(path)
            self.assertEqual(quarantine.read_bytes(), before)

    def test_invalid_utf8_quarantine_index_does_not_block_primary_log(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "audit.jsonl"
            path.write_text('{"sequence":1,"details":{}}\n', encoding="utf-8")
            quarantine = path.with_suffix(".corrupt.log")
            quarantine.write_bytes(b"private-index-token\xff\n")

            audit = AuditLog(path)
            appended = audit.append(
                category="control",
                action="shutdown",
                outcome="accepted",
                state="idle",
            )

            # The legacy (unchained) file gains a chain_start marker before
            # the first chained record.
            records = audit.read()
            self.assertEqual([record["sequence"] for record in records], [1, 2, 3])
            self.assertEqual(records[1]["action"], "chain_start")
            self.assertEqual(appended["sequence"], 3)

    def test_oversized_line_is_streamed_to_hash_and_following_record_loads(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "audit.jsonl"
            oversized = b"private-token=" + b"x" * 256
            path.write_bytes(oversized + b"\n" + b'{"sequence":1,"details":{}}\n')

            with mock.patch("mocap_g1.operator.audit._MAX_AUDIT_LINE_BYTES", 128):
                audit = AuditLog(path)

            self.assertEqual([record["sequence"] for record in audit.read()], [1])
            quarantine = path.with_suffix(".corrupt.log")
            quarantine_bytes = quarantine.read_bytes()
            self.assertNotIn(b"private-token", quarantine_bytes)
            payload = json.loads(quarantine_bytes)
            self.assertEqual(payload["error_type"], "AuditLineTooLong")
            self.assertEqual(payload["content_bytes"], len(oversized))
            self.assertEqual(
                payload["content_sha256"],
                hashlib.sha256(oversized).hexdigest(),
            )

    def test_quarantine_index_is_bounded_and_stable_across_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "audit.jsonl"
            path.write_text("bad-one\nbad-two\n", encoding="utf-8")

            with mock.patch("mocap_g1.operator.audit._MAX_QUARANTINE_KEYS", 1):
                AuditLog(path)
                quarantine = Path(directory) / "audit.corrupt.log"
                first = quarantine.read_bytes()
                self.assertEqual(len(first.splitlines()), 1)

                AuditLog(path)
                self.assertEqual(quarantine.read_bytes(), first)

    def test_malformed_quarantine_key_cannot_suppress_corruption_record(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "audit.jsonl"
            corrupt = b"not-json"
            path.write_bytes(corrupt + b"\n")
            quarantine = path.with_suffix(".corrupt.log")
            quarantine.write_text(
                json.dumps({
                    "quarantined_from": str(path),
                    "line_number": True,
                    "content_sha256": hashlib.sha256(corrupt).hexdigest(),
                }) + "\n",
                encoding="utf-8",
            )

            AuditLog(path)

            records = [
                json.loads(line)
                for line in quarantine.read_text(encoding="utf-8").splitlines()
            ]
            self.assertEqual(len(records), 2)
            self.assertEqual(records[-1]["line_number"], 1)
            self.assertEqual(records[-1]["error_type"], "JSONDecodeError")

    def test_deeply_nested_existing_record_is_quarantined(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "audit.jsonl"
            nested = "[" * 2_000 + "0" + "]" * 2_000
            path.write_text(
                f'{{"sequence":1,"details":{nested}}}\n',
                encoding="utf-8",
            )

            audit = AuditLog(path)

            self.assertEqual(audit.read(), [])
            self.assertTrue((Path(directory) / "audit.corrupt.log").is_file())



VALID_SPLIT_CONFIG = {"topology": "split", "mocap": {"source_ip": "192.168.50.10"}}
_API_TESTS_AVAILABLE = (
    importlib.util.find_spec("fastapi") is not None
    and importlib.util.find_spec("httpx") is not None
)


@unittest.skipUnless(_API_TESTS_AVAILABLE, "FastAPI/httpx test dependencies are not installed")
class HostAllowlistTests(unittest.TestCase):
    """DNS rebinding: Host and Origin must match an explicit allowlist."""

    LOCAL = "http://127.0.0.1:8766"
    REBOUND = ("evil.example:8766", "http://evil.example:8766")

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)

    def client(self, **kwargs):
        from fastapi.testclient import TestClient

        from mocap_g1.operator.app import create_app

        kwargs.setdefault("data_root", self.temporary.name)
        kwargs.setdefault("demo", True)
        client = TestClient(create_app(**kwargs), base_url=self.LOCAL)
        self.addCleanup(client.close)
        return client

    def assert_invalid_host(self, response):
        self.assertEqual(response.status_code, 400, response.text)
        self.assertEqual(response.json()["detail"], "Invalid host header")

    def test_rebinding_host_and_origin_cannot_mutate_or_read(self):
        client = self.client()
        initial_config = client.get("/api/v1/config").json()
        for host, origin in (
            self.REBOUND,
            ("evil.example", "http://evil.example"),
            ("127.0.0.1.evil.example:8766", "http://127.0.0.1.evil.example:8766"),
            ("127.0.0.1:9999", "http://127.0.0.1:9999"),
        ):
            with self.subTest(host=host):
                headers = {"host": host, "origin": origin}
                self.assert_invalid_host(
                    client.post("/api/v1/actions/check", headers=headers)
                )
                self.assert_invalid_host(
                    client.put(
                        "/api/v1/config",
                        json=VALID_SPLIT_CONFIG,
                        headers=headers,
                    )
                )
                self.assert_invalid_host(
                    client.get("/api/v1/diagnostics", headers=headers)
                )
                self.assert_invalid_host(client.get("/api/v1/status", headers={"host": host}))
        self.assertEqual(client.get("/api/v1/status").json()["state"], "idle")
        self.assertEqual(client.get("/api/v1/config").json(), initial_config)

    def test_rebinding_websocket_handshake_is_rejected(self):
        from starlette.websockets import WebSocketDisconnect

        client = self.client()
        host, origin = self.REBOUND
        for headers in ({"host": host, "origin": origin}, {"host": host}):
            with self.subTest(headers=headers):
                with self.assertRaises(WebSocketDisconnect) as raised:
                    with client.websocket_connect("/api/v1/events", headers=headers):
                        pass
                self.assertEqual(raised.exception.code, 1008)

    def test_allowed_host_with_foreign_origin_is_rejected(self):
        from starlette.websockets import WebSocketDisconnect

        client = self.client()
        for origin in (
            "http://evil.example:8766",
            "http://127.0.0.1:9999",
            "https://127.0.0.1:8766",
            "null",
        ):
            with self.subTest(origin=origin):
                response = client.post("/api/v1/actions/check", headers={"origin": origin})
                self.assertEqual(response.status_code, 403, response.text)
                response = client.get("/api/v1/diagnostics", headers={"origin": origin})
                self.assertEqual(response.status_code, 403, response.text)
                with self.assertRaises(WebSocketDisconnect) as raised:
                    with client.websocket_connect(
                        "/api/v1/events", headers={"origin": origin}
                    ):
                        pass
                self.assertEqual(raised.exception.code, 1008)
        self.assertEqual(client.get("/api/v1/status").json()["state"], "idle")

    def test_loopback_origins_and_origin_less_clients_still_work(self):
        client = self.client()
        for host, origin in (
            ("127.0.0.1:8766", "http://127.0.0.1:8766"),
            ("localhost:8766", "http://localhost:8766"),
            ("[::1]:8766", "http://[::1]:8766"),
        ):
            with self.subTest(host=host):
                response = client.put(
                    "/api/v1/config",
                    json=VALID_SPLIT_CONFIG,
                    headers={"host": host, "origin": origin},
                )
                self.assertEqual(response.status_code, 200, response.text)
                with client.websocket_connect(
                    "/api/v1/events", headers={"host": host, "origin": origin}
                ) as websocket:
                    self.assertEqual(websocket.receive_json()["type"], "snapshot")
        response = client.post("/api/v1/actions/check")
        self.assertEqual(response.status_code, 200, response.text)
        self.assertEqual(response.json()["state"], "checking")
        self.assertEqual(client.get("/api/v1/diagnostics").status_code, 200)

    # Non-loopback exposure requires an identity proxy (T2-5).
    PROXY = {"identity_header": "X-Authenticated-User", "trusted_proxies": ("127.0.0.1",)}

    def test_explicit_bind_address_is_allowed_but_wildcard_is_not(self):
        lan = ("192.168.50.20:8766", "http://192.168.50.20:8766")
        client = self.client(bind_host="192.168.50.20", **self.PROXY)
        response = client.post(
            "/api/v1/actions/check", headers={"host": lan[0], "origin": lan[1]}
        )
        self.assertEqual(response.status_code, 200, response.text)

        wildcard = self.client(
            bind_host="0.0.0.0",
            data_root=Path(self.temporary.name) / "wildcard",
            **self.PROXY,
        )
        self.assert_invalid_host(
            wildcard.post(
                "/api/v1/actions/check", headers={"host": lan[0], "origin": lan[1]}
            )
        )

    def test_extra_allowed_host_supports_reverse_proxy(self):
        client = self.client(
            allowed_hosts=["https://console.example", "lab-ws"], **self.PROXY
        )
        proxy = {"host": "console.example", "origin": "https://console.example"}
        # Proxy that forwards the upstream Host still needs the public Origin.
        response = client.put(
            "/api/v1/config",
            json=VALID_SPLIT_CONFIG,
            headers={"origin": "https://console.example"},
        )
        self.assertEqual(response.status_code, 200, response.text)
        response = client.post("/api/v1/actions/check", headers=proxy)
        self.assertEqual(response.status_code, 200, response.text)
        with client.websocket_connect("/api/v1/events", headers=proxy) as websocket:
            self.assertEqual(websocket.receive_json()["type"], "snapshot")
        # A bare name means direct HTTP on the console port.
        response = client.get(
            "/api/v1/status",
            headers={"host": "lab-ws:8766", "origin": "http://lab-ws:8766"},
        )
        self.assertEqual(response.status_code, 200, response.text)
        # Scheme and port are part of the match.
        response = client.post(
            "/api/v1/actions/mocap-accept",
            headers={"host": "console.example", "origin": "http://console.example"},
        )
        self.assertEqual(response.status_code, 403, response.text)
        self.assert_invalid_host(
            client.get("/api/v1/status", headers={"host": "console.example:8443"})
        )

    def test_invalid_allowlist_configuration_fails_closed(self):
        from mocap_g1.operator.app import create_app

        for kwargs in (
            {"allowed_hosts": ["https://console.example/path"]},
            {"allowed_hosts": ["user@console.example"]},
            {"allowed_hosts": ["ftp://console.example"]},
            {"port": 0},
        ):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                create_app(
                    data_root=self.temporary.name, demo=True, **self.PROXY, **kwargs
                )
        with self.assertRaises(TypeError):
            create_app(data_root=self.temporary.name, demo=True, allowed_hosts="lab-ws")

    def test_cli_passes_bind_address_and_repeatable_allowed_hosts(self):
        from mocap_g1.operator import cli as operator_console

        fake_uvicorn = mock.Mock()
        fake_factory = mock.Mock(return_value=object())
        with mock.patch.object(
            operator_console,
            "_load_runtime",
            return_value=(fake_uvicorn, fake_factory),
        ):
            result = operator_console.main(
                [
                    "--data-root", self.temporary.name,
                    "--host", "192.168.50.20",
                    "--port", "9000",
                    "--allowed-host", "https://console.example",
                    "--allowed-host", "lab-ws",
                    "--identity-header", "X-Authenticated-User",
                    "--trusted-proxy", "127.0.0.1",
                ]
            )
            self.assertEqual(result, 0)
            kwargs = fake_factory.call_args.kwargs
            self.assertEqual(kwargs["bind_host"], "192.168.50.20")
            self.assertEqual(kwargs["port"], 9000)
            self.assertEqual(
                kwargs["allowed_hosts"], ("https://console.example", "lab-ws")
            )

            operator_console.main(["--data-root", self.temporary.name])
            self.assertEqual(fake_factory.call_args.kwargs["allowed_hosts"], ())



@unittest.skipUnless(_API_TESTS_AVAILABLE, "FastAPI/httpx test dependencies are not installed")
class AntiFramingHeaderTests(unittest.TestCase):
    """Clickjacking: no page may frame the console (T2-1)."""

    LOCAL = "http://127.0.0.1:8766"

    def setUp(self):
        from fastapi.testclient import TestClient

        from mocap_g1.operator.app import create_app

        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.client = TestClient(
            create_app(data_root=self.temporary.name, demo=True), base_url=self.LOCAL
        )
        self.addCleanup(self.client.close)

    def assert_frame_denied(self, response):
        self.assertEqual(response.headers.get("x-frame-options"), "DENY")
        self.assertEqual(
            response.headers.get_list("x-frame-options"), ["DENY"], response.headers
        )
        csp = response.headers.get("content-security-policy", "")
        self.assertIn("frame-ancestors 'none'", csp)

    def test_static_root_and_api_routes_deny_framing(self):
        for method, path in (
            ("GET", "/"),
            ("GET", "/api/v1/status"),
            ("GET", "/api/v1/config"),
            ("POST", "/api/v1/actions/check"),
        ):
            with self.subTest(path=path):
                response = self.client.request(method, path)
                self.assertEqual(response.status_code, 200, response.text)
                self.assert_frame_denied(response)

    def test_error_and_rejected_host_responses_deny_framing(self):
        conflict = self.client.post("/api/v1/actions/start-tracking")
        self.assertEqual(conflict.status_code, 409, conflict.text)
        self.assert_frame_denied(conflict)
        missing = self.client.get("/api/v1/playback")
        self.assertEqual(missing.status_code, 404)
        self.assert_frame_denied(missing)
        foreign = self.client.post(
            "/api/v1/actions/check",
            headers={"origin": "http://evil.example"},
        )
        self.assertEqual(foreign.status_code, 403)
        self.assert_frame_denied(foreign)
        rebound = self.client.get(
            "/api/v1/status", headers={"host": "evil.example:8766"}
        )
        self.assertEqual(rebound.status_code, 400)
        self.assert_frame_denied(rebound)

    def test_unhandled_exception_500_denies_framing(self):
        # Starlette's outermost ServerErrorMiddleware renders unhandled
        # exceptions; that 500 must carry the anti-framing headers too.
        from fastapi.testclient import TestClient

        from mocap_g1.operator.app import create_app

        app = create_app(data_root=self.temporary.name, demo=True)

        async def explode():
            raise RuntimeError("unhandled failure")

        # Ahead of the static "/" mount, which would otherwise match first.
        app.router.add_api_route("/api/v1/explode", explode)
        app.router.routes.insert(0, app.router.routes.pop())
        client = TestClient(app, base_url=self.LOCAL, raise_server_exceptions=False)
        self.addCleanup(client.close)
        for path in ("/api/v1/explode", "/api/v1/config"):
            with self.subTest(path=path), mock.patch.object(
                app.state.controller,
                "get_config",
                side_effect=RuntimeError("unhandled failure"),
            ):
                response = client.get(path)
                self.assertEqual(response.status_code, 500, response.text)
                self.assert_frame_denied(response)
        rebound = client.get("/api/v1/explode", headers={"host": "evil.example:8766"})
        self.assertEqual(rebound.status_code, 400)
        self.assert_frame_denied(rebound)


if __name__ == "__main__":
    unittest.main()
