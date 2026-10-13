"""T2-4: audit durability (fsync on the strict path) and hash-chain evidence."""

from __future__ import annotations

from importlib.util import find_spec
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from mocap_g1.operator import AuditLog, ControlRequest, SessionController
from mocap_g1.operator.audit import GENESIS_SHA256

_API_TESTS_AVAILABLE = find_spec("fastapi") is not None and find_spec("httpx") is not None


def write_records(audit: AuditLog, count: int) -> None:
    for index in range(count):
        audit.append(
            category="control",
            action=f"action-{index}",
            outcome="accepted",
            state="idle",
            details={"index": index, "token": "secret-value"},
        )


class AuditDurabilityTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.path = Path(self.temporary.name) / "audit.jsonl"

    def test_strict_append_fsyncs_and_best_effort_does_not(self):
        audit = AuditLog(self.path)
        with mock.patch("mocap_g1.operator.audit.os.fsync") as fsync:
            audit.append(category="c", action="a", outcome="accepted", state="idle")
            self.assertGreaterEqual(fsync.call_count, 1)
            fsync.reset_mock()
            audit.append(
                category="c", action="b", outcome="accepted", state="idle", durable=False
            )
            fsync.assert_not_called()

    def test_controller_strict_events_fsync_and_safety_events_do_not(self):
        audit = AuditLog(self.path)
        controller = SessionController(audit=audit, demo=True)
        with mock.patch("mocap_g1.operator.audit.os.fsync") as fsync:
            controller.perform(ControlRequest(action="check"))
            self.assertGreaterEqual(fsync.call_count, 1)
            fsync.reset_mock()
            # Disengage from a safe state is recorded best-effort: a slow disk
            # must never delay withdrawing authority.
            controller.perform(ControlRequest(action="disengage"))
            fsync.assert_not_called()


class AuditChainTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.path = Path(self.temporary.name) / "audit.jsonl"

    def lines(self) -> list[bytes]:
        return self.path.read_bytes().splitlines()

    def test_chain_links_each_written_line(self):
        write_records(AuditLog(self.path), 4)
        lines = self.lines()
        records = [json.loads(line) for line in lines]
        self.assertEqual(records[0]["prev_sha256"], GENESIS_SHA256)
        for previous, record in zip(lines, records[1:]):
            # The hash covers the redacted line exactly as written.
            self.assertEqual(record["prev_sha256"], hashlib.sha256(previous).hexdigest())
        self.assertNotIn(b"secret-value", self.path.read_bytes())

        reloaded = AuditLog(self.path)
        summary = reloaded.integrity_summary()
        self.assertTrue(summary["chain_verified"])
        self.assertEqual(summary["chain_breaks"], 0)
        self.assertEqual(summary["chained_records"], 4)
        report = reloaded.verify_chain()
        self.assertTrue(report["chain_verified"])
        self.assertTrue(report["tail_matches_last_write"])
        # Appending after reload continues the same chain.
        write_records(reloaded, 1)
        self.assertTrue(AuditLog(self.path).integrity_summary()["chain_verified"])

    def assert_break_detected(self, mutate) -> None:
        write_records(AuditLog(self.path), 5)
        lines = self.lines()
        mutate(lines)
        self.path.write_bytes(b"\n".join(lines) + b"\n")
        with self.assertLogs("mocap_g1.operator.audit", level="ERROR"):
            audit = AuditLog(self.path)
        summary = audit.integrity_summary()
        self.assertFalse(summary["chain_verified"])
        self.assertGreaterEqual(summary["chain_breaks"], 1)
        # The finding is appended as a chained audit record and is visible in
        # the ordinary audit view; loading never raises.
        finding = audit.read(newest=True, limit=1)[0]
        self.assertEqual((finding["category"], finding["action"]), ("audit", "integrity_check"))
        self.assertEqual(finding["outcome"], "rejected")
        self.assertGreaterEqual(finding["details"]["chain_breaks"], 1)

    def test_tampered_line_is_detected(self):
        def tamper(lines):
            record = json.loads(lines[1])
            record["actor"] = "someone-else"
            lines[1] = json.dumps(record, sort_keys=True, separators=(",", ":")).encode()

        self.assert_break_detected(tamper)

    def test_deleted_line_is_detected(self):
        self.assert_break_detected(lambda lines: lines.pop(2))

    def test_reordered_lines_are_detected(self):
        def swap(lines):
            lines[1], lines[2] = lines[2], lines[1]

        self.assert_break_detected(swap)

    def test_runtime_truncation_is_detected_by_verify_chain(self):
        audit = AuditLog(self.path)
        write_records(audit, 3)
        lines = self.lines()
        self.path.write_bytes(b"\n".join(lines[:-1]) + b"\n")
        report = audit.verify_chain()
        self.assertFalse(report["chain_verified"])
        self.assertFalse(report["tail_matches_last_write"])
        self.assertFalse(audit.integrity_summary()["chain_verified"])

    def test_legacy_unchained_file_loads_and_chain_starts_fresh(self):
        legacy = [
            {"sequence": 1, "timestamp": 1.0, "category": "control", "action": "check",
             "outcome": "accepted", "state": "idle", "actor": "x", "details": {}},
            {"sequence": 2, "timestamp": 2.0, "category": "control", "action": "arm",
             "outcome": "accepted", "state": "idle", "actor": "x", "details": {}},
        ]
        original = "".join(json.dumps(record) + "\n" for record in legacy).encode()
        self.path.write_bytes(original)

        audit = AuditLog(self.path)
        summary = audit.integrity_summary()
        self.assertTrue(summary["chain_verified"])
        self.assertEqual(summary["unchained_records"], 2)
        self.assertEqual([record["sequence"] for record in audit.read()], [1, 2])
        # Loading alone never modifies the legacy file.
        self.assertEqual(self.path.read_bytes(), original)

        write_records(audit, 1)
        records = audit.read()
        self.assertEqual(records[2]["action"], "chain_start")
        self.assertEqual(records[2]["details"]["unchained_records"], 2)
        lines = self.lines()
        self.assertEqual(
            json.loads(lines[2])["prev_sha256"], hashlib.sha256(lines[1]).hexdigest()
        )
        reloaded = AuditLog(self.path).integrity_summary()
        self.assertTrue(reloaded["chain_verified"])
        self.assertEqual(reloaded["unchained_records"], 2)
        self.assertEqual(reloaded["chained_records"], 2)

    def test_torn_final_line_does_not_swallow_the_next_record(self):
        write_records(AuditLog(self.path), 2)
        with self.path.open("ab") as handle:
            handle.write(b'{"sequence":3,"trunc')
        audit = AuditLog(self.path)
        write_records(audit, 1)
        lines = self.lines()
        self.assertEqual(lines[2], b'{"sequence":3,"trunc')
        appended = json.loads(lines[3])
        self.assertEqual(appended["prev_sha256"], hashlib.sha256(lines[2]).hexdigest())
        self.assertTrue(AuditLog(self.path).integrity_summary()["chain_verified"])

    def test_in_memory_log_reports_non_persistent(self):
        summary = AuditLog().integrity_summary()
        self.assertFalse(summary["persistent"])
        self.assertTrue(summary["chain_verified"])


@unittest.skipUnless(_API_TESTS_AVAILABLE, "FastAPI/httpx test dependencies are not installed")
class AuditIntegrityApiTests(unittest.TestCase):
    def test_status_and_diagnostics_surface_a_chain_break(self):
        from fastapi.testclient import TestClient

        from mocap_g1.operator.app import create_app

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "audit.jsonl"
            write_records(AuditLog(path), 3)
            lines = path.read_bytes().splitlines()
            del lines[1]
            path.write_bytes(b"\n".join(lines) + b"\n")
            with self.assertLogs("mocap_g1.operator.audit", level="ERROR"):
                app = create_app(data_root=directory, demo=True)
            with TestClient(app, base_url="http://127.0.0.1:8766") as client:
                status = client.get("/api/v1/status").json()
                self.assertFalse(status["audit_integrity"]["chain_verified"])
                self.assertEqual(status["state"], "idle")
                diagnostics = json.loads(client.get("/api/v1/diagnostics").content)
        self.assertFalse(diagnostics["audit_integrity"]["chain_verified"])
        self.assertTrue(diagnostics["audit_integrity"]["breaks"])


if __name__ == "__main__":
    unittest.main()
