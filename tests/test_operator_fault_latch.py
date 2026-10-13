"""Regression tests for non-recoverable fault latching and audit-gated authority."""

from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

from mocap_g1.operator import (
    AuditLog,
    ControlRequest,
    DemoRuntime,
    FaultEvent,
    InvalidAction,
    SessionController,
    SessionState,
)


PREFLIGHT = {
    "clear_area": True,
    "harness_or_support": True,
    "stop_drill": True,
}
SESSION_ACTIONS = (
    "check",
    "mocap-accept",
    "start-simulation",
    "connect-robot",
    "confirm-preflight",
    "arm",
    "start-tracking",
    "rearm",
)


def request(action: str, **kwargs) -> ControlRequest:
    return ControlRequest(action=action, **kwargs)


def reach_armed(controller: SessionController) -> None:
    for action in ("check", "mocap-accept", "start-simulation", "connect-robot"):
        controller.perform(request(action))
    controller.perform(
        request("confirm-preflight", spotter="spotter", metadata=PREFLIGHT)
    )
    controller.perform(request("arm"))


class FailingAudit(AuditLog):
    def __init__(self) -> None:
        super().__init__()
        self.fail_writes = False

    def append(self, **kwargs):
        if self.fail_writes:
            raise OSError("audit disk full")
        return super().append(**kwargs)


class TrackingRuntime(DemoRuntime):
    def __init__(self) -> None:
        self.tracking_calls: list[bool] = []
        self.safe_stop_calls = 0
        self.shutdown_calls = 0

    def set_tracking(self, enabled: bool) -> None:
        self.tracking_calls.append(bool(enabled))

    def safe_stop(self, health):
        self.safe_stop_calls += 1
        return super().safe_stop(health)

    def shutdown(self, health):
        self.shutdown_calls += 1
        return super().shutdown(health)


class NonRecoverableFaultLatchTests(unittest.TestCase):
    def _latched_controller(self, **kwargs) -> SessionController:
        controller = SessionController(demo=True, **kwargs)
        reach_armed(controller)
        controller.perform(request("start-tracking"))
        controller.latch_fault(
            FaultEvent(
                code="hardware_damage",
                reason="inspection required",
                recoverable=False,
            )
        )
        return controller

    def test_shutdown_retains_nonrecoverable_fault(self):
        runtime = TrackingRuntime()
        controller = self._latched_controller(runtime=runtime)

        status = controller.perform(request("shutdown"))

        self.assertEqual(runtime.shutdown_calls, 1)
        self.assertEqual(status["state"], "fault_latched")
        self.assertEqual(status["active_fault"]["code"], "hardware_damage")
        self.assertFalse(status["active_fault"]["recoverable"])
        self.assertFalse(status["health"]["command_authority"])
        shutdown_event = controller.audit.read()[-1]
        self.assertEqual(shutdown_event["action"], "shutdown")
        self.assertEqual(shutdown_event["outcome"], "accepted")
        self.assertTrue(shutdown_event["details"]["restart_required"])

    def test_session_actions_rejected_until_restart(self):
        controller = self._latched_controller()
        controller.perform(request("shutdown"))

        for action in SESSION_ACTIONS:
            with self.subTest(action=action):
                with self.assertRaises(InvalidAction) as raised:
                    controller.perform(
                        request(action, spotter="spotter", metadata=PREFLIGHT)
                    )
                self.assertEqual(raised.exception.code, "fault_requires_restart")
                self.assertEqual(controller.state, SessionState.FAULT_LATCHED)
        rejected = controller.audit.read()[-1]
        self.assertEqual(rejected["outcome"], "rejected")
        self.assertEqual(rejected["details"]["code"], "fault_requires_restart")

        # Stop and shutdown stay available and never clear the latch.
        self.assertEqual(
            controller.perform(request("disengage"))["state"], "fault_latched"
        )
        self.assertEqual(
            controller.perform(request("shutdown"))["state"], "fault_latched"
        )
        self.assertFalse(controller.health.command_authority)

    def test_rearm_reports_restart_required_before_shutdown(self):
        controller = self._latched_controller()
        with self.assertRaises(InvalidAction) as raised:
            controller.perform(request("rearm"))
        self.assertEqual(raised.exception.code, "fault_requires_restart")

    def test_controls_disable_every_session_action(self):
        controller = self._latched_controller()
        for status in (
            controller.status_snapshot(),
            controller.perform(request("shutdown")),
        ):
            controls = status["controls"]
            for action in SESSION_ACTIONS:
                with self.subTest(action=action):
                    self.assertFalse(controls[action]["enabled"])
                    self.assertIn("restart", controls[action]["reason"].lower())
            self.assertTrue(controls["disengage"]["enabled"])
            self.assertTrue(controls["shutdown"]["enabled"])
            self.assertFalse(controls["edit-config"]["enabled"])

    def test_shutdown_still_resets_raw_mocap_consent(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            controller = SessionController(demo=True, config_path=path)
            controller.update_config({"privacy": {"record_raw_mocap": True}})
            reach_armed(controller)
            controller.latch_fault(
                FaultEvent(code="hardware_damage", reason="x", recoverable=False)
            )
            status = controller.perform(request("shutdown"))
            self.assertEqual(status["state"], "fault_latched")
            self.assertFalse(controller.get_config()["privacy"]["record_raw_mocap"])
            self.assertFalse(
                json.loads(path.read_text(encoding="utf-8"))["privacy"][
                    "record_raw_mocap"
                ]
            )

    def test_nonrecoverable_fault_escalates_a_latched_recoverable_fault(self):
        controller = SessionController(demo=True)
        reach_armed(controller)
        controller.perform(request("start-tracking"))
        controller.latch_fault(FaultEvent(code="dds_timeout", reason="stale"))
        controller.latch_fault(
            FaultEvent(code="hardware_damage", reason="inspect", recoverable=False)
        )
        status = controller.status_snapshot()
        self.assertEqual(status["active_fault"]["code"], "hardware_damage")
        with self.assertRaises(InvalidAction) as raised:
            controller.perform(request("rearm"))
        self.assertEqual(raised.exception.code, "fault_requires_restart")
        self.assertEqual(
            controller.perform(request("shutdown"))["state"], "fault_latched"
        )

    def test_recoverable_fault_shutdown_still_returns_to_idle(self):
        controller = SessionController(demo=True)
        reach_armed(controller)
        controller.perform(request("start-tracking"))
        controller.latch_fault(FaultEvent(code="dds_timeout", reason="stale"))
        status = controller.perform(request("shutdown"))
        self.assertEqual(status["state"], "idle")
        self.assertIsNone(status["active_fault"])
        self.assertTrue(status["controls"]["check"]["enabled"])


class AuditGatedAuthorityTests(unittest.TestCase):
    def test_start_tracking_audit_failure_revokes_authority(self):
        runtime = TrackingRuntime()
        audit = FailingAudit()
        controller = SessionController(runtime=runtime, audit=audit, demo=True)
        reach_armed(controller)
        audit.fail_writes = True

        with self.assertRaisesRegex(OSError, "audit disk full"):
            controller.perform(request("start-tracking"))

        status = controller.status_snapshot()
        self.assertEqual(status["state"], "fault_latched")
        self.assertEqual(status["active_fault"]["code"], "audit_write_failed")
        self.assertFalse(status["health"]["command_authority"])
        self.assertEqual(runtime.tracking_calls[-1], False)
        self.assertGreaterEqual(runtime.safe_stop_calls, 1)
        self.assertFalse(controller.events_after()[-1]["details"]["audit_persisted"])

    def test_start_tracking_audit_failure_does_not_auto_resume(self):
        audit = FailingAudit()
        controller = SessionController(audit=audit, demo=True)
        reach_armed(controller)
        audit.fail_writes = True
        with self.assertRaises(OSError):
            controller.perform(request("start-tracking"))
        audit.fail_writes = False

        status = controller.perform(request("rearm"))
        self.assertEqual(status["state"], "armed_disabled")
        self.assertFalse(status["health"]["command_authority"])
        status = controller.perform(request("start-tracking"))
        self.assertTrue(status["health"]["command_authority"])

    def test_rearm_audit_failure_keeps_fault_latched(self):
        audit = FailingAudit()
        controller = SessionController(audit=audit, demo=True)
        reach_armed(controller)
        controller.perform(request("start-tracking"))
        controller.latch_fault(FaultEvent(code="dds_timeout", reason="stale"))
        audit.fail_writes = True

        with self.assertRaises(OSError):
            controller.perform(request("rearm"))

        status = controller.status_snapshot()
        self.assertEqual(status["state"], "fault_latched")
        self.assertEqual(status["active_fault"]["code"], "dds_timeout")
        self.assertFalse(status["health"]["command_authority"])


if __name__ == "__main__":
    unittest.main()
