from __future__ import annotations

from pathlib import Path
import unittest
from unittest.mock import patch

from mocap_g1.operator import (
    AuditLog,
    CapabilityLock,
    ControlRequest,
    DemoRuntime,
    FaultEvent,
    HealthSnapshot,
    InvalidAction,
    ReadOnlySystemProbe,
    SessionController,
    SessionState,
)


PREFLIGHT = {
    "clear_area": True,
    "harness_or_support": True,
    "stop_drill": True,
}


def reach_tracking(controller):
    for action in ("check", "mocap-accept", "start-simulation", "connect-robot"):
        controller.perform(ControlRequest(action=action))
    controller.perform(
        ControlRequest(
            action="confirm-preflight", spotter="spotter", metadata=PREFLIGHT
        )
    )
    controller.perform(ControlRequest(action="arm"))
    controller.perform(ControlRequest(action="start-tracking"))


class RecordingRuntime(DemoRuntime):
    def __init__(self, *, fail_stop=False, unsafe_stop=False, fail_shutdown=False):
        self.fail_stop = fail_stop
        self.unsafe_stop = unsafe_stop
        self.fail_shutdown = fail_shutdown
        self.safe_stop_calls = 0
        self.shutdown_calls = 0

    def safe_stop(self, health):
        self.safe_stop_calls += 1
        if self.fail_stop:
            raise RuntimeError("supervisor stop transport failed")
        if self.unsafe_stop:
            return {"command_authority": True}
        return super().safe_stop(health)

    def shutdown(self, health):
        self.shutdown_calls += 1
        if self.fail_shutdown:
            raise RuntimeError("supervisor shutdown transport failed")
        return super().shutdown(health)


class RealRecordingRuntime(RecordingRuntime):
    is_simulated = False


class FailingAudit(AuditLog):
    def __init__(self):
        super().__init__()
        self.fail_writes = False

    def append(self, **kwargs):
        if self.fail_writes:
            raise OSError("audit disk full")
        return super().append(**kwargs)


class OperatorRuntimeTests(unittest.TestCase):
    def test_demo_runtime_progresses_health_without_granting_authority(self):
        runtime = DemoRuntime()
        health = HealthSnapshot()
        checked = runtime.check(health, {})
        self.assertTrue(checked["mocap_online"])
        self.assertFalse(checked.get("command_authority", False))
        simulated = runtime.start_simulation(health, {})
        self.assertTrue(simulated["simulation_ready"])
        robot = runtime.connect_robot(health, {})
        self.assertTrue(robot["robot_connected"])
        stopped = runtime.safe_stop(health)
        self.assertFalse(stopped["command_authority"])

    def test_system_probe_is_read_only_and_json_friendly(self):
        repo = Path(__file__).resolve().parents[1]
        result = ReadOnlySystemProbe(repo).snapshot()
        self.assertTrue(result["repo"]["mocap_package_present"])
        self.assertIn("fastapi", result["python_modules"])
        self.assertIn("systemctl", result["commands"])

    def test_new_fault_invokes_safe_stop_before_latching(self):
        runtime = RecordingRuntime()
        controller = SessionController(runtime=runtime, demo=True)
        reach_tracking(controller)
        controller.latch_fault(FaultEvent(code="test_fault", reason="stop now"))
        self.assertEqual(runtime.safe_stop_calls, 1)
        self.assertEqual(controller.state, SessionState.FAULT_LATCHED)
        self.assertFalse(controller.health.command_authority)
        stop_events = [
            event
            for event in controller.events_after()
            if event["action"] == "safe_stop"
        ]
        self.assertEqual(stop_events[-1]["outcome"], "accepted")

    def test_safe_stop_error_is_recorded_and_fault_still_latches_closed(self):
        runtime = RecordingRuntime(fail_stop=True)
        controller = SessionController(runtime=runtime, demo=True)
        reach_tracking(controller)
        controller.latch_fault(FaultEvent(code="dds_timeout", reason="DDS stale"))
        self.assertEqual(runtime.safe_stop_calls, 1)
        self.assertEqual(controller.state, SessionState.FAULT_LATCHED)
        self.assertFalse(controller.health.command_authority)
        stop_events = [
            event
            for event in controller.events_after()
            if event["action"] == "safe_stop"
        ]
        self.assertEqual(stop_events[-1]["outcome"], "rejected")
        self.assertIn("transport failed", stop_events[-1]["details"]["reason"])

    def test_unsafe_safe_stop_report_cannot_restore_authority(self):
        runtime = RecordingRuntime(unsafe_stop=True)
        controller = SessionController(runtime=runtime, demo=True)
        reach_tracking(controller)
        controller.latch_fault(FaultEvent(code="policy_failure", reason="policy failed"))
        self.assertEqual(controller.state, SessionState.FAULT_LATCHED)
        self.assertFalse(controller.health.command_authority)
        stop_events = [
            event
            for event in controller.events_after()
            if event["action"] == "safe_stop"
        ]
        self.assertEqual(stop_events[-1]["outcome"], "rejected")

    def test_disengage_latches_when_safe_stop_does_not_verify_zero_authority(self):
        runtime = RecordingRuntime(unsafe_stop=True)
        controller = SessionController(runtime=runtime, demo=True)
        reach_tracking(controller)

        status = controller.perform(ControlRequest(action="disengage"))

        self.assertEqual(status["state"], "fault_latched")
        self.assertEqual(status["active_fault"]["code"], "disengage_unverified")
        self.assertFalse(status["health"]["command_authority"])

    def test_disengage_latches_when_safe_stop_raises(self):
        runtime = RecordingRuntime(fail_stop=True)
        controller = SessionController(runtime=runtime, demo=True)
        reach_tracking(controller)

        status = controller.perform(ControlRequest(action="disengage"))

        self.assertEqual(status["state"], "fault_latched")
        self.assertEqual(status["active_fault"]["code"], "disengage_runtime_error")
        self.assertFalse(status["health"]["command_authority"])

    def test_shutdown_error_latches_nonrecoverable_fault(self):
        runtime = RecordingRuntime(fail_shutdown=True)
        controller = SessionController(runtime=runtime, demo=True)

        with self.assertRaisesRegex(RuntimeError, "shutdown transport failed"):
            controller.perform(ControlRequest(action="shutdown"))

        status = controller.status_snapshot()
        self.assertEqual(status["state"], "fault_latched")
        self.assertEqual(status["active_fault"]["code"], "shutdown_runtime_error")
        self.assertFalse(status["active_fault"]["recoverable"])
        self.assertFalse(status["health"]["command_authority"])

    def test_shutdown_blocks_any_fault_without_verified_zero_authority(self):
        cases = (
            ({"unsafe_stop": True}, "policy_failure"),
            ({"fail_stop": True}, "dds_timeout"),
        )
        for runtime_options, fault_code in cases:
            with self.subTest(fault_code=fault_code):
                runtime = RecordingRuntime(**runtime_options)
                controller = SessionController(runtime=runtime, demo=True)
                reach_tracking(controller)
                controller.latch_fault(
                    FaultEvent(code=fault_code, reason="runtime fault")
                )

                with self.assertRaises(InvalidAction) as raised:
                    controller.perform(ControlRequest(action="shutdown"))

                self.assertEqual(raised.exception.code, "shutdown_stop_unverified")
                status = controller.status_snapshot()
                self.assertEqual(status["state"], "fault_latched")
                self.assertEqual(status["active_fault"]["code"], fault_code)
                self.assertEqual(runtime.safe_stop_calls, 2)
                self.assertEqual(runtime.shutdown_calls, 0)

    def test_generic_fault_can_shutdown_after_fresh_zero_authority_proof(self):
        runtime = RecordingRuntime(unsafe_stop=True)
        controller = SessionController(runtime=runtime, demo=True)
        reach_tracking(controller)
        controller.latch_fault(
            FaultEvent(code="policy_failure", reason="policy failed")
        )
        runtime.unsafe_stop = False

        status = controller.perform(ControlRequest(action="shutdown"))

        self.assertEqual(status["state"], "idle")
        self.assertIsNone(status["active_fault"])
        self.assertEqual(runtime.safe_stop_calls, 2)
        self.assertEqual(runtime.shutdown_calls, 1)

    def test_fault_disengage_can_supply_stop_proof_before_rearm(self):
        runtime = RecordingRuntime(unsafe_stop=True)
        controller = SessionController(runtime=runtime, demo=True)
        reach_tracking(controller)
        controller.latch_fault(
            FaultEvent(code="policy_failure", reason="policy failed")
        )
        fault_status = controller.status_snapshot()
        self.assertFalse(fault_status["controls"]["rearm"]["enabled"])
        self.assertIn(
            "zero command authority",
            fault_status["controls"]["rearm"]["reason"],
        )

        with self.assertRaises(InvalidAction) as raised:
            controller.perform(ControlRequest(action="rearm"))
        self.assertEqual(raised.exception.code, "stop_unverified")

        runtime.unsafe_stop = False
        stopped = controller.perform(ControlRequest(action="disengage"))
        self.assertEqual(stopped["state"], "fault_latched")
        rearmed = controller.perform(ControlRequest(action="rearm"))
        self.assertEqual(rearmed["state"], "armed_disabled")
        self.assertFalse(rearmed["health"]["command_authority"])

    def test_two_shutdowns_cannot_clear_a_failed_disengage(self):
        cases = (
            ({"unsafe_stop": True}, "disengage_unverified"),
            ({"fail_stop": True}, "disengage_runtime_error"),
        )
        for runtime_options, fault_code in cases:
            with self.subTest(fault_code=fault_code):
                runtime = RecordingRuntime(**runtime_options)
                controller = SessionController(runtime=runtime, demo=True)
                reach_tracking(controller)

                for _ in range(2):
                    with self.assertRaisesRegex(InvalidAction, "zero command authority"):
                        controller.perform(ControlRequest(action="shutdown"))

                    status = controller.status_snapshot()
                    self.assertEqual(status["state"], "fault_latched")
                    self.assertEqual(status["active_fault"]["code"], fault_code)
                    self.assertFalse(status["health"]["command_authority"])

                self.assertEqual(runtime.shutdown_calls, 0)

    def test_shutdown_clears_failed_disengage_after_fresh_stop_verification(self):
        runtime = RecordingRuntime(unsafe_stop=True)
        controller = SessionController(runtime=runtime, demo=True)
        reach_tracking(controller)

        with self.assertRaisesRegex(InvalidAction, "zero command authority"):
            controller.perform(ControlRequest(action="shutdown"))

        runtime.unsafe_stop = False
        status = controller.perform(ControlRequest(action="shutdown"))

        self.assertEqual(status["state"], "idle")
        self.assertIsNone(status["active_fault"])
        self.assertFalse(status["health"]["command_authority"])
        self.assertEqual(runtime.shutdown_calls, 1)

    def test_audit_failure_cannot_prevent_explicit_disengage(self):
        runtime = RecordingRuntime()
        audit = FailingAudit()
        controller = SessionController(runtime=runtime, audit=audit, demo=True)
        reach_tracking(controller)
        audit.fail_writes = True

        status = controller.perform(ControlRequest(action="disengage"))

        self.assertGreaterEqual(runtime.safe_stop_calls, 1)
        self.assertEqual(status["state"], "armed_disabled")
        self.assertFalse(status["health"]["command_authority"])
        self.assertFalse(controller.events_after()[-1]["details"]["audit_persisted"])

    def test_audit_failure_cannot_prevent_fault_stop(self):
        runtime = RecordingRuntime()
        audit = FailingAudit()
        controller = SessionController(runtime=runtime, audit=audit, demo=True)
        reach_tracking(controller)
        audit.fail_writes = True

        controller.latch_fault(FaultEvent(code="watchdog", reason="state stale"))

        self.assertGreaterEqual(runtime.safe_stop_calls, 1)
        self.assertEqual(controller.state, SessionState.FAULT_LATCHED)
        self.assertFalse(controller.health.command_authority)

    def test_watchdog_tick_stops_an_active_session_on_capability_expiry(self):
        runtime = RealRecordingRuntime()
        capability = CapabilityLock(
            whole_body_unlocked=True,
            manifest_verified=True,
            reason="test capability",
            manifest_id="test",
            expires_at="2030-01-01T00:00:00Z",
            capabilities=("whole_body_hardware",),
        )
        before_expiry = 1_800_000_000.0
        after_expiry = 1_900_000_000.0
        with patch("mocap_g1.operator.core.time.time", return_value=before_expiry):
            controller = SessionController(
                runtime=runtime, capability_lock=capability, demo=False
            )
            reach_tracking(controller)
        with patch("mocap_g1.operator.core.time.time", return_value=after_expiry):
            controller.watchdog_tick()

        # Do not call status_snapshot here. It also enforces capability expiry,
        # which would mask a no-op watchdog implementation.
        self.assertEqual(controller.state, SessionState.FAULT_LATCHED)
        self.assertEqual(controller._active_fault.code, "capability_expired")
        self.assertFalse(controller.health.command_authority)
        self.assertFalse(controller.capability_lock.whole_body_unlocked)
        self.assertGreaterEqual(runtime.safe_stop_calls, 1)


if __name__ == "__main__":
    unittest.main()
