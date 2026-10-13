from __future__ import annotations

import json
from pathlib import Path
import tempfile
import threading
import unittest

from mocap_g1.operator import (
    CapabilityLock,
    ControlRequest,
    FaultEvent,
    HealthSnapshot,
    InvalidAction,
    SessionController,
    SessionState,
    Topology,
)
from mocap_g1.operator.core import (
    CapabilityLocked,
    InvalidConfiguration,
    validate_config,
)


PREFLIGHT = {
    "clear_area": True,
    "harness_or_support": True,
    "stop_drill": True,
}


def request(action: str, **kwargs):
    return ControlRequest(action=action, **kwargs)


def reach_tracking(controller: SessionController) -> None:
    for action in ("check", "mocap-accept", "start-simulation", "connect-robot"):
        controller.perform(request(action))
    controller.perform(
        request("confirm-preflight", operator="operator-a", spotter="spotter-b", metadata=PREFLIGHT)
    )
    controller.perform(request("arm", operator="operator-a"))
    controller.perform(request("start-tracking", operator="operator-a"))


class OperatorModelTests(unittest.TestCase):
    def test_public_enums_have_stable_json_values(self):
        self.assertEqual(Topology.ALL_IN_ONE.value, "all_in_one")
        self.assertEqual(Topology.SPLIT.value, "split")
        self.assertEqual(SessionState.FAULT_LATCHED.value, "fault_latched")

    def test_dataclasses_are_json_serializable(self):
        values = [
            CapabilityLock(),
            HealthSnapshot(messages=("ready",)),
            FaultEvent(code="test", reason="test fault"),
            ControlRequest(action="check", metadata={"nested": (1, 2)}),
        ]
        for value in values:
            json.dumps(value.to_dict())

    def test_health_rejects_nonfinite_or_negative_metrics(self):
        with self.assertRaisesRegex(ValueError, "mocap_rate_hz"):
            HealthSnapshot(mocap_rate_hz=float("nan"))
        with self.assertRaisesRegex(ValueError, "packet_loss_percent"):
            HealthSnapshot(packet_loss_percent=-1.0)

    def test_control_request_requires_action_and_operator(self):
        with self.assertRaisesRegex(ValueError, "action"):
            ControlRequest(action="")
        with self.assertRaisesRegex(ValueError, "operator"):
            ControlRequest(action="check", operator="")


class OperatorSessionTests(unittest.TestCase):
    def test_demo_follows_complete_approved_transition_chain(self):
        controller = SessionController(demo=True)
        expected = [
            ("check", SessionState.CHECKING),
            ("mocap-accept", SessionState.MOCAP_ACCEPTED),
            ("start-simulation", SessionState.SIM_VALIDATED),
            ("connect-robot", SessionState.ROBOT_CONNECTED),
        ]
        for action, state in expected:
            status = controller.perform(request(action))
            self.assertEqual(status["state"], state.value)
        status = controller.perform(
            request("confirm-preflight", spotter="spotter", metadata=PREFLIGHT)
        )
        self.assertEqual(status["state"], SessionState.PREFLIGHT_PASSED.value)
        self.assertEqual(controller.perform(request("arm"))["state"], "armed_disabled")
        status = controller.perform(request("start-tracking"))
        self.assertEqual(status["state"], "tracking")
        self.assertTrue(status["health"]["command_authority"])

    def test_out_of_order_action_is_rejected_and_audited(self):
        controller = SessionController(demo=True)
        with self.assertRaisesRegex(InvalidAction, "requires state"):
            controller.perform(request("start-tracking", operator="alice"))
        records = controller.audit.read()
        self.assertEqual(records[-1]["outcome"], "rejected")
        self.assertEqual(records[-1]["actor"], "alice")

    def test_mocap_acceptance_thresholds_are_enforced(self):
        controller = SessionController(demo=False)
        controller.perform(request("check"))
        controller.update_health(
            mocap_online=True,
            mocap_rate_hz=49.9,
            packet_loss_percent=0.0,
            source_age_ms=1.0,
        )
        with self.assertRaisesRegex(InvalidAction, "below 50"):
            controller.perform(request("mocap-accept"))
        controller.update_health(mocap_rate_hz=60.0, packet_loss_percent=1.1)
        with self.assertRaisesRegex(InvalidAction, "exceeds 1"):
            controller.perform(request("mocap-accept"))
        controller.update_health(packet_loss_percent=0.5, source_age_ms=101.0)
        with self.assertRaisesRegex(InvalidAction, "stale"):
            controller.perform(request("mocap-accept"))
        controller.update_health(source_age_ms=10.0)
        controller.perform(request("mocap-accept"))
        self.assertEqual(controller.state, SessionState.MOCAP_ACCEPTED)

    def test_preflight_requires_spotter_and_every_confirmation(self):
        controller = SessionController(demo=True)
        for action in ("check", "mocap-accept", "start-simulation", "connect-robot"):
            controller.perform(request(action))
        with self.assertRaisesRegex(InvalidAction, "spotter"):
            controller.perform(request("confirm-preflight", metadata=PREFLIGHT))
        with self.assertRaisesRegex(InvalidAction, "stop_drill"):
            controller.perform(
                request(
                    "confirm-preflight",
                    spotter="spotter",
                    metadata={"clear_area": True, "harness_or_support": True},
                )
            )
        controller.perform(
            request("confirm-preflight", spotter="spotter", metadata=PREFLIGHT)
        )
        self.assertTrue(controller.health.preflight_confirmed)

    def test_default_real_hardware_connection_is_capability_locked(self):
        controller = SessionController(demo=False)
        controller.perform(request("check"))
        controller.update_health(
            mocap_online=True,
            mocap_rate_hz=60.0,
            packet_loss_percent=0.0,
            source_age_ms=2.0,
            simulation_ready=True,
            robot_connected=True,
            robot_state_fresh=True,
            controller_ready=True,
        )
        controller.perform(request("mocap-accept"))
        controller.perform(request("start-simulation"))
        with self.assertRaises(CapabilityLocked):
            controller.perform(request("connect-robot"))
        status = controller.status_snapshot()
        self.assertEqual(status["state"], "sim_validated")
        self.assertFalse(status["controls"]["connect-robot"]["enabled"])

    def test_disengage_is_confirmation_free_and_idempotent(self):
        controller = SessionController(demo=True)
        reach_tracking(controller)
        status = controller.perform(request("disengage"))
        self.assertEqual(status["state"], "armed_disabled")
        self.assertFalse(status["health"]["command_authority"])
        status = controller.perform(request("disengage"))
        self.assertEqual(status["state"], "armed_disabled")
        actions = [event["action"] for event in controller.events_after()]
        self.assertIn("disengaging", actions)

    def test_tracking_health_loss_latches_and_never_auto_resumes(self):
        controller = SessionController(demo=True)
        reach_tracking(controller)
        controller.update_health(source_age_ms=250.0)
        status = controller.status_snapshot()
        self.assertEqual(status["state"], "fault_latched")
        self.assertFalse(status["health"]["command_authority"])
        self.assertEqual(status["active_fault"]["code"], "tracking_health_lost")
        controller.update_health(source_age_ms=2.0)
        self.assertEqual(controller.state, SessionState.FAULT_LATCHED)
        status = controller.perform(request("rearm"))
        self.assertEqual(status["state"], "armed_disabled")
        self.assertFalse(status["health"]["command_authority"])

    def test_real_fault_requires_runtime_clear_and_fresh_health(self):
        unlocked = CapabilityLock(
            whole_body_unlocked=True,
            manifest_verified=True,
            reason="test qualification",
            expires_at="2099-01-01T00:00:00Z",
        )
        controller = SessionController(demo=False, capability_lock=unlocked)
        controller.perform(request("check"))
        controller.update_health(
            mocap_online=True,
            mocap_accepted=False,
            mocap_rate_hz=60.0,
            packet_loss_percent=0.0,
            source_age_ms=2.0,
            simulation_ready=True,
            robot_connected=True,
            robot_state_fresh=True,
            controller_ready=True,
        )
        controller.perform(request("mocap-accept"))
        controller.perform(request("start-simulation"))
        controller.perform(request("connect-robot"))
        controller.perform(
            request("confirm-preflight", spotter="spotter", metadata=PREFLIGHT)
        )
        controller.perform(request("arm"))
        controller.perform(request("start-tracking"))
        controller.latch_fault(FaultEvent(code="dds_timeout", reason="DDS state stale"))
        with self.assertRaisesRegex(InvalidAction, "not cleared"):
            controller.perform(request("rearm"))
        controller.clear_fault_condition("dds_timeout")
        status = controller.perform(request("rearm"))
        self.assertEqual(status["state"], "armed_disabled")

    def test_nonrecoverable_fault_cannot_rearm(self):
        controller = SessionController(demo=True)
        reach_tracking(controller)
        controller.latch_fault(
            FaultEvent(code="hardware_damage", reason="inspection required", recoverable=False)
        )
        with self.assertRaisesRegex(InvalidAction, "not recoverable"):
            controller.perform(request("rearm"))

    def test_shutdown_clears_authority_and_returns_to_idle(self):
        controller = SessionController(demo=True)
        reach_tracking(controller)
        status = controller.perform(request("shutdown"))
        self.assertEqual(status["state"], "idle")
        self.assertFalse(status["health"]["command_authority"])
        self.assertEqual(status["health"]["overall"], "offline")

    def test_status_and_events_are_json_serializable_and_sequenced(self):
        controller = SessionController(demo=True)
        controller.perform(request("check", operator="engineer-a"))
        json.dumps(controller.status_snapshot())
        events = controller.events_after()
        json.dumps(events)
        self.assertEqual(
            [event["sequence"] for event in events],
            list(range(1, len(events) + 1)),
        )
        self.assertEqual(controller.events_after(events[-1]["sequence"]), [])
        control_event = next(
            event for event in events
            if event["type"] == "control" and event["action"] == "check"
        )
        self.assertEqual(control_event["actor"], "engineer-a")

    def test_concurrent_disengage_calls_remain_safe(self):
        controller = SessionController(demo=True)
        reach_tracking(controller)
        errors = []

        def stop():
            try:
                controller.perform(request("disengage"))
            except Exception as exc:  # pragma: no cover - asserted below
                errors.append(exc)

        threads = [threading.Thread(target=stop) for _ in range(12)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        self.assertEqual(errors, [])
        self.assertEqual(controller.state, SessionState.ARMED_DISABLED)
        self.assertFalse(controller.health.command_authority)


class OperatorConfigTests(unittest.TestCase):
    def test_setup_required_is_cleared_by_valid_configuration(self):
        controller = SessionController(demo=True, setup_complete=False)
        self.assertEqual(controller.state, SessionState.SETUP_REQUIRED)
        controller.update_config({"mocap": {"port": 5556}})
        self.assertEqual(controller.state, SessionState.IDLE)

    def test_split_topology_requires_source_ip(self):
        controller = SessionController(demo=True)
        with self.assertRaisesRegex(InvalidConfiguration, "source_ip"):
            controller.update_config({"topology": "split"})
        config = controller.update_config(
            {"topology": "split", "mocap": {"source_ip": "192.168.50.10"}}
        )
        self.assertEqual(config["topology"], "split")

    def test_configuration_is_locked_while_armed_and_fault_latched(self):
        controller = SessionController(demo=True)
        reach_tracking(controller)
        controller.perform(request("disengage"))
        with self.assertRaisesRegex(InvalidConfiguration, "cannot change"):
            controller.update_config({"mocap": {"port": 5556}})
        controller.perform(request("start-tracking"))
        controller.latch_fault(FaultEvent(code="test", reason="test"))
        with self.assertRaisesRegex(InvalidConfiguration, "cannot change"):
            controller.update_config({"mocap": {"port": 5556}})

    def test_configuration_is_locked_once_checks_begin(self):
        controller = SessionController(demo=True)
        controller.perform(request("check"))
        with self.assertRaisesRegex(InvalidConfiguration, "only before session checks"):
            controller.update_config({"mocap": {"port": 5556}})

    def test_runtime_mode_is_launch_only_and_ignores_persisted_fake_mode(self):
        controller = SessionController(config={"runtime_mode": "fake"}, demo=False)
        self.assertEqual(controller.get_config()["runtime_mode"], "real")
        self.assertEqual(controller.status_snapshot()["runtime_mode"], "real")
        with self.assertRaisesRegex(InvalidConfiguration, "service launcher"):
            controller.update_config({"runtime_mode": "fake"})

    def test_valid_configuration_is_atomically_persisted(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            controller = SessionController(demo=True, config_path=path)
            saved = controller.update_config({"mocap": {"port": 6000}})
            self.assertEqual(json.loads(path.read_text(encoding="utf-8")), saved)
            self.assertFalse(path.with_name(".config.json.tmp").exists())

    def test_raw_mocap_opt_in_is_never_inherited_by_a_new_session(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            controller = SessionController(
                demo=True,
                config={"privacy": {"record_raw_mocap": True}},
                config_path=path,
            )
            self.assertFalse(controller.get_config()["privacy"]["record_raw_mocap"])
            self.assertFalse(
                json.loads(path.read_text(encoding="utf-8"))["privacy"][
                    "record_raw_mocap"
                ]
            )

    def test_raw_mocap_can_be_explicitly_enabled_for_current_session(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            controller = SessionController(demo=True, config_path=path)
            config = controller.update_config(
                {"privacy": {"record_raw_mocap": True}}, actor="operator-a"
            )
            self.assertTrue(config["privacy"]["record_raw_mocap"])
            self.assertTrue(
                json.loads(path.read_text(encoding="utf-8"))["privacy"][
                    "record_raw_mocap"
                ]
            )

    def test_shutdown_atomically_resets_raw_mocap_opt_in(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            controller = SessionController(demo=True, config_path=path)
            controller.update_config({"privacy": {"record_raw_mocap": True}})
            status = controller.perform(request("shutdown"))
            self.assertEqual(status["state"], "idle")
            self.assertFalse(controller.get_config()["privacy"]["record_raw_mocap"])
            self.assertFalse(
                json.loads(path.read_text(encoding="utf-8"))["privacy"][
                    "record_raw_mocap"
                ]
            )
            self.assertFalse(path.with_name(".config.json.tmp").exists())

    def test_unknown_and_invalid_configuration_values_are_rejected(self):
        with self.assertRaisesRegex(InvalidConfiguration, "unknown"):
            validate_config({"surprise": True})
        controller = SessionController(demo=True)
        with self.assertRaisesRegex(InvalidConfiguration, "between 1 and 65535"):
            controller.update_config({"mocap": {"port": 70000}})


if __name__ == "__main__":
    unittest.main()
