"""T2-6 lower-severity operator items and the remaining T2-7 test gaps."""

from __future__ import annotations

from importlib.util import find_spec
import json
from pathlib import Path
import sys
import tempfile
import unittest

from mocap_g1.operator import ControlRequest, DemoRuntime, SessionController
from mocap_g1.operator.core import ACTIONS, HEALTH_AUDIT_INTERVAL_S, OperatorError
from mocap_g1.operator.runtime import ReadOnlySystemProbe

_API_TESTS_AVAILABLE = find_spec("fastapi") is not None and find_spec("httpx") is not None
PREFLIGHT = {"clear_area": True, "harness_or_support": True, "stop_drill": True}


def request(action: str, **kwargs) -> ControlRequest:
    return ControlRequest(action=action, **kwargs)


def reach_armed(controller: SessionController) -> None:
    for action in ("check", "mocap-accept", "start-simulation", "connect-robot"):
        controller.perform(request(action))
    controller.perform(request("confirm-preflight", spotter="spotter", metadata=PREFLIGHT))
    controller.perform(request("arm"))


class FakeClock:
    def __init__(self) -> None:
        self.now = 1000.0

    def __call__(self) -> float:
        return self.now


class RaisingCheckRuntime(DemoRuntime):
    def check(self, health, config):
        raise RuntimeError("supervisor unreachable")


class FlakyTickRuntime(DemoRuntime):
    def __init__(self) -> None:
        self.fail = False

    def tick(self, health):
        if self.fail:
            raise RuntimeError("tick exploded")
        return super().tick(health)


class CheckTransitionTests(unittest.TestCase):
    def test_raising_check_stays_idle_and_is_audited(self):
        controller = SessionController(runtime=RaisingCheckRuntime(), demo=True)
        with self.assertRaises(OperatorError) as raised:
            controller.perform(request("check"))
        self.assertEqual(raised.exception.code, "runtime_error")
        self.assertEqual(raised.exception.status_code, 409)
        self.assertEqual(controller.state.value, "idle")
        event = controller.events_after()[-1]
        self.assertEqual((event["action"], event["outcome"]), ("check", "rejected"))
        self.assertIn("supervisor unreachable", event["details"]["reason"])

    def test_unexpected_adapter_error_is_still_audited(self):
        class RaisingSimulation(DemoRuntime):
            def start_simulation(self, health, config):
                raise ValueError("bad npz")

        controller = SessionController(runtime=RaisingSimulation(), demo=True)
        controller.perform(request("check"))
        controller.perform(request("mocap-accept"))
        with self.assertRaises(ValueError):
            controller.perform(request("start-simulation"))
        event = controller.events_after()[-1]
        self.assertEqual((event["action"], event["outcome"]), ("start-simulation", "rejected"))
        self.assertEqual(event["details"]["error_type"], "ValueError")
        self.assertEqual(controller.state.value, "mocap_accepted")


class TickErrorOutsideTrackingTests(unittest.TestCase):
    def test_tick_error_is_recorded_once_and_gates_start_tracking(self):
        runtime = FlakyTickRuntime()
        controller = SessionController(runtime=runtime, demo=True)
        reach_armed(controller)
        runtime.fail = True
        for _ in range(5):
            controller.watchdog_tick()
        status = controller.status_snapshot()
        self.assertEqual(status["state"], "armed_disabled")
        self.assertEqual(status["health"]["overall"], "fault")
        self.assertFalse(status["controls"]["start-tracking"]["enabled"])
        ticks = [e for e in controller.events_after() if e["action"] == "watchdog_tick"]
        self.assertEqual(len(ticks), 1)
        self.assertEqual(ticks[0]["outcome"], "rejected")
        with self.assertRaises(OperatorError) as raised:
            controller.perform(request("start-tracking"))
        self.assertEqual(raised.exception.code, "tracking_not_ready")

        runtime.fail = False
        controller.watchdog_tick()
        status = controller.status_snapshot()
        self.assertNotEqual(status["health"]["overall"], "fault")
        self.assertTrue(status["controls"]["start-tracking"]["enabled"])
        ticks = [e for e in controller.events_after() if e["action"] == "watchdog_tick"]
        self.assertEqual(ticks[-1]["details"], {"recovered": True})
        status = controller.perform(request("start-tracking"))
        self.assertEqual(status["state"], "tracking")


class HealthAuditRateLimitTests(unittest.TestCase):
    def test_telemetry_updates_are_rate_limited_but_material_changes_are_not(self):
        clock = FakeClock()
        controller = SessionController(demo=True, clock=clock, event_limit=50)
        controller.perform(request("check"))
        before = len(controller.events_after())
        for index in range(200):
            clock.now += 1.0 / 60.0
            controller.update_health(source_age_ms=float(index % 7))
        health_events = [e for e in controller.events_after() if e["type"] == "health"]
        # 200 updates over ~3.3 s: the first is audited, the rest are not.
        self.assertEqual(len(health_events), 1)
        self.assertEqual(len(controller.events_after()), before + 1)
        # The control event survives in the bounded ring.
        self.assertIn("check", [e["action"] for e in controller.events_after()])

        controller.update_health(mocap_online=False)
        self.assertEqual(controller.events_after()[-1]["details"]["changes"], {"mocap_online": False})
        clock.now += HEALTH_AUDIT_INTERVAL_S
        controller.update_health(source_age_ms=3.0)
        self.assertEqual(controller.events_after()[-1]["details"]["changes"], {"source_age_ms": 3.0})


class DiagnosticsScrubTests(unittest.TestCase):
    def test_system_probe_contains_no_absolute_paths(self):
        repo = Path(__file__).resolve().parents[1]
        text = json.dumps(ReadOnlySystemProbe(repo).snapshot())
        self.assertNotIn(str(repo), text)
        self.assertNotIn(sys.executable, text)
        self.assertNotIn(str(Path.home()), text)


class ActionListTests(unittest.TestCase):
    def test_controls_cover_every_action(self):
        controls = SessionController(demo=True).status_snapshot()["controls"]
        self.assertEqual(set(ACTIONS), set(controls) - {"edit-config"})


@unittest.skipUnless(_API_TESTS_AVAILABLE, "FastAPI/httpx test dependencies are not installed")
class StateConflictStatusTests(unittest.TestCase):
    def test_config_locked_is_409_and_unknown_action_is_404(self):
        from fastapi.testclient import TestClient

        from mocap_g1.operator.app import create_app

        with tempfile.TemporaryDirectory() as directory:
            app = create_app(data_root=directory, demo=True)
            with TestClient(app, base_url="http://127.0.0.1:8766") as client:
                self.assertEqual(client.post("/api/v1/actions/check").status_code, 200)
                response = client.put("/api/v1/config", json={"mocap": {"port": 5556}})
                self.assertEqual(response.status_code, 409, response.text)
                self.assertEqual(response.json()["error"]["code"], "configuration_locked")
                self.assertEqual(client.post("/api/v1/actions/launch").status_code, 404)
                response = client.post("/api/v1/actions/arm")
                self.assertEqual(response.status_code, 409, response.text)


if __name__ == "__main__":
    unittest.main()
