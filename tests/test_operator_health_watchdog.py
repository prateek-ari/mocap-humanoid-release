"""Health staleness, re-applied stream gates, authority clamp, setup shutdown.

Covers review items T1-3, T1-4, T2-3 and T4-3 for the operator core.
"""

from __future__ import annotations

from importlib.util import find_spec
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from mocap_g1.operator import (
    ControlRequest,
    DemoRuntime,
    InvalidAction,
    PassiveRuntime,
    SessionController,
    SessionState,
)
from mocap_g1.operator.core import DEFAULT_HEALTH_STALE_TIMEOUT_S

_API_TESTS_AVAILABLE = find_spec("fastapi") is not None and find_spec("httpx") is not None
PREFLIGHT = {"clear_area": True, "harness_or_support": True, "stop_drill": True}
HEALTHY_STREAM = {
    "mocap_online": True,
    "mocap_rate_hz": 60.0,
    "packet_loss_percent": 0.0,
    "source_age_ms": 5.0,
}


class FakeClock:
    def __init__(self) -> None:
        self.value = 1_000.0

    def __call__(self) -> float:
        return self.value

    def advance(self, seconds: float) -> None:
        self.value += seconds


class VirtualRobotRuntime(PassiveRuntime):
    """Passive (externally fed) runtime that may reach TRACKING without hardware."""

    is_simulated = True
    uses_virtual_robot = True

    def __init__(self) -> None:
        self.tick_changes: dict = {}

    def tick(self, health):
        return dict(self.tick_changes)


def request(action: str, **kwargs) -> ControlRequest:
    return ControlRequest(action=action, **kwargs)


def drive_to(controller: SessionController, final: str) -> None:
    steps = (
        "check",
        "mocap-accept",
        "start-simulation",
        "connect-robot",
        "confirm-preflight",
        "arm",
        "start-tracking",
    )
    for action in steps:
        if action == "confirm-preflight":
            controller.perform(request(action, spotter="spotter", metadata=PREFLIGHT))
        else:
            controller.perform(request(action))
        if action == final:
            return


def passive_tracking(clock: FakeClock, runtime: VirtualRobotRuntime | None = None):
    runtime = runtime or VirtualRobotRuntime()
    controller = SessionController(runtime=runtime, clock=clock)
    controller.perform(request("check"))
    controller.update_health(
        **HEALTHY_STREAM,
        simulation_ready=True,
        robot_connected=True,
        robot_state_fresh=True,
        controller_ready=True,
    )
    for action in ("mocap-accept", "start-simulation", "connect-robot"):
        controller.perform(request(action))
    controller.perform(request("confirm-preflight", spotter="spotter", metadata=PREFLIGHT))
    controller.perform(request("arm"))
    controller.perform(request("start-tracking"))
    assert controller.state is SessionState.TRACKING
    return controller, runtime


class HealthStalenessTests(unittest.TestCase):
    def test_default_threshold_is_source_budget_plus_watchdog_slack(self):
        # 100 ms source budget < threshold < 500 ms envelope hard timeout.
        self.assertGreater(DEFAULT_HEALTH_STALE_TIMEOUT_S, 0.1)
        self.assertLess(DEFAULT_HEALTH_STALE_TIMEOUT_S, 0.5)
        with self.assertRaises(ValueError):
            SessionController(health_stale_timeout_s=0.0)
        with self.assertRaises(ValueError):
            SessionController(health_stale_timeout_s=float("nan"))

    def test_silence_after_one_good_update_latches_health_stale(self):
        clock = FakeClock()
        controller, _runtime = passive_tracking(clock)

        clock.advance(DEFAULT_HEALTH_STALE_TIMEOUT_S * 0.8)
        controller.watchdog_tick()
        self.assertEqual(controller.state, SessionState.TRACKING)
        self.assertTrue(controller.health.command_authority)

        clock.advance(DEFAULT_HEALTH_STALE_TIMEOUT_S * 0.4)
        controller.watchdog_tick()
        status = controller.status_snapshot()
        self.assertEqual(status["state"], "fault_latched")
        self.assertEqual(status["active_fault"]["code"], "health_stale")
        self.assertTrue(status["active_fault"]["recoverable"])
        self.assertFalse(status["health"]["command_authority"])
        latch_events = [
            event for event in controller.events_after() if event["action"] == "latch"
        ]
        self.assertEqual(latch_events[-1]["details"]["code"], "health_stale")

    def test_one_hour_of_silence_latches_on_the_first_tick(self):
        clock = FakeClock()
        controller, _runtime = passive_tracking(clock)
        clock.advance(3600.0)
        controller.watchdog_tick()
        self.assertEqual(controller.state, SessionState.FAULT_LATCHED)
        self.assertFalse(controller.health.command_authority)

    def test_runtime_without_tick_still_ages_out(self):
        clock = FakeClock()
        runtime = VirtualRobotRuntime()
        controller, _ = passive_tracking(clock, runtime)
        with mock.patch.object(VirtualRobotRuntime, "tick", None):
            clock.advance(1.0)
            controller.watchdog_tick()
        self.assertEqual(controller._active_fault.code, "health_stale")

    def test_empty_runtime_tick_is_not_a_heartbeat_but_reports_are(self):
        clock = FakeClock()
        controller, runtime = passive_tracking(clock)
        for _ in range(10):
            clock.advance(0.1)
            controller.update_health(source_age_ms=4.0)
            controller.watchdog_tick()
        self.assertEqual(controller.state, SessionState.TRACKING)

        runtime.tick_changes = {"source_age_ms": 6.0}
        for _ in range(10):
            clock.advance(0.1)
            controller.watchdog_tick()
        self.assertEqual(controller.state, SessionState.TRACKING)

        runtime.tick_changes = {}
        for _ in range(3):
            clock.advance(0.1)
            controller.watchdog_tick()
        self.assertEqual(controller._active_fault.code, "health_stale")

    def test_backwards_clock_is_treated_as_stale(self):
        clock = FakeClock()
        controller, _runtime = passive_tracking(clock)
        clock.advance(-1.0)
        controller.watchdog_tick()
        self.assertEqual(controller._active_fault.code, "health_stale")

    def test_nonfinite_clock_is_treated_as_stale(self):
        clock = FakeClock()
        controller, _runtime = passive_tracking(clock)
        clock.value = float("nan")
        controller.watchdog_tick()
        self.assertEqual(controller.state, SessionState.FAULT_LATCHED)
        self.assertFalse(controller.health.command_authority)

    def test_stale_health_refuses_start_tracking_and_rearm(self):
        clock = FakeClock()
        runtime = VirtualRobotRuntime()
        controller = SessionController(runtime=runtime, clock=clock)
        controller.perform(request("check"))
        controller.update_health(
            **HEALTHY_STREAM,
            simulation_ready=True,
            robot_connected=True,
            robot_state_fresh=True,
            controller_ready=True,
        )
        for action in ("mocap-accept", "start-simulation", "connect-robot"):
            controller.perform(request(action))
        controller.perform(request("confirm-preflight", spotter="spotter", metadata=PREFLIGHT))
        controller.perform(request("arm"))

        clock.advance(1.0)
        with self.assertRaises(InvalidAction) as raised:
            controller.perform(request("start-tracking"))
        self.assertEqual(raised.exception.code, "tracking_not_ready")
        self.assertIn("stale", str(raised.exception))
        self.assertFalse(controller.health.command_authority)

        controller.update_health(source_age_ms=3.0)
        controller.perform(request("start-tracking"))
        clock.advance(1.0)
        controller.watchdog_tick()
        controller.clear_fault_condition("health_stale")
        with self.assertRaises(InvalidAction) as raised:
            controller.perform(request("rearm"))
        self.assertEqual(raised.exception.code, "rearm_not_ready")
        controller.update_health(source_age_ms=3.0)
        status = controller.perform(request("rearm"))
        self.assertEqual(status["state"], "armed_disabled")
        self.assertFalse(status["health"]["command_authority"])

    def test_demo_runtime_heartbeat_keeps_tracking(self):
        clock = FakeClock()
        controller = SessionController(demo=True, clock=clock)
        drive_to(controller, "start-tracking")
        for _ in range(100):
            clock.advance(0.1)
            controller.watchdog_tick()
        self.assertEqual(controller.state, SessionState.TRACKING)
        self.assertTrue(controller.health.command_authority)

    def test_demo_runtime_does_not_heartbeat_before_checks(self):
        self.assertEqual(DemoRuntime().tick(SessionController().health), {})

    def test_demo_without_heartbeat_latches(self):
        clock = FakeClock()
        controller = SessionController(demo=True, clock=clock)
        drive_to(controller, "start-tracking")
        with mock.patch.object(DemoRuntime, "tick", return_value={}):
            clock.advance(DEFAULT_HEALTH_STALE_TIMEOUT_S + 0.05)
            controller.watchdog_tick()
        self.assertEqual(controller._active_fault.code, "health_stale")


class DegradedStreamDuringTrackingTests(unittest.TestCase):
    DEGRADATIONS = (
        ({"mocap_online": False}, "offline"),
        ({"packet_loss_percent": 99.0}, "loss"),
        ({"mocap_rate_hz": 1.0}, "below 50"),
        ({"source_age_ms": 250.0}, "stale"),
    )

    def test_update_health_degradation_latches(self):
        for changes, reason in self.DEGRADATIONS:
            with self.subTest(changes=changes):
                controller, _runtime = passive_tracking(FakeClock())
                controller.update_health(**changes)
                self.assertEqual(controller.state, SessionState.FAULT_LATCHED)
                self.assertEqual(controller._active_fault.code, "tracking_health_lost")
                self.assertIn(reason, controller._active_fault.reason)
                self.assertFalse(controller.health.command_authority)

    def test_combined_degradation_from_review_latches(self):
        controller, _runtime = passive_tracking(FakeClock())
        controller.update_health(
            mocap_online=False, packet_loss_percent=99.0, mocap_rate_hz=1.0
        )
        self.assertEqual(controller.state, SessionState.FAULT_LATCHED)

    def test_runtime_tick_degradation_latches(self):
        for changes, reason in self.DEGRADATIONS:
            with self.subTest(changes=changes):
                clock = FakeClock()
                controller, runtime = passive_tracking(clock)
                runtime.tick_changes = changes
                clock.advance(0.1)
                controller.watchdog_tick()
                self.assertEqual(controller._active_fault.code, "tracking_health_lost")
                self.assertIn(reason, controller._active_fault.reason)
                self.assertFalse(controller.health.command_authority)


class RecordedSimulationKeepsTrackingTests(unittest.TestCase):
    def test_healthy_recorded_simulation_tracks_through_many_ticks(self):
        from mocap_g1.operator.simulation import RecordedSimulationRuntime
        from tests.test_operator_simulation import write_qpos_npz

        with tempfile.TemporaryDirectory() as directory:
            # 30 Hz exercises the recorded-source exemption from the 50 Hz gate.
            source = Path(directory) / "thirty hz.npz"
            write_qpos_npz(source, fps=30.0)
            clock = FakeClock()
            runtime = RecordedSimulationRuntime(source, clock=clock)
            controller = SessionController(runtime=runtime, clock=clock)
            drive_to(controller, "start-tracking")
            self.assertEqual(controller.state, SessionState.TRACKING)
            # 10 s at the app's 100 ms watchdog period, looping the recording.
            for _ in range(100):
                clock.advance(0.1)
                controller.watchdog_tick()
            status = controller.status_snapshot()
            self.assertEqual(status["state"], "tracking", status["active_fault"])
            self.assertTrue(status["health"]["command_authority"])
            self.assertEqual(status["simulation"]["state"], "playing")


class CommandAuthorityClampTests(unittest.TestCase):
    def clamp_events(self, controller):
        return [
            event
            for event in controller.events_after()
            if event["action"] == "command_authority_clamped"
        ]

    def test_runtime_check_cannot_assert_authority(self):
        class ClaimingRuntime(DemoRuntime):
            def check(self, health, config):
                return {**super().check(health, config), "command_authority": True}

        controller = SessionController(runtime=ClaimingRuntime())
        status = controller.perform(request("check"))
        self.assertEqual(status["state"], "checking")
        self.assertFalse(status["health"]["command_authority"])
        events = self.clamp_events(controller)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["outcome"], "rejected")
        self.assertEqual(events[0]["actor"], "runtime")
        self.assertEqual(
            controller.audit.read()[-2]["action"], "command_authority_clamped"
        )

    def test_update_health_cannot_assert_authority_outside_tracking(self):
        controller = SessionController(demo=True)
        controller.update_health(command_authority=True, actor="supervisor")
        self.assertEqual(controller.state, SessionState.IDLE)
        self.assertFalse(controller.health.command_authority)
        events = self.clamp_events(controller)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["actor"], "supervisor")
        self.assertTrue(events[0]["details"]["claimed"])

        drive_to(controller, "arm")
        controller.update_health(command_authority=True)
        self.assertEqual(controller.state, SessionState.ARMED_DISABLED)
        self.assertFalse(controller.health.command_authority)
        controller.update_health(command_authority=False)
        self.assertEqual(len(self.clamp_events(controller)), 2)

    def test_tracking_may_report_authority(self):
        controller = SessionController(demo=True)
        drive_to(controller, "start-tracking")
        controller.update_health(command_authority=True)
        self.assertTrue(controller.health.command_authority)
        self.assertEqual(self.clamp_events(controller), [])


class SetupRequiredShutdownTests(unittest.TestCase):
    def test_shutdown_keeps_setup_required_and_never_writes_config(self):
        with tempfile.TemporaryDirectory() as directory:
            config_path = Path(directory) / "config.json"
            rejected = b'{"mocap": null, "privacy": {"record_raw_mocap": true}}\n'
            config_path.write_bytes(rejected)
            controller = SessionController(
                config_path=config_path, setup_complete=False, demo=True
            )
            status = controller.perform(request("shutdown"))
            self.assertEqual(status["state"], "setup_required")
            self.assertFalse(status["health"]["command_authority"])
            self.assertEqual(config_path.read_bytes(), rejected)
            self.assertEqual(
                sorted(path.name for path in Path(directory).iterdir()), ["config.json"]
            )
            event = controller.audit.read()[-1]
            self.assertEqual(event["action"], "shutdown")
            self.assertEqual(event["outcome"], "accepted")
            self.assertFalse(event["details"]["config_persisted"])
            # Saving a valid configuration is still the way out of setup.
            controller.update_config({"mocap": {"port": 5556}})
            self.assertEqual(controller.state, SessionState.IDLE)
            self.assertEqual(json.loads(config_path.read_text())["mocap"]["port"], 5556)

    def test_normal_shutdown_still_persists_consent_reset(self):
        with tempfile.TemporaryDirectory() as directory:
            config_path = Path(directory) / "config.json"
            controller = SessionController(config_path=config_path, demo=True)
            controller.update_config({"privacy": {"record_raw_mocap": True}})
            controller.perform(request("shutdown"))
            saved = json.loads(config_path.read_text())
            self.assertFalse(saved["privacy"]["record_raw_mocap"])

    @unittest.skipUnless(_API_TESTS_AVAILABLE, "FastAPI/httpx test dependencies are not installed")
    def test_api_shutdown_after_rejected_config_leaves_file_untouched(self):
        from fastapi.testclient import TestClient

        from mocap_g1.operator.app import create_app

        with tempfile.TemporaryDirectory() as directory:
            config_path = Path(directory) / "config.json"
            rejected = b'{"schema_version": 1, "mocap": {"port": 0}}'
            config_path.write_bytes(rejected)
            with TestClient(
                create_app(data_root=directory), base_url="http://127.0.0.1:8766"
            ) as client:
                self.assertEqual(
                    client.get("/api/v1/status").json()["state"], "setup_required"
                )
                response = client.post("/api/v1/actions/shutdown")
                self.assertEqual(response.status_code, 200, response.text)
                self.assertEqual(response.json()["state"], "setup_required")
            self.assertEqual(config_path.read_bytes(), rejected)


class ServiceVersionTests(unittest.TestCase):
    def test_version_matches_distribution_and_pyproject(self):
        from importlib.metadata import version

        from mocap_g1.operator._version import _source_tree_version, service_version

        self.assertEqual(service_version(), version("mocap-g1"))
        self.assertEqual(service_version(), _source_tree_version())

    def test_unpackaged_tree_falls_back_to_pyproject(self):
        from importlib.metadata import PackageNotFoundError

        from mocap_g1.operator import _version

        _version.service_version.cache_clear()
        self.addCleanup(_version.service_version.cache_clear)
        with mock.patch.object(
            _version, "version", side_effect=PackageNotFoundError("mocap-g1")
        ):
            self.assertEqual(_version.service_version(), _version._source_tree_version())
            self.assertRegex(_version.service_version(), r"^\d+\.\d+")
        _version.service_version.cache_clear()
        with mock.patch.object(
            _version, "version", side_effect=PackageNotFoundError("mocap-g1")
        ), mock.patch.object(_version, "_source_tree_version", return_value=None):
            self.assertEqual(_version.service_version(), _version.UNKNOWN_VERSION)

    @unittest.skipUnless(_API_TESTS_AVAILABLE, "FastAPI/httpx test dependencies are not installed")
    def test_openapi_reports_service_version(self):
        from mocap_g1.operator._version import service_version
        from mocap_g1.operator.app import create_app

        with tempfile.TemporaryDirectory() as directory:
            app = create_app(data_root=directory, demo=True)
        self.assertEqual(app.version, service_version())


if __name__ == "__main__":
    unittest.main()
