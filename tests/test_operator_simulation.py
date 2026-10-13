from __future__ import annotations

from importlib.util import find_spec
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import numpy as np

from mocap_g1.operator.core import SessionController
from mocap_g1.operator.models import ControlRequest, HealthSnapshot
from mocap_g1.operator.simulation import RecordedSimulationRuntime


_API_TESTS_AVAILABLE = find_spec("fastapi") is not None and find_spec("httpx") is not None


class FakeClock:
    def __init__(self) -> None:
        self.value = 100.0

    def __call__(self) -> float:
        return self.value

    def advance(self, seconds: float) -> None:
        self.value += seconds


def write_qpos_npz(
    path: Path,
    *,
    frame_count: int = 121,
    fps: float = 60.0,
) -> np.ndarray:
    phase = np.linspace(0.0, 1.0, frame_count, dtype=np.float64)
    qpos = np.zeros((frame_count, 36), dtype=np.float64)
    qpos[:, 0] = 0.2 * phase
    qpos[:, 2] = 0.8
    qpos[:, 3] = 1.0
    qpos[:, 7:19] = 0.1 * phase[:, None]
    qpos[:, 19:] = 0.6 * phase[:, None]
    metadata = {
        "robot": "unitree_g1",
        "root_quaternion_order": "wxyz",
        "purpose": "operator-recorded-simulation-test",
    }
    np.savez_compressed(
        path,
        metadata=np.asarray(json.dumps(metadata)),
        qpos=qpos,
        fps=np.asarray(fps, dtype=np.float64),
    )
    return qpos


def request(action: str) -> ControlRequest:
    return ControlRequest(action=action, operator="simulation-test")


class RecordedSimulationRuntimeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.source = Path(self.temporary.name) / "recorded motion.npz"
        self.qpos = write_qpos_npz(self.source)
        self.clock = FakeClock()
        self.runtime = RecordedSimulationRuntime(self.source, clock=self.clock)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def test_monotonic_sequence_reports_reference_and_safety_bounded_qpos(self):
        initial = self.runtime.simulation_snapshot()
        self.assertEqual(initial["mode"], "recorded_simulation")
        self.assertEqual(initial["state"], "ready")
        self.assertEqual(initial["source"]["name"], self.source.name)
        self.assertEqual(initial["source"]["frame_count"], len(self.qpos))
        self.assertEqual(initial["source"]["fps"], 60.0)
        self.assertEqual(initial["source"]["duration_s"], 2.0)
        self.assertEqual(len(initial["source"]["sha256"]), 64)
        self.assertEqual(len(initial["reference_qpos"]), 36)
        self.assertEqual(len(initial["bounded_qpos"]), 36)
        self.assertTrue(initial["limitations"])

        checked = self.runtime.check(HealthSnapshot(), {})
        self.assertTrue(checked["mocap_online"])
        self.assertFalse(checked["controller_ready"])
        started = self.runtime.start_simulation(HealthSnapshot(), {})
        self.assertTrue(started["simulation_ready"])
        self.assertFalse(started["command_authority"])
        self.runtime.set_tracking(True)

        self.clock.advance(0.25)
        self.runtime.tick(HealthSnapshot())
        status = self.runtime.simulation_snapshot()
        self.assertEqual(status["state"], "playing")
        self.assertEqual(status["frame_index"], 15)
        self.assertAlmostEqual(status["progress_pct"], 12.5)
        np.testing.assert_allclose(status["reference_qpos"], self.qpos[15])
        np.testing.assert_allclose(status["bounded_qpos"][:19], self.qpos[15, :19])
        self.assertFalse(
            np.allclose(status["bounded_qpos"][19:], self.qpos[15, 19:])
        )
        telemetry = status["telemetry"]
        self.assertEqual(telemetry["source_rate_hz"], 60.0)
        self.assertEqual(telemetry["command_rate_hz"], 60.0)
        self.assertEqual(telemetry["safety_reason"], "tracking")
        self.assertFalse(telemetry["reset_required"])

        stopped = self.runtime.safe_stop(HealthSnapshot(command_authority=True))
        self.assertFalse(stopped["command_authority"])
        self.assertEqual(self.runtime.simulation_snapshot()["state"], "stopped")

    def test_stale_hard_timeout_and_explicit_recovery(self):
        self.runtime.start_simulation(HealthSnapshot(), {})
        self.runtime.set_tracking(True)
        self.clock.advance(0.20)
        self.runtime.tick(HealthSnapshot())
        stalled_frame = self.runtime.simulation_snapshot()["frame_index"]

        self.runtime.set_source_available(False)
        self.clock.advance(0.15)
        self.runtime.tick(HealthSnapshot())
        stale = self.runtime.simulation_snapshot()
        self.assertEqual(stale["frame_index"], stalled_frame)
        self.assertEqual(stale["telemetry"]["safety_reason"], "stale_source")
        self.assertFalse(stale["telemetry"]["reset_required"])

        self.clock.advance(0.40)
        self.runtime.tick(HealthSnapshot())
        timed_out = self.runtime.simulation_snapshot()
        self.assertEqual(timed_out["state"], "blocked_reset_required")
        self.assertEqual(timed_out["telemetry"]["safety_reason"], "hard_timeout")
        self.assertTrue(timed_out["telemetry"]["reset_required"])
        self.assertEqual(timed_out["telemetry"]["blend_weight"], 0.0)

        self.runtime.set_source_available(True)
        self.runtime.recover()
        recovered = self.runtime.simulation_snapshot()
        self.assertEqual(recovered["state"], "playing")
        self.assertEqual(recovered["telemetry"]["safety_reason"], "recovered")
        self.assertFalse(recovered["telemetry"]["reset_required"])
        self.runtime.set_tracking(True)
        self.clock.advance(0.05)
        self.runtime.tick(HealthSnapshot())
        self.assertEqual(
            self.runtime.simulation_snapshot()["telemetry"]["safety_reason"],
            "tracking",
        )

    def test_session_flow_never_uses_demo_hardware_bypass(self):
        low_rate_source = Path(self.temporary.name) / "30hz.npz"
        write_qpos_npz(low_rate_source, fps=30.0)
        runtime = RecordedSimulationRuntime(low_rate_source, clock=self.clock)
        controller = SessionController(runtime=runtime)

        controller.perform(request("check"))
        controller.perform(request("mocap-accept"))
        status = controller.perform(request("start-simulation"))
        self.assertEqual(status["state"], "sim_validated")
        self.assertEqual(status["runtime_mode"], "fake")
        self.assertEqual(status["simulation"]["state"], "playing")
        self.assertFalse(status["health"]["command_authority"])
        status = controller.perform(request("connect-robot"))
        self.assertEqual(status["state"], "robot_connected")
        self.assertTrue(status["health"]["robot_connected"])
        self.assertFalse(status["capability_lock"]["whole_body_unlocked"])
        controller.perform(
            ControlRequest(
                action="confirm-preflight",
                operator="simulation-test",
                spotter="simulation-spotter",
                metadata={
                    "clear_area": True,
                    "harness_or_support": True,
                    "stop_drill": True,
                },
            )
        )
        controller.perform(request("arm"))
        status = controller.perform(request("start-tracking"))
        self.assertEqual(status["state"], "tracking")
        self.assertEqual(status["simulation"]["frame_index"], 0)

    def test_tracking_stale_fault_recovers_to_disabled_rearm(self):
        controller = SessionController(runtime=self.runtime)
        for action in ("check", "mocap-accept", "start-simulation", "connect-robot"):
            controller.perform(request(action))
        controller.perform(
            ControlRequest(
                action="confirm-preflight",
                operator="simulation-test",
                spotter="simulation-spotter",
                metadata={
                    "clear_area": True,
                    "harness_or_support": True,
                    "stop_drill": True,
                },
            )
        )
        controller.perform(request("arm"))
        controller.perform(request("start-tracking"))

        self.runtime.set_source_available(False)
        self.clock.advance(0.12)
        controller.watchdog_tick()
        faulted = controller.status_snapshot()
        self.assertEqual(faulted["state"], "fault_latched")
        self.assertEqual(faulted["active_fault"]["code"], "tracking_health_lost")
        self.assertFalse(faulted["health"]["command_authority"])

        rearmed = controller.perform(request("rearm"))
        self.assertEqual(rearmed["state"], "armed_disabled")
        self.assertEqual(rearmed["simulation"]["state"], "playing")
        self.assertEqual(rearmed["health"]["source_age_ms"], 0.0)
        resumed = controller.perform(request("start-tracking"))
        self.assertEqual(resumed["state"], "tracking")

    def test_tracking_restarts_and_loops_the_recorded_timeline(self):
        short_source = Path(self.temporary.name) / "short loop.npz"
        write_qpos_npz(short_source, frame_count=7, fps=60.0)
        runtime = RecordedSimulationRuntime(short_source, clock=self.clock)
        runtime.start_simulation(HealthSnapshot(), {})
        self.clock.advance(0.20)
        runtime.tick(HealthSnapshot())
        self.assertEqual(runtime.simulation_snapshot()["state"], "completed")

        runtime.set_tracking(True)
        restarted = runtime.simulation_snapshot()
        self.assertEqual(restarted["frame_index"], 0)
        self.assertEqual(restarted["progress_pct"], 0.0)
        self.clock.advance(0.15)
        runtime.tick(HealthSnapshot())
        looped = runtime.simulation_snapshot()
        self.assertEqual(looped["state"], "playing")
        self.assertLess(looped["frame_index"], looped["source"]["frame_count"] - 1)


@unittest.skipUnless(_API_TESTS_AVAILABLE, "FastAPI/httpx test dependencies are not installed")
class RecordedSimulationApiTests(unittest.TestCase):
    def setUp(self) -> None:
        from fastapi.testclient import TestClient

        from mocap_g1.operator.app import create_app

        self.temporary = tempfile.TemporaryDirectory()
        self.source = Path(self.temporary.name) / "api simulation.npz"
        self.qpos = write_qpos_npz(self.source, frame_count=241)
        self.production_root = Path(self.temporary.name) / "production-must-not-exist"
        self.app = create_app(
            data_root=self.production_root,
            simulation=self.source,
            # The TestClient event socket always uses Host "testserver", which
            # is not loopback, so it needs the identity-proxy configuration.
            allowed_hosts=("http://testserver",),
            identity_header="X-Test-Identity",
            trusted_proxies=("192.0.2.1",),
        )
        self.client = TestClient(self.app)

    def tearDown(self) -> None:
        self.client.close()
        self.temporary.cleanup()

    def test_contract_normal_sequence_periodic_snapshots_and_hardware_lock(self):
        self.assertIsNone(self.app.state.data_root)
        self.assertFalse(self.production_root.exists())
        status = self.client.get("/api/v1/status").json()
        self.assertEqual(status["presentation_mode"], "recorded_simulation")
        self.assertEqual(status["runtime_mode"], "fake")
        self.assertFalse(status["health"]["command_authority"])
        self.assertFalse(status["capability_lock"]["whole_body_unlocked"])
        self.assertFalse(status["capability_lock"]["manifest_verified"])
        simulation = status["simulation"]
        self.assertEqual(
            set(simulation),
            {
                "mode",
                "state",
                "source",
                "frame_index",
                "progress_pct",
                "reference_qpos",
                "bounded_qpos",
                "telemetry",
                "limitations",
            },
        )
        self.assertEqual(
            set(simulation["source"]),
            {"name", "frame_count", "fps", "duration_s", "sha256"},
        )
        self.assertEqual(
            set(simulation["telemetry"]),
            {
                "source_age_ms",
                "source_rate_hz",
                "command_rate_hz",
                "safety_reason",
                "blend_weight",
                "reset_required",
                "clamped_joint_count",
            },
        )

        for action, expected_state in (
            ("check", "checking"),
            ("mocap-accept", "mocap_accepted"),
            ("start-simulation", "sim_validated"),
        ):
            response = self.client.post(f"/api/v1/actions/{action}")
            self.assertEqual(response.status_code, 200, response.text)
            self.assertEqual(response.json()["state"], expected_state)

        connected = self.client.post("/api/v1/actions/connect-robot")
        self.assertEqual(connected.status_code, 200, connected.text)
        self.assertEqual(connected.json()["state"], "robot_connected")
        self.assertTrue(connected.json()["health"]["robot_connected"])
        self.assertFalse(connected.json()["capability_lock"]["whole_body_unlocked"])
        preflight = self.client.post(
            "/api/v1/actions/confirm-preflight",
            json={
                "spotter": "simulation-spotter",
                "metadata": {
                    "clear_area": True,
                    "harness_or_support": True,
                    "stop_drill": True,
                },
            },
        )
        self.assertEqual(preflight.status_code, 200, preflight.text)
        armed = self.client.post("/api/v1/actions/arm")
        self.assertEqual(armed.status_code, 200, armed.text)
        tracking = self.client.post("/api/v1/actions/start-tracking")
        self.assertEqual(tracking.status_code, 200, tracking.text)
        self.assertEqual(tracking.json()["state"], "tracking")
        self.assertFalse(
            self.client.get("/api/v1/status").json()["health"]["command_authority"]
        )

        with self.client.websocket_connect("/api/v1/events") as websocket:
            first = websocket.receive_json()
            self.assertEqual(first["type"], "snapshot")
            saw_event = False
            saw_periodic_snapshot = False
            for _ in range(10):
                message = websocket.receive_json()
                if message["type"] == "snapshot":
                    saw_periodic_snapshot = True
                    self.assertEqual(
                        message["status"]["presentation_mode"],
                        "recorded_simulation",
                    )
                    break
                saw_event = True
            self.assertTrue(saw_event)
            self.assertTrue(saw_periodic_snapshot)

        shutdown = self.client.post("/api/v1/actions/shutdown")
        self.assertEqual(shutdown.status_code, 200, shutdown.text)
        self.assertEqual(shutdown.json()["simulation"]["state"], "stopped")
        self.assertFalse(shutdown.json()["health"]["command_authority"])

    def test_create_app_modes_are_mutually_exclusive(self):
        from mocap_g1.operator.app import create_app

        with self.assertRaisesRegex(ValueError, "mutually exclusive"):
            create_app(demo=True, simulation=self.source)
        video = Path(self.temporary.name) / "review.mp4"
        video.write_bytes(b"\x00\x00\x00\x18ftypmp42recorded-review")
        with self.assertRaisesRegex(ValueError, "mutually exclusive"):
            create_app(playback=video, simulation=self.source)


class RecordedSimulationCliTests(unittest.TestCase):
    def test_simulation_is_mutually_exclusive_with_other_cli_modes(self):
        from mocap_g1.operator.cli import build_parser

        with self.assertRaises(SystemExit):
            build_parser().parse_args(["--demo", "--simulation", "motion.npz"])
        with self.assertRaises(SystemExit):
            build_parser().parse_args(
                ["--playback", "review.mp4", "--simulation", "motion.npz"]
            )

    def test_cli_resolves_simulation_without_creating_production_state(self):
        from mocap_g1.operator import cli as operator_console

        fake_uvicorn = mock.Mock()
        fake_factory = mock.Mock(return_value=object())
        with tempfile.TemporaryDirectory() as temporary, mock.patch.object(
            operator_console,
            "_load_runtime",
            return_value=(fake_uvicorn, fake_factory),
        ):
            source = Path(temporary) / "cli motion.npz"
            write_qpos_npz(source)
            production_root = Path(temporary) / "production-must-not-exist"
            result = operator_console.main(
                [
                    "--data-root",
                    str(production_root),
                    "--simulation",
                    str(source),
                ]
            )

        self.assertEqual(result, 0)
        self.assertFalse(production_root.exists())
        self.assertIsNone(fake_factory.call_args.kwargs["data_root"])
        self.assertEqual(
            fake_factory.call_args.kwargs["simulation"], source.resolve()
        )
        self.assertIsNone(fake_factory.call_args.kwargs["playback"])
        self.assertFalse(fake_factory.call_args.kwargs["demo"])

    def test_cli_rejects_missing_simulation_before_runtime_load(self):
        from mocap_g1.operator import cli as operator_console

        with tempfile.TemporaryDirectory() as temporary, mock.patch.object(
            operator_console, "_load_runtime"
        ) as load_runtime:
            result = operator_console.main(
                ["--simulation", str(Path(temporary) / "missing.npz")]
            )
        self.assertEqual(result, 2)
        load_runtime.assert_not_called()


if __name__ == "__main__":
    unittest.main()
