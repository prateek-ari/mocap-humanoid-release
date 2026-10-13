"""T2-2: runtime-adapter calls are bounded; a hung adapter cannot freeze stops."""

from __future__ import annotations

from importlib.util import find_spec
import threading
import time
import unittest
from unittest import mock

from mocap_g1.operator import ControlRequest, DemoRuntime, SessionController
from mocap_g1.operator.adapter_worker import AdapterWorker
from mocap_g1.operator.core import (
    OperatorError,
    RuntimeAdapterTimeout,
    RuntimeAdapterUnavailable,
)

_API_TESTS_AVAILABLE = find_spec("fastapi") is not None and find_spec("httpx") is not None
PREFLIGHT = {"clear_area": True, "harness_or_support": True, "stop_drill": True}
# Short deadlines keep the suite fast; the hung calls are released explicitly.
FAST = {"tick": 0.2, "safe_stop": 0.2, "set_tracking": 0.2, "shutdown": 0.2}


def request(action: str, **kwargs) -> ControlRequest:
    return ControlRequest(action=action, **kwargs)


def reach_tracking(controller: SessionController) -> None:
    for action in ("check", "mocap-accept", "start-simulation", "connect-robot"):
        controller.perform(request(action))
    controller.perform(request("confirm-preflight", spotter="spotter", metadata=PREFLIGHT))
    controller.perform(request("arm"))
    controller.perform(request("start-tracking"))


def wait_for(predicate, timeout: float = 2.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


class HangingRuntime(DemoRuntime):
    """Demo runtime whose selected calls block until ``release`` is set."""

    def __init__(self) -> None:
        self.hang: set[str] = set()
        self.entered = threading.Event()
        self.release = threading.Event()
        self.safe_stop_calls = 0

    def _maybe_hang(self, name: str) -> None:
        if name in self.hang:
            self.entered.set()
            self.release.wait(5.0)

    def tick(self, health):
        self._maybe_hang("tick")
        return super().tick(health)

    def set_tracking(self, enabled):
        if enabled:
            self._maybe_hang("set_tracking_on")
        return None

    def safe_stop(self, health):
        self.safe_stop_calls += 1
        self._maybe_hang("safe_stop")
        return super().safe_stop(health)


class AdapterWorkerTests(unittest.TestCase):
    def worker(self) -> AdapterWorker:
        return AdapterWorker(
            timeout_error=RuntimeAdapterTimeout,
            unavailable_error=RuntimeAdapterUnavailable,
        )

    def test_results_and_exceptions_cross_threads_unchanged(self):
        worker = self.worker()
        self.assertEqual(worker.call("add", lambda a, b: a + b, 2, 3, timeout_s=1.0), 5)
        error = KeyError("boom")

        def fail():
            raise error

        with self.assertRaises(KeyError) as raised:
            worker.call("fail", fail, timeout_s=1.0)
        self.assertIs(raised.exception, error)

    def test_overdue_call_blocks_later_calls_until_it_returns(self):
        worker = self.worker()
        release = threading.Event()
        started = time.monotonic()
        with self.assertRaises(RuntimeAdapterTimeout):
            worker.call("tick", release.wait, 5.0, timeout_s=0.05)
        self.assertLess(time.monotonic() - started, 1.0)
        self.assertEqual(worker.blocked_call, "tick")
        started = time.monotonic()
        with self.assertRaises(RuntimeAdapterUnavailable) as raised:
            worker.call("safe_stop", lambda: "never", timeout_s=1.0)
        # Fails fast instead of queueing behind the wedged call.
        self.assertLess(time.monotonic() - started, 0.5)
        self.assertEqual(raised.exception.blocked_call, "tick")
        release.set()
        self.assertTrue(wait_for(lambda: worker.blocked_call is None))
        self.assertEqual(worker.call("safe_stop", lambda: "ok", timeout_s=1.0), "ok")

    def test_calls_never_overlap(self):
        worker = self.worker()
        active = 0
        peak = 0
        guard = threading.Lock()

        def probe():
            nonlocal active, peak
            with guard:
                active += 1
                peak = max(peak, active)
            time.sleep(0.001)
            with guard:
                active -= 1

        threads = [
            threading.Thread(
                target=lambda: [worker.call("probe", probe, timeout_s=2.0) for _ in range(20)]
            )
            for _ in range(4)
        ]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(5.0)
        self.assertEqual(peak, 1)


class HungAdapterControllerTests(unittest.TestCase):
    def setUp(self):
        self.runtime = HangingRuntime()
        self.addCleanup(self.runtime.release.set)
        self.controller = SessionController(
            runtime=self.runtime, demo=True, adapter_timeouts_s=FAST
        )
        reach_tracking(self.controller)

    def test_hung_tick_latches_fault_within_deadline_and_disengage_returns(self):
        self.runtime.hang.add("tick")
        started = time.monotonic()
        self.controller.watchdog_tick()
        self.assertLess(time.monotonic() - started, 1.0)

        status = self.controller.status_snapshot()
        self.assertEqual(status["state"], "fault_latched")
        self.assertEqual(status["active_fault"]["code"], "runtime_adapter_timeout")
        self.assertFalse(status["health"]["command_authority"])
        self.assertEqual(status["runtime_adapter"]["blocked_call"], "tick")
        # safe_stop could not be delivered, so the stop is unverified.
        self.assertFalse(status["controls"]["rearm"]["enabled"])
        safe_stop = [
            event
            for event in self.controller.events_after()
            if event["type"] == "runtime" and event["action"] == "safe_stop"
        ][-1]
        self.assertEqual(safe_stop["outcome"], "rejected")
        self.assertIs(safe_stop["details"]["delivered"], False)
        self.assertEqual(self.runtime.safe_stop_calls, 0)

        started = time.monotonic()
        status = self.controller.perform(request("disengage"))
        self.assertLess(time.monotonic() - started, 0.5)
        self.assertFalse(status["health"]["command_authority"])
        disengage = self.controller.events_after()[-1]
        self.assertEqual(disengage["action"], "disengage")
        self.assertFalse(disengage["details"]["verified"])
        self.assertIs(disengage["details"]["safe_stop"]["delivered"], False)

        # Once the adapter returns, a disengage can verify the stop again.
        self.runtime.hang.clear()
        self.runtime.release.set()
        self.assertTrue(
            wait_for(lambda: self.controller.status_snapshot()["runtime_adapter"]["blocked_call"] is None)
        )
        self.controller.perform(request("disengage"))
        self.assertTrue(self.controller.events_after()[-1]["details"]["verified"])
        self.assertTrue(self.controller.status_snapshot()["controls"]["rearm"]["enabled"])

    def test_disengage_while_tick_is_hung_returns_promptly(self):
        self.runtime.hang.add("tick")
        ticker = threading.Thread(target=self.controller.watchdog_tick)
        ticker.start()
        self.addCleanup(ticker.join, 2.0)
        self.assertTrue(self.runtime.entered.wait(2.0))

        started = time.monotonic()
        status = self.controller.perform(request("disengage"))
        elapsed = time.monotonic() - started
        # Bounded by the tick deadline, not by the hung adapter.
        self.assertLess(elapsed, 1.0)
        self.assertFalse(status["health"]["command_authority"])
        self.assertEqual(status["state"], "fault_latched")
        ticker.join(2.0)
        self.assertFalse(ticker.is_alive())

    def test_hung_safe_stop_is_bounded_and_keeps_stop_unverified(self):
        self.runtime.hang.add("safe_stop")
        started = time.monotonic()
        status = self.controller.perform(request("disengage"))
        self.assertLess(time.monotonic() - started, 1.0)
        self.assertEqual(status["state"], "fault_latched")
        self.assertEqual(status["active_fault"]["code"], "disengage_runtime_error")
        self.assertFalse(status["health"]["command_authority"])
        self.assertFalse(status["controls"]["rearm"]["enabled"])
        with self.assertRaises(OperatorError) as raised:
            self.controller.perform(request("rearm"))
        self.assertEqual(raised.exception.code, "stop_unverified")


class HungStartTrackingTests(unittest.TestCase):
    def test_set_tracking_timeout_latches_fault(self):
        runtime = HangingRuntime()
        self.addCleanup(runtime.release.set)
        controller = SessionController(runtime=runtime, demo=True, adapter_timeouts_s=FAST)
        for action in ("check", "mocap-accept", "start-simulation", "connect-robot"):
            controller.perform(request(action))
        controller.perform(request("confirm-preflight", spotter="s", metadata=PREFLIGHT))
        controller.perform(request("arm"))
        runtime.hang.add("set_tracking_on")
        with self.assertRaises(OperatorError):
            controller.perform(request("start-tracking"))
        status = controller.status_snapshot()
        # set_tracking(True) may still complete later: never leave that open.
        self.assertEqual(status["state"], "fault_latched")
        self.assertEqual(status["active_fault"]["code"], "runtime_adapter_timeout")
        self.assertFalse(status["health"]["command_authority"])
        self.assertFalse(status["controls"]["rearm"]["enabled"])
        rejected = [e for e in controller.events_after() if e["outcome"] == "rejected"]
        self.assertEqual(rejected[-1]["action"], "start-tracking")


class ConcurrentWatchdogAndPerformTests(unittest.TestCase):
    """T2-7 gap: watchdog ticks racing operator actions stay consistent."""

    def test_concurrent_watchdog_tick_and_perform(self):
        active = 0
        peak = 0
        guard = threading.Lock()

        class ProbeRuntime(DemoRuntime):
            def _enter(self):
                nonlocal active, peak
                with guard:
                    active += 1
                    peak = max(peak, active)

            def _leave(self):
                nonlocal active
                with guard:
                    active -= 1

            def tick(self, health):
                self._enter()
                try:
                    return super().tick(health)
                finally:
                    self._leave()

            def safe_stop(self, health):
                self._enter()
                try:
                    return super().safe_stop(health)
                finally:
                    self._leave()

        controller = SessionController(runtime=ProbeRuntime(), demo=True)
        stop = threading.Event()
        errors: list[BaseException] = []

        def ticker():
            try:
                while not stop.is_set():
                    controller.watchdog_tick()
                    time.sleep(0.0005)
            except BaseException as exc:  # pragma: no cover - reported below
                errors.append(exc)

        thread = threading.Thread(target=ticker)
        thread.start()
        try:
            for _ in range(6):
                for action in (
                    "check", "mocap-accept", "start-simulation", "connect-robot",
                ):
                    controller.perform(request(action))
                controller.perform(
                    request("confirm-preflight", spotter="s", metadata=PREFLIGHT)
                )
                for action in ("arm", "start-tracking", "disengage", "shutdown"):
                    status = controller.perform(request(action))
                    if status["health"]["command_authority"]:
                        self.assertEqual(status["state"], "tracking")
                self.assertEqual(status["state"], "idle")
        finally:
            stop.set()
            thread.join(5.0)
        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(peak, 1)


@unittest.skipUnless(_API_TESTS_AVAILABLE, "FastAPI/httpx test dependencies are not installed")
class WatchdogLoopSupervisionTests(unittest.TestCase):
    def app(self):
        import tempfile

        from mocap_g1.operator.app import create_app

        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        return create_app(data_root=directory.name, demo=True)

    def test_overdue_tick_is_reported_and_the_loop_survives(self):
        from fastapi.testclient import TestClient

        from mocap_g1.operator import app as app_module

        app = self.app()
        controller = app.state.controller
        release = threading.Event()
        self.addCleanup(release.set)
        calls = []

        def stuck_tick():
            calls.append(time.monotonic())
            if len(calls) == 1:
                release.wait(5.0)

        with mock.patch.object(app_module, "_WATCHDOG_OVERDUE_S", 0.05), mock.patch.object(
            controller, "watchdog_tick", side_effect=stuck_tick
        ), mock.patch.object(
            controller, "report_watchdog_overdue", wraps=controller.report_watchdog_overdue
        ) as report, self.assertLogs("mocap_g1.operator.app", level="CRITICAL"):
            with TestClient(app, base_url="http://127.0.0.1:8766") as client:
                self.assertTrue(wait_for(lambda: report.call_count == 1))
                status = client.get("/api/v1/status").json()
                self.assertTrue(status["watchdog"]["running"])
                self.assertTrue(status["watchdog"]["overdue"])
                # Only one tick runs at a time while the first is stuck.
                self.assertEqual(len(calls), 1)
                release.set()
                self.assertTrue(wait_for(lambda: len(calls) >= 3))
                self.assertFalse(app.state.watchdog_task.done())
                status = client.get("/api/v1/status").json()
                self.assertFalse(status["watchdog"]["overdue"])
        overdue = [
            event for event in controller.events_after() if event["action"] == "watchdog_overdue"
        ]
        self.assertEqual(len(overdue), 1)

    def test_runtime_tick_exception_does_not_stop_the_loop(self):
        from fastapi.testclient import TestClient

        app = self.app()
        controller = app.state.controller
        failures = []

        def failing_tick(health):
            failures.append(1)
            raise RuntimeError("adapter exploded")

        with mock.patch.object(controller._runtime, "tick", side_effect=failing_tick):
            with TestClient(app, base_url="http://127.0.0.1:8766") as client:
                self.assertTrue(wait_for(lambda: len(failures) >= 3))
                self.assertFalse(app.state.watchdog_task.done())
                status = client.get("/api/v1/status").json()
                self.assertEqual(status["health"]["overall"], "fault")
                self.assertFalse(status["controls"]["start-tracking"]["enabled"])
        ticks = [e for e in controller.events_after() if e["action"] == "watchdog_tick"]
        # Recorded once, not once per 100 ms tick.
        self.assertEqual(len(ticks), 1)
        self.assertEqual(ticks[0]["details"]["code"], "runtime_watchdog_error")


if __name__ == "__main__":
    unittest.main()
