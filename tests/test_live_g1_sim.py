"""Hardware-free tests for ``tools/xsens/live_g1_sim.py``.

The live loop needs a UDP receiver, GMR and a MuJoCo viewer.  These tests run
``main()`` end to end against fakes for exactly those three edges (plus a
deterministic monotonic clock), with the real safety envelope, live filter,
upsampler and latency recorder in between.

``LiveLoopCharacterizationTests`` pins the observable behaviour -- every
viewer command, every printed line, the exit code and the number of clock
reads -- against ``tests/golden/live_g1_sim_characterization.json``, which was
recorded from the pre-refactor ``main()``.  Regenerate it only for an
intended behaviour change: ``MOCAP_UPDATE_GOLDEN=1 python -m pytest
tests/test_live_g1_sim.py``.
"""

from __future__ import annotations

import contextlib
import hashlib
import io
import json
import math
import os
from pathlib import Path
import signal
import sys
import types
import unittest
from unittest import mock

import numpy as np

from mocap_g1.mxtp02 import ProtocolError
from tools.xsens import live_g1_sim

REPO_ROOT = Path(__file__).resolve().parents[1]
GOLDEN = REPO_ROOT / "tests/golden/live_g1_sim_characterization.json"
SOURCE = ("10.0.0.5", 9763)


class FakeClock:
    """Monotonic clock that advances a little on every read.

    Advancing per read makes the recorded behaviour sensitive to the number
    and order of clock reads, not only to the I/O schedule.
    """

    def __init__(self, start: float = 1000.0, per_read: float = 1e-4):
        self.now = start
        self.per_read = per_read
        self.reads = 0

    def __call__(self) -> float:
        self.reads += 1
        self.now += self.per_read
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class FakeBody:
    def __init__(self, counter: int):
        self.sample_counter = counter


class FakeReceiver:
    """Scripted ``LatestBodyReceiver``.

    Script entries: ``("body", counter, delay_s)``, ``("timeout",)`` and
    ``("stop",)`` (delivers SIGINT's handler, then times out).  An exhausted
    script times out forever.
    """

    def __init__(self, harness, script, **kwargs):
        self.harness = harness
        self.script = list(script)
        self.kwargs = kwargs
        self.calls: list[float] = []
        self.closed = 0
        self.unexpected_sources = {("10.0.0.9", 1)}
        self.total_body_packets = 11
        self.total_ignored_packets = 2
        self.total_malformed_packets = 1
        self.total_reordered_body_packets = 0
        self.overload_events = 0
        self.total_discarded_over_budget_packets = 0
        self.max_drain_packets = 64

    def receive_latest(self, timeout: float):
        self.calls.append(round(timeout, 9))
        event = self.script.pop(0) if self.script else ("timeout",)
        if event[0] == "body":
            _kind, counter, delay = event
            self.harness.clock.advance(delay)
            return FakeBody(counter), SOURCE
        if event[0] == "stop":
            live_g1_sim._stop(signal.SIGINT, None)
        self.harness.clock.advance(timeout)
        raise TimeoutError

    def close(self) -> None:
        self.closed += 1


class FakeConverter:
    def __init__(self, harness):
        self.harness = harness

    def convert(self, body):
        if body.sample_counter in self.harness.bad_frames:
            raise ProtocolError(f"bad frame {body.sample_counter}")
        return ("frame", body.sample_counter)


class FakeRetargeter:
    def __init__(self, harness, **kwargs):
        self.harness = harness
        self.kwargs = kwargs
        self.calls = 0
        self.scaled_human_data = None
        harness.clock.advance(harness.construction_delay_s)

    def retarget(self, frame, offset_to_ground=True):
        assert offset_to_ground is False
        self.calls += 1
        counter = frame[1]
        special = self.harness.retarget_specials.get(counter)
        if special == "raise":
            raise RuntimeError(f"solver diverged at {counter}")
        if special == "clock_back":
            # Filter time must increase; a clock step backwards is the only
            # way a finite 36-vector can make the live filter fail.
            self.harness.clock.advance(-0.5)
        qpos = np.zeros(36, dtype=np.float64)
        qpos[0:3] = (0.01 * counter, -0.002 * counter, 0.79)
        qpos[3] = 1.0
        joints = np.arange(29, dtype=np.float64)
        qpos[7:] = 0.25 * np.sin(0.15 * counter + 0.3 * joints)
        if special == "bad_shape":
            qpos = qpos[:35]
        elif special == "nan":
            qpos[10] = math.nan
        elif special == "extreme":
            qpos[7:] = 3.5  # far outside most waist/arm ranges: clamps engage
        self.scaled_human_data = {"counter": counter}
        return qpos


class FakeViewer:
    def __init__(self, harness, **kwargs):
        self.harness = harness
        self.kwargs = {
            key: value for key, value in kwargs.items() if key != "keyboard_callback"
        }
        self.keyboard_callback = kwargs["keyboard_callback"]
        self.data = types.SimpleNamespace(qpos=np.zeros(36, dtype=np.float64))
        self.data.qpos[2] = 0.79
        self.data.qpos[3] = 1.0
        self.steps: list[dict] = []
        self.closed = 0

    def step(self, *, root_pos, root_rot, dof_pos, human_motion_data, rate_limit, follow_camera):
        index = len(self.steps)
        self.steps.append(
            {
                "root": [float(value) for value in np.concatenate([root_pos, root_rot])],
                "dof": [float(value) for value in dof_pos],
                "human": human_motion_data,
                "rate_limit": rate_limit,
                "follow_camera": follow_camera,
            }
        )
        # A kinematic preview: MuJoCo now shows exactly what was commanded.
        self.data.qpos[:3] = root_pos
        self.data.qpos[3:7] = root_rot
        self.data.qpos[7:] = dof_pos
        self.harness.clock.advance(1.0 / self.kwargs["motion_fps"])
        for key in self.harness.key_presses.get(index, ()):
            self.keyboard_callback(ord(key))

    def close(self) -> None:
        self.closed += 1


class LiveSimHarness:
    """Runs ``live_g1_sim.main`` against fakes and returns a snapshot."""

    def __init__(
        self,
        *,
        argv=(),
        script=(),
        bad_frames=(),
        retarget_specials=None,
        key_presses=None,
        construction_delay_s=0.0,
        gmr_available=True,
        hardware_envelope=True,
    ):
        self.argv = list(argv)
        self.envelope_flag = ["--g1-hardware-envelope"] if hardware_envelope else []
        self.script = list(script)
        self.bad_frames = set(bad_frames)
        self.retarget_specials = dict(retarget_specials or {})
        self.key_presses = dict(key_presses or {})
        self.construction_delay_s = construction_delay_s
        self.gmr_available = gmr_available
        self.clock = FakeClock()
        self.receiver: FakeReceiver | None = None
        self.retargeter: FakeRetargeter | None = None
        self.viewer: FakeViewer | None = None

    def _receiver(self, **kwargs):
        self.receiver = FakeReceiver(self, self.script, **kwargs)
        return self.receiver

    def _retargeter(self, **kwargs):
        self.retargeter = FakeRetargeter(self, **kwargs)
        return self.retargeter

    def _viewer(self, **kwargs):
        self.viewer = FakeViewer(self, **kwargs)
        return self.viewer

    def run(self) -> dict:
        gmr = types.ModuleType("general_motion_retargeting")
        gmr.GeneralMotionRetargeting = self._retargeter
        gmr.RobotMotionViewer = self._viewer
        # Both live robots install a project IK config into GMR's registry.
        gmr.params = types.SimpleNamespace(IK_CONFIG_DICT={})
        self.gmr = gmr
        stdout, stderr = io.StringIO(), io.StringIO()
        previous = {
            signum: signal.getsignal(signum) for signum in (signal.SIGINT, signal.SIGTERM)
        }
        try:
            with contextlib.ExitStack() as stack:
                stack.enter_context(
                    mock.patch.dict(
                        sys.modules,
                        {"general_motion_retargeting": gmr if self.gmr_available else None},
                    )
                )
                stack.enter_context(
                    # The golden characterizes the G1 hardware safety envelope path.
                    mock.patch.object(
                        sys, "argv", ["live_g1_sim.py", *self.envelope_flag, *self.argv]
                    )
                )
                stack.enter_context(mock.patch("time.monotonic", self.clock))
                stack.enter_context(
                    mock.patch.object(live_g1_sim, "LatestBodyReceiver", self._receiver)
                )
                stack.enter_context(
                    mock.patch.object(
                        live_g1_sim, "GmrFrameConverter", lambda: FakeConverter(self)
                    )
                )
                stack.enter_context(mock.patch.object(live_g1_sim, "RUNNING", True))
                stack.enter_context(contextlib.redirect_stdout(stdout))
                stack.enter_context(contextlib.redirect_stderr(stderr))
                code = live_g1_sim.main()
        finally:
            for signum, handler in previous.items():
                signal.signal(signum, handler)
        return self.snapshot(code, stdout.getvalue(), stderr.getvalue())

    def snapshot(self, code, stdout, stderr) -> dict:
        steps = self.viewer.steps if self.viewer is not None else []
        rounded = [
            {
                **step,
                "root": [round(value, 7) for value in step["root"]],
                "dof": [round(value, 7) for value in step["dof"]],
            }
            for step in steps
        ]
        trace = json.dumps(rounded, sort_keys=True).encode("utf-8")
        return {
            "exit_code": code,
            "stdout": stdout.splitlines(),
            "stderr": stderr.splitlines(),
            "clock_reads": self.clock.reads,
            "receiver": None
            if self.receiver is None
            else {
                "kwargs": self.receiver.kwargs,
                "timeouts": self.receiver.calls,
                "closed": self.receiver.closed,
            },
            "retargeter": None
            if self.retargeter is None
            else {"kwargs": self.retargeter.kwargs, "calls": self.retargeter.calls},
            "viewer": None
            if self.viewer is None
            else {
                "kwargs": self.viewer.kwargs,
                "closed": self.viewer.closed,
                "steps": len(steps),
                "trace_sha256": hashlib.sha256(trace).hexdigest(),
                "first_steps": rounded[:2],
                "last_step": rounded[-1:] if rounded else [],
            },
        }


def _stream(counters, *, delay=0.004):
    return [("body", counter, delay) for counter in counters]


# Each scenario drives one whole ``main()`` run.  Together they cover startup
# (timeout, rejected frames, operator stop, stale startup frame), frame
# acceptance and drop accounting (gaps, a stream restart), every frame-local
# failure path (protocol, retarget exception, bad shape, NaN, filter error),
# operator keys (Space, R), the periodic stats line and the shutdown report,
# for the default filter, the critically-damped/hold filter and --no-filter.
SCENARIOS = {
    "default_filter": dict(
        argv=["--duration", "2.6"],
        script=[
            *_stream([100]),
            *_stream(range(101, 121)),
            ("timeout",),
            *_stream([123, 124, 125, 126, 130]),
            *_stream([131, 132, 133, 134, 135]),
            ("timeout",),
            ("timeout",),
            *_stream(range(136, 150)),
            *_stream([5, 6, 7]),  # confirmed stream restart
            *_stream(range(8, 40)),
        ],
        bad_frames={110, 133},
        retarget_specials={112: "raise", 125: "bad_shape", 131: "nan", 140: "raise"},
        key_presses={30: " ", 45: " ", 60: "r"},
    ),
    "filter_error_and_reinit": dict(
        argv=["--duration", "1.2", "--control-rate", "50"],
        script=[*_stream(range(1, 20)), *_stream(range(20, 40), delay=0.02)],
        retarget_specials={15: "clock_back"},
    ),
    "critically_damped_hold_start_disabled": dict(
        argv=[
            "--duration",
            "2.2",
            "--filter-type",
            "critically_damped",
            "--command-interpolation",
            "hold",
            "--start-disabled",
            "--no-rate-limit",
            "--human-height",
            "1.8",
            "--source-ip",
            "10.0.0.5",
        ],
        script=[*_stream(range(1, 60)), ("timeout",), *_stream(range(60, 140))],
        key_presses={10: " ", 90: " ", 100: " "},
    ),
    "no_filter": dict(
        argv=["--duration", "2.1", "--no-filter", "--control-rate", "40"],
        script=[*_stream(range(1, 30)), *_stream(range(35, 70)), ("timeout",)],
        bad_frames={12},
        retarget_specials={20: "raise", **{c: "extreme" for c in range(40, 52)}},
        key_presses={25: "r", 26: "x"},
    ),
    "stale_startup_frame_is_discarded": dict(
        argv=["--duration", "0.4"],
        script=_stream(range(1, 30)),
        construction_delay_s=3.0,
    ),
    "operator_stop_mid_run": dict(
        argv=["--duration", "0"],
        script=[*_stream(range(1, 25)), ("stop",)],
    ),
    "startup_rejections_then_stream": dict(
        argv=["--duration", "0.3", "--bind", "127.0.0.1", "--port", "6000"],
        script=[("timeout",), *_stream([1, 2, 3, 4, 5]), *_stream(range(6, 30))],
        bad_frames={1, 2, 3, 4},
    ),
    "startup_timeout": dict(
        argv=["--startup-timeout", "0.6"],
        script=[("timeout",), *_stream([1, 2])],
        bad_frames={1, 2},
    ),
    "operator_stop_during_startup": dict(
        argv=[],
        script=[("timeout",), ("stop",)],
    ),
    "gmr_missing": dict(argv=[], gmr_available=False),
}


def run_scenario(name: str) -> dict:
    return LiveSimHarness(**SCENARIOS[name]).run()


class LiveLoopCharacterizationTests(unittest.TestCase):
    maxDiff = None

    @classmethod
    def setUpClass(cls):
        cls.results = {name: run_scenario(name) for name in SCENARIOS}
        if os.environ.get("MOCAP_UPDATE_GOLDEN") == "1":
            GOLDEN.parent.mkdir(parents=True, exist_ok=True)
            GOLDEN.write_text(
                json.dumps(cls.results, indent=1, sort_keys=True) + "\n", encoding="utf-8"
            )
        cls.golden = json.loads(GOLDEN.read_text(encoding="utf-8"))

    def test_scenarios_match_the_recorded_behaviour(self):
        self.assertEqual(sorted(self.golden), sorted(SCENARIOS))
        for name in SCENARIOS:
            with self.subTest(scenario=name):
                # JSON round trip: tuples become lists, as in the golden file.
                actual = json.loads(json.dumps(self.results[name], sort_keys=True))
                self.assertEqual(actual, self.golden[name])

    def test_scenarios_exercise_the_paths_they_claim(self):
        default = self.results["default_filter"]
        stdout = "\n".join(default["stdout"])
        stderr = "\n".join(default["stderr"])
        self.assertEqual(default["exit_code"], 0)
        self.assertIn("Rejected frame: bad frame 110", stderr)
        self.assertIn("Retargeting failed: solver diverged at 112", stderr)
        self.assertIn("SIM SAFETY rearmed", stdout)
        self.assertIn("G1 live:", stdout)
        self.assertIn("=== live latency", stdout)
        self.assertEqual(default["viewer"]["closed"], 1)
        self.assertEqual(default["receiver"]["closed"], 1)
        self.assertIn(
            "Live filtering failed",
            "\n".join(self.results["filter_error_and_reinit"]["stderr"]),
        )
        self.assertEqual(self.results["startup_timeout"]["exit_code"], 3)
        self.assertEqual(self.results["gmr_missing"]["exit_code"], 2)
        self.assertEqual(self.results["operator_stop_during_startup"]["exit_code"], 0)
        self.assertIsNone(self.results["operator_stop_during_startup"]["viewer"])
        stale = self.results["stale_startup_frame_is_discarded"]
        # The startup frame (counter 1) is never retargeted after a slow start.
        self.assertNotIn({"counter": 1}, [s["human"] for s in stale["viewer"]["first_steps"]])



def _limits():
    from mocap_g1.schema import load_official_joint_limits

    return load_official_joint_limits(live_g1_sim.DEFAULT_LIMITS_URDF)


def _safety_output(reason="tracking", *, reset_required=False, clamped=(), engaged=True):
    return types.SimpleNamespace(
        reason=reason,
        engaged=engaged,
        source_age_s=0.0123,
        blend_weight=0.5,
        reset_required=reset_required,
        clamped_joints=tuple(clamped),
    )


class PendingFrameTests(unittest.TestCase):
    def test_expires_at_the_stale_timeout_boundary(self):
        expired = live_g1_sim.pending_frame_expired
        self.assertFalse(expired(10.0, 10.125, 0.25))
        self.assertTrue(expired(10.0, 10.25, 0.25))
        self.assertTrue(expired(10.0, 13.0, 0.25))


class LiveLoopStatsTests(unittest.TestCase):
    def test_failures_are_all_counted_but_only_the_first_three_print(self):
        stats = live_g1_sim.LiveLoopStats()
        printed = [stats.record_failure(f"failure {index}") for index in range(5)]
        self.assertEqual(printed, ["failure 0", "failure 1", "failure 2", None, None])
        self.assertEqual(stats.failures, 5)

    def test_drop_accounting_handles_gaps_wrap_and_restart(self):
        stats = live_g1_sim.LiveLoopStats()
        for counter in (10, 11, 15, 0xFFFFFFFF, 1, 2):
            stats.accept_counter(counter)
        # 11->15 drops 3; 15->0xFFFFFFFF is a restart (negative delta, 0);
        # 0xFFFFFFFF->1 wraps over 0 (1 dropped).
        self.assertEqual(stats.estimated_drops, 4)
        self.assertEqual(stats.previous_counter, 2)

    def test_periodic_line_fires_every_interval(self):
        stats = live_g1_sim.LiveLoopStats(frames=30, safety_ticks=40, reported_at=100.0)
        self.assertIsNone(stats.periodic_line(now=101.99, started=100.0, safety_reason="x"))
        line = stats.periodic_line(now=102.0, started=100.0, safety_reason="tracking")
        self.assertEqual(
            line,
            "G1 live: 15.0 FPS, frames=30, safety_ticks=40, estimated_drops=0, "
            "failures=0, safety=tracking",
        )
        self.assertEqual(stats.reported_at, 102.0)
        self.assertIsNone(stats.periodic_line(now=103.0, started=100.0, safety_reason="x"))


class SafetyStatusReporterTests(unittest.TestCase):
    def test_status_prints_only_on_change_with_hints(self):
        reporter = live_g1_sim.SafetyStatusReporter()
        first = reporter.status_line(_safety_output(), enabled=True, last_error=None)
        self.assertEqual(
            first,
            "SIM SAFETY tracking: enabled=True, engaged=True, source_age=0.012s, blend=0.500.",
        )
        self.assertIsNone(reporter.status_line(_safety_output(), enabled=True, last_error=None))
        toggled = reporter.status_line(_safety_output(), enabled=False, last_error=None)
        self.assertIn("enabled=False", toggled)
        latched = reporter.status_line(
            _safety_output("hard_timeout", reset_required=True, engaged=False),
            enabled=False,
            last_error="boom",
        )
        self.assertTrue(latched.endswith(" Press R to reset/rearm. Detail: boom."), latched)

    def test_clamp_line_reports_new_clamp_sets_only(self):
        reporter = live_g1_sim.SafetyStatusReporter()
        self.assertIsNone(reporter.clamp_line(()))
        self.assertEqual(
            reporter.clamp_line(("a", "b")), "SIM SAFETY position clamp: a, b"
        )
        self.assertIsNone(reporter.clamp_line(("a", "b")))
        self.assertIsNone(reporter.clamp_line(()))  # clearing is silent
        self.assertEqual(reporter.clamp_line(("a",)), "SIM SAFETY position clamp: a")


class ReferencePipelineTests(unittest.TestCase):
    def setUp(self):
        from mocap_g1.live_filter import (
            CommandUpsampler,
            LatencyRecorder,
            OneEuroConfig,
            UpsamplerConfig,
            make_filter,
        )
        from mocap_g1.live_safety import LiveG1SafetyController

        self.safety = mock.Mock(spec=LiveG1SafetyController)
        self.latency = LatencyRecorder()
        self.make_filter = lambda: make_filter(OneEuroConfig(), 29)
        self.make_upsampler = lambda: CommandUpsampler(
            UpsamplerConfig(command_rate_hz=250.0, source_rate_hz=60.0), 29
        )
        self.clock = FakeClock(start=50.0, per_read=0.001)

    def timeline(self):
        timeline = self.latency.open(self.clock())
        for stage in ("parse", "retarget", "filter"):
            timeline.mark(stage, self.clock())
        return timeline

    def test_without_conditioning_each_reference_is_one_target(self):
        pipeline = live_g1_sim.ReferencePipeline(self.safety, self.latency)
        reference = np.full(29, 0.1)
        self.assertIs(pipeline.condition(reference, self.clock), reference)
        reads = self.clock.reads
        with mock.patch("time.monotonic", self.clock):
            pipeline.submit(
                reference, source_timestamp=49.0, timeline=self.timeline(), clock=self.clock
            )
            pipeline.pump(self.clock)
        self.safety.update_target.assert_called_once_with(reference, source_timestamp=49.0)
        self.assertEqual(self.latency.frames, 1)
        self.assertEqual(self.clock.reads - reads, 5)  # 4 timeline marks + emit; no pump
        self.assertEqual(pipeline.report_lines(), [self.latency.format_report()])

    def test_upsampled_reference_is_emitted_by_the_first_pumped_command(self):
        pipeline = live_g1_sim.ReferencePipeline(
            self.safety,
            self.latency,
            joint_filter=self.make_filter(),
            upsampler=self.make_upsampler(),
        )
        filtered = pipeline.condition(np.full(29, 0.2), self.clock)
        np.testing.assert_allclose(filtered, 0.2)  # first sample passes through
        timeline = self.timeline()
        pipeline.submit(filtered, source_timestamp=49.5, timeline=timeline, clock=self.clock)
        self.safety.update_target.assert_not_called()
        self.assertIs(pipeline.open_timeline, timeline)
        self.clock.advance(0.02)
        pipeline.pump(self.clock)
        self.assertIsNone(pipeline.open_timeline)
        self.assertEqual(self.latency.frames, 1)
        args, kwargs = self.safety.update_target.call_args
        self.assertEqual(kwargs, {"source_timestamp": 49.5})
        self.assertEqual(len(pipeline.report_lines()), 2)  # latency + upsampler

    def test_reset_drops_history_and_open_timeline(self):
        joint_filter = self.make_filter()
        pipeline = live_g1_sim.ReferencePipeline(
            self.safety, self.latency, joint_filter=joint_filter, upsampler=self.make_upsampler()
        )
        pipeline.condition(np.zeros(29), self.clock)
        pipeline.submit(np.zeros(29), source_timestamp=49.0, timeline=self.timeline(), clock=self.clock)
        pipeline.reset()
        self.assertIsNone(pipeline.open_timeline)
        self.assertFalse(joint_filter.initialized)
        self.assertFalse(pipeline.upsampler.has_reference)

    def test_reinitialization_line_only_after_a_gap(self):
        joint_filter = self.make_filter()
        pipeline = live_g1_sim.ReferencePipeline(
            self.safety, self.latency, joint_filter=joint_filter
        )
        pipeline.condition(np.zeros(29), self.clock)
        self.assertEqual(len(pipeline.report_lines()), 1)
        self.clock.advance(5.0)  # beyond max_reference_gap_s
        pipeline.condition(np.zeros(29), self.clock)
        self.assertEqual(
            pipeline.report_lines()[-1],
            "filter: 1 reference-gap reinitializations over 2 samples",
        )


class WaitForFirstFrameTests(unittest.TestCase):
    def harness(self, script, bad=()):
        harness = LiveSimHarness(script=script, bad_frames=bad)
        receiver = harness._receiver()
        return harness, receiver, FakeConverter(harness)

    def test_returns_the_first_convertible_frame(self):
        harness, receiver, converter = self.harness(
            [("timeout",), *_stream([1, 2, 3, 4, 5])], bad={1, 2, 3, 4}
        )
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            startup = live_g1_sim.wait_for_first_frame(
                receiver, converter, timeout_s=5.0, clock=harness.clock, running=lambda: True
            )
        self.assertEqual(startup.frame, ("frame", 5))
        self.assertEqual(startup.body.sample_counter, 5)
        self.assertEqual(startup.source, SOURCE)
        self.assertEqual(stderr.getvalue().count("Rejected startup frame"), 3)

    def test_times_out_and_honours_an_operator_stop(self):
        harness, receiver, converter = self.harness([])
        with self.assertRaises(TimeoutError):
            live_g1_sim.wait_for_first_frame(
                receiver, converter, timeout_s=0.6, clock=harness.clock, running=lambda: True
            )
        self.assertEqual(receiver.calls[:2], [0.25, 0.25])
        harness, receiver, converter = self.harness(_stream([1]))
        self.assertIsNone(
            live_g1_sim.wait_for_first_frame(
                receiver, converter, timeout_s=5.0, clock=harness.clock, running=lambda: False
            )
        )
        self.assertEqual(receiver.calls, [])


class LiveLoopTickTests(unittest.TestCase):
    """Drive ``LiveLoop`` directly, without ``main()``."""

    def make_loop(self, *, script, startup_age_s=0.0, upsample=False, **harness_kwargs):
        from mocap_g1.live_filter import CommandUpsampler, LatencyRecorder, UpsamplerConfig
        from mocap_g1.live_safety import LiveG1SafetyController, LiveOperatorControls

        harness = LiveSimHarness(script=script, **harness_kwargs)
        receiver = harness._receiver()
        controls = LiveOperatorControls()
        viewer = harness._viewer(motion_fps=60.0, keyboard_callback=controls.keyboard_callback)
        retargeter = FakeRetargeter(harness)
        safety = LiveG1SafetyController(_limits())
        pipeline = live_g1_sim.ReferencePipeline(
            safety,
            LatencyRecorder(),
            upsampler=CommandUpsampler(UpsamplerConfig(source_rate_hz=60.0), 29)
            if upsample
            else None,
        )
        startup_time = harness.clock()
        harness.clock.advance(startup_age_s)
        loop = live_g1_sim.LiveLoop(
            receiver=receiver,
            converter=FakeConverter(harness),
            retargeter=retargeter,
            viewer=viewer,
            safety=safety,
            controls=controls,
            pipeline=pipeline,
            startup=live_g1_sim.StartupFrame(FakeBody(1), ("frame", 1), SOURCE, startup_time),
            control_rate_hz=60.0,
            stale_timeout_s=0.10,
            duration_s=0.5,
            clock=harness.clock,
            running=lambda: True,
        )
        loop.start()
        self.clock = harness.clock
        return loop, harness, receiver, retargeter, viewer, controls

    def run_quietly(self, callable_):
        # Latency timelines read time.monotonic themselves.
        with mock.patch("time.monotonic", self.clock), contextlib.redirect_stdout(
            io.StringIO()
        ) as out, contextlib.redirect_stderr(io.StringIO()) as err:
            callable_()
        return out.getvalue(), err.getvalue()

    def test_fresh_startup_frame_is_used_on_the_first_tick(self):
        loop, _harness, receiver, retargeter, viewer, _controls = self.make_loop(
            script=_stream([2])
        )
        self.run_quietly(loop.tick)
        self.assertEqual(retargeter.calls, 1)
        self.assertEqual(receiver.calls, [])  # nothing drained
        self.assertEqual(viewer.steps[-1]["human"], {"counter": 1})
        self.assertEqual(loop.stats.frames, 1)
        self.assertEqual(loop.stats.safety_ticks, 1)

    def test_stale_startup_frame_is_dropped_for_a_fresh_one(self):
        loop, _harness, receiver, retargeter, viewer, _controls = self.make_loop(
            script=_stream([7]), startup_age_s=0.5
        )
        self.run_quietly(loop.tick)
        self.assertEqual(len(receiver.calls), 1)
        self.assertEqual(viewer.steps[-1]["human"], {"counter": 7})
        self.assertEqual(loop.stats.frames, 1)

    def test_frame_local_failures_do_not_stop_the_loop(self):
        loop, _harness, _receiver, _retargeter, viewer, _controls = self.make_loop(
            script=_stream([2, 3, 4, 9]),
            bad_frames={3},
            retarget_specials={1: "raise", 2: "nan"},
        )
        _out, err = self.run_quietly(lambda: [loop.tick() for _ in range(5)])
        self.assertEqual(loop.stats.failures, 3)
        self.assertEqual(loop.stats.frames, 2)  # counters 4 and 9
        self.assertEqual(loop.stats.estimated_drops, 4)  # 5..8
        self.assertEqual(len(viewer.steps), 5)  # the viewer is stepped every tick
        self.assertIn("Retargeting failed: solver diverged at 1", err)
        self.assertIn("Rejected frame: bad frame 3", err)

    def test_rearm_drops_pending_state_and_reports(self):
        loop, _harness, _receiver, retargeter, _viewer, controls = self.make_loop(
            script=_stream([2, 3]), upsample=True
        )
        self.run_quietly(loop.tick)
        controls.keyboard_callback(ord("r"))
        loop.pending_frame = ("frame", 99)  # would be retargeted without the rearm
        out, _err = self.run_quietly(loop.tick)
        self.assertIn("SIM SAFETY rearmed", out)
        self.assertEqual(retargeter.calls, 2)  # 1, then a freshly received 2
        self.assertEqual(loop.last_human_frame, {"counter": 2})

    def test_run_stops_at_the_duration_or_operator_stop(self):
        loop, harness, _receiver, _retargeter, viewer, _controls = self.make_loop(
            script=_stream(range(2, 100))
        )
        self.run_quietly(loop.run)
        # Each tick takes at least one 60 Hz viewer period of fake time.
        self.assertGreaterEqual(harness.clock.now - loop.started, 0.5)
        self.assertTrue(0 < len(viewer.steps) <= 30, len(viewer.steps))
        stopped = {"ticks": 0}

        def running():
            stopped["ticks"] += 1
            return stopped["ticks"] <= 3

        loop.running = running
        loop.duration_s = 0
        before = len(viewer.steps)
        self.run_quietly(loop.run)
        self.assertEqual(len(viewer.steps) - before, 3)


if __name__ == "__main__":
    unittest.main()
