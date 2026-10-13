"""Hardware-free tests for the T1 and G1 range-preview paths of ``live_g1_sim``.

Reuses the fakes of ``tests/test_live_g1_sim.py``; only the retargeter and
viewer are resized to the T1 mocap layout (nq = 34, 27 hinges).
"""

from __future__ import annotations

import contextlib
import io
import types
import unittest
from unittest import mock

import numpy as np

from mocap_g1.live_robots import LIVE_ROBOTS, T1_MOCAP_JOINT_RANGES
from test_live_g1_sim import FakeRetargeter, FakeViewer, LiveSimHarness, _stream
from tools.xsens import live_g1_sim

T1 = LIVE_ROBOTS["booster_t1_29dof"]
_LOWER = np.array([lower for _name, lower, _upper in T1_MOCAP_JOINT_RANGES])
_UPPER = np.array([upper for _name, _lower, upper in T1_MOCAP_JOINT_RANGES])


class T1FakeRetargeter(FakeRetargeter):
    """Returns a finite in-range 34-length T1 qpos per frame."""

    def retarget(self, frame, offset_to_ground=True):
        assert offset_to_ground is False
        self.calls += 1
        counter = frame[1]
        qpos = np.zeros(T1.nq, dtype=np.float64)
        qpos[0:3] = (0.01 * counter, 0.0, 0.68)
        qpos[3] = 1.0
        middle, half = 0.5 * (_LOWER + _UPPER), 0.5 * (_UPPER - _LOWER)
        qpos[7:] = middle + 0.3 * half * np.sin(0.15 * counter + 0.3 * np.arange(27))
        self.scaled_human_data = {"counter": counter}
        return qpos


class T1FakeViewer(FakeViewer):
    def __init__(self, harness, **kwargs):
        super().__init__(harness, **kwargs)
        self.data = types.SimpleNamespace(qpos=np.zeros(T1.nq, dtype=np.float64))
        self.data.qpos[2] = 0.68
        self.data.qpos[3] = 1.0


class T1SimHarness(LiveSimHarness):
    def _retargeter(self, **kwargs):
        self.retargeter = T1FakeRetargeter(self, **kwargs)
        return self.retargeter

    def _viewer(self, **kwargs):
        self.viewer = T1FakeViewer(self, **kwargs)
        return self.viewer


class RecordingLiveLoop(live_g1_sim.LiveLoop):
    instances: list = []

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.kwargs = kwargs
        RecordingLiveLoop.instances.append(self)


def run_harness(harness):
    RecordingLiveLoop.instances = []
    with mock.patch.object(live_g1_sim, "LiveLoop", RecordingLiveLoop):
        snapshot = harness.run()
    loop = RecordingLiveLoop.instances[-1] if RecordingLiveLoop.instances else None
    return snapshot, loop


class LiveT1SimTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # The "<robot> live:" stats line prints every STATS_INTERVAL_S (2 s), so
        # the run must outlast it.
        cls.harness = T1SimHarness(
            argv=["--robot", "booster_t1_29dof", "--duration", "2.5"],
            script=_stream(range(1, 400)),
            hardware_envelope=False,
        )
        cls.result, cls.loop = run_harness(cls.harness)
        cls.stdout = "\n".join(cls.result["stdout"])

    def test_runs_to_completion_with_the_t1_preview_screening(self):
        self.assertEqual(self.result["exit_code"], 0, self.result["stderr"])
        self.assertIn("T1 live:", self.stdout)
        self.assertNotIn("G1 live:", self.stdout)
        self.assertIn("MuJoCo T1 preview screening", self.stdout)
        self.assertNotIn("waist/arm safety enabled", self.stdout)
        self.assertIn("no robot SDK", self.stdout)
        self.assertNotIn("Retargeting failed", "\n".join(self.result["stderr"]))

    def test_project_ik_config_is_installed_into_gmr(self):
        registry = self.harness.gmr.params.IK_CONFIG_DICT
        self.assertEqual(registry, {"xsens_mvn": {"booster_t1_29dof": str(T1.ik_config)}})
        self.assertEqual(self.result["retargeter"]["kwargs"]["tgt_robot"], "booster_t1_29dof")
        self.assertEqual(self.result["retargeter"]["kwargs"]["src_human"], "xsens_mvn")
        self.assertEqual(self.result["viewer"]["kwargs"]["robot_type"], "booster_t1_29dof")
        self.assertTrue(
            self.result["viewer"]["kwargs"]["video_path"].endswith("t1-live.mp4")
        )

    def test_every_viewer_step_gets_the_27_t1_hinges(self):
        steps = self.harness.viewer.steps
        self.assertGreater(len(steps), 100)
        for index, step in enumerate(steps):
            self.assertEqual(len(step["dof"]), 27, index)
            self.assertTrue(np.isfinite(step["dof"]).all(), index)
        self.assertGreater(self.harness.retargeter.calls, 100)

    def test_loop_uses_the_t1_layout_and_range_preview(self):
        self.assertIsNotNone(self.loop)
        self.assertEqual(self.loop.expected_nq, T1.nq)
        self.assertEqual(self.loop.feet, T1.feet)
        self.assertEqual(self.loop.kwargs["abduction_deg"], T1.arm_abduction_deg)
        self.assertIsInstance(self.loop.safety, live_g1_sim.LiveRangePreviewController)
        self.assertEqual(self.loop.safety.joint_names, T1.dof_joints)


class LiveG1PreviewTests(unittest.TestCase):
    def test_g1_without_hardware_envelope_uses_the_range_preview(self):
        harness = LiveSimHarness(
            argv=["--duration", "1.0"],
            script=_stream(range(1, 200)),
            hardware_envelope=False,
        )
        result, loop = run_harness(harness)
        stdout = "\n".join(result["stdout"])
        self.assertEqual(result["exit_code"], 0, result["stderr"])
        self.assertIn("MuJoCo G1 preview screening", stdout)
        self.assertNotIn("waist/arm safety enabled", stdout)
        self.assertIn("no Unitree SDK", stdout)
        self.assertEqual(harness.gmr.params.IK_CONFIG_DICT, {})  # GMR's own G1 config
        self.assertIsInstance(loop.safety, live_g1_sim.LiveRangePreviewController)
        self.assertEqual(loop.kwargs["abduction_deg"], LIVE_ROBOTS["unitree_g1"].arm_abduction_deg)
        self.assertTrue(all(len(step["dof"]) == 29 for step in harness.viewer.steps))

    def test_g1_hardware_envelope_flag_keeps_the_envelope(self):
        harness = LiveSimHarness(argv=["--duration", "0.3"], script=_stream(range(1, 60)))
        result, loop = run_harness(harness)
        self.assertEqual(result["exit_code"], 0, result["stderr"])
        self.assertIn("MuJoCo waist/arm safety enabled", "\n".join(result["stdout"]))
        self.assertIsInstance(loop.safety, live_g1_sim.LiveG1SafetyController)


class CalibrateAfterArgumentTests(unittest.TestCase):
    def test_negative_or_non_finite_calibrate_after_is_rejected(self):
        for value in ("-1", "-0.5", "nan", "inf"):
            with self.subTest(value=value):
                stderr = io.StringIO()
                with contextlib.redirect_stderr(stderr), self.assertRaises(SystemExit) as raised:
                    live_g1_sim.main(["--calibrate-after", value])
                self.assertEqual(raised.exception.code, 2)
                self.assertIn("--calibrate-after", stderr.getvalue())

    def test_parser_accepts_zero_and_robot_choices(self):
        parser = live_g1_sim.build_parser()
        args = parser.parse_args(["--calibrate-after", "0", "--robot", "booster_t1_29dof"])
        self.assertEqual(args.calibrate_after, 0.0)
        self.assertIsNone(args.arm_alignment)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parser.parse_args(["--robot", "unknown_robot"])


if __name__ == "__main__":
    unittest.main()


class CalibrationFailureTests(unittest.TestCase):
    def test_a_solver_error_during_n_calibration_does_not_end_the_session(self):
        import contextlib
        import io
        from unittest import mock

        loop = live_g1_sim.LiveLoop.__new__(live_g1_sim.LiveLoop)
        loop.arm_alignment = mock.Mock(mode="position-only")
        loop.arm_alignment.calibrate.side_effect = RuntimeError("NoSolutionFound: daqp failed")
        loop.pending_frame = {}
        loop.abduction_deg = 0.0
        loop.feet = ()
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            loop.calibrate_arms()
        self.assertIn("ARM ALIGNMENT failed: NoSolutionFound", stderr.getvalue())
