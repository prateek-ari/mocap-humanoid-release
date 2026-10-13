"""Unit tests for eval/metrics.py: gate v2, contact inference, foot metrics,
the hardened GMR pkl loader, the hard joint-limit count, and the
score_motion.py CLI exit contract.

Gate v2 and the contact helpers are exercised on synthetic inputs so every
precondition can be driven independently; the joint-limit and CLI tests use
the real G1 MJCF (always available in CI via MOCAP_WS).
"""

from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import pickle
import subprocess
import sys
import tempfile
import unittest

import numpy as np

from eval import metrics
from mocap_g1.gmr_pkl import RootQuatOrderError
from mocap_g1.schema import G1_29DOF_JOINTS
from conftest import G1_MJCF as MODEL
from eval.retarget_metrics.robot import dof_column_joint_names, verify_dof_joint_order


REPO_ROOT = Path(__file__).resolve().parents[1]
FPS = 30


def passing_result() -> dict:
    """A score_motion_file-shaped dict that passes every gate v2 check."""
    return {
        "foot_contact": {
            "contact_policy": "anchored",
            "non_floating_pct": 95.0,
            "non_penetration_pct": 95.0,
            "non_skating_pct": 90.0,
            "contact_frames": 150,
            "contact_frac": 0.5,
            "contact_frac_by_foot": {"left": 0.3, "right": 0.3},
        },
        "root_motion": {
            "static_frame_frac": 0.0,
            "root_disp_median_m": 0.01,
            "dof_disp_median_rad": 0.05,
            "accel": {"p99": {"mag": 10.0}, "max": {"mag": 50.0}},
            "frames_over_thresh": {"threshold_mps2": 60.0, "count": 0},
        },
        "joint_feasibility": {"position_hard_violation_frames": 0},
    }


def failing_checks(gate: dict) -> set[str]:
    return {name for name, check in gate["checks"].items() if not check["pass"]}


class GateV2Tests(unittest.TestCase):
    def test_baseline_fixture_passes(self):
        gate = metrics.evaluate_gate_v2(passing_result())
        self.assertTrue(gate["pass"], gate["reasons"])
        self.assertEqual(gate["reasons"], [])
        self.assertIn("contact_coverage", gate["checks"])
        self.assertIn("joint_position_limits", gate["checks"])
        self.assertNotIn("root_accel_paired_delta", gate["checks"])

    def test_single_contact_frame_cannot_carry_nf_np_ns(self):
        result = passing_result()
        result["foot_contact"].update({
            "non_floating_pct": 100.0, "non_penetration_pct": 100.0,
            "non_skating_pct": 100.0, "contact_frames": 1,
            "contact_frac": round(1 / 300, 4),
            "contact_frac_by_foot": {"left": round(1 / 300, 4), "right": 0.0},
        })
        gate = metrics.evaluate_gate_v2(result)
        self.assertFalse(gate["pass"])
        self.assertEqual(failing_checks(gate), {"contact_coverage"})
        self.assertTrue(any("insufficient contact coverage" in r for r in gate["reasons"]))

    def test_one_sparse_foot_fails_coverage(self):
        result = passing_result()
        result["foot_contact"]["contact_frac_by_foot"] = {"left": 0.5, "right": 0.01}
        gate = metrics.evaluate_gate_v2(result)
        self.assertEqual(failing_checks(gate), {"contact_coverage"})
        # The documented defaults, and the knobs that move them.
        self.assertEqual(metrics.GATE_V2_MIN_CONTACT_FRAC, 0.05)
        self.assertEqual(metrics.GATE_V2_MIN_FOOT_CONTACT_FRAC, 0.02)
        self.assertTrue(metrics.evaluate_gate_v2(result, min_foot_contact_frac=0.01)["pass"])

    def test_missing_or_nan_coverage_fails_closed(self):
        for mutate in (
            lambda fc: fc.pop("contact_frac"),
            lambda fc: fc.pop("contact_frac_by_foot"),
            lambda fc: fc.update(contact_frac_by_foot={}),
            lambda fc: fc.update(contact_frac=float("nan")),
            lambda fc: fc.update(contact_frac_by_foot={"left": 0.4, "right": float("nan")}),
        ):
            result = passing_result()
            mutate(result["foot_contact"])
            with self.subTest(foot_contact=result["foot_contact"]):
                self.assertIn("contact_coverage", failing_checks(metrics.evaluate_gate_v2(result)))

    def test_nan_statistics_never_pass_not_degenerate(self):
        for key in ("static_frame_frac", "root_disp_median_m", "dof_disp_median_rad"):
            for bad in (float("nan"), float("inf"), None):
                result = passing_result()
                result["root_motion"][key] = bad
                with self.subTest(key=key, value=bad):
                    gate = metrics.evaluate_gate_v2(result)
                    self.assertIn("not_degenerate", failing_checks(gate))
                    self.assertFalse(gate["pass"])

    def test_nan_numeric_criteria_fail(self):
        result = passing_result()
        result["foot_contact"]["non_skating_pct"] = float("nan")
        result["root_motion"]["accel"]["p99"]["mag"] = float("nan")
        result["root_motion"]["frames_over_thresh"]["count"] = float("nan")
        self.assertEqual(
            failing_checks(metrics.evaluate_gate_v2(result)),
            {"non_skating", "root_accel_p99_mag", "root_accel_frames_over_thresh"},
        )

    def test_joint_position_limit_violations_fail_closed(self):
        result = passing_result()
        result["joint_feasibility"]["position_hard_violation_frames"] = 3
        gate = metrics.evaluate_gate_v2(result)
        self.assertEqual(failing_checks(gate), {"joint_position_limits"})
        self.assertTrue(any("joint_position_limits failed" in r for r in gate["reasons"]))
        for missing in ({}, {"position_hard_violation_frames": None},
                        {"position_hard_violation_frames": 0.0}):
            result = passing_result()
            result["joint_feasibility"] = missing
            with self.subTest(joint_feasibility=missing):
                self.assertIn("joint_position_limits",
                              failing_checks(metrics.evaluate_gate_v2(result)))

    def test_paired_delta_fails_when_it_cannot_be_computed(self):
        result = passing_result()
        for baseline in ({}, {"accel": {}}, {"accel": {"max": {"mag": None}}},
                         {"accel": {"max": {"mag": float("nan")}}},
                         {"warning": "too few frames for root accel/jerk"}):
            with self.subTest(baseline=baseline):
                gate = metrics.evaluate_gate_v2(result, baseline_root_motion=baseline)
                self.assertEqual(failing_checks(gate), {"root_accel_paired_delta"})
                self.assertIsNone(gate["checks"]["root_accel_paired_delta"]["value"])
        missing_candidate = copy.deepcopy(result)
        del missing_candidate["root_motion"]["accel"]["max"]
        gate = metrics.evaluate_gate_v2(
            missing_candidate, baseline_root_motion={"accel": {"max": {"mag": 50.0}}})
        self.assertIn("root_accel_paired_delta", failing_checks(gate))

    def test_paired_delta_grades_a_computable_delta(self):
        result = passing_result()
        ok = metrics.evaluate_gate_v2(
            result, baseline_root_motion={"accel": {"max": {"mag": 48.0}}})
        self.assertTrue(ok["pass"], ok["reasons"])
        self.assertEqual(ok["checks"]["root_accel_paired_delta"]["value"], 2.0)
        worse = metrics.evaluate_gate_v2(
            result, baseline_root_motion={"accel": {"max": {"mag": 40.0}}})
        self.assertEqual(failing_checks(worse), {"root_accel_paired_delta"})

    def test_wrong_policy_still_fails_precondition(self):
        result = passing_result()
        result["foot_contact"]["contact_policy"] = "adaptive"
        self.assertEqual(failing_checks(metrics.evaluate_gate_v2(result)),
                         {"contact_policy_anchored"})


class InferContactTests(unittest.TestCase):
    def test_anchored_requires_small_vertical_speed(self):
        mask, threshold = metrics.infer_contact(np.zeros(20), FPS, policy="anchored")
        self.assertTrue(mask.all())
        self.assertEqual(threshold, 0.03)
        slow = 0.02 - 0.001 * np.arange(10)   # 0.03 m/s, inside the band
        mask, _ = metrics.infer_contact(slow, FPS, policy="anchored")
        self.assertTrue(mask.all())
        fast = 0.02 - 0.01 * np.arange(10)    # 0.3 m/s through the band
        mask, _ = metrics.infer_contact(fast, FPS, policy="anchored")
        self.assertFalse(mask.any())

    def test_fixed_is_height_only(self):
        swinging = 0.01 * np.arange(10)  # 0.3 m/s, starts inside the band
        mask, _ = metrics.infer_contact(swinging, FPS, policy="fixed")
        self.assertEqual(mask.tolist(), [h < 0.03 for h in swinging])

    def test_adaptive_threshold_is_clamped_at_ground(self):
        height = np.full(40, -0.08)  # deep, constant penetration
        mask, threshold = metrics.infer_contact(height, FPS, policy="adaptive")
        self.assertEqual(threshold, 0.0)
        self.assertTrue(mask.all())

    def test_unknown_policy_is_refused(self):
        with self.assertRaisesRegex(ValueError, "contact policy"):
            metrics.infer_contact(np.zeros(4), FPS, policy="guess")


def synthetic_fk(left_low, right_low, left_xy_speed=0.0, right_xy_speed=0.0):
    frames = len(left_low)
    t = np.arange(frames) / FPS
    fk = {"foot_low": {}, "foot_pos": {}, "ncon_self": np.zeros(frames, dtype=int)}
    for side, low, speed in (("left", left_low, left_xy_speed),
                             ("right", right_low, right_xy_speed)):
        fk["foot_low"][side] = np.asarray(low, dtype=float)
        pos = np.zeros((frames, 3))
        pos[:, 0] = speed * t
        fk["foot_pos"][side] = pos
    return fk


class FootContactMetricsTests(unittest.TestCase):
    def test_planted_feet_score_100_with_full_coverage(self):
        fk = synthetic_fk(np.zeros(60), np.zeros(60))
        out = metrics.foot_contact_metrics(fk, FPS, contact_policy="anchored")
        self.assertEqual(out["contact_policy"], "anchored")
        self.assertEqual(out["contact_frames"], 60)
        self.assertEqual(out["contact_frac"], 1.0)
        self.assertEqual(out["contact_frac_by_foot"], {"left": 1.0, "right": 1.0})
        for key in ("non_floating_pct", "non_penetration_pct", "non_skating_pct"):
            self.assertEqual(out[key], 100.0)
        self.assertNotIn("warning", out)

    def test_sliding_stance_foot_fails_non_skating(self):
        fk = synthetic_fk(np.zeros(60), np.full(60, 0.5), left_xy_speed=0.2)
        out = metrics.foot_contact_metrics(fk, FPS, contact_policy="anchored")
        self.assertEqual(out["contact_frames_by_foot"], {"left": 60, "right": 0})
        self.assertEqual(out["non_skating_pct"], 0.0)
        self.assertIn("warning", out)  # right foot never in contact

    def test_one_contact_frame_reads_100_but_is_flagged_and_gated(self):
        left = np.full(300, 0.5)
        left[150] = 0.0
        fk = synthetic_fk(left, np.full(300, 0.5))
        out = metrics.foot_contact_metrics(fk, FPS, contact_policy="fixed")
        # The documented denominator semantics: one frame -> 100 %.
        self.assertEqual(out["contact_frames"], 1)
        self.assertEqual(out["non_penetration_pct"], 100.0)
        self.assertIn("warning", out)
        result = passing_result()
        result["foot_contact"] = {**out, "contact_policy": "anchored"}
        self.assertIn("contact_coverage", failing_checks(metrics.evaluate_gate_v2(result)))

    def test_no_contact_is_undefined_not_perfect(self):
        fk = synthetic_fk(np.full(30, 0.5), np.full(30, 0.5))
        out = metrics.foot_contact_metrics(fk, FPS, contact_policy="anchored")
        self.assertIsNone(out["non_skating_pct"])
        self.assertIn("no contact frames", out["warning"])


def write_pkl(path: Path, **overrides) -> Path:
    frames = overrides.pop("frames", 5)
    motion = {
        "fps": 30,
        "root_pos": np.zeros((frames, 3)),
        "root_rot": np.tile([0.0, 0.0, 0.0, 1.0], (frames, 1)),
        "root_quat_order": "xyzw",
        "dof_pos": np.zeros((frames, 29)),
    }
    for key, value in overrides.items():
        if value is None:
            motion.pop(key, None)
        else:
            motion[key] = value
    with path.open("wb") as handle:
        pickle.dump(motion, handle)
    return path


class LoadGmrPklTests(unittest.TestCase):
    def setUp(self):
        self._directory = tempfile.TemporaryDirectory()
        self.root = Path(self._directory.name)

    def tearDown(self):
        self._directory.cleanup()

    def test_declared_xyzw_is_converted_to_wxyz(self):
        rot = np.tile([0.1, 0.2, 0.3, math.sqrt(1 - 0.14)], (5, 1))
        motion = metrics.load_gmr_pkl(write_pkl(self.root / "ok.pkl", root_rot=rot))
        np.testing.assert_allclose(motion["root_rot_wxyz"][0], [math.sqrt(0.86), 0.1, 0.2, 0.3])
        self.assertEqual(motion["fps"], 30)

    def test_order_contract(self):
        keyless = write_pkl(self.root / "keyless.pkl", root_quat_order=None)
        with self.assertRaisesRegex(RootQuatOrderError, "no 'root_quat_order' key"):
            metrics.load_gmr_pkl(keyless)
        motion = metrics.load_gmr_pkl(keyless, quat_order="xyzw")  # explicit legacy override
        np.testing.assert_array_equal(motion["root_rot_wxyz"][0], [1.0, 0.0, 0.0, 0.0])
        legacy_wxyz = write_pkl(self.root / "legacy_wxyz.pkl", root_quat_order=None,
                                root_rot=np.tile([1.0, 0.0, 0.0, 0.0], (5, 1)))
        motion = metrics.load_gmr_pkl(legacy_wxyz, quat_order="wxyz")
        np.testing.assert_array_equal(motion["root_rot_wxyz"][0], [1.0, 0.0, 0.0, 0.0])
        declared_wxyz = write_pkl(self.root / "wxyz.pkl", root_quat_order="wxyz")
        with self.assertRaisesRegex(RootQuatOrderError, "must be 'xyzw'"):
            metrics.load_gmr_pkl(declared_wxyz)
        with self.assertRaisesRegex(RootQuatOrderError, "conflicts"):
            metrics.load_gmr_pkl(write_pkl(self.root / "c.pkl"), quat_order="wxyz")

    def test_joint_names_are_validated_and_optional(self):
        names = list(G1_29DOF_JOINTS)
        motion = metrics.load_gmr_pkl(write_pkl(self.root / "named.pkl", joint_names=names))
        self.assertEqual(motion["joint_names"], names)
        legacy = metrics.load_gmr_pkl(write_pkl(self.root / "legacy.pkl"))
        self.assertIsNone(legacy["joint_names"])
        for label, bad in (
            ("short", names[:-1]),
            ("duplicate", names[:-1] + [names[0]]),
            ("non-string", names[:-1] + [7]),
            ("not a list", "left_hip_pitch_joint"),
        ):
            with self.subTest(label):
                path = write_pkl(self.root / f"{label}.pkl", joint_names=bad)
                with self.assertRaisesRegex(ValueError, "joint_names"):
                    metrics.load_gmr_pkl(path)

    def test_rejects_non_finite_non_unit_and_inconsistent_arrays(self):
        nan_pos = np.zeros((5, 3))
        nan_pos[2, 1] = np.nan
        nan_rot = np.tile([0.0, 0.0, 0.0, 1.0], (5, 1))
        nan_rot[3, 0] = np.nan
        inf_dof = np.zeros((5, 29))
        inf_dof[0, 4] = np.inf
        cases = {
            "root_pos contains non-finite": {"root_pos": nan_pos},
            "root_rot: contains non-finite": {"root_rot": nan_rot},
            "dof_pos contains non-finite": {"dof_pos": inf_dof},
            "non-unit quaternion at frame 0": {"root_rot": np.tile([0.0, 0.0, 0.0, 2.0], (5, 1))},
            "root_rot has 4 frames": {"root_rot": np.tile([0.0, 0.0, 0.0, 1.0], (4, 1))},
            "dof_pos must have shape": {"dof_pos": np.zeros((4, 29))},
            "fps must be finite": {"fps": 0},
            "missing key 'dof_pos'": {"dof_pos": None},
        }
        for index, (message, overrides) in enumerate(cases.items()):
            with self.subTest(case=message):
                path = write_pkl(self.root / f"bad{index}.pkl", **overrides)
                with self.assertRaisesRegex(ValueError, message):
                    metrics.load_gmr_pkl(path)
        nan_fps = write_pkl(self.root / "nan_fps.pkl", fps=float("nan"))
        with self.assertRaisesRegex(ValueError, "fps"):
            metrics.load_gmr_pkl(nan_fps)


class JointPositionLimitTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not MODEL.is_file():
            raise unittest.SkipTest("external GMR MuJoCo model is unavailable")
        cls.rm = metrics.build_robot("unitree_g1", mjcf_override=str(MODEL))

    def _motion(self, dof: np.ndarray) -> dict:
        return {"fps": FPS, "dof_pos": dof}

    def test_in_range_motion_has_no_hard_violations(self):
        result = metrics.joint_feasibility(self.rm, self._motion(np.zeros((10, 29))))
        self.assertEqual(result["position_hard_violation_frames"], 0)
        self.assertEqual(result["position_hard_violations"], {})

    def test_frames_past_the_mechanical_limit_are_counted(self):
        knee = G1_29DOF_JOINTS.index("left_knee_joint")
        info = next(ji for ji in self.rm.joints if ji.name == "left_knee_joint")
        self.assertEqual(info.pos_source, "urdf")
        dof = np.zeros((10, 29))
        dof[:, knee] = 0.5 * (info.lo + info.hi)
        dof[3:6, knee] = info.hi + 0.01
        dof[7, knee] = info.hi  # exactly on the limit is allowed
        result = metrics.joint_feasibility(self.rm, self._motion(dof))
        self.assertEqual(result["position_hard_violation_frames"], 3)
        record = result["position_hard_violations"]["left_knee_joint"]
        self.assertEqual(record["frames"], 3)
        self.assertAlmostEqual(record["max_excess_rad"], 0.01, places=6)


class JointOrderTests(unittest.TestCase):
    """T3-3: pkl joint_names are checked against the MJCF hinge order."""

    @classmethod
    def setUpClass(cls):
        if not MODEL.is_file():
            raise unittest.SkipTest("external GMR MuJoCo model is unavailable")
        cls.rm = metrics.build_robot("unitree_g1", mjcf_override=str(MODEL))

    def test_mjcf_hinge_order_is_the_canonical_column_order(self):
        self.assertEqual(dof_column_joint_names(self.rm), list(G1_29DOF_JOINTS))

    def test_matching_names_verify(self):
        self.assertTrue(verify_dof_joint_order(self.rm, list(G1_29DOF_JOINTS)))

    def test_legacy_pkl_without_names_is_unverified_not_refused(self):
        self.assertFalse(verify_dof_joint_order(self.rm, None))

    def test_reordered_pkl_names_are_refused(self):
        names = list(G1_29DOF_JOINTS)
        names[15], names[22] = names[22], names[15]  # swap left/right shoulder pitch
        with self.assertRaisesRegex(ValueError, "column 15"):
            verify_dof_joint_order(self.rm, names)

    def test_differently_ordered_mjcf_is_refused(self):
        # Same joints, but the MJCF declares the two arms in the opposite order
        # (simulated by permuting dof columns of the loaded model's table).
        permuted = copy.copy(self.rm)
        permuted.joints = [copy.copy(ji) for ji in self.rm.joints]
        for ji in permuted.joints:
            if 15 <= ji.dof_col < 22:
                ji.dof_col += 7
            elif 22 <= ji.dof_col < 29:
                ji.dof_col -= 7
        with self.assertRaisesRegex(ValueError, "do not match the hinge order"):
            verify_dof_joint_order(permuted, list(G1_29DOF_JOINTS))

    def test_run_fk_pass_refuses_a_mismatched_pkl(self):
        names = list(G1_29DOF_JOINTS)
        names[0], names[6] = names[6], names[0]
        motion = {
            "root_pos": np.zeros((2, 3)),
            "root_rot_wxyz": np.tile([1.0, 0.0, 0.0, 0.0], (2, 1)),
            "dof_pos": np.zeros((2, 29)),
            "joint_names": names,
        }
        with self.assertRaisesRegex(ValueError, "do not match the hinge order"):
            metrics.run_fk_pass(self.rm, motion)


class ScoreMotionCliTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not MODEL.is_file():
            raise unittest.SkipTest("external GMR MuJoCo model is unavailable")

    def _score(self, target: Path, out: Path, *extra: str) -> subprocess.CompletedProcess:
        return subprocess.run(
            [sys.executable, str(REPO_ROOT / "eval/score_motion.py"), str(target),
             "--mjcf", str(MODEL), "--json-only", "--out", str(out), *extra],
            cwd=REPO_ROOT, capture_output=True, text=True, timeout=180,
        )

    def _clip(self, path: Path, **overrides) -> Path:
        frames = 40
        root_pos = np.zeros((frames, 3))
        root_pos[:, 2] = 0.79
        root_pos[:, 0] = 0.01 * np.arange(frames)
        return write_pkl(path, frames=frames, root_pos=root_pos, **overrides)

    def test_partial_failure_exits_nonzero_but_keeps_results(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self._clip(root / "good.pkl")
            self._clip(root / "unlabeled.pkl", root_quat_order=None)
            out = root / "scores.json"
            completed = self._score(root, out, "--gate", "v2", "--contact-policy", "anchored")
            self.assertEqual(completed.returncode, 1, completed.stderr)
            payload = json.loads(out.read_text(encoding="utf-8"))
            self.assertEqual(sorted(payload["motions"]), ["good"])
            # failures and motions share one key space: the pkl stem.
            self.assertIn("root_quat_order", payload["failures"]["unlabeled"])
            self.assertIn("failed to score", completed.stderr)
            checks = payload["motions"]["good"]["gate_v2"]["checks"]
            self.assertIn("contact_coverage", checks)
            self.assertTrue(checks["joint_position_limits"]["pass"])

    def test_all_scored_exits_zero(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            pkl = self._clip(root / "good.pkl")
            completed = self._score(pkl, root / "scores.json")
            self.assertEqual(completed.returncode, 0, completed.stderr)

    def test_joint_names_verified_mismatched_refused_legacy_flagged(self):
        swapped = list(G1_29DOF_JOINTS)
        swapped[3], swapped[9] = swapped[9], swapped[3]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self._clip(root / "named.pkl", joint_names=list(G1_29DOF_JOINTS))
            self._clip(root / "legacy.pkl")
            self._clip(root / "swapped.pkl", joint_names=swapped)
            out = root / "scores.json"
            completed = self._score(root, out, "--gate", "v2")
            self.assertEqual(completed.returncode, 1, completed.stderr)
            payload = json.loads(out.read_text(encoding="utf-8"))
            self.assertEqual(sorted(payload["motions"]), ["legacy", "named"])
            self.assertIn("do not match the hinge order", payload["failures"]["swapped"])
            named = payload["motions"]["named"]
            self.assertEqual(named["joint_order"]["status"], "verified")
            legacy = payload["motions"]["legacy"]
            self.assertEqual(legacy["joint_order"]["status"], "unverified_legacy_pkl")
            self.assertTrue(any("legacy pkl without joint_names" in note
                                for note in legacy["notes"]))

    def test_torque_feasibility_is_labelled_informational_not_gated(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            pkl = self._clip(root / "good.pkl")
            out = root / "scores.json"
            completed = self._score(pkl, out, "--gate", "v2", "--contact-policy", "anchored")
            self.assertEqual(completed.returncode, 0, completed.stderr)
            motion = json.loads(out.read_text(encoding="utf-8"))["motions"]["good"]
        torque = motion["torque_feasibility"]
        self.assertIs(torque["gated"], False)
        self.assertIn("informational and not gated", torque["gate_note"])
        self.assertIn(torque["gate_note"], motion["notes"])
        # No gate-v2 check reads it: the verdict is independent of the metric.
        self.assertFalse(any("torque" in name for name in motion["gate_v2"]["checks"]))

    def test_report_records_limit_sources(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            pkl = self._clip(root / "good.pkl")
            urdf = REPO_ROOT / "eval/assets/g1_29dof_official.urdf"
            out = root / "scores.json"
            completed = self._score(pkl, out, "--pos-urdf", str(urdf))
            self.assertEqual(completed.returncode, 0, completed.stderr)
            provenance = json.loads(out.read_text(encoding="utf-8"))["provenance"]
            self.assertEqual(provenance["position_limits_urdf"]["path"], str(urdf.resolve()))
            self.assertEqual(len(provenance["position_limits_urdf"]["sha256"]), 64)
            self.assertEqual(provenance["mjcf"]["path"], str(MODEL.resolve()))

    def test_non_finite_payload_is_refused_not_written(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            pkl = self._clip(root / "good.pkl")
            out = root / "scores.json"
            completed = self._score(pkl, out, "--gate", "v2", "--gate-nf-min", "nan")
            self.assertEqual(completed.returncode, 1, completed.stderr)
            self.assertIn("not strict JSON", completed.stderr)
            self.assertFalse(out.exists())


class FacadeTests(unittest.TestCase):
    def test_top_level_import_style_matches_package_import(self):
        completed = subprocess.run(
            [sys.executable, "-c",
             "import sys; sys.path.insert(0, 'eval'); import metrics; "
             "print(metrics.evaluate_gate_v2.__module__, metrics.load_gmr_pkl.__module__)"],
            cwd=REPO_ROOT, capture_output=True, text=True, timeout=60,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertEqual(completed.stdout.split(),
                         ["retarget_metrics.gate", "retarget_metrics.io"])
        self.assertTrue(callable(metrics._central_diff))
        self.assertEqual(metrics.score_motion_file.__module__, "eval.retarget_metrics.scoring")


if __name__ == "__main__":
    unittest.main()
