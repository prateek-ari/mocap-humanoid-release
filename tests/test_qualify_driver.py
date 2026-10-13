from __future__ import annotations

from contextlib import contextmanager
from contextlib import redirect_stderr
import hashlib
import io
import json
from pathlib import Path
import pickle
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from mocap_g1.schema import G1_29DOF_JOINTS
from mocap_g1.window_audit import MotionArrays, load_npz_motion
from tools.convert_npz_to_gmr_pkl import ConversionError, convert_motion, read_npz_metadata
from mocap_g1.io_utils import git_worktree_dirty
from tools import qualify_g1_recording as qualify_driver


REPO_ROOT = Path(__file__).resolve().parents[1]
DRIVER = REPO_ROOT / "tools/qualify_g1_recording.py"
CONVERTER = REPO_ROOT / "tools/convert_npz_to_gmr_pkl.py"

VALID_STATUSES = {"PASS", "BLOCKED", "FAIL"}
STATUS_RANK = {"PASS": 0, "BLOCKED": 1, "FAIL": 2}
EXPECTED_STAGES = [
    "window_audit",
    "conditioning",
    "contact_refinement",
    "self_collision_gate",
    "controller_screen",
    "balance_screen",
    "npz_to_pkl",
    "phuma_gate_v2",
]
EXPECTED_STAGES_WITHOUT_REFINEMENT = [
    "window_audit",
    "conditioning",
    "self_collision_gate",
    "controller_screen",
    "balance_screen",
    "npz_to_pkl",
    "phuma_gate_v2",
]
STAGE_ORDERS = {name: index for index, name in enumerate(EXPECTED_STAGES, start=1)}


def build_trajectory(frames: int = 181, fps: float = 60.0) -> np.ndarray:
    """A tiny, deliberately tame synthetic G1 qpos trajectory.

    Small enough amplitude/frequency (0.03 rad @ 0.5 Hz on every joint, slow
    root drift/yaw) to stay comfortably inside every hardening stage's
    default velocity/acceleration/collision margins -- modeled on the
    proven-safe fixtures already used by tests/test_conditioning.py's
    ``trajectory()`` and tests/test_controller_screen.py's ``reference()``.
    (0.05 rad was tried first and measurably self-collides left_shoulder_yaw
    vs. torso for ~11% of frames on the real G1 MJCF -- a genuine, if tiny,
    geometric margin near this repo's neutral pose, not a driver bug; 0.03
    rad clears it and gives a cleaner example run.)
    """
    time = np.arange(frames) / fps
    qpos = np.zeros((frames, 36), dtype=np.float64)
    qpos[:, 0] = 0.02 * time
    angle = 0.1 * time
    qpos[:, 3] = np.cos(angle / 2.0)  # w
    qpos[:, 6] = np.sin(angle / 2.0)  # z
    qpos[:, 7:] = 0.03 * np.sin(
        2.0 * np.pi * 0.5 * time[:, None] + np.arange(29)[None, :] * 0.01
    )
    return qpos


def write_recording_npz(path: Path, *, frames: int = 181, fps: float = 60.0,
                        offsets_key: str = "zero-preview") -> np.ndarray:
    qpos = build_trajectory(frames=frames, fps=fps)
    metadata = {
        "robot": "unitree_g1",
        "root_quaternion_order": "wxyz",
        "offsets_key": offsets_key,
        "cache_version": 2,
    }
    np.savez_compressed(
        path,
        metadata=np.asarray(json.dumps(metadata, sort_keys=True)),
        qpos=qpos,
        fps=np.asarray(fps, dtype=np.float64),
    )
    return qpos


def controller_report(screen_status: str, screen_reasons: list[str],
                      hardware_reasons: list[str]) -> dict:
    """Minimal stage-5 report in the shape mocap_g1.controller_screen emits."""
    return {
        "controller_screen": {"status": screen_status, "reasons": screen_reasons},
        "hardware_candidate": {
            "status": "BLOCKED" if hardware_reasons else "CONTROLLER_SCREEN_ONLY",
            "reasons": hardware_reasons,
        },
    }


def balance_report(screen_status: str, screen_reasons: list[str],
                   hardware_reasons: list[str], schema_version: int = 2) -> dict:
    """Minimal stage-6 report in the shape mocap_g1.balance_screen emits."""
    return {
        "schema_version": schema_version,
        "balance_screen": {"status": screen_status, "reasons": screen_reasons},
        "hardware_candidate": {
            "status": "BLOCKED" if hardware_reasons else "BALANCE_SCREEN_ONLY",
            "reasons": hardware_reasons,
        },
    }


class ConverterUnitTests(unittest.TestCase):
    """Deliverable B: npz->pkl bridge round-trip / convention tests."""

    def _motion(self, *, root_pos, root_rot_wxyz, dof_pos, fps) -> MotionArrays:
        return MotionArrays(
            root_pos=np.asarray(root_pos, dtype=np.float64),
            root_rot=np.asarray(root_rot_wxyz, dtype=np.float64),
            dof_pos=np.asarray(dof_pos, dtype=np.float64),
            fps=float(fps),
            input_layout="qpos",
        )

    def test_quaternion_reorder_wxyz_to_xyzw(self):
        # Distinct per-component values (not unit-norm; convert_motion does
        # not re-validate norm, that already happened at NPZ-load time) so a
        # wrong permutation is unambiguously detectable.
        motion = self._motion(
            root_pos=[[1.0, 2.0, 3.0]],
            root_rot_wxyz=[[0.1, 0.2, 0.3, 0.4]],  # w, x, y, z
            dof_pos=[[0.0] * len(G1_29DOF_JOINTS)],
            fps=60.0,
        )
        pkl, _notes = convert_motion(motion, source_metadata={"root_quaternion_order": "wxyz"})
        np.testing.assert_array_equal(pkl["root_rot"][0], [0.2, 0.3, 0.4, 0.1])  # x, y, z, w
        np.testing.assert_array_equal(pkl["root_pos"][0], [1.0, 2.0, 3.0])

    def test_dof_pos_frame_count_order_and_dtype_are_preserved(self):
        frames = 12
        dof = np.arange(frames * len(G1_29DOF_JOINTS), dtype=np.float64).reshape(
            frames, len(G1_29DOF_JOINTS)
        )
        motion = self._motion(
            root_pos=np.zeros((frames, 3)),
            root_rot_wxyz=np.tile([1.0, 0.0, 0.0, 0.0], (frames, 1)),
            dof_pos=dof,
            fps=60.0,
        )
        pkl, notes = convert_motion(motion)
        self.assertEqual(pkl["dof_pos"].shape, (frames, len(G1_29DOF_JOINTS)))
        np.testing.assert_array_equal(pkl["dof_pos"], dof)  # identity reorder
        self.assertEqual(pkl["dof_pos"].dtype, np.float64)
        self.assertEqual(pkl["root_pos"].dtype, np.float64)
        self.assertEqual(pkl["root_rot"].dtype, np.float64)
        self.assertEqual(notes["frames"], frames)
        self.assertEqual(notes["joint_count"], len(G1_29DOF_JOINTS))

    def test_fps_is_rounded_not_truncated(self):
        motion = self._motion(
            root_pos=np.zeros((4, 3)), root_rot_wxyz=np.tile([1.0, 0.0, 0.0, 0.0], (4, 1)),
            dof_pos=np.zeros((4, len(G1_29DOF_JOINTS))), fps=59.97,
        )
        pkl, notes = convert_motion(motion)
        self.assertIsInstance(pkl["fps"], int)
        self.assertEqual(pkl["fps"], 60)  # round(), not the truncating int() score_motion.py uses
        self.assertIsNotNone(notes["fps_rounding_note"])

    def test_fps_whose_rounding_rescales_the_timebase_is_refused(self):
        # 60/7 Hz (a --decimate 7 result) would round to 9 and rescale every
        # velocity-based gate-v2 metric by 5 %; the converter refuses instead.
        for fps in (60.0 / 7.0, 15.5, 0.4):
            with self.subTest(fps=fps):
                motion = self._motion(
                    root_pos=np.zeros((4, 3)),
                    root_rot_wxyz=np.tile([1.0, 0.0, 0.0, 0.0], (4, 1)),
                    dof_pos=np.zeros((4, len(G1_29DOF_JOINTS))), fps=fps,
                )
                with self.assertRaisesRegex(ConversionError, "fps"):
                    convert_motion(motion)

    def test_joint_names_are_written_in_column_order(self):
        motion = self._motion(
            root_pos=np.zeros((2, 3)), root_rot_wxyz=np.tile([1.0, 0.0, 0.0, 0.0], (2, 1)),
            dof_pos=np.zeros((2, len(G1_29DOF_JOINTS))), fps=60.0,
        )
        pkl, notes = convert_motion(motion)
        self.assertEqual(pkl["joint_names"], list(G1_29DOF_JOINTS))
        self.assertEqual(notes["joint_names"], list(G1_29DOF_JOINTS))
        reordered = list(G1_29DOF_JOINTS)
        reordered[0], reordered[1] = reordered[1], reordered[0]
        with self.assertRaisesRegex(ConversionError, "joint_names"):
            convert_motion(motion, source_metadata={"joint_names": reordered})

    def test_exact_integer_fps_has_no_rounding_note(self):
        motion = self._motion(
            root_pos=np.zeros((4, 3)), root_rot_wxyz=np.tile([1.0, 0.0, 0.0, 0.0], (4, 1)),
            dof_pos=np.zeros((4, len(G1_29DOF_JOINTS))), fps=30.0,
        )
        pkl, notes = convert_motion(motion)
        self.assertEqual(pkl["fps"], 30)
        self.assertIsNone(notes["fps_rounding_note"])

    def test_rejects_non_wxyz_declared_metadata(self):
        motion = self._motion(
            root_pos=np.zeros((4, 3)), root_rot_wxyz=np.tile([1.0, 0.0, 0.0, 0.0], (4, 1)),
            dof_pos=np.zeros((4, len(G1_29DOF_JOINTS))), fps=30.0,
        )
        with self.assertRaisesRegex(ConversionError, "root_quaternion_order"):
            convert_motion(motion, source_metadata={"root_quaternion_order": "xyzw"})

    def test_cli_round_trip_matches_source_npz(self):
        with _tmp_dir() as root:
            source = root / "source.npz"
            qpos = write_recording_npz(source, frames=40, fps=60.0)
            output = root / "clip.pkl"
            report = root / "report.json"
            completed = subprocess.run(
                [sys.executable, str(CONVERTER), str(source), str(output), "--report", str(report)],
                cwd=REPO_ROOT, capture_output=True, text=True, timeout=60,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertTrue(output.is_file())
            self.assertTrue(report.is_file())

            with output.open("rb") as handle:
                pkl = pickle.load(handle)
            self.assertEqual(
                set(pkl),
                {"fps", "root_pos", "root_rot", "root_quat_order", "dof_pos", "joint_names"},
            )
            self.assertEqual(pkl["joint_names"], list(G1_29DOF_JOINTS))
            self.assertEqual(pkl["root_quat_order"], "xyzw")
            self.assertEqual(pkl["fps"], 60)
            self.assertEqual(pkl["dof_pos"].shape, (40, len(G1_29DOF_JOINTS)))
            np.testing.assert_allclose(pkl["dof_pos"], qpos[:, 7:])
            np.testing.assert_allclose(pkl["root_pos"], qpos[:, 0:3])
            # xyzw -> wxyz should recover the original wxyz quaternions exactly.
            recovered_wxyz = pkl["root_rot"][:, [3, 0, 1, 2]]
            np.testing.assert_allclose(recovered_wxyz, qpos[:, 3:7])

            payload = json.loads(report.read_text(encoding="utf-8"))
            self.assertEqual(payload["status"], "PASS")
            self.assertEqual(payload["schema_version"], 2)
            self.assertEqual(payload["input"]["sha256"], hashlib.sha256(source.read_bytes()).hexdigest())

    def test_cli_refuses_to_overwrite_existing_output(self):
        with _tmp_dir() as root:
            source = root / "source.npz"
            write_recording_npz(source, frames=10)
            output = root / "clip.pkl"
            output.write_bytes(b"not a pickle")
            completed = subprocess.run(
                [sys.executable, str(CONVERTER), str(source), str(output)],
                cwd=REPO_ROOT, capture_output=True, text=True, timeout=60,
            )
            self.assertEqual(completed.returncode, 2)
            self.assertIn("refusing to overwrite", completed.stderr)

    def test_read_npz_metadata_roundtrips(self):
        with _tmp_dir() as root:
            source = root / "source.npz"
            write_recording_npz(source, frames=5, offsets_key="reviewed-2026-08-16")
            metadata = read_npz_metadata(source)
            self.assertEqual(metadata["offsets_key"], "reviewed-2026-08-16")
            self.assertEqual(metadata["root_quaternion_order"], "wxyz")

    def test_load_npz_motion_accepts_the_converter_input_layout(self):
        # Sanity check that the compact qpos+fps+metadata layout this test
        # module writes is exactly what mocap_g1.window_audit (used by both
        # the converter and stage 1 of the driver) expects.
        with _tmp_dir() as root:
            source = root / "source.npz"
            write_recording_npz(source, frames=8)
            motion = load_npz_motion(source)
            self.assertEqual(motion.dof_pos.shape, (8, len(G1_29DOF_JOINTS)))
            self.assertEqual(motion.root_rot.shape, (8, 4))


class QualifyDriverRobustnessTests(unittest.TestCase):
    def test_timeout_expired_bytes_are_normalized_for_ledger_reasons(self):
        timeout = subprocess.TimeoutExpired(
            ["fake-stage"], 1.0, output=b"partial stdout", stderr=b"partial stderr"
        )
        with patch.object(qualify_driver.subprocess, "run", side_effect=timeout):
            completed, duration = qualify_driver._run(["fake-stage"], timeout=1.0)

        self.assertEqual(completed.returncode, 124)
        self.assertEqual(completed.stdout, "partial stdout")
        self.assertIn("partial stderr", completed.stderr)
        self.assertIn("timed out after 1.0s", completed.stderr)
        self.assertGreaterEqual(duration, 0.0)

    def test_malformed_stage_reports_block_without_raising(self):
        cases = (
            ("window", [], lambda root: qualify_driver._stage_window_audit(
                root / "capture.npz", root / "window", root / "limits.urdf"
            )),
            ("conditioning", {}, lambda root: qualify_driver._stage_conditioning(
                root / "capture.npz", root / "conditioning", root / "limits.urdf"
            )[0]),
            ("contact", {"status": "REJECT", "summary": []}, lambda root:
                qualify_driver._stage_contact_refinement(
                    root / "conditioned.npz", root / "contact", root / "model.xml",
                    root / "limits.urdf",
                )[0]),
            ("self-collision", {"summary": []}, lambda root:
                qualify_driver._stage_self_collision_gate(
                    root / "conditioned.npz", root / "self-collision", root / "model.xml"
                )),
            ("controller", {"controller_screen": []}, lambda root:
                qualify_driver._stage_controller_screen(
                    root / "conditioned.npz", root / "controller", root / "model.xml",
                    root / "limits.urdf",
                )),
            ("balance", {"schema_version": 2, "balance_screen": []}, lambda root:
                qualify_driver._stage_balance_screen(
                    root / "conditioned.npz", root / "balance", root / "model.xml",
                    root / "limits.urdf",
                )),
            ("converter", [], lambda root: qualify_driver._stage_npz_to_pkl(
                root / "conditioned.npz", root / "converter"
            )[0]),
            ("phuma", {"motions": []}, lambda root: qualify_driver._stage_phuma_gate_v2(
                root / "clip.pkl", root / "phuma", root / "model.xml",
                "unitree_g1", "adaptive", root / "limits.urdf",
            )),
        )
        report_names = {
            "window": "01_window_audit.json",
            "conditioning": "02_conditioning.report.json",
            "contact": "03_contact_refinement.json",
            "self-collision": "04_self_collision_gate.json",
            "controller": "05_controller_screen.json",
            "balance": "06_balance_screen.json",
            "converter": "07_convert.report.json",
            "phuma": "08_phuma_gate_v2.json",
        }
        for name, payload, invoke in cases:
            with self.subTest(stage=name), _tmp_dir() as root:
                stage_dir = root / name
                stage_dir.mkdir()
                report_path = stage_dir / report_names[name]
                report_path.write_text(json.dumps(payload), encoding="utf-8")
                if name == "conditioning":
                    (stage_dir / "02_conditioned.npz").write_bytes(b"not-an-npz")
                elif name == "converter":
                    (stage_dir / "07_clip.pkl").write_bytes(b"not-a-pickle")
                with patch.object(
                    qualify_driver, "_run",
                    return_value=(subprocess.CompletedProcess([], 0), 0.01),
                ):
                    stage = invoke(root)
                self.assertEqual(stage["status"], "BLOCKED")
                self.assertTrue(stage["reasons"])

    def test_status_translation_table_is_the_single_vocabulary_map(self):
        table = qualify_driver.STAGE_STATUS_TRANSLATION
        # Every ledger value is in the ledger vocabulary, and no tool word is
        # ever translated to BLOCKED (BLOCKED is reserved for "could not check").
        for stage, mapping in table.items():
            with self.subTest(stage=stage):
                self.assertIn(stage, qualify_driver.STAGE_ORDER)
                self.assertTrue(set(mapping.values()) <= {"PASS", "FAIL"})
        self.assertEqual(table["contact_refinement"]["REJECT"], "FAIL")
        self.assertEqual(table["balance_screen"], {"PASS": "PASS", "FAIL": "FAIL"})
        self.assertNotIn("BLOCKED", table["balance_screen"])
        self.assertIsNone(qualify_driver._translate_status("balance_screen", "BLOCKED"))
        self.assertIsNone(qualify_driver._translate_status("window_audit", None))

    def test_stage_dict_refuses_an_order_that_disagrees_with_the_stage(self):
        with self.assertRaises(AssertionError):
            qualify_driver._stage_dict("npz_to_pkl", 6, "PASS", [], None, 0, 0.0, {})

    def _stage_with_report(self, stage_fn, report_name: str, payload: dict,
                           returncode: int, *args) -> tuple[dict, list[str]]:
        """Run a real stage wrapper against a stubbed report; return (stage, cmd)."""
        with _tmp_dir() as root:
            out_dir = root / "out"
            out_dir.mkdir()
            (out_dir / report_name).write_text(json.dumps(payload), encoding="utf-8")
            calls: list[list[str]] = []

            def fake_run(cmd, **_kwargs):
                calls.append(cmd)
                return subprocess.CompletedProcess(cmd, returncode), 0.01

            with patch.object(qualify_driver, "_run", side_effect=fake_run):
                result = stage_fn(root / "conditioned.npz", out_dir, root / "model.xml", *args)
            stage = result[0] if isinstance(result, tuple) else result
            return stage, calls[0]

    def _balance_stage(self, payload: dict, returncode: int) -> tuple[dict, list[str]]:
        return self._stage_with_report(
            qualify_driver._stage_balance_screen, "06_balance_screen.json", payload,
            returncode, Path("/limits.urdf"),
        )

    def test_balance_screen_pass(self):
        stage, cmd = self._balance_stage(balance_report("PASS", [], []), 0)
        self.assertEqual((stage["stage"], stage["order"]), ("balance_screen", 6))
        self.assertEqual(stage["status"], "PASS")
        self.assertTrue(stage["gates_verdict"])
        self.assertIn("tools/control/screen_g1_balance.py", " ".join(cmd))
        self.assertEqual(cmd[cmd.index("--limits-urdf") + 1], "/limits.urdf")

    def test_balance_screen_physical_failure_is_fail(self):
        for preview in ([], ["preview_calibration"]):
            with self.subTest(preview=preview):
                stage, _cmd = self._balance_stage(
                    balance_report("FAIL", ["pelvis_tilt_exceeded"],
                                   preview + ["pelvis_tilt_exceeded"]), 1)
                self.assertEqual(stage["status"], "FAIL")
                self.assertEqual(stage["reasons"], ["pelvis_tilt_exceeded"])

    def test_balance_screen_blocked_cases(self):
        cases = (
            ("preview calibration only", balance_report("PASS", [], ["preview_calibration"]), 0),
            ("schema v1 BLOCKED vocabulary",
             balance_report("BLOCKED", ["fall_abort"], ["fall_abort"], schema_version=1), 1),
            ("unknown status", balance_report("BLOCKED", ["fall_abort"], ["fall_abort"]), 1),
            ("crash", balance_report("PASS", [], []), 2),
            ("exit code disagrees", balance_report("PASS", [], []), 1),
        )
        for name, payload, returncode in cases:
            with self.subTest(name):
                stage, _cmd = self._balance_stage(payload, returncode)
                self.assertEqual(stage["status"], "BLOCKED")
                self.assertTrue(stage["reasons"])

    def test_balance_screen_without_model_or_npz_is_blocked(self):
        with _tmp_dir() as root:
            no_model = qualify_driver._stage_balance_screen(
                root / "a.npz", root, None, root / "limits.urdf")
            no_npz = qualify_driver._stage_balance_screen(
                None, root, root / "model.xml", root / "limits.urdf")
        self.assertEqual(no_model["status"], "BLOCKED")
        self.assertEqual(no_npz["status"], "BLOCKED")

    def test_limits_urdf_is_forwarded_to_stages_3_5_and_8(self):
        refine = {"status": "PASS", "summary": {}}
        _stage, cmd = self._stage_with_report(
            qualify_driver._stage_contact_refinement, "03_contact_refinement.json", refine,
            0, Path("/limits.urdf"))
        self.assertEqual(cmd[cmd.index("--limits-urdf") + 1], "/limits.urdf")
        _stage, cmd = self._stage_with_report(
            qualify_driver._stage_controller_screen, "05_controller_screen.json",
            controller_report("PASS", [], []), 0, Path("/limits.urdf"))
        self.assertEqual(cmd[cmd.index("--limits-urdf") + 1], "/limits.urdf")
        with _tmp_dir() as root:
            calls: list[list[str]] = []

            def fake_run(cmd, **_kwargs):
                calls.append(cmd)
                return subprocess.CompletedProcess(cmd, 2), 0.01

            with patch.object(qualify_driver, "_run", side_effect=fake_run):
                qualify_driver._stage_phuma_gate_v2(
                    root / "clip.pkl", root, root / "model.xml", "unitree_g1",
                    "anchored", Path("/limits.urdf"))
        cmd = calls[0]
        self.assertEqual(cmd[cmd.index("--pos-urdf") + 1], "/limits.urdf")
        self.assertEqual(cmd[cmd.index("--contact-policy") + 1], "anchored")

    def test_contact_refinement_reject_is_fail_and_falls_back(self):
        payload = {
            "status": "REJECT",
            "summary": {"penetration_failed_frames": 3, "residual_skating_frames": 0},
            "rejections": [{"reason": "penetration_unrepairable_within_budgets"}],
        }
        with _tmp_dir() as root:
            out_dir = root / "out"
            out_dir.mkdir()
            (out_dir / "03_contact_refinement.json").write_text(
                json.dumps(payload), encoding="utf-8")
            with patch.object(qualify_driver, "_run",
                              return_value=(subprocess.CompletedProcess([], 1), 0.01)):
                stage, refined = qualify_driver._stage_contact_refinement(
                    root / "conditioned.npz", out_dir, root / "model.xml",
                    root / "limits.urdf")
        self.assertEqual(stage["status"], "FAIL")
        self.assertIsNone(refined)
        self.assertEqual(stage["reasons"][0], "penetration_unrepairable_within_budgets")

    def _phuma_stage(self, motion: dict) -> dict:
        with _tmp_dir() as root:
            out_dir = root / "out"
            out_dir.mkdir()
            (out_dir / "08_phuma_gate_v2.json").write_text(
                json.dumps({"motions": {"07_clip": motion}, "failures": {}}),
                encoding="utf-8")
            with patch.object(qualify_driver, "_run",
                              return_value=(subprocess.CompletedProcess([], 0), 0.01)):
                return qualify_driver._stage_phuma_gate_v2(
                    root / "07_clip.pkl", out_dir, root / "model.xml", "unitree_g1",
                    "anchored", root / "limits.urdf")

    def test_phuma_requires_a_verified_joint_order(self):
        gate = {"pass": True, "reasons": []}
        verified = self._phuma_stage(
            {"gate_v2": gate, "joint_order": {"status": "verified"}})
        self.assertEqual(verified["status"], "PASS")
        for joint_order in ({"status": "unverified_legacy_pkl"}, None):
            with self.subTest(joint_order=joint_order):
                motion = {"gate_v2": gate}
                if joint_order is not None:
                    motion["joint_order"] = joint_order
                stage = self._phuma_stage(motion)
                self.assertEqual(stage["status"], "BLOCKED")
                self.assertIn("joint order was not verified", stage["reasons"][-1])
        failed = self._phuma_stage({
            "gate_v2": {"pass": False, "reasons": ["non_skating below 80"]},
            "joint_order": {"status": "verified"},
        })
        self.assertEqual(failed["status"], "FAIL")

    def test_report_status_and_return_code_mismatch_blocks_stage(self):
        with _tmp_dir() as root:
            out_dir = root / "window"
            out_dir.mkdir()
            report = out_dir / "01_window_audit.json"
            report.write_text(
                json.dumps({"summary": {"status": "PASS", "reasons": []}}),
                encoding="utf-8",
            )
            with patch.object(
                qualify_driver, "_run",
                return_value=(subprocess.CompletedProcess([], 1), 0.01),
            ):
                stage = qualify_driver._stage_window_audit(
                    root / "capture.npz", out_dir, root / "limits.urdf"
                )
            self.assertEqual(stage["status"], "BLOCKED")
            self.assertIn("conflicts with subprocess exit code", stage["reasons"][-1])

    def test_self_collision_summary_invariants_fail_closed(self):
        cases = (
            ("zero frames", {"status": "PASS", "frames": 0,
                              "self_collision_frames": 0, "self_collision_fraction": 0.0}),
            ("fractional frames", {"status": "PASS", "frames": 3.0,
                                    "self_collision_frames": 0, "self_collision_fraction": 0.0}),
            ("collision count exceeds frames", {"status": "FAIL", "frames": 3,
                                                  "self_collision_frames": 4,
                                                  "self_collision_fraction": 1.0}),
            ("fraction outside range", {"status": "FAIL", "frames": 3,
                                          "self_collision_frames": 1,
                                          "self_collision_fraction": 1.1}),
            ("fraction disagrees", {"status": "FAIL", "frames": 3,
                                     "self_collision_frames": 1,
                                     "self_collision_fraction": 0.0}),
            ("status disagrees", {"status": "PASS", "frames": 3,
                                   "self_collision_frames": 1,
                                   "self_collision_fraction": 1.0}),
        )
        for name, summary in cases:
            with self.subTest(summary=name), _tmp_dir() as root:
                out_dir = root / "self-collision"
                out_dir.mkdir()
                (out_dir / "04_self_collision_gate.json").write_text(
                    json.dumps({"summary": summary}), encoding="utf-8"
                )
                with patch.object(
                    qualify_driver, "_run",
                    return_value=(subprocess.CompletedProcess([], 0), 0.01),
                ):
                    stage = qualify_driver._stage_self_collision_gate(
                        root / "conditioned.npz", out_dir, root / "model.xml"
                    )
                self.assertEqual(stage["status"], "BLOCKED")
                self.assertTrue(stage["reasons"])

    def test_valid_self_collision_summary_keeps_physical_verdict(self):
        cases = (
            ({"status": "PASS", "frames": 3, "self_collision_frames": 0,
              "self_collision_fraction": 0.0}, "PASS"),
            ({"status": "FAIL", "frames": 3, "self_collision_frames": 1,
              "self_collision_fraction": 1 / 3}, "FAIL"),
        )
        for summary, expected_status in cases:
            with self.subTest(summary=summary), _tmp_dir() as root:
                out_dir = root / "self-collision"
                out_dir.mkdir()
                (out_dir / "04_self_collision_gate.json").write_text(
                    json.dumps({"summary": summary}), encoding="utf-8"
                )
                with patch.object(
                    qualify_driver, "_run",
                    return_value=(
                        subprocess.CompletedProcess(
                            [], 0 if expected_status == "PASS" else 1
                        ),
                        0.01,
                    ),
                ):
                    stage = qualify_driver._stage_self_collision_gate(
                        root / "conditioned.npz", out_dir, root / "model.xml"
                    )
                self.assertEqual(stage["status"], expected_status)

    def _controller_stage(self, screen_status, screen_reasons, hardware_reasons):
        """Run the real stage-5 wrapper against a stubbed controller report."""
        with _tmp_dir() as root:
            out_dir = root / "controller"
            out_dir.mkdir()
            (out_dir / "05_controller_screen.json").write_text(
                json.dumps(controller_report(screen_status, screen_reasons, hardware_reasons)),
                encoding="utf-8",
            )
            returncode = 0 if screen_status == "PASS" else 1
            with patch.object(
                qualify_driver, "_run",
                return_value=(subprocess.CompletedProcess([], returncode), 0.01),
            ):
                return qualify_driver._stage_controller_screen(
                    root / "conditioned.npz", out_dir, root / "model.xml",
                    root / "limits.urdf",
                )

    def test_controller_screen_physical_failure_stays_fail(self):
        # Real reports put preview_calibration and the failure reasons together
        # in hardware_candidate.reasons (controller_screen.py); FAIL must win.
        for reasons in (
            ["torque_saturation"],
            ["tracking_error"],
            ["self_collision"],
            ["torque_saturation", "self_collision"],
        ):
            for preview in ([], ["preview_calibration"]):
                with self.subTest(reasons=reasons, preview=preview):
                    stage = self._controller_stage("FAIL", reasons, preview + reasons)
                    self.assertEqual(stage["status"], "FAIL")
                    self.assertEqual(stage["reasons"], reasons)

    def test_controller_screen_preview_calibration_only_is_blocked(self):
        stage = self._controller_stage("PASS", [], ["preview_calibration"])
        self.assertEqual(stage["status"], "BLOCKED")
        self.assertEqual(stage["reasons"], ["preview_calibration"])

    def test_controller_screen_pass_without_blockers_stays_pass(self):
        stage = self._controller_stage("PASS", [], [])
        self.assertEqual(stage["status"], "PASS")
        self.assertEqual(stage["reasons"], [])


class QualifyDriverEndToEndTests(unittest.TestCase):
    """Deliverable A: full-chain driver + ledger structure."""

    def test_ledger_end_to_end(self):
        with _tmp_dir() as root:
            recording = root / "capture.npz"
            write_recording_npz(recording, frames=181, fps=60.0)
            offsets = root / "offsets.json"
            offsets.write_text(json.dumps({"Hips": {"X": 0, "Y": 0, "Z": 0}}), encoding="utf-8")
            out_dir = root / "qualify-out"

            completed = subprocess.run(
                [
                    sys.executable, str(DRIVER), str(recording),
                    "--offsets", str(offsets), "--out-dir", str(out_dir),
                ],
                cwd=REPO_ROOT, capture_output=True, text=True, timeout=240,
            )
            self.assertIn(completed.returncode, (0, 1), completed.stderr)

            ledger_path = out_dir / "ledger.json"
            self.assertTrue(ledger_path.is_file(), completed.stdout + completed.stderr)
            ledger = json.loads(ledger_path.read_text(encoding="utf-8"))

            # --- schema / versioning -------------------------------------
            self.assertEqual(ledger["ledger_version"], 3)
            self.assertTrue(ledger["config"]["contact_refinement_enabled"])
            self.assertIn("generated_at_utc", ledger)
            self.assertIn("project_git_rev", ledger["driver"])
            self.assertIn(ledger["driver"]["project_git_dirty"], (True, False, None))
            for package in ("numpy", "mujoco"):
                self.assertIsInstance(ledger["environment"][package], str)
            limits_urdf = Path(ledger["config"]["limits_urdf"])
            self.assertEqual(ledger["config"]["limits_urdf_sha256"],
                             hashlib.sha256(limits_urdf.read_bytes()).hexdigest())
            if ledger["config"]["model_path"] is not None:
                self.assertEqual(
                    ledger["config"]["model_sha256"],
                    hashlib.sha256(Path(ledger["config"]["model_path"]).read_bytes()).hexdigest(),
                )
            self.assertEqual(ledger["config"]["contact_policy_requested"], "adaptive")
            refinement = next(s for s in ledger["stages"] if s["stage"] == "contact_refinement")
            self.assertEqual(
                ledger["config"]["contact_policy"],
                "anchored" if refinement["status"] == "PASS" else "adaptive",
            )

            # --- SHA256 correctness ---------------------------------------
            expected_sha256 = hashlib.sha256(recording.read_bytes()).hexdigest()
            self.assertEqual(ledger["input"]["recording_sha256"], expected_sha256)
            self.assertEqual(ledger["input"]["recording_path"], str(recording))

            # --- offsets provenance -----------------------------------------
            self.assertEqual(ledger["input"]["offsets"]["path"], str(offsets))
            self.assertEqual(
                ledger["input"]["offsets"]["sha256"],
                hashlib.sha256(offsets.read_bytes()).hexdigest(),
            )
            # The synthetic fixture declares offsets_key "zero-preview" in
            # its own embedded metadata; the driver should surface it too.
            self.assertEqual(ledger["input"]["offsets"]["key"], "zero-preview")

            # --- stages: all present, in order, each well-formed ----------
            stages = ledger["stages"]
            self.assertEqual([s["stage"] for s in stages], EXPECTED_STAGES)
            self.assertEqual([s["order"] for s in stages], [1, 2, 3, 4, 5, 6, 7, 8])
            for stage in stages:
                with self.subTest(stage=stage["stage"]):
                    self.assertIn(stage["status"], VALID_STATUSES)
                    self.assertIsInstance(stage["reasons"], list)
                    self.assertIn("artifacts", stage)
                    self.assertIn("duration_s", stage)
                    self.assertIn("returncode", stage)
                    # Every stage that launched a subprocess records its
                    # effective limit; the 3 s fixture gets the 2 h floor + 60 s/s.
                    if stage["command"] is None:
                        self.assertIsNone(stage["timeout_s"])
                    else:
                        self.assertGreaterEqual(
                            stage["timeout_s"], qualify_driver.STAGE_TIMEOUT_FLOOR_S)
            self.assertEqual(ledger["config"]["stage_timeouts"]["policy"],
                             "scaled_by_clip_duration")

            # window_audit and conditioning read the raw input directly and
            # never depend on the external MJCF model, so they must always
            # produce a real verdict (never a model-availability BLOCKED).
            by_name = {s["stage"]: s for s in stages}
            self.assertIn(by_name["window_audit"]["status"], {"PASS", "FAIL"})
            self.assertIn(by_name["conditioning"]["status"], {"PASS", "BLOCKED"})

            # --- overall verdict consistency -------------------------------
            # window_audit grades the RAW input and is diagnostic only: it is
            # reported (gates_verdict false) but excluded from the verdict.
            self.assertFalse(by_name["window_audit"]["gates_verdict"])
            self.assertEqual(ledger["verdict"]["diagnostic_stages"], ["window_audit"])
            gating = [s for s in stages if s["gates_verdict"]]
            self.assertEqual([s["stage"] for s in gating], EXPECTED_STAGES[1:])
            worst = max(gating, key=lambda s: STATUS_RANK[s["status"]])
            self.assertEqual(ledger["verdict"]["overall"], worst["status"])
            self.assertEqual(ledger["verdict"]["worst_stage"], worst["stage"])
            self.assertEqual(
                completed.returncode, 0 if ledger["verdict"]["overall"] == "PASS" else 1
            )

            # If conditioning produced an output, downstream artifact-rooted
            # stages must not be BLOCKED purely for "no upstream artifact"
            # reasons (they may still be BLOCKED/FAIL for their own reasons,
            # e.g. no MJCF model resolved in this sandbox, or gate v2's
            # anchored-contact-policy precondition on an ungrounded clip --
            # both are documented, expected outcomes, not driver bugs).
            if by_name["conditioning"]["status"] == "PASS":
                self.assertIsNotNone(by_name["conditioning"]["artifacts"].get("output_npz"))
                for stage_name in ("self_collision_gate", "controller_screen", "balance_screen",
                                   "npz_to_pkl"):
                    reasons = by_name[stage_name]["reasons"]
                    self.assertFalse(
                        any("did not produce an output artifact" in r for r in reasons),
                        f"{stage_name} incorrectly reports a missing-conditioning reason: {reasons}",
                    )

    def test_no_contact_refinement_restores_legacy_chain(self):
        """--no-contact-refinement skips stage 3 entirely.

        Stage `order` values are fixed pipeline positions, so the chain runs
        as 1,2,4,5,6,7,8 with position 3 intentionally vacant.
        """
        with _tmp_dir() as root:
            recording = root / "capture.npz"
            write_recording_npz(recording, frames=60, fps=60.0)
            out_dir = root / "qualify-out"
            completed = subprocess.run(
                [
                    sys.executable, str(DRIVER), str(recording),
                    "--no-contact-refinement", "--out-dir", str(out_dir),
                ],
                cwd=REPO_ROOT, capture_output=True, text=True, timeout=240,
            )
            self.assertIn(completed.returncode, (0, 1), completed.stderr)
            ledger = json.loads((out_dir / "ledger.json").read_text(encoding="utf-8"))
            stages = ledger["stages"]
            self.assertEqual(
                [s["stage"] for s in stages], EXPECTED_STAGES_WITHOUT_REFINEMENT
            )
            self.assertEqual([s["order"] for s in stages], [1, 2, 4, 5, 6, 7, 8])
            self.assertFalse(ledger["config"]["contact_refinement_enabled"])
            self.assertEqual(ledger["config"]["contact_policy"], "adaptive")
            self.assertIn("--no-contact-refinement", ledger["config"]["contact_policy_reason"])

    def _run_with_stub_stages(self, root: Path, window_status: str,
                              phuma_status: str = "PASS",
                              controller_report_payload: dict | None = None,
                              balance_status: str = "PASS",
                              refinement_status: str = "PASS",
                              extra_args: tuple[str, ...] = (),
                              ) -> tuple[int, dict]:
        """Run main() in-process with every stage tool stubbed out.

        With ``controller_report_payload`` the real stage-5 wrapper runs against
        that stubbed controller report instead of a stubbed stage result.
        The stage-8 stub records the contact policy it was called with in
        ``self.phuma_calls``.
        """
        recording = root / "capture.npz"
        write_recording_npz(recording, frames=8)
        out_dir = root / "out"

        def stage(name, status):
            return qualify_driver._stage_dict(
                name, STAGE_ORDERS[name], status,
                [] if status == "PASS" else [f"{name} {status}"],
                None, 0 if status == "PASS" else 1, 0.0, {},
            )

        npz = root / "shared.npz"
        pkl = root / "clip.pkl"
        # Stage 5 is skipped as BLOCKED without a model; any existing file will do.
        model_args: list[str] = []
        if controller_report_payload is not None:
            stub_model = root / "model.xml"
            stub_model.write_text("<mujoco/>", encoding="utf-8")
            model_args = ["--model", str(stub_model)]
        if controller_report_payload is None:
            controller_patch = patch.object(
                qualify_driver, "_stage_controller_screen",
                return_value=stage("controller_screen", "PASS"))
        else:
            screen_status = controller_report_payload["controller_screen"]["status"]

            def run_controller(cmd, **_kwargs):
                report = Path(cmd[cmd.index("--output") + 1])
                report.parent.mkdir(parents=True, exist_ok=True)
                report.write_text(json.dumps(controller_report_payload), encoding="utf-8")
                return subprocess.CompletedProcess(cmd, 0 if screen_status == "PASS" else 1), 0.0

            controller_patch = patch.object(qualify_driver, "_run", side_effect=run_controller)
        self.phuma_calls: list[tuple] = []

        def phuma(*args, **_kwargs):
            self.phuma_calls.append(args)
            return stage("phuma_gate_v2", phuma_status)

        refined = npz if refinement_status == "PASS" else None
        with patch.object(qualify_driver, "_stage_window_audit",
                          return_value=stage("window_audit", window_status)), \
             patch.object(qualify_driver, "_stage_conditioning",
                          return_value=(stage("conditioning", "PASS"), npz)), \
             patch.object(qualify_driver, "_stage_contact_refinement",
                          return_value=(stage("contact_refinement", refinement_status),
                                        refined)), \
             patch.object(qualify_driver, "_stage_self_collision_gate",
                          return_value=stage("self_collision_gate", "PASS")), \
             controller_patch, \
             patch.object(qualify_driver, "_stage_balance_screen",
                          return_value=stage("balance_screen", balance_status)), \
             patch.object(qualify_driver, "_stage_npz_to_pkl",
                          return_value=(stage("npz_to_pkl", "PASS"), pkl)), \
             patch.object(qualify_driver, "_stage_phuma_gate_v2", side_effect=phuma), \
             redirect_stderr(io.StringIO()), \
             patch("sys.stdout", new_callable=io.StringIO):
            result = qualify_driver.main(
                [str(recording), "--out-dir", str(out_dir), *model_args, *extra_args]
            )
        ledger = json.loads((out_dir / "ledger.json").read_text(encoding="utf-8"))
        return result, ledger

    def test_balance_screen_verdicts_gate_the_ledger(self):
        for balance_status, expected_result in (("PASS", 0), ("FAIL", 1), ("BLOCKED", 1)):
            with self.subTest(balance=balance_status), _tmp_dir() as root:
                result, ledger = self._run_with_stub_stages(
                    root, window_status="PASS", balance_status=balance_status)
                balance = next(s for s in ledger["stages"] if s["stage"] == "balance_screen")
                self.assertEqual(balance["order"], 6)
                self.assertTrue(balance["gates_verdict"])
                self.assertEqual(ledger["verdict"]["overall"], balance_status)
                if balance_status != "PASS":
                    self.assertEqual(ledger["verdict"]["worst_stage"], "balance_screen")
                self.assertEqual(result, expected_result)

    def test_contact_policy_is_anchored_only_after_refinement_pass(self):
        with _tmp_dir() as root:
            _result, ledger = self._run_with_stub_stages(root, window_status="PASS")
        self.assertEqual(self.phuma_calls[0][4], "anchored")
        config = ledger["config"]
        self.assertEqual(config["contact_policy"], "anchored")
        self.assertEqual(config["contact_policy_requested"], "adaptive")
        self.assertIn("PASSed", config["contact_policy_reason"])

        for status in ("FAIL", "BLOCKED"):
            with self.subTest(refinement=status), _tmp_dir() as root:
                _result, ledger = self._run_with_stub_stages(
                    root, window_status="PASS", refinement_status=status,
                    extra_args=("--contact-policy", "fixed"))
                self.assertEqual(self.phuma_calls[0][4], "fixed")
                config = ledger["config"]
                self.assertEqual(config["contact_policy"], "fixed")
                self.assertEqual(config["contact_policy_requested"], "fixed")
                self.assertIn("did not PASS", config["contact_policy_reason"])

    def test_ledger_records_provenance_hashes_and_versions(self):
        with _tmp_dir() as root:
            _result, ledger = self._run_with_stub_stages(root, window_status="PASS")
        default_urdf = qualify_driver.DEFAULT_URDF.resolve()
        self.assertEqual(ledger["config"]["limits_urdf"], str(default_urdf))
        self.assertEqual(ledger["config"]["limits_urdf_sha256"],
                         hashlib.sha256(default_urdf.read_bytes()).hexdigest())
        self.assertIn("project_git_dirty", ledger["driver"])
        self.assertEqual(set(ledger["environment"]), {"python", "numpy", "mujoco"})
        # The limits URDF reaches stage 8 (same file stages 2 and 8 grade against).
        self.assertEqual(Path(self.phuma_calls[0][5]), default_urdf)

    def test_git_probes_degrade_to_unknown_outside_a_checkout(self):
        with _tmp_dir() as root:
            self.assertIsNone(git_worktree_dirty(root))
        with patch("mocap_g1.io_utils.subprocess.run", side_effect=OSError("no git")):
            self.assertIsNone(git_worktree_dirty(REPO_ROOT))

    def test_missing_limits_urdf_fails_before_running_stages(self):
        with _tmp_dir() as root:
            recording = root / "capture.npz"
            write_recording_npz(recording, frames=8)
            stderr = io.StringIO()
            with redirect_stderr(stderr), patch.object(
                qualify_driver, "_stage_window_audit",
                side_effect=AssertionError("pipeline must not start"),
            ):
                result = qualify_driver.main([
                    str(recording), "--out-dir", str(root / "out"),
                    "--limits-urdf", str(root / "missing.urdf"),
                ])
        self.assertEqual(result, 2)
        self.assertIn("--limits-urdf", stderr.getvalue())

    def test_window_audit_failure_alone_does_not_fail_the_verdict(self):
        with _tmp_dir() as root:
            result, ledger = self._run_with_stub_stages(root, window_status="FAIL")
        self.assertEqual(result, 0)
        self.assertEqual(ledger["verdict"]["overall"], "PASS")
        self.assertNotEqual(ledger["verdict"]["worst_stage"], "window_audit")
        self.assertEqual(ledger["verdict"]["diagnostic_stages"], ["window_audit"])
        window = ledger["stages"][0]
        self.assertEqual(window["stage"], "window_audit")
        self.assertEqual(window["status"], "FAIL")  # still reported verbatim
        self.assertFalse(window["gates_verdict"])
        self.assertTrue(all(s["gates_verdict"] for s in ledger["stages"][1:]))

    def test_gating_stage_failure_still_fails_the_verdict(self):
        with _tmp_dir() as root:
            result, ledger = self._run_with_stub_stages(
                root, window_status="FAIL", phuma_status="FAIL"
            )
        self.assertEqual(result, 1)
        self.assertEqual(ledger["verdict"]["overall"], "FAIL")
        # The earlier diagnostic FAIL must not win the tie-break.
        self.assertEqual(ledger["verdict"]["worst_stage"], "phuma_gate_v2")

    def test_controller_screen_physical_failure_fails_the_ledger(self):
        payload = controller_report(
            "FAIL", ["torque_saturation"], ["preview_calibration", "torque_saturation"]
        )
        with _tmp_dir() as root:
            result, ledger = self._run_with_stub_stages(
                root, window_status="PASS", controller_report_payload=payload
            )
        controller = next(s for s in ledger["stages"] if s["stage"] == "controller_screen")
        self.assertEqual(controller["status"], "FAIL")
        self.assertEqual(ledger["verdict"]["overall"], "FAIL")
        self.assertEqual(ledger["verdict"]["worst_stage"], "controller_screen")
        self.assertEqual(result, 1)

    def test_controller_screen_preview_calibration_only_blocks_the_ledger(self):
        payload = controller_report("PASS", [], ["preview_calibration"])
        with _tmp_dir() as root:
            result, ledger = self._run_with_stub_stages(
                root, window_status="PASS", controller_report_payload=payload
            )
        controller = next(s for s in ledger["stages"] if s["stage"] == "controller_screen")
        self.assertEqual(controller["status"], "BLOCKED")
        self.assertEqual(ledger["verdict"]["overall"], "BLOCKED")
        self.assertEqual(result, 1)

    def test_missing_recording_fails_closed(self):
        with _tmp_dir() as root:
            missing = root / "does-not-exist.npz"
            out_dir = root / "out"
            completed = subprocess.run(
                [sys.executable, str(DRIVER), str(missing), "--out-dir", str(out_dir)],
                cwd=REPO_ROOT, capture_output=True, text=True, timeout=30,
            )
            self.assertEqual(completed.returncode, 2)
            self.assertFalse((out_dir / "ledger.json").exists())

    def test_explicit_missing_offsets_fail_before_running_stages(self):
        with _tmp_dir() as root:
            recording = root / "capture.npz"
            write_recording_npz(recording, frames=8)
            out_dir = root / "out"
            missing_offsets = root / "missing-offsets.json"
            completed = subprocess.run(
                [
                    sys.executable, str(DRIVER), str(recording),
                    "--offsets", str(missing_offsets), "--out-dir", str(out_dir),
                ],
                cwd=REPO_ROOT, capture_output=True, text=True, timeout=30,
            )
            self.assertEqual(completed.returncode, 2)
            self.assertIn("--offsets file does not exist", completed.stderr)
            self.assertFalse((out_dir / "ledger.json").exists())

    def test_existing_stage_artifacts_fail_closed_before_pipeline(self):
        with _tmp_dir() as root:
            recording = root / "capture.npz"
            write_recording_npz(recording, frames=8)
            out_dir = root / "out"
            out_dir.mkdir()
            for name in qualify_driver.QUALIFICATION_ARTIFACT_NAMES:
                path = out_dir / name
                path.write_text("{}", encoding="utf-8")

            stderr = io.StringIO()
            with redirect_stderr(stderr), patch.object(
                qualify_driver, "_stage_window_audit",
                side_effect=AssertionError("pipeline must not start with stale artifacts"),
            ):
                result = qualify_driver.main([
                    str(recording), "--out-dir", str(out_dir),
                ])

            self.assertEqual(result, 2)
            message = stderr.getvalue()
            self.assertIn("refusing to reuse qualification output directory", message)
            for name in qualify_driver.QUALIFICATION_ARTIFACT_NAMES:
                self.assertIn(name, message)
            self.assertFalse((out_dir / "ledger.json").exists())

    def test_makefile_qualify_target_invokes_the_driver(self):
        completed = subprocess.run(
            ["make", "-n", "qualify", "RECORDING=/tmp/example.npz", "OFFSETS=/tmp/offsets.json"],
            cwd=REPO_ROOT, capture_output=True, text=True, timeout=30,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("tools/qualify_g1_recording.py", completed.stdout)
        self.assertIn("/tmp/example.npz", completed.stdout)
        self.assertIn("--offsets", completed.stdout)
        self.assertIn("/tmp/offsets.json", completed.stdout)


class QualifyDriverTimeoutTests(unittest.TestCase):
    """Decision D1 (2026-10-12): generous, clip-scaled stage timeouts."""

    def test_timeout_formula_scales_with_clip_duration(self):
        floor = qualify_driver.STAGE_TIMEOUT_FLOOR_S
        per_second = qualify_driver.STAGE_TIMEOUT_PER_CLIP_SECOND_S
        self.assertEqual(floor, 7200.0)
        self.assertEqual(per_second, 60.0)
        self.assertEqual(qualify_driver.stage_timeout_s(None), floor)
        self.assertEqual(qualify_driver.stage_timeout_s(0.0), floor)
        # Reference capture: 36 s raw, time-scaled 7.9x to 288 s by conditioning;
        # the old stage 3 took 870 s on it (~24 s per raw second).
        self.assertAlmostEqual(qualify_driver.stage_timeout_s(36.35), floor + 60.0 * 36.35)
        self.assertGreater(qualify_driver.stage_timeout_s(287.84), 20 * 870.0)
        # A 5-minute clip gets 7 h -- >2x even the pre-speed-up per-raw-second rate.
        self.assertEqual(qualify_driver.stage_timeout_s(300.0), 25200.0)
        self.assertGreater(qualify_driver.stage_timeout_s(300.0), 2 * 24.0 * 300.0)
        # The override replaces the scaled value outright.
        self.assertEqual(qualify_driver.stage_timeout_s(300.0, 5.0), 5.0)

    def test_clip_duration_reads_frames_over_fps_and_degrades_to_none(self):
        with _tmp_dir() as root:
            recording = root / "clip.npz"
            write_recording_npz(recording, frames=181, fps=60.0)
            self.assertAlmostEqual(qualify_driver._clip_duration_s(recording), 181 / 60.0)
            garbage = root / "garbage.npz"
            garbage.write_bytes(b"not an npz")
            self.assertIsNone(qualify_driver._clip_duration_s(garbage))
            self.assertIsNone(qualify_driver._clip_duration_s(root / "missing.npz"))
            self.assertIsNone(qualify_driver._clip_duration_s(None))

    def test_stage_timeout_override_rejects_non_positive_values(self):
        parser = qualify_driver.build_parser()
        self.assertEqual(
            parser.parse_args(["clip.npz", "--stage-timeout-s", "90"]).stage_timeout_s, 90.0)
        self.assertIsNone(parser.parse_args(["clip.npz"]).stage_timeout_s)
        for bad in ("0", "-5", "nan", "inf", "soon"):
            with self.subTest(value=bad), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as raised:
                    parser.parse_args(["clip.npz", "--stage-timeout-s", bad])
                self.assertEqual(raised.exception.code, 2)

    def test_timed_out_stage_is_blocked_with_its_limit_in_the_reason(self):
        timeout = subprocess.TimeoutExpired(["refine"], 123.0, output=b"", stderr=b"")
        with _tmp_dir() as root, patch.object(
            qualify_driver.subprocess, "run", side_effect=timeout,
        ) as run:
            stage, refined = qualify_driver._stage_contact_refinement(
                root / "conditioned.npz", root, root / "model.xml", root / "limits.urdf",
                timeout_s=123.0,
            )
        self.assertEqual(run.call_args.kwargs["timeout"], 123.0)
        self.assertEqual(stage["status"], "BLOCKED")
        self.assertIsNone(refined)
        self.assertEqual(stage["returncode"], qualify_driver.TIMEOUT_RETURNCODE)
        self.assertIn("[driver] timed out after 123.0s", stage["reasons"])

    def _run_main_recording_timeouts(self, root: Path, extra_args: tuple[str, ...] = ()):
        """Run main() with every subprocess faked; conditioning emits a 3x longer clip."""
        recording = root / "capture.npz"
        write_recording_npz(recording, frames=181, fps=60.0)
        model = root / "model.xml"
        model.write_text("<mujoco/>", encoding="utf-8")
        seen: dict[str, float] = {}

        def fake_run(cmd, *, timeout):
            tool = Path(cmd[1]).name
            seen[tool] = timeout
            if tool == "condition_g1_motion.py":
                write_recording_npz(Path(cmd[3]), frames=543, fps=60.0)
                report = {key: {} for key in
                          ("input", "config", "limits", "time_scaling", "output")}
                Path(cmd[cmd.index("--report") + 1]).write_text(
                    json.dumps(report), encoding="utf-8")
                return subprocess.CompletedProcess(cmd, 0), 0.0
            return subprocess.CompletedProcess(cmd, 2, stderr="boom"), 0.0

        with patch.object(qualify_driver, "_run", side_effect=fake_run), \
             redirect_stderr(io.StringIO()), \
             patch("sys.stdout", new_callable=io.StringIO):
            qualify_driver.main([str(recording), "--out-dir", str(root / "out"),
                                 "--model", str(model), *extra_args])
        ledger = json.loads((root / "out" / "ledger.json").read_text(encoding="utf-8"))
        return ledger, seen

    def test_ledger_records_scaled_timeouts_per_stage(self):
        floor = qualify_driver.STAGE_TIMEOUT_FLOOR_S
        per_second = qualify_driver.STAGE_TIMEOUT_PER_CLIP_SECOND_S
        with _tmp_dir() as root:
            ledger, seen = self._run_main_recording_timeouts(root)
        input_limit = round(floor + per_second * 181 / 60.0, 3)
        # Stages 3-8 are sized by the (longer) conditioned clip.
        chain_limit = round(floor + per_second * 543 / 60.0, 3)
        by_name = {stage["stage"]: stage for stage in ledger["stages"]}
        self.assertEqual(by_name["window_audit"]["timeout_s"], input_limit)
        self.assertEqual(by_name["conditioning"]["timeout_s"], input_limit)
        for name in ("contact_refinement", "self_collision_gate", "controller_screen",
                     "balance_screen", "npz_to_pkl"):
            with self.subTest(stage=name):
                self.assertEqual(by_name[name]["timeout_s"], chain_limit)
        # Stage 8 never launched (no pkl): no limit applies.
        self.assertIsNone(by_name["phuma_gate_v2"]["command"])
        self.assertIsNone(by_name["phuma_gate_v2"]["timeout_s"])
        self.assertAlmostEqual(seen["refine_g1_contacts.py"], chain_limit, places=3)
        self.assertAlmostEqual(seen["audit_g1_recording.py"], input_limit, places=3)
        config = ledger["config"]["stage_timeouts"]
        self.assertEqual(config["policy"], "scaled_by_clip_duration")
        self.assertIsNone(config["override_s"])
        self.assertEqual(config["floor_s"], floor)
        self.assertEqual(config["per_clip_second_s"], per_second)
        self.assertEqual(config["recording_duration_s"], round(181 / 60.0, 3))
        self.assertEqual(config["conditioned_duration_s"], round(543 / 60.0, 3))
        self.assertEqual(config["timeout_outcome"], "BLOCKED")

    def test_ledger_records_the_override_for_every_stage(self):
        with _tmp_dir() as root:
            ledger, seen = self._run_main_recording_timeouts(
                root, ("--stage-timeout-s", "42.5"))
        self.assertTrue(seen)
        self.assertEqual(set(seen.values()), {42.5})
        launched = [stage for stage in ledger["stages"] if stage["command"] is not None]
        self.assertTrue(launched)
        self.assertEqual({stage["timeout_s"] for stage in launched}, {42.5})
        self.assertEqual(ledger["config"]["stage_timeouts"]["policy"], "override")
        self.assertEqual(ledger["config"]["stage_timeouts"]["override_s"], 42.5)


@contextmanager
def _tmp_dir():
    with tempfile.TemporaryDirectory() as tmp:
        yield Path(tmp)


if __name__ == "__main__":
    unittest.main()
