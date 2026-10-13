import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from tools.xsens.loop_bvh_g1 import (
    _cache_metadata,
    _configure_position_only_arms,
    _load_offsets,
    _partial_video_path,
    _play,
    _read_cache,
    _resolve_arm_mode,
    _runtime_identity,
    _write_cache,
)


class XsensBvhLoopTests(unittest.TestCase):
    def test_runtime_identity_hashes_commit_config_and_model(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            commit = root / ".git" / "HEAD"
            config = (
                root
                / "general_motion_retargeting"
                / "ik_configs"
                / "bvh_xsens_to_g1.json"
            )
            model = root / "assets" / "unitree_g1" / "g1_mocap_29dof.xml"
            package_init = root / "general_motion_retargeting" / "__init__.py"
            retarget_code = root / "general_motion_retargeting" / "motion_retarget.py"
            for path, content in (
                (commit, "abc123\n"),
                (config, "config"),
                (model, "model"),
                (package_init, "package"),
                (retarget_code, "retarget"),
            ):
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(content, encoding="utf-8")
            identity = _runtime_identity(root)
        self.assertEqual(identity["gmr_commit"], "abc123")
        self.assertEqual(len(identity["ik_config_sha256"]), 64)
        self.assertEqual(len(identity["g1_model_sha256"]), 64)
        self.assertEqual(len(identity["package_init_sha256"]), 64)
        self.assertEqual(len(identity["retarget_code_sha256"]), 64)

    def test_offsets_file_does_not_auto_promote_full_orientation(self):
        self.assertEqual(
            _resolve_arm_mode(
                "auto", has_offsets=True, has_calibration_bvh=False, calibration_seconds=0.0
            ),
            "position-only",
        )
        self.assertEqual(
            _resolve_arm_mode(
                "calibrated", has_offsets=True, has_calibration_bvh=False, calibration_seconds=0.0
            ),
            "calibrated",
        )
        self.assertEqual(
            _resolve_arm_mode(
                "auto", has_offsets=False, has_calibration_bvh=True, calibration_seconds=1.0
            ),
            "npose",
        )

    def test_zero_offsets_require_explicit_preview_opt_in(self):
        with self.assertRaisesRegex(ValueError, "allow-zero-offsets"):
            _load_offsets(None, ["Hips", "LeftWrist"], allow_zero=False)
        offsets, key = _load_offsets(None, ["Hips", "LeftWrist"], allow_zero=True)
        np.testing.assert_array_equal(offsets, np.zeros((2, 3)))
        self.assertEqual(key, "zero-preview")

    def test_complete_offsets_are_loaded_in_xyz_order(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "offsets.json"
            path.write_text(
                json.dumps({"Hips": {"X": 1, "Y": 2, "Z": 3}}), encoding="utf-8"
            )
            offsets, key = _load_offsets(path, ["Hips", "Head_end_site"], allow_zero=False)
        np.testing.assert_array_equal(offsets, [[1, 2, 3], [0, 0, 0]])
        self.assertEqual(len(key), 64)

    def test_position_only_mode_disables_arm_orientation_tasks(self):
        arm_entry = ["LeftWrist", 10, 5, [0, 0, 0], [1, 0, 0, 0]]
        pelvis_entry = ["Hips", 100, 5, [0, 0, 0], [1, 0, 0, 0]]
        retargeter = SimpleNamespace(
            ik_match_table1={
                "left_wrist_yaw_link": arm_entry.copy(),
                "pelvis": pelvis_entry.copy(),
            },
            ik_match_table2={"right_elbow_link": arm_entry.copy()},
            setup_retarget_configuration=lambda: setattr(retargeter, "configured", True),
        )
        _configure_position_only_arms(retargeter)
        self.assertEqual(retargeter.ik_match_table1["left_wrist_yaw_link"][2], 0.0)
        self.assertEqual(retargeter.ik_match_table2["right_elbow_link"][2], 0.0)
        self.assertEqual(retargeter.ik_match_table1["pelvis"][2], 5)
        self.assertTrue(retargeter.configured)

    def test_cache_accepts_resolved_metadata_but_rejects_wrong_arm_mode(self):
        args = SimpleNamespace(
            robot="unitree_g1",
            scale=0.01,
            start=1,
            end=None,
            frame_step=4,
            human_height=None,
            resolved_arm_mode="position-only",
            calibration_source_sha256=None,
            calibration_start_seconds=0.0,
            calibration_seconds=0.0,
            gmr_runtime_identity={"gmr_commit": "test"},
        )
        expected = _cache_metadata(args, "source-hash", "zero-preview")
        metadata = {**expected, "resolved_human_height_m": 1.6338}
        qpos = np.zeros((3, 36), dtype=np.float64)
        qpos[:, 3] = 1.0
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "motion.npz"
            _write_cache(path, metadata, qpos, 60.0)
            loaded = _read_cache(path, expected)
            self.assertIsNotNone(loaded)
            stale = {**expected, "arm_mode": "calibrated"}
            self.assertIsNone(_read_cache(path, stale))

    def test_boolean_offset_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "offsets.json"
            path.write_text(
                json.dumps({"Hips": {"X": True, "Y": 0, "Z": 0}}), encoding="utf-8"
            )
            with self.assertRaisesRegex(ValueError, "finite numeric"):
                _load_offsets(path, ["Hips"], allow_zero=False)

    def test_partial_video_keeps_mp4_suffix_for_ffmpeg(self):
        self.assertEqual(
            _partial_video_path(Path("/tmp/review.mp4")),
            Path("/tmp/.review.partial.mp4"),
        )

    def test_video_is_atomically_promoted_only_after_all_frames(self):
        class FakeViewer:
            def __init__(self, **kwargs):
                self.video_path = Path(kwargs["video_path"])
                self.viewer = self

            def is_running(self):
                return True

            def step(self, **_kwargs):
                return None

            def close(self):
                self.video_path.write_bytes(b"complete-video")

        qpos = np.zeros((3, 36), dtype=np.float64)
        qpos[:, 3] = 1.0
        with tempfile.TemporaryDirectory() as temporary:
            final = Path(temporary) / "review.mp4"
            args = SimpleNamespace(
                playback_fps=None,
                loop_mode="repeat",
                robot="unitree_g1",
                loops=1,
                record_video=True,
                video_path=final,
                video_frame_step=1,
                video_width=640,
                video_height=480,
            )
            module = SimpleNamespace(RobotMotionViewer=FakeViewer)
            with patch.dict("sys.modules", {"general_motion_retargeting": module}):
                _play(args, qpos, 60.0)
            self.assertEqual(final.read_bytes(), b"complete-video")
            self.assertFalse(_partial_video_path(final).exists())

    def test_interrupted_video_remains_partial_and_fails(self):
        class ClosedViewer:
            def __init__(self, **kwargs):
                self.video_path = Path(kwargs["video_path"])
                self.viewer = self

            def is_running(self):
                return False

            def close(self):
                self.video_path.write_bytes(b"partial-video")

        qpos = np.zeros((2, 36), dtype=np.float64)
        qpos[:, 3] = 1.0
        with tempfile.TemporaryDirectory() as temporary:
            final = Path(temporary) / "review.mp4"
            args = SimpleNamespace(
                playback_fps=None,
                loop_mode="repeat",
                robot="unitree_g1",
                loops=1,
                record_video=True,
                video_path=final,
                video_frame_step=1,
                video_width=640,
                video_height=480,
            )
            module = SimpleNamespace(RobotMotionViewer=ClosedViewer)
            with patch.dict("sys.modules", {"general_motion_retargeting": module}):
                with self.assertRaisesRegex(RuntimeError, "0/2 frames"):
                    _play(args, qpos, 60.0)
            self.assertFalse(final.exists())
            self.assertEqual(_partial_video_path(final).read_bytes(), b"partial-video")

    def test_video_close_failure_is_not_promoted(self):
        class CloseFailureViewer:
            def __init__(self, **kwargs):
                self.video_path = Path(kwargs["video_path"])
                self.viewer = self

            def is_running(self):
                return True

            def step(self, **_kwargs):
                return None

            def close(self):
                self.video_path.write_bytes(b"unfinalized-video")
                raise RuntimeError("writer close failed")

        qpos = np.zeros((1, 36), dtype=np.float64)
        qpos[:, 3] = 1.0
        with tempfile.TemporaryDirectory() as temporary:
            final = Path(temporary) / "review.mp4"
            args = SimpleNamespace(
                playback_fps=None,
                loop_mode="repeat",
                robot="unitree_g1",
                loops=1,
                record_video=True,
                video_path=final,
                video_frame_step=1,
                video_width=640,
                video_height=480,
            )
            module = SimpleNamespace(RobotMotionViewer=CloseFailureViewer)
            with patch.dict("sys.modules", {"general_motion_retargeting": module}):
                with self.assertRaisesRegex(RuntimeError, "1/1 frames"):
                    _play(args, qpos, 60.0)
            self.assertFalse(final.exists())
            self.assertEqual(
                _partial_video_path(final).read_bytes(), b"unfinalized-video"
            )


if __name__ == "__main__":
    unittest.main()
