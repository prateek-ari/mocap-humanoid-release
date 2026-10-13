"""Focused contract tests for the robot-generic recorded retarget runner."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from tools.retarget.retarget_robot_recorded import (
    RetargetError,
    _load_offsets,
    _mean_calibration_frame,
    _retarget_all_then_decimate,
    _write_zero_offsets,
    build_parser,
    validate_qpos,
)


BVH = """HIERARCHY
ROOT Hips
{
    OFFSET 0 0 0
    CHANNELS 6 Xposition Yposition Zposition Xrotation Yrotation Zrotation
    JOINT Spine
    {
        OFFSET 0 1 0
        CHANNELS 3 Xrotation Yrotation Zrotation
        End Site
        {
            OFFSET 0 1 0
        }
    }
}
MOTION
Frames: 1
Frame Time: 0.0166667
0 0 0 0 0 0 0 0 0
"""


def test_dynamic_qpos_width_and_quaternion_validation() -> None:
    qpos = np.zeros((3, 11), dtype=np.float64)
    qpos[:, 3] = 1.0
    validate_qpos(qpos, expected_nq=11)

    with pytest.raises(RetargetError, match="shape.*11"):
        validate_qpos(np.zeros((3, 36)), expected_nq=11)

    qpos[1, 3] = 0.5
    with pytest.raises(RetargetError, match="non-unit"):
        validate_qpos(qpos, expected_nq=11)


def test_zero_offsets_are_complete_and_markable(tmp_path: Path) -> None:
    bvh = tmp_path / "recording.bvh"
    bvh.write_text(BVH, encoding="utf-8")
    offsets_path, count = _write_zero_offsets(bvh, tmp_path)
    offsets = json.loads(offsets_path.read_text(encoding="utf-8"))

    assert count == 2
    assert set(offsets) == {"Hips", "Spine"}
    assert all(axis in values for values in offsets.values() for axis in ("X", "Y", "Z"))
    _, loaded_count, all_zero = _load_offsets(offsets_path, bvh)
    assert loaded_count == count
    assert all_zero is True


def test_cli_exposes_explicit_robot_ik_and_frame_controls() -> None:
    args = build_parser().parse_args(
        [
            "--gmr-root", "/opt/GMR",
            "--bvh-file", "motion.bvh",
            "--save-path", "motion.pkl",
            "--robot", "booster_t1_29dof",
            "--frame-step", "4",
            "--allow-zero-offsets",
        ]
    )
    assert args.robot == "booster_t1_29dof"
    assert args.ik_config is None
    assert args.frame_step == 4
    assert args.allow_zero_offsets is True
    assert args.arm_mode is None
    assert args.overwrite is False


def test_cli_exposes_reviewed_npose_calibration_controls() -> None:
    args = build_parser().parse_args(
        [
            "--gmr-root", "/opt/GMR",
            "--bvh-file", "motion.bvh",
            "--save-path", "motion.pkl",
            "--robot", "booster_t1_29dof",
            "--allow-zero-offsets",
            "--arm-mode", "npose",
            "--calibration-bvh-file", "neutral.bvh",
            "--calibration-start-seconds", "5.5",
            "--calibration-seconds", "1.0",
            "--calibration-output", "neutral.json",
            "--accept-reviewed-calibration-window",
            "--human-height", "1.6337895",
        ]
    )
    assert args.arm_mode == "npose"
    assert args.calibration_bvh_file == Path("neutral.bvh")
    assert args.calibration_start_seconds == 5.5
    assert args.calibration_seconds == 1.0
    assert args.calibration_output == Path("neutral.json")
    assert args.accept_reviewed_calibration_window is True
    assert args.human_height == 1.6337895


def test_mean_calibration_frame_uses_median_position_and_quaternion_mean() -> None:
    frames = [
        {"LeftShoulder": (np.asarray([0.0, 1.0, 2.0]), np.asarray([1.0, 0.0, 0.0, 0.0]))},
        {"LeftShoulder": (np.asarray([0.2, 1.2, 2.2]), np.asarray([-1.0, 0.0, 0.0, 0.0]))},
        {"LeftShoulder": (np.asarray([9.0, 9.0, 9.0]), np.asarray([1.0, 0.0, 0.0, 0.0]))},
    ]
    mean = _mean_calibration_frame(frames)
    np.testing.assert_allclose(mean["LeftShoulder"][0], [0.2, 1.2, 2.2])
    assert np.isclose(abs(mean["LeftShoulder"][1][0]), 1.0)
    np.testing.assert_allclose(mean["LeftShoulder"][1][1:], [0.0, 0.0, 0.0])


def test_stateful_ik_consumes_every_source_frame_before_decimation() -> None:
    class StatefulRetargeter:
        def __init__(self) -> None:
            self.seen: list[int] = []

        def retarget(self, frame: dict[str, int]) -> np.ndarray:
            self.seen.append(frame["index"])
            return np.asarray([len(self.seen), frame["index"]], dtype=np.float64)

    retargeter = StatefulRetargeter()
    source, output = _retarget_all_then_decimate(
        retargeter,
        [{"index": index} for index in range(8)],
        frame_step=4,
    )

    assert retargeter.seen == list(range(8))
    np.testing.assert_allclose(source[:, 1], np.arange(8))
    np.testing.assert_allclose(output[:, 1], [0, 4])
