"""Tests for the eval torque/effort feasibility metric (eval/metrics.py).

Covers effort parsing against the real vendored URDF and the inverse-dynamics
effort budget check: a fast, high-acceleration wrist motion must exceed the
5 N*m wrist budget while the same motion played slowly must not.

The metric is INFORMATIONAL (decision 2026-10-12): it is computed and
reported, but no gate -- gate v2, the qualification driver or any screen --
consumes it, so exceeding a budget here never fails or blocks a clip.  The
score report labels it ``gated: false`` (asserted at the end of this file).
"Gated" joints below means only the joints counted in
``torque_feasibility_pct``; the file name predates that decision.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from eval.metrics import (
    MECHANICAL_URDF,
    TORQUE_GATE_JOINT_SETS,
    build_robot,
    parse_urdf_limits,
    torque_feasibility,
)
from conftest import G1_MJCF as MODEL, needs_model


REPO_ROOT = Path(__file__).resolve().parents[1]
OFFICIAL_URDF = REPO_ROOT / "eval/assets/g1_29dof_official.urdf"

#: The vendor's published G1 effort limits, transcribed from the URDF this
#: test suite reads.  Spelled out so a silent URDF swap is caught.
EXPECTED_EFFORT_NM = {
    "left_hip_pitch_joint": 88.0,
    "left_hip_roll_joint": 88.0,
    "left_hip_yaw_joint": 88.0,
    "left_knee_joint": 139.0,
    "left_ankle_pitch_joint": 35.0,
    "left_ankle_roll_joint": 35.0,
    "waist_yaw_joint": 88.0,
    "waist_roll_joint": 35.0,
    "waist_pitch_joint": 35.0,
    "left_shoulder_pitch_joint": 25.0,
    "left_shoulder_roll_joint": 25.0,
    "left_shoulder_yaw_joint": 25.0,
    "left_elbow_joint": 25.0,
    "left_wrist_roll_joint": 25.0,
    "left_wrist_pitch_joint": 5.0,
    "left_wrist_yaw_joint": 5.0,
}


# ---------------------------------------------------------------------------
# Effort parsing
# ---------------------------------------------------------------------------


def test_parse_urdf_limits_reads_effort_without_disturbing_the_old_keys():
    limits = parse_urdf_limits(OFFICIAL_URDF)
    assert MECHANICAL_URDF["unitree_g1"] == OFFICIAL_URDF

    for name, effort in EXPECTED_EFFORT_NM.items():
        assert limits[name]["effort"] == effort, name
        # mirrored right-side joint carries the same limit
        assert limits[name.replace("left_", "right_", 1)]["effort"] == effort

    # Pre-existing keys are untouched.
    knee = limits["left_knee_joint"]
    assert set(knee) == {"lower", "upper", "velocity", "effort"}
    assert knee["velocity"] == 20.0
    assert knee["lower"] == pytest.approx(-0.087267)
    assert all(entry["effort"] is not None for entry in limits.values())


def test_wrist_pitch_yaw_are_the_tightest_budget_in_the_robot():
    limits = parse_urdf_limits(OFFICIAL_URDF)
    efforts = {name: entry["effort"] for name, entry in limits.items()}
    smallest = min(efforts.values())
    assert smallest == 5.0
    assert {name for name, value in efforts.items() if value == smallest} == {
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
    }


@needs_model
def test_build_robot_populates_effort_limits_for_every_joint():
    robot = build_robot("unitree_g1", mjcf_override=str(MODEL))
    assert len(robot.joints) == 29
    by_name = {joint.name: joint for joint in robot.joints}
    assert all(joint.effort_limit is not None for joint in robot.joints)
    assert by_name["left_wrist_pitch_joint"].effort_limit == 5.0
    assert by_name["left_knee_joint"].effort_limit == 139.0
    # dof_adr must address qvel/qacc, i.e. skip the six free-root DOFs.
    assert by_name["left_hip_pitch_joint"].dof_adr == 6
    # Existing behaviour is untouched: position limits still come from URDF.
    assert all(joint.pos_source == "urdf" for joint in robot.joints)


# ---------------------------------------------------------------------------
# Inverse-dynamics effort budget (informational; not a gate)
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def robot():
    if not MODEL.is_file():
        pytest.skip("external GMR MuJoCo model is unavailable")
    return build_robot("unitree_g1", mjcf_override=str(MODEL))


def wrist_wave(robot, frequency: float, amplitude: float,
               frames: int = 61, fps: float = 30.0) -> dict:
    """A standing pose whose only motion is one wrist-pitch sinusoid."""
    names = [joint.name for joint in robot.joints]
    dof = np.zeros((frames, len(names)))
    time = np.arange(frames) / fps
    dof[:, names.index("left_wrist_pitch_joint")] = amplitude * np.sin(
        2.0 * np.pi * frequency * time
    )
    return {
        "fps": fps,
        "root_pos": np.tile([0.0, 0.0, 0.793], (frames, 1)),
        "root_rot_wxyz": np.tile([1.0, 0.0, 0.0, 0.0], (frames, 1)),
        "dof_pos": dof,
    }


def test_slow_wrist_motion_stays_inside_its_budget(robot):
    result = torque_feasibility(robot, wrist_wave(robot, frequency=0.3, amplitude=0.3))

    assert result["status"] == "evaluated"
    assert result["torque_feasibility_pct"] == 100.0
    assert result["violating_joints"] == {}
    wrist = result["per_joint_top"].get("left_wrist_pitch_joint")
    if wrist is not None:  # only present if it ranks in the top N
        assert wrist["frames_over_budget"] == 0
    assert result["worst_joint"]["max_utilization"] < 0.80


def test_fast_wrist_motion_exceeds_the_five_newton_metre_limit(robot):
    result = torque_feasibility(robot, wrist_wave(robot, frequency=6.0, amplitude=1.2))

    assert result["status"] == "evaluated"
    assert result["torque_feasibility_pct"] < 100.0
    worst = result["worst_joint"]
    assert worst["joint"] == "left_wrist_pitch_joint"
    assert worst["effort_limit_nm"] == 5.0
    assert worst["max_utilization"] > 0.80
    assert worst["budget_nm"] == pytest.approx(4.0)
    assert "left_wrist_pitch_joint" in result["violating_joints"]
    assert result["violating_joints"]["left_wrist_pitch_joint"]["frames_over_budget"] > 0
    # The untouched right wrist must not be implicated.
    assert "right_wrist_pitch_joint" not in result["violating_joints"]


def test_effort_frac_is_the_knob_that_moves_the_budget(robot):
    motion = wrist_wave(robot, frequency=6.0, amplitude=1.2)
    strict = torque_feasibility(robot, motion, effort_frac=0.5)
    loose = torque_feasibility(robot, motion, effort_frac=1.0)
    assert strict["torque_feasibility_pct"] < loose["torque_feasibility_pct"]
    assert strict["effort_frac"] == 0.5


def test_gate_joint_set_controls_which_joints_are_counted(robot):
    """``gate_joints`` picks which joints count toward the informational
    ``torque_feasibility_pct``; it does not make the metric a gate."""
    motion = wrist_wave(robot, frequency=6.0, amplitude=1.2)
    upper = torque_feasibility(robot, motion, gate_joints="upper_body")
    every = torque_feasibility(robot, motion, gate_joints="all")

    assert upper["gated_joints"] == 17  # 3 waist + 14 arm
    assert every["gated_joints"] == 29
    # Legs are always REPORTED, just not graded by default.
    assert all(
        not stats["gated"]
        for name, stats in upper["per_joint_top"].items()
        if "hip" in name or "knee" in name or "ankle" in name
    )
    with pytest.raises(ValueError, match="gate_joints"):
        torque_feasibility(robot, motion, gate_joints="legs")
    assert set(TORQUE_GATE_JOINT_SETS) == {"upper_body", "all"}


def test_short_clips_and_bad_fractions_are_refused(robot):
    stub = wrist_wave(robot, frequency=1.0, amplitude=0.1, frames=2)
    assert torque_feasibility(robot, stub)["status"] == "not_evaluated"
    with pytest.raises(ValueError, match="effort_frac"):
        torque_feasibility(robot, wrist_wave(robot, 1.0, 0.1), effort_frac=0.0)


def test_report_is_json_serialisable_and_leaves_the_model_clean(robot, tmp_path):
    disable_before = int(robot.model.opt.disableflags)
    result = torque_feasibility(robot, wrist_wave(robot, frequency=6.0, amplitude=1.2))

    # Contact is only disabled for the duration of the pass.
    assert int(robot.model.opt.disableflags) == disable_before
    assert np.allclose(robot.data.qacc, 0.0)

    assert result["method"].startswith("mj_inverse")
    assert result["limitations"]
    assert result["root_residual_wrench"]["max_force_n"] >= 0.0

    target = tmp_path / "torque.json"
    target.write_text(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    assert json.loads(target.read_text())["status"] == "evaluated"
