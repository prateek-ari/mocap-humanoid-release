"""Tests for the floating-base standing-balance screen.

Deliberately small: short clips, coarse fps, and a 0.3 s settle.  The screen
integrates real physics at 1 kHz, so a 0.5 s clip is ~800 mj_step calls.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from mocap_g1.balance_screen import (
    ARM_COLUMNS,
    BalanceScreenConfig,
    BalanceScreenError,
    _build_floating_model,
    _validate_reference,
    build_stance,
    convex_hull,
    gain_vectors,
    parse_urdf_effort_limits,
    run_balance_screen,
    support_margin,
)
from mocap_g1.schema import G1_29DOF_JOINTS
from conftest import G1_MJCF as MODEL, needs_model


REPO_ROOT = Path(__file__).resolve().parents[1]
OFFICIAL_URDF = REPO_ROOT / "eval/assets/g1_29dof_official.urdf"

NAMES = list(G1_29DOF_JOINTS)


def arm_swing(frames: int, fps: float, frequency: float, amplitude: float) -> np.ndarray:
    """A [T,36] reference whose only motion is a symmetric shoulder/elbow swing."""
    values = np.zeros((frames, 36), dtype=np.float64)
    values[:, 2] = 0.793
    values[:, 3] = 1.0
    time = np.arange(frames) / fps
    wave = amplitude * np.sin(2.0 * np.pi * frequency * time)
    for side in ("left", "right"):
        values[:, 7 + NAMES.index(f"{side}_shoulder_pitch_joint")] = wave
        values[:, 7 + NAMES.index(f"{side}_elbow_joint")] = 0.5 * np.abs(wave)
    return values


FAST_CONFIG = BalanceScreenConfig(settle_seconds=0.3)


# ---------------------------------------------------------------------------
# Pure geometry / configuration -- no MuJoCo needed
# ---------------------------------------------------------------------------


def test_convex_hull_and_margin_on_a_unit_square():
    square = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.5, 0.5)]
    hull = convex_hull(square)
    assert len(hull) == 4
    assert (0.5, 0.5) not in hull  # interior point dropped
    assert support_margin((0.5, 0.5), hull) == pytest.approx(0.5)
    assert support_margin((0.1, 0.5), hull) == pytest.approx(0.1)
    assert support_margin((-0.2, 0.5), hull) < 0.0  # outside -> negative


def test_degenerate_polygon_has_no_margin():
    collinear = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    assert convex_hull(collinear) == []
    assert support_margin((0.0, 0.0), convex_hull(collinear)) is None
    assert support_margin((0.0, 0.0), [(0.0, 0.0), (1.0, 0.0)]) is None


def test_config_validation_rejects_unusable_values():
    with pytest.raises(BalanceScreenError, match="ankle_kp"):
        BalanceScreenConfig(ankle_kp=0.0).validate()
    with pytest.raises(BalanceScreenError, match="torque_limit_fraction"):
        BalanceScreenConfig(torque_limit_fraction=1.5).validate()
    with pytest.raises(BalanceScreenError, match="min_support_points"):
        BalanceScreenConfig(min_support_points=2).validate()
    # 2 ms is the MJCF default and is NOT stable with the stiff stance gains.
    with pytest.raises(BalanceScreenError, match="numerically stable"):
        BalanceScreenConfig(timestep_s=0.002).validate()
    BalanceScreenConfig().validate()


def test_stance_and_gains_match_the_published_values():
    stance = build_stance(BalanceScreenConfig())
    assert stance[NAMES.index("left_knee_joint")] == pytest.approx(0.30)
    assert stance[NAMES.index("right_hip_pitch_joint")] == pytest.approx(-0.10)
    assert stance[NAMES.index("left_ankle_pitch_joint")] == pytest.approx(-0.20)
    assert stance[NAMES.index("left_shoulder_pitch_joint")] == 0.0

    kp, kd = gain_vectors(BalanceScreenConfig())
    # Arms carry Unitree's published arm_sdk gains.
    for column in ARM_COLUMNS:
        assert kp[column] == pytest.approx(60.0)
        assert kd[column] == pytest.approx(1.5)
    # Ankles are the screening surrogate and must be the stiffest joints.
    assert kp[NAMES.index("left_ankle_pitch_joint")] == pytest.approx(600.0)
    assert kp[NAMES.index("left_knee_joint")] == pytest.approx(300.0)


def test_effort_urdf_parses_the_asymmetric_wrist_limits():
    efforts = parse_urdf_effort_limits(OFFICIAL_URDF)
    assert set(G1_29DOF_JOINTS) <= set(efforts)
    assert efforts["left_wrist_pitch_joint"] == 5.0
    assert efforts["left_ankle_pitch_joint"] == 35.0


def test_rejects_malformed_references():
    with pytest.raises(BalanceScreenError, match="shape"):
        run_balance_screen(np.zeros((3, 35)), 30.0, model_path=MODEL)
    with pytest.raises(BalanceScreenError, match="fps"):
        run_balance_screen(arm_swing(5, 30.0, 0.2, 0.1), 0.0, model_path=MODEL)


def test_rejects_non_unit_root_quaternions():
    # balance_screen used to carry a private copy of the reference validator
    # without the unit-quaternion check; it now shares controller_screen's.
    reference = arm_swing(5, 30.0, 0.2, 0.1)
    reference[2, 3] = 1.01  # |q| = 1.01, outside the 1e-3 tolerance
    with pytest.raises(BalanceScreenError, match="non-unit root quaternions"):
        run_balance_screen(reference, 30.0, model_path=MODEL)
    reference[2, 3] = 1.0005  # inside the tolerance: accepted by the validator
    values, rate = _validate_reference(reference, 30.0)
    assert rate == 30.0 and values.shape == (5, 36)


# ---------------------------------------------------------------------------
# Physics
# ---------------------------------------------------------------------------


@needs_model
def test_shared_lowest_point_matches_the_former_sphere_formula_on_g1():
    """contact.geom_lowest_z reproduces balance_screen's removed _geom_lowest_z.

    The removed helper used ``z - radius`` for spheres and ``z - rbound``
    otherwise.  On the G1 every sole geom is a sphere, so the shared
    implementation must agree bit-for-bit across arbitrary base poses.
    """
    import mujoco

    from mocap_g1.contact import geom_lowest_z

    rig = _build_floating_model(MODEL.resolve(), FAST_CONFIG)
    model = rig.model
    data = mujoco.MjData(model)
    assert len(rig.sole_geoms) >= FAST_CONFIG.min_support_points
    assert all(
        int(model.geom_type[geom]) == int(mujoco.mjtGeom.mjGEOM_SPHERE)
        for geom in rig.sole_geoms
    )

    def former(geom: int) -> float:
        if int(model.geom_type[geom]) == int(mujoco.mjtGeom.mjGEOM_SPHERE):
            return float(data.geom_xpos[geom][2] - model.geom_size[geom][0])
        return float(data.geom_xpos[geom][2] - model.geom_rbound[geom])

    rng = np.random.default_rng(7)
    joint_ids = [list(model.jnt_qposadr).index(address) for address in rig.qpos_addresses]
    lower, upper = model.jnt_range[joint_ids, 0], model.jnt_range[joint_ids, 1]
    mesh_cache: dict[int, np.ndarray] = {}
    for trial in range(25):
        quaternion = rng.normal(size=4) if trial else np.array([1.0, 0.0, 0.0, 0.0])
        data.qpos[:3] = rng.uniform(-0.5, 0.5, size=3) + np.array([0.0, 0.0, 0.8])
        data.qpos[3:7] = quaternion / np.linalg.norm(quaternion)
        data.qpos[rig.qpos_addresses] = rng.uniform(lower, upper) if trial else build_stance(
            FAST_CONFIG
        )
        mujoco.mj_forward(model, data)
        for geom in rig.sole_geoms:
            assert geom_lowest_z(model, data, geom, mesh_cache, mujoco) == former(geom)


@needs_model
def test_benign_arm_hold_passes_every_gate():
    reference = arm_swing(frames=16, fps=30.0, frequency=0.25, amplitude=0.15)
    report = run_balance_screen(reference, 30.0, model_path=MODEL, config=FAST_CONFIG)

    assert report["balance_screen"]["status"] == "PASS"
    assert report["balance_screen"]["reasons"] == []
    gates = report["gates"]
    assert all(gate["status"] == "PASS" for gate in gates.values())
    # The robot really is standing on both feet: eight sole spheres down.
    assert gates["com_support_margin"]["min_support_points_observed"] == 8
    assert gates["com_support_margin"]["support_polygon_degenerate_steps"] == 0
    assert gates["com_support_margin"]["worst_margin_m"] > FAST_CONFIG.com_margin_min_m
    assert report["events"]["aborted_step"] is None
    assert report["events"]["executed_steps"] == report["events"]["planned_steps"]


@needs_model
def test_aggressive_arm_swing_trips_a_gate():
    reference = arm_swing(frames=31, fps=60.0, frequency=2.5, amplitude=1.4)
    report = run_balance_screen(reference, 60.0, model_path=MODEL, config=FAST_CONFIG)

    # Schema v2: a violated physical gate is FAIL (BLOCKED = could not check).
    assert report["balance_screen"]["status"] == "FAIL"
    assert report["hardware_candidate"]["status"] == "BLOCKED"
    assert report["balance_screen"]["reasons"], "an aggressive swing must trip a gate"
    assert any(gate["status"] == "FAIL" for gate in report["gates"].values())
    # It should be the balance gates that complain, not a bookkeeping failure.
    assert set(report["balance_screen"]["reasons"]) & {
        "com_support_margin_exceeded",
        "ankle_torque_headroom_exceeded",
        "pelvis_tilt_exceeded",
        "pelvis_angular_speed_exceeded",
        "support_polygon_lost",
        "pelvis_height_collapse",
        "fall_abort",
    }


@needs_model
def test_report_schema_and_json_round_trip(tmp_path):
    reference = arm_swing(frames=16, fps=30.0, frequency=0.25, amplitude=0.15)
    report = run_balance_screen(
        reference,
        30.0,
        model_path=MODEL,
        config=FAST_CONFIG,
        source_sha256="0" * 64,
        source_metadata={"offsets_key": "zero-preview"},
    )

    assert report["schema_version"] == 2
    assert report["joint_limits"]["source"] == "urdf"
    assert len(report["joint_limits"]["sha256"]) == 64
    assert report["purpose"] == (
        "floating_base_stiff_pd_standing_balance_screen_not_balance_validation"
    )
    for key in (
        "balance_screen",
        "hardware_candidate",
        "input",
        "configuration",
        "gains",
        "model",
        "stance",
        "gates",
        "events",
        "limitations",
    ):
        assert key in report, key

    assert set(report["gates"]) == {
        "com_support_margin",
        "ankle_torque_headroom",
        "pelvis_attitude",
    }
    for gate in report["gates"].values():
        assert gate["status"] in {"PASS", "FAIL"}

    # Provenance is explicit and honest about which half is verified.
    assert report["gains"]["arm_gain_verification_status"] == "verified_published"
    assert report["gains"]["arm_kp"] == 60.0
    assert report["gains"]["arm_kd"] == 1.5
    assert report["gains"]["stance_gain_verification_status"] == (
        "screening_surrogate_not_published"
    )
    assert all(
        "unitreerobotics" in url and "sdk_dds_example" in url
        for url in report["gains"]["arm_gain_sources"]
    )

    # Preview calibration must block the hardware verdict even on a PASS.
    assert report["balance_screen"]["status"] == "PASS"
    assert report["hardware_candidate"]["status"] == "BLOCKED"
    assert "preview_calibration" in report["hardware_candidate"]["reasons"]

    # The vendor effort limit (35 N*m), not the looser MJCF range (50), is used.
    ankle = report["gates"]["ankle_torque_headroom"]
    assert ankle["per_joint_limit_nm"]["left_ankle_pitch_joint"] == 35.0
    assert ankle["mjcf_actuatorfrcrange_nm"]["left_ankle_pitch_joint"] == 50.0
    assert report["model"]["base"] == "pelvis_freejoint_retained"
    assert len(report["model"]["controlled_arm_joints"]) == 14
    assert report["limitations"]

    # The report must survive the CLI's strict JSON writer.
    target = tmp_path / "balance.json"
    target.write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
    assert json.loads(target.read_text())["balance_screen"]["status"] == "PASS"


@needs_model
def test_cli_writes_a_report_and_refuses_to_overwrite(tmp_path):
    from tools.control import screen_g1_balance

    source = tmp_path / "clip.npz"
    np.savez(
        source,
        qpos=arm_swing(frames=16, fps=30.0, frequency=0.25, amplitude=0.15),
        fps=np.asarray(30.0),
        metadata=np.asarray(
            json.dumps({"robot": "unitree_g1", "root_quaternion_order": "wxyz"})
        ),
    )
    output = tmp_path / "report.json"
    argv = [
        str(source),
        "--model",
        str(MODEL),
        "--output",
        str(output),
        "--settle-seconds",
        "0.3",
    ]
    assert screen_g1_balance.main(argv) == 0

    report = json.loads(output.read_text())
    assert report["balance_screen"]["status"] == "PASS"
    assert report["source_selection"]["start_frame_inclusive"] == 0
    assert report["input"]["source_sha256"]

    # Second run must refuse rather than clobber the existing report.
    assert screen_g1_balance.main(argv) == 2
