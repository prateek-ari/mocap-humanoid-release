from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np

from mocap_g1.assets import official_urdf_path
from mocap_g1.contact_refine import (
    ContactRefineConfig,
    ContactRefineError,
    _ankle_bounds,
    infer_contact_states,
    refine_reference_contacts,
)
from mocap_g1.schema import G1_29DOF_JOINTS, JointLimit, load_official_joint_limits
from conftest import G1_MJCF as MODEL, requires_g1_mjcf


REPO_ROOT = Path(__file__).resolve().parents[1]
FPS = 60.0


def base_qpos(frames: int) -> np.ndarray:
    """Zero-pose floating-base qpos: identity root quaternion at origin."""
    qpos = np.zeros((frames, 36), dtype=np.float64)
    qpos[:, 3] = 1.0
    return qpos


def _standing_height() -> float:
    """Root z that puts the zero-pose G1 soles exactly on the world plane.

    Computed by FK bisection against the real MJCF; cached at module scope.
    """
    global _STANDING_HEIGHT_CACHE
    if _STANDING_HEIGHT_CACHE is None:
        import mujoco

        model = mujoco.MjModel.from_xml_path(str(MODEL))
        data = mujoco.MjData(model)
        row = base_qpos(1)[0]

        def min_sole(z: float) -> float:
            import mocap_g1.contact_refine as cr

            row[2] = z
            data.qpos[:] = row
            mujoco.mj_forward(model, data)
            feet = cr._resolve_feet(model, mujoco)
            state = cr._sole_state(model, data, mujoco, feet, {})
            return min(low_z for low_z, _ in state.values())

        low, high = 0.4, 1.3
        for _ in range(60):
            mid = 0.5 * (low + high)
            if min_sole(mid) < 0.0:
                low = mid
            else:
                high = mid
        _STANDING_HEIGHT_CACHE = 0.5 * (low + high)
    return _STANDING_HEIGHT_CACHE


_STANDING_HEIGHT_CACHE: float | None = None


def _minimal_model_xml(joint_names: tuple[str, ...] = G1_29DOF_JOINTS) -> str:
    """Self-contained floating-base model with canonical qpos order and sole spheres."""
    groups = (
        ("left_leg", joint_names[0:6]),
        ("right_leg", joint_names[6:12]),
        ("waist", joint_names[12:15]),
        ("left_arm", joint_names[15:22]),
        ("right_arm", joint_names[22:29]),
    )

    def chain_xml(group: str, names: tuple[str, ...]) -> list[str]:
        lines: list[str] = []
        indent = "      "
        is_leg = group in {"left_leg", "right_leg"}
        side = group.split("_", 1)[0] if is_leg else None
        for index, name in enumerate(names):
            if is_leg and index == len(names) - 1:
                body_name = f"{side}_ankle_roll_link"
            else:
                body_name = f"{group}_link_{index}"
            if is_leg:
                y = 0.09 if side == "left" and index == 0 else -0.09 if index == 0 else 0.0
                position = f"0 {y} -0.05"
            else:
                position = "0 0 0.03"
            axis = "1 0 0" if "roll" in name else "0 0 1" if "yaw" in name else "0 1 0"
            lines.extend([
                f'{indent}<body name="{body_name}" pos="{position}">',
                f'{indent}  <inertial pos="0 0 0" mass="0.01" diaginertia="0.0001 0.0001 0.0001"/>',
                f'{indent}  <joint name="{name}" axis="{axis}" range="-2 2"/>',
            ])
            indent += "  "
        if is_leg:
            for x, y in ((-0.05, -0.025), (-0.05, 0.025), (0.12, -0.03), (0.12, 0.03)):
                lines.append(
                    f'{indent}<geom type="sphere" size="0.01" pos="{x} {y} -0.01"/>'
                )
        for _ in names:
            indent = indent[:-2]
            lines.append(f"{indent}</body>")
        return lines

    body_lines: list[str] = []
    for group, names in groups:
        body_lines.extend(chain_xml(group, tuple(names)))
    return "\n".join([
        '<mujoco model="contact_refine_minimal">',
        '  <compiler angle="radian"/>',
        '  <option gravity="0 0 0"/>',
        '  <default><joint type="hinge" limited="true"/></default>',
        '  <worldbody>',
        '    <body name="pelvis">',
        '      <freejoint name="pelvis"/>',
        '      <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>',
        *body_lines,
        '    </body>',
        '  </worldbody>',
        '</mujoco>',
    ])


def _standing_height_for(model_path: Path, row: np.ndarray | None = None,
                         target: float = 0.0) -> float:
    import mocap_g1.contact_refine as cr
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    feet = cr._resolve_feet(model, mujoco)
    trial = base_qpos(1)[0] if row is None else row.copy()

    def minimum(z: float) -> float:
        trial[2] = z
        data.qpos[:] = trial
        mujoco.mj_forward(model, data)
        return min(
            low_z
            for low_z, _point in cr._sole_state(model, data, mujoco, feet, {}).values()
        )

    low, high = 0.0, 1.5
    for _ in range(70):
        mid = 0.5 * (low + high)
        if minimum(mid) < target:
            low = mid
        else:
            high = mid
    return 0.5 * (low + high)


def _dithered_flat_stance(frames: int, height: float, amplitude_rad: float,
                          seed: int = 3) -> np.ndarray:
    """Both feet flat and planted; ankle pitch/roll carry +-amplitude sign dither.

    The dither is sub-millimetre at the sole but flips WHICH foot corner is
    lowest almost every frame -- the case that used to read as m/s skating
    when skating was the frame-to-frame motion of the argmin lowest point.
    """
    rng = np.random.default_rng(seed)
    qpos = base_qpos(frames)
    qpos[:, 2] = height
    for column in (4, 5, 10, 11):  # left/right ankle pitch, ankle roll
        qpos[:, 7 + column] = amplitude_rad * rng.choice((-1.0, 1.0), size=frames)
    return qpos


def _argmin_point_speeds(model_path: Path, qpos: np.ndarray) -> np.ndarray:
    """Per-foot speeds of the OLD skating measurement (argmin lowest point)."""
    import mocap_g1.contact_refine as cr
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    feet = cr._resolve_feet(model, mujoco)
    sole = cr._fk_sole_pass(model, data, mujoco, feet, qpos)
    return np.stack([
        np.linalg.norm(np.diff(sole[side]["xy"], axis=0), axis=1) * FPS
        for side in sorted(sole)
    ])


class ContactStateMachineTests(unittest.TestCase):
    """Pure tests of infer_contact_states (no model required)."""

    def test_config_validation_rejects_bad_values(self):
        with self.assertRaisesRegex(ContactRefineError, "target_clearance"):
            ContactRefineConfig(target_clearance_m=0.0).validate()
        with self.assertRaisesRegex(ContactRefineError, "clearance"):
            ContactRefineConfig(
                target_clearance_m=0.0005, penetration_tolerance_m=0.002
            ).validate()
        with self.assertRaisesRegex(ContactRefineError, "skate_taper_frames"):
            ContactRefineConfig(skate_taper_frames=0).validate()
        with self.assertRaisesRegex(ContactRefineError, "hysteresis_enter_frames"):
            ContactRefineConfig(hysteresis_enter_frames=True).validate()
        with self.assertRaisesRegex(ContactRefineError, "finite and positive"):
            ContactRefineConfig(contact_height_band_m=-1.0).validate()
        for invalid in ("0.02", True):
            with self.subTest(contact_height_band_m=invalid):
                with self.assertRaisesRegex(ContactRefineError, "contact_height_band_m"):
                    ContactRefineConfig(contact_height_band_m=invalid).validate()
        with self.assertRaisesRegex(ContactRefineError, "boolean"):
            ContactRefineConfig(skate_repair_enabled="yes").validate()

    def test_enters_stance_only_after_enter_debounce(self):
        config = ContactRefineConfig(hysteresis_enter_frames=3, hysteresis_exit_frames=2)
        raw = np.ones(10, dtype=bool)
        states, threshold = infer_contact_states(
            np.zeros(10), FPS, config, raw_candidate=raw
        )
        self.assertEqual(threshold, config.contact_height_band_m)
        self.assertEqual(states.tolist(), [False, False, True, True, True,
                                           True, True, True, True, True])

    def test_holds_through_gaps_shorter_than_exit_debounce(self):
        config = ContactRefineConfig(hysteresis_enter_frames=2, hysteresis_exit_frames=3)
        # candidate pattern: enter, 1-frame gap (held), then a long gap (exit)
        raw = np.array([1, 1, 0, 1, 0, 0, 0, 0, 1, 1], dtype=bool)
        states, _ = infer_contact_states(np.zeros(10), FPS, config, raw_candidate=raw)
        self.assertEqual(states.tolist(),
                         [False, True, True, True, True, True, False, False,
                          False, True])

    def test_exits_only_after_exit_debounce(self):
        config = ContactRefineConfig(hysteresis_enter_frames=1, hysteresis_exit_frames=4)
        raw = np.array([1, 1, 0, 0, 0, 0, 0], dtype=bool)
        states, _ = infer_contact_states(np.zeros(7), FPS, config, raw_candidate=raw)
        self.assertEqual(states.tolist(), [True, True, True, True, True, False, False])

    def test_airborne_constant_height_is_never_stance(self):
        config = ContactRefineConfig()
        height = np.full(12, 0.5)
        states, _ = infer_contact_states(height, FPS, config)
        self.assertFalse(states.any())

    def test_fast_vertical_motion_is_not_stance(self):
        # Fast descent through the ground band: heights are under the band in
        # the later frames but vertical speed stays far above the gate, so
        # candidacy fails on vz alone.
        config = ContactRefineConfig(contact_vz_max_m_s=0.4)
        height = np.linspace(0.05, -0.05, 12)
        self.assertLess(abs((height[2] - height[0]) * FPS / 2.0),
                        config.contact_vz_max_m_s + 10.0)  # sanity: fixture loads
        states, _ = infer_contact_states(height, FPS, config)
        self.assertFalse(states.any())


class AnkleBoundsLimitSourceTests(unittest.TestCase):
    """T1-7: ankle correction bounds come from the mechanical URDF."""

    def test_bounds_are_urdf_range_intersected_with_budget(self):
        limits = load_official_joint_limits(official_urdf_path())
        config = ContactRefineConfig()
        column = G1_29DOF_JOINTS.index("left_ankle_pitch_joint")
        lower = limits["left_ankle_pitch_joint"].lower
        # Near the lower URDF limit the budget is cut by the URDF, not the MJCF.
        value = lower + 0.1
        low, high = _ankle_bounds(limits, column, value, config)
        self.assertAlmostEqual(low, -0.1)
        self.assertAlmostEqual(high, config.max_ankle_correction_rad)

    def test_custom_limit_table_is_honoured(self):
        limits = {name: JointLimit(-0.05, 0.05, 1.0) for name in G1_29DOF_JOINTS}
        column = G1_29DOF_JOINTS.index("right_ankle_roll_joint")
        low, high = _ankle_bounds(limits, column, 0.0, ContactRefineConfig())
        self.assertAlmostEqual(low, -0.05)
        self.assertAlmostEqual(high, 0.05)


class ContactRefineSelfContainedTests(unittest.TestCase):
    """Core repair safety tests using a generated 29-joint MuJoCo model."""

    @classmethod
    def setUpClass(cls):
        try:
            import mujoco  # noqa: F401
        except ImportError as exc:
            raise unittest.SkipTest("MuJoCo runtime extra is unavailable") from exc
        cls._model_directory = tempfile.TemporaryDirectory()
        cls.model = Path(cls._model_directory.name) / "minimal_g1.xml"
        cls.model.write_text(_minimal_model_xml(), encoding="utf-8")
        cls.standing_height = _standing_height_for(cls.model)

    @classmethod
    def tearDownClass(cls):
        cls._model_directory.cleanup()

    def _single_to_double_stance_clip(self) -> np.ndarray:
        frames = 30
        qpos = base_qpos(frames)
        qpos[:, 2] = self.standing_height + 0.004
        qpos[:13, 7 + 6] = 1.2  # right leg tilted clear of the ground
        step = 0.0301 / FPS
        for frame in range(2, 10):
            qpos[frame:, 0] += step
        return qpos

    def test_clean_grounded_clip_passes_self_contained_model(self):
        qpos = base_qpos(12)
        qpos[:, 2] = self.standing_height + 0.004
        result = refine_reference_contacts(qpos, FPS, model_path=self.model)
        self.assertEqual(result.report["status"], "PASS", result.report["rejections"])
        self.assertEqual(result.report["schema_version"], 3)
        self.assertGreater(result.report["summary"]["final_contact_frames"], 0)
        self.assertTrue(np.array_equal(result.qpos, qpos))

    def test_report_records_the_limit_source(self):
        qpos = base_qpos(12)
        qpos[:, 2] = self.standing_height + 0.004
        result = refine_reference_contacts(qpos, FPS, model_path=self.model)
        limits = result.report["joint_limits"]
        self.assertEqual(limits["source"], "urdf")
        self.assertEqual(limits["path"], str(official_urdf_path().resolve()))
        self.assertEqual(len(limits["sha256"]), 64)
        with self.assertRaisesRegex(ContactRefineError, "joint position limits"):
            refine_reference_contacts(
                qpos, FPS, model_path=self.model,
                config=ContactRefineConfig(limits_urdf_path=str(self.model) + ".missing"),
            )

    def test_airborne_clip_rejects_instead_of_passing_vacuously(self):
        qpos = base_qpos(12)
        qpos[:, 2] = self.standing_height + 0.5
        result = refine_reference_contacts(qpos, FPS, model_path=self.model)
        self.assertEqual(result.report["status"], "REJECT")
        self.assertIsNone(result.qpos)
        reasons = {item["reason"] for item in result.report["rejections"]}
        self.assertIn("no_world_contact_observed", reasons)
        self.assertEqual(result.report["summary"]["final_contact_frames"], 0)

    def test_root_lift_never_exceeds_configured_budget(self):
        target = ContactRefineConfig().target_clearance_m
        target_height = _standing_height_for(self.model, target=target)
        qpos = base_qpos(6)
        qpos[:, 2] = target_height - 0.04999
        result = refine_reference_contacts(qpos, FPS, model_path=self.model)
        self.assertEqual(result.report["status"], "PASS", result.report["rejections"])
        lifts = [
            repair["lift_m"]
            for frame in result.report["penetration_repairs"]
            for repair in frame["repairs"]
            if repair["strategy"] == "root_z_lift"
        ]
        self.assertTrue(lifts)
        self.assertLessEqual(max(lifts), 0.05)

    def test_skating_onset_never_hides_vertical_correction_dynamics(self):
        row = base_qpos(1)[0]
        row[7 + 6] = 1.2
        height = _standing_height_for(self.model, row=row, target=0.004)
        qpos = np.tile(row, (12, 1))
        qpos[:, 2] = height
        qpos[1:, 2] -= 0.04
        qpos[2:, 0] += 0.0301 / FPS

        result = refine_reference_contacts(qpos, FPS, model_path=self.model)
        self.assertEqual(result.report["status"], "REJECT")
        reasons = {item["reason"] for item in result.report["rejections"]}
        self.assertTrue(
            reasons & {
                "correction_velocity_limit_exceeded_m",
                "correction_acceleration_limit_exceeded_m",
                "correction_velocity_limit_exceeded_rad",
                "correction_acceleration_limit_exceeded_rad",
            },
            result.report["rejections"],
        )

    def test_taper_refuses_to_cross_new_contact(self):
        result = refine_reference_contacts(
            self._single_to_double_stance_clip(), FPS, model_path=self.model
        )
        self.assertEqual(result.report["status"], "REJECT")
        failures = result.report["skating_repair_failures"]
        self.assertTrue(failures)
        self.assertEqual(failures[0]["failure"], "no_contact_free_dynamics_safe_taper")
        self.assertEqual(failures[0]["available_contact_free_taper_frames"], 0)
        self.assertEqual(
            result.report["summary"]["residual_double_stance_skating_frames"], 0
        )

    def test_final_pass_rejects_double_stance_skating_introduced_by_repair(self):
        step = 0.0301 / FPS

        def unsafe_old_style_repair(working, side, start, end, sole_points,
                                    contact_free, rate, config):
            offsets = np.zeros(len(working))
            for frame in range(2, 10):
                offsets[frame:] -= step
            for frame in range(15, 23):
                offsets[frame:] += step
            working[:, 0] += offsets
            return {
                "side": side,
                "start_frame": int(start),
                "end_frame": int(end),
                "applied": True,
                "applied_steps": 16,
                "net_offset_xy_m": [float(offsets[end]), 0.0],
                "tapered_to_zero_by_frame": 22,
            }

        with mock.patch(
            "mocap_g1.contact_refine._repair_skating_segment",
            side_effect=unsafe_old_style_repair,
        ):
            result = refine_reference_contacts(
                self._single_to_double_stance_clip(), FPS, model_path=self.model
            )
        reasons = {item["reason"] for item in result.report["rejections"]}
        self.assertIn("final_validation_double_stance_skating", reasons)
        self.assertGreater(
            result.report["summary"]["residual_double_stance_skating_frames"], 0
        )

    def test_dithered_flat_stance_is_not_reported_as_skating(self):
        qpos = _dithered_flat_stance(60, self.standing_height + 0.004, 0.0015)
        threshold = ContactRefineConfig().skate_speed_threshold_m_s
        # Guard the fixture: the argmin lowest point really does jump between
        # corners, so the old measurement would have flagged most frames.
        old_speeds = _argmin_point_speeds(self.model, qpos)
        self.assertGreater(int(np.count_nonzero(old_speeds > threshold)), 40)

        result = refine_reference_contacts(qpos, FPS, model_path=self.model)
        summary = result.report["summary"]
        self.assertEqual(result.report["status"], "PASS", result.report["rejections"])
        self.assertEqual(summary["double_stance_skate_frames"], 0)
        self.assertEqual(summary["residual_skating_frames"], 0)
        self.assertEqual(summary["residual_double_stance_skating_frames"], 0)
        self.assertEqual(summary["skating_segments_repaired"], 0)
        self.assertGreater(summary["final_contact_frames"], 0)
        self.assertTrue(np.array_equal(result.qpos, qpos))

    def test_reordered_canonical_joints_are_rejected(self):
        names = list(G1_29DOF_JOINTS)
        names[0], names[1] = names[1], names[0]
        reordered = Path(self._model_directory.name) / "reordered.xml"
        reordered.write_text(_minimal_model_xml(tuple(names)), encoding="utf-8")
        qpos = base_qpos(3)
        qpos[:, 2] = self.standing_height
        with self.assertRaisesRegex(ContactRefineError, "wrong_addresses"):
            refine_reference_contacts(qpos, FPS, model_path=reordered)

    def test_missing_canonical_joint_name_is_rejected(self):
        names = list(G1_29DOF_JOINTS)
        names[4] = "renamed_left_ankle_pitch_joint"
        renamed = Path(self._model_directory.name) / "renamed.xml"
        renamed.write_text(_minimal_model_xml(tuple(names)), encoding="utf-8")
        qpos = base_qpos(3)
        qpos[:, 2] = self.standing_height
        with self.assertRaisesRegex(ContactRefineError, "missing=.*left_ankle_pitch_joint"):
            refine_reference_contacts(qpos, FPS, model_path=renamed)

    def test_cli_removes_derivative_if_report_creation_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.npz"
            qpos = base_qpos(6)
            qpos[:, 2] = self.standing_height + 0.004
            metadata = {
                "robot": "unitree_g1",
                "root_quaternion_order": "wxyz",
                "offsets_key": "reviewed-test",
            }
            np.savez_compressed(
                source,
                metadata=np.asarray(json.dumps(metadata, sort_keys=True)),
                qpos=qpos,
                fps=np.asarray(FPS),
            )
            output = root / "refined.npz"
            report_parent = root / "not-a-directory"
            report_parent.write_text("occupied", encoding="utf-8")
            completed = subprocess.run(
                [
                    sys.executable,
                    str(REPO_ROOT / "tools/retarget/refine_g1_contacts.py"),
                    str(source),
                    "--model", str(self.model),
                    "--output", str(output),
                    "--report", str(report_parent / "report.json"),
                ],
                cwd=REPO_ROOT,
                capture_output=True,
                text=True,
                timeout=60,
            )
            self.assertEqual(completed.returncode, 2)
            self.assertFalse(output.exists(), completed.stderr)


class ContactRefineModelTests(unittest.TestCase):
    """Model-gated behavior tests on the real G1 MJCF."""

    def setUp(self):
        if not MODEL.is_file():
            self.skipTest("external GMR MuJoCo model is unavailable")

    @requires_g1_mjcf
    def test_clean_grounded_trajectory_passes_bitwise_unchanged(self):
        height = _standing_height() + 0.004
        qpos = base_qpos(30)
        qpos[:, 2] = height
        before = qpos.copy()
        result = refine_reference_contacts(qpos, FPS, model_path=MODEL)
        self.assertEqual(result.report["status"], "PASS")
        self.assertEqual(result.report["summary"]["penetration_repaired_frames"], 0)
        self.assertTrue(np.array_equal(result.qpos, before))
        self.assertTrue(np.array_equal(qpos, before))
        self.assertEqual(result.report["hardware_candidate"]["status"],
                         "CONTACT_REFINE_ONLY")

    @requires_g1_mjcf
    def test_toe_penetration_repaired_by_ankle_strategy(self):
        height = _standing_height() + 0.004
        qpos = base_qpos(24)
        qpos[:, 2] = height
        qpos[:, 7 + 4] = 0.25  # left ankle plantarflexion digs the toe in
        before = qpos.copy()
        result = refine_reference_contacts(qpos, FPS, model_path=MODEL)
        report = result.report
        self.assertEqual(report["status"], "PASS", report["rejections"])
        strategies = [
            item["strategy"]
            for record in report["penetration_repairs"]
            for item in record["repairs"]
        ]
        self.assertIn("left:ankle_pitch", strategies)
        self.assertNotIn("root_z_lift", strategies)
        self.assertLessEqual(
            report["summary"]["joint_correction_dynamics"]["max_step"],
            report["configuration"]["max_ankle_correction_rad"] + 1e-9,
        )
        self.assertTrue(np.array_equal(qpos, before))
        self._assert_no_residual_penetration(result)

    @requires_g1_mjcf
    def test_sole_sink_repaired_by_root_z_lift_fallback(self):
        height = _standing_height() + 0.004
        qpos = base_qpos(24)
        qpos[:, 2] = height
        qpos[:, 7 + 4] = -0.6  # dorsiflexion sinks the heel; ankle lever too small
        before = qpos.copy()
        result = refine_reference_contacts(qpos, FPS, model_path=MODEL)
        report = result.report
        self.assertEqual(report["status"], "PASS", report["rejections"])
        strategies = [
            item["strategy"]
            for record in report["penetration_repairs"]
            for item in record["repairs"]
        ]
        self.assertIn("root_z_lift", strategies)
        lifts = [
            item["lift_m"]
            for record in report["penetration_repairs"]
            for item in record["repairs"]
            if item["strategy"] == "root_z_lift"
        ]
        self.assertTrue(lifts and max(lifts) <= 0.05 + 1e-9)
        self.assertTrue(np.array_equal(qpos, before))
        self._assert_no_residual_penetration(result)

    def _assert_no_residual_penetration(self, result):
        import mocap_g1.contact_refine as cr
        import mujoco

        model = mujoco.MjModel.from_xml_path(str(MODEL))
        data = mujoco.MjData(model)
        feet = cr._resolve_feet(model, mujoco)
        worst = min(
            low_z
            for arrays in cr._fk_sole_pass(model, data, mujoco, feet, result.qpos).values()
            for low_z in arrays["z"]
        )
        self.assertGreaterEqual(worst, -result.report["configuration"]["penetration_tolerance_m"])

    @requires_g1_mjcf
    def test_unrepairable_penetration_rejects_without_qpos(self):
        height = _standing_height() + 0.004
        qpos = base_qpos(12)
        qpos[:, 2] = height - 0.30  # whole robot far under ground; lift budget is 5 cm
        result = refine_reference_contacts(qpos, FPS, model_path=MODEL)
        self.assertEqual(result.report["status"], "REJECT")
        self.assertIsNone(result.qpos)
        reasons = {item["reason"] for item in result.report["rejections"]}
        self.assertIn("final_validation_residual_penetration", reasons)

    @requires_g1_mjcf
    def test_single_stance_skating_is_pinned_by_root_translation(self):
        height = _standing_height() + 0.004
        frames = 40
        time = np.arange(frames) / FPS
        qpos = base_qpos(frames)
        qpos[:, 0] = 0.06 * time          # root slides forward at 6 cm/s
        qpos[:, 2] = height
        qpos[:, 7 + 6] = -0.9             # right hip pitch: thigh raised forward
        qpos[:, 7 + 9] = 1.5              # right knee folded
        qpos[:, 7 + 10] = -0.6            # right ankle roughly level
        before = qpos.copy()
        result = refine_reference_contacts(qpos, FPS, model_path=MODEL)
        report = result.report
        self.assertEqual(report["status"], "PASS", report["rejections"])
        self.assertEqual(report["summary"]["skating_segments_repaired"], 1)
        self.assertEqual(
            report["summary"]["residual_skating_frames"], 0, report["rejections"]
        )
        # The pinning must have translated the root backwards along x.
        net_x = float(np.mean(result.qpos[10:, 0] - before[10:, 0]))
        self.assertLess(net_x, -0.01)
        self.assertTrue(np.array_equal(qpos, before))

    @requires_g1_mjcf
    def test_dithered_flat_stance_on_g1_is_not_reported_as_skating(self):
        qpos = _dithered_flat_stance(60, _standing_height() + 0.004, 0.002)
        old_speeds = _argmin_point_speeds(MODEL, qpos)
        self.assertGreater(
            int(np.count_nonzero(old_speeds > ContactRefineConfig().skate_speed_threshold_m_s)),
            40,
        )
        result = refine_reference_contacts(qpos, FPS, model_path=MODEL)
        summary = result.report["summary"]
        self.assertEqual(result.report["status"], "PASS", result.report["rejections"])
        self.assertEqual(summary["double_stance_skate_frames"], 0)
        self.assertEqual(summary["residual_skating_frames"], 0)
        self.assertEqual(summary["residual_double_stance_skating_frames"], 0)

    @requires_g1_mjcf
    def test_double_stance_skating_rejects_honestly(self):
        height = _standing_height() + 0.004
        frames = 30
        time = np.arange(frames) / FPS
        qpos = base_qpos(frames)
        qpos[:, 0] = 0.06 * time          # both feet planted, both sliding
        qpos[:, 2] = height
        result = refine_reference_contacts(qpos, FPS, model_path=MODEL)
        self.assertEqual(result.report["status"], "REJECT")
        self.assertIsNone(result.qpos)
        reasons = {item["reason"] for item in result.report["rejections"]}
        self.assertIn("double_stance_skating_unrepaired_in_this_version", reasons)
        self.assertGreater(result.report["summary"]["double_stance_skate_frames"], 0)

    @requires_g1_mjcf
    def test_cli_writes_derivative_and_provenance_without_touching_source(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.npz"
            height = _standing_height() + 0.004
            qpos = base_qpos(20)
            qpos[:, 2] = height
            metadata = {
                "robot": "unitree_g1",
                "root_quaternion_order": "wxyz",
                "offsets_key": "zero-preview",
            }
            np.savez_compressed(
                source,
                metadata=np.asarray(json.dumps(metadata, sort_keys=True)),
                qpos=qpos,
                fps=np.asarray(FPS),
            )
            before = source.read_bytes()
            output = root / "refined.npz"
            report = root / "report.json"
            completed = subprocess.run(
                [
                    sys.executable,
                    str(REPO_ROOT / "tools/retarget/refine_g1_contacts.py"),
                    str(source),
                    "--model", str(MODEL),
                    "--output", str(output),
                    "--report", str(report),
                ],
                cwd=REPO_ROOT,
                check=False,
                capture_output=True,
                text=True,
                timeout=300,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertEqual(source.read_bytes(), before)
            self.assertTrue(output.is_file())
            payload = json.loads(report.read_text())
            self.assertEqual(payload["status"], "PASS")
            self.assertTrue(payload["input"]["source_file_preserved"])
            self.assertTrue(payload["output"]["written"])
            with np.load(output, allow_pickle=False) as archive:
                derived = json.loads(str(archive["metadata"].item()))
            self.assertEqual(
                derived["contact_refinement"]["purpose"],
                "world_contact_refinement_not_calibration",
            )

            # Refuse-overwrite contract inherited by the CLI.
            rerun = subprocess.run(
                [
                    sys.executable,
                    str(REPO_ROOT / "tools/retarget/refine_g1_contacts.py"),
                    str(source),
                    "--model", str(MODEL),
                    "--output", str(output),
                    "--report", str(root / "other.json"),
                ],
                cwd=REPO_ROOT, capture_output=True, text=True, timeout=300,
            )
            self.assertEqual(rerun.returncode, 2)
            self.assertIn("refusing to overwrite", rerun.stderr)


class ContactRefineSpeedupEquivalenceTests(unittest.TestCase):
    """The 2026-10-12 stage-3 speed-up must not change any result.

    It rests on three exact equivalences, each pinned here on the real G1
    MJCF: ``mj_kinematics`` writes the same body/geom poses as ``mj_forward``;
    the vectorised all-sphere lowest point equals the shared
    ``mocap_g1.contact`` helper bit for bit; and skipping an ankle search
    whose fixed (other-foot) geoms already miss the clearance target returns
    exactly what the full search would have.
    """

    def setUp(self):
        if not MODEL.is_file():
            self.skipTest("external GMR MuJoCo model is unavailable")

    @staticmethod
    def _random_rows(count: int, seed: int = 7) -> np.ndarray:
        rng = np.random.default_rng(seed)
        rows = base_qpos(count)
        rows[:, 0:2] = rng.uniform(-0.5, 0.5, size=(count, 2))
        rows[:, 2] = rng.uniform(0.5, 0.9, size=count)
        quats = rng.normal(size=(count, 4))
        rows[:, 3:7] = quats / np.linalg.norm(quats, axis=1, keepdims=True)
        rows[:, 7:] = rng.uniform(-0.6, 0.6, size=(count, 29))
        return rows

    @requires_g1_mjcf
    def test_kinematics_matches_forward_bitwise(self):
        import mujoco

        model = mujoco.MjModel.from_xml_path(str(MODEL))
        forward, kinematic = mujoco.MjData(model), mujoco.MjData(model)
        for row in self._random_rows(25):
            forward.qpos[:] = row
            forward.qvel[:] = 0.0
            mujoco.mj_forward(model, forward)
            kinematic.qpos[:] = row
            mujoco.mj_kinematics(model, kinematic)
            for field in ("xpos", "xmat", "geom_xpos", "geom_xmat"):
                self.assertEqual(getattr(forward, field).tobytes(),
                                 getattr(kinematic, field).tobytes(), field)

    @requires_g1_mjcf
    def test_vectorised_lowest_point_matches_shared_helper_bitwise(self):
        import mujoco
        import mocap_g1.contact_refine as cr
        from mocap_g1.contact import lowest_point_of_geoms

        model = mujoco.MjModel.from_xml_path(str(MODEL))
        data = mujoco.MjData(model)
        feet = cr._resolve_feet(model, mujoco)
        self.assertTrue(all(info["sphere_radii"] is not None for info in feet.values()))
        rows = self._random_rows(25)
        # Exact ties (flat foot, identity root) must keep the first geom.
        rows[0] = base_qpos(1)[0]
        rows[0][2] = 0.8
        for row in rows:
            cr._fk(model, data, mujoco, row)
            for info in feet.values():
                expected_z, expected_point = lowest_point_of_geoms(
                    model, data, info["geoms"], {}, mujoco)
                low_z, point = cr._side_lowest_point(model, data, mujoco, info, {})
                self.assertEqual(np.float64(low_z).tobytes(),
                                 np.float64(expected_z).tobytes())
                self.assertEqual(point.tobytes(), expected_point.tobytes())
                self.assertEqual(
                    cr._side_lowest_z(model, data, mujoco, info, {}), expected_z)

    @requires_g1_mjcf
    def test_ankle_joints_cannot_move_the_other_foot(self):
        import mujoco
        import mocap_g1.contact_refine as cr

        model = mujoco.MjModel.from_xml_path(str(MODEL))
        feet = cr._resolve_feet(model, mujoco)
        columns = {name: G1_29DOF_JOINTS.index(name)
                   for name in cr.ANKLE_ROLE_COLUMNS.values()}
        independent = cr._geoms_independent_of_columns(
            model, mujoco, feet, sorted(columns.values()))
        for (side, _role), name in cr.ANKLE_ROLE_COLUMNS.items():
            other = "right" if side == "left" else "left"
            self.assertEqual(sorted(independent[columns[name]]),
                             sorted(feet[other]["geoms"]), name)

    def _refine_with_and_without_shortcut(self, qpos: np.ndarray):
        import mocap_g1.contact_refine as cr

        with mock.patch.object(cr, "_frame_predicate", wraps=cr._frame_predicate) as fast:
            shortcut = refine_reference_contacts(qpos, FPS, model_path=MODEL)
        with mock.patch.object(cr, "_geoms_independent_of_columns", return_value={}), \
             mock.patch.object(cr, "_frame_predicate", wraps=cr._frame_predicate) as full:
            exhaustive = refine_reference_contacts(qpos, FPS, model_path=MODEL)
        self.assertEqual(json.dumps(shortcut.report, sort_keys=True),
                         json.dumps(exhaustive.report, sort_keys=True))
        self.assertEqual(shortcut.qpos is None, exhaustive.qpos is None)
        if shortcut.qpos is not None:
            self.assertEqual(shortcut.qpos.tobytes(), exhaustive.qpos.tobytes())
        return shortcut, fast.call_count, full.call_count

    @requires_g1_mjcf
    def test_infeasible_ankle_search_shortcut_changes_nothing(self):
        # Both soles 3 mm under the plane, left toe also dug in: every ankle
        # search is provably infeasible (the other foot stays under target),
        # so the shortcut must skip them and still land on the same root lift.
        both = base_qpos(12)
        both[:, 2] = _standing_height() - 0.003
        both[:, 7 + 4] = 0.25
        result, fast_calls, full_calls = self._refine_with_and_without_shortcut(both)
        self.assertEqual(result.report["status"], "PASS", result.report["rejections"])
        strategies = {item["strategy"] for record in result.report["penetration_repairs"]
                      for item in record["repairs"]}
        self.assertEqual(strategies, {"root_z_lift"})
        self.assertLess(fast_calls, full_calls)

        # One penetrating foot, the other clear: the shortcut never applies.
        one = base_qpos(12)
        one[:, 2] = _standing_height() + 0.004
        one[:, 7 + 4] = 0.25
        result, fast_calls, full_calls = self._refine_with_and_without_shortcut(one)
        self.assertEqual(result.report["status"], "PASS", result.report["rejections"])
        self.assertEqual(fast_calls, full_calls)


if __name__ == "__main__":
    unittest.main()
