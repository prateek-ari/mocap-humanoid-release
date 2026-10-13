"""End-to-end checks of the GMR pkl root-quaternion contract (mocap_g1/gmr_pkl.py).

Every pkl WRITER must store root_rot as GMR's xyzw plus an explicit
``root_quat_order`` key, and every LOADER must refuse a keyless or
mismatched file unless an explicit legacy override is given.  The GMR-backed
writers run against a tiny stub ``general_motion_retargeting`` package (the
real one is not needed to exercise the save path), so these tests never skip.
Also covers tools/checks/validate_patched.py's exit status and the shared
contact helpers used by tools/retarget/postprocess_grounding.py.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import pickle
import subprocess
import sys
import tempfile
import textwrap
import unittest

import numpy as np

from mocap_g1.gmr_pkl import (
    RootQuatOrderError,
    declared_root_quat_order,
    resolve_root_quat_order,
    root_rot_as,
    wxyz_to_xyzw,
    xyzw_to_wxyz,
)
from mocap_g1.schema import G1_29DOF_JOINTS
from conftest import G1_MJCF as MODEL


REPO_ROOT = Path(__file__).resolve().parents[1]
# A non-trivial unit quaternion so a reorder bug cannot hide behind identity.
QPOS_ROOT_WXYZ = np.array([0.9, 0.1, 0.3, np.sqrt(1.0 - 0.81 - 0.01 - 0.09)])
BVH = """HIERARCHY
ROOT Hips
{
  OFFSET 0 0 0
  CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation
  End Site
  {
    OFFSET 0 1 0
  }
}
MOTION
Frames: 3
Frame Time: 0.0083333333
0 0 0 0 0 0
0 0 0 0 0 0
0 0 0 0 0 0
"""


def _write_stub_gmr(root: Path, robot_xml: Path | None = None) -> Path:
    package = root / "general_motion_retargeting"
    (package / "utils").mkdir(parents=True, exist_ok=True)
    root_wxyz = ", ".join(repr(float(value)) for value in QPOS_ROOT_WXYZ)
    (package / "__init__.py").write_text(textwrap.dedent(f"""
        import numpy as np

        class GeneralMotionRetargeting:
            def __init__(self, *args, **kwargs):
                pass

            def retarget(self, frame, offset_to_ground=False):
                qpos = np.zeros(36)
                qpos[:3] = [0.1, 0.2, 0.8]
                qpos[3:7] = [{root_wxyz}]  # MuJoCo qpos order: wxyz
                return qpos
        """), encoding="utf-8")
    (package / "utils/__init__.py").write_text("", encoding="utf-8")
    (package / "utils/lafan1.py").write_text(
        "def load_bvh_file(path, format='lafan1'):\n    return [object()] * 4, 1.75\n",
        encoding="utf-8",
    )
    (package / "utils/xsens.py").write_text(
        "def load_xsens_file(args):\n    return [object()] * 4, 1.75, 1.0 / 120.0\n",
        encoding="utf-8",
    )
    (package / "params.py").write_text(
        f"ROBOT_XML_DICT = {{'unitree_g1': {str(robot_xml or '')!r}}}\n", encoding="utf-8"
    )
    return root


def _run(args: list[str], *, stub: Path, cwd: Path = REPO_ROOT) -> subprocess.CompletedProcess:
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join([str(stub), env.get("PYTHONPATH", "")])
    return subprocess.run(
        [sys.executable, *args], cwd=cwd, env=env, capture_output=True, text=True, timeout=180,
    )


def _load(path: Path) -> dict:
    with path.open("rb") as handle:
        return pickle.load(handle)


class ResolverTests(unittest.TestCase):
    def test_resolver_contract(self):
        self.assertEqual(resolve_root_quat_order({"root_quat_order": "xyzw"}), "xyzw")
        with self.assertRaisesRegex(RootQuatOrderError, "no 'root_quat_order' key"):
            resolve_root_quat_order({})
        self.assertEqual(resolve_root_quat_order({}, legacy_order="wxyz"), "wxyz")
        with self.assertRaisesRegex(RootQuatOrderError, "must be 'xyzw'"):
            resolve_root_quat_order({"root_quat_order": "wxyz"})
        with self.assertRaisesRegex(RootQuatOrderError, "conflicts"):
            resolve_root_quat_order({"root_quat_order": "xyzw"}, legacy_order="wxyz")
        with self.assertRaisesRegex(RootQuatOrderError, "legacy quaternion order"):
            resolve_root_quat_order({}, legacy_order="zyxw")
        with self.assertRaisesRegex(RootQuatOrderError, "conflicting"):
            declared_root_quat_order(
                {"root_quat_order": "xyzw", "metadata": {"layout": {"root_rot": "wxyz"}}}
            )

    def test_root_rot_as_validates_and_converts(self):
        xyzw = np.tile(QPOS_ROOT_WXYZ[[1, 2, 3, 0]], (3, 1))
        motion = {"root_rot": xyzw, "root_quat_order": "xyzw"}
        np.testing.assert_allclose(root_rot_as(motion, "wxyz")[0], QPOS_ROOT_WXYZ)
        bad = {"root_rot": xyzw * 2.0, "root_quat_order": "xyzw"}
        with self.assertRaisesRegex(ValueError, "non-unit quaternion"):
            root_rot_as(bad, "wxyz")
        nan = xyzw.copy()
        nan[1, 2] = np.nan
        with self.assertRaisesRegex(ValueError, "non-finite"):
            root_rot_as({"root_rot": nan, "root_quat_order": "xyzw"}, "wxyz")

    def test_permutation_helpers_match_the_former_hand_written_indexing(self):
        # The call sites that used to spell these permutations inline:
        # 2-D slices ``q[:, [1, 2, 3, 0]]`` / ``q[:, [3, 0, 1, 2]]``, 1-D rows
        # ``q[[1, 2, 3, 0]]`` and the ``x, y, z, w = row; [w, x, y, z]`` unpack.
        rng = np.random.default_rng(3)
        wxyz = rng.normal(size=(6, 4))
        xyzw = rng.normal(size=(6, 4))
        np.testing.assert_array_equal(wxyz_to_xyzw(wxyz), wxyz[:, [1, 2, 3, 0]])
        np.testing.assert_array_equal(xyzw_to_wxyz(xyzw), xyzw[:, [3, 0, 1, 2]])
        np.testing.assert_array_equal(wxyz_to_xyzw(wxyz[2]), wxyz[2][[1, 2, 3, 0]])
        x, y, z, w = xyzw[4]
        np.testing.assert_array_equal(xyzw_to_wxyz(xyzw[4]), [w, x, y, z])
        np.testing.assert_array_equal(xyzw_to_wxyz(wxyz_to_xyzw(wxyz)), wxyz)
        qpos = rng.normal(size=(5, 36))
        np.testing.assert_array_equal(
            wxyz_to_xyzw(qpos[:, 3:7]), qpos[:, 3:7][:, [1, 2, 3, 0]]
        )
        # A copy, never a view: callers mutate the result.
        converted = wxyz_to_xyzw(wxyz)
        converted[0, 0] = 99.0
        self.assertNotEqual(wxyz[0, 1], 99.0)
        self.assertEqual(xyzw_to_wxyz(xyzw.astype(np.float32)).dtype, np.float32)


class WriterTests(unittest.TestCase):
    def setUp(self):
        self._directory = tempfile.TemporaryDirectory()
        self.root = Path(self._directory.name)
        self.stub = _write_stub_gmr(self.root / "stub")

    def tearDown(self):
        self._directory.cleanup()

    def _assert_gmr_xyzw(self, pkl: dict) -> None:
        self.assertEqual(pkl["root_quat_order"], "xyzw")
        np.testing.assert_allclose(pkl["root_rot"][0], QPOS_ROOT_WXYZ[[1, 2, 3, 0]])
        np.testing.assert_allclose(root_rot_as(pkl, "wxyz")[0], QPOS_ROOT_WXYZ)

    def test_lafan_headless_writers_emit_xyzw_and_the_key(self):
        bvh = self.root / "clip.bvh"
        bvh.write_text(BVH, encoding="utf-8")
        for script in ("retarget_headless.py", "retarget_patched.py"):
            with self.subTest(script=script):
                output = self.root / f"{script}.pkl"
                completed = _run(
                    [str(REPO_ROOT / "tools/retarget" / script), "--bvh_file", str(bvh),
                     "--save_path", str(output)],
                    stub=self.stub,
                )
                self.assertEqual(completed.returncode, 0, completed.stderr)
                self._assert_gmr_xyzw(_load(output))

    def test_xsens_headless_writer_now_emits_xyzw_not_wxyz(self):
        bvh = self.root / "clip.bvh"
        bvh.write_text(BVH, encoding="utf-8")
        offsets = self.root / "offsets.json"
        offsets.write_text(json.dumps({"Hips": {"X": 0, "Y": 0, "Z": 0}}), encoding="utf-8")
        output = self.root / "xsens.pkl"
        completed = _run(
            [str(REPO_ROOT / "tools/retarget/retarget_xsens_headless.py"),
             "--gmr-root", str(self.stub), "--bvh-file", str(bvh),
             "--offsets-file", str(offsets), "--save-path", str(output), "--scale", "0.01"],
            stub=self.stub,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self._assert_gmr_xyzw(_load(output))


class ValidatePatchedTests(unittest.TestCase):
    CLIPS = ("dance1_subject2", "walk3_subject1", "run1_subject5",
             "fight1_subject3", "obstacles1_subject1", "fallAndGetUp3_subject1")

    @classmethod
    def setUpClass(cls):
        if not MODEL.is_file():
            raise unittest.SkipTest("external GMR MuJoCo model is unavailable")

    def _suite(self, root: Path, *, bad_clip: str | None = None) -> tuple[Path, Path]:
        patched, unpatched = root / "patched", root / "unpatched"
        for directory in (patched, unpatched):
            directory.mkdir()
            for clip in self.CLIPS:
                motion = {
                    "fps": 30,
                    "root_pos": np.tile([0.0, 0.0, 0.8], (12, 1)),
                    "root_rot": np.tile([0.0, 0.0, 0.0, 1.0], (12, 1)),
                    "root_quat_order": "xyzw",
                    "dof_pos": np.zeros((12, len(G1_29DOF_JOINTS))),
                }
                if directory is patched and clip == bad_clip:
                    motion["dof_pos"][4, 3] = 50.0  # far outside every knee range
                with (directory / f"{clip}.pkl").open("wb") as handle:
                    pickle.dump(motion, handle)
        return patched, unpatched

    def _validate(self, root: Path, patched: Path, unpatched: Path, *extra: str):
        stub = _write_stub_gmr(root / "stub", robot_xml=MODEL)
        return _run(
            [str(REPO_ROOT / "tools/checks/validate_patched.py"), "--patched", str(patched),
             "--unpatched", str(unpatched), "--stride", "4", *extra],
            stub=stub,
        )

    def test_clean_suite_exits_zero(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            completed = self._validate(root, *self._suite(root))
            self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)
            self.assertIn("PASS: all 6 clips", completed.stdout)

    def test_any_failed_criterion_exits_nonzero(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            completed = self._validate(root, *self._suite(root, bad_clip="run1_subject5"))
            self.assertEqual(completed.returncode, 1, completed.stdout + completed.stderr)
            self.assertIn("run1_subject5", completed.stdout.splitlines()[-1])

    def test_missing_clip_and_unlabeled_order_fail(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            patched, unpatched = self._suite(root)
            (unpatched / "walk3_subject1.pkl").unlink()
            completed = self._validate(root, patched, unpatched)
            self.assertEqual(completed.returncode, 1)
            self.assertIn("walk3_subject1", completed.stdout)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            patched, unpatched = self._suite(root)
            for path in patched.glob("*.pkl"):
                motion = _load(path)
                del motion["root_quat_order"]
                with path.open("wb") as handle:
                    pickle.dump(motion, handle)
            self.assertEqual(self._validate(root, patched, unpatched).returncode, 1)
            # ...but the explicit legacy override restores a passing run.
            completed = self._validate(root, patched, unpatched, "--quat-order", "xyzw")
            self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)


class PostprocessGroundingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not MODEL.is_file():
            raise unittest.SkipTest("external GMR MuJoCo model is unavailable")

    def _input(self, path: Path, **extra) -> Path:
        motion = {
            "fps": 30,
            "root_pos": np.tile([0.0, 0.0, 0.85], (20, 1)),
            "root_rot": np.tile([0.0, 0.0, 0.0, 1.0], (20, 1)),
            "dof_pos": np.zeros((20, len(G1_29DOF_JOINTS))),
            **extra,
        }
        with path.open("wb") as handle:
            pickle.dump(motion, handle)
        return path

    def _run(self, source: Path, output: Path, *extra: str) -> subprocess.CompletedProcess:
        return subprocess.run(
            [sys.executable, str(REPO_ROOT / "tools/retarget/postprocess_grounding.py"),
             "-i", str(source), "-o", str(output), "--mjcf", str(MODEL), *extra],
            cwd=REPO_ROOT, capture_output=True, text=True, timeout=120,
        )

    def test_output_is_grounded_and_declares_xyzw(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = self._input(root / "in.pkl", root_quat_order="xyzw")
            output = root / "out.pkl"
            completed = self._run(source, output)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            grounded = _load(output)
            self.assertEqual(grounded["root_quat_order"], "xyzw")
            self.assertLess(float(grounded["root_pos"][0, 2]), 0.85)

    def test_unlabeled_input_requires_explicit_legacy_order(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = self._input(root / "in.pkl")
            completed = self._run(source, root / "refused.pkl")
            self.assertNotEqual(completed.returncode, 0)
            self.assertIn("root_quat_order", completed.stderr)
            self.assertFalse((root / "refused.pkl").exists())
            completed = self._run(source, root / "legacy.pkl", "--quat-order", "xyzw")
            self.assertEqual(completed.returncode, 0, completed.stderr)


class SharedContactHelperTests(unittest.TestCase):
    def test_material_point_slip_ignores_corner_switching(self):
        from mocap_g1.contact import material_point_slip_xy

        # Stationary foot whose argmin lowest point alternates between two
        # corners 0.17 m apart: slip of the carried material point is zero.
        frames = 6
        body_pos = np.zeros((frames, 3))
        body_mat = np.tile(np.eye(3), (frames, 1, 1))
        corners = np.array([[-0.05, 0.0, -0.04], [0.12, 0.0, -0.04]])
        points = corners[np.arange(frames) % 2]
        self.assertEqual(float(np.abs(material_point_slip_xy(points, body_pos, body_mat)).max()),
                         0.0)
        # A rigid translation of the foot is measured exactly.
        body_pos[:, 0] = 0.01 * np.arange(frames)
        slip = material_point_slip_xy(points + body_pos, body_pos, body_mat)
        np.testing.assert_allclose(slip[1:, 0], 0.01)
        np.testing.assert_allclose(slip[1:, 1], 0.0)

    def test_contact_predicate_and_central_diff(self):
        from mocap_g1.contact import central_diff, contact_candidates

        height = np.array([0.0, 0.01, 0.02, 0.05])
        vz = central_diff(height, 10.0)
        np.testing.assert_allclose(vz, [0.1, 0.1, 0.2, 0.3])
        np.testing.assert_array_equal(
            contact_candidates(height, vz, height_max=0.03, vz_max=0.15),
            [True, True, False, False],
        )
        np.testing.assert_array_equal(
            contact_candidates(height, None, height_max=0.03, vz_max=None),
            [True, True, True, False],
        )
        with self.assertRaisesRegex(ValueError, "vertical_speed"):
            contact_candidates(height, None, height_max=0.03, vz_max=0.1)


if __name__ == "__main__":
    unittest.main()
