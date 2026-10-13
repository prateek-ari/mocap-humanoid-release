from __future__ import annotations

import csv
import json
from pathlib import Path
import pickle
import tempfile
import unittest

import numpy as np

from mocap_g1.dataset import (
    DatasetError,
    load_and_validate_job,
    load_canonical_motion,
    run_pipeline,
)
from mocap_g1.schema import G1_29DOF_JOINTS


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
Frames: 2
Frame Time: 0.0083333333
0 0 0 0 0 0
0 0 0 0 0 0
"""


class DatasetTests(unittest.TestCase):
    def make_job(
        self,
        root: Path,
        *,
        license_scope: str = "commissioned",
        distribution: str = "client",
    ) -> Path:
        (root / "clip.bvh").write_text(BVH, encoding="utf-8")
        (root / "offsets.json").write_text(
            '{"Hips": {"X": 0, "Y": 0, "Z": 0}}\n',
            encoding="utf-8",
        )
        job = {
            "schema_version": 1,
            "dataset_id": "test-session",
            "robot": "unitree_g1",
            "distribution": distribution,
            "license_scope": license_scope,
            "retarget_output_quaternion_order": "wxyz",
            "grounded": False,
            "subject": {
                "id": "subject-1",
                "height_m": 1.75,
                "calibration_pose": "N-pose",
            },
            "capture": {
                "software_version": "test",
                "export_format": "3DSM",
                "fps": 120,
                "position_scale_to_m": 0.01,
            },
            "offsets_file": "offsets.json",
            "clips": [{"name": "clip", "bvh": "clip.bvh"}],
        }
        path = root / "job.json"
        path.write_text(json.dumps(job), encoding="utf-8")
        return path

    def test_validates_and_finalizes_existing_motion(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            job = load_and_validate_job(self.make_job(root, distribution="internal_rnd"))
            existing = root / "existing"
            existing.mkdir()
            frames = 2
            motion = {
                "fps": 120,
                "root_pos": np.zeros((frames, 3)),
                "root_rot": np.tile([1.0, 0.0, 0.0, 0.0], (frames, 1)),
                "dof_pos": np.zeros((frames, len(G1_29DOF_JOINTS))),
            }
            with (existing / "clip.pkl").open("wb") as handle:
                pickle.dump(motion, handle)
            manifest_path = run_pipeline(
                job,
                root / "output",
                existing_pkl_dir=existing,
                skip_score=True,
            )
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            self.assertEqual(manifest["robot"], "unitree_g1")
            self.assertEqual(manifest["joint_names"], list(G1_29DOF_JOINTS))
            self.assertEqual(manifest["clips"][0]["frames"], 2)
            canonical_path = manifest_path.parent / "pkl/clip.pkl"
            with canonical_path.open("rb") as handle:
                canonical = pickle.load(handle)
            np.testing.assert_array_equal(canonical["root_rot"][0], [0, 0, 0, 1])
            self.assertEqual(canonical["root_quat_order"], "xyzw")
            with (manifest_path.parent / "csv/clip.csv").open(newline="", encoding="utf-8") as handle:
                header = next(csv.reader(handle))
            self.assertEqual(header[-29:], list(G1_29DOF_JOINTS))

    def _write_motion(self, path: Path, **extra) -> Path:
        motion = {
            "fps": 120,
            "root_pos": np.zeros((2, 3)),
            "root_rot": np.tile([0.0, 0.0, 0.0, 1.0], (2, 1)),
            "dof_pos": np.zeros((2, len(G1_29DOF_JOINTS))),
            **extra,
        }
        with path.open("wb") as handle:
            pickle.dump(motion, handle)
        return path

    def test_quaternion_order_must_be_declared_or_explicitly_overridden(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            keyless = self._write_motion(root / "keyless.pkl")
            with self.assertRaisesRegex(DatasetError, "root_quat_order"):
                load_canonical_motion(keyless)
            self.assertEqual(
                load_canonical_motion(keyless, quaternion_order="xyzw")["root_quat_order"],
                "xyzw",
            )
            declared = self._write_motion(root / "declared.pkl", root_quat_order="xyzw")
            motion = load_canonical_motion(declared)
            np.testing.assert_array_equal(motion["root_rot"][0], [0, 0, 0, 1])
            with self.assertRaisesRegex(DatasetError, "conflicts"):
                load_canonical_motion(declared, quaternion_order="wxyz")
            wrong = self._write_motion(root / "wrong.pkl", root_quat_order="wxyz")
            with self.assertRaisesRegex(DatasetError, "must be 'xyzw'"):
                load_canonical_motion(wrong)

    def test_job_without_legacy_override_refuses_keyless_existing_motion(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            job_path = self.make_job(root, distribution="internal_rnd")
            job = json.loads(job_path.read_text(encoding="utf-8"))
            del job["retarget_output_quaternion_order"]
            job_path.write_text(json.dumps(job), encoding="utf-8")
            existing = root / "existing"
            existing.mkdir()
            self._write_motion(existing / "clip.pkl")
            with self.assertRaisesRegex(DatasetError, "root_quat_order"):
                run_pipeline(load_and_validate_job(job_path), root / "output",
                             existing_pkl_dir=existing, skip_score=True)
            job["retarget_output_quaternion_order"] = "zyxw"
            job_path.write_text(json.dumps(job), encoding="utf-8")
            with self.assertRaisesRegex(DatasetError, "legacy override"):
                load_and_validate_job(job_path)

    def test_client_job_rejects_noncommissioned_data(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            with self.assertRaises(DatasetError):
                load_and_validate_job(self.make_job(root, license_scope="internal_rnd"))

    def test_rejects_placeholder_offsets_without_bvh_joint_coverage(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            job_path = self.make_job(root)
            (root / "offsets.json").write_text(
                '{"placeholder": {"X": 0, "Y": 0, "Z": 0}}\n',
                encoding="utf-8",
            )
            with self.assertRaisesRegex(DatasetError, "lacks 1 BVH joints"):
                load_and_validate_job(job_path)

    def test_client_job_cannot_skip_scoring(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            job = load_and_validate_job(self.make_job(root))
            with self.assertRaisesRegex(DatasetError, "cannot use --skip-score"):
                run_pipeline(job, root / "output", skip_score=True)


if __name__ == "__main__":
    unittest.main()
