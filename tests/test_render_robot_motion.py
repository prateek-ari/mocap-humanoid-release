from __future__ import annotations

from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from tools.retarget.render_robot_motion import (
    RenderError,
    load_motion,
    partial_video_path,
    render_motion,
    validate_motion_payload,
)


def payload(frames: int = 4, dof: int = 2) -> dict[str, object]:
    root_pos = np.zeros((frames, 3), dtype=np.float64)
    # GMR stores xyzw. Identity is therefore [0, 0, 0, 1].
    root_rot = np.zeros((frames, 4), dtype=np.float64)
    root_rot[:, 3] = 1.0
    return {
        "fps": 60,
        "root_pos": root_pos,
        "root_rot": root_rot,
        "root_quat_order": "xyzw",
        "dof_pos": np.zeros((frames, dof), dtype=np.float64),
    }


class FakeViewer:
    instances: list["FakeViewer"] = []

    def __init__(self, **kwargs: object) -> None:
        self.kwargs = kwargs
        self.viewer = self
        self.steps: list[dict[str, object]] = []
        self.closed = False
        FakeViewer.instances.append(self)

    def is_running(self) -> bool:
        return True

    def step(self, **kwargs: object) -> None:
        self.steps.append(kwargs)

    def close(self) -> None:
        self.closed = True
        video_path = self.kwargs.get("video_path")
        if self.kwargs.get("record_video") and video_path:
            Path(str(video_path)).write_bytes(b"complete video")


class RenderRobotMotionTests(unittest.TestCase):
    def setUp(self) -> None:
        FakeViewer.instances.clear()

    def test_validate_gmr_xyzw_and_convert_to_mujoco_wxyz(self) -> None:
        arrays = validate_motion_payload(
            {
                **payload(frames=1, dof=2),
                "root_rot": np.asarray([[0.1, 0.2, 0.3, 0.92736185]]),
            }
        )
        np.testing.assert_allclose(arrays.root_rot_wxyz[0], [0.92736185, 0.1, 0.2, 0.3], atol=1e-7)
        self.assertEqual(arrays.frames, 1)
        self.assertEqual(arrays.dof_pos.shape, (1, 2))

    def test_validate_rejects_non_xyzw_declared_convention(self) -> None:
        with self.assertRaisesRegex(RenderError, "must be xyzw"):
            validate_motion_payload({**payload(), "quaternion_order": "wxyz"})

    def test_validate_requires_an_explicit_quaternion_order(self) -> None:
        keyless = payload()
        del keyless["root_quat_order"]
        with self.assertRaisesRegex(RenderError, "no 'root_quat_order' key"):
            validate_motion_payload(keyless)
        # An explicit legacy override is honoured, in either direction.
        arrays = validate_motion_payload(keyless, legacy_quat_order="xyzw")
        np.testing.assert_array_equal(arrays.root_rot_wxyz[0], [1.0, 0.0, 0.0, 0.0])
        legacy_wxyz = {**keyless, "root_rot": np.tile([1.0, 0.0, 0.0, 0.0], (4, 1))}
        arrays = validate_motion_payload(legacy_wxyz, legacy_quat_order="wxyz")
        np.testing.assert_array_equal(arrays.root_rot_wxyz[0], [1.0, 0.0, 0.0, 0.0])
        # retarget_robot_recorded.py's metadata.layout declaration also counts.
        declared = {**keyless, "metadata": {"layout": {"root_rot": "xyzw"}}}
        self.assertEqual(validate_motion_payload(declared).frames, 4)

    def test_validate_rejects_mismatched_or_conflicting_order(self) -> None:
        with self.assertRaisesRegex(RenderError, "must be 'xyzw'"):
            validate_motion_payload({**payload(), "root_quat_order": "wxyz"})
        with self.assertRaisesRegex(RenderError, "conflicts with the declared"):
            validate_motion_payload(payload(), legacy_quat_order="wxyz")
        with self.assertRaisesRegex(RenderError, "must be xyzw"):
            validate_motion_payload(
                {**payload(), "metadata": {"layout": {"root_rot": "wxyz"}}}
            )

    def test_validate_rejects_bad_shapes_nonfinite_and_nonunit_quaternions(self) -> None:
        with self.assertRaisesRegex(RenderError, "same frame count"):
            validate_motion_payload({**payload(), "dof_pos": np.zeros((3, 2))})
        with self.assertRaisesRegex(RenderError, "non-finite"):
            bad = payload()
            bad["root_pos"] = np.full((4, 3), np.nan)
            validate_motion_payload(bad)
        with self.assertRaisesRegex(RenderError, "unit quaternions"):
            bad = payload()
            bad["root_rot"] = np.zeros((4, 4))
            validate_motion_payload(bad)
        with self.assertRaisesRegex(RenderError, "at least one articulated joint"):
            validate_motion_payload(payload(dof=0))

    def test_render_uses_dynamic_nq_frame_step_and_finite_loops(self) -> None:
        arrays = validate_motion_payload(payload(frames=5, dof=2))
        with patch(
            "tools.retarget.render_robot_motion.expected_robot_nq", return_value=9
        ):
            rendered = render_motion(
                gmr_root=Path("/unused"),
                robot="booster_t1_29dof",
                motion=arrays,
                loops=2,
                frame_step=2,
                viewer_class=FakeViewer,
            )
        self.assertEqual(rendered, 6)
        viewer = FakeViewer.instances[0]
        self.assertTrue(viewer.closed)
        self.assertEqual(viewer.kwargs["motion_fps"], 30.0)
        self.assertEqual(len(viewer.steps), 6)
        np.testing.assert_allclose(viewer.steps[0]["root_rot"], [1, 0, 0, 0])
        np.testing.assert_allclose(viewer.steps[1]["root_rot"], [1, 0, 0, 0])
        self.assertEqual(viewer.steps[0]["dof_pos"].shape, (2,))

    def test_video_is_promoted_atomically_only_after_close(self) -> None:
        arrays = validate_motion_payload(payload(frames=3, dof=2))
        with tempfile.TemporaryDirectory() as temporary:
            final = Path(temporary) / "review.mp4"
            with patch(
                "tools.retarget.render_robot_motion.expected_robot_nq", return_value=9
            ):
                rendered = render_motion(
                    gmr_root=Path("/unused"),
                    robot="booster_t1_29dof",
                    motion=arrays,
                    loops=1,
                    record_video=True,
                    video_path=final,
                    viewer_class=FakeViewer,
                )
            self.assertEqual(rendered, 3)
            self.assertEqual(final.read_bytes(), b"complete video")
            self.assertFalse(partial_video_path(final).exists())

    def test_existing_video_requires_explicit_overwrite(self) -> None:
        arrays = validate_motion_payload(payload(frames=1, dof=2))
        with tempfile.TemporaryDirectory() as temporary:
            final = Path(temporary) / "review.mp4"
            final.write_bytes(b"old")
            with patch(
                "tools.retarget.render_robot_motion.expected_robot_nq", return_value=9
            ):
                with self.assertRaisesRegex(RenderError, "refusing to overwrite"):
                    render_motion(
                        gmr_root=Path("/unused"),
                        robot="booster_t1_29dof",
                        motion=arrays,
                        record_video=True,
                        video_path=final,
                        viewer_class=FakeViewer,
                    )

    def test_load_motion_rejects_missing_file(self) -> None:
        with self.assertRaisesRegex(RenderError, "does not exist"):
            load_motion(Path("/definitely/not/a/motion.pkl"))


if __name__ == "__main__":
    unittest.main()
