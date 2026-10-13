"""Hardware- and GMR-free tests for the replayer's ``--xsens-bvh`` path."""

from __future__ import annotations

import contextlib
import io
import sys
import unittest
from unittest import mock

import numpy as np

from mocap_g1.mxtp02 import XSENS_SEGMENTS
from tools import mxtp02_replayer


def fake_xsens_frame():
    """A GMR Xsens-loader frame: bone -> (position m, unnormalised wxyz quat)."""
    rng = np.random.default_rng(7)
    frame = {}
    for index, bone in enumerate(sorted(set(mxtp02_replayer.XSENS_BVH_TO_SEGMENT.values()))):
        quat = rng.normal(size=4)
        frame[bone] = (np.array([index, -index, 0.01 * index]), (2.0 + index) * quat)
    frame["UnusedBone"] = (np.full(3, 99.0), np.array([1.0, 0, 0, 0]))
    return frame


class XsensBvhSegmentsTests(unittest.TestCase):
    def test_maps_every_mvn_segment_in_wire_order(self):
        self.assertEqual(set(mxtp02_replayer.XSENS_BVH_TO_SEGMENT), set(XSENS_SEGMENTS))
        bones = list(mxtp02_replayer.XSENS_BVH_TO_SEGMENT.values())
        self.assertEqual(len(bones), len(set(bones)), "a bone feeds two segments")

    def test_frame_becomes_23_segment_arrays_with_unchanged_orientations(self):
        frame = fake_xsens_frame()
        pos, quat = mxtp02_replayer.xsens_bvh_segments(frame)
        self.assertEqual(pos.shape, (23, 3))
        self.assertEqual(quat.shape, (23, 4))
        for index, segment in enumerate(XSENS_SEGMENTS):
            with self.subTest(segment=segment):
                bone_pos, bone_quat = frame[mxtp02_replayer.XSENS_BVH_TO_SEGMENT[segment]]
                np.testing.assert_array_equal(pos[index], bone_pos)
                np.testing.assert_allclose(quat[index], bone_quat / np.linalg.norm(bone_quat))
                self.assertAlmostEqual(float(np.linalg.norm(quat[index])), 1.0)
        self.assertNotIn(99.0, pos)

    def test_segments_build_a_valid_body_datagram(self):
        pos, quat = mxtp02_replayer.xsens_bvh_segments(fake_xsens_frame())
        datagram = mxtp02_replayer.build_datagram(1, 2, pos, quat)
        self.assertTrue(datagram.startswith(b"MXTP02"))


class XsensBvhArgumentTests(unittest.TestCase):
    def run_main(self, *argv):
        stderr = io.StringIO()
        never = mock.Mock(side_effect=AssertionError("loading must not start"))
        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(sys, "argv", ["mxtp02_replayer.py", *argv]))
            for loader in ("load_xsens_bvh_frames", "load_bvh_frames", "synthetic_frames"):
                stack.enter_context(mock.patch.object(mxtp02_replayer, loader, never))
            stack.enter_context(mock.patch("socket.socket", never))
            stack.enter_context(contextlib.redirect_stderr(stderr))
            with self.assertRaises(SystemExit) as raised:
                mxtp02_replayer.main()
        never.assert_not_called()
        return raised.exception.code, stderr.getvalue()

    def test_xsens_bvh_cannot_be_combined_with_synthetic_or_bvh(self):
        for extra in (["--synthetic"], ["--bvh", "clip.bvh"]):
            with self.subTest(extra=extra):
                code, stderr = self.run_main("--xsens-bvh", "take.bvh", *extra)
                self.assertEqual(code, 2)
                self.assertIn("--xsens-bvh cannot be combined", stderr)

    def test_negative_window_is_rejected(self):
        for flag in ("--start-seconds", "--seconds"):
            with self.subTest(flag=flag):
                code, stderr = self.run_main("--xsens-bvh", "take.bvh", flag, "-1")
                self.assertEqual(code, 2)
                self.assertIn("cannot be negative", stderr)


if __name__ == "__main__":
    unittest.main()
