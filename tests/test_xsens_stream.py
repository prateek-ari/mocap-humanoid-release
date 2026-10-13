from __future__ import annotations

import unittest

from mocap_g1.mxtp02 import (
    XSENS_JOINT_SEGMENT_PAIRS,
    XSENS_SEGMENTS,
    build_body_datagram,
    build_joint_angles_datagram,
)
from mocap_g1.xsens_stream import XsensStreamAcceptance


def body_positions(scale: float = 1.0):
    positions = [(0.0, 0.0, 1.0)] * len(XSENS_SEGMENTS)
    positions[XSENS_SEGMENTS.index("head")] = (0.0, 0.0, 1.8)
    positions[XSENS_SEGMENTS.index("right_hand")] = (0.0, -0.9, 1.45)
    positions[XSENS_SEGMENTS.index("left_hand")] = (0.0, 0.9, 1.45)
    positions[XSENS_SEGMENTS.index("right_toe")] = (0.3, -0.1, 0.05)
    positions[XSENS_SEGMENTS.index("left_toe")] = (0.3, 0.1, 0.05)
    return [tuple(value * scale for value in position) for position in positions]


class XsensStreamAcceptanceTests(unittest.TestCase):
    def feed_frame(
        self,
        acceptance: XsensStreamAcceptance,
        sample: int,
        *,
        received_at: float,
        scale: float = 1.0,
        with_joints: bool = True,
        quaternion=(1.0, 0.0, 0.0, 0.0),
    ) -> None:
        time_code = round(sample * 1000 / 60)
        acceptance.observe(
            build_body_datagram(
                sample,
                time_code,
                body_positions(scale),
                [quaternion] * len(XSENS_SEGMENTS),
            ),
            received_at=received_at,
            source="192.0.2.10:49152",
        )
        if with_joints:
            acceptance.observe(
                build_joint_angles_datagram(
                    sample,
                    time_code,
                    [(0.0, 0.0, 0.0)] * len(XSENS_JOINT_SEGMENT_PAIRS),
                ),
                received_at=received_at,
                source="192.0.2.10:49152",
            )

    def test_passes_paired_si_unit_stream(self):
        acceptance = XsensStreamAcceptance(require_joint_angles=True)
        for sample in range(60):
            self.feed_frame(acceptance, sample, received_at=100.0 + sample / 60)

        report = acceptance.report()
        self.assertEqual(report["verdict"], "PASS")
        self.assertAlmostEqual(report["body"]["observed_receive_hz"], 60.0)
        self.assertAlmostEqual(report["body"]["time_code_hz"], 60.0, delta=0.1)
        self.assertEqual(report["body_joint_pairing_fraction"], 1.0)
        self.assertEqual(
            report["geometry"]["translation_unit_assessment"],
            "metres_plausible",
        )

    def test_fails_closed_on_centimetres_loss_and_missing_joints(self):
        acceptance = XsensStreamAcceptance(
            require_joint_angles=True,
            maximum_loss_fraction=0.01,
        )
        samples = list(range(20)) + list(range(22, 62))
        for index, sample in enumerate(samples):
            self.feed_frame(
                acceptance,
                sample,
                received_at=200.0 + index / 60,
                scale=100.0,
                with_joints=False,
            )

        report = acceptance.report()
        self.assertEqual(report["verdict"], "FAIL")
        self.assertEqual(
            report["geometry"]["translation_unit_assessment"],
            "centimetres_likely",
        )
        failures = "\n".join(report["failures"])
        self.assertIn("body packet loss", failures)
        self.assertIn("MXTP20 joint frames", failures)
        self.assertIn("centimetres_likely", failures)

    def test_reports_bad_quaternions_and_malformed_packets(self):
        acceptance = XsensStreamAcceptance(minimum_body_frames=1, minimum_rate_hz=1)
        self.feed_frame(
            acceptance,
            0,
            received_at=300.0,
            with_joints=False,
            quaternion=(2.0, 0.0, 0.0, 0.0),
        )
        acceptance.observe(b"MXTP02 broken", received_at=300.1)

        report = acceptance.report()
        self.assertEqual(report["verdict"], "FAIL")
        self.assertEqual(report["malformed_datagrams"], 1)
        self.assertEqual(report["geometry"]["bad_quaternion_norms"], 23)

    def test_late_counter_is_reordered_not_double_counted_as_loss(self):
        acceptance = XsensStreamAcceptance(minimum_body_frames=1, minimum_rate_hz=1)
        for index, sample in enumerate((1, 2, 4, 3, 5, 6)):
            self.feed_frame(
                acceptance, sample, received_at=400.0 + index / 60, with_joints=False
            )

        body = acceptance.report()["body"]
        self.assertEqual(body["estimated_dropped_frames"], 0)
        self.assertEqual(body["out_of_order_frames"], 1)
        self.assertEqual(body["duplicate_frames"], 0)
        self.assertEqual(body["loss_fraction"], 0.0)

    def test_real_gap_survives_reordering_and_late_duplicates(self):
        acceptance = XsensStreamAcceptance(minimum_body_frames=1, minimum_rate_hz=1)
        # 4 and 7 never arrive; 5 arrives late; 2 is repeated late.
        for index, sample in enumerate((1, 2, 3, 6, 5, 8, 2, 9)):
            self.feed_frame(
                acceptance, sample, received_at=500.0 + index / 60, with_joints=False
            )

        body = acceptance.report()["body"]
        self.assertEqual(body["estimated_dropped_frames"], 2)
        self.assertEqual(body["out_of_order_frames"], 1)
        self.assertEqual(body["duplicate_frames"], 1)
        # Time-code steps come only from frames that advanced the counter.
        self.assertTrue(all(step > 0 for step in acceptance.body.time_steps_ms))

    def test_counter_wrap_is_not_loss_but_a_real_gap_across_it_is(self):
        acceptance = XsensStreamAcceptance(minimum_body_frames=1, minimum_rate_hz=1)
        # 0xFFFFFFFE -> 0xFFFFFFFF -> 0 is contiguous; 0 -> 3 loses two frames.
        for index, sample in enumerate((0xFFFFFFFE, 0xFFFFFFFF, 0, 3)):
            self.feed_frame(
                acceptance, sample, received_at=600.0 + index / 60, with_joints=False
            )

        body = acceptance.report()["body"]
        self.assertEqual(body["estimated_dropped_frames"], 2)
        self.assertEqual(body["out_of_order_frames"], 0)
        self.assertEqual(body["duplicate_frames"], 0)


if __name__ == "__main__":
    unittest.main()
