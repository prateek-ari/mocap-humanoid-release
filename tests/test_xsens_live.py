import math
import socket
import unittest
from dataclasses import replace
from unittest.mock import Mock, patch

import numpy as np

from mocap_g1.mxtp02 import (
    XSENS_SEGMENTS,
    build_body_datagram,
    build_joint_angles_datagram,
    SegmentPose,
)
from mocap_g1.mxtp02 import ProtocolError
from mocap_g1.xsens_live import (
    GmrFrameConverter,
    LatestBodyReceiver,
    latest_body_from_packets,
)
from tools.xsens.live_g1_sim import _dropped_between


def body_packet(counter=1, yaw=0.0):
    positions = [(float(index), 0.0, 1.0) for index in range(len(XSENS_SEGMENTS))]
    quat = (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))
    return build_body_datagram(counter, counter * 17, positions, [quat] * len(positions))


class XsensLiveTests(unittest.TestCase):
    def test_keeps_latest_body_when_joint_packet_is_last(self):
        packets = [
            (body_packet(7), ("192.168.1.246", 40000)),
            (
                build_joint_angles_datagram(7, 119, [(0.0, 0.0, 0.0)] * 22),
                ("192.168.1.246", 40000),
            ),
        ]
        result = latest_body_from_packets(packets, expected_source="192.168.1.246")
        self.assertEqual(result.body.sample_counter, 7)
        self.assertEqual(result.body_packets, 1)
        self.assertEqual(result.ignored_packets, 1)

    def test_filters_unexpected_source(self):
        result = latest_body_from_packets(
            [(body_packet(), ("192.168.1.99", 40000))],
            expected_source="192.168.1.246",
        )
        self.assertIsNone(result.body)
        self.assertEqual(result.unexpected_sources, ("192.168.1.99",))

    def test_selects_newest_counter_when_udp_arrival_is_reordered(self):
        result = latest_body_from_packets(
            [
                (body_packet(11), ("192.168.1.246", 40000)),
                (body_packet(10), ("192.168.1.246", 40000)),
            ]
        )
        self.assertEqual(result.body.sample_counter, 11)
        self.assertEqual(result.reordered_body_packets, 1)

    def test_counter_selection_handles_uint32_wrap(self):
        result = latest_body_from_packets(
            [
                (body_packet(0xFFFFFFFF), ("192.168.1.246", 40000)),
                (body_packet(0), ("192.168.1.246", 40000)),
            ]
        )
        self.assertEqual(result.body.sample_counter, 0)
        self.assertEqual(result.reordered_body_packets, 0)

    def test_receiver_rejects_a_finite_backlog_without_returning_a_frame(self):
        source = ("192.168.1.246", 40000)
        fake_socket = Mock()
        fake_socket.recvfrom.side_effect = [
            *((body_packet(counter), source) for counter in range(1, 6)),
            socket.timeout(),
        ]

        with patch("mocap_g1.xsens_live.socket.socket", return_value=fake_socket):
            receiver = LatestBodyReceiver(
                bind="127.0.0.1",
                port=5555,
                expected_source=source[0],
                max_drain_packets=4,
            )
            with self.assertRaisesRegex(ProtocolError, "UDP receive overload"):
                receiver.receive_latest(timeout=1.0)
            with self.assertRaises(TimeoutError):
                receiver.receive_latest(timeout=1.0)

        self.assertEqual(fake_socket.recvfrom.call_count, 6)
        self.assertEqual(receiver.drain_budget_exhaustions, 1)
        self.assertEqual(receiver.overload_events, 1)
        self.assertEqual(receiver.total_discarded_over_budget_packets, 1)
        self.assertEqual(receiver.last_batch_result.discarded_over_budget_packets, 1)
        self.assertEqual(receiver.last_overload_error.observed_packets, 5)

    def test_receiver_rejects_continuous_backlog_on_every_bounded_batch(self):
        source = ("192.168.1.246", 40000)
        fake_socket = Mock()
        counter = iter(range(1, 1000))
        fake_socket.recvfrom.side_effect = lambda _size: (
            body_packet(next(counter)), source
        )

        with patch("mocap_g1.xsens_live.socket.socket", return_value=fake_socket):
            receiver = LatestBodyReceiver(
                bind="127.0.0.1",
                port=5555,
                expected_source=source[0],
                max_drain_packets=4,
            )
            for _ in range(3):
                with self.assertRaisesRegex(ProtocolError, "UDP receive overload"):
                    receiver.receive_latest(timeout=1.0)

        self.assertEqual(fake_socket.recvfrom.call_count, 15)
        self.assertEqual(receiver.drain_budget_exhaustions, 3)
        self.assertEqual(receiver.overload_events, 3)
        self.assertEqual(receiver.total_discarded_over_budget_packets, 3)
        self.assertEqual(receiver.last_batch_result.discarded_over_budget_packets, 1)

    def test_failed_bind_closes_the_socket(self):
        fake_socket = Mock()
        fake_socket.bind.side_effect = OSError(98, "Address already in use")
        with patch("mocap_g1.xsens_live.socket.socket", return_value=fake_socket):
            with self.assertRaises(OSError):
                LatestBodyReceiver(bind="127.0.0.1", port=5555, expected_source=None)
        fake_socket.close.assert_called_once_with()

    def test_failed_setsockopt_closes_the_socket(self):
        fake_socket = Mock()
        fake_socket.setsockopt.side_effect = OSError("no buffer for you")
        with patch("mocap_g1.xsens_live.socket.socket", return_value=fake_socket):
            with self.assertRaises(OSError):
                LatestBodyReceiver(bind="127.0.0.1", port=5555, expected_source=None)
        fake_socket.close.assert_called_once_with()

    def test_successful_construction_keeps_the_socket_open(self):
        fake_socket = Mock()
        with patch("mocap_g1.xsens_live.socket.socket", return_value=fake_socket):
            LatestBodyReceiver(bind="127.0.0.1", port=5555, expected_source=None)
        fake_socket.close.assert_not_called()

    def test_sim_drop_estimate_handles_gaps_wrap_and_receiver_restart(self):
        self.assertEqual(_dropped_between(10, 11), 0)
        self.assertEqual(_dropped_between(10, 14), 3)
        self.assertEqual(_dropped_between(0xFFFFFFFF, 0), 0)
        self.assertEqual(_dropped_between(0xFFFFFFFE, 2), 3)
        # Receiver-accepted restart: the counter went back, which is not ~2**32 drops.
        self.assertEqual(_dropped_between(5000, 4), 0)

    def test_receiver_restart_does_not_inflate_the_sim_drop_estimate(self):
        receiver = self._receiver_for_batches([5000, 0, 1, 2, 3, 4, 5, 6])
        accepted = [receiver.receive_latest(timeout=1.0)[0].sample_counter for _ in range(3)]
        self.assertEqual(accepted, [5000, 4, 5])
        self.assertEqual(receiver.stream_restarts, 1)
        drops = sum(_dropped_between(a, b) for a, b in zip(accepted, accepted[1:]))
        self.assertEqual(drops, 0)

    def test_receiver_rejects_invalid_drain_budget_before_opening_socket(self):
        with patch("mocap_g1.xsens_live.socket.socket") as socket_factory:
            with self.assertRaisesRegex(ValueError, "max_drain_packets"):
                LatestBodyReceiver(
                    bind="127.0.0.1",
                    port=5555,
                    expected_source=None,
                    max_drain_packets=0,
                )
        socket_factory.assert_not_called()

    def _receiver_for_batches(self, counters):
        """A receiver whose socket yields one single-packet batch per counter."""
        source = ("192.168.1.246", 40000)
        side_effect = []
        for counter in counters:
            side_effect.extend(((body_packet(counter), source), BlockingIOError()))
        side_effect.append(socket.timeout())
        fake_socket = Mock()
        fake_socket.recvfrom.side_effect = side_effect
        with patch("mocap_g1.xsens_live.socket.socket", return_value=fake_socket):
            return LatestBodyReceiver(
                bind="127.0.0.1", port=5555, expected_source=source[0]
            )

    def test_streamer_restart_is_accepted_after_persistent_backward_jump(self):
        receiver = self._receiver_for_batches([5000, 0, 1, 2, 3, 4, 5])
        body, _ = receiver.receive_latest(timeout=1.0)
        self.assertEqual(body.sample_counter, 5000)

        # The counter reset to 0.  Without restart handling every frame below
        # 5000 is dropped as reordered and the stream is locked out.
        body, _ = receiver.receive_latest(timeout=1.0)
        self.assertEqual(body.sample_counter, 4)
        self.assertEqual(receiver.stream_restarts, 1)
        self.assertEqual(receiver.total_reordered_body_packets, 4)
        body, _ = receiver.receive_latest(timeout=1.0)
        self.assertEqual(body.sample_counter, 5)
        self.assertEqual(receiver.stream_restarts, 1)

    def test_forged_high_counter_cannot_lock_out_the_genuine_stream(self):
        receiver = self._receiver_for_batches(
            [10, 10 + 1_000_000, 11, 12, 13, 14, 15, 16]
        )
        self.assertEqual(receiver.receive_latest(timeout=1.0)[0].sample_counter, 10)
        self.assertEqual(
            receiver.receive_latest(timeout=1.0)[0].sample_counter, 10 + 1_000_000
        )
        self.assertEqual(receiver.receive_latest(timeout=1.0)[0].sample_counter, 15)
        self.assertEqual(receiver.receive_latest(timeout=1.0)[0].sample_counter, 16)
        self.assertEqual(receiver.stream_restarts, 1)

    def test_ordinary_reordering_and_scattered_old_counters_are_not_restarts(self):
        # Small backward steps are UDP reordering; large but non-increasing
        # backward counters never form a consistent restarted stream.
        receiver = self._receiver_for_batches(
            [1000, 999, 998, 997, 996, 995, 994, 500, 300, 600, 200, 400]
        )
        self.assertEqual(receiver.receive_latest(timeout=1.0)[0].sample_counter, 1000)
        with self.assertRaises(TimeoutError):
            receiver.receive_latest(timeout=1.0)
        self.assertEqual(receiver.stream_restarts, 0)
        self.assertEqual(receiver.total_reordered_body_packets, 11)
        self.assertEqual(receiver.last_sample_counter, 1000)

    def test_receiver_rejects_invalid_restart_settings(self):
        for kwargs in ({"restart_backjump": 0}, {"restart_confirm_frames": True}):
            with self.subTest(kwargs=kwargs):
                with patch("mocap_g1.xsens_live.socket.socket") as socket_factory:
                    with self.assertRaises(ValueError):
                        LatestBodyReceiver(
                            bind="127.0.0.1", port=5555, expected_source=None, **kwargs
                        )
                socket_factory.assert_not_called()

    def test_converter_removes_initial_pelvis_yaw(self):
        parsed = latest_body_from_packets(
            [(body_packet(yaw=math.pi / 2.0), ("192.168.1.246", 40000))]
        ).body
        frame = GmrFrameConverter().convert(parsed)
        np.testing.assert_allclose(frame["Pelvis"][1], (1.0, 0.0, 0.0, 0.0), atol=1e-7)
        self.assertEqual(len(frame), 23)

    def test_converter_rejects_invalid_required_task_body(self):
        parsed = latest_body_from_packets(
            [(body_packet(), ("192.168.1.246", 40000))]
        ).body
        segments = dict(parsed.segments)
        chest = segments["t8"]
        segments["t8"] = SegmentPose(chest.position, (0.0, 0.0, 0.0, 0.0))
        with self.assertRaisesRegex(ProtocolError, "Chest"):
            GmrFrameConverter().convert(replace(parsed, segments=segments))


if __name__ == "__main__":
    unittest.main()
