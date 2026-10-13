from __future__ import annotations

from pathlib import Path
import struct
import subprocess
import sys
import unittest

from mocap_g1.mxtp02 import (
    HEADER_STRUCT,
    JOINT_ANGLE_STRUCT,
    SEGMENT_STRUCT,
    ProtocolError,
    XSENS_JOINT_SEGMENT_PAIRS,
    XSENS_SEGMENTS,
    build_body_datagram,
    build_joint_angles_datagram,
    counter_delta,
    counter_is_newer,
    parse_body_datagram,
    parse_joint_angles_datagram,
)


class Mxtp02Tests(unittest.TestCase):
    def setUp(self):
        self.positions = [(index, index + 0.25, index - 0.5) for index in range(23)]
        self.quaternions = [(1.0, 0.0, 0.0, 0.0)] * 23

    def test_round_trip_and_unsigned_wrap(self):
        packet = build_body_datagram(
            0x1_0000_0003,
            0x1_0000_0004,
            self.positions,
            self.quaternions,
        )
        decoded = parse_body_datagram(packet)
        self.assertEqual(len(packet), 760)
        self.assertEqual(decoded.sample_counter, 3)
        self.assertEqual(decoded.time_code_ms, 4)
        self.assertEqual(tuple(decoded.segments), XSENS_SEGMENTS)
        self.assertAlmostEqual(decoded.segments["right_hand"].position[0], 10.0)

    def test_rejects_truncated_and_zero_quaternion(self):
        packet = build_body_datagram(0, 0, self.positions, self.quaternions)
        with self.assertRaises(ProtocolError):
            parse_body_datagram(packet[:-1])
        bad = list(self.quaternions)
        bad[0] = (0.0, 0.0, 0.0, 0.0)
        with self.assertRaises(ProtocolError):
            build_body_datagram(0, 0, self.positions, bad)

    def test_joint_angle_round_trip_and_tree_validation(self):
        rotations = [(index, -index, index / 2) for index in range(22)]
        packet = build_joint_angles_datagram(17, 29, rotations)
        decoded = parse_joint_angles_datagram(packet)
        self.assertEqual(len(packet), 464)
        self.assertEqual(decoded.sample_counter, 17)
        self.assertEqual(decoded.time_code_ms, 29)
        self.assertEqual(
            tuple((joint.parent_segment_id, joint.child_segment_id) for joint in decoded.joints),
            XSENS_JOINT_SEGMENT_PAIRS,
        )
        self.assertEqual(decoded.joints[3].rotation_xyz_deg, rotations[3])

        corrupted = bytearray(packet)
        corrupted[24:28] = (23 * 256).to_bytes(4, "big")
        with self.assertRaises(ProtocolError):
            parse_joint_angles_datagram(bytes(corrupted))


# Header byte offsets (">6sIBBIBBBBHH"): datagram counter, item count, body
# count, prop count, payload size.
DATAGRAM_COUNTER_OFFSET = 10
ITEM_COUNT_OFFSET = 11
BODY_COUNT_OFFSET = 17
PROP_COUNT_OFFSET = 18
PAYLOAD_SIZE_OFFSET = 22


def with_payload_size(packet: bytes, size: int) -> bytes:
    data = bytearray(packet)
    struct.pack_into(">H", data, PAYLOAD_SIZE_OFFSET, size)
    return bytes(data)


class Mxtp02WireValidationTests(unittest.TestCase):
    def setUp(self):
        self.positions = [(index * 0.1, 0.0, 1.0) for index in range(23)]
        self.quaternions = [(1.0, 0.0, 0.0, 0.0)] * 23
        self.body = build_body_datagram(5, 50, self.positions, self.quaternions)
        self.joints = build_joint_angles_datagram(5, 50, [(1.0, 2.0, 3.0)] * 22)

    def segment_offset(self, index: int) -> int:
        return HEADER_STRUCT.size + index * SEGMENT_STRUCT.size

    def test_rejects_duplicate_and_out_of_range_segment_ids(self):
        for record, segment_id, message in (
            (1, 1, "duplicate segment id 1"),
            (22, 7, "duplicate segment id 7"),
            (0, 0, "outside the body model"),
            (4, 24, "outside the body model"),
            (4, 0xFFFFFFFF, "outside the body model"),
        ):
            with self.subTest(record=record, segment_id=segment_id):
                data = bytearray(self.body)
                struct.pack_into(">I", data, self.segment_offset(record), segment_id)
                for strict in (True, False):
                    with self.assertRaisesRegex(ProtocolError, message):
                        parse_body_datagram(bytes(data), strict=strict)

    def test_rejects_non_finite_floats_on_the_wire(self):
        for value in (float("nan"), float("inf"), float("-inf")):
            for field in range(7):  # x, y, z, qw, qx, qy, qz
                with self.subTest(value=value, field=field):
                    data = bytearray(self.body)
                    struct.pack_into(">f", data, self.segment_offset(3) + 4 + 4 * field, value)
                    with self.assertRaisesRegex(ProtocolError, "non-finite"):
                        parse_body_datagram(bytes(data))
            with self.subTest(value=value, message="MXTP20"):
                data = bytearray(self.joints)
                offset = HEADER_STRUCT.size + 2 * JOINT_ANGLE_STRUCT.size + 8
                struct.pack_into(">f", data, offset, value)
                with self.assertRaisesRegex(ProtocolError, "non-finite"):
                    parse_joint_angles_datagram(bytes(data))

    def test_rejects_payload_size_mismatch(self):
        for packet, parser, record in (
            (self.body, parse_body_datagram, SEGMENT_STRUCT.size),
            (self.joints, parse_joint_angles_datagram, JOINT_ANGLE_STRUCT.size),
        ):
            declared = len(packet) - HEADER_STRUCT.size
            for size in (0, declared - record, declared - 1, declared + 1, 0xFFFF):
                with self.subTest(parser=parser.__name__, size=size):
                    for strict in (True, False):
                        with self.assertRaisesRegex(ProtocolError, "payload header"):
                            parser(with_payload_size(packet, size), strict=strict)

    def test_rejects_oversized_and_truncated_datagrams(self):
        for packet, parser in (
            (self.body, parse_body_datagram),
            (self.joints, parse_joint_angles_datagram),
        ):
            for data in (
                packet + b"\x00",
                packet + packet,
                packet + bytes(65535 - len(packet)),
                packet[:-1],
                packet[: HEADER_STRUCT.size],
                packet[: HEADER_STRUCT.size - 1],
            ):
                with self.subTest(parser=parser.__name__, length=len(data)):
                    for strict in (True, False):
                        with self.assertRaises(ProtocolError):
                            parser(data, strict=strict)

    def test_multipart_datagram_counter_is_strict_only(self):
        for packet, parser in (
            (self.body, parse_body_datagram),
            (self.joints, parse_joint_angles_datagram),
        ):
            for counter in (0x00, 0x01, 0x81):
                with self.subTest(parser=parser.__name__, counter=counter):
                    data = bytearray(packet)
                    data[DATAGRAM_COUNTER_OFFSET] = counter
                    with self.assertRaisesRegex(ProtocolError, "multipart"):
                        parser(bytes(data))
                    decoded = parser(bytes(data), strict=False)
                    self.assertEqual(decoded.sample_counter, 5)

    def test_non_strict_accepts_partial_body_and_prop_headers(self):
        partial = bytearray(self.body[: self.segment_offset(15)])
        partial[ITEM_COUNT_OFFSET] = 15
        partial[BODY_COUNT_OFFSET] = 15
        partial[PROP_COUNT_OFFSET] = 2
        partial = with_payload_size(bytes(partial), 15 * SEGMENT_STRUCT.size)
        with self.assertRaises(ProtocolError):
            parse_body_datagram(partial)
        decoded = parse_body_datagram(partial, strict=False)
        self.assertEqual(tuple(decoded.segments), XSENS_SEGMENTS[:15])
        self.assertAlmostEqual(decoded.segments["l5"].position[0], 0.1, places=6)

        empty = bytearray(self.body[: HEADER_STRUCT.size])
        empty[ITEM_COUNT_OFFSET] = 0
        decoded = parse_body_datagram(with_payload_size(bytes(empty), 0), strict=False)
        self.assertEqual(decoded.segments, {})

    def test_non_strict_joint_angles_skip_only_the_tree_checks(self):
        data = bytearray(self.joints)
        struct.pack_into(">I", data, HEADER_STRUCT.size, 23 * 256)
        with self.assertRaises(ProtocolError):
            parse_joint_angles_datagram(bytes(data))
        decoded = parse_joint_angles_datagram(bytes(data), strict=False)
        self.assertEqual(decoded.joints[0].parent_segment_id, 23)

        # Non-zero point identifiers are rejected in every mode.
        struct.pack_into(">I", data, HEADER_STRUCT.size, 23 * 256 + 1)
        with self.assertRaisesRegex(ProtocolError, "zero-point"):
            parse_joint_angles_datagram(bytes(data), strict=False)


class CounterDeltaTests(unittest.TestCase):
    def test_signed_delta_across_the_uint32_wrap(self):
        cases = (
            (5, 5, 0),
            (6, 5, 1),
            (5, 6, -1),
            (0, 0xFFFFFFFF, 1),
            (0xFFFFFFFF, 0, -1),
            (2, 0xFFFFFFFE, 4),
            (0xFFFFFFFE, 2, -4),
            (0x7FFFFFFF, 0, 0x7FFFFFFF),
            (0x80000001, 0, -0x7FFFFFFF),
        )
        for candidate, reference, expected in cases:
            with self.subTest(candidate=candidate, reference=reference):
                self.assertEqual(counter_delta(candidate, reference), expected)

    def test_half_range_is_ambiguous_and_reads_as_behind(self):
        self.assertEqual(counter_delta(0x80000000, 0), -0x80000000)
        self.assertEqual(counter_delta(0, 0x80000000), -0x80000000)
        self.assertFalse(counter_is_newer(0x80000000, 0))

    def test_counter_is_newer_matches_wrap_semantics(self):
        self.assertTrue(counter_is_newer(0, 0xFFFFFFFF))
        self.assertTrue(counter_is_newer(1, 0))
        self.assertFalse(counter_is_newer(0xFFFFFFFF, 0))
        self.assertFalse(counter_is_newer(7, 7))


class ReplayerCliTests(unittest.TestCase):
    REPLAYER = Path(__file__).resolve().parents[1] / "tools/mxtp02_replayer.py"

    def run_replayer(self, *args: str) -> subprocess.CompletedProcess:
        return subprocess.run(
            [sys.executable, str(self.REPLAYER), "--synthetic", *args],
            capture_output=True, text=True, timeout=30, check=False,
        )

    def test_non_positive_or_non_finite_rate_is_an_argparse_error(self):
        for rate in ("0", "0.0", "-5", "nan", "inf", "fast"):
            with self.subTest(rate=rate):
                completed = self.run_replayer("--rate", rate, "--duration", "0.1", "--quiet")
                self.assertEqual(completed.returncode, 2, completed.stderr)
                self.assertIn("--rate", completed.stderr)
                self.assertNotIn("Traceback", completed.stderr)

    def test_positive_rate_still_runs(self):
        completed = self.run_replayer("--rate", "120", "--duration", "0.1", "--quiet")
        self.assertEqual(completed.returncode, 0, completed.stderr)


if __name__ == "__main__":
    unittest.main()
