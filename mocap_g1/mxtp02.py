"""Strict, dependency-free Xsens body and joint-angle datagram codecs."""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct
from typing import Iterable, Sequence


XSENS_SEGMENTS = (
    "pelvis", "l5", "l3", "t12", "t8", "neck", "head",
    "right_shoulder", "right_upper_arm", "right_forearm", "right_hand",
    "left_shoulder", "left_upper_arm", "left_forearm", "left_hand",
    "right_upper_leg", "right_lower_leg", "right_foot", "right_toe",
    "left_upper_leg", "left_lower_leg", "left_foot", "left_toe",
)

HEADER_STRUCT = struct.Struct(">6sIBBIBBBBHH")
SEGMENT_STRUCT = struct.Struct(">I3f4f")
JOINT_ANGLE_STRUCT = struct.Struct(">II3f")
DATAGRAM_COUNTER_SINGLE = 0x80

UINT32_MODULUS = 1 << 32
UINT32_HALF_RANGE = 1 << 31

# Xsens MVN's 23-segment body tree, in the order used by MXTP20 packets.
XSENS_JOINT_SEGMENT_PAIRS = (
    (1, 2), (2, 3), (3, 4), (4, 5), (5, 6), (6, 7),
    (5, 8), (8, 9), (9, 10), (10, 11),
    (5, 12), (12, 13), (13, 14), (14, 15),
    (1, 16), (16, 17), (17, 18), (18, 19),
    (1, 20), (20, 21), (21, 22), (22, 23),
)

# MXTP20 endpoint fields are point IDs encoded as
# ``256 * segment_id + segment-local point_id``. These observed local IDs name
# the endpoints for the canonical 22-joint body tree. Legacy synthetic packets
# use local point zero at both ends instead.
XSENS_MXTP20_BODY_CONNECTIONS = (
    (1, 2, 2, 1), (2, 2, 3, 1), (3, 2, 4, 1), (4, 2, 5, 1),
    (5, 2, 6, 1), (6, 2, 7, 1), (5, 3, 8, 1), (8, 2, 9, 1),
    (9, 2, 10, 1), (10, 2, 11, 1), (5, 4, 12, 1), (12, 2, 13, 1),
    (13, 2, 14, 1), (14, 2, 15, 1), (1, 3, 16, 1), (16, 2, 17, 1),
    (17, 2, 18, 1), (18, 2, 19, 1), (1, 4, 20, 1), (20, 2, 21, 1),
    (21, 2, 22, 1), (22, 2, 23, 1),
)

# Six extra records were observed after the canonical tree in an Xsens Humanoid
# 2026.9 stream capture. The published protocol defines the MXTP20 record and
# point-ID encoding, but does not specify this suffix; keep the captured IDs
# explicit so unrelated or malformed records remain rejected.
XSENS_MXTP20_EXTENDED_CONNECTIONS = (
    (5, 0, 7, 0), (5, 0, 13, 0), (5, 0, 9, 0),
    (1, 0, 5, 0), (1, 0, 1, 0), (1, 0, 5, 0),
)


class ProtocolError(ValueError):
    """Raised when a body datagram violates the expected MXTP02 contract."""


def counter_delta(candidate: int, reference: int) -> int:
    """Signed distance from ``reference`` to ``candidate`` for uint32 counters.

    The result is wrap-aware and lies in ``[-2**31, 2**31)``: positive when
    ``candidate`` is ahead of ``reference`` (``0xFFFFFFFF`` -> ``0`` is ``+1``),
    negative when it is behind (a reordered packet or a stream restart), and zero
    for a repeat. A half-range distance is ambiguous and reads as behind.
    """
    return ((candidate - reference + UINT32_HALF_RANGE) % UINT32_MODULUS) - UINT32_HALF_RANGE


def counter_is_newer(candidate: int, reference: int) -> bool:
    """True when ``candidate`` is ahead of ``reference`` across the uint32 wrap."""
    return counter_delta(candidate, reference) > 0


@dataclass(frozen=True)
class SegmentPose:
    position: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]


@dataclass(frozen=True)
class BodyDatagram:
    sample_counter: int
    time_code_ms: int
    segments: dict[str, SegmentPose]
    character_id: int = 0


@dataclass(frozen=True)
class JointAngle:
    parent_segment_id: int
    child_segment_id: int
    rotation_xyz_deg: tuple[float, float, float]
    parent_point_id: int = 0
    child_point_id: int = 0


@dataclass(frozen=True)
class JointAnglesDatagram:
    sample_counter: int
    time_code_ms: int
    joints: tuple[JointAngle, ...]
    character_id: int = 0


def _floats(values: Iterable[float], size: int, label: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != size or not all(math.isfinite(value) for value in result):
        raise ProtocolError(f"{label} must contain {size} finite values")
    return result


def build_body_datagram(
    sample_counter: int,
    time_code_ms: int,
    positions: Sequence[Sequence[float]],
    quaternions_wxyz: Sequence[Sequence[float]],
    *,
    character_id: int = 0,
) -> bytes:
    """Encode one complete 23-body MXTP02 datagram."""
    count = len(XSENS_SEGMENTS)
    if len(positions) != count or len(quaternions_wxyz) != count:
        raise ProtocolError(f"MXTP02 body frame requires exactly {count} segment poses")
    if not 0 <= character_id <= 255:
        raise ProtocolError("character_id must fit in an unsigned byte")

    payload_size = count * SEGMENT_STRUCT.size
    parts = [HEADER_STRUCT.pack(
        b"MXTP02",
        int(sample_counter) & 0xFFFFFFFF,
        DATAGRAM_COUNTER_SINGLE,
        count,
        int(time_code_ms) & 0xFFFFFFFF,
        character_id,
        count,
        0,
        0,
        0,
        payload_size,
    )]
    for index, (position, quaternion) in enumerate(zip(positions, quaternions_wxyz), 1):
        pos = _floats(position, 3, f"segment {index} position")
        quat = _floats(quaternion, 4, f"segment {index} quaternion")
        norm = math.sqrt(sum(value * value for value in quat))
        if norm < 1e-8:
            raise ProtocolError(f"segment {index} quaternion has zero norm")
        parts.append(SEGMENT_STRUCT.pack(index, *pos, *quat))
    return b"".join(parts)


def build_joint_angles_datagram(
    sample_counter: int,
    time_code_ms: int,
    rotations_xyz_deg: Sequence[Sequence[float]],
    *,
    character_id: int = 0,
) -> bytes:
    """Encode one complete 22-joint MXTP20 datagram.

    This synthetic encoder retains the legacy point-zero connection IDs, so
    their wire value is ``256 * segment_id``. Rotations are protocol XYZ degrees.
    """
    count = len(XSENS_JOINT_SEGMENT_PAIRS)
    if len(rotations_xyz_deg) != count:
        raise ProtocolError(f"MXTP20 frame requires exactly {count} joint rotations")
    if not 0 <= character_id <= 255:
        raise ProtocolError("character_id must fit in an unsigned byte")

    payload_size = count * JOINT_ANGLE_STRUCT.size
    parts = [HEADER_STRUCT.pack(
        b"MXTP20",
        int(sample_counter) & 0xFFFFFFFF,
        DATAGRAM_COUNTER_SINGLE,
        count,
        int(time_code_ms) & 0xFFFFFFFF,
        character_id,
        0,
        0,
        0,
        0,
        payload_size,
    )]
    for index, ((parent_id, child_id), rotation) in enumerate(
        zip(XSENS_JOINT_SEGMENT_PAIRS, rotations_xyz_deg), 1
    ):
        rot = _floats(rotation, 3, f"joint {index} rotation")
        parts.append(JOINT_ANGLE_STRUCT.pack(parent_id * 256, child_id * 256, *rot))
    return b"".join(parts)


def parse_body_datagram(data: bytes, *, strict: bool = True) -> BodyDatagram:
    """Decode one body datagram, with strict structural checks by default."""
    if len(data) < HEADER_STRUCT.size:
        raise ProtocolError("datagram is shorter than the 24-byte MXTP02 header")
    (
        message_id,
        sample_counter,
        datagram_counter,
        item_count,
        time_code,
        character_id,
        body_count,
        prop_count,
        finger_count,
        reserved,
        payload_size,
    ) = HEADER_STRUCT.unpack_from(data)
    if message_id != b"MXTP02":
        raise ProtocolError(f"unexpected message id {message_id!r}")
    if strict and datagram_counter != DATAGRAM_COUNTER_SINGLE:
        raise ProtocolError("interleaved/multipart MXTP02 datagrams are not accepted")
    # MVN sends the 23 body segments first, then props and fingers (MVN 2026.9 with
    # "Send finger data": 23 + 40 records). Only the body is used; the rest is skipped.
    if strict and (body_count != len(XSENS_SEGMENTS)
                   or item_count != body_count + prop_count + finger_count):
        raise ProtocolError("expected one complete 23-segment body frame")
    if strict and reserved:
        raise ProtocolError("body datagram has non-zero reserved header data")
    expected_payload = item_count * SEGMENT_STRUCT.size
    if payload_size != expected_payload:
        raise ProtocolError(
            f"payload header says {payload_size} bytes; {item_count} records require {expected_payload}"
        )
    if len(data) != HEADER_STRUCT.size + payload_size:
        raise ProtocolError(
            f"datagram length {len(data)} does not match header length "
            f"{HEADER_STRUCT.size + payload_size}"
        )

    segments: dict[str, SegmentPose] = {}
    offset = HEADER_STRUCT.size
    for _ in range(min(item_count, len(XSENS_SEGMENTS))):
        segment_id, x, y, z, qw, qx, qy, qz = SEGMENT_STRUCT.unpack_from(data, offset)
        offset += SEGMENT_STRUCT.size
        if not 1 <= segment_id <= len(XSENS_SEGMENTS):
            raise ProtocolError(f"segment id {segment_id} is outside the body model")
        name = XSENS_SEGMENTS[segment_id - 1]
        if name in segments:
            raise ProtocolError(f"duplicate segment id {segment_id}")
        pose = SegmentPose((x, y, z), (qw, qx, qy, qz))
        if not all(math.isfinite(value) for value in (*pose.position, *pose.quaternion_wxyz)):
            raise ProtocolError(f"segment {name} contains non-finite values")
        segments[name] = pose
    return BodyDatagram(sample_counter, time_code, segments, character_id)


def parse_joint_angles_datagram(data: bytes, *, strict: bool = True) -> JointAnglesDatagram:
    """Decode MXTP20 and validate a supported complete body-angle layout."""
    if len(data) < HEADER_STRUCT.size:
        raise ProtocolError("datagram is shorter than the 24-byte MXTP20 header")
    (
        message_id,
        sample_counter,
        datagram_counter,
        item_count,
        time_code,
        character_id,
        body_count,
        prop_count,
        finger_count,
        reserved,
        payload_size,
    ) = HEADER_STRUCT.unpack_from(data)
    if message_id != b"MXTP20":
        raise ProtocolError(f"unexpected message id {message_id!r}")
    if strict and datagram_counter != DATAGRAM_COUNTER_SINGLE:
        raise ProtocolError("interleaved/multipart MXTP20 datagrams are not accepted")
    if strict and item_count not in (
        len(XSENS_JOINT_SEGMENT_PAIRS),
        len(XSENS_JOINT_SEGMENT_PAIRS) + len(XSENS_MXTP20_EXTENDED_CONNECTIONS),
    ):
        raise ProtocolError("expected one complete 22-joint or supported 28-record MXTP20 frame")
    # The protocol specifies 23 body segments in every packet type. The old
    # synthetic zero-point layout also used zero, so preserve that one profile.
    if strict and body_count not in (0, len(XSENS_SEGMENTS)):
        raise ProtocolError("MXTP20 body segment count must be zero or 23")
    if strict and (prop_count or finger_count or reserved):
        raise ProtocolError("MXTP20 props, fingers, and reserved header fields must be zero")
    expected_payload = item_count * JOINT_ANGLE_STRUCT.size
    if payload_size != expected_payload:
        raise ProtocolError(
            f"payload header says {payload_size} bytes; {item_count} records require {expected_payload}"
        )
    if len(data) != HEADER_STRUCT.size + payload_size:
        raise ProtocolError(
            f"datagram length {len(data)} does not match header length "
            f"{HEADER_STRUCT.size + payload_size}"
        )

    joints: list[JointAngle] = []
    offset = HEADER_STRUCT.size
    for index in range(item_count):
        parent, child, rx, ry, rz = JOINT_ANGLE_STRUCT.unpack_from(data, offset)
        offset += JOINT_ANGLE_STRUCT.size
        parent_id, parent_point_id = divmod(parent, 256)
        child_id, child_point_id = divmod(child, 256)
        if not 1 <= parent_id <= len(XSENS_SEGMENTS):
            raise ProtocolError(f"joint {index + 1} parent segment id {parent_id} is outside the body model")
        if not 1 <= child_id <= len(XSENS_SEGMENTS):
            raise ProtocolError(f"joint {index + 1} child segment id {child_id} is outside the body model")
        rotation = (rx, ry, rz)
        if not all(math.isfinite(value) for value in rotation):
            raise ProtocolError(f"joint {index + 1} contains non-finite values")
        joints.append(JointAngle(
            parent_id,
            child_id,
            rotation,
            parent_point_id,
            child_point_id,
        ))

    if strict:
        connections = tuple(
            (
                joint.parent_segment_id,
                joint.parent_point_id,
                joint.child_segment_id,
                joint.child_point_id,
            )
            for joint in joints
        )
        legacy_connections = tuple(
            (parent_id, 0, child_id, 0)
            for parent_id, child_id in XSENS_JOINT_SEGMENT_PAIRS
        )
        current_connections = XSENS_MXTP20_BODY_CONNECTIONS
        if item_count == len(XSENS_JOINT_SEGMENT_PAIRS):
            valid_layouts = (legacy_connections, current_connections)
        else:
            valid_layouts = (
                current_connections + XSENS_MXTP20_EXTENDED_CONNECTIONS,
            )
        if connections not in valid_layouts:
            expected_layout = min(
                valid_layouts,
                key=lambda layout: sum(
                    actual != expected for actual, expected in zip(connections, layout)
                ),
            )
            mismatch = next(
                index
                for index, (actual, expected) in enumerate(zip(connections, expected_layout))
                if actual != expected
            )
            expected = expected_layout[mismatch]
            actual = connections[mismatch]
            raise ProtocolError(
                f"joint {mismatch + 1} has connection "
                f"{actual[0]}.{actual[1]}->{actual[2]}.{actual[3]}; expected "
                f"{expected[0]}.{expected[1]}->{expected[2]}.{expected[3]}"
            )
        if body_count == 0 and connections != legacy_connections:
            raise ProtocolError("MXTP20 body segment count must be 23 for point-based layouts")
    return JointAnglesDatagram(sample_counter, time_code, tuple(joints), character_id)
