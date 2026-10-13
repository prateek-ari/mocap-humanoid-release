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

    Connection identifiers use point ID zero, so their wire value is
    ``256 * segment_id``. Rotations are the protocol's XYZ degrees.
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
    if strict and (item_count != len(XSENS_SEGMENTS) or body_count != len(XSENS_SEGMENTS)):
        raise ProtocolError("expected one complete 23-segment body frame")
    if strict and (prop_count or finger_count or reserved):
        raise ProtocolError("body-only stream unexpectedly contains props, fingers, or reserved data")
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
    for _ in range(item_count):
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
    """Decode one MXTP20 packet and validate the canonical body tree."""
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
    if strict and item_count != len(XSENS_JOINT_SEGMENT_PAIRS):
        raise ProtocolError("expected one complete 22-joint angle frame")
    if strict and (body_count or prop_count or finger_count or reserved):
        raise ProtocolError("MXTP20 reserved header fields must be zero")
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
        if parent % 256 or child % 256:
            raise ProtocolError("only zero-point connection identifiers are supported")
        parent_id, child_id = parent // 256, child // 256
        if strict and (parent_id, child_id) != XSENS_JOINT_SEGMENT_PAIRS[index]:
            raise ProtocolError(
                f"joint {index + 1} has connection {parent_id}->{child_id}; "
                f"expected {XSENS_JOINT_SEGMENT_PAIRS[index][0]}->"
                f"{XSENS_JOINT_SEGMENT_PAIRS[index][1]}"
            )
        rotation = (rx, ry, rz)
        if not all(math.isfinite(value) for value in rotation):
            raise ProtocolError(f"joint {index + 1} contains non-finite values")
        joints.append(JointAngle(parent_id, child_id, rotation))
    return JointAnglesDatagram(sample_counter, time_code, tuple(joints), character_id)
