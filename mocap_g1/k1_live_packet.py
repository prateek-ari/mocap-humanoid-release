"""Live K1 arm packets: the live tool's filtered reference, sent to the K1 live bridge.

One UDP datagram per accepted retarget result, on loopback only.  Packet v1 is a fixed
little-endian struct (``FORMAT``, 136 bytes):

    magic "K1LV", version, reserved, session id, sequence number,
    sender monotonic time, suit receive monotonic time (host CLOCK_MONOTONIC
    when the suit frame arrived), suit sample counter, solver generation,
    calibration epoch (0 = not calibrated), enabled flag (the console's tracking
    switch), the first 8 bytes of the K1 MJCF SHA-256, the first 8 bytes of the
    joint-order hash, then the 8 K1 arm targets in ``K1_UPPER_BODY_JOINTS`` order.

Root pose waits for v2 (K5).  This module only encodes and checks structure;
the bridge (tools/control/k1_live_bridge.py) applies the stream rules.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
import hashlib
import ipaddress
import math
import secrets
import socket
import struct
import sys
import time
from typing import Any
from urllib.parse import urlsplit

from .live_robots import K1_UPPER_BODY_JOINTS

MAGIC = b"K1LV"
VERSION = 1
FORMAT = "<4sHHQQddIIIB3x8s8s8d"
SIZE = struct.calcsize(FORMAT)
ARM_COUNT = len(K1_UPPER_BODY_JOINTS)
if SIZE >= 512 or ARM_COUNT != 8:
    raise ImportError("K1 live packet layout changed: fix FORMAT")


class PacketError(ValueError):
    """A datagram that is not a valid v1 packet; the message is the drop reason."""


def joint_order_hash(names: Sequence[str] = K1_UPPER_BODY_JOINTS) -> bytes:
    """8 bytes naming the target order: sender and bridge must agree."""
    return hashlib.sha256("\n".join(names).encode("utf-8")).digest()[:8]


def model_hash_prefix(sha256_hex: str) -> bytes:
    """The first 8 bytes of a hex SHA-256 (the K1 MJCF the IK ran on)."""
    return bytes.fromhex(sha256_hex)[:8]


@dataclass(frozen=True)
class LivePacket:
    session_id: int
    sequence: int
    sender_monotonic_s: float
    suit_receive_s: float
    sample_counter: int
    generation: int
    calibration_epoch: int
    enabled: bool
    model_hash: bytes
    joint_order: bytes
    targets: tuple[float, ...]

    def arm_targets(self) -> dict[str, float]:
        return dict(zip(K1_UPPER_BODY_JOINTS, self.targets))


def encode(packet: LivePacket) -> bytes:
    if len(packet.targets) != ARM_COUNT:
        raise PacketError(f"need {ARM_COUNT} arm targets, got {len(packet.targets)}")
    return struct.pack(FORMAT, MAGIC, VERSION, 0, packet.session_id, packet.sequence,
                       packet.sender_monotonic_s, packet.suit_receive_s,
                       packet.sample_counter & 0xFFFFFFFF, packet.generation & 0xFFFFFFFF,
                       packet.calibration_epoch & 0xFFFFFFFF, 1 if packet.enabled else 0,
                       packet.model_hash, packet.joint_order, *packet.targets)


def decode(data: bytes) -> LivePacket:
    """The packet in ``data``; raises :class:`PacketError` with the drop reason."""
    if len(data) != SIZE:
        raise PacketError(f"size {len(data)} != {SIZE}")
    (magic, version, _reserved, session_id, sequence, sender, suit, counter, generation, epoch,
     enabled, model, order, *targets) = struct.unpack(FORMAT, data)
    if magic != MAGIC:
        raise PacketError("magic")
    if version != VERSION:
        raise PacketError(f"version {version}")
    if enabled not in (0, 1):
        raise PacketError("enabled flag")
    values = (sender, suit, *targets)
    if not all(math.isfinite(value) for value in values):
        raise PacketError("non-finite")
    return LivePacket(session_id, sequence, sender, suit, counter, generation, epoch,
                      bool(enabled), model, order, tuple(targets))


def parse_stream_url(url: str) -> tuple[str, int]:
    """``udp://127.0.0.1:<port>`` → ``(host, port)``; loopback IP literals only."""
    parts = urlsplit(url)
    if parts.scheme != "udp" or not parts.hostname or parts.path not in ("", "/"):
        raise ValueError(f"{url!r}: use udp://127.0.0.1:<port>")
    try:
        loopback = ipaddress.ip_address(parts.hostname).is_loopback
        port = parts.port
    except ValueError as exc:
        raise ValueError(f"{url!r}: {exc}") from exc
    if not loopback:
        raise ValueError(f"{url!r}: the robot stream stays on this PC (loopback only)")
    if port is None or not 1 <= port <= 65535:
        raise ValueError(f"{url!r}: needs a port 1-65535")
    return parts.hostname, port


class LivePacketSender:
    """Sends one packet per accepted reference.  A send failure is counted and
    reported once; it never raises into the caller (the simulation goes on)."""

    def __init__(self, address: tuple[str, int], *, model_sha256: str,
                 arm_indices: Sequence[int], clock: Callable[[], float] = time.monotonic,
                 sock: Any = None, session_id: int | None = None) -> None:
        self.address = address
        self.model_hash = model_hash_prefix(model_sha256)
        self.arm_indices = tuple(arm_indices)
        if len(self.arm_indices) != ARM_COUNT:
            raise ValueError(f"need {ARM_COUNT} arm indices")
        self.clock = clock
        self.socket = sock if sock is not None else socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.session_id = session_id if session_id is not None else secrets.randbits(63) or 1
        self.sequence = 0
        self.sent = 0
        self.failures = 0

    def send(self, reference: Sequence[float], *, suit_receive_s: float, counter: int,
             generation: int, epoch: int, enabled: bool) -> bool:
        try:
            self.sequence += 1
            data = encode(LivePacket(
                self.session_id, self.sequence, float(self.clock()), float(suit_receive_s),
                int(counter), int(generation), int(epoch), bool(enabled), self.model_hash,
                joint_order_hash(), tuple(float(reference[index]) for index in self.arm_indices)))
            self.socket.sendto(data, self.address)
        except Exception as exc:  # noqa: BLE001 - the stream is optional; the sim must go on
            self.failures += 1
            if self.failures == 1:
                print(f"ROBOT STREAM send failed ({exc}); the simulation continues", file=sys.stderr)
            return False
        self.sent += 1
        return True

    def close(self) -> None:
        self.socket.close()
