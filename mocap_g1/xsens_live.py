"""Live Xsens MXTP02 ingest helpers for GMR retargeting."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
import socket
import time
from typing import Iterable

import numpy as np

from .mxtp02 import (
    BodyDatagram,
    ProtocolError,
    counter_delta,
    counter_is_newer,
    parse_body_datagram,
)


DEFAULT_MAX_DRAIN_PACKETS = 256

#: A body counter this far *behind* the newest accepted one is not UDP
#: reordering.  It is either an MVN Network Streamer restart (the counter
#: resets) or a stray/forged high counter that would otherwise lock out every
#: genuine frame until the stream climbed past it.
DEFAULT_RESTART_BACKJUMP = 32

#: Consecutive, mutually increasing backward-jump frames required before the
#: receiver accepts them as a restarted stream.
DEFAULT_RESTART_CONFIRM_FRAMES = 5


XSENS_TO_GMR = {
    "pelvis": "Pelvis",
    "l5": "Spine",
    "l3": "Spine1",
    "t12": "Spine2",
    "t8": "Chest",
    "neck": "Neck",
    "head": "Head",
    "left_shoulder": "Left_Shoulder",
    "left_upper_arm": "Left_UpperArm",
    "left_forearm": "Left_Forearm",
    "left_hand": "Left_Hand",
    "right_shoulder": "Right_Shoulder",
    "right_upper_arm": "Right_UpperArm",
    "right_forearm": "Right_Forearm",
    "right_hand": "Right_Hand",
    "left_upper_leg": "Left_UpperLeg",
    "left_lower_leg": "Left_LowerLeg",
    "left_foot": "Left_Foot",
    "left_toe": "Left_Toe",
    "right_upper_leg": "Right_UpperLeg",
    "right_lower_leg": "Right_LowerLeg",
    "right_foot": "Right_Foot",
    "right_toe": "Right_Toe",
}

REQUIRED_GMR_BODIES = {
    "Pelvis",
    "Chest",
    "Head",
    "Left_UpperArm",
    "Left_Forearm",
    "Left_Hand",
    "Right_UpperArm",
    "Right_Forearm",
    "Right_Hand",
    "Left_UpperLeg",
    "Left_LowerLeg",
    "Left_Foot",
    "Right_UpperLeg",
    "Right_LowerLeg",
    "Right_Foot",
}


@dataclass(frozen=True)
class PacketBatchResult:
    body: BodyDatagram | None
    source: tuple[str, int] | None
    body_packets: int
    ignored_packets: int
    malformed_packets: int
    reordered_body_packets: int
    unexpected_sources: tuple[str, ...]
    discarded_over_budget_packets: int = 0


class ReceiverOverloadError(ProtocolError):
    """Raised when a bounded UDP drain proves that the socket stays backlogged."""

    def __init__(self, *, max_drain_packets: int, observed_packets: int) -> None:
        self.max_drain_packets = max_drain_packets
        self.observed_packets = observed_packets
        self.discarded_over_budget_packets = max(0, observed_packets - max_drain_packets)
        super().__init__(
            "UDP receive overload: bounded drain observed at least "
            f"{observed_packets} packets (budget {max_drain_packets}); "
            f"discarding batch and {self.discarded_over_budget_packets} over-budget "
            "packet(s)"
        )


def latest_body_from_packets(
    packets: Iterable[tuple[bytes, tuple[str, int]]],
    *,
    expected_source: str | None = None,
    strict: bool = True,
) -> PacketBatchResult:
    """Return the latest valid body frame while safely ignoring paired MXTP20 packets."""
    latest = None
    latest_source = None
    body_packets = ignored_packets = malformed_packets = reordered_body_packets = 0
    unexpected_sources: set[str] = set()
    for data, source in packets:
        if expected_source and source[0] != expected_source:
            unexpected_sources.add(source[0])
            ignored_packets += 1
            continue
        if not data.startswith(b"MXTP02"):
            ignored_packets += 1
            continue
        try:
            parsed = parse_body_datagram(data, strict=strict)
        except ProtocolError:
            malformed_packets += 1
            continue
        body_packets += 1
        if latest is None or counter_is_newer(parsed.sample_counter, latest.sample_counter):
            latest = parsed
            latest_source = source
        elif parsed.sample_counter != latest.sample_counter:
            reordered_body_packets += 1
    return PacketBatchResult(
        latest,
        latest_source,
        body_packets,
        ignored_packets,
        malformed_packets,
        reordered_body_packets,
        tuple(sorted(unexpected_sources)),
    )


class LatestBodyReceiver:
    """UDP receiver that drains each burst and keeps the newest valid MXTP02 frame."""

    def __init__(
        self,
        *,
        bind: str,
        port: int,
        expected_source: str | None,
        strict: bool = True,
        max_drain_packets: int = DEFAULT_MAX_DRAIN_PACKETS,
        restart_backjump: int = DEFAULT_RESTART_BACKJUMP,
        restart_confirm_frames: int = DEFAULT_RESTART_CONFIRM_FRAMES,
    ) -> None:
        for name, value in (
            ("max_drain_packets", max_drain_packets),
            ("restart_backjump", restart_backjump),
            ("restart_confirm_frames", restart_confirm_frames),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f"{name} must be a positive integer")
        self.restart_backjump = restart_backjump
        self.restart_confirm_frames = restart_confirm_frames
        self.expected_source = expected_source
        self.strict = strict
        self.max_drain_packets = max_drain_packets
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
            self.socket.bind((bind, port))
        except BaseException:
            # A failed bind (port in use, bad address) must not leak the fd.
            self.socket.close()
            raise
        self.total_body_packets = 0
        self.total_ignored_packets = 0
        self.total_malformed_packets = 0
        self.total_reordered_body_packets = 0
        self.total_discarded_over_budget_packets = 0
        self.overload_events = 0
        self.drain_budget_exhaustions = 0
        self.last_sample_counter: int | None = None
        self.stream_restarts = 0
        self._restart_candidate: int | None = None
        self._restart_candidate_frames = 0
        self.last_batch_result: PacketBatchResult | None = None
        self.last_overload_error: ReceiverOverloadError | None = None
        self.unexpected_sources: set[str] = set()

    def receive_latest(self, *, timeout: float) -> tuple[BodyDatagram, tuple[str, int]]:
        """Return one fresh body frame or fail closed on a provable backlog.

        ``ReceiverOverloadError`` is a ``ProtocolError`` so live consumers can
        discard the batch through their existing malformed-frame path.
        """
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("no valid MXTP02 body frame received")
            self.socket.settimeout(remaining)
            try:
                packets = [self.socket.recvfrom(65535)]
            except socket.timeout as exc:
                raise TimeoutError("no valid MXTP02 body frame received") from exc

            self.socket.setblocking(False)
            discarded_over_budget_packets = 0
            try:
                while len(packets) < self.max_drain_packets:
                    try:
                        packets.append(self.socket.recvfrom(65535))
                    except BlockingIOError:
                        break
                if len(packets) == self.max_drain_packets:
                    # One bounded probe distinguishes a full-but-drained burst
                    # from a queue which still has data.  If another packet is
                    # waiting, the queue is provably still backlogged.  Do not
                    # return a sampled frame as if it were the newest one: the
                    # caller must drop this batch and let stale safety handling
                    # take over.  Remaining queue depth is deliberately not
                    # drained here: sustained traffic must not starve the caller.
                    try:
                        packets.append(self.socket.recvfrom(65535))
                    except BlockingIOError:
                        pass
                    else:
                        discarded_over_budget_packets = 1
            finally:
                self.socket.setblocking(True)

            result = latest_body_from_packets(
                packets,
                expected_source=self.expected_source,
                strict=self.strict,
            )
            if discarded_over_budget_packets:
                result = replace(
                    result,
                    discarded_over_budget_packets=discarded_over_budget_packets,
                )
            self.last_batch_result = result
            self.total_body_packets += result.body_packets
            self.total_ignored_packets += result.ignored_packets
            self.total_malformed_packets += result.malformed_packets
            self.total_reordered_body_packets += result.reordered_body_packets
            self.total_discarded_over_budget_packets += (
                result.discarded_over_budget_packets
            )
            self.unexpected_sources.update(result.unexpected_sources)
            if discarded_over_budget_packets:
                self.overload_events += 1
                self.drain_budget_exhaustions += 1
                error = ReceiverOverloadError(
                    max_drain_packets=self.max_drain_packets,
                    observed_packets=len(packets),
                )
                self.last_overload_error = error
                raise error
            if result.body is not None and result.source is not None:
                counter = result.body.sample_counter
                if self.last_sample_counter is None or counter_is_newer(
                    counter, self.last_sample_counter
                ):
                    self.last_sample_counter = counter
                    self._restart_candidate = None
                    self._restart_candidate_frames = 0
                    return result.body, result.source
                if counter == self.last_sample_counter:
                    continue
                backjump = -counter_delta(counter, self.last_sample_counter)
                if backjump > self.restart_backjump and self._confirm_restart(counter):
                    # Treat the persistent backward jump as a new stream
                    # rather than dropping every frame as reordered.
                    self.last_sample_counter = counter
                    self.stream_restarts += 1
                    return result.body, result.source
                self.total_reordered_body_packets += 1

    def _confirm_restart(self, counter: int) -> bool:
        """Track a candidate restarted stream; True once it has persisted."""
        if self._restart_candidate is not None and counter_is_newer(
            counter, self._restart_candidate
        ):
            self._restart_candidate_frames += 1
        else:
            self._restart_candidate_frames = 1
        self._restart_candidate = counter
        if self._restart_candidate_frames < self.restart_confirm_frames:
            return False
        self._restart_candidate = None
        self._restart_candidate_frames = 0
        return True

    def close(self) -> None:
        self.socket.close()


def _quat_multiply_wxyz(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return np.asarray(
        (
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ),
        dtype=np.float64,
    )


class GmrFrameConverter:
    """Map Xsens segment names and normalize the initial pelvis heading."""

    def __init__(self) -> None:
        self._initial_yaw_inverse: float | None = None

    def convert(self, body: BodyDatagram) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        frame: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        for xsens_name, gmr_name in XSENS_TO_GMR.items():
            pose = body.segments.get(xsens_name)
            if pose is None:
                continue
            position = np.asarray(pose.position, dtype=np.float64)
            quaternion = np.asarray(pose.quaternion_wxyz, dtype=np.float64)
            norm = float(np.linalg.norm(quaternion))
            if norm <= 1e-8:
                continue
            frame[gmr_name] = position, quaternion / norm

        missing = sorted(REQUIRED_GMR_BODIES.difference(frame))
        if missing:
            raise ProtocolError(f"body frame is missing GMR segments: {', '.join(missing)}")

        pelvis_quat = frame["Pelvis"][1]
        if self._initial_yaw_inverse is None:
            w, x, y, z = pelvis_quat
            yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            self._initial_yaw_inverse = -yaw

        yaw_inverse = self._initial_yaw_inverse
        cos_yaw = math.cos(yaw_inverse)
        sin_yaw = math.sin(yaw_inverse)
        yaw_quat = np.asarray(
            (math.cos(yaw_inverse / 2.0), 0.0, 0.0, math.sin(yaw_inverse / 2.0)),
            dtype=np.float64,
        )
        normalized = {}
        for name, (position, quaternion) in frame.items():
            rotated_position = np.asarray(
                (
                    cos_yaw * position[0] - sin_yaw * position[1],
                    sin_yaw * position[0] + cos_yaw * position[1],
                    position[2],
                ),
                dtype=np.float64,
            )
            normalized[name] = rotated_position, _quat_multiply_wxyz(yaw_quat, quaternion)
        return normalized
