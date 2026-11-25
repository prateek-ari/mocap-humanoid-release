"""Dependency-free acceptance metrics for an Xsens MVN UDP stream."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from statistics import fmean, median
import time
from typing import Any

from .mxtp02 import (
    UINT32_HALF_RANGE,
    UINT32_MODULUS,
    ProtocolError,
    counter_delta,
    parse_body_datagram,
    parse_joint_angles_datagram,
)


@dataclass
class _PacketSeries:
    valid: int = 0
    first_received_at: float | None = None
    last_received_at: float | None = None
    first_counter: int | None = None
    max_counter: int | None = None
    previous_time_code_ms: int | None = None
    dropped: int = 0
    duplicates: int = 0
    out_of_order: int = 0
    time_steps_ms: list[int] = field(default_factory=list)
    counters: set[int] = field(default_factory=set)
    character_ids: set[int] = field(default_factory=set)

    def add(self, counter: int, time_code_ms: int, received_at: float, character_id: int) -> None:
        self.valid += 1
        if self.first_received_at is None:
            self.first_received_at = received_at
        self.last_received_at = received_at
        already_seen = counter in self.counters
        self.counters.add(counter)
        self.character_ids.add(character_id)

        # Gaps are measured against the highest counter seen, not the previous
        # datagram: otherwise 1,2,4,3,5 would count the 3 as both a drop
        # (2->4) and a second gap (3->5).  A late counter inside an earlier gap
        # was counted as dropped and is credited back here.
        advanced = self.max_counter is None
        if self.max_counter is None:
            self.first_counter = counter
            self.max_counter = counter
        else:
            delta = counter_delta(counter, self.max_counter)
            if delta == 0 or already_seen:
                self.duplicates += 1
            elif delta > 0:
                self.dropped += delta - 1
                self.max_counter = counter
                advanced = True
            else:
                self.out_of_order += 1
                assert self.first_counter is not None
                since_first = (counter - self.first_counter) % UINT32_MODULUS
                span = (self.max_counter - self.first_counter) % UINT32_MODULUS
                if 0 < since_first < span and self.dropped > 0:
                    self.dropped -= 1

        if advanced:
            if self.previous_time_code_ms is not None:
                time_delta = (time_code_ms - self.previous_time_code_ms) % UINT32_MODULUS
                if 0 < time_delta < UINT32_HALF_RANGE:
                    self.time_steps_ms.append(time_delta)
            self.previous_time_code_ms = time_code_ms

    def as_dict(self) -> dict[str, Any]:
        duration = None
        observed_hz = None
        if self.first_received_at is not None and self.last_received_at is not None:
            duration = self.last_received_at - self.first_received_at
            if duration > 0 and self.valid > 1:
                observed_hz = (self.valid - 1) / duration
        median_step = median(self.time_steps_ms) if self.time_steps_ms else None
        mean_step = fmean(self.time_steps_ms) if self.time_steps_ms else None
        time_code_hz = 1000.0 / mean_step if mean_step else None
        expected = self.valid + self.dropped
        loss_fraction = self.dropped / expected if expected else 0.0
        return {
            "valid_datagrams": self.valid,
            "receive_duration_s": duration,
            "observed_receive_hz": observed_hz,
            "median_time_code_step_ms": median_step,
            "mean_time_code_step_ms": mean_step,
            "time_code_hz": time_code_hz,
            "estimated_dropped_frames": self.dropped,
            "loss_fraction": loss_fraction,
            "duplicate_frames": self.duplicates,
            "out_of_order_frames": self.out_of_order,
            "character_ids": sorted(self.character_ids),
        }


class XsensStreamAcceptance:
    """Accumulate and grade the body-only MXTP02 plus MXTP20 stream we consume.

    The strict body parser takes the 23 body segments and skips any prop/finger
    records after them, so GMR always gets one complete 23-segment body frame.
    """

    def __init__(
        self,
        *,
        minimum_body_frames: int = 30,
        minimum_rate_hz: float = 50.0,
        maximum_loss_fraction: float = 0.02,
        require_joint_angles: bool = False,
        quaternion_norm_tolerance: float = 0.02,
    ) -> None:
        if minimum_body_frames < 1:
            raise ValueError("minimum_body_frames must be positive")
        if minimum_rate_hz <= 0:
            raise ValueError("minimum_rate_hz must be positive")
        if not 0 <= maximum_loss_fraction <= 1:
            raise ValueError("maximum_loss_fraction must be between zero and one")
        if quaternion_norm_tolerance <= 0:
            raise ValueError("quaternion_norm_tolerance must be positive")

        self.minimum_body_frames = minimum_body_frames
        self.minimum_rate_hz = minimum_rate_hz
        self.maximum_loss_fraction = maximum_loss_fraction
        self.require_joint_angles = require_joint_angles
        self.quaternion_norm_tolerance = quaternion_norm_tolerance

        self.body = _PacketSeries()
        self.joints = _PacketSeries()
        self.total_datagrams = 0
        self.malformed_datagrams = 0
        self.unknown_datagrams: dict[str, int] = {}
        self.errors: list[str] = []
        self.sources: set[str] = set()
        self.body_spans: list[float] = []
        self.bad_quaternion_norms = 0
        self.minimum_quaternion_norm: float | None = None
        self.maximum_quaternion_norm: float | None = None

    def observe(
        self,
        data: bytes,
        *,
        received_at: float | None = None,
        source: str | None = None,
    ) -> None:
        """Consume one UDP datagram and retain only aggregate diagnostics."""
        self.total_datagrams += 1
        received_at = time.monotonic() if received_at is None else float(received_at)
        if source:
            self.sources.add(source)

        message_id = data[:6]
        if message_id == b"MXTP02":
            try:
                packet = parse_body_datagram(data, strict=True)
            except ProtocolError as error:
                self._malformed("MXTP02", error)
                return
            self.body.add(
                packet.sample_counter,
                packet.time_code_ms,
                received_at,
                packet.character_id,
            )
            pelvis = packet.segments["pelvis"].position
            self.body_spans.append(max(
                math.dist(pelvis, pose.position) for pose in packet.segments.values()
            ))
            for pose in packet.segments.values():
                norm = math.sqrt(sum(value * value for value in pose.quaternion_wxyz))
                if self.minimum_quaternion_norm is None or norm < self.minimum_quaternion_norm:
                    self.minimum_quaternion_norm = norm
                if self.maximum_quaternion_norm is None or norm > self.maximum_quaternion_norm:
                    self.maximum_quaternion_norm = norm
                if abs(norm - 1.0) > self.quaternion_norm_tolerance:
                    self.bad_quaternion_norms += 1
            return

        if message_id == b"MXTP20":
            try:
                packet = parse_joint_angles_datagram(data, strict=True)
            except ProtocolError as error:
                self._malformed("MXTP20", error)
                return
            self.joints.add(
                packet.sample_counter,
                packet.time_code_ms,
                received_at,
                packet.character_id,
            )
            return

        label = message_id.decode("ascii", errors="backslashreplace") or "<empty>"
        self.unknown_datagrams[label] = self.unknown_datagrams.get(label, 0) + 1

    def _malformed(self, message_id: str, error: ProtocolError) -> None:
        self.malformed_datagrams += 1
        if len(self.errors) < 20:
            self.errors.append(f"{message_id}: {error}")

    def report(self) -> dict[str, Any]:
        """Return a stable JSON-compatible acceptance report."""
        body = self.body.as_dict()
        joints = self.joints.as_dict()
        span = median(self.body_spans) if self.body_spans else None
        if span is None:
            unit_assessment = "unavailable"
        elif 0.4 <= span <= 2.5:
            unit_assessment = "metres_plausible"
        elif 20.0 <= span <= 300.0:
            unit_assessment = "centimetres_likely"
        else:
            unit_assessment = "implausible_or_unknown"

        paired_frames = len(self.body.counters & self.joints.counters)
        pairing_fraction = paired_frames / self.body.valid if self.body.valid else 0.0
        failures: list[str] = []
        warnings: list[str] = []

        if self.body.valid < self.minimum_body_frames:
            failures.append(
                f"received {self.body.valid} valid MXTP02 body frames; "
                f"need at least {self.minimum_body_frames}"
            )
        rate = body["observed_receive_hz"]
        if rate is None:
            failures.append("not enough body frames to measure the receive rate")
        elif rate < self.minimum_rate_hz:
            failures.append(
                f"body receive rate {rate:.2f} Hz is below {self.minimum_rate_hz:.2f} Hz"
            )
        if body["loss_fraction"] > self.maximum_loss_fraction:
            failures.append(
                f"body packet loss {body['loss_fraction']:.2%} exceeds "
                f"{self.maximum_loss_fraction:.2%}"
            )
        if body["out_of_order_frames"]:
            failures.append("body sample counters arrived out of order")
        if self.malformed_datagrams:
            failures.append(f"received {self.malformed_datagrams} malformed expected datagrams")
        if self.bad_quaternion_norms:
            # A warning, not a failure (owner, 2026-11-16): MVN sent 2 quaternions of norm 0.90 in
            # 165,600 over Wi-Fi with no loss; the live tool normalises every quaternion.
            warnings.append(
                f"found {self.bad_quaternion_norms} body quaternions outside the "
                f"±{self.quaternion_norm_tolerance:g} norm tolerance (the live tool normalises them)"
            )
        if unit_assessment != "metres_plausible":
            failures.append(f"translation-unit check returned {unit_assessment}")

        if self.require_joint_angles:
            if self.joints.valid < self.minimum_body_frames:
                failures.append(
                    f"received {self.joints.valid} valid MXTP20 joint frames; "
                    f"need at least {self.minimum_body_frames}"
                )
            if pairing_fraction < 0.95:
                failures.append(
                    f"only {pairing_fraction:.1%} of body counters have matching joint packets"
                )
            if joints["loss_fraction"] > self.maximum_loss_fraction:
                failures.append(
                    f"joint packet loss {joints['loss_fraction']:.2%} exceeds "
                    f"{self.maximum_loss_fraction:.2%}"
                )
            if joints["out_of_order_frames"]:
                failures.append("joint sample counters arrived out of order")
        elif not self.joints.valid:
            warnings.append("no MXTP20 joint-angle packets observed")

        if body["duplicate_frames"]:
            warnings.append(f"observed {body['duplicate_frames']} duplicate body counters")
        if joints["duplicate_frames"]:
            warnings.append(f"observed {joints['duplicate_frames']} duplicate joint counters")
        if self.unknown_datagrams:
            warnings.append("other MXTP message types were observed and not decoded")
        if len(self.sources) > 1:
            warnings.append("datagrams arrived from more than one network source")
        if self.body.character_ids and self.joints.character_ids:
            if self.body.character_ids != self.joints.character_ids:
                failures.append("body and joint streams use different character IDs")

        return {
            "schema_version": 1,
            "verdict": "PASS" if not failures else "FAIL",
            "requirements": {
                "minimum_body_frames": self.minimum_body_frames,
                "minimum_rate_hz": self.minimum_rate_hz,
                "maximum_loss_fraction": self.maximum_loss_fraction,
                "require_joint_angles": self.require_joint_angles,
                "quaternion_norm_tolerance": self.quaternion_norm_tolerance,
            },
            "total_datagrams": self.total_datagrams,
            "sources": sorted(self.sources),
            "body": body,
            "joints": joints,
            "paired_body_joint_frames": paired_frames,
            "body_joint_pairing_fraction": pairing_fraction,
            "geometry": {
                "median_max_distance_from_pelvis": span,
                "translation_unit_assessment": unit_assessment,
                "minimum_quaternion_norm": self.minimum_quaternion_norm,
                "maximum_quaternion_norm": self.maximum_quaternion_norm,
                "bad_quaternion_norms": self.bad_quaternion_norms,
            },
            "malformed_datagrams": self.malformed_datagrams,
            "malformed_examples": list(self.errors),
            "unknown_datagrams": dict(sorted(self.unknown_datagrams.items())),
            "failures": failures,
            "warnings": warnings,
        }
