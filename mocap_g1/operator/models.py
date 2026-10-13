"""JSON-safe public models for the operator console.

Dataclasses keep the safety-critical core usable in minimal robot and test
environments which do not install FastAPI or Pydantic.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
import math
import time
from typing import Any, Mapping


class Topology(str, Enum):
    """Supported mocap/control-machine layouts."""

    ALL_IN_ONE = "all_in_one"
    SPLIT = "split"


class SessionState(str, Enum):
    """Authoritative operator-session state.

    ``DISENGAGING`` is deliberately observable, even though the fake runtime
    completes it synchronously.  A hardware adapter may remain in that state
    until it has verified that command authority is zero.
    """

    SETUP_REQUIRED = "setup_required"
    IDLE = "idle"
    CHECKING = "checking"
    MOCAP_ACCEPTED = "mocap_accepted"
    SIM_VALIDATED = "sim_validated"
    ROBOT_CONNECTED = "robot_connected"
    PREFLIGHT_PASSED = "preflight_passed"
    ARMED_DISABLED = "armed_disabled"
    TRACKING = "tracking"
    DISENGAGING = "disengaging"
    FAULT_LATCHED = "fault_latched"


@dataclass(frozen=True)
class CapabilityLock:
    """Result of capability-manifest verification.

    Whole-body hardware control is locked unless every verification field is
    affirmative.  Simulation never depends on this lock.
    """

    whole_body_unlocked: bool = False
    manifest_verified: bool = False
    reason: str = "No verified capability manifest is installed"
    manifest_id: str | None = None
    expires_at: str | None = None
    capabilities: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        return _json_safe(asdict(self))


@dataclass(frozen=True)
class HealthSnapshot:
    """Latest health values consumed by guards and displayed by the GUI."""

    mocap_online: bool = False
    mocap_accepted: bool = False
    mocap_rate_hz: float = 0.0
    packet_loss_percent: float = 100.0
    source_age_ms: float | None = None
    simulation_ready: bool = False
    robot_connected: bool = False
    robot_state_fresh: bool = False
    controller_ready: bool = False
    preflight_confirmed: bool = False
    command_authority: bool = False
    overall: str = "offline"
    updated_at: float = field(default_factory=time.time)
    messages: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        numeric = {
            "mocap_rate_hz": self.mocap_rate_hz,
            "packet_loss_percent": self.packet_loss_percent,
        }
        if self.source_age_ms is not None:
            numeric["source_age_ms"] = self.source_age_ms
        for name, value in numeric.items():
            if not math.isfinite(float(value)) or float(value) < 0.0:
                raise ValueError(f"{name} must be a finite non-negative number")

    def to_dict(self) -> dict[str, Any]:
        return _json_safe(asdict(self))


@dataclass(frozen=True)
class FaultEvent:
    """One latched fault; the operator must explicitly rearm after recovery."""

    code: str
    reason: str
    source: str = "supervisor"
    timestamp: float = field(default_factory=time.time)
    latched: bool = True
    recoverable: bool = True

    def to_dict(self) -> dict[str, Any]:
        return _json_safe(asdict(self))


@dataclass(frozen=True)
class ControlRequest:
    """Auditable operator request accepted by the state machine."""

    action: str
    operator: str = "local-operator"
    spotter: str | None = None
    request_id: str | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict)
    # Self-asserted name supplied by the browser.  ``operator`` is the actor
    # the transport attributes the request to; this is never authoritative.
    claimed_operator: str | None = None

    def __post_init__(self) -> None:
        if not self.action or not self.action.strip():
            raise ValueError("action is required")
        if not self.operator or not self.operator.strip():
            raise ValueError("operator is required")

    def to_dict(self) -> dict[str, Any]:
        return _json_safe(asdict(self))


def _json_safe(value: Any) -> Any:
    """Recursively convert public model data into JSON-native values."""

    if isinstance(value, Enum):
        return value.value
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value
