"""Thread-safe, fail-closed operator session state machine."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import replace
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import threading
import time
from typing import Any, Callable, Mapping
import uuid

from .adapter_worker import AdapterWorker
from .audit import AuditLog, redact
from .models import (
    CapabilityLock,
    ControlRequest,
    FaultEvent,
    HealthSnapshot,
    SessionState,
    Topology,
)
from .runtime import DemoRuntime, PassiveRuntime, RuntimeAdapter


SCHEMA_VERSION = 1
# Mocap stream acceptance gates.  They are checked at ``mocap-accept`` and
# re-applied on every health change and watchdog tick while tracking.
MOCAP_MIN_RATE_HZ = 50.0
MOCAP_MAX_PACKET_LOSS_PERCENT = 1.0
MOCAP_MAX_SOURCE_AGE_MS = 100.0
# Maximum age of the controller's own view of runtime health while tracking.
# Health must be refreshed by ``update_health`` or a non-empty runtime result
# (check, simulation, robot, recovery, or the periodic watchdog ``tick``).  The
# 100 ms source-age budget is measured by the runtime; this bound covers the
# case where nobody reports at all.  The web watchdog ticks every 100 ms, so
# 250 ms tolerates one late tick plus scheduling jitter while still latching
# well inside the 500 ms safety-envelope hard timeout.
DEFAULT_HEALTH_STALE_TIMEOUT_S = 0.25
# Deadlines for runtime-adapter calls, which run on one supervised worker
# thread (see ``adapter_worker.py``).  The controller lock is held while a call
# runs, so these bound how long any other action -- including disengage -- can
# wait behind a hung adapter.  ``tick`` defaults to the health staleness
# budget: a tick slower than that is as bad as stale health.  A call that
# overruns leaves the worker blocked; later calls fail fast until it returns.
DEFAULT_ADAPTER_TIMEOUTS_S: Mapping[str, float] = {
    "check": 5.0,
    "start_simulation": 5.0,
    "connect_robot": 5.0,
    "recover": 5.0,
    "shutdown": 5.0,
    "safe_stop": 1.0,
    "set_tracking": 0.5,
    "simulation_snapshot": 0.25,
}
# Health fields whose change is always audited by ``update_health``.  The
# remaining numeric telemetry (rate, loss, source age) is audited at most once
# per ``HEALTH_AUDIT_INTERVAL_S`` so a high-rate supervisor cannot evict
# control events from the bounded in-memory event ring.
_MATERIAL_HEALTH_FIELDS = frozenset(
    {
        "mocap_online",
        "mocap_accepted",
        "simulation_ready",
        "robot_connected",
        "robot_state_fresh",
        "controller_ready",
        "preflight_confirmed",
        "command_authority",
        "overall",
        "messages",
    }
)
HEALTH_AUDIT_INTERVAL_S = 5.0
# Every operator action, in checklist order.  The HTTP route allowlist and the
# controller's handler table are both derived from this tuple.
ACTIONS: tuple[str, ...] = (
    "check",
    "mocap-accept",
    "start-simulation",
    "connect-robot",
    "confirm-preflight",
    "arm",
    "start-tracking",
    "disengage",
    "rearm",
    "shutdown",
)
# States in which a robot connection may exist.  An adapter call that times
# out here may still complete later, so the controller latches a fault and
# requires a verified safe_stop before the session can continue.
_ROBOT_SESSION_STATES = frozenset(
    {
        SessionState.SIM_VALIDATED,
        SessionState.ROBOT_CONNECTED,
        SessionState.PREFLIGHT_PASSED,
        SessionState.ARMED_DISABLED,
        SessionState.TRACKING,
        SessionState.DISENGAGING,
    }
)
_CONFIG_LOCKED_STATES = set(SessionState) - {
    SessionState.SETUP_REQUIRED,
    SessionState.IDLE,
}
# A non-recoverable fault survives shutdown; only stopping stays available
# until the operator service process is restarted.
_RESTART_SAFE_ACTIONS = frozenset({"disengage", "shutdown"})
_RESTART_REQUIRED_REASON = (
    "The active fault is not recoverable; restart the operator service "
    "before starting another session"
)


class OperatorError(RuntimeError):
    """Base error with a stable machine-readable code."""

    def __init__(self, message: str, *, code: str, status_code: int = 409) -> None:
        super().__init__(message)
        self.code = code
        self.status_code = status_code


class InvalidAction(OperatorError):
    def __init__(self, message: str, *, code: str = "invalid_action") -> None:
        super().__init__(message, code=code, status_code=409)


class CapabilityLocked(OperatorError):
    def __init__(self, message: str) -> None:
        super().__init__(message, code="capability_locked", status_code=423)


class InvalidConfiguration(OperatorError):
    def __init__(
        self,
        message: str,
        *,
        code: str = "invalid_configuration",
        status_code: int = 422,
    ) -> None:
        super().__init__(message, code=code, status_code=status_code)


class RuntimeAdapterTimeout(OperatorError):
    """A runtime-adapter call did not return within its deadline."""

    def __init__(self, call: str, timeout_s: float) -> None:
        super().__init__(
            f"Runtime adapter call {call} did not return within {timeout_s:.3f} s",
            code="runtime_adapter_timeout",
            status_code=503,
        )
        self.call = call


class RuntimeAdapterUnavailable(OperatorError):
    """An earlier adapter call is still overdue; nothing else can be sent."""

    def __init__(self, call: str, blocked_call: str) -> None:
        super().__init__(
            f"Runtime adapter call {call} was not sent: the adapter is still "
            f"blocked in an overdue {blocked_call} call",
            code="runtime_adapter_unavailable",
            status_code=503,
        )
        self.call = call
        self.blocked_call = blocked_call


_ADAPTER_BLOCKED_ERRORS = (RuntimeAdapterTimeout, RuntimeAdapterUnavailable)


def runtime_mode_for(*, simulated: bool) -> str:
    """The configuration ``runtime_mode`` for a launcher's runtime choice."""

    return "fake" if simulated else "real"


def default_config(*, demo: bool = False) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "topology": Topology.ALL_IN_ONE.value,
        "runtime_mode": runtime_mode_for(simulated=demo),
        "mocap": {
            "listen_address": "0.0.0.0",
            "port": 5555,
            "source_ip": None,
        },
        "robot": {"interface": None},
        "privacy": {"record_raw_mocap": False},
    }


def _merge_dict(base: Mapping[str, Any], updates: Mapping[str, Any]) -> dict[str, Any]:
    result = deepcopy(dict(base))
    for key, value in updates.items():
        if isinstance(value, Mapping) and isinstance(result.get(key), Mapping):
            result[key] = _merge_dict(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


def validate_config(config: Mapping[str, Any]) -> dict[str, Any]:
    """Validate and normalize the small v1 operator configuration."""

    allowed_top = {"schema_version", "topology", "runtime_mode", "mocap", "robot", "privacy"}
    unknown = sorted(set(config) - allowed_top)
    if unknown:
        raise InvalidConfiguration(f"unknown configuration keys: {', '.join(unknown)}")
    try:
        schema_version = int(config["schema_version"])
        topology = Topology(str(config["topology"]))
        runtime_mode = str(config["runtime_mode"])
        mocap = dict(config["mocap"])
        robot = dict(config["robot"])
        privacy = dict(config["privacy"])
    except (KeyError, TypeError, ValueError) as exc:
        raise InvalidConfiguration(f"configuration shape is invalid: {exc}") from exc
    if schema_version != SCHEMA_VERSION:
        raise InvalidConfiguration(
            f"unsupported schema_version {schema_version}; expected {SCHEMA_VERSION}"
        )
    if runtime_mode not in {"real", "fake"}:
        raise InvalidConfiguration("runtime_mode must be 'real' or 'fake'")
    for label, values, allowed in (
        ("mocap", mocap, {"listen_address", "port", "source_ip"}),
        ("robot", robot, {"interface"}),
        ("privacy", privacy, {"record_raw_mocap"}),
    ):
        extras = sorted(set(values) - allowed)
        if extras:
            raise InvalidConfiguration(f"unknown {label} keys: {', '.join(extras)}")
    listen_address = mocap.get("listen_address")
    if not isinstance(listen_address, str) or not listen_address.strip():
        raise InvalidConfiguration("mocap.listen_address must be a non-empty string")
    try:
        port = int(mocap.get("port"))
    except (TypeError, ValueError) as exc:
        raise InvalidConfiguration("mocap.port must be an integer") from exc
    if port < 1 or port > 65535:
        raise InvalidConfiguration("mocap.port must be between 1 and 65535")
    source_ip = mocap.get("source_ip")
    if source_ip is not None and (not isinstance(source_ip, str) or not source_ip.strip()):
        raise InvalidConfiguration("mocap.source_ip must be null or a non-empty string")
    interface = robot.get("interface")
    if interface is not None and (not isinstance(interface, str) or not interface.strip()):
        raise InvalidConfiguration("robot.interface must be null or a non-empty string")
    if topology is Topology.SPLIT and source_ip is None:
        raise InvalidConfiguration("split topology requires mocap.source_ip")
    record_raw = privacy.get("record_raw_mocap")
    if not isinstance(record_raw, bool):
        raise InvalidConfiguration("privacy.record_raw_mocap must be boolean")
    return {
        "schema_version": SCHEMA_VERSION,
        "topology": topology.value,
        "runtime_mode": runtime_mode,
        "mocap": {
            "listen_address": listen_address.strip(),
            "port": port,
            "source_ip": source_ip.strip() if isinstance(source_ip, str) else None,
        },
        "robot": {"interface": interface.strip() if isinstance(interface, str) else None},
        "privacy": {"record_raw_mocap": record_raw},
    }


class SessionController:
    """Authoritative state, guards, audit, and fake-runtime progression.

    The controller never sends a robot command.  A future supervisor adapter
    supplies real health updates and calls :meth:`latch_fault` on watchdog or
    hardware failures.  All public mutation methods share one re-entrant lock.
    """

    def __init__(
        self,
        *,
        audit: AuditLog | None = None,
        config: Mapping[str, Any] | None = None,
        config_path: str | Path | None = None,
        capability_lock: CapabilityLock | None = None,
        runtime: RuntimeAdapter | None = None,
        demo: bool = False,
        setup_complete: bool = True,
        event_limit: int = 1000,
        health_stale_timeout_s: float = DEFAULT_HEALTH_STALE_TIMEOUT_S,
        clock: Callable[[], float] = time.monotonic,
        adapter_timeouts_s: Mapping[str, float] | None = None,
    ) -> None:
        stale_timeout = float(health_stale_timeout_s)
        if not stale_timeout > 0.0 or stale_timeout == float("inf"):
            raise ValueError("health_stale_timeout_s must be positive and finite")
        timeouts = {**DEFAULT_ADAPTER_TIMEOUTS_S, "tick": stale_timeout}
        for name, value in (adapter_timeouts_s or {}).items():
            if name not in timeouts:
                raise ValueError(f"unknown adapter timeout: {name}")
            timeouts[name] = float(value)
        for name, value in timeouts.items():
            if not 0.0 < value < float("inf"):
                raise ValueError(f"adapter timeout {name} must be positive and finite")
        self._adapter_timeouts_s = timeouts
        self._adapter = AdapterWorker(
            timeout_error=RuntimeAdapterTimeout,
            unavailable_error=RuntimeAdapterUnavailable,
        )
        self._lock = threading.RLock()
        # Monotonic clock for health staleness.  Wall-clock ``time.time`` is
        # kept for display timestamps and capability expiry only.
        self._clock = clock
        self._health_stale_timeout_s = stale_timeout
        self._health_refreshed_at = float(clock())
        self.audit = audit or AuditLog()
        self.config_path = Path(config_path) if config_path is not None else None
        launch_demo = bool(demo or (runtime is not None and runtime.is_simulated))
        raw_config = default_config(demo=launch_demo)
        if config is not None:
            raw_config = _merge_dict(raw_config, config)
        # Runtime selection is a launcher decision. A persisted config cannot
        # silently switch a live process onto the simulated adapter.
        raw_config["runtime_mode"] = runtime_mode_for(simulated=launch_demo)
        self._config = validate_config(raw_config)
        # Raw mocap is sensitive body-motion data.  Consent is deliberately
        # session-scoped: never inherit an opt-in from a prior process, even if
        # that process exited before its normal shutdown reset.
        inherited_raw_opt_in = self._config["privacy"]["record_raw_mocap"]
        if inherited_raw_opt_in:
            self._config = deepcopy(self._config)
            self._config["privacy"]["record_raw_mocap"] = False
            self._persist_config(self._config)
        self._runtime = runtime or (DemoRuntime() if launch_demo else PassiveRuntime())
        self._demo = bool(
            self._runtime.is_simulated
            # Fail closed: a simulated adapter bypasses the hardware capability
            # only when it explicitly declares that it can never reach hardware.
            and getattr(self._runtime, "allows_hardware_bypass", False) is True
        )
        self._virtual_robot = bool(
            self._runtime.is_simulated
            and getattr(self._runtime, "uses_virtual_robot", False)
        )
        self._state = SessionState.IDLE if setup_complete else SessionState.SETUP_REQUIRED
        self._health = HealthSnapshot(
            overall="ready" if setup_complete else "setup_required",
            messages=("Operator service ready",) if setup_complete else ("Complete setup",),
        )
        self._capability_lock = capability_lock or CapabilityLock()
        self._active_fault: FaultEvent | None = None
        self._fault_condition_cleared = False
        self._stop_verification_required = False
        self._event_limit = max(10, int(event_limit))
        self._events: list[dict[str, Any]] = []
        self._event_sequence = 0
        # The in-memory event counter restarts with the process.  Clients use
        # this per-process identifier to detect a restart and re-bootstrap
        # rather than resuming with a cursor from a previous instance.
        self._instance_id = uuid.uuid4().hex
        # Last watchdog-tick failure outside TRACKING (None when healthy) and
        # the ``overall`` value it displaced, restored once ticks succeed.
        self._tick_error: str | None = None
        self._tick_error_prior_overall: str | None = None
        self._last_health_audit_at: float | None = None
        self._record_event("session", "initialize", "accepted", actor="system")

    @property
    def state(self) -> SessionState:
        with self._lock:
            return self._state

    @property
    def health(self) -> HealthSnapshot:
        with self._lock:
            return self._health

    @property
    def instance_id(self) -> str:
        return self._instance_id

    @property
    def capability_lock(self) -> CapabilityLock:
        with self._lock:
            return self._capability_lock

    def get_config(self) -> dict[str, Any]:
        with self._lock:
            return deepcopy(self._config)

    def update_config(
        self,
        updates: Mapping[str, Any],
        *,
        actor: str = "local-operator",
        claimed_operator: str | None = None,
    ) -> dict[str, Any]:
        with self._lock:
            if self._state in _CONFIG_LOCKED_STATES:
                self._reject(
                    "config",
                    actor,
                    (
                        "Configuration cannot change after session checks begin; "
                        "edit only before session checks"
                    ),
                    code="configuration_locked",
                    claimed_operator=claimed_operator,
                )
            if not isinstance(updates, Mapping):
                raise InvalidConfiguration("configuration update must be an object")
            candidate = validate_config(_merge_dict(self._config, updates))
            if candidate["runtime_mode"] != self._config["runtime_mode"]:
                raise InvalidConfiguration(
                    "runtime_mode is set by the service launcher and cannot change via config"
                )
            self._persist_config(candidate)
            self._config = candidate
            if self._state is SessionState.SETUP_REQUIRED:
                self._state = SessionState.IDLE
                self._health = replace(
                    self._health,
                    overall="ready",
                    updated_at=time.time(),
                    messages=("Operator service ready",),
                )
            details: dict[str, Any] = {"updates": updates}
            if claimed_operator is not None:
                details["claimed_operator"] = claimed_operator
            self._record_event(
                "configuration",
                "update",
                "accepted",
                actor=actor,
                details=details,
            )
            return deepcopy(self._config)

    def set_capability_lock(
        self, capability_lock: CapabilityLock, *, actor: str = "system"
    ) -> None:
        with self._lock:
            if self._state in _CONFIG_LOCKED_STATES:
                self._reject(
                    "capability",
                    actor,
                    "Capability manifest cannot change in the current state",
                    code="configuration_locked",
                )
            self._capability_lock = capability_lock
            self._record_event(
                "capability",
                "verify_manifest",
                "accepted" if capability_lock.manifest_verified else "rejected",
                actor=actor,
                details=capability_lock.to_dict(),
            )

    def update_health(self, *, actor: str = "runtime", **changes: Any) -> None:
        """Apply runtime health and automatically latch critical tracking loss."""

        with self._lock:
            allowed = set(HealthSnapshot.__dataclass_fields__) - {"updated_at"}
            unknown = sorted(set(changes) - allowed)
            if unknown:
                raise ValueError(f"unknown health fields: {', '.join(unknown)}")
            if "messages" in changes:
                changes["messages"] = tuple(changes["messages"])
            previous = self._health
            applied = self._clamp_command_authority(changes, actor=actor)
            self._health = replace(self._health, **applied, updated_at=time.time())
            now = float(self._clock())
            self._health_refreshed_at = now
            if self._state is SessionState.TRACKING:
                self._expire_capability_locked()
            if self._state is SessionState.TRACKING:
                critical = self._tracking_health_error()
                if critical is not None:
                    self._latch_fault_locked(
                        FaultEvent(code="tracking_health_lost", reason=critical, source=actor)
                    )
            material = any(
                name in _MATERIAL_HEALTH_FIELDS and getattr(previous, name) != value
                for name, value in applied.items()
            )
            last = self._last_health_audit_at
            if material or last is None or not 0.0 <= now - last < HEALTH_AUDIT_INTERVAL_S:
                self._last_health_audit_at = now
                self._record_event_best_effort(
                    "health",
                    "update",
                    "accepted",
                    actor=actor,
                    details={"changes": changes},
                )

    def clear_fault_condition(self, code: str, *, actor: str = "runtime") -> None:
        """Mark a diagnosed fault cause cleared; this does not rearm motion."""

        with self._lock:
            if self._state is not SessionState.FAULT_LATCHED or self._active_fault is None:
                raise InvalidAction("There is no latched fault to clear")
            if code != self._active_fault.code:
                raise InvalidAction("Fault code does not match the active fault")
            self._fault_condition_cleared = True
            self._record_event(
                "fault", "clear_condition", "accepted", actor=actor, details={"code": code}
            )

    def latch_fault(self, fault: FaultEvent, *, actor: str = "runtime") -> None:
        with self._lock:
            self._latch_fault_locked(fault, actor=actor)

    def perform(self, request: ControlRequest) -> dict[str, Any]:
        action = request.action.strip().lower().replace("_", "-")
        if action not in ACTIONS:
            raise InvalidAction(f"Unknown action: {request.action}")
        handler = getattr(self, "_" + action.replace("-", "_"))
        with self._lock:
            self._expire_capability_locked()
            try:
                if action not in _RESTART_SAFE_ACTIONS and self._restart_required():
                    raise InvalidAction(
                        _RESTART_REQUIRED_REASON, code="fault_requires_restart"
                    )
                handler(request)
            except OperatorError as exc:
                if isinstance(exc, _ADAPTER_BLOCKED_ERRORS):
                    self._latch_adapter_fault_locked(exc)
                self._record_rejection(request, action, reason=str(exc), code=exc.code)
                raise
            except Exception as exc:
                # An adapter (or internal) error still leaves an audit trail;
                # the original exception propagates unchanged.
                self._record_rejection(
                    request,
                    action,
                    reason=f"{type(exc).__name__}: {exc}",
                    code="runtime_error",
                    error_type=type(exc).__name__,
                    best_effort=True,
                )
                raise
            return self.status_snapshot()

    def _record_rejection(
        self,
        request: ControlRequest,
        action: str,
        *,
        reason: str,
        code: str,
        error_type: str | None = None,
        best_effort: bool = False,
    ) -> None:
        details: dict[str, Any] = {
            "reason": reason,
            "code": code,
            "request": request.to_dict(),
        }
        if error_type is not None:
            details["error_type"] = error_type
        if request.claimed_operator is not None:
            details["claimed_operator"] = request.claimed_operator
        recorder = self._record_event_best_effort if best_effort else self._record_event
        recorder("control", action, "rejected", actor=request.operator, details=details)

    def _latch_adapter_fault_locked(self, exc: OperatorError) -> None:
        """Latch a fault when an adapter call overran while a robot may be live.

        A timed-out call may still complete later (``connect_robot`` or
        ``set_tracking(True)`` included), so its outcome is unknown; require a
        verified safe_stop before the session continues.
        """

        if self._state not in _ROBOT_SESSION_STATES:
            return
        self._latch_fault_locked(
            FaultEvent(
                code=exc.code,
                reason=str(exc),
                source="controller",
                recoverable=True,
            ),
            actor="controller",
        )

    def report_watchdog_overdue(self, elapsed_s: float, *, actor: str = "watchdog") -> None:
        """Called by the transport when a watchdog tick has not returned in time.

        While tracking this latches ``watchdog_overdue``; otherwise the delay is
        only recorded, because no time-based authority check is pending.
        """

        with self._lock:
            reason = (
                "Operator watchdog tick did not complete for "
                f"{float(elapsed_s) * 1000.0:.0f} ms"
            )
            if self._state is SessionState.TRACKING:
                self._latch_fault_locked(
                    FaultEvent(
                        code="watchdog_overdue",
                        reason=reason,
                        source="watchdog",
                        recoverable=True,
                    ),
                    actor=actor,
                )
                return
            self._record_event_best_effort(
                "runtime",
                "watchdog_overdue",
                "rejected",
                actor=actor,
                details={"reason": reason},
            )

    def status_snapshot(self) -> dict[str, Any]:
        with self._lock:
            self._expire_capability_locked()
            return {
                "schema_version": SCHEMA_VERSION,
                "instance_id": self._instance_id,
                "topology": self._config["topology"],
                "runtime_mode": self._config["runtime_mode"],
                "state": self._state.value,
                "capability_lock": self._capability_lock.to_dict(),
                "health": self._health.to_dict(),
                "active_fault": (
                    self._active_fault.to_dict() if self._active_fault is not None else None
                ),
                "fault_condition_cleared": self._fault_condition_cleared,
                "last_event_sequence": self._event_sequence,
                "controls": self._control_availability(),
                "simulation": self._simulation_snapshot_locked(),
                "runtime_adapter": {"blocked_call": self._adapter.blocked_call},
                "audit_integrity": self._audit_integrity_summary(),
            }

    def _simulation_snapshot_locked(self) -> dict[str, Any] | None:
        if not callable(getattr(self._runtime, "simulation_snapshot", None)):
            return None
        try:
            return self._call_runtime("simulation_snapshot")
        except _ADAPTER_BLOCKED_ERRORS:
            # Status must stay available while the adapter is blocked; the
            # blocked call is reported under ``runtime_adapter``.
            return None

    def _audit_integrity_summary(self) -> dict[str, Any] | None:
        summary = getattr(self.audit, "integrity_summary", None)
        return summary() if callable(summary) else None

    def watchdog_tick(self) -> None:
        """Enforce time-based safety checks without operator or health traffic."""

        with self._lock:
            self._expire_capability_locked()
            if callable(getattr(self._runtime, "tick", None)):
                try:
                    changes = self._call_runtime("tick", self._health)
                    # An empty result reports nothing new and is deliberately
                    # not treated as a heartbeat.
                    if changes:
                        self._apply_runtime_health(changes)
                    self._clear_tick_error_locked()
                    self._enforce_tracking_health_locked()
                except Exception as exc:
                    self._handle_tick_error_locked(exc)
            else:
                # Without a runtime heartbeat, health must arrive through
                # update_health; silence still ages it out.
                self._enforce_tracking_health_locked()

    def _handle_tick_error_locked(self, exc: Exception) -> None:
        if isinstance(exc, _ADAPTER_BLOCKED_ERRORS):
            code = exc.code
            reason = f"Runtime watchdog failed: {exc}"
        else:
            code = "runtime_watchdog_error"
            reason = f"Runtime watchdog failed: {type(exc).__name__}: {exc}"
        fault = FaultEvent(code=code, reason=reason, source="controller", recoverable=True)
        if self._state is SessionState.TRACKING:
            self._latch_fault_locked(fault, actor="controller")
            return
        if self._tick_error is None:
            self._tick_error_prior_overall = self._health.overall
        self._health = replace(
            self._health,
            command_authority=False,
            overall="fault",
            updated_at=time.time(),
            messages=(fault.reason,),
        )
        # Ticks repeat every 100 ms; audit a failure only when it changes so a
        # persistent error cannot flood the event ring.
        if reason != self._tick_error:
            self._tick_error = reason
            self._record_event_best_effort(
                "runtime",
                "watchdog_tick",
                "rejected",
                actor="controller",
                details={"code": code, "reason": reason},
            )

    def _clear_tick_error_locked(self) -> None:
        if self._tick_error is None:
            return
        self._tick_error = None
        prior = self._tick_error_prior_overall
        self._tick_error_prior_overall = None
        if self._state is not SessionState.FAULT_LATCHED and self._health.overall == "fault":
            self._health = replace(
                self._health,
                overall=prior if prior not in (None, "fault") else "ready",
                updated_at=time.time(),
                messages=("Runtime watchdog recovered",),
            )
        self._record_event_best_effort(
            "runtime", "watchdog_tick", "accepted", actor="controller",
            details={"recovered": True},
        )

    def _call_runtime(self, name: str, *args: Any) -> Any:
        """Call one runtime-adapter method on the supervised adapter thread."""

        return self._adapter.call(
            name,
            getattr(self._runtime, name),
            *args,
            timeout_s=self._adapter_timeouts_s[name],
        )

    def events_after(self, sequence: int = 0, *, limit: int = 200) -> list[dict[str, Any]]:
        if limit < 1 or limit > 5000:
            raise ValueError("limit must be between 1 and 5000")
        with self._lock:
            return deepcopy(
                [event for event in self._events if event["sequence"] > sequence][:limit]
            )

    def _require(self, expected: SessionState, action: str) -> None:
        if self._state is not expected:
            raise InvalidAction(
                f"{action} requires state {expected.value}; current state is {self._state.value}"
            )

    def _check(self, request: ControlRequest) -> None:
        self._require(SessionState.IDLE, "check")
        # Apply the runtime's report first; a failing check leaves the
        # session in IDLE (and perform() audits the rejection).
        try:
            changes = self._call_runtime("check", self._health, self._config)
        except OperatorError:
            raise
        except Exception as exc:
            raise InvalidAction(
                f"Runtime check failed: {type(exc).__name__}: {exc}",
                code="runtime_error",
            ) from exc
        self._apply_runtime_health(changes)
        self._state = SessionState.CHECKING
        self._accept(request, "check")

    def _mocap_accept(self, request: ControlRequest) -> None:
        self._require(SessionState.CHECKING, "mocap-accept")
        stream_error = self._mocap_stream_error()
        if stream_error is not None:
            reason, code = stream_error
            raise InvalidAction(reason, code=code)
        self._health = replace(
            self._health,
            mocap_accepted=True,
            overall="ready",
            updated_at=time.time(),
            messages=("Mocap stream accepted",),
        )
        self._state = SessionState.MOCAP_ACCEPTED
        self._accept(request, "mocap-accept")

    def _start_simulation(self, request: ControlRequest) -> None:
        self._require(SessionState.MOCAP_ACCEPTED, "start-simulation")
        self._apply_runtime_health(
            self._call_runtime("start_simulation", self._health, self._config)
        )
        if not self._health.simulation_ready:
            raise InvalidAction(
                "Simulation runtime has not reported a passing validation",
                code="simulation_not_ready",
            )
        self._state = SessionState.SIM_VALIDATED
        self._accept(request, "start-simulation")

    def _connect_robot(self, request: ControlRequest) -> None:
        self._require(SessionState.SIM_VALIDATED, "connect-robot")
        if not self._demo and not self._virtual_robot:
            self._require_hardware_capability()
        self._apply_runtime_health(
            self._call_runtime("connect_robot", self._health, self._config)
        )
        if not self._health.robot_connected or not self._health.robot_state_fresh:
            raise InvalidAction(
                "Robot supervisor has not reported a fresh connection",
                code="robot_not_ready",
            )
        self._state = SessionState.ROBOT_CONNECTED
        self._accept(request, "connect-robot")

    def _confirm_preflight(self, request: ControlRequest) -> None:
        self._require(SessionState.ROBOT_CONNECTED, "confirm-preflight")
        if not request.spotter or not request.spotter.strip():
            raise InvalidAction("A spotter name is required", code="preflight_incomplete")
        required = ("clear_area", "harness_or_support", "stop_drill")
        missing = [name for name in required if request.metadata.get(name) is not True]
        if missing:
            raise InvalidAction(
                f"Preflight confirmations missing: {', '.join(missing)}",
                code="preflight_incomplete",
            )
        self._health = replace(
            self._health,
            preflight_confirmed=True,
            updated_at=time.time(),
            messages=("Operator and spotter preflight confirmed",),
        )
        self._state = SessionState.PREFLIGHT_PASSED
        self._accept(request, "confirm-preflight")

    def _arm(self, request: ControlRequest) -> None:
        self._require(SessionState.PREFLIGHT_PASSED, "arm")
        if not self._demo and not self._virtual_robot:
            self._require_hardware_capability()
        if not self._health.robot_state_fresh or not self._health.controller_ready:
            raise InvalidAction("Fresh robot/controller state is required", code="robot_not_ready")
        self._state = SessionState.ARMED_DISABLED
        self._health = replace(
            self._health,
            command_authority=False,
            updated_at=time.time(),
            messages=("Armed with command output disabled",),
        )
        self._accept(request, "arm")

    def _start_tracking(self, request: ControlRequest) -> None:
        self._require(SessionState.ARMED_DISABLED, "start-tracking")
        if not self._demo and not self._virtual_robot:
            self._require_hardware_capability()
        health_error = self._tracking_health_error()
        if health_error is not None:
            raise InvalidAction(health_error, code="tracking_not_ready")
        if self._health.overall == "fault":
            raise InvalidAction(
                "Runtime health reports a fault: "
                + ("; ".join(self._health.messages) or "no detail"),
                code="tracking_not_ready",
            )
        self._notify_runtime_tracking(True, required=True)
        self._health = replace(
            self._health,
            command_authority=True,
            overall="tracking",
            updated_at=time.time(),
            messages=("Tracking active",),
        )
        self._state = SessionState.TRACKING
        try:
            self._accept(request, "start-tracking")
        except Exception as exc:
            # Authority must never outlive a failed audit write.  The runtime
            # has already been told to track, so withdraw it through the same
            # verified safe-stop path as any other fault; the operator must
            # explicitly rearm once auditing works again.
            self._latch_fault_locked(
                FaultEvent(
                    code="audit_write_failed",
                    reason=(
                        "Audit write failed while granting command authority: "
                        f"{type(exc).__name__}: {exc}"
                    ),
                    source="controller",
                    recoverable=True,
                ),
                actor="controller",
            )
            raise

    def _disengage(self, request: ControlRequest) -> None:
        self._notify_runtime_tracking(False, required=False)
        if self._state is SessionState.FAULT_LATCHED:
            failure = self._safe_stop_locked()
            details: dict[str, Any] = {"idempotent": True, "verified": failure is None}
            if failure is not None:
                details["safe_stop"] = failure
            self._accept(request, "disengage", details=details, audit_best_effort=True)
            return
        if self._state in {SessionState.ARMED_DISABLED, SessionState.TRACKING}:
            self._state = SessionState.DISENGAGING
            self._health = replace(
                self._health,
                command_authority=False,
                overall="disengaging",
                updated_at=time.time(),
                messages=("Disengaging command authority",),
            )
            failure = self._safe_stop_locked()
            if failure is not None:
                if "error_type" in failure:
                    fault = FaultEvent(
                        code="disengage_runtime_error",
                        reason=(
                            f"Runtime safe_stop failed: {failure['error_type']}: "
                            f"{failure['reason']}"
                        ),
                        source="controller",
                        recoverable=True,
                    )
                else:
                    fault = FaultEvent(
                        code="disengage_unverified",
                        reason=(
                            "Runtime safe_stop returned without confirming zero "
                            "command authority"
                        ),
                        source="controller",
                        recoverable=True,
                    )
                self._latch_fault_locked(fault, actor="controller")
                self._accept(
                    request,
                    "disengage",
                    details={"verified": False, "safe_stop": failure},
                    audit_best_effort=True,
                )
                return
            self._record_event_best_effort(
                "transition", "disengaging", "accepted", actor=request.operator
            )
            # Fake/core-only runtime can verify zero authority synchronously.
            # A hardware supervisor will call this only after its own zero-output
            # verification and can retain DISENGAGING while that work completes.
            self._state = SessionState.ARMED_DISABLED
            self._health = replace(
                self._health,
                overall="ready",
                updated_at=time.time(),
                messages=("Command authority disabled",),
            )
            self._accept(request, "disengage", audit_best_effort=True)
            return
        # Safe states treat disengage as an idempotent stop request.
        self._health = replace(self._health, command_authority=False, updated_at=time.time())
        self._accept(
            request, "disengage", details={"idempotent": True}, audit_best_effort=True
        )

    def _rearm(self, request: ControlRequest) -> None:
        self._require(SessionState.FAULT_LATCHED, "rearm")
        assert self._active_fault is not None
        if not self._active_fault.recoverable:
            raise InvalidAction("The active fault is not recoverable", code="fault_not_recoverable")
        if self._stop_verification_required:
            raise InvalidAction(
                "Cannot rearm until the runtime verifies zero command authority",
                code="stop_unverified",
            )
        if self._demo:
            self._fault_condition_cleared = True
        if (
            not self._fault_condition_cleared
            and getattr(self._runtime, "supports_operator_recovery", False)
        ):
            if callable(getattr(self._runtime, "recover", None)):
                try:
                    self._apply_runtime_health(self._call_runtime("recover", self._health))
                except Exception as exc:
                    raise InvalidAction(
                        "Runtime recovery failed: "
                        f"{type(exc).__name__}: {exc}",
                        code="fault_not_cleared",
                    ) from exc
                self._fault_condition_cleared = True
                self._record_event_best_effort(
                    "runtime", "recover", "accepted", actor=request.operator
                )
        if not self._fault_condition_cleared:
            raise InvalidAction(
                "The runtime has not cleared the fault condition", code="fault_not_cleared"
            )
        health_error = self._tracking_health_error()
        if health_error is not None:
            raise InvalidAction(health_error, code="rearm_not_ready")
        latched_fault = self._active_fault
        latched_health = self._health
        self._active_fault = None
        self._fault_condition_cleared = False
        self._state = SessionState.ARMED_DISABLED
        self._health = replace(
            self._health,
            command_authority=False,
            overall="ready",
            updated_at=time.time(),
            messages=("Fault cleared; explicitly start tracking when ready",),
        )
        try:
            self._accept(
                request, "rearm", details={"cleared_fault": latched_fault.code}
            )
        except Exception:
            # A fault is only cleared with a durable record of who cleared it.
            self._active_fault = latched_fault
            self._fault_condition_cleared = True
            self._state = SessionState.FAULT_LATCHED
            self._health = replace(
                latched_health, command_authority=False, updated_at=time.time()
            )
            raise

    def _shutdown(self, request: ControlRequest) -> None:
        if self._stop_verification_required:
            self._verify_stop_before_shutdown_locked(actor=request.operator)
        if self._state in {SessionState.TRACKING, SessionState.ARMED_DISABLED}:
            self._disengage(
                ControlRequest(
                    action="disengage",
                    operator=request.operator,
                    claimed_operator=request.claimed_operator,
                    metadata={},
                )
            )
            if self._state is SessionState.FAULT_LATCHED:
                raise InvalidAction(
                    "Runtime shutdown is blocked because zero command authority was not verified",
                    code="shutdown_stop_unverified",
                )
        self._notify_runtime_tracking(False, required=False)
        try:
            self._call_runtime("shutdown", self._health)
        except Exception as exc:
            fault = FaultEvent(
                code="shutdown_runtime_error",
                reason=f"Runtime shutdown failed: {type(exc).__name__}: {exc}",
                source="controller",
                recoverable=False,
            )
            self._latch_fault_locked(fault, actor="controller")
            raise
        if self._state is SessionState.SETUP_REQUIRED:
            # No session can have started, and the in-memory configuration is
            # only the defaults standing in for a rejected config.json.  Never
            # persist it (that would overwrite the file the operator must
            # review) and stay in SETUP_REQUIRED until a valid save.
            self._config = deepcopy(self._config)
            self._config["privacy"]["record_raw_mocap"] = False
            self._fault_condition_cleared = False
            self._stop_verification_required = False
            self._health = HealthSnapshot(
                overall="setup_required", messages=("Complete setup",)
            )
            self._accept(
                request,
                "shutdown",
                details={"setup_required": True, "config_persisted": False},
            )
            return
        # End the per-session recording consent and durably reset it before a
        # future process can begin another session.
        self._config = deepcopy(self._config)
        self._config["privacy"]["record_raw_mocap"] = False
        self._persist_config(self._config)
        self._fault_condition_cleared = False
        self._stop_verification_required = False
        if self._restart_required():
            # The runtime is stopped, but a non-recoverable fault stays latched
            # for the life of this process so no new session can begin.
            assert self._active_fault is not None
            self._health = HealthSnapshot(
                overall="fault",
                messages=(
                    "Session shut down; restart the operator service to clear "
                    f"non-recoverable fault {self._active_fault.code}",
                ),
            )
            self._accept(
                request,
                "shutdown",
                details={
                    "restart_required": True,
                    "retained_fault": self._active_fault.code,
                },
            )
            return
        self._active_fault = None
        self._state = SessionState.IDLE
        self._health = HealthSnapshot(overall="offline", messages=("Session shut down",))
        self._accept(request, "shutdown")

    def _verify_stop_before_shutdown_locked(self, *, actor: str) -> None:
        """Require fresh runtime proof after any failed stop attempt."""

        stop_details = self._safe_stop_locked()
        if stop_details is None:
            self._record_event_best_effort(
                "runtime", "safe_stop", "accepted", actor=actor
            )
            return
        self._record_event_best_effort(
            "runtime", "safe_stop", "rejected", actor=actor, details=stop_details
        )
        raise InvalidAction(
            "Runtime shutdown is blocked because zero command authority was not verified",
            code="shutdown_stop_unverified",
        )

    def _safe_stop_locked(self) -> dict[str, Any] | None:
        """Ask the runtime to remove authority and verify that it did.

        The single safe-stop path for disengage, fault latching and shutdown.
        Returns ``None`` when the runtime verified zero command authority, or
        audit details describing why it did not.  Never raises for adapter
        failures, and always leaves the local authority bit false.
        """

        self._stop_verification_required = True
        failure: dict[str, Any] | None = None
        try:
            stop_changes = self._call_runtime("safe_stop", self._health)
            if not isinstance(stop_changes, Mapping):
                failure = {"reason": "Runtime returned invalid health data"}
            elif stop_changes.get("command_authority") is not False:
                failure = {"reason": "Runtime did not verify zero command authority"}
            else:
                self._apply_runtime_health(stop_changes, heartbeat=False)
                self._stop_verification_required = False
        except Exception as exc:
            failure = {"reason": str(exc), "error_type": type(exc).__name__}
            if isinstance(exc, RuntimeAdapterUnavailable):
                # The request never reached the adapter.
                failure["delivered"] = False
            elif isinstance(exc, RuntimeAdapterTimeout):
                failure["timed_out"] = True
        finally:
            self._health = replace(
                self._health, command_authority=False, updated_at=time.time()
            )
        return failure

    def _restart_required(self) -> bool:
        return (
            self._state is SessionState.FAULT_LATCHED
            and self._active_fault is not None
            and not self._active_fault.recoverable
        )

    def _require_hardware_capability(self) -> None:
        self._expire_capability_locked()
        if not self._capability_lock.whole_body_unlocked:
            raise CapabilityLocked(self._capability_lock.reason)

    def _expire_capability_locked(self) -> None:
        if self._demo or not self._capability_lock.whole_body_unlocked:
            return
        expires_at = self._capability_lock.expires_at
        try:
            expiry = datetime.fromisoformat(str(expires_at).replace("Z", "+00:00"))
            if expiry.tzinfo is None:
                expiry = expiry.replace(tzinfo=timezone.utc)
            expired = expiry.timestamp() <= time.time()
        except (OverflowError, TypeError, ValueError):
            expired = True
        if not expired:
            return
        self._capability_lock = replace(
            self._capability_lock,
            whole_body_unlocked=False,
            reason="Capability manifest has expired",
        )
        if self._state is SessionState.TRACKING:
            self._latch_fault_locked(
                FaultEvent(
                    code="capability_expired",
                    reason="Capability manifest expired while command authority was active",
                    source="controller",
                    recoverable=True,
                ),
                actor="controller",
            )

    def _apply_runtime_health(
        self, changes: Mapping[str, Any], *, heartbeat: bool = True
    ) -> None:
        """Apply runtime-reported health.

        ``heartbeat`` marks the report as fresh evidence of stream and robot
        state.  A ``safe_stop`` acknowledgement is not: it must not make stale
        health look current for a later start-tracking or rearm.
        """
        if not isinstance(changes, Mapping):
            raise InvalidAction("Runtime returned invalid health data", code="runtime_error")
        allowed = set(HealthSnapshot.__dataclass_fields__) - {"updated_at"}
        unknown = sorted(set(changes) - allowed)
        if unknown:
            raise InvalidAction(
                f"Runtime returned unknown health fields: {', '.join(unknown)}",
                code="runtime_error",
            )
        normalized = dict(changes)
        if "messages" in normalized:
            normalized["messages"] = tuple(normalized["messages"])
        normalized = self._clamp_command_authority(normalized, actor="runtime")
        self._health = replace(self._health, **normalized, updated_at=time.time())
        if heartbeat:
            self._health_refreshed_at = float(self._clock())

    def _clamp_command_authority(
        self, changes: Mapping[str, Any], *, actor: str
    ) -> dict[str, Any]:
        """Never let reported health assert authority outside TRACKING.

        Only :meth:`_start_tracking` grants authority.  A runtime or supervisor
        claiming ``command_authority=True`` in any other state is recorded and
        stored as ``False``.
        """

        result = dict(changes)
        if (
            "command_authority" in result
            and result["command_authority"] is not False
            and self._state is not SessionState.TRACKING
        ):
            claimed = result["command_authority"]
            result["command_authority"] = False
            self._record_event_best_effort(
                "health",
                "command_authority_clamped",
                "rejected",
                actor=actor,
                details={
                    "reason": "command_authority may only be true while tracking",
                    "claimed": claimed if isinstance(claimed, bool) else repr(claimed),
                },
            )
        return result

    def _health_stale_error(self) -> str | None:
        age_s = float(self._clock()) - self._health_refreshed_at
        # A clock that runs backwards (or returns NaN) is not evidence of
        # fresh health; the negated range check also rejects NaN.
        if not 0.0 <= age_s <= self._health_stale_timeout_s:
            return (
                f"Runtime health is stale ({age_s * 1000.0:.0f} ms since the last "
                f"update; limit {self._health_stale_timeout_s * 1000.0:.0f} ms)"
            )
        return None

    def _enforce_tracking_health_locked(self) -> None:
        """Latch a fault if tracking health is stale or no longer acceptable."""

        if self._state is not SessionState.TRACKING:
            return
        stale = self._health_stale_error()
        if stale is not None:
            self._latch_fault_locked(
                FaultEvent(
                    code="health_stale",
                    reason=stale,
                    source="controller",
                    recoverable=True,
                ),
                actor="controller",
            )
            return
        critical = self._tracking_health_error()
        if critical is not None:
            self._latch_fault_locked(
                FaultEvent(code="tracking_health_lost", reason=critical, source="runtime")
            )

    def _mocap_stream_error(self) -> tuple[str, str] | None:
        """Return ``(reason, code)`` for the first failed stream acceptance gate."""

        health = self._health
        if not health.mocap_online:
            return "Mocap stream is offline", "mocap_not_ready"
        if (
            not getattr(self._runtime, "uses_recorded_source", False)
            and health.mocap_rate_hz < MOCAP_MIN_RATE_HZ
        ):
            return "Mocap rate is below 50 Hz", "mocap_acceptance_failed"
        if health.packet_loss_percent > MOCAP_MAX_PACKET_LOSS_PERCENT:
            return "Mocap loss exceeds 1%", "mocap_acceptance_failed"
        if health.source_age_ms is None or health.source_age_ms > MOCAP_MAX_SOURCE_AGE_MS:
            return "Mocap source is stale", "mocap_acceptance_failed"
        return None

    def _notify_runtime_tracking(self, enabled: bool, *, required: bool) -> None:
        """Notify adapters which track controller authority transitions."""

        if not callable(getattr(self._runtime, "set_tracking", None)):
            return
        try:
            self._call_runtime("set_tracking", enabled)
        except _ADAPTER_BLOCKED_ERRORS:
            if required:
                raise
        except Exception as exc:
            if required:
                raise InvalidAction(
                    "Runtime rejected the tracking transition: "
                    f"{type(exc).__name__}: {exc}",
                    code="runtime_error",
                ) from exc

    def _tracking_health_error(self) -> str | None:
        health = self._health
        if not health.mocap_accepted:
            return "Accepted mocap stream is required"
        # The acceptance gates are not a one-time check: a stream that degrades
        # after acceptance must withdraw authority.
        stream_error = self._mocap_stream_error()
        if stream_error is not None:
            return stream_error[0]
        stale = self._health_stale_error()
        if stale is not None:
            return stale
        checks = (
            (health.simulation_ready, "Simulation validation is required"),
            (health.robot_connected, "Robot connection is required"),
            (health.robot_state_fresh, "Fresh robot state is required"),
            (health.controller_ready, "Controller is not ready"),
            (health.preflight_confirmed, "Preflight confirmation is required"),
        )
        for passed, reason in checks:
            if not passed:
                return reason
        return None

    def _latch_fault_locked(self, fault: FaultEvent, *, actor: str = "runtime") -> None:
        if self._state is SessionState.FAULT_LATCHED:
            # Authority is already removed.  A non-recoverable fault must
            # still replace a recoverable one so it cannot be rearmed away.
            if not fault.recoverable and (
                self._active_fault is None or self._active_fault.recoverable
            ):
                self._active_fault = fault
                self._fault_condition_cleared = False
                self._health = replace(
                    self._health,
                    command_authority=False,
                    overall="fault",
                    updated_at=time.time(),
                    messages=(fault.reason,),
                )
                self._record_event_best_effort(
                    "fault", "escalate", "accepted", actor=actor, details=fault.to_dict()
                )
            return
        self._state = SessionState.DISENGAGING
        self._health = replace(
            self._health,
            command_authority=False,
            overall="fault",
            updated_at=time.time(),
            messages=(fault.reason,),
        )
        # A newly latched fault always asks the runtime to remove authority.
        # Adapter errors are evidence to retain, not a reason to abandon the
        # fail-closed transition.  The local authority bit is forced false
        # after the call even if a faulty adapter reports otherwise.
        self._stop_verification_required = True
        try:
            self._notify_runtime_tracking(False, required=False)
            stop_details = self._safe_stop_locked()
        finally:
            self._health = replace(
                self._health,
                command_authority=False,
                overall="fault",
                updated_at=time.time(),
                messages=(fault.reason,),
            )
        stop_outcome = "accepted" if stop_details is None else "rejected"
        self._record_event_best_effort(
            "transition",
            "disengaging",
            "accepted",
            actor=actor,
            details={"fault": fault.to_dict()},
        )
        self._record_event_best_effort(
            "runtime", "safe_stop", stop_outcome, actor=actor, details=stop_details
        )
        self._active_fault = fault
        self._fault_condition_cleared = False
        self._state = SessionState.FAULT_LATCHED
        self._record_event_best_effort(
            "fault", "latch", "accepted", actor=actor, details=fault.to_dict()
        )

    def _accept(
        self,
        request: ControlRequest,
        action: str,
        *,
        details: Mapping[str, Any] | None = None,
        audit_best_effort: bool = False,
    ) -> None:
        payload = {"request": request.to_dict()}
        if request.claimed_operator is not None:
            payload["claimed_operator"] = request.claimed_operator
        if details:
            payload.update(details)
        recorder = self._record_event_best_effort if audit_best_effort else self._record_event
        recorder(
            "control", action, "accepted", actor=request.operator, details=payload
        )

    def _reject(
        self,
        action: str,
        actor: str,
        reason: str,
        *,
        code: str,
        claimed_operator: str | None = None,
    ) -> None:
        details: dict[str, Any] = {"reason": reason, "code": code}
        if claimed_operator is not None:
            details["claimed_operator"] = claimed_operator
        self._record_event(
            "control",
            action,
            "rejected",
            actor=actor,
            details=details,
        )
        if code == "configuration_locked":
            # A state conflict, not a malformed configuration.
            raise InvalidConfiguration(reason, code=code, status_code=409)
        raise InvalidAction(reason, code=code)

    def _record_event(
        self,
        category: str,
        action: str,
        outcome: str,
        *,
        actor: str,
        details: Mapping[str, Any] | None = None,
        durable: bool = True,
    ) -> None:
        # Strict records (accepted control actions, rejections, configuration)
        # are fsynced: a rearm or start-tracking is only valid with a durable
        # record of who did it.  Best-effort safety records are flushed only,
        # so a slow disk can never delay withdrawing authority.
        audit_record = self.audit.append(
            category=category,
            action=action,
            outcome=outcome,
            state=self._state.value,
            actor=actor,
            details=details,
            durable=durable,
        )
        self._event_sequence += 1
        event = {
            "schema_version": SCHEMA_VERSION,
            "instance_id": self._instance_id,
            "sequence": self._event_sequence,
            # The durable AuditLog numbering is independent of the in-memory
            # event counter and survives restarts; never conflate the two.
            "audit_sequence": audit_record["sequence"],
            "timestamp": audit_record["timestamp"],
            "type": category,
            "action": action,
            "outcome": outcome,
            "state": self._state.value,
            "actor": audit_record["actor"],
            "details": audit_record["details"],
        }
        self._events.append(event)
        if len(self._events) > self._event_limit:
            del self._events[: len(self._events) - self._event_limit]

    def _record_event_best_effort(
        self,
        category: str,
        action: str,
        outcome: str,
        *,
        actor: str,
        details: Mapping[str, Any] | None = None,
    ) -> None:
        """Retain an in-memory event if durable audit I/O is unavailable.

        Safety actions must never be delayed or skipped because the audit disk
        is full. The transient event makes the loss visible to connected
        operators without pretending it was persisted.
        """
        try:
            self._record_event(
                category, action, outcome, actor=actor, details=details, durable=False
            )
            return
        except OSError as exc:
            safe_details = dict(redact(details or {}))
            safe_details["audit_persisted"] = False
            safe_details["audit_error_type"] = type(exc).__name__
        self._event_sequence += 1
        self._events.append({
            "schema_version": SCHEMA_VERSION,
            "instance_id": self._instance_id,
            "sequence": self._event_sequence,
            "audit_sequence": None,
            "timestamp": time.time(),
            "type": category,
            "action": action,
            "outcome": outcome,
            "state": self._state.value,
            "actor": str(actor),
            "details": safe_details,
        })
        if len(self._events) > self._event_limit:
            del self._events[: len(self._events) - self._event_limit]

    def _control_availability(self) -> dict[str, dict[str, Any]]:
        state = self._state
        expected = {
            "check": SessionState.IDLE,
            "mocap-accept": SessionState.CHECKING,
            "start-simulation": SessionState.MOCAP_ACCEPTED,
            "connect-robot": SessionState.SIM_VALIDATED,
            "confirm-preflight": SessionState.ROBOT_CONNECTED,
            "arm": SessionState.PREFLIGHT_PASSED,
            "start-tracking": SessionState.ARMED_DISABLED,
            "rearm": SessionState.FAULT_LATCHED,
        }
        controls: dict[str, dict[str, Any]] = {}
        restart_required = self._restart_required()
        for action, required in expected.items():
            if restart_required:
                controls[action] = {"enabled": False, "reason": _RESTART_REQUIRED_REASON}
                continue
            enabled = state is required
            reason = None if enabled else f"Requires {required.value}"
            if (
                action in {"connect-robot", "arm", "start-tracking"}
                and not self._demo
                and not self._virtual_robot
            ):
                if not self._capability_lock.whole_body_unlocked:
                    enabled = False
                    reason = self._capability_lock.reason
            if action == "rearm" and self._stop_verification_required:
                enabled = False
                reason = "Requires verified zero command authority"
            if action == "start-tracking" and enabled and self._health.overall == "fault":
                enabled = False
                reason = "Runtime health reports a fault"
            controls[action] = {"enabled": enabled, "reason": reason}
        controls["disengage"] = {"enabled": True, "reason": None}
        controls["shutdown"] = {"enabled": True, "reason": None}
        controls["edit-config"] = {
            "enabled": state not in _CONFIG_LOCKED_STATES,
            "reason": (
                None
                if state not in _CONFIG_LOCKED_STATES
                else "Configuration is locked in the current state"
            ),
        }
        return controls

    def _persist_config(self, config: Mapping[str, Any]) -> None:
        if self.config_path is None:
            return
        self.config_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.config_path.with_name(f".{self.config_path.name}.tmp")
        with temporary.open("w", encoding="utf-8") as handle:
            json.dump(config, handle, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, self.config_path)
