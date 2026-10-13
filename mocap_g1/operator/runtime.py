"""Runtime adapter seam; v1 includes no physical-robot implementation."""

from __future__ import annotations

from importlib.util import find_spec
from pathlib import Path
import platform
import shutil
import sys
from typing import Any, Mapping, Protocol

from .models import HealthSnapshot


class RuntimeAdapter(Protocol):
    """Narrow supervisor contract used by :class:`SessionController`.

    Methods return health-field changes only.  The controller calls every
    method on one dedicated adapter thread with a per-call deadline
    (``DEFAULT_ADAPTER_TIMEOUTS_S``); calls never overlap.  A method must not
    call back into the controller synchronously (it would wait on the lock the
    caller holds); report asynchronous faults from the adapter's own threads.
    A call that overruns its deadline is treated as a fault, and later calls
    fail fast until it returns.  A real implementation must keep
    DDS and real-time work outside the web process, verify zero authority in
    ``safe_stop``, and feed asynchronous watchdog faults back to the controller.

    While tracking, fresh health must reach the controller more often than its
    ``health_stale_timeout_s`` (default 250 ms), either as a non-empty ``tick``
    result or through ``SessionController.update_health``.  An empty mapping is
    not a heartbeat, and a ``safe_stop`` result never counts as one.  Reported
    ``command_authority=True`` is ignored (and audited) outside TRACKING.
    """

    is_simulated: bool
    allows_hardware_bypass: bool
    uses_virtual_robot: bool

    def check(self, health: HealthSnapshot, config: Mapping[str, Any]) -> Mapping[str, Any]: ...

    def start_simulation(
        self, health: HealthSnapshot, config: Mapping[str, Any]
    ) -> Mapping[str, Any]: ...

    def connect_robot(
        self, health: HealthSnapshot, config: Mapping[str, Any]
    ) -> Mapping[str, Any]: ...

    def safe_stop(self, health: HealthSnapshot) -> Mapping[str, Any]: ...

    def shutdown(self, health: HealthSnapshot) -> Mapping[str, Any]: ...

    def tick(self, health: HealthSnapshot) -> Mapping[str, Any]: ...

    def set_tracking(self, enabled: bool) -> None: ...

    def simulation_snapshot(self) -> dict[str, Any] | None: ...

    def recover(self, health: HealthSnapshot) -> Mapping[str, Any]: ...


class PassiveRuntime:
    """Fail-closed placeholder for a future external hardware supervisor."""

    is_simulated = False
    allows_hardware_bypass = False
    uses_virtual_robot = False

    def check(self, health: HealthSnapshot, config: Mapping[str, Any]) -> Mapping[str, Any]:
        return {
            "overall": "checking",
            "command_authority": False,
            "messages": ("Waiting for external supervisor health",),
        }

    def start_simulation(
        self, health: HealthSnapshot, config: Mapping[str, Any]
    ) -> Mapping[str, Any]:
        return {}

    def connect_robot(
        self, health: HealthSnapshot, config: Mapping[str, Any]
    ) -> Mapping[str, Any]:
        return {}

    def safe_stop(self, health: HealthSnapshot) -> Mapping[str, Any]:
        return {
            "command_authority": False,
            "overall": "ready",
            "messages": ("Command authority disabled",),
        }

    def shutdown(self, health: HealthSnapshot) -> Mapping[str, Any]:
        return {}

    def tick(self, health: HealthSnapshot) -> Mapping[str, Any]:
        return {}

    def set_tracking(self, enabled: bool) -> None:
        return None

    def simulation_snapshot(self) -> dict[str, Any] | None:
        return None

    def recover(self, health: HealthSnapshot) -> Mapping[str, Any]:
        return {}


class DemoRuntime(PassiveRuntime):
    """Deterministic fake which exercises the complete UI without hardware."""

    is_simulated = True
    allows_hardware_bypass = True
    uses_virtual_robot = True

    def check(self, health: HealthSnapshot, config: Mapping[str, Any]) -> Mapping[str, Any]:
        return {
            "mocap_online": True,
            "mocap_rate_hz": 60.0,
            "packet_loss_percent": 0.1,
            "source_age_ms": 4.0,
            "controller_ready": True,
            "overall": "checking",
            "messages": ("Fake runtime checks passed", "Waiting for mocap acceptance"),
        }

    def start_simulation(
        self, health: HealthSnapshot, config: Mapping[str, Any]
    ) -> Mapping[str, Any]:
        return {
            "simulation_ready": True,
            "messages": ("Simulation validation passed",),
        }

    def connect_robot(
        self, health: HealthSnapshot, config: Mapping[str, Any]
    ) -> Mapping[str, Any]:
        return {
            "robot_connected": True,
            "robot_state_fresh": True,
            "controller_ready": True,
            "messages": ("Fake robot connected",),
        }

    def tick(self, health: HealthSnapshot) -> Mapping[str, Any]:
        # Once checks have brought the synthetic stream online, report a fresh
        # sample every watchdog tick like a live supervisor would; the
        # controller latches ``health_stale`` if heartbeats stop.
        if not health.mocap_online:
            return {}
        return {"source_age_ms": 4.0}


class ReadOnlySystemProbe:
    """Collect local availability facts without starting services or opening DDS."""

    def __init__(self, repo_root: str | Path | None = None) -> None:
        self.repo_root = Path(repo_root) if repo_root is not None else Path.cwd()

    def snapshot(self) -> dict[str, Any]:
        # No absolute paths: a diagnostics bundle is shared outside the
        # workstation, and interpreter/checkout paths reveal user names and
        # directory layout (qualification summaries are scrubbed the same way).
        return {
            "python": {
                "version": platform.python_version(),
                "implementation": platform.python_implementation(),
                "virtual_environment": sys.prefix != sys.base_prefix,
            },
            "platform": platform.platform(),
            "repo": {
                "mocap_package_present": (self.repo_root / "mocap_g1").is_dir(),
                # False for a wheel install, where the root is site-packages.
                "source_checkout": (self.repo_root / "pyproject.toml").is_file(),
            },
            "commands": {
                name: shutil.which(name) is not None
                for name in ("systemctl", "docker", "nvidia-smi")
            },
            "python_modules": {
                name: find_spec(name) is not None
                for name in ("fastapi", "mujoco", "cyclonedds", "unitree_sdk2py")
            },
        }


__all__ = [
    "DemoRuntime",
    "PassiveRuntime",
    "ReadOnlySystemProbe",
    "RuntimeAdapter",
]
