"""Fail-closed operator-console backend.

The core state machine intentionally depends only on the Python standard
library.  Import :func:`create_app` explicitly from ``mocap_g1.operator.app``
when the optional web dependencies are installed.
"""

from .audit import AuditLog
from .capabilities import CapabilityManifestVerifier, sign_manifest
from .core import InvalidAction, SessionController
from .models import (
    CapabilityLock,
    ControlRequest,
    FaultEvent,
    HealthSnapshot,
    SessionState,
    Topology,
)
from .runtime import DemoRuntime, PassiveRuntime, ReadOnlySystemProbe, RuntimeAdapter

# The recorded-simulation runtime needs NumPy.  Import it lazily so the
# standard-library core and the console launcher's ``--help`` stay usable in
# minimal environments.
_LAZY_SIMULATION_EXPORTS = frozenset(
    {"RecordedSimulationConfigurationError", "RecordedSimulationRuntime"}
)


def __getattr__(name: str):
    if name in _LAZY_SIMULATION_EXPORTS:
        from . import simulation

        return getattr(simulation, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

__all__ = [
    "AuditLog",
    "CapabilityLock",
    "CapabilityManifestVerifier",
    "ControlRequest",
    "DemoRuntime",
    "FaultEvent",
    "HealthSnapshot",
    "InvalidAction",
    "PassiveRuntime",
    "ReadOnlySystemProbe",
    "RecordedSimulationConfigurationError",
    "RecordedSimulationRuntime",
    "RuntimeAdapter",
    "SessionController",
    "SessionState",
    "Topology",
    "sign_manifest",
]
