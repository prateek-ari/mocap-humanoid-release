"""Location of the external GMR runtime's G1 MJCF.

The model lives outside this repository (the pinned GMR checkout). Resolution
order: ``$MOCAP_WS/GMR/assets/unitree_g1/g1_mocap_29dof.xml`` when ``MOCAP_WS`` is
set, otherwise ``../mocap_g1_runtime/GMR/assets/unitree_g1/g1_mocap_29dof.xml``
next to this repository.
"""

from __future__ import annotations

import os
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

G1_MJCF_RELATIVE = Path("GMR/assets/unitree_g1/g1_mocap_29dof.xml")
SIBLING_RUNTIME_DIR = "mocap_g1_runtime"


def default_g1_mjcf_path(env: dict[str, str] | None = None) -> Path:
    """Path the G1 MJCF is expected at; it need not exist.

    ``$MOCAP_WS`` wins whenever it is set, even if the file is missing, so a
    misconfigured workspace is reported against the path the user chose rather
    than silently falling back to the sibling checkout.
    """
    environ = os.environ if env is None else env
    workspace = environ.get("MOCAP_WS")
    if workspace:
        return Path(workspace) / G1_MJCF_RELATIVE
    return REPO_ROOT.parent / SIBLING_RUNTIME_DIR / G1_MJCF_RELATIVE


def resolve_g1_mjcf(env: dict[str, str] | None = None) -> Path | None:
    """First existing G1 MJCF: ``$MOCAP_WS`` candidate, then the sibling checkout."""
    environ = os.environ if env is None else env
    workspace = environ.get("MOCAP_WS")
    if workspace:
        candidate = Path(workspace) / G1_MJCF_RELATIVE
        if candidate.is_file():
            return candidate
    fallback = REPO_ROOT.parent / SIBLING_RUNTIME_DIR / G1_MJCF_RELATIVE
    return fallback if fallback.is_file() else None
