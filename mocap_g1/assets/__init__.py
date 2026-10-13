"""Package data shipped inside the ``mocap-g1`` distribution.

``g1_29dof_official.urdf`` is a byte-identical copy of
``eval/assets/g1_29dof_official.urdf`` (``tests/test_packaging.py`` asserts the
two hashes match).  Runtime code resolves it through :mod:`importlib.resources`
so a non-editable wheel install does not depend on the source checkout.
"""

from __future__ import annotations

import atexit
from contextlib import ExitStack
from importlib.resources import as_file, files
from pathlib import Path
import threading

OFFICIAL_URDF_NAME = "g1_29dof_official.urdf"

_extracted: dict[str, Path] = {}
_extracted_lock = threading.Lock()
_extracted_stack = ExitStack()
atexit.register(_extracted_stack.close)


def resource_path(name: str) -> Path:
    """Return a real filesystem path for one packaged asset.

    A normal install already stores the file on disk.  If the package is
    imported from an archive, the resource is extracted once and kept for the
    life of the process, because callers keep the path as a module default.
    """

    resource = files(__name__) / name
    if isinstance(resource, Path):
        return resource
    with _extracted_lock:
        cached = _extracted.get(name)
        if cached is None:
            cached = _extracted_stack.enter_context(as_file(resource))
            _extracted[name] = cached
        return cached


def official_urdf_path() -> Path:
    """Path to the vendored official Unitree G1 29-DOF URDF."""

    return resource_path(OFFICIAL_URDF_NAME)


__all__ = ["OFFICIAL_URDF_NAME", "official_urdf_path", "resource_path"]
