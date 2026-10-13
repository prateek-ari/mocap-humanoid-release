"""Single source for the service version reported by the operator console."""

from __future__ import annotations

from functools import lru_cache
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path
import re

DISTRIBUTION_NAME = "mocap-g1"
UNKNOWN_VERSION = "0+unknown"
_PROJECT_VERSION = re.compile(r'^version\s*=\s*"([^"]+)"\s*$', re.MULTILINE)


def _source_tree_version() -> str | None:
    """Read ``[project] version`` when running from an unpackaged checkout."""

    pyproject = Path(__file__).resolve().parents[2] / "pyproject.toml"
    try:
        text = pyproject.read_text(encoding="utf-8")
    except OSError:
        return None
    project = text.partition("[project]")[2].partition("\n[")[0]
    match = _PROJECT_VERSION.search(project)
    return match.group(1) if match else None


@lru_cache(maxsize=1)
def service_version() -> str:
    """Installed ``mocap-g1`` version, else the checkout's pyproject version."""

    try:
        return version(DISTRIBUTION_NAME)
    except PackageNotFoundError:
        return _source_tree_version() or UNKNOWN_VERSION


__all__ = ["DISTRIBUTION_NAME", "service_version"]
