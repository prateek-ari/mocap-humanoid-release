"""Shared pytest configuration.

Set MOCAP_REQUIRE_ALL_TESTS=1 (CI does) to make the run fail when any test is
skipped, so a missing GMR workspace or optional dependency cannot silently turn
the safety gates into no-ops.

It also owns the one place tests resolve the external G1 MJCF: ``G1_MJCF``
(import it with ``from conftest import G1_MJCF``), the ``requires_g1_mjcf``
unittest decorator, the ``needs_model`` pytest mark and the ``g1_mjcf`` fixture.
The resolution rule itself lives in ``mocap_g1.paths``.
"""

from __future__ import annotations

import os
from pathlib import Path
import unittest

import pytest

from mocap_g1.paths import default_g1_mjcf_path

#: G1 MJCF from the external GMR runtime ($MOCAP_WS, else the sibling checkout).
G1_MJCF: Path = default_g1_mjcf_path()
_MJCF_UNAVAILABLE = "external GMR MuJoCo model is unavailable"

#: ``@unittest.skipUnless`` equivalent for tests that need the G1 MJCF.
requires_g1_mjcf = unittest.skipUnless(G1_MJCF.is_file(), _MJCF_UNAVAILABLE)
#: pytest-style equivalent of ``requires_g1_mjcf``.
needs_model = pytest.mark.skipif(not G1_MJCF.is_file(), reason=_MJCF_UNAVAILABLE)


@pytest.fixture(scope="session")
def g1_mjcf() -> Path:
    """Path to the G1 MJCF; skips the requesting test when it is unavailable."""
    if not G1_MJCF.is_file():
        pytest.skip(_MJCF_UNAVAILABLE)
    return G1_MJCF

_REQUIRE_ALL = os.environ.get("MOCAP_REQUIRE_ALL_TESTS") == "1"
_skipped: list[str] = []


def pytest_runtest_logreport(report: pytest.TestReport) -> None:
    if _REQUIRE_ALL and report.skipped:
        _skipped.append(report.nodeid)


def pytest_sessionfinish(session: pytest.Session, exitstatus: int) -> None:
    if _REQUIRE_ALL and _skipped and exitstatus == pytest.ExitCode.OK:
        session.exitstatus = pytest.ExitCode.TESTS_FAILED


def pytest_terminal_summary(terminalreporter) -> None:
    if _REQUIRE_ALL and _skipped:
        terminalreporter.section("MOCAP_REQUIRE_ALL_TESTS", sep="=", red=True, bold=True)
        terminalreporter.write_line(
            f"{len(_skipped)} test(s) were skipped but MOCAP_REQUIRE_ALL_TESTS=1:"
        )
        for nodeid in dict.fromkeys(_skipped):
            terminalreporter.write_line(f"  {nodeid}")
