"""Stream clock rule 1: the K1 live stream's sender and the bridge must read the same
``CLOCK_MONOTONIC``.  On one PC (the stream is loopback only) the one way to break that is
a time namespace with a monotonic offset."""

from __future__ import annotations

from pathlib import Path

OFFSETS = Path("/proc/self/timens_offsets")


def time_namespace_problem(path: Path = OFFSETS) -> str | None:
    """Why this process's monotonic clock may differ from the host's, or ``None``.

    No file: the kernel has no time namespaces (pass).  A file that cannot be read or
    parsed refuses, as does a nonzero monotonic offset."""
    try:
        text = path.read_text(encoding="ascii")
    except FileNotFoundError:
        return None
    except (OSError, UnicodeError) as exc:
        return f"cannot read {path}: {exc}"
    for line in text.splitlines():
        parts = line.split()
        if parts and parts[0] == "monotonic":
            try:
                seconds, nanoseconds = int(parts[1]), int(parts[2])
            except (IndexError, ValueError):
                break
            if (seconds, nanoseconds) != (0, 0):
                return (f"this process runs in a time namespace with a monotonic offset of "
                        f"{seconds} s {nanoseconds} ns: run the live tool and the bridge on "
                        "the host's clock")
            return None
    return f"cannot parse {path}: no monotonic line"
