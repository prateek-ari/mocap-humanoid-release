#!/usr/bin/env python3
"""Run the operator console launcher from a source checkout.

The implementation lives in :mod:`mocap_g1.operator.cli`, which is also the
``mocap-g1-console`` entry point.  This shim only makes the checkout
importable when the script is executed directly.
"""

from __future__ import annotations

from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mocap_g1.operator.cli import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
