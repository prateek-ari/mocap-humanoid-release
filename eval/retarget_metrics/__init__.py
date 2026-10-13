"""Implementation package behind the ``eval/metrics.py`` facade.

``eval/metrics.py`` re-exports every public (and previously reachable
private) name from these modules, so both import styles in use keep working:
``import metrics`` with ``eval/`` on ``sys.path`` (score_motion.py and the
tools/checks scripts) and ``from eval.metrics import ...`` (tests).

  io          GMR pkl loading, MJCF resolution, URDF limit parsing
  robot       RobotModel / JointInfo, limit sources, foot bodies
  contact     geom lowest point, FK pass, contact inference, foot metrics
  kinematics  joint/torque feasibility, jerk, root-translation smoothness
  fidelity    motion fidelity against GMR's scaled human reference
  scoring     score_motion_file
  gate        gate v2
"""

from __future__ import annotations

from pathlib import Path
import sys

# The shared kinematic contact primitives and the pkl quaternion contract
# live in the mocap_g1 package.  It is normally installed (editable); the
# fallback keeps `python eval/score_motion.py` working from a bare checkout.
_REPO_ROOT = Path(__file__).resolve().parents[2]
try:
    import mocap_g1  # noqa: F401
except ImportError:  # pragma: no cover - exercised only without an install
    sys.path.append(str(_REPO_ROOT))
