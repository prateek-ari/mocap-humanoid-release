"""Make GMR's live IK honour its joint limits.

GMR (pinned revision) calls ``mink.solve_ik(config, tasks, dt, solver, damping,
self.ik_limits)``.  In mink 1.2.0 the sixth positional parameter is
``safety_break``, so the limits list is taken as a truthy ``safety_break`` and
``limits`` stays ``None``: any limit GMR or a caller puts in ``ik_limits``
never reaches the solver.  :func:`install_solve_ik_fix` corrects the call for
the live process only (the recorded path keeps upstream behaviour).

Live IK uses joint-range limits only.  GMR's 3*pi rad/s velocity limit stays
off (it never took effect upstream, and on replayed real motion it made the
hands lag: p95 hand error 4 -> 14 cm).  mink's ``CollisionAvoidanceLimit`` is
not used either: in mink 1.2.0 it bounds the step with a velocity-sized value
(``/dt``) and never requires recovery, and in GMR's whole-body solve it made
the robot vibrate while standing (worst joint 4.8 -> 57 rad/s^2).  Arm/hip
clearance comes from :mod:`mocap_g1.live_arm_clearance` instead.
"""

from __future__ import annotations

from typing import Any


class _SolveIkFix:
    """Module proxy whose ``solve_ik`` maps GMR's positional limits correctly."""

    def __init__(self, mink: Any) -> None:
        self._mink = mink

    def __getattr__(self, name: str) -> Any:
        return getattr(self._mink, name)

    def solve_ik(self, configuration, tasks, dt, solver, damping=1e-12,
                 safety_break=False, limits=None, **kwargs):
        if limits is None and not isinstance(safety_break, bool):
            limits, safety_break = list(safety_break), False
        return self._mink.solve_ik(
            configuration, tasks, dt, solver, damping, safety_break, limits, **kwargs
        )


def install_solve_ik_fix(motion_retarget: Any) -> None:
    """Route GMR's ``mink.solve_ik`` call through :class:`_SolveIkFix`."""
    current = motion_retarget.mink
    if not isinstance(current, _SolveIkFix):
        motion_retarget.mink = _SolveIkFix(current)


def configure_live_ik_limits(retargeter: Any, *, mink: Any) -> None:
    """Joint-range limits only (see the module docstring)."""
    retargeter.ik_limits = [mink.ConfigurationLimit(retargeter.model)]
