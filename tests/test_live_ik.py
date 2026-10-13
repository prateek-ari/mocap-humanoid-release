"""Tests for ``mocap_g1.live_ik`` with fake ``mink`` and GMR modules."""

from __future__ import annotations

import types
import unittest

from mocap_g1.live_ik import configure_live_ik_limits, install_solve_ik_fix


class FakeMink:
    """Records ``solve_ik`` calls with mink 1.2.0's positional signature."""

    def __init__(self):
        self.calls = []
        self.SOME_CONSTANT = "kept"

    def solve_ik(self, configuration, tasks, dt, solver, damping=1e-12,
                 safety_break=False, limits=None, **kwargs):
        self.calls.append(
            {
                "args": (configuration, tasks, dt, solver, damping),
                "safety_break": safety_break,
                "limits": limits,
                "kwargs": kwargs,
            }
        )
        return "velocity"

    class ConfigurationLimit:
        def __init__(self, model):
            self.model = model


def fake_motion_retarget():
    mink = FakeMink()
    return types.SimpleNamespace(mink=mink), mink


class SolveIkFixTests(unittest.TestCase):
    def test_gmr_positional_limits_list_arrives_as_limits(self):
        module, mink = fake_motion_retarget()
        install_solve_ik_fix(module)
        limits = ["joint_limit"]
        # GMR: mink.solve_ik(config, tasks, dt, solver, damping, self.ik_limits)
        result = module.mink.solve_ik("config", ["task"], 0.01, "daqp", 0.5, limits)
        self.assertEqual(result, "velocity")
        call = mink.calls[-1]
        self.assertEqual(call["args"], ("config", ["task"], 0.01, "daqp", 0.5))
        self.assertEqual(call["limits"], limits)
        self.assertIs(call["safety_break"], False)

    def test_empty_limits_list_is_still_mapped(self):
        module, mink = fake_motion_retarget()
        install_solve_ik_fix(module)
        module.mink.solve_ik("config", [], 0.01, "daqp", 0.5, [])
        self.assertEqual(mink.calls[-1]["limits"], [])
        self.assertIs(mink.calls[-1]["safety_break"], False)

    def test_real_bool_safety_break_and_explicit_limits_are_untouched(self):
        module, mink = fake_motion_retarget()
        install_solve_ik_fix(module)
        module.mink.solve_ik("config", [], 0.01, "daqp", 0.5, True)
        self.assertIs(mink.calls[-1]["safety_break"], True)
        self.assertIsNone(mink.calls[-1]["limits"])
        module.mink.solve_ik("config", [], 0.01, "daqp", safety_break=False, limits=["x"], extra=1)
        self.assertIs(mink.calls[-1]["safety_break"], False)
        self.assertEqual(mink.calls[-1]["limits"], ["x"])
        self.assertEqual(mink.calls[-1]["kwargs"], {"extra": 1})

    def test_installing_twice_does_not_double_wrap_and_proxies_attributes(self):
        module, mink = fake_motion_retarget()
        install_solve_ik_fix(module)
        proxy = module.mink
        install_solve_ik_fix(module)
        self.assertIs(module.mink, proxy)
        self.assertIs(proxy._mink, mink)
        self.assertEqual(proxy.SOME_CONSTANT, "kept")
        self.assertIs(proxy.ConfigurationLimit, FakeMink.ConfigurationLimit)
        proxy.solve_ik("config", [], 0.01, "daqp", 0.5, ["limit"])
        self.assertEqual(len(mink.calls), 1)


class ConfigureLiveIkLimitsTests(unittest.TestCase):
    def test_sets_only_a_configuration_limit_for_the_model(self):
        model = object()
        retargeter = types.SimpleNamespace(model=model, ik_limits=["velocity_limit"])
        configure_live_ik_limits(retargeter, mink=FakeMink())
        self.assertEqual(len(retargeter.ik_limits), 1)
        self.assertIsInstance(retargeter.ik_limits[0], FakeMink.ConfigurationLimit)
        self.assertIs(retargeter.ik_limits[0].model, model)


if __name__ == "__main__":
    unittest.main()
