from __future__ import annotations

from pathlib import Path
import tempfile
import unittest

from mocap_g1.paths import REPO_ROOT, default_g1_mjcf_path, resolve_g1_mjcf

RELATIVE = "GMR/assets/unitree_g1/g1_mocap_29dof.xml"
SIBLING = REPO_ROOT.parent / "mocap_g1_runtime" / RELATIVE


class G1MjcfPathTests(unittest.TestCase):
    def test_workspace_env_wins_even_when_the_file_is_missing(self):
        self.assertEqual(
            default_g1_mjcf_path({"MOCAP_WS": "/nonexistent/ws"}),
            Path("/nonexistent/ws") / RELATIVE,
        )

    def test_unset_or_empty_workspace_uses_the_sibling_checkout(self):
        self.assertEqual(default_g1_mjcf_path({}), SIBLING)
        self.assertEqual(default_g1_mjcf_path({"MOCAP_WS": ""}), SIBLING)

    def test_resolve_prefers_an_existing_workspace_model(self):
        with tempfile.TemporaryDirectory() as tmp:
            model = Path(tmp) / RELATIVE
            model.parent.mkdir(parents=True)
            model.write_text("<mujoco/>", encoding="utf-8")
            self.assertEqual(resolve_g1_mjcf({"MOCAP_WS": tmp}), model)

    def test_resolve_falls_back_to_the_sibling_checkout_or_none(self):
        with tempfile.TemporaryDirectory() as tmp:
            expected = SIBLING if SIBLING.is_file() else None
            self.assertEqual(resolve_g1_mjcf({"MOCAP_WS": tmp}), expected)
            self.assertEqual(resolve_g1_mjcf({}), expected)


if __name__ == "__main__":
    unittest.main()
