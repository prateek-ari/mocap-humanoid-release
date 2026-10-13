"""Exercise the session helper with piped answers and --defaults, in a throwaway HOME."""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "tools/setup/new_session.sh"


class NewSessionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        base = Path(self.temporary.name)
        self.home = base / "home"
        self.home.mkdir()
        self.root = base / "checkout space"
        (self.root / "tools/setup").mkdir(parents=True)
        shutil.copy2(SCRIPT, self.root / "tools/setup/new_session.sh")

    def invoke(self, *args: str, answers: str = "") -> subprocess.CompletedProcess[str]:
        env = {"PATH": os.environ["PATH"], "HOME": str(self.home)}
        return subprocess.run(
            ["/bin/bash", str(self.root / "tools/setup/new_session.sh"), *args],
            cwd="/", env=env, input=answers, text=True, capture_output=True, check=False,
        )

    def load(self) -> dict[str, str]:
        """Source the linked session.env in a clean shell and return its variables."""
        names = ["SESSION_ROUTE", "SESSION_DIR", "ROBOT", "HUMAN_HEIGHT_M", "MOCAP_WS", "IK_CONFIG",
                 "BVH_SCALE", "START_FRAME", "END_FRAME", "FRAME_STEP", "XSENS_SOURCE_IP",
                 "LINUX_XSENS_IP", "XSENS_UDP_PORT"]
        script = 'source "$1"; ' + "; ".join(f'printf "%s\\n" "${name}"' for name in names)
        output = subprocess.run(["/bin/bash", "-c", script, "-", str(self.root / "session.env")],
                                env={"PATH": os.environ["PATH"]}, text=True, capture_output=True,
                                check=True).stdout.splitlines()
        return dict(zip(names, output))

    def test_defaults_write_live_g1_session_with_recorded_values(self) -> None:
        result = self.invoke("--defaults")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.load()["SESSION_ROUTE"], "live")
        result = self.invoke("--defaults", "--route", "recorded")
        self.assertEqual(result.returncode, 0, result.stderr)
        values = self.load()
        self.assertEqual(values["SESSION_ROUTE"], "recorded")
        self.assertEqual(values["ROBOT"], "unitree_g1")
        self.assertEqual(values["HUMAN_HEIGHT_M"], "1.75")
        self.assertEqual(values["MOCAP_WS"], str(self.home / "mocap-runtime"))
        self.assertTrue(values["IK_CONFIG"].endswith("ik_configs/bvh_xsens_to_g1.json"))
        self.assertEqual((values["BVH_SCALE"], values["START_FRAME"], values["END_FRAME"]), ("0.01", "0", "600"))
        self.assertTrue(values["SESSION_DIR"].startswith(str(self.home / "mocap-sessions")))
        self.assertTrue((self.root / "session.env").is_symlink())
        self.assertTrue(Path(values["SESSION_DIR"], "session.env").is_file())

    def test_t1_uses_project_mapping_and_second_session_gets_new_folder(self) -> None:
        workspace = self.home / "runtime with space"
        self.assertEqual(self.invoke("--defaults").returncode, 0)
        first = self.load()["SESSION_DIR"]
        result = self.invoke("--defaults", "--robot", "booster_t1_29dof", "--workspace", str(workspace))
        self.assertEqual(result.returncode, 0, result.stderr)
        values = self.load()
        self.assertNotEqual(values["SESSION_DIR"], first)
        self.assertEqual(values["MOCAP_WS"], str(workspace))
        self.assertEqual(values["IK_CONFIG"],
                         str(self.root.resolve() / "retarget_configs/bvh_xsens_to_booster_t1_29dof.json"))

    def test_piped_live_answers_and_prefill_from_previous_session(self) -> None:
        # route 1 (live), robot 2 (T1), folder, invalid then valid height, invalid then valid IPs.
        answers = "1\n2\n~/mocap-sessions/live take\n3\n1.68\n999.1.1.1\n192.168.1.246\n192.168.1.105\n"
        result = self.invoke(answers=answers)
        self.assertEqual(result.returncode, 0, result.stderr)
        values = self.load()
        self.assertEqual(values["SESSION_DIR"], str(self.home / "mocap-sessions/live take"))
        self.assertEqual((values["ROBOT"], values["HUMAN_HEIGHT_M"]), ("booster_t1_29dof", "1.68"))
        self.assertEqual((values["XSENS_SOURCE_IP"], values["LINUX_XSENS_IP"]), ("192.168.1.246", "192.168.1.105"))
        self.assertEqual(values["XSENS_UDP_PORT"], "5555")
        # The next session offers the previous route, height and addresses as defaults.
        result = self.invoke("--defaults")
        self.assertEqual(result.returncode, 0, result.stderr)
        values = self.load()
        self.assertEqual((values["SESSION_ROUTE"], values["HUMAN_HEIGHT_M"]), ("live", "1.68"))
        self.assertEqual(values["XSENS_SOURCE_IP"], "192.168.1.246")

    def test_refusals_write_nothing(self) -> None:
        for args in (("--robot", "bad", "--defaults"),
                     ("--route", "bad", "--defaults"), ("--unknown",)):
            with self.subTest(args=args):
                self.assertEqual(self.invoke(*args).returncode, 2)
        # Closing input mid-way saves nothing.
        self.assertNotEqual(self.invoke(answers="1\n").returncode, 0)
        self.assertFalse((self.root / "session.env").exists())
        self.assertFalse((self.home / "mocap-sessions").exists())
        # An existing session folder is never replaced silently.
        session = self.home / "taken"
        self.assertEqual(self.invoke(answers=f"1\n1\n{session}\n" + "\n" * 8).returncode, 0)
        result = self.invoke(answers=f"1\n1\n{session}\nn\n")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("kept the existing session", result.stderr)


if __name__ == "__main__":
    unittest.main()
