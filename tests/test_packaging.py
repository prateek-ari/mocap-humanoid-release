"""Packaging: the wheel must carry everything a non-editable install needs (T4-1)."""

from __future__ import annotations

from hashlib import sha256
from importlib.util import find_spec
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
import zipfile

from mocap_g1.assets import OFFICIAL_URDF_NAME, official_urdf_path

REPO_ROOT = Path(__file__).resolve().parents[1]
EVAL_URDF = REPO_ROOT / "eval/assets" / OFFICIAL_URDF_NAME
PACKAGED_URDF = REPO_ROOT / "mocap_g1/assets" / OFFICIAL_URDF_NAME
_BUILD_BACKEND_AVAILABLE = find_spec("setuptools") is not None and find_spec("pip") is not None


def _sha256(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


class VendoredUrdfTests(unittest.TestCase):
    def test_packaged_urdf_matches_eval_copy(self):
        # eval/ and tools/ keep using eval/assets; the package ships a copy.
        self.assertEqual(_sha256(PACKAGED_URDF), _sha256(EVAL_URDF))

    def test_runtime_defaults_resolve_to_the_packaged_urdf(self):
        from mocap_g1.balance_screen import DEFAULT_EFFORT_URDF
        from mocap_g1.operator.simulation import DEFAULT_LIMITS_URDF

        packaged = official_urdf_path()
        self.assertTrue(packaged.is_file())
        self.assertEqual(packaged.resolve(), PACKAGED_URDF.resolve())
        self.assertEqual(Path(DEFAULT_LIMITS_URDF), packaged)
        self.assertEqual(Path(DEFAULT_EFFORT_URDF), packaged)


@unittest.skipUnless(_BUILD_BACKEND_AVAILABLE, "setuptools/pip build backend is unavailable")
class WheelInstallTests(unittest.TestCase):
    """Build a wheel offline, install it outside the checkout, and run it."""

    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix="mocap-g1-wheel-")
        root = Path(cls.temporary.name)
        # Build from a copy so the build never writes build/ or *.egg-info
        # into the working tree.
        source = root / "source"
        source.mkdir()
        shutil.copy2(REPO_ROOT / "pyproject.toml", source / "pyproject.toml")
        shutil.copytree(
            REPO_ROOT / "mocap_g1",
            source / "mocap_g1",
            ignore=shutil.ignore_patterns("__pycache__", "*.pyc"),
        )
        dist = root / "dist"
        build = subprocess.run(
            [
                sys.executable, "-m", "pip", "wheel", "--no-deps",
                "--no-build-isolation", "--no-index", "--quiet",
                "--wheel-dir", str(dist), str(source),
            ],
            capture_output=True,
            text=True,
            timeout=180,
            check=False,
        )
        if build.returncode != 0:
            cls.temporary.cleanup()
            raise AssertionError(f"wheel build failed:\n{build.stdout}\n{build.stderr}")
        wheels = sorted(dist.glob("mocap_g1-*.whl"))
        if len(wheels) != 1:
            cls.temporary.cleanup()
            raise AssertionError(f"expected one wheel, found {wheels}")
        cls.wheel = wheels[0]
        cls.target = root / "site"
        install = subprocess.run(
            [
                sys.executable, "-m", "pip", "install", "--no-deps", "--no-index",
                "--quiet", "--target", str(cls.target), str(cls.wheel),
            ],
            capture_output=True,
            text=True,
            timeout=180,
            check=False,
        )
        if install.returncode != 0:
            cls.temporary.cleanup()
            raise AssertionError(f"wheel install failed:\n{install.stdout}\n{install.stderr}")
        cls.workdir = root / "elsewhere"
        cls.workdir.mkdir()

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def run_installed(self, *args: str) -> subprocess.CompletedProcess:
        # Dependencies come from the test interpreter's environment; mocap_g1
        # itself must come from the installed wheel, never the checkout.
        env = {
            key: value for key, value in os.environ.items() if key != "PYTHONPATH"
        }
        env["PYTHONPATH"] = str(self.target)
        env["PYTHONDONTWRITEBYTECODE"] = "1"
        return subprocess.run(
            [sys.executable, *args],
            cwd=self.workdir,
            env=env,
            capture_output=True,
            text=True,
            timeout=60,
            check=False,
        )

    def test_wheel_contains_urdf_static_bundle_and_entry_point(self):
        with zipfile.ZipFile(self.wheel) as archive:
            names = set(archive.namelist())
            self.assertIn(f"mocap_g1/assets/{OFFICIAL_URDF_NAME}", names)
            self.assertIn("mocap_g1/operator/static/index.html", names)
            packaged = archive.read(f"mocap_g1/assets/{OFFICIAL_URDF_NAME}")
            entry_points = next(
                archive.read(name).decode("utf-8")
                for name in names
                if name.endswith(".dist-info/entry_points.txt")
            )
        self.assertEqual(sha256(packaged).hexdigest(), _sha256(EVAL_URDF))
        self.assertIn("mocap-g1-console = mocap_g1.operator.cli:main", entry_points)
        self.assertFalse(any(name.startswith(("eval/", "tests/", "tools/")) for name in names))

    def test_installed_package_resolves_urdf_and_version_without_checkout(self):
        script = "\n".join(
            (
                "from pathlib import Path",
                "import mocap_g1",
                "from mocap_g1.operator._version import service_version",
                "from mocap_g1.operator.simulation import DEFAULT_LIMITS_URDF",
                "from mocap_g1.schema import load_official_joint_limits",
                "print(Path(mocap_g1.__file__).resolve())",
                "print(Path(DEFAULT_LIMITS_URDF).resolve())",
                "print(len(load_official_joint_limits(DEFAULT_LIMITS_URDF)))",
                "print(service_version())",
            )
        )
        result = self.run_installed("-I", "-c", f"import sys; sys.path.insert(0, {str(self.target)!r})\n{script}")
        self.assertEqual(result.returncode, 0, result.stderr)
        package_file, urdf, joint_count, version = result.stdout.split()
        target = self.target.resolve()
        self.assertTrue(Path(package_file).is_relative_to(target), package_file)
        self.assertTrue(Path(urdf).is_relative_to(target), urdf)
        self.assertGreater(int(joint_count), 0)
        from mocap_g1.operator._version import _source_tree_version

        self.assertEqual(version, _source_tree_version())

    def test_installed_console_script_runs_help(self):
        console = self.target / "bin" / "mocap-g1-console"
        self.assertTrue(console.is_file(), sorted(self.target.iterdir()))
        result = self.run_installed(str(console), "--help")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--simulation", result.stdout)


if __name__ == "__main__":
    unittest.main()
