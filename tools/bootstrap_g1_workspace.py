#!/usr/bin/env python3
"""Bootstrap or verify a pinned, patched G1 GMR runtime workspace."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


REPO_ROOT = Path(__file__).resolve().parents[1]
GMR_REPOSITORY = "https://github.com/YanjieZe/GMR.git"
GMR_COMMIT = "bb1bbe40774794fceb2a7c579a3464a28e68c844"
GMR_XSENS_DEPENDENCIES = ("PyQt6>=6.7,<7", "matplotlib>=3.8,<4")
# Windows installs the whole runtime from this hashed lock (generated from
# tools/windows/runtime-requirements.in), then GMR and this repository with
# --no-deps, so GMR's Linux-oriented setup.py dependencies are never resolved.
WINDOWS_RUNTIME_LOCK = REPO_ROOT / "constraints/windows-runtime.txt"
# Windows wheels, the GMR fetch and the first pip run can take far longer than
# the Linux default on a slow link; never let a large download hit 300 s.
WINDOWS_INSTALL_TIMEOUT = 3600


class BootstrapError(RuntimeError):
    pass


def _is_windows(platform: str | None = None) -> bool:
    return (sys.platform if platform is None else platform) == "win32"


def venv_python(venv: Path, *, platform: str | None = None) -> Path:
    """Return the interpreter of a venv: Scripts/python.exe on Windows, bin/python elsewhere."""

    if _is_windows(platform):
        return venv / "Scripts" / "python.exe"
    return venv / "bin" / "python"


def _run(
    command: list[str],
    *,
    cwd: Path | None = None,
    capture: bool = False,
    timeout: float = 300,
    env: dict[str, str] | None = None,
) -> str:
    try:
        result = subprocess.run(
            command,
            cwd=cwd,
            check=False,
            text=True,
            capture_output=capture,
            timeout=timeout,
            env=env,
        )
    except subprocess.TimeoutExpired as exc:
        raise BootstrapError(f"command timed out: {' '.join(command)}") from exc
    if result.returncode:
        detail = result.stderr.strip() if capture else ""
        raise BootstrapError(
            f"command failed ({result.returncode}): {' '.join(command)}"
            + (f"\n{detail}" if detail else "")
        )
    return result.stdout.strip() if capture else ""


def _safe_workspace(value: str | Path) -> Path:
    path = Path(value).expanduser().resolve()
    if path == Path(path.anchor) or len(path.parts) < 4:
        raise BootstrapError(f"refusing broad workspace path: {path}")
    # normcase is the identity on POSIX and folds case on Windows (C:\Repo == c:\repo).
    repo = os.path.normcase(str(REPO_ROOT))
    if os.path.normcase(str(path)) == repo or any(
        os.path.normcase(str(parent)) == repo for parent in path.parents
    ):
        raise BootstrapError("runtime workspace must be outside this source checkout")
    return path


def normalized_patch_bytes(data: bytes) -> bytes:
    """Return patch bytes with CRLF line endings converted to LF.

    The patches target LF files in GMR (checked out with core.autocrlf=false).
    A Windows clone or ZIP extraction tool may have given the patch file CRLF
    endings, which `git apply` would then fail to match.
    """

    return data.replace(b"\r\n", b"\n")


def _apply_patch(
    gmr: Path,
    patch: Path,
    marker: str,
    *,
    source_path: str = "general_motion_retargeting/motion_retarget.py",
) -> None:
    source = gmr / source_path
    if marker in source.read_text(encoding="utf-8"):
        return
    data = patch.read_bytes()
    normalized = normalized_patch_bytes(data)
    if normalized == data:
        _run(["git", "apply", "--check", str(patch)], cwd=gmr)
        _run(["git", "apply", str(patch)], cwd=gmr)
        return
    with tempfile.TemporaryDirectory(prefix="mocap-patch-") as directory:
        lf_patch = Path(directory) / patch.name
        lf_patch.write_bytes(normalized)
        _run(["git", "apply", "--check", str(lf_patch)], cwd=gmr)
        _run(["git", "apply", str(lf_patch)], cwd=gmr)


def _install_windows_runtime(python: str, gmr: Path) -> None:
    """Install the hashed Windows runtime lock, then GMR and this repo without deps."""

    timeout = WINDOWS_INSTALL_TIMEOUT
    # GMR's setup.py reads its UTF-8 README with the locale encoding (cp1252 on
    # most Windows PCs), which fails on its emoji; UTF-8 mode fixes the build.
    env = {**os.environ, "PYTHONUTF8": "1"}
    # Hashes in the lock switch pip to --require-hashes for this call only; the
    # editable installs below are separate calls (editables cannot be hashed).
    _run([python, "-m", "pip", "install", "--require-hashes", "-r", str(WINDOWS_RUNTIME_LOCK)],
         timeout=timeout, env=env)
    # setuptools comes from the hashed lock, so build without an isolated env
    # (which would download an unpinned setuptools).
    for project in (gmr, REPO_ROOT):
        _run([python, "-m", "pip", "install", "--no-deps", "--no-build-isolation",
              "-e", str(project)], timeout=timeout, env=env)


def _require_windows_python(python: str) -> None:
    """The live loop needs Python >= 3.13 on Windows (finer time.monotonic)."""

    probe = subprocess.run(
        [python, "-c", "import sys; sys.exit(0 if sys.version_info >= (3, 13) else 1)"],
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    if probe.returncode:
        raise BootstrapError(
            f"{python} is older than Python 3.13 (required on Windows); move the "
            "runtime venv folder aside and rerun the bootstrap"
        )


def _git_succeeds(command: list[str], cwd: Path) -> bool:
    return subprocess.run(
        command, cwd=cwd, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    ).returncode == 0


def _fetch_pinned_gmr(gmr: Path, *, add_remote: bool) -> None:
    # Before checkout: keep upstream LF endings (Git for Windows defaults to
    # core.autocrlf=true), so the LF patches below apply. A no-op on Linux.
    _run(["git", "config", "core.autocrlf", "false"], cwd=gmr)
    if add_remote:
        _run(["git", "remote", "add", "origin", GMR_REPOSITORY], cwd=gmr)
    _run(["git", "fetch", "--depth", "1", "origin", GMR_COMMIT], cwd=gmr,
         timeout=WINDOWS_INSTALL_TIMEOUT if _is_windows() else 300)
    _run(["git", "checkout", "-q", "FETCH_HEAD"], cwd=gmr)


def bootstrap(workspace: Path, *, install: bool) -> None:
    workspace.mkdir(parents=True, exist_ok=True)
    gmr = workspace / "GMR"
    windows = _is_windows()
    if not gmr.exists():
        gmr.mkdir()
        _run(["git", "init", "-q"], cwd=gmr)
        _fetch_pinned_gmr(gmr, add_remote=True)
    elif not (gmr / ".git").is_dir():
        raise BootstrapError(f"{gmr} exists but is not a Git checkout")
    elif not _git_succeeds(["git", "rev-parse", "--verify", "-q", "HEAD"], gmr):
        # An earlier run stopped between `git init` and checkout (for example a
        # dropped connection during the fetch): resume instead of failing forever.
        _fetch_pinned_gmr(
            gmr, add_remote=not _git_succeeds(["git", "remote", "get-url", "origin"], gmr)
        )
    revision = _run(["git", "rev-parse", "HEAD"], cwd=gmr, capture=True)
    if revision != GMR_COMMIT:
        raise BootstrapError(f"{gmr} is at {revision}; expected pinned commit {GMR_COMMIT}")

    _apply_patch(
        gmr,
        REPO_ROOT / "retarget_configs/gmr-issue-180.patch",
        "ground_offset = self.ground_offset",
    )
    _apply_patch(
        gmr,
        REPO_ROOT / "retarget_configs/gmr-ground-offset-config.patch",
        'ik_config.get("ground_offset", 0.0)',
    )
    _apply_patch(
        gmr,
        REPO_ROOT / "retarget_configs/gmr-optional-torch.patch",
        "KinematicsModel = None",
        source_path="general_motion_retargeting/__init__.py",
    )
    shutil.copy2(
        REPO_ROOT / "retarget_configs/bvh_lafan1_to_g1.json",
        gmr / "general_motion_retargeting/ik_configs/bvh_lafan1_to_g1.json",
    )

    if install:
        venv = workspace / "venv"
        runtime_python = venv_python(venv)
        if windows and sys.version_info < (3, 13):
            raise BootstrapError("the Windows runtime needs Python >= 3.13; run Install-Windows.cmd")
        venv_ready = runtime_python.is_file() and subprocess.run(
            [str(runtime_python), "-m", "pip", "--version"],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        ).returncode == 0
        if not venv_ready:
            try:
                _run([sys.executable, "-m", "venv", str(venv)], capture=True)
            except BootstrapError as stdlib_error:
                virtualenv = shutil.which("virtualenv")
                if not virtualenv:
                    raise BootstrapError(
                        f"stdlib venv creation failed and virtualenv is unavailable: {stdlib_error}"
                    ) from stdlib_error
                _run([virtualenv, "--python", sys.executable, str(venv)])
        python = str(runtime_python)
        if windows:
            _require_windows_python(python)
            _install_windows_runtime(python, gmr)
            frozen = _run([python, "-m", "pip", "freeze", "--all"], capture=True)
            (workspace / "runtime-lock.txt").write_text(frozen + "\n", encoding="utf-8")
            return
        _run([python, "-m", "pip", "install", "--upgrade", "pip", "setuptools", "wheel"])
        gmr_installed = subprocess.run(
            [python, "-c", "import general_motion_retargeting"],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        ).returncode == 0
        gmr_install = [python, "-m", "pip", "install", "-e", str(gmr)]
        if gmr_installed:
            gmr_install.append("--no-deps")
        _run(gmr_install)
        # GMR documents PyQt6 separately for Xsens BVH calibration, while its
        # Xsens loader imports the editor (and Matplotlib) even in headless mode.
        _run([python, "-m", "pip", "install", *GMR_XSENS_DEPENDENCIES])
        _run([python, "-m", "pip", "install", "-e", str(REPO_ROOT)])
        frozen = _run([python, "-m", "pip", "freeze", "--all"], capture=True)
        (workspace / "runtime-lock.txt").write_text(frozen + "\n", encoding="utf-8")


def check(workspace: Path, *, require_runtime: bool = True) -> dict:
    gmr = workspace / "GMR"
    python = venv_python(workspace / "venv")
    checks: dict[str, dict[str, object]] = {}

    def record(name: str, ok: bool, detail: str) -> None:
        checks[name] = {"ok": ok, "detail": detail}

    record("workspace", workspace.is_dir(), str(workspace))
    record("gmr_checkout", (gmr / ".git").is_dir(), str(gmr))
    if (gmr / ".git").is_dir():
        try:
            revision = _run(["git", "rev-parse", "HEAD"], cwd=gmr, capture=True)
        except BootstrapError as exc:
            revision = str(exc)
        record("gmr_commit", revision == GMR_COMMIT, revision)
    else:
        record("gmr_commit", False, "checkout absent")

    source = gmr / "general_motion_retargeting/motion_retarget.py"
    source_text = source.read_text(encoding="utf-8") if source.is_file() else ""
    record(
        "grounding_patch",
        "ground_offset = self.ground_offset" in source_text
        and 'ik_config.get("ground_offset", 0.0)' in source_text,
        str(source),
    )
    optional_torch_source = gmr / "general_motion_retargeting/__init__.py"
    optional_torch_text = (
        optional_torch_source.read_text(encoding="utf-8")
        if optional_torch_source.is_file()
        else ""
    )
    record(
        "optional_torch_patch",
        "KinematicsModel = None" in optional_torch_text,
        str(optional_torch_source),
    )
    expected_config = REPO_ROOT / "retarget_configs/bvh_lafan1_to_g1.json"
    installed_config = gmr / "general_motion_retargeting/ik_configs/bvh_lafan1_to_g1.json"
    config_ok = installed_config.is_file() and installed_config.read_bytes() == expected_config.read_bytes()
    record("g1_config", config_ok, str(installed_config))
    record(
        "g1_mjcf",
        (gmr / "assets/unitree_g1/g1_mocap_29dof.xml").is_file(),
        str(gmr / "assets/unitree_g1/g1_mocap_29dof.xml"),
    )
    record(
        "xsens_smoke_bvh",
        (gmr / "assets/xsens_bvh_test/251021_04_boxing_120Hz_cm_3DsMax.bvh").is_file(),
        "pinned upstream sample",
    )
    record("venv_python", python.is_file(), str(python))
    if python.is_file():
        probe = subprocess.run(
            [
                str(python),
                "-c",
                "import general_motion_retargeting,numpy,scipy,mujoco,PyQt6,matplotlib; "
                "import general_motion_retargeting.utils.xsens; "
                "print(numpy.__version__, scipy.__version__, mujoco.__version__, "
                "matplotlib.__version__)",
            ],
            check=False,
            text=True,
            capture_output=True,
        )
        record("runtime_imports", probe.returncode == 0, (probe.stdout or probe.stderr).strip())
    else:
        record("runtime_imports", False, "venv absent")
    record("runtime_lock", (workspace / "runtime-lock.txt").is_file(), str(workspace / "runtime-lock.txt"))

    preparation_checks = (
        "workspace", "gmr_checkout", "gmr_commit", "grounding_patch", "optional_torch_patch",
        "g1_config", "g1_mjcf", "xsens_smoke_bvh",
    )
    runtime_checks = ("venv_python", "runtime_imports", "runtime_lock")
    prepared_ok = all(bool(checks[name]["ok"]) for name in preparation_checks)
    runtime_ok = all(bool(checks[name]["ok"]) for name in runtime_checks)
    return {
        "ok": prepared_ok and (runtime_ok if require_runtime else True),
        "mode": "runtime" if require_runtime else "prepared",
        "prepared_ok": prepared_ok,
        "runtime_ok": runtime_ok,
        "gmr_commit_expected": GMR_COMMIT,
        "checks": checks,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("bootstrap", "check"))
    parser.add_argument("--workspace", required=True, help="external runtime workspace")
    parser.add_argument(
        "--prepare-only",
        action="store_true",
        help="clone/patch/configure but do not create the venv or install dependencies",
    )
    args = parser.parse_args()
    if args.command == "check" and args.prepare_only:
        parser.error("--prepare-only is valid only with the bootstrap command")
    try:
        workspace = _safe_workspace(args.workspace)
        if args.command == "bootstrap":
            bootstrap(workspace, install=not args.prepare_only)
        report = check(
            workspace,
            require_runtime=not (args.command == "bootstrap" and args.prepare_only),
        )
    except BootstrapError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
