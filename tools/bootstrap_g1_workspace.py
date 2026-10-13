#!/usr/bin/env python3
"""Bootstrap or verify a pinned, patched G1 GMR runtime workspace."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
GMR_REPOSITORY = "https://github.com/YanjieZe/GMR.git"
GMR_COMMIT = "bb1bbe40774794fceb2a7c579a3464a28e68c844"
GMR_XSENS_DEPENDENCIES = ("PyQt6>=6.7,<7", "matplotlib>=3.8,<4")


class BootstrapError(RuntimeError):
    pass


def _run(command: list[str], *, cwd: Path | None = None, capture: bool = False) -> str:
    try:
        result = subprocess.run(
            command,
            cwd=cwd,
            check=False,
            text=True,
            capture_output=capture,
            timeout=300,
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
    if path == REPO_ROOT or REPO_ROOT in path.parents:
        raise BootstrapError("runtime workspace must be outside this source checkout")
    return path


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
    _run(["git", "apply", "--check", str(patch)], cwd=gmr)
    _run(["git", "apply", str(patch)], cwd=gmr)


def bootstrap(workspace: Path, *, install: bool) -> None:
    workspace.mkdir(parents=True, exist_ok=True)
    gmr = workspace / "GMR"
    if not gmr.exists():
        gmr.mkdir()
        _run(["git", "init", "-q"], cwd=gmr)
        _run(["git", "remote", "add", "origin", GMR_REPOSITORY], cwd=gmr)
        _run(["git", "fetch", "--depth", "1", "origin", GMR_COMMIT], cwd=gmr)
        _run(["git", "checkout", "-q", "FETCH_HEAD"], cwd=gmr)
    elif not (gmr / ".git").is_dir():
        raise BootstrapError(f"{gmr} exists but is not a Git checkout")
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
        venv_python = venv / "bin/python"
        venv_ready = venv_python.is_file() and subprocess.run(
            [str(venv_python), "-m", "pip", "--version"],
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
        python = str(venv_python)
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
    python = workspace / "venv/bin/python"
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
