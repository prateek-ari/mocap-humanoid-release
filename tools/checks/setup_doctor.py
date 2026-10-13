#!/usr/bin/env python3
"""Read-only prerequisites check for core, recorded, and live simulation paths."""

from __future__ import annotations

import argparse
import importlib
import ipaddress
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
IMAGE = "gmr-live:bb1bbe4"
PINNED_GMR = "bb1bbe40774794fceb2a7c579a3464a28e68c844"


def _check(name: str, status: str, detail: str, fix: str = "") -> dict[str, str]:
    return {"name": name, "status": status, "detail": detail, "fix": fix}


def _command(argv: list[str]) -> tuple[int | None, str]:
    """Return only status and output from bounded, read-only subprocesses."""
    try:
        result = subprocess.run(argv, text=True, capture_output=True, timeout=8, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return None, str(exc)
    return result.returncode, (result.stdout or result.stderr).strip()


def _module(name: str) -> object | None:
    try:
        return importlib.import_module(name)
    except Exception:
        return None


def _version_pair(version: str) -> tuple[int, int] | None:
    match = re.match(r"^(\d+)\.(\d+)", version)
    return (int(match.group(1)), int(match.group(2))) if match else None


def _python_checks(*, include_numpy: bool = True) -> list[dict[str, str]]:
    checks = []
    project_text = (ROOT / "pyproject.toml").read_text(encoding="utf-8")
    requirement = re.search(r'^requires-python\s*=\s*">=(\d+)\.(\d+)"', project_text, re.M)
    if not requirement:
        checks.append(_check("python", "unknown", "Cannot read project Python requirement"))
    elif sys.version_info[:2] >= (int(requirement.group(1)), int(requirement.group(2))):
        minimum = f">={requirement.group(1)}.{requirement.group(2)}"
        checks.append(_check("python", "pass", f"Python {sys.version_info.major}.{sys.version_info.minor} satisfies {minimum}"))
    else:
        minimum = f">={requirement.group(1)}.{requirement.group(2)}"
        checks.append(_check("python", "block", f"Python {sys.version_info.major}.{sys.version_info.minor}; requires {minimum}",
                             f"Install Python {requirement.group(1)}.{requirement.group(2)}+ and rerun with that interpreter."))
    if not include_numpy:
        return checks
    numpy = _module("numpy")
    if numpy:
        version = str(getattr(numpy, "__version__", "unknown"))
        pair = _version_pair(version)
        valid = pair is not None and (2, 2) <= pair < (2, 3)
        checks.append(_check("numpy", "pass" if valid else "block",
                             f"NumPy {version} imports" + ("" if valid else "; project requires >=2.2,<2.3"),
                             "python3 -m pip install -e ."))
    else:
        checks.append(_check("numpy", "block", "NumPy cannot be imported",
                             "python3 -m pip install -e ."))
    return checks


def _optional_modules(names: tuple[str, ...], extra: str) -> list[dict[str, str]]:
    checks = []
    for name in names:
        if _module(name):
            checks.append(_check(name, "pass", f"{name} imports"))
        else:
            checks.append(_check(name, "block", f"{name} cannot be imported",
                                 f"python3 -m pip install -e '.[{extra}]'"))
    return checks


def _workspace_checks(workspace: Path | None, robot: str) -> list[dict[str, str]]:
    selected_robot = robot
    if workspace is None:
        return [_check("workspace", "block", "No external GMR workspace selected",
                       "Set MOCAP_WS=/absolute/external/runtime or pass --workspace /absolute/external/runtime.")]
    gmr = workspace / "GMR"
    python = workspace / "venv/bin/python"
    lock = workspace / "runtime-lock.txt"
    workspace_arg = shlex.quote(str(workspace))
    bootstrap = f"python3 tools/bootstrap_g1_workspace.py bootstrap --workspace {workspace_arg}"
    verify = f"python3 tools/bootstrap_g1_workspace.py check --workspace {workspace_arg}"
    checks = []
    checks.append(_check("gmr_checkout", "pass" if (gmr / ".git").exists() else "block",
                         str(gmr), bootstrap))
    if (gmr / ".git").exists():
        status, revision = _command(["git", "-C", str(gmr), "rev-parse", "HEAD"])
        if status is None:
            checks.append(_check("gmr_revision", "unknown", revision, verify))
        elif status == 0 and revision == PINNED_GMR:
            checks.append(_check("gmr_revision", "pass", revision))
        else:
            checks.append(_check("gmr_revision", "block", f"Expected {PINNED_GMR}; found {revision}", verify))
    for name, path in (("venv_python", python), ("runtime_lock", lock)):
        checks.append(_check(name, "pass" if path.is_file() else "block", str(path), bootstrap))
    if python.is_file():
        status, detail = _command([
            str(python), "-c",
            "import json,numpy,scipy,mujoco,general_motion_retargeting; "
            "print(json.dumps({'numpy':numpy.__version__,'scipy':scipy.__version__,"
            "'mujoco':mujoco.__version__}))",
        ])
        try:
            versions = json.loads(detail.splitlines()[-1]) if status == 0 else {}
        except (ValueError, IndexError):
            versions = {}
        pairs = {name: _version_pair(str(versions.get(name, ""))) for name in
                 ("numpy", "scipy", "mujoco")}
        valid = (
            pairs["numpy"] is not None and (2, 2) <= pairs["numpy"] < (2, 3)
            and pairs["scipy"] is not None and (1, 15) <= pairs["scipy"] < (2, 0)
            and pairs["mujoco"] is not None and (3, 11) <= pairs["mujoco"] < (4, 0)
        )
        failed_detail = (f"Versions {versions}; project requires NumPy >=2.2,<2.3, "
                         "SciPy >=1.15,<2, MuJoCo >=3.11,<4" if versions else
                         (detail[:500] if detail else "External runtime imports failed"))
        checks.append(_check("venv_imports", "pass" if valid else "block",
                             f"NumPy {versions['numpy']}, SciPy {versions['scipy']}, "
                             f"MuJoCo {versions['mujoco']}, and GMR import" if valid else
                             failed_detail, bootstrap))
    for model_name, relative in (("g1_model", "unitree_g1/g1_mocap_29dof.xml"),
                            ("booster_model", "booster_t1_29dof/t1_mocap.xml")):
        path = gmr / "assets" / relative
        missing_status = "block" if model_name == "g1_model" or selected_robot == "booster_t1_29dof" else "warn"
        checks.append(_check(model_name, "pass" if path.is_file() else missing_status, str(path), verify))
    return checks


def _udp_check(port: int) -> dict[str, str]:
    if not shutil.which("ss"):
        return _check("udp_port", "unknown", f"Cannot inspect UDP {port}: ss is unavailable",
                      f"ss -H -lun | grep ':{port}'")
    status, output = _command(["ss", "-H", "-lun"])
    if status is None or status != 0:
        return _check("udp_port", "unknown", f"Cannot inspect UDP {port}: {output}",
                      f"ss -H -lun | grep ':{port}'")
    occupied = any(re.search(rf":{port}(?:\s|$)", line) for line in output.splitlines())
    if occupied:
        return _check("udp_port", "block", f"UDP {port} is already bound",
                      f"Stop the existing receiver, then check: ss -H -lun | grep ':{port}'")
    return _check("udp_port", "pass", f"No listener found on UDP {port}")


def _live_checks(env: dict[str, str]) -> list[dict[str, str]]:
    checks = []
    source = env.get("XSENS_SOURCE_IP", "")
    try:
        address = ipaddress.IPv4Address(source)
        valid_source = not (address.is_unspecified or address.is_loopback or address.is_multicast)
    except ipaddress.AddressValueError:
        valid_source = False
    checks.append(_check("source_ipv4", "pass" if valid_source else "block",
                         source if valid_source else "XSENS_SOURCE_IP must be a usable Windows Xsens-facing IPv4",
                         "export XSENS_SOURCE_IP=<Windows Xsens-facing IPv4 from ipconfig>"))
    raw_port = env.get("XSENS_UDP_PORT", "5555")
    try:
        port = int(raw_port)
        valid_port = raw_port.isdecimal() and 1 <= port <= 65535
    except ValueError:
        valid_port = False
    if valid_port:
        checks.append(_udp_check(port))
    else:
        checks.append(_check("udp_port", "block", f"Invalid XSENS_UDP_PORT={raw_port!r}",
                             "export XSENS_UDP_PORT=5555"))

    docker = shutil.which("docker")
    if not docker:
        checks.append(_check("docker_cli", "block", "Docker CLI is unavailable", "Install Docker Engine and add docker to PATH."))
        for name in ("docker_daemon", "nvidia_runtime", "gmr_image"):
            checks.append(_check(name, "unknown", "Cannot check without Docker CLI"))
    else:
        checks.append(_check("docker_cli", "pass", docker))
        status, detail = _command([docker, "info", "--format", "{{json .Runtimes}}"])
        if status != 0:
            checks.append(_check("docker_daemon", "block", detail or "Docker daemon is unavailable",
                                 "Start Docker Engine and verify: docker info"))
            checks.append(_check("nvidia_runtime", "unknown", "Cannot check while Docker daemon is unavailable"))
            checks.append(_check("gmr_image", "unknown", "Cannot check while Docker daemon is unavailable"))
        else:
            checks.append(_check("docker_daemon", "pass", "Docker daemon responds"))
            try:
                runtimes = json.loads(detail)
            except ValueError:
                runtimes = None
            if runtimes is None:
                checks.append(_check("nvidia_runtime", "unknown", "Could not read Docker runtimes",
                                     "docker info --format '{{json .Runtimes}}'"))
            else:
                checks.append(_check("nvidia_runtime", "pass" if "nvidia" in runtimes else "block",
                                     "NVIDIA Container Toolkit runtime registered" if "nvidia" in runtimes
                                     else "NVIDIA Container Toolkit runtime is absent",
                                     "Install NVIDIA Container Toolkit, configure Docker, and restart Docker."))
            status, _ = _command([docker, "image", "inspect", "--format", "{{.Id}}", IMAGE])
            checks.append(_check("gmr_image", "pass" if status == 0 else "block", IMAGE,
                                 "docker build -t gmr-live:bb1bbe4 -f docker/gmr-live/Dockerfile ."))

    gpu = shutil.which("nvidia-smi")
    if not gpu:
        checks.append(_check("nvidia_gpu", "block", "nvidia-smi is unavailable",
                             "Install a working NVIDIA driver and verify: nvidia-smi -L"))
    else:
        status, detail = _command([gpu, "-L"])
        checks.append(_check("nvidia_gpu", "pass" if status == 0 else "block", detail or "No NVIDIA GPU reported",
                             "Verify the NVIDIA driver: nvidia-smi -L"))

    display = env.get("DISPLAY") or ":0"
    match = re.fullmatch(r"(?:(?:unix)?/)?:(\d+)(?:\.\d+)?", display)
    socket = Path("/tmp/.X11-unix") / f"X{match.group(1)}" if match else None
    checks.append(_check("x11_socket", "pass" if socket and socket.is_socket() else "block",
                         str(socket) if socket else f"Unsupported local DISPLAY={display!r}",
                         "Start a local X11 session and export DISPLAY=:0 (or its active display)."))
    authority = Path(env.get("XAUTHORITY") or f"/run/user/{os.getuid()}/gdm/Xauthority")
    checks.append(_check("xauthority", "pass" if authority.is_file() else "block", str(authority),
                         "export XAUTHORITY=/path/to/your/current/Xauthority"))
    return checks


def diagnose(profile: str, *, workspace: Path | None = None,
             robot: str = "unitree_g1", with_operator: bool = False,
             env: dict[str, str] | None = None) -> dict[str, object]:
    if profile not in ("core", "recorded", "live"):
        raise ValueError(f"unsupported profile: {profile}")
    env = dict(os.environ if env is None else env)
    checks = _python_checks(include_numpy=profile == "core")
    if profile == "recorded":
        selected = workspace or (Path(env["MOCAP_WS"]).expanduser() if env.get("MOCAP_WS") else None)
        checks.extend(_workspace_checks(selected, robot))
    if with_operator:
        checks.extend(_optional_modules(("fastapi", "uvicorn"), "operator"))
    if profile == "live":
        checks.extend(_live_checks(env))
    report: dict[str, object] = {
        "profile": profile, "ok": not any(item["status"] == "block" for item in checks),
        "checks": checks,
    }
    if profile == "recorded" and selected is not None:
        report["full_verification"] = (
            f"python3 tools/bootstrap_g1_workspace.py check --workspace {shlex.quote(str(selected))}"
        )
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("profile", choices=("core", "recorded", "live"))
    parser.add_argument("--workspace", type=Path, help="external GMR workspace (recorded profile)")
    parser.add_argument("--robot", choices=("unitree_g1", "booster_t1_29dof"),
                        default="unitree_g1", help="required recorded model (default: %(default)s)")
    parser.add_argument("--with-operator", action="store_true", help="also test FastAPI and Uvicorn")
    parser.add_argument("--json", action="store_true", help="print machine-readable report")
    args = parser.parse_args(argv)
    if args.workspace and args.profile != "recorded":
        parser.error("--workspace is only used by the recorded profile")
    if args.profile == "live" and args.robot != "unitree_g1":
        parser.error("live streaming supports unitree_g1 only; use recorded for Booster T1")
    report = diagnose(args.profile, workspace=args.workspace, robot=args.robot,
                      with_operator=args.with_operator)
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        for item in report["checks"]:
            print(f"[{item['status'].upper():7}] {item['name']}: {item['detail']}")
            if item["status"] != "pass" and item["fix"]:
                print(f"          Fix: {item['fix']}")
        if report.get("full_verification"):
            print(f"Full GMR patch/import verification: {report['full_verification']}")
        advisory = any(item["status"] in ("warn", "unknown") for item in report["checks"])
        print("BLOCKED" if not report["ok"] else
              "READY (review warnings and unknown checks)" if advisory else "READY")
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
