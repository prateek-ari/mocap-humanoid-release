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
import socket
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
IMAGE = "gmr-live:bb1bbe4"
PINNED_GMR = "bb1bbe40774794fceb2a7c579a3464a28e68c844"
#: Robots of the recorded route: key, check name, model under GMR's assets/.
#: Listed here, not read from mocap_g1, because the doctor runs before the venv exists.
RECORDED_MODELS = (
    ("booster_k1", "k1_model", "booster_k1/K1_serial.xml"),
    ("unitree_g1", "g1_model", "unitree_g1/g1_mocap_29dof.xml"),
    ("booster_t1_29dof", "booster_model", "booster_t1_29dof/t1_mocap.xml"),
)
WINDOWS_MIN_PYTHON = (3, 13)
WINDOWS_PYTHON_REASON = "Windows time.monotonic is 15.6 ms-granular before Python 3.13"
WINDOWS_INSTALL = "Run Install-Windows.cmd (it installs Python 3.13 and rebuilds the runtime)."
# The installed Windows app (MOCAP_APP=1): no project .venv, no git checkouts, a bundled runtime.
APP_REINSTALL = "Reinstall Mocap Humanoid (run its Setup program again)."
WINDOWS_RUNTIME_MODULES = ("general_motion_retargeting", "mujoco", "mink", "numpy", "scipy", "imageio_ffmpeg")
GL_PROBE = (
    "import mujoco\n"
    "xml = \"<mujoco><worldbody><light pos='0 0 3'/><geom type='sphere' size='0.1'/></worldbody></mujoco>\"\n"
    "model = mujoco.MjModel.from_xml_string(xml)\n"
    "data = mujoco.MjData(model)\n"
    "mujoco.mj_forward(model, data)\n"
    "renderer = mujoco.Renderer(model, 64, 64)\n"
    "renderer.update_scene(data)\n"
    "frame = renderer.render()\n"
    "renderer.close()\n"
    "print('GL_OK', frame.shape)\n"
)
MODULE_PROBE = (
    "import importlib, json, sys\n"
    "out = {'python': list(sys.version_info[:3]), 'modules': {}, 'errors': {}}\n"
    "for name in %s:\n"
    "    try:\n"
    "        module = importlib.import_module(name)\n"
    "        out['modules'][name] = str(getattr(module, '__version__', 'installed'))\n"
    "    except Exception as exc:\n"
    "        out['errors'][name] = type(exc).__name__ + ': ' + str(exc)[:200]\n"
    "print(json.dumps(out))\n"
) % repr(WINDOWS_RUNTIME_MODULES)


def _check(name: str, status: str, detail: str, fix: str = "") -> dict[str, str]:
    return {"name": name, "status": status, "detail": detail, "fix": fix}


def _is_windows(platform: str | None = None) -> bool:
    return (sys.platform if platform is None else platform) == "win32"


def _app_mode(env: dict[str, str] | None) -> bool:
    return (os.environ if env is None else env).get("MOCAP_APP") == "1"


def _interpreter_version() -> tuple[int, int]:
    return sys.version_info[0], sys.version_info[1]


def _command(argv: list[str], timeout: float = 8) -> tuple[int | None, str]:
    """Return only status and output from bounded, read-only subprocesses."""
    try:
        result = subprocess.run(argv, text=True, capture_output=True, timeout=timeout, check=False)
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


def _python_checks(*, include_numpy: bool = True, windows: bool = False,
                   app: bool = False) -> list[dict[str, str]]:
    checks = []
    pip_fix = (APP_REINSTALL if app else
               r".venv\Scripts\python.exe -m pip install -e ." if windows else "python3 -m pip install -e .")
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
                             APP_REINSTALL if app else WINDOWS_INSTALL if windows else
                             f"Install Python {requirement.group(1)}.{requirement.group(2)}+ and rerun with that interpreter."))
    if windows:
        checks.append(_windows_python_check("python_313", _interpreter_version(), "doctor interpreter"))
    if not include_numpy:
        return checks
    numpy = _module("numpy")
    if numpy:
        version = str(getattr(numpy, "__version__", "unknown"))
        pair = _version_pair(version)
        valid = pair is not None and (2, 2) <= pair < (2, 3)
        checks.append(_check("numpy", "pass" if valid else "block",
                             f"NumPy {version} imports" + ("" if valid else "; project requires >=2.2,<2.3"),
                             pip_fix))
    else:
        checks.append(_check("numpy", "block", "NumPy cannot be imported", pip_fix))
    return checks


def _windows_python_check(name: str, version: tuple[int, int], label: str) -> dict[str, str]:
    shown = f"{version[0]}.{version[1]}"
    if version >= WINDOWS_MIN_PYTHON:
        return _check(name, "pass", f"{label} Python {shown} satisfies >=3.13")
    return _check(name, "block", f"{label} Python {shown}; Windows requires >=3.13 ({WINDOWS_PYTHON_REASON})",
                  WINDOWS_INSTALL)


def _optional_modules(names: tuple[str, ...], extra: str, *, windows: bool = False,
                      app: bool = False) -> list[dict[str, str]]:
    checks = []
    for name in names:
        if _module(name):
            checks.append(_check(name, "pass", f"{name} imports"))
        else:
            fix = (APP_REINSTALL if app else
                   rf'.venv\Scripts\python.exe -m pip install -e ".[{extra}]"' if windows
                   else f"python3 -m pip install -e '.[{extra}]'")
            checks.append(_check(name, "block", f"{name} cannot be imported", fix))
    return checks


def _runtime_python(workspace: Path, windows: bool, env: dict[str, str] | None = None) -> Path:
    """The workspace venv interpreter; a bundled runtime (no venv) uses MOCAP_RUNTIME_PYTHON."""
    venv = workspace / ("venv/Scripts/python.exe" if windows else "venv/bin/python")
    bundled = (os.environ if env is None else env).get("MOCAP_RUNTIME_PYTHON", "").strip()
    if bundled and not (workspace / "venv").exists():
        return Path(bundled)
    return venv


def _bundled_gmr_revision(workspace: Path) -> str | None:
    """The commit in GMR_COMMIT.txt of an installed app's runtime (GMR there has no .git)."""
    for path in (workspace / "GMR" / "GMR_COMMIT.txt", workspace / "GMR_COMMIT.txt"):
        try:
            text = path.read_text(encoding="utf-8").strip()
        except OSError:
            continue
        return text.split()[0] if text else ""
    return None


def _verify_command(workspace: Path, windows: bool) -> str:
    if windows:
        return (r".venv\Scripts\python.exe tools\bootstrap_g1_workspace.py check "
                f'--workspace "{workspace}"')
    return f"python3 tools/bootstrap_g1_workspace.py check --workspace {shlex.quote(str(workspace))}"


def _patch_check(gmr: Path, fix: str) -> dict[str, str]:
    markers = (
        ("general_motion_retargeting/motion_retarget.py", "ground_offset = self.ground_offset", "grounding"),
        ("general_motion_retargeting/motion_retarget.py", 'ik_config.get("ground_offset", 0.0)', "grounding"),
        ("general_motion_retargeting/__init__.py", "KinematicsModel = None", "optional-torch"),
    )
    missing: list[str] = []
    for relative, marker, label in markers:
        try:
            text = (gmr / relative).read_text(encoding="utf-8")
        except OSError:
            text = ""
        if marker not in text and label not in missing:
            missing.append(label)
    if missing:
        return _check("gmr_patches", "block", "GMR patches not applied: " + ", ".join(missing), fix)
    return _check("gmr_patches", "pass", "grounding and optional-torch patches are applied")


def _workspace_checks(workspace: Path | None, robot: str, *, windows: bool = False,
                      env: dict[str, str] | None = None) -> list[dict[str, str]]:
    selected_robot = robot
    app = _app_mode(env)
    if workspace is None:
        if app:
            return [_check("workspace", "block", "No runtime workspace selected (MOCAP_WS is unset)",
                           APP_REINSTALL)]
        if windows:
            return [_check("workspace", "block", "No runtime workspace selected",
                           r"Set MOCAP_WS or pass --workspace C:\Users\you\mocap-runtime, or run Install-Windows.cmd.")]
        return [_check("workspace", "block", "No external GMR workspace selected",
                       "Set MOCAP_WS=/absolute/external/runtime or pass --workspace /absolute/external/runtime.")]
    gmr = workspace / "GMR"
    python = _runtime_python(workspace, windows, env)
    lock = workspace / "runtime-lock.txt"
    if app:
        bootstrap = verify = APP_REINSTALL
    elif windows:
        bootstrap = verify = WINDOWS_INSTALL
    else:
        workspace_arg = shlex.quote(str(workspace))
        bootstrap = f"python3 tools/bootstrap_g1_workspace.py bootstrap --workspace {workspace_arg}"
        verify = f"python3 tools/bootstrap_g1_workspace.py check --workspace {workspace_arg}"
    probe_timeout = {"timeout": 60} if windows else {}
    checks = []
    bundled_revision = None if (gmr / ".git").exists() else _bundled_gmr_revision(workspace)
    if bundled_revision is not None:
        # The installed app ships GMR without .git; GMR_COMMIT.txt names its commit.
        checks.append(_check("gmr_checkout", "pass" if gmr.is_dir() else "block", str(gmr), bootstrap))
        checks.append(_check("gmr_revision", "pass" if bundled_revision == PINNED_GMR else "block",
                             bundled_revision if bundled_revision == PINNED_GMR else
                             f"Expected {PINNED_GMR}; GMR_COMMIT.txt names {bundled_revision or 'nothing'}",
                             verify))
    else:
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
    if windows:
        checks.append(_patch_check(gmr, bootstrap))
    # The Windows runtime lock is a repo constraints file, not a workspace file; a
    # bundled runtime has no workspace lock either.
    for name, path in ((("venv_python", python),) if windows or bundled_revision is not None
                       else (("venv_python", python), ("runtime_lock", lock))):
        checks.append(_check(name, "pass" if path.is_file() else "block", str(path), bootstrap))
    if python.is_file():
        status, detail = _command([
            str(python), "-c",
            "import json,numpy,scipy,mujoco,general_motion_retargeting; "
            "print(json.dumps({'numpy':numpy.__version__,'scipy':scipy.__version__,"
            "'mujoco':mujoco.__version__}))",
        ], **probe_timeout)
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
    # Only the selected robot's model blocks; the others are optional.
    for robot_key, model_name, relative in RECORDED_MODELS:
        path = gmr / "assets" / relative
        missing_status = "block" if robot_key == selected_robot else "warn"
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


def _bind_probe(port: int, bind: str = "0.0.0.0") -> str | None:
    """Return None when UDP *port* can be bound on *bind* (default all interfaces), else the OS error."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((bind, port))
    except OSError as exc:
        return str(exc)
    finally:
        sock.close()
    return None


def _udp_bind_check(port: int, bind: str = "0.0.0.0") -> dict[str, str]:
    error = _bind_probe(port, bind)
    if error is None:
        return _check("udp_port", "pass", f"UDP {port} is free (bind probe succeeded)")
    return _check("udp_port", "block", f"UDP {port} cannot be bound: {error}",
                  "Close the program using the port (another receiver, or the Setup console's stream check), "
                  f"then check: netstat -ano -p UDP | findstr :{port}")


def _windows_source_check(source: str) -> dict[str, str]:
    fix = ('Set the Xsens source IP in the Setup console, or in PowerShell: $env:XSENS_SOURCE_IP = "127.0.0.1" '
           "(Xsens on this PC) or the other PC's IPv4 from ipconfig")
    try:
        address = ipaddress.IPv4Address(source or "127.0.0.1")
    except ipaddress.AddressValueError:
        return _check("source_ipv4", "block", f"XSENS_SOURCE_IP {source!r} is not an IPv4 address", fix)
    if address.is_unspecified or address.is_multicast:
        return _check("source_ipv4", "block", f"XSENS_SOURCE_IP {address} is not a usable source address", fix)
    if address.is_loopback:
        return _check("source_ipv4", "pass", f"{address} (Xsens streams from this PC)")
    return _check("source_ipv4", "pass", f"{address} (Xsens streams from another PC)")


def _json_tail(text: str) -> dict[str, object]:
    try:
        value = json.loads(text.splitlines()[-1])
    except (ValueError, IndexError):
        return {}
    return value if isinstance(value, dict) else {}


def _windows_runtime_checks(workspace: Path, env: dict[str, str] | None = None) -> list[dict[str, str]]:
    """Runtime venv Python version, module imports, and an OpenGL offscreen probe."""
    python = _runtime_python(workspace, True, env)
    install = APP_REINSTALL if _app_mode(env) else WINDOWS_INSTALL
    checks: list[dict[str, str]] = []
    if not python.is_file():
        for name in ("runtime_python", "runtime_modules", "opengl"):
            checks.append(_check(name, "unknown", f"Cannot check without {python}", install))
        return checks
    status, detail = _command([str(python), "-c", MODULE_PROBE], timeout=60)
    info = _json_tail(detail) if status == 0 else {}
    raw_version = info.get("python")
    if isinstance(raw_version, list) and len(raw_version) >= 2:
        checks.append(_windows_python_check("runtime_python", (int(raw_version[0]), int(raw_version[1])),
                                            "runtime venv"))
    else:
        checks.append(_check("runtime_python", "block", (detail or "Runtime venv Python did not run")[:500],
                             install))
    modules = info.get("modules") if isinstance(info.get("modules"), dict) else {}
    errors = info.get("errors") if isinstance(info.get("errors"), dict) else {}
    missing = [name for name in WINDOWS_RUNTIME_MODULES if name not in modules]
    if info and not missing:
        checks.append(_check("runtime_modules", "pass",
                             ", ".join(f"{name} {modules[name]}" for name in WINDOWS_RUNTIME_MODULES)))
    elif info:
        checks.append(_check("runtime_modules", "block",
                             "Cannot import: " + "; ".join(f"{name} ({errors.get(name, 'missing')})"
                                                           for name in missing), install))
    else:
        checks.append(_check("runtime_modules", "block", (detail or "Runtime import probe failed")[:500],
                             install))
    status, detail = _command([str(python), "-c", GL_PROBE], timeout=30)
    if status == 0 and "GL_OK" in detail:
        checks.append(_check("opengl", "pass", "Offscreen MuJoCo renderer produced a frame"))
    else:
        reason = (detail or "OpenGL probe failed")[-500:]
        if status is None:
            reason = f"OpenGL probe did not finish within 30 s: {reason}"
        checks.append(_check("opengl", "block", f"Offscreen MuJoCo render failed: {reason}",
                             "Update the graphics driver (Intel, AMD or NVIDIA) from the vendor or Windows Update, "
                             "then run the Setup console check again. Remote desktop sessions often lack OpenGL; "
                             "use the PC's own display."))
    return checks


def _windows_live_checks(env: dict[str, str], workspace: Path | None,
                         robot: str) -> list[dict[str, str]]:
    checks = [_windows_source_check(env.get("XSENS_SOURCE_IP", ""))]
    raw_port = env.get("XSENS_UDP_PORT", "5555")
    try:
        port = int(raw_port)
        valid_port = raw_port.isdecimal() and 1 <= port <= 65535
    except ValueError:
        valid_port = False
    if valid_port:
        # Xsens on this PC: probe 127.0.0.1 like the viewer, so no Windows Firewall prompt.
        loopback = env.get("XSENS_SOURCE_IP", "127.0.0.1") in ("", "127.0.0.1")
        checks.append(_udp_bind_check(port, "127.0.0.1" if loopback else "0.0.0.0"))
    else:
        checks.append(_check("udp_port", "block", f"Invalid XSENS_UDP_PORT={raw_port!r}",
                             '$env:XSENS_UDP_PORT = "5555"'))
    checks.extend(_workspace_checks(workspace, robot, windows=True, env=env))
    if workspace is not None:
        checks.extend(_windows_runtime_checks(workspace, env))
    gpu = shutil.which("nvidia-smi")
    if not gpu:
        checks.append(_check("nvidia_gpu", "unknown",
                             "nvidia-smi not found (informational; MuJoCo also runs on Intel and AMD GPUs)"))
    else:
        status, detail = _command([gpu, "-L"])
        checks.append(_check("nvidia_gpu", "pass" if status == 0 else "unknown",
                             detail or "nvidia-smi reported no NVIDIA GPU (informational)"))
    return checks


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


def _default_workspace(env: dict[str, str]) -> Path:
    if env.get("MOCAP_WS"):
        return Path(env["MOCAP_WS"]).expanduser()
    home = env.get("USERPROFILE")
    return (Path(home) if home else Path.home()) / "mocap-runtime"


def diagnose(profile: str, *, workspace: Path | None = None,
             robot: str = "booster_k1", with_operator: bool = False,
             env: dict[str, str] | None = None,
             platform: str | None = None) -> dict[str, object]:
    if profile not in ("core", "recorded", "live"):
        raise ValueError(f"unsupported profile: {profile}")
    windows = _is_windows(platform)
    env = dict(os.environ if env is None else env)
    app = _app_mode(env)
    checks = _python_checks(include_numpy=profile == "core", windows=windows, app=app)
    selected = None
    if profile == "recorded":
        selected = workspace or (Path(env["MOCAP_WS"]).expanduser() if env.get("MOCAP_WS") else None)
        checks.extend(_workspace_checks(selected, robot, windows=windows, env=env))
    if with_operator:
        checks.extend(_optional_modules(("fastapi", "uvicorn"), "operator", windows=windows, app=app))
    if profile == "live":
        if windows:
            checks.extend(_windows_live_checks(env, workspace or _default_workspace(env), robot))
        else:
            checks.extend(_live_checks(env))
    report: dict[str, object] = {
        "profile": profile, "ok": not any(item["status"] == "block" for item in checks),
        "checks": checks,
    }
    if profile == "recorded" and selected is not None and not app:
        report["full_verification"] = _verify_command(selected, windows)
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("profile", choices=("core", "recorded", "live"))
    parser.add_argument("--workspace", type=Path, help="external GMR workspace (recorded profile; live on Windows)")
    parser.add_argument("--robot", choices=tuple(key for key, _name, _model in RECORDED_MODELS),
                        default="booster_k1",
                        help="required recorded model (default: %(default)s); the live profile "
                        "drives every robot in mocap_g1/live_robots.py")
    parser.add_argument("--with-operator", action="store_true", help="also test FastAPI and Uvicorn")
    parser.add_argument("--json", action="store_true", help="print machine-readable report")
    args = parser.parse_args(argv)
    if args.workspace and args.profile != "recorded" and not (args.profile == "live" and _is_windows()):
        parser.error("--workspace is only used by the recorded profile")
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
