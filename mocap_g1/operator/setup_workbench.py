"""Setup workbench: session values and the engineer setup-guide steps as jobs.

The workbench backs the operator console's Setup workspace.  It reads and
writes ``session.env`` in exactly the format ``tools/setup/new_session.sh``
writes, and runs the guide's steps (system check, stream check, live
simulation, BVH check, retarget, render, save) as background jobs.  On Linux
the live steps go through ``tools/xsens/live_operator.sh`` and Docker; on a
native Windows install (``platform="win32"``) they run the Python tools
directly with the runtime venv, and Xsens may stream from this PC (127.0.0.1).

With ``MOCAP_APP=1`` (the installed Windows app) the workbench keeps its
writable state (``session.env``, ``prefs.json``) under ``MOCAP_STATE_DIR``
because the project folder is read-only, runs the bundled interpreter
(``MOCAP_RUNTIME_PYTHON``), and owns a :class:`StreamMonitor` that shows
whether Xsens data is arriving.  Without ``MOCAP_APP`` nothing changes.

It never touches the robot ``SessionController`` and never grants command
authority.  Every job is an argv list (never a shell); ``session.env`` is
parsed with :mod:`shlex`, never sourced.  It is only enabled on a
loopback-only console (see ``create_app(setup_root=...)``).
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import errno
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from typing import Any, Callable, Mapping

try:  # POSIX only; the K1 test window and its lock do not exist on Windows
    import fcntl
except ImportError:  # pragma: no cover - Windows
    fcntl = None  # type: ignore[assignment]
import ipaddress

from ..live_robots import LIVE_ROBOTS
from ._version import service_version
from .core import OperatorError
from .stream_monitor import StreamMonitor, off_snapshot


SESSION_KEYS = (
    "SESSION_ROUTE",
    "SESSION_DIR",
    "ROBOT",
    "HUMAN_HEIGHT_M",
    "MOCAP_WS",
    "IK_CONFIG",
    "MOTION_BVH",
    "NPOSE_BVH",
    "OFFSETS_FILE",
    "BVH_SCALE",
    "START_FRAME",
    "END_FRAME",
    "FRAME_STEP",
    "NPOSE_START_S",
    "NPOSE_DURATION_S",
    "WINDOWS_XSENS_IP",
    "LINUX_XSENS_IP",
    "XSENS_SOURCE_IP",
    "XSENS_UDP_PORT",
)
# JSON field -> session.env key, in SessionValues order.
_FIELD_KEYS = {
    "route": "SESSION_ROUTE",
    "session_dir": "SESSION_DIR",
    "robot": "ROBOT",
    "human_height_m": "HUMAN_HEIGHT_M",
    "mocap_ws": "MOCAP_WS",
    "motion_bvh": "MOTION_BVH",
    "npose_bvh": "NPOSE_BVH",
    "offsets_file": "OFFSETS_FILE",
    "bvh_scale": "BVH_SCALE",
    "start_frame": "START_FRAME",
    "end_frame": "END_FRAME",
    "frame_step": "FRAME_STEP",
    "npose_start_s": "NPOSE_START_S",
    "npose_duration_s": "NPOSE_DURATION_S",
    "windows_xsens_ip": "WINDOWS_XSENS_IP",
    "linux_xsens_ip": "LINUX_XSENS_IP",
}
_FLOAT_FIELDS = ("human_height_m", "bvh_scale", "npose_start_s", "npose_duration_s")
_INT_FIELDS = ("start_frame", "end_frame", "frame_step")
ROUTES = ("live", "recorded")
ROBOTS = ("booster_k1", "unitree_g1", "booster_t1_29dof")
#: Robots whose recorded preview keeps the IK config's arm orientations and the live
#: tool's arm abduction (no wrist, so no hand target to point the forearm).
CONFIG_ARM_ROBOTS = ("booster_k1",)
XSENS_UDP_PORT = "5555"
JOB_KINDS = ("doctor", "acceptance", "live", "bvh_check", "retarget", "render", "save",
             "compare", "k1_install")
JOB_STATES = ("running", "passed", "failed", "stopped")
#: Xsens Humanoid on the same Windows PC streams to (and from) this address.
LOOPBACK_SOURCE = "127.0.0.1"
# Mirrors live_operator.sh acceptance; the duration comes from stream_check_seconds().
ACCEPTANCE_LIMITS = ("--minimum-body-frames", "300", "--minimum-rate", "50",
                     "--maximum-loss", "0.01", "--require-joint-angles")
MAX_LOG_LINES = 5000
MAX_LINES_PER_RESPONSE = 2000
MAX_LISTED_JOBS = 20
LIVE_CONTAINER = "gmr-xsens-live"
# Served file types; .pkl motions are listed but never served.
SERVED_MEDIA_TYPES = {
    ".mp4": "video/mp4",
    ".json": "application/json",
    ".txt": "text/plain; charset=utf-8",
    ".log": "text/plain; charset=utf-8",
}
_FILE_KINDS = {".pkl": "motion", ".mp4": "video", ".json": "report", ".txt": "report", ".log": "report"}
_ANSI = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]|\x1b\][^\x07\x1b]*(?:\x07|\x1b\\)|\x1b[@-Z\\-_]")
_CONTROL = re.compile(r"[\x00-\x08\x0b-\x1f\x7f]")
_VERDICT = re.compile(r"PASS|FAIL|ERROR")
_NUMBER = re.compile(r"^[0-9]+([.][0-9]+)?$")
_IPV4 = re.compile(r"^[0-9]{1,3}([.][0-9]{1,3}){3}$")
NO_DISPLAY_HINT = (
    "No desktop display: close the console with `bash tools/setup/open_console.sh --stop`, "
    "then open it from the Mocap Setup menu entry on this PC (not over SSH)."
)
SESSION_DIR_LOCATION_MESSAGE = (
    "Session folder must be inside your home folder and outside the project checkout"
)
DEFAULT_STREAM_CHECK_S = 30
#: Jobs that bind (or probe) the Xsens UDP port: the stream monitor releases it first.
PORT_JOB_KINDS = ("doctor", "acceptance", "live", "compare")
#: Jobs that bind the Xsens port and open the viewer in the ``gmr-xsens-live`` container.
VIEWER_JOB_KINDS = ("live", "compare")
#: Linux-only jobs (Docker comparison viewer, the K1 tools installer).
LINUX_JOB_KINDS = ("compare", "k1_install")
#: The K1 test window's single-instance lock; tools/setup/install.sh takes the same file.
K1_LOCK_NAME = "k1-tests.lock"
K1_LOG_NAME = "k1-tests.log"
#: k1_commission_gui.py exits with this code when the lock is held.
K1_ALREADY_OPEN_EXIT = 3
#: The K1 window has this long to fail (lock held, Tk missing) before it counts as opened.
K1_OPEN_WAIT_S = 1.0
K1_MODEL = ("GMR", "assets", "booster_k1", "K1_serial.xml")
PRIVILEGED_ACTIONS = ("firewall", "install_tk")
#: pkexec exit codes: 126 = the user dismissed the password prompt.
PKEXEC_CANCELLED = 126
PRIVILEGED_TIMEOUT_S = 600
UFW = "/usr/sbin/ufw"
#: Commands a running Windows live simulation accepts on stdin (``--control-stdin``).
CONTROL_COMMANDS = ("calibrate", "toggle_tracking", "rearm")
#: The machine status line live_g1_sim.py prints: ``STATUS {json LiveStatus}``.
STATUS_PREFIX = "STATUS "
APP_MODE = "windows_app"
PREFS_SCHEMA = 1
PREFS_FILE = "prefs.json"
STOPPED_SUMMARY = "Stopped by the operator."
_LOCAL_DISPLAY = re.compile(r"(?:(?:unix)?/)?:(\d+)(?:\.\d+)?")
_ARCHIVE_REVISION = re.compile(r"^node:\s*([0-9a-f]{40})\s*$", re.M)
_ENV_HEADER = (
    "# Mocap session created {created} by the operator console Setup tab\n"
    "# Load in each new terminal from the checkout: source session.env\n"
    "# Change values in the console Setup tab or rerun tools/setup/new_session.sh.\n"
)

CommandOverride = Callable[[Mapping[str, Any], Mapping[str, Any]], list[str]]


class SetupError(OperatorError):
    """A Setup API error; ``field`` and ``job_id`` are added to the JSON body."""

    def __init__(
        self,
        message: str,
        *,
        code: str,
        status_code: int = 409,
        field: str | None = None,
        job_id: str | None = None,
    ) -> None:
        super().__init__(message, code=code, status_code=status_code)
        self.field = field
        self.job_id = job_id

    def payload(self) -> dict[str, Any]:
        error: dict[str, Any] = {"code": self.code, "message": str(self)}
        if self.field is not None:
            error["field"] = self.field
        if self.job_id is not None:
            error["job_id"] = self.job_id
        return {"error": error}


def _invalid(message: str, field_name: str) -> SetupError:
    return SetupError(message, code="invalid_session", status_code=422, field=field_name)


def strip_ansi(text: str) -> str:
    """Remove ANSI escape sequences and other control characters except tab."""

    return _CONTROL.sub("", _ANSI.sub("", text))


def parse_live_status(line: str) -> dict[str, Any] | None:
    """Return the LiveStatus of a ``STATUS {json}`` line, or ``None`` if malformed."""

    if not line.startswith(STATUS_PREFIX):
        return None
    try:
        data = json.loads(line[len(STATUS_PREFIX):])
    except ValueError:
        return None
    if not isinstance(data, dict):
        return None

    def whole(name: str) -> int | None:
        value = data.get(name)
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            return None
        return value

    fps = data.get("fps")
    flags = (data.get("enabled"), data.get("calibrated"))
    counts = (whole("frames"), whole("drops"), whole("failures"))
    if (
        not all(isinstance(flag, bool) for flag in flags)
        or not isinstance(data.get("reason"), str)
        or isinstance(fps, bool)
        or not isinstance(fps, (int, float))
        or not math.isfinite(fps)
        or any(count is None for count in counts)
    ):
        return None
    return {
        "enabled": data["enabled"],
        "reason": strip_ansi(data["reason"])[:200],
        "fps": float(fps),
        "frames": counts[0],
        "drops": counts[1],
        "failures": counts[2],
        "calibrated": data["calibrated"],
    }


def default_prefs() -> dict[str, Any]:
    """The app preferences before anything is saved (defaults live in code)."""

    return {"schema": PREFS_SCHEMA, "record_video": False, "calibrate_after_s": None}


def _invalid_prefs(message: str, field_name: str) -> SetupError:
    return SetupError(message, code="invalid_prefs", status_code=400, field=field_name)


def merge_prefs(current: Mapping[str, Any], changes: Mapping[str, Any]) -> dict[str, Any]:
    """Merge a partial Prefs body into ``current``; raise a 400 SetupError if invalid."""

    if not isinstance(changes, Mapping):
        raise _invalid_prefs("Preferences must be an object", "schema")
    prefs = dict(current)
    for name, value in changes.items():
        if name == "schema":
            if value != PREFS_SCHEMA or isinstance(value, bool):
                raise _invalid_prefs(f"schema must be {PREFS_SCHEMA}", name)
        elif name == "record_video":
            if not isinstance(value, bool):
                raise _invalid_prefs("record_video must be true or false", name)
            prefs[name] = value
        elif name == "calibrate_after_s":
            if value is not None:
                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    raise _invalid_prefs("calibrate_after_s must be a number of seconds or null",
                                         name)
                if not (math.isfinite(value) and 0.5 <= value <= 60):
                    raise _invalid_prefs("calibrate_after_s must be 0.5–60 s", name)
                value = float(value)
            prefs[name] = value
        else:
            raise _invalid_prefs(f"Unknown preference: {name}", str(name))
    prefs["schema"] = PREFS_SCHEMA
    return prefs


def usable_ipv4(value: str) -> bool:
    """Mirror ``ipv4`` in new_session.sh: dotted quad, not 0/127/multicast."""

    if not _IPV4.match(value):
        return False
    parts = [int(part, 10) for part in value.split(".")]
    if any(part > 255 for part in parts):
        return False
    return parts[0] != 0 and parts[0] != 127 and parts[0] < 224


def is_windows(platform: str | None = None) -> bool:
    """Whether ``platform`` (default ``sys.platform``) is a native Windows install."""

    return (sys.platform if platform is None else platform) == "win32"


def parse_session_env(text: str) -> dict[str, str]:
    """Return the known ``export KEY=VALUE`` assignments of a session.env.

    The text is tokenized with :func:`shlex.split`; nothing is executed or
    expanded.  Unknown keys, comments and malformed lines are ignored.
    """

    values: dict[str, str] = {}
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped.startswith("export "):
            continue
        try:
            tokens = shlex.split(stripped, comments=True)
        except ValueError:
            continue
        if len(tokens) != 2 or tokens[0] != "export" or "=" not in tokens[1]:
            continue
        key, _, value = tokens[1].partition("=")
        if key in SESSION_KEYS:
            values[key] = value
    return values


def _format_number(value: float | int) -> str:
    if isinstance(value, int):
        return str(value)
    return repr(float(value))


def _tilde(path: str, home: Path | str, *, fold_case: bool = False) -> str:
    home_text = home if isinstance(home, str) else str(home)
    key, home_key = (path.casefold(), home_text.casefold()) if fold_case else (path, home_text)
    if key == home_key:
        return "~"
    if key.startswith(home_key + "/"):
        return "~/" + path[len(home_text) + 1 :]
    return path


# Container and VM bridges never carry the Xsens stream; listing them only confuses the choice.
_VIRTUAL_INTERFACE = re.compile(r"^(docker\d*|br-[0-9a-f]+|virbr\d+|veth.*|lxc.*|lxdbr\d+|cni\d+|flannel.*|tailscale\d*|zt.*)$")


# Windows adapter names that never carry the Xsens stream (WSL/Hyper-V, VPNs, VMs, loopback).
_WINDOWS_VIRTUAL_INTERFACE = re.compile(
    r"loopback|vethernet|hyper-v|wsl|virtualbox|vmware|bluetooth|tailscale|zerotier|tap-|npcap|vpn",
    re.I,
)


def _windows_interfaces() -> list[dict[str, str]]:
    """Return up IPv4 adapters from psutil (Windows has no ``ip``); empty without psutil."""

    try:
        import psutil
    except ImportError:
        return []
    try:
        stats = psutil.net_if_stats()
        addresses = psutil.net_if_addrs()
    except OSError:
        return []
    interfaces = []
    for name, entries in addresses.items():
        if _WINDOWS_VIRTUAL_INTERFACE.search(name) or (name in stats and not stats[name].isup):
            continue
        for entry in entries:
            local = entry.address
            if entry.family == socket.AF_INET and usable_ipv4(local) and not local.startswith("169.254."):
                interfaces.append({"name": name, "ipv4": local})
    return interfaces


def default_interfaces() -> list[dict[str, str]]:
    """Return physical IPv4 interfaces from ``ip -j -4 addr`` (no loopback or bridges)."""

    if is_windows():
        return _windows_interfaces()
    try:
        result = subprocess.run(
            ["ip", "-j", "-4", "addr"],
            capture_output=True,
            text=True,
            timeout=3,
            check=False,
        )
        data = json.loads(result.stdout or "[]")
    except (OSError, subprocess.SubprocessError, ValueError):
        return []
    interfaces = []
    for item in data if isinstance(data, list) else []:
        name = item.get("ifname") if isinstance(item, dict) else None
        if not isinstance(name, str) or name == "lo" or _VIRTUAL_INTERFACE.match(name):
            continue
        for address in item.get("addr_info") or []:
            local = address.get("local") if isinstance(address, dict) else None
            if address.get("family") == "inet" and isinstance(local, str):
                interfaces.append({"name": name, "ipv4": local})
    return interfaces


def default_route_source(destination: str) -> str | None:
    """Return the local source address ``ip -4 route get`` picks for ``destination``."""

    if not usable_ipv4(destination):
        return None
    if is_windows():
        # Connecting a UDP socket only selects the route; nothing is sent.
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.connect((destination, 9))
            local = probe.getsockname()[0]
        except OSError:
            return None
        finally:
            probe.close()
        return local if usable_ipv4(local) else None
    try:
        result = subprocess.run(
            ["ip", "-4", "route", "get", destination],
            capture_output=True,
            text=True,
            timeout=3,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    match = re.search(r"\bsrc ([0-9.]+)", result.stdout)
    return match.group(1) if match else None


def default_stop_container() -> None:
    """Stop the live viewer container (``docker stop gmr-xsens-live``), ignoring failure."""

    if is_windows():  # no container: the Windows viewer is the job's own process
        return
    try:
        subprocess.run(
            ["docker", "stop", LIVE_CONTAINER],
            capture_output=True,
            timeout=20,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        pass


def default_display_available(environ: Mapping[str, str] | None = None) -> bool:
    """Whether a local X11 desktop is reachable, as setup_doctor.py and gmr-live/run.sh check.

    DISPLAY must be set to a local display (``:N`` / ``unix/:N``) whose socket
    exists, and XAUTHORITY (default ``/run/user/<uid>/gdm/Xauthority``) must
    be a file.  An SSH session has no DISPLAY or a forwarded ``host:N`` one.
    """

    if is_windows():  # the console runs in the signed-in desktop session
        return True
    environ = os.environ if environ is None else environ
    match = _LOCAL_DISPLAY.fullmatch(environ.get("DISPLAY") or "")
    if match is None:
        return False
    if not (Path("/tmp/.X11-unix") / f"X{match.group(1)}").is_socket():
        return False
    authority = environ.get("XAUTHORITY") or f"/run/user/{os.getuid()}/gdm/Xauthority"
    return Path(authority).is_file()


def default_viewer_running() -> bool:
    """Whether a ``gmr-xsens-live`` container is running; no Docker counts as no."""

    if is_windows():
        return False
    try:
        result = subprocess.run(
            ["docker", "ps", "-q", "--filter", f"name=^{LIVE_CONTAINER}$"],
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return False
    return result.returncode == 0 and bool(result.stdout.strip())


def default_udp_port_busy(port: int, bind: str = "0.0.0.0") -> bool:
    """Whether another program has bound UDP ``port`` (bind without SO_REUSEADDR).

    The probe binds the address the job will bind (127.0.0.1 when Xsens is on
    this Windows PC, so the probe raises no Windows Firewall prompt either).
    """

    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.bind((bind, port))
    except OSError as exc:
        # WSAEACCES (10013): Windows reports an exclusively bound port as access denied.
        return exc.errno == errno.EADDRINUSE or getattr(exc, "winerror", None) in (10048, 10013)
    finally:
        probe.close()
    return False


def default_user_state_dir(home: Path, environ: Mapping[str, str] | None = None) -> Path:
    """``${XDG_STATE_HOME:-~/.local/state}/mocap-g1``: open_console.sh, install.sh and the K1 window use it."""

    environ = os.environ if environ is None else environ
    base = environ.get("XDG_STATE_HOME", "").strip()
    return (Path(base) if base else home / ".local" / "state") / "mocap-g1"


def default_firewall_state(conf: Path = Path("/etc/ufw/ufw.conf")) -> str:
    """UFW's state read without privileges: ``active``, ``inactive``, ``not_installed`` or ``unknown``.

    ``ufw status`` needs root, so this reads ENABLED= in ufw.conf and asks
    systemd whether the ufw unit is up.  Anything unclear is ``unknown``
    (never ``inactive``), so the console keeps offering the firewall button.
    """

    if not Path(UFW).exists():
        return "not_installed"
    try:
        match = re.search(r"^\s*ENABLED\s*=\s*(\w+)", conf.read_text(encoding="utf-8"), re.M)
    except OSError:
        return "unknown"
    if match is None:
        return "unknown"
    if match.group(1).lower() == "no":
        return "inactive"
    if match.group(1).lower() != "yes":
        return "unknown"
    try:
        unit = subprocess.run(["systemctl", "is-active", "ufw"], capture_output=True, text=True,
                              timeout=3, check=False).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return "unknown"
    return "active" if unit == "active" else "unknown"


def default_run_privileged(argv: list[str]) -> int:
    """Run a fixed ``pkexec`` argv (the desktop's graphical password prompt); return its exit code.

    127 when pkexec is missing; 124 when nobody answered in PRIVILEGED_TIMEOUT_S.
    """

    try:
        return subprocess.run(argv, stdin=subprocess.DEVNULL, capture_output=True,
                              timeout=PRIVILEGED_TIMEOUT_S, check=False).returncode
    except subprocess.TimeoutExpired:
        return 124
    except OSError:
        return 127


def default_k1_lock_held(path: Path) -> bool:
    """Whether someone (the K1 window, install.sh) holds the K1 lock; tries it without waiting."""

    if fcntl is None or not path.exists():
        return False
    try:
        with open(path, "a", encoding="utf-8") as handle:
            try:
                fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                return True
            fcntl.flock(handle, fcntl.LOCK_UN)
    except OSError:
        return False
    return False


def default_spawn_k1(argv: list[str], cwd: Path, env: Mapping[str, str], log_path: Path,
                     wait_s: float) -> int | None:
    """Start the K1 window in its own session; its exit code if it ended within ``wait_s``, else None.

    The window must outlive the console (its safety handling keeps running when
    Setup shuts down), so it is not a job and gets no process group of ours.
    """

    log_path.parent.mkdir(parents=True, exist_ok=True)
    with open(log_path, "ab") as log:
        process = subprocess.Popen(argv, cwd=cwd, env=dict(env), stdin=subprocess.DEVNULL,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        return process.wait(timeout=wait_s)
    except subprocess.TimeoutExpired:
        return None


_K1_PROBE = (
    "import importlib.metadata as m, json\n"
    "out = {'sdk_version': None, 'tk': False}\n"
    "try:\n"
    "    import booster_robotics_sdk_python\n"
    "    out['sdk_version'] = m.version('booster_robotics_sdk_python')\n"
    "except Exception:\n"
    "    pass\n"
    "try:\n"
    "    import tkinter\n"
    "    out['tk'] = True\n"
    "except Exception:\n"
    "    pass\n"
    "print(json.dumps(out))\n"
)


def default_probe_k1(python: str) -> dict[str, Any]:
    """Ask the project interpreter (the one the K1 window runs on) for the SDK version and Tk."""

    found: dict[str, Any] = {"sdk_version": None, "tk": False}
    try:
        result = subprocess.run([python, "-c", _K1_PROBE], capture_output=True, text=True,
                                timeout=20, check=False)
        reported = json.loads(result.stdout.strip().splitlines()[-1])
    except (OSError, subprocess.SubprocessError, ValueError, IndexError):
        return found
    if isinstance(reported, dict):
        version = reported.get("sdk_version")
        found["sdk_version"] = version if isinstance(version, str) and version else None
        found["tk"] = reported.get("tk") is True
    return found


def default_shutdown_console() -> None:
    """Stop this console as Ctrl+C does: uvicorn stops jobs (lifespan) and exits."""

    os.kill(os.getpid(), signal.SIGINT)


@dataclass
class _Job:
    id: str
    kind: str
    label: str
    started_at: float
    wall_started_at: float
    session_dir: Path
    outputs: list[Path] = field(default_factory=list)
    state: str = "running"
    finished_at: float | None = None
    exit_code: int | None = None
    summary: str | None = None
    lines: deque = field(default_factory=lambda: deque(maxlen=MAX_LOG_LINES))
    total_lines: int = 0
    process: subprocess.Popen | None = None
    stop_requested: bool = False
    thread: threading.Thread | None = None
    terminator: threading.Thread | None = None
    log_handle: Any = None
    #: The latest LiveStatus from a live job's ``STATUS {json}`` lines.
    live_status: dict[str, Any] | None = None
    #: A Windows live job's stdin (``--control-stdin``); written under ``lock``.
    control: Any = None
    lock: threading.Lock = field(default_factory=threading.Lock)

    def append(self, text: str) -> None:
        for raw in text.replace("\r\n", "\n").split("\n"):
            # A progress bar redraws with \r: keep its latest state only.
            segments = [segment for segment in raw.split("\r") if segment.strip()]
            line = strip_ansi(segments[-1] if segments else "").rstrip()
            status = parse_live_status(line) if self.kind == "live" else None
            with self.lock:
                if status is not None:
                    # Machine status goes to the log file only, not the visible log.
                    self.live_status = status
                else:
                    self.lines.append(line)
                    self.total_lines += 1
                handle = self.log_handle
            if handle is not None:
                try:
                    handle.write(line + "\n")
                    handle.flush()
                except (OSError, ValueError):
                    pass

    def summary_unlocked(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "kind": self.kind,
            "label": self.label,
            "state": self.state,
            "started_at": self.started_at,
            "finished_at": self.finished_at,
            "exit_code": self.exit_code,
            "summary": self.summary,
            "outputs": [path.name for path in self.outputs],
            "live": dict(self.live_status) if self.live_status is not None else None,
        }

    def summary_dict(self) -> dict[str, Any]:
        with self.lock:
            return self.summary_unlocked()

    def lines_after(self, after: int) -> tuple[list[str], int]:
        """Return buffered lines from absolute index ``after`` and the next index."""

        with self.lock:
            first = self.total_lines - len(self.lines)
            start = min(max(after, first), self.total_lines)
            stop = min(self.total_lines, start + MAX_LINES_PER_RESPONSE)
            return list(self.lines)[start - first : stop - first], stop

    def compute_summary(self) -> str | None:
        with self.lock:
            lines = [line for line in self.lines if line.strip()]
        for line in reversed(lines):
            if _VERDICT.search(line):
                return line.strip()
        return lines[-1].strip() if lines else None


class SetupWorkbench:
    """Session values and setup-guide jobs for one source checkout.

    ``command_overrides`` maps a job kind to a callable
    ``(session_values, options) -> argv`` that replaces the kind's external
    command (for ``bvh_check`` it replaces the in-process check; for ``save``
    it replaces the doctor command whose stdout becomes ``core-doctor.json``;
    for ``acceptance`` the report path arrives as ``options["output"]``).
    ``interfaces``, ``route_source``, ``stop_container``,
    ``display_available``, ``viewer_running`` and ``udp_port_busy`` replace
    the ``ip``, ``docker`` and X11/UDP probes.  Pre-checks (runtime, display,
    leftover viewer, busy port) apply whether or not a command is overridden.

    ``platform`` (default ``sys.platform``) selects the session and command
    conventions: ``"win32"`` uses forward-slash drive paths, a copied
    ``session.env`` instead of a symlink, ``venv/Scripts/python.exe``, the
    Python tools without Docker, and accepts 127.0.0.1 as the Xsens source.
    How job processes are started and stopped always follows the host OS.

    ``app_mode`` (default: ``MOCAP_APP=1``) is the installed Windows app:
    ``session.env`` and ``prefs.json`` live in ``state_dir`` (default
    ``MOCAP_STATE_DIR``), the runtime is the bundled interpreter, live video
    is recorded into the session folder, and a :class:`StreamMonitor`
    (``stream_monitor`` replaces the default one) watches the Xsens port.
    """

    def __init__(
        self,
        repo_root: str | Path,
        *,
        home: str | Path | None = None,
        command_overrides: Mapping[str, CommandOverride] | None = None,
        now: Callable[[], float] = time.time,
        interfaces: Callable[[], list[dict[str, str]]] = default_interfaces,
        route_source: Callable[[str], str | None] = default_route_source,
        stop_container: Callable[[], None] = default_stop_container,
        display_available: Callable[[], bool] = default_display_available,
        viewer_running: Callable[[], bool] = default_viewer_running,
        udp_port_busy: Callable[[int], bool] = default_udp_port_busy,
        stop_grace_s: float = 5.0,
        platform: str | None = None,
        app_mode: bool | None = None,
        state_dir: str | Path | None = None,
        stream_monitor: StreamMonitor | None = None,
        firewall_state: Callable[[], str] = default_firewall_state,
        run_privileged: Callable[[list[str]], int] = default_run_privileged,
        k1_lock_held: Callable[[Path], bool] = default_k1_lock_held,
        spawn_k1: Callable[..., int | None] = default_spawn_k1,
        probe_k1: Callable[[str], dict[str, Any]] = default_probe_k1,
        shutdown_console: Callable[[], None] = default_shutdown_console,
        user_state_dir: str | Path | None = None,
    ) -> None:
        self.repo_root = Path(repo_root).expanduser().resolve()
        if not (self.repo_root / "tools" / "xsens" / "live_operator.sh").is_file():
            raise ValueError(
                f"{self.repo_root} is not a mocap_humanoid source checkout "
                "(tools/xsens/live_operator.sh is missing); the Setup workspace "
                "needs the checkout's tools"
            )
        self.windows = is_windows(platform)
        self.home = Path(home) if home is not None else Path.home()
        #: The home folder as the API shows it (forward slashes on Windows).
        self.home_text = self._text(self.home)
        self.command_overrides = dict(command_overrides or {})
        unknown = set(self.command_overrides) - set(JOB_KINDS)
        if unknown:
            raise ValueError(f"Unknown job kinds in command_overrides: {sorted(unknown)}")
        self._now = now
        self._interfaces = interfaces
        self._route_source = route_source
        self._stop_container = stop_container
        self._display_available = display_available
        self._viewer_running = viewer_running
        self._udp_port_busy = udp_port_busy
        self.stop_grace_s = float(stop_grace_s)
        self._lock = threading.Lock()
        self._jobs: dict[str, _Job] = {}
        self._order: list[str] = []
        self._counter = 0
        self.app_mode = os.environ.get("MOCAP_APP") == "1" if app_mode is None else bool(app_mode)
        if state_dir is None and os.environ.get("MOCAP_STATE_DIR", "").strip():
            state_dir = os.environ["MOCAP_STATE_DIR"].strip()
        if state_dir is None and self.app_mode:
            local = os.environ.get("LOCALAPPDATA", "").strip()
            state_dir = (Path(local) / "MocapHumanoid" if local
                         else self.home / ".local" / "state" / "mocap-humanoid")
        #: Per-user writable state (app mode); ``None`` keeps today's checkout layout.
        self.state_dir = Path(state_dir).expanduser().resolve() if state_dir is not None else None
        self._prefs_lock = threading.Lock()
        self._memory_prefs = default_prefs()
        if stream_monitor is None and self.app_mode:
            stream_monitor = StreamMonitor()
        self._monitor = stream_monitor
        self._monitor_started = False
        self._firewall_state = firewall_state
        self._run_privileged = run_privileged
        self._k1_lock_held = k1_lock_held
        self._spawn_k1 = spawn_k1
        self._probe_k1 = probe_k1
        self._shutdown_console = shutdown_console
        #: ``${XDG_STATE_HOME:-~/.local/state}/mocap-g1``: console log, K1 lock and K1 log.
        self.user_state_dir = (Path(user_state_dir) if user_state_dir is not None
                               else default_user_state_dir(self.home))
        # One password prompt at a time (firewall or Tk).
        self._privileged_lock = threading.Lock()

    # ------------------------------------------------------------------ paths

    @property
    def link_path(self) -> Path:
        """``<repo>/session.env``; in app mode ``<state>/session.env`` (the project is read-only)."""

        if self.app_mode and self.state_dir is not None:
            return self.state_dir / "session.env"
        return self.repo_root / "session.env"

    def _stamp(self, fmt: str) -> str:
        return time.strftime(fmt, time.localtime(self._now()))

    def _text(self, path: Path | str) -> str:
        """A path as session values and the API spell it (forward slashes on Windows)."""

        return Path(path).as_posix() if self.windows else str(path)

    def _tilde(self, path: str) -> str:
        return _tilde(path, self.home_text, fold_case=self.windows)

    def _is_absolute(self, value: str) -> bool:
        if not self.windows:
            return value.startswith("/")
        # A drive path (C:/...) on Windows; UNC shares (//server/...) are not accepted.
        return not value.startswith("//") and Path(value).is_absolute()

    def _absolute(self, value: str, field_name: str, label: str) -> str:
        if self.windows:
            value = value.replace("\\", "/")
        if value == "~":
            value = self.home_text
        elif value.startswith("~/"):
            value = f"{self.home_text}/{value[2:]}"
        if not self._is_absolute(value):
            example = "C:/Users/you/..." if self.windows else "~/..."
            raise _invalid(f"{label}: use an absolute path (or {example})", field_name)
        if _CONTROL.search(value) or "\n" in value or "\t" in value:
            raise _invalid(f"{label} must not contain control characters", field_name)
        return value

    def _ik_config(self, robot: str, mocap_ws: str) -> str:
        if robot in ("booster_t1_29dof", "booster_k1"):
            return self._text(self.repo_root / "retarget_configs" / f"bvh_xsens_to_{robot}.json")
        return f"{mocap_ws}/GMR/general_motion_retargeting/ik_configs/bvh_xsens_to_g1.json"

    def runtime_python(self, mocap_ws: str) -> str:
        """The GMR runtime venv interpreter inside ``mocap_ws``.

        A bundled runtime (the installed app: ``mocap_ws`` holds ``GMR`` but no
        ``venv``) runs on ``MOCAP_RUNTIME_PYTHON``, or in app mode on the
        console's own interpreter, which has GMR on its path.
        """

        venv = (f"{mocap_ws}/venv/Scripts/python.exe" if self.windows
                else f"{mocap_ws}/venv/bin/python")
        if (Path(mocap_ws) / "venv").exists():
            return venv
        bundled = os.environ.get("MOCAP_RUNTIME_PYTHON", "").strip()
        if not bundled and self.app_mode:
            bundled = sys.executable
        return self._text(bundled) if bundled else venv

    def _runtime_ready(self, mocap_ws: str) -> bool:
        return Path(self.runtime_python(mocap_ws)).exists() and (Path(mocap_ws) / "GMR").is_dir()

    def _installer_hint(self, route: str) -> str:
        if self.app_mode:
            return "Reinstall Mocap Humanoid (run its Setup program again)."
        if self.windows:
            return "Run Install-Windows.cmd in the project folder again."
        return "Run the installer and choose 2 · Recorded." if route == "recorded" else (
            "Run the installer and choose 1 · Live."
        )

    # ------------------------------------------------------------- session I/O

    def _default_runtime(self) -> str:
        """``~/mocap-runtime``; on Windows the installer's MOCAP_WS when it set another one."""

        if self.windows:
            chosen = os.environ.get("MOCAP_WS", "").strip().replace("\\", "/")
            if chosen and self._is_absolute(chosen):
                return chosen.rstrip("/")
        return f"{self.home_text}/mocap-runtime"

    def builtin_defaults(self) -> dict[str, Any]:
        home = self.home_text
        return {
            "route": "live",
            "session_dir": "",
            "robot": "booster_k1",
            "human_height_m": 1.75,
            "mocap_ws": self._default_runtime(),
            "motion_bvh": f"{home}/mocap-data/motion.bvh",
            "npose_bvh": f"{home}/mocap-data/neutral.bvh",
            "offsets_file": "",
            "bvh_scale": 0.01,
            "start_frame": 0,
            "end_frame": 600,
            "frame_step": 1,
            "npose_start_s": 2.0,
            "npose_duration_s": 1.0,
            # A Windows install defaults to Xsens Humanoid on the same PC.
            "windows_xsens_ip": LOOPBACK_SOURCE if self.windows else "",
            "linux_xsens_ip": "",
        }

    def _values_from_env(self, raw: Mapping[str, str]) -> dict[str, Any]:
        values = self.builtin_defaults()
        for name, key in _FIELD_KEYS.items():
            if key not in raw:
                continue
            text = raw[key]
            if name in _FLOAT_FIELDS:
                try:
                    number = float(text)
                except ValueError:
                    continue
                if math.isfinite(number):
                    values[name] = number
            elif name in _INT_FIELDS:
                try:
                    values[name] = int(text, 10)
                except ValueError:
                    continue
            else:
                values[name] = text
        return values

    def read_session_raw(self) -> dict[str, str] | None:
        """Return the raw assignments of ``<repo>/session.env``, or ``None``.

        On Windows ``<repo>/session.env`` is a copy (symlinks need extra
        rights there), so the session folder's own ``session.env`` is read
        when it still exists: a hand edit there is not lost.
        """

        raw = self._read_env(self.link_path)
        if raw is None or not self.windows or self.link_path.is_symlink():
            return raw
        session_dir = raw.get("SESSION_DIR")
        if session_dir and self._is_absolute(session_dir):
            current = self._read_env(Path(session_dir) / "session.env")
            if current is not None and current.get("SESSION_DIR") == session_dir:
                return current
        return raw

    @staticmethod
    def _read_env(path: Path) -> dict[str, str] | None:
        try:
            text = path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            return None
        return parse_session_env(text)

    def current_session(self) -> dict[str, Any] | None:
        raw = self.read_session_raw()
        if raw is None:
            return None
        return self._values_from_env(raw)

    def _fresh_session_dir(self) -> str:
        base = f"{self.home_text}/mocap-sessions/{self._stamp('%Y-%m-%d-%H%M')}"
        candidate, suffix = base, 1
        while os.path.lexists(candidate):
            suffix += 1
            candidate = f"{base}-{suffix}"
        return candidate

    def new_session_defaults(self, current: Mapping[str, Any] | None = None) -> dict[str, Any]:
        """Prefill a new session: previous values in a fresh dated folder."""

        values = dict(current) if current is not None else self.builtin_defaults()
        values["session_dir"] = self._fresh_session_dir()
        if values.get("windows_xsens_ip") and not self._same_pc(values["windows_xsens_ip"]):
            detected = self._route_source(values["windows_xsens_ip"])
            if detected and usable_ipv4(detected) and detected != values["windows_xsens_ip"]:
                values["linux_xsens_ip"] = detected
        return values

    def validate(self, body: Mapping[str, Any]) -> dict[str, Any]:
        """Validate SessionValues exactly like new_session.sh; raise 422 SetupError."""

        def text(name: str, label: str) -> str:
            if name not in body:
                raise _invalid(f"{label} is required", name)
            value = body[name]
            if not isinstance(value, str):
                raise _invalid(f"{label} must be a string", name)
            return value.strip()

        def number(name: str, label: str) -> float:
            if name not in body:
                raise _invalid(f"{label} is required", name)
            value = body[name]
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                if isinstance(value, str) and _NUMBER.match(value.strip()):
                    value = float(value)
                else:
                    raise _invalid(f"{label} must be a number", name)
            if not math.isfinite(value) or value < 0:
                raise _invalid(f"{label} must be a positive number", name)
            return value

        def whole(name: str, label: str) -> int:
            if name not in body:
                raise _invalid(f"{label} is required", name)
            value = body[name]
            if isinstance(value, str) and re.match(r"^[0-9]+$", value.strip()):
                return int(value.strip(), 10)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise _invalid(f"{label} must be a whole number", name)
            if isinstance(value, float) and not (math.isfinite(value) and value.is_integer()):
                raise _invalid(f"{label} must be a whole number", name)
            if value < 0:
                raise _invalid(f"{label} must be a whole number", name)
            return int(value)

        route = text("route", "Route")
        if route not in ROUTES:
            raise _invalid("Route must be live or recorded", "route")
        robot = text("robot", "Robot")
        if robot not in ROBOTS:
            raise _invalid("Robot must be booster_k1, unitree_g1 or booster_t1_29dof", "robot")
        session_dir = self._absolute(text("session_dir", "Session folder"), "session_dir", "Session folder")
        if not self._session_dir_allowed(session_dir):
            raise _invalid(SESSION_DIR_LOCATION_MESSAGE, "session_dir")
        height = number("human_height_m", "Height")
        if not 0.8 <= height <= 2.5:
            raise _invalid("Height must be 0.8–2.5 m", "human_height_m")
        mocap_ws = self._absolute(text("mocap_ws", "Runtime folder"), "mocap_ws", "Runtime folder")
        motion = self._absolute(text("motion_bvh", "Motion BVH"), "motion_bvh", "Motion BVH")
        npose = self._absolute(text("npose_bvh", "Neutral-pose BVH"), "npose_bvh", "Neutral-pose BVH")
        offsets = text("offsets_file", "Offsets file")
        if offsets in ("", "none"):
            offsets = ""
        else:
            offsets = self._absolute(offsets, "offsets_file", "Offsets file")
        scale = number("bvh_scale", "BVH scale")
        if scale not in (0.01, 1):
            raise _invalid("BVH scale must be 0.01 (centimetres) or 1 (metres)", "bvh_scale")
        start = whole("start_frame", "First frame")
        end = whole("end_frame", "End frame")
        if end <= start:
            raise _invalid("End frame must be after the first frame", "end_frame")
        frame_step = whole("frame_step", "Frame step")
        if frame_step < 1:
            raise _invalid("Frame step must be a positive whole number", "frame_step")
        npose_start = number("npose_start_s", "Neutral pose start")
        if not 0 <= npose_start <= 100000:
            raise _invalid("Neutral pose start must be 0 or more seconds", "npose_start_s")
        npose_duration = number("npose_duration_s", "Neutral pose duration")
        if not 0.01 <= npose_duration <= 100000:
            raise _invalid("Neutral pose duration must be at least 0.01 s", "npose_duration_s")
        windows_ip = text("windows_xsens_ip", "Xsens PC IP" if self.windows else "Windows Xsens IP")
        if windows_ip and not (usable_ipv4(windows_ip) or self._same_pc(windows_ip)):
            example = "127.0.0.1 (Xsens on this PC) or " if self.windows else ""
            raise _invalid(f"Enter an IPv4 address such as {example}192.168.1.246",
                           "windows_xsens_ip")
        linux_ip = text("linux_xsens_ip", "This PC's IP" if self.windows else "Linux Xsens IP")
        if linux_ip and not usable_ipv4(linux_ip):
            raise _invalid("Enter this PC's IPv4 on the Xsens network", "linux_xsens_ip")
        if linux_ip and self._same_pc(windows_ip):
            raise _invalid("Leave this PC's IP empty when Xsens runs on this PC", "linux_xsens_ip")
        if linux_ip and linux_ip == windows_ip:
            message = ("This PC's IP must differ from the Xsens PC IP" if self.windows
                       else "The Linux IP must differ from the Windows IP")
            raise _invalid(message, "linux_xsens_ip")
        return {
            "route": route,
            "session_dir": session_dir,
            "robot": robot,
            "human_height_m": height,
            "mocap_ws": mocap_ws,
            "motion_bvh": motion,
            "npose_bvh": npose,
            "offsets_file": offsets,
            "bvh_scale": 0.01 if scale == 0.01 else 1,
            "start_frame": start,
            "end_frame": end,
            "frame_step": frame_step,
            "npose_start_s": npose_start,
            "npose_duration_s": npose_duration,
            "windows_xsens_ip": windows_ip,
            "linux_xsens_ip": linux_ip,
        }

    def _same_pc(self, source_ip: str) -> bool:
        """Whether ``source_ip`` means Xsens streams from this PC (Windows installs only)."""

        return self.windows and source_ip == LOOPBACK_SOURCE

    def _session_dir_allowed(self, session_dir: str) -> bool:
        """Whether ``session_dir`` resolves inside home and outside the checkout."""

        if not self._is_absolute(session_dir):
            return False
        resolved = Path(os.path.realpath(session_dir))
        home = Path(os.path.realpath(self.home))
        return (
            resolved != home
            and home in resolved.parents
            and resolved != self.repo_root
            and self.repo_root not in resolved.parents
        )

    def render_session_env(self, values: Mapping[str, Any]) -> str:
        """Return session.env text in new_session.sh's ``export KEY=quoted`` format."""

        ordered = [
            ("SESSION_ROUTE", values["route"]),
            ("SESSION_DIR", values["session_dir"]),
            ("ROBOT", values["robot"]),
            ("HUMAN_HEIGHT_M", _format_number(values["human_height_m"])),
            ("MOCAP_WS", values["mocap_ws"]),
            ("IK_CONFIG", self._ik_config(values["robot"], values["mocap_ws"])),
            ("MOTION_BVH", values["motion_bvh"]),
            ("NPOSE_BVH", values["npose_bvh"]),
            ("OFFSETS_FILE", values["offsets_file"]),
            ("BVH_SCALE", "0.01" if values["bvh_scale"] == 0.01 else "1"),
            ("START_FRAME", str(values["start_frame"])),
            ("END_FRAME", str(values["end_frame"])),
            ("FRAME_STEP", str(values["frame_step"])),
            ("NPOSE_START_S", _format_number(values["npose_start_s"])),
            ("NPOSE_DURATION_S", _format_number(values["npose_duration_s"])),
            ("WINDOWS_XSENS_IP", values["windows_xsens_ip"]),
            ("LINUX_XSENS_IP", values["linux_xsens_ip"]),
            ("XSENS_SOURCE_IP", values["windows_xsens_ip"]),
            ("XSENS_UDP_PORT", XSENS_UDP_PORT),
        ]
        header = _ENV_HEADER.format(created=self._stamp("%Y-%m-%d %H:%M"))
        return header + "".join(f"export {key}={shlex.quote(str(value))}\n" for key, value in ordered)

    def save_session(self, body: Mapping[str, Any]) -> dict[str, Any]:
        """Validate, write ``<session_dir>/session.env`` atomically and link it."""

        if not isinstance(body, Mapping):
            raise _invalid("Session values must be an object", "route")
        replace = body.get("replace", False)
        if not isinstance(replace, bool):
            raise _invalid("replace must be true or false", "replace")
        values = self.validate(body)
        link = self.link_path
        if os.path.lexists(link) and not link.is_symlink() and not self._is_console_copy(link):
            raise SetupError(
                f"{link} exists and is not a link; move it aside and save again",
                code="session_link_conflict",
            )
        session_dir = Path(values["session_dir"])
        env_file = session_dir / "session.env"
        if env_file.exists() and not replace and not self._is_current(env_file):
            raise SetupError(
                f"{self._tilde(self._text(env_file))} exists. Replace it, or choose another folder",
                code="session_exists",
            )
        try:
            session_dir.mkdir(parents=True, exist_ok=True)
            temporary = session_dir / f".session.env.tmp-{os.getpid()}"
            temporary.write_text(self.render_session_env(values), encoding="utf-8")
            os.replace(temporary, env_file)
            link.parent.mkdir(parents=True, exist_ok=True)
            link_temporary = link.parent / f".session.env.link-{os.getpid()}"
            if os.path.lexists(link_temporary):
                link_temporary.unlink()
            if self.windows:
                shutil.copyfile(env_file, link_temporary)
            else:
                os.symlink(env_file, link_temporary)
            os.replace(link_temporary, link)
        except OSError as exc:
            raise SetupError(
                f"Could not save the session: {exc}",
                code="session_write_failed",
                status_code=500,
            ) from exc
        self._configure_monitor()
        return values

    def _is_current(self, env_file: Path) -> bool:
        if self.windows and not self.link_path.is_symlink():
            raw = self._read_env(self.link_path)
            folder = raw.get("SESSION_DIR") if raw else None
            return bool(folder) and Path(folder) / "session.env" == env_file
        try:
            return os.path.realpath(self.link_path) == os.path.realpath(env_file) and (
                self.link_path.exists()
            )
        except OSError:
            return False

    def _is_console_copy(self, path: Path) -> bool:
        """Whether ``path`` is a Windows session.env copy written by this console."""

        if not self.windows:
            return False
        try:
            with path.open("r", encoding="utf-8", errors="replace") as handle:
                first = handle.readline()
        except OSError:
            return False
        return first.startswith("# Mocap session created ") and "operator console" in first

    # ---------------------------------------------------------------- payload

    def files(self, session: Mapping[str, Any] | None) -> list[dict[str, Any]]:
        """Regular files directly in the session folder with a known type."""

        if session is None or not session.get("session_dir"):
            return []
        folder = Path(session["session_dir"])
        listed = []
        try:
            entries = list(os.scandir(folder))
        except OSError:
            return []
        for entry in entries:
            suffix = Path(entry.name).suffix.lower()
            if entry.name.startswith(".") or suffix not in _FILE_KINDS:
                continue
            try:
                if not entry.is_file(follow_symlinks=False):
                    continue
                stat = entry.stat(follow_symlinks=False)
            except OSError:
                continue
            listed.append(
                {
                    "name": entry.name,
                    "size": stat.st_size,
                    "modified": stat.st_mtime,
                    "kind": _FILE_KINDS[suffix],
                }
            )
        return sorted(listed, key=lambda item: item["name"])

    def payload(self) -> dict[str, Any]:
        """The GET /api/v1/setup document."""

        session = self.current_session()
        windows_ip = session["windows_xsens_ip"] if session else ""
        linux_ip = session["linux_xsens_ip"] if session else ""
        reference = session if session is not None else self.builtin_defaults()
        same_pc = self._same_pc(reference["windows_xsens_ip"])
        detected = (
            self._route_source(windows_ip) if windows_ip and not self._same_pc(windows_ip) else None
        )
        display = bool(self._display_available())
        firewall = None
        if windows_ip and linux_ip and not self.windows:
            firewall = (
                f"sudo ufw allow from {windows_ip} to {linux_ip} port {XSENS_UDP_PORT} proto udp"
            )
        runtime_ready = self._runtime_ready(reference["mocap_ws"])
        return {
            "platform": "windows" if self.windows else "linux",
            "repo_root": self._tilde(self._text(self.repo_root)),
            # Lets the UI show paths under the home folder as ~/... (inputs accept both).
            "home": self.home_text,
            # Xsens Humanoid streams to 127.0.0.1 on this (Windows) PC.
            "xsens_on_this_pc": same_pc,
            "session": session,
            "defaults": self.new_session_defaults(session),
            "network": {
                "interfaces": list(self._interfaces()),
                "detected_linux_ip": detected,
            },
            "firewall_command": firewall,
            # Linux only: UFW read without privileges; drives the Allow in firewall button.
            "firewall_state": None if self.windows else self._firewall_state(),
            # Linux checkout console only: step 13 (real K1) and Shut down the console.
            "desktop_tools": not self.windows and not self.app_mode,
            "stream_check_s": stream_check_seconds(),
            "readiness": {
                "live_image": None,
                # On Windows the live viewer runs from the same runtime as recorded jobs.
                "live_runtime": runtime_ready if self.windows else None,
                "recorded_runtime": runtime_ready,
                "motion_bvh": Path(reference["motion_bvh"]).is_file(),
                "npose_bvh": Path(reference["npose_bvh"]).is_file(),
                "display": display,
                "display_hint": None if display else NO_DISPLAY_HINT,
            },
            "files": self.files(session),
            "jobs": self.list_jobs(),
            "app": {"mode": APP_MODE, "version": self.app_version()} if self.app_mode else None,
            "stream": self.stream_snapshot() if self._monitor is not None else None,
            "prefs": self.prefs(),
        }

    # ------------------------------------------------------- stream monitor

    def _stream_target(self, raw: Mapping[str, str] | None = None) -> tuple[str, int]:
        """The monitor's address: loopback when Xsens is on this PC, else all interfaces."""

        if raw is None:
            raw = self.read_session_raw()
        if raw is None:
            source = self.builtin_defaults()["windows_xsens_ip"]
            port_text = XSENS_UDP_PORT
        else:
            source = raw.get("WINDOWS_XSENS_IP", "")
            port_text = raw.get("XSENS_UDP_PORT") or XSENS_UDP_PORT
        try:
            port = int(port_text, 10)
        except ValueError:
            port = int(XSENS_UDP_PORT)
        if not 1 <= port <= 65535:
            port = int(XSENS_UDP_PORT)
        return (LOOPBACK_SOURCE if source == LOOPBACK_SOURCE else "0.0.0.0"), port

    def _configure_monitor(self) -> None:
        if self._monitor is None:
            return
        bind, port = self._stream_target()
        self._monitor.configure(bind=bind, port=port)

    def start_stream_monitor(self) -> None:
        """Start the stream monitor (app mode); called once the console is serving."""

        if self._monitor is None or self._monitor_started:
            return
        self._monitor_started = True
        self._configure_monitor()
        self._monitor.start()

    def stream_snapshot(self) -> dict[str, Any]:
        """GET /api/v1/setup/stream: the monitor's StreamSnapshot (state ``off`` without one)."""

        if self._monitor is None:
            bind, port = self._stream_target()
            return off_snapshot(bind=bind, port=port)
        if self._monitor_started:
            self._configure_monitor()  # follows a hand-edited session.env too
        return self._monitor.snapshot()

    def _release_port(self) -> None:
        if self._monitor is not None:
            self._monitor.pause()

    def _reclaim_port(self) -> None:
        if self._monitor is not None:
            self._monitor.resume()

    # ---------------------------------------------------------------- prefs

    @property
    def prefs_path(self) -> Path | None:
        return self.state_dir / PREFS_FILE if self.state_dir is not None else None

    def prefs(self) -> dict[str, Any]:
        """The saved Prefs; defaults for a missing, unreadable or invalid file."""

        path = self.prefs_path
        with self._prefs_lock:
            if path is None:
                return dict(self._memory_prefs)
            try:
                saved = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                return default_prefs()
        if not isinstance(saved, dict):
            return default_prefs()
        prefs = default_prefs()
        for name in ("record_video", "calibrate_after_s"):
            if name in saved:
                try:
                    prefs = merge_prefs(prefs, {name: saved[name]})
                except SetupError:
                    pass  # an invalid saved value falls back to its default
        return prefs

    def save_prefs(self, changes: Mapping[str, Any]) -> dict[str, Any]:
        """PUT /api/v1/setup/prefs: merge, validate (400) and save atomically."""

        prefs = merge_prefs(self.prefs(), changes)
        path = self.prefs_path
        with self._prefs_lock:
            if path is None:
                self._memory_prefs = prefs
                return dict(prefs)
            try:
                path.parent.mkdir(parents=True, exist_ok=True)
                temporary = path.parent / f".{PREFS_FILE}.tmp-{os.getpid()}-{threading.get_ident()}"
                temporary.write_text(json.dumps(prefs, indent=2) + "\n", encoding="utf-8")
                os.replace(temporary, path)
            except OSError as exc:
                raise SetupError(
                    f"Could not save the preferences: {exc}",
                    code="prefs_write_failed",
                    status_code=500,
                ) from exc
        return dict(prefs)

    def file_path(self, name: str) -> tuple[Path, str]:
        """Resolve a served session file by basename; raise 404 SetupError otherwise."""

        missing = SetupError("No such session file", code="file_not_found", status_code=404)
        if (
            not name
            or name in (".", "..")
            or "/" in name
            or "\\" in name
            or "\x00" in name
            or name.startswith(".")
        ):
            raise missing
        suffix = Path(name).suffix.lower()
        if suffix not in SERVED_MEDIA_TYPES:
            raise missing
        session = self.current_session()
        if not any(item["name"] == name for item in self.files(session)):
            raise missing
        assert session is not None
        path = Path(session["session_dir"]) / name
        if path.is_symlink() or not path.is_file():
            raise missing
        return path, SERVED_MEDIA_TYPES[suffix]

    # ------------------------------------------------------------------- jobs

    def list_jobs(self) -> list[dict[str, Any]]:
        with self._lock:
            jobs = [self._jobs[job_id] for job_id in reversed(self._order[-MAX_LISTED_JOBS:])]
        return [job.summary_dict() for job in jobs]

    def _get_job(self, job_id: str) -> _Job:
        with self._lock:
            job = self._jobs.get(job_id)
        if job is None:
            raise SetupError("No such job", code="job_not_found", status_code=404)
        return job

    def job_log(self, job_id: str, after: int = 0) -> dict[str, Any]:
        job = self._get_job(job_id)
        lines, next_index = job.lines_after(max(0, int(after)))
        return {"job": job.summary_dict(), "lines": lines, "next": next_index}

    def running_job(self) -> _Job | None:
        with self._lock:
            for job in self._jobs.values():
                if job.state == "running":
                    return job
        return None

    def _session_env(self, raw: Mapping[str, str]) -> dict[str, str]:
        env = dict(os.environ)
        env.update({key: value for key, value in raw.items() if key in SESSION_KEYS})
        env["PYTHONUNBUFFERED"] = "1"
        if os.name == "nt":
            # Job output is read as UTF-8; a pipe would otherwise use the ANSI code page.
            env["PYTHONIOENCODING"] = "utf-8"
            # EGL/OSMesa exist only on Linux; MuJoCo raises for them on Windows.
            if env.get("MUJOCO_GL", "").lower() in ("egl", "osmesa"):
                del env["MUJOCO_GL"]
        # Tools that call python3 (live_operator.sh) get the console's own interpreter.
        interpreter_dir = os.path.dirname(sys.executable)
        env["PATH"] = os.pathsep.join(
            part for part in (interpreter_dir, env.get("PATH", "")) if part
        )
        return env

    def _validate_options(self, kind: str, options: Mapping[str, Any]) -> dict[str, Any]:
        def bad(message: str) -> SetupError:
            return SetupError(message, code="invalid_options", status_code=422)

        def flag(name: str) -> bool:
            value = options.get(name, False)
            if not isinstance(value, bool):
                raise bad(f"{name} must be true or false")
            return value

        def choice(name: str, allowed: tuple[str, ...]) -> str:
            value = options.get(name, allowed[0])
            if value not in allowed:
                raise bad(f"{name} must be one of: {', '.join(allowed)}")
            return value

        if kind == "compare":
            calibrate = options.get("calibrate_after_s")
            if calibrate is not None:
                if isinstance(calibrate, bool) or not isinstance(calibrate, (int, float)):
                    raise bad("calibrate_after_s must be a number of seconds or null")
                if not (math.isfinite(calibrate) and 0.5 <= calibrate <= 60):
                    raise bad("calibrate_after_s must be 0.5–60 s")
            return {"calibrate_after_s": calibrate}
        if kind == "live":
            # Omitted options take the saved app preferences (defaults: no video, no timer).
            prefs = self.prefs()
            options = {"calibrate_after_s": prefs["calibrate_after_s"],
                       "record_video": prefs["record_video"], **options}
            calibrate = options.get("calibrate_after_s")
            if calibrate is not None:
                if isinstance(calibrate, bool) or not isinstance(calibrate, (int, float)):
                    raise bad("calibrate_after_s must be a number of seconds or null")
                if not (math.isfinite(calibrate) and 0.5 <= calibrate <= 60):
                    raise bad("calibrate_after_s must be 0.5–60 s")
            return {"calibrate_after_s": calibrate, "record_video": flag("record_video")}
        # Outputs are always overwritten; an ``overwrite`` option is accepted and ignored.
        if kind == "retarget":
            return {"mode": choice("mode", ("preview", "npose"))}
        if kind == "render":
            return {
                "motion": choice("motion", ("preview", "npose")),
                "record_video": flag("record_video"),
            }
        return {}

    def start_job(self, kind: str, options: Mapping[str, Any] | None = None) -> dict[str, Any]:
        """Start one job; at most one job runs at a time."""

        if kind not in JOB_KINDS:
            raise SetupError(f"Unknown job kind: {kind!r}", code="invalid_job", status_code=422)
        if options is None:
            options = {}
        if not isinstance(options, Mapping):
            raise SetupError("options must be an object", code="invalid_options", status_code=422)
        if kind in LINUX_JOB_KINDS and (self.windows or self.app_mode):
            raise SetupError("This step is only available on the Ubuntu PC",
                             code="not_supported", status_code=409)
        options = self._validate_options(kind, options)
        raw = self.read_session_raw()
        if raw is None or not raw.get("SESSION_DIR"):
            raise SetupError("Save the session values first", code="no_session")
        session = self._values_from_env(raw)
        # session.env may have been edited by hand: outputs must stay in an allowed folder.
        if not self._session_dir_allowed(session["session_dir"]):
            raise SetupError(SESSION_DIR_LOCATION_MESSAGE, code="invalid_session_dir")
        if kind in ("acceptance", "live", "compare") and not session["windows_xsens_ip"]:
            raise SetupError(
                "Choose where Xsens runs (this PC or its IP) in the session first"
                if self.windows else "Enter the Windows Xsens IP in the session first",
                code="missing_windows_ip",
            )
        needs_runtime = kind in ("retarget", "render") or (
            kind == "doctor" and session["route"] == "recorded"
        )
        if self.windows and kind == "live":
            needs_runtime = True  # no Docker image: the viewer runs from the runtime venv
        if needs_runtime and not Path(self.runtime_python(session["mocap_ws"])).exists():
            label = "Runtime" if self.windows else "Recorded runtime"
            raise SetupError(
                f"{label} not found in {self._tilde(session['mocap_ws'])}. "
                + self._installer_hint(session["route"]),
                code="recorded_runtime_missing",
            )
        if (kind == "retarget" and options["mode"] == "npose"
                and session["robot"] in CONFIG_ARM_ROBOTS):
            raise SetupError(
                "Booster K1 arms are aligned by its IK config in the preview motion; "
                "an N-pose motion would leave its forearms free. Use Make preview motion.",
                code="npose_not_supported",
            )
        if kind in ("live", "render", "compare") and not self._display_available():
            raise SetupError(NO_DISPLAY_HINT, code="no_display")
        if kind == "k1_install" and self._k1_lock_held(self.k1_lock_path):
            raise SetupError(
                "Close the K1 test window first: the installer changes the Python "
                "environment it runs on.",
                code="k1_window_open",
            )
        self._require_no_running_job()
        if kind in ("acceptance", "live", "compare"):
            # Only after the running-job check: our own job would hold these.
            if self._viewer_running():
                raise SetupError(
                    "A live viewer from an earlier run is still running. Stop it, then try again.",
                    code="viewer_running",
                )
        uses_port = kind in PORT_JOB_KINDS
        if uses_port:
            # The stream monitor lets go of the port before the probe and the job;
            # the job thread takes it back when the job ends.
            self._release_port()
        try:
            if kind in ("acceptance", "live", "compare"):
                port = int(raw.get("XSENS_UDP_PORT") or XSENS_UDP_PORT)
                if self._port_busy(port, self._udp_bind(session)):
                    raise SetupError(
                        f"Another program is using UDP port {port} (a terminal stream check or "
                        "viewer?). Stop it, then try again.",
                        code="udp_port_busy",
                    )
            job = self._create_job(kind, session, raw, options, uses_port)
        except BaseException:
            if uses_port:
                self._reclaim_port()
            raise
        job.thread.start()
        return job.summary_dict()

    def _create_job(self, kind: str, session: dict[str, Any], raw: dict[str, str],
                    options: dict[str, Any], uses_port: bool) -> _Job:
        with self._lock:
            self._require_no_running_job_locked()
            self._counter += 1
            started = self._now()
            job = _Job(
                id=f"j-{self._stamp('%Y%m%d-%H%M%S')}-{self._counter}",
                kind=kind,
                label=self._label(kind, options),
                started_at=started,
                wall_started_at=time.time(),
                session_dir=Path(session["session_dir"]),
            )
            self._jobs[job.id] = job
            self._order.append(job.id)
            job.thread = threading.Thread(
                target=self._execute,
                args=(job, session, raw, options, uses_port),
                name=f"mocap-setup-{job.id}",
                daemon=True,
            )
        return job

    def _port_busy(self, port: int, bind: str) -> bool:
        if self._udp_port_busy is default_udp_port_busy:
            return default_udp_port_busy(port, bind)
        return self._udp_port_busy(port)  # an injected probe takes the port only

    def _require_no_running_job_locked(self) -> None:
        running = next((job for job in self._jobs.values() if job.state == "running"), None)
        if running is not None:
            raise SetupError(
                f"{running.label} is still running; stop it first",
                code="job_running",
                job_id=running.id,
            )

    def _require_no_running_job(self) -> None:
        with self._lock:
            self._require_no_running_job_locked()

    def stop_viewer(self) -> dict[str, Any]:
        """Stop a leftover live viewer container (``docker stop gmr-xsens-live``)."""

        running = self.running_job()
        if running is not None and running.kind in VIEWER_JOB_KINDS:
            raise SetupError(
                f"{running.label} is running from this console; use its Stop button",
                code="job_running",
                job_id=running.id,
            )
        self._stop_container()
        return {"stopped": True}

    def control_job(self, job_id: str, command: Any) -> dict[str, Any]:
        """Send ``command`` (calibrate, toggle_tracking, rearm) to a running live simulation.

        The command is written as one line to the job's stdin (Windows live jobs
        run ``live_g1_sim.py --control-stdin``).  400 for an unknown command,
        409 unless the job is a running live job that still reads its stdin.
        """

        if not isinstance(command, str) or command not in CONTROL_COMMANDS:
            raise SetupError(
                f"command must be one of: {', '.join(CONTROL_COMMANDS)}",
                code="invalid_command",
                status_code=400,
                field="command",
            )
        job = self._get_job(job_id)
        refused = SetupError(
            "Only a running live simulation started from this console accepts commands",
            code="job_not_controllable",
            job_id=job.id,
        )
        with job.lock:
            if job.kind != "live" or job.state != "running" or job.stop_requested:
                raise refused
            control = job.control
            if control is None:
                raise refused
            try:
                control.write((command + "\n").encode("utf-8"))
                control.flush()
            except (OSError, ValueError) as exc:  # BrokenPipeError, or a closed pipe
                job.control = None
                raise SetupError(
                    "The live simulation is no longer reading commands",
                    code="job_not_controllable",
                    job_id=job.id,
                ) from exc
        return {"accepted": True}

    @staticmethod
    def _label(kind: str, options: Mapping[str, Any]) -> str:
        if kind == "retarget":
            return "Make arm-aligned motion" if options["mode"] == "npose" else "Make preview motion"
        if kind == "render":
            action = "Record MP4" if options["record_video"] else "Open viewer"
            return f"{action} ({options['motion']})"
        return {
            "doctor": "System check",
            "acceptance": "Stream check",
            "live": "Live simulation",
            "compare": "G1 + T1 side by side",
            "k1_install": "Install K1 test tools",
            "bvh_check": "Check BVH files",
            "save": "Save report",
        }[kind]

    def _execute(
        self,
        job: _Job,
        session: dict[str, Any],
        raw: dict[str, str],
        options: dict[str, Any],
        uses_port: bool = False,
    ) -> None:
        try:
            self._execute_job(job, session, raw, options)
        finally:
            if uses_port:
                self._reclaim_port()

    def _execute_job(
        self,
        job: _Job,
        session: dict[str, Any],
        raw: dict[str, str],
        options: dict[str, Any],
    ) -> None:
        try:
            (job.session_dir / "logs").mkdir(parents=True, exist_ok=True)
            job.log_handle = open(
                job.session_dir / "logs" / f"{job.id}.log", "a", encoding="utf-8"
            )
        except OSError:
            job.log_handle = None
        passed = False
        exit_code: int | None = None
        try:
            runner = getattr(self, f"_run_{job.kind}")
            exit_code, passed = runner(job, session, raw, options)
        except Exception as exc:  # the job must always finish
            job.append(f"ERROR: {type(exc).__name__}: {exc}")
            exit_code = exit_code if exit_code is not None else 1
            passed = False
        finally:
            with job.lock:
                job.outputs = [path for path in job.outputs if path.exists()]
            if job.stop_requested:
                job.append(STOPPED_SUMMARY)
                summary: str | None = STOPPED_SUMMARY
            else:
                summary = job.compute_summary()
            with job.lock:
                job.exit_code = exit_code
                job.finished_at = self._now()
                job.summary = summary
                job.state = "stopped" if job.stop_requested else "passed" if passed else "failed"
                handle, job.log_handle = job.log_handle, None
                job.process = None
            if handle is not None:
                try:
                    handle.close()
                except OSError:
                    pass

    def _argv(self, kind: str, session: Mapping[str, Any], options: Mapping[str, Any],
              default: Callable[[], list[str]]) -> list[str]:
        override = self.command_overrides.get(kind)
        argv = list(override(session, options)) if override is not None else default()
        if not argv or not all(isinstance(item, str) for item in argv):
            raise ValueError(f"{kind} command must be a non-empty list of strings")
        return argv

    def _run_process(
        self,
        job: _Job,
        argv: list[str],
        env: Mapping[str, str],
        *,
        stdout_path: Path | None = None,
        control: bool = False,
    ) -> int:
        """Run ``argv`` in its own process group, streaming output into the job log.

        ``control=True`` gives the process a stdin pipe that :meth:`control_job`
        writes commands to (a Windows live simulation with ``--control-stdin``).
        """

        job.append("$ " + self._display_command(argv))
        stdout_handle = None
        try:
            if stdout_path is not None:
                stdout_handle = open(stdout_path, "wb")
            with job.lock:
                if job.stop_requested:
                    return 130
                process = subprocess.Popen(
                    argv,
                    cwd=self.repo_root,
                    env=dict(env),
                    stdin=subprocess.PIPE if control else subprocess.DEVNULL,
                    stdout=stdout_handle if stdout_handle is not None else subprocess.PIPE,
                    stderr=subprocess.PIPE if stdout_handle is not None else subprocess.STDOUT,
                    **_new_process_group(),
                )
                job.process = process
                job.control = process.stdin if control else None
        except OSError as exc:
            if stdout_handle is not None:
                stdout_handle.close()
            job.append(f"ERROR: cannot start {argv[0]}: {exc.strerror or exc}")
            return 127
        stream = process.stderr if stdout_handle is not None else process.stdout
        assert stream is not None
        try:
            for raw_line in iter(stream.readline, b""):
                job.append(raw_line.decode("utf-8", errors="replace").rstrip("\n"))
        finally:
            stream.close()
            code = process.wait()
            if stdout_handle is not None:
                stdout_handle.close()
            if control:
                with job.lock:
                    job.control = None
                try:
                    process.stdin.close()
                except (OSError, ValueError):
                    pass
        return code

    def _display_command(self, argv: list[str]) -> str:
        """Shell-quoted command line with home-folder paths shown as ``~/...``."""

        if self.windows:
            shown = [self._tilde(arg.replace("\\", "/")) for arg in argv]
            return subprocess.list2cmdline(shown)
        home = str(self.home).rstrip("/") + "/"
        return " ".join(
            "~/" + shlex.quote(arg[len(home):]) if arg.startswith(home) and len(arg) > len(home)
            else shlex.quote(arg)
            for arg in argv
        )

    # Each runner returns (exit_code, passed).

    def _run_doctor(self, job, session, raw, options) -> tuple[int, bool]:
        def default() -> list[str]:
            doctor = str(self.repo_root / "tools" / "checks" / "setup_doctor.py")
            if session["route"] == "live":
                if self.windows:
                    return [sys.executable, doctor, "live", "--workspace", session["mocap_ws"],
                            "--robot", session["robot"]]
                return [sys.executable, doctor, "live"]
            return [sys.executable, doctor, "recorded", "--workspace", session["mocap_ws"],
                    "--robot", session["robot"]]

        code = self._run_process(job, self._argv("doctor", session, options, default),
                                 self._session_env(raw))
        if not job.stop_requested:
            job.append("PASS: system check is ready" if code == 0
                       else f"FAIL: system check is blocked (exit {code})")
        return code, code == 0

    def _run_acceptance(self, job, session, raw, options) -> tuple[int, bool]:
        output = job.session_dir / f"stream-acceptance-{self._stamp('%Y%m%d-%H%M%S')}.json"
        job.outputs.append(output)

        env = self._session_env(raw)

        def default() -> list[str]:
            if self.windows:
                return self.acceptance_argv(session, env, output)
            return [str(self.repo_root / "tools" / "xsens" / "live_operator.sh"), "acceptance",
                    "--output", str(output)]

        # An override receives the report path as options["output"].
        argv = self._argv("acceptance", session, {**options, "output": str(output)}, default)
        code = self._run_process(job, argv, env)
        verdict = None
        try:
            report = json.loads(output.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            report = None
        if isinstance(report, dict) and "verdict" in report:
            verdict = report.get("verdict")
            body = report.get("body") if isinstance(report.get("body"), dict) else {}
            frames = body.get("valid_datagrams")
            rate = body.get("observed_receive_hz")
            failures = [str(item) for item in report.get("failures") or []]
            if verdict == "PASS":
                rate_text = f" at {rate:.1f} Hz" if isinstance(rate, (int, float)) else ""
                job.append(f"PASS: {frames} body frames{rate_text}")
            else:
                job.append("FAIL: " + (failures[0] if failures else f"verdict {verdict}"))
        passed = code == 0 and (verdict is None or verdict == "PASS")
        return code, passed

    def _udp_bind(self, session: Mapping[str, Any]) -> str:
        """127.0.0.1 when Xsens is on this PC (no Windows Firewall prompt), else all interfaces."""

        return LOOPBACK_SOURCE if self._same_pc(session["windows_xsens_ip"]) else "0.0.0.0"

    def _live_video(self, session: Mapping[str, Any], env: Mapping[str, str]) -> Path:
        """Where the live viewer records (docker/gmr-live/run.sh mounts this folder).

        In app mode the project folder is read-only: the viewer records straight
        into the session folder under the name the console would copy it to.
        """

        if self.app_mode and self.windows:
            return Path(session["session_dir"]) / (
                f"{session['robot']}-live-{self._stamp('%Y%m%d-%H%M%S')}.mp4"
            )
        runtime_root = Path(env.get("GMR_LIVE_RUNTIME_ROOT")
                            or self.repo_root / "artifacts" / "xsens-live")
        short = LIVE_ROBOTS[session["robot"]].label.lower()
        return runtime_root / f"{short}-live.mp4"

    def acceptance_argv(self, session: Mapping[str, Any], env: Mapping[str, str],
                        output: Path) -> list[str]:
        """The Windows stream check: live_operator.sh acceptance without bash."""

        port = env.get("XSENS_UDP_PORT") or XSENS_UDP_PORT
        return [
            sys.executable, str(self.repo_root / "tools" / "xsens" / "stream_acceptance.py"),
            "--bind", self._udp_bind(session), "--port", port,
            "--expected-source", session["windows_xsens_ip"],
            "--duration", str(stream_check_seconds(env)),
            *ACCEPTANCE_LIMITS,
            "--output", str(output),
        ]

    def live_argv(self, session: Mapping[str, Any], options: Mapping[str, Any],
                  env: Mapping[str, str], *, video: Path | None = None) -> list[str]:
        """The live viewer command: Docker on Linux, the runtime venv on Windows.

        On Windows the viewer takes control commands on stdin and binds
        127.0.0.1 when Xsens is on this PC.
        """

        if self.windows:
            argv = [
                self.runtime_python(session["mocap_ws"]),
                str(self.repo_root / "tools" / "xsens" / "live_g1_sim.py"),
                "--bind", self._udp_bind(session),
                "--port", env.get("XSENS_UDP_PORT") or XSENS_UDP_PORT,
                "--source-ip", session["windows_xsens_ip"],
                "--video-path", str(video if video is not None else self._live_video(session, env)),
                "--control-stdin",
            ]
        else:
            argv = [str(self.repo_root / "tools" / "xsens" / "live_operator.sh"), "gmr"]
        argv += ["--robot", session["robot"],
                 "--human-height", _format_number(session["human_height_m"])]
        if options["calibrate_after_s"] is not None:
            argv += ["--calibrate-after", _format_number(options["calibrate_after_s"])]
        if options["record_video"]:
            argv.append("--record-video")
        return argv

    def _run_live(self, job, session, raw, options) -> tuple[int, bool]:
        env = self._session_env(raw)
        video = self._live_video(session, env)
        if self.windows and options["record_video"]:
            video.parent.mkdir(parents=True, exist_ok=True)
        code = self._run_process(
            job,
            self._argv("live", session, options,
                       lambda: self.live_argv(session, options, env, video=video)),
            env,
            control=self.windows,
        )
        if options["record_video"]:
            try:
                fresh = video.is_file() and video.stat().st_mtime >= job.wall_started_at
            except OSError:
                fresh = False
            if fresh and video.parent == job.session_dir:
                job.outputs.append(video)  # app mode: recorded straight into the session
                job.append(f"Saved video {video.name} in the session folder")
            elif fresh:
                target = job.session_dir / (
                    f"{session['robot']}-live-{self._stamp('%Y%m%d-%H%M%S')}.mp4"
                )
                shutil.copy2(video, target)
                job.outputs.append(target)
                job.append(f"Saved video {target.name} in the session folder")
            else:
                job.append("No new live video was recorded")
        return code, code == 0

    def compare_argv(self, session: Mapping[str, Any], options: Mapping[str, Any]) -> list[str]:
        """G1 and T1 side by side: docker/gmr-live/run-compare.sh (container gmr-xsens-live)."""

        argv = [str(self.repo_root / "docker" / "gmr-live" / "run-compare.sh"),
                "--human-height", _format_number(session["human_height_m"])]
        if options["calibrate_after_s"] is not None:
            argv += ["--calibrate-after", _format_number(options["calibrate_after_s"])]
        return argv

    def _run_compare(self, job, session, raw, options) -> tuple[int, bool]:
        env = self._session_env(raw)
        env["XSENS_SOURCE_IP"] = session["windows_xsens_ip"]
        code = self._run_process(
            job, self._argv("compare", session, options, lambda: self.compare_argv(session, options)),
            env,
        )
        return code, code == 0

    def k1_install_argv(self, session: Mapping[str, Any]) -> list[str]:
        """The recorded runtime (with the K1 model) and the Booster SDK, without sudo."""

        return ["bash", str(self.repo_root / "tools" / "setup" / "install.sh"), "recorded",
                "--with-booster", "--workspace", session["mocap_ws"], "--robot", "booster_k1"]

    def _run_k1_install(self, job, session, raw, options) -> tuple[int, bool]:
        # The installer's own environment: no session values, no console interpreter first on PATH.
        env = {**os.environ, "PYTHONUNBUFFERED": "1"}
        code = self._run_process(
            job, self._argv("k1_install", session, options, lambda: self.k1_install_argv(session)),
            env,
        )
        if job.stop_requested:
            return code, False
        if code != 0:
            job.append(f"FAIL: the K1 test tools did not install (exit {code}). Fix the error "
                       "above, then click Install K1 test tools again; finished parts are skipped.")
            return code, False
        shortcut = self._run_process(
            job, ["bash", str(self.repo_root / "tools" / "setup" / "open_console.sh"),
                  "--install-shortcut"], env)
        job.append("PASS: K1 test tools installed"
                   + ("; menu entries updated" if shortcut == 0
                      else "; the menu entries could not be updated"))
        return 0, True

    def _run_bvh_check(self, job, session, raw, options) -> tuple[int, bool]:
        if "bvh_check" in self.command_overrides:
            code = self._run_process(job, self._argv("bvh_check", session, options, list),
                                     self._session_env(raw))
            return code, code == 0
        output = job.session_dir / "bvh-sha256.txt"
        results = {}
        for label, name in (("Motion", "motion_bvh"), ("Neutral pose", "npose_bvh")):
            path = Path(session[name])
            if job.stop_requested:
                return 130, False
            if not path.is_file():
                job.append(f"ERROR: {label} BVH not found: {path}")
                return 1, False
            frames, frame_time = _bvh_header(path)
            if frames is None or frame_time is None:
                job.append(f"ERROR: {label} BVH has no Frames:/Frame Time: lines: {path}")
                return 1, False
            rate = 1.0 / frame_time if frame_time > 0 else float("nan")
            job.append(f"{label}: {path.name}: {frames} frames, Frame Time {frame_time:g} "
                       f"({rate:.1f} Hz)")
            results[name] = (path, frames, _sha256(path))
        lines = [f"{digest}  {path}\n" for path, _frames, digest in results.values()]
        output.write_text("".join(lines), encoding="utf-8")
        job.outputs.append(output)
        job.append(f"Wrote {output.name}")
        motion_frames = results["motion_bvh"][1]
        if motion_frames < session["end_frame"]:
            job.append(f"FAIL: the motion take has {motion_frames} frames but the end frame is "
                       f"{session['end_frame']}; lower the end frame in the session")
            return 1, False
        job.append(f"PASS: both BVH files are readable ({motion_frames} motion frames)")
        return 0, True

    def _run_retarget(self, job, session, raw, options) -> tuple[int, bool]:
        robot = session["robot"]
        folder = session["session_dir"]
        save_path = f"{folder}/{robot}-{options['mode']}.pkl"
        job.outputs.append(Path(save_path))
        if options["mode"] == "npose":
            job.outputs.append(Path(f"{folder}/{robot}-arm-calibration.json"))

        def default() -> list[str]:
            mocap_ws = session["mocap_ws"]
            argv = [
                self.runtime_python(mocap_ws),
                str(self.repo_root / "tools" / "retarget" / "retarget_robot_recorded.py"),
                "--gmr-root", f"{mocap_ws}/GMR", "--robot", robot,
                "--ik-config", self._ik_config(robot, mocap_ws),
                "--bvh-file", session["motion_bvh"], "--save-path", save_path,
                "--scale", raw.get("BVH_SCALE") or _format_number(session["bvh_scale"]),
                "--human-height", _format_number(session["human_height_m"]),
                "--start", str(session["start_frame"]), "--end", str(session["end_frame"]),
                "--frame-step", str(session["frame_step"]),
                "--allow-zero-offsets",
            ]
            if options["mode"] == "npose":
                argv += [
                    "--arm-mode", "npose",
                    "--calibration-bvh-file", session["npose_bvh"],
                    "--calibration-start-seconds", _format_number(session["npose_start_s"]),
                    "--calibration-seconds", _format_number(session["npose_duration_s"]),
                    "--calibration-output", f"{folder}/{robot}-arm-calibration.json",
                    "--accept-reviewed-calibration-window",
                ]
            elif robot in CONFIG_ARM_ROBOTS:
                argv += ["--arm-mode", "config", "--arm-abduction",
                         _format_number(LIVE_ROBOTS[robot].arm_abduction_deg)]
            else:
                argv += ["--arm-mode", "position-only"]
            argv.append("--overwrite")
            return argv

        code = self._run_process(job, self._argv("retarget", session, options, default),
                                 self._session_env(raw))
        # The tool ends with a multi-line JSON summary; add a verdict line for the summary.
        passed = code == 0 and Path(save_path).is_file()
        # On a non-zero exit the tool's own ERROR line stays the summary.
        if code == 0 and not job.stop_requested:
            job.append(f"PASS: made {Path(save_path).name}" if passed
                       else f"FAIL: {Path(save_path).name} was not written")
        return code, passed

    def _run_render(self, job, session, raw, options) -> tuple[int, bool]:
        robot = session["robot"]
        folder = session["session_dir"]
        video = f"{folder}/{robot}-{options['motion']}.mp4"
        if options["record_video"]:
            job.outputs.append(Path(video))

        def default() -> list[str]:
            mocap_ws = session["mocap_ws"]
            argv = [
                self.runtime_python(mocap_ws),
                str(self.repo_root / "tools" / "retarget" / "render_robot_motion.py"),
                "--gmr-root", f"{mocap_ws}/GMR", "--robot", robot,
                "--motion", f"{folder}/{robot}-{options['motion']}.pkl", "--loops", "1",
            ]
            if options["record_video"]:
                argv += ["--record-video", "--video-path", video, "--overwrite-video"]
            return argv

        code = self._run_process(job, self._argv("render", session, options, default),
                                 self._session_env(raw))
        return code, code == 0

    def _run_save(self, job, session, raw, options) -> tuple[int, bool]:
        revision_file = job.session_dir / "project-revision.txt"
        report_file = job.session_dir / "core-doctor.json"
        job.outputs += [revision_file, report_file]
        failure: str | None = None
        if self.app_mode and not (self.repo_root / ".git").exists():
            # The installed app has no .git (and maybe no git); never ask a parent repository.
            failure, code = "the installed app has no Git checkout", 1
        else:
            job.append("$ git rev-parse HEAD")
            try:
                result = subprocess.run(
                    ["git", "rev-parse", "HEAD"],
                    cwd=self.repo_root,
                    capture_output=True,
                    text=True,
                    timeout=30,
                    check=False,
                )
            except (OSError, subprocess.SubprocessError) as exc:
                failure, code = str(exc), 1
            else:
                revision = result.stdout.strip()
                if result.returncode != 0 or not revision:
                    failure = strip_ansi(result.stderr.strip())
                    code = result.returncode or 1
        if failure is not None:
            # A GitHub "Download ZIP" has no .git; its .git_archival.txt names the commit.
            archived = self.archive_revision()
            if archived is not None:
                revision = archived
                job.append(f"No Git checkout; revision {revision} from .git_archival.txt")
            elif self.app_mode:
                revision = f"unknown (mocap-g1 {self.app_version()})"
                job.append(f"No Git checkout or .git_archival.txt; recording {revision}")
            else:
                job.append(f"ERROR: git rev-parse failed: {failure}")
                return code, False
        revision_file.write_text(revision + "\n", encoding="utf-8")
        job.append(f"Project revision {revision} -> {revision_file.name}")

        def default() -> list[str]:
            return [sys.executable, str(self.repo_root / "tools" / "checks" / "setup_doctor.py"),
                    "core", "--with-operator", "--json"]

        partial = job.session_dir / ".core-doctor.json.partial"
        code = self._run_process(job, self._argv("save", session, options, default),
                                 self._session_env(raw), stdout_path=partial)
        if job.stop_requested:
            partial.unlink(missing_ok=True)
            return code, False
        if partial.exists() and partial.stat().st_size > 0:
            os.replace(partial, report_file)
        else:
            partial.unlink(missing_ok=True)
        try:
            ready = bool(json.loads(report_file.read_text(encoding="utf-8")).get("ok"))
        except (OSError, ValueError, AttributeError):
            ready = False
        if code == 0:
            job.append(f"PASS: saved {revision_file.name} and {report_file.name}")
        else:
            state = "blocked" if not ready else f"exit {code}"
            job.append(f"FAIL: core check is {state}; saved {report_file.name}")
        return code, code == 0

    def app_version(self) -> str:
        """The installed build's ``<project>/VERSION`` (written by the app build), else the package version."""

        try:
            text = (self.repo_root / "VERSION").read_text(encoding="utf-8").strip()
        except OSError:
            text = ""
        first = text.splitlines()[0].strip() if text else ""
        if first and len(first) <= 100 and not _CONTROL.search(first):
            return first
        return service_version()

    def archive_revision(self) -> str | None:
        """The commit recorded in ``.git_archival.txt`` by ``git archive``, or ``None``."""

        try:
            text = (self.repo_root / ".git_archival.txt").read_text(encoding="utf-8")
        except OSError:
            return None
        match = _ARCHIVE_REVISION.search(text)
        return match.group(1) if match else None

    # ------------------------------------------------------------------- stop

    def stop_job(self, job_id: str, *, wait: bool = False) -> dict[str, Any]:
        """Request a stop: SIGINT, then SIGTERM, then SIGKILL the job's process group.

        On Windows: CTRL_BREAK (the job scripts treat SIGBREAK like SIGINT),
        then ``taskkill /T /F`` for the process tree.

        The termination sequence runs on a background thread and the summary
        is returned at once (its state may still be ``running``; it becomes
        ``stopped`` when the job thread finishes).  ``wait=True`` blocks,
        bounded, until the job has finished.
        """

        job = self._get_job(job_id)
        with job.lock:
            if job.state != "running":
                return job.summary_unlocked()
            job.stop_requested = True
            process = job.process
            if process is not None and job.terminator is None:
                job.terminator = threading.Thread(
                    target=self._terminate,
                    args=(job, process),
                    name=f"mocap-setup-stop-{job.id}",
                    daemon=True,
                )
                job.terminator.start()
            terminator = job.terminator
        if wait:
            # Worst case: SIGINT, container stop, SIGTERM and SIGKILL waits.
            deadline = time.monotonic() + self.stop_grace_s * 4 + 25
            for thread in (terminator, job.thread):
                if thread is not None:
                    thread.join(timeout=max(0.0, deadline - time.monotonic()))
        return job.summary_dict()

    def _terminate(self, job: _Job, process: subprocess.Popen) -> None:
        if os.name == "nt":
            self._terminate_windows(job, process)
            return

        def signal_group(signum: int) -> None:
            try:
                os.killpg(process.pid, signum)
            except (ProcessLookupError, PermissionError):
                pass

        def exited(timeout: float) -> bool:
            try:
                process.wait(timeout=timeout)
                return True
            except subprocess.TimeoutExpired:
                return False

        if process.poll() is not None:
            return
        signal_group(signal.SIGINT)
        if exited(self.stop_grace_s):
            return
        if job.kind in VIEWER_JOB_KINDS:
            self._stop_container()
            if exited(self.stop_grace_s):
                return
        signal_group(signal.SIGTERM)
        if exited(self.stop_grace_s):
            return
        signal_group(signal.SIGKILL)
        exited(self.stop_grace_s)

    def _terminate_windows(self, job: _Job, process: subprocess.Popen) -> None:
        """CTRL_BREAK the job's process group, then kill its process tree.

        CTRL_BREAK reaches the job only through a console it shares with this
        server, which is why the Windows launcher gives the console a hidden one.
        A venv's ``Scripts\\python.exe`` is a launcher that runs the real
        interpreter as its child, so the fallback is ``taskkill /T`` while the
        launcher is still alive: terminating the launcher alone would orphan the
        job, which would keep the UDP port and the output pipe.
        """

        def exited(timeout: float) -> bool:
            try:
                process.wait(timeout=timeout)
                return True
            except subprocess.TimeoutExpired:
                return False

        if process.poll() is not None:
            return
        try:
            os.kill(process.pid, signal.CTRL_BREAK_EVENT)  # type: ignore[attr-defined]
        except OSError:
            pass
        else:
            if exited(self.stop_grace_s):
                return
        try:
            subprocess.run(
                ["taskkill", "/PID", str(process.pid), "/T", "/F"],
                capture_output=True,
                timeout=20,
                check=False,
            )
        except (OSError, subprocess.SubprocessError):
            pass
        if exited(self.stop_grace_s):
            return
        try:
            process.kill()
        except OSError:
            pass
        exited(self.stop_grace_s)

    # ------------------------------------------- K1 tools, password prompts, shutdown

    @property
    def k1_lock_path(self) -> Path:
        return self.user_state_dir / K1_LOCK_NAME

    @property
    def k1_log_path(self) -> Path:
        return self.user_state_dir / K1_LOG_NAME

    def _require_desktop_tools(self) -> None:
        if self.windows or self.app_mode:
            raise SetupError("Only available in the Setup console on the Ubuntu PC",
                             code="not_supported")

    def _session_or_defaults(self) -> dict[str, Any]:
        raw = self.read_session_raw()
        if raw is not None and raw.get("SESSION_DIR"):
            return self._values_from_env(raw)
        return self.builtin_defaults()

    def k1_status(self) -> dict[str, Any]:
        """GET /api/v1/setup/k1: whether the K1 test tools are installed and the window is open.

        The SDK and Tk are checked with the project interpreter (the K1 window's);
        GMR and the K1 model in the session's runtime folder (default ~/mocap-runtime).
        """

        if self.windows or self.app_mode:
            return {"available": False}
        mocap_ws = self._session_or_defaults()["mocap_ws"]
        python = self.repo_root / ".venv" / "bin" / "python"
        probe = self._probe_k1(str(python)) if python.exists() else {"sdk_version": None, "tk": False}
        model = Path(mocap_ws).joinpath(*K1_MODEL)
        runtime = self._runtime_ready(mocap_ws)
        model_found = model.is_file()
        sdk = probe.get("sdk_version")
        tk = probe.get("tk") is True
        return {
            "available": True,
            "sdk_version": sdk if isinstance(sdk, str) else None,
            "tk": tk,
            "runtime": runtime,
            "runtime_dir": self._tilde(mocap_ws),
            "model": self._tilde(str(model)),
            "model_found": model_found,
            "installed": bool(sdk) and tk and runtime and model_found,
            "window_open": bool(self._k1_lock_held(self.k1_lock_path)),
            "display": bool(self._display_available()),
            "log": self._tilde(str(self.k1_log_path)),
        }

    def open_k1_window(self) -> dict[str, Any]:
        """Start tools/control/k1_commission_gui.py detached on this desktop.

        The window takes the K1 lock itself; when the lock is held it exits 3
        and this reports ``already_open`` (a window is open, or an install runs).
        """

        self._require_desktop_tools()
        if not self._display_available():
            raise SetupError(NO_DISPLAY_HINT, code="no_display")
        python = self.repo_root / ".venv" / "bin" / "python"
        gui = self.repo_root / "tools" / "control" / "k1_commission_gui.py"
        if not python.exists() or not gui.is_file():
            raise SetupError("The K1 test tools are not installed: click Install K1 test tools",
                             code="k1_not_installed")
        try:
            code = self._spawn_k1([str(python), str(gui)], self.repo_root, dict(os.environ),
                                  self.k1_log_path, K1_OPEN_WAIT_S)
        except OSError as exc:
            return {"outcome": "failed", "message": f"The K1 test window could not start: {exc}"}
        if code is None:
            return {"outcome": "opened", "message": "The K1 test window is open on this desktop."}
        if code == K1_ALREADY_OPEN_EXIT:
            return {"outcome": "already_open",
                    "message": "The K1 test window is already open, or an install is running."}
        return {"outcome": "failed",
                "message": f"The K1 test window closed at once (exit {code}). "
                           f"See {self._tilde(str(self.k1_log_path))}."}

    def privileged_argv(self, action: str) -> list[str]:
        """The fixed pkexec argv for ``action``; IPs come from the saved session, validated."""

        if action == "install_tk":
            return ["pkexec", "/usr/bin/apt-get", "install", "-y", "python3-tk"]
        if action != "firewall":
            raise SetupError(f"action must be one of: {', '.join(PRIVILEGED_ACTIONS)}",
                             code="invalid_action", status_code=422, field="action")
        session = self.current_session()
        addresses = []
        for name in ("windows_xsens_ip", "linux_xsens_ip"):
            value = session.get(name, "") if session else ""
            try:
                address = ipaddress.IPv4Address(value)
            except ValueError:
                raise SetupError("Save both IP addresses in step 03 first",
                                 code="no_firewall_rule") from None
            if address.is_loopback or address.is_unspecified or address.is_multicast:
                raise SetupError("Both IP addresses must be real network addresses",
                                 code="no_firewall_rule")
            addresses.append(str(address))
        return ["pkexec", UFW, "allow", "from", addresses[0], "to", addresses[1],
                "port", XSENS_UDP_PORT, "proto", "udp"]

    def run_privileged(self, action: Any) -> dict[str, Any]:
        """Run one fixed admin command behind the desktop password prompt (pkexec).

        Blocks until the prompt is answered (the API calls it in a worker
        thread).  Outcomes: ``done`` (exit 0), ``cancelled`` (126, the prompt was
        dismissed) or ``unavailable`` (no agent, not authorised, or the command
        failed): the UI then keeps the Copy fallback for a terminal.
        """

        self._require_desktop_tools()
        if not isinstance(action, str):
            raise SetupError(f"action must be one of: {', '.join(PRIVILEGED_ACTIONS)}",
                             code="invalid_action", status_code=422, field="action")
        argv = self.privileged_argv(action)
        if not self._display_available():
            raise SetupError("No desktop display for the password prompt. Use Copy and a terminal.",
                             code="no_display")
        if not self._privileged_lock.acquire(blocking=False):
            raise SetupError("A password prompt is already open", code="prompt_open")
        try:
            code = self._run_privileged(argv)
        finally:
            self._privileged_lock.release()
        what = "Firewall rule added" if action == "firewall" else "Tk installed"
        if code == 0:
            return {"action": action, "outcome": "done", "exit_code": 0, "message": what + "."}
        if code == PKEXEC_CANCELLED:
            return {"action": action, "outcome": "cancelled", "exit_code": code,
                    "message": "Cancelled: nothing was changed."}
        return {"action": action, "outcome": "unavailable", "exit_code": code,
                "message": f"Could not ask for your password, or the command failed (exit {code}). "
                           "Use Copy and run the command in a terminal."}

    def request_shutdown(self) -> dict[str, Any]:
        """Shut down this console: running jobs stop first (lifespan), the K1 window stays open."""

        self._require_desktop_tools()
        return {"shutting_down": True}

    def shutdown_console(self) -> None:
        """Called by the API after the shutdown response has been sent."""

        pid_file = self.user_state_dir / "setup-console.pid"
        try:
            # open_console.sh's pid file, only when it names this process.
            if pid_file.read_text(encoding="utf-8").strip() == str(os.getpid()):
                pid_file.unlink()
        except OSError:
            pass
        self._shutdown_console()

    def shutdown(self) -> None:
        """Stop any running job and the stream monitor; called on console shutdown."""

        job = self.running_job()
        if job is not None:
            self.stop_job(job.id, wait=True)
        if self._monitor is not None:
            self._monitor.close()


def _new_process_group() -> dict[str, Any]:
    """Popen arguments that put a job in its own process group on this OS."""

    if os.name == "nt":
        return {"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP}  # type: ignore[attr-defined]
    return {"start_new_session": True}


def stream_check_seconds(environ: Mapping[str, str] | None = None) -> int:
    """The stream-check duration live_operator.sh will use (XSENS_DURATION_S, default 30)."""

    environ = os.environ if environ is None else environ
    try:
        value = int(environ.get("XSENS_DURATION_S", ""), 10)
    except ValueError:
        return DEFAULT_STREAM_CHECK_S
    return value if value > 0 else DEFAULT_STREAM_CHECK_S


def _bvh_header(path: Path) -> tuple[int | None, float | None]:
    frames = frame_time = None
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for index, line in enumerate(handle):
            stripped = line.strip()
            if stripped.startswith("Frames:"):
                try:
                    frames = int(stripped.split(":", 1)[1].strip())
                except ValueError:
                    return None, None
            elif stripped.startswith("Frame Time:"):
                try:
                    frame_time = float(stripped.split(":", 1)[1].strip())
                except ValueError:
                    return None, None
                break
            if index > 100000:
                break
    return frames, frame_time


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def require_setup_mode(*, playback: bool, simulation: bool) -> None:
    """Refuse the workbench in recorded playback or simulation review."""

    if playback or simulation:
        raise ValueError(
            "--setup cannot be combined with --playback or --simulation; use it with "
            "the default console or --demo"
        )


__all__ = [
    "CONTROL_COMMANDS",
    "JOB_KINDS",
    "LOOPBACK_SOURCE",
    "SESSION_KEYS",
    "SetupError",
    "SetupWorkbench",
    "default_prefs",
    "merge_prefs",
    "parse_live_status",
    "parse_session_env",
    "is_windows",
    "require_setup_mode",
    "strip_ansi",
    "usable_ipv4",
]
