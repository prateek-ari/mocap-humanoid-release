"""Setup workbench: session values and the engineer setup-guide steps as jobs.

The workbench backs the operator console's Setup workspace.  It reads and
writes ``session.env`` in exactly the format ``tools/setup/new_session.sh``
writes, and runs the guide's Linux steps (system check, stream check, live
simulation, BVH check, retarget, render, save) as background jobs.

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

from .core import OperatorError


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
ROBOTS = ("unitree_g1", "booster_t1_29dof")
XSENS_UDP_PORT = "5555"
JOB_KINDS = ("doctor", "acceptance", "live", "bvh_check", "retarget", "render", "save")
JOB_STATES = ("running", "passed", "failed", "stopped")
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
STOPPED_SUMMARY = "Stopped by the operator."
_LOCAL_DISPLAY = re.compile(r"(?:(?:unix)?/)?:(\d+)(?:\.\d+)?")
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


def usable_ipv4(value: str) -> bool:
    """Mirror ``ipv4`` in new_session.sh: dotted quad, not 0/127/multicast."""

    if not _IPV4.match(value):
        return False
    parts = [int(part, 10) for part in value.split(".")]
    if any(part > 255 for part in parts):
        return False
    return parts[0] != 0 and parts[0] != 127 and parts[0] < 224


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


def _tilde(path: str, home: Path) -> str:
    home_text = str(home)
    if path == home_text:
        return "~"
    if path.startswith(home_text + "/"):
        return "~/" + path[len(home_text) + 1 :]
    return path


# Container and VM bridges never carry the Xsens stream; listing them only confuses the choice.
_VIRTUAL_INTERFACE = re.compile(r"^(docker\d*|br-[0-9a-f]+|virbr\d+|veth.*|lxc.*|lxdbr\d+|cni\d+|flannel.*|tailscale\d*|zt.*)$")


def default_interfaces() -> list[dict[str, str]]:
    """Return physical IPv4 interfaces from ``ip -j -4 addr`` (no loopback or bridges)."""

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


def default_udp_port_busy(port: int) -> bool:
    """Whether another program has bound UDP ``port`` (bind without SO_REUSEADDR)."""

    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.bind(("0.0.0.0", port))
    except OSError as exc:
        return exc.errno == errno.EADDRINUSE
    finally:
        probe.close()
    return False


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
    lock: threading.Lock = field(default_factory=threading.Lock)

    def append(self, text: str) -> None:
        for raw in text.replace("\r\n", "\n").split("\n"):
            # A progress bar redraws with \r: keep its latest state only.
            segments = [segment for segment in raw.split("\r") if segment.strip()]
            line = strip_ansi(segments[-1] if segments else "").rstrip()
            with self.lock:
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
    ) -> None:
        self.repo_root = Path(repo_root).expanduser().resolve()
        if not (self.repo_root / "tools" / "xsens" / "live_operator.sh").is_file():
            raise ValueError(
                f"{self.repo_root} is not a mocap_humanoid source checkout "
                "(tools/xsens/live_operator.sh is missing); the Setup workspace "
                "needs the checkout's tools"
            )
        self.home = Path(home) if home is not None else Path.home()
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

    # ------------------------------------------------------------------ paths

    @property
    def link_path(self) -> Path:
        return self.repo_root / "session.env"

    def _stamp(self, fmt: str) -> str:
        return time.strftime(fmt, time.localtime(self._now()))

    def _absolute(self, value: str, field_name: str, label: str) -> str:
        if value == "~":
            value = str(self.home)
        elif value.startswith("~/"):
            value = f"{self.home}/{value[2:]}"
        if not value.startswith("/"):
            raise _invalid(f"{label}: use an absolute path (or ~/...)", field_name)
        if _CONTROL.search(value) or "\n" in value or "\t" in value:
            raise _invalid(f"{label} must not contain control characters", field_name)
        return value

    def _ik_config(self, robot: str, mocap_ws: str) -> str:
        if robot == "booster_t1_29dof":
            return str(self.repo_root / "retarget_configs" / "bvh_xsens_to_booster_t1_29dof.json")
        return f"{mocap_ws}/GMR/general_motion_retargeting/ik_configs/bvh_xsens_to_g1.json"

    # ------------------------------------------------------------- session I/O

    def builtin_defaults(self) -> dict[str, Any]:
        home = str(self.home)
        return {
            "route": "live",
            "session_dir": "",
            "robot": "unitree_g1",
            "human_height_m": 1.75,
            "mocap_ws": f"{home}/mocap-runtime",
            "motion_bvh": f"{home}/mocap-data/motion.bvh",
            "npose_bvh": f"{home}/mocap-data/neutral.bvh",
            "offsets_file": "",
            "bvh_scale": 0.01,
            "start_frame": 0,
            "end_frame": 600,
            "frame_step": 1,
            "npose_start_s": 2.0,
            "npose_duration_s": 1.0,
            "windows_xsens_ip": "",
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
        """Return the raw assignments of ``<repo>/session.env``, or ``None``."""

        try:
            text = self.link_path.read_text(encoding="utf-8", errors="replace")
        except (FileNotFoundError, NotADirectoryError, IsADirectoryError):
            return None
        except OSError:
            return None
        return parse_session_env(text)

    def current_session(self) -> dict[str, Any] | None:
        raw = self.read_session_raw()
        if raw is None:
            return None
        return self._values_from_env(raw)

    def _fresh_session_dir(self) -> str:
        base = f"{self.home}/mocap-sessions/{self._stamp('%Y-%m-%d-%H%M')}"
        candidate, suffix = base, 1
        while os.path.lexists(candidate):
            suffix += 1
            candidate = f"{base}-{suffix}"
        return candidate

    def new_session_defaults(self, current: Mapping[str, Any] | None = None) -> dict[str, Any]:
        """Prefill a new session: previous values in a fresh dated folder."""

        values = dict(current) if current is not None else self.builtin_defaults()
        values["session_dir"] = self._fresh_session_dir()
        if values.get("windows_xsens_ip"):
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
            raise _invalid("Robot must be unitree_g1 or booster_t1_29dof", "robot")
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
        windows_ip = text("windows_xsens_ip", "Windows Xsens IP")
        if windows_ip and not usable_ipv4(windows_ip):
            raise _invalid("Enter an IPv4 address such as 192.168.1.246", "windows_xsens_ip")
        linux_ip = text("linux_xsens_ip", "Linux Xsens IP")
        if linux_ip and not usable_ipv4(linux_ip):
            raise _invalid("Enter this PC's IPv4 on the Xsens network", "linux_xsens_ip")
        if linux_ip and linux_ip == windows_ip:
            raise _invalid("The Linux IP must differ from the Windows IP", "linux_xsens_ip")
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

    def _session_dir_allowed(self, session_dir: str) -> bool:
        """Whether ``session_dir`` resolves inside home and outside the checkout."""

        if not session_dir.startswith("/"):
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
        if os.path.lexists(link) and not link.is_symlink():
            raise SetupError(
                f"{link} exists and is not a link; move it aside and save again",
                code="session_link_conflict",
            )
        session_dir = Path(values["session_dir"])
        env_file = session_dir / "session.env"
        if env_file.exists() and not replace and not self._is_current(env_file):
            raise SetupError(
                f"{_tilde(str(env_file), self.home)} exists. Replace it, or choose another folder",
                code="session_exists",
            )
        try:
            session_dir.mkdir(parents=True, exist_ok=True)
            temporary = session_dir / f".session.env.tmp-{os.getpid()}"
            temporary.write_text(self.render_session_env(values), encoding="utf-8")
            os.replace(temporary, env_file)
            link_temporary = self.repo_root / f".session.env.link-{os.getpid()}"
            if os.path.lexists(link_temporary):
                link_temporary.unlink()
            os.symlink(env_file, link_temporary)
            os.replace(link_temporary, link)
        except OSError as exc:
            raise SetupError(
                f"Could not save the session: {exc}",
                code="session_write_failed",
                status_code=500,
            ) from exc
        return values

    def _is_current(self, env_file: Path) -> bool:
        try:
            return os.path.realpath(self.link_path) == os.path.realpath(env_file) and (
                self.link_path.exists()
            )
        except OSError:
            return False

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
        detected = self._route_source(windows_ip) if windows_ip else None
        reference = session if session is not None else self.builtin_defaults()
        mocap_ws = Path(reference["mocap_ws"])
        display = bool(self._display_available())
        firewall = None
        if windows_ip and linux_ip:
            firewall = (
                f"sudo ufw allow from {windows_ip} to {linux_ip} port {XSENS_UDP_PORT} proto udp"
            )
        return {
            "repo_root": _tilde(str(self.repo_root), self.home),
            # Lets the UI show paths under the home folder as ~/... (inputs accept both).
            "home": str(self.home),
            "session": session,
            "defaults": self.new_session_defaults(session),
            "network": {
                "interfaces": list(self._interfaces()),
                "detected_linux_ip": detected,
            },
            "firewall_command": firewall,
            "stream_check_s": stream_check_seconds(),
            "readiness": {
                "live_image": None,
                "recorded_runtime": (mocap_ws / "venv" / "bin" / "python").exists()
                and (mocap_ws / "GMR").is_dir(),
                "motion_bvh": Path(reference["motion_bvh"]).is_file(),
                "npose_bvh": Path(reference["npose_bvh"]).is_file(),
                "display": display,
                "display_hint": None if display else NO_DISPLAY_HINT,
            },
            "files": self.files(session),
            "jobs": self.list_jobs(),
        }

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

        if kind == "live":
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
        options = self._validate_options(kind, options)
        raw = self.read_session_raw()
        if raw is None or not raw.get("SESSION_DIR"):
            raise SetupError("Save the session values first", code="no_session")
        session = self._values_from_env(raw)
        # session.env may have been edited by hand: outputs must stay in an allowed folder.
        if not self._session_dir_allowed(session["session_dir"]):
            raise SetupError(SESSION_DIR_LOCATION_MESSAGE, code="invalid_session_dir")
        if kind in ("acceptance", "live") and not session["windows_xsens_ip"]:
            raise SetupError(
                "Enter the Windows Xsens IP in the session first",
                code="missing_windows_ip",
            )
        if kind in ("retarget", "render") or (kind == "doctor" and session["route"] == "recorded"):
            runtime_python = Path(session["mocap_ws"]) / "venv" / "bin" / "python"
            if not runtime_python.exists():
                raise SetupError(
                    f"Recorded runtime not found in {_tilde(session['mocap_ws'], self.home)}. "
                    "Run the installer and choose 2 · Recorded.",
                    code="recorded_runtime_missing",
                )
        if kind in ("live", "render") and not self._display_available():
            raise SetupError(NO_DISPLAY_HINT, code="no_display")
        self._require_no_running_job()
        if kind in ("acceptance", "live"):
            # Only after the running-job check: our own job would hold these.
            if self._viewer_running():
                raise SetupError(
                    "A live viewer from an earlier run is still running. Stop it, then try again.",
                    code="viewer_running",
                )
            port = int(raw.get("XSENS_UDP_PORT") or XSENS_UDP_PORT)
            if self._udp_port_busy(port):
                raise SetupError(
                    f"Another program is using UDP port {port} (a terminal stream check or "
                    "viewer?). Stop it, then try again.",
                    code="udp_port_busy",
                )
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
                args=(job, session, raw, options),
                name=f"mocap-setup-{job.id}",
                daemon=True,
            )
        job.thread.start()
        return job.summary_dict()

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
        if running is not None and running.kind == "live":
            raise SetupError(
                "The live simulation is running from this console; use its Stop button",
                code="job_running",
                job_id=running.id,
            )
        self._stop_container()
        return {"stopped": True}

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
            "bvh_check": "Check BVH files",
            "save": "Save report",
        }[kind]

    def _execute(
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
    ) -> int:
        """Run ``argv`` in its own process group, streaming output into the job log."""

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
                    stdin=subprocess.DEVNULL,
                    stdout=stdout_handle if stdout_handle is not None else subprocess.PIPE,
                    stderr=subprocess.PIPE if stdout_handle is not None else subprocess.STDOUT,
                    start_new_session=True,
                )
                job.process = process
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
        return code

    def _display_command(self, argv: list[str]) -> str:
        """Shell-quoted command line with home-folder paths shown as ``~/...``."""

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

        def default() -> list[str]:
            return [str(self.repo_root / "tools" / "xsens" / "live_operator.sh"), "acceptance",
                    "--output", str(output)]

        # An override receives the report path as options["output"].
        argv = self._argv("acceptance", session, {**options, "output": str(output)}, default)
        code = self._run_process(job, argv, self._session_env(raw))
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

    def _run_live(self, job, session, raw, options) -> tuple[int, bool]:
        def default() -> list[str]:
            argv = [str(self.repo_root / "tools" / "xsens" / "live_operator.sh"), "gmr",
                    "--robot", session["robot"],
                    "--human-height", _format_number(session["human_height_m"])]
            if options["calibrate_after_s"] is not None:
                argv += ["--calibrate-after", _format_number(options["calibrate_after_s"])]
            if options["record_video"]:
                argv.append("--record-video")
            return argv

        env = self._session_env(raw)
        code = self._run_process(job, self._argv("live", session, options, default), env)
        if options["record_video"]:
            runtime_root = Path(env.get("GMR_LIVE_RUNTIME_ROOT")
                                or self.repo_root / "artifacts" / "xsens-live")
            short = "t1" if session["robot"] == "booster_t1_29dof" else "g1"
            video = runtime_root / f"{short}-live.mp4"
            try:
                fresh = video.is_file() and video.stat().st_mtime >= job.wall_started_at
            except OSError:
                fresh = False
            if fresh:
                target = job.session_dir / (
                    f"{session['robot']}-live-{self._stamp('%Y%m%d-%H%M%S')}.mp4"
                )
                shutil.copy2(video, target)
                job.outputs.append(target)
                job.append(f"Saved video {target.name} in the session folder")
            else:
                job.append("No new live video was recorded")
        return code, code == 0

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
                f"{mocap_ws}/venv/bin/python",
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
                f"{mocap_ws}/venv/bin/python",
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
            job.append(f"ERROR: git rev-parse failed: {exc}")
            return 1, False
        revision = result.stdout.strip()
        if result.returncode != 0 or not revision:
            job.append(f"ERROR: git rev-parse failed: {strip_ansi(result.stderr.strip())}")
            return result.returncode or 1, False
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

    # ------------------------------------------------------------------- stop

    def stop_job(self, job_id: str, *, wait: bool = False) -> dict[str, Any]:
        """Request a stop: SIGINT, then SIGTERM, then SIGKILL the job's process group.

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
        if job.kind == "live":
            self._stop_container()
            if exited(self.stop_grace_s):
                return
        signal_group(signal.SIGTERM)
        if exited(self.stop_grace_s):
            return
        signal_group(signal.SIGKILL)
        exited(self.stop_grace_s)

    def shutdown(self) -> None:
        """Stop any running job; called on console shutdown."""

        job = self.running_job()
        if job is not None:
            self.stop_job(job.id, wait=True)


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
    "JOB_KINDS",
    "SESSION_KEYS",
    "SetupError",
    "SetupWorkbench",
    "parse_session_env",
    "require_setup_mode",
    "strip_ansi",
    "usable_ipv4",
]
