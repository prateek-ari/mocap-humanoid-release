#!/usr/bin/env python3
"""Open the Setup console in the browser, starting it in the background first if needed.

Windows counterpart of tools/setup/open_console.sh. The "Mocap Setup" shortcut
runs it with .venv\\Scripts\\pythonw.exe (no console window), so errors are also
shown in a message box.

  (no option)    start the Setup console if it is not running, then open it
  --no-browser   start it without opening a browser
  --status       report whether the console is running (exit 0 when it is)
  --stop         stop a Setup console started by this launcher

The console listens on http://127.0.0.1:8784/ only (this PC); set
MOCAP_SETUP_PORT to change it. Its log is
%LOCALAPPDATA%\\mocap-humanoid\\setup-console.log.

The server gets its own hidden console (CREATE_NO_WINDOW, not DETACHED_PROCESS)
and its own process group, so --stop can deliver CTRL_BREAK to it: uvicorn
shuts down gracefully and the console stops its running job (which shares that
console) the same way. Only if that does not finish in time is the process tree
killed with taskkill.

This module imports cleanly on Linux (Windows APIs are looked up lazily) so its
logic is unit-tested there; on Linux it falls back to POSIX signals.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Callable, Mapping, Sequence
import urllib.error
import urllib.request
import webbrowser


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PORT = 8784
TITLE = "Mocap Setup"
START_TIMEOUT = 30.0  # first start compiles bytecode and imports FastAPI
STOP_GRACE = 45.0  # the console stops a running job (live viewer: up to ~40 s)

# Windows constants (kept numeric so this module imports on Linux).
CREATE_NEW_PROCESS_GROUP = 0x00000200
CREATE_NO_WINDOW = 0x08000000
CREATE_BREAKAWAY_FROM_JOB = 0x01000000
DETACHED_PROCESS = 0x00000008
CTRL_BREAK_EVENT = 1
PROCESS_QUERY_LIMITED_INFORMATION = 0x1000
SYNCHRONIZE = 0x00100000
STILL_ACTIVE = 259
MB_ICONERROR = 0x10
MB_ICONINFORMATION = 0x40


class LauncherError(RuntimeError):
    pass


# ---------------------------------------------------------------- paths/config


def is_windows(platform: str | None = None) -> bool:
    return (sys.platform if platform is None else platform) == "win32"


def state_dir(env: Mapping[str, str] | None = None) -> Path:
    """%LOCALAPPDATA%\\mocap-humanoid (the installer's tools dir)."""

    env = os.environ if env is None else env
    base = env.get("LOCALAPPDATA")
    return (Path(base) if base else Path.home() / "AppData" / "Local") / "mocap-humanoid"


def venv_python(repo_root: Path = REPO_ROOT, *, gui: bool = False,
                platform: str | None = None) -> Path:
    if is_windows(platform):
        return repo_root / ".venv" / "Scripts" / ("pythonw.exe" if gui else "python.exe")
    return repo_root / ".venv" / "bin" / "python"


def setup_port(env: Mapping[str, str] | None = None) -> int:
    env = os.environ if env is None else env
    value = env.get("MOCAP_SETUP_PORT", "").strip() or str(DEFAULT_PORT)
    if not value.isdigit() or not 1 <= int(value) <= 65535:
        raise LauncherError("MOCAP_SETUP_PORT must be 1-65535")
    return int(value)


def console_url(port: int) -> str:
    return f"http://127.0.0.1:{port}/"


def server_argv(repo_root: Path, port: int, *, platform: str | None = None) -> list[str]:
    return [
        str(venv_python(repo_root, platform=platform)),
        str(repo_root / "tools" / "operator_console.py"),
        "--setup",
        "--port",
        str(port),
    ]


def server_env(base: Mapping[str, str], tools_dir: Path) -> dict[str, str]:
    """Environment for the console: unbuffered log, the installer's MOCAP_WS, MinGit on PATH."""

    env = dict(base)
    env["PYTHONUNBUFFERED"] = "1"
    # The installer's runtime workspace becomes the console's default runtime folder.
    try:
        recorded = (tools_dir / "runtime-workspace.txt").read_text(encoding="utf-8").strip()
    except OSError:
        recorded = ""
    if recorded and not env.get("MOCAP_WS"):
        env["MOCAP_WS"] = recorded
    mingit = tools_dir / "mingit" / "cmd"
    if mingit.is_dir():
        # Windows env keys are case-insensitive but a dict copy is not.
        key = next((name for name in env if name.upper() == "PATH"), "PATH")
        current = env.get(key, "")
        parts = [part for part in current.split(os.pathsep) if part]
        if os.path.normcase(str(mingit)) not in {os.path.normcase(p) for p in parts}:
            env[key] = os.pathsep.join([str(mingit), *parts])
    return env


def creation_flags(*, breakaway: bool) -> int:
    flags = CREATE_NO_WINDOW | CREATE_NEW_PROCESS_GROUP
    return flags | CREATE_BREAKAWAY_FROM_JOB if breakaway else flags


# ---------------------------------------------------------------- probe


@dataclass(frozen=True)
class Probe:
    state: str  # "setup" (our console answers), "none" (nothing listens), "busy" (another program)
    detail: str = ""
    app: bool = False  # the console runs in Windows-app mode (its "app" field is set)


# Windows retries a refused loopback connect for about 2 s before failing, so a
# 1 s timeout (as in open_console.sh) would misreport a free port as "busy".
PROBE_TIMEOUT = 5.0


def probe(url: str, *, timeout: float = PROBE_TIMEOUT,
          opener: Callable[..., object] = urllib.request.urlopen) -> Probe:
    try:
        with opener(url + "api/v1/setup", timeout=timeout) as response:  # type: ignore[attr-defined]
            payload = json.load(response)
            status = getattr(response, "status", 200)
    except urllib.error.HTTPError as exc:
        return Probe("busy", f"HTTP {exc.code}")
    except urllib.error.URLError as exc:
        if isinstance(exc.reason, ConnectionRefusedError):
            return Probe("none")
        if isinstance(exc.reason, OSError) and getattr(exc.reason, "winerror", None) == 10061:
            return Probe("none")
        return Probe("busy", str(exc.reason))
    except ConnectionRefusedError:
        return Probe("none")
    except Exception as exc:  # answered, but not with HTTP/JSON
        return Probe("busy", type(exc).__name__)
    if status == 200 and isinstance(payload, dict) and "readiness" in payload:
        return Probe("setup", str(payload.get("platform", "")), app=bool(payload.get("app")))
    return Probe("busy", f"HTTP {status}")


# ---------------------------------------------------------------- pid file


def read_pid(pid_file: Path) -> int | None:
    try:
        text = pid_file.read_text(encoding="ascii").strip()
    except (OSError, UnicodeDecodeError):
        return None
    return int(text) if text.isdigit() and int(text) > 0 else None


def write_pid(pid_file: Path, pid: int) -> None:
    pid_file.parent.mkdir(parents=True, exist_ok=True)
    temporary = pid_file.with_suffix(".tmp")
    temporary.write_text(f"{pid}\n", encoding="ascii")
    os.replace(temporary, pid_file)


def remove_pid(pid_file: Path) -> None:
    try:
        pid_file.unlink()
    except FileNotFoundError:
        pass


# ---------------------------------------------------------------- processes


def _kernel32():
    import ctypes
    from ctypes import wintypes

    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel32.OpenProcess.restype = wintypes.HANDLE
    kernel32.OpenProcess.argtypes = (wintypes.DWORD, wintypes.BOOL, wintypes.DWORD)
    kernel32.CloseHandle.argtypes = (wintypes.HANDLE,)
    kernel32.GetExitCodeProcess.argtypes = (wintypes.HANDLE, ctypes.POINTER(wintypes.DWORD))
    kernel32.QueryFullProcessImageNameW.argtypes = (
        wintypes.HANDLE, wintypes.DWORD, wintypes.LPWSTR, ctypes.POINTER(wintypes.DWORD))
    kernel32.AttachConsole.argtypes = (wintypes.DWORD,)
    kernel32.SetConsoleCtrlHandler.argtypes = (ctypes.c_void_p, wintypes.BOOL)
    kernel32.GenerateConsoleCtrlEvent.argtypes = (wintypes.DWORD, wintypes.DWORD)
    return kernel32


def process_image(pid: int) -> str | None:
    """Full executable path of a running process (Windows), else None."""

    if not is_windows():
        return None
    import ctypes
    from ctypes import wintypes

    kernel32 = _kernel32()
    handle = kernel32.OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, False, pid)
    if not handle:
        return None
    try:
        size = wintypes.DWORD(32768)
        buffer = ctypes.create_unicode_buffer(size.value)
        if not kernel32.QueryFullProcessImageNameW(handle, 0, buffer, ctypes.byref(size)):
            return None
        return buffer.value
    finally:
        kernel32.CloseHandle(handle)


def pid_alive(pid: int) -> bool:
    if is_windows():
        import ctypes
        from ctypes import wintypes

        kernel32 = _kernel32()
        handle = kernel32.OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, False, pid)
        if not handle:
            return False
        try:
            code = wintypes.DWORD()
            if not kernel32.GetExitCodeProcess(handle, ctypes.byref(code)):
                return False
            return code.value == STILL_ACTIVE
        finally:
            kernel32.CloseHandle(handle)
    try:
        # Reap it if it is our own exited child (the desktop app starts the
        # server as its child); a zombie would otherwise look alive below.
        if os.waitpid(pid, os.WNOHANG)[0] == pid:
            return False
    except ChildProcessError:
        pass  # not our child
    except OSError:
        pass
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def is_our_server(pid: int, expected_python: Path,
                  image: Callable[[int], str | None] = process_image) -> bool:
    """Guard against a stale pid file whose pid now belongs to another program.

    On Windows the recorded pid is the venv's python.exe launcher (it runs the
    real interpreter as its child, in the same console and process group).
    Elsewhere there is no cheap image lookup, so liveness alone decides.
    """

    path = image(pid)
    if path is None:
        return not is_windows()
    return os.path.normcase(os.path.abspath(path)) == os.path.normcase(
        os.path.abspath(str(expected_python)))


def send_ctrl_break_here(pid: int) -> bool:
    """Attach to the server's console and raise CTRL_BREAK in its process group.

    Runs in a short-lived helper process (see request_ctrl_break) because
    FreeConsole/AttachConsole would otherwise take this launcher off its own
    console. Returns True when the event was generated.
    """

    kernel32 = _kernel32()
    kernel32.FreeConsole()
    if not kernel32.AttachConsole(pid):
        return False
    try:
        # This helper is not in the server's process group, so it does not
        # receive the CTRL_BREAK; ignore CTRL_C too in case a console sends one.
        kernel32.SetConsoleCtrlHandler(None, True)
        return bool(kernel32.GenerateConsoleCtrlEvent(CTRL_BREAK_EVENT, pid))
    finally:
        kernel32.FreeConsole()


def ctrl_break_helper_argv(pid: int) -> list[str]:
    return [sys.executable, str(Path(__file__).resolve()), "--ctrl-break-helper", str(pid)]


def request_ctrl_break(pid: int) -> bool:
    """Ask the server to stop gracefully (Windows: CTRL_BREAK, else SIGINT)."""

    if not is_windows():
        try:
            os.kill(pid, signal.SIGINT)
        except OSError:
            return False
        return True
    try:
        result = subprocess.run(
            ctrl_break_helper_argv(pid),
            creationflags=DETACHED_PROCESS,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=30,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return False
    return result.returncode == 0


def taskkill_argv(pid: int, env: Mapping[str, str] | None = None) -> list[str]:
    env = os.environ if env is None else env
    system_root = env.get("SystemRoot") or env.get("SYSTEMROOT") or r"C:\Windows"
    return [os.path.join(system_root, "System32", "taskkill.exe"), "/PID", str(pid), "/T", "/F"]


def force_kill(pid: int) -> None:
    if is_windows():
        subprocess.run(taskkill_argv(pid), check=False, stdin=subprocess.DEVNULL,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=60)
        return
    try:
        os.killpg(pid, signal.SIGKILL)
    except OSError:
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass


def wait_until(predicate: Callable[[], bool], timeout: float, *, interval: float = 0.1,
               clock: Callable[[], float] = time.monotonic,
               sleep: Callable[[float], None] = time.sleep) -> bool:
    deadline = clock() + timeout
    while True:
        if predicate():
            return True
        if clock() >= deadline:
            return False
        sleep(interval)


# ---------------------------------------------------------------- lifecycle


@dataclass
class Launcher:
    """Start, track and stop one console server.

    By default it runs the ZIP route's checkout (.venv python, state in
    %LOCALAPPDATA%\\mocap-humanoid). The desktop app (tools/windows/app.py)
    passes the interpreter, full command, environment and file locations
    explicitly instead.
    """

    repo_root: Path = REPO_ROOT
    tools_dir: Path | None = None
    port: int = DEFAULT_PORT
    out: Callable[[str], None] = print
    # Explicit server command; None derives each from repo_root/tools_dir as before.
    python: Path | None = None  # also the image a tracked pid must run
    argv: Sequence[str] | None = None
    env: Mapping[str, str] | None = None
    cwd: Path | None = None
    pid_path: Path | None = None
    log_path: Path | None = None
    name: str = "Setup console"
    missing_hint: str = "run Install-Windows.cmd"
    # Seams for tests.
    probe_fn: Callable[[str], Probe] = probe
    alive_fn: Callable[[int], bool] = pid_alive
    owner_fn: Callable[[int, Path], bool] = is_our_server
    break_fn: Callable[[int], bool] = request_ctrl_break
    kill_fn: Callable[[int], None] = force_kill
    popen_fn: Callable[..., subprocess.Popen] = subprocess.Popen
    clock: Callable[[], float] = time.monotonic
    sleep: Callable[[float], None] = time.sleep

    def __post_init__(self) -> None:
        if self.tools_dir is None:
            self.tools_dir = state_dir()

    @property
    def url(self) -> str:
        return console_url(self.port)

    @property
    def pid_file(self) -> Path:
        if self.pid_path is not None:
            return self.pid_path
        assert self.tools_dir is not None
        return self.tools_dir / "setup-console.pid"

    @property
    def log_file(self) -> Path:
        if self.log_path is not None:
            return self.log_path
        assert self.tools_dir is not None
        return self.tools_dir / "setup-console.log"

    @property
    def server_python(self) -> Path:
        return self.python if self.python is not None else venv_python(self.repo_root)

    def server_command(self) -> list[str]:
        if self.argv is not None:
            return [str(part) for part in self.argv]
        return server_argv(self.repo_root, self.port)

    def server_environment(self) -> dict[str, str]:
        if self.env is not None:
            return dict(self.env)
        assert self.tools_dir is not None
        return server_env(os.environ, self.tools_dir)

    def tracked_pid(self) -> int | None:
        """The pid from the pid file if that process is still our console server."""

        pid = read_pid(self.pid_file)
        if pid is None:
            return None
        if self.alive_fn(pid) and self.owner_fn(pid, self.server_python):
            return pid
        remove_pid(self.pid_file)  # stale
        return None

    def start(self) -> int:
        python = self.server_python
        if not python.is_file():
            raise LauncherError(f"project environment missing ({python}); {self.missing_hint}")
        self.log_file.parent.mkdir(parents=True, exist_ok=True)
        argv = self.server_command()
        env = self.server_environment()
        with open(self.log_file, "ab") as log:
            log.write(f"\n--- {time.strftime('%Y-%m-%d %H:%M:%S')} starting: {argv}\n".encode())
            log.flush()
            options: dict[str, object] = dict(
                cwd=str(self.cwd or self.repo_root), env=env, stdin=subprocess.DEVNULL,
                stdout=log, stderr=subprocess.STDOUT, close_fds=True,
            )
            if is_windows():
                try:
                    # Break away from a kill-on-close job (some terminals and
                    # CI runners), so the console outlives this launcher.
                    process = self.popen_fn(argv, creationflags=creation_flags(breakaway=True),
                                            **options)
                except OSError:
                    process = self.popen_fn(argv, creationflags=creation_flags(breakaway=False),
                                            **options)
            else:
                process = self.popen_fn(argv, start_new_session=True, **options)
        write_pid(self.pid_file, process.pid)

        def ready() -> bool:
            if self.probe_fn(self.url).state == "setup":
                return True
            if process.poll() is not None:
                remove_pid(self.pid_file)
                raise LauncherError(f"the {self.name} stopped; see {self.log_file}")
            return False

        if not wait_until(ready, START_TIMEOUT, clock=self.clock, sleep=self.sleep):
            raise LauncherError(f"the {self.name} did not start within {START_TIMEOUT:.0f} s; "
                                f"see {self.log_file}")
        return process.pid

    def stop(self) -> bool:
        """Stop a console started by this launcher. False when none was tracked."""

        pid = self.tracked_pid()
        if pid is None:
            remove_pid(self.pid_file)
            return False
        self.out(f"Stopping the {self.name} and any running job...")
        def gone() -> bool:
            return not self.alive_fn(pid)

        if self.break_fn(pid) and wait_until(gone, STOP_GRACE, clock=self.clock, sleep=self.sleep):
            remove_pid(self.pid_file)
            return True
        self.out(f"The {self.name} did not stop in time; ending its process tree.")
        self.kill_fn(pid)
        wait_until(gone, 10.0, clock=self.clock, sleep=self.sleep)
        remove_pid(self.pid_file)
        return True

    def status(self) -> tuple[int, str]:
        result = self.probe_fn(self.url)
        pid = self.tracked_pid()
        owner = f" (pid {pid})" if pid else ""
        if result.state == "setup":
            return 0, f"{self.name} running at {self.url}{owner}"
        if result.state == "busy":
            return 1, f"port {self.port} is used by another program ({result.detail})"
        return 1, f"{self.name} is not running"

    def open(self, *, browser: bool = True,
             open_browser: Callable[[str], bool] = webbrowser.open) -> None:
        state = self.probe_fn(self.url).state
        if state == "busy":
            raise LauncherError(
                f"port {self.port} is used by another program; stop it or set MOCAP_SETUP_PORT")
        if state == "none":
            self.out("Starting the Setup console...")
            self.start()
        self.out(f"Setup console: {self.url}")
        if browser and not open_browser(self.url):
            self.out(f"Open {self.url} in your browser.")


# ---------------------------------------------------------------- reporting


def has_console_output() -> bool:
    """False under pythonw.exe (the shortcut), where sys.stdout/stderr are None."""

    return sys.stderr is not None and sys.stdout is not None


def message_box(text: str, *, error: bool = True, title: str = TITLE) -> None:
    if not is_windows():
        return
    import ctypes

    flags = MB_ICONERROR if error else MB_ICONINFORMATION
    ctypes.windll.user32.MessageBoxW(None, text, title, flags)  # type: ignore[attr-defined]


def say(text: str) -> None:
    if has_console_output():
        print(text, flush=True)


def report_error(text: str) -> None:
    if has_console_output():
        print(f"ERROR: {text}", file=sys.stderr, flush=True)
    else:
        message_box(text)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Open the Mocap Setup console (Windows launcher).")
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--stop", action="store_true",
                        help="stop a Setup console started by this launcher")
    action.add_argument("--status", action="store_true",
                        help="report whether the Setup console is running")
    action.add_argument("--ctrl-break-helper", type=int, metavar="PID", help=argparse.SUPPRESS)
    parser.add_argument("--no-browser", action="store_true",
                        help="start the console without opening a browser")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    try:
        args = build_parser().parse_args(argv)
    except SystemExit as exc:
        if not has_console_output() and exc.code:
            message_box("Invalid launcher options; see tools/windows/console_launcher.py --help")
        raise
    if args.ctrl_break_helper is not None:
        return 0 if send_ctrl_break_here(args.ctrl_break_helper) else 1
    try:
        launcher = Launcher(port=setup_port(), out=say)
        if args.stop:
            say("Stopped the Setup console." if launcher.stop()
                else "No Setup console was started by this launcher.")
            return 0
        if args.status:
            code, text = launcher.status()
            say(text)
            return code
        launcher.open(browser=not args.no_browser)
    except LauncherError as exc:
        report_error(str(exc))
        return 2
    except Exception as exc:  # the shortcut has no console: always surface failures
        report_error(f"{type(exc).__name__}: {exc}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
