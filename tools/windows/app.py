#!/usr/bin/env python3
"""Mocap Humanoid desktop app: one native window showing the console UI.

The desktop/Start-menu shortcut runs

    {app}\\python\\pythonw.exe "{app}\\project\\tools\\windows\\app.py"

It starts the console backend (tools/operator_console.py --setup) as a child
with a hidden console, opens a window titled "Mocap Humanoid" on
http://127.0.0.1:8784/ and, when that window closes, stops the backend
gracefully (CTRL_BREAK -> uvicorn shutdown -> the workbench stops a running
live sim), exactly as `console_launcher.py --stop` does.

Window, in order of preference:
  1. pywebview on WebView2 (a "Starting..." page appears at once and switches
     to the console when the backend answers);
  2. Microsoft Edge in app mode (--app=URL, own profile), when pywebview or
     WebView2 is unavailable, or MOCAP_FORCE_EDGE=1 (CI);
  3. the default browser plus a message box whose OK button stops everything.

  --smoke SECONDS  CI check: open as normal, wait for the page, confirm
                   GET /api/v1/setup reports "app", stay open SECONDS, close,
                   stop the backend; exit 0 only when all of that worked
  --no-window      run the backend only, until Ctrl+C (headless tests)
  --stop           stop a backend this install left running, then exit

Per-user state lives in %LOCALAPPDATA%\\MocapHumanoid (MOCAP_STATE_DIR
overrides it): logs\\app.log, logs\\backend.log, app.pid (the backend pid),
webview\\, edge\\, operator-data\\, pycache\\. MOCAP_SETUP_PORT changes the port.

Imports cleanly on any platform; on Linux (development) it runs the backend
from this checkout and uses pywebview when installed, else the browser.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import logging
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from typing import Callable, Mapping, Sequence
import urllib.request
import webbrowser

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import console_launcher as cl  # noqa: E402


PROJECT_ROOT = HERE.parents[1]  # {app}\project (or the source checkout)
APP_ROOT = PROJECT_ROOT.parent  # {app}
TITLE = "Mocap Humanoid"
APP_ID = "MocapHumanoid.App"
MUTEX_NAME = "Local\\MocapHumanoidApp"
ICON = HERE / "mocap.ico"
WINDOW_SIZE = (1280, 820)
MIN_SIZE = (1000, 680)
EDGE_HANDOFF_S = 3.0  # an Edge that exits this fast handed the URL to another Edge
LOAD_TIMEOUT = 60.0
ERROR_ALREADY_EXISTS = 183
SW_RESTORE = 9

log = logging.getLogger("mocap_app")


class AppError(RuntimeError):
    pass


class WindowUnavailable(RuntimeError):
    """pywebview or WebView2 cannot open a window; try the next fallback."""


# ---------------------------------------------------------------- layout/config


@dataclass(frozen=True)
class Layout:
    project: Path  # setup root: tools/, mocap_g1/
    python: Path  # backend interpreter (console python.exe, never pythonw.exe)
    runtime: Path | None  # {app}\runtime (MOCAP_WS); None in a dev checkout
    installed: bool


def resolve_layout(project: Path = PROJECT_ROOT, *, platform: str | None = None,
                   executable: str | None = None) -> Layout:
    """The installed layout ({app}\\python, {app}\\runtime) or this dev checkout."""

    app_root = project.parent
    if cl.is_windows(platform):
        bundled = app_root / "python" / "python.exe"
    else:
        bundled = app_root / "python" / "bin" / "python3"
    if bundled.is_file():
        runtime = app_root / "runtime"
        return Layout(project, bundled, runtime, True)
    venv = cl.venv_python(project, platform=platform)
    if venv.is_file():
        return Layout(project, venv, None, False)
    exe = Path(executable or sys.executable)
    if exe.name.lower() == "pythonw.exe":
        exe = exe.with_name("python.exe")
    return Layout(project, exe, None, False)


def app_state_dir(env: Mapping[str, str] | None = None, *,
                  platform: str | None = None) -> Path:
    """%LOCALAPPDATA%\\MocapHumanoid (MOCAP_STATE_DIR wins; XDG state dir off Windows)."""

    env = os.environ if env is None else env
    if env.get("MOCAP_STATE_DIR"):
        return Path(env["MOCAP_STATE_DIR"])
    if cl.is_windows(platform):
        base = env.get("LOCALAPPDATA")
        return (Path(base) if base else Path.home() / "AppData" / "Local") / "MocapHumanoid"
    xdg = env.get("XDG_STATE_HOME")
    return (Path(xdg) if xdg else Path.home() / ".local" / "state") / "MocapHumanoid"


def backend_argv(layout: Layout, state: Path, port: int) -> list[str]:
    return [
        str(layout.python),
        str(layout.project / "tools" / "operator_console.py"),
        "--setup",
        "--port",
        str(port),
        "--data-root",
        str(state / "operator-data"),
    ]


def backend_env(base: Mapping[str, str], layout: Layout, state: Path) -> dict[str, str]:
    """The contract environment for the backend (see windows-app contracts)."""

    env = dict(base)
    env.update({
        "MOCAP_APP": "1",
        "MOCAP_STATE_DIR": str(state),
        "MOCAP_RUNTIME_PYTHON": str(layout.python),
        "PYTHONPYCACHEPREFIX": str(state / "pycache"),
        "PYTHONUTF8": "1",
        "PYTHONUNBUFFERED": "1",
    })
    if layout.runtime is not None:
        env["MOCAP_WS"] = str(layout.runtime)
    if layout.installed:
        # The bundled interpreter must only see {app}: a PYTHONPATH/PYTHONHOME or
        # user site-packages left on this PC by another Python would shadow it.
        for name in ("PYTHONPATH", "PYTHONHOME", "PYTHONSTARTUP"):
            env.pop(name, None)
        env["PYTHONNOUSERSITE"] = "1"
    return env


def make_launcher(layout: Layout, state: Path, port: int,
                  base_env: Mapping[str, str] | None = None, **seams) -> cl.Launcher:
    return cl.Launcher(
        repo_root=layout.project,
        tools_dir=state,
        port=port,
        out=log.info,
        python=layout.python,
        argv=backend_argv(layout, state, port),
        env=backend_env(os.environ if base_env is None else base_env, layout, state),
        cwd=layout.project,
        pid_path=state / "app.pid",
        log_path=state / "logs" / "backend.log",
        name="Mocap Humanoid backend",
        missing_hint="reinstall Mocap Humanoid",
        **seams,
    )


def ensure_backend(launcher: cl.Launcher) -> str:
    """Start the backend, or reuse a healthy one this install started. Returns its URL."""

    result = launcher.probe_fn(launcher.url)
    if result.state == "busy":
        raise AppError(f"port {launcher.port} is used by another program "
                       f"({result.detail}); close it and start Mocap Humanoid again")
    if result.state == "setup":
        if result.app and launcher.tracked_pid() is not None:
            log.info("reusing the running backend at %s", launcher.url)
            return launcher.url
        raise AppError(f"another Mocap console is already running on port {launcher.port} "
                       "(a Mocap Setup console, or Mocap Humanoid for another Windows user); "
                       "stop it and start Mocap Humanoid again")
    log.info("starting the backend")
    launcher.start()
    return launcher.url


def fetch_setup(url: str, *, timeout: float = 10.0,
                opener: Callable[..., object] = urllib.request.urlopen) -> dict:
    with opener(url + "api/v1/setup", timeout=timeout) as response:  # type: ignore[attr-defined]
        payload = json.load(response)
    if not isinstance(payload, dict):
        raise AppError("GET /api/v1/setup did not return a JSON object")
    return payload


def check_app_mode(url: str, fetch: Callable[[str], dict] = fetch_setup) -> None:
    """Smoke check: the backend answers in Windows-app mode."""

    payload = fetch(url)
    if not payload.get("app"):
        raise AppError(f'GET /api/v1/setup has no "app" (got {payload.get("app")!r})')
    log.info("smoke: backend app mode %s", payload["app"])


# ---------------------------------------------------------------- Win32


def set_app_user_model_id() -> None:
    if not cl.is_windows():
        return
    import ctypes

    try:
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(APP_ID)  # type: ignore[attr-defined]
    except (AttributeError, OSError):
        log.warning("could not set the AppUserModelID")


class SingleInstance:
    """Named mutex Local\\MocapHumanoidApp; held for the life of the process."""

    def __init__(self, name: str = MUTEX_NAME) -> None:
        self.name = name
        self.handle = None

    def acquire(self) -> bool:
        """False when another instance holds the mutex. Always True off Windows."""

        if not cl.is_windows():
            return True
        import ctypes
        from ctypes import wintypes

        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel32.CreateMutexW.restype = wintypes.HANDLE
        kernel32.CreateMutexW.argtypes = (ctypes.c_void_p, wintypes.BOOL, wintypes.LPCWSTR)
        self.handle = kernel32.CreateMutexW(None, False, self.name)
        return not (self.handle and ctypes.get_last_error() == ERROR_ALREADY_EXISTS)


def bring_to_front(title: str = TITLE) -> bool:
    """Restore and focus the existing app window. False when none is found."""

    if not cl.is_windows():
        return False
    import ctypes

    user32 = ctypes.windll.user32  # type: ignore[attr-defined]
    hwnd = user32.FindWindowW(None, title)
    if not hwnd:
        return False
    if user32.IsIconic(hwnd):
        user32.ShowWindow(hwnd, SW_RESTORE)
    user32.SetForegroundWindow(hwnd)
    return True


def focus_running_app(bring: Callable[[], bool] = lambda: bring_to_front(),
                      sleep: Callable[[float], None] = time.sleep,
                      wait_s: float = 15.0, interval: float = 0.5) -> bool:
    """Bring the running app's window forward, waiting while it is still starting."""

    for _ in range(max(1, int(wait_s / interval))):
        if bring():
            return True
        sleep(interval)
    return False


def set_window_icon(title: str = TITLE, icon: Path = ICON) -> bool:
    """Give the WebView2 window (and its taskbar button) our icon instead of python's."""

    if not cl.is_windows() or not icon.is_file():
        return False
    import ctypes

    user32 = ctypes.windll.user32  # type: ignore[attr-defined]
    user32.LoadImageW.restype = ctypes.c_void_p
    user32.SendMessageW.argtypes = (ctypes.c_void_p, ctypes.c_uint, ctypes.c_void_p,
                                    ctypes.c_void_p)
    hwnd = user32.FindWindowW(None, title)
    if not hwnd:
        return False
    image_icon, load_from_file, load_default_size = 1, 0x10, 0x40
    handle = user32.LoadImageW(None, str(icon), image_icon, 0, 0,
                               load_from_file | load_default_size)
    if not handle:
        return False
    wm_seticon = 0x0080
    for size in (0, 1):  # ICON_SMALL, ICON_BIG
        user32.SendMessageW(hwnd, wm_seticon, size, handle)
    return True


# ---------------------------------------------------------------- Edge


def edge_candidates(env: Mapping[str, str] | None = None,
                    registry: Sequence[str] = ()) -> list[Path]:
    """msedge.exe locations: App Paths registry values first, then the usual folders."""

    env = os.environ if env is None else env
    found = [Path(value) for value in registry if value]
    for key in ("ProgramFiles(x86)", "ProgramFiles", "LOCALAPPDATA"):
        if env.get(key):
            found.append(Path(env[key]) / "Microsoft" / "Edge" / "Application" / "msedge.exe")
    return found


def edge_registry_paths() -> list[str]:
    if not cl.is_windows():
        return []
    import winreg

    values = []
    subkey = r"SOFTWARE\Microsoft\Windows\CurrentVersion\App Paths\msedge.exe"
    for hive in (winreg.HKEY_CURRENT_USER, winreg.HKEY_LOCAL_MACHINE):
        try:
            with winreg.OpenKey(hive, subkey) as key:
                values.append(str(winreg.QueryValueEx(key, "")[0]).strip('"'))
        except OSError:
            pass
    return values


def find_edge(env: Mapping[str, str] | None = None,
              registry: Callable[[], list[str]] = edge_registry_paths) -> Path | None:
    for path in edge_candidates(env, registry()):
        if path.is_file():
            return path
    return None


def edge_argv(edge: Path, url: str, state: Path) -> list[str]:
    return [str(edge), f"--app={url}", f"--user-data-dir={state / 'edge'}",
            "--no-first-run", "--no-default-browser-check"]


# ---------------------------------------------------------------- windows


STARTING_HTML = """<!doctype html><html><head><meta charset="utf-8"><title>Mocap Humanoid</title>
<style>html,body{height:100%;margin:0}body{display:flex;align-items:center;justify-content:center;
font:16px "Segoe UI",system-ui,sans-serif;background:#f6f7f9;color:#1d2330}
@media (prefers-color-scheme:dark){body{background:#15181e;color:#e6e8ec}}
.s{width:28px;height:28px;border:3px solid #8a93a6;border-top-color:transparent;border-radius:50%;
animation:r 1s linear infinite;margin:0 auto 14px}@keyframes r{to{transform:rotate(360deg)}}</style>
</head><body><div style="text-align:center"><div class="s"></div>Starting Mocap Humanoid&hellip;
</div></body></html>"""


@dataclass
class Session:
    """What a window run needs: the backend start, the URL, the per-user state, smoke."""

    launcher: cl.Launcher
    state: Path
    smoke_seconds: float | None = None
    start_fn: Callable[[cl.Launcher], str] = ensure_backend
    check_fn: Callable[[str], None] = check_app_mode
    sleep: Callable[[float], None] = time.sleep

    def start_backend(self) -> str:
        url = self.start_fn(self.launcher)
        if self.smoke_seconds is not None:
            self.check_fn(url)
        return url


def windows_renderer() -> str:
    """The engine pywebview picks on Windows: "edgechromium" (WebView2) or "mshtml"."""

    from webview.platforms import winforms  # loads pythonnet, as webview.start() would

    return str(winforms.renderer)


def run_webview(session: Session, webview_module=None) -> int:
    """pywebview/WebView2 window. Raises WindowUnavailable before any window opened."""

    if webview_module is None:
        try:
            import webview as webview_module  # type: ignore[no-redef]

            renderer = windows_renderer() if cl.is_windows() else None
        except Exception as exc:  # ImportError, or pythonnet failing to load
            raise WindowUnavailable(f"pywebview unavailable: {exc}") from exc
        # Without the WebView2 runtime pywebview silently uses MSHTML (Internet
        # Explorer), which cannot run the console; Edge --app can.
        if renderer not in (None, "edgechromium"):
            raise WindowUnavailable(f"no WebView2 runtime (pywebview would use {renderer})")
    webview = webview_module
    outcome: dict[str, object] = {"shown": False}
    loaded = threading.Event()
    closed = threading.Event()
    target: dict[str, str] = {}

    window = webview.create_window(TITLE, html=STARTING_HTML, width=WINDOW_SIZE[0],
                                   height=WINDOW_SIZE[1], min_size=MIN_SIZE)

    def on_shown() -> None:
        outcome["shown"] = True
        log.info("WebView2 window shown")
        set_window_icon()

    def on_loaded() -> None:
        if not target:
            return
        try:
            current = window.get_current_url() or ""
        except Exception:
            current = ""
        if current.startswith(target["url"]):
            loaded.set()

    window.events.shown += on_shown
    window.events.loaded += on_loaded
    def on_closed() -> None:
        closed.set()

    window.events.closed += on_closed

    def worker() -> None:
        try:
            url = session.start_backend()
            target["url"] = url
            window.load_url(url)
            if session.smoke_seconds is not None:
                if not loaded.wait(LOAD_TIMEOUT):
                    raise AppError(f"the console page did not load within {LOAD_TIMEOUT:.0f} s")
                log.info("smoke: page loaded; keeping the window open %.0f s",
                         session.smoke_seconds)
                session.sleep(session.smoke_seconds)
                window.destroy()
        except Exception as exc:
            if closed.is_set():  # the user closed the window while it was starting
                log.info("window closed during start-up (%s)", exc)
                return
            outcome["error"] = exc
            log.exception("backend/window worker failed")
            window.destroy()

    # Our own thread, joined below, so a window closed during start-up still
    # leaves the backend started (and tracked) before main() stops it.
    thread = threading.Thread(target=worker, name="mocap-backend-start", daemon=True)

    options: dict[str, object] = dict(private_mode=False,
                                      storage_path=str(session.state / "webview"))
    if cl.is_windows():
        options["gui"] = "edgechromium"
    elif ICON.is_file():
        options["icon"] = str(ICON)  # GTK/Qt only
    try:
        webview.start(thread.start, **options)
    except Exception as exc:
        if not outcome["shown"] and not thread.is_alive():
            raise WindowUnavailable(f"WebView2 window failed: {exc}") from exc
        raise
    finally:
        closed.set()
        if thread.ident is not None:
            thread.join(cl.START_TIMEOUT + 5)
    if "error" in outcome:
        raise outcome["error"]  # type: ignore[misc]
    return 0


def run_edge(session: Session, *, edge: Path | None = None,
             popen_fn: Callable[..., subprocess.Popen] = subprocess.Popen,
             clock: Callable[[], float] = time.monotonic,
             notify: Callable[[str], None] | None = None) -> int:
    """Edge --app window; returns when that Edge process exits."""

    edge = edge or find_edge()
    if edge is None:
        raise WindowUnavailable("Microsoft Edge was not found")
    url = session.start_backend()
    argv = edge_argv(edge, url, session.state)
    log.info("opening Edge: %s", argv)
    started = clock()
    try:
        process = popen_fn(argv, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL)
    except OSError as exc:
        raise WindowUnavailable(f"Edge did not start: {exc}") from exc
    if session.smoke_seconds is not None:
        session.sleep(session.smoke_seconds)
        if process.poll() is not None:
            raise AppError(f"Edge exited early (code {process.returncode})")
        if cl.is_windows():
            cl.force_kill(process.pid)  # Edge's whole process tree
        else:
            process.terminate()
        process.wait()
        return 0
    process.wait()
    if clock() - started < EDGE_HANDOFF_S:
        # Our profile's Edge was already open and took the window over.
        (notify or _stop_prompt)(url)
    return 0


def run_browser(session: Session, *, open_browser: Callable[[str], bool] = webbrowser.open,
                notify: Callable[[str], None] | None = None) -> int:
    """Last resort: the default browser, and a message box to stop."""

    url = session.start_backend()
    if session.smoke_seconds is not None:
        session.sleep(session.smoke_seconds)
        return 0
    open_browser(url)
    (notify or _stop_prompt)(url)
    return 0


def _stop_prompt(url: str) -> None:
    text = (f"Mocap Humanoid is running in your browser at {url}.\n\n"
            "Click OK to stop it (this also stops a running simulation).")
    if cl.is_windows():
        cl.message_box(text, error=False, title=TITLE)
    else:
        say(text.replace("Click OK", "Press Ctrl+C"))
        wait_for_interrupt()


def wait_for_interrupt() -> None:
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass


def run_headless(session: Session) -> int:
    url = session.start_backend()
    say(f"Mocap Humanoid backend: {url}")
    if session.smoke_seconds is not None:
        session.sleep(session.smoke_seconds)
    else:
        say("Press Ctrl+C to stop.")
        wait_for_interrupt()
    return 0


def run_window(session: Session, env: Mapping[str, str] | None = None, *,
               webview_fn: Callable[[Session], int] = run_webview,
               edge_fn: Callable[[Session], int] = run_edge,
               browser_fn: Callable[[Session], int] = run_browser) -> int:
    """Try pywebview, then Edge --app, then the default browser."""

    env = os.environ if env is None else env
    attempts: list[Callable[[Session], int]] = []
    if env.get("MOCAP_FORCE_EDGE") != "1":
        attempts.append(webview_fn)
    attempts.append(edge_fn)
    if session.smoke_seconds is None or not cl.is_windows():
        attempts.append(browser_fn)  # CI on Windows must exercise a real window
    last: Exception | None = None
    for attempt in attempts:
        try:
            return attempt(session)
        except WindowUnavailable as exc:
            log.warning("%s", exc)
            last = exc
    raise AppError(f"no window could be opened: {last}")


# ---------------------------------------------------------------- main


def say(text: str) -> None:
    log.info(text)
    cl.say(text)


def report_error(text: str) -> None:
    log.error(text)
    if cl.has_console_output():
        print(f"ERROR: {text}", file=sys.stderr, flush=True)
    else:
        cl.message_box(text, title=TITLE)


def setup_logging(state: Path) -> None:
    (state / "logs").mkdir(parents=True, exist_ok=True)
    handler = logging.FileHandler(state / "logs" / "app.log", encoding="utf-8")
    handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    root = logging.getLogger()
    root.addHandler(handler)
    root.setLevel(logging.INFO)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Open the Mocap Humanoid app window.")
    parser.add_argument("--smoke", type=float, metavar="SECONDS",
                        help="CI check: open, verify app mode, stay SECONDS, close, exit 0")
    parser.add_argument("--no-window", action="store_true",
                        help="run only the backend (until Ctrl+C, or SECONDS with --smoke)")
    parser.add_argument("--stop", action="store_true",
                        help="stop a backend left running by this install, then exit")
    return parser


def stop_backend(launcher_fn: Callable[..., cl.Launcher] = make_launcher) -> int:
    """--stop: end the backend tracked in app.pid (CI, or after a hard-killed app)."""

    launcher = launcher_fn(resolve_layout(), app_state_dir(), cl.setup_port())
    say("Mocap Humanoid backend stopped." if launcher.stop()
        else "No Mocap Humanoid backend was running.")
    return 0


def main(argv: Sequence[str] | None = None, *,
         instance: SingleInstance | None = None,
         launcher_fn: Callable[..., cl.Launcher] = make_launcher,
         window_fn: Callable[[Session], int] = run_window,
         headless_fn: Callable[[Session], int] = run_headless) -> int:
    try:
        args = build_parser().parse_args(argv)
    except SystemExit as exc:
        if not cl.has_console_output() and exc.code:
            cl.message_box("Invalid options; see tools/windows/app.py --help", title=TITLE)
        raise
    if args.stop:
        return stop_backend(launcher_fn)
    set_app_user_model_id()
    instance = instance or SingleInstance()
    if not instance.acquire():
        if not focus_running_app():
            cl.message_box(f"{TITLE} is still starting or closing. Try again in a moment.",
                           error=False, title=TITLE)
        return 0
    launcher = None
    code = 2
    try:
        state = app_state_dir()
        state.mkdir(parents=True, exist_ok=True)
        setup_logging(state)
        sys.pycache_prefix = str(state / "pycache")  # {app} is read-only
        layout = resolve_layout()
        log.info("starting: project=%s python=%s state=%s", layout.project, layout.python, state)
        launcher = launcher_fn(layout, state, cl.setup_port())
        session = Session(launcher, state, smoke_seconds=args.smoke)
        code = headless_fn(session) if args.no_window else window_fn(session)
    except (AppError, cl.LauncherError) as exc:
        report_error(str(exc))
    except Exception as exc:  # pythonw has no console: always surface failures
        log.exception("unexpected failure")
        report_error(f"{type(exc).__name__}: {exc}")
    finally:
        if launcher is not None:
            try:
                launcher.stop()
            except Exception:
                log.exception("stopping the backend failed")
                code = code or 2
    log.info("exit %s", code)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
