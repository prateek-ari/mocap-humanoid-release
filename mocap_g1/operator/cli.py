"""Launch the operator-facing mocap G1 status and session console.

This process serves the operator web application. It deliberately does not
import or launch a robot transport, controller, or motion process.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
import threading
import webbrowser
from typing import Any, Callable, Sequence


INSTALL_HINT = (
    "Install the operator dependencies with: pip install 'mocap-g1[operator]'"
)
IDENTITY_HEADER_ENV = "MOCAP_G1_IDENTITY_HEADER"
TRUSTED_PROXIES_ENV = "MOCAP_G1_TRUSTED_PROXIES"


class ConsoleDependencyError(RuntimeError):
    """Raised when the optional operator-console runtime is unavailable."""


def _default_data_root() -> Path:
    state_home = os.environ.get("XDG_STATE_HOME")
    base = Path(state_home).expanduser() if state_home else Path.home() / ".local" / "state"
    return base / "mocap-g1" / "operator-console"


def _env_list(name: str) -> list[str]:
    return [item.strip() for item in os.environ.get(name, "").split(",") if item.strip()]


def _port(value: str) -> int:
    try:
        port = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("port must be an integer") from exc
    if not 1 <= port <= 65535:
        raise argparse.ArgumentTypeError("port must be between 1 and 65535")
    return port


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run the mocap G1 status and session console. This command never "
            "starts robot control or grants motion authority."
        )
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help=(
            "HTTP bind address (default: %(default)s). A non-loopback address "
            "is refused unless --identity-header and --trusted-proxy name an "
            "authenticating reverse proxy"
        ),
    )
    parser.add_argument(
        "--port",
        type=_port,
        default=8766,
        help="HTTP port (default: %(default)s)",
    )
    parser.add_argument(
        "--allowed-host",
        action="append",
        default=[],
        dest="allowed_hosts",
        metavar="HOST",
        help=(
            "additional trusted Host/Origin, repeatable: https://NAME[:PORT] "
            "for the authenticated reverse proxy, or NAME[:PORT] for direct "
            "HTTP (default port: --port). Loopback names and a specific --host "
            "address are always allowed; all other hosts are rejected. A "
            "non-loopback entry requires --identity-header and --trusted-proxy"
        ),
    )
    parser.add_argument(
        "--identity-header",
        metavar="HEADER",
        help=(
            "request header carrying the authenticated operator identity set by "
            f"a trusted reverse proxy (default: ${IDENTITY_HEADER_ENV}). Honoured "
            "only from --trusted-proxy peers; otherwise actions are attributed "
            "to the local console and browser-entered names are recorded as "
            "claimed_operator"
        ),
    )
    parser.add_argument(
        "--trusted-proxy",
        action="append",
        dest="trusted_proxies",
        metavar="ADDRESS",
        help=(
            "TCP peer address or CIDR of that identity proxy, repeatable "
            f"(default: comma-separated ${TRUSTED_PROXIES_ENV}); required with "
            "--identity-header"
        ),
    )
    parser.add_argument(
        "--data-root",
        type=Path,
        default=_default_data_root(),
        help="persistent console state directory (default: %(default)s)",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--demo",
        action="store_true",
        help="use synthetic status data; never connect to robot hardware",
    )
    mode.add_argument(
        "--playback",
        type=Path,
        help=(
            "show one local recorded-review MP4 in passive mode; never connect "
            "to robot hardware"
        ),
    )
    mode.add_argument(
        "--simulation",
        type=Path,
        help=(
            "replay one local qpos NPZ with the in-process safety envelope; "
            "never connect to robot hardware"
        ),
    )
    parser.add_argument(
        "--qualification-root",
        type=Path,
        help=(
            "read sanitized qualification ledger summaries below this fixed "
            "directory; never modify qualification artifacts"
        ),
    )
    parser.add_argument(
        "--setup",
        action="store_true",
        help=(
            "enable the Setup workspace: enter session values and run the "
            "engineer setup-guide steps from the browser. Needs a source "
            "checkout and a loopback-only console; not with --playback or "
            "--simulation. tools/setup/open_console.sh starts it on port 8784 "
            "so it does not clash with a status console on the default port"
        ),
    )
    parser.add_argument(
        "--open",
        action="store_true",
        dest="open_browser",
        help="open the console in the default browser after launch",
    )
    return parser


def _load_runtime() -> tuple[Any, Callable[..., Any]]:
    """Import optional dependencies only after argument parsing."""

    try:
        import uvicorn
    except (ImportError, ModuleNotFoundError) as exc:
        raise ConsoleDependencyError(INSTALL_HINT) from exc

    try:
        from mocap_g1.operator.app import create_app
    except (ImportError, ModuleNotFoundError) as exc:
        raise ConsoleDependencyError(
            f"The operator console runtime is unavailable. {INSTALL_HINT}"
        ) from exc

    return uvicorn, create_app


def _setup_root() -> Path:
    """Return the source checkout holding the setup tools, or raise ValueError."""

    import mocap_g1

    root = Path(mocap_g1.__file__).resolve().parents[1]
    if not (root / "tools" / "xsens" / "live_operator.sh").is_file():
        raise ValueError(
            f"--setup needs a source checkout of mocap_humanoid; {root} has no "
            "tools/xsens/live_operator.sh. Run the console from the checkout "
            "(.venv/bin/mocap-g1-console --setup after tools/setup/install.sh)"
        )
    return root


def _browser_url(host: str, port: int) -> str:
    browser_host = host
    if host == "0.0.0.0":
        browser_host = "127.0.0.1"
    elif host == "::":
        browser_host = "::1"
    if ":" in browser_host and not browser_host.startswith("["):
        browser_host = f"[{browser_host}]"
    return f"http://{browser_host}:{port}/"


def _schedule_browser(url: str) -> None:
    timer = threading.Timer(0.75, webbrowser.open, args=(url,))
    timer.daemon = True
    timer.start()


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    if args.setup and (args.playback is not None or args.simulation is not None):
        print(
            "ERROR: --setup cannot be combined with --playback or --simulation; "
            "use it with the default console or --demo",
            file=sys.stderr,
        )
        return 2

    playback = None
    if args.playback is not None:
        from mocap_g1.operator.playback import (
            PlaybackConfigurationError,
            resolve_playback_path,
        )

        try:
            playback = resolve_playback_path(args.playback)
        except PlaybackConfigurationError as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            return 2

    simulation = None
    if args.simulation is not None:
        from mocap_g1.operator.simulation import (
            RecordedSimulationConfigurationError,
            resolve_simulation_path,
        )

        try:
            simulation = resolve_simulation_path(args.simulation)
        except RecordedSimulationConfigurationError as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            return 2

    qualification_root = None
    if args.qualification_root is not None:
        from mocap_g1.operator.qualification import (
            QualificationRootError,
            resolve_qualification_root,
        )

        try:
            qualification_root = resolve_qualification_root(
                args.qualification_root
            )
        except QualificationRootError as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            return 2

    identity_header = (
        args.identity_header
        if args.identity_header is not None
        else os.environ.get(IDENTITY_HEADER_ENV) or None
    )
    trusted_proxies = tuple(
        args.trusted_proxies
        if args.trusted_proxies is not None
        else _env_list(TRUSTED_PROXIES_ENV)
    )
    # Checked before anything is loaded or created; create_app enforces the
    # same policy for programmatic callers.
    try:
        from mocap_g1.operator.app import require_identity_proxy_for_network_access
    except (ImportError, ModuleNotFoundError) as exc:
        print(
            f"ERROR: operator runtime dependency is unavailable: {exc}. {INSTALL_HINT}",
            file=sys.stderr,
        )
        return 2

    try:
        require_identity_proxy_for_network_access(
            bind_host=args.host,
            port=args.port,
            allowed_hosts=tuple(args.allowed_hosts),
            identity_header=identity_header,
            trusted_proxies=trusted_proxies,
        )
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    setup_root = None
    if args.setup:
        from mocap_g1.operator.app import require_loopback_only_for_setup

        try:
            require_loopback_only_for_setup(
                bind_host=args.host,
                port=args.port,
                allowed_hosts=tuple(args.allowed_hosts),
                identity_header=identity_header,
                trusted_proxies=trusted_proxies,
            )
            setup_root = _setup_root()
        except ValueError as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            return 2

    try:
        uvicorn, create_app = _load_runtime()
    except ConsoleDependencyError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    data_root = None
    if playback is None and simulation is None:
        data_root = args.data_root.expanduser().resolve()
        try:
            data_root.mkdir(mode=0o700, parents=True, exist_ok=True)
        except OSError as exc:
            print(f"ERROR: cannot create data root {data_root}: {exc}", file=sys.stderr)
            return 2

    try:
        app = create_app(
            data_root=data_root,
            demo=args.demo,
            playback=playback,
            simulation=simulation,
            qualification_root=qualification_root,
            bind_host=args.host,
            port=args.port,
            allowed_hosts=tuple(args.allowed_hosts),
            identity_header=identity_header,
            trusted_proxies=trusted_proxies,
            setup_root=setup_root,
        )
    except (ImportError, ModuleNotFoundError) as exc:
        print(
            f"ERROR: operator runtime dependency is unavailable: {exc}. {INSTALL_HINT}",
            file=sys.stderr,
        )
        return 2
    except RuntimeError as exc:
        if isinstance(exc.__cause__, ImportError) or "FastAPI" in str(exc):
            print(f"ERROR: {exc}. {INSTALL_HINT}", file=sys.stderr)
            return 2
        raise
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    if args.open_browser:
        _schedule_browser(_browser_url(args.host, args.port))

    try:
        # Never let X-Forwarded-For rewrite the peer address: trusted-proxy
        # attribution must be decided on the real TCP peer.
        uvicorn.run(app, host=args.host, port=args.port, proxy_headers=False)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
