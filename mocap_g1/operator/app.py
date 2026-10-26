"""Optional FastAPI transport and redacted diagnostics export."""

import asyncio
from contextlib import asynccontextmanager, suppress
from datetime import datetime, timezone
import json
import logging
import os
from pathlib import Path
import ipaddress
import re
import threading
from typing import Any, Callable, Iterable, Mapping
from urllib.parse import urlsplit

from ._version import service_version
from .audit import AuditLog, redact
from .capabilities import CapabilityManifestVerifier
from .core import (
    ACTIONS,
    InvalidConfiguration,
    OperatorError,
    SessionController,
    default_config,
)
from .models import CapabilityLock, ControlRequest, FaultEvent, SessionState
from .playback import playback_manifest, resolve_playback_path
from .qualification import read_qualification_runs
from .runtime import ReadOnlySystemProbe
from .setup_workbench import SetupError, SetupWorkbench, require_setup_mode
from .simulation import RecordedSimulationRuntime


PLAYBACK_READ_ONLY_MESSAGE = (
    "Recorded playback is read-only; operator actions and configuration changes are disabled"
)
SIMULATION_CAPABILITY_LOCK_REASON = (
    "Recorded simulation mode never grants hardware authority"
)
_MAX_JSON_BODY_BYTES = 64 * 1024
_PACKAGE_PARENT = Path(__file__).resolve().parents[2]
_WATCHDOG_INTERVAL_S = 0.1
# A watchdog tick that has not returned after this long means time-based
# checks are not running (the controller lock or adapter is stuck).  Adapter
# calls are individually bounded (``DEFAULT_ADAPTER_TIMEOUTS_S``), so this
# only fires when something outside the adapter deadline is blocked.
_WATCHDOG_OVERDUE_S = 1.0

# Actor recorded for requests that do not arrive through a configured,
# trusted identity proxy.  Browser-supplied names are kept only as
# ``claimed_operator`` evidence and are never used as the audit actor.
LOCAL_CONSOLE_ACTOR = "local-console"
CAPABILITY_KEY_CREDENTIAL = "mocap-g1-capability-key"
CAPABILITY_KEY_ENV = "MOCAP_G1_CAPABILITY_KEY"
_MAX_IDENTITY_LENGTH = 256
_HTTP_TOKEN = re.compile(r"^[!#$%&'*+.^_`|~0-9A-Za-z-]+$")

logger = logging.getLogger(__name__)


def load_capability_key(environ: Mapping[str, str] | None = None) -> bytes | str | None:
    """Return the capability verification key.

    A systemd ``LoadCredential=`` file named ``mocap-g1-capability-key`` in
    ``$CREDENTIALS_DIRECTORY`` takes precedence.  When no such credential is
    present, the legacy ``MOCAP_G1_CAPABILITY_KEY`` environment variable is
    used.  An unreadable or empty credential fails closed (no key).
    """

    environ = os.environ if environ is None else environ
    credentials = environ.get("CREDENTIALS_DIRECTORY")
    if credentials:
        path = Path(credentials) / CAPABILITY_KEY_CREDENTIAL
        try:
            value = path.read_bytes()
        except FileNotFoundError:
            pass
        except OSError as exc:
            logger.error("Capability key credential is unreadable: %s", type(exc).__name__)
            return None
        else:
            return value.rstrip(b"\r\n") or None
    return environ.get(CAPABILITY_KEY_ENV)


def _identity_header_name(value: str | None) -> str | None:
    if value is None:
        return None
    name = str(value).strip()
    if not name or not _HTTP_TOKEN.match(name):
        raise ValueError(f"Invalid identity header name: {value!r}")
    if name.lower() in {"host", "origin", "content-type", "content-length"}:
        raise ValueError(f"Identity header cannot be {name!r}")
    return name.lower()


def _trusted_proxy_networks(
    values: Iterable[str],
) -> tuple[ipaddress.IPv4Network | ipaddress.IPv6Network, ...]:
    if isinstance(values, str):
        raise TypeError("trusted_proxies must be an iterable of addresses")
    networks = []
    for value in values:
        try:
            networks.append(ipaddress.ip_network(str(value).strip(), strict=False))
        except ValueError as exc:
            raise ValueError(f"Invalid trusted proxy address: {value!r}") from exc
    return tuple(networks)


def _clean_identity(value: str | None) -> str | None:
    if value is None:
        return None
    text = str(value).strip()
    if (
        not text
        or len(text) > _MAX_IDENTITY_LENGTH
        or any(ord(character) < 32 or ord(character) == 127 for character in text)
    ):
        return None
    return text


def _claimed_name(value: object) -> str | None:
    """Keep a self-asserted name as bounded, printable audit evidence."""

    if value is None:
        return None
    text = "".join(
        character
        for character in str(value)
        if ord(character) >= 32 and ord(character) != 127
    ).strip()
    return text[:_MAX_IDENTITY_LENGTH] or None


def _reject_nonfinite_json_constant(value: str) -> None:
    raise ValueError(f"Non-finite JSON number is not allowed: {value}")


def _url_origin(value: str, *, require_origin_syntax: bool = False) -> tuple[str, str, int] | None:
    """Return a comparable HTTP origin, or ``None`` for invalid input."""

    try:
        parsed = urlsplit(value)
        scheme = parsed.scheme.lower()
        if scheme == "ws":
            scheme = "http"
        elif scheme == "wss":
            scheme = "https"
        if scheme not in {"http", "https"} or parsed.hostname is None:
            return None
        if parsed.username is not None or parsed.password is not None:
            return None
        if require_origin_syntax and (
            parsed.path not in {"", "/"} or parsed.query or parsed.fragment
        ):
            return None
        port = parsed.port
    except ValueError:
        return None
    if port is None:
        port = 443 if scheme == "https" else 80
    return scheme, parsed.hostname.lower(), port


_DEFAULT_BIND_HOST = "127.0.0.1"
_DEFAULT_PORT = 8766
_LOOPBACK_NAMES = ("127.0.0.1", "localhost", "::1")
_WILDCARD_BIND_HOSTS = {"", "0.0.0.0", "::"}
_HOST_HEADER_FORBIDDEN = frozenset("/?#@\\ \t\r\n")


def _parse_host_header(value: str) -> tuple[str, int | None] | None:
    """Return ``(hostname, port)`` from a Host header, or ``None`` if invalid."""

    if not value or value.endswith(":") or any(c in _HOST_HEADER_FORBIDDEN for c in value):
        return None
    try:
        parsed = urlsplit(f"http://{value}")
        hostname = parsed.hostname
        port = parsed.port
    except ValueError:
        return None
    if not hostname:
        return None
    hostname = hostname.lower()
    try:
        hostname = ipaddress.ip_address(hostname).compressed
    except ValueError:
        pass
    return hostname, port


def _allowed_origin_entry(value: str, *, default_port: int) -> tuple[str, str, int]:
    """Parse one ``--allowed-host`` entry into a comparable origin.

    ``https://name[:port]`` / ``http://name[:port]`` are taken literally (the
    reverse-proxy form).  A bare ``name`` or ``name:port`` means plain HTTP on
    that port, defaulting to the console's own port.
    """

    text = str(value).strip()
    if "://" in text:
        origin = _url_origin(text, require_origin_syntax=True)
        if origin is None:
            raise ValueError(f"Invalid allowed host origin: {value!r}")
        return origin
    parsed = _parse_host_header(text)
    if parsed is None:
        raise ValueError(f"Invalid allowed host: {value!r}")
    hostname, port = parsed
    return "http", hostname, port if port is not None else default_port


def _allowed_origins(
    *,
    bind_host: str,
    port: int,
    allowed_hosts: Iterable[str],
) -> frozenset[tuple[str, str, int]]:
    """Build the explicit Host/Origin allowlist.

    Loopback names on the configured port are always allowed.  A specific
    (non-wildcard) bind address is added as well.  Wildcard binds add nothing:
    the operator must name the reachable host explicitly.
    """

    if isinstance(allowed_hosts, str):
        raise TypeError("allowed_hosts must be an iterable of host strings")
    if not 1 <= int(port) <= 65535:
        raise ValueError("port must be between 1 and 65535")
    port = int(port)
    origins = {("http", name, port) for name in _LOOPBACK_NAMES}
    bind = str(bind_host).strip().strip("[]").lower()
    if bind not in _WILDCARD_BIND_HOSTS:
        try:
            bind = ipaddress.ip_address(bind).compressed
        except ValueError:
            pass
        if any(c in _HOST_HEADER_FORBIDDEN for c in bind) or not bind:
            raise ValueError(f"Invalid bind host: {bind_host!r}")
        origins.add(("http", bind, port))
    for entry in allowed_hosts:
        origins.add(_allowed_origin_entry(entry, default_port=port))
    return frozenset(origins)


class NetworkExposureError(ValueError):
    """The console would be reachable beyond loopback without an identity proxy."""


def _is_loopback_host(value: str) -> bool:
    name = str(value).strip().strip("[]").lower()
    if name == "localhost":
        return True
    try:
        address = ipaddress.ip_address(name)
    except ValueError:
        return False
    if isinstance(address, ipaddress.IPv6Address) and address.ipv4_mapped is not None:
        address = address.ipv4_mapped
    return address.is_loopback


def require_identity_proxy_for_network_access(
    *,
    bind_host: str,
    port: int,
    allowed_hosts: Iterable[str],
    identity_header: str | None,
    trusted_proxies: Iterable[str],
) -> None:
    """Refuse any non-loopback exposure unless an identity proxy is configured.

    The console has no user authentication of its own.  Binding anything other
    than a loopback address (wildcards included), or trusting any non-loopback
    Host/Origin, is therefore allowed only behind an authenticating reverse
    proxy that is named with both ``identity_header`` and ``trusted_proxies``.
    """

    if isinstance(allowed_hosts, str) or isinstance(trusted_proxies, str):
        raise TypeError("allowed_hosts and trusted_proxies must be iterables of strings")
    exposed = []
    if not _is_loopback_host(bind_host):
        exposed.append(f"bind address {bind_host!r}")
    for entry in allowed_hosts:
        _scheme, hostname, _port = _allowed_origin_entry(entry, default_port=int(port))
        if not _is_loopback_host(hostname):
            exposed.append(f"allowed host {entry!r}")
    if not exposed:
        return
    has_header = bool(identity_header and str(identity_header).strip())
    has_proxies = any(str(value).strip() for value in trusted_proxies)
    if has_header and has_proxies:
        return
    raise NetworkExposureError(
        "Refusing to expose the operator console beyond loopback ("
        + ", ".join(exposed)
        + ") without an authenticating identity proxy. The console has no "
        "login of its own: put it behind an authenticated reverse proxy and set "
        "both --identity-header and --trusted-proxy (or MOCAP_G1_IDENTITY_HEADER "
        "and MOCAP_G1_TRUSTED_PROXIES), or bind to 127.0.0.1, ::1 or localhost "
        "with loopback --allowed-host values only."
    )


def require_loopback_only_for_setup(
    *,
    bind_host: str,
    port: int,
    allowed_hosts: Iterable[str],
    identity_header: str | None,
    trusted_proxies: Iterable[str],
) -> None:
    """Refuse the Setup workbench unless the console is reachable from loopback only.

    The workbench runs local commands and writes session files, so it is never
    offered beyond this PC, not even behind an authenticating identity proxy.
    """

    if isinstance(allowed_hosts, str) or isinstance(trusted_proxies, str):
        raise TypeError("allowed_hosts and trusted_proxies must be iterables of strings")
    reasons = []
    if not _is_loopback_host(bind_host):
        reasons.append(f"bind address {bind_host!r}")
    for entry in allowed_hosts:
        _scheme, hostname, _port = _allowed_origin_entry(entry, default_port=int(port))
        if not _is_loopback_host(hostname):
            reasons.append(f"allowed host {entry!r}")
    if identity_header and str(identity_header).strip():
        reasons.append("an identity header")
    if any(str(value).strip() for value in trusted_proxies):
        reasons.append("trusted proxies")
    if reasons:
        raise ValueError(
            "--setup runs local setup commands and is only available on a loopback-only "
            "console; remove " + ", ".join(reasons) + " (bind 127.0.0.1 and open the "
            "console on this PC)"
        )


def _host_is_allowed(
    host_header: str | None,
    allowed: frozenset[tuple[str, str, int]],
) -> bool:
    if host_header is None:
        return False
    parsed = _parse_host_header(host_header)
    if parsed is None:
        return False
    hostname, port = parsed
    for scheme, allowed_name, allowed_port in allowed:
        if hostname != allowed_name:
            continue
        if port is None:
            # A port-less Host means the default port of the client-facing
            # scheme (e.g. a TLS reverse proxy forwarding ``Host: name``).
            if allowed_port == (443 if scheme == "https" else 80):
                return True
        elif port == allowed_port:
            return True
    return False


class _TrustedHostGuard:
    """ASGI guard rejecting requests whose Host header is not allowlisted.

    Starlette builds ``request.url`` from the client-supplied Host header, so
    a DNS-rebinding page can make Host and Origin agree on an attacker name.
    This runs before routing for HTTP and WebSocket scopes.
    """

    def __init__(self, app: Any, *, allowed: frozenset[tuple[str, str, int]]):
        self.app = app
        self.allowed = allowed

    async def __call__(self, scope: Any, receive: Any, send: Any) -> None:
        if scope["type"] not in {"http", "websocket"}:
            await self.app(scope, receive, send)
            return
        hosts = [
            value.decode("latin-1")
            for key, value in scope.get("headers", ())
            if key.lower() == b"host"
        ]
        host = hosts[0] if len(hosts) == 1 else None
        if _host_is_allowed(host, self.allowed):
            await self.app(scope, receive, send)
            return
        if scope["type"] == "websocket":
            await send({"type": "websocket.close", "code": 1008, "reason": "Invalid host header"})
            return
        body = json.dumps({"detail": "Invalid host header"}).encode("utf-8")
        await send(
            {
                "type": "http.response.start",
                "status": 400,
                "headers": [
                    (b"content-type", b"application/json"),
                    (b"content-length", str(len(body)).encode("ascii")),
                    (b"cache-control", b"no-store"),
                ],
            }
        )
        await send({"type": "http.response.body", "body": body})


_FRAME_ANCESTORS_NONE = b"frame-ancestors 'none'"


class _FrameDenyHeaders:
    """ASGI middleware forbidding any page from framing the console.

    A framed console could be clicked into a same-origin action such as
    ``start-tracking``.  ``X-Frame-Options: DENY`` and a CSP
    ``frame-ancestors 'none'`` directive are set on every HTTP response:
    static assets, API results, handled API errors, the host guard's
    rejections, and unhandled-exception 500s.  ``create_app`` therefore wraps
    the *whole* built stack with it, outside Starlette's ServerErrorMiddleware
    (which renders unhandled exceptions and is otherwise the outermost layer).
    """

    def __init__(self, app: Any):
        self.app = app

    async def __call__(self, scope: Any, receive: Any, send: Any) -> None:
        if scope["type"] != "http":
            await self.app(scope, receive, send)
            return

        async def send_with_headers(message: Any) -> None:
            if message["type"] == "http.response.start":
                headers = []
                csp_seen = False
                for name, value in message.get("headers", ()):
                    lowered = name.lower()
                    if lowered == b"x-frame-options":
                        continue
                    if lowered == b"content-security-policy":
                        csp_seen = True
                        if b"frame-ancestors" not in value.lower():
                            value = value + b"; " + _FRAME_ANCESTORS_NONE
                    headers.append((name, value))
                headers.append((b"x-frame-options", b"DENY"))
                if not csp_seen:
                    headers.append((b"content-security-policy", _FRAME_ANCESTORS_NONE))
                message = {**message, "headers": headers}
            await send(message)

        await self.app(scope, receive, send_with_headers)


def _read_json_object(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    with path.open("r", encoding="utf-8") as handle:
        value = json.load(handle)
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def create_app(
    *,
    data_root: str | Path | None = None,
    demo: bool = False,
    playback: str | Path | None = None,
    simulation: str | Path | None = None,
    qualification_root: str | Path | None = None,
    bind_host: str = _DEFAULT_BIND_HOST,
    port: int = _DEFAULT_PORT,
    allowed_hosts: Iterable[str] = (),
    identity_header: str | None = None,
    trusted_proxies: Iterable[str] = (),
    setup_root: "str | Path | SetupWorkbench | None" = None,
):
    """Create the operator web application.

    FastAPI is imported only when this factory is called, so robot-side code
    and core tests do not acquire a web-framework dependency.  ``demo=True``
    uses the fake runtime and never grants real robot command authority.
    ``simulation`` replays one validated qpos NPZ as a kinematic preview.

    Every HTTP request and WebSocket handshake must carry a Host header on an
    explicit allowlist: ``127.0.0.1``, ``localhost`` and ``[::1]`` on ``port``,
    the specific ``bind_host`` (never a wildcard), and each ``allowed_hosts``
    entry (``https://name[:port]`` for a reverse proxy, or ``name[:port]``).
    Browser ``Origin`` headers are matched against the same allowlist.

    Audit attribution is taken from ``identity_header`` only on requests whose
    TCP peer is in ``trusted_proxies`` (addresses or CIDR networks).  Every
    other request is attributed to ``LOCAL_CONSOLE_ACTOR``; a browser-supplied
    operator name is recorded separately as ``claimed_operator``.

    ``setup_root`` (a source checkout, or a prepared ``SetupWorkbench``)
    enables the ``/api/v1/setup*`` Setup workspace.  It is refused with
    ``playback``/``simulation`` and on any console that is not loopback-only.
    Without it every Setup route answers 404.
    """

    selected_modes = int(bool(demo)) + int(playback is not None) + int(
        simulation is not None
    )
    if selected_modes > 1:
        raise ValueError("demo, playback, and simulation modes are mutually exclusive")
    allowed_origins = _allowed_origins(
        bind_host=bind_host,
        port=port,
        allowed_hosts=allowed_hosts,
    )
    allowed_hosts = tuple(allowed_hosts)
    trusted_proxies = tuple(trusted_proxies)
    identity_header_name = _identity_header_name(identity_header)
    trusted_proxy_networks = _trusted_proxy_networks(trusted_proxies)
    if (identity_header_name is None) != (not trusted_proxy_networks):
        raise ValueError(
            "identity_header and trusted_proxies must be configured together"
        )
    require_identity_proxy_for_network_access(
        bind_host=bind_host,
        port=port,
        allowed_hosts=allowed_hosts,
        identity_header=identity_header_name,
        trusted_proxies=trusted_proxies,
    )
    setup: SetupWorkbench | None = None
    if setup_root is not None:
        require_setup_mode(playback=playback is not None, simulation=simulation is not None)
        require_loopback_only_for_setup(
            bind_host=bind_host,
            port=port,
            allowed_hosts=allowed_hosts,
            identity_header=identity_header_name,
            trusted_proxies=trusted_proxies,
        )
        setup = (
            setup_root
            if isinstance(setup_root, SetupWorkbench)
            else SetupWorkbench(setup_root)
        )

    try:
        from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
        from fastapi.responses import FileResponse, JSONResponse, Response
        from fastapi.staticfiles import StaticFiles
    except ImportError as exc:  # pragma: no cover - environment dependent
        raise RuntimeError(
            "The operator web console requires FastAPI; install the console dependencies"
        ) from exc

    playback_path = resolve_playback_path(playback) if playback is not None else None
    simulation_runtime = (
        RecordedSimulationRuntime(simulation) if simulation is not None else None
    )
    simulation_path = (
        simulation_runtime.source_path if simulation_runtime is not None else None
    )
    qualification_path = (
        Path(qualification_root).expanduser()
        if qualification_root is not None
        else None
    )
    if playback_path is not None or simulation_path is not None:
        # Recorded review is deliberately isolated from production state.  It
        # neither reads nor writes the configured data root, manifest, or key.
        root = None
        audit = AuditLog()
        config_path = None
        config = default_config()
        setup_complete = True
    else:
        root = Path(data_root) if data_root is not None else Path.cwd() / ".operator-data"
        audit = AuditLog(root / "audit.jsonl")
        config_path = root / "config.json"
        setup_complete = True
        try:
            config = _read_json_object(config_path)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            config = default_config(demo=demo)
            setup_complete = False
            audit.append(
                category="configuration",
                action="load",
                outcome="rejected",
                state="setup_required",
                details={"reason": str(exc)},
            )
        # runtime_mode is not read from config: SessionController derives it
        # from the launcher's runtime choice (``demo`` / simulation).

    try:
        controller = SessionController(
            audit=audit,
            config=config,
            config_path=config_path,
            runtime=simulation_runtime,
            demo=demo,
            setup_complete=setup_complete,
        )
    except InvalidConfiguration as exc:
        # A config file that parses but fails validation must not crash-loop
        # the service.  Start from defaults in setup_required, exactly as for
        # an unparseable file; the operator saves a valid configuration.
        audit.append(
            category="configuration",
            action="load",
            outcome="rejected",
            state="setup_required",
            details={"reason": str(exc), "code": exc.code},
        )
        controller = SessionController(
            audit=audit,
            config=default_config(demo=demo),
            config_path=config_path,
            runtime=simulation_runtime,
            demo=demo,
            setup_complete=False,
        )

    if playback_path is None and simulation_path is None:
        assert root is not None
        try:
            manifest = _read_json_object(root / "capability_manifest.json")
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            manifest = None
            audit.append(
                category="capability",
                action="load_manifest",
                outcome="rejected",
                state=controller.state.value,
                details={"reason": str(exc)},
            )
        verifier = CapabilityManifestVerifier(load_capability_key())
        controller.set_capability_lock(verifier.verify(manifest))
    elif simulation_path is not None:
        controller.set_capability_lock(
            CapabilityLock(reason=SIMULATION_CAPABILITY_LOCK_REASON)
        )

    def latch_after_tick_failure(exc: Exception) -> None:
        # A failing tick means expiry and stale-source checks are not being
        # enforced; never let command authority continue without them.
        try:
            if controller.state is SessionState.TRACKING:
                controller.latch_fault(
                    FaultEvent(
                        code="watchdog_tick_failed",
                        reason=f"Operator watchdog tick failed: {type(exc).__name__}: {exc}",
                        source="watchdog",
                        recoverable=True,
                    ),
                    actor="watchdog",
                )
        except Exception:
            logger.exception("Could not latch a fault after a watchdog tick failure")

    def run_detached(target: Callable[..., None], *args: Any) -> None:
        # For calls that may block on the controller lock while it is stuck:
        # neither the event loop nor the watchdog loop may wait on them.
        threading.Thread(
            target=target, args=args, name="mocap-g1-watchdog-report", daemon=True
        ).start()

    def report_overdue(elapsed_s: float) -> None:
        try:
            controller.report_watchdog_overdue(elapsed_s)
        except Exception:
            logger.exception("Could not record an overdue watchdog tick")

    async def watchdog_loop() -> None:
        loop = asyncio.get_running_loop()
        pending: "asyncio.Future[None] | None" = None
        started = 0.0
        overdue_reported = False
        while True:
            try:
                if pending is None:
                    # The controller lock may be held by blocking I/O (audit
                    # or config fsync) or a bounded adapter call; keep that
                    # wait off the event loop.  Never start a second tick
                    # while one is still running.
                    pending = asyncio.ensure_future(
                        asyncio.to_thread(controller.watchdog_tick)
                    )
                    started = loop.time()
                    overdue_reported = False
                await asyncio.wait({pending}, timeout=_WATCHDOG_INTERVAL_S)
                if not pending.done():
                    elapsed = loop.time() - started
                    application_state.watchdog_overdue = elapsed >= _WATCHDOG_OVERDUE_S
                    if elapsed >= _WATCHDOG_OVERDUE_S and not overdue_reported:
                        overdue_reported = True
                        logger.critical(
                            "Operator watchdog tick has not returned for %.3f s", elapsed
                        )
                        run_detached(report_overdue, elapsed)
                    continue
                finished, pending = pending, None
                application_state.watchdog_overdue = False
                application_state.watchdog_completed_at = loop.time()
                error = finished.exception()
                if error is not None:
                    logger.error(
                        "Operator watchdog tick failed",
                        exc_info=(type(error), error, error.__traceback__),
                    )
                    await asyncio.to_thread(latch_after_tick_failure, error)
                await asyncio.sleep(_WATCHDOG_INTERVAL_S)
            except asyncio.CancelledError:
                raise
            except Exception:
                # The loop itself must never end on an unexpected error.
                logger.exception("Operator watchdog loop error")
                await asyncio.sleep(_WATCHDOG_INTERVAL_S)

    @asynccontextmanager
    async def lifespan(application: Any):
        if getattr(application.state, "watchdog_task", None) is not None:
            raise RuntimeError("Operator watchdog task is already running")
        stopping = False

        def on_watchdog_done(finished: "asyncio.Task[None]") -> None:
            if stopping:
                return
            if finished.cancelled():
                cause = "cancelled"
            else:
                error = finished.exception()
                cause = f"{type(error).__name__}: {error}" if error else "returned"
            logger.critical("Operator watchdog stopped unexpectedly (%s)", cause)
            run_detached(latch_watchdog_stopped, cause)

        def latch_watchdog_stopped(cause: str) -> None:
            # Runs on its own thread: the event loop must not wait on the
            # controller lock, which a stuck call may be holding.
            try:
                controller.latch_fault(
                    FaultEvent(
                        code="watchdog_stopped",
                        reason=(
                            "Operator watchdog stopped unexpectedly; time-based "
                            f"safety checks are no longer enforced ({cause})"
                        ),
                        source="watchdog",
                        recoverable=False,
                    ),
                    actor="watchdog",
                )
            except Exception:
                logger.exception("Could not latch the watchdog-stopped fault")

        task = asyncio.create_task(
            watchdog_loop(),
            name="mocap-g1-operator-watchdog",
        )
        task.add_done_callback(on_watchdog_done)
        application.state.watchdog_task = task
        if setup is not None:
            # App mode only: watch the Xsens port for the Setup screen.
            try:
                await asyncio.to_thread(setup.start_stream_monitor)
            except Exception:
                logger.exception("Could not start the Xsens stream monitor")
        try:
            yield
        finally:
            stopping = True
            task.cancel()
            try:
                with suppress(asyncio.CancelledError):
                    await task
            finally:
                application.state.watchdog_task = None
                if setup is not None:
                    # Never leave a setup job (stream check, live viewer,
                    # render) running after the console exits; this also
                    # closes the stream monitor and releases the UDP port.
                    try:
                        await asyncio.to_thread(setup.shutdown)
                    except Exception:
                        logger.exception("Could not stop the running setup job")

    class _OperatorConsoleApp(FastAPI):
        def build_middleware_stack(self) -> Any:
            # Outermost layer, outside ServerErrorMiddleware, so the 500 it
            # sends for an unhandled exception also denies framing.
            return _FrameDenyHeaders(super().build_middleware_stack())

    app = _OperatorConsoleApp(
        title="Mocap Operator Console",
        version=service_version(),
        docs_url="/api/docs",
        redoc_url=None,
        openapi_url="/api/openapi.json",
        lifespan=lifespan,
    )
    app.state.controller = controller
    app.state.data_root = root
    app.state.demo = bool(demo)
    app.state.playback = playback_path
    app.state.simulation = simulation_path
    app.state.qualification_root = qualification_path
    app.state.watchdog_task = None
    app.state.watchdog_overdue = False
    app.state.watchdog_completed_at = None
    app.state.allowed_origins = allowed_origins
    app.state.setup_workbench = setup
    application_state = app.state
    # Runs before routing.  _FrameDenyHeaders wraps the whole built stack
    # (see _OperatorConsoleApp), so the guard's rejections are covered too.
    app.add_middleware(_TrustedHostGuard, allowed=allowed_origins)

    def public_status() -> dict[str, Any]:
        status = controller.status_snapshot()
        task = application_state.watchdog_task
        status["watchdog"] = {
            "running": task is not None and not task.done(),
            "overdue": bool(application_state.watchdog_overdue),
        }
        status["presentation_mode"] = (
            "recorded_playback"
            if playback_path is not None
            else "recorded_simulation"
            if simulation_path is not None
            else "demo"
            if demo
            else "live"
        )
        if playback_path is not None:
            status["health"] = dict(status["health"])
            status["health"]["command_authority"] = False
            status["capability_lock"] = dict(status["capability_lock"])
            status["capability_lock"].update(
                whole_body_unlocked=False,
                reason="Recorded playback mode never grants hardware authority",
            )
            status["controls"] = {
                name: {"enabled": False, "reason": PLAYBACK_READ_ONLY_MESSAGE}
                for name in status["controls"]
            }
        elif simulation_path is not None:
            status["health"] = dict(status["health"])
            status["health"]["command_authority"] = False
            status["capability_lock"] = dict(status["capability_lock"])
            status["capability_lock"].update(
                whole_body_unlocked=False,
                manifest_verified=False,
                reason=SIMULATION_CAPABILITY_LOCK_REASON,
            )
        return status

    def reject_playback_mutation() -> None:
        if playback_path is not None:
            raise OperatorError(
                PLAYBACK_READ_ONLY_MESSAGE,
                code="playback_read_only",
                status_code=423,
            )

    def require_same_origin(origin: str | None) -> None:
        # Browsers send Origin on cross-origin POSTs and WebSocket handshakes.
        # Non-browser clients often omit it, so absence remains valid.  The
        # comparison is against the explicit allowlist, never ``request.url``,
        # which Starlette derives from the client-controlled Host header.
        if origin is None:
            return
        supplied = _url_origin(origin, require_origin_syntax=True)
        if supplied is None or supplied not in allowed_origins:
            raise HTTPException(status_code=403, detail="Cross-origin request is not allowed")

    async def read_json_object(
        request: Request,
        *,
        label: str,
        allow_empty: bool,
    ) -> Mapping[str, Any]:
        declared_length = request.headers.get("content-length")
        if declared_length is not None:
            try:
                parsed_length = int(declared_length)
            except ValueError as exc:
                raise HTTPException(
                    status_code=400,
                    detail="Content-Length must be a non-negative integer",
                ) from exc
            if parsed_length < 0:
                raise HTTPException(
                    status_code=400,
                    detail="Content-Length must be a non-negative integer",
                )
            if parsed_length > _MAX_JSON_BODY_BYTES:
                raise HTTPException(status_code=413, detail="Request body is too large")

        raw = bytearray()
        async for chunk in request.stream():
            if len(raw) + len(chunk) > _MAX_JSON_BODY_BYTES:
                raise HTTPException(status_code=413, detail="Request body is too large")
            raw.extend(chunk)

        if not raw:
            if allow_empty:
                return {}
            raise HTTPException(status_code=400, detail="Request body must be JSON")

        media_type = request.headers.get("content-type", "").partition(";")[0]
        media_type = media_type.strip().lower()
        if media_type != "application/json" and not media_type.endswith("+json"):
            raise HTTPException(
                status_code=415,
                detail="Content-Type must be application/json",
            )

        try:
            body = json.loads(raw, parse_constant=_reject_nonfinite_json_constant)
        except (
            json.JSONDecodeError,
            RecursionError,
            UnicodeDecodeError,
            ValueError,
        ) as exc:
            raise HTTPException(status_code=400, detail="Request body must be JSON") from exc
        if not isinstance(body, Mapping):
            raise HTTPException(status_code=422, detail=f"{label} must be an object")
        return body

    def request_actor(request: Request) -> str:
        """Attribute a request to a trusted proxy identity or the local console."""

        if identity_header_name is None or request.client is None:
            return LOCAL_CONSOLE_ACTOR
        try:
            peer = ipaddress.ip_address(request.client.host)
        except ValueError:
            return LOCAL_CONSOLE_ACTOR
        if isinstance(peer, ipaddress.IPv6Address) and peer.ipv4_mapped is not None:
            peer = peer.ipv4_mapped
        if not any(peer in network for network in trusted_proxy_networks):
            return LOCAL_CONSOLE_ACTOR
        values = request.headers.getlist(identity_header_name)
        if len(values) != 1:
            return LOCAL_CONSOLE_ACTOR
        return _clean_identity(values[0]) or LOCAL_CONSOLE_ACTOR

    @app.exception_handler(OperatorError)
    async def operator_error_handler(_request: Request, exc: OperatorError):
        return JSONResponse(
            status_code=exc.status_code,
            content={"error": {"code": exc.code, "message": str(exc)}},
        )

    @app.exception_handler(SetupError)
    async def setup_error_handler(_request: Request, exc: SetupError):
        return JSONResponse(status_code=exc.status_code, content=exc.payload())

    def require_setup() -> SetupWorkbench:
        if setup is None:
            raise HTTPException(status_code=404, detail="The Setup workspace is not enabled")
        return setup

    @app.get("/api/v1/setup")
    async def get_setup():
        workbench = require_setup()
        return await asyncio.to_thread(workbench.payload)

    @app.put("/api/v1/setup/session")
    async def put_setup_session(request: Request):
        workbench = require_setup()
        require_same_origin(request.headers.get("origin"))
        body = await read_json_object(request, label="Session values", allow_empty=False)
        await asyncio.to_thread(workbench.save_session, body)
        return await asyncio.to_thread(workbench.payload)

    @app.post("/api/v1/setup/jobs", status_code=201)
    async def post_setup_job(request: Request):
        workbench = require_setup()
        require_same_origin(request.headers.get("origin"))
        body = await read_json_object(request, label="Job request", allow_empty=False)
        kind = body.get("kind")
        if not isinstance(kind, str):
            raise SetupError("kind is required", code="invalid_job", status_code=422)
        return await asyncio.to_thread(workbench.start_job, kind, body.get("options"))

    @app.get("/api/v1/setup/jobs/{job_id}")
    async def get_setup_job(job_id: str, after: int = 0):
        workbench = require_setup()
        return await asyncio.to_thread(workbench.job_log, job_id, after)

    @app.post("/api/v1/setup/jobs/{job_id}/stop")
    async def stop_setup_job(job_id: str, request: Request):
        workbench = require_setup()
        require_same_origin(request.headers.get("origin"))
        await read_json_object(request, label="Stop request", allow_empty=True)
        return await asyncio.to_thread(workbench.stop_job, job_id)

    @app.post("/api/v1/setup/jobs/{job_id}/control", status_code=202)
    async def control_setup_job(job_id: str, request: Request):
        workbench = require_setup()
        require_same_origin(request.headers.get("origin"))
        body = await read_json_object(request, label="Control request", allow_empty=False)
        return await asyncio.to_thread(workbench.control_job, job_id, body.get("command"))

    @app.get("/api/v1/setup/stream")
    async def get_setup_stream():
        workbench = require_setup()
        return await asyncio.to_thread(workbench.stream_snapshot)

    @app.get("/api/v1/setup/prefs")
    async def get_setup_prefs():
        workbench = require_setup()
        return await asyncio.to_thread(workbench.prefs)

    @app.put("/api/v1/setup/prefs")
    async def put_setup_prefs(request: Request):
        workbench = require_setup()
        require_same_origin(request.headers.get("origin"))
        body = await read_json_object(request, label="Preferences", allow_empty=True)
        return await asyncio.to_thread(workbench.save_prefs, body)

    @app.post("/api/v1/setup/viewer/stop")
    async def stop_setup_viewer(request: Request):
        workbench = require_setup()
        require_same_origin(request.headers.get("origin"))
        await read_json_object(request, label="Stop request", allow_empty=False)
        return await asyncio.to_thread(workbench.stop_viewer)

    @app.get("/api/v1/setup/files/{name}")
    async def get_setup_file(name: str):
        workbench = require_setup()
        path, media_type = await asyncio.to_thread(workbench.file_path, name)
        return FileResponse(
            path,
            media_type=media_type,
            filename=path.name,
            content_disposition_type="inline",
            headers={
                "Cache-Control": "private, no-store",
                "Cross-Origin-Resource-Policy": "same-origin",
                "X-Content-Type-Options": "nosniff",
            },
        )

    @app.get("/api/v1/status")
    async def get_status():
        return await asyncio.to_thread(public_status)

    @app.get("/api/v1/config")
    async def get_config():
        return await asyncio.to_thread(controller.get_config)

    @app.get("/api/v1/playback")
    async def get_playback():
        if playback_path is None:
            raise HTTPException(status_code=404, detail="No recorded review is configured")
        return playback_manifest(playback_path)

    @app.get("/api/v1/playback/video")
    async def get_playback_video():
        if playback_path is None:
            raise HTTPException(status_code=404, detail="No recorded review is configured")
        return FileResponse(
            playback_path,
            media_type="video/mp4",
            filename=playback_path.name,
            content_disposition_type="inline",
            headers={
                "Cache-Control": "private, no-store",
                "Cross-Origin-Resource-Policy": "same-origin",
                "X-Content-Type-Options": "nosniff",
            },
        )

    @app.get("/api/v1/qualification-runs")
    async def get_qualification_runs():
        if qualification_path is None:
            return {"runs": []}
        return await asyncio.to_thread(
            read_qualification_runs,
            qualification_path,
        )

    @app.put("/api/v1/config")
    async def put_config(request: Request):
        require_same_origin(request.headers.get("origin"))
        reject_playback_mutation()
        body = await read_json_object(
            request,
            label="Configuration",
            allow_empty=False,
        )
        return await asyncio.to_thread(
            controller.update_config,
            body,
            actor=request_actor(request),
            claimed_operator=_claimed_name(request.headers.get("x-operator-name")),
        )

    @app.post("/api/v1/actions/{action}")
    async def post_action(action: str, request: Request):
        require_same_origin(request.headers.get("origin"))
        reject_playback_mutation()
        if action not in ACTIONS:
            raise HTTPException(status_code=404, detail="Unknown action")
        body = await read_json_object(
            request,
            label="Action body",
            allow_empty=True,
        )
        metadata = body.get("metadata", {})
        if not isinstance(metadata, Mapping):
            raise HTTPException(status_code=422, detail="metadata must be an object")
        spotter = body.get("spotter")
        if demo and action == "confirm-preflight" and not spotter:
            spotter = "demo-spotter"
            metadata = {
                **metadata,
                "clear_area": True,
                "harness_or_support": True,
                "stop_drill": True,
            }
        claimed = body.get("operator")
        try:
            control_request = ControlRequest(
                action=action,
                operator=request_actor(request),
                claimed_operator=_claimed_name(claimed),
                spotter=str(spotter) if spotter is not None else None,
                request_id=(
                    str(body["request_id"]) if body.get("request_id") is not None else None
                ),
                metadata=dict(metadata),
            )
        except (TypeError, ValueError) as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc
        await asyncio.to_thread(controller.perform, control_request)
        return await asyncio.to_thread(public_status)

    @app.get("/api/v1/audit")
    async def get_audit(after_sequence: int = 0, limit: int = 200, newest: bool = False):
        try:
            records = await asyncio.to_thread(
                audit.read, after_sequence=after_sequence, limit=limit, newest=newest
            )
        except ValueError as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc
        return {"records": records, "last_sequence": audit.sequence}

    @app.get("/api/v1/diagnostics")
    async def get_diagnostics(request: Request):
        require_same_origin(request.headers.get("origin"))
        body, filename = await asyncio.to_thread(build_diagnostics)
        return Response(
            content=body,
            media_type="application/json",
            headers={"Content-Disposition": f'attachment; filename="{filename}"'},
        )

    def build_diagnostics() -> tuple[bytes, str]:
        # Deep-copying, redacting, and serializing thousands of audit records
        # is CPU-bound; this runs in a worker thread, not on the event loop.
        generated = datetime.now(timezone.utc)
        # AuditLog redacts on append; redact again at the export boundary so a
        # legacy/imported record can never bypass the current policy.
        export = {
            "schema_version": 1,
            "service_version": service_version(),
            "generated_at": generated.isoformat().replace("+00:00", "Z"),
            "status": public_status(),
            "config": controller.get_config(),
            "audit": redact(audit.read(limit=5000, newest=True)),
            # Fresh re-read of the whole file; breaks are listed, not repaired.
            "audit_integrity": audit.verify_chain(),
            # The directory containing the ``mocap_g1`` package: the checkout
            # for an editable/source run, site-packages for a wheel install.
            "system_probe": ReadOnlySystemProbe(_PACKAGE_PARENT).snapshot(),
        }
        filename = f"mocap-g1-diagnostics-{generated.strftime('%Y%m%dT%H%M%SZ')}.json"
        body = json.dumps(
            export,
            ensure_ascii=False,
            allow_nan=False,
            separators=(",", ":"),
        ).encode("utf-8")
        return body, filename

    @app.websocket("/api/v1/events")
    async def events(websocket: WebSocket):
        try:
            require_same_origin(websocket.headers.get("origin"))
        except HTTPException:
            await websocket.close(code=1008, reason="Cross-origin request is not allowed")
            return
        await websocket.accept()
        try:
            requested = websocket.query_params.get("after_sequence", "0")
            sequence = max(0, int(requested))
        except ValueError:
            sequence = 0
        status = await asyncio.to_thread(public_status)
        # A cursor from a previous process can exceed this instance's counter.
        # Clamp it so new events are never silently skipped; the client sees
        # the changed ``instance_id`` and re-bootstraps.
        sequence = min(sequence, int(status["last_event_sequence"]))
        await websocket.send_json(
            {
                "schema_version": 1,
                "instance_id": controller.instance_id,
                "sequence": sequence,
                "type": "snapshot",
                "status": status,
            }
        )
        loop = asyncio.get_running_loop()
        next_snapshot = loop.time() + _WATCHDOG_INTERVAL_S
        try:
            while True:
                events = await asyncio.to_thread(controller.events_after, sequence)
                for event in events:
                    await websocket.send_json(event)
                    sequence = event["sequence"]
                now = loop.time()
                if now >= next_snapshot:
                    await websocket.send_json(
                        {
                            "schema_version": 1,
                            "instance_id": controller.instance_id,
                            "sequence": sequence,
                            "type": "snapshot",
                            "status": await asyncio.to_thread(public_status),
                        }
                    )
                    next_snapshot = now + _WATCHDOG_INTERVAL_S
                # ASGI delivers both client closes and server-shutdown closes
                # through receive().  Poll it with a short timeout so an idle
                # event stream cannot keep Uvicorn waiting during shutdown.
                receive_timeout = max(
                    0.001,
                    min(_WATCHDOG_INTERVAL_S, next_snapshot - loop.time()),
                )
                try:
                    message = await asyncio.wait_for(
                        websocket.receive(), timeout=receive_timeout
                    )
                except asyncio.TimeoutError:
                    continue
                if message.get("type") == "websocket.disconnect":
                    return
        except WebSocketDisconnect:
            return

    static_root = Path(__file__).with_name("static")
    if static_root.is_dir() and (static_root / "index.html").is_file():
        favicon_path = static_root / "favicon.svg"
        if favicon_path.is_file():
            @app.get("/favicon.ico", include_in_schema=False)
            async def get_favicon():
                return FileResponse(
                    favicon_path,
                    media_type="image/svg+xml",
                    headers={"Cache-Control": "public, max-age=86400"},
                )

        app.mount("/", StaticFiles(directory=static_root, html=True), name="operator-static")
    else:
        @app.get("/")
        async def service_description():
            return {
                "service": "Mocap Operator Console",
                "api": "/api/v1/status",
                "documentation": "/api/docs",
                "ui": "Static console assets are not installed",
            }

    return app


__all__ = [
    "LOCAL_CONSOLE_ACTOR",
    "NetworkExposureError",
    "create_app",
    "load_capability_key",
    "require_identity_proxy_for_network_access",
    "require_loopback_only_for_setup",
]
