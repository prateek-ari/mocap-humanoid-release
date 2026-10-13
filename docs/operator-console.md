# Mocap G1 operator console

## Safety and current capability

The operator console replaces routine terminal use with a browser-based session
and status view. It is intentionally separate from robot command authority. The
provided launcher and systemd unit **never start robot control**, mocap ingest,
DDS, or a motion policy.

Full-body hardware mirroring is currently **locked and unvalidated**. A disabled
control in the GUI is a real capability boundary, not an instruction to work
around it. It may only be unlocked after the controller, safety supervisor, and
staged hardware qualification described in the bring-up runbook have passed.

`DISENGAGE` requests a controlled software stop. It is **not an emergency stop**
and is not a substitute for the Unitree remote/damping path, the physical stop
procedure, a clear test area, harness, trained operator, and spotter. If the UI
is unavailable or the robot behaves unexpectedly, use the independent physical
stop/damping procedure first.

Keyboard: `Esc` presses `DISENGAGE` with the same gating as the on-screen key
(it still works while the event stream is lost). It is ignored while typing in a
text field, with Ctrl/Alt/Meta held, or on key repeat. It is not an emergency
stop either.

The statements above describe the default console and the supplied service.
**Setup mode** (`--setup`, below) is a separate, opt-in simulation workbench
that does start the stream check and the MuJoCo viewer; it still never opens
DDS or a robot transport.

The operator must still start and calibrate Xsens Humanoid manually. Robot
power-up, inspection, placement, vendor operating mode, and independent stop
checks also remain manual. The GUI reports the evidence it can observe; it does
not claim that a manual step happened merely because a button was pressed.

## Supported layouts

### Ubuntu all-in-one (recommended)

Use a native Ubuntu workstation for Xsens Humanoid, the operator console, the
future supervised controller, and the Unitree connection. Keep three network
roles separate where practical:

- Xsens/capture NIC: mocap UDP input, normally port `5555`.
- Unitree NIC: robot DDS traffic only.
- Management NIC or Wi-Fi: administration and optional remote browser access.

Run the console on `127.0.0.1:8766` and use the desktop shortcut on the Ubuntu
workstation. Do not bridge the Xsens and Unitree networks and do not enable IP
forwarding between them.

### Windows capture to Ubuntu control

Run and calibrate Xsens Humanoid natively on Windows, and set its UDP destination
to the fixed address of the Ubuntu capture NIC. The console and all robot-side
software run natively on Ubuntu. Allow UDP `5555` on the Ubuntu capture NIC only
from the paired Windows host. Use DHCP reservations or static addresses so the
pairing does not silently change.

Operate the console on the Ubuntu workstation where possible. If the Windows PC
must display it, keep the application bound to loopback and put an authenticated
HTTPS reverse proxy on the Ubuntu management interface. Permit TCP `443` only
from the paired management address.

The console has no login of its own, so it refuses at startup to be reachable
beyond loopback without that proxy. A non-loopback `--host` (including
`0.0.0.0` / `::`) or any non-loopback `--allowed-host` is accepted only when
both `--identity-header` and `--trusted-proxy` are set (see
[Audit attribution](#audit-attribution)); otherwise `mocap-g1-console` exits
with status `2` and an `ERROR: Refusing to expose the operator console beyond
loopback ...` message. Loopback binds and allowlist entries (`127.0.0.0/8`,
`::1`, `localhost`) need no proxy. Plain direct HTTP on the LAN without an
identity proxy is no longer supported. If the proxy runs on another host, bind
the exact management IP (not `0.0.0.0`), name that proxy with
`--trusted-proxy`, and firewall port `8766` to it.

Every request and event socket must carry a `Host` header on an explicit
allowlist, which blocks DNS-rebinding pages. The default allowlist is
`127.0.0.1`, `localhost`, and `[::1]` on `--port`, plus the exact `--host`
address when it is a specific IP (a wildcard such as `0.0.0.0` adds nothing).
Behind the reverse proxy, add its public origin with
`--allowed-host https://console.example` (repeatable; a bare `NAME[:PORT]`
means direct HTTP, defaulting to `--port`). Browser `Origin` headers on
mutations, diagnostics, and event sockets must match the same allowlist by
scheme, host, and port. Every HTTP response carries `X-Frame-Options: DENY`
and `Content-Security-Policy: frame-ancestors 'none'`, so no other page can
frame the console and click its controls. Mutation JSON is capped at 64 KiB. These are
request-boundary protections, not user authentication: origin-less
engineering clients remain supported. Any
network-exposed console therefore requires the authenticated reverse proxy
(enforced at startup, as above) and the source firewall described above.

### Audit attribution

Names typed into the browser are self-asserted and are never used as the audit
actor. By default every action and configuration change is attributed to
`actor: "local-console"`, and the typed operator name is kept alongside as
`details.claimed_operator`. To record an authenticated identity, run the
console behind the authenticated reverse proxy and name both the header that
proxy sets and the proxy's TCP source address:

```bash
mocap-g1-console --identity-header X-Authenticated-User --trusted-proxy 127.0.0.1
# or: MOCAP_G1_IDENTITY_HEADER=X-Authenticated-User MOCAP_G1_TRUSTED_PROXIES=127.0.0.1
```

`--trusted-proxy` is repeatable (address or CIDR) and both options must be set
together. They are mandatory for any non-loopback `--host` or
`--allowed-host`, such as the reverse-proxy origin
`--allowed-host https://console.example`. The header is honoured only when the request's TCP peer is a trusted
proxy; uvicorn's `X-Forwarded-For` handling is disabled so a client cannot
spoof that peer. The proxy must strip any client-supplied copy of the header.
Because the proxy normally connects over loopback, any local account that can
reach `127.0.0.1:8766` can also assert that header; restrict local access
accordingly. Requests from other peers fall back to `local-console`.

Windows plus WSL is **not supported** for robot control. WSL networking and USB /
realtime behavior introduce avoidable ambiguity at the safety boundary.

| Flow | Destination | Default policy |
| --- | --- | --- |
| Xsens capture | Ubuntu capture NIC, UDP `5555` | Allow only the paired source |
| Local console | `127.0.0.1`, TCP `8766` | Localhost only |
| Remote console | Ubuntu management NIC, TCP `443` | Authenticated HTTPS; paired source only |
| Unitree DDS | Dedicated Ubuntu robot NIC | No forwarding or bridge |

## Install on Ubuntu

The templates in `deploy/operator-console/` are:

- `mocap-g1-console.service` starts only the unprivileged status/session web console on
  `127.0.0.1:8766`. It does not start mocap ingest, DDS, a controller or robot motion, and it
  never passes `--setup`: the Setup workspace is started per user by `tools/setup/open_console.sh`
  on `127.0.0.1:8784` (see [Setup mode](#setup-mode)).
- `mocap-g1-console.desktop` opens that local service in the default browser.

The unit is sandboxed: loopback-only IP policy (`IPAddressDeny=any`, `IPAddressAllow=localhost`),
`SystemCallFilter=@system-service` without privileged or resource-control calls,
`PrivateUsers=yes`, `RemoveIPC=yes`, `UMask=0077`, no capabilities, and a read-only file system
apart from its state directory. Binding a management interface requires a reviewed drop-in that
adds only the paired source to `IPAddressAllow=`, and the console refuses any non-loopback bind or
`--allowed-host` unless it sits behind the identity-aware proxy described above. Review local
paths, the dedicated service account and firewall policy before installing the templates, and do
not weaken the service hardening to give the console robot access.

These commands assume an approved release is available at `/path/to/release`.
Use a versioned directory so an update can be rolled back without rewriting the
previous environment. Set `VERSION` to the release's package version: the
`version` field of its `pyproject.toml`. The running console reports the same
value as `service_version` in the diagnostics export and on `/api/docs`.

```bash
VERSION=<version>   # for example the release's pyproject.toml version
sudo useradd --system --home-dir /var/lib/mocap-g1-console --no-create-home \
  --shell /usr/sbin/nologin mocap-g1
sudo install -d -o root -g root -m 0755 /opt/mocap-g1/releases
sudo python3 -m venv "/opt/mocap-g1/releases/$VERSION"
sudo "/opt/mocap-g1/releases/$VERSION/bin/python" -m pip install \
  '/path/to/release[operator]'
sudo ln -s "/opt/mocap-g1/releases/$VERSION" /opt/mocap-g1/current

sudo install -o root -g root -m 0644 \
  deploy/operator-console/mocap-g1-console.service \
  /etc/systemd/system/mocap-g1-console.service
sudo install -o root -g root -m 0644 \
  deploy/operator-console/mocap-g1-console.desktop \
  /usr/local/share/applications/mocap-g1-console.desktop
sudo install -d -o root -g root -m 0755 /usr/local/share/doc/mocap-g1
sudo install -o root -g root -m 0644 docs/operator-console.md \
  /usr/local/share/doc/mocap-g1/operator-console.md
# Capability key credential: empty while hardware remains locked.
sudo install -d -o root -g root -m 0700 /etc/credstore
sudo install -o root -g root -m 0600 /dev/null /etc/credstore/mocap-g1-capability-key
sudo systemctl daemon-reload
sudo systemctl enable --now mocap-g1-console.service
```

The unit loads the capability verification key with `LoadCredential=` from
`/etc/credstore/mocap-g1-capability-key`; the console reads it from
`$CREDENTIALS_DIRECTORY` and falls back to the legacy
`MOCAP_G1_CAPABILITY_KEY` environment variable only when no credential is
present. systemd refuses to start the unit if the credential file is missing,
which is why the install steps above create an empty one. Write the issued key
into that file (no trailing data other than an optional newline) and restart
the unit to load it.

If the account already exists, `useradd` will report that fact; verify its home,
shell, and group rather than recreating it. Production releases should be
installed from a signed, hash-verified wheel or offline bundle. Do not install
the optional `robot` dependency into the console environment.

Verify the status-only service:

```bash
systemctl status mocap-g1-console.service
curl --fail http://127.0.0.1:8766/
journalctl -u mocap-g1-console.service --since today
```

The desktop entry opens the already-running service. It never starts the service
or a robot process. For a temporary engineering demo without systemd, stop the
system service and run `mocap-g1-console --demo --open`; demo mode uses synthetic
state and cannot connect to hardware.

For a recorded NPZ simulation session, stop the system service and run:

```bash
mocap-g1-console \
  --simulation /path/to/conditioned-g1-motion.npz \
  --qualification-root /path/to/artifacts/qualify-runs \
  --open
```

Recorded simulation accepts the repository's strict G1 preview-cache format:
one root pose plus the canonical 29 joints, a finite positive frame rate, and
the required `unitree_g1` metadata. The console follows the recording's clock,
feeds the 17 waist and arm joints through the existing live safety envelope,
and shows the reference beside the bounded command. The 12 leg joints remain a
kinematic preview. Starting simulated tracking restarts the source timeline and
loops it until the operator stops or ends the session. This mode is not a
floating-base dynamics result, a measured robot trace, or hardware qualification.

The recorded-simulation runtime does not open DDS or a robot transport. Its
capability lock remains closed, and its stop path only removes simulated command
authority. The independent physical stop procedure is still required anywhere
a powered robot is present. The source NPZ is opened read-only and is never
modified by the console.

If `--qualification-root` is set, Engineering lists sanitized summaries from
qualification `ledger.json` files beneath that fixed directory. The endpoint
does not expose absolute paths or artifact contents, ignores malformed and
escaping files, and does not provide write, delete, or arbitrary path-selection
operations.

For a one-time review of an approved recorded simulation, stop the system
service and run:

```bash
mocap-g1-console --playback /path/to/approved-recorded-review.mp4 --open
```

`--demo`, `--playback`, and `--simulation` are mutually exclusive. Playback uses
the passive runtime:
it does not start mocap ingest, simulation, DDS, a controller, or robot transport;
it never grants command authority or changes the operator state when the video is
played, paused, looped, or scrubbed. The center panel labels the media as a
recorded simulation review with hardware blocked and authority at zero.
The API status identifies this as `presentation_mode: recorded_playback`. In this mode
every operator action and configuration write is rejected before it reaches the
session controller; the GUI disables session, stop, and configuration controls.
Diagnostics export remains available as a read-only operation.

Review videos are not bundled into the application wheel. Provision each MP4
separately from an approved, hash-verified data package, give the console
account read-only access, and pass its explicit path at launch. The browser can
request only that configured file; it cannot select or browse server paths, and
the API manifest exposes only the filename. Treat the video as body-motion data:
do not put it in source control, and do not bind the temporary playback service
to a network interface unless the commissioning access controls permit it.

The command defaults are:

```text
host:      127.0.0.1
port:      8766
data root: $XDG_STATE_HOME/mocap-g1/operator-console, or
           ~/.local/state/mocap-g1/operator-console
qualification root: disabled unless --qualification-root is supplied
```

Use `--host`, `--port`, `--allowed-host`, and `--data-root` only as part of the commissioned
deployment configuration. A browser is opened only when `--open` is explicitly
provided.

## Setup mode

`mocap-g1-console --setup` adds a **Setup** workspace for the
[engineer setup guide](engineer-setup-guide.md): it saves the session values
(`session.env`), runs the 30-second stream check, starts and stops the live
MuJoCo viewer, checks BVH files, retargets and renders recorded takes, and saves
the session report. Engineers normally start it from the **Mocap Setup**
applications-menu entry, which runs `tools/setup/open_console.sh` (port `8784`,
log `~/.local/state/mocap-g1/setup-console.log`, stop with
`bash tools/setup/open_console.sh --stop`).

Boundaries:

- **Simulation only.** Every Setup job is a fixed command from this checkout
  (`live_operator.sh acceptance|gmr`, `retarget_robot_recorded.py`,
  `render_robot_motion.py`, `setup_doctor.py`). None opens DDS, a robot
  transport or a controller, and the hardware capability lock is unchanged.
  The Setup workspace does not show the `DISENGAGE` dock because it has no
  robot authority to withdraw.
- **This PC only.** `--setup` is refused unless the console is bound to
  loopback with no non-loopback `--allowed-host` and no identity proxy, and it
  cannot be combined with `--playback` or `--simulation`. Mutations need a
  same-origin JSON request, as for the rest of the console. Any local process or
  account that can reach `127.0.0.1:8784` can start these jobs; use it on a
  single-user engineering PC.
- **Needs the desktop.** The live viewer and renderer need the local X11
  display, so start the console from the desktop (menu entry), not over SSH.
- **Arguments, not shells.** Jobs run as argument lists in their own process
  group; `session.env` is parsed, never executed. Files are served only from the
  session folder, by plain file name, for `.mp4`, `.json`, `.txt` and `.log`.
- One job runs at a time. Stopping a job sends Ctrl+C to it, then stops the
  `gmr-xsens-live` container if needed. Closing the console stops a running job.

## Session procedure

1. Confirm the clear area, harness, dedicated spotter, Unitree physical stop /
   damping path, and posted abort procedure. Enter the operator and spotter names.
2. Start and calibrate Xsens Humanoid manually. Confirm its configured destination
   and that the console reports fresh, correctly paired mocap frames.
3. Power and prepare the Unitree manually. Keep it in the approved safe mode and
   confirm the physical stop drill before any qualified control service is armed.
4. Run the console system check and 30-second stream acceptance. Resolve stale,
   malformed, wrong-source, low-rate, or excessive-loss results; never bypass a
   failed gate.
5. Preview the motion in simulation. Confirm scale, joint mapping, direction, and
   limits before considering hardware.
6. For released capabilities only, connect the independently supervised robot
   service, complete preflight, then explicitly arm and start. Full-body hardware
   remains locked in the current release.
7. Use `DISENGAGE` for an orderly software stop. Independently place the robot in
   its safe vendor mode, stop Xsens streaming, then open **Engineering** and choose
   **Export diagnostics**. Review warnings before **End session** clears the crew
   checks and resets raw-mocap consent to off.

Opening or reloading the console is read-only: it never writes the
configuration. The supervisor itself resets raw-mocap consent to off at service
start and on **End session**. In `setup_required`, **End session** writes
nothing: the console stays in `setup_required` and a rejected `config.json` is
left on disk for review. Only saving a valid configuration in **Engineering**
replaces it.

Starting the status console at boot does not authorize motion. A browser refresh,
service restart, network reconnection, or recovered mocap stream must never resume
robot control automatically.

## Recovery and diagnostics

- **Unexpected motion or uncertain authority:** use the Unitree physical
  stop/damping path immediately. Do not wait for the web UI. Keep people clear,
  then collect logs only after the robot is safe.
- **Console unavailable:** check `systemctl status` and `journalctl` for the
  console unit, then restart only that unit. A console restart must not start or
  reconnect control. Open browser tabs detect the new service instance, reload
  their state, and clear the crew preflight checks; they never resume a cursor
  from the previous process.
- **Setup required after restart:** a `config.json` that is unreadable or fails
  validation (for example port `0`, an unknown key, or `"mocap": null`) is
  rejected in the audit log and the console starts in `setup_required` with
  defaults instead of crash-looping. Review and save the configuration in
  **Engineering**.
- **`watchdog_stopped` fault:** the time-based safety watchdog ended
  unexpectedly. The fault is not recoverable; end the session and restart the
  console service. A watchdog tick that raises is logged; during tracking it
  also latches a recoverable `watchdog_tick_failed` fault. The status payload's
  `watchdog.running` / `watchdog.overdue` show the loop's state.
- **Hung runtime adapter (`runtime_adapter_timeout`,
  `runtime_adapter_unavailable`):** every runtime-adapter call runs on one
  supervised thread with a deadline (watchdog `tick` 250 ms, `set_tracking`
  0.5 s, `safe_stop` 1 s, `check` / `start_simulation` / `connect_robot` /
  `recover` / `shutdown` 5 s). A call that overruns latches
  `runtime_adapter_timeout` while a robot session may be live (from
  `sim_validated` onward), and the adapter is reported as blocked in
  `runtime_adapter.blocked_call`. Until that call returns, every later adapter
  call, including the `safe_stop` behind **DISENGAGE**, fails immediately with
  `runtime_adapter_unavailable` instead of waiting. **DISENGAGE** still returns
  promptly with command authority off; its audit record shows
  `verified: false` and `safe_stop.delivered: false`, and rearm stays disabled
  until a later **DISENGAGE** gets a verified stop from the recovered adapter.
  A blocked adapter is not proof that the robot stopped: use the physical
  stop/damping path. Outside tracking, a failing or overdue tick sets
  `health.overall` to `fault`, is audited once (`runtime` / `watchdog_tick`),
  and disables **Start tracking** until a tick succeeds again.
- **`watchdog_overdue` fault:** a watchdog tick has not returned for 1 s
  (something outside the adapter deadlines, such as the controller lock or
  audit disk, is stuck). While tracking this latches a recoverable fault;
  otherwise it is only audited.
- **`health_stale` fault:** while tracking, no runtime or supervisor health
  update arrived for more than 250 ms (the 100 ms source-age budget plus slack
  for the 100 ms watchdog tick). The stream acceptance gates (online, at least
  50 Hz for a live source, at most 1 % loss, source age at most 100 ms) are also re-checked on
  every health update and watchdog tick during tracking; a degraded stream
  latches `tracking_health_lost`. Both are recoverable and require explicit
  rearm once fresh, passing health is reported again.
- **Mocap stale or rejected:** leave hardware disabled, correct the Xsens
  destination/firewall, recalibrate or restart Xsens manually, and repeat the
  complete acceptance window.
- **Robot or policy fault latched:** preserve the safe physical state, export the
  redacted diagnostics JSON from **Engineering**, correct the underlying cause,
  verify fresh measured state, and require explicit rearm. Never auto-clear a
  latched fault.
- **Power or workstation restart:** the status console may return automatically;
  Xsens, robot preparation, preflight, capability checks, and arming all start
  again from the beginning.

### Audit log integrity

`audit.jsonl` is append-only. Control actions, rejections and configuration
changes are fsynced before the action completes; safety records written on the
stop path (fault latches, disengage, `safe_stop` results, health updates) are
flushed but not fsynced so a slow disk never delays withdrawing authority.

Each record carries `prev_sha256`, the SHA-256 of the previous line exactly as
written (all zeros for the first line of a new file), so an edited, deleted,
or reordered line breaks the chain. The chain is verified when the console
starts and again for every diagnostics export. A break never stops the
console; it is reported:

- `audit_integrity` in `/api/v1/status` (`chain_verified`, `chain_breaks`,
  `chained_records`, `unchained_records`, `unreadable_lines`);
- an `audit` / `integrity_check` record with outcome `rejected` (listing the
  first breaks by line number), appended at startup and visible in the audit
  view; it is repeated on every start until the file is replaced;
- the full report under `audit_integrity` in the diagnostics export, which
  also flags a file whose last line no longer matches the last record this
  process wrote (`tail_matches_last_write: false`, for example truncation).

Deleting only the final line(s) of a file between runs cannot be detected by
the chain alone; keep the file on storage the console user cannot rewrite
outside the service, and archive exported diagnostics. Logs written before
chaining load unchanged; the first new record is preceded by an `audit` /
`chain_start` record giving the number of unchained records, and the chain
starts from the last legacy line. To start a fresh chain after investigating a
break, stop the service and archive `audit.jsonl` (and `audit.corrupt.log`).

Health updates are audited when a material field changes (stream online,
acceptance, simulation, robot, controller, preflight, authority, `overall`,
messages) and otherwise at most every 5 s, so a high-rate supervisor cannot
evict control events from the in-memory event view.

A request that conflicts with the session state (for example a configuration
change after checks begin, `configuration_locked`) returns HTTP `409`;
malformed input returns `422`. The diagnostics export's `system_probe` no
longer contains the interpreter or checkout path.

Persistent console data is under `/var/lib/mocap-g1-console` for the supplied
service. Treat exported motion/session records as personal data: limit access,
retain safety telemetry according to your data-handling policy, and collect
raw body motion only with explicit per-session consent.

## Frontend development

The browser bundle is built from `operator_ui/` (Node `22.12.0`, see
`.nvmrc`) into `mocap_g1/operator/static/`, which is committed; CI fails if
`npm run build` changes it.

```bash
npm --prefix operator_ui ci
npm --prefix operator_ui run typecheck
npm --prefix operator_ui test
npm --prefix operator_ui run build
```

`npm --prefix operator_ui run dev` serves the UI on `http://127.0.0.1:5173`
and proxies `/api` (including the event socket) to the backend on
`127.0.0.1:8766` (override with `MOCAP_G1_CONSOLE_PORT` or
`MOCAP_G1_CONSOLE_URL`). The proxy rewrites `Host`, but the browser's `Origin`
remains the dev server's, so start the backend with that origin explicitly
allowed; the default allowlist is not widened:

```bash
mocap-g1-console --demo --allowed-host 127.0.0.1:5173
```

## Updates and rollback

1. Verify the release signature and digest on an offline/staging machine. Review
   release notes for configuration or schema migrations.
2. Back up `/var/lib/mocap-g1-console` and record the current `/opt/mocap-g1/current`
   target. Do not alter a running session.
3. Install the new `[operator]` wheel into a new
   `/opt/mocap-g1/releases/<version>` virtual environment. Do not add robot
   dependencies or change the service hardening.
4. Stop the console, point `/opt/mocap-g1/current` at the new version, reload
   systemd, and start the console. Confirm the version, local health page, locked
   full-body capability, both network-source checks, and diagnostics export.
5. If validation fails, stop the console, restore the previous symlink and data
   backup if a migration changed it, then restart and document the rollback.

Updates to the status console do not qualify or unlock robot motion. Controller,
policy, firmware, network, or safety-threshold changes require their own staged
simulation and hardware acceptance process.
