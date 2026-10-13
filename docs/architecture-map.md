# Interactive architecture map

> **Caveat (2026-10-12): the map predates the operator console.** `architecture.html` was
> generated before `operator_ui/` (React/Vite frontend), `mocap_g1/operator/` (FastAPI service,
> simulation adapter, audit log, qualification-ledger review) and `tools/operator_console.py`
> existed, so none of them appear in the 25 buildings, and it does not show the CI/packaging
> changes since. Treat it as the architecture of the live-ingest, offline-motion, qualification
> and guarded-control paths only. For the operator console read
> [operator-console.md](operator-console.md). The HTML has not been regenerated.

[`architecture.html`](architecture.html) is a self-contained, interactive view of the
`mocap_humanoid` codebase. It groups the repository into an isometric city so the live-ingest,
offline-motion, qualification, and guarded-control paths can be inspected without installing a
documentation toolchain.

## What the map shows

The map groups 25 subsystem buildings into six neighborhoods:

- **Contracts & runtime** — the canonical G1 joint schema and pinned external workspace boundary.
- **Live Xsens ingest** — MXTP decoding, stream acceptance, newest-frame selection, causal
  filtering, simulator safety, the operator modes, and the GMR/MuJoCo viewer.
- **Motion production** — retarget adapters, repeatable BVH preview, grounding, offline
  conditioning, NPZ-to-PKL conversion, and dataset publication.
- **Qualification** — immutable auditing, collision tooling, controller and balance screens,
  deterministic fault replay, the qualification ledger, PHUMA-style metrics, and independent
  checks.
- **Safe control seam** — the named-joint safety envelope and the measured-state G1 arm bridge.
- **Verification** — the hardware-free executable test mesh.

Lines represent traced call, data, or supporting-contract relationships. Dashed supporting edges
show shared schemas or independent cross-checks; the selected payload flow is highlighted and
animated. External boxes identify systems that are deliberately outside this repository, including
Windows Xsens, GMR/MuJoCo, and Unitree DDS.

## Open it locally

The file has no external fonts, scripts, stylesheets, CDN requests, or package dependencies. The
supported viewing path is a loopback-only server from the repository root:

```bash
python3 -m http.server 8765 --bind 127.0.0.1
```

Then visit <http://127.0.0.1:8765/docs/architecture.html>. Stop the server with `Ctrl+C`.

The server exposes files under the repository root over HTTP while it is running. Keep it bound to
`127.0.0.1`; do not replace that address with `0.0.0.0` or expose the map publicly. The page's
`noindex` metadata discourages search indexing but is not authentication or access control.

Opening `architecture.html` directly will also work in many modern browsers, but file-origin or
local-storage policies can prevent initialization or theme persistence. Opening the map itself runs
no mocap, simulator, network, or robot code.

## Interactions

- Select one of the four **Trace a payload** buttons to animate an evidence-backed route and show
  its concise sequence.
- Select a building to open its purpose, design rationale, measured Python files/LOC,
  representative paths, and implementation vocabulary. Buildings are keyboard-selectable with
  `Tab`, then `Enter` or `Space`; `Escape` closes the inspector.
- Search by subsystem, role, file path, neighborhood, or vocabulary. Non-matches are dimmed rather
  than removed, preserving the overall topology.
- Drag empty map space to pan. Use the mouse wheel or `−` / `+` controls to zoom, and **Reset** to
  restore the initial view.
- Use `◐` to switch between dark and light themes. The preference is saved in browser local
  storage. On a narrow screen, `☰` opens the map controls.

## The four traced flows

1. **Live Xsens preview** follows an MXTP02 body packet through newest-frame ingest, external GMR
   retargeting, causal filtering and upsampling, bounded simulator waist/arm output, and the MuJoCo
   viewer. The animation simplifies the GMR crossing inside `live_g1_sim.py`; it does not omit that
   runtime stage from the real sequence.
2. **Publish a dataset** follows a validated commissioned BVH job through pinned GMR retargeting,
   canonical PKL/CSV output, scoring, and the hashed publication manifest.
3. **Qualify a recording** follows raw audit and conditioning into independent simulator gates,
   NPZ-to-PKL conversion, gate-v2 scoring, and the consolidated qualification ledger. The
   aggregate qualifier invokes collision *audit*, not the optional collision projection command.
4. **Guarded arm replay** follows a recorded 29-DOF reference into measured-state arming, the
   17-joint safety envelope, and CRC-stamped fake or DDS transport.

## Measurement scope

The displayed inventory is a snapshot of **64 Python files and 21,034 physical lines of Python**
in the repository. Every Python source is assigned to one of the 25 subsystem buildings, which is
why the header reports `64 / 64 sources mapped`. Counts and LOC shown in inspectors are Python-only;
representative Bash, Docker, configuration, URDF, workflow, and documentation paths can be listed
for architectural context without increasing those measurements.

These measurements do not update automatically. The example deliberately has no coverage file,
sync script, or CI drift check, so a source change requires the manual update workflow below.

Building height is calculated at runtime from the `loc` value embedded in each node record, using a
capped square-root scale so large modules remain visible without flattening smaller ones. Width and
depth are influenced by the embedded Python file count. The subsystem prose, grouping, coordinates,
edge routing, and overall layout are an authored first draft; they are an explanatory model rather
than automatically generated dependency-analysis output.

## Safety boundaries

The visualization is intentionally strict about boundaries that matter for physical safety:

- The live GMR path ends in **MuJoCo**. There is no map edge from the live viewer to
  `G1ArmBridge`, because that runtime integration does not exist.
- Acceptance, ROS 2, and GMR are alternate, sequential operator modes. They each bind UDP `5555`;
  they are not concurrent consumers unless a separate destination, port, or explicit relay is
  designed and validated.
- Raw whole-body `qpos` is never a safe motor command. The hardware seam extracts only the named
  17 waist/arm targets and applies measured-state arming, position/rate/acceleration limits,
  staleness handling, authority blending, motor-slot selection, and CRC construction.
- `FakeTransport` is the default arm-bridge backend. Real DDS publication remains explicitly
  unverified on hardware and requires operator acknowledgement; the map is not evidence of
  hardware readiness.
- Fixed-base control, floating-base balance, collision, and fault-replay results are simulator
  screens with deliberately limited claims. A passing screen is not a hardware go/no-go.

## Updating the map

The visualization is maintained directly in the inline data near the bottom of
`architecture.html`:

1. Recount repository Python files and physical LOC, then update the three header metrics.
2. Update each entry in `nodes` (`count`, `loc`, representative files, prose, and coordinates).
   Ensure every Python file belongs to exactly one subsystem before retaining the `64 / 64` style
   coverage claim.
3. Update `edges` only for relationships supported by imports, subprocess calls, wrapper scripts,
   or concrete artifact flow. Do not add a conceptual arrow where no runtime or data connection
   exists.
4. Update `flows` with edge IDs that already exist in `edges`; verify the captions still state the
   real ordering and branching behavior.
5. Open the page at desktop and narrow widths. Exercise search, every flow, node inspectors,
   keyboard selection, pan/zoom/reset, both themes, and reduced-motion mode.
6. Recheck the safety distinctions above against the implementation and current commissioning
   status whenever the live or DDS seams change.

Because all CSS, SVG, data, and JavaScript are inline, an update to the map should not introduce a
build step or network dependency.

That statement applies only to viewing this documentation artifact. The software it describes can
still require the external GMR checkout, MuJoCo/model assets, Docker, Xsens licensing and network
setup, or physical hardware.
