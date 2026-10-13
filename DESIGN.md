---
name: Mocap / G1 Operator Console
description: Bench-instrument control panel for evidence-gated Xsens-to-Unitree G1 sessions.
colors:
  panel: "#e6e4df"
  panel-deep: "#d9d6cf"
  bezel: "#cfccc4"
  display: "#f8f8f5"
  key: "#f3f2ee"
  key-off: "#ecebe6"
  ink: "#23262b"
  silk: "#4a4e55"
  muted: "#5b6067"
  frame: "#9ea2a8"
  rule: "#a9adb3"
  rule-soft: "#c9cfd6"
  grid: "#dce1e6"
  lamp-next-blue: "#2f5d9e"
  lamp-passed-green: "#2e7d4f"
  warning-amber: "#a8650d"
  stop-red: "#c4352b"
typography:
  body:
    fontFamily: '"Barlow", "Noto Sans", "Liberation Sans", ui-sans-serif, system-ui, sans-serif'
    fontSize: "1rem"
    lineHeight: 1.45
  silkscreen:
    fontFamily: '"Barlow Condensed", "Barlow", sans-serif'
    fontSize: "0.8125rem"
    fontWeight: 600
    letterSpacing: "0.14em"
    textTransform: uppercase
  state-word:
    fontFamily: '"Barlow Condensed", "Barlow", sans-serif'
    fontSize: "clamp(2rem, 3.4vw, 3rem)"
    fontWeight: 700
    lineHeight: 0.95
  readout:
    fontFamily: '"JetBrains Mono", ui-monospace, monospace'
    fontSize: "1.625rem"
    fontWeight: 500
    fontVariantNumeric: tabular-nums
rounded:
  group: "3px"
  key: "4px"
  bezel: "6px"
  lamp: "50%"
---

# Design System: Mocap / G1 Operator Console

## Overview

**North star: "The Bench Instrument."** The console is one calibrated lab instrument on a light
panel: a recessed display window shows the G1 on a graticule, fixed-width readouts behave like a
multimeter, the seven session gates are keys with indicator lamps, and Disengage is the one red
STOP key, physically separated from routine keys. Light mode is the primary panel; a dark "night
panel" keeps the same roles.

It must never read as a sci-fi HUD (no glow, neon, or fake depth) or a generic SaaS dashboard
(no card grids or KPI tiles). Keys are flat with a single 3px lower edge; there are no bevels,
gradients, or plastic sheen.

## Color roles

- **Panel / bezel / display:** three surface levels. The panel holds controls, the darker bezel
  frames the display, and the near-white display holds measured content.
- **Ink, silk, muted:** primary text, silkscreen labels, and supporting copy.
- **Blue lamp:** the next permitted action, focus rings, and the dock status-line kicker.
- **Green lamp:** passed gates, healthy chain links, active tracking (pulsing).
- **Amber:** warning readouts, for example clamped joints above zero.
- **Red:** faults, locked hardware authority, and the STOP key. The largest red fill is reserved
  for STOP.
- An unlit lamp means locked or not reported. Every lamp has a written state beside it.

## Typography

- **Barlow** for body and key labels; **Barlow Condensed** uppercase, tracked, for silkscreen
  group labels and the poster-scale state word; **JetBrains Mono** with tabular numerals for every
  measured value, timestamp, and identifier.
- Fonts are self-hosted through `@fontsource` (latin subsets) so the console works on an offline
  lab network.

## Layout

- **Header:** wordmark, Session/Engineering switch, runtime-source annunciator, hardware-authority
  annunciator, clock, panel theme.
- **Session:** display window (annunciator line with lamp + state word + scope sentence + chips;
  graticule with the G1 front view, legend and control-authority readout; source footer; progress)
  beside a side column (Measure readouts, Runtime chain, Hardware authority, Crew preflight).
- **Action dock (always visible):** status line (next action, hold reason, or the focused key's
  gate reason), Evidence keys (5), Authority keys (3), and the Stop group (End session + STOP).
- **Fault:** a red-framed alarm panel above the workspace with reason, source, time, recovery,
  authority report, the physical-stop reminder, and diagnostics export; the display bezel tints red.
- **≤960px:** single column, sticky dock reduced to status line, the one next-permitted key, End
  session and STOP; the full key bank moves into the page below the side column.
- **≤520px:** the dock compresses to about 180px; STOP and End session stay side by side.

## Components

- **Group (`fieldset.group`):** 1px frame with a silkscreen legend sitting on the border.
- **Gate key:** label on top, lamp plus state word (Passed / Available / Active / Locked) below.
  Unavailable keys use `aria-disabled`, stay focusable, and announce their gate reason; focus or
  hover writes the reason into the dock status line.
- **Readout:** label, right-aligned mono value with unit. Unreported values render as a dashed
  empty readout reading "not reported", never as a zero.
- **STOP key:** red fill, octagon icon, condensed uppercase label, 4px lower edge. "STOP SIM" in
  simulated sessions, "DISENGAGE" otherwise, always with "software authority only" context.

## Motion

Key press: 1px drop. Lamps: active tracking pulses. Twin: a ±0.6° posture sway while live
tracking. All motion collapses under `prefers-reduced-motion`.

## Rules

- Never fabricate telemetry; absent data stays visibly unreported.
- Never style a locked capability like an editable choice or an available key.
- Nothing competes with STOP for red fill or size.
