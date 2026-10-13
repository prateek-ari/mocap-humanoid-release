# Live filtering and latency — 2026-08-16

Before this change the live path applied only the safety envelope's rate/acceleration clamp, so
suit jitter reached the commands bounded but unfiltered, and no end-to-end suit-to-command number
existed. `mocap_g1/live_filter.py` adds a strictly causal smoothing filter, a fixed-rate command
upsampler, and latency instrumentation. `mocap_g1/conditioning.py` is unchanged and stays offline:
its FIR is zero-phase and reads future samples, so it can never run in a controller.

## Where it sits

```text
suit packet -> parse -> retarget -> live filter -> command upsampler -> safety envelope -> command
```

The two new stages shape references only. `ArmSafetyEnvelope` still clamps position, velocity,
acceleration and staleness on every command it is handed and remains the last authority. The
upsampler forwards the newest reference's **source timestamp unchanged**, so the envelope's
stale/hard-timeout decisions are exactly what they were before; the upsampler reports staleness but
never decides on it. A rearm (`R`) resets the filter and the upsampler along with the envelope, so
the first frame after a rearm passes through unsmoothed instead of ramping out of pre-rearm state.

## Filter design and defaults

The default is **One-Euro** (Casiez et al.): a first-order low-pass whose cutoff rises with the
observed joint speed. Slow poses get smoothed hard, where jitter is what the operator sees; fast
motion gets smoothed lightly, where lag is what the operator feels. It is vectorised over all 29 G1
joints, each joint independent, and its output at time `t` depends only on samples at or before `t`.

| Parameter | Default | Meaning |
|---|---|---|
| `--filter-min-cutoff` | 2.0 Hz | cutoff at zero joint speed |
| `--filter-beta` | 0.6 Hz per rad/s | speed-adaptive cutoff gain |
| `--filter-derivative-cutoff` | 1.0 Hz | cutoff of the internal speed estimate |
| reference-gap reinit | `--hard-timeout` (0.50 s) | history older than this is dropped; the next sample passes through |

`--filter-type critically_damped` selects the alternative: an exactly integrated second-order
critically damped low-pass (`--filter-cutoff`, default 4.0 Hz, -3 dB). It never overshoots and its
lag is amplitude-independent — pick it when a constant, predictable lag matters more than the
One-Euro tradeoff. The two defaults are deliberately matched at the reference sine.

## Measured lag at the defaults

Reproduce with `python3 -m mocap_g1.live_filter` (all figures at a 60 Hz source rate; the utilities
are `measure_step_lag`, `measure_phase_lag`, `measure_jitter_attenuation`).

| Measurement | `one_euro` (default) | `critically_damped` |
|---|---|---|
| Step 0.20 rad, time to 50 % | **26.0 ms** | 26.8 ms |
| Step 0.20 rad, time to 90 % | 109.7 ms | 83.0 ms |
| Step overshoot | 0.0 % | 0.0 % |
| Phase lag, 1 Hz / 0.30 rad sine | **42.4 ms** (15.3°, gain 0.990) | 42.5 ms (gain 0.974) |
| Jitter rejection, 0.01 rad std | 7.4 dB (0.0097 → 0.0041 rad rms) | 7.3 dB |

One-Euro is nonlinear, so its lag is reported against a stated excitation. That amplitude
dependence is the point of choosing it:

| 1 Hz sine amplitude | 0.02 rad | 0.15 rad | 0.30 rad | 1.00 rad |
|---|---|---|---|---|
| `one_euro` phase lag | 68.1 ms | 51.8 ms | 42.4 ms | 26.2 ms |
| `critically_damped` phase lag | 42.5 ms | 42.5 ms | 42.5 ms | 42.5 ms |

Jitter rejection moves the opposite way (9.2 dB at 0.01 rad motion, 5.9 dB at 1.0 rad): the filter
spends its smoothing where the operator is still and gives it back where the operator is moving.

`DOCUMENTED_STEP_LAG_50_BOUND_MS`, `DOCUMENTED_STEP_LAG_90_BOUND_MS`,
`DOCUMENTED_SINE_LAG_BOUND_MS` and `DOCUMENTED_JITTER_ATTENUATION_MIN_DB` in the module are
asserted by `tests/test_live_filter.py`, so a parameter change cannot silently move the budget.

## Upsampler semantics

`CommandUpsampler` resamples the ~60 Hz reference stream onto a monotonic-clock command schedule
(`--command-rate`, default 250 Hz, configurable to 500).

- **Interpolate** (default): each command ramps from its current value toward the newest reference
  over one observed source interval, bounded by `max_segment_s` (two source periods). The output is
  continuous and is never extrapolated past a received reference, at the cost of running up to one
  source period behind — **16.7 ms at 60 Hz**, the number that appears in the latency budget.
- **Hold** (`--command-interpolation hold`): zero-order hold. No added lag, stepped output.
- **Source stall**: the ramp saturates at the newest reference and the command is held there
  (`saturated`), and commands whose source age exceeds `--stale-timeout` are flagged (`stalled`).
  This composes with, and never substitutes for, the envelope's own stale/hard-timeout machinery,
  which sees the same frozen source timestamp and decelerates, then latches.
- **Bounded catch-up**: after a long scheduling gap the upsampler skips ahead and counts the
  skipped ticks instead of emitting a burst of backdated commands.

In MuJoCo the upsampler is an honest **measured pass-through**: every scheduled command is
generated and timed, but the viewer and IK cannot run at 250 Hz, so only the most recent command
becomes that tick's envelope target. The reported `emit_late` therefore reflects the simulator loop
(commands arrive in bursts of ~4 per 60 Hz iteration, so p95 ≈ 16 ms), not the schedule itself,
which is exact — a 4 s sim run emitted 1000 commands at 250.0 Hz achieved with zero dropped ticks.
On hardware the same object would be pumped by the SDK publish loop.

## Reading the latency report

`tools/xsens/live_g1_sim.py` prints the filter/upsampler characterisation at startup and a latency
summary on exit; `tools/profile_live_ingest.py` reports the same stages against a synthetic MXTP02
stream. The budget separates two quantities that cannot be traded against each other:

- **compute latency** — measured wall-clock time per frame through parse, retarget, filter and
  emit, reported mean/p50/p95/max;
- **algorithmic lag** — the filter's phase lag (its 50 % step lag) plus the upsampler's
  interpolation lag. These are properties of the configuration, measured offline at startup, not
  by timing the code. At the defaults: 26.0 + 16.7 = **42.7 ms**.

The compute figures captured on this checkout used a stub retargeter (the GMR wheel is not
installed here), so only the new stages' costs are meaningful: the filter costs ~0.13 ms and the
upsampler ~0.09 ms per frame (p50, 29 joints, 250 Hz). Run `tools/profile_live_ingest.py` inside
the GMR container for a real IK number; `--no-filter` there gives the pre-filter baseline.

## Flags

| Flag | Effect |
|---|---|
| `--no-filter` | restores the pre-filter path exactly: raw retargeted references go straight to the envelope, one target per frame |
| `--filter-type`, `--filter-min-cutoff`, `--filter-beta`, `--filter-derivative-cutoff`, `--filter-cutoff` | filter selection and tuning |
| `--command-rate` | SDK-facing command rate (default 250 Hz; must be at least `--control-rate`) |
| `--command-interpolation` | `interpolate` (default) or `hold` |

## What is still unmeasured

Everything above stops at the command hand-off inside this process: the suit-to-host network
transport and the Unitree DDS publish/actuation path cannot be measured until a real DDS bridge and
hardware exist, so the true glass-to-joint latency remains unknown.
