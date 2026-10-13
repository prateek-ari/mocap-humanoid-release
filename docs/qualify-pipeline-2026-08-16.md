# G1 recording qualification pipeline (2026-08-16)

`tools/qualify_g1_recording.py` is the single driver that runs a captured G1
NPZ through every existing hardware-free hardening/scoring tool in one pass
and writes one consolidated JSON ledger. It closes a gap identified in a
2026-08-16 review: `eval/score_motion.py` only ever globbed `*.pkl` (GMR
pickle format) while every hardening tool
(`tools/xsens/audit_g1_recording.py`, `tools/condition_g1_motion.py`,
`tools/control/audit_g1_self_collisions.py`,
`tools/control/qualify_g1_upper_body.py`, `tools/control/screen_g1_balance.py`) reads `.npz` — so the team's own
Xsens captures had never actually been run through the PHUMA metrics / gate
v2 harness. This driver bridges the two halves; it does not change either
side's semantics or thresholds.

## Usage

```bash
python tools/qualify_g1_recording.py path/to/capture.npz \
    [--offsets path/to/offsets.json] [--out-dir DIR] [--model MJCF] \
    [--robot unitree_g1] [--contact-policy adaptive] [--limits-urdf URDF]

# or, via the Makefile:
make qualify RECORDING=path/to/capture.npz [OFFSETS=path/to/offsets.json] [OUT_DIR=dir]
```

Default `--out-dir` is `artifacts/qualify-runs/<recording-stem>-<UTC
timestamp>/`, so repeat runs never collide. Every artifact for the run
(each stage's own JSON report, the intermediate conditioned `.npz`, the
converted `.pkl`, and `ledger.json`) lands in that one directory.

Exit code is `0` only when the ledger's overall verdict is `PASS`; `1`
otherwise (`BLOCKED` or `FAIL`). `2` means the driver itself could not even
start (bad recording path, bad `--model`, missing `--limits-urdf`).

`--limits-urdf` (default: the vendored `mocap_g1/assets/g1_29dof_official.urdf`)
is the **single joint-position-limit source** for every stage that applies
limits: window audit (1), conditioning (2), contact refinement (3, ankle
correction bounds), controller screen (5) and balance screen (6) (both overwrite
the MJCF's IK-narrowed hinge ranges with it before compiling), and gate v2 (8,
`--pos-urdf`). *Behaviour change (2026-10-12, review T1-7):* stages 3, 5 and 6
used the MJCF ranges before, which are narrower on 15/29 joints (e.g.
`left_shoulder_roll` MJCF `[-0.6, 2.25]` vs URDF `[-1.59, 2.25]`), so a clip
legal for stages 2 and 8 could fail stage 5 with a misleading tracking error.
Stage 5 now also clamps PD targets to the URDF limits and reports reference
samples outside them as `joint_position_limit_violation`
(`events.reference_joint_position_violations`), not as tracking error. Each
of those stage reports records the URDF path and sha256 under `joint_limits`.

## The chain

```
capture.npz ──► [1] window_audit ──────────────► 01_window_audit.json
        │
        └──────► [2] conditioning ──► 02_conditioned.npz ──► 02_conditioning.report.json
                                             │
                             [3] contact_refinement (--no-contact-refinement skips;
                                             │      position 3 then stays vacant)
                                             │ 03_contact_refined.npz + 03_contact_refinement.json
                                             │ (on FAIL/BLOCK: stages 4–8 fall back to the
                                             │  conditioned NPZ)
              ┌──────────────────┬───────────┴──────┬────────────────────┐
              ▼                  ▼                  ▼                    ▼
   [4] self_collision_gate [5] controller_screen [6] balance_screen  [7] npz_to_pkl
   04_self_collision_gate  05_controller_screen  06_balance_screen   ──► 07_clip.pkl
   .json                   .json                 .json                     │
                                                                           ▼
                                                                [8] phuma_gate_v2
                                                              08_phuma_gate_v2.json
```

Every stage invokes its CLI via `subprocess`, so the driver inherits each
tool's exact flags, defaults, and JSON schema rather than re-implementing any
of it; the driver owns only the orchestration, report validation, the status
translation table, and stage 7 (the format bridge,
`tools/convert_npz_to_gmr_pkl.py`).

**Dependency graph** (why a failure/BLOCK in one stage does or doesn't
propagate):

- Stages 1 (`window_audit`) and 2 (`conditioning`) both read the *raw* input
  recording directly. `window_audit` is a read-only **diagnostic**: it never
  gates `conditioning` and (since 2026-10-10) it is excluded from the overall
  verdict — see "Status semantics" below.
- Stage 2 (`conditioning`) is given the official URDF's mechanical joint
  position limits: zero-phase FIR overshoot that would carry an in-range
  sample past a limit is clamped back (the PCHIP time-scaling resampler is
  monotone, so it cannot re-introduce one), and conditioning fails closed if
  its output leaves range anywhere the source did not. Samples whose *source*
  value is already out of range are left as-is and reported under
  `limits.position` in `02_conditioning.report.json`, so the stage-7 gate
  still sees them.
- Stage 3 (`contact_refinement`, added at `ledger_version: 2`, 2026-08-22)
  requires the conditioned NPZ stage 2 produces. It repairs measured
  world-contact artifacts (pitched-foot penetration via bounded ankle/root-z
  searches, single-stance skating via clamped root pinning — see
  `mocap_g1/contact_refine.py`). On `PASS`, stages 4–7 consume its refined
  NPZ (`03_contact_refined.npz`); on `FAIL`/`BLOCK` they fall back to the
  conditioned NPZ directly — a rejected repair must not silently hide the
  rest of the chain's verdicts. With `--no-contact-refinement` the stage is
  skipped entirely and position 3 stays vacant in the ledger.
- Stages 4 (`self_collision_gate`), 5 (`controller_screen`), 6
  (`balance_screen`) and 7 (`npz_to_pkl`) all require that shared NPZ. If
  stage 2 is `BLOCKED`, stages 3–7 are marked `BLOCKED` immediately (reason:
  "upstream stage 'conditioning' did not produce an output artifact")
  *without* attempting a subprocess call — there is nothing for them to read.
  Stages 4, 5 and 6 do not gate each other or gate 7/8; they are independent
  screens on the same clip, and each one gates the overall verdict.
- Stage 6 (`balance_screen`, added at `ledger_version: 3`, 2026-10-12, review
  T3-2) runs `mocap_g1/balance_screen.py`'s floating-base stiff-PD standing
  screen: pelvis free joint intact, legs/waist held at Unitree's standing
  stance, arms PD-tracking the reference at 1 ms physics steps, gates on CoM
  support margin, ankle torque headroom, pelvis attitude and height. It is a
  SCREEN, not balance validation (see that module and
  `docs/balance-screen-2026-08-16.md`). Its report must be schema v2 (PASS /
  FAIL vocabulary); a v1 report is BLOCKED. Measured cost: ~0.27 ms per
  physics step, i.e. ~16 s per 60 s of clip that does not fall (a fall aborts
  early) -- far inside the clip-scaled stage timeout (see "Stage timeouts").
- Stage 8 (`phuma_gate_v2`) requires the `.pkl` stage 7 produces; a `BLOCKED`
  stage 7 makes stage 8 `BLOCKED` transitively. Stage 8 is also `BLOCKED` if
  the score report does not show `joint_order.status: "verified"` (the pkl's
  `joint_names` checked against the MJCF hinge order).

## Stage timeouts

*Behaviour change (2026-10-12, decision D1):* the fixed 900 s per-stage
subprocess limit is gone. On a 36 s reference capture the old stage
3 took 870 s, a hair under that limit. Each stage now gets

```
timeout_s = 7200 s (2 h floor) + 60 s x clip duration in seconds
```

(`STAGE_TIMEOUT_FLOOR_S`, `STAGE_TIMEOUT_PER_CLIP_SECOND_S` in
`tools/qualify_g1_recording.py`). The clip duration is frames / fps of the
input recording for stages 1-2. Stages 3-8 use the larger of that and the
conditioned NPZ's duration, because time-scaling can lengthen a clip a lot:
conditioning stretched that capture 7.9x, from 36 s to 288 s. If the
duration cannot be read, the stage gets the floor. On that capture stages 1-2
get 9381 s and stages 3-8 get 24470 s (6.8 h), against the old 870 s for
stage 3. A 5-minute raw clip gets at least 7 h. The limit is deliberately kept
very large for now.

`--stage-timeout-s SECONDS` replaces the scaled value with one fixed limit
for every stage. It must be a finite number > 0; `0`, negative values, `nan`
and `inf` are rejected with exit code 2. `make qualify` does not forward it,
so run the driver directly to use it. A stage that hits its limit is
**BLOCKED**, never PASS. It records exit code 124, and its reasons include
`[driver] timed out after <N>s`. Each stage entry records its effective
`timeout_s`. A stage that never launched a subprocess (for example one
BLOCKED by a missing upstream artifact) records `null`.
`config.stage_timeouts` records the policy and its inputs.

## Ledger schema (`ledger_version: 3`)

Written atomically to `<out-dir>/ledger.json` on every run (this file, unlike
each stage tool's own report, is intentionally allowed to be overwritten by a
re-run into the same `--out-dir`):

```jsonc
{
  "ledger_version": 3,
  "generated_at_utc": "...",
  "driver": {"path": "tools/qualify_g1_recording.py", "project_git_rev": "<git sha or null>",
             "project_git_dirty": true},   // `git status --porcelain` non-empty; null = unknown
  "environment": {"python": "3.11.16", "numpy": "2.2.6", "mujoco": "3.15.0"},  // null if absent
  "input": {
    "recording_path": "...",
    "recording_sha256": "...",
    "recording_metadata": { /* the recording's own embedded metadata, or null */ },
    "offsets": {"key": "<recording_metadata.offsets_key, if any>", "path": "<--offsets, if given>", "sha256": "..."}
  },
  "config": {"robot": "...",
             "contact_policy": "anchored",            // EFFECTIVE stage-8 policy
             "contact_policy_requested": "adaptive",  // --contact-policy
             "contact_policy_reason": "...",
             "contact_refinement_enabled": true,
             "model_path": "...", "model_sha256": "...",
             "limits_urdf": "...", "limits_urdf_sha256": "...", "out_dir": "...",
             "stage_timeouts": {               // additive, 2026-10-12 (see "Stage timeouts")
               "policy": "scaled_by_clip_duration",   // or "override" (--stage-timeout-s)
               "override_s": null, "floor_s": 7200.0, "per_clip_second_s": 60.0,
               "recording_duration_s": 36.353, "conditioned_duration_s": 287.84,
               "input_stage_timeout_s": 9381.174,     // stages 1-2
               "chain_stage_timeout_s": 24470.382,    // stages 3-8 (time-scaled clip)
               "timeout_outcome": "BLOCKED"}},
  "stages": [
    {
      "stage": "window_audit",       // one of the 8 names below, always in this order
      "order": 1,
      "status": "PASS | FAIL | BLOCKED",
      "gates_verdict": false,        // true for every stage except window_audit
      "reasons": ["..."],
      "command": ["python", "tools/xsens/audit_g1_recording.py", "..."],
      "returncode": 0,
      "duration_s": 0.42,
      "timeout_s": 9381.174,         // effective limit; null if no subprocess was launched
      "artifacts": {"report": {"path": "...", "sha256": "..."}}
    },
    // ... conditioning (2), contact_refinement (3), self_collision_gate (4),
    // controller_screen (5), balance_screen (6), npz_to_pkl (7), phuma_gate_v2 (8)
  ],
  "verdict": {
    "overall": "PASS | FAIL | BLOCKED",
    "worst_stage": "<stage name>",   // never a diagnostic stage
    "diagnostic_stages": ["window_audit"],
    "rule": "overall = status of the worst verdict-gating stage (stages with gates_verdict=true; diagnostic_stages are reported only); severity PASS(0) < BLOCKED(1) < FAIL(2); ties broken by earliest stage in pipeline order"
  }
}
```

### Status semantics

- **PASS** — the stage's own tool ran and reported a passing verdict.
- **FAIL** — the stage's own tool ran and reported an explicit failing
  verdict: a physical/safety criterion was violated (e.g. a joint limit
  exceeded, a self-collision found, a gate-v2 criterion not met).
- **BLOCKED** — the stage could not produce a trustworthy verdict at all:
  the subprocess crashed/timed out, its JSON report was missing or
  unreadable, an upstream stage it depends on produced no artifact, or a
  required external asset (the G1 MJCF model) was not available.

**One translation table.** Each stage tool keeps its own verdict word; the
driver maps it onto the ledger vocabulary in exactly one place,
`STAGE_STATUS_TRANSLATION` in `tools/qualify_g1_recording.py` (2026-10-12,
review T3-6). A word missing from a stage's row is never guessed at — the
stage is `BLOCKED` with an "unexpected status" reason — and no tool word ever
maps to `BLOCKED`:

| stage | field read | tool value → ledger status |
|---|---|---|
| `window_audit` | `summary.status` | `PASS`→PASS, `FAIL`→FAIL |
| `conditioning` | exit code + report sections | 0→PASS (no FAIL outcome) |
| `contact_refinement` | `status` | `PASS`→PASS, `REJECT`→FAIL |
| `self_collision_gate` | `summary.status` | `PASS`→PASS, `FAIL`→FAIL |
| `controller_screen` | `controller_screen.status` | `PASS`→PASS, `FAIL`→FAIL |
| `balance_screen` | `balance_screen.status` (report schema v2) | `PASS`→PASS, `FAIL`→FAIL |
| `npz_to_pkl` | `status` | `PASS`→PASS (no FAIL outcome) |
| `phuma_gate_v2` | `motions.<clip>.gate_v2.pass` | `true`→PASS, `false`→FAIL |

Crashes, timeouts, missing/malformed reports, and status/exit-code
disagreements are `BLOCKED` before the table is consulted. The balance screen
used to report `BLOCKED` for a violated physical gate; its report schema v2
says `FAIL` (and a v1 report is refused as `BLOCKED`), so `BLOCKED` now
means "could not check" everywhere. `contact_refinement` keeps its tool-level
`REJECT` (a refused repair, also used by `mocap_g1/collision_gate.py`) and
the table translates it.

**Overall verdict = the worst verdict-gating stage**, with severity
`PASS(0) < BLOCKED(1) < FAIL(2)` — an explicit failed physical check outranks
"we could not check." Ties are broken by earliest pipeline order. This mirrors
the PASS/FAIL/BLOCKED vocabulary the underlying hardening tools already use.

`window_audit` (stage 1) is **diagnostic only** (`gates_verdict: false`,
listed in `verdict.diagnostic_stages`): it grades the raw, unconditioned
input, which is never the trajectory that ships, so its PASS/FAIL/BLOCKED is
always recorded but cannot decide the overall verdict. *Behaviour change
(2026-10-10):* previously a raw-input `window_audit` FAIL made the whole run
FAIL even when every stage graded on the shipped trajectory passed. The
position limits of the trajectory that does ship (conditioned, and
contact-refined when stage 3 passes) are now graded by stage 8's gate v2
`joint_position_limits` criterion instead (see below). Ledger consumers that
recompute the verdict (e.g. `mocap_g1/operator/qualification.py`) must skip
stages with `gates_verdict: false`.

`controller_screen`'s and `balance_screen`'s stage status also fold in the
underlying tool's own `hardware_candidate.status` override: a clip can pass
the screen's own checks yet still report `BLOCKED` at the ledger level
because it only carries placeholder `zero-preview` calibration (see
`mocap_g1/controller_screen.py` and `mocap_g1/balance_screen.py`) — this is not the driver double-guessing
the tool, it is surfacing a verdict the tool already computes one field
deeper. A physical FAIL always stays FAIL.

## npz → GMR-pkl mapping (stage 7, `tools/convert_npz_to_gmr_pkl.py`)

The only real data conversion is a quaternion reorder; everything else is a
dtype/shape normalization. See the module docstring for the full derivation;
summary:

| field | NPZ side (`mocap_g1`/`tools`) | GMR pkl side (`eval/score_motion.py`) | conversion |
|---|---|---|---|
| `root_pos` | `(T,3)` meters | `(T,3)` float64 meters | dtype cast only |
| `root_rot` | `(T,4)` **wxyz** (every NPZ producer in this repo) | `(T,4)` **xyzw** (GMR native) + explicit `root_quat_order: "xyzw"` key | index permutation `[1,2,3,0]` |
| `dof_pos` | `(T,29)`, `mocap_g1.schema.G1_29DOF_JOINTS` order | `(T,29)`, MJCF `qpos[7:]` order | **identity**, now *checked*: see `joint_names` |
| `joint_names` | implied by `G1_29DOF_JOINTS` (a `metadata.joint_names` that disagrees is refused) | `list[str]`, the 29 names in `dof_pos` column order | written by the converter (report schema v2, 2026-10-12, review T3-3); stage 8's `run_fk_pass` refuses a pkl whose names differ from the `--mjcf` hinge order, and the driver BLOCKs stage 8 unless the score shows `joint_order.status: "verified"` |
| `fps` | scalar float (may be non-integer after `mocap_g1.conditioning` decimation) | plain Python `int` | `round()`, not truncation — `score_motion.py`'s loader does `int(d["fps"])`, which truncates. Rounding rescales every velocity metric by `fps_out/fps_in`, so it is **refused** (exit 2 → stage `BLOCKED`) when that relative change exceeds 0.1 % (`FPS_MAX_RELATIVE_ROUNDING_ERROR`): Xsens' 59.9952 Hz → 60 (0.008 %) is accepted with a note; 60/7 Hz → 9 (5 %) is refused |

Legacy pkls without `joint_names` (e.g. GMR's own `bvh_to_robot.py` output)
are still scored by `eval/score_motion.py`, with
`joint_order.status: "unverified_legacy_pkl"` and a WARNING note in the
result, because GMR writes `qpos[7:]` of the same MJCF it scores against and
refusing them would break every existing suite; the qualification driver
never produces one and refuses to accept one.

Convention ambiguity resolved: `metadata.root_quaternion_order` is required to
be `"wxyz"` if present at all (fail-closed otherwise) since that is the only
convention any tool in this repo ever writes; there is no live code path that
produces NPZ-side `xyzw`.

On the pkl side every writer in this repo (this converter, the
`tools/retarget/*` retarget/projection/grounding tools, and
`mocap_g1/dataset.py`) now stamps `root_quat_order: "xyzw"`, and every loader
(`eval/metrics.py`'s `load_gmr_pkl`, `mocap_g1.dataset`, the renderer, the
check scripts) refuses a pkl without that key, or declaring anything other
than `xyzw`, unless an explicit legacy override is passed (`--quat-order` on
the CLIs). `load_gmr_pkl` also refuses non-finite arrays and non-unit root
quaternions. See `mocap_g1/gmr_pkl.py`.

### Stage 8 gate v2 additions (2026-10-10, each fails closed)

- `contact_coverage` precondition: `contact_frac >= 0.05` and every foot's
  `contact_frac_by_foot >= 0.02`. NF/NP/NS are percentages of contact frames,
  so one inferred contact frame used to score 100/100/100.
- `joint_position_limits` criterion: zero frames with any joint outside its
  mechanical URDF `[lower, upper]` (1e-9 rad numerical allowance).
- `root_accel_paired_delta` (only with `--baseline-pkl`) now FAILS when the
  delta cannot be computed; NaN statistics can no longer pass
  `not_degenerate`.

Stage 8 now passes `--pos-urdf <--limits-urdf>` (it used the default URDF
before, so a non-default `--limits-urdf` made stages 2 and 8 grade against
different files), and `eval/score_motion.py` records the MJCF and both limit
URDFs with sha256 under the report's `provenance` key. Its JSON is written
atomically, and `failures` is keyed by pkl stem like `motions`.

## Stage-8 contact policy

The npz→pkl bridge is a faithful format conversion only — it does **not**
apply GMR's `offset_to_ground` grounding pass. `eval/metrics.py`'s gate v2
refuses to grade Non-Floating/Non-Penetration/Non-Skating unless
`--contact-policy anchored`, and `anchored` is only valid for clips whose
contacts sit at world z=0 (see `eval/metrics.py`'s "Contact inference"
section).

*Behaviour change (2026-10-12, review T3-1):* when stage 3 **PASSes** and its
refined NPZ is what ships, the driver scores stage 8 with `anchored`
regardless of `--contact-policy`. A stage-3 PASS requires at least one
contact frame observed inside the absolute world band (sole within 2 cm of
z=0 in the same MJCF frame), no residual penetration beyond tolerance and no
residual single- or double-stance skating, all re-verified by an independent
FK pass over the exact returned array — exactly the grounding `anchored`
assumes. Gate v2 still grades contact coverage and NF/NP/NS itself; `anchored`
only makes them gradable. Before this change no run with default flags could
ever PASS.

Otherwise (refinement disabled, FAILed or BLOCKED) the driver uses
`--contact-policy` (default `adaptive`, the honest choice for an ungrounded
clip), and the expected, correct result is a `phuma_gate_v2` `FAIL` citing the
contact-policy precondition — not a driver bug. Pass `--contact-policy
anchored` there only when the recording's provenance separately establishes
that the reference is grounded. The ledger records `contact_policy`
(effective), `contact_policy_requested`, and `contact_policy_reason`.

## Stage 3: contact refinement (`tools/retarget/refine_g1_contacts.py`)

Added 2026-08-22 (`ledger_version: 2`; contact-refinement report schema v2; schema v3,
2026-10-12, bounds ankle corrections by `--limits-urdf` instead of the MJCF ranges).
A PhySINK-style contact-consistent post-pass over the conditioned floating-base
trajectory:

1. **Contact inference** — world-anchored kinematic detection (sole lowest
   point within `--contact-height-band-m` of z=0 AND low vertical speed)
   with enter/exit hysteresis debouncing. A clip with zero observed contact
   frames is rejected instead of passing the checks vacuously. The geom
   lowest-point and the height/vertical-speed predicate are the shared
   `mocap_g1/contact.py` implementations also used by `eval/metrics.py` and
   the grounding preview pass; only the thresholds differ (named constants
   there, with the reason for each).
2. **Penetration repair** — per-frame bounded deterministic search:
   ankle pitch → ankle roll → root-z lift; every candidate FK-verified so
   BOTH feet clear `--target-clearance-m`.
3. **Skating repair** — single-stance segments only: root shifted by the
   negative of the contact *material point's* frame-to-frame slip (the
   previous frame's lowest sole point carried rigidly with the foot body).
   *Behaviour change (2026-10-10):* skating used to be the frame-to-frame
   motion of the argmin lowest point, which on a flat foot flips between
   corners under sub-millimetre pitch/roll dither and read as m/s skating
   (rejecting honest flat stance); a foot pivoting about a planted toe/heel
   now also correctly reads ~0. Thresholds are unchanged. Each XY step
   obeys the configured correction velocity and acceleration bounds. A
   terminal offset may taper only through an immediately following run where
   neither foot is in contact. Double-stance skating is detected and reported
   but deliberately not repaired; those frames reject the clip.
4. **Verification** — correction dynamics bounds + an independent second FK
   pass over the exact returned array; no onset frame is exempt. Any residual
   single- or double-stance skating, penetration, or budget violation rejects
   the whole derivative (fail-closed, mirroring the collision gate).

The original report-schema-v1 exercise on a 0–10 s stress capture repaired penetration on 4021/4021 frames with max root step 0.48
mm/frame and zero joint deviation, then rejected 36 double-stance skating
frames near 3694–3771. Re-run the clip under schema v2 before using those
historical counts; v2 removed onset exemptions and tightened taper validation.

**Performance (2026-10-12).** On a conditioned reference capture
(36 s raw, time-scaled to 288 s = 17269 frames at 60 fps), stage 3 took 870 s. A profile showed
`mj_forward` at ~70 % of the time (~330 µs per call: collision detection
dominates while the sole spheres penetrate the plane) and the per-geom Python
lowest-point helper at ~20 %. Every frame needed a root-z lift with both feet
under the plane, so each frame ran 128 doomed ankle-search FK evaluations
plus 16 lift evaluations. Three changes, none of which changes a result:

1. FK uses `mj_kinematics`, the first step of `mj_forward` and the only one
   that writes the poses read here. The output is bit-identical.
2. For all-sphere feet, the lowest point is computed vectorised with the
   shared helper's exact float operations and tie order.
3. An ankle search is skipped when a foot geom that the ankle joint cannot
   move (outside its kinematic subtree) is already below the clearance
   target. Every candidate would fail, so the full search returns `None`
   anyway.

The independent final verification pass runs on a fresh `MjData`, and the
input hash check is unchanged. On that capture the refined array and the
whole report are byte-identical to the old code, and so is every
`refine_reference_contacts` call in `tests/test_contact_refine.py`.
`tests/test_contact_refine.py::ContactRefineSpeedupEquivalenceTests` pins all
three equivalences.

## Tests

`tests/test_qualify_driver.py` builds a tiny synthetic NPZ, runs the driver
end-to-end via `subprocess`, and asserts the ledger's structure (all 8
stages present in order — or the 7-stage shape under
`--no-contact-refinement` — each with `status`/`reasons`, SHA256 and
provenance correctness,
overall-verdict consistency with the worst-gating-stage rule, and — with
every stage stubbed — that a `window_audit`-only FAIL still yields an overall
PASS while any gating-stage FAIL still yields FAIL) without asserting a
specific PASS/FAIL/BLOCKED outcome per stage — outcomes legitimately vary
with environment (e.g. whether the external G1 MJCF checkout is available)
and are not this test's concern. Stubbed-stage tests cover the translation
table, balance-screen PASS/FAIL/BLOCKED (including a refused schema-v1
report), the controller-screen override, the stage-3 REJECT fallback, both
contact-policy branches, `--limits-urdf` forwarding to stages 3/5/6/8, and the
stage-8 joint-order requirement. It also unit-tests the npz→pkl converter's
round-trip properties (frame count, `joint_names`, dtype, fps rounding and
refusal, quaternion reorder) directly. Run with:

```bash
python -m pytest tests/test_qualify_driver.py -q
```

The contact-refinement stage itself is covered by `tests/test_contact_refine.py`.
Its generated 29-joint MuJoCo fixture runs the core repair, layout-validation,
and CLI failure tests without the external GMR checkout; separate compatibility
tests still exercise the real G1 MJCF when it is available.
