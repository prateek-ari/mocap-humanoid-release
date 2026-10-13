#!/usr/bin/env python3
"""Run a G1 NPZ recording through the full hardware-free qualification chain
and emit one consolidated JSON ledger.

Chain (fixed order; see the module docstring section "Stage dependencies"
below for exactly which stages gate which):

  1. window_audit        tools/xsens/audit_g1_recording.py        (raw input)
  2. conditioning        tools/condition_g1_motion.py             (raw input)
  3. contact_refinement  tools/retarget/refine_g1_contacts.py     (conditioned)
  4. self_collision_gate tools/control/audit_g1_self_collisions.py (refined)
  5. controller_screen   tools/control/qualify_g1_upper_body.py    (refined)
  6. balance_screen      tools/control/screen_g1_balance.py        (refined)
  7. npz_to_pkl          tools/convert_npz_to_gmr_pkl.py           (refined)
  8. phuma_gate_v2       eval/score_motion.py --gate v2             (pkl)

Every hardening stage is invoked as a subprocess against its CLI, so this
driver inherits each tool's exact semantics (flags, defaults, JSON schema)
rather than re-implementing any of them. This driver only owns the
orchestration, the per-stage report validation, and the translation of each
tool's own status word into the ledger vocabulary (``STAGE_STATUS_TRANSLATION``,
one table for every stage).

Stage 3 (contact_refinement) repairs measured world-contact artifacts
(pitched-foot penetration, single-stance skating) in the conditioned
trajectory via mocap_g1.contact_refine's fail-closed bounded searches; it
writes 03_contact_refined.npz on PASS. Pass --no-contact-refinement to skip
it entirely: stages 4-8 then consume the conditioned NPZ directly and no
stage-3 entry is emitted (position 3 stays vacant).

One joint-limit source: ``--limits-urdf`` (default: the vendored official
G1 URDF) is passed to every stage that applies joint position limits --
window audit (1), conditioning (2), contact refinement (3, ankle bounds),
controller screen (5, fixed-base model ranges), balance screen (6,
floating-base model ranges) and gate v2 (8, ``--pos-urdf``) -- and its path
and sha256 are recorded in the ledger ``config``.

Ledger schema (``ledger_version: 3``, documented in full in
docs/qualify-pipeline-2026-08-16.md)
---------------------------------------------------------------------------
Each stage entry is
``{"stage", "order", "status", "gates_verdict", "reasons", "command",
"returncode", "duration_s", "artifacts"}`` with ``status`` in
``{"PASS", "FAIL", "BLOCKED"}``:

  - PASS    the stage's own tool ran and reported a passing verdict.
  - FAIL    the stage's own tool ran and reported an explicit failing
            verdict (a physical/safety criterion was violated).
  - BLOCKED the stage could not produce a trustworthy verdict at all: the
            subprocess crashed or timed out, its JSON report was missing,
            unreadable or inconsistent, an upstream stage this one depends on
            did not produce the artifact it needs (see "Stage dependencies"),
            or a required external asset (the G1 MJCF model) is unavailable.

ledger_version 3 (2026-10-12) adds the balance_screen stage at order 6
(npz_to_pkl and phuma_gate_v2 move to 7 and 8) and provenance: model and
limits-URDF sha256, the git dirty flag, numpy/mujoco versions, and the
requested/effective stage-8 contact policy.

Overall verdict = the WORST *gating* stage, where severity is PASS(0) <
BLOCKED(1) < FAIL(2) -- an explicit failed physical check outranks "we could
not check". Ties (multiple stages tied for worst) are broken by earliest
pipeline order. window_audit is a DIAGNOSTIC stage (``gates_verdict:
false`` in its stage entry, listed in ``verdict.diagnostic_stages``): it
grades the raw, unconditioned input, which is never what ships, so it is
always run and reported but never decides the overall verdict. Joint
position limits of the trajectory that DOES ship (conditioned, and
contact-refined when stage 3 passes) are graded instead by stage 8's gate v2
``joint_position_limits`` criterion; conditioning itself clamps FIR overshoot
so an in-range input can never be pushed out of range.

Stage dependencies
-------------------
window_audit (1) and conditioning (2) both read the RAW input recording
directly, so a BLOCKED window_audit never prevents conditioning from
running (window_audit is a read-only diagnostic on the untouched input;
it does not gate the rest of the chain).

contact_refinement (3) requires the conditioned NPZ from stage 2. When it
PASSes, stages 4-7 consume its refined NPZ; when it FAILs or BLOCKs,
stages 4-8 fall back to the conditioned NPZ directly (the refinement stage
never gates them -- a rejected repair must not silently hide the rest of
the chain's verdicts). With --no-contact-refinement there is no stage 3.

self_collision_gate (4), controller_screen (5), balance_screen (6) and
npz_to_pkl (7) all require that shared NPZ. Stages 4, 5 and 6 do not gate
each other or gate 7/8 -- they are independent screens on the same clip, and
each one gates the overall verdict.

phuma_gate_v2 (8) requires the .pkl that stage 7 produces; if stage 7 is
BLOCKED, stage 8 is marked BLOCKED transitively. Stage 8 also refuses (BLOCKED)
a score whose pkl joint order was not verified against the MJCF.

Stage-8 contact policy (honesty)
--------------------------------
The npz_to_pkl bridge is a faithful format conversion, nothing more -- it does
NOT apply GMR's ``offset_to_ground`` grounding pass. Gate v2 refuses to grade
NF/NP/NS unless the contact policy is ``anchored`` (its precondition 0), and
``anchored`` is only valid for clips whose contacts sit at world z=0.

  - When stage 3 PASSes and its refined NPZ is what ships, the driver scores
    with ``anchored`` regardless of ``--contact-policy``. That is justified,
    not optimistic: a stage-3 PASS requires at least one kinematically
    observed contact frame inside the ABSOLUTE world band
    (sole within ``contact_height_band_m`` = 2 cm of z=0 in the same MJCF
    frame), no residual penetration beyond tolerance, and no residual
    single- or double-stance skating, re-verified by an independent FK pass
    over the exact returned array -- i.e. the clip is grounded in exactly the
    sense ``anchored`` assumes. Gate v2 still grades contact coverage and
    NF/NP/NS on its own; anchored only makes them gradable.
  - Otherwise (refinement disabled, FAILed or BLOCKED) the driver uses
    ``--contact-policy`` (default ``adaptive``, the honest choice for an
    ungrounded clip). The expected, CORRECT result then is a phuma_gate_v2
    FAIL citing the contact-policy precondition -- not a driver bug.

The requested policy, the effective policy and the reason are recorded in the
ledger ``config`` (``contact_policy`` is the effective one).

Stage timeouts
--------------
Every stage subprocess runs under a wall-clock limit. By default the limit
scales with the clip: ``STAGE_TIMEOUT_FLOOR_S`` (2 h) plus
``STAGE_TIMEOUT_PER_CLIP_SECOND_S`` (60 s) per second of clip, where the clip
duration is frames / fps of the input recording (stages 1-2) or, for stages
3-8, the larger of that and the conditioned NPZ's duration (time-scaling can
lengthen a clip). A clip whose duration cannot be read gets the floor.
``--stage-timeout-s S`` replaces the scaled limit with one fixed limit for
every stage (it must be a finite number > 0). The limits are deliberately very
generous (decision 2026-10-12): a timeout means the stage could not be
checked, never that it passed, so it is BLOCKED (exit code 124, reason
"[driver] timed out after ..."). Each stage entry records its effective
``timeout_s`` (null for a stage that never launched a subprocess) and the
ledger ``config.stage_timeouts`` records the policy and inputs.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from importlib import metadata as importlib_metadata
import json
import math
from pathlib import Path
import platform
import subprocess
import sys
import time

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from mocap_g1.assets import official_urdf_path  # noqa: E402
from mocap_g1.io_utils import (  # noqa: E402
    git_revision,
    git_worktree_dirty,
    read_npz_metadata,
    sha256_file,
    write_json_atomic,
)
from mocap_g1.paths import resolve_g1_mjcf  # noqa: E402


LEDGER_VERSION = 3
STATUS_RANK = {"PASS": 0, "BLOCKED": 1, "FAIL": 2}
#: Stages that are always run and reported but never decide the overall
#: verdict (see the module docstring, "Overall verdict").
DIAGNOSTIC_STAGES = frozenset({"window_audit"})
#: Fixed pipeline positions (``order``); a skipped stage leaves its slot vacant.
STAGE_ORDER = {
    "window_audit": 1,
    "conditioning": 2,
    "contact_refinement": 3,
    "self_collision_gate": 4,
    "controller_screen": 5,
    "balance_screen": 6,
    "npz_to_pkl": 7,
    "phuma_gate_v2": 8,
}
#: The ONE place each stage tool's own verdict word is mapped onto the ledger
#: vocabulary (PASS / FAIL = physical criterion violated / BLOCKED = could
#: not check).  A raw value absent from a stage's row is never guessed at: the
#: stage is BLOCKED with an "unexpected status" reason.  Crashes, timeouts,
#: missing/malformed reports and exit-code/status disagreements are BLOCKED
#: before this table is consulted.  conditioning and npz_to_pkl have no FAIL
#: outcome by construction (they either produce their artifact or exit 2).
#: Field read per stage:
#:   window_audit         summary.status
#:   contact_refinement   status          (REJECT = repair refused -> FAIL)
#:   self_collision_gate  summary.status
#:   controller_screen    controller_screen.status
#:   balance_screen       balance_screen.status (report schema v2)
#:   npz_to_pkl           status
#:   phuma_gate_v2        motions.<clip>.gate_v2.pass (bool, as "true"/"false")
STAGE_STATUS_TRANSLATION: dict[str, dict[str, str]] = {
    "window_audit": {"PASS": "PASS", "FAIL": "FAIL"},
    "contact_refinement": {"PASS": "PASS", "REJECT": "FAIL"},
    "self_collision_gate": {"PASS": "PASS", "FAIL": "FAIL"},
    "controller_screen": {"PASS": "PASS", "FAIL": "FAIL"},
    "balance_screen": {"PASS": "PASS", "FAIL": "FAIL"},
    "npz_to_pkl": {"PASS": "PASS"},
    "phuma_gate_v2": {"true": "PASS", "false": "FAIL"},
}
#: Screens whose report also carries a ``hardware_candidate`` verdict.  A
#: screen that PASSes its own checks but whose hardware_candidate is BLOCKED
#: (e.g. placeholder zero-preview calibration) is BLOCKED in the ledger; a
#: physical FAIL always stays FAIL.  Value = (report key, accepted
#: hardware_candidate statuses).
HARDWARE_CANDIDATE_SCREENS = {
    "controller_screen": ("controller_screen", ("BLOCKED", "CONTROLLER_SCREEN_ONLY")),
    "balance_screen": ("balance_screen", ("BLOCKED", "BALANCE_SCREEN_ONLY")),
}
#: Report schema the driver requires from a stage tool, where it matters.
REQUIRED_REPORT_SCHEMA = {"balance_screen": 2}
DEFAULT_URDF = official_urdf_path()
#: Stage subprocess wall-clock limit = floor + per-clip-second allowance x clip
#: duration (see the module docstring, "Stage timeouts"). Deliberately very
#: generous (decision 2026-10-12): before its 2026-10-12 speed-up stage 3
#: took 870 s on a 36 s reference capture, which conditioning time-scales
#: 7.9x to 288 s
#: (~24 s per raw clip second, ~3 s per conditioned second); its limit is now
#: 7200 + 60 * 288 = 24470 s.
STAGE_TIMEOUT_FLOOR_S = 7200.0
STAGE_TIMEOUT_PER_CLIP_SECOND_S = 60.0
#: Exit code recorded for a subprocess killed at its timeout (as coreutils
#: ``timeout``); any code outside a stage's accepted set is BLOCKED.
TIMEOUT_RETURNCODE = 124
QUALIFICATION_ARTIFACT_NAMES = (
    "01_window_audit.json",
    "02_conditioned.npz",
    "02_conditioning.report.json",
    "03_contact_refined.npz",
    "03_contact_refinement.json",
    "04_self_collision_gate.json",
    "05_controller_screen.json",
    "06_balance_screen.json",
    "07_clip.pkl",
    "07_convert.report.json",
    "08_phuma_gate_v2.json",
)


# ---------------------------------------------------------------------------
# small shared helpers
# ---------------------------------------------------------------------------


def _run(cmd: list[str], *, timeout: float = STAGE_TIMEOUT_FLOOR_S
        ) -> tuple[subprocess.CompletedProcess, float]:
    start = time.monotonic()
    try:
        completed = subprocess.run(
            cmd, cwd=REPO_ROOT, capture_output=True, text=True, timeout=timeout, check=False,
        )
    except subprocess.TimeoutExpired as exc:
        duration = time.monotonic() - start
        completed = subprocess.CompletedProcess(
            cmd, returncode=TIMEOUT_RETURNCODE,
            stdout=_decode_output(exc.stdout),
            stderr=_decode_output(exc.stderr)
            + f"\n[driver] timed out after {timeout}s",
        )
        return completed, duration
    return completed, time.monotonic() - start


def _decode_output(value: str | bytes | None) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return str(value)


def _clip_duration_s(path: Path | None) -> float | None:
    """Clip duration (frames / fps) of a qpos NPZ, or None when unreadable.

    Only sizes the stage timeout; validating the NPZ is the stage tools' job.
    """
    if path is None:
        return None
    try:
        with np.load(path, allow_pickle=False) as archive:
            frames = int(archive["qpos"].shape[0])
            fps = float(archive["fps"])
    except (OSError, ValueError, KeyError, TypeError, IndexError):
        return None
    if frames < 1 or not math.isfinite(fps) or fps <= 0.0:
        return None
    return frames / fps


def stage_timeout_s(clip_duration_s: float | None, override_s: float | None = None) -> float:
    """Effective per-stage subprocess timeout (see "Stage timeouts")."""
    if override_s is not None:
        return float(override_s)
    if clip_duration_s is None:
        return STAGE_TIMEOUT_FLOOR_S
    return STAGE_TIMEOUT_FLOOR_S + STAGE_TIMEOUT_PER_CLIP_SECOND_S * float(clip_duration_s)


def _with_timeout(stage: dict, timeout_s: float) -> dict:
    """Record the effective timeout on a stage entry that launched a subprocess."""
    stage["timeout_s"] = round(timeout_s, 3) if stage.get("command") is not None else None
    return stage


def _positive_seconds(value: str) -> float:
    try:
        seconds = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"not a number: {value!r}") from exc
    if not math.isfinite(seconds) or seconds <= 0.0:
        raise argparse.ArgumentTypeError(
            f"must be a finite number of seconds > 0, got {value!r}"
        )
    return seconds


def _read_json(path: Path) -> dict | None:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    return payload if isinstance(payload, dict) else None


def _stderr_tail(completed: subprocess.CompletedProcess) -> list[str]:
    text = _decode_output(completed.stderr).strip()
    return [text.splitlines()[-1]] if text else []


def _report_failure_reasons(
    completed: subprocess.CompletedProcess, path: Path, payload: dict | None
) -> list[str]:
    reasons = _cli_crash_reasons(completed)
    if payload is None:
        reasons.append(f"missing or malformed JSON report: {path}")
    return reasons


def _string_list(value: object) -> list[str] | None:
    if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
        return None
    return list(value)


def _translate_status(stage: str, raw: object) -> str | None:
    """Ledger status for a stage tool's own verdict word, or None if unknown."""
    if not isinstance(raw, str):
        return None
    return STAGE_STATUS_TRANSLATION[stage].get(raw)


def _status_returncode_reason(status: str, returncode: int) -> str | None:
    expected = 0 if status == "PASS" else 1
    if returncode != expected:
        return f"report status {status!r} conflicts with subprocess exit code {returncode}"
    return None


def _finite_number(value: object) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _integer_count(value: object, *, positive: bool = False) -> bool:
    return (
        isinstance(value, int)
        and not isinstance(value, bool)
        and (value > 0 if positive else value >= 0)
    )


def _self_collision_summary_reason(status: object, summary: dict) -> str | None:
    frames = summary.get("frames")
    collisions = summary.get("self_collision_frames")
    fraction = summary.get("self_collision_fraction")
    if not _integer_count(frames, positive=True):
        return "self-collision summary frames must be a positive integer"
    if not _integer_count(collisions):
        return "self-collision summary collision count must be a nonnegative integer"
    if collisions > frames:
        return "self-collision summary collision count exceeds frame count"
    if not _finite_number(fraction) or not 0.0 <= float(fraction) <= 1.0:
        return "self-collision summary fraction must be finite and within [0, 1]"
    expected_fraction = collisions / frames
    if not math.isclose(float(fraction), expected_fraction, rel_tol=1e-9, abs_tol=1e-12):
        return "self-collision summary fraction disagrees with frame counts"
    expected_status = "PASS" if collisions == 0 else "FAIL"
    if status != expected_status:
        return (
            f"self-collision summary status {status!r} conflicts with collision count "
            f"{collisions}"
        )
    return None


def _blocked_report(
    stage: str,
    order: int,
    completed: subprocess.CompletedProcess,
    duration: float,
    cmd: list[str],
    report_path: Path,
    payload: dict | None,
    artifacts: dict,
    extra_reason: str | None = None,
) -> dict:
    reasons = _report_failure_reasons(completed, report_path, payload)
    if extra_reason is not None:
        reasons.append(extra_reason)
    return _stage_dict(stage, order, "BLOCKED", reasons, cmd, completed.returncode,
                       duration, artifacts)


def _artifact(path: Path | None) -> dict | None:
    if path is None or not path.is_file():
        return None
    return {"path": str(path.resolve()), "sha256": sha256_file(path)}


def _stage_dict(stage: str, order: int, status: str, reasons: list[str],
                command: list[str] | None, returncode: int | None,
                duration_s: float | None, artifacts: dict) -> dict:
    if status not in STATUS_RANK:
        raise AssertionError(f"internal error: invalid stage status {status!r}")
    if STAGE_ORDER.get(stage) != order:
        raise AssertionError(f"internal error: stage {stage!r} cannot have order {order}")
    return {
        "stage": stage,
        "order": order,
        "status": status,
        "gates_verdict": stage not in DIAGNOSTIC_STAGES,
        "reasons": list(reasons),
        "command": command,
        "returncode": returncode,
        "duration_s": round(duration_s, 3) if duration_s is not None else None,
        "artifacts": artifacts,
    }


def _blocked_stage(stage: str, order: int, reasons: list[str]) -> dict:
    return _stage_dict(stage, order, "BLOCKED", reasons, None, None, None, {})


def _cli_crash_reasons(completed: subprocess.CompletedProcess) -> list[str]:
    reasons = [f"exit code {completed.returncode}"]
    reasons.extend(_stderr_tail(completed))
    return reasons


# ---------------------------------------------------------------------------
# stage 1: window audit (raw input; diagnostic only, does not gate the chain
# and is excluded from the overall verdict -- see DIAGNOSTIC_STAGES)
# ---------------------------------------------------------------------------


def _stage_window_audit(recording: Path, out_dir: Path, limits_urdf: Path, *,
                        timeout_s: float = STAGE_TIMEOUT_FLOOR_S) -> dict:
    order = STAGE_ORDER["window_audit"]
    report_path = out_dir / "01_window_audit.json"
    cmd = [
        sys.executable, str(REPO_ROOT / "tools/xsens/audit_g1_recording.py"),
        str(recording), "--output", str(report_path), "--limits-urdf", str(limits_urdf),
    ]
    completed, duration = _run(cmd, timeout=timeout_s)
    payload = _read_json(report_path) if report_path.is_file() else None
    artifacts = {"report": _artifact(report_path)}
    if completed.returncode not in (0, 1) or payload is None:
        return _blocked_report("window_audit", order, completed, duration, cmd,
                               report_path, payload, artifacts)
    summary = payload.get("summary")
    if not isinstance(summary, dict):
        return _blocked_report(
            "window_audit", order, completed, duration, cmd, report_path, payload,
            artifacts, "report summary must be an object",
        )
    raw_status = summary.get("status")
    status = _translate_status("window_audit", raw_status)
    reasons = _string_list(summary.get("reasons"))
    if status is None:
        return _blocked_report(
            "window_audit", order, completed, duration, cmd, report_path, payload,
            artifacts, f"unexpected summary.status {raw_status!r} in {report_path}",
        )
    if reasons is None:
        return _blocked_report(
            "window_audit", order, completed, duration, cmd, report_path, payload,
            artifacts, "report summary.reasons must be a list of strings",
        )
    mismatch = _status_returncode_reason(status, completed.returncode)
    if mismatch is not None:
        return _blocked_report(
            "window_audit", order, completed, duration, cmd, report_path, payload,
            artifacts, mismatch,
        )
    return _stage_dict("window_audit", order, status, reasons, cmd, completed.returncode,
                        duration, artifacts)


# ---------------------------------------------------------------------------
# stage 2: conditioning (raw input -> conditioned NPZ; roots stages 3-8)
# ---------------------------------------------------------------------------


def _stage_conditioning(recording: Path, out_dir: Path, urdf: Path, *,
                        timeout_s: float = STAGE_TIMEOUT_FLOOR_S) -> tuple[dict, Path | None]:
    order = STAGE_ORDER["conditioning"]
    output = out_dir / "02_conditioned.npz"
    report_path = out_dir / "02_conditioning.report.json"
    cmd = [
        sys.executable, str(REPO_ROOT / "tools/condition_g1_motion.py"),
        str(recording), str(output), "--report", str(report_path), "--urdf", str(urdf),
    ]
    completed, duration = _run(cmd, timeout=timeout_s)
    payload = _read_json(report_path) if report_path.is_file() else None
    artifacts = {
        "report": _artifact(report_path),
        "output_npz": _artifact(output),
    }
    required_objects = ("input", "config", "limits", "time_scaling", "output")
    schema_ok = payload is not None and all(
        isinstance(payload.get(key), dict) for key in required_objects
    )
    if completed.returncode == 0 and output.is_file() and schema_ok:
        stage = _stage_dict(
            "conditioning", order, "PASS", [], cmd, completed.returncode, duration,
            artifacts,
        )
        return stage, output
    # condition_g1_motion.py has no FAIL outcome by construction: it either
    # resolves every velocity/acceleration cap via time-scaling and returns
    # 0, or raises (exit 2) before ever writing an incomplete artifact.
    extra_reason = None
    if completed.returncode == 0 and not schema_ok:
        extra_reason = (
            "conditioning report is missing required object sections: "
            + ", ".join(required_objects)
        )
    elif completed.returncode == 0 and not output.is_file():
        extra_reason = "conditioning reported success but did not produce an output NPZ"
    stage = _blocked_report("conditioning", order, completed, duration, cmd, report_path,
                            payload, artifacts, extra_reason)
    return stage, None


# ---------------------------------------------------------------------------
# stage 3: contact refinement (conditioned NPZ; repairs world-contact
# artifacts; never gates stages 4-8, which fall back to the conditioned NPZ)
# ---------------------------------------------------------------------------


def _stage_contact_refinement(conditioned: Path | None, out_dir: Path,
                              model: Path | None, limits_urdf: Path, *,
                              timeout_s: float = STAGE_TIMEOUT_FLOOR_S,
                              ) -> tuple[dict, Path | None]:
    order = STAGE_ORDER["contact_refinement"]
    if conditioned is None:
        return _blocked_stage("contact_refinement", order,
                              ["upstream stage 'conditioning' did not produce an output artifact"]), None
    if model is None:
        return _blocked_stage("contact_refinement", order,
                              ["no G1 MJCF model resolved (set $MOCAP_WS or pass --model)"]), None
    output = out_dir / "03_contact_refined.npz"
    report_path = out_dir / "03_contact_refinement.json"
    cmd = [
        sys.executable, str(REPO_ROOT / "tools/retarget/refine_g1_contacts.py"),
        str(conditioned), "--model", str(model),
        "--output", str(output), "--report", str(report_path),
        "--limits-urdf", str(limits_urdf),
    ]
    completed, duration = _run(cmd, timeout=timeout_s)
    payload = _read_json(report_path) if report_path.is_file() else None
    artifacts = {
        **({"report": _artifact(report_path)} if report_path.is_file() else {}),
        **({"output_npz": _artifact(output)} if output.is_file() else {}),
    }
    if completed.returncode not in (0, 1) or payload is None:
        stage = _blocked_report("contact_refinement", order, completed, duration, cmd,
                                report_path, payload, artifacts)
        return stage, None
    raw_status = payload.get("status")
    status = _translate_status("contact_refinement", raw_status)
    summary = payload.get("summary")
    if status is None:
        return _blocked_report(
            "contact_refinement", order, completed, duration, cmd, report_path,
            payload, artifacts, f"unexpected report.status {raw_status!r} in {report_path}",
        ), None
    if not isinstance(summary, dict):
        return _blocked_report(
            "contact_refinement", order, completed, duration, cmd, report_path,
            payload, artifacts, "contact-refinement report summary must be an object",
        ), None
    mismatch = _status_returncode_reason(status, completed.returncode)
    if mismatch is not None:
        return _blocked_report(
            "contact_refinement", order, completed, duration, cmd, report_path,
            payload, artifacts, mismatch,
        ), None
    if status == "PASS":
        if not output.is_file():
            return _blocked_report(
                "contact_refinement", order, completed, duration, cmd, report_path,
                payload, artifacts, "tool reported PASS but did not write its output NPZ",
            ), None
        refined = output
        reasons: list[str] = []
    else:  # FAIL (tool status REJECT): the repair was refused
        rejections = payload.get("rejections")
        if not isinstance(rejections, list) or any(
            not isinstance(item, dict) or not isinstance(item.get("reason"), str)
            for item in rejections
        ):
            return _blocked_report(
                "contact_refinement", order, completed, duration, cmd, report_path,
                payload, artifacts, "REJECT report rejections must be a list of reason objects",
            ), None
        reasons = [
            item["reason"] for item in rejections
        ] or ["rejected"]
        reasons.append(
            f"stages 4-8 fall back to the unrefined conditioned NPZ "
            f"(penetration_failed={summary.get('penetration_failed_frames')}, "
            f"residual_skating={summary.get('residual_skating_frames')})"
        )
        refined = None
    return _stage_dict("contact_refinement", order, status,
                       reasons, cmd, completed.returncode, duration, artifacts), refined


# ---------------------------------------------------------------------------
# stage 4: self-collision gate (shared NPZ; gates the verdict, not other stages)
# ---------------------------------------------------------------------------


def _stage_self_collision_gate(shared_npz: Path | None, out_dir: Path, model: Path | None,
                               *, timeout_s: float = STAGE_TIMEOUT_FLOOR_S) -> dict:
    order = STAGE_ORDER["self_collision_gate"]
    if shared_npz is None:
        return _blocked_stage("self_collision_gate", order,
                              ["upstream stage 'conditioning' did not produce an output artifact"])
    if model is None:
        return _blocked_stage("self_collision_gate", order,
                              ["no G1 MJCF model resolved (set $MOCAP_WS or pass --model)"])
    report_path = out_dir / "04_self_collision_gate.json"
    cmd = [
        sys.executable, str(REPO_ROOT / "tools/control/audit_g1_self_collisions.py"),
        str(shared_npz), "--output", str(report_path), "--model", str(model),
    ]
    completed, duration = _run(cmd, timeout=timeout_s)
    payload = _read_json(report_path) if report_path.is_file() else None
    artifacts = {"report": _artifact(report_path)} if report_path.is_file() else {}
    if completed.returncode not in (0, 1) or payload is None:
        return _blocked_report("self_collision_gate", order, completed, duration, cmd,
                               report_path, payload, artifacts)
    summary = payload.get("summary")
    if not isinstance(summary, dict):
        return _blocked_report(
            "self_collision_gate", order, completed, duration, cmd, report_path,
            payload, artifacts, "self-collision report summary must be an object",
        )
    raw_status = summary.get("status")
    status = _translate_status("self_collision_gate", raw_status)
    if status is None:
        return _blocked_report(
            "self_collision_gate", order, completed, duration, cmd, report_path,
            payload, artifacts, f"unexpected summary.status {raw_status!r} in {report_path}",
        )
    summary_reason = _self_collision_summary_reason(status, summary)
    if summary_reason is not None:
        return _blocked_report(
            "self_collision_gate", order, completed, duration, cmd, report_path,
            payload, artifacts, summary_reason,
        )
    mismatch = _status_returncode_reason(status, completed.returncode)
    if mismatch is not None:
        return _blocked_report(
            "self_collision_gate", order, completed, duration, cmd, report_path,
            payload, artifacts, mismatch,
        )
    reasons: list[str] = []
    if status == "FAIL":
        frames = summary.get("frames")
        collide = summary.get("self_collision_frames")
        frac = summary.get("self_collision_fraction")
        reasons = [
            f"{collide}/{frames} frames self-collide "
            f"({round(100.0 * frac, 2) if frac is not None else '?'}%)"
        ]
    return _stage_dict("self_collision_gate", order, status, reasons, cmd, completed.returncode,
                        duration, artifacts)


# ---------------------------------------------------------------------------
# stages 5 + 6: controller screen and balance screen (shared NPZ; each gates
# the verdict, neither gates any other stage)
# ---------------------------------------------------------------------------


def _screen_stage(stage: str, shared_npz: Path | None, out_dir: Path, model: Path | None,
                  tool: str, report_name: str, extra_args: list[str], *,
                  timeout_s: float = STAGE_TIMEOUT_FLOOR_S) -> dict:
    """Run one MuJoCo screen that reports ``<key>.status`` + ``hardware_candidate``."""
    order = STAGE_ORDER[stage]
    if shared_npz is None:
        return _blocked_stage(stage, order,
                              ["upstream stage 'conditioning' did not produce an output artifact"])
    if model is None:
        return _blocked_stage(stage, order,
                              ["no G1 MJCF model resolved (set $MOCAP_WS or pass --model)"])
    report_key, hardware_statuses = HARDWARE_CANDIDATE_SCREENS[stage]
    report_path = out_dir / report_name
    cmd = [
        sys.executable, str(REPO_ROOT / tool),
        str(shared_npz), "--model", str(model), "--output", str(report_path), *extra_args,
    ]
    completed, duration = _run(cmd, timeout=timeout_s)
    payload = _read_json(report_path) if report_path.is_file() else None
    artifacts = {"report": _artifact(report_path)} if report_path.is_file() else {}

    def blocked(reason: str | None = None) -> dict:
        return _blocked_report(stage, order, completed, duration, cmd, report_path,
                               payload, artifacts, reason)

    if completed.returncode not in (0, 1) or payload is None:
        return blocked()
    required_schema = REQUIRED_REPORT_SCHEMA.get(stage)
    if required_schema is not None and payload.get("schema_version") != required_schema:
        return blocked(
            f"{stage} report schema_version {payload.get('schema_version')!r} is not the "
            f"required {required_schema} (status vocabulary changed at v{required_schema})"
        )
    screen = payload.get(report_key)
    hw = payload.get("hardware_candidate")
    if not isinstance(screen, dict) or not isinstance(hw, dict):
        return blocked(f"{stage} report requires {report_key} and hardware_candidate objects")
    raw_status = screen.get("status")
    status = _translate_status(stage, raw_status)
    reasons = _string_list(screen.get("reasons"))
    hardware_status = hw.get("status")
    hardware_reasons = _string_list(hw.get("reasons"))
    if status is None or reasons is None:
        return blocked(f"malformed {report_key} status/reasons in {report_path}")
    if hardware_status not in hardware_statuses or hardware_reasons is None:
        return blocked(f"malformed hardware_candidate status/reasons in {report_path}")
    mismatch = _status_returncode_reason(status, completed.returncode)
    if mismatch is not None:
        return blocked(mismatch)
    # The "hardware-candidate status" convention: a screen can pass its
    # own checks yet still be BLOCKED as hardware evidence (e.g. it only
    # carries placeholder zero-preview calibration). Surface that override at
    # the ledger's stage level. Only a screen that would otherwise PASS is
    # downgraded: a physical failure stays FAIL, since BLOCKED means "could
    # not check", not "checked and violated".
    if status == "PASS" and hardware_status == "BLOCKED":
        status = "BLOCKED"
        reasons = hardware_reasons
    return _stage_dict(stage, order, status, reasons, cmd, completed.returncode,
                       duration, artifacts)


def _stage_controller_screen(shared_npz: Path | None, out_dir: Path, model: Path | None,
                             limits_urdf: Path, *, timeout_s: float = STAGE_TIMEOUT_FLOOR_S) -> dict:
    return _screen_stage(
        "controller_screen", shared_npz, out_dir, model,
        "tools/control/qualify_g1_upper_body.py", "05_controller_screen.json",
        ["--limits-urdf", str(limits_urdf)], timeout_s=timeout_s,
    )


def _stage_balance_screen(shared_npz: Path | None, out_dir: Path, model: Path | None,
                          limits_urdf: Path, *, timeout_s: float = STAGE_TIMEOUT_FLOOR_S) -> dict:
    return _screen_stage(
        "balance_screen", shared_npz, out_dir, model,
        "tools/control/screen_g1_balance.py", "06_balance_screen.json",
        ["--limits-urdf", str(limits_urdf)], timeout_s=timeout_s,
    )


# ---------------------------------------------------------------------------
# stage 7: npz -> pkl conversion (shared NPZ; roots stage 8)
# ---------------------------------------------------------------------------


def _stage_npz_to_pkl(shared_npz: Path | None, out_dir: Path, *,
                      timeout_s: float = STAGE_TIMEOUT_FLOOR_S) -> tuple[dict, Path | None]:
    order = STAGE_ORDER["npz_to_pkl"]
    if shared_npz is None:
        return _blocked_stage(
            "npz_to_pkl", order,
            ["upstream stage 'conditioning' did not produce an output artifact"],
        ), None
    output = out_dir / "07_clip.pkl"
    report_path = out_dir / "07_convert.report.json"
    cmd = [
        sys.executable, str(REPO_ROOT / "tools/convert_npz_to_gmr_pkl.py"),
        str(shared_npz), str(output), "--report", str(report_path),
    ]
    completed, duration = _run(cmd, timeout=timeout_s)
    payload = _read_json(report_path) if report_path.is_file() else None
    conversion_reasons = (
        _string_list(payload.get("reasons"))
        if payload is not None and "reasons" in payload
        else None
    )
    schema_ok = (
        payload is not None
        and _translate_status("npz_to_pkl", payload.get("status")) == "PASS"
        and conversion_reasons == []
        and isinstance(payload.get("input"), dict)
        and isinstance(payload.get("output"), dict)
        and isinstance(payload.get("conversion"), dict)
    )
    artifacts = {
        "report": _artifact(report_path),
        "output_pkl": _artifact(output),
    }
    if completed.returncode == 0 and output.is_file() and schema_ok:
        stage = _stage_dict(
            "npz_to_pkl", order, "PASS", [], cmd, completed.returncode, duration,
            artifacts,
        )
        return stage, output
    extra_reason = None
    if completed.returncode == 0 and not schema_ok:
        extra_reason = "converter report is missing the PASS/input/output/conversion schema"
    elif completed.returncode == 0 and not output.is_file():
        extra_reason = "converter reported success but did not produce an output PKL"
    stage = _blocked_report("npz_to_pkl", order, completed, duration, cmd, report_path,
                            payload, artifacts, extra_reason)
    return stage, None


# ---------------------------------------------------------------------------
# stage 8: PHUMA metrics + gate v2 (via eval/score_motion.py)
# ---------------------------------------------------------------------------


def _stage_phuma_gate_v2(pkl: Path | None, out_dir: Path, model: Path | None, robot: str,
                         contact_policy: str, limits_urdf: Path, *,
                         timeout_s: float = STAGE_TIMEOUT_FLOOR_S) -> dict:
    order = STAGE_ORDER["phuma_gate_v2"]
    if pkl is None:
        return _blocked_stage(
            "phuma_gate_v2", order,
            ["upstream stage 'npz_to_pkl' did not produce an output artifact"],
        )
    if model is None:
        return _blocked_stage(
            "phuma_gate_v2", order,
            ["no G1 MJCF model resolved for --mjcf (set $MOCAP_WS or pass --model); "
             "without an explicit --mjcf, score_motion.py falls back to resolving the "
             "robot through the general_motion_retargeting package, which this driver "
             "does not require to be installed"],
        )
    out_path = out_dir / "08_phuma_gate_v2.json"
    cmd = [
        sys.executable, str(REPO_ROOT / "eval/score_motion.py"), str(pkl),
        "--robot", robot, "--mjcf", str(model), "--contact-policy", contact_policy,
        "--pos-urdf", str(limits_urdf),
        "--gate", "v2", "--out", str(out_path), "--json-only",
    ]
    completed, duration = _run(cmd, timeout=timeout_s)
    payload = _read_json(out_path) if out_path.is_file() else None
    artifacts = {"report": _artifact(out_path)} if out_path.is_file() else {}
    if completed.returncode != 0 or payload is None:
        return _blocked_report("phuma_gate_v2", order, completed, duration, cmd,
                               out_path, payload, artifacts)
    motions = payload.get("motions")
    if not isinstance(motions, dict):
        return _blocked_report(
            "phuma_gate_v2", order, completed, duration, cmd, out_path, payload,
            artifacts, "score report motions must be an object",
        )
    if len(motions) != 1:
        failures = payload.get("failures")
        failure_keys = list(failures) if isinstance(failures, dict) else []
        return _blocked_report(
            "phuma_gate_v2", order, completed, duration, cmd, out_path, payload,
            artifacts,
            f"expected exactly one scored motion in {out_path}, found {len(motions)} "
            f"(failures: {failure_keys})",
        )
    (motion_result,) = motions.values()
    if not isinstance(motion_result, dict):
        return _blocked_report(
            "phuma_gate_v2", order, completed, duration, cmd, out_path, payload,
            artifacts, "scored motion result must be an object",
        )
    gate = motion_result.get("gate_v2")
    if not isinstance(gate, dict):
        return _blocked_report(
            "phuma_gate_v2", order, completed, duration, cmd, out_path, payload,
            artifacts, f"score_motion.py did not return an object gate_v2 verdict in {out_path}",
        )
    joint_order = motion_result.get("joint_order")
    if not isinstance(joint_order, dict) or joint_order.get("status") != "verified":
        # Stage 7 always writes joint_names, so anything but "verified" means
        # the dof_pos columns were scored without an order check.
        return _blocked_report(
            "phuma_gate_v2", order, completed, duration, cmd, out_path, payload,
            artifacts, f"pkl joint order was not verified against the MJCF in {out_path}",
        )
    gate_pass = gate.get("pass")
    reasons = _string_list(gate.get("reasons"))
    status = (
        _translate_status("phuma_gate_v2", "true" if gate_pass else "false")
        if isinstance(gate_pass, bool) else None
    )
    if status is None or reasons is None:
        return _blocked_report(
            "phuma_gate_v2", order, completed, duration, cmd, out_path, payload,
            artifacts, f"malformed gate_v2 pass/reasons in {out_path}",
        )
    return _stage_dict("phuma_gate_v2", order, status, reasons, cmd, completed.returncode,
                        duration, artifacts)


# ---------------------------------------------------------------------------
# model / metadata resolution + orchestration
# ---------------------------------------------------------------------------


def _resolve_model(explicit: Path | None) -> Path | None:
    if explicit is not None:
        return explicit
    return resolve_g1_mjcf()


def _npz_metadata(path: Path) -> dict:
    return read_npz_metadata(path)


def _overall_verdict(stages: list[dict]) -> tuple[str, dict]:
    """(overall status, worst gating stage); diagnostic stages never decide it."""
    gating = [stage for stage in stages if stage["stage"] not in DIAGNOSTIC_STAGES]
    if not gating:
        raise AssertionError("internal error: no verdict-gating stage was run")
    # max() keeps the first of equal keys, i.e. the earliest pipeline stage.
    worst = max(gating, key=lambda stage: STATUS_RANK[stage["status"]])
    return worst["status"], worst


def _package_version(name: str) -> str | None:
    try:
        return importlib_metadata.version(name)
    except importlib_metadata.PackageNotFoundError:
        return None


def _provenance() -> dict:
    """Code + environment provenance; every probe degrades to None, never raises."""
    return {
        "driver": {
            "path": "tools/qualify_g1_recording.py",
            "project_git_rev": git_revision(REPO_ROOT),
            # True/False from `git status --porcelain`; None = not a git
            # checkout or git unavailable (unknown, NOT clean).
            "project_git_dirty": git_worktree_dirty(REPO_ROOT),
        },
        "environment": {
            "python": platform.python_version(),
            "numpy": _package_version("numpy"),
            "mujoco": _package_version("mujoco"),
        },
    }


def _select_contact_policy(requested: str, refinement_enabled: bool,
                           refined: Path | None) -> tuple[str, str]:
    """(effective stage-8 contact policy, reason) -- see "Stage-8 contact policy"."""
    if refined is not None:
        return "anchored", (
            "stage 3 contact_refinement PASSed and its refined NPZ ships: its world-frame "
            "contact, penetration and skating checks establish the grounding 'anchored' "
            "requires"
        )
    if not refinement_enabled:
        return requested, "--no-contact-refinement: no grounding evidence; using --contact-policy"
    return requested, (
        "stage 3 contact_refinement did not PASS: no grounding evidence; using --contact-policy"
    )


def _default_out_dir(recording: Path) -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return REPO_ROOT / "artifacts" / "qualify-runs" / f"{recording.stem}-{stamp}"


def _prepare_out_dir(out_dir: Path) -> None:
    if out_dir.exists():
        if not out_dir.is_dir():
            raise OSError(f"qualification output path is not a directory: {out_dir}")
        stale = [out_dir / name for name in QUALIFICATION_ARTIFACT_NAMES
                 if (out_dir / name).exists()]
        if stale:
            names = ", ".join(path.name for path in stale)
            raise OSError(
                "refusing to reuse qualification output directory with existing "
                f"stage artifacts: {names}; choose a fresh --out-dir"
            )
        return
    out_dir.mkdir(parents=True, exist_ok=True)


def _write_ledger(path: Path, payload: dict) -> None:
    # Deliberately allowed to overwrite: this is the driver's own top-level
    # run summary, refreshed every invocation (unlike the stage tools'
    # individual report files, whose overwrite-refusal semantics are
    # inherited unmodified).
    write_json_atomic(path, payload, overwrite=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("recording", type=Path, help="G1 .npz capture (raw or already conditioned)")
    parser.add_argument(
        "--offsets", type=Path, default=None,
        help="optional calibration/offsets JSON to record (path + sha256) in the ledger for "
             "provenance; not consumed by any stage tool, which read calibration state from the "
             "recording's own embedded metadata.offsets_key",
    )
    parser.add_argument(
        "--out-dir", type=Path, default=None,
        help="directory for all stage artifacts and ledger.json (default: "
             "artifacts/qualify-runs/<recording-stem>-<UTC timestamp>/)",
    )
    parser.add_argument(
        "--model", type=Path, default=None,
        help="G1 mocap MJCF for stages 3/4/5/6/8 (default: $MOCAP_WS/GMR/assets/unitree_g1/"
             "g1_mocap_29dof.xml, else ../mocap_g1_runtime/GMR/assets/unitree_g1/"
             "g1_mocap_29dof.xml next to this repo)",
    )
    parser.add_argument("--robot", default="unitree_g1", help="GMR robot name for stage 8")
    parser.add_argument(
        "--contact-policy", default="adaptive", choices=["adaptive", "fixed", "anchored"],
        help="stage 8 contact policy passed to score_motion.py when stage 3 does NOT pass "
             "(default: adaptive, the honest choice for an ungrounded clip). A stage-3 PASS "
             "always scores with 'anchored' -- see the module docstring",
    )
    parser.add_argument(
        "--no-contact-refinement", action="store_true",
        help="skip stage 3 (contact refinement) entirely; stages 4-8 then consume the "
             "conditioned NPZ directly and position 3 stays vacant",
    )
    parser.add_argument(
        "--limits-urdf", type=Path, default=DEFAULT_URDF,
        help="mechanical URDF: the single joint-position-limit source for stages "
             "1/2/3/5/6/8 (default: the vendored official G1 URDF)",
    )
    parser.add_argument(
        "--stage-timeout-s", type=_positive_seconds, default=None, metavar="SECONDS",
        help="fixed wall-clock limit for EVERY stage subprocess, replacing the default "
             f"{STAGE_TIMEOUT_FLOOR_S:.0f} s + {STAGE_TIMEOUT_PER_CLIP_SECOND_S:.0f} s per "
             "second of clip; must be finite and > 0. A timed-out stage is BLOCKED",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        recording = args.recording.resolve()
        if not recording.is_file():
            print(f"ERROR: recording not found: {recording}", file=sys.stderr)
            return 2

        explicit_model = args.model.resolve() if args.model else None
        if explicit_model is not None and not explicit_model.is_file():
            print(f"ERROR: --model {explicit_model} does not exist", file=sys.stderr)
            return 2
        model = _resolve_model(explicit_model)
        limits_urdf = args.limits_urdf.resolve()
        if not limits_urdf.is_file():
            print(f"ERROR: --limits-urdf {limits_urdf} does not exist", file=sys.stderr)
            return 2

        out_dir = (args.out_dir.resolve() if args.out_dir else _default_out_dir(recording))
        _prepare_out_dir(out_dir)

        recording_sha256 = sha256_file(recording)
        recording_metadata = _npz_metadata(recording)
        model_sha256 = sha256_file(model) if model is not None else None
        limits_urdf_sha256 = sha256_file(limits_urdf)

        offsets_info: dict[str, object] = {
            "key": recording_metadata.get("offsets_key"),
            "path": None,
            "sha256": None,
        }
        if args.offsets is not None:
            offsets_path = args.offsets.resolve()
            if not offsets_path.is_file():
                raise OSError(f"--offsets file does not exist: {offsets_path}")
            offsets_info["path"] = str(offsets_path)
            offsets_info["sha256"] = sha256_file(offsets_path)
    except (OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    stages: list[dict] = []
    override_s = args.stage_timeout_s
    recording_duration_s = _clip_duration_s(recording)
    input_timeout_s = stage_timeout_s(recording_duration_s, override_s)

    stages.append(_with_timeout(
        _stage_window_audit(recording, out_dir, limits_urdf, timeout_s=input_timeout_s),
        input_timeout_s,
    ))

    conditioning_stage, conditioned = _stage_conditioning(
        recording, out_dir, limits_urdf, timeout_s=input_timeout_s
    )
    stages.append(_with_timeout(conditioning_stage, input_timeout_s))
    # Stages 3-8 work on the conditioned clip; time-scaling may lengthen it.
    conditioned_duration_s = _clip_duration_s(conditioned)
    known_durations = [d for d in (recording_duration_s, conditioned_duration_s) if d is not None]
    chain_timeout_s = stage_timeout_s(
        max(known_durations) if known_durations else None, override_s
    )

    shared_npz = conditioned
    refined: Path | None = None
    if not args.no_contact_refinement:
        refinement_stage, refined = _stage_contact_refinement(
            conditioned, out_dir, model, limits_urdf, timeout_s=chain_timeout_s
        )
        stages.append(_with_timeout(refinement_stage, chain_timeout_s))
        # A rejected/absent refinement never gates the rest of the chain: the
        # downstream diagnostics still run on the conditioned NPZ so their
        # verdicts remain visible in the ledger.
        if refined is not None:
            shared_npz = refined
    contact_policy, contact_policy_reason = _select_contact_policy(
        args.contact_policy, not args.no_contact_refinement, refined
    )

    stages.append(_with_timeout(_stage_self_collision_gate(
        shared_npz, out_dir, model, timeout_s=chain_timeout_s), chain_timeout_s))
    stages.append(_with_timeout(_stage_controller_screen(
        shared_npz, out_dir, model, limits_urdf, timeout_s=chain_timeout_s), chain_timeout_s))
    stages.append(_with_timeout(_stage_balance_screen(
        shared_npz, out_dir, model, limits_urdf, timeout_s=chain_timeout_s), chain_timeout_s))

    npz_to_pkl_stage, pkl = _stage_npz_to_pkl(shared_npz, out_dir, timeout_s=chain_timeout_s)
    stages.append(_with_timeout(npz_to_pkl_stage, chain_timeout_s))

    stages.append(_with_timeout(_stage_phuma_gate_v2(
        pkl, out_dir, model, args.robot, contact_policy, limits_urdf,
        timeout_s=chain_timeout_s,
    ), chain_timeout_s))

    overall, worst = _overall_verdict(stages)

    ledger = {
        "ledger_version": LEDGER_VERSION,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        **_provenance(),
        "input": {
            "recording_path": str(recording),
            "recording_sha256": recording_sha256,
            "recording_metadata": recording_metadata or None,
            "offsets": offsets_info,
        },
        "config": {
            "robot": args.robot,
            # contact_policy is the EFFECTIVE stage-8 policy.
            "contact_policy": contact_policy,
            "contact_policy_requested": args.contact_policy,
            "contact_policy_reason": contact_policy_reason,
            "contact_refinement_enabled": not args.no_contact_refinement,
            "model_path": str(model) if model else None,
            "model_sha256": model_sha256,
            "limits_urdf": str(limits_urdf),
            "limits_urdf_sha256": limits_urdf_sha256,
            "out_dir": str(out_dir),
            # Effective per-stage limits are each stage entry's timeout_s.
            "stage_timeouts": {
                "policy": "override" if override_s is not None else "scaled_by_clip_duration",
                "override_s": override_s,
                "floor_s": STAGE_TIMEOUT_FLOOR_S,
                "per_clip_second_s": STAGE_TIMEOUT_PER_CLIP_SECOND_S,
                "recording_duration_s": (
                    round(recording_duration_s, 3) if recording_duration_s is not None else None
                ),
                "conditioned_duration_s": (
                    round(conditioned_duration_s, 3)
                    if conditioned_duration_s is not None else None
                ),
                "input_stage_timeout_s": round(input_timeout_s, 3),
                "chain_stage_timeout_s": round(chain_timeout_s, 3),
                "timeout_outcome": "BLOCKED",
            },
        },
        "stages": stages,
        "verdict": {
            "overall": overall,
            "worst_stage": worst["stage"],
            "diagnostic_stages": sorted(DIAGNOSTIC_STAGES),
            "rule": "overall = status of the worst verdict-gating stage (stages with "
                    "gates_verdict=true; diagnostic_stages are reported only); severity "
                    "PASS(0) < BLOCKED(1) < FAIL(2); ties broken by earliest stage in "
                    "pipeline order",
        },
    }
    ledger_path = out_dir / "ledger.json"
    _write_ledger(ledger_path, ledger)

    print(f"overall verdict: {overall} (worst stage: {worst['stage']})")
    for stage in stages:
        tag = "" if stage["gates_verdict"] else "  (diagnostic; not in verdict)"
        print(f"  [{stage['status']:>7}] {stage['order']}. {stage['stage']}{tag}  "
              f"reasons={stage['reasons']}")
    print(f"ledger: {ledger_path}")
    return 0 if overall == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
