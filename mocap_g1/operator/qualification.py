"""Bounded, read-only summaries of recording-qualification ledgers.

The qualification driver writes detailed provenance, commands, and absolute
artifact paths.  None of that belongs in the operator API.  This module only
accepts the driver's aggregate ``ledger.json`` format and projects it into a
small, JSON-safe summary.
"""

from __future__ import annotations

from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import stat
from typing import Any
from urllib.parse import quote


LEDGER_FILENAMES = frozenset({"ledger.json"})
SUPPORTED_LEDGER_VERSIONS = frozenset({1, 2, 3})
# Ledger v3 added the balance screen as stage 6; v1/v2 ledgers keep their
# seven-stage order so older runs remain visible.
_STAGE_ORDER_V2 = {
    "window_audit": 1,
    "conditioning": 2,
    "contact_refinement": 3,
    "self_collision_gate": 4,
    "controller_screen": 5,
    "npz_to_pkl": 6,
    "phuma_gate_v2": 7,
}
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
STAGE_ORDER_BY_VERSION = {1: _STAGE_ORDER_V2, 2: _STAGE_ORDER_V2, 3: STAGE_ORDER}


def _valid_sequences(order: dict[str, int]) -> frozenset[tuple[str, ...]]:
    return frozenset(
        {
            tuple(order),
            tuple(name for name in order if name != "contact_refinement"),
        }
    )


VALID_STAGE_SEQUENCES_BY_VERSION = {
    version: _valid_sequences(order) for version, order in STAGE_ORDER_BY_VERSION.items()
}
STATUS_RANK = {"PASS": 0, "BLOCKED": 1, "FAIL": 2}

DEFAULT_MAX_FILES = 256
DEFAULT_MAX_JSON_BYTES = 1_048_576
DEFAULT_MAX_RESULTS = 100
DEFAULT_MAX_SCAN_ENTRIES = 10_000

_HARD_MAX_FILES = 1_024
_HARD_MAX_JSON_BYTES = 4_194_304
_HARD_MAX_RESULTS = 250
_HARD_MAX_SCAN_ENTRIES = 100_000
_MAX_ID_LENGTH = 512
_MAX_NAME_LENGTH = 160
_MAX_REASON_LENGTH = 512
_MAX_OPTIONAL_LABEL_LENGTH = 120

_POSIX_ABSOLUTE_PATH = re.compile(
    r"(?<![\w])/(?:[^\s/,:;(){}\[\]<>\"']+/)*[^\s/,:;(){}\[\]<>\"']+"
)
_WINDOWS_ABSOLUTE_PATH = re.compile(
    r"(?i)(?<![\w])(?:[a-z]:[\\/]|\\\\)[^\s,:;(){}\[\]<>\"']+"
)


class QualificationRootError(ValueError):
    """Raised when a configured qualification root is not a safe directory."""


def _bounded_limit(value: int, name: str, hard_maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ValueError(f"{name} must be a positive integer")
    return min(value, hard_maximum)


def _root_without_symlink_components(value: str | Path) -> Path | None:
    """Return a resolved directory, rejecting symlinks in its configured path."""

    try:
        absolute = Path(os.path.abspath(os.fspath(Path(value).expanduser())))
    except (OSError, TypeError, ValueError):
        return None

    current = Path(absolute.anchor)
    try:
        for part in absolute.parts[1:]:
            current /= part
            if stat.S_ISLNK(os.lstat(current).st_mode):
                return None
        resolved = absolute.resolve(strict=True)
        if not resolved.is_dir():
            return None
    except OSError:
        return None
    return resolved


def resolve_qualification_root(value: str | Path) -> Path:
    """Resolve an explicit artifact root while rejecting linked path components."""

    resolved = _root_without_symlink_components(value)
    if resolved is None:
        raise QualificationRootError(
            "qualification root must be an existing directory without symlinked path components"
        )
    return resolved


def _ledger_candidates(
    root: Path,
    *,
    max_scan_entries: int,
) -> list[Path]:
    """Walk without following links and stop at the scan-entry bound.

    Every ledger found within the bound is returned.  The file-count bound is
    applied later, by ``created_at``, so lexically late (typically newest)
    runs are never dropped merely because of their directory names.
    """

    candidates: list[Path] = []
    pending = [root]
    scanned = 0

    while pending and scanned < max_scan_entries:
        directory = pending.pop()
        child_directories: list[Path] = []
        try:
            with os.scandir(directory) as iterator:
                entries = []
                for entry in iterator:
                    if scanned >= max_scan_entries:
                        break
                    entries.append(entry)
                    scanned += 1
                entries.sort(key=lambda entry: entry.name)
        except OSError:
            continue

        for entry in entries:
            try:
                if entry.is_symlink():
                    continue
                if entry.is_dir(follow_symlinks=False):
                    child_directories.append(Path(entry.path))
                elif (
                    entry.name in LEDGER_FILENAMES
                    and entry.is_file(follow_symlinks=False)
                ):
                    candidates.append(Path(entry.path))
            except OSError:
                continue

        # ``pending`` is LIFO.  Reverse here so lexical order remains the
        # deterministic traversal order when the scan bound is reached.
        pending.extend(reversed(child_directories))

    return candidates


def _opened_file_is_within_root(
    path: Path,
    root: Path,
    opened_stat: os.stat_result,
) -> bool:
    """Defend the check/open gap before consuming bytes from an opened file."""

    try:
        resolved = path.resolve(strict=True)
        current_stat = resolved.stat()
    except OSError:
        return False
    if not resolved.is_relative_to(root):
        return False
    return (
        current_stat.st_dev == opened_stat.st_dev
        and current_stat.st_ino == opened_stat.st_ino
    )


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    value: dict[str, Any] = {}
    for key, item in pairs:
        if key in value:
            raise ValueError(f"duplicate JSON key: {key}")
        value[key] = item
    return value


def _reject_nonfinite_constant(value: str) -> None:
    raise ValueError(f"non-finite JSON number: {value}")


def _read_bounded_json(path: Path, root: Path, max_json_bytes: int) -> dict | None:
    flags = os.O_RDONLY
    flags |= getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(path, flags)
    except OSError:
        return None

    try:
        opened_stat = os.fstat(descriptor)
        if not stat.S_ISREG(opened_stat.st_mode):
            return None
        if opened_stat.st_size < 1 or opened_stat.st_size > max_json_bytes:
            return None
        if not _opened_file_is_within_root(path, root, opened_stat):
            return None

        chunks: list[bytes] = []
        remaining = max_json_bytes + 1
        while remaining:
            chunk = os.read(descriptor, min(65_536, remaining))
            if not chunk:
                break
            chunks.append(chunk)
            remaining -= len(chunk)
        raw = b"".join(chunks)
        if len(raw) > max_json_bytes:
            return None
    except OSError:
        return None
    finally:
        os.close(descriptor)

    try:
        payload = json.loads(
            raw.decode("utf-8"),
            object_pairs_hook=_reject_duplicate_keys,
            parse_constant=_reject_nonfinite_constant,
        )
    except (UnicodeDecodeError, ValueError, RecursionError):
        return None
    return payload if isinstance(payload, dict) else None


def _clean_text(value: object, *, limit: int) -> str | None:
    if not isinstance(value, str):
        return None
    text = " ".join(value.split())
    text = _WINDOWS_ABSOLUTE_PATH.sub("[path]", text)
    text = _POSIX_ABSOLUTE_PATH.sub("[path]", text)
    text = text.strip()
    if not text:
        return None
    if len(text) > limit:
        text = text[: max(1, limit - 1)].rstrip() + "…"
    return text


def _parse_created_at(value: object) -> tuple[str, datetime] | None:
    if not isinstance(value, str) or len(value) > 80:
        return None
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
        if parsed.tzinfo is None:
            return None
        utc = parsed.astimezone(timezone.utc)
    except (OverflowError, ValueError):
        return None
    return utc.isoformat().replace("+00:00", "Z"), utc


def _finite_number(value: object, *, nonnegative: bool = False) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    if not math.isfinite(number) or (nonnegative and number < 0):
        return None
    return number


def _stage_summary(
    value: object, stage_order: dict[str, int]
) -> tuple[dict[str, object], int] | None:
    if not isinstance(value, dict):
        return None
    name = value.get("stage")
    status = value.get("status")
    order = value.get("order")
    reasons = value.get("reasons")
    if name not in stage_order or status not in STATUS_RANK:
        return None
    if isinstance(order, bool) or order != stage_order[name]:
        return None
    if not isinstance(reasons, list) or not all(
        isinstance(reason, str) for reason in reasons
    ):
        return None

    summary: dict[str, object] = {"name": name, "status": status}
    cleaned_reasons = [
        clean
        for reason in reasons
        if (clean := _clean_text(reason, limit=_MAX_REASON_LENGTH)) is not None
    ]
    if cleaned_reasons:
        reason = "; ".join(cleaned_reasons)
        if len(reason) > _MAX_REASON_LENGTH:
            reason = reason[: _MAX_REASON_LENGTH - 1].rstrip() + "…"
        summary["reason"] = reason

    margin = _finite_number(value.get("margin"))
    if margin is not None:
        summary["margin"] = margin
    worst_time = _finite_number(value.get("worst_time_s"), nonnegative=True)
    if worst_time is not None:
        summary["worst_time_s"] = worst_time
    return summary, STATUS_RANK[status]


def _optional_public_label(value: object) -> str | bool | None:
    if isinstance(value, bool):
        return value
    return _clean_text(value, limit=_MAX_OPTIONAL_LABEL_LENGTH)


def _summary_name(relative_parent: Path, payload: dict) -> str:
    if relative_parent.parts:
        candidate = relative_parent.name
    else:
        recording = payload.get("input")
        recording_path = (
            recording.get("recording_path") if isinstance(recording, dict) else None
        )
        if isinstance(recording_path, str):
            candidate = (
                recording_path.replace("\\", "/").rstrip("/").rsplit("/", 1)[-1]
            )
            candidate = Path(candidate).stem
        else:
            candidate = "qualification-run"
    return _clean_text(candidate, limit=_MAX_NAME_LENGTH) or "qualification-run"


def _ledger_summary(payload: dict, relative_path: Path) -> tuple[dict, datetime] | None:
    version = payload.get("ledger_version")
    if isinstance(version, bool) or version not in SUPPORTED_LEDGER_VERSIONS:
        return None
    driver = payload.get("driver")
    if not isinstance(driver, dict) or driver.get("path") != "tools/qualify_g1_recording.py":
        return None

    created = _parse_created_at(payload.get("generated_at_utc"))
    if created is None:
        return None
    created_at, created_sort = created

    raw_stages = payload.get("stages")
    if not isinstance(raw_stages, list):
        return None
    parsed_stages: list[dict[str, object]] = []
    ranks: list[int] = []
    gating: list[int] = []
    for raw_stage in raw_stages:
        parsed = _stage_summary(raw_stage, STAGE_ORDER_BY_VERSION[version])
        if parsed is None:
            return None
        stage, rank = parsed
        # Diagnostic stages (gates_verdict: false, e.g. the raw-input window
        # audit) are shown but never decide the overall verdict. Ledgers
        # written before the flag existed treat every stage as gating.
        gates_verdict = raw_stage.get("gates_verdict", True)
        if not isinstance(gates_verdict, bool):
            return None
        if gates_verdict:
            gating.append(len(ranks))
        parsed_stages.append(stage)
        ranks.append(rank)

    stage_names = tuple(stage["name"] for stage in parsed_stages)
    if stage_names not in VALID_STAGE_SEQUENCES_BY_VERSION[version]:
        return None

    verdict = payload.get("verdict")
    if not isinstance(verdict, dict):
        return None
    overall = verdict.get("overall")
    if overall not in STATUS_RANK:
        return None
    if not gating:
        return None
    # max() keeps the first of equal keys, i.e. the earliest pipeline stage.
    worst_index = max(gating, key=ranks.__getitem__)
    if (
        ranks[worst_index] != STATUS_RANK[overall]
        or verdict.get("worst_stage") != stage_names[worst_index]
    ):
        return None

    relative_parent = relative_path.parent
    if ".." in relative_parent.parts:
        return None
    run_id = "/".join(quote(part, safe="-._~") for part in relative_parent.parts) or "."
    if len(run_id) > _MAX_ID_LENGTH:
        return None

    summary: dict[str, object] = {
        "id": run_id,
        "name": _summary_name(relative_parent, payload),
        "created_at": created_at,
        "verdict": overall,
        "stages": parsed_stages,
    }
    for key in ("reference_quality", "simulator_execution"):
        optional = _optional_public_label(payload.get(key))
        if optional is not None:
            summary[key] = optional
    return summary, created_sort


def read_qualification_runs(
    root: str | Path,
    *,
    max_files: int = DEFAULT_MAX_FILES,
    max_json_bytes: int = DEFAULT_MAX_JSON_BYTES,
    max_results: int = DEFAULT_MAX_RESULTS,
    max_scan_entries: int = DEFAULT_MAX_SCAN_ENTRIES,
) -> dict[str, list[dict[str, object]]]:
    """Return sanitized qualification summaries below one configured root.

    Missing, unreadable, linked, oversized, and malformed entries are ignored.
    Every ledger within ``max_scan_entries`` directory entries is considered;
    ``max_files`` bounds how many of the newest (by ``created_at``) summaries
    are retained.  Bounds are hard-capped even when a caller supplies a larger
    value.
    """

    file_limit = _bounded_limit(max_files, "max_files", _HARD_MAX_FILES)
    json_limit = _bounded_limit(
        max_json_bytes, "max_json_bytes", _HARD_MAX_JSON_BYTES
    )
    result_limit = _bounded_limit(max_results, "max_results", _HARD_MAX_RESULTS)
    scan_limit = _bounded_limit(
        max_scan_entries, "max_scan_entries", _HARD_MAX_SCAN_ENTRIES
    )

    try:
        resolved_root = resolve_qualification_root(root)
    except QualificationRootError:
        return {"runs": []}

    runs: list[tuple[dict[str, object], datetime]] = []
    candidates = _ledger_candidates(
        resolved_root,
        max_scan_entries=scan_limit,
    )
    for candidate in candidates:
        try:
            relative = candidate.relative_to(resolved_root)
        except ValueError:
            continue
        if relative.name not in LEDGER_FILENAMES or ".." in relative.parts:
            continue
        payload = _read_bounded_json(candidate, resolved_root, json_limit)
        if payload is None:
            continue
        summary = _ledger_summary(payload, relative)
        if summary is not None:
            runs.append(summary)
            # Retain only the newest ``max_files`` summaries so memory stays
            # bounded however many ledgers the scan bound admits.
            if len(runs) > 2 * file_limit:
                runs = _newest_first(runs)[:file_limit]

    runs = _newest_first(runs)[:file_limit]
    return {"runs": [summary for summary, _created in runs[:result_limit]]}


def _newest_first(
    runs: list[tuple[dict[str, object], datetime]],
) -> list[tuple[dict[str, object], datetime]]:
    # Stable two-pass sorting avoids converting extreme-but-valid datetimes to
    # platform timestamps while retaining an ID tiebreaker.
    ordered = sorted(runs, key=lambda item: str(item[0]["id"]))
    ordered.sort(key=lambda item: item[1], reverse=True)
    return ordered


__all__ = [
    "DEFAULT_MAX_FILES",
    "DEFAULT_MAX_JSON_BYTES",
    "DEFAULT_MAX_RESULTS",
    "LEDGER_FILENAMES",
    "QualificationRootError",
    "read_qualification_runs",
    "resolve_qualification_root",
]
