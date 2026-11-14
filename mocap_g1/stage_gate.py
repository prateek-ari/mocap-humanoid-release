"""The stage gate: which real-robot stage may run on this robot, from its ledger.

Every real-robot stage has a gate (its prerequisite stages).  A runner calls
:func:`require` with the stage it is about to run; ``require(X)`` answers "are
the prerequisites of X valid for this robot, these settings and now?", so an
empty list means X may start.  It walks the whole chain down to ``acceptance``
(S0) on every call and re-checks each entry.  After a run, the runner writes a
PASS with :meth:`Ledger.record_pass`; a ``release_unconfirmed``, a fall, or a
fault the stage forbids revokes the gate stage it ran under
(:meth:`Ledger.record_revocation`).  Drills tag their expected faults
``expected_fault`` in the attempt; only the runners decide what revokes.

One ledger per robot: ``<root>/<serial>/stages.json``.  Practice (fake robot)
runs write only ``<root>/PRACTICE/stages.json``, which never unlocks anything.
"""

from __future__ import annotations

from collections.abc import Iterator, Mapping
from contextlib import contextmanager
from dataclasses import dataclass
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import re
from typing import Any

from .io_utils import sha256_file, write_json_atomic

LEDGER_SCHEMA = 1
PRACTICE_DIR = "PRACTICE"
MAX_AGE = dt.timedelta(days=30)
IDENTITY_KEYS = ("serial", "firmware", "sdk_binding_version")
#: Settings that must equal the gate's PASS, and the caps that must not exceed it.
EXACT_SETTINGS = ("kp", "kd", "control_rate_hz", "model_sha256", "envelope_sha256",
                  "stop_settings")
CAP_SETTINGS = ("speed_max_rad_s", "accel_max_rad_s2")
#: The acceptance record approves one kp/kd pair and the caps, nothing else.
ACCEPTANCE_SETTINGS = ("kp", "kd")
ROOT_ENV = "K1_COMMISSIONING_ROOT"
REPO_ROOT = Path(__file__).resolve().parents[1]


class StageGateError(RuntimeError):
    """The ledger cannot be read or written as asked."""


@dataclass(frozen=True)
class StageSpec:
    key: str
    title: str
    gate: tuple[str, ...]
    #: Bump on a safety-relevant behaviour change of the stage's runner: old PASSes stop counting.
    contract: int


STAGES: dict[str, StageSpec] = {spec.key: spec for spec in (
    StageSpec("acceptance", "S0 · K2 hoist tests", (), 1),
    StageSpec("replay@hoist", "S0r · K3 replay on the hoist", ("acceptance",), 1),
    StageSpec("live-arms@hoist", "S1 · live arms on the hoist", ("replay@hoist",), 1),
    StageSpec("replay@standing", "S1r · K3 replay standing", ("live-arms@hoist",), 1),
    StageSpec("live-arms@standing", "S2 · live arms standing", ("replay@standing",), 1),
)}


def commissioning_root() -> Path:
    """Where sessions and ledgers live: ``$K1_COMMISSIONING_ROOT``, else
    ``<repo>/k1-commissioning``; always absolute, never the working directory."""
    configured = os.environ.get(ROOT_ENV, "").strip()
    path = Path(configured).expanduser() if configured else REPO_ROOT / "k1-commissioning"
    return path.resolve()


def now_utc() -> dt.datetime:
    return dt.datetime.now(dt.timezone.utc)


def entry_sha256(entry: Mapping[str, Any]) -> str:
    """The fingerprint a later PASS stores of this prerequisite entry."""
    text = json.dumps(entry, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def envelope_sha256(limits: Mapping[str, Any]) -> str:
    """Fingerprint of the envelope ranges (``JointLimit`` per joint)."""
    rows = {name: [limit.lower, limit.upper, limit.velocity] for name, limit in limits.items()}
    return hashlib.sha256(json.dumps(rows, sort_keys=True).encode("utf-8")).hexdigest()


def _folder(serial: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]", "_", serial) or "unknown"


class Ledger:
    """The stage entries of one robot (or of the practice robot)."""

    def __init__(self, root: Path, serial: str, *, practice: bool) -> None:
        folder = PRACTICE_DIR if practice else _folder(str(serial))
        if not practice and folder == PRACTICE_DIR:
            raise StageGateError(f"serial {serial!r} would write into the practice ledger")
        self.root = Path(root).resolve()
        self.serial = str(serial)
        self.practice = practice
        self.path = self.root / folder / "stages.json"

    def entries(self) -> list[dict[str, Any]]:
        """Every entry, oldest first; :class:`StageGateError` on a damaged file."""
        if not self.path.exists():
            return []
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            raise StageGateError(f"cannot read the ledger {self.path}: {exc}") from exc
        if not isinstance(data, dict) or data.get("schema") != LEDGER_SCHEMA:
            raise StageGateError(f"ledger {self.path} is not schema {LEDGER_SCHEMA}")
        entries = data.get("entries")
        if not isinstance(entries, list) or not all(isinstance(e, dict) for e in entries):
            raise StageGateError(f"ledger {self.path} has no entry list")
        if data.get("practice") is not self.practice:
            raise StageGateError(f"ledger {self.path} practice flag does not match its folder")
        return entries

    def latest(self, stage: str) -> dict[str, Any] | None:
        """The newest entry of ``stage`` (a PASS or a revocation), or ``None``."""
        found = None
        for entry in self.entries():
            if entry.get("stage") == stage:
                found = entry
        return found

    @contextmanager
    def _locked(self) -> Iterator[None]:
        import fcntl  # POSIX only, like the Booster SDK

        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd = os.open(self.path.parent / "stages.lock", os.O_CREAT | os.O_RDWR, 0o644)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX)
            yield
        finally:
            os.close(fd)  # closing releases the lock

    def _append(self, entry: dict[str, Any]) -> dict[str, Any]:
        with self._locked():
            entries = self.entries()
            entries.append(entry)
            write_json_atomic(self.path, {"schema": LEDGER_SCHEMA, "serial": self.serial,
                                          "practice": self.practice, "entries": entries},
                              overwrite=True)
        return entry

    def record_pass(self, stage: str, *, identity: Mapping[str, str], settings: Mapping[str, Any],
                    evidence: Mapping[str, Path], operator: str,
                    extra: Mapping[str, Any] | None = None) -> dict[str, Any]:
        """Append a PASS bound to its evidence files and to the current entries of its gate."""
        spec = _spec(stage)
        prerequisites: dict[str, str | None] = {}
        for gate in spec.gate:
            latest = self.latest(gate)
            if latest is None or latest.get("verdict") != "PASS":
                if not self.practice:  # require() ran first; this is a lost race
                    raise StageGateError(f"{stage} needs a {gate} PASS in the ledger first")
                prerequisites[gate] = None
            else:
                prerequisites[gate] = entry_sha256(latest)
        return self._append({
            "stage": stage, "verdict": "PASS", "date": now_utc().isoformat(timespec="seconds"),
            "operator": str(operator), "identity": {key: identity.get(key) for key in IDENTITY_KEYS},
            "settings": dict(settings), "contract": spec.contract,
            "evidence": _hash_evidence(evidence), "prerequisites": prerequisites,
            "practice": self.practice, "extra": dict(extra or {}),
        })

    def record_revocation(self, stage: str, *, reason: str,
                          evidence: Mapping[str, Path]) -> dict[str, Any]:
        """Append a revocation: ``stage`` stays locked until it passes again."""
        _spec(stage)
        return self._append({
            "stage": stage, "verdict": "REVOKED", "date": now_utc().isoformat(timespec="seconds"),
            "reason": str(reason), "evidence": _hash_evidence(evidence),
            "practice": self.practice,
        })


def _spec(stage: str) -> StageSpec:
    try:
        return STAGES[stage]
    except KeyError:
        raise StageGateError(f"unknown stage {stage!r}") from None


def _hash_evidence(evidence: Mapping[str, Path]) -> dict[str, dict[str, str]]:
    hashed = {}
    for name, path in evidence.items():
        resolved = Path(path).resolve()
        hashed[name] = {"path": str(resolved), "sha256": sha256_file(resolved)}
    return hashed


def runner_kind(stage: str) -> str:
    """``replay`` for ``replay@hoist``: stages of one kind share a runner and its stop settings."""
    return stage.split("@", 1)[0]


def settings_problems(gate: str, passed: Mapping[str, Any], requested: Mapping[str, Any],
                      *, stage: str | None = None) -> list[str]:
    """Where ``requested`` is not covered by the settings ``gate`` passed with.

    ``stop_settings`` describe one runner's stop behaviour (a replay and the live bridge stop
    differently by design), so they bind only between stages of the same kind; gains, caps,
    rate, model and envelope bind across kinds.
    """
    exact: tuple[str, ...] = ACCEPTANCE_SETTINGS if gate == "acceptance" else EXACT_SETTINGS
    if stage is not None and runner_kind(stage) != runner_kind(gate):
        exact = tuple(key for key in exact if key != "stop_settings")
    found = []
    for key in (*exact, *CAP_SETTINGS):
        if key not in requested:
            found.append(f"setting {key} is missing from this run")
        elif key not in passed:
            found.append(f"setting {key} is missing from the {gate} PASS")
        elif key in CAP_SETTINGS:
            if not _number(requested[key]) or not _number(passed[key]) \
                    or requested[key] > passed[key]:
                found.append(f"{key} {requested[key]} exceeds the {passed[key]} {gate} passed with")
        elif requested[key] != passed[key]:
            found.append(f"{key} {requested[key]} is not the {passed[key]} {gate} passed with")
    return found


def _number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def require(stage: str, *, ledger: Ledger, identity: Mapping[str, str], settings: Mapping[str, Any],
            policy_sha: str | None = None, now: dt.datetime | None = None) -> list[str]:
    """Why ``stage`` may not run on the real robot now; ``[]`` means it may.

    Checks the stage's gate entries and, recursively, theirs down to
    ``acceptance``: missing, revoked, practice, contract changed, identity
    (serial, firmware, SDK bindings), evidence files changed or deleted,
    prerequisite entries changed since the PASS, older than 30 days.  The
    requested ``settings`` are compared with the direct gate's PASS only
    (each PASS was itself bounded by its gate when it ran), except
    ``stop_settings``: they bind to the nearest PASS of the same runner kind in
    the chain (``live-arms@standing`` to ``live-arms@hoist``), because a replay
    and the live bridge stop differently by design.  ``policy_sha`` must match
    any gate entry that records a ``policy_sha256``.
    """
    spec = _spec(stage)
    if ledger.practice:
        return ["the PRACTICE ledger never unlocks a real robot"]
    if identity.get("serial") != ledger.serial:
        return [f"robot {identity.get('serial')} is not this ledger's {ledger.serial}"]
    try:
        entries = ledger.entries()
    except StageGateError as exc:
        return [str(exc)]
    when = now or now_utc()
    problems: list[str] = []
    for gate in spec.gate:
        entry = _latest(entries, gate)
        problems += _chain_problems(gate, entry, entries, identity, when, depth=0)
        if entry is not None and entry.get("verdict") == "PASS":
            problems += [f"{STAGES[gate].title}: {p}"
                         for p in settings_problems(gate, entry.get("settings") or {}, settings, stage=stage)]
            recorded = (entry.get("extra") or {}).get("policy_sha256")
            if recorded is not None and recorded != policy_sha:
                problems.append(f"{STAGES[gate].title}: passed with policy {recorded}, "
                                f"not {policy_sha}")
    ancestor = _same_kind_ancestor(stage, entries)
    if ancestor is not None:
        passed = (ancestor.get("settings") or {}).get("stop_settings")
        if "stop_settings" not in settings:
            problems.append("setting stop_settings is missing from this run")
        elif settings["stop_settings"] != passed:
            problems.append(f"{STAGES[ancestor['stage']].title}: stop_settings "
                            f"{settings['stop_settings']} are not the {passed} it passed with")
    return problems


def entry_problems(stage: str, *, ledger: Ledger, identity: Mapping[str, str],
                   now: dt.datetime | None = None) -> list[str]:
    """Why the latest entry of ``stage`` is not a valid PASS for this robot now, its chain
    included (the checks :func:`require` makes of a gate entry, without the run's settings);
    ``[]`` means it is.  For the window's stage ladder."""
    _spec(stage)
    if ledger.practice:
        return ["the PRACTICE ledger never unlocks a real robot"]
    if identity.get("serial") != ledger.serial:
        return [f"robot {identity.get('serial')} is not this ledger's {ledger.serial}"]
    try:
        entries = ledger.entries()
    except StageGateError as exc:
        return [str(exc)]
    return _chain_problems(stage, _latest(entries, stage), entries, identity, now or now_utc(),
                           depth=0)


def _same_kind_ancestor(stage: str, entries: list[dict[str, Any]]) -> dict[str, Any] | None:
    """The nearest PASS below ``stage`` in its gate chain run by the same kind of runner."""
    kind, frontier, seen = runner_kind(stage), list(STAGES[stage].gate), set()
    while frontier:
        key = frontier.pop(0)
        if key in seen:
            continue
        seen.add(key)
        if runner_kind(key) == kind:
            entry = _latest(entries, key)
            return entry if entry is not None and entry.get("verdict") == "PASS" else None
        frontier += list(STAGES[key].gate)
    return None


def _latest(entries: list[dict[str, Any]], stage: str) -> dict[str, Any] | None:
    found = None
    for entry in entries:
        if entry.get("stage") == stage:
            found = entry
    return found


def _chain_problems(stage: str, entry: Mapping[str, Any] | None, entries: list[dict[str, Any]],
                    identity: Mapping[str, str], now: dt.datetime, *, depth: int) -> list[str]:
    spec = STAGES[stage]
    title = spec.title
    if depth > len(STAGES):
        return [f"{title}: the gate chain loops"]
    if entry is None:
        return [f"{title}: no PASS for this robot"]
    if entry.get("verdict") == "REVOKED":
        return [f"{title}: revoked {entry.get('date')}: {entry.get('reason')}; pass it again"]
    if entry.get("verdict") != "PASS":
        return [f"{title}: latest entry is {entry.get('verdict')}"]
    found = []
    if entry.get("practice") is not False:
        found.append(f"{title}: a practice entry never unlocks a real robot")
    if entry.get("contract") != spec.contract:
        found.append(f"{title}: passed under contract {entry.get('contract')}, the runner is now "
                     f"{spec.contract}; pass it again")
    found += [f"{title}: {p}" for p in _identity_problems(entry.get("identity") or {}, identity)]
    found += [f"{title}: {p}" for p in _evidence_problems(entry.get("evidence"))]
    found += [f"{title}: {p}" for p in _age_problems(entry.get("date"), now)]
    stored = entry.get("prerequisites") or {}
    for gate in spec.gate:
        below = _latest(entries, gate)
        if below is None or stored.get(gate) != entry_sha256(below):
            found.append(f"{title}: its {STAGES[gate].title} entry changed since it passed; "
                         "pass it again")
        found += _chain_problems(gate, below, entries, identity, now, depth=depth + 1)
    return found


def _identity_problems(passed: Mapping[str, Any], requested: Mapping[str, str]) -> list[str]:
    found = []
    for key in IDENTITY_KEYS:
        if not requested.get(key) or requested.get(key) != passed.get(key):
            what = {"serial": "robot", "firmware": "firmware",
                    "sdk_binding_version": "SDK bindings"}[key]
            found.append(f"{what} {requested.get(key)} != the {passed.get(key)} it passed with"
                         + ("; redo S0 in full, then requalify each stage" if key != "serial"
                            else ""))
    return found


def _evidence_problems(evidence: Any) -> list[str]:
    if not isinstance(evidence, dict) or not evidence:
        return ["no evidence files"]
    found = []
    for name, item in sorted(evidence.items()):
        path = Path(str((item or {}).get("path", "")))
        if not path.is_file():
            found.append(f"evidence {name} ({path}) is missing")
        elif sha256_file(path) != (item or {}).get("sha256"):
            found.append(f"evidence {name} ({path}) changed since the PASS")
    return found


def _age_problems(date: Any, now: dt.datetime) -> list[str]:
    try:
        passed = dt.datetime.fromisoformat(str(date))
    except ValueError:
        return [f"unreadable date {date!r}"]
    if passed.tzinfo is None:
        return [f"date {date} has no time zone"]
    if now - passed > MAX_AGE:
        return [f"passed {date}, more than {MAX_AGE.days} days ago; requalify (short run + drills)"]
    return []
