"""Thread-safe append-only audit storage with recursive secret redaction.

Tamper evidence: every record written by this version carries
``prev_sha256``, the SHA-256 of the previous physical line of the file exactly
as written (UTF-8 bytes, no trailing newline), or :data:`GENESIS_SHA256` for
the first line of a new file.  Records are serialized canonically (sorted
keys, compact separators) after redaction, so the hash covers what is on disk.
The chain is verified when the log is loaded and again on demand
(:meth:`AuditLog.verify_chain`, used by the diagnostics export).  A break is
reported, never repaired: the console keeps running, ``integrity_summary``
flags it in the status payload, and an ``audit``/``integrity_check`` record
is appended so the finding is itself part of the chain.  Files written before
chaining load unchanged; the chain then starts at their last line and an
``audit``/``chain_start`` record notes how many earlier records are unchained.
"""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import threading
import time
import logging
from typing import Any, BinaryIO, Iterator, Mapping


_SENSITIVE_FRAGMENTS = (
    "authorization",
    "cookie",
    "credential",
    "password",
    "private",
    "raw_mocap",
    "mocap_payload",
    "body_trajectory",
    "secret",
    "token",
    "api_key",
    "signing_key",
)
logger = logging.getLogger(__name__)

_MAX_QUARANTINE_KEYS = 10_000
_MAX_REPORTED_CHAIN_BREAKS = 20
GENESIS_SHA256 = "0" * 64
_MAX_AUDIT_LINE_BYTES = 1024 * 1024
_LINE_READ_CHUNK_BYTES = 64 * 1024


def _without_trailing_newline(raw_line: bytes) -> bytes:
    return raw_line[:-1] if raw_line.endswith(b"\n") else raw_line


def _consume_line_summary(handle: BinaryIO, first_chunk: bytes) -> tuple[str, int]:
    """Hash one logical line without retaining it in memory."""

    digest = hashlib.sha256()
    content_bytes = 0
    chunk = first_chunk
    while chunk:
        content = _without_trailing_newline(chunk)
        digest.update(content)
        content_bytes += len(content)
        if chunk.endswith(b"\n"):
            break
        chunk = handle.readline(_LINE_READ_CHUNK_BYTES)
    return digest.hexdigest(), content_bytes


def _parse_json_line(raw_line: bytes) -> Any:
    try:
        return json.loads(raw_line.decode("utf-8"))
    except (json.JSONDecodeError, RecursionError, UnicodeDecodeError, ValueError):
        return None


def _iter_audit_lines(handle: BinaryIO) -> Iterator[tuple[int, str, Any]]:
    """Yield ``(line_number, content_sha256, parsed_or_None)`` per non-empty line."""

    line_number = 0
    while True:
        raw_line = handle.readline(_MAX_AUDIT_LINE_BYTES + 2)
        if not raw_line:
            return
        line_number += 1
        content = _without_trailing_newline(raw_line)
        if len(content) > _MAX_AUDIT_LINE_BYTES:
            content_hash, _ = _consume_line_summary(handle, raw_line)
            yield line_number, content_hash, None
            continue
        if not content.strip():
            continue
        yield line_number, hashlib.sha256(content).hexdigest(), _parse_json_line(raw_line)


def _fsync_directory(path: Path) -> None:
    """Persist a new directory entry; best effort where unsupported."""

    try:
        descriptor = os.open(path, os.O_RDONLY)
    except OSError:
        return
    try:
        os.fsync(descriptor)
    except OSError:
        pass
    finally:
        os.close(descriptor)


class _ChainVerifier:
    """Check ``prev_sha256`` links over the physical lines of an audit file."""

    def __init__(self) -> None:
        self.previous_line_sha256: str | None = None
        self.chained_records = 0
        self.unchained_records = 0
        self.unreadable_lines = 0
        self.break_count = 0
        self.breaks: list[dict[str, Any]] = []

    def feed(
        self, line_number: int, line_sha256: str, record: Any
    ) -> None:
        if not isinstance(record, dict):
            # Unparseable lines are quarantined by the loader.  They stay part
            # of the physical chain: the record after a damaged line names the
            # hash of the original line, so tampering shows up as a break.
            self.unreadable_lines += 1
        elif "prev_sha256" in record:
            expected = (
                GENESIS_SHA256
                if self.previous_line_sha256 is None
                else self.previous_line_sha256
            )
            if record["prev_sha256"] != expected:
                self._break(line_number, record, "prev_sha256 does not match the previous line")
            self.chained_records += 1
        elif self.chained_records:
            self._break(line_number, record, "unchained record after the chain started")
        else:
            self.unchained_records += 1
        self.previous_line_sha256 = line_sha256

    def _break(self, line_number: int, record: dict, reason: str) -> None:
        self.break_count += 1
        if len(self.breaks) < _MAX_REPORTED_CHAIN_BREAKS:
            sequence = record.get("sequence")
            self.breaks.append(
                {
                    "line_number": int(line_number),
                    "sequence": sequence if isinstance(sequence, int) else None,
                    "reason": reason,
                }
            )

    def report(self) -> dict[str, Any]:
        return {
            "chain_verified": self.break_count == 0,
            "chain_breaks": self.break_count,
            "breaks": list(self.breaks),
            "chained_records": self.chained_records,
            "unchained_records": self.unchained_records,
            "unreadable_lines": self.unreadable_lines,
        }


def redact(value: Any) -> Any:
    """Return a recursively redacted copy suitable for durable logging."""

    if isinstance(value, Mapping):
        result: dict[str, Any] = {}
        for raw_key, item in value.items():
            key = str(raw_key)
            normalized = key.lower().replace("-", "_")
            if any(fragment in normalized for fragment in _SENSITIVE_FRAGMENTS):
                result[key] = "[REDACTED]"
            else:
                result[key] = redact(item)
        return result
    if isinstance(value, (list, tuple)):
        return [redact(item) for item in value]
    return deepcopy(value)


class AuditLog:
    """Append audit events to JSONL and retain a bounded in-memory view."""

    def __init__(self, path: str | Path | None = None, *, memory_limit: int = 2000) -> None:
        if memory_limit < 1:
            raise ValueError("memory_limit must be positive")
        self.path = Path(path) if path is not None else None
        self.memory_limit = int(memory_limit)
        self._lock = threading.RLock()
        self._records: list[dict[str, Any]] = []
        self._sequence = 0
        self._quarantine_keys: set[tuple[str, int, str]] = set()
        # SHA-256 of the last physical line in the file (None: empty file).
        self._last_line_sha256: str | None = None
        # A torn final line (no newline) must not swallow the next record.
        self._needs_leading_newline = False
        self._pending_chain_start: dict[str, Any] | None = None
        self._integrity: dict[str, Any] = {
            "persistent": self.path is not None,
            "chain_verified": True,
            "chain_breaks": 0,
            "breaks": [],
            "chained_records": 0,
            "unchained_records": 0,
            "unreadable_lines": 0,
            "checked_at": time.time(),
        }
        if self.path is not None:
            self._load_quarantine_index()
            self._load_existing()
            self._record_load_findings()

    @property
    def sequence(self) -> int:
        with self._lock:
            return self._sequence

    def append(
        self,
        *,
        category: str,
        action: str,
        outcome: str,
        state: str,
        actor: str = "system",
        details: Mapping[str, Any] | None = None,
        timestamp: float | None = None,
        durable: bool = True,
    ) -> dict[str, Any]:
        """Append one record.

        ``durable`` fsyncs the file before returning; best-effort safety
        records pass ``False`` and are only flushed to the OS.
        """
        with self._lock:
            if self._pending_chain_start is not None:
                chain_start = self._pending_chain_start
                self._pending_chain_start = None
                try:
                    self.append(
                        category="audit",
                        action="chain_start",
                        outcome="accepted",
                        state=str(state),
                        actor="system",
                        details=chain_start,
                        durable=durable,
                    )
                except BaseException:
                    self._pending_chain_start = chain_start
                    raise
            self._sequence += 1
            record = {
                "sequence": self._sequence,
                "timestamp": float(time.time() if timestamp is None else timestamp),
                "category": str(category),
                "action": str(action),
                "outcome": str(outcome),
                "state": str(state),
                "actor": str(actor),
                "details": redact(details or {}),
            }
            if self.path is not None:
                record["prev_sha256"] = (
                    self._last_line_sha256
                    if self._last_line_sha256 is not None
                    else GENESIS_SHA256
                )
                line = json.dumps(record, sort_keys=True, separators=(",", ":"))
                self._write_line(line, durable=durable)
                self._last_line_sha256 = hashlib.sha256(line.encode("utf-8")).hexdigest()
            self._records.append(record)
            if len(self._records) > self.memory_limit:
                del self._records[: len(self._records) - self.memory_limit]
            return deepcopy(record)

    def _write_line(self, line: str, *, durable: bool) -> None:
        assert self.path is not None
        self.path.parent.mkdir(parents=True, exist_ok=True)
        created = not self.path.exists()
        # The process lock and O_APPEND descriptor preserve record ordering.
        # The primary log is never rewritten or truncated.
        descriptor = os.open(
            self.path,
            os.O_WRONLY | os.O_CREAT | os.O_APPEND,
            0o600,
        )
        try:
            os.chmod(self.path, 0o600)
            with os.fdopen(descriptor, "a", encoding="utf-8") as handle:
                descriptor = -1
                if self._needs_leading_newline:
                    handle.write("\n")
                handle.write(line)
                handle.write("\n")
                handle.flush()
                self._needs_leading_newline = False
                if durable:
                    os.fsync(handle.fileno())
        finally:
            if descriptor >= 0:
                os.close(descriptor)
        if created and durable:
            _fsync_directory(self.path.parent)

    def integrity_summary(self) -> dict[str, Any]:
        """Result of the most recent chain verification (load or on demand)."""

        with self._lock:
            summary = deepcopy(self._integrity)
        summary.pop("breaks", None)
        return summary

    def verify_chain(self) -> dict[str, Any]:
        """Re-read the file and verify the hash chain without loading records.

        Also checks that the file still ends with the line this process last
        wrote, which catches truncation or foreign appends while running.
        """

        with self._lock:
            if self.path is None:
                return deepcopy(self._integrity)
            verifier = _ChainVerifier()
            try:
                with self.path.open("rb") as handle:
                    for line_number, line_sha256, record in _iter_audit_lines(handle):
                        verifier.feed(line_number, line_sha256, record)
            except FileNotFoundError:
                pass
            report = verifier.report()
            report["persistent"] = True
            report["checked_at"] = time.time()
            if verifier.previous_line_sha256 != self._last_line_sha256:
                report["chain_verified"] = False
                report["tail_matches_last_write"] = False
            else:
                report["tail_matches_last_write"] = True
            self._integrity = deepcopy(report)
            return report

    def read(
        self, *, after_sequence: int = 0, limit: int = 200, newest: bool = False
    ) -> list[dict[str, Any]]:
        """Records after ``after_sequence`` in sequence order.

        By default the oldest ``limit`` matches are returned, for paging
        forward. ``newest`` returns the most recent ``limit`` instead, which is
        what a bootstrap view or a diagnostics bundle needs.
        """
        if limit < 1 or limit > 5000:
            raise ValueError("limit must be between 1 and 5000")
        with self._lock:
            matching = [
                record for record in self._records if record["sequence"] > after_sequence
            ]
            selected = matching[-limit:] if newest else matching[:limit]
            return deepcopy(selected)

    def _load_existing(self) -> None:
        assert self.path is not None
        if not self.path.exists():
            return
        os.chmod(self.path, 0o600)
        records: list[dict[str, Any]] = []
        verifier = _ChainVerifier()
        last_raw_line = b""
        with self.path.open("rb") as handle:
            line_number = 0
            while True:
                raw_line = handle.readline(_MAX_AUDIT_LINE_BYTES + 2)
                if not raw_line:
                    break
                line_number += 1
                content = _without_trailing_newline(raw_line)
                if len(content) > _MAX_AUDIT_LINE_BYTES:
                    content_hash, content_bytes = _consume_line_summary(
                        handle, raw_line
                    )
                    verifier.feed(line_number, content_hash, None)
                    last_raw_line = b"\n"
                    self._quarantine_summary(
                        line_number=line_number,
                        content_hash=content_hash,
                        content_bytes=content_bytes,
                        error_type="AuditLineTooLong",
                    )
                    continue
                if not content.strip():
                    continue
                last_raw_line = raw_line
                verifier.feed(
                    line_number,
                    hashlib.sha256(content).hexdigest(),
                    _parse_json_line(raw_line),
                )
                try:
                    line = raw_line.decode("utf-8")
                    record = json.loads(line)
                    sequence = record["sequence"]
                    if isinstance(sequence, bool) or not isinstance(sequence, int):
                        raise ValueError("audit sequence must be a JSON integer")
                    if sequence <= self._sequence:
                        raise ValueError("audit sequences must be positive and increasing")
                    safe_record = redact(record)
                except (
                    json.JSONDecodeError,
                    KeyError,
                    RecursionError,
                    TypeError,
                    UnicodeDecodeError,
                    ValueError,
                ) as exc:
                    self._quarantine_line(line_number, raw_line, exc)
                    continue
                # Old append-only logs may predate the current redaction list.
                # Keep the source file intact for audit integrity, but never
                # expose a legacy secret through the in-memory/API view.
                records.append(safe_record)
                self._sequence = max(self._sequence, sequence)
        self._records = records[-self.memory_limit :]
        self._last_line_sha256 = verifier.previous_line_sha256
        self._needs_leading_newline = bool(last_raw_line) and not last_raw_line.endswith(b"\n")
        report = verifier.report()
        report["persistent"] = True
        report["checked_at"] = time.time()
        self._integrity = report

    def _record_load_findings(self) -> None:
        """Append chain findings from loading so they are themselves chained."""

        report = self._integrity
        try:
            if report["chain_breaks"]:
                logger.error(
                    "Audit log %s failed hash-chain verification (%d break(s))",
                    self.path,
                    report["chain_breaks"],
                )
                self.append(
                    category="audit",
                    action="integrity_check",
                    outcome="rejected",
                    state="startup",
                    details={
                        "reason": "Audit hash chain is broken",
                        "chain_breaks": report["chain_breaks"],
                        "breaks": report["breaks"],
                    },
                )
            elif report["unchained_records"] and not report["chained_records"]:
                # Written just before the first chained record, so loading a
                # legacy file never modifies it by itself.
                self._pending_chain_start = {
                    "reason": "Existing records predate audit hash chaining",
                    "unchained_records": report["unchained_records"],
                }
        except OSError:
            logger.exception("Could not record audit-chain findings")

    def _quarantine_line(
        self, line_number: int, raw_line: bytes, exc: Exception
    ) -> None:
        content = _without_trailing_newline(raw_line)
        self._quarantine_summary(
            line_number=line_number,
            content_hash=hashlib.sha256(content).hexdigest(),
            content_bytes=len(content),
            error_type=type(exc).__name__,
        )

    def _quarantine_summary(
        self,
        *,
        line_number: int,
        content_hash: str,
        content_bytes: int,
        error_type: str,
    ) -> None:
        assert self.path is not None
        quarantine_path = self.path.with_suffix(".corrupt.log")
        key = (str(self.path), int(line_number), content_hash)
        if key in self._quarantine_keys:
            return
        # Corrupt source lines remain in the primary append-only log. Bound
        # the auxiliary deduplication index so a damaged or adversarial file
        # cannot consume unbounded startup memory or grow its sidecar forever.
        if len(self._quarantine_keys) >= _MAX_QUARANTINE_KEYS:
            return
        quarantine_path.parent.mkdir(parents=True, exist_ok=True)
        descriptor = os.open(
            quarantine_path,
            os.O_WRONLY | os.O_CREAT | os.O_APPEND,
            0o600,
        )
        try:
            os.chmod(quarantine_path, 0o600)
            with os.fdopen(descriptor, "a", encoding="utf-8") as quarantine:
                descriptor = -1
                quarantine.write(
                    json.dumps(
                        {
                            "quarantined_from": str(self.path),
                            "line_number": line_number,
                            "error_type": str(error_type),
                            "content_sha256": content_hash,
                            "content_bytes": int(content_bytes),
                        },
                        sort_keys=True,
                    )
                    + "\n"
                )
        finally:
            if descriptor >= 0:
                os.close(descriptor)
        self._quarantine_keys.add(key)

    def _load_quarantine_index(self) -> None:
        assert self.path is not None
        quarantine_path = self.path.with_suffix(".corrupt.log")
        if not quarantine_path.is_file():
            return
        try:
            with quarantine_path.open("rb") as handle:
                line_number = 0
                while line_number < _MAX_QUARANTINE_KEYS:
                    raw_line = handle.readline(_MAX_AUDIT_LINE_BYTES + 2)
                    if not raw_line:
                        break
                    line_number += 1
                    content = _without_trailing_newline(raw_line)
                    if len(content) > _MAX_AUDIT_LINE_BYTES:
                        _consume_line_summary(handle, raw_line)
                        continue
                    try:
                        line = raw_line.decode("utf-8")
                        record = json.loads(line)
                        source = record["quarantined_from"]
                        indexed_line = record["line_number"]
                        content_hash = record["content_sha256"]
                        if not isinstance(source, str) or source != str(self.path):
                            raise ValueError("quarantine source does not match audit log")
                        if (
                            isinstance(indexed_line, bool)
                            or not isinstance(indexed_line, int)
                            or indexed_line < 1
                        ):
                            raise ValueError("quarantine line number must be positive")
                        if (
                            not isinstance(content_hash, str)
                            or len(content_hash) != 64
                            or any(
                                character not in "0123456789abcdef"
                                for character in content_hash
                            )
                        ):
                            raise ValueError("quarantine content hash is invalid")
                        key = (source, indexed_line, content_hash)
                    except (
                        json.JSONDecodeError,
                        KeyError,
                        RecursionError,
                        TypeError,
                        UnicodeDecodeError,
                        ValueError,
                    ):
                        continue
                    self._quarantine_keys.add(key)
                    if len(self._quarantine_keys) >= _MAX_QUARANTINE_KEYS:
                        break
        except OSError:
            # Failure to inspect an old quarantine file must not make the
            # primary append-only audit log unavailable.
            return
