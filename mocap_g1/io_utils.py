"""Shared, dependency-light I/O helpers for the offline qualification chain.

One implementation each of: file / array fingerprints, the optional NPZ
``metadata`` JSON reader, the atomic text/JSON writers, the atomic qpos-NPZ
writer, and the git-checkout provenance probes (revision + dirty flag).
Every module that previously carried a private copy (window_audit,
controller_screen, dataset, conditioning, collision_gate, contact_refine, the
qualify driver, the npz->pkl converter, and the tools/ report writers) now
delegates here so a fix lands everywhere at once.
"""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
from typing import Any, Mapping

import numpy as np


def sha256_file(path: str | Path) -> str:
    """Return a source fingerprint without modifying the file."""
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sha256_text(value: str) -> str:
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def array_sha256(values: np.ndarray) -> str:
    """Fingerprint of an in-memory array including its dtype and shape."""
    contiguous = np.ascontiguousarray(values)
    digest = hashlib.sha256()
    digest.update(str(contiguous.dtype).encode("ascii"))
    digest.update(str(contiguous.shape).encode("ascii"))
    digest.update(contiguous.tobytes())
    return digest.hexdigest()


def read_npz_metadata(path: str | Path) -> dict:
    """Best-effort read of an NPZ's optional scalar ``metadata`` JSON object.

    Returns ``{}`` when the key is absent, not a scalar, not a JSON object,
    or the archive is unreadable.  Callers that need a guarantee (e.g. the
    quaternion order) must treat ``{}`` as "unknown", never as "default".
    """
    try:
        with np.load(Path(path), allow_pickle=False) as archive:
            if "metadata" not in archive.files:
                return {}
            raw = archive["metadata"]
            if raw.shape != ():
                return {}
            value = json.loads(str(raw.item()))
            return value if isinstance(value, dict) else {}
    except (OSError, ValueError, KeyError, json.JSONDecodeError):
        return {}


def write_text_atomic(path: str | Path, text: str, *, overwrite: bool = False,
                      exists_error: type[Exception] = FileExistsError,
                      label: str = "existing report") -> None:
    """Atomically write ``text`` (UTF-8) via a same-directory temp file.

    A reader never sees a partial file, and a failed write leaves neither a
    partial target nor the temp file behind.  Unless ``overwrite`` is set, an
    existing target raises ``exists_error("refusing to overwrite {label}: ...")``.
    """
    target = Path(path)
    if not overwrite and target.exists():
        raise exists_error(f"refusing to overwrite {label}: {target}")
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", prefix=f".{target.name}.", suffix=".tmp",
            dir=target.parent, delete=False,
        ) as handle:
            temporary_name = handle.name
            handle.write(text)
        os.replace(temporary_name, target)
    finally:
        if temporary_name is not None:
            Path(temporary_name).unlink(missing_ok=True)


def write_json_atomic(path: str | Path, payload: Any, *, overwrite: bool = False,
                      exists_error: type[Exception] = FileExistsError,
                      label: str = "existing report") -> None:
    """Atomically write ``payload`` as strict JSON (NaN/Infinity rejected).

    Serialization happens before any file is touched, so a non-finite value
    raises ``ValueError`` without leaving a partial or empty report behind.
    Unless ``overwrite`` is set, an existing target raises ``exists_error``
    (message ``"refusing to overwrite {label}: {path}"``).
    """
    target = Path(path)
    if not overwrite and target.exists():
        raise exists_error(f"refusing to overwrite {label}: {target}")
    text = json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n"
    write_text_atomic(target, text, overwrite=True)


def write_qpos_npz_atomic(path: str | Path, *, qpos: np.ndarray, fps: float,
                          metadata: Mapping[str, Any],
                          exists_error: type[Exception] = FileExistsError) -> None:
    """Atomically create a compressed ``{metadata, qpos, fps}`` motion NPZ.

    The layout every qpos loader in the chain reads: ``metadata`` is a 0-d
    unicode array holding ``json.dumps(metadata, sort_keys=True)``, ``qpos``
    is stored as given, and ``fps`` is a 0-d float64.  Refuses to overwrite
    any existing file (``exists_error``); a failure removes the temp file and
    never leaves a partial target.
    """
    target = Path(path)
    if target.exists():
        raise exists_error(f"refusing to overwrite existing output: {target}")
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb", prefix=f".{target.name}.", suffix=".tmp",
            dir=target.parent, delete=False,
        ) as handle:
            temporary_name = handle.name
            np.savez_compressed(
                handle,
                metadata=np.asarray(json.dumps(metadata, sort_keys=True)),
                qpos=qpos,
                fps=np.asarray(fps, dtype=np.float64),
            )
        os.replace(temporary_name, target)
    finally:
        if temporary_name is not None:
            Path(temporary_name).unlink(missing_ok=True)


def _git(path: str | Path, *args: str) -> subprocess.CompletedProcess | None:
    try:
        return subprocess.run(
            ["git", "-C", str(path), *args],
            capture_output=True, text=True, check=False, timeout=30,
        )
    except (OSError, subprocess.SubprocessError):
        return None


def git_revision(path: str | Path) -> str | None:
    """HEAD commit of the checkout containing ``path``, or None.

    None (never an exception) when git is missing, ``path`` is not inside a
    git worktree, or the repository has no commits.
    """
    result = _git(path, "rev-parse", "HEAD")
    if result is None or result.returncode != 0:
        return None
    return result.stdout.strip() or None


def git_worktree_dirty(path: str | Path) -> bool | None:
    """True when ``git status --porcelain`` reports any change, else False.

    None means "unknown" (no git, not a checkout, or git failed) -- callers
    recording provenance must not treat that as clean.
    """
    result = _git(path, "status", "--porcelain")
    if result is None or result.returncode != 0:
        return None
    return bool(result.stdout.strip())
