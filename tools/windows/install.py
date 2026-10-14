#!/usr/bin/env python3
"""Python stage of the Windows installer (runs inside <repo>\\.venv).

tools/windows/install.ps1 prepares uv, a pinned CPython 3.13, the project venv
and Git, then hands over to this script for everything else:

1. install the project into the venv (pinned by constraints/requirements-ci.txt),
2. run the core setup doctor and the offline stream-acceptance self-test,
3. bootstrap and verify the GMR runtime workspace (MOCAP_WS).

Every step is idempotent, so a failed run is resumed by running the installer
again. Pure helpers (argv building, path validation, revision lookup) are kept
apart from execution so they can be unit-tested on any platform.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Callable, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[2]
CONSTRAINTS = "constraints/requirements-ci.txt"
EXTRAS = "runtime,operator"
# Files whose content decides what the project-install step installs; the step
# is skipped when a stamp of their hashes matches (a rerun after a later failure).
INSTALL_INPUTS = ("pyproject.toml", CONSTRAINTS)
INSTALL_STAMP = "mocap-install.stamp"
# Read by console_launcher.py, which passes it to the Setup console as MOCAP_WS.
WORKSPACE_RECORD = "runtime-workspace.txt"
MINIMUM_PYTHON = (3, 13)


class InstallError(RuntimeError):
    pass


@dataclass(frozen=True)
class Step:
    name: str
    argv: tuple[str, ...]
    # Skip when this stamp file holds `stamp_value` (set for the project install).
    stamp: Path | None = None
    stamp_value: str = ""


def default_workspace(env: Mapping[str, str] | None = None) -> Path:
    """%USERPROFILE%\\mocap-runtime (the contract's default MOCAP_WS)."""

    env = os.environ if env is None else env
    home = env.get("USERPROFILE") or env.get("HOME")
    return (Path(home) if home else Path.home()) / "mocap-runtime"


def _normcase(path: Path) -> str:
    return os.path.normcase(os.path.normpath(str(path)))


def _is_within(path: Path, parent: Path) -> bool:
    child, root = _normcase(path), _normcase(parent)
    return child == root or child.startswith(root.rstrip("\\/") + os.sep)


def validate_workspace(value: str | Path, repo_root: Path = REPO_ROOT) -> Path:
    """Return an absolute, normalised runtime workspace or raise InstallError.

    Same rules as tools/setup/install.sh and bootstrap_g1_workspace.py: an
    absolute path, at least three levels below the drive root, outside the
    checkout, not containing the checkout, and without symlinked components.
    """

    text = str(value).strip()
    if not text:
        raise InstallError("the runtime workspace path is empty")
    path = Path(text).expanduser()
    if not path.is_absolute():
        raise InstallError(f"the runtime workspace must be an absolute path: {text}")
    path = Path(os.path.normpath(str(path)))
    if path == Path(path.anchor) or len(path.parts) < 4:
        raise InstallError(f"refusing broad workspace path: {path}")
    if _is_within(path, repo_root):
        raise InstallError("the runtime workspace must be outside this source checkout")
    if _is_within(repo_root, path):
        raise InstallError("the runtime workspace must not contain the source checkout")
    candidate = Path(path.anchor)
    for part in path.parts[1:]:
        candidate = candidate / part
        if candidate.is_symlink() or _is_junction(candidate):
            raise InstallError(f"the runtime workspace contains a link: {candidate}")
    return path


def _is_junction(path: Path) -> bool:
    is_junction = getattr(os.path, "isjunction", None)  # Python >= 3.12
    return bool(is_junction and is_junction(path))


def install_stamp_value(repo_root: Path = REPO_ROOT) -> str:
    digest = hashlib.sha256()
    for name in INSTALL_INPUTS:
        digest.update(name.encode())
        digest.update(b"\0")
        digest.update((repo_root / name).read_bytes())
    return digest.hexdigest()


def build_steps(python: str, workspace: Path, repo_root: Path = REPO_ROOT,
                *, venv_dir: Path | None = None, with_runtime: bool = True) -> list[Step]:
    """Return the ordered install steps as argv lists (never a shell string)."""

    root = str(repo_root)

    def tool(*parts: str) -> str:
        return str(repo_root.joinpath(*parts))

    venv_dir = venv_dir if venv_dir is not None else repo_root / ".venv"
    steps = [
        Step(
            "project packages",
            (python, "-m", "pip", "install", "--disable-pip-version-check",
             "-c", tool(*CONSTRAINTS.split("/")), "-e", f"{root}[{EXTRAS}]"),
            stamp=venv_dir / INSTALL_STAMP,
            stamp_value=install_stamp_value(repo_root),
        ),
        Step("core setup doctor",
             (python, tool("tools", "checks", "setup_doctor.py"), "core", "--with-operator")),
        Step("stream acceptance self-test",
             (python, tool("tools", "xsens", "stream_acceptance.py"),
              "--self-test", "--require-joint-angles")),
    ]
    if with_runtime:
        bootstrap = tool("tools", "bootstrap_g1_workspace.py")
        steps += [
            Step("GMR runtime workspace", (python, bootstrap, "bootstrap", "--workspace", str(workspace))),
            Step("runtime workspace check", (python, bootstrap, "check", "--workspace", str(workspace))),
        ]
    return steps


def project_revision(repo_root: Path = REPO_ROOT,
                     run: Callable[..., subprocess.CompletedProcess] = subprocess.run) -> str | None:
    """Commit of the checkout: `git rev-parse HEAD`, else .git_archival.txt (ZIP download)."""

    if (repo_root / ".git").exists():
        try:
            result = run(["git", "-C", str(repo_root), "rev-parse", "HEAD"],
                         check=False, capture_output=True, text=True, timeout=30)
        except (OSError, subprocess.SubprocessError):
            result = None
        if result is not None and result.returncode == 0 and result.stdout.strip():
            return result.stdout.strip()
    return archival_revision(repo_root / ".git_archival.txt")


def archival_revision(path: Path) -> str | None:
    """Return the `node:` commit from an expanded .git_archival.txt, else None."""

    try:
        text = path.read_text(encoding="utf-8")
    except OSError:
        return None
    for line in text.splitlines():
        key, _, value = line.partition(":")
        value = value.strip()
        if key.strip() == "node" and len(value) == 40 and all(c in "0123456789abcdef" for c in value):
            return value
    return None


def format_argv(argv: Sequence[str]) -> str:
    """A copy-pasteable rendering for the log (cmd.exe quoting on Windows)."""

    if sys.platform == "win32":
        return subprocess.list2cmdline(list(argv))
    import shlex

    return shlex.join(argv)


def _stamp_matches(step: Step) -> bool:
    if step.stamp is None:
        return False
    try:
        return step.stamp.read_text(encoding="ascii").strip() == step.stamp_value
    except OSError:
        return False


def run_steps(steps: Sequence[Step], *, dry_run: bool = False,
              run: Callable[..., subprocess.CompletedProcess] = subprocess.run,
              out: Callable[[str], None] = print) -> None:
    total = len(steps)
    for index, step in enumerate(steps, 1):
        out(f"\n== [{index}/{total}] {step.name} ==")
        if _stamp_matches(step):
            out("Already done for this pyproject.toml and constraint set; skipping.")
            continue
        out("+ " + format_argv(step.argv))
        if dry_run:
            continue
        started = time.monotonic()
        try:
            result = run(list(step.argv), check=False, cwd=str(REPO_ROOT))
        except OSError as exc:
            raise InstallError(f"{step.name}: could not start {step.argv[0]}: {exc}") from exc
        if result.returncode:
            raise InstallError(f"{step.name} failed (exit {result.returncode})")
        if step.stamp is not None:
            step.stamp.write_text(step.stamp_value + "\n", encoding="ascii")
        out(f"OK: {step.name} ({time.monotonic() - started:.0f} s)")


def record_workspace(workspace: Path, env: Mapping[str, str] | None = None) -> Path | None:
    """Remember the runtime workspace for the Setup console (Windows tools dir)."""

    env = os.environ if env is None else env
    base = env.get("LOCALAPPDATA")
    if not base:
        return None
    record = Path(base) / "mocap-humanoid" / WORKSPACE_RECORD
    record.parent.mkdir(parents=True, exist_ok=True)
    record.write_text(str(workspace) + "\n", encoding="utf-8")
    return record


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--workspace", default="",
                        help="GMR runtime workspace (default: %%USERPROFILE%%\\mocap-runtime)")
    parser.add_argument("--skip-runtime", action="store_true",
                        help="install only the project venv (no GMR runtime workspace)")
    parser.add_argument("--dry-run", action="store_true",
                        help="print the steps without executing anything")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    os.environ.setdefault("PYTHONUNBUFFERED", "1")
    try:
        if sys.version_info < MINIMUM_PYTHON:
            raise InstallError(
                "the project venv must use Python >= 3.13 on Windows; rerun Install-Windows.cmd"
            )
        workspace = validate_workspace(args.workspace or default_workspace())
        revision = project_revision()
        print(f"Project checkout: {REPO_ROOT}")
        print(f"Project revision: {revision or 'unknown (no .git and no expanded .git_archival.txt)'}")
        print(f"Runtime workspace (MOCAP_WS): {workspace}")
        steps = build_steps(sys.executable, workspace, with_runtime=not args.skip_runtime)
        run_steps(steps, dry_run=args.dry_run)
        if not args.dry_run and not args.skip_runtime:
            record_workspace(workspace)
    except InstallError as exc:
        print(f"\nFAIL: {exc}. Fix the error above and run the installer again; "
              "finished steps are reused.", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("\nFAIL: interrupted. Run the installer again to resume.", file=sys.stderr)
        return 130
    if args.dry_run:
        print("\nDRY RUN: NOT VERIFIED; nothing was installed or checked.")
    else:
        print("\nPASS: project environment and runtime workspace are ready.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
