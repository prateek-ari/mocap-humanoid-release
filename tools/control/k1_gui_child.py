#!/usr/bin/env python3
"""Child side of the K1 test GUI (tools/control/k1_commission_gui.py).

Runs ``k1_commission.main`` or ``k1_arm_bridge.main`` unchanged, in this process,
for the GUI.  The GUI starts it with stdout and stderr on a log file (never a
pipe: a dead GUI must not break a ``print`` on the release path) and stdin on a
pipe.  What this wrapper adds:

- the parent-death signal: if the GUI dies, this process gets SIGINT, the same
  as Ctrl-C, so the tool's own stop-and-release path runs;
- structured prompts: every question becomes one ``@@K1GUI`` event line with an
  id, and only a stdin line ``<id><TAB><answer>`` answers it;
- ``session`` and ``attempt`` events, so the GUI binds each run to its evidence;
- on the fake robot only: ``rpc`` events, and test hooks (``K1_GUI_TEST_*``).
"""

from __future__ import annotations

import argparse
import ctypes
import itertools
import json
import os
from pathlib import Path
import signal
import sys
import time
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

MARK = "@@K1GUI "
PR_SET_PDEATHSIG = 1
TEST_ENV = ("K1_GUI_TEST_PAUSE", "K1_GUI_TEST_FAIL")
_ids = itertools.count(1)
_bridges: list[Any] = []


def emit(event: str, **fields: Any) -> None:
    print(MARK + json.dumps({"event": event, **fields}), flush=True)


def prompt_kind(question: str) -> str:
    if "Type yes" in question:
        return "confirm"
    if "[y/n]" in question:
        return "yesno"
    return "text"


def gui_ask(question: str, detail: str = "") -> str:
    """One question, answered only by the stdin line carrying its id."""
    pid = next(_ids)
    emit("prompt", id=pid, kind=prompt_kind(question), text=question, detail=detail)
    while True:
        line = sys.stdin.readline()
        if not line:
            raise EOFError("the GUI closed while a question was open")
        got, _, answer = line.rstrip("\n").partition("\t")
        if got == str(pid):  # a stale or duplicate reply has another id
            print(f"{question}{answer}", flush=True)
            return answer


def set_parent_death_signal(parent_pid: int) -> None:
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(PR_SET_PDEATHSIG, int(signal.SIGINT), 0, 0, 0) != 0:
        raise OSError(ctypes.get_errno(), "prctl(PR_SET_PDEATHSIG) failed")
    if os.getppid() != parent_pid:  # the GUI died before the signal was registered
        os.kill(os.getpid(), signal.SIGINT)


def session_transport(argv: list[str]) -> str | None:
    if "--session" not in argv:
        return None
    index = argv.index("--session") + 1
    try:
        meta = json.loads((Path(argv[index]) / "session.json").read_text(encoding="utf-8"))
    except (IndexError, OSError, ValueError):
        return None
    return str(meta.get("transport"))


def install_hooks(tool_name: str, argv: list[str]) -> None:
    from mocap_g1 import k1_bridge

    original_init = k1_bridge.K1ArmBridge.__init__

    def keep(self: Any, *args: Any, **kwargs: Any) -> None:
        original_init(self, *args, **kwargs)
        _bridges.append(self)

    k1_bridge.K1ArmBridge.__init__ = keep  # type: ignore[method-assign]

    fake = k1_bridge.FakeK1Transport  # the practice robot only; the real transport is untouched
    original_rpc, original_publish = fake.upper_body_control, fake.publish
    pause = os.environ.get("K1_GUI_TEST_PAUSE", "")
    paused = [False]

    def barrier() -> None:
        """Test-only: wait for the test's ``go`` file (bounded)."""
        paused[0] = True
        folder = Path(os.environ["K1_GUI_TEST_DIR"])
        (folder / "paused").write_text(str(os.getpid()), encoding="utf-8")
        end = time.monotonic() + 30.0
        while not (folder / "go").exists():
            if time.monotonic() > end:
                raise RuntimeError("test barrier: no go file within 30 s")
            time.sleep(0.05)

    def rpc(self: Any, enable: bool) -> None:
        if pause == "takeover" and enable and not paused[0]:
            barrier()
        try:
            original_rpc(self, enable)
        except BaseException:
            emit("rpc", enable=bool(enable), ok=False)
            raise
        emit("rpc", enable=bool(enable), ok=True)

    def publish(self: Any, frame: Any) -> None:
        if pause == "motion" and not paused[0]:
            barrier()
        original_publish(self, frame)

    fake.upper_body_control = rpc  # type: ignore[method-assign]
    fake.publish = publish  # type: ignore[method-assign]

    if tool_name == "commission":
        from tools.control import k1_commission as tool

        def ask(question: str) -> str:
            return gui_ask(question)

        def confirm(text: str) -> bool:
            print(text, flush=True)
            reply = gui_ask("Type yes to continue (anything else stops): ", detail=text)
            return reply.strip().lower() == "yes"

        tool.ask = ask
        tool.confirm = confirm
        create, start = tool.Session.create.__func__, tool.Session.start  # type: ignore[attr-defined]

        def session_create(cls: Any, root: Path, meta: Any) -> Any:
            session = create(cls, root, meta)
            emit("session", path=str(Path(session.path).resolve()))
            return session

        def session_start(self: Any, key: str, settings: Any, uses: Any) -> Any:
            attempt = start(self, key, settings, uses)
            emit("attempt", id=attempt.id, step=key, path=str(attempt.path))
            return attempt

        tool.Session.create = classmethod(session_create)  # type: ignore[assignment]
        tool.Session.start = session_start  # type: ignore[method-assign]
        if os.environ.get("K1_GUI_TEST_FAIL") == "probe":
            def failing_probe(*_args: Any, **_kwargs: Any) -> Any:
                raise k1_bridge.K1BridgeError("test: probe failed before a session")

            tool.probe_robot = failing_probe


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--parent-pid", type=int, required=True)
    parser.add_argument("--tool", choices=("commission", "replay"), required=True)
    parser.add_argument("args", nargs=argparse.REMAINDER)
    options = parser.parse_args(argv)
    args = options.args[1:] if options.args[:1] == ["--"] else options.args
    try:
        try:
            set_parent_death_signal(options.parent_pid)
        except OSError as exc:  # without it a dead GUI could not stop this run
            print(f"REFUSED: {exc}", file=sys.stderr)
            return 2
        if any(os.environ.get(name) for name in TEST_ENV) and (
                "booster" in args or session_transport(args) not in (None, "fake")):
            print("REFUSED: K1_GUI_TEST_* hooks run on the practice robot only", file=sys.stderr)
            return 2
        install_hooks(options.tool, args)
        if options.tool == "commission":
            from tools.control import k1_commission as tool
        else:
            from tools.control import k1_arm_bridge as tool  # type: ignore[no-redef]
        code = int(tool.main(args))
    except KeyboardInterrupt:
        print("Stopped (Ctrl-C).", flush=True)
        code = 130
    except EOFError as exc:
        print(f"Stopped: {exc}", flush=True)
        code = 3
    finally:
        if _bridges:
            emit("exit", bridge_state=_bridges[-1].state, fault=_bridges[-1].fault)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
