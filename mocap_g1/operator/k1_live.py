"""Live K1 arms and head from the Setup console (step 13): check, Go, status, STOP.

The console runs the same live bridge the K1 test window runs, through the same child
wrapper (``tools/control/k1_gui_child.py``): its ``@@K1GUI`` event lines carry the
auto-check results (``checks``), the live status (``live``), the session and the attempt.
The wrapper gets the console's pid, so if the console dies the bridge gets SIGINT and runs
its own stop-and-release path. STOP sends SIGINT too (the same as Ctrl-C in a terminal).

``check`` runs the two read-only auto-checks and answers the Go question with "no": nothing
moves. ``go`` passes ``--go``: after the two auto-checks the arms take over.
On a ``go`` the robot's 500 Hz joint data is recorded (``k1_flight_recorder.py``, read-only)
into the run's session folder, and stopped when the run ends.
"""

from __future__ import annotations

from collections import deque
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import threading
import time
from typing import Any, Callable

MARK = "@@K1GUI "
RUN_LIMITS_S = (60, 120, 300, 600)
#: ROS 2 / DDS discovery and user ports (domain 0 and the next few): another program here
#: (Isaac Sim, the ros2 CLI daemon, Booster Studio) can crash the K1's software (2026-11-16).
DDS_PORTS = range(7400, 7500)
_RESULT = re.compile(r"attempt \d+: (PASS|FAIL)\b(.*)")
_SS_USER = re.compile(r'\("([^"]+)",pid=(\d+)')


def other_dds_programs(own_pids: set[int]) -> list[dict[str, Any]]:
    """Programs other than ours that hold a UDP port in 7400-7499 (``ss``; Linux only)."""
    try:
        out = subprocess.run(["ss", "-Huanp"], capture_output=True, text=True, timeout=5,
                             check=False).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    found: dict[int, str] = {}
    for line in out.splitlines():
        fields = line.split()
        if len(fields) < 4:
            continue
        port = fields[3].rsplit(":", 1)[-1]
        if not port.isdigit() or int(port) not in DDS_PORTS:
            continue
        for name, pid in _SS_USER.findall(line):
            if int(pid) not in own_pids:
                found[int(pid)] = name
    return [{"name": name, "pid": pid} for pid, name in sorted(found.items())]


def descendants(pid: int) -> set[int]:
    """``pid`` and every process below it (from /proc; empty off Linux)."""
    children: dict[int, list[int]] = {}
    try:
        entries = os.listdir("/proc")
    except OSError:
        return {pid}
    for entry in entries:
        if not entry.isdigit():
            continue
        try:
            stat = Path("/proc", entry, "stat").read_text()
        except OSError:
            continue
        parent = int(stat.rsplit(")", 1)[1].split()[1])
        children.setdefault(parent, []).append(int(entry))
    out, todo = set(), [pid]
    while todo:
        current = todo.pop()
        if current not in out:
            out.add(current)
            todo.extend(children.get(current, []))
    return out


class K1LiveRunner:
    """One check or run at a time; thread-safe. ``popen`` is injectable for tests."""

    def __init__(self, repo_root: Path, log_dir: Path, *,
                 popen: Callable[..., Any] = subprocess.Popen,
                 clock: Callable[[], float] = time.time,
                 dds_check: Callable[[set[int]], list[dict[str, Any]]] = other_dds_programs) -> None:
        self.repo_root = Path(repo_root)
        self.log_dir = Path(log_dir)
        self._popen = popen
        self._clock = clock
        self._dds_check = dds_check
        self._lock = threading.Lock()
        self._run: dict[str, Any] | None = None
        #: ``fake`` = the practice robot (rehearsal without a robot): ``bash k1-demo.sh --practice``.
        self.transport = "fake" if os.environ.get("MOCAP_K1_TRANSPORT") == "fake" else "booster"

    # -- public ------------------------------------------------------------------------------

    def start(self, action: str, *, net: str, operator: str, head: bool, time_limit_s: int,
              model: Path) -> dict[str, Any]:
        """Start a ``check`` or a ``go``; refuses while one runs."""
        if action not in ("check", "go"):
            raise ValueError("action must be check or go")
        if time_limit_s not in RUN_LIMITS_S:
            raise ValueError(f"time_limit_s must be one of {RUN_LIMITS_S}")
        if not re.fullmatch(r"[0-9A-Za-z.:_-]{1,64}", net or ""):
            raise ValueError("PC interface: an IPv4 address such as 192.168.10.10, or an interface name")
        operator = " ".join((operator or "").split())[:60] or "operator"
        with self._lock:
            if self._run is not None and self._run["process"].poll() is None:
                raise RuntimeError("A robot check or run is already going. Wait for it, or press STOP.")
            python = self.repo_root / ".venv" / "bin" / "python"
            child = self.repo_root / "tools" / "control" / "k1_gui_child.py"
            argv = [str(python), "-u", str(child), "--parent-pid", str(os.getpid()), "--tool", "live",
                    "--", "--transport", self.transport, "--net", net, "--model", str(model),
                    "--operator", operator, "--support", "hoist",
                    "--time-limit", str(time_limit_s)]
            if not head:
                argv.append("--no-head")
            if action == "go":
                argv.append("--go")
            self.log_dir.mkdir(parents=True, exist_ok=True)
            log_path = self.log_dir / f"{time.strftime('%Y%m%d-%H%M%S')}-console-{action}.log"
            log = open(log_path, "wb")
            # stdout to a file, never a pipe: a stuck reader must not block a print on the release path
            process = self._popen(argv, cwd=str(self.repo_root), stdin=subprocess.PIPE, stdout=log,
                                  stderr=subprocess.STDOUT, start_new_session=True)
            log.close()
            self._run = {"action": action, "process": process, "log_path": log_path,
                         "started_at": self._clock(), "finished_at": None, "exit_code": None,
                         "checks": {}, "live": None, "session": None, "attempt": None,
                         "lines": deque(maxlen=200), "result": None, "offset": 0,
                         "settings": {"net": net, "operator": operator, "head": head,
                                      "time_limit_s": time_limit_s},
                         "recorder": None, "robot_data": None, "stop_requested": False}
            threading.Thread(target=self._follow, args=(self._run,), daemon=True,
                             name="k1-live-follow").start()
        return {"started": True, "action": action}

    def stop(self) -> dict[str, Any]:
        """STOP: SIGINT to the bridge (it slows the arms, hands them back, turns the head straight)."""
        with self._lock:
            run = self._run
            if run is None or run["process"].poll() is not None:
                return {"stopped": False, "message": "Nothing is running."}
            run["stop_requested"] = True
            try:
                os.kill(run["process"].pid, signal.SIGINT)
            except OSError:
                pass
        return {"stopped": True, "message": "Stopping: the arms slow down and are handed back."}

    def status(self) -> dict[str, Any]:
        with self._lock:
            run = self._run
            own: set[int] = set()
            if run is not None and run["process"].poll() is None:
                own |= descendants(run["process"].pid)
            if run is not None and run.get("recorder") is not None and run["recorder"].poll() is None:
                own |= descendants(run["recorder"].pid)
            out: dict[str, Any] = {"other_dds": self._dds_check(own), "run": None,
                                   "limits_s": list(RUN_LIMITS_S), "practice": self.transport == "fake"}
            if run is None:
                return out
            out["run"] = {
                "action": run["action"], "running": run["process"].poll() is None,
                "started_at": run["started_at"], "finished_at": run["finished_at"],
                "exit_code": run["exit_code"], "settings": dict(run["settings"]),
                "checks": [dict(key=key, **value) for key, value in run["checks"].items()],
                "live": run["live"], "session": run["session"], "attempt": run["attempt"],
                "result": run["result"], "stop_requested": run["stop_requested"],
                "robot_data": run["robot_data"], "log": str(run["log_path"]),
                "lines": list(run["lines"])[-40:],
            }
            return out

    def shutdown(self, wait_s: float = 20.0) -> None:
        """Console shutdown: STOP a running run and wait for its release."""
        self.stop()
        with self._lock:
            run = self._run
        if run is None:
            return
        try:
            run["process"].wait(timeout=wait_s)
        except subprocess.TimeoutExpired:
            pass

    # -- follower thread ----------------------------------------------------------------------

    def _follow(self, run: dict[str, Any]) -> None:
        process = run["process"]
        while True:
            done = process.poll() is not None
            self._read(run)
            if done:
                break
            time.sleep(0.1)
        self._stop_recorder(run)
        with self._lock:
            run["exit_code"] = process.returncode
            run["finished_at"] = self._clock()
            run["result"] = self._result(run)

    def _read(self, run: dict[str, Any]) -> None:
        try:
            with open(run["log_path"], "rb") as handle:
                handle.seek(run["offset"])
                data = handle.read()
        except OSError:
            return
        complete = data.rfind(b"\n") + 1
        if not complete:
            return
        run["offset"] += complete
        for raw in data[:complete].decode("utf-8", "replace").splitlines():
            self._line(run, raw.rstrip("\r"))

    def _line(self, run: dict[str, Any], line: str) -> None:
        if not line.startswith(MARK):
            if line.strip() and "InitChannel" not in line:
                with self._lock:
                    run["lines"].append(line)
            return
        try:
            event = json.loads(line[len(MARK):])
        except ValueError:
            return
        kind = event.get("event")
        with self._lock:
            if kind == "checks":
                for item in event.get("items") or []:
                    if isinstance(item, (list, tuple)) and len(item) >= 3:
                        run["checks"][str(item[0])] = {"ok": item[1], "text": str(item[2])}
                    elif isinstance(item, dict) and "key" in item:
                        run["checks"][str(item["key"])] = {"ok": item.get("ok"), "text": str(item.get("text", ""))}
            elif kind == "live":
                run["live"] = {k: v for k, v in event.items() if k != "event"}
            elif kind == "session":
                run["session"] = event.get("path")
            elif kind == "attempt":
                run["attempt"] = event.get("path")
        if kind == "prompt":
            # check: answer the Go question "no" (nothing moves); go never asks, so any other
            # question is answered "no" too and the bridge stops safely.
            self._answer(run, event.get("id"), "no")
        if kind == "attempt" and run["action"] == "go" and self.transport == "booster":
            self._start_recorder(run)

    def _answer(self, run: dict[str, Any], prompt_id: Any, answer: str) -> None:
        try:
            run["process"].stdin.write(f"{prompt_id}\t{answer}\n".encode())
            run["process"].stdin.flush()
        except (OSError, ValueError, AttributeError):
            pass

    def _start_recorder(self, run: dict[str, Any]) -> None:
        session = run.get("session")
        if run.get("recorder") is not None or not session:
            return
        folder = Path(session) / "robot-data" / time.strftime("%Y%m%d-%H%M%S")
        recorder = self.repo_root / "tools" / "control" / "k1_flight_recorder.py"
        python = self.repo_root / ".venv" / "bin" / "python"
        try:
            folder.mkdir(parents=True, exist_ok=True)
            log = open(folder / "recorder.log", "wb")
            process = self._popen([str(python), "-u", str(recorder), str(folder), run["settings"]["net"], "0"],
                                  cwd=str(self.repo_root), stdin=subprocess.DEVNULL, stdout=log,
                                  stderr=subprocess.STDOUT, start_new_session=True)
            log.close()
        except OSError as exc:
            with self._lock:
                run["robot_data"] = {"folder": str(folder), "state": f"not recorded: {exc}"}
            return
        with self._lock:
            run["recorder"] = process
            run["robot_data"] = {"folder": str(folder), "state": "recording"}

    def _stop_recorder(self, run: dict[str, Any]) -> None:
        process = run.get("recorder")
        if process is None:
            return
        if process.poll() is None:
            try:
                os.kill(process.pid, signal.SIGTERM)
                process.wait(timeout=15)
            except (OSError, subprocess.TimeoutExpired):
                try:
                    os.kill(process.pid, signal.SIGKILL)
                except OSError:
                    pass
        with self._lock:
            folder = run["robot_data"]["folder"] if run.get("robot_data") else None
            count = len(list(Path(folder).glob("*.npz"))) if folder else 0
            run["robot_data"] = {"folder": folder, "state": f"saved ({count} files)" if count else "no data saved"}

    def _result(self, run: dict[str, Any]) -> dict[str, Any]:
        """``ready`` (check: both checks green), ``passed``, ``stopped``, ``refused`` or ``failed``."""
        lines = list(run["lines"])
        code = run["process"].returncode
        refused = next((l for l in reversed(lines) if l.startswith("REFUSED")), None)
        if run["action"] == "check":
            if refused and "not confirmed" in refused:
                return {"outcome": "ready", "reason": None}
            return {"outcome": "refused", "reason": (refused or f"the check stopped (exit {code})")}
        final = next((m for m in (_RESULT.search(l) for l in reversed(lines)) if m), None)
        if final and final.group(1) == "PASS":
            return {"outcome": "passed", "reason": None}
        if run["stop_requested"] or code == 130:
            return {"outcome": "stopped", "reason": "STOP"}
        if refused:
            return {"outcome": "refused", "reason": refused}
        reason = final.group(2).strip(" :-") if final else f"exit {code}"
        return {"outcome": "failed", "reason": reason or f"exit {code}"}
