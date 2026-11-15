#!/usr/bin/env python3
"""Click-through GUI for the Booster K1: the quick start (live arms after two automatic
checks and the owner's Go) and, under "Advanced tests", the 13 steps of
docs/k1-bringup-runbook.md.

    .venv/bin/python tools/control/k1_commission_gui.py

It never moves the robot itself: the quick start and step 13 run
tools/control/k1_live_bridge.py, step 12 tools/control/k1_arm_bridge.py and the
other steps tools/control/k1_commission.py, each in a child process through
tools/control/k1_gui_child.py.  It shows the stage ladder of the open session's
robot (mocap_g1/stage_gate.py); stage-gate problems are warnings on the Go
screen.  The GUI turns the
tool's questions into buttons, shows its output live, explains failures, and
raises a red alarm when a release is not confirmed or a run ends abnormally.
The real stop is always the Booster remote: LT+BACK (damping).

Linux only (the Booster SDK is Linux only); needs Tk (Ubuntu: ``sudo apt install python3-tk``).
"""

from __future__ import annotations

from dataclasses import dataclass, field
import datetime as dt
import ipaddress
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from tools.control.k1_arm_bridge import MANUAL_STOP  # noqa: E402
from tools.control.k1_commission import (  # noqa: E402
    GAP_LEVELS,
    HOLD_MAX_S,
    ROBOT_IP,
    SAFE,
    CommissionError,
    Session,
    hint,
    ladder,
)
from tools.control import k1_live_bridge as live  # noqa: E402
from tools.control.k1_gui_child import MARK  # noqa: E402
from mocap_g1.stage_gate import STAGES, Ledger, StageGateError, commissioning_root, entry_problems  # noqa: E402

CHILD = Path(__file__).with_name("k1_gui_child.py")
SENDER = Path(__file__).with_name("k1_live_synth.py")
ROOT = commissioning_root()
#: Step 13's "Run" choices and the bridge's --drill value of each.
LIVE_DRILLS = {"Following": None, "Stream-loss drill": "stream-loss", "STOP drill": "stop"}
#: The practice sender outlives any run (gate wait + time limit); it is stopped with the step.
SENDER_SECONDS = live.MAX_TIME_LIMIT_S + 2 * live.GATE_WAIT_S + 60
MOTION = ("quick-release", "hold", "balance", "command-loss", "signs", "gains")
REPLAY_MAX_S = 30.0

#: (key, title, what happens, robot moves)
STEPS: list[tuple[str, str, str, str]] = [
    ("preflight", "1 · Preflight", "Checks this PC: Python, MuJoCo, the K1 model, the Booster "
     "SDK, the network address and that the robot answers ping.", "No"),
    ("probe", "2 · Probe (new session)", "Watches the robot for 10 s without commanding it and "
     "starts a new session folder with the robot's serial and firmware.", "No"),
    ("mirror", "3 · Mirror", "Opens a 3D view of the K1 posed from the robot's measured joints. "
     "Keep it open during every later step: the mirror must always look like the robot.", "No"),
    ("zero-check", "4 · Zero check", "You measure four angles with a protractor (upper arm from "
     "down, elbow bend, each arm); the tool compares them with the model. Robot in walking mode, "
     "arms still.", "No"),
    ("quick-release", "5 · Quick release", "First takeover: the PC takes the arms and gives them "
     "back at once. No arm command is sent.", "No (takeover and release only)"),
    ("hold", "6 · Hold", "The arms hold still (elbows settle 3° inward), then slow to rest and are "
     "released.", "Settling only"),
    ("balance", "7 · Balance", "The arms hold still for 30 s while you push the robot gently at "
     "the chest, robot still in the hoist.", "Settling only"),
    ("command-loss", "8 · Command loss", "Commands stop for a short gap; the tool watches what the "
     "arms do, then releases. Run every gap in order: 0.1, 0.25, 0.5, 1.0 s.",
     "Settling, then no commands"),
    ("signs", "9 · Signs", "Each arm joint moves 0.1 rad (6°) and back, one at a time. Watch the "
     "robot and the mirror together; afterwards you confirm each joint.", "6° per joint"),
    ("gains", "10 · Gains", "Each joint out and back, then all together; measures how closely the "
     "arms follow. Run L1, then L2, then L3 with the same kp/kd.", "Up to 17° per joint"),
    ("record", "11 · Record", "Checks every step, asks which gains run to approve, asks two "
     "questions for later phases and writes the acceptance record when you type your name.", "No"),
    ("replay", "12 · First real replay", "Replays a short, slow, qualified recording with the "
     "approved limits from the record. Qualify the recording first (guide, step 12).",
     "The recording"),
    ("live", "13 · Live arms", "The arms follow the performer in the suit, streamed by the live "
     "tool (--robot-stream). The arms take over only when the performer's arms match the "
     "robot's; STOP slows them down and hands them back. Run following attempts and both "
     "drills, then Evaluate stage.", "The performer's arm motion"),
]
TITLES = {key: title for key, title, _what, _moves in STEPS}
TITLES["quick"] = "Quick start"
QUICK_TEXT = ("Live arms in two clicks. Check: the tool probes the robot (read-only) and makes a "
              "new session, then checks the robot (walking, not fallen, fresh joint state, joints "
              "in range) and the suit stream (fresh, calibrated, tracking on, model, clock, the "
              "performer's arms near the robot's). It shows a summary with any warnings; Go starts "
              "the arms. Before Check: the live tool streams (--robot-stream), Calibrate and "
              "Tracking on are done, the performer stands with arms down.")
#: Steps whose confirm question is the owner's Go (the summary is its detail).
GO_STEPS = ("quick", "live", "replay")

ALARM_TEXT = ("STOP THE ROBOT WITH THE REMOTE NOW: LT+BACK (damping). Keep it on the hoist. "
              "Then read the log below and send the session folder to the developers.")


# -- the child process ------------------------------------------------------------


@dataclass
class Prompt:
    id: int
    kind: str  # "yesno", "confirm" or "text"
    text: str
    detail: str = ""
    answered: bool = False


@dataclass
class RunResult:
    verdict: str  # PASS FAIL INCOMPLETE DONE REFUSED CANCELLED STOPPED CRASHED
    alarm: str | None
    hint: str = ""


@dataclass
class StepRunner:
    """One tool run in a child process; no Tk, so it is testable headless."""

    tool: str  # "commission" or "replay"
    args: list[str]
    log_path: Path
    step: str = ""
    env: dict[str, str] | None = None
    process: subprocess.Popen[str] | None = None
    lines: list[str] = field(default_factory=list)
    prompt: Prompt | None = None
    session: Path | None = None
    attempt: dict[str, Any] | None = None
    bridge_state: str | None = None
    rpc: list[bool] = field(default_factory=list)
    #: The newest ``live`` status of the live bridge (step 13).
    live: dict[str, Any] | None = None
    #: Helpers (the practice sender) stopped when this run has ended.
    companions: list[Any] = field(default_factory=list)
    _reader: Any = None
    _partial: str = ""
    _drained: bool = False

    def start(self) -> None:
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        log = self.log_path.open("w", encoding="utf-8")
        try:  # the child writes to the file, never to a pipe: a dead GUI cannot break its prints
            self.process = subprocess.Popen(
                [sys.executable, "-u", str(CHILD), "--parent-pid", str(os.getpid()),
                 "--tool", self.tool, "--", *self.args],
                stdin=subprocess.PIPE, stdout=log, stderr=subprocess.STDOUT, text=True,
                cwd=REPO_ROOT, env={**os.environ, **(self.env or {})})
        finally:
            log.close()
        self._reader = self.log_path.open("r", encoding="utf-8", errors="replace")

    @property
    def running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    @property
    def finished(self) -> bool:
        return self._drained

    def poll(self) -> list[tuple[str, Any]]:
        """New ``("line", text)`` and ``("event", dict)`` items since the last call."""
        if self._reader is None or self._drained:
            return []
        exited = not self.running  # read after checking: everything before the exit is in the file
        self._partial += self._reader.read()
        *complete, self._partial = self._partial.split("\n")
        if exited:
            complete += [self._partial] if self._partial else []
            self._partial = ""
            self._drained = True
            self._reader.close()
        items: list[tuple[str, Any]] = []
        for line in complete:
            if line.startswith(MARK):
                try:
                    event = json.loads(line[len(MARK):])
                except ValueError:
                    items.append(("line", line))
                    continue
                self._handle(event)
                items.append(("event", event))
            else:
                self.lines.append(line)
                items.append(("line", line))
        if exited:
            self.prompt = None
            for companion in self.companions:
                companion.stop()
        return items

    def _handle(self, event: dict[str, Any]) -> None:
        kind = event.get("event")
        if kind == "prompt":
            self.prompt = Prompt(int(event["id"]), str(event["kind"]), str(event["text"]),
                                 str(event.get("detail", "")))
        elif kind == "session":
            self.session = Path(event["path"])
        elif kind == "attempt" and not (self.step == "quick" and event.get("step") == "probe"):
            self.attempt = event  # the quick start's verdict is its live attempt, not the probe
        elif kind == "rpc":
            self.rpc.append(bool(event["enable"]))
        elif kind == "exit":
            self.bridge_state = event.get("bridge_state")
        elif kind == "live":
            self.live = event

    def answer(self, prompt_id: int, text: str) -> bool:
        """Send one answer to the open question; refuses stale ids, repeats and newlines."""
        prompt = self.prompt
        if (prompt is None or prompt.answered or prompt.id != prompt_id or "\n" in text
                or "\r" in text or not self.running or self.process is None
                or self.process.stdin is None):
            return False
        prompt.answered = True
        try:
            self.process.stdin.write(f"{prompt_id}\t{text}\n")
            self.process.stdin.flush()
        except OSError:
            return False
        return True

    def stop(self) -> None:
        """Ctrl-C for the tool: it stops and releases the arms itself."""
        if self.running and self.process is not None:
            self.process.send_signal(signal.SIGINT)

    def result(self) -> RunResult:
        code = self.process.returncode if self.process is not None else None
        text = "\n".join(self.lines)
        verdict = None
        if self.attempt is not None:
            try:
                data = json.loads(Path(self.attempt["path"]).read_text(encoding="utf-8"))
                verdict = str(data.get("verdict"))
            except (OSError, ValueError, KeyError):
                verdict = "INCOMPLETE"
        if verdict is None:
            if code == 0:
                verdict = "PASS" if self.step in ("preflight", "replay", "live", "quick") else "DONE"
            elif code == 1:
                verdict = "FAIL"
            elif code == 2:
                verdict = "CANCELLED" if "REFUSED: not confirmed" in text else "REFUSED"
            elif code in (3, 130):
                verdict = "STOPPED"
            else:
                verdict = "CRASHED"
        reasons = []
        if MANUAL_STOP in text:
            reasons.append("the release was NOT confirmed")
        if code is None or code < 0:
            reasons.append(f"the tool was killed (signal {-code if code else '?'})")
        elif code not in (0, 1, 2, 3, 130):
            reasons.append(f"the tool ended abnormally (exit {code})")
        if "Traceback (most recent call last)" in text:
            reasons.append("the tool crashed (see the Traceback in the log)")
        if verdict == "INCOMPLETE":
            reasons.append("the attempt is INCOMPLETE")
        if self.bridge_state in ("tracking", "release_unconfirmed"):
            reasons.append(f"the arms bridge ended in state '{self.bridge_state}'")
        alarm = ALARM_TEXT + "\nWhy: " + "; ".join(reasons) + "." if reasons else None
        found = next((line.split("What to check:", 1)[1].strip() for line in reversed(self.lines)
                      if "What to check:" in line), "")
        if not found and verdict in ("FAIL", "REFUSED"):
            refused = next((line for line in reversed(self.lines)
                            if line.startswith(("REFUSED:", "ERROR:"))), "")
            found = hint(refused.split(":", 1)[1].strip()) if refused else ""
        return RunResult(verdict, alarm, found)


@dataclass
class PracticeSender:
    """tools/control/k1_live_synth.py streaming to the live bridge: practice without the suit.

    Its own child, stopped with the step (or by the operator: the stream-loss drill)."""

    port: int
    log_path: Path
    model: str | None = None
    process: subprocess.Popen[bytes] | None = None

    def start(self) -> None:
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        argv = [sys.executable, "-u", str(SENDER), "--port", str(self.port), "--seconds",
                f"{SENDER_SECONDS:g}", *(["--model", self.model] if self.model else [])]
        with self.log_path.open("w", encoding="utf-8") as log:
            self.process = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=log,
                                            stderr=subprocess.STDOUT, cwd=REPO_ROOT,
                                            preexec_fn=_die_with_window)

    @property
    def running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def stop(self, timeout: float = 5.0) -> None:
        if not self.running or self.process is None:
            return
        self.process.send_signal(signal.SIGINT)
        try:
            self.process.wait(timeout)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()


def _die_with_window() -> None:
    """In the sender child before exec: SIGINT when the window dies (like k1_gui_child)."""
    import ctypes

    ctypes.CDLL(None).prctl(1, int(signal.SIGINT), 0, 0, 0)  # PR_SET_PDEATHSIG


def bounded(label: str, text: str, low: float, high: float) -> str:
    try:
        value = float(text)
    except ValueError:
        raise ValueError(f"{label}: not a number") from None
    if not low <= value <= high:
        raise ValueError(f"{label}: must be {low:g} to {high:g}")
    return f"{value:g}"


def live_command(session: Path, meta: dict[str, Any], *, stage: str, time_limit: str, port: str,
                 drill: str, operator: str = "", evaluate: bool = False,
                 attested: str = "", record: Path | None = None, head: bool = True) -> list[str]:
    """The k1_live_bridge.py arguments of step 13 (a run, or ``--evaluate``)."""
    if stage not in ("hoist", "standing"):
        raise ValueError("stage: choose hoist or standing")
    args = ["--session", str(session), "--stage", stage]
    if evaluate:
        if not operator.strip():
            raise ValueError("type your name as the operator: it signs the ledger entry")
        return [*args, "--evaluate", "--operator", operator.strip()]
    if drill not in LIVE_DRILLS:
        raise ValueError("run: choose Following or a drill")
    args += ["--time-limit", bounded("time limit", time_limit, 1, live.MAX_TIME_LIMIT_S),
             "--port", bounded("port", port if port.strip().isdigit() else "x", 1024, 65535)]
    if LIVE_DRILLS[drill]:
        args += ["--drill", str(LIVE_DRILLS[drill])]
    if attested.strip():
        args += ["--attested-robot-id", attested.strip()]
    if not head:
        args.append("--no-head")
    if operator.strip():
        args += ["--operator", operator.strip()]
    if meta.get("transport") == "booster" and record is not None:  # optional: a warning if absent
        args += ["--acceptance-record", str(record)]
    return args


def quick_command(*, transport: str, net: str, domain: str, model: str, stage: str,
                  time_limit: str, port: str, support: str, strap_clear: bool, head: bool,
                  record: str = "", operator: str = "", attested: str = "", firmware: str = "",
                  root: Path | None = None) -> list[str]:
    """The k1_live_bridge.py arguments of the quick start (from cold: no ``--session``)."""
    if transport not in ("fake", "booster"):
        raise ValueError("transport: choose fake or booster")
    if stage not in ("hoist", "standing"):
        raise ValueError("stage: choose hoist or standing")
    if not support.strip() or "\n" in support:
        raise ValueError("support: say how the robot is held")
    args = ["--transport", transport, "--domain-id", bounded("DDS domain", domain, 0, 232),
            "--root", str(root or ROOT), "--stage", stage,
            "--time-limit", bounded("time limit", time_limit, 1, live.MAX_TIME_LIMIT_S),
            "--port", bounded("port", port if port.strip().isdigit() else "x", 1024, 65535),
            "--support", support.strip()]
    for flag, value in (("--net", net_value(net)), ("--model", model.strip()),
                        ("--acceptance-record", record.strip()), ("--operator", operator.strip()),
                        ("--attested-robot-id", attested.strip()), ("--firmware", firmware.strip())):
        if value:
            args += [flag, value]
    if record.strip() and not Path(record.strip()).is_file():
        raise ValueError(f"acceptance record: {record.strip()} is not a file")
    if stage == "hoist" and not strap_clear:
        args.append("--strap-not-clear")
    if not head:
        args.append("--no-head")
    return args


def refusal_text(lines: list[str]) -> str:
    """The tool's last ``REFUSED:``/``ERROR:`` line with its indented detail lines."""
    for index in range(len(lines) - 1, -1, -1):
        if lines[index].startswith(("REFUSED:", "ERROR:")):
            found = [lines[index].split(":", 1)[1].strip()]
            for line in lines[index + 1:]:
                if not line.startswith("  "):
                    break
                found.append(line.strip())
            return "\n".join(found)
    return ""


def sender_model(meta: dict[str, Any]) -> str | None:
    """The practice sender sends the hash of the model the bridge uses (the session's)."""
    return str(meta["model_path"]) if meta.get("model_sha256") and meta.get("model_path") else None


LIVE_COLOURS = {"following": "valid", "lag-hold": "incomplete", "stopping": "incomplete",
                "waiting": "info", "aligning": "info", "released": "none"}


def live_status_text(event: dict[str, Any]) -> tuple[str, str]:
    """``(lamp kind, two lines)`` for the live status panel from one ``live`` event."""
    def number(key: str, form: str) -> str:
        value = event.get(key)
        return "-" if value is None else form.format(value)

    state = str(event.get("state"))
    head = f"LIVE ARMS {event.get('stage')}: {state.upper()}"
    if event.get("drill"):
        head += f"  (drill: {event['drill']})"
    if state == "waiting" and event.get("gate"):
        head += f"  - {event['gate']}"
    if state == "lag-hold":
        head += "  - performer too fast: holding, then re-aligning"
    if event.get("trigger"):
        head += f"  - ended by {event['trigger']}"
    enabled = event.get("enabled")
    line = (f"stream {number('rate_hz', '{:.0f}')} Hz   age {number('source_age_s', '{:.3f}')} s   "
            f"epoch {number('epoch', '{}')}   tracking "
            f"{'-' if enabled is None else 'on' if enabled else 'OFF'}   worst gap "
            + (f"{event['worst_joint']} {number('worst_gap_rad', '{:.2f}')} rad   "
               if event.get("worst_joint") else "-   ")
            + f"following {number('following_s', '{:.0f}')}/{number('time_limit_s', '{:.0f}')} s   "
            f"release {event.get('release', '-')}")
    kind = LIVE_COLOURS.get(state, "bad")  # fault_latched, release_unconfirmed, ...
    if event.get("release") == "failed":
        kind = "bad"
    return kind, f"{head}\n{line}"


# -- the stage ladder -------------------------------------------------------------


def session_ledger(meta: dict[str, Any], root: Path | None = None) -> Ledger:
    """The ledger of the session's robot; a practice session's is the PRACTICE ledger."""
    return Ledger(root or ROOT, str(meta.get("serial")), practice=bool(meta.get("practice")))


def ladder_rows(ledger: Ledger, identity: dict[str, str],
                now: dt.datetime | None = None) -> list[tuple[str, str, str, str]]:
    """``(stage, title, kind, text)`` per stage; kind is passed, invalid, revoked, locked,
    ready or practice.  The run's settings are compared when it starts (not here)."""
    try:
        ledger.entries()
    except StageGateError as exc:
        return [(key, spec.title, "invalid", str(exc)) for key, spec in STAGES.items()]
    rows = []
    for key, spec in STAGES.items():
        latest = ledger.latest(key)
        verdict = latest.get("verdict") if latest else None
        date = str(latest.get("date", ""))[:10] if latest else ""
        if verdict == "REVOKED":
            assert latest is not None
            rows.append((key, spec.title, "revoked", f"revoked {date}: {latest.get('reason')}"))
        elif ledger.practice:
            rows.append((key, spec.title, "practice", f"practice pass {date} (unlocks nothing)"
                         if verdict == "PASS" else "practice: not passed"))
        elif verdict == "PASS":
            problems = entry_problems(key, ledger=ledger, identity=identity, now=now)
            rows.append((key, spec.title, "invalid", f"passed {date}, no longer valid: {problems[0]}")
                        if problems else (key, spec.title, "passed", f"passed {date}"))
        else:
            problems = [p for gate in spec.gate
                        for p in entry_problems(gate, ledger=ledger, identity=identity, now=now)]
            rows.append((key, spec.title, "locked", f"locked: {problems[0]}") if problems
                        else (key, spec.title, "ready", "not passed; its gate is open"))
    return rows


LADDER_LAMP = {"passed": "valid", "invalid": "bad", "revoked": "bad", "locked": "none",
               "ready": "none", "practice": "info"}


def ladder_lamp(rows: list[tuple[str, str, str, str]], kind: str) -> tuple[str, str] | None:
    """Step 12 (``replay``) or 13 (``live-arms``) evidence from the ledger rows."""
    mine = [row for row in rows if row[0].startswith(kind + "@")]
    if not mine:
        return None
    kinds = [row[2] for row in mine]
    lamp_kind = ("bad" if {"invalid", "revoked"} & set(kinds) else "valid" if kinds[0] == "passed"
                 else LADDER_LAMP[kinds[0]])
    text = "  ·  ".join(f"{title.split(' · ')[0]} {text}" for _key, title, _kind, text in mine)
    return lamp_kind, text


# -- evidence ---------------------------------------------------------------------


def session_problem(meta: dict[str, Any]) -> str | None:
    """Why a session must not be used from the GUI, or None."""
    transport = meta.get("transport")
    if transport not in ("fake", "booster"):
        return f"session.json has no valid transport ({transport!r})"
    if meta.get("practice") is not (transport == "fake"):
        return "session.json: 'practice' does not match the transport"
    return None


def lamp(session: Session, key: str) -> tuple[str, str]:
    latest = session.latest(key)
    if latest is None:
        return "none", "not run"
    problem = session.problem(latest, key)
    if problem is None:
        return "valid", f"valid (attempt {latest.get('id')})"
    if latest.get("verdict") == "INCOMPLETE":
        return "incomplete", problem
    return "bad", problem


def evidence(path: Path, mode: str) -> dict[str, tuple[str, str]]:
    """Each step's lamp from the session folder: valid, bad, incomplete, none or info."""
    session = Session(path)
    rows: dict[str, tuple[str, str]] = {"probe": lamp(session, "probe"),
                                        "zero-check": lamp(session, "zero-check")}
    for step in ("quick-release", "hold", "balance", "signs"):
        rows[step] = lamp(session, f"{step}@{mode}")
    gaps = {gap: lamp(session, f"command-loss@{mode}@{gap:g}") for gap in GAP_LEVELS}
    marks = {"valid": "ok", "none": "-", "bad": "X", "incomplete": "?"}
    state = ("valid" if all(v[0] == "valid" for v in gaps.values())
             else "none" if all(v[0] == "none" for v in gaps.values()) else "bad")
    rows["command-loss"] = (state, "  ".join(f"{gap:g}s {marks[v[0]]}" for gap, v in gaps.items()))
    gains = sorted({a["step"] for a in session.attempts()
                    if str(a.get("step", "")).startswith(f"gains@{mode}@")})
    valid = [key.split("@", 2)[2] for key in gains if lamp(session, key)[0] == "valid"]
    rows["gains"] = (("valid", "valid: " + ", ".join(valid)) if valid
                     else ("bad", "no valid gains run") if gains else ("none", "not run"))
    mirrors = sorted(path.glob("mirror-*.json"), key=lambda p: p.stat().st_mtime)
    rows["mirror"] = (("info", f"ranges saved {stamp(mirrors[-1])}") if mirrors
                      else ("none", "not run"))
    records = sorted(path.glob("k1-acceptance*.json"), key=lambda p: p.stat().st_mtime)
    if records:
        record = records[-1]
        keys = ladder(mode)
        newer = [key for key in keys if (latest := session.latest(key)) is not None
                 and (path / "attempts" / f"{latest['id']}.json").stat().st_mtime
                 > record.stat().st_mtime]
        broken = [key for key in keys if lamp(session, key)[0] != "valid"]
        rows["record"] = (("bad", "STALE: " + ", ".join(newer or broken) + " changed since; "
                           "run record again") if newer or broken
                          else ("info", f"{record.name} written {stamp(record)}"))
    else:
        rows["record"] = ("none", "not written")
    return rows


def stamp(path: Path) -> str:
    return dt.datetime.fromtimestamp(path.stat().st_mtime).strftime("%Y-%m-%d %H:%M")


def acceptance_record(path: Path) -> Path | None:
    record = path / "k1-acceptance.json"
    return record if record.is_file() else None


def lock_path() -> Path:
    state = Path(os.environ.get("XDG_STATE_HOME") or Path.home() / ".local" / "state")
    return state / "mocap-g1" / "k1-tests.lock"


def take_instance_lock(path: Path | None = None) -> Any:
    """The open lock file while this window runs, or None when the window is already open or
    tools/setup/install.sh is running (it holds the same lock while it changes .venv)."""
    import fcntl

    path = path or lock_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    handle = path.open("a")
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        handle.close()
        return None
    return handle


def default_model() -> str:
    try:
        from tools.control.k1_arm_bridge import resolve_model
        found = resolve_model(None)
    except Exception:  # noqa: BLE001 - only a default for a field
        found = None
    fallback = Path.home() / "mocap-runtime" / "GMR" / "assets" / "booster_k1" / "K1_serial.xml"
    return str(found) if found else str(fallback) if fallback.is_file() else ""


def interface_choices() -> list[str]:
    """``"<address> (<interface>)"`` for each IPv4 address of this PC."""
    from tools.control.k1_commission import local_addresses

    return [f"{address} ({name})" for name, address in local_addresses() if name != "lo"]


def net_value(choice: str) -> str:
    """The ``--net`` value from a dropdown entry or a typed address or interface."""
    return choice.strip().split(" ", 1)[0] if choice.strip() else ""


def robot_network(robot_ip: str) -> ipaddress.IPv4Network | ipaddress.IPv6Network:
    return ipaddress.ip_network(f"{robot_ip.strip()}/24", strict=False)  # ValueError if invalid


def on_robot_network(choices: list[str], robot_ip: str) -> list[str]:
    try:
        network = robot_network(robot_ip)
    except ValueError:
        return []
    found = []
    for choice in choices:
        try:
            if ipaddress.ip_address(net_value(choice)) in network:
                found.append(choice)
        except ValueError:
            continue
    return found


def firewall_state(run: Any = subprocess.run, conf: Path = Path("/etc/ufw/ufw.conf")) -> str:
    """``active``, ``inactive`` or ``unknown``, read without root (``ufw status`` needs root)."""
    try:
        unit = run(["systemctl", "is-active", "ufw"], capture_output=True, text=True,
                   check=False).stdout.strip()
    except OSError:
        unit = ""
    try:
        text = conf.read_text(encoding="utf-8")
        enabled: bool | None = any(line.strip().lower() == "enabled=yes"
                                   for line in text.splitlines())
    except OSError:
        enabled = None
    if unit == "active" and enabled:
        return "active"
    if enabled is False or not conf.parent.exists():  # switched off, or ufw not installed
        return "inactive"
    return "unknown"


def firewall_command(robot_ip: str) -> list[str]:
    return ["pkexec", "/usr/sbin/ufw", "allow", "from", str(robot_network(robot_ip))]


def run_privileged(argv: list[str], run: Any = subprocess.run) -> tuple[str, str]:
    """``(outcome, text)``: outcome ``done``, ``cancelled`` or ``failed``."""
    try:
        done = run(argv, capture_output=True, text=True, check=False, timeout=300)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return "failed", str(exc)
    text = (done.stdout + done.stderr).strip()
    if done.returncode == 0:
        return "done", text
    if done.returncode == 126:  # pkexec: the password dialog was dismissed
        return "cancelled", text
    return "failed", text or f"exit {done.returncode}"


def zip_session(path: Path) -> Path:
    """``<session>.zip`` next to the session folder; never overwrites (``-2``, ``-3``...)."""
    import zipfile

    path = Path(path).resolve()
    target = path.with_suffix(".zip")
    number = 2
    while target.exists():
        target = path.parent / f"{path.name}-{number}.zip"
        number += 1
    partial = target.with_name(target.name + ".tmp")
    try:
        with zipfile.ZipFile(partial, "w", zipfile.ZIP_DEFLATED) as archive:
            for file in sorted(path.rglob("*")):
                if file.is_file():
                    archive.write(file, Path(path.name) / file.relative_to(path))
        os.replace(partial, target)
    finally:
        partial.unlink(missing_ok=True)
    return target


def newest_preview(root: Path | None = None) -> Path | None:
    """The newest K1 preview recording the Setup console made (a hint, never a choice)."""
    root = root or Path.home() / "mocap-sessions"
    found = [p for p in root.glob("**/booster_k1-preview.pkl") if p.is_file()] if root.is_dir() else []
    return max(found, key=lambda p: p.stat().st_mtime) if found else None


def maintenance_result(work: Any) -> tuple[str, str]:
    """Run firewall or zip work; never raises, so the window's maintenance slot always clears."""
    try:
        outcome, text = work()
    except Exception as exc:  # noqa: BLE001 - reported in the log
        return "failed", str(exc)
    return str(outcome), str(text)


def launch_block(key: str, state: dict[str, Any], *, read_guide: bool) -> str | None:
    """Why step ``key`` cannot start now, or None."""
    if state.get("alarm"):
        return "Acknowledge the red alarm first."
    if state.get("maintenance") is not None:
        return f"Wait: {state['maintenance']} is still running."
    if key == "mirror":
        if state.get("mirror") is not None:
            return "The mirror is already open."
    elif state.get("step") is not None:
        return "Another step is running."
    if key not in ("preflight", "probe", "quick") and state.get("session") is None:
        return "Needs a session: run 2 · Probe or open a session."
    if key == "probe" and state.get("mirror") is not None:
        return "Close the mirror before a new probe."
    if key == "replay" and not read_guide:
        return "Tick the box below the duration first."
    if key == "quick" and state.get("mirror") is not None:
        return "Close the mirror before a quick start (it makes a new session)."
    if key == "live":
        stage = state.get("live_stage", "hoist")
        reasons = (state.get("live_lock") or {}).get(stage) or []
        if reasons:
            return f"Live arms {stage} cannot start: {reasons[0]}"
    return None


def valid_gains_attempts(path: Path, mode: str) -> list[str]:
    session = Session(path)
    return [f"{a['id']}  {a['step'].split('@', 2)[2]}" for a in session.attempts()
            if str(a.get("step", "")).startswith(f"gains@{mode}@")
            and lamp(session, a["step"])[0] == "valid"
            and session.latest(a["step"]).get("id") == a.get("id")]  # type: ignore[union-attr]


# -- the window -------------------------------------------------------------------


def run_gui() -> int:  # pragma: no cover - exercised by hand and by the smoke test
    import queue
    import threading
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk

    root = tk.Tk()
    root.title("K1 hardware tests")
    root.geometry("1400x900")
    big = ("TkDefaultFont", 13, "bold")
    colours = {"valid": "#1e7b34", "bad": "#b3261e", "incomplete": "#b26a00", "none": "#666666",
               "info": "#1f5fa8"}

    state: dict[str, Any] = {"session": None, "meta": {}, "step": None, "mirror": None,
                             "selected": "preflight", "alarm": False, "closing": False,
                             "preflight": None, "replay": None, "maintenance": None,
                             "live_run": None, "live_stage": "hoist", "live_lock": {},
                             "sender": None, "ladder": [], "ladder_window": None}
    results: queue.Queue[tuple[str, str, str]] = queue.Queue()  # maintenance results
    v = {name: tk.StringVar(value=value) for name, value in {
        "transport": "fake", "net": "", "robot_ip": ROBOT_IP, "domain": "0", "model": "",
        "attested": "", "firmware": "", "mode": "walking", "seconds": "10",
        "gap": f"{GAP_LEVELS[0]:g}", "level": "L1", "kp": f"{SAFE['kp']:g}",
        "kd": f"{SAFE['kd']:g}", "operator": "", "gains_attempt": "", "pkl": "",
        "duration": "10", "stage": "hoist", "answer": "", "live_stage": "hoist",
        "time_limit": f"{live.TIME_LIMIT_S:g}", "port": str(live.DEFAULT_PORT),
        "drill": "Following", "support": live.SUPPORT_DEFAULT, "record": ""}.items()}
    strap_clear = tk.BooleanVar(value=True)
    head_follows = tk.BooleanVar(value=True)
    has_head = live.head_available()
    read_guide = tk.BooleanVar(value=False)
    v["pkl"].trace_add("write", lambda *_: read_guide.set(False))  # a new file needs a new tick
    state["firewall"] = firewall_state()
    v["model"].set(default_model())
    on_net = on_robot_network(interface_choices(), ROBOT_IP)
    v["net"].set(on_net[0] if on_net else "")

    # banner and alarm
    banner = tk.Label(root, font=("TkDefaultFont", 15, "bold"), fg="white", pady=6)
    banner.pack(fill="x")
    alarm = tk.Frame(root, bg="#b3261e")
    alarm_label = tk.Label(alarm, bg="#b3261e", fg="white", font=big, justify="left",
                           wraplength=950)

    def acknowledge() -> None:
        state["alarm"] = False
        alarm.pack_forget()
        refresh()

    tk.Button(alarm, text="I have stopped the robot\nwith the remote", font=big,
              command=acknowledge).pack(side="right", padx=10, pady=8)
    alarm_label.pack(side="left", padx=10, pady=8)  # after the button, so the button keeps room

    # session bar
    bar = ttk.LabelFrame(root, text="Robot connection and session")
    bar.pack(fill="x", padx=8, pady=4)
    fields = [("Transport", "transport", ("fake", "booster")), ("PC interface or IP (--net)", "net",
              None), ("Robot IP", "robot_ip", None), ("DDS domain", "domain", None),
              ("Mode (set with the remote)", "mode", ("walking", "prepare")),
              ("Attested serial", "attested", None), ("Firmware (with attested)", "firmware", None)]
    for column, (label, name, choices) in enumerate(fields):
        ttk.Label(bar, text=label).grid(row=0, column=column, sticky="w", padx=4)
        widget = (ttk.Combobox(bar, textvariable=v[name], values=choices, state="readonly",
                               width=10) if choices else ttk.Entry(bar, textvariable=v[name],
                                                                    width=13))
        if name == "net":  # this PC's addresses; typing an address or interface still works
            widget = ttk.Combobox(bar, textvariable=v[name], values=interface_choices(),
                                  width=22)
        widget.grid(row=1, column=column, sticky="we", padx=4)
    ttk.Label(bar, text="K1 model (K1_serial.xml)").grid(row=2, column=0, sticky="w", padx=4)
    ttk.Entry(bar, textvariable=v["model"], width=60).grid(row=3, column=0, columnspan=4,
                                                           sticky="we", padx=4)
    model_button = ttk.Button(bar, text="Browse...", command=lambda: v["model"].set(
        filedialog.askopenfilename(filetypes=[("MuJoCo XML", "*.xml")]) or v["model"].get()))
    model_button.grid(row=3, column=4, sticky="w")
    open_button = ttk.Button(bar, text="Open session...")
    open_button.grid(row=3, column=5, sticky="w", padx=4)
    net_label = ttk.Label(bar, foreground="#666666", wraplength=900)
    net_label.grid(row=2, column=1, columnspan=6, sticky="w", padx=4)
    firewall_row = ttk.Frame(bar)
    firewall_row.grid(row=4, column=0, columnspan=7, sticky="w", padx=4)
    firewall_label = ttk.Label(firewall_row)
    firewall_label.pack(side="left")
    firewall_button = ttk.Button(firewall_row, text="Allow the robot network in the firewall")
    firewall_button.pack(side="left", padx=8)
    session_label = ttk.Label(bar, text="No session yet: Quick start makes one.", font=big)
    session_label.grid(row=5, column=0, columnspan=7, sticky="w", padx=4, pady=4)
    for name in ("mode", "transport", "net", "robot_ip"):
        v[name].trace_add("write", lambda *_: refresh())

    # the quick start (first screen) and the 13 steps under "Advanced tests"
    tabs = ttk.Notebook(root)
    tabs.pack(fill="x", padx=8)
    quick = ttk.Frame(tabs)
    tabs.add(quick, text="Quick start")
    middle = ttk.Frame(tabs)
    tabs.add(middle, text="Advanced tests (13 steps)")
    ttk.Label(quick, text=QUICK_TEXT, wraplength=1300, justify="left").pack(anchor="w", padx=6,
                                                                          pady=4)
    qform = ttk.Frame(quick)
    qform.pack(anchor="w", padx=6)
    for column, (label, name, width, values) in enumerate((
            ("Stage", "live_stage", 9, ("hoist", "standing")),
            (f"Time limit (s, max {live.MAX_TIME_LIMIT_S:g})", "time_limit", 8, None),
            ("Stream port", "port", 7, None), ("Support", "support", 24, None),
            ("Your name (operator)", "operator", 18, None))):
        ttk.Label(qform, text=label).grid(row=0, column=column, sticky="w", padx=4)
        (ttk.Combobox(qform, textvariable=v[name], values=values, state="readonly", width=width)
         if values else ttk.Entry(qform, textvariable=v[name], width=width)).grid(
            row=1, column=column, sticky="w", padx=4)
    qrecord = ttk.Frame(quick)
    qrecord.pack(anchor="w", padx=6, pady=2)
    ttk.Label(qrecord, text="Acceptance record (optional; without one: attempt 049's settings, "
              "a warning)").pack(side="left")
    ttk.Entry(qrecord, textvariable=v["record"], width=40).pack(side="left", padx=4)
    record_button = ttk.Button(qrecord, text="Browse...", command=lambda: v["record"].set(
        filedialog.askopenfilename(filetypes=[("Acceptance record", "*.json")],
                                   initialdir=str(ROOT if ROOT.is_dir() else REPO_ROOT))
        or v["record"].get()))
    record_button.pack(side="left")
    qticks = ttk.Frame(quick)
    qticks.pack(anchor="w", padx=6, pady=2)
    ttk.Checkbutton(qticks, variable=strap_clear, text="Strap clear of both arms (hoist; "
                    "untick: shoulders stay within 0.6 rad of rest)").pack(side="left")
    if has_head:  # without the head module --no-head does nothing: no tick box
        ttk.Checkbutton(qticks, variable=head_follows, text="Head follows").pack(side="left",
                                                                                 padx=16)
    qbuttons = ttk.Frame(quick)
    qbuttons.pack(anchor="w", padx=6, pady=6)
    check_button = tk.Button(qbuttons, text="Check robot and stream", font=big, bg="#1f5fa8",
                             fg="white", width=22)
    check_button.pack(side="left")
    quick_stop = tk.Button(qbuttons, text="STOP", font=big, bg="#b3261e", fg="white", width=10)
    quick_stop.pack(side="left", padx=8)
    quick_sender = ttk.Button(qbuttons, text="Start practice sender")
    ttk.Label(qbuttons, text="STOP = Ctrl-C to the tool: it slows the arms, then releases "
              "them.\nThe real stop is the remote: LT+BACK.", foreground="#b3261e").pack(side="left")
    quick_result = tk.Label(quick, font=big, justify="left", wraplength=1300, anchor="w")
    quick_result.pack(anchor="w", padx=6, pady=4)
    tree = ttk.Treeview(middle, columns=("evidence",), height=12, selectmode="browse")
    tree.heading("#0", text="Step")
    tree.heading("evidence", text="Evidence in this session")
    tree.column("#0", width=260)
    tree.column("evidence", width=520)
    for key, title, _what, _moves in STEPS:
        tree.insert("", "end", iid=key, text=title, values=("",))
    for name, colour in colours.items():
        tree.tag_configure(name, foreground=colour)
    tree.pack(side="left", fill="y")
    panel = ttk.LabelFrame(middle, text="Step")
    panel.pack(side="left", fill="both", expand=True, padx=8)
    what = ttk.Label(panel, wraplength=520, justify="left")
    what.pack(anchor="w", padx=6, pady=4)
    options = ttk.Frame(panel)
    options.pack(anchor="w", padx=6)
    buttons = ttk.Frame(panel)
    buttons.pack(anchor="w", padx=6, pady=6)
    run_button = tk.Button(buttons, text="Run step", font=big, bg="#1f5fa8", fg="white",
                           width=14)
    run_button.pack(side="left")
    stop_button = tk.Button(buttons, text="STOP step", font=big, bg="#b3261e", fg="white",
                            width=12)
    stop_button.pack(side="left", padx=8)
    ttk.Label(buttons, text="= Ctrl-C to the tool: it stops and releases\nthe arms. "
              "The real stop is the remote.", foreground="#b3261e").pack(side="left")
    mirror_stop = ttk.Button(buttons, text="Close mirror")
    why_label = ttk.Label(panel, foreground="#666666")
    why_label.pack(anchor="w", padx=6)
    result_label = tk.Label(panel, font=big, justify="left", wraplength=560)
    result_label.pack(anchor="w", padx=6)
    hint_label = tk.Label(panel, justify="left", wraplength=560, bg="#fff4cc")

    # prompt
    prompt_frame = tk.LabelFrame(root, text="The tool asks", font=big, fg="#1f5fa8")
    prompt_detail = tk.Text(prompt_frame, height=10, wrap="word")
    prompt_text = tk.Label(prompt_frame, font=big, justify="left", wraplength=1300)
    prompt_buttons = ttk.Frame(prompt_frame)
    answer_entry = ttk.Entry(prompt_frame, textvariable=v["answer"], width=60, font=big)

    # live status (step 13), shown above the log
    live_label = tk.Label(root, font=("TkFixedFont", 11, "bold"), justify="left", anchor="w",
                          fg="white", padx=8, pady=4)
    live_bg = {"valid": "#1e7b34", "incomplete": "#b26a00", "info": "#1f5fa8", "none": "#555555",
               "bad": "#b3261e"}

    def show_live(event: dict[str, Any] | None) -> None:
        if event is None:
            live_label.pack_forget()
            return
        kind, text = live_status_text(event)
        live_label.configure(text=text, bg=live_bg[kind])
        if not live_label.winfo_ismapped():
            live_label.pack(fill="x", padx=8, before=logs)

    # log
    logs = ttk.LabelFrame(root, text="Log (also saved in the session's gui-logs folder)")
    logs.pack(fill="both", expand=True, padx=8, pady=4)
    log_tools = ttk.Frame(logs)
    log_tools.pack(fill="x")
    log = tk.Text(logs, wrap="word", height=14, font=("TkFixedFont", 10))
    scroll = ttk.Scrollbar(logs, command=log.yview)
    log.configure(yscrollcommand=scroll.set)
    scroll.pack(side="right", fill="y")
    log.pack(fill="both", expand=True)
    for tag, colour in (("pass", "#1e7b34"), ("fail", "#b3261e"), ("check", "#b26a00"),
                        ("mirror", "#888888"), ("gui", "#1f5fa8")):
        log.tag_configure(tag, foreground=colour)
    log.tag_configure("fail", font=("TkFixedFont", 10, "bold"))

    def write(text: str, tag: str = "") -> None:
        if not tag:
            upper = text.upper()
            tag = ("fail" if any(w in upper for w in ("FAIL", "REFUSED", "ERROR", "TRACEBACK",
                                                       "NOT CONFIRMED"))
                   else "check" if "What to check" in text else "pass" if "PASS" in upper else "")
        log.insert("end", text + "\n", tag)
        log.see("end")

    def folder() -> Path | None:
        return state["session"]

    def open_folder() -> None:
        target = folder() or ROOT
        subprocess.Popen(["xdg-open", str(target)])

    def copy_log() -> None:
        root.clipboard_clear()
        root.clipboard_append(log.get("1.0", "end"))

    ttk.Button(log_tools, text="Open session folder", command=open_folder).pack(side="left")
    ttk.Button(log_tools, text="Copy log", command=copy_log).pack(side="left", padx=4)
    ttk.Button(log_tools, text="Clear", command=lambda: log.delete("1.0", "end")).pack(side="left")
    zip_button = ttk.Button(log_tools, text="Zip session for the developers")
    zip_button.pack(side="left", padx=12)
    ttk.Button(log_tools, text="Open the guide", command=lambda: subprocess.Popen(
        ["xdg-open", str(REPO_ROOT / "docs" / "k1-bringup-runbook.md")])).pack(side="left")
    ttk.Button(log_tools, text="Stage ladder", command=lambda: show_ladder()).pack(side="left",
                                                                                padx=12)

    def maintain(kind: str, work: Any) -> None:
        """Run ``work`` (firewall or zip) off the Tk thread; steps wait until it ends."""
        if busy():
            return
        state["maintenance"] = kind
        write(f"=== {kind}: started", "gui")
        refresh()

        def worker() -> None:
            results.put((kind, *maintenance_result(work)))

        threading.Thread(target=worker, daemon=True).start()

    def allow_firewall() -> None:
        try:
            argv = firewall_command(v["robot_ip"].get())
        except ValueError as exc:
            result_label.configure(text=f"Robot IP: {exc}", fg=colours["bad"])
            return
        maintain("Firewall", lambda: run_privileged(argv))

    def zip_now() -> None:
        session = state["session"]
        if session is None:
            return
        maintain("Zip", lambda: ("done", str(zip_session(session))))

    firewall_button.configure(command=allow_firewall)
    zip_button.configure(command=zip_now)

    # behaviour
    def busy() -> bool:
        return (state["step"] is not None or state["mirror"] is not None
                or state["maintenance"] is not None)

    def set_session(path: Path | None) -> None:
        if path is None:
            state["session"], state["meta"] = None, {}
        else:
            try:
                session = Session(path)
            except CommissionError as exc:
                messagebox.showerror("Not a session", str(exc))
                return
            problem = session_problem(session.meta)
            if problem:
                messagebox.showerror("This session cannot be used", problem)
                return
            state["session"], state["meta"] = Path(path).resolve(), session.meta
        state["live_run"] = None  # the last live run belongs to the previous session
        show_live(None)
        reload_gate()
        refresh()

    def choose_session() -> None:
        if busy():
            return
        chosen = filedialog.askdirectory(initialdir=str(ROOT if ROOT.is_dir() else REPO_ROOT),
                                         title="Choose a session folder (under k1-commissioning)")
        if chosen:
            set_session(Path(chosen))

    open_button.configure(command=choose_session)

    def transport_now() -> str:
        return str(state["meta"].get("transport") or v["transport"].get())

    def refresh() -> None:
        real = transport_now() == "booster"
        banner.configure(
            text=("REAL ROBOT" if real else "PRACTICE (fake robot: nothing moves)")
            + "   ·   The real stop is the remote: LT+BACK (damping)",
            bg="#b3261e" if real else "#555555")
        meta = state["meta"]
        if state["session"] is not None:
            session_label.configure(text=f"Session {state['session'].name}   robot "
                                    f"{meta.get('serial')}   firmware {meta.get('firmware')}   "
                                    f"transport {meta.get('transport')}")
            try:
                rows = evidence(state["session"], v["mode"].get())
            except CommissionError as exc:
                rows = {}
                session_label.configure(text=str(exc))
        else:
            session_label.configure(text="No session yet: Quick start makes one.")
            rows = {}
        if state["preflight"]:
            rows["preflight"] = ("info", f"last run: {state['preflight']}")
        for key, kind, last in (("replay", "replay", state["replay"]),
                                ("live", "live-arms", state["live_run"])):
            found = ladder_lamp(state["ladder"], kind)
            if found or last:
                lamp_kind, text = found or ("info", "")
                rows[key] = (lamp_kind, "  ·  ".join(t for t in (
                    f"last run: {last}" if last else "", f"ledger: {text}" if text else "") if t))
        for key, *_rest in STEPS:
            kind, text = rows.get(key, ("none", "not run" if state["session"] else "no session"))
            tree.item(key, values=(text,), tags=(kind,))
        locked = busy()
        for widget in (open_button, model_button, firewall_button):
            widget.configure(state="disabled" if locked else "normal")
        zip_button.configure(state="disabled" if locked or state["session"] is None else "normal")
        on_net = on_robot_network([v["net"].get()], v["robot_ip"].get())
        net_label.configure(text=(
            f"On the robot's network ({v['robot_ip'].get()}): yes" if on_net else
            "This PC address is not on the robot's network: pick another, or give the PC a "
            "static address in Ubuntu Settings > Network (guide: Before the robot arrives)"))
        firewall = state["firewall"]
        firewall_label.configure(text={
            "active": "Firewall (ufw): ON. The robot's DDS traffic must be allowed:",
            "inactive": "Firewall (ufw): off, nothing to allow.",
            "unknown": "Firewall (ufw): unknown. If it is on, allow the robot network:"}[firewall])
        if firewall == "inactive":
            firewall_button.pack_forget()
        else:
            firewall_button.pack(side="left", padx=8)
        stop_button.configure(state="normal" if state["step"] is not None else "disabled")
        quick_stop.configure(state="normal" if state["step"] is not None else "disabled")
        check_button.configure(state="disabled" if can_run("quick") else "normal",
                               text="Re-check" if state.get("quick_run") else
                               "Check robot and stream")
        if v["transport"].get() == "booster":
            quick_sender.pack_forget()
        else:
            quick_sender.pack(side="left", padx=8, before=quick_stop)
        if state["mirror"] is not None:
            mirror_stop.pack(side="left", padx=8)
        else:
            mirror_stop.pack_forget()
        why = can_run(state["selected"])
        run_button.configure(state="disabled" if why else "normal")
        why_label.configure(text=why or "")
        sender = state["sender"]
        sending = "Stop practice sender" if sender is not None and sender.running \
            else "Start practice sender"
        quick_sender.configure(text=sending)
        if "sender" in live_widgets:
            live_widgets["sender"].configure(text=sending)
        if "evaluate" in live_widgets:
            live_widgets["evaluate"].configure(state="disabled" if why else "normal")

    def can_run(key: str) -> str | None:
        return launch_block(key, state, read_guide=read_guide.get())

    live_widgets: dict[str, Any] = {}

    def reload_gate() -> None:
        """The ladder rows and the live locks of the open session's robot (after each run)."""
        session, meta = state["session"], state["meta"]
        state["ladder"], state["live_lock"] = [], {}
        if session is not None:
            try:
                state["ladder"] = ladder_rows(session_ledger(meta), live.identity_of(meta))
            except (OSError, StageGateError) as exc:
                state["ladder"] = [("acceptance", "Ledger", "invalid", str(exc))]
            if meta.get("transport") == "booster":
                record = acceptance_record(session)
                state["live_lock"] = {stage: live.lock_reasons(session, stage, record)
                                      for stage in ("hoist", "standing")}
        show_lock()
        fill_ladder()

    def show_lock() -> None:
        label = live_widgets.get("lock")
        if label is None or not label.winfo_exists():
            return
        stage = v["live_stage"].get()
        if transport_now() != "booster":
            label.configure(text="Practice robot: never locked; passes go to the PRACTICE "
                            "ledger and unlock nothing.", foreground=colours["info"])
            return
        reasons = state["live_lock"].get(stage) or []
        label.configure(text=(f"{stage}: can start; stage-gate problems show as warnings before "
                              "Go." if not reasons else
                              f"{stage} cannot start:\n  " + "\n  ".join(reasons[:4])
                              + (f"\n  ... and {len(reasons) - 4} more" if len(reasons) > 4 else "")),
                        foreground=colours["valid" if not reasons else "bad"])

    def on_live_stage(*_args: Any) -> None:
        state["live_stage"] = v["live_stage"].get()
        show_lock()
        if "roles" in live_widgets and live_widgets["roles"].winfo_exists():
            live_widgets["roles"].configure(text=f"Roles: {live.ROLES[state['live_stage']]}.")
        refresh()

    v["live_stage"].trace_add("write", on_live_stage)

    def show_ladder() -> None:
        window = state["ladder_window"]
        if window is None or not window.winfo_exists():
            window = tk.Toplevel(root)
            window.title("K1 stage ladder")
            window.geometry("980x300")
            header = ttk.Label(window, font=big, wraplength=940, justify="left")
            header.pack(anchor="w", padx=8, pady=6)
            view = ttk.Treeview(window, columns=("status",), height=5, selectmode="none")
            view.heading("#0", text="Stage")
            view.heading("status", text="Status")
            view.column("#0", width=260)
            view.column("status", width=680)
            for name, colour in colours.items():
                view.tag_configure(name, foreground=colour)
            view.pack(fill="both", expand=True, padx=8)
            ttk.Label(window, foreground="#666666", wraplength=940, justify="left", text=(
                "Each stage needs the one above it. A run's settings (gains, caps, model, stop "
                "settings) are compared with its gate when it starts: step 13 shows the full "
                "reasons. A stage is revoked by an unconfirmed release, a fall or a fault it "
                "forbids.")).pack(anchor="w", padx=8, pady=6)
            state["ladder_window"] = window
            live_widgets["ladder"] = (header, view)
        fill_ladder()
        window.lift()

    def fill_ladder() -> None:
        window = state["ladder_window"]
        if window is None or not window.winfo_exists():
            return
        header, view = live_widgets["ladder"]
        meta = state["meta"]
        if state["session"] is None:
            header.configure(text="No session open: open or start one to see its robot's ladder.")
        else:
            ledger = session_ledger(meta)
            header.configure(text=(
                f"PRACTICE ledger {ledger.path}: practice passes unlock nothing on a real robot."
                if ledger.practice else f"Robot {meta.get('serial')}: ledger {ledger.path}"))
        view.delete(*view.get_children())
        lamp_of = {"passed": "valid", "invalid": "bad", "revoked": "bad", "locked": "incomplete",
                   "ready": "none", "practice": "info"}
        for key, title, kind, text in state["ladder"]:
            view.insert("", "end", iid=key, text=title, values=(text,), tags=(lamp_of[kind],))

    def select(_event: Any = None) -> None:
        chosen = tree.selection()
        if chosen:
            state["selected"] = chosen[0]
        build_options()
        refresh()

    tree.bind("<<TreeviewSelect>>", select)

    def entry(label: str, name: str, width: int = 10, values: Any = None) -> None:
        row = ttk.Frame(options)
        row.pack(anchor="w", pady=2)
        ttk.Label(row, text=label, width=34).pack(side="left")
        if values is not None:
            ttk.Combobox(row, textvariable=v[name], values=values, state="readonly",
                         width=width).pack(side="left")
        else:
            ttk.Entry(row, textvariable=v[name], width=width).pack(side="left")

    def build_options() -> None:
        for child in options.winfo_children():
            child.destroy()
        for name in ("lock", "sender", "evaluate", "roles"):  # destroyed with the options
            live_widgets.pop(name, None)
        key = state["selected"]
        _key, title, text, moves = next(s for s in STEPS if s[0] == key)
        panel.configure(text=title)
        what.configure(text=f"{text}\nRobot moves: {moves}.")
        if key == "hold":
            entry(f"Seconds (max {HOLD_MAX_S:g})", "seconds")
        elif key == "command-loss":
            entry("Gap (s), in order", "gap", values=[f"{g:g}" for g in GAP_LEVELS])
        elif key == "gains":
            entry("Level", "level", values=["L1", "L2", "L3"])
            entry("kp (stiffness)", "kp")
            entry("kd (damping)", "kd")
            ttk.Label(options, text="If the arms lag: kp 30 / kd 1.5, then kp 40 / kd 2, "
                      "starting again at L1.").pack(anchor="w")
        elif key == "record":
            entry("Your name (operator)", "operator", width=30)
            attempts = (valid_gains_attempts(state["session"], v["mode"].get())
                        if state["session"] else [])
            entry("Gains attempt to approve", "gains_attempt", width=40, values=attempts)
        elif key == "replay":
            row = ttk.Frame(options)
            row.pack(anchor="w", pady=2)
            ttk.Label(row, text="Recording (.pkl, qualified)", width=34).pack(side="left")
            ttk.Entry(row, textvariable=v["pkl"], width=22).pack(side="left")
            sessions = Path.home() / "mocap-sessions"
            ttk.Button(row, text="Browse...", command=lambda: v["pkl"].set(
                filedialog.askopenfilename(
                    filetypes=[("K1 motion", "*.pkl")],
                    initialdir=str(sessions if sessions.is_dir() else Path.home()))
                or v["pkl"].get())).pack(side="left")
            newest = newest_preview()
            if newest is not None:
                ttk.Label(options, text=f"Newest K1 preview from the Setup console: {newest}\n"
                          "(a hint only: choose the qualified recording yourself)",
                          foreground="#666666", wraplength=520).pack(anchor="w")
            entry(f"Duration (s, max {REPLAY_MAX_S:g})", "duration")
            entry("Stage (standing needs live arms on the hoist)", "stage",
                  values=["hoist", "standing"])
            ttk.Checkbutton(options, variable=read_guide, text="I have read the guide's "
                            "unverified section and step 12; the recording is qualified.",
                            command=refresh).pack(anchor="w")
        elif key == "live":
            live_options()
        if key in MOTION:
            ttk.Label(options, text="Before you press Run: robot in the hoist, a second person "
                      "on the remote, nobody within reach, the robot in "
                      f"{v['mode'].get()} mode, the mirror open.", foreground="#b3261e",
                      wraplength=520).pack(anchor="w", pady=4)

    def live_options() -> None:
        entry("Stage", "live_stage", values=["hoist", "standing"])
        lock = ttk.Label(options, wraplength=520, justify="left")
        lock.pack(anchor="w", pady=2)
        live_widgets["lock"] = lock
        entry(f"Time limit (s of following, max {live.MAX_TIME_LIMIT_S:g})", "time_limit")
        entry("Stream port (UDP on this PC)", "port")
        entry("Run", "drill", width=18, values=list(LIVE_DRILLS))
        entry("Your name (operator, signs Evaluate)", "operator", width=30)
        roles = ttk.Label(options, wraplength=520, justify="left",
                          text=f"Roles: {live.ROLES[v['live_stage'].get()]}.")
        roles.pack(anchor="w", pady=2)
        live_widgets["roles"] = roles
        practice = transport_now() != "booster"
        ttk.Label(options, foreground="#666666", wraplength=520, justify="left", text=(
            "Stream-loss drill: while the arms follow, stop the stream ("
            + ("Stop practice sender" if practice else "stop the live tool in the console")
            + "). STOP drill: while the arms follow, press STOP step. Evaluate stage judges this "
            "session's attempts and writes the ledger entry.")).pack(anchor="w", pady=2)
        row = ttk.Frame(options)
        row.pack(anchor="w", pady=4)
        evaluate = ttk.Button(row, text="Evaluate stage", command=lambda: launch(evaluate=True))
        evaluate.pack(side="left")
        live_widgets["evaluate"] = evaluate
        if practice:
            sender = ttk.Button(row, text="Start practice sender", command=toggle_sender)
            sender.pack(side="left", padx=8)
            live_widgets["sender"] = sender
        ttk.Button(row, text="Stage ladder", command=show_ladder).pack(side="left", padx=8)
        show_lock()

    def toggle_sender(quick_start: bool = False) -> None:
        sender = state["sender"]
        if sender is not None and sender.running:
            write("=== Practice sender: stopping (the stream ends).", "gui")
            sender.stop()
            state["sender"] = None
            refresh()
            return
        if (v["transport"].get() if quick_start else transport_now()) == "booster" or (
                not quick_start and state["session"] is None):
            return
        try:
            port = int(bounded("port", v["port"].get(), 1024, 65535))
        except ValueError as exc:
            result_label.configure(text=f"Cannot start the sender: {exc}", fg=colours["bad"])
            return
        # the quick start's new session uses the model field; step 13 the open session's model
        model = (v["model"].get().strip() or None) if quick_start else sender_model(state["meta"])
        sender = PracticeSender(port, (state["session"] or ROOT) / "gui-logs" /
                                f"{dt.datetime.now():%Y%m%d-%H%M%S}-sender.log", model)
        try:
            sender.start()
        except OSError as exc:
            result_label.configure(text=f"Cannot start the sender: {exc}", fg=colours["bad"])
            return
        state["sender"] = sender
        if state["step"] is not None and state["step"].step in ("live", "quick"):
            state["step"].companions.append(sender)
        write(f"=== Practice sender: streaming to 127.0.0.1:{port} (log {sender.log_path})", "gui")
        refresh()

    def number(name: str, low: float, high: float) -> str:
        try:
            value = float(v[name].get())
        except ValueError:
            raise ValueError(f"{name}: not a number") from None
        if not low <= value <= high:
            raise ValueError(f"{name}: must be {low:g} to {high:g}")
        return f"{value:g}"

    def command(key: str, evaluate: bool = False) -> tuple[str, list[str]]:
        session, meta = state["session"], state["meta"]
        if key == "quick":
            return "live", quick_command(
                transport=v["transport"].get(), net=v["net"].get(), domain=v["domain"].get(),
                model=v["model"].get(), stage=v["live_stage"].get(),
                time_limit=v["time_limit"].get(), port=v["port"].get(),
                support=v["support"].get(), strap_clear=strap_clear.get(),
                head=head_follows.get() or not has_head, record=v["record"].get(),
                operator=v["operator"].get(), attested=v["attested"].get(),
                firmware=v["firmware"].get())
        model = ["--model", v["model"].get()] if v["model"].get().strip() else []
        attested = (["--attested-robot-id", v["attested"].get().strip()]
                    if v["attested"].get().strip() else [])
        mode = ["--mode", v["mode"].get()]
        if key == "preflight":
            net = ["--net", net_value(v["net"].get())] if net_value(v["net"].get()) else []
            return "commission", ["preflight", "--transport", v["transport"].get(), *net,
                                  "--robot-ip", v["robot_ip"].get().strip(), *model]
        if key == "probe":
            net = ["--net", net_value(v["net"].get())] if net_value(v["net"].get()) else []
            firmware = (["--firmware", v["firmware"].get().strip()]
                        if v["firmware"].get().strip() else [])
            return "commission", ["probe", "--transport", v["transport"].get(), *net,
                                  "--domain-id", str(int(number("domain", 0, 232))),
                                  "--root", str(ROOT), *model, *attested, *firmware]
        base = ["--session", str(session)]
        if key in ("mirror", "zero-check"):
            return "commission", [key, *base]
        if key in ("quick-release", "balance", "signs"):
            return "commission", [key, *base, *mode, *attested]
        if key == "hold":
            return "commission", ["hold", *base, *mode, *attested, "--seconds",
                                  number("seconds", 1, HOLD_MAX_S)]
        if key == "command-loss":
            return "commission", ["command-loss", *base, *mode, *attested, "--gap", v["gap"].get()]
        if key == "gains":
            if v["level"].get() not in ("L1", "L2", "L3"):
                raise ValueError("level: choose L1, L2 or L3")
            return "commission", ["gains", *base, *mode, *attested, "--level", v["level"].get(),
                                  "--kp", number("kp", 1, 200), "--kd", number("kd", 0, 20)]
        if key == "record":
            operator = v["operator"].get().strip()
            if not operator:
                raise ValueError("type your name as the operator")
            chosen = v["gains_attempt"].get().split()
            return "commission", ["record", *base, *mode, "--operator", operator,
                                  *(["--gains-attempt", chosen[0]] if chosen else [])]
        if key == "replay":
            if not read_guide.get():
                raise ValueError("tick the box: you have read the guide's step 12")
            pkl = v["pkl"].get().strip()
            if not pkl or not Path(pkl).is_file():
                raise ValueError("choose the recording (.pkl)")
            if not meta.get("model_path"):
                raise ValueError("the session has no K1 model: start a new session with a model")
            jsonl = session / "gui-logs" / f"replay-{dt.datetime.now():%Y%m%d-%H%M%S}.jsonl"
            args = [pkl, "--transport", meta["transport"], "--model", meta["model_path"],
                    "--duration", number("duration", 1, REPLAY_MAX_S), "--jsonl", str(jsonl),
                    "--session", str(session), "--stage", v["stage"].get(), *attested]
            if v["operator"].get().strip():
                args += ["--operator", v["operator"].get().strip()]
            if meta["transport"] == "booster":
                record = acceptance_record(session)
                if record is None:
                    raise ValueError("no k1-acceptance.json in this session: run 11 · Record")
                if evidence(session, v["mode"].get())["record"][0] == "bad":
                    raise ValueError("the acceptance record is stale: run 11 · Record again")
                args += ["--net", str(meta.get("net", "")), "--domain-id",
                         str(meta.get("domain_id", 0)), "--acceptance-record", str(record),
                         "--yes-i-have-read-the-unverified-section"]
            return "replay", args
        if key == "live":
            record = acceptance_record(session) if meta["transport"] == "booster" else None
            return "live", live_command(
                session, meta, stage=v["live_stage"].get(), time_limit=v["time_limit"].get(),
                port=v["port"].get(), drill=v["drill"].get(), operator=v["operator"].get(),
                evaluate=evaluate, attested=v["attested"].get(), record=record,
                head=head_follows.get() or not has_head)
        raise ValueError(f"unknown step {key}")

    def launch(evaluate: bool = False, key: str | None = None) -> None:
        key = key or state["selected"]
        shown = quick_result if key == "quick" else result_label
        why = can_run(key)
        if why:
            shown.configure(text=why, fg=colours["bad"])
            return
        try:
            tool, args = command(key, evaluate)
        except ValueError as exc:
            shown.configure(text=f"Cannot start: {exc}", fg=colours["bad"])
            return
        logs_dir = (state["session"] or ROOT) / "gui-logs"
        runner = StepRunner(tool, args, logs_dir / f"{dt.datetime.now():%Y%m%d-%H%M%S}-{key}.log",
                            step=key)
        runner.session = state["session"] if key not in ("probe", "quick") else None
        write(f"=== {TITLES[key]}: {' '.join(args)}", "gui")
        try:
            runner.start()
        except OSError as exc:
            shown.configure(text=f"Cannot start: {exc}", fg=colours["bad"])
            return
        state["mirror" if key == "mirror" else "step"] = runner
        if key in ("live", "quick") and state["sender"] is not None:
            runner.companions.append(state["sender"])  # stopped with the step
        if key != "mirror":
            show_live(None)
        shown.configure(text=("Checking the robot and the stream (read-only)..." if key == "quick"
                              else f"{TITLES[key]}: running..."), fg=colours["info"])
        hint_label.pack_forget()
        refresh()

    run_button.configure(command=lambda: launch())
    check_button.configure(command=lambda: launch(key="quick"))
    quick_sender.configure(command=lambda: toggle_sender(quick_start=True))
    def request_stop() -> None:
        if state["step"] is not None:
            write("=== Stop requested (Ctrl-C): the tool stops and releases the arms.", "gui")
            state["step"].stop()

    stop_button.configure(command=request_stop)
    quick_stop.configure(command=request_stop)

    def clear_go(*_args: Any) -> None:
        """The Go belongs to the settings on the screen: a change cancels the open Go."""
        runner = state["step"]
        prompt = runner.prompt if runner is not None and runner.step == "quick" else None
        if prompt is not None and not prompt.answered and runner.answer(prompt.id, "no"):
            write("=== A setting changed: the Go is cleared. Press Re-check.", "gui")
            quick_result.configure(text="A setting changed: the Go is cleared. Press Re-check.",
                                   fg=colours["bad"])
            show_prompt(runner)

    for name in ("live_stage", "time_limit", "port", "support", "record", "operator", "transport",
                 "net", "domain", "model", "attested", "firmware"):
        v[name].trace_add("write", clear_go)
    for flag in (strap_clear, head_follows):
        flag.trace_add("write", clear_go)
    mirror_stop.configure(command=lambda: state["mirror"] and state["mirror"].stop())

    def show_prompt(runner: StepRunner) -> None:
        prompt = runner.prompt
        for child in prompt_buttons.winfo_children():
            child.destroy()
        answer_entry.pack_forget()
        prompt_detail.pack_forget()
        if prompt is None or prompt.answered:
            prompt_frame.pack_forget()
            return
        prompt_frame.pack(fill="x", padx=8, pady=4, before=logs)
        go = prompt.kind == "confirm" and runner.step in GO_STEPS
        prompt_text.configure(text=(
            "Read the summary above. Go starts the robot; a changed setting clears the Go. "
            "(In this window, Ctrl-C is the red STOP button.)" if go else
            "Read the plan and the checklist above. Is all of it true? "
            "(In this window, Ctrl-C is the red STOP button.)" if prompt.kind == "confirm"
            else prompt.text))
        if prompt.detail:
            prompt_detail.configure(state="normal", height=min(20, prompt.detail.count("\n") + 2))
            prompt_detail.delete("1.0", "end")
            prompt_detail.insert("1.0", prompt.detail)
            prompt_detail.configure(state="disabled")
            prompt_detail.pack(fill="x", padx=6)
        prompt_text.pack(anchor="w", padx=6, pady=4)
        prompt_buttons.pack(anchor="w", padx=6, pady=6)

        def send(text: str) -> None:
            if runner.answer(prompt.id, text):
                for child in prompt_buttons.winfo_children():
                    child.configure(state="disabled")  # type: ignore[call-arg, call-overload]
                answer_entry.configure(state="disabled")
            else:
                write("=== That answer was not sent: the question is no longer open.", "gui")

        if prompt.kind == "confirm":
            tk.Button(prompt_buttons, text="Go" if go else "All of the above is true: start",
                      font=big, bg="#1e7b34", fg="white", width=10 if go else 0,
                      command=lambda: send("yes")).pack(side="left")
            tk.Button(prompt_buttons, text="Cancel", font=big,
                      command=lambda: send("no")).pack(side="left", padx=8)
        elif prompt.kind == "yesno":
            tk.Button(prompt_buttons, text="Yes", font=big, width=8,
                      command=lambda: send("y")).pack(side="left")
            tk.Button(prompt_buttons, text="No", font=big, width=8,
                      command=lambda: send("n")).pack(side="left", padx=8)
        else:
            v["answer"].set("")
            answer_entry.configure(state="normal")
            answer_entry.pack(in_=prompt_buttons, side="left")
            answer_entry.focus_set()
            submit = ttk.Button(prompt_buttons, text="Send",
                                command=lambda: send(v["answer"].get().strip()))
            submit.pack(side="left", padx=8)
            answer_entry.bind("<Return>", lambda _e: submit.invoke())

    def finish(runner: StepRunner, slot: str) -> None:
        state[slot] = None
        outcome = runner.result()
        key = runner.step
        if key == "probe" and runner.session is not None:
            target = runner.session / "gui-logs"
            target.mkdir(exist_ok=True)
            shutil.copy2(runner.log_path, target / runner.log_path.name)
            set_session(runner.session)
            write(f"=== New session: {runner.session}", "gui")
        elif key == "probe":
            write("=== Probe failed: no session was created; the open session is unchanged.",
                  "fail")
        if key == "quick":
            if runner.session is not None:
                target = runner.session / "gui-logs"
                target.mkdir(exist_ok=True)
                shutil.copy2(runner.log_path, target / runner.log_path.name)
                set_session(runner.session)
            state["quick_run"] = f"{outcome.verdict} {dt.datetime.now():%H:%M}"
            why = refusal_text(runner.lines)
            quick_result.configure(
                text=(f"Quick start: {outcome.verdict}" + (f": {why}" if why else "")
                      + ("\nFix what is named above, then press Re-check."
                         if outcome.verdict == "REFUSED" else "")),
                fg=colours["valid"] if outcome.verdict in ("PASS", "DONE") else colours["bad"])
        if key == "preflight":
            state["preflight"] = f"{outcome.verdict} {dt.datetime.now():%H:%M}"
        if key == "replay":
            state["replay"] = f"{outcome.verdict} {dt.datetime.now():%H:%M}"
        if key in ("live", "quick"):
            state["live_run"] = f"{outcome.verdict} {dt.datetime.now():%H:%M}"
        if state["sender"] is not None and not state["sender"].running:
            state["sender"] = None
        if key != "mirror":
            reload_gate()  # a run may have written a PASS or a revocation
        colour = colours["valid"] if outcome.verdict in ("PASS", "DONE") else colours["bad"]
        result_label.configure(text=f"{TITLES.get(key, key)}: {outcome.verdict}"
                               + (f" (attempt {runner.attempt['id']})" if runner.attempt else ""),
                               fg=colour)
        write(f"=== {TITLES.get(key, key)} finished: {outcome.verdict} "
              f"(exit {runner.process.returncode if runner.process else '?'}); "
              f"log {runner.log_path}", "gui")
        if outcome.hint:
            hint_label.configure(text=f"What to check: {outcome.hint}")
            hint_label.pack(anchor="w", padx=6, pady=4, fill="x")
        if outcome.alarm and runner.live is not None and runner.live.get("state") != "released":
            show_live({**runner.live, "state": "unknown: the run ended abnormally"})  # red
        if outcome.alarm:
            state["alarm"] = True
            state["closing"] = False
            alarm_label.configure(text=outcome.alarm)
            alarm.pack(fill="x", after=banner)
            write(outcome.alarm, "fail")
        show_prompt(runner)
        refresh()

    def tick() -> None:
        while not results.empty():
            kind, outcome, text = results.get()
            state["maintenance"] = None
            message = {"done": "done", "cancelled": "cancelled (no change)",
                       "failed": "FAILED"}.get(outcome, outcome)
            write(f"=== {kind}: {message}" + (f": {text}" if text else ""),
                  "fail" if outcome == "failed" else "gui")
            if kind == "Firewall":
                state["firewall"] = firewall_state()
            if kind == "Firewall" and outcome == "failed":
                write("Could not ask for your password. Run this in a terminal instead: sudo "
                      + " ".join(firewall_command(v["robot_ip"].get())[1:]), "fail")
            if kind == "Zip" and outcome == "done":
                subprocess.Popen(["xdg-open", str(Path(text).parent)])
            refresh()
        for slot in ("step", "mirror"):
            runner = state[slot]
            if runner is None:
                continue
            for kind, item in runner.poll():
                if kind == "event" and item.get("event") == "live":
                    show_live(item)
                if kind == "line":
                    write(("[mirror] " + item) if slot == "mirror" else item,
                          "mirror" if slot == "mirror" else "")
            if slot == "step":
                show_prompt(runner)
            if runner.finished:
                finish(runner, slot)
        sender = state["sender"]
        if sender is not None and not sender.running:
            state["sender"] = None
            write("=== Practice sender stopped.", "gui")
            refresh()
        if state["closing"] and not busy() and not state["alarm"]:
            if state["sender"] is not None:
                state["sender"].stop()
            root.destroy()
            return
        root.after(100, tick)

    def on_close() -> None:
        if state["alarm"]:
            messagebox.showwarning("Alarm open", "Acknowledge the red alarm first.")
            return
        if busy():
            if not messagebox.askyesno("A step is running", "A step or the mirror is running. "
                                       "Request a stop (the tool releases the arms) and close "
                                       "when it has ended?"):
                return
            for slot in ("step", "mirror"):
                if state[slot] is not None:
                    state[slot].stop()
        state["closing"] = True

    root.protocol("WM_DELETE_WINDOW", on_close)
    tree.selection_set("preflight")
    select()
    tabs.select(quick)
    root.after(100, tick)
    root.mainloop()
    return 0


def main(argv: list[str] | None = None) -> int:  # pragma: no cover
    if not sys.platform.startswith("linux"):
        print("The K1 test GUI runs on Linux only (the Booster SDK is Linux only).",
              file=sys.stderr)
        return 2
    try:
        import tkinter  # noqa: F401
    except ImportError:
        print("Tk is missing: sudo apt install python3-tk", file=sys.stderr)
        return 2
    lock = take_instance_lock()
    if lock is None:
        message = "K1 test window: already open, or an install is running"
        print(message, file=sys.stderr)
        subprocess.run(["notify-send", "K1 hardware tests", message], check=False,
                       capture_output=True)
        return 3
    try:
        return run_gui()
    finally:
        lock.close()


if __name__ == "__main__":
    raise SystemExit(main())
