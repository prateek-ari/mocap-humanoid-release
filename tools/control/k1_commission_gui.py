#!/usr/bin/env python3
"""Click-through GUI for the Booster K1 hardware tests (the 12 steps of
docs/k1-bringup-runbook.md).

    .venv/bin/python tools/control/k1_commission_gui.py

It never moves the robot itself: each step runs the unchanged command-line tool
(tools/control/k1_commission.py, and tools/control/k1_arm_bridge.py for step 12)
in a child process through tools/control/k1_gui_child.py.  The GUI turns the
tool's questions into buttons, shows its output live, explains failures, and
raises a red alarm when a release is not confirmed or a run ends abnormally.
The real stop is always the Booster remote: LT+BACK (damping).

Linux only (the Booster SDK is Linux only); needs Tk (Ubuntu: ``sudo apt install python3-tk``).
"""

from __future__ import annotations

from dataclasses import dataclass, field
import datetime as dt
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
    DEFAULT_ROOT,
    GAP_LEVELS,
    HOLD_MAX_S,
    ROBOT_IP,
    SAFE,
    CommissionError,
    Session,
    hint,
    ladder,
)
from tools.control.k1_gui_child import MARK  # noqa: E402

CHILD = Path(__file__).with_name("k1_gui_child.py")
ROOT = REPO_ROOT / DEFAULT_ROOT
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
]
TITLES = {key: title for key, title, _what, _moves in STEPS}

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
        return items

    def _handle(self, event: dict[str, Any]) -> None:
        kind = event.get("event")
        if kind == "prompt":
            self.prompt = Prompt(int(event["id"]), str(event["kind"]), str(event["text"]),
                                 str(event.get("detail", "")))
        elif kind == "session":
            self.session = Path(event["path"])
        elif kind == "attempt":
            self.attempt = event
        elif kind == "rpc":
            self.rpc.append(bool(event["enable"]))
        elif kind == "exit":
            self.bridge_state = event.get("bridge_state")

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
                verdict = "PASS" if self.step in ("preflight", "replay") else "DONE"
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


def valid_gains_attempts(path: Path, mode: str) -> list[str]:
    session = Session(path)
    return [f"{a['id']}  {a['step'].split('@', 2)[2]}" for a in session.attempts()
            if str(a.get("step", "")).startswith(f"gains@{mode}@")
            and lamp(session, a["step"])[0] == "valid"
            and session.latest(a["step"]).get("id") == a.get("id")]  # type: ignore[union-attr]


# -- the window -------------------------------------------------------------------


def run_gui() -> int:  # pragma: no cover - exercised by hand and by the smoke test
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
                             "preflight": None, "replay": None}
    v = {name: tk.StringVar(value=value) for name, value in {
        "transport": "fake", "net": "", "robot_ip": ROBOT_IP, "domain": "0", "model": "",
        "attested": "", "firmware": "", "mode": "walking", "seconds": "10",
        "gap": f"{GAP_LEVELS[0]:g}", "level": "L1", "kp": f"{SAFE['kp']:g}",
        "kd": f"{SAFE['kd']:g}", "operator": "", "gains_attempt": "", "pkl": "",
        "duration": "10", "answer": ""}.items()}
    read_guide = tk.BooleanVar(value=False)
    try:
        from tools.control.k1_arm_bridge import resolve_model
        found = resolve_model(None)
        v["model"].set(str(found) if found else "")
    except Exception:  # noqa: BLE001 - only a default for a field
        pass

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
                                                                    width=16))
        widget.grid(row=1, column=column, sticky="we", padx=4)
    ttk.Label(bar, text="K1 model (K1_serial.xml)").grid(row=2, column=0, sticky="w", padx=4)
    ttk.Entry(bar, textvariable=v["model"], width=80).grid(row=3, column=0, columnspan=5,
                                                           sticky="we", padx=4)
    model_button = ttk.Button(bar, text="Browse...", command=lambda: v["model"].set(
        filedialog.askopenfilename(filetypes=[("MuJoCo XML", "*.xml")]) or v["model"].get()))
    model_button.grid(row=3, column=5, sticky="w")
    open_button = ttk.Button(bar, text="Open session...")
    open_button.grid(row=3, column=6, sticky="e", padx=4)
    session_label = ttk.Label(bar, text="No session: run 1 · Preflight, then 2 · Probe.",
                              font=big)
    session_label.grid(row=4, column=0, columnspan=7, sticky="w", padx=4, pady=4)
    v["mode"].trace_add("write", lambda *_: refresh())
    v["transport"].trace_add("write", lambda *_: refresh())

    # middle: steps, step panel
    middle = ttk.Frame(root)
    middle.pack(fill="both", expand=False, padx=8)
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

    # behaviour
    def busy() -> bool:
        return state["step"] is not None or state["mirror"] is not None

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
            session_label.configure(text="No session: run 1 · Preflight, then 2 · Probe.")
            rows = {}
        if state["preflight"]:
            rows["preflight"] = ("info", f"last run: {state['preflight']}")
        if state["replay"]:
            rows["replay"] = ("info", f"last run: {state['replay']}")
        for key, *_rest in STEPS:
            kind, text = rows.get(key, ("none", "not run" if state["session"] else "no session"))
            tree.item(key, values=(text,), tags=(kind,))
        locked = busy()
        for widget in (open_button, model_button):
            widget.configure(state="disabled" if locked else "normal")
        stop_button.configure(state="normal" if state["step"] is not None else "disabled")
        if state["mirror"] is not None:
            mirror_stop.pack(side="left", padx=8)
        else:
            mirror_stop.pack_forget()
        why = can_run(state["selected"])
        run_button.configure(state="disabled" if why else "normal")
        why_label.configure(text=why or "")

    def can_run(key: str) -> str | None:
        if state["alarm"]:
            return "Acknowledge the red alarm first."
        if key == "mirror":
            if state["mirror"] is not None:
                return "The mirror is already open."
        elif state["step"] is not None:
            return "Another step is running."
        if key not in ("preflight", "probe") and state["session"] is None:
            return "Needs a session: run 2 · Probe or open a session."
        if key == "probe" and state["mirror"] is not None:
            return "Close the mirror before a new probe."
        if key == "replay" and not read_guide.get():
            return "Tick the box below the duration first."
        return None

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
            ttk.Button(row, text="Browse...", command=lambda: v["pkl"].set(
                filedialog.askopenfilename(filetypes=[("K1 motion", "*.pkl")])
                or v["pkl"].get())).pack(side="left")
            entry(f"Duration (s, max {REPLAY_MAX_S:g})", "duration")
            ttk.Checkbutton(options, variable=read_guide, text="I have read the guide's "
                            "unverified section and step 12; the recording is qualified.",
                            command=refresh).pack(anchor="w")
        if key in MOTION:
            ttk.Label(options, text="Before you press Run: robot in the hoist, a second person "
                      "on the remote, nobody within reach, the robot in "
                      f"{v['mode'].get()} mode, the mirror open.", foreground="#b3261e",
                      wraplength=520).pack(anchor="w", pady=4)

    def number(name: str, low: float, high: float) -> str:
        try:
            value = float(v[name].get())
        except ValueError:
            raise ValueError(f"{name}: not a number") from None
        if not low <= value <= high:
            raise ValueError(f"{name}: must be {low:g} to {high:g}")
        return f"{value:g}"

    def command(key: str) -> tuple[str, list[str]]:
        session, meta = state["session"], state["meta"]
        model = ["--model", v["model"].get()] if v["model"].get().strip() else []
        attested = (["--attested-robot-id", v["attested"].get().strip()]
                    if v["attested"].get().strip() else [])
        mode = ["--mode", v["mode"].get()]
        if key == "preflight":
            net = ["--net", v["net"].get().strip()] if v["net"].get().strip() else []
            return "commission", ["preflight", "--transport", v["transport"].get(), *net,
                                  "--robot-ip", v["robot_ip"].get().strip(), *model]
        if key == "probe":
            net = ["--net", v["net"].get().strip()] if v["net"].get().strip() else []
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
                    *attested]
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
        raise ValueError(f"unknown step {key}")

    def launch() -> None:
        key = state["selected"]
        why = can_run(key)
        if why:
            result_label.configure(text=why, fg=colours["bad"])
            return
        try:
            tool, args = command(key)
        except ValueError as exc:
            result_label.configure(text=f"Cannot start: {exc}", fg=colours["bad"])
            return
        logs_dir = (state["session"] or ROOT) / "gui-logs"
        runner = StepRunner(tool, args, logs_dir / f"{dt.datetime.now():%Y%m%d-%H%M%S}-{key}.log",
                            step=key)
        runner.session = state["session"] if key != "probe" else None
        write(f"=== {TITLES[key]}: {' '.join(args)}", "gui")
        try:
            runner.start()
        except OSError as exc:
            result_label.configure(text=f"Cannot start: {exc}", fg=colours["bad"])
            return
        state["mirror" if key == "mirror" else "step"] = runner
        result_label.configure(text=f"{TITLES[key]}: running...", fg=colours["info"])
        hint_label.pack_forget()
        refresh()

    run_button.configure(command=launch)
    def request_stop() -> None:
        if state["step"] is not None:
            write("=== Stop requested (Ctrl-C): the tool stops and releases the arms.", "gui")
            state["step"].stop()

    stop_button.configure(command=request_stop)
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
        prompt_text.configure(text=(
            "Read the plan and the checklist above. Is all of it true? "
            "(In this window, Ctrl-C is the red STOP button.)" if prompt.kind == "confirm"
            else prompt.text))
        if prompt.detail:
            prompt_detail.configure(state="normal")
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
            tk.Button(prompt_buttons, text="All of the above is true: start", font=big,
                      bg="#1e7b34", fg="white", command=lambda: send("yes")).pack(side="left")
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
        if key == "preflight":
            state["preflight"] = f"{outcome.verdict} {dt.datetime.now():%H:%M}"
        if key == "replay":
            state["replay"] = f"{outcome.verdict} {dt.datetime.now():%H:%M}"
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
        if outcome.alarm:
            state["alarm"] = True
            state["closing"] = False
            alarm_label.configure(text=outcome.alarm)
            alarm.pack(fill="x", after=banner)
            write(outcome.alarm, "fail")
        show_prompt(runner)
        refresh()

    def tick() -> None:
        for slot in ("step", "mirror"):
            runner = state[slot]
            if runner is None:
                continue
            for kind, item in runner.poll():
                if kind == "line":
                    write(("[mirror] " + item) if slot == "mirror" else item,
                          "mirror" if slot == "mirror" else "")
            if slot == "step":
                show_prompt(runner)
            if runner.finished:
                finish(runner, slot)
        if state["closing"] and not busy() and not state["alarm"]:
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
    return run_gui()


if __name__ == "__main__":
    raise SystemExit(main())
