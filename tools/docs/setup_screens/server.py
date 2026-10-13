"""Serve the Setup console with fake tools for documentation screenshots.

Usage: python server.py WORKDIR REPO PORT (started by capture.py)
WORKDIR gets a fake home with a fake checkout (~/mocap_humanoid), runtime and BVH
files, so the real checkout's session.env is never touched. Each tool lives at its
real path and prints realistic output, so the job logs read like a real run.
"""
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

work, repo, port = Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve(), int(sys.argv[3])
sys.path.insert(0, str(repo))
import uvicorn  # noqa: E402
from mocap_g1.operator.app import create_app  # noqa: E402
from mocap_g1.operator.setup_workbench import SetupWorkbench  # noqa: E402

PY = sys.executable
home = work / "home"
fake = home / "mocap_humanoid"
runtime = home / "mocap-runtime"
# Local sample videos (ignored artifacts) copied in place of a real render.
SAMPLE = {"unitree_g1": repo / "artifacts/samples/unitree_g1-preview.mp4",
          "booster_t1_29dof": repo / "artifacts/samples/booster_t1_29dof-preview.mp4"}


def write(path: Path, text: str, mode=0o755):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    path.chmod(mode)


wrapper = f'#!/bin/sh\nexec {PY} "$@"\n'
write(fake / ".venv/bin/python", wrapper)
write(runtime / "venv/bin/python", wrapper)
(runtime / "GMR").mkdir(parents=True, exist_ok=True)

write(fake / "tools/checks/setup_doctor.py", r'''
import json, sys
if "--json" in sys.argv:
    print(json.dumps({"ok": True, "profile": "core", "checks": []})); raise SystemExit
for line in ["PASS python 3.12.3 in .venv", "PASS docker daemon reachable; nvidia runtime registered",
             "PASS GPU NVIDIA GeForce RTX 4090", "PASS desktop display :0 with Xauthority",
             "PASS image gmr-live:bb1bbe4", "PASS XSENS_SOURCE_IP is set"]:
    print(line)
''')

write(fake / "tools/xsens/live_operator.sh", f'#!/bin/sh\nexec {PY} "$(dirname "$0")/_fake_live_operator.py" "$@"\n')
write(fake / "tools/xsens/_fake_live_operator.py", r'''
import json, os, pathlib, sys, time
action, args = sys.argv[1], sys.argv[2:]
src = os.environ["XSENS_SOURCE_IP"]
if action == "acceptance":
    out = pathlib.Path(args[args.index("--output") + 1])
    print("Listening for Xsens UDP on 0.0.0.0:5555 for 30 s...", flush=True)
    for i in range(1, 31):
        time.sleep(1)
        if i % 5 == 0:
            print(f"{i:2d} s  body frames {i*60:4d}  rate 60.0 Hz  loss 0.0%  joint angles OK", flush=True)
    out.write_text(json.dumps({"verdict": "PASS", "failures": [],
                               "body": {"valid_datagrams": 1800, "observed_receive_hz": 60.0}}))
    print('  "verdict": "PASS"')
    raise SystemExit
robot = "Unitree G1" if args[args.index("--robot") + 1] == "unitree_g1" else "Booster T1"
print(f"Waiting for strict MXTP02 on 0.0.0.0:5555 from {src}...", flush=True)
time.sleep(0.6)
print(f"Receiving Xsens body frames from {src}:50124")
print(f"MuJoCo {robot} preview screening: joint ranges, stale=0.1s, hard=0.5s. No hardware velocity envelope: simulation only.")
print("Viewer controls: Space enables/disables bounded waist/arm tracking; R resets/rearms from the current MuJoCo pose.")
print("Arms: npose orientation. Stand still in a neutral pose (arms down) and press N to calibrate arm orientation and floor height.")
if "--calibrate-after" in args:
    print(f"Automatic N calibration in {args[args.index('--calibrate-after') + 1]} s: the performer should stand still with arms down.")
print("Scope: simulator-only waist/arm command screening; legs and floating base are still a kinematic preview.", flush=True)
time.sleep(1.0)
print("NEUTRAL CALIBRATION: arm orientation and floor height. Press N again in a neutral pose to redo it.", flush=True)
n = 0
while True:
    time.sleep(1.0); n += 1
    print(f"live {n*60:5d} frames  60.0 Hz  input age 9 ms  tracking ON", flush=True)
''')

write(fake / "tools/retarget/retarget_robot_recorded.py", r'''
import json, pathlib, sys, time
a = sys.argv
out = pathlib.Path(a[a.index("--save-path") + 1]); robot = a[a.index("--robot") + 1]
start, end = int(a[a.index("--start") + 1]), int(a[a.index("--end") + 1])
mode = a[a.index("--arm-mode") + 1]
print(f"Loading {pathlib.Path(a[a.index('--bvh-file') + 1]).name}: 1500 frames at 60 Hz, scale {a[a.index('--scale') + 1]}", flush=True)
for i in range(start, end + 1, 150):
    time.sleep(0.3); print(f"retarget frame {i}/{end}", flush=True)
out.write_bytes(bytes(186_000))
print(json.dumps({"robot": robot, "frames": end - start, "fps": 60.0, "arm_mode": mode, "output": out.name}, indent=2))
''')

write(fake / "tools/retarget/render_robot_motion.py", f'''
import os, shutil, sys, time
a = sys.argv
robot = a[a.index("--robot") + 1]
samples = {{"unitree_g1": {str(SAMPLE["unitree_g1"])!r}, "booster_t1_29dof": {str(SAMPLE["booster_t1_29dof"])!r}}}
if "--record-video" not in a:
    print("MuJoCo viewer open; it closes after one loop.", flush=True); time.sleep(3600)
for i in range(0, 601, 150):
    time.sleep(0.3); print(f"rendered frame {{i}}/600", flush=True)
video = a[a.index("--video-path") + 1]
shutil.copy(samples[robot], video)
print(f"PASS: rendered 600 {{robot}} frame(s)")
print("video: " + video.replace({str(home)!r}, "~", 1))
''')


def bvh(frames: int) -> str:
    return ("HIERARCHY\nROOT Hips\n{\n  OFFSET 0 0 0\n  CHANNELS 6 Xposition Yposition Zposition "
            "Zrotation Xrotation Yrotation\n  End Site\n  {\n    OFFSET 0 10 0\n  }\n}\nMOTION\n"
            f"Frames: {frames}\nFrame Time: 0.0166667\n" + "0 90 0 0 0 0\n" * frames)


write(home / "mocap-data/motion.bvh", bvh(1500), 0o644)
write(home / "mocap-data/neutral.bvh", bvh(420), 0o644)

if not (fake / ".git").exists():
    env = {**os.environ, "GIT_AUTHOR_NAME": "docs", "GIT_AUTHOR_EMAIL": "docs@example.invalid",
           "GIT_COMMITTER_NAME": "docs", "GIT_COMMITTER_EMAIL": "docs@example.invalid"}
    subprocess.run(["git", "init", "-q", str(fake)], check=True)
    subprocess.run(["git", "-C", str(fake), "add", "-A"], check=True)
    subprocess.run(["git", "-C", str(fake), "commit", "-qm", "docs"], check=True, env=env)

venv_python = str(fake / ".venv/bin/python")
doctor = str(fake / "tools/checks/setup_doctor.py")
workbench = SetupWorkbench(
    fake, home=home,
    command_overrides={
        # The default doctor/save use the console's own interpreter; show the checkout's.
        "doctor": lambda s, o: [venv_python, doctor, "live"] if s["route"] == "live"
        else [venv_python, doctor, "recorded", "--workspace", s["mocap_ws"], "--robot", s["robot"]],
        "save": lambda s, o: [venv_python, doctor, "core", "--with-operator", "--json"],
    },
    interfaces=lambda: [{"name": "enp4s0", "ipv4": "192.168.1.20"},
                        {"name": "wlp5s0", "ipv4": "10.0.0.37"}],
    route_source=lambda ip: "192.168.1.20",
    stop_container=lambda: None,
    stop_grace_s=1.0,
    display_available=lambda: True,
    viewer_running=lambda: False,
    udp_port_busy=lambda port: False,
)
app = create_app(data_root=work / "data", setup_root=workbench, port=port)
uvicorn.run(app, host="127.0.0.1", port=port, log_level="warning")
