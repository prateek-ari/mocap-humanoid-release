"""Setup workspace: session.env round trip, guide-step jobs and the HTTP contract."""

from __future__ import annotations

from importlib.util import find_spec
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[1]
NEW_SESSION = REPO_ROOT / "tools" / "setup" / "new_session.sh"
PYTHON = sys.executable
LOOPBACK = "http://127.0.0.1:8766"
JSON_HEADERS = {"Content-Type": "application/json", "Origin": LOOPBACK}
_API_TESTS_AVAILABLE = find_spec("fastapi") is not None and find_spec("httpx") is not None


def python_command(code: str):
    """A command override running a tiny Python snippet."""

    return lambda _session, _options: [PYTHON, "-c", code]


class SetupFixture:
    """A throwaway HOME and source checkout (paths contain spaces)."""

    def make_tree(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        base = Path(self.temporary.name)
        self.home = base / "home dir"
        self.home.mkdir()
        self.repo = base / "check out"
        (self.repo / "tools" / "xsens").mkdir(parents=True)
        (self.repo / "tools" / "xsens" / "live_operator.sh").write_text("#!/bin/sh\nexit 2\n")
        (self.repo / "tools" / "setup").mkdir(parents=True)
        shutil.copy2(NEW_SESSION, self.repo / "tools" / "setup" / "new_session.sh")
        self.session_dir = self.home / "mocap sessions" / "take 1"
        self.stopped_containers: list[str] = []
        self.display = True
        self.viewer = False
        self.busy_ports: list[int] = []

    def workbench(self, overrides=None, **kwargs):
        from mocap_g1.operator.setup_workbench import SetupWorkbench

        return SetupWorkbench(
            self.repo,
            home=self.home,
            command_overrides=overrides or {},
            interfaces=lambda: [{"name": "enp3s0", "ipv4": "192.168.1.20"}],
            route_source=lambda _ip: "192.168.1.20",
            stop_container=lambda: self.stopped_containers.append("gmr-xsens-live"),
            display_available=lambda: self.display,
            viewer_running=lambda: self.viewer,
            udp_port_busy=lambda port: port in self.busy_ports,
            stop_grace_s=0.5,
            **kwargs,
        )

    def values(self, **changes):
        values = {
            "route": "recorded",
            "session_dir": str(self.session_dir),
            "robot": "unitree_g1",
            "human_height_m": 1.82,
            "mocap_ws": str(self.home / "mocap runtime"),
            "motion_bvh": str(self.home / "mocap data" / "motion take.bvh"),
            "npose_bvh": str(self.home / "mocap data" / "neutral take.bvh"),
            "offsets_file": "",
            "bvh_scale": 0.01,
            "start_frame": 1,
            "end_frame": 600,
            "frame_step": 1,
            "npose_start_s": 2.0,
            "npose_duration_s": 1.0,
            "windows_xsens_ip": "192.168.1.246",
            "linux_xsens_ip": "192.168.1.20",
        }
        values.update(changes)
        return values

    def make_runtime(self) -> Path:
        """A fake recorded runtime whose Python accepts any arguments.

        Like the real retargeter it writes the ``--save-path`` output.
        """

        python = self.home / "mocap runtime" / "venv" / "bin" / "python"
        python.parent.mkdir(parents=True, exist_ok=True)
        python.write_text(
            "#!/bin/sh\n"
            'while [ $# -gt 0 ]; do [ "$1" = --save-path ] && : > "$2"; shift; done\n'
            "exit 0\n"
        )
        python.chmod(0o755)
        (self.home / "mocap runtime" / "GMR").mkdir(exist_ok=True)
        return python

    def source(self, path: Path, *names: str) -> list[str]:
        """Source ``path`` in a clean bash and print the named variables."""

        script = 'source "$1"; ' + "; ".join(f'printf "%s\\n" "${name}"' for name in names)
        return subprocess.run(
            ["/bin/bash", "-c", script, "-", str(path)],
            env={"PATH": os.environ["PATH"]},
            text=True,
            capture_output=True,
            check=True,
        ).stdout.splitlines()


class SessionFileTests(SetupFixture, unittest.TestCase):
    """The session.env reader/writer, without the web layer."""

    def setUp(self):
        self.make_tree()
        self.addCleanup(self.temporary.cleanup)

    def test_written_file_sources_to_the_same_values_in_bash(self):
        bench = self.workbench()
        bench.save_session(self.values())
        link = self.repo / "session.env"
        self.assertTrue(link.is_symlink())
        self.assertEqual(os.readlink(link), str(self.session_dir / "session.env"))
        text = (self.session_dir / "session.env").read_text()
        self.assertIn("by the operator console Setup tab", text.splitlines()[0])
        self.assertFalse(list(self.session_dir.glob(".session.env.tmp*")))

        names = ["SESSION_ROUTE", "SESSION_DIR", "ROBOT", "HUMAN_HEIGHT_M", "MOCAP_WS",
                 "IK_CONFIG", "MOTION_BVH", "NPOSE_BVH", "OFFSETS_FILE", "BVH_SCALE",
                 "START_FRAME", "END_FRAME", "FRAME_STEP", "NPOSE_START_S", "NPOSE_DURATION_S",
                 "WINDOWS_XSENS_IP", "LINUX_XSENS_IP", "XSENS_SOURCE_IP", "XSENS_UDP_PORT"]
        sourced = dict(zip(names, self.source(link, *names)))
        runtime = str(self.home / "mocap runtime")
        self.assertEqual(sourced, {
            "SESSION_ROUTE": "recorded",
            "SESSION_DIR": str(self.session_dir),
            "ROBOT": "unitree_g1",
            "HUMAN_HEIGHT_M": "1.82",
            "MOCAP_WS": runtime,
            "IK_CONFIG": f"{runtime}/GMR/general_motion_retargeting/ik_configs/bvh_xsens_to_g1.json",
            "MOTION_BVH": str(self.home / "mocap data" / "motion take.bvh"),
            "NPOSE_BVH": str(self.home / "mocap data" / "neutral take.bvh"),
            "OFFSETS_FILE": "",
            "BVH_SCALE": "0.01",
            "START_FRAME": "1",
            "END_FRAME": "600",
            "FRAME_STEP": "1",
            "NPOSE_START_S": "2.0",
            "NPOSE_DURATION_S": "1.0",
            "WINDOWS_XSENS_IP": "192.168.1.246",
            "LINUX_XSENS_IP": "192.168.1.20",
            "XSENS_SOURCE_IP": "192.168.1.246",
            "XSENS_UDP_PORT": "5555",
        })
        # The reader parses the same values back without executing anything.
        self.assertEqual(bench.current_session(), self.values())

    def test_t1_ik_config_lives_in_the_checkout_and_metres_scale_is_one(self):
        bench = self.workbench()
        bench.save_session(self.values(robot="booster_t1_29dof", bvh_scale=1))
        ik_config, scale = self.source(self.repo / "session.env", "IK_CONFIG", "BVH_SCALE")
        self.assertEqual(
            ik_config, str(self.repo / "retarget_configs" / "bvh_xsens_to_booster_t1_29dof.json")
        )
        self.assertEqual(scale, "1")

    def test_reads_a_file_written_by_new_session_sh(self):
        env = {"PATH": os.environ["PATH"], "HOME": str(self.home)}
        result = subprocess.run(
            ["/bin/bash", str(self.repo / "tools/setup/new_session.sh"), "--defaults",
             "--route", "recorded", "--robot", "booster_t1_29dof"],
            cwd="/", env=env, text=True, capture_output=True, check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        session = self.workbench().current_session()
        self.assertEqual(session["route"], "recorded")
        self.assertEqual(session["robot"], "booster_t1_29dof")
        self.assertEqual(session["human_height_m"], 1.75)
        self.assertEqual(session["mocap_ws"], str(self.home / "mocap-runtime"))
        self.assertEqual((session["start_frame"], session["end_frame"]), (0, 600))
        self.assertEqual(session["windows_xsens_ip"], "")

    def test_new_session_sh_prefills_from_a_console_written_file(self):
        self.workbench().save_session(self.values(human_height_m=1.64, end_frame=900))
        env = {"PATH": os.environ["PATH"], "HOME": str(self.home)}
        result = subprocess.run(
            ["/bin/bash", str(self.repo / "tools/setup/new_session.sh"), "--defaults",
             "--route", "recorded"],
            cwd="/", env=env, text=True, capture_output=True, check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        height, end, motion = self.source(
            self.repo / "session.env", "HUMAN_HEIGHT_M", "END_FRAME", "MOTION_BVH"
        )
        self.assertEqual((height, end), ("1.64", "900"))
        self.assertEqual(motion, str(self.home / "mocap data" / "motion take.bvh"))

    def test_reader_never_executes_the_file(self):
        marker = self.home / "executed"
        self.session_dir.mkdir(parents=True)
        env_file = self.session_dir / "session.env"
        env_file.write_text(
            f"touch {shlex.quote(str(marker))}\n"
            f"export ROBOT=$(touch {shlex.quote(str(marker))})\n"
            "export SESSION_DIR='/tmp/a b'\n"
        )
        os.symlink(env_file, self.repo / "session.env")
        session = self.workbench().current_session()
        self.assertFalse(marker.exists())
        self.assertEqual(session["session_dir"], "/tmp/a b")
        # A malformed (multi-word) assignment is ignored, never evaluated.
        self.assertEqual(session["robot"], "unitree_g1")

    def test_tilde_paths_expand_to_home(self):
        values = self.workbench().validate(self.values(session_dir="~/mocap-sessions/x"))
        self.assertEqual(values["session_dir"], f"{self.home}/mocap-sessions/x")

    def test_defaults_use_a_fresh_dated_folder_and_detected_linux_ip(self):
        now = time.mktime((2026, 10, 13, 10, 15, 0, 0, 0, -1))
        bench = self.workbench(now=lambda: now)
        (self.home / "mocap-sessions" / "2026-10-13-1015").mkdir(parents=True)
        defaults = bench.new_session_defaults(self.values(linux_xsens_ip=""))
        self.assertEqual(defaults["session_dir"], f"{self.home}/mocap-sessions/2026-10-13-1015-2")
        self.assertEqual(defaults["linux_xsens_ip"], "192.168.1.20")
        self.assertEqual(defaults["human_height_m"], 1.82)


@unittest.skipUnless(_API_TESTS_AVAILABLE, "FastAPI/httpx test dependencies are not installed")
class SetupApiTests(SetupFixture, unittest.TestCase):
    def setUp(self):
        self.make_tree()
        self.addCleanup(self.temporary.cleanup)
        self.make_runtime()
        self.clients = []

    def tearDown(self):
        for client in self.clients:
            client.close()

    def client(self, overrides=None, **kwargs):
        from fastapi.testclient import TestClient

        from mocap_g1.operator.app import create_app

        self.bench = self.workbench(overrides, **kwargs)
        app = create_app(
            data_root=Path(self.temporary.name) / "data", demo=True, setup_root=self.bench
        )
        client = TestClient(app, base_url=LOOPBACK)
        self.clients.append(client)
        self.addCleanup(self.bench.shutdown)
        return client

    def save(self, client, **changes):
        response = client.put("/api/v1/setup/session", json=self.values(**changes),
                              headers=JSON_HEADERS)
        self.assertEqual(response.status_code, 200, response.text)
        return response.json()

    def start(self, client, kind, options=None, expect=201):
        body = {"kind": kind}
        if options is not None:
            body["options"] = options
        response = client.post("/api/v1/setup/jobs", json=body, headers=JSON_HEADERS)
        self.assertEqual(response.status_code, expect, response.text)
        return response.json()

    def wait(self, client, job_id, timeout=20.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            job = client.get(f"/api/v1/setup/jobs/{job_id}").json()["job"]
            if job["state"] != "running":
                return job
            time.sleep(0.05)
        self.fail(f"job {job_id} did not finish")

    def log(self, client, job_id, after=0):
        response = client.get(f"/api/v1/setup/jobs/{job_id}", params={"after": after})
        self.assertEqual(response.status_code, 200, response.text)
        return response.json()

    def wait_for_line(self, client, job_id, text, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            # Whole-line match: the echoed command line contains the text too.
            if text in self.log(client, job_id)["lines"]:
                return
            time.sleep(0.05)
        self.fail(f"{text!r} never appeared as a job log line")

    # ------------------------------------------------------------- payload

    def test_payload_without_session(self):
        client = self.client()
        response = client.get("/api/v1/setup")
        self.assertEqual(response.status_code, 200, response.text)
        payload = response.json()
        self.assertIsNone(payload["session"])
        self.assertEqual(payload["stream_check_s"], 30)
        self.assertEqual(payload["repo_root"], str(self.repo.resolve()))
        self.assertTrue(payload["defaults"]["session_dir"].startswith(f"{self.home}/mocap-sessions/"))
        self.assertEqual(payload["defaults"]["route"], "live")
        self.assertEqual(payload["network"]["interfaces"],
                         [{"name": "enp3s0", "ipv4": "192.168.1.20"}])
        self.assertIsNone(payload["network"]["detected_linux_ip"])
        self.assertIsNone(payload["firewall_command"])
        self.assertEqual(payload["readiness"], {"live_image": None, "recorded_runtime": False,
                                                "motion_bvh": False, "npose_bvh": False,
                                                "display": True, "display_hint": None})
        self.assertEqual((payload["files"], payload["jobs"]), ([], []))

    def test_put_session_returns_payload_with_firewall_and_readiness(self):
        client = self.client()
        (self.home / "mocap data").mkdir()
        (self.home / "mocap data" / "motion take.bvh").write_text("")
        payload = self.save(client)
        self.assertEqual(payload["session"], self.values())
        self.assertEqual(
            payload["firewall_command"],
            "sudo ufw allow from 192.168.1.246 to 192.168.1.20 port 5555 proto udp",
        )
        self.assertEqual(payload["network"]["detected_linux_ip"], "192.168.1.20")
        self.assertEqual(payload["readiness"], {"live_image": None, "recorded_runtime": True,
                                                "motion_bvh": True, "npose_bvh": False,
                                                "display": True, "display_hint": None})
        self.assertNotEqual(payload["defaults"]["session_dir"], str(self.session_dir))
        self.assertEqual(client.get("/api/v1/setup").json()["session"], self.values())

    def test_validation_errors_name_the_field(self):
        client = self.client()
        cases = [
            ({"human_height_m": 2.6}, "human_height_m", "Height must be 0.8–2.5 m"),
            ({"human_height_m": True}, "human_height_m", None),
            ({"route": "both"}, "route", None),
            ({"robot": "h1"}, "robot", None),
            ({"session_dir": "relative/take"}, "session_dir", None),
            ({"session_dir": "/tmp/take"}, "session_dir",
             "Session folder must be inside your home folder and outside the project checkout"),
            ({"session_dir": "~"}, "session_dir", None),
            ({"session_dir": "~/a/../../outside"}, "session_dir", None),
            ({"mocap_ws": "/a\nb"}, "mocap_ws", None),
            ({"bvh_scale": 0.1}, "bvh_scale", None),
            ({"start_frame": 600, "end_frame": 600}, "end_frame", None),
            ({"start_frame": 1.5}, "start_frame", None),
            ({"frame_step": 0}, "frame_step", None),
            ({"npose_duration_s": 0.001}, "npose_duration_s", None),
            ({"windows_xsens_ip": "127.0.0.1"}, "windows_xsens_ip", None),
            ({"windows_xsens_ip": "224.0.0.1"}, "windows_xsens_ip", None),
            ({"linux_xsens_ip": "192.168.1.300"}, "linux_xsens_ip", None),
            ({"linux_xsens_ip": "192.168.1.246"}, "linux_xsens_ip", None),
            ({"offsets_file": "offsets.json"}, "offsets_file", None),
        ]
        for changes, field, message in cases:
            with self.subTest(changes=changes):
                response = client.put("/api/v1/setup/session", json=self.values(**changes),
                                      headers=JSON_HEADERS)
                self.assertEqual(response.status_code, 422, response.text)
                error = response.json()["error"]
                self.assertEqual(error["code"], "invalid_session")
                self.assertEqual(error["field"], field)
                if message is not None:
                    self.assertEqual(error["message"], message)
        missing = self.values()
        del missing["robot"]
        response = client.put("/api/v1/setup/session", json=missing, headers=JSON_HEADERS)
        self.assertEqual(response.json()["error"]["field"], "robot")
        self.assertFalse((self.repo / "session.env").exists())
        # Empty IPs are allowed (not known yet).
        self.save(client, windows_xsens_ip="", linux_xsens_ip="")

    def test_existing_session_needs_replace_unless_it_is_current(self):
        client = self.client()
        other = self.home / "mocap sessions" / "take 2"
        self.save(client)
        self.save(client, session_dir=str(other))
        response = client.put("/api/v1/setup/session", json=self.values(), headers=JSON_HEADERS)
        self.assertEqual(response.status_code, 409, response.text)
        self.assertEqual(response.json()["error"]["code"], "session_exists")
        self.assertEqual(os.readlink(self.repo / "session.env"), str(other / "session.env"))
        # Re-saving the current session needs no confirmation.
        self.save(client, session_dir=str(other), human_height_m=1.7)
        response = client.put("/api/v1/setup/session",
                              json={**self.values(), "replace": True}, headers=JSON_HEADERS)
        self.assertEqual(response.status_code, 200, response.text)
        self.assertEqual(os.readlink(self.repo / "session.env"),
                         str(self.session_dir / "session.env"))

    def test_regular_session_env_in_checkout_is_never_replaced(self):
        client = self.client()
        (self.repo / "session.env").write_text("export ROBOT=unitree_g1\n")
        response = client.put("/api/v1/setup/session", json=self.values(), headers=JSON_HEADERS)
        self.assertEqual(response.status_code, 409, response.text)
        self.assertEqual(response.json()["error"]["code"], "session_link_conflict")
        self.assertEqual((self.repo / "session.env").read_text(), "export ROBOT=unitree_g1\n")
        self.assertFalse((self.session_dir / "session.env").exists())

    def test_mutations_require_same_origin_and_json(self):
        client = self.client()
        response = client.put("/api/v1/setup/session", json=self.values(),
                              headers={"Origin": "http://evil.example"})
        self.assertEqual(response.status_code, 403)
        response = client.put("/api/v1/setup/session", content=json.dumps(self.values()),
                              headers={"Content-Type": "text/plain", "Origin": LOOPBACK})
        self.assertEqual(response.status_code, 415)
        response = client.post("/api/v1/setup/jobs", json={"kind": "doctor"},
                               headers={"Origin": "http://evil.example"})
        self.assertEqual(response.status_code, 403)
        self.assertFalse((self.repo / "session.env").exists())

    # ---------------------------------------------------------------- jobs

    def test_jobs_need_a_saved_session_and_a_known_kind(self):
        client = self.client({"doctor": python_command("print('PASS')")})
        error = self.start(client, "doctor", expect=409)["error"]
        self.assertEqual(error["code"], "no_session")
        self.save(client)
        self.assertEqual(self.start(client, "format_disk", expect=422)["error"]["code"],
                         "invalid_job")
        error = self.start(client, "retarget", {"mode": "fast"}, expect=422)["error"]
        self.assertEqual(error["code"], "invalid_options")
        error = self.start(client, "live", {"calibrate_after_s": 90}, expect=422)["error"]
        self.assertEqual(error["code"], "invalid_options")

    def test_passing_job_strips_ansi_and_writes_a_log_file(self):
        client = self.client({"doctor": python_command(
            "import os; print('\\x1b[32mchecking\\x1b[0m', os.environ['XSENS_SOURCE_IP'],"
            " os.environ['ROBOT'])"
        )})
        self.save(client)
        started = self.start(client, "doctor")
        self.assertEqual(started["state"], "running")
        self.assertEqual(started["label"], "System check")
        self.assertRegex(started["id"], r"^j-\d{8}-\d{6}-1$")
        job = self.wait(client, started["id"])
        self.assertEqual(job["state"], "passed")
        self.assertEqual(job["exit_code"], 0)
        self.assertEqual(job["summary"], "PASS: system check is ready")
        self.assertIsNotNone(job["finished_at"])
        lines = self.log(client, job["id"])["lines"]
        self.assertIn("checking 192.168.1.246 unitree_g1", lines)
        self.assertFalse(any("\x1b" in line for line in lines))
        log_file = self.session_dir / "logs" / f"{job['id']}.log"
        self.assertEqual(log_file.read_text().splitlines(), lines)
        self.assertEqual(client.get("/api/v1/setup").json()["jobs"][0]["id"], job["id"])

    def test_failing_job_reports_exit_code_and_summary(self):
        client = self.client({"retarget": python_command(
            "import sys; print('loading'); print('ERROR: BVH not found', flush=True);"
            " print('cleanup'); sys.exit(3)"
        )})
        self.save(client)
        job = self.wait(client, self.start(client, "retarget", {"mode": "preview"})["id"])
        self.assertEqual((job["state"], job["exit_code"]), ("failed", 3))
        self.assertEqual(job["summary"], "ERROR: BVH not found")
        self.assertEqual(job["label"], "Make preview motion")

    def test_stop_interrupts_the_process_group(self):
        client = self.client({"render": python_command(
            "import time; print('PASS: rendered 10 frames', flush=True);"
            " print('rendering', flush=True); time.sleep(60)"
        )})
        self.save(client)
        job_id = self.start(client, "render", {"motion": "npose", "record_video": True})["id"]
        self.wait_for_line(client, job_id, "rendering")
        response = client.post(f"/api/v1/setup/jobs/{job_id}/stop", headers={"Origin": LOOPBACK})
        self.assertEqual(response.status_code, 200, response.text)
        self.assertIn(response.json()["state"], ("running", "stopped"))
        job = self.wait(client, job_id)
        self.assertEqual(job["state"], "stopped")
        # The stop wins over an earlier PASS line in the log.
        self.assertEqual(job["summary"], "Stopped by the operator.")
        self.assertEqual(self.stopped_containers, [])
        # Stopping a finished job is harmless and keeps its state.
        again = client.post(f"/api/v1/setup/jobs/{job_id}/stop", headers={"Origin": LOOPBACK})
        self.assertEqual(again.json()["state"], "stopped")
        self.assertEqual(client.post("/api/v1/setup/jobs/nope/stop",
                                     headers={"Origin": LOOPBACK}).status_code, 404)

    def test_stopping_live_escalates_and_stops_the_container(self):
        client = self.client({"live": python_command(
            "import signal, time; signal.signal(signal.SIGINT, signal.SIG_IGN);"
            " print('viewer up', flush=True); time.sleep(60)"
        )})
        self.save(client)
        job_id = self.start(client, "live")["id"]
        self.wait_for_line(client, job_id, "viewer up")
        started = time.monotonic()
        response = client.post(f"/api/v1/setup/jobs/{job_id}/stop",
                               headers={"Origin": LOOPBACK})
        elapsed = time.monotonic() - started
        # The request returns before the escalation (0.5 s per step here) runs.
        self.assertLess(elapsed, 0.4)
        self.assertEqual(response.json()["state"], "running")
        # A repeated stop while terminating does not start a second sequence.
        again = client.post(f"/api/v1/setup/jobs/{job_id}/stop", headers={"Origin": LOOPBACK})
        self.assertEqual(again.status_code, 200)
        job = self.wait(client, job_id)
        self.assertEqual((job["state"], job["summary"]), ("stopped", "Stopped by the operator."))
        self.assertEqual(self.stopped_containers, ["gmr-xsens-live"])

    def test_only_one_job_runs_at_a_time(self):
        client = self.client({
            "doctor": python_command("import time; print('busy', flush=True); time.sleep(60)"),
            "bvh_check": python_command("print('PASS')"),
        })
        self.save(client)
        running = self.start(client, "doctor")
        error = self.start(client, "bvh_check", expect=409)["error"]
        self.assertEqual((error["code"], error["job_id"]), ("job_running", running["id"]))
        client.post(f"/api/v1/setup/jobs/{running['id']}/stop", headers={"Origin": LOOPBACK})
        self.assertEqual(self.wait(client, running["id"])["state"], "stopped")
        self.assertEqual(self.wait(client, self.start(client, "bvh_check")["id"])["state"],
                         "passed")

    def test_log_pagination_and_ring_buffer(self):
        client = self.client({"render": python_command(
            "for i in range(6000): print('line', i)"
        )})
        self.save(client)
        job = self.wait(client, self.start(client, "render", {"motion": "preview"})["id"])
        self.assertEqual(job["state"], "passed")
        first = self.log(client, job["id"], after=0)
        # 6001 lines (command echo + 6000), only the last 5000 are kept.
        self.assertEqual(first["lines"][0], "line 1000")
        self.assertEqual(len(first["lines"]), 2000)
        self.assertEqual(first["next"], 3001)
        rest = []
        cursor = first["next"]
        while True:
            page = self.log(client, job["id"], after=cursor)
            if not page["lines"]:
                break
            rest += page["lines"]
            cursor = page["next"]
        self.assertEqual(cursor, 6001)
        self.assertEqual(rest[-1], "line 5999")
        self.assertEqual(len(first["lines"]) + len(rest), 5000)
        self.assertEqual(self.log(client, job["id"], after=99999), {
            "job": job, "lines": [], "next": 6001,
        })
        log_lines = (self.session_dir / "logs" / f"{job['id']}.log").read_text().splitlines()
        self.assertEqual(len(log_lines), 6001)
        self.assertEqual(client.get("/api/v1/setup/jobs/missing").status_code, 404)

    def test_acceptance_needs_windows_ip_and_honours_the_report_verdict(self):
        report = (
            "import json, sys; out = sys.argv[1]; verdict = sys.argv[2];"
            " json.dump({'verdict': verdict, 'failures': ['no UDP traffic'] if verdict == 'FAIL'"
            " else [], 'body': {'valid_datagrams': 1800, 'observed_receive_hz': 60.0}},"
            " open(out, 'w')); print('done')"
        )
        verdict = {"value": "PASS"}
        client = self.client({"acceptance": lambda _session, options: [
            PYTHON, "-c", report, options["output"], verdict["value"]
        ]})
        self.save(client, windows_xsens_ip="", linux_xsens_ip="")
        self.assertEqual(self.start(client, "acceptance", expect=409)["error"]["code"],
                         "missing_windows_ip")
        self.save(client)
        job = self.wait(client, self.start(client, "acceptance")["id"])
        self.assertEqual(job["state"], "passed")
        self.assertEqual(job["summary"], "PASS: 1800 body frames at 60.0 Hz")
        self.assertEqual(len(job["outputs"]), 1)
        self.assertRegex(job["outputs"][0], r"^stream-acceptance-\d{8}-\d{6}\.json$")
        self.assertTrue((self.session_dir / job["outputs"][0]).is_file())

        verdict["value"] = "FAIL"
        job = self.wait(client, self.start(client, "acceptance")["id"])
        self.assertEqual((job["state"], job["exit_code"]), ("failed", 0))
        self.assertEqual(job["summary"], "FAIL: no UDP traffic")

    def test_live_copies_a_fresh_recording_into_the_session(self):
        video = self.repo / "artifacts" / "xsens-live" / "t1-live.mp4"
        client = self.client({"live": lambda _session, _options: [
            PYTHON, "-c",
            "import pathlib, sys; p = pathlib.Path(sys.argv[1]);"
            " p.parent.mkdir(parents=True, exist_ok=True); p.write_bytes(b'mp4')",
            str(video),
        ]})
        self.save(client, robot="booster_t1_29dof")
        job = self.wait(client, self.start(client, "live", {"record_video": True,
                                                              "calibrate_after_s": 5})["id"])
        self.assertEqual(job["state"], "passed", self.log(client, job["id"])["lines"])
        self.assertEqual(len(job["outputs"]), 1)
        self.assertRegex(job["outputs"][0], r"^booster_t1_29dof-live-\d{8}-\d{6}\.mp4$")
        self.assertEqual((self.session_dir / job["outputs"][0]).read_bytes(), b"mp4")

    def test_default_commands_match_the_guide(self):
        client = self.client()
        self.save(client, route="recorded")
        runtime = str(self.home / "mocap runtime")
        folder = str(self.session_dir)
        common = [
            f"{runtime}/venv/bin/python",
            str(self.repo.resolve() / "tools/retarget/retarget_robot_recorded.py"),
            "--gmr-root", f"{runtime}/GMR", "--robot", "unitree_g1",
            "--ik-config", f"{runtime}/GMR/general_motion_retargeting/ik_configs/bvh_xsens_to_g1.json",
            "--bvh-file", str(self.home / "mocap data" / "motion take.bvh"),
        ]
        frames = ["--scale", "0.01", "--human-height", "1.82", "--start", "1", "--end", "600",
                  "--frame-step", "1", "--allow-zero-offsets"]
        expected = {
            ("retarget", "preview"): common + ["--save-path", f"{folder}/unitree_g1-preview.pkl"]
            + frames + ["--arm-mode", "position-only", "--overwrite"],
            ("retarget", "npose"): common + ["--save-path", f"{folder}/unitree_g1-npose.pkl"]
            + frames + [
                "--arm-mode", "npose",
                "--calibration-bvh-file", str(self.home / "mocap data" / "neutral take.bvh"),
                "--calibration-start-seconds", "2.0", "--calibration-seconds", "1.0",
                "--calibration-output", f"{folder}/unitree_g1-arm-calibration.json",
                "--accept-reviewed-calibration-window", "--overwrite",
            ],
            ("render", "preview"): [
                f"{runtime}/venv/bin/python",
                str(self.repo.resolve() / "tools/retarget/render_robot_motion.py"),
                "--gmr-root", f"{runtime}/GMR", "--robot", "unitree_g1",
                "--motion", f"{folder}/unitree_g1-preview.pkl", "--loops", "1",
                "--record-video", "--video-path", f"{folder}/unitree_g1-preview.mp4",
                "--overwrite-video",
            ],
        }
        options = {
            ("retarget", "preview"): {"mode": "preview"},
            ("retarget", "npose"): {"mode": "npose", "overwrite": True},
            ("render", "preview"): {"motion": "preview", "record_video": True},
        }
        for key, argv in expected.items():
            with self.subTest(key=key):
                job = self.wait(client, self.start(client, key[0], options[key])["id"])
                lines = self.log(client, job["id"])["lines"]
                # The log shows home-folder paths as ~/... and the rest shell-quoted.
                home = str(self.home) + "/"
                shown = " ".join(
                    "~/" + shlex.quote(arg[len(home):]) if arg.startswith(home)
                    else shlex.quote(arg)
                    for arg in argv
                )
                self.assertEqual(lines[0], "$ " + shown)
                self.assertIn("~/", shown)
                self.assertEqual((job["state"], job["exit_code"]), ("passed", 0))
                if key[0] == "retarget":
                    # The JSON summary's closing brace must not become the verdict.
                    self.assertEqual(job["summary"], f"PASS: made unitree_g1-{key[1]}.pkl")

    def test_bvh_check_reports_frames_and_writes_checksums(self):
        client = self.client()
        data = self.home / "mocap data"
        data.mkdir()
        header = "HIERARCHY\nROOT Hips\n{\n}\nMOTION\nFrames: FRAMES\nFrame Time: 0.0166667\n0 0\n"
        (data / "motion take.bvh").write_text(header.replace("FRAMES", "700"))
        (data / "neutral take.bvh").write_text(header.replace("FRAMES", "120"))
        self.save(client)
        job = self.wait(client, self.start(client, "bvh_check")["id"])
        self.assertEqual(job["state"], "passed", self.log(client, job["id"])["lines"])
        self.assertEqual(job["outputs"], ["bvh-sha256.txt"])
        lines = self.log(client, job["id"])["lines"]
        self.assertIn("Motion: motion take.bvh: 700 frames, Frame Time 0.0166667 (60.0 Hz)", lines)
        check = subprocess.run(["sha256sum", "-c", "bvh-sha256.txt"], cwd=self.session_dir,
                               capture_output=True, text=True, check=False)
        self.assertEqual(check.returncode, 0, check.stdout + check.stderr)

        (data / "motion take.bvh").write_text(header.replace("FRAMES", "500"))
        job = self.wait(client, self.start(client, "bvh_check")["id"])
        self.assertEqual(job["state"], "failed")
        self.assertTrue(job["summary"].startswith("FAIL: the motion take has 500 frames"))

    def test_save_writes_revision_and_core_report(self):
        client = self.client({"save": python_command("import json; print(json.dumps({'ok': True}))")})
        self.save(client)
        job = self.wait(client, self.start(client, "save")["id"])
        self.assertEqual(job["state"], "failed")
        self.assertTrue(job["summary"].startswith("ERROR: git rev-parse failed"))

        git = ["git", "-c", "user.name=t", "-c", "user.email=t@example.invalid",
               "-c", "commit.gpgsign=false"]
        subprocess.run(git + ["init", "-q"], cwd=self.repo, check=True)
        subprocess.run(git + ["commit", "-q", "--allow-empty", "-m", "x"], cwd=self.repo,
                       check=True)
        revision = subprocess.run(["git", "rev-parse", "HEAD"], cwd=self.repo, check=True,
                                  capture_output=True, text=True).stdout.strip()
        job = self.wait(client, self.start(client, "save")["id"])
        self.assertEqual(job["state"], "passed", self.log(client, job["id"])["lines"])
        self.assertEqual(sorted(job["outputs"]), ["core-doctor.json", "project-revision.txt"])
        self.assertEqual((self.session_dir / "project-revision.txt").read_text(), revision + "\n")
        self.assertEqual(json.loads((self.session_dir / "core-doctor.json").read_text()),
                         {"ok": True})

    def test_shutdown_stops_the_running_job(self):
        from fastapi.testclient import TestClient

        from mocap_g1.operator.app import create_app

        bench = self.workbench({"doctor": python_command(
            "import time; print('busy', flush=True); time.sleep(60)"
        )})
        app = create_app(data_root=Path(self.temporary.name) / "data", demo=True,
                         setup_root=bench)
        with TestClient(app, base_url=LOOPBACK) as client:
            self.save(client)
            job_id = self.start(client, "doctor")["id"]
            self.wait_for_line(client, job_id, "busy")
        self.assertEqual(bench.job_log(job_id)["job"]["state"], "stopped")

    # ----------------------------------------------------------- pre-checks

    def test_no_display_is_reported_and_blocks_viewer_jobs(self):
        self.display = False
        client = self.client({kind: python_command("print('PASS')")
                              for kind in ("live", "render", "doctor")})
        self.save(client)
        readiness = client.get("/api/v1/setup").json()["readiness"]
        self.assertFalse(readiness["display"])
        self.assertEqual(
            readiness["display_hint"],
            "No desktop display: close the console with `bash tools/setup/open_console.sh "
            "--stop`, then open it from the Mocap Setup menu entry on this PC (not over SSH).",
        )
        for kind, options in (("live", {}), ("render", {"motion": "preview"}),
                              ("render", {"motion": "preview", "record_video": True})):
            with self.subTest(kind=kind, options=options):
                error = self.start(client, kind, options, expect=409)["error"]
                self.assertEqual(error["code"], "no_display")
                self.assertEqual(error["message"], readiness["display_hint"])
        self.assertEqual(self.wait(client, self.start(client, "doctor")["id"])["state"], "passed")

    def test_missing_recorded_runtime_blocks_runtime_jobs(self):
        shutil.rmtree(self.home / "mocap runtime")
        client = self.client({kind: python_command("print('PASS')")
                              for kind in ("doctor", "retarget", "render")})
        self.save(client)
        for kind in ("retarget", "render", "doctor"):
            with self.subTest(kind=kind):
                error = self.start(client, kind, expect=409)["error"]
                self.assertEqual(error["code"], "recorded_runtime_missing")
                self.assertEqual(
                    error["message"],
                    "Recorded runtime not found in ~/mocap runtime. "
                    "Run the installer and choose 2 · Recorded.",
                )
        # The live route's system check does not need the recorded runtime.
        self.save(client, route="live")
        self.assertEqual(self.wait(client, self.start(client, "doctor")["id"])["state"], "passed")

    def test_leftover_viewer_and_busy_port_block_udp_jobs(self):
        client = self.client({kind: python_command("print('PASS')")
                              for kind in ("acceptance", "live", "doctor")})
        self.save(client)
        self.viewer = True
        for kind in ("acceptance", "live"):
            with self.subTest(kind=kind, check="viewer"):
                error = self.start(client, kind, expect=409)["error"]
                self.assertEqual(error["code"], "viewer_running")
                self.assertEqual(error["message"], "A live viewer from an earlier run is still "
                                 "running. Stop it, then try again.")
        self.viewer = False
        self.busy_ports = [5555]
        for kind in ("acceptance", "live"):
            with self.subTest(kind=kind, check="port"):
                error = self.start(client, kind, expect=409)["error"]
                self.assertEqual(error["code"], "udp_port_busy")
                self.assertIn("UDP port 5555", error["message"])
        self.assertEqual(self.wait(client, self.start(client, "doctor")["id"])["state"], "passed")
        self.busy_ports = []
        self.assertEqual(self.wait(client, self.start(client, "acceptance")["id"])["state"],
                         "passed")

    def test_running_job_wins_over_port_and_viewer_checks(self):
        client = self.client({"live": python_command(
            "import time; print('viewer up', flush=True); time.sleep(60)"
        )})
        self.save(client)
        running = self.start(client, "live")
        # Our own viewer now holds the port and container.
        self.viewer, self.busy_ports = True, [5555]
        error = self.start(client, "acceptance", expect=409)["error"]
        self.assertEqual((error["code"], error["job_id"]), ("job_running", running["id"]))
        client.post(f"/api/v1/setup/jobs/{running['id']}/stop", headers={"Origin": LOOPBACK})

    def test_stream_check_duration_follows_the_console_environment(self):
        client = self.client()
        for value, expected in (("45", 45), ("0", 30), ("-5", 30), ("ten", 30), ("", 30)):
            with self.subTest(value=value), mock.patch.dict(os.environ,
                                                            {"XSENS_DURATION_S": value}):
                self.assertEqual(client.get("/api/v1/setup").json()["stream_check_s"], expected)

    def test_hand_edited_session_folder_is_rechecked_before_a_job(self):
        client = self.client({"doctor": python_command("print('PASS')")})
        self.save(client)
        env_file = self.session_dir / "session.env"
        original = env_file.read_text()
        for folder in ("/tmp/elsewhere", str(self.repo / "sessions"), str(self.home)):
            with self.subTest(folder=folder):
                env_file.write_text(original.replace(
                    f"export SESSION_DIR={shlex.quote(str(self.session_dir))}",
                    f"export SESSION_DIR={shlex.quote(folder)}",
                ))
                error = self.start(client, "doctor", expect=409)["error"]
                self.assertEqual(error["code"], "invalid_session_dir")
                self.assertEqual(error["message"], "Session folder must be inside your home "
                                 "folder and outside the project checkout")
        self.assertFalse((self.repo / "sessions").exists())
        env_file.write_text(original)
        self.assertEqual(self.wait(client, self.start(client, "doctor")["id"])["state"], "passed")

    def test_viewer_stop_endpoint(self):
        client = self.client({"live": python_command(
            "import time; print('viewer up', flush=True); time.sleep(60)"
        )})
        self.save(client)
        response = client.post("/api/v1/setup/viewer/stop", json={}, headers=JSON_HEADERS)
        self.assertEqual(response.status_code, 200, response.text)
        self.assertEqual(response.json(), {"stopped": True})
        self.assertEqual(self.stopped_containers, ["gmr-xsens-live"])
        self.assertEqual(client.post("/api/v1/setup/viewer/stop", json={},
                                     headers={"Origin": "http://evil.example"}).status_code, 403)
        self.assertEqual(client.post("/api/v1/setup/viewer/stop", content="{}",
                                     headers={"Content-Type": "text/plain",
                                              "Origin": LOOPBACK}).status_code, 415)
        running = self.start(client, "live")
        error = client.post("/api/v1/setup/viewer/stop", json={},
                            headers=JSON_HEADERS).json()["error"]
        self.assertEqual((error["code"], error["job_id"]), ("job_running", running["id"]))
        self.assertEqual(self.stopped_containers, ["gmr-xsens-live"])
        client.post(f"/api/v1/setup/jobs/{running['id']}/stop", headers={"Origin": LOOPBACK})

    def test_jobs_find_the_console_python_first_on_path(self):
        client = self.client({"doctor": python_command(
            "import os; print('PATH0', os.environ['PATH'].split(os.pathsep)[0])"
        )})
        self.save(client)
        job = self.wait(client, self.start(client, "doctor")["id"])
        self.assertIn(f"PATH0 {os.path.dirname(PYTHON)}", self.log(client, job["id"])["lines"])

    # --------------------------------------------------------------- files

    def test_files_are_listed_and_served_from_the_session_folder_only(self):
        client = self.client()
        self.save(client)
        (self.session_dir / "unitree_g1-preview.mp4").write_bytes(b"\x00\x00\x00\x18ftypmp42")
        (self.session_dir / "unitree_g1-preview.pkl").write_bytes(b"pkl")
        (self.session_dir / "report.json").write_text("{}")
        (self.session_dir / ".hidden.json").write_text("{}")
        outside = self.home / "secret.txt"
        outside.write_text("secret")
        os.symlink(outside, self.session_dir / "linked.txt")
        files = client.get("/api/v1/setup").json()["files"]
        self.assertEqual(
            [(item["name"], item["kind"]) for item in files],
            [("report.json", "report"), ("unitree_g1-preview.mp4", "video"),
             ("unitree_g1-preview.pkl", "motion")],
        )
        self.assertEqual(files[1]["size"], 12)

        response = client.get("/api/v1/setup/files/unitree_g1-preview.mp4")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.headers["content-type"], "video/mp4")
        self.assertEqual(response.content, b"\x00\x00\x00\x18ftypmp42")
        self.assertEqual(client.get("/api/v1/setup/files/report.json").status_code, 200)
        for name in ("unitree_g1-preview.pkl", "session.env", "linked.txt", ".hidden.json",
                     "missing.mp4", "%2e%2e", "..%2Fsecret.txt", "..%2F..%2Fsecret.txt",
                     "logs%2Fx.log", "%2Fetc%2Fpasswd"):
            with self.subTest(name=name):
                self.assertEqual(client.get(f"/api/v1/setup/files/{name}").status_code, 404)
        # The HTTP client normalizes a bare ".." away; check the resolver itself.
        from mocap_g1.operator.setup_workbench import SetupError

        for name in ("..", ".", "../secret.txt", "logs/x.log", "a\\b.txt", ""):
            with self.subTest(name=name), self.assertRaises(SetupError):
                self.bench.file_path(name)

    def test_every_setup_route_is_404_without_setup(self):
        from fastapi.testclient import TestClient

        from mocap_g1.operator.app import create_app

        app = create_app(data_root=Path(self.temporary.name) / "data", demo=True)
        with TestClient(app, base_url=LOOPBACK) as client:
            responses = [
                client.get("/api/v1/setup"),
                client.put("/api/v1/setup/session", json=self.values(), headers=JSON_HEADERS),
                client.post("/api/v1/setup/jobs", json={"kind": "doctor"}, headers=JSON_HEADERS),
                client.get("/api/v1/setup/jobs/j-1"),
                client.post("/api/v1/setup/jobs/j-1/stop", headers={"Origin": LOOPBACK}),
                client.get("/api/v1/setup/files/report.json"),
            ]
        self.assertEqual([response.status_code for response in responses], [404] * 6)
        self.assertFalse((self.repo / "session.env").exists())

    def test_create_app_refuses_setup_beyond_loopback_or_in_review_modes(self):
        from mocap_g1.operator.app import create_app

        data = Path(self.temporary.name) / "data"
        proxy = {"identity_header": "X-User", "trusted_proxies": ("192.0.2.1",)}
        refused = [
            {"playback": self.home / "take.mp4"},
            {"simulation": self.home / "take.npz"},
            {"bind_host": "0.0.0.0", **proxy},
            {"allowed_hosts": ("http://testserver",), **proxy},
            {"allowed_hosts": ("localhost:9000",), **proxy},
        ]
        for kwargs in refused:
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                create_app(data_root=data, setup_root=self.workbench(), **kwargs)
        # Loopback aliases remain fine without a proxy.
        create_app(data_root=data, setup_root=self.repo, allowed_hosts=("localhost:9000",))
        with self.assertRaises(ValueError):
            create_app(data_root=data, setup_root=self.home)


class SetupProbeTests(unittest.TestCase):
    def test_display_probe_needs_a_local_display_socket_and_authority(self):
        from mocap_g1.operator.setup_workbench import default_display_available

        with tempfile.TemporaryDirectory() as tmp:
            authority = Path(tmp) / "Xauthority"
            authority.write_text("")
            self.assertFalse(default_display_available({"XAUTHORITY": str(authority)}))
            self.assertFalse(default_display_available(
                {"DISPLAY": "localhost:10.0", "XAUTHORITY": str(authority)}))
            sockets = sorted(Path("/tmp/.X11-unix").glob("X*")) if Path(
                "/tmp/.X11-unix").is_dir() else []
            sockets = [path for path in sockets if path.is_socket()]
            if sockets:
                display = ":" + sockets[0].name[1:]
                self.assertTrue(default_display_available(
                    {"DISPLAY": display, "XAUTHORITY": str(authority)}))
                self.assertFalse(default_display_available(
                    {"DISPLAY": display, "XAUTHORITY": str(Path(tmp) / "missing")}))
            self.assertFalse(default_display_available(
                {"DISPLAY": ":987654", "XAUTHORITY": str(authority)}))

    def test_udp_probe_detects_a_bound_port(self):
        import socket

        from mocap_g1.operator.setup_workbench import default_udp_port_busy

        holder = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            holder.bind(("0.0.0.0", 0))
            port = holder.getsockname()[1]
            self.assertTrue(default_udp_port_busy(port))
        finally:
            holder.close()
        self.assertFalse(default_udp_port_busy(port))

    def test_viewer_probe_treats_missing_docker_as_no_viewer(self):
        from mocap_g1.operator.setup_workbench import default_viewer_running

        with mock.patch.dict(os.environ, {"PATH": "/nonexistent"}):
            self.assertFalse(default_viewer_running())


class LiveOperatorAcceptanceTests(unittest.TestCase):
    """live_operator.sh acceptance: a caller --output skips the default report folder."""

    def run_acceptance(self, *args: str) -> tuple[subprocess.CompletedProcess[str], Path]:
        tmp = Path(self.temporary_path())
        repo = tmp / "repo"
        for relative in ("tools/xsens/live_operator.sh", "tools/lib/xsens_source_ip.sh"):
            (repo / relative).parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(REPO_ROOT / relative, repo / relative)
        fake_bin = tmp / "bin"
        fake_bin.mkdir()
        real_python = shutil.which("python3")
        # The source-IP check runs "python3 - IP"; the acceptance tool call is echoed instead.
        (fake_bin / "python3").write_text(
            "#!/bin/sh\n"
            f'if [ "$1" = "-" ]; then exec {shlex.quote(real_python)} "$@"; fi\n'
            'printf "%s\\n" "$@"\n'
        )
        (fake_bin / "python3").chmod(0o755)
        env = {"PATH": f"{fake_bin}:{os.environ['PATH']}", "XSENS_SOURCE_IP": "192.168.1.246"}
        result = subprocess.run(
            ["/bin/bash", str(repo / "tools/xsens/live_operator.sh"), "acceptance", *args],
            env=env, capture_output=True, text=True, check=False,
        )
        return result, repo

    def temporary_path(self) -> str:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        return temporary.name

    def test_default_writes_a_dated_report_folder(self):
        result, repo = self.run_acceptance()
        self.assertEqual(result.returncode, 0, result.stderr)
        argv = result.stdout.splitlines()
        folders = list((repo / "artifacts" / "xsens-live").iterdir())
        self.assertEqual(len(folders), 1)
        self.assertEqual(argv[argv.index("--output") + 1],
                         str(folders[0] / "stream-acceptance.json"))

    def test_caller_output_skips_the_default_folder(self):
        for args in (("--output", "/tmp/x y.json"), ("--output=/tmp/x.json",)):
            with self.subTest(args=args):
                result, repo = self.run_acceptance(*args)
                self.assertEqual(result.returncode, 0, result.stderr)
                argv = result.stdout.splitlines()
                self.assertFalse((repo / "artifacts").exists())
                self.assertEqual([item for item in argv if item.startswith("--output")],
                                 [args[0]])
                self.assertEqual(argv[-len(args):], list(args))
                self.assertIn("--require-joint-angles", argv)


class SetupCliTests(unittest.TestCase):
    def run_cli(self, *args: str, env: dict[str, str] | None = None):
        from mocap_g1.operator import cli as operator_console

        fake_factory = mock.Mock(return_value=object())
        stderr = []
        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(
            operator_console, "_load_runtime", return_value=(mock.Mock(), fake_factory)
        ), mock.patch.dict(os.environ, env or {}, clear=False), mock.patch(
            "sys.stderr"
        ) as fake_stderr:
            if env is None:
                os.environ.pop("MOCAP_G1_IDENTITY_HEADER", None)
                os.environ.pop("MOCAP_G1_TRUSTED_PROXIES", None)
            result = operator_console.main(["--data-root", tmp, *args])
            stderr = "".join(call.args[0] for call in fake_stderr.write.call_args_list)
        return result, fake_factory, stderr

    def test_setup_help_names_the_setup_console_port(self):
        from mocap_g1.operator.cli import build_parser

        help_text = " ".join(build_parser().format_help().split())
        self.assertIn("--setup", help_text)
        self.assertIn("tools/setup/open_console.sh starts it on port 8784", help_text)
        self.assertEqual(build_parser().parse_args([]).port, 8766)

    def test_setup_passes_the_checkout_to_the_app_factory(self):
        result, factory, _ = self.run_cli("--setup", "--demo")
        self.assertEqual(result, 0)
        self.assertEqual(factory.call_args.kwargs["setup_root"], REPO_ROOT)
        self.assertTrue(factory.call_args.kwargs["demo"])
        result, factory, _ = self.run_cli()
        self.assertEqual(result, 0)
        self.assertIsNone(factory.call_args.kwargs["setup_root"])

    def test_setup_is_refused_in_review_modes_and_beyond_loopback(self):
        with tempfile.TemporaryDirectory() as tmp:
            video = Path(tmp) / "take.mp4"
            video.write_bytes(b"\x00\x00\x00\x18ftypmp42")
            cases = [
                (("--setup", "--playback", str(video)), None, "--playback"),
                (("--setup", "--simulation", str(Path(tmp) / "x.npz")), None, "--simulation"),
                (("--setup", "--host", "0.0.0.0", "--identity-header", "X-User",
                  "--trusted-proxy", "127.0.0.1"), None, "loopback"),
                (("--setup", "--allowed-host", "https://console.example", "--identity-header",
                  "X-User", "--trusted-proxy", "127.0.0.1"), None, "loopback"),
                (("--setup",), {"MOCAP_G1_IDENTITY_HEADER": "X-User",
                                "MOCAP_G1_TRUSTED_PROXIES": "127.0.0.1"}, "identity header"),
            ]
            for args, env, needle in cases:
                with self.subTest(args=args):
                    result, factory, stderr = self.run_cli(*args, env=env)
                    self.assertEqual(result, 2)
                    self.assertTrue(stderr.startswith("ERROR: "), stderr)
                    self.assertIn(needle, stderr)
                    factory.assert_not_called()

    def test_setup_needs_a_source_checkout(self):
        from mocap_g1.operator import cli as operator_console

        with tempfile.TemporaryDirectory() as tmp:
            fake_package = Path(tmp) / "site" / "mocap_g1" / "__init__.py"
            fake_package.parent.mkdir(parents=True)
            fake_package.write_text("")
            with mock.patch("mocap_g1.__file__", str(fake_package)):
                with self.assertRaisesRegex(ValueError, "source checkout"):
                    operator_console._setup_root()


if __name__ == "__main__":
    unittest.main()
