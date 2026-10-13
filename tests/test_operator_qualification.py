from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from mocap_g1.operator.qualification import read_qualification_runs


STAGES = (
    "window_audit",
    "conditioning",
    "contact_refinement",
    "self_collision_gate",
    "controller_screen",
    "npz_to_pkl",
    "phuma_gate_v2",
)
STAGES_V3 = (
    "window_audit",
    "conditioning",
    "contact_refinement",
    "self_collision_gate",
    "controller_screen",
    "balance_screen",
    "npz_to_pkl",
    "phuma_gate_v2",
)
STATUS_RANK = {"PASS": 0, "BLOCKED": 1, "FAIL": 2}


def ledger(
    created_at: str,
    *,
    statuses: dict[str, str] | None = None,
    reason: str | None = None,
    version: int = 2,
    stage_names: tuple[str, ...] | None = None,
) -> dict:
    if stage_names is None:
        stage_names = STAGES_V3 if version >= 3 else STAGES
    status_by_stage = {stage: "PASS" for stage in stage_names}
    status_by_stage.update(statuses or {})
    stages = []
    for order, stage in enumerate(stage_names, start=1):
        entry = {
            "stage": stage,
            "order": order,
            "status": status_by_stage[stage],
            "reasons": [reason] if reason and stage == "conditioning" else [],
            "command": ["python", "/private/tool.py"],
            "returncode": 0,
            "duration_s": 0.1,
            "artifacts": {
                "report": {"path": "/private/report.json", "sha256": "abc"}
            },
        }
        stages.append(entry)
    worst = max(stages, key=lambda item: STATUS_RANK[item["status"]])
    return {
        "ledger_version": version,
        "generated_at_utc": created_at,
        "driver": {
            "path": "tools/qualify_g1_recording.py",
            "project_git_rev": "deadbeef",
        },
        "input": {
            "recording_path": "/private/recordings/walk.npz",
            "recording_sha256": "secret-sha",
        },
        "config": {
            "model_path": "/private/model.xml",
            "out_dir": "/private/output",
        },
        "stages": stages,
        "verdict": {
            "overall": worst["status"],
            "worst_stage": worst["stage"],
        },
    }


def write_ledger(root: Path, run_id: str, payload: dict) -> Path:
    run = root / run_id
    run.mkdir(parents=True)
    path = run / "ledger.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


class QualificationRunReaderTests(unittest.TestCase):
    def test_current_ledger_becomes_a_sanitized_public_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            payload = ledger(
                "2026-08-25T12:30:00+00:00",
                statuses={"conditioning": "BLOCKED"},
                reason="missing report: /srv/mocap/run/02_conditioning.report.json",
            )
            payload["reference_quality"] = "recorded"
            payload["simulator_execution"] = "mujoco"
            payload["stages"][0]["margin"] = 0.125
            payload["stages"][0]["worst_time_s"] = 1.75
            write_ledger(root, "run-alpha", payload)

            result = read_qualification_runs(root)

            self.assertEqual(len(result["runs"]), 1)
            run = result["runs"][0]
            self.assertEqual(run["id"], "run-alpha")
            self.assertEqual(run["name"], "run-alpha")
            self.assertEqual(run["created_at"], "2026-08-25T12:30:00Z")
            self.assertEqual(run["verdict"], "BLOCKED")
            self.assertEqual(run["reference_quality"], "recorded")
            self.assertEqual(run["simulator_execution"], "mujoco")
            self.assertEqual(run["stages"][0]["margin"], 0.125)
            self.assertEqual(run["stages"][0]["worst_time_s"], 1.75)
            self.assertIn("[path]", run["stages"][1]["reason"])

            serialized = json.dumps(result)
            for private_value in (
                "/srv/mocap",
                "/private",
                "secret-sha",
                "deadbeef",
            ):
                self.assertNotIn(private_value, serialized)

    def test_diagnostic_window_audit_failure_does_not_decide_the_verdict(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            payload = ledger(
                "2026-08-25T12:30:00Z", statuses={"window_audit": "FAIL"}
            )
            for stage in payload["stages"]:
                stage["gates_verdict"] = stage["stage"] != "window_audit"
            payload["verdict"] = {"overall": "PASS", "worst_stage": "conditioning"}
            write_ledger(root, "diagnostic-only", payload)

            # A ledger that let the diagnostic stage decide is inconsistent.
            wrong = json.loads(json.dumps(payload))
            wrong["verdict"] = {"overall": "FAIL", "worst_stage": "window_audit"}
            write_ledger(root, "inconsistent", wrong)

            result = read_qualification_runs(root)

            self.assertEqual([run["id"] for run in result["runs"]], ["diagnostic-only"])
            self.assertEqual(result["runs"][0]["verdict"], "PASS")
            self.assertEqual(result["runs"][0]["stages"][0]["status"], "FAIL")

    def test_v3_ledger_with_balance_stage_is_shown_and_v2_still_reads(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_ledger(
                root,
                "v3-balance-fail",
                ledger(
                    "2026-10-12T10:00:00Z",
                    version=3,
                    statuses={"balance_screen": "FAIL"},
                ),
            )
            write_ledger(root, "v2-pass", ledger("2026-10-11T10:00:00Z"))
            # A v2 ledger may not carry the v3 stage order, and vice versa.
            write_ledger(
                root,
                "v2-with-v3-stages",
                ledger("2026-10-10T10:00:00Z", version=2, stage_names=STAGES_V3),
            )
            write_ledger(
                root,
                "v3-with-v2-stages",
                ledger("2026-10-09T10:00:00Z", version=3, stage_names=STAGES),
            )

            result = read_qualification_runs(root)

            runs = {run["id"]: run for run in result["runs"]}
            self.assertEqual(sorted(runs), ["v2-pass", "v3-balance-fail"])
            v3 = runs["v3-balance-fail"]
            self.assertEqual(v3["verdict"], "FAIL")
            self.assertEqual(
                [stage["name"] for stage in v3["stages"]], list(STAGES_V3)
            )
            self.assertEqual(v3["stages"][5]["status"], "FAIL")
            self.assertEqual(runs["v2-pass"]["verdict"], "PASS")

    def test_malformed_and_oversized_ledgers_are_ignored(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_ledger(root, "valid", ledger("2026-08-25T12:30:00Z"))

            malformed = root / "malformed"
            malformed.mkdir()
            (malformed / "ledger.json").write_text("{not-json", encoding="utf-8")

            oversized = root / "oversized"
            oversized.mkdir()
            (oversized / "ledger.json").write_text(
                json.dumps({"padding": "x" * 20_000}), encoding="utf-8"
            )

            result = read_qualification_runs(root, max_json_bytes=8_192)

            self.assertEqual([run["id"] for run in result["runs"]], ["valid"])

    def test_symlinked_directories_files_and_configured_roots_are_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            root = base / "root"
            outside = base / "outside"
            root.mkdir()
            outside.mkdir()
            write_ledger(root, "safe", ledger("2026-08-25T12:30:00Z"))
            outside_ledger = write_ledger(
                outside, "escaped", ledger("2026-08-26T12:30:00Z")
            )

            try:
                (root / "linked-directory").symlink_to(
                    outside_ledger.parent, target_is_directory=True
                )
                linked_file = root / "linked-file"
                linked_file.mkdir()
                (linked_file / "ledger.json").symlink_to(outside_ledger)
                linked_root = base / "linked-root"
                linked_root.symlink_to(root, target_is_directory=True)
            except (NotImplementedError, OSError) as exc:
                self.skipTest(f"symlinks unavailable: {exc}")

            result = read_qualification_runs(root)

            self.assertEqual([run["id"] for run in result["runs"]], ["safe"])
            self.assertEqual(read_qualification_runs(linked_root), {"runs": []})

    def test_newest_first_order_and_file_and_result_caps(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_ledger(root, "run-z-old", ledger("2026-08-23T09:00:00Z"))
            write_ledger(root, "run-a-new", ledger("2026-08-25T09:00:00Z"))
            write_ledger(root, "run-m-middle", ledger("2026-08-24T09:00:00Z"))

            all_runs = read_qualification_runs(root)
            limited_runs = read_qualification_runs(root, max_results=2)
            one_file = read_qualification_runs(root, max_files=1)

            self.assertEqual(
                [run["id"] for run in all_runs["runs"]],
                ["run-a-new", "run-m-middle", "run-z-old"],
            )
            self.assertEqual(
                [run["id"] for run in limited_runs["runs"]],
                ["run-a-new", "run-m-middle"],
            )
            self.assertEqual(
                [run["id"] for run in one_file["runs"]], ["run-a-new"]
            )

    def test_newest_runs_survive_more_ledgers_than_the_file_bound(self):
        # Regression: the walk used to stop after the first 256 ledgers in
        # lexical order, so lexically late (newest) runs vanished.
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            total = 300
            for index in range(total):
                day = 1 + index // 24
                hour = index % 24
                write_ledger(
                    root,
                    f"run-{index:04d}",
                    ledger(f"2026-08-{day:02d}T{hour:02d}:00:00Z"),
                )

            result = read_qualification_runs(root, max_results=5)
            capped = read_qualification_runs(root, max_files=3)

            self.assertEqual(
                [run["id"] for run in result["runs"]],
                [f"run-{index:04d}" for index in range(total - 1, total - 6, -1)],
            )
            self.assertEqual(
                [run["id"] for run in capped["runs"]],
                ["run-0299", "run-0298", "run-0297"],
            )

    def test_reader_does_not_modify_the_artifact_root(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_ledger(root, "run", ledger("2026-08-25T12:30:00Z"))
            before = {
                path.relative_to(root): (path.stat().st_mode, path.stat().st_mtime_ns)
                for path in root.rglob("*")
            }

            read_qualification_runs(root)

            after = {
                path.relative_to(root): (path.stat().st_mode, path.stat().st_mtime_ns)
                for path in root.rglob("*")
            }
            self.assertEqual(after, before)

    def test_limits_reject_boolean_and_nonpositive_values(self):
        with tempfile.TemporaryDirectory() as directory:
            for keyword, value in (
                ("max_files", 0),
                ("max_json_bytes", -1),
                ("max_results", True),
                ("max_scan_entries", 0),
            ):
                with self.subTest(keyword=keyword), self.assertRaises(ValueError):
                    read_qualification_runs(directory, **{keyword: value})


class QualificationRunApiTests(unittest.TestCase):
    def test_configured_reader_is_exposed_without_artifact_paths(self):
        try:
            from fastapi.testclient import TestClient
            from mocap_g1.operator.app import create_app
        except ImportError as exc:
            self.skipTest(f"operator API dependencies unavailable: {exc}")

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            qualification_root = root / "qualification-runs"
            qualification_root.mkdir()
            write_ledger(
                qualification_root,
                "run-alpha",
                ledger("2026-08-25T12:30:00Z"),
            )

            with TestClient(
                create_app(
                    data_root=root / "operator-state",
                    demo=True,
                    qualification_root=qualification_root,
                ),
                base_url="http://127.0.0.1:8766",
            ) as client:
                response = client.get("/api/v1/qualification-runs")

            self.assertEqual(response.status_code, 200, response.text)
            self.assertEqual(
                [run["id"] for run in response.json()["runs"]],
                ["run-alpha"],
            )
            self.assertNotIn("/private", response.text)

    def test_unconfigured_api_returns_an_empty_run_list(self):
        try:
            from fastapi.testclient import TestClient
            from mocap_g1.operator.app import create_app
        except ImportError as exc:
            self.skipTest(f"operator API dependencies unavailable: {exc}")

        with tempfile.TemporaryDirectory() as directory:
            with TestClient(
                create_app(
                    data_root=Path(directory) / "operator-state",
                    demo=True,
                ),
                base_url="http://127.0.0.1:8766",
            ) as client:
                response = client.get("/api/v1/qualification-runs")

        self.assertEqual(response.status_code, 200, response.text)
        self.assertEqual(response.json(), {"runs": []})


class QualificationRunCliTests(unittest.TestCase):
    def test_launcher_passes_a_validated_fixed_root_to_the_app(self):
        from mocap_g1.operator import cli as operator_console

        fake_uvicorn = mock.Mock()
        fake_factory = mock.Mock(return_value=object())
        with tempfile.TemporaryDirectory() as directory, mock.patch.object(
            operator_console,
            "_load_runtime",
            return_value=(fake_uvicorn, fake_factory),
        ):
            root = Path(directory)
            qualification_root = root / "qualification-runs"
            qualification_root.mkdir()
            result = operator_console.main(
                [
                    "--demo",
                    "--data-root",
                    str(root / "operator-state"),
                    "--qualification-root",
                    str(qualification_root),
                ]
            )

            self.assertEqual(result, 0)
            self.assertEqual(
                fake_factory.call_args.kwargs["qualification_root"],
                qualification_root.resolve(),
            )

    def test_launcher_rejects_an_invalid_root_before_loading_web_dependencies(self):
        from mocap_g1.operator import cli as operator_console

        with tempfile.TemporaryDirectory() as directory, mock.patch.object(
            operator_console,
            "_load_runtime",
        ) as load_runtime:
            result = operator_console.main(
                [
                    "--qualification-root",
                    str(Path(directory) / "missing"),
                ]
            )

        self.assertEqual(result, 2)
        load_runtime.assert_not_called()


if __name__ == "__main__":
    unittest.main()
