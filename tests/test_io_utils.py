"""Equivalence tests for the shared atomic writers in ``mocap_g1.io_utils``.

The reference implementations below are verbatim copies of the private
writers that conditioning / collision_gate / contact_refine and six tools/
CLIs carried before they were collapsed into ``write_qpos_npz_atomic`` and
``write_json_atomic``.  The tests prove the shared helpers produce the same
archive contents / bytes and the same refusal messages and exception types.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import tempfile
import zipfile

import numpy as np
import pytest

from mocap_g1.balance_screen import BalanceScreenError
from mocap_g1.collision_gate import CollisionGateError
from mocap_g1.contact_refine import ContactRefineError
from mocap_g1.controller_screen import ControllerScreenError
from mocap_g1.fault_replay import FaultReplayError
from mocap_g1.io_utils import write_json_atomic, write_qpos_npz_atomic, write_text_atomic
from tools.control import (
    audit_g1_self_collisions,
    project_g1_self_collisions,
    qualify_g1_upper_body,
    replay_g1_faults,
    screen_g1_balance,
)
from tools.retarget import refine_g1_contacts
from tools.xsens import audit_g1_recording


def _former_npz_writer(path: Path, qpos, fps, metadata) -> None:
    target = Path(path)
    if target.exists():
        raise RuntimeError(f"refusing to overwrite existing output: {target}")
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


def _former_json_writer(path: Path, report, error, label: str) -> None:
    target = path.resolve()
    if target.exists():
        raise error(f"refusing to overwrite {label}: {target}")
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", prefix=f".{target.name}.", suffix=".tmp",
            dir=target.parent, delete=False,
        ) as handle:
            temporary_name = handle.name
            json.dump(report, handle, indent=2, sort_keys=True, allow_nan=False)
            handle.write("\n")
        os.replace(temporary_name, target)
    finally:
        if temporary_name is not None:
            Path(temporary_name).unlink(missing_ok=True)


def _archive(path: Path) -> dict[str, np.ndarray]:
    with np.load(path, allow_pickle=False) as archive:
        return {key: archive[key] for key in archive.files}


REPORT = {
    "summary": {"status": "PASS", "reasons": []},
    "z": [1.5, -0.0, 1e-300, "ünïcode"],
    "a": {"nested": [True, None, 3]},
}

# (writer, error type, label used in the refusal message)
TOOL_JSON_WRITERS = [
    (audit_g1_self_collisions._write, ControllerScreenError, "output"),
    (qualify_g1_upper_body._write_json, ControllerScreenError, "output"),
    (screen_g1_balance._write_json, BalanceScreenError, "output"),
    (refine_g1_contacts._write_report, ContactRefineError, "report"),
    (project_g1_self_collisions._write_report, CollisionGateError, "report"),
    (replay_g1_faults._write_report, FaultReplayError, "report"),
]


@pytest.mark.parametrize("qpos_dtype", [np.float64, np.float32])
def test_qpos_npz_matches_the_former_private_writers(tmp_path, qpos_dtype):
    qpos = np.random.default_rng(1).normal(size=(7, 36)).astype(qpos_dtype)
    metadata = {"z": 1, "a": {"nested": [1.5, None]}, "root_quaternion_order": "wxyz"}
    former, shared = tmp_path / "former.npz", tmp_path / "sub" / "shared.npz"
    _former_npz_writer(former, qpos, 29.97, metadata)
    write_qpos_npz_atomic(shared, qpos=qpos, fps=29.97, metadata=metadata)

    old, new = _archive(former), _archive(shared)
    assert list(old) == list(new) == ["metadata", "qpos", "fps"]
    for key in old:
        assert old[key].dtype == new[key].dtype and old[key].shape == new[key].shape
        np.testing.assert_array_equal(old[key], new[key])
    assert new["metadata"].shape == () and new["fps"].dtype == np.float64
    assert json.loads(str(new["metadata"].item())) == metadata
    with zipfile.ZipFile(shared) as archive:
        assert {info.compress_type for info in archive.infolist()} == {zipfile.ZIP_DEFLATED}
    assert sorted(path.name for path in shared.parent.iterdir()) == ["shared.npz"]


def test_qpos_npz_refuses_overwrite_and_cleans_up_on_failure(tmp_path):
    target = tmp_path / "out.npz"
    target.write_bytes(b"keep")
    with pytest.raises(ContactRefineError, match="^refusing to overwrite existing output: "):
        write_qpos_npz_atomic(
            target, qpos=np.zeros((3, 36)), fps=30.0, metadata={},
            exists_error=ContactRefineError,
        )
    assert target.read_bytes() == b"keep"

    fresh = tmp_path / "fresh.npz"
    with pytest.raises(TypeError):  # metadata is not JSON-serializable
        write_qpos_npz_atomic(fresh, qpos=np.zeros((3, 36)), fps=30.0, metadata={"x": object()})
    assert sorted(path.name for path in tmp_path.iterdir()) == ["out.npz"]


@pytest.mark.parametrize("writer,error,label", TOOL_JSON_WRITERS)
def test_tool_json_writers_match_the_former_private_writers(tmp_path, writer, error, label):
    former, shared = tmp_path / "former.json", tmp_path / "nested" / "shared.json"
    _former_json_writer(former, REPORT, error, label)
    writer(shared, REPORT)
    assert shared.read_bytes() == former.read_bytes()
    assert sorted(path.name for path in shared.parent.iterdir()) == ["shared.json"]

    # Same exception type and exact message on an existing target, which is
    # reported by its resolved path and left untouched.
    link_dir = tmp_path / "link"
    link_dir.symlink_to(shared.parent)
    with pytest.raises(error) as caught:
        writer(link_dir / "shared.json", {"other": 1})
    assert str(caught.value) == f"refusing to overwrite {label}: {shared.resolve()}"
    assert shared.read_bytes() == former.read_bytes()

    # Strict JSON: NaN is refused and nothing is left behind.
    with pytest.raises(ValueError):
        writer(tmp_path / "nan.json", {"value": float("nan")})
    assert not (tmp_path / "nan.json").exists()
    assert not list(tmp_path.glob(".nan.json.*"))


def test_window_audit_cli_writer_overwrites_atomically(tmp_path):
    target = tmp_path / "audit.json"
    target.write_text("old", encoding="utf-8")
    payload = json.dumps(REPORT, indent=2, sort_keys=True, allow_nan=False) + "\n"
    audit_g1_recording._write_json(target, payload)
    assert target.read_text(encoding="utf-8") == payload
    assert sorted(path.name for path in tmp_path.iterdir()) == ["audit.json"]


def test_shared_json_writer_default_message_and_overwrite(tmp_path):
    target = tmp_path / "report.json"
    write_json_atomic(target, {"a": 1})
    with pytest.raises(FileExistsError, match="^refusing to overwrite existing report: "):
        write_json_atomic(target, {"a": 2})
    write_json_atomic(target, {"a": 2}, overwrite=True)
    assert json.loads(target.read_text(encoding="utf-8")) == {"a": 2}
    write_text_atomic(target, "plain\n", overwrite=True)
    assert target.read_text(encoding="utf-8") == "plain\n"
