"""Validated Xsens-BVH to canonical G1 dataset/provenance workflow."""

from __future__ import annotations

import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import pickle
import re
import shutil
import subprocess
import sys
from typing import Any

import numpy as np

from .gmr_pkl import (
    ROOT_QUAT_ORDER_KEY,
    UNIT_QUATERNION_TOLERANCE,
    declare_gmr_root_quat_order,
    root_rot_as,
)
from .io_utils import git_revision as _git_revision, sha256_file, write_json_atomic
from .schema import G1_29DOF_JOINTS


SCHEMA_VERSION = 1
ROBOT = "unitree_g1"
SAFE_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")


class DatasetError(RuntimeError):
    """Fail-closed error for invalid inputs or incomplete processing."""


def _require(mapping: dict[str, Any], key: str, context: str) -> Any:
    value = mapping.get(key)
    if value is None or value == "":
        raise DatasetError(f"{context}: missing required field {key!r}")
    return value


def _resolve(base: Path, value: str) -> Path:
    path = Path(value).expanduser()
    return (base / path).resolve() if not path.is_absolute() else path.resolve()


def _validate_bvh(path: Path, expected_fps: float) -> dict[str, float | int]:
    if path.suffix.lower() != ".bvh" or not path.is_file():
        raise DatasetError(f"source is not a readable .bvh file: {path}")
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        header = handle.read(2 * 1024 * 1024)
    for marker in ("HIERARCHY", "MOTION", "Frames:", "Frame Time:"):
        if marker not in header:
            raise DatasetError(f"{path}: missing BVH marker {marker!r}")
    frames_match = re.search(r"Frames:\s*(\d+)", header)
    frame_time_match = re.search(r"Frame Time:\s*([0-9.eE+-]+)", header)
    if not frames_match or not frame_time_match:
        raise DatasetError(f"{path}: cannot parse frame count/frame time")
    frames = int(frames_match.group(1))
    frame_time = float(frame_time_match.group(1))
    if frames < 2 or frame_time <= 0:
        raise DatasetError(f"{path}: invalid frames={frames}, frame_time={frame_time}")
    measured_fps = 1.0 / frame_time
    if abs(measured_fps - expected_fps) > max(0.25, expected_fps * 0.005):
        raise DatasetError(
            f"{path}: BVH reports {measured_fps:.4f} FPS but job declares {expected_fps:.4f}"
        )
    return {"frames": frames, "frame_time_s": frame_time, "fps": measured_fps}


def validate_offsets_for_bvh(offsets: dict[str, Any], bvh_path: str | Path) -> int:
    """Require explicit finite XYZ offsets for every articulated BVH joint."""
    path = Path(bvh_path)
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        hierarchy = handle.read(2 * 1024 * 1024).split("MOTION", 1)[0]
    joint_names = re.findall(r"^\s*(?:ROOT|JOINT)\s+([^\s{]+)", hierarchy, re.MULTILINE)
    if not joint_names:
        raise DatasetError(f"{path}: cannot find ROOT/JOINT names for offset validation")
    missing = [name for name in joint_names if name not in offsets]
    if missing:
        raise DatasetError(
            f"offsets calibration lacks {len(missing)} BVH joints: {', '.join(missing[:8])}"
            + (" ..." if len(missing) > 8 else "")
        )
    for name in joint_names:
        channels = offsets[name]
        if not isinstance(channels, dict):
            raise DatasetError(f"offsets calibration joint {name!r} must be an object")
        for axis in ("X", "Y", "Z"):
            value = channels.get(axis)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise DatasetError(f"offsets calibration {name}.{axis} must be numeric")
            if not math.isfinite(float(value)):
                raise DatasetError(f"offsets calibration {name}.{axis} must be finite")
    return len(joint_names)


def load_and_validate_job(job_path: str | Path) -> dict[str, Any]:
    job_file = Path(job_path).resolve()
    try:
        job = json.loads(job_file.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise DatasetError(f"cannot read job {job_file}: {exc}") from exc
    if not isinstance(job, dict):
        raise DatasetError("job root must be a JSON object")
    if job.get("schema_version") != SCHEMA_VERSION:
        raise DatasetError(f"schema_version must be {SCHEMA_VERSION}")
    if job.get("robot", ROBOT) != ROBOT:
        raise DatasetError(f"this pipeline only supports {ROBOT}")
    legacy_order = job.get("retarget_output_quaternion_order")
    if legacy_order is not None and legacy_order not in ("xyzw", "wxyz"):
        raise DatasetError(
            "retarget_output_quaternion_order (legacy override for keyless pickles "
            "only) must be 'xyzw' or 'wxyz'"
        )

    dataset_id = str(_require(job, "dataset_id", "job"))
    if not SAFE_ID.fullmatch(dataset_id):
        raise DatasetError("dataset_id may contain only letters, digits, dot, underscore, and dash")
    distribution = job.get("distribution", "internal_rnd")
    license_scope = _require(job, "license_scope", "job")
    if distribution == "client" and license_scope != "commissioned":
        raise DatasetError("client datasets must declare license_scope='commissioned'")
    if distribution not in {"internal_rnd", "client"}:
        raise DatasetError("distribution must be 'internal_rnd' or 'client'")

    subject = _require(job, "subject", "job")
    if not isinstance(subject, dict):
        raise DatasetError("subject must be an object")
    _require(subject, "id", "subject")
    height = float(_require(subject, "height_m", "subject"))
    if not 0.8 <= height <= 2.5:
        raise DatasetError(f"subject.height_m={height} is outside the sanity range 0.8..2.5")
    _require(subject, "calibration_pose", "subject")

    capture = _require(job, "capture", "job")
    if not isinstance(capture, dict):
        raise DatasetError("capture must be an object")
    _require(capture, "software_version", "capture")
    if str(_require(capture, "export_format", "capture")).upper() != "3DSM":
        raise DatasetError("capture.export_format must be '3DSM'")
    fps = float(_require(capture, "fps", "capture"))
    if fps <= 0:
        raise DatasetError("capture.fps must be positive")
    scale = float(_require(capture, "position_scale_to_m", "capture"))
    if scale <= 0:
        raise DatasetError("capture.position_scale_to_m must be positive")

    base = job_file.parent
    calibration = _resolve(base, str(_require(job, "offsets_file", "job")))
    try:
        offsets = json.loads(calibration.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise DatasetError(f"cannot read offsets calibration {calibration}: {exc}") from exc
    if not isinstance(offsets, dict) or not offsets:
        raise DatasetError(f"offsets calibration must be a non-empty JSON object: {calibration}")

    clips = _require(job, "clips", "job")
    if not isinstance(clips, list) or not clips:
        raise DatasetError("clips must be a non-empty list")
    names: set[str] = set()
    validated_clips = []
    for index, clip in enumerate(clips):
        if not isinstance(clip, dict):
            raise DatasetError(f"clips[{index}] must be an object")
        name = str(_require(clip, "name", f"clips[{index}]"))
        if not SAFE_ID.fullmatch(name) or name in names:
            raise DatasetError(f"clip name {name!r} is invalid or duplicated")
        names.add(name)
        source = _resolve(base, str(_require(clip, "bvh", f"clips[{index}]")))
        bvh_info = _validate_bvh(source, fps)
        bvh_info["calibrated_joint_count"] = validate_offsets_for_bvh(offsets, source)
        validated_clips.append({**clip, "name": name, "source": source, "bvh_info": bvh_info})

    return {
        **job,
        "_job_file": job_file,
        "_calibration": calibration,
        "_clips": validated_clips,
        "_fps": fps,
        "_scale": scale,
        "robot": ROBOT,
    }


def load_canonical_motion(path: str | Path, *,
                          quaternion_order: str | None = None) -> dict[str, Any]:
    """Load a trusted local GMR pickle and normalize root rotation to xyzw.

    The pickle must declare ``root_quat_order`` (every writer in this repo
    does; see ``mocap_g1.gmr_pkl``).  ``quaternion_order`` is an explicit
    LEGACY override for keyless pre-contract pickles only (e.g. raw output of
    the old retarget_xsens_headless.py, which stored wxyz); it must agree
    with the declaration when one is present.
    """
    with Path(path).open("rb") as handle:
        motion = pickle.load(handle)
    for key in ("fps", "root_pos", "root_rot", "dof_pos"):
        if key not in motion:
            raise DatasetError(f"{path}: motion pickle lacks {key!r}")
    root_pos = np.asarray(motion["root_pos"], dtype=np.float64)
    root_rot = np.asarray(motion["root_rot"], dtype=np.float64)
    dof_pos = np.asarray(motion["dof_pos"], dtype=np.float64)
    if root_pos.ndim != 2 or root_pos.shape[1] != 3:
        raise DatasetError(f"{path}: root_pos must have shape [T,3]")
    if root_rot.shape != (root_pos.shape[0], 4):
        raise DatasetError(f"{path}: root_rot must have shape [T,4]")
    if dof_pos.shape != (root_pos.shape[0], len(G1_29DOF_JOINTS)):
        raise DatasetError(f"{path}: dof_pos must have shape [T,29]")
    if not all(np.isfinite(array).all() for array in (root_pos, root_rot, dof_pos)):
        raise DatasetError(f"{path}: motion contains non-finite values")
    norms = np.linalg.norm(root_rot, axis=1)
    if np.max(np.abs(norms - 1.0)) > UNIT_QUATERNION_TOLERANCE:
        raise DatasetError(f"{path}: root quaternions are not normalized")
    try:
        root_rot = root_rot_as(motion, "xyzw", legacy_order=quaternion_order, source=path)
    except ValueError as exc:
        raise DatasetError(str(exc)) from exc
    return declare_gmr_root_quat_order({
        "fps": int(motion["fps"]),
        "root_pos": root_pos,
        "root_rot": root_rot,
        "dof_pos": dof_pos,
        "local_body_pos": motion.get("local_body_pos"),
        "link_body_list": motion.get("link_body_list"),
    })


def save_canonical_motion(path: str | Path, motion: dict[str, Any]) -> None:
    if motion.get(ROOT_QUAT_ORDER_KEY) != "xyzw":
        raise DatasetError("canonical motion must declare root_quat_order='xyzw' before saving")
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("wb") as handle:
        pickle.dump(motion, handle, protocol=pickle.HIGHEST_PROTOCOL)


def export_named_csv(path: str | Path, motion: dict[str, Any]) -> None:
    """Export canonical motion with explicit root and joint-name columns."""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "frame", "time_s", "root_x_m", "root_y_m", "root_z_m",
        "root_qx", "root_qy", "root_qz", "root_qw", *G1_29DOF_JOINTS,
    )
    with target.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        fps = float(motion["fps"])
        for frame, (root_pos, root_rot, dof_pos) in enumerate(zip(
            motion["root_pos"], motion["root_rot"], motion["dof_pos"]
        )):
            writer.writerow((frame, frame / fps, *root_pos, *root_rot, *dof_pos))


def _atomic_json(path: Path, value: Any) -> None:
    write_json_atomic(path, value, overwrite=True)


def _run(command: list[str], *, cwd: Path | None = None) -> None:
    result = subprocess.run(command, cwd=cwd, check=False)
    if result.returncode:
        raise DatasetError(f"command failed ({result.returncode}): {' '.join(command)}")


def run_pipeline(
    job: dict[str, Any],
    output_root: str | Path,
    *,
    gmr_root: str | Path | None = None,
    python: str = sys.executable,
    existing_pkl_dir: str | Path | None = None,
    skip_score: bool = False,
) -> Path:
    """Run/finalize every clip and atomically publish a provenance manifest."""
    if skip_score and job.get("distribution", "internal_rnd") == "client":
        raise DatasetError("client dataset jobs cannot use --skip-score")
    output = Path(output_root).resolve() / str(job["dataset_id"])
    if output.exists() and any(output.iterdir()):
        raise DatasetError(f"refusing to overwrite non-empty dataset directory: {output}")
    output.mkdir(parents=True, exist_ok=True)
    pkl_dir = output / "pkl"
    csv_dir = output / "csv"
    score_dir = output / "scores"
    work_dir = output / "work"
    for directory in (pkl_dir, csv_dir, score_dir, work_dir):
        directory.mkdir()

    repo_root = Path(__file__).resolve().parents[1]
    gmr = Path(gmr_root).resolve() if gmr_root else None
    source_pkls = Path(existing_pkl_dir).resolve() if existing_pkl_dir else None
    if source_pkls is None:
        if gmr is None:
            raise DatasetError("--gmr-root is required when retargeting source BVHs")
        script = repo_root / "tools/retarget/retarget_xsens_headless.py"
        if not (gmr / "general_motion_retargeting").is_dir():
            raise DatasetError(f"not a GMR checkout: {gmr}")
    elif not source_pkls.is_dir():
        raise DatasetError(f"existing pkl directory does not exist: {source_pkls}")

    calibration = Path(job["_calibration"])
    # Legacy override only (keyless pre-contract pickles in --existing-pkl-dir);
    # freshly retargeted clips declare root_quat_order themselves.
    legacy_order = job.get("retarget_output_quaternion_order")
    quaternion_order = None if legacy_order is None else str(legacy_order)
    clip_records = []
    for clip in job["_clips"]:
        name = clip["name"]
        raw_pkl = work_dir / f"{name}.raw.pkl"
        command: list[str] | None = None
        if source_pkls is not None:
            candidate = source_pkls / f"{name}.pkl"
            if not candidate.is_file():
                raise DatasetError(f"missing existing motion for clip {name}: {candidate}")
            shutil.copy2(candidate, raw_pkl)
        else:
            clip_work = work_dir / name
            clip_work.mkdir()
            command = [
                python,
                str(script),
                "--gmr-root", str(gmr),
                "--offsets-file", str(calibration),
                "--robot", ROBOT,
                "--scale", str(job["_scale"]),
                "--reset-to-zero",
                "--bvh-format", "3DSM",
                "--bvh-file", str(clip["source"]),
                "--save-path", str(raw_pkl),
            ]
            if clip.get("start_frame") is not None:
                command.extend(("--start", str(int(clip["start_frame"]))))
            if clip.get("end_frame") is not None:
                command.extend(("--end", str(int(clip["end_frame"]))))
            _run(command, cwd=clip_work)

        canonical = load_canonical_motion(raw_pkl, quaternion_order=quaternion_order)
        if abs(float(canonical["fps"]) - float(job["_fps"])) > 0.25:
            raise DatasetError(
                f"{name}: output FPS {canonical['fps']} disagrees with capture FPS {job['_fps']}"
            )
        output_pkl = pkl_dir / f"{name}.pkl"
        output_csv = csv_dir / f"{name}.csv"
        save_canonical_motion(output_pkl, canonical)
        export_named_csv(output_csv, canonical)

        score_path = score_dir / f"{name}.json"
        if not skip_score:
            policy = "anchored" if bool(job.get("grounded", False)) else "adaptive"
            _run([
                python,
                str(repo_root / "eval/score_motion.py"),
                str(output_pkl),
                "--robot", ROBOT,
                "--contact-policy", policy,
                "--out", str(score_path),
                "--json-only",
            ])

        record = {
            "name": name,
            "source": {
                "path": str(clip["source"]),
                "sha256": sha256_file(clip["source"]),
                **clip["bvh_info"],
            },
            "retarget_command": command,
            "frames": int(canonical["dof_pos"].shape[0]),
            "fps": int(canonical["fps"]),
            "pkl": {"path": str(output_pkl.relative_to(output)), "sha256": sha256_file(output_pkl)},
            "csv": {"path": str(output_csv.relative_to(output)), "sha256": sha256_file(output_csv)},
            "score": (
                {"path": str(score_path.relative_to(output)), "sha256": sha256_file(score_path)}
                if score_path.is_file()
                else {"status": "not_run", "reason": "--skip-score"}
            ),
        }
        clip_records.append(record)

    manifest = {
        "schema_version": SCHEMA_VERSION,
        "dataset_id": job["dataset_id"],
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "robot": ROBOT,
        "dof": len(G1_29DOF_JOINTS),
        "joint_names": list(G1_29DOF_JOINTS),
        "root_quaternion_order": "xyzw",
        "distribution": job.get("distribution", "internal_rnd"),
        "license_scope": job["license_scope"],
        "approval_status": "engineering_output_requires_review",
        "subject": job["subject"],
        "capture": job["capture"],
        "offsets": {"path": str(calibration), "sha256": sha256_file(calibration)},
        "job": {"path": str(job["_job_file"]), "sha256": sha256_file(job["_job_file"])},
        "software": {
            "project_commit": _git_revision(repo_root),
            "gmr_commit": _git_revision(gmr) if gmr else None,
            "python": sys.version.split()[0],
            "numpy": np.__version__,
        },
        "clips": clip_records,
    }
    manifest_path = output / "manifest.json"
    _atomic_json(manifest_path, manifest)
    return manifest_path
