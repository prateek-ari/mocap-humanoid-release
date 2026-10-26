#!/usr/bin/env python3
"""Retarget a recorded Xsens 3DSM BVH to any robot registered by GMR.

This runner is intentionally separate from the legacy G1-only tools.  It
loads the robot MJCF selected by GMR, derives the expected qpos width from
that model, and writes the canonical GMR motion dictionary.  GMR's upstream
class does not expose an IK-config argument; ``--ik-config`` is therefore
installed into its in-memory registry for this process only.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import pickle
import re
import shutil
import signal
import sys
import tempfile
import time
from typing import Any, Iterable

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from mocap_g1.dataset import DatasetError, validate_offsets_for_bvh  # noqa: E402
from mocap_g1.gmr_pkl import (  # noqa: E402
    GMR_ROOT_QUAT_ORDER,
    ROOT_QUAT_ORDER_KEY,
    wxyz_to_xyzw,
)
from mocap_g1.io_utils import git_revision as _git_revision, sha256_file  # noqa: E402


class RetargetError(RuntimeError):
    """Fail-closed error raised for invalid setup or retarget output."""


ARM_SOURCE_BODIES = {
    "LeftArm",
    "RightArm",
    "LeftForeArm",
    "RightForeArm",
    "LeftHand",
    "RightHand",
    "LeftShoulder",
    "RightShoulder",
    "LeftElbow",
    "RightElbow",
    "LeftWrist",
    "RightWrist",
}


def _resolve_file(path: Path, label: str) -> Path:
    resolved = path.expanduser().resolve()
    if not resolved.is_file():
        raise RetargetError(f"{label} does not exist or is not a file: {resolved}")
    return resolved


def _hierarchy_joint_names(bvh_path: Path) -> list[str]:
    hierarchy = bvh_path.read_text(encoding="utf-8", errors="replace").split("MOTION", 1)[0]
    names = re.findall(r"^\s*(?:ROOT|JOINT)\s+([^\s{]+)", hierarchy, re.MULTILINE)
    if not names:
        raise RetargetError(f"{bvh_path}: cannot find ROOT/JOINT names in BVH hierarchy")
    return names


def _write_zero_offsets(bvh_path: Path, directory: Path) -> tuple[Path, int]:
    """Create a complete, explicitly preview-only zero-offset calibration."""

    names = _hierarchy_joint_names(bvh_path)
    output = directory / "offsets.zero.preview.json"
    offsets = {name: {"X": 0.0, "Y": 0.0, "Z": 0.0} for name in names}
    output.write_text(json.dumps(offsets, sort_keys=True, indent=2) + "\n", encoding="utf-8")
    return output, len(names)


def _load_offsets(path: Path, bvh_path: Path) -> tuple[dict[str, Any], int, bool]:
    try:
        offsets = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RetargetError(f"cannot read offsets calibration {path}: {exc}") from exc
    if not isinstance(offsets, dict) or not offsets:
        raise RetargetError("offsets calibration must be a non-empty JSON object")
    try:
        joint_count = validate_offsets_for_bvh(offsets, bvh_path)
    except (DatasetError, OSError) as exc:
        raise RetargetError(str(exc)) from exc
    joint_names = _hierarchy_joint_names(bvh_path)
    all_zero = all(
        float(offsets[name][axis]) == 0.0
        for name in joint_names
        for axis in ("X", "Y", "Z")
    )
    return offsets, joint_count, all_zero


def _default_ik_config(gmr_root: Path, src_human: str, robot: str) -> Path | None:
    """Find GMR's conventional config path, including common robot aliases."""

    config_root = gmr_root / "general_motion_retargeting" / "ik_configs"
    candidates = [config_root / f"{src_human}_to_{robot}.json"]
    # The upstream Xsens naming convention uses bvh_xsens; permit callers to
    # request a future alias without changing the runner's source contract.
    if src_human == "bvh_xsens":
        candidates.extend(
            [
                config_root / f"bvh_xsens_to_{robot.removeprefix('booster_')}.json",
                config_root / f"bvh_to_{robot}.json",
            ]
        )
    return next((candidate.resolve() for candidate in candidates if candidate.is_file()), None)


def _install_ik_override(gmr_root: Path, src_human: str, robot: str, config: Path) -> None:
    """Install an IK path into GMR's process-local registry.

    ``motion_retarget.py`` imports the dictionary object from ``params`` at
    module import time. Mutating the nested mapping is consequently enough and
    does not modify the checkout on disk.
    """

    if str(gmr_root) not in sys.path:
        sys.path.insert(0, str(gmr_root))
    try:
        from general_motion_retargeting import params
    except ImportError as exc:
        raise RetargetError(f"cannot import GMR from {gmr_root}: {exc}") from exc
    params.IK_CONFIG_DICT.setdefault(src_human, {})[robot] = str(config)


def _robot_model_info(gmr_root: Path, robot: str) -> tuple[Path, Any, list[str], list[str]]:
    """Load a GMR robot MJCF and return model, qpos width, and joint names."""

    if str(gmr_root) not in sys.path:
        sys.path.insert(0, str(gmr_root))
    try:
        from general_motion_retargeting import params
        import mujoco
    except ImportError as exc:
        raise RetargetError(f"GMR robot model dependencies are unavailable: {exc}") from exc
    try:
        model_path = Path(params.ROBOT_XML_DICT[robot]).resolve()
    except KeyError as exc:
        known = ", ".join(sorted(params.ROBOT_XML_DICT))
        raise RetargetError(f"unknown GMR robot {robot!r}; registered robots: {known}") from exc
    if not model_path.is_file():
        raise RetargetError(f"GMR MJCF for robot {robot!r} does not exist: {model_path}")
    try:
        model = mujoco.MjModel.from_xml_path(str(model_path))
    except Exception as exc:  # MuJoCo uses multiple exception types by version.
        raise RetargetError(f"cannot load MJCF {model_path}: {exc}") from exc
    if model.nq < 7:
        raise RetargetError(f"robot MJCF must have a free-base qpos (nq >= 7), got nq={model.nq}")
    joint_names = []
    hinge_names = []
    for joint_id in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        joint_names.append(name or f"joint_{joint_id}")
        if model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_HINGE:
            hinge_names.append(name or f"joint_{joint_id}")
    return model_path, model, joint_names, hinge_names


def validate_qpos(qpos: np.ndarray, expected_nq: int, *, tolerance: float = 1e-3) -> None:
    """Validate a sequence of MuJoCo qpos rows against a dynamic MJCF width."""

    values = np.asarray(qpos, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != expected_nq:
        raise RetargetError(
            f"GMR output must have shape [frames,{expected_nq}] from the robot MJCF, got {values.shape}"
        )
    if values.shape[0] == 0:
        raise RetargetError("GMR output contains no frames")
    if not np.isfinite(values).all():
        raise RetargetError("GMR output contains non-finite qpos values")
    norms = np.linalg.norm(values[:, 3:7], axis=1)
    if not np.isfinite(norms).all() or np.max(np.abs(norms - 1.0)) > tolerance:
        raise RetargetError(
            "GMR output contains a non-unit root quaternion "
            f"(max deviation {float(np.max(np.abs(norms - 1.0))):.6g})"
        )


def _retarget_all_then_decimate(
    retargeter: Any,
    frames: list[dict[str, Any]],
    frame_step: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Preserve stateful IK sequencing and decimate only completed robot poses."""

    if frame_step <= 0:
        raise RetargetError("frame step must be positive")
    if not frames:
        raise RetargetError("cannot retarget an empty frame sequence")
    source_qpos = np.asarray(
        [retargeter.retarget(frame) for frame in frames], dtype=np.float64
    )
    return source_qpos, np.ascontiguousarray(source_qpos[::frame_step])


def _loader_args(args: argparse.Namespace, bvh_path: Path) -> argparse.Namespace:
    return argparse.Namespace(
        bvh_file=str(bvh_path),
        scale=args.scale,
        start=args.start,
        end=args.end,
        reset_to_zero=args.reset_to_zero,
        bvh_format=args.bvh_format,
    )


def _load_xsens_with_offsets(
    loader: Any, loader_args: argparse.Namespace, offsets_path: Path
) -> tuple[list[dict[str, Any]], float, float]:
    """Run GMR's cwd-sensitive Xsens loader with an explicit offsets file."""

    previous_cwd = Path.cwd()
    with tempfile.TemporaryDirectory(prefix="gmr-xsens-calibration-") as temporary:
        calibration_dir = Path(temporary)
        shutil.copy2(offsets_path, calibration_dir / "offsets.json")
        try:
            os.chdir(calibration_dir)
            return loader(loader_args)
        finally:
            os.chdir(previous_cwd)


def _bvh_frame_time(path: Path) -> float:
    """Read the positive frame time declared in a BVH header."""

    with path.open("r", encoding="utf-8", errors="replace") as handle:
        header = handle.read(2 * 1024 * 1024)
    match = re.search(r"Frame Time:\s*([0-9.eE+-]+)", header)
    if match is None:
        raise RetargetError(f"{path}: cannot parse BVH Frame Time")
    frame_time = float(match.group(1))
    if not np.isfinite(frame_time) or frame_time <= 0:
        raise RetargetError(f"{path}: invalid BVH Frame Time {frame_time!r}")
    return frame_time


def _xsens_bind_height(path: Path, scale: float) -> float:
    """Measure skeleton height from BVH bind offsets, independent of the selected pose."""

    try:
        import general_motion_retargeting.utils.lafan_vendor.utils as lafan_utils
        from general_motion_retargeting.utils.xsens_vendor.BVHParser import Anim, BVHParser
    except ImportError as exc:
        raise RetargetError(f"cannot load GMR Xsens bind-height dependencies: {exc}") from exc
    parser = BVHParser(axis_order="zxy", scale=scale)
    parser.parse(
        path.read_text(encoding="utf-8"), start=1, end=2, reset_to_zero=False
    )
    names = list(parser.names)
    index = {name: position for position, name in enumerate(names)}
    required = ("Head_end_site", "LeftToe_end_site", "RightToe_end_site")
    missing = [name for name in required if name not in index]
    if missing:
        raise RetargetError(f"{path}: BVH hierarchy lacks {', '.join(missing)}")
    quaternions = np.zeros((1, len(names), 4), dtype=np.float64)
    quaternions[..., 0] = 1.0
    positions = np.asarray(parser.offsets, dtype=np.float64)[None, ...].copy()
    positions[0, 0] = 0.0
    animation = Anim(
        quaternions,
        positions,
        np.asarray(parser.offsets, dtype=np.float64),
        np.asarray(parser.parents, dtype=int),
        names,
    )
    _global_quaternions, global_positions = lafan_utils.quat_fk(
        animation.quats, animation.pos, animation.parents
    )
    height = float(
        global_positions[0, index["Head_end_site"], 2]
        - min(
            global_positions[0, index["LeftToe_end_site"], 2],
            global_positions[0, index["RightToe_end_site"], 2],
        )
    )
    if not np.isfinite(height) or not 0.8 <= height <= 2.5:
        raise RetargetError(f"{path}: inferred bind height {height:.4f} m is implausible")
    return height


def _mean_calibration_frame(frames: list[dict[str, Any]]) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Build a robust mean pose from a reviewed neutral-pose window."""

    if not frames:
        raise RetargetError("neutral-pose calibration window contains no frames")
    try:
        from scipy.spatial.transform import Rotation
    except ImportError as exc:
        raise RetargetError(f"neutral-pose calibration requires SciPy: {exc}") from exc
    names = set(frames[0])
    if any(set(frame) != names for frame in frames):
        raise RetargetError("neutral-pose frames do not contain a stable body-name set")
    result: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    for name in sorted(names):
        positions = np.asarray([frame[name][0] for frame in frames], dtype=np.float64)
        rotations = np.asarray([frame[name][1] for frame in frames], dtype=np.float64)
        result[name] = (
            np.median(positions, axis=0),
            Rotation.from_quat(rotations, scalar_first=True)
            .mean()
            .as_quat(scalar_first=True),
        )
    return result


def _configure_position_only_arms(retargeter: Any) -> None:
    """Disable orientation targets for human arm segments in an IK config."""

    for table in (retargeter.ik_match_table1, retargeter.ik_match_table2):
        for entry in table.values():
            if isinstance(entry, list) and len(entry) >= 3 and entry[0] in ARM_SOURCE_BODIES:
                entry[2] = 0.0
    retargeter.setup_retarget_configuration()


def _configure_npose_arms(
    retargeter_class: Any,
    calibration_frame: dict[str, tuple[np.ndarray, np.ndarray]],
    *,
    robot: str,
    human_height: float,
) -> tuple[Any, dict[str, dict[str, Any]]]:
    """Resolve each arm IK branch from a reviewed neutral-pose frame."""

    try:
        import mujoco
        from scipy.spatial.transform import Rotation
    except ImportError as exc:
        raise RetargetError(f"neutral-pose arm calibration dependencies are unavailable: {exc}") from exc

    position_reference = retargeter_class(
        src_human="bvh_xsens",
        tgt_robot=robot,
        actual_human_height=human_height,
        verbose=False,
    )
    _configure_position_only_arms(position_reference)
    reference_qpos = position_reference.retarget(
        {
            name: (position.copy(), quaternion.copy())
            for name, (position, quaternion) in calibration_frame.items()
        }
    )

    retargeter = retargeter_class(
        src_human="bvh_xsens",
        tgt_robot=robot,
        actual_human_height=human_height,
        verbose=False,
    )
    reference_data = mujoco.MjData(retargeter.model)
    reference_data.qpos[:] = reference_qpos
    mujoco.mj_forward(retargeter.model, reference_data)

    derived: dict[str, dict[str, Any]] = {}
    for table_name, table, rotation_offsets in (
        ("ik_match_table1", retargeter.ik_match_table1, retargeter.rot_offsets1),
        ("ik_match_table2", retargeter.ik_match_table2, retargeter.rot_offsets2),
    ):
        for robot_frame, entry in table.items():
            human_body = entry[0]
            if human_body not in ARM_SOURCE_BODIES:
                continue
            if human_body not in calibration_frame:
                raise RetargetError(
                    f"neutral-pose calibration lacks arm body {human_body!r} for {robot_frame!r}"
                )
            try:
                body_id = retargeter.model.body(robot_frame).id
            except KeyError as exc:
                raise RetargetError(
                    f"robot {robot!r} lacks configured arm frame {robot_frame!r}"
                ) from exc
            human_rotation = Rotation.from_quat(
                calibration_frame[human_body][1], scalar_first=True
            )
            robot_rotation = Rotation.from_quat(
                reference_data.xquat[body_id], scalar_first=True
            )
            offset = human_rotation.inv() * robot_rotation
            rotation_offsets[human_body] = offset
            derived[f"{table_name}.{robot_frame}"] = {
                "robot_frame": robot_frame,
                "human_body": human_body,
                "quaternion_wxyz": offset.as_quat(scalar_first=True).tolist(),
            }

    retargeter.configuration.data.qpos[:] = reference_qpos
    mujoco.mj_forward(retargeter.configuration.model, retargeter.configuration.data)
    return retargeter, derived


def _write_json_atomic(path: Path, value: dict[str, Any], *, overwrite: bool) -> None:
    """Write a JSON artifact without exposing a half-written final file."""

    output = path.expanduser().resolve()
    partial = output.with_name(f".{output.name}.partial")
    existing = [str(candidate) for candidate in (output, partial) if candidate.exists()]
    if existing and not overwrite:
        raise RetargetError(
            "refusing to overwrite calibration output; pass --overwrite or choose another path: "
            + ", ".join(existing)
        )
    output.parent.mkdir(parents=True, exist_ok=True)
    if overwrite:
        partial.unlink(missing_ok=True)
    try:
        with partial.open("w", encoding="utf-8") as handle:
            json.dump(value, handle, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(partial, output)
    except Exception:
        partial.unlink(missing_ok=True)
        raise


def run(args: argparse.Namespace) -> dict[str, Any]:
    """Run the recorded retarget and return the written motion dictionary."""

    gmr_root_arg = Path(args.gmr_root).expanduser().resolve()
    gmr_root = _resolve_file(
        gmr_root_arg / "general_motion_retargeting" / "__init__.py", "GMR root"
    ).parent.parent
    bvh_path = _resolve_file(args.bvh_file, "BVH")
    output = args.save_path.expanduser().resolve()
    partial_output = output.with_name(f".{output.name}.partial")
    existing = [str(path) for path in (output, partial_output) if path.exists()]
    if existing and not args.overwrite:
        raise RetargetError(
            "refusing to overwrite motion output; pass --overwrite or choose another path: "
            + ", ".join(existing)
        )
    if args.calibration_output is not None:
        calibration_output = args.calibration_output.expanduser().resolve()
        calibration_partial = calibration_output.with_name(
            f".{calibration_output.name}.partial"
        )
        calibration_existing = [
            str(path) for path in (calibration_output, calibration_partial) if path.exists()
        ]
        if calibration_existing and not args.overwrite:
            raise RetargetError(
                "refusing to overwrite calibration output; pass --overwrite or choose another path: "
                + ", ".join(calibration_existing)
            )
    output.parent.mkdir(parents=True, exist_ok=True)
    if args.overwrite:
        partial_output.unlink(missing_ok=True)
    preview_calibration = bool(args.allow_zero_offsets)
    temporary_preview_dir: tempfile.TemporaryDirectory[str] | None = None
    offsets_metadata_path: str
    if args.offsets_file is None:
        if not args.allow_zero_offsets:
            raise RetargetError("--offsets-file is required unless --allow-zero-offsets is set")
        temporary_preview_dir = tempfile.TemporaryDirectory(prefix="gmr-zero-offsets-")
        offsets_path, _ = _write_zero_offsets(bvh_path, Path(temporary_preview_dir.name))
        offsets_metadata_path = "<generated-zero-offsets-from-bvh>"
    else:
        offsets_path = _resolve_file(args.offsets_file, "offsets calibration")
        offsets_metadata_path = str(offsets_path)
    if not args.robot or not str(args.robot).strip():
        raise RetargetError("--robot must be explicit and non-empty")
    if args.scale <= 0 or not np.isfinite(args.scale):
        raise RetargetError("--scale must be a positive finite number")
    if args.human_height is not None and not 0.8 <= args.human_height <= 2.5:
        raise RetargetError("--human-height must be between 0.8 and 2.5 metres")
    if args.start is not None and args.start < 0:
        raise RetargetError("--start must be non-negative")
    if args.end is not None and args.end <= 0:
        raise RetargetError("--end must be positive")
    _, calibrated_joint_count, all_zero_offsets = _load_offsets(offsets_path, bvh_path)
    if all_zero_offsets and not args.allow_zero_offsets:
        raise RetargetError(
            "offsets calibration is all zero; use --allow-zero-offsets for an explicitly preview-only run"
        )
    if args.arm_mode == "calibrated" and (args.allow_zero_offsets or all_zero_offsets):
        raise RetargetError("--arm-mode calibrated is not allowed with zero/preview offsets")
    arm_mode = args.arm_mode or ("position-only" if args.allow_zero_offsets else "calibrated")
    if not 0.0 <= args.arm_abduction <= 45.0:
        raise RetargetError("--arm-abduction must be between 0 and 45 degrees")
    if args.arm_abduction and arm_mode != "config":
        raise RetargetError("--arm-abduction requires --arm-mode config")
    if args.calibration_output is not None and arm_mode != "npose":
        raise RetargetError("--calibration-output requires --arm-mode npose")
    if arm_mode == "npose":
        if args.calibration_bvh_file is None:
            raise RetargetError("--arm-mode npose requires --calibration-bvh-file")
        if not args.accept_reviewed_calibration_window:
            raise RetargetError(
                "--arm-mode npose requires --accept-reviewed-calibration-window"
            )
        if args.calibration_start_seconds < 0 or args.calibration_seconds <= 0:
            raise RetargetError(
                "neutral-pose calibration start must be non-negative and duration must be positive"
            )

    ik_path = (
        _resolve_file(args.ik_config, "IK config")
        if args.ik_config is not None
        else _default_ik_config(gmr_root, "bvh_xsens", args.robot)
    )
    if ik_path is None:
        raise RetargetError(
            f"no installed IK config for bvh_xsens -> {args.robot}; provide --ik-config"
        )
    model_path, model, joint_names, hinge_names = _robot_model_info(gmr_root, args.robot)
    _install_ik_override(gmr_root, "bvh_xsens", args.robot, ik_path)
    try:
        from general_motion_retargeting import GeneralMotionRetargeting
        from general_motion_retargeting.utils.xsens import load_xsens_file
    except ImportError as exc:
        raise RetargetError(f"cannot import GMR runtime from {gmr_root}: {exc}") from exc

    loader_args = _loader_args(args, bvh_path)
    started = time.perf_counter()
    frames, loader_human_height, frame_time = _load_xsens_with_offsets(
        load_xsens_file, loader_args, offsets_path
    )
    if not frames or frame_time <= 0:
        raise RetargetError("Xsens loader returned no frames or an invalid frame time")
    bind_height = _xsens_bind_height(bvh_path, args.scale)
    human_height = float(args.human_height or bind_height)
    source_frame_count = len(frames)
    if not frames[:: args.frame_step]:
        raise RetargetError("--frame-step removed every frame")
    if arm_mode == "config":
        # The live stream's arm frames: what the IK config's arm offsets are reviewed against.
        from mocap_g1.xsens_live import _quat_multiply_wxyz
        from tools.mxtp02_replayer import arm_bind_corrections

        try:
            corrections = arm_bind_corrections(bvh_path)
        except ValueError as exc:
            raise RetargetError(str(exc)) from exc
        for frame in frames:
            for bone, correction in corrections.items():
                position, orientation = frame[bone]
                frame[bone] = (position, _quat_multiply_wxyz(np.asarray(orientation), correction))

    arm_calibration: dict[str, Any] | None = None
    if arm_mode == "npose":
        calibration_bvh_path = _resolve_file(args.calibration_bvh_file, "calibration BVH")
        motion_hierarchy = _hierarchy_joint_names(bvh_path)
        calibration_hierarchy = _hierarchy_joint_names(calibration_bvh_path)
        if calibration_hierarchy != motion_hierarchy:
            raise RetargetError("motion and calibration BVH joint hierarchies differ")
        _load_offsets(offsets_path, calibration_bvh_path)
        calibration_frame_time = _bvh_frame_time(calibration_bvh_path)
        # Recorded Xsens exports treat frame 0 as an exporter frame. The
        # project motion path starts at frame 1, so time zero for calibration
        # is also frame 1.
        calibration_start = 1 + int(
            round(args.calibration_start_seconds / calibration_frame_time)
        )
        calibration_count = max(
            1, int(round(args.calibration_seconds / calibration_frame_time))
        )
        calibration_end = calibration_start + calibration_count
        calibration_args = argparse.Namespace(
            bvh_file=str(calibration_bvh_path),
            scale=args.scale,
            start=calibration_start,
            end=calibration_end,
            reset_to_zero=args.reset_to_zero,
            bvh_format=args.bvh_format,
        )
        calibration_frames, calibration_loader_height, loaded_calibration_frame_time = (
            _load_xsens_with_offsets(load_xsens_file, calibration_args, offsets_path)
        )
        calibration_frames = calibration_frames[:: args.frame_step]
        if not calibration_frames:
            raise RetargetError("neutral-pose calibration window contains no frames")
        if abs(loaded_calibration_frame_time - calibration_frame_time) > 1e-12:
            raise RetargetError("calibration BVH frame time changed while loading")
        calibration_bind_height = _xsens_bind_height(calibration_bvh_path, args.scale)
        if abs(calibration_bind_height - bind_height) > 1e-6:
            raise RetargetError(
                "motion and calibration BVH bind heights differ: "
                f"{bind_height:.6f} versus {calibration_bind_height:.6f} m"
            )
        calibration_frame = _mean_calibration_frame(calibration_frames)
        retargeter, derived_offsets = _configure_npose_arms(
            GeneralMotionRetargeting,
            calibration_frame,
            robot=args.robot,
            human_height=human_height,
        )
        arm_calibration = {
            "method": "npose_gmr_arm_frame_alignment",
            "reviewed_window_accepted": True,
            "path": str(calibration_bvh_path),
            "sha256": sha256_file(calibration_bvh_path),
            "start_seconds": float(args.calibration_start_seconds),
            "duration_seconds": float(args.calibration_seconds),
            "start_frame": calibration_start,
            "frames_used": len(calibration_frames),
            "frame_time_s": float(calibration_frame_time),
            "loader_reported_human_height_m": float(calibration_loader_height),
            "bind_height_m": float(calibration_bind_height),
            "derived_offsets": derived_offsets,
        }
    else:
        retargeter = GeneralMotionRetargeting(
            src_human="bvh_xsens",
            tgt_robot=args.robot,
            actual_human_height=human_height,
            verbose=False,
        )
        if arm_mode == "position-only":
            _configure_position_only_arms(retargeter)
        if args.arm_abduction:
            from mocap_g1.live_arm_clearance import BVH_ARMS, ArmAbduction

            ArmAbduction(retargeter, args.arm_abduction, chest="Chest4", root="Hips",
                         arms=BVH_ARMS)
    # GMR is stateful: each pose seeds the next IK solve.  Retarget every
    # captured frame, then decimate the finished trajectory.  Decimating the
    # human frames first creates large solver jumps and can change IK branches.
    source_qpos, qpos = _retarget_all_then_decimate(
        retargeter, frames, args.frame_step
    )
    validate_qpos(qpos, int(model.nq))
    # MuJoCo qpos is wxyz; canonical GMR motion PKLs are xyzw.
    root_rot_xyzw = np.ascontiguousarray(wxyz_to_xyzw(qpos[:, 3:7]))
    dof_pos = np.ascontiguousarray(qpos[:, 7:])
    source_fps = float(1.0 / frame_time)
    fps = float(source_fps / args.frame_step)
    elapsed = time.perf_counter() - started
    metadata = {
        "schema_version": 2,
        "producer": "tools/retarget/retarget_robot_recorded.py",
        "robot": args.robot,
        "source": {
            "path": str(bvh_path),
            "sha256": sha256_file(bvh_path),
            "format": args.bvh_format,
            "start": args.start,
            "end": args.end,
            "scale": float(args.scale),
            "reset_to_zero": bool(args.reset_to_zero),
            "frames": len(qpos),
            "source_frames": source_frame_count,
            "retargeted_source_frames": len(source_qpos),
            "frame_step": int(args.frame_step),
            "source_fps": source_fps,
            "output_fps": fps,
            "frame_time_s": float(frame_time),
            "human_height_m": float(human_height),
            "bind_height_m": float(bind_height),
            "loader_reported_human_height_m": float(loader_human_height),
        },
        "calibration": {
            "offsets_path": offsets_metadata_path,
            "offsets_sha256": sha256_file(offsets_path),
            "joint_count": calibrated_joint_count,
            "preview_calibration": preview_calibration or all_zero_offsets,
            "quality_approved": not (preview_calibration or all_zero_offsets),
            "arm": arm_calibration,
        },
        "gmr": {
            "root": str(gmr_root),
            "revision": _git_revision(gmr_root),
            "ik_config_path": str(ik_path),
            "ik_config_sha256": sha256_file(ik_path),
        },
        "mjcf": {
            "path": str(model_path),
            "sha256": sha256_file(model_path),
            "nq": int(model.nq),
            "nv": int(model.nv),
            "joint_names": joint_names,
            "hinge_joint_names": hinge_names,
        },
        "layout": {
            "qpos": "root_pos[3] + root_quat_wxyz[4] + dof_pos[nq-7]",
            "root_rot": "xyzw",
            "dof_joint_names": hinge_names,
            "arm_mode": arm_mode,
            "arm_abduction_deg": float(args.arm_abduction),
        },
        "runtime": {
            "elapsed_s": elapsed,
            "throughput_fps": len(source_qpos) / max(elapsed, 1e-9),
        },
    }
    motion = {
        "fps": fps,
        "root_pos": np.ascontiguousarray(qpos[:, :3]),
        "root_rot": root_rot_xyzw,
        ROOT_QUAT_ORDER_KEY: GMR_ROOT_QUAT_ORDER,
        "dof_pos": dof_pos,
        "local_body_pos": None,
        "link_body_list": None,
        "metadata": metadata,
    }
    try:
        with partial_output.open("wb") as handle:
            pickle.dump(motion, handle, protocol=pickle.HIGHEST_PROTOCOL)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(partial_output, output)
    except Exception:
        partial_output.unlink(missing_ok=True)
        raise
    if arm_calibration is not None and args.calibration_output is not None:
        _write_json_atomic(
            args.calibration_output,
            {
                "schema_version": 1,
                "robot": args.robot,
                "human_height_m": human_height,
                **arm_calibration,
            },
            overwrite=args.overwrite,
        )
    if temporary_preview_dir is not None:
        temporary_preview_dir.cleanup()
    return motion


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gmr-root", required=True, type=Path)
    parser.add_argument("--bvh-file", "--bvh_file", required=True, type=Path)
    parser.add_argument("--offsets-file", "--offsets_file", type=Path)
    parser.add_argument("--save-path", "--save_path", required=True, type=Path)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--robot", required=True, help="GMR robot registry key, e.g. booster_t1_29dof")
    parser.add_argument("--ik-config", "--ik_config", type=Path, help="Explicit GMR IK JSON override")
    parser.add_argument("--scale", type=float, default=0.01)
    parser.add_argument("--start", type=int)
    parser.add_argument("--end", type=int)
    parser.add_argument("--frame-step", type=int, default=1, help="Keep every Nth loaded BVH frame")
    parser.add_argument(
        "--human-height",
        type=float,
        help="Measured human height in metres; defaults to the BVH bind-skeleton height",
    )
    parser.add_argument(
        "--allow-zero-offsets",
        action="store_true",
        help="Generate complete zero offsets for preview-only runs; output is not quality-approved",
    )
    parser.add_argument(
        "--arm-mode",
        choices=("position-only", "npose", "calibrated", "config"),
        help="Arm handling; npose derives robot-specific orientations from a reviewed neutral pose; "
        "config keeps the IK config's arm orientations, as the live tool does (allowed for preview)",
    )
    parser.add_argument(
        "--arm-abduction",
        type=float,
        default=0.0,
        metavar="DEGREES",
        help="with --arm-mode config: rotate arm targets outward about the shoulders, as the "
        "live tool does (mocap_g1/live_arm_clearance.py)",
    )
    parser.add_argument("--calibration-bvh-file", type=Path)
    parser.add_argument("--calibration-start-seconds", type=float, default=0.0)
    parser.add_argument("--calibration-seconds", type=float, default=1.0)
    parser.add_argument("--calibration-output", type=Path)
    parser.add_argument(
        "--accept-reviewed-calibration-window",
        action="store_true",
        help="Confirm that the neutral-pose interval was visually reviewed",
    )
    parser.add_argument("--bvh-format", "--bvh_format", choices=("3DSM",), default="3DSM")
    parser.add_argument("--reset-to-zero", "--reset_to_zero", action="store_true")
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if hasattr(signal, "SIGBREAK"):  # Windows Ctrl+Break -> same path as Ctrl+C
        signal.signal(signal.SIGBREAK, signal.default_int_handler)
    try:
        if args.frame_step <= 0:
            raise RetargetError("--frame-step must be a positive integer")
        motion = run(args)
    except (RetargetError, OSError, ValueError) as exc:
        parser.error(str(exc))
    metadata = motion["metadata"]
    print(
        json.dumps(
            {
                "frames": int(len(motion["dof_pos"])),
                "fps": motion["fps"],
                "robot": metadata["robot"],
                "nq": metadata["mjcf"]["nq"],
                "output": str(args.save_path.expanduser().resolve()),
                "ik_config_sha256": metadata["gmr"]["ik_config_sha256"],
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
