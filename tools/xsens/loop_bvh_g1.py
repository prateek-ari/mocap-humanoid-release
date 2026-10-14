#!/usr/bin/env python3
"""Retarget an Xsens 3DSM BVH once, cache it, and loop it in MuJoCo."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import tempfile
import time

import numpy as np


RUNNING = True
CACHE_VERSION = 6


def _stop(_signum: int, _frame: object) -> None:
    global RUNNING
    RUNNING = False


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _runtime_identity(gmr_root: Path) -> dict[str, str]:
    """Hash the pinned code/configuration that determines the cached G1 poses."""
    inputs = {
        "gmr_commit": gmr_root / ".git" / "HEAD",
        "ik_config_sha256": (
            gmr_root
            / "general_motion_retargeting"
            / "ik_configs"
            / "bvh_xsens_to_g1.json"
        ),
        "g1_model_sha256": gmr_root / "assets" / "unitree_g1" / "g1_mocap_29dof.xml",
        "package_init_sha256": gmr_root / "general_motion_retargeting" / "__init__.py",
        "retarget_code_sha256": (
            gmr_root / "general_motion_retargeting" / "motion_retarget.py"
        ),
    }
    missing = [str(path) for path in inputs.values() if not path.is_file()]
    if missing:
        raise ValueError(f"GMR runtime identity inputs are missing: {', '.join(missing)}")
    return {
        name: path.read_text(encoding="utf-8").strip()
        if name == "gmr_commit"
        else _sha256(path)
        for name, path in inputs.items()
    }


def _load_offsets(path: Path | None, names: list[str], *, allow_zero: bool) -> tuple[np.ndarray, str]:
    if path is None:
        if not allow_zero:
            raise ValueError("provide --offsets-file, or explicitly use --allow-zero-offsets for a preview")
        print("WARNING: using zero BVH rotation offsets; this is a structural preview, not calibrated output")
        return np.zeros((len(names), 3), dtype=np.float64), "zero-preview"

    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read offsets file {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError("offsets file must contain a JSON object")

    result = np.zeros((len(names), 3), dtype=np.float64)
    missing: list[str] = []
    for index, name in enumerate(names):
        if name.endswith("_end_site"):
            continue
        channels = value.get(name)
        if not isinstance(channels, dict):
            missing.append(name)
            continue
        try:
            values = [channels[axis] for axis in ("X", "Y", "Z")]
            if any(isinstance(value, bool) for value in values):
                raise TypeError("boolean offsets are not numeric calibration values")
            result[index] = [float(value) for value in values]
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"offsets for {name} must contain finite numeric X/Y/Z values") from exc
    if missing:
        raise ValueError(f"offsets file is missing articulated joints: {', '.join(missing)}")
    if not np.isfinite(result).all():
        raise ValueError("offsets file contains a non-finite value")
    return result, _sha256(path)


def _bind_height(parser: object, Anim: object, quat_fk: object) -> float:
    names = list(parser.names)
    index = {name: position for position, name in enumerate(names)}
    required = ("Head_end_site", "LeftToe_end_site", "RightToe_end_site")
    if any(name not in index for name in required):
        raise ValueError("BVH hierarchy lacks the Xsens head/toe end sites needed for height scaling")
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
    _global_quaternions, global_positions = quat_fk(
        animation.quats, animation.pos, animation.parents
    )
    head = global_positions[0, index["Head_end_site"], 2]
    toes = min(
        global_positions[0, index["LeftToe_end_site"], 2],
        global_positions[0, index["RightToe_end_site"], 2],
    )
    height = float(head - toes)
    if not 0.8 <= height <= 2.5:
        raise ValueError(f"inferred bind-pose height {height:.4f} m is implausible")
    return height


def _cache_metadata(args: argparse.Namespace, source_sha256: str, offsets_key: str) -> dict[str, object]:
    return {
        "cache_version": CACHE_VERSION,
        "source_sha256": source_sha256,
        "offsets_key": offsets_key,
        "robot": args.robot,
        "scale": args.scale,
        "start": args.start,
        "end": args.end,
        "frame_step": args.frame_step,
        "human_height_m": args.human_height,
        "arm_mode": args.resolved_arm_mode,
        "calibration_source_sha256": args.calibration_source_sha256,
        "calibration_start_seconds": args.calibration_start_seconds,
        "calibration_seconds": args.calibration_seconds,
        "reset_to_zero": True,
        "root_quaternion_order": "wxyz",
        "gmr_runtime": args.gmr_runtime_identity,
    }


def _arm_frames() -> tuple[str, ...]:
    return (
        "left_shoulder_yaw_link",
        "left_elbow_link",
        "left_wrist_yaw_link",
        "right_shoulder_yaw_link",
        "right_elbow_link",
        "right_wrist_yaw_link",
    )


def _resolve_arm_mode(
    requested: str, *, has_offsets: bool, has_calibration_bvh: bool, calibration_seconds: float
) -> str:
    if requested != "auto":
        return requested
    if has_calibration_bvh and calibration_seconds > 0:
        return "npose"
    # Merely finding an offsets file is not evidence that a human reviewed it.
    # Full orientation must therefore be an explicit --arm-mode calibrated choice.
    return "position-only"


def _configure_position_only_arms(retargeter: object, *, announce: bool = True) -> None:
    for table in (retargeter.ik_match_table1, retargeter.ik_match_table2):
        for frame_name in _arm_frames():
            if frame_name in table:
                table[frame_name][2] = 0.0
    retargeter.setup_retarget_configuration()
    if announce:
        print(
            "Preview arm mode: position-only shoulder/elbow/wrist targets; "
            "orientation tracking is disabled until offsets are calibrated"
        )


def _copy_frame(frame: dict[str, tuple[np.ndarray, np.ndarray]]) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    return {name: (position.copy(), quaternion.copy()) for name, (position, quaternion) in frame.items()}


def _mean_frame(
    bones: list[str], global_positions: np.ndarray, global_quaternions: np.ndarray, frame_count: int
) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    from scipy.spatial.transform import Rotation

    if frame_count < 1 or frame_count > len(global_positions):
        raise ValueError(f"invalid N-pose frame count {frame_count}")
    result: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    for index, bone in enumerate(bones):
        position = np.median(global_positions[:frame_count, index], axis=0)
        quaternion = Rotation.from_quat(
            global_quaternions[:frame_count, index], scalar_first=True
        ).mean().as_quat(scalar_first=True)
        result[bone] = (position, quaternion)
    result["LeftFootMod"] = (result["LeftAnkle"][0].copy(), result["LeftAnkle"][1].copy())
    result["RightFootMod"] = (result["RightAnkle"][0].copy(), result["RightAnkle"][1].copy())
    return result


def _configure_npose_arms(
    GeneralMotionRetargeting: object,
    calibration_frame: dict[str, tuple[np.ndarray, np.ndarray]],
    *,
    robot: str,
    human_height: float,
) -> tuple[object, dict[str, dict[str, object]]]:
    import mujoco
    from scipy.spatial.transform import Rotation

    position_reference = GeneralMotionRetargeting(
        src_human="bvh_xsens",
        tgt_robot=robot,
        actual_human_height=human_height,
        verbose=False,
    )
    _configure_position_only_arms(position_reference, announce=False)
    reference_qpos = position_reference.retarget(_copy_frame(calibration_frame))

    retargeter = GeneralMotionRetargeting(
        src_human="bvh_xsens",
        tgt_robot=robot,
        actual_human_height=human_height,
        verbose=False,
    )
    reference_data = mujoco.MjData(retargeter.model)
    reference_data.qpos[:] = reference_qpos
    mujoco.mj_forward(retargeter.model, reference_data)

    derived: dict[str, dict[str, object]] = {}
    table_offsets = (
        (retargeter.ik_match_table1, retargeter.rot_offsets1),
        (retargeter.ik_match_table2, retargeter.rot_offsets2),
    )
    for table, rotation_offsets in table_offsets:
        for robot_frame in _arm_frames():
            human_body = table[robot_frame][0]
            human_rotation = Rotation.from_quat(
                calibration_frame[human_body][1], scalar_first=True
            )
            robot_rotation = Rotation.from_quat(
                reference_data.xquat[retargeter.model.body(robot_frame).id], scalar_first=True
            )
            offset = human_rotation.inv() * robot_rotation
            rotation_offsets[human_body] = offset
            derived[robot_frame] = {
                "human_body": human_body,
                "quaternion_wxyz": offset.as_quat(scalar_first=True).tolist(),
            }

    retargeter.configuration.data.qpos[:] = reference_qpos
    mujoco.mj_forward(retargeter.configuration.model, retargeter.configuration.data)
    print("N-pose arm mode: derived independent left/right orientation offsets from the neutral take")
    return retargeter, derived


def _write_json(path: Path, value: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def _read_cache(path: Path, expected: dict[str, object]) -> tuple[np.ndarray, float] | None:
    if not path.is_file():
        return None
    try:
        with np.load(path, allow_pickle=False) as archive:
            metadata = json.loads(str(archive["metadata"].item()))
            qpos = np.asarray(archive["qpos"], dtype=np.float64)
            fps = float(archive["fps"].item())
    except (OSError, KeyError, ValueError, json.JSONDecodeError) as exc:
        print(f"Ignoring unreadable cache {path}: {exc}")
        return None
    if any(metadata.get(key) != value for key, value in expected.items()):
        print(f"Ignoring stale cache {path}")
        return None
    if qpos.ndim != 2 or qpos.shape[1] != 36 or not np.isfinite(qpos).all() or fps <= 0:
        print(f"Ignoring invalid cache arrays in {path}")
        return None
    quaternion_norms = np.linalg.norm(qpos[:, 3:7], axis=1)
    if np.max(np.abs(quaternion_norms - 1.0)) > 1e-3:
        print(f"Ignoring cache with non-unit root quaternions in {path}")
        return None
    print(f"Loaded {len(qpos)} cached G1 frames from {path}")
    return qpos, fps


def _write_cache(path: Path, metadata: dict[str, object], qpos: np.ndarray, fps: float) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb", prefix=f".{path.name}.", suffix=".tmp", dir=path.parent, delete=False
        ) as handle:
            temporary_name = handle.name
            np.savez_compressed(
                handle,
                metadata=np.asarray(json.dumps(metadata, sort_keys=True)),
                qpos=qpos,
                fps=np.asarray(fps, dtype=np.float64),
            )
        os.replace(temporary_name, path)
    finally:
        if temporary_name is not None:
            Path(temporary_name).unlink(missing_ok=True)
    print(f"Saved G1 cache to {path}")


def _retarget(args: argparse.Namespace, metadata_seed: dict[str, object]) -> tuple[np.ndarray, float, dict[str, object]]:
    from general_motion_retargeting import GeneralMotionRetargeting
    import general_motion_retargeting.utils.lafan_vendor.utils as lafan_utils
    from general_motion_retargeting.utils.xsens_vendor.BVHParser import Anim, BVHParser

    text = args.bvh_file.read_text(encoding="utf-8")
    parser = BVHParser(axis_order="zxy", scale=args.scale)
    rotations, _positions = parser.parse(
        text, start=args.start, end=args.end, reset_to_zero=False
    )
    offsets, offsets_key = _load_offsets(
        args.offsets_file, list(parser.names), allow_zero=args.allow_zero_offsets
    )
    if offsets_key != metadata_seed["offsets_key"]:
        raise RuntimeError("offsets changed while the BVH was being prepared")
    rotations = rotations + offsets[None, ...]
    quaternions, positions, skeleton_offsets, parents = parser._MOTION_data_post_processing(
        rotations, np.copy(parser.positions), reset_to_zero=True
    )
    quaternions = quaternions[:: args.frame_step]
    positions = positions[:: args.frame_step]
    if not len(quaternions):
        raise ValueError("selected BVH frame range is empty")
    animation = Anim(quaternions, positions, skeleton_offsets, parents, parser.names)
    global_quaternions, global_positions = lafan_utils.quat_fk(
        animation.quats, animation.pos, animation.parents
    )
    human_height = args.human_height or _bind_height(parser, Anim, lafan_utils.quat_fk)
    fps = (1.0 / parser.frame_time) / args.frame_step
    print(
        f"Preparing {len(quaternions)} G1 frames at {fps:.3f} FPS "
        f"from bind height {human_height:.4f} m"
    )

    derived_calibration: dict[str, dict[str, object]] | None = None
    if args.resolved_arm_mode == "npose":
        if args.calibration_bvh_file == args.bvh_file:
            calibration_animation = animation
            calibration_global_quaternions = global_quaternions
            calibration_global_positions = global_positions
            calibration_fps = fps
        else:
            calibration_parser = BVHParser(axis_order="zxy", scale=args.scale)
            calibration_rotations, _calibration_positions = calibration_parser.parse(
                args.calibration_bvh_file.read_text(encoding="utf-8"),
                start=1,
                end=None,
                reset_to_zero=False,
            )
            if list(calibration_parser.names) != list(parser.names):
                raise ValueError("calibration and motion BVH hierarchies differ")
            calibration_rotations = calibration_rotations + offsets[None, ...]
            (
                calibration_quaternions,
                calibration_positions,
                calibration_skeleton_offsets,
                calibration_parents,
            ) = calibration_parser._MOTION_data_post_processing(
                calibration_rotations,
                np.copy(calibration_parser.positions),
                reset_to_zero=True,
            )
            calibration_quaternions = calibration_quaternions[:: args.frame_step]
            calibration_positions = calibration_positions[:: args.frame_step]
            calibration_animation = Anim(
                calibration_quaternions,
                calibration_positions,
                calibration_skeleton_offsets,
                calibration_parents,
                calibration_parser.names,
            )
            calibration_global_quaternions, calibration_global_positions = lafan_utils.quat_fk(
                calibration_animation.quats,
                calibration_animation.pos,
                calibration_animation.parents,
            )
            calibration_fps = (1.0 / calibration_parser.frame_time) / args.frame_step
        calibration_start_frame = int(round(args.calibration_start_seconds * calibration_fps))
        if calibration_start_frame >= len(calibration_global_positions):
            raise ValueError("N-pose calibration start is beyond the end of the calibration BVH")
        calibration_frame_count = min(
            len(calibration_global_positions) - calibration_start_frame,
            max(1, int(round(args.calibration_seconds * calibration_fps))),
        )
        calibration_frame = _mean_frame(
            list(calibration_animation.bones),
            calibration_global_positions[calibration_start_frame:],
            calibration_global_quaternions[calibration_start_frame:],
            calibration_frame_count,
        )
        retargeter, derived_calibration = _configure_npose_arms(
            GeneralMotionRetargeting,
            calibration_frame,
            robot=args.robot,
            human_height=human_height,
        )
        if args.calibration_output is not None:
            _write_json(
                args.calibration_output,
                {
                    "schema_version": 1,
                    "method": "npose_gmr_arm_frame_alignment",
                    "calibration_bvh_sha256": args.calibration_source_sha256,
                    "calibration_start_seconds": args.calibration_start_seconds,
                    "calibration_seconds": args.calibration_seconds,
                    "calibration_start_frame": calibration_start_frame,
                    "calibration_frames_used": calibration_frame_count,
                    "calibration_fps": calibration_fps,
                    "human_height_m": human_height,
                    "robot": args.robot,
                    "arm_offsets": derived_calibration,
                },
            )
            print(f"Saved N-pose arm calibration to {args.calibration_output}")
    else:
        retargeter = GeneralMotionRetargeting(
            src_human="bvh_xsens",
            tgt_robot=args.robot,
            actual_human_height=human_height,
            verbose=False,
        )
        if args.resolved_arm_mode == "position-only":
            _configure_position_only_arms(retargeter)
    qpos = np.empty((len(quaternions), 36), dtype=np.float64)
    started = time.perf_counter()
    for frame_index in range(len(quaternions)):
        frame = {
            bone: (global_positions[frame_index, index], global_quaternions[frame_index, index])
            for index, bone in enumerate(animation.bones)
        }
        frame["LeftFootMod"] = (frame["LeftAnkle"][0].copy(), frame["LeftAnkle"][1])
        frame["RightFootMod"] = (frame["RightAnkle"][0].copy(), frame["RightAnkle"][1])
        qpos[frame_index] = retargeter.retarget(frame)
        completed = frame_index + 1
        if completed == 1 or completed % 250 == 0 or completed == len(quaternions):
            elapsed = time.perf_counter() - started
            print(f"Retargeted {completed}/{len(quaternions)} frames ({completed / elapsed:.1f} FPS)")
    if not np.isfinite(qpos).all():
        raise RuntimeError("GMR produced a non-finite pose")
    norms = np.linalg.norm(qpos[:, 3:7], axis=1)
    if np.max(np.abs(norms - 1.0)) > 1e-3:
        raise RuntimeError("GMR produced a non-unit root quaternion")
    metadata = dict(metadata_seed)
    metadata["resolved_human_height_m"] = human_height
    if derived_calibration is not None:
        metadata["derived_arm_offsets"] = derived_calibration
    return qpos, fps, metadata


def _partial_video_path(path: Path) -> Path:
    return path.with_name(f".{path.stem}.partial{path.suffix}")


def _play(args: argparse.Namespace, qpos: np.ndarray, fps: float) -> None:
    from general_motion_retargeting import RobotMotionViewer

    playback_fps = args.playback_fps or fps
    if args.loop_mode == "ping-pong" and len(qpos) > 2:
        playback = np.concatenate((qpos, qpos[-2:0:-1]), axis=0)
    else:
        playback = qpos
    if args.record_video and args.video_frame_step > 1:
        playback = playback[:: args.video_frame_step]
        playback_fps /= args.video_frame_step
    final_video_path = args.video_path
    partial_video_path = (
        _partial_video_path(final_video_path) if final_video_path is not None else None
    )
    viewer = RobotMotionViewer(
        robot_type=args.robot,
        camera_follow=True,
        motion_fps=playback_fps,
        transparent_robot=0,
        record_video=args.record_video,
        video_path=str(partial_video_path) if partial_video_path is not None else None,
        video_width=args.video_width,
        video_height=args.video_height,
    )
    loops = 0
    print(
        f"Looping {len(playback)} G1 frames in {args.loop_mode} mode at {playback_fps:.3f} FPS "
        f"({len(playback) / playback_fps:.2f} s/loop). "
        "Close the MuJoCo window or press Ctrl+C to stop."
    )
    rendered_frames = 0
    completed = False
    stop_requested = False
    close_ok = True
    try:
        while RUNNING and (args.loops == 0 or loops < args.loops):
            for pose in playback:
                if not RUNNING or not viewer.viewer.is_running():
                    stop_requested = True
                    break
                viewer.step(
                    root_pos=pose[:3],
                    root_rot=pose[3:7],
                    dof_pos=pose[7:],
                    # Offline export should encode at playback_fps without waiting
                    # for wall-clock playback. Interactive review remains real time.
                    rate_limit=not args.record_video,
                    follow_camera=True,
                )
                rendered_frames += 1
            if stop_requested:
                break
            loops += 1
            print(f"Completed loop {loops}")
        completed = not stop_requested and args.loops > 0 and loops == args.loops
    finally:
        try:
            viewer.close()
        except Exception as exc:
            close_ok = False
            print(f"Viewer shutdown warning: {exc}")
    if args.record_video:
        expected_frames = len(playback) * args.loops
        if not completed or rendered_frames != expected_frames or not close_ok:
            raise RuntimeError(
                f"video export stopped after {rendered_frames}/{expected_frames} frames; "
                f"partial output retained at {partial_video_path}"
            )
        assert partial_video_path is not None and final_video_path is not None
        os.replace(partial_video_path, final_video_path)
        print(f"Completed video saved to {final_video_path}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bvh-file", type=Path, required=True)
    parser.add_argument("--cache-file", type=Path, required=True)
    parser.add_argument("--robot", choices=("unitree_g1",), default="unitree_g1")
    parser.add_argument("--scale", type=float, default=0.01)
    parser.add_argument("--start", type=int, default=0)
    parser.add_argument("--end", type=int)
    parser.add_argument("--frame-step", type=int, default=1)
    parser.add_argument("--human-height", type=float)
    parser.add_argument("--gmr-root", type=Path, default=Path("/opt/GMR"), help=argparse.SUPPRESS)
    parser.add_argument("--offsets-file", type=Path)
    parser.add_argument("--calibration-bvh-file", type=Path)
    parser.add_argument("--calibration-output", type=Path)
    parser.add_argument("--calibration-start-seconds", type=float, default=0.0)
    parser.add_argument("--calibration-seconds", type=float, default=0.0)
    parser.add_argument("--accept-reviewed-calibration-window", action="store_true")
    parser.add_argument("--allow-zero-offsets", action="store_true")
    parser.add_argument(
        "--arm-mode",
        choices=("auto", "calibrated", "npose", "position-only"),
        default="auto",
        help="auto uses an acknowledged N-pose window when supplied, otherwise position-only arms",
    )
    parser.add_argument("--playback-fps", type=float)
    parser.add_argument(
        "--loop-mode",
        choices=("ping-pong", "repeat"),
        default="ping-pong",
        help="ping-pong avoids a discontinuity when the first and last poses differ",
    )
    parser.add_argument("--loops", type=int, default=0, help="zero loops until stopped")
    parser.add_argument("--record-video", action="store_true")
    parser.add_argument("--video-path", type=Path)
    parser.add_argument("--overwrite-video", action="store_true")
    parser.add_argument(
        "--video-frame-step",
        type=int,
        default=1,
        help="record every Nth playback frame and divide the MP4 FPS by N",
    )
    parser.add_argument("--video-width", type=int, default=640)
    parser.add_argument("--video-height", type=int, default=480)
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--rebuild-cache", action="store_true")
    args = parser.parse_args()

    args.bvh_file = args.bvh_file.expanduser().resolve()
    args.cache_file = args.cache_file.expanduser().resolve()
    args.gmr_root = args.gmr_root.expanduser().resolve()
    if args.offsets_file is not None:
        args.offsets_file = args.offsets_file.expanduser().resolve()
    if args.calibration_bvh_file is not None:
        args.calibration_bvh_file = args.calibration_bvh_file.expanduser().resolve()
    if args.calibration_output is not None:
        args.calibration_output = args.calibration_output.expanduser().resolve()
    if args.video_path is not None:
        args.video_path = args.video_path.expanduser().resolve()
    if not args.bvh_file.is_file():
        parser.error(f"BVH file does not exist: {args.bvh_file}")
    if args.scale <= 0 or args.frame_step <= 0 or args.start < 0:
        parser.error("--scale and --frame-step must be positive; --start cannot be negative")
    if args.end is not None and args.end <= args.start:
        parser.error("--end must be greater than --start")
    if args.human_height is not None and not 0.8 <= args.human_height <= 2.5:
        parser.error("--human-height must be between 0.8 and 2.5 metres")
    if args.playback_fps is not None and args.playback_fps <= 0:
        parser.error("--playback-fps must be positive")
    if args.loops < 0:
        parser.error("--loops cannot be negative")
    if args.record_video and args.video_path is None:
        parser.error("--record-video requires --video-path")
    if args.record_video and args.loops == 0:
        parser.error("--record-video requires a finite --loops value so the MP4 is finalized")
    if args.video_path is not None and not args.record_video:
        parser.error("--video-path requires --record-video")
    if args.video_frame_step <= 0:
        parser.error("--video-frame-step must be positive")
    if args.video_width < 160 or args.video_height < 120:
        parser.error("video dimensions must be at least 160x120")
    if args.prepare_only and args.record_video:
        parser.error("--prepare-only cannot be combined with --record-video")
    if args.record_video:
        partial_path = _partial_video_path(args.video_path)
        existing = [path for path in (args.video_path, partial_path) if path.exists()]
        if existing and not args.overwrite_video:
            parser.error(
                "video output already exists; choose another path or pass --overwrite-video: "
                + ", ".join(str(path) for path in existing)
            )
        if args.overwrite_video:
            partial_path.unlink(missing_ok=True)
    if args.calibration_start_seconds < 0 or args.calibration_seconds < 0:
        parser.error("calibration times cannot be negative")
    if args.calibration_bvh_file is not None and not args.calibration_bvh_file.is_file():
        parser.error(f"calibration BVH file does not exist: {args.calibration_bvh_file}")
    args.resolved_arm_mode = _resolve_arm_mode(
        args.arm_mode,
        has_offsets=args.offsets_file is not None,
        has_calibration_bvh=args.calibration_bvh_file is not None,
        calibration_seconds=args.calibration_seconds,
    )
    if args.resolved_arm_mode == "calibrated" and args.offsets_file is None:
        parser.error("--arm-mode calibrated requires --offsets-file")
    if args.resolved_arm_mode == "npose":
        if not args.accept_reviewed_calibration_window:
            parser.error(
                "N-pose mode requires --accept-reviewed-calibration-window after visual review"
            )
        if args.calibration_seconds <= 0:
            parser.error("--arm-mode npose requires positive --calibration-seconds")
        if args.calibration_bvh_file is None:
            args.calibration_bvh_file = args.bvh_file
        args.calibration_source_sha256 = _sha256(args.calibration_bvh_file)
    else:
        args.calibration_source_sha256 = None
    args.gmr_runtime_identity = _runtime_identity(args.gmr_root)

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    if hasattr(signal, "SIGBREAK"):  # Windows console stop (CTRL_BREAK_EVENT)
        signal.signal(signal.SIGBREAK, _stop)

    source_sha256 = _sha256(args.bvh_file)
    if args.offsets_file is None:
        offsets_key = "zero-preview"
    else:
        if not args.offsets_file.is_file():
            parser.error(f"offsets file does not exist: {args.offsets_file}")
        offsets_key = _sha256(args.offsets_file)
    expected = _cache_metadata(args, source_sha256, offsets_key)
    cached = None if args.rebuild_cache else _read_cache(args.cache_file, expected)
    if (
        cached is not None
        and args.resolved_arm_mode == "npose"
        and args.calibration_output is not None
        and not args.calibration_output.is_file()
    ):
        print("Ignoring cache because its derived calibration record is missing")
        cached = None
    if cached is None:
        qpos, fps, metadata = _retarget(args, expected)
        _write_cache(args.cache_file, metadata, qpos, fps)
    else:
        qpos, fps = cached
    if args.prepare_only:
        print("Preparation complete; viewer was not started")
        return 0
    _play(args, qpos, fps)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
