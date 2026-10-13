#!/usr/bin/env python3
"""Render one finite GMR robot-motion pickle in MuJoCo.

GMR pickles store ``root_rot`` in ``xyzw`` order, while
``RobotMotionViewer.step`` expects MuJoCo's scalar-first ``wxyz`` order.  This
adapter validates that boundary, checks the articulated width against the
selected robot's actual MJCF ``nq``, and then plays a finite number of loops.
Video frames are written to a sibling partial file and promoted atomically
only after every requested frame has rendered successfully.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import pickle
import sys
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from mocap_g1.gmr_pkl import (  # noqa: E402
    UNIT_QUATERNION_TOLERANCE,
    RootQuatOrderError,
    resolve_root_quat_order,
    xyzw_to_wxyz,
)


QUATERNION_TOLERANCE = UNIT_QUATERNION_TOLERANCE


class RenderError(RuntimeError):
    """Raised when a motion cannot be rendered safely."""


@dataclass(frozen=True)
class MotionArrays:
    """Validated arrays in the convention consumed by ``RobotMotionViewer``."""

    fps: float
    root_pos: np.ndarray
    root_rot_wxyz: np.ndarray
    dof_pos: np.ndarray

    @property
    def frames(self) -> int:
        return int(self.root_pos.shape[0])


def _as_array(value: Any, name: str, *, ndim: int) -> np.ndarray:
    try:
        array = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise RenderError(f"{name} must be a numeric NumPy-compatible array") from exc
    if array.ndim != ndim:
        raise RenderError(f"{name} must have {ndim} dimensions, got shape {array.shape}")
    if not np.isfinite(array).all():
        raise RenderError(f"{name} contains non-finite values")
    return np.ascontiguousarray(array)


def validate_motion_payload(payload: Any, *, legacy_quat_order: str | None = None
                            ) -> MotionArrays:
    """Validate a canonical GMR pickle and convert its root quaternion.

    GMR's canonical pickle schema is ``fps``, ``root_pos`` (T, 3),
    ``root_rot`` (T, 4, xyzw), and ``dof_pos`` (T, D).  The order must be
    declared explicitly (``root_quat_order``, see ``mocap_g1.gmr_pkl``; an
    older ``metadata.layout.root_rot`` declaration is also accepted) and must
    be ``xyzw``.  ``legacy_quat_order`` is an explicit override for keyless
    pre-contract pickles only.  The returned root rotations are scalar-first
    ``wxyz`` because that is what MuJoCo's ``qpos`` and ``RobotMotionViewer``
    require.
    """

    if not isinstance(payload, dict):
        raise RenderError("motion pickle must contain a dictionary")
    required = ("fps", "root_pos", "root_rot", "dof_pos")
    missing = [key for key in required if key not in payload]
    if missing:
        raise RenderError(f"motion pickle is missing required field(s): {', '.join(missing)}")

    declared_order = payload.get("quaternion_order", payload.get("root_quaternion_order"))
    metadata = payload.get("metadata")
    if isinstance(metadata, dict):
        layout = metadata.get("layout")
        if isinstance(layout, dict) and "root_rot" in layout:
            declared_order = layout["root_rot"]
        elif "root_quaternion_order" in metadata:
            declared_order = metadata["root_quaternion_order"]
    if declared_order is not None and str(declared_order).lower() != "xyzw":
        raise RenderError(
            "GMR root_rot convention must be xyzw; "
            f"pickle declares {declared_order!r}"
        )
    try:
        stored_order = resolve_root_quat_order(
            payload,
            legacy_order=(
                legacy_quat_order if legacy_quat_order is not None
                else (str(declared_order).lower() if declared_order is not None else None)
            ),
            source="motion pickle",
        )
    except RootQuatOrderError as exc:
        raise RenderError(str(exc)) from exc

    try:
        fps_array = np.asarray(payload["fps"])
        if fps_array.ndim != 0:
            raise RenderError(f"fps must be a scalar, got shape {fps_array.shape}")
        fps = float(fps_array.item())
    except (TypeError, ValueError, OverflowError) as exc:
        raise RenderError("fps must be a finite positive scalar") from exc
    if not np.isfinite(fps) or fps <= 0:
        raise RenderError(f"fps must be finite and positive, got {fps!r}")

    root_pos = _as_array(payload["root_pos"], "root_pos", ndim=2)
    root_rot_xyzw = _as_array(payload["root_rot"], "root_rot", ndim=2)
    dof_pos = _as_array(payload["dof_pos"], "dof_pos", ndim=2)

    if root_pos.shape[1] != 3:
        raise RenderError(f"root_pos must have shape (T, 3), got {root_pos.shape}")
    if root_rot_xyzw.shape[1] != 4:
        raise RenderError(f"root_rot must have shape (T, 4) in xyzw order, got {root_rot_xyzw.shape}")
    if dof_pos.shape[1] == 0:
        raise RenderError(f"dof_pos must contain at least one articulated joint, got {dof_pos.shape}")
    frames = root_pos.shape[0]
    if frames == 0:
        raise RenderError("motion contains zero frames")
    if root_rot_xyzw.shape[0] != frames or dof_pos.shape[0] != frames:
        raise RenderError(
            "root_pos, root_rot, and dof_pos must have the same frame count: "
            f"{root_pos.shape[0]}, {root_rot_xyzw.shape[0]}, {dof_pos.shape[0]}"
        )

    norms = np.linalg.norm(root_rot_xyzw, axis=1)
    if np.max(np.abs(norms - 1.0)) > QUATERNION_TOLERANCE:
        raise RenderError(
            "root_rot must contain unit quaternions in GMR xyzw order; "
            f"maximum norm error is {np.max(np.abs(norms - 1.0)):.6g}"
        )
    root_rot_wxyz = np.ascontiguousarray(
        xyzw_to_wxyz(root_rot_xyzw) if stored_order == "xyzw" else root_rot_xyzw
    )
    return MotionArrays(fps, root_pos, root_rot_wxyz, dof_pos)


def load_motion(path: Path, *, legacy_quat_order: str | None = None) -> MotionArrays:
    """Load and validate a GMR motion pickle from ``path``."""

    if not path.is_file():
        raise RenderError(f"motion pickle does not exist: {path}")
    try:
        with path.open("rb") as handle:
            payload = pickle.load(handle)
    except (
        OSError,
        pickle.PickleError,
        EOFError,
        ValueError,
        AttributeError,
        ImportError,
        ModuleNotFoundError,
    ) as exc:
        raise RenderError(f"cannot read motion pickle {path}: {exc}") from exc
    return validate_motion_payload(payload, legacy_quat_order=legacy_quat_order)


def _load_gmr_runtime(gmr_root: Path) -> tuple[Any, dict[str, Path]]:
    """Import GMR from the explicitly selected checkout."""

    if not gmr_root.is_dir():
        raise RenderError(f"GMR root does not exist or is not a directory: {gmr_root}")
    root = str(gmr_root.resolve())
    if root not in sys.path:
        sys.path.insert(0, root)
    try:
        from general_motion_retargeting import ROBOT_XML_DICT, RobotMotionViewer
    except (ImportError, ModuleNotFoundError) as exc:
        raise RenderError(f"cannot import GMR from {gmr_root}: {exc}") from exc
    return RobotMotionViewer, dict(ROBOT_XML_DICT)


def expected_robot_nq(gmr_root: Path, robot: str) -> int:
    """Return the selected robot's actual MJCF ``nq``."""

    _viewer_class, robot_xml = _load_gmr_runtime(gmr_root)
    if robot not in robot_xml:
        available = ", ".join(sorted(robot_xml))
        raise RenderError(f"unknown GMR robot {robot!r}; available robots: {available}")
    xml_path = Path(robot_xml[robot])
    if not xml_path.is_file():
        raise RenderError(f"MJCF for robot {robot!r} does not exist: {xml_path}")
    try:
        import mujoco

        model = mujoco.MjModel.from_xml_path(str(xml_path))
    except (ImportError, OSError, RuntimeError) as exc:
        raise RenderError(f"cannot load MJCF for robot {robot!r}: {exc}") from exc
    nq = int(model.nq)
    if nq < 7:
        raise RenderError(f"robot {robot!r} MJCF has nq={nq}; expected a 7-value free root")
    return nq


def partial_video_path(path: Path) -> Path:
    """Return the sibling path used while an MP4 is being rendered."""

    return path.with_name(f".{path.stem}.partial{path.suffix}")


def _viewer_is_running(viewer: Any) -> bool:
    handle = getattr(viewer, "viewer", viewer)
    is_running = getattr(handle, "is_running", None)
    return True if is_running is None else bool(is_running())


def render_motion(
    *,
    gmr_root: Path,
    robot: str,
    motion: MotionArrays,
    loops: int = 1,
    frame_step: int = 1,
    record_video: bool = False,
    video_path: Path | None = None,
    video_width: int = 640,
    video_height: int = 480,
    overwrite_video: bool = False,
    viewer_class: Any | None = None,
) -> int:
    """Render a validated finite motion and return the number of frames sent."""

    if loops <= 0:
        raise RenderError("loops must be a positive integer")
    if frame_step <= 0:
        raise RenderError("frame-step must be a positive integer")
    if video_width <= 0 or video_height <= 0:
        raise RenderError("video dimensions must be positive")
    if record_video and video_path is None:
        raise RenderError("record-video requires video-path")
    if not record_video and video_path is not None:
        raise RenderError("video-path requires record-video")

    expected_nq = expected_robot_nq(gmr_root, robot)
    expected_dof = expected_nq - 7
    if motion.dof_pos.shape[1] != expected_dof:
        raise RenderError(
            f"motion has {motion.dof_pos.shape[1]} dof values, but robot {robot!r} MJCF "
            f"has nq={expected_nq} and therefore expects {expected_dof}"
        )

    playback = slice(None, None, frame_step)
    root_pos = motion.root_pos[playback]
    root_rot = motion.root_rot_wxyz[playback]
    dof_pos = motion.dof_pos[playback]
    playback_fps = motion.fps / frame_step
    frame_count = int(root_pos.shape[0])
    if frame_count == 0:
        raise RenderError("frame-step selected zero frames")

    final_video = video_path.resolve() if video_path is not None else None
    partial_video = partial_video_path(final_video) if final_video is not None else None
    if final_video is not None:
        existing = [str(path) for path in (final_video, partial_video) if path.exists()]
        if existing and not overwrite_video:
            raise RenderError(
                "refusing to overwrite video output; pass --overwrite-video or choose another path: "
                + ", ".join(existing)
            )
        final_video.parent.mkdir(parents=True, exist_ok=True)
        if overwrite_video and partial_video is not None:
            partial_video.unlink(missing_ok=True)

    if viewer_class is None:
        viewer_class, _robot_xml = _load_gmr_runtime(gmr_root)
    viewer = None
    rendered_frames = 0
    completed = False
    close_ok = True
    shutdown_error: Exception | None = None
    try:
        viewer = viewer_class(
            robot_type=robot,
            camera_follow=True,
            motion_fps=playback_fps,
            transparent_robot=0,
            record_video=record_video,
            video_path=str(partial_video) if partial_video is not None else None,
            video_width=video_width,
            video_height=video_height,
        )
        for _loop in range(loops):
            for index in range(frame_count):
                if not _viewer_is_running(viewer):
                    raise RenderError(
                        f"viewer closed after {rendered_frames}/{frame_count * loops} frames"
                    )
                viewer.step(
                    root_pos=root_pos[index],
                    root_rot=root_rot[index],
                    dof_pos=dof_pos[index],
                    rate_limit=not record_video,
                    follow_camera=True,
                )
                rendered_frames += 1
        completed = True
    finally:
        if viewer is not None:
            try:
                viewer.close()
            except Exception as exc:  # pragma: no cover - exercised by integration/runtime
                close_ok = False
                shutdown_error = exc

    if shutdown_error is not None:
        raise RenderError(f"viewer shutdown failed: {shutdown_error}") from shutdown_error

    expected_frames = frame_count * loops
    if record_video:
        assert final_video is not None and partial_video is not None
        if not completed or not close_ok or rendered_frames != expected_frames:
            raise RenderError(
                f"video export stopped after {rendered_frames}/{expected_frames} frames; "
                f"partial output retained at {partial_video}"
            )
        os.replace(partial_video, final_video)
    return rendered_frames


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gmr-root", type=Path, required=True, help="GMR checkout containing general_motion_retargeting")
    parser.add_argument("--robot", required=True, help="GMR robot key, e.g. booster_t1_29dof")
    parser.add_argument("--motion", type=Path, required=True, help="GMR robot-motion .pkl")
    parser.add_argument("--record-video", action="store_true")
    parser.add_argument("--video-path", type=Path)
    parser.add_argument("--video-width", "--width", dest="video_width", type=int, default=640)
    parser.add_argument("--video-height", "--height", dest="video_height", type=int, default=480)
    parser.add_argument("--frame-step", type=int, default=1)
    parser.add_argument("--loops", "--loop-count", dest="loops", type=int, default=1)
    parser.add_argument("--overwrite-video", action="store_true")
    parser.add_argument(
        "--quat-order", choices=("xyzw", "wxyz"), default=None,
        help="LEGACY override: root_rot order of a pickle WITHOUT a root_quat_order "
             "key (default: require the declaration)",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        gmr_root = args.gmr_root.expanduser().resolve()
        motion_path = args.motion.expanduser().resolve()
        video_path = args.video_path.expanduser().resolve() if args.video_path is not None else None
        if args.record_video and video_path is None:
            raise RenderError("--record-video requires --video-path")
        if not args.record_video and video_path is not None:
            raise RenderError("--video-path requires --record-video")
        arrays = load_motion(motion_path, legacy_quat_order=args.quat_order)
        rendered = render_motion(
            gmr_root=gmr_root,
            robot=args.robot,
            motion=arrays,
            loops=args.loops,
            frame_step=args.frame_step,
            record_video=args.record_video,
            video_path=video_path,
            video_width=args.video_width,
            video_height=args.video_height,
            overwrite_video=args.overwrite_video,
        )
    except (RenderError, OSError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    print(f"PASS: rendered {rendered} {args.robot} frame(s)")
    if video_path is not None:
        print(f"video: {video_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
