#!/usr/bin/env python3
"""Show one live Xsens performer on G1 and T1 side by side in one MuJoCo window."""

from __future__ import annotations

import argparse
from dataclasses import replace
import math
from pathlib import Path
import signal
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from mocap_g1.live_filter import CommandUpsampler, LatencyRecorder, make_filter
from mocap_g1.live_robots import LIVE_ROBOTS, T1_MOCAP_JOINT_RANGES
from mocap_g1.live_safety import LiveOperatorControls, LiveRangePreviewController
from mocap_g1.mxtp02 import ProtocolError
from mocap_g1.safety import SafetyConfig
from mocap_g1.schema import G1_29DOF_JOINTS, load_official_joint_limits
from mocap_g1.xsens_live import GmrFrameConverter, LatestBodyReceiver
from tools.xsens.live_g1_sim import (
    LiveLoop, ReferencePipeline, _build_live_conditioning,
    _print_receiver_diagnostics, _validate_safety_args, build_arm_alignment,
    build_parser as single_parser, configure_ik, layout_error, wait_for_first_frame,
)

ROBOT_KEYS = ("unitree_g1", "booster_t1_29dof")
OFFSETS_Y = (-1.1, 1.1)
RUNNING = True


def _stop(_signum, _frame):
    global RUNNING
    RUNNING = False


def copy_frame(frame):
    """Each IK solver gets an independent copy of the same received pose."""
    return {name: (np.array(pos, copy=True), np.array(rot, copy=True))
            for name, (pos, rot) in frame.items()}


def build_scene(xml_paths):
    """Compile a single scene with two prefixed robots and one shared floor."""
    import mujoco

    spec = mujoco.MjSpec()
    spec.modelname = "Live Xsens: G1 + T1"
    spec.worldbody.add_geom(name="ground", type=mujoco.mjtGeom.mjGEOM_PLANE,
                            size=[0, 0, 0.01], rgba=[0.16, 0.19, 0.24, 1])
    for key, y, prefix in zip(ROBOT_KEYS, OFFSETS_Y, ("g1_", "t1_")):
        child = mujoco.MjSpec.from_file(str(xml_paths[key]))
        for geom in list(child.worldbody.geoms):
            if geom.type == mujoco.mjtGeom.mjGEOM_PLANE:
                child.delete(geom)
        frame = spec.worldbody.add_frame(pos=[0, y, 0])
        spec.attach(child, prefix=prefix, frame=frame)
    return spec.compile()


def qpos_indices(model, robot, prefix, free_index):
    """Map a standalone robot qpos to its namespaced joints in the shared scene."""
    import mujoco

    free = [int(model.jnt_qposadr[j]) for j in range(model.njnt)
            if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE]
    if len(free) != 2:
        raise ValueError(f"expected two free roots, got {len(free)}")
    indices = list(range(free[free_index], free[free_index] + 7))
    for name in robot.dof_joints:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, prefix + name)
        if jid < 0:
            raise ValueError(f"shared scene lacks {prefix + name}")
        indices.append(int(model.jnt_qposadr[jid]))
    if len(indices) != robot.nq or len(set(indices)) != robot.nq:
        raise ValueError(f"invalid shared qpos layout for {robot.label}")
    return np.asarray(indices, dtype=np.int32)


class RobotPose:
    """Standalone pose for each safety loop; render copies into one scene."""

    def __init__(self, model, initial_qpos):
        import mujoco
        self.model = model
        self.data = mujoco.MjData(model)
        self.data.qpos[:] = initial_qpos
        mujoco.mj_forward(model, self.data)

    def set_pose(self, root, dofs):
        import mujoco
        self.data.qpos[:7] = root
        self.data.qpos[7:] = dofs
        mujoco.mj_forward(self.model, self.data)


class PairViewer:
    def __init__(self, model, indices, poses, *, headless=False, key_callback=None):
        import mujoco
        self.mujoco = mujoco
        self.model = model
        self.data = mujoco.MjData(model)
        self.indices = indices
        self.poses = poses
        for y, idx, pose in zip(OFFSETS_Y, indices, poses):
            combined = pose.data.qpos.copy()
            combined[1] += y
            self.data.qpos[idx] = combined
        mujoco.mj_forward(model, self.data)
        self.viewer = None
        self.camera_center = np.mean([pose.data.qpos[:3] + [0, y, 0]
                                      for y, pose in zip(OFFSETS_Y, poses)], axis=0)
        if not headless:
            import mujoco.viewer
            self.viewer = mujoco.viewer.launch_passive(
                model, self.data, show_left_ui=False, show_right_ui=False,
                key_callback=key_callback,
            )
            self.viewer.cam.lookat[:] = [0, 0, 0.95]
            self.viewer.cam.distance = 4.7
            self.viewer.cam.azimuth = 0
            self.viewer.cam.elevation = -10

    def step(self, roots, dofs):
        for y, idx, pose, root, dof in zip(OFFSETS_Y, self.indices, self.poses, roots, dofs):
            pose.set_pose(root, dof)
            combined = pose.data.qpos.copy()
            combined[1] += y
            self.data.qpos[idx] = combined
        self.mujoco.mj_forward(self.model, self.data)
        if self.viewer is not None:
            self.viewer.user_scn.ngeom = 0
            for label, idx in zip(("G1", "T1"), self.indices):
                scene = self.viewer.user_scn
                if scene.ngeom >= scene.maxgeom:
                    break
                geom = scene.geoms[scene.ngeom]
                label_pos = self.data.qpos[idx[:3]].copy()
                label_pos[2] += 1.05
                self.mujoco.mjv_initGeom(
                    geom, self.mujoco.mjtGeom.mjGEOM_LABEL,
                    [0, 0, 0], label_pos,
                    np.eye(3).ravel(), [1, 1, 1, 1],
                )
                geom.label = label
                scene.ngeom += 1
            center = np.mean([self.data.qpos[idx[:3]] for idx in self.indices], axis=0)
            # Shift the camera target by the performer's displacement. This keeps
            # manual pan/zoom/orbit offsets intact while following translation.
            self.viewer.cam.lookat[:] += center - self.camera_center
            self.camera_center = center
            self.viewer.sync()

    def is_running(self):
        return self.viewer is None or self.viewer.is_running()

    def close(self):
        if self.viewer is not None:
            self.viewer.close()


def build_parser() -> argparse.ArgumentParser:
    parser = single_parser()
    parser.description = __doc__
    for action in list(parser._actions):
        if action.dest in {"robot", "record_video", "video_path", "g1_hardware_envelope",
                           "rate_limit", "position_margin"}:
            parser._remove_action(action)
            for option in action.option_strings:
                parser._option_string_actions.pop(option, None)
            for group in parser._action_groups:
                if action in group._group_actions:
                    group._group_actions.remove(action)
    # The shared validator expects this field; paired preview always uses
    # exact MJCF/URDF position ranges rather than a configurable extra margin.
    parser.set_defaults(calibrate_after=5.0, position_margin=0.0)
    for action in parser._actions:
        if action.dest == "human_height":
            action.required = True
            action.help = "performer height in metres (required for retargeting scale)"
            break
    parser.add_argument("--headless", action="store_true", help="run without a viewer")
    return parser


def _make_loop(args, robot, retargeter, pose, controls, startup, receiver, converter,
               filter_config, upsampler_config):
    if robot.key == "unitree_g1":
        limits = load_official_joint_limits(args.limits_urdf)
        ranges = tuple((name, limits[name].lower, limits[name].upper)
                       for name in G1_29DOF_JOINTS)
    else:
        ranges = T1_MOCAP_JOINT_RANGES
    safety_config = SafetyConfig(
        position_margin_rad=0.0,
        velocity_limit_fraction=args.velocity_limit_fraction,
        acceleration_limit_rad_s2=args.acceleration_limit,
        stale_timeout_s=args.stale_timeout,
        hard_timeout_s=args.hard_timeout,
        blend_rate_per_s=args.blend_rate,
    )
    safety = LiveRangePreviewController(ranges, robot.upper_body_joints, safety_config)
    pipeline = ReferencePipeline(
        safety, LatencyRecorder(),
        joint_filter=make_filter(filter_config, len(robot.dof_joints)) if filter_config else None,
        upsampler=CommandUpsampler(upsampler_config, len(robot.dof_joints)) if upsampler_config else None,
    )
    abduction = robot.arm_abduction_deg if args.arm_abduction is None else args.arm_abduction
    configure_ik(retargeter, abduction)
    alignment = build_arm_alignment(retargeter, args.arm_alignment or robot.default_arm_alignment)
    loop = LiveLoop(
        receiver=receiver, converter=converter, retargeter=retargeter, viewer=pose,
        safety=safety, controls=controls, pipeline=pipeline,
        startup=replace(startup, frame=copy_frame(startup.frame)),
        control_rate_hz=args.control_rate, stale_timeout_s=args.stale_timeout,
        duration_s=args.duration, rate_limit=False, expected_nq=robot.nq,
        label=robot.label, arm_alignment=alignment, feet=robot.feet,
        calibrate_after_s=args.calibrate_after, abduction_deg=abduction,
        running=lambda: RUNNING,
    )
    return loop


def run(args):
    from general_motion_retargeting import GeneralMotionRetargeting, params
    from general_motion_retargeting.robot_motion_viewer import ROBOT_XML_DICT

    t1 = LIVE_ROBOTS["booster_t1_29dof"]
    params.IK_CONFIG_DICT.setdefault("xsens_mvn", {})[t1.key] = str(t1.ik_config)
    receiver = None
    viewer = None
    loops = []
    try:
        receiver = LatestBodyReceiver(bind=args.bind, port=args.port,
                                      expected_source=args.source_ip, strict=True)
        converter = GmrFrameConverter()
        print(f"Waiting for strict MXTP02 on {args.bind}:{args.port} from {args.source_ip or 'any source'}...", flush=True)
        startup = wait_for_first_frame(receiver, converter, timeout_s=args.startup_timeout,
                                       clock=time.monotonic, running=lambda: RUNNING)
        if startup is None:
            return 0
        print(f"Receiving Xsens body frames from {startup.source[0]}:{startup.source[1]}", flush=True)
        retargeters = []
        poses = []
        for key in ROBOT_KEYS:
            robot = LIVE_ROBOTS[key]
            retargeter = GeneralMotionRetargeting(
                src_human="xsens_mvn", tgt_robot=key,
                actual_human_height=args.human_height, solver="daqp",
                damping=1.0, use_velocity_limit=True, verbose=False,
            )
            mismatch = layout_error(retargeter, robot)
            if mismatch:
                raise ValueError(mismatch)
            retargeters.append(retargeter)
            poses.append(RobotPose(retargeter.model, retargeter.model.qpos0.copy()))
        model = build_scene(ROBOT_XML_DICT)
        indices = [qpos_indices(model, LIVE_ROBOTS[key], prefix, i)
                   for i, (key, prefix) in enumerate(zip(ROBOT_KEYS, ("g1_", "t1_")))]
        controls = [LiveOperatorControls(enabled=not args.start_disabled) for _ in ROBOT_KEYS]

        def on_key(keycode):
            for control in controls:
                control.keyboard_callback(keycode)

        viewer = PairViewer(model, indices, poses, headless=args.headless, key_callback=on_key)
        filter_config, upsampler_config = _build_live_conditioning(args, build_parser())
        loops = [_make_loop(args, LIVE_ROBOTS[key], retargeter, pose, control,
                            startup, receiver, converter, filter_config, upsampler_config)
                 for key, retargeter, pose, control in zip(ROBOT_KEYS, retargeters, poses, controls)]
        for loop in loops:
            loop.start()
        started = time.monotonic()
        for loop in loops:
            loop.started = started
            loop.stats.reported_at = started
        print("One viewer: G1 left, T1 right. Space toggles both; R rearms both; N recalibrates both.", flush=True)
        print(f"Automatic neutral calibration after {args.calibrate_after:g} s of fresh stream. Stand still, arms down.", flush=True)
        period = 1 / args.control_rate
        next_tick = started
        while RUNNING and viewer.is_running() and (args.duration == 0 or time.monotonic() - started < args.duration):
            for loop in loops:
                loop.handle_rearm()
                loop.expire_stale_pending()
            if all(loop.pending_frame is None for loop in loops):
                try:
                    body, _source = receiver.receive_latest(timeout=period)
                    received = time.monotonic()
                    frame = converter.convert(body)
                    for loop in loops:
                        loop.pending_frame = copy_frame(frame)
                        loop.pending_counter = body.sample_counter
                        loop.pending_source_timestamp = received
                        loop.pending_timeline = loop.pipeline.latency.open(received)
                        loop.pending_timeline.mark("parse")
                except TimeoutError:
                    pass
                except ProtocolError as exc:
                    for loop in loops:
                        loop._report_failure(f"Rejected frame: {exc}")
            for loop in loops:
                if loop.pending_frame is not None:
                    loop.process_pending()
                loop.pipeline.pump(loop.clock)
            outputs = [loop.step_safety() for loop in loops]
            for loop, output in zip(loops, outputs):
                line = loop.reporter.clamp_line(output.clamped_joints)
                if line:
                    print(f"{loop.stats.label}: {line}", file=sys.stderr)
                line = loop.stats.periodic_line(now=loop.clock(), started=loop.started,
                                                safety_reason=output.reason)
                if line:
                    print(line)
            viewer.step([loop.last_root for loop in loops], [o.dof_pos for o in outputs])
            next_tick += period
            remaining = next_tick - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            else:
                next_tick = time.monotonic()
        return 0
    except TimeoutError:
        print(f"ERROR: no valid MXTP02 body frame arrived on UDP {args.port}", file=sys.stderr)
        return 3
    finally:
        if viewer is not None:
            viewer.close()
        if receiver is not None:
            receiver.close()
        for loop in loops:
            print(f"{loop.stats.label}: {loop.pipeline.latency.format_report()}")
        if receiver is not None:
            _print_receiver_diagnostics(receiver)


def main(argv=None):
    global RUNNING
    RUNNING = True
    parser = build_parser()
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if not math.isfinite(args.human_height) or args.human_height <= 0:
        parser.error("--human-height must be positive")
    if not math.isfinite(args.calibrate_after) or args.calibrate_after < 0:
        parser.error("--calibrate-after must be zero or positive")
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error("--duration must be zero or positive")
    if not math.isfinite(args.startup_timeout) or args.startup_timeout <= 0:
        parser.error("--startup-timeout must be positive")
    _validate_safety_args(args, parser)
    _build_live_conditioning(args, parser)
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
