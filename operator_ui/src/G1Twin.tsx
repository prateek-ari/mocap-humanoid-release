import { useState } from 'react'
import type { PlaybackManifest, SessionState, SimulationStatus } from './types'

interface G1TwinProps {
  state: SessionState
  mock: boolean
  playback: PlaybackManifest | null
  simulation?: SimulationStatus | null
  recordedSimulation?: boolean
}

type Point = [number, number]

interface PoseGeometry {
  head: Point
  neck: Point
  leftShoulder: Point
  leftElbow: Point
  leftWrist: Point
  rightShoulder: Point
  rightElbow: Point
  rightWrist: Point
  pelvis: Point
  leftHip: Point
  leftKnee: Point
  leftAnkle: Point
  rightHip: Point
  rightKnee: Point
  rightAnkle: Point
}

const REFERENCE_POSE: PoseGeometry = {
  head: [160, 81], neck: [160, 116], leftShoulder: [129, 127], leftElbow: [111, 171], leftWrist: [98, 216],
  rightShoulder: [192, 127], rightElbow: [211, 169], rightWrist: [225, 212], pelvis: [160, 203],
  leftHip: [143, 228], leftKnee: [132, 282], leftAnkle: [128, 345], rightHip: [177, 228], rightKnee: [188, 282], rightAnkle: [194, 345],
}

const BOUNDED_POSE: PoseGeometry = {
  head: [160, 81], neck: [160, 116], leftShoulder: [128, 127], leftElbow: [112, 173], leftWrist: [101, 218],
  rightShoulder: [192, 127], rightElbow: [209, 171], rightWrist: [222, 215], pelvis: [160, 203],
  leftHip: [144, 228], leftKnee: [135, 283], leftAnkle: [133, 345], rightHip: [176, 228], rightKnee: [185, 283], rightAnkle: [188, 345],
}

function canonicalJointPositions(values: number[] | null | undefined): number[] | null {
  if (!Array.isArray(values)) return null
  if (values.length >= 36) return values.slice(7, 36)
  if (values.length >= 29) return values.slice(0, 29)
  return null
}

function qposValue(joints: number[], index: number): number {
  const value = joints[index]
  if (!Number.isFinite(value)) return 0
  return Math.max(-1, Math.min(1, value / 1.5))
}

function projectedPose(values: number[] | null | undefined, fallback: PoseGeometry): PoseGeometry {
  const joints = canonicalJointPositions(values)
  if (!joints) return fallback
  const q = (index: number) => qposValue(joints, index)
  const center = q(14) * 4
  const pelvisY = q(12) * 3
  return {
    head: [160 + center + q(13) * 2, 81 + q(12) * 2],
    neck: [160 + center, 116],
    leftShoulder: [128 + center, 127 + q(16) * 3],
    leftElbow: [112 + center + q(16) * 8 + q(18) * 6, 173 - q(15) * 8],
    leftWrist: [101 + center + q(16) * 9 + q(18) * 10 + q(19) * 4, 218 - q(15) * 10 - q(20) * 4],
    rightShoulder: [192 + center, 127 + q(23) * 3],
    rightElbow: [209 + center + q(23) * 8 + q(25) * 6, 171 - q(22) * 8],
    rightWrist: [222 + center + q(23) * 9 + q(25) * 10 + q(26) * 4, 215 - q(22) * 10 - q(27) * 4],
    pelvis: [160 + center, 203 + pelvisY],
    leftHip: [144 + center + q(1) * 3, 228 + pelvisY],
    leftKnee: [135 + center + q(0) * 8 + q(3) * 4, 283 + pelvisY],
    leftAnkle: [133 + center + q(0) * 10 + q(3) * 10 + q(4) * 4, 345 + pelvisY - Math.abs(q(3)) * 4],
    rightHip: [176 + center + q(7) * 3, 228 + pelvisY],
    rightKnee: [185 + center + q(6) * 8 + q(9) * 4, 283 + pelvisY],
    rightAnkle: [188 + center + q(6) * 10 + q(9) * 10 + q(10) * 4, 345 + pelvisY - Math.abs(q(9)) * 4],
  }
}

function point(point: Point): string {
  return `${point[0].toFixed(1)} ${point[1].toFixed(1)}`
}

function posePath(pose: PoseGeometry): string {
  return [
    `M${point(pose.head)}L${point(pose.neck)}L${point(pose.leftShoulder)}L${point(pose.leftElbow)}L${point(pose.leftWrist)}`,
    `M${point(pose.neck)}L${point(pose.rightShoulder)}L${point(pose.rightElbow)}L${point(pose.rightWrist)}`,
    `M${point(pose.neck)}L${point(pose.pelvis)}L${point(pose.leftHip)}L${point(pose.leftKnee)}L${point(pose.leftAnkle)}`,
    `M${point(pose.pelvis)}L${point(pose.rightHip)}L${point(pose.rightKnee)}L${point(pose.rightAnkle)}`,
  ].join('')
}

function poseJoints(pose: PoseGeometry): Point[] {
  return [pose.neck, pose.leftShoulder, pose.leftElbow, pose.leftWrist, pose.rightShoulder, pose.rightElbow, pose.rightWrist, pose.pelvis, pose.leftHip, pose.leftKnee, pose.leftAnkle, pose.rightHip, pose.rightKnee, pose.rightAnkle]
}

function LegendLine({ kind }: { kind: 'reference' | 'bounded' }) {
  return (
    <svg className={`legend-line ${kind}`} width="28" height="6" viewBox="0 0 28 6" aria-hidden="true">
      <path d="M0 3H28" />
    </svg>
  )
}

export function G1Twin({ state, mock, playback, simulation, recordedSimulation = false }: G1TwinProps) {
  // The failed URL, not a flag, so a new manifest URL retries without an effect.
  const [failedVideoUrl, setFailedVideoUrl] = useState<string | null>(null)
  const videoError = playback !== null && failedVideoUrl === playback.video_url
  const referenceQpos = Array.isArray(simulation?.reference_qpos) && simulation.reference_qpos.length ? simulation.reference_qpos : null
  const boundedQpos = Array.isArray(simulation?.bounded_qpos) && simulation.bounded_qpos.length ? simulation.bounded_qpos : null
  const reportedKinematics = referenceQpos !== null || boundedQpos !== null
  const active = reportedKinematics || ['mocap_accepted', 'sim_validated', 'robot_connected', 'preflight_passed', 'armed_disabled', 'tracking'].includes(state)
  const tracking = state === 'tracking' && !recordedSimulation
  const referencePose = projectedPose(referenceQpos, REFERENCE_POSE)
  const boundedPose = projectedPose(boundedQpos, BOUNDED_POSE)
  const sourceName = simulation?.source?.name?.trim() || 'reported joint positions'
  const authority = recordedSimulation
    ? state === 'tracking' ? 'SIM TRACKING · NO MOTORS' : 'NO MOTORS · SIM IDLE'
    : tracking ? (mock ? 'SIMULATION ACTIVE' : 'ACTIVE') : '0% · DISABLED'

  if (playback) {
    return (
      <div className="twin">
        <div className="graticule is-playback">
          <span className="graticule-label top-left">Recorded review</span>
          <span className="graticule-label top-right">Simulation preview · hardware blocked</span>
          {videoError ? (
            <div className="playback-unavailable" role="alert">
              <strong>Recorded review unavailable</strong>
              <span>The configured media could not be loaded. Controls remain disabled and command authority remains zero.</span>
            </div>
          ) : (
            <video
              aria-label={`Recorded simulation review: ${playback.filename}`}
              src={playback.video_url}
              autoPlay
              controls
              loop
              muted
              playsInline
              preload="metadata"
              onError={() => setFailedVideoUrl(playback.video_url)}
            />
          )}
          <div className="authority-readout">
            <span>Control authority</span>
            <strong>0% · DISABLED</strong>
          </div>
        </div>
        <div className="twin-footer">
          <span><b>Source</b> {playback.filename}</span>
          <span><b>Mode</b> simulation preview</span>
          <span><b>Hardware</b> blocked</span>
        </div>
      </div>
    )
  }

  return (
    <div className="twin">
      <div className={`graticule ${active ? 'is-live' : ''} ${tracking ? 'is-tracking' : ''}`}>
        <span className="graticule-label top-left">Front · Z+</span>
        <span className="graticule-label top-right">{reportedKinematics ? 'Kinematic only · not physics' : '29-DOF · G1 EDU'}</span>
        <svg
          className="twin-figure"
          viewBox="40 44 240 326"
          role="img"
          aria-label={reportedKinematics
            ? 'Kinematic-only front projection comparing reported reference and bounded joint positions. This is not a physics visualization.'
            : 'Front view schematic of the Unitree G1 showing target and measured joint posture'}
        >
          <g className="pose reference-pose" aria-hidden="true">
            <path d={posePath(referencePose)} />
            <rect x={referencePose.head[0] - 17} y={referencePose.head[1] - 22} width="34" height="27" rx="6" />
          </g>
          <g className="pose bounded-pose" aria-hidden="true">
            <path d={posePath(boundedPose)} />
            <rect x={boundedPose.head[0] - 17} y={boundedPose.head[1] - 22} width="34" height="27" rx="6" />
            <path className="feet" d={`M${point([boundedPose.leftAnkle[0] - 7, boundedPose.leftAnkle[1] + 9])}h20M${point([boundedPose.rightAnkle[0] - 7, boundedPose.rightAnkle[1] + 9])}h20`} />
            {poseJoints(boundedPose).map(([x, y], index) => <circle key={index} cx={x} cy={y} r="4.4" />)}
          </g>
          <path className="ground" d="M64 362H256" aria-hidden="true" />
        </svg>
        <div className="twin-legend" aria-label="Display legend">
          <span><LegendLine kind="reference" />{reportedKinematics ? 'Reference (retargeted)' : 'Target'}</span>
          <span><LegendLine kind="bounded" />{reportedKinematics ? 'Bounded (after safety envelope)' : 'Measured'}</span>
        </div>
        <div className="authority-readout" aria-live="polite">
          <span>Control authority</span>
          <strong>{authority}</strong>
        </div>
      </div>
      <div className="twin-footer">
        {reportedKinematics ? (
          <>
            <span><b>Source</b> {sourceName}</span>
            <span><b>Reference</b> {referenceQpos ? `${referenceQpos.length} qpos` : 'not reported'}</span>
            <span><b>Bounded</b> {boundedQpos ? `${boundedQpos.length} qpos` : 'not reported'}</span>
          </>
        ) : (
          <>
            <span><b>Reference</b> {active ? 'fresh' : 'waiting'}</span>
            <span><b>Measured</b> {state === 'robot_connected' || state === 'preflight_passed' ? 'DDS live' : mock ? 'synthetic' : 'simulation'}</span>
            <span><b>View</b> pelvis / world</span>
          </>
        )}
      </div>
    </div>
  )
}
