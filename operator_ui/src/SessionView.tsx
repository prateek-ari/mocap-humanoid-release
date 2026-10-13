import type { ReactNode } from 'react'
import { ChevronDown } from 'lucide-react'
import { G1Twin } from './G1Twin'
import { Lamp, Readout } from './components'
import {
  STATE_ORDER,
  derivedHealth,
  healthTone,
  isRecordedSimulation,
  isReported,
  ledForTone,
  stateIndex,
  stateLabel,
  stateScope,
  stateWord,
  valueText,
  type Led,
} from './status'
import type { HealthItem, OperatorStatus, PlaybackManifest } from './types'

export interface CrewProps {
  status: OperatorStatus
  readOnly: boolean
  operatorName: string
  spotterName: string
  stopChecked: boolean
  areaChecked: boolean
  harnessChecked: boolean
  onOperatorName: (value: string) => void
  onSpotterName: (value: string) => void
  onStopChecked: (value: boolean) => void
  onAreaChecked: (value: boolean) => void
  onHarnessChecked: (value: boolean) => void
}

function CrewPreflight(props: CrewProps) {
  const { status } = props
  const localReady = Boolean(props.operatorName.trim() && props.spotterName.trim() && props.stopChecked && props.areaChecked && props.harnessChecked)
  const held = status.health.preflight_confirmed
  const awaiting = status.state === 'robot_connected'
  return (
    <details className="group crew" open={awaiting || undefined}>
      <summary>
        <span className="silk">Crew preflight</span>
        <span className="crew-state"><Lamp state={held || localReady ? 'passed' : awaiting ? 'next' : 'off'} />{held ? 'Held by supervisor' : localReady ? 'Ready to confirm' : 'Names and stop checks required'}</span>
        <ChevronDown aria-hidden="true" />
      </summary>
      <div className="crew-fields">
        <label>Operator<input disabled={props.readOnly} value={props.operatorName} onChange={(event) => props.onOperatorName(event.target.value)} placeholder="Full name" autoComplete="name" /></label>
        <label>Spotter<input disabled={props.readOnly} value={props.spotterName} onChange={(event) => props.onSpotterName(event.target.value)} placeholder="Full name" autoComplete="name" /></label>
        <label className="check-row"><input disabled={props.readOnly} type="checkbox" checked={props.areaChecked} onChange={(event) => props.onAreaChecked(event.target.checked)} /><span>Operating area is clear</span></label>
        <label className="check-row"><input disabled={props.readOnly} type="checkbox" checked={props.harnessChecked} onChange={(event) => props.onHarnessChecked(event.target.checked)} /><span>Harness or physical support is ready</span></label>
        <label className="check-row"><input disabled={props.readOnly} type="checkbox" checked={props.stopChecked} onChange={(event) => props.onStopChecked(event.target.checked)} /><span>Independent stop drill rehearsed</span></label>
      </div>
    </details>
  )
}

function MeasurePanel({ status, demo }: { status: OperatorStatus; demo: boolean }) {
  const health = derivedHealth(status)
  const locked = !status.capability_lock.whole_body_unlocked
  const simulation = status.simulation ?? null
  const recordedSimulation = isRecordedSimulation(status)
  const finiteItem = (label: string, value: number | null | undefined, unit: string, detail: string, tone: HealthItem['status'] = 'ok', digits = 2): HealthItem => (
    typeof value === 'number' && Number.isFinite(value)
      ? { label, value: value.toFixed(digits), unit, detail, status: tone }
      : { label, value: 'NOT REPORTED', detail: 'The supervisor did not report this measurement.', status: 'offline' }
  )
  const inputRows: HealthItem[] = recordedSimulation ? [
    finiteItem('Source rate', simulation?.telemetry?.source_rate_hz ?? simulation?.source?.fps, 'Hz', 'Rate reported for recorded source frames.'),
    finiteItem('Input age', simulation?.telemetry?.source_age_ms, 'ms', 'Age of the source frame at the simulation boundary. Stale limit 100 ms.', (simulation?.telemetry?.source_age_ms ?? 0) <= 100 ? 'ok' : 'fault', 1),
  ] : [health.stream_rate, health.input_age, health.packet_loss]
  const clamped = simulation?.telemetry?.clamped_joint_count
  const simulationRows: HealthItem[] = simulation ? [
    finiteItem('Command rate', simulation.telemetry?.command_rate_hz, 'Hz', 'Rate of bounded simulation commands.'),
    finiteItem('Blend weight', simulation.telemetry?.blend_weight, '', 'Reported reference-to-bounded blend weight (0–1).', simulation.telemetry?.reset_required ? 'fault' : 'ok'),
  ] : []
  const clampItem: HealthItem | null = simulation ? (
    typeof clamped === 'number'
      ? { label: 'Clamped joints', value: clamped, detail: simulation.telemetry?.safety_reason?.trim() || 'No safety reason reported.', status: simulation.telemetry?.reset_required ? 'fault' : clamped > 0 ? 'warning' : 'ok' }
      : { label: 'Clamped joints', value: 'NOT REPORTED', detail: 'The supervisor did not report clamped joints.', status: 'offline' }
  ) : null
  const runtimeSystems = recordedSimulation
    ? ['simulation', 'controller', 'safety']
    : ['xsens', 'simulation', 'robot', 'controller', 'safety']
  const robotMeasurements = ['robot_age', 'battery', 'temperature', 'clamp'].map((key) => health[key])
  const reportedRobotMeasurements = robotMeasurements.filter((item) => isReported(item))
  const safetyReason = simulation?.telemetry?.safety_reason?.trim()

  return (
    <aside className="measure-column" aria-label={recordedSimulation ? 'Simulation evidence' : 'Live telemetry'}>
      <fieldset className="group">
        <legend className="silk">Measure{demo ? ' · synthetic' : ''}</legend>
        <div className="readout-grid">
          {[...inputRows, ...simulationRows].map((item) => <Readout key={item.label} item={item} />)}
          {clampItem && (
            <div className={`readout wide tone-${healthTone(clampItem)} ${isReported(clampItem) ? '' : 'is-unreported'}`} title={clampItem.detail}>
              <span className="silk">Clamped joints</span>
              {isReported(clampItem) ? (
                <strong>{String(clampItem.value)}{Number(clampItem.value) > 0 && safetyReason ? <small className="reason">{safetyReason}</small> : null}</strong>
              ) : <strong className="unreported">— —<small>not reported</small></strong>}
            </div>
          )}
        </div>
        {simulation && Array.isArray(simulation.limitations) && simulation.limitations.length > 0 && (
          <details className="fine-print">
            <summary>Reported limitations ({simulation.limitations.length})</summary>
            <ul>{simulation.limitations.map((limitation, index) => <li key={`${limitation}-${index}`}>{limitation}</li>)}</ul>
          </details>
        )}
      </fieldset>

      <fieldset className="group">
        <legend className="silk">Runtime chain</legend>
        <ul className="chain">
          {runtimeSystems.map((key) => {
            const item = health[key]
            return (
              <li key={key} title={item?.detail}>
                <Lamp state={ledForTone(healthTone(item))} label={item?.status} />
                <span>{item?.label ?? key}<small>{item?.detail}</small></span>
                <strong>{valueText(item)}</strong>
              </li>
            )
          })}
        </ul>
        {!recordedSimulation && (
          <details className="fine-print">
            <summary>{reportedRobotMeasurements.length ? `Robot measurements (${reportedRobotMeasurements.length})` : 'Robot measurements unavailable'}</summary>
            {reportedRobotMeasurements.length ? (
              <ul>{reportedRobotMeasurements.map((item) => <li key={item.label}>{item.label}: {valueText(item)}</li>)}</ul>
            ) : <p>Battery, joint temperature, robot state age, and safety clamps are not exposed by this supervisor.</p>}
          </details>
        )}
      </fieldset>

      <fieldset className={`group authority ${locked || recordedSimulation ? 'is-locked' : 'is-verified'}`}>
        <legend className="silk">Hardware authority</legend>
        <div className="authority-line">
          <Lamp state={locked || recordedSimulation ? 'fault' : 'passed'} />
          <strong>{recordedSimulation ? 'Blocked in simulation' : locked ? 'Locked' : status.state === 'tracking' ? 'Active' : 'Signed · available'}</strong>
        </div>
        <p>{recordedSimulation ? 'Recorded simulation never grants motor authority.' : status.capability_lock.reason}</p>
      </fieldset>
    </aside>
  )
}

interface SessionViewProps extends CrewProps {
  demo: boolean
  playback: PlaybackManifest | null
  keyBank: ReactNode
}

export function SessionView(props: SessionViewProps) {
  const { status } = props
  const recordedSimulation = isRecordedSimulation(status)
  const fault = status.state === 'fault_latched' || Boolean(status.active_fault)
  const simulation = status.simulation ?? null
  const progress = typeof simulation?.progress_pct === 'number' && Number.isFinite(simulation.progress_pct)
    ? Math.max(0, Math.min(100, simulation.progress_pct))
    : null
  const frameIndex = typeof simulation?.frame_index === 'number' && Number.isFinite(simulation.frame_index) ? simulation.frame_index : null
  const frameCount = typeof simulation?.source?.frame_count === 'number' && Number.isFinite(simulation.source.frame_count) ? simulation.source.frame_count : null
  const stateLed: Led = fault ? 'fault' : status.state === 'tracking' ? 'active' : status.state === 'checking' || status.state === 'disengaging' ? 'next' : stateIndex(status.state) > STATE_ORDER.indexOf('idle') ? 'passed' : 'off'

  return (
    <main className="session-workspace">
      <section className={`display-bezel ${fault ? 'is-fault' : ''}`} aria-label="Unitree G1 display">
        <div className="display">
          <div className="annunciator" aria-live="polite">
            <Lamp state={stateLed} />
            <div className="state-word">
              <strong>{stateWord(status)}</strong>
              <span>{stateLabel(status)}</span>
            </div>
            <p className="state-scope">{stateScope(status)}</p>
            <div className="annunciator-chips">
              {props.demo && <span className="chip">DEMO DATA</span>}
              {recordedSimulation && <span className="chip">RECORDED SIM</span>}
              {frameIndex !== null && <span className="chip mono">FRAME {frameIndex.toLocaleString()}{frameCount !== null ? ` / ${frameCount.toLocaleString()}` : ''}</span>}
            </div>
          </div>
          <G1Twin state={status.state} mock={props.demo} playback={props.playback} simulation={simulation} recordedSimulation={recordedSimulation} />
          {progress !== null && (
            <div className="progress-line">
              <span className="silk">Progress</span>
              <div className="progress-track" role="progressbar" aria-label="Recorded simulation progress" aria-valuemin={0} aria-valuemax={100} aria-valuenow={progress}><i style={{ width: `${progress}%` }} /></div>
              <strong className="mono">{progress.toFixed(1)}%</strong>
            </div>
          )}
        </div>
      </section>
      <div className="side-column">
        <MeasurePanel status={status} demo={props.demo} />
        <CrewPreflight {...props} />
      </div>
      <div className="inline-key-bank">{props.keyBank}</div>
    </main>
  )
}
