import type { ReactNode } from 'react'
import { RefreshCw, XOctagon } from 'lucide-react'
import { Lamp } from './components'
import { keyLed, keyUnavailable } from './gating'
import { DISENGAGE_SHORTCUT_KEY, DISENGAGE_SHORTCUT_LABEL } from './shortcuts'
import {
  ACTIONS,
  LED_WORDS,
  actionCopy,
  friendlyGateReason,
  isDemoStatus,
  isReadOnlyPresentation,
  isRecordedSimulation,
} from './status'
import type { ActionName, ControlState, OperatorStatus } from './types'

interface KeyBankProps {
  status: OperatorStatus
  pending: ReadonlySet<ActionName>
  gateFor: (name: ActionName) => ControlState
  onAction: (action: ActionName) => void
  onInspect: (text: string | null) => void
  idPrefix: string
}

export function KeyBank({ status, pending, gateFor, onAction, onInspect, idPrefix }: KeyBankProps) {
  const renderKey = (name: ActionName) => {
    const gate = gateFor(name)
    const copy = actionCopy(name, status)
    const reason = friendlyGateReason(gate.reason)
    const led = keyLed(name, gate.enabled, status)
    const unavailable = keyUnavailable(name, gate, pending)
    const describedBy = `${idPrefix}-${name}-reason`
    return (
      <button
        key={name}
        className={`panel-key gate-key led-${led}`}
        aria-disabled={unavailable}
        aria-describedby={describedBy}
        onClick={() => { if (!unavailable) onAction(name) }}
        onFocus={() => onInspect(`${copy.label}: ${gate.enabled ? 'available now.' : reason}`)}
        onBlur={() => onInspect(null)}
        onMouseEnter={() => onInspect(`${copy.label}: ${gate.enabled ? 'available now.' : reason}`)}
        onMouseLeave={() => onInspect(null)}
      >
        {pending.has(name) ? <RefreshCw className="spin key-spinner" aria-hidden="true" /> : <Lamp state={led} />}
        <span className="key-label">{copy.compact}</span>
        <span className="key-state">{LED_WORDS[led]}</span>
        <span className="sr-only" id={describedBy}>{gate.enabled ? 'Available now.' : reason}</span>
      </button>
    )
  }
  return (
    <div className="key-bank">
      <fieldset className="group evidence-group">
        <legend className="silk">Evidence</legend>
        <div className="keys">{ACTIONS.filter((action) => action.group === 'evidence').map((action) => renderKey(action.name))}</div>
      </fieldset>
      <fieldset className="group authority-group">
        <legend className="silk">Authority</legend>
        <div className="keys">{ACTIONS.filter((action) => action.group === 'authority').map((action) => renderKey(action.name))}</div>
      </fieldset>
    </div>
  )
}

interface ActionDockProps {
  status: OperatorStatus
  pending: ReadonlySet<ActionName>
  preflightComplete: boolean
  gateFor: (name: ActionName) => ControlState
  inspect: string | null
  keyBank: ReactNode
  onAction: (action: ActionName) => void
}

export function ActionDock({ status, pending, preflightComplete, gateFor, inspect, keyBank, onAction }: ActionDockProps) {
  const next = ACTIONS.find((action) => action.group !== 'stop' && action.group !== 'end' && gateFor(action.name).enabled)
  const shutdownGate = gateFor('shutdown')
  const disengageGate = gateFor('disengage')
  const nextCopy = next ? actionCopy(next.name, status) : null
  const nextLabel = nextCopy?.label
    ?? (isRecordedSimulation(status)
      ? status.state === 'tracking' ? 'Simulation tracking active' : 'Awaiting simulation supervisor'
      : status.presentation_mode === 'recorded_playback'
      ? 'Recorded review / controls disabled'
      : status.state === 'robot_connected' && !preflightComplete
      ? 'Complete crew preflight'
      : (!status.capability_lock.whole_body_unlocked && status.runtime_mode !== 'fake'
        ? 'Hardware capability required'
        : 'Awaiting supervisor'))
  const contextDetail = next
    ? 'Only this routine action can advance the session.'
    : status.state === 'robot_connected' && !preflightComplete
    ? 'Enter both names and all three physical checks under Crew preflight.'
    : isReadOnlyPresentation(status)
    ? 'Recorded evidence never grants command authority.'
    : status.active_fault
    ? 'Review the fault evidence before rearming or ending the session.'
    : 'The supervisor has not exposed another routine action.'
  const simulatedSession = isRecordedSimulation(status) || isDemoStatus(status)
  // DISENGAGE and End session never wait behind another pending action;
  // each is guarded only against a double submit of itself.
  const stopUnavailable = keyUnavailable('disengage', disengageGate, pending)
  const endUnavailable = keyUnavailable('shutdown', shutdownGate, pending)
  const nextUnavailable = next ? keyUnavailable(next.name, gateFor(next.name), pending) : true

  return (
    <footer className="action-dock" aria-label="Session actions">
      <div className="status-line" role="status" aria-live="polite">
        <span className="silk">{inspect ? 'Key' : next ? 'Next' : 'Hold'}</span>
        <strong>{inspect ?? nextLabel}</strong>
        {!inspect && <small>{contextDetail}</small>}
      </div>
      {next && nextCopy && (
        <button className="panel-key gate-key led-next next-key" aria-disabled={nextUnavailable} onClick={() => { if (!nextUnavailable) onAction(next.name) }}>
          {pending.has(next.name) ? <RefreshCw className="spin key-spinner" aria-hidden="true" /> : <Lamp state="next" />}
          <span className="key-label">{nextCopy.label}</span>
          <span className="key-state">Next permitted action</span>
        </button>
      )}
      <div className="dock-bank">{keyBank}</div>
      <fieldset className="group stop-group">
        <legend className="silk">Stop</legend>
        <button
          className="panel-key end-key"
          aria-disabled={endUnavailable}
          onClick={() => { if (!endUnavailable) onAction('shutdown') }}
          aria-label={`${actionCopy('shutdown', status).label}. ${shutdownGate.enabled ? 'Ends the current runtime session.' : friendlyGateReason(shutdownGate.reason)}`}
        >
          {pending.has('shutdown') ? <RefreshCw className="spin key-spinner" aria-hidden="true" /> : <span className="silk">Session</span>}
          <span className="key-label">{actionCopy('shutdown', status).compact}</span>
        </button>
        <button
          className="stop-key"
          aria-disabled={stopUnavailable}
          aria-keyshortcuts={DISENGAGE_SHORTCUT_KEY}
          onClick={() => { if (!stopUnavailable) onAction('disengage') }}
          aria-label={`${actionCopy('disengage', status).label}. ${disengageGate.enabled ? simulatedSession ? 'Stops simulated tracking.' : 'Removes software command authority. The independent physical stop remains separate.' : friendlyGateReason(disengageGate.reason)}`}
        >
          {pending.has('disengage') ? <RefreshCw className="spin" aria-hidden="true" /> : <XOctagon aria-hidden="true" />}
          <strong>{simulatedSession ? 'STOP SIM' : 'DISENGAGE'}</strong>
          <small>{simulatedSession ? 'Simulated authority only' : 'Software authority only'}</small>
        </button>
        <p className="stop-note">Not an emergency stop. Keep the Unitree remote / damping path within reach. Shortcut: <kbd>{DISENGAGE_SHORTCUT_LABEL}</kbd> (outside text fields).</p>
      </fieldset>
    </footer>
  )
}
