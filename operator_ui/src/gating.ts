import { ACTIONS, isReadOnlyPresentation, stateIndex, type Led } from './status'
import type { ActionName, ControlState, OperatorStatus } from './types'

/**
 * Actions that remove or end authority.  They must never wait behind a
 * pending routine action; each is guarded only against its own double submit.
 */
export const SAFETY_ACTIONS: ReadonlySet<ActionName> = new Set<ActionName>(['disengage', 'shutdown'])

export function isSafetyAction(action: ActionName): boolean {
  return SAFETY_ACTIONS.has(action)
}

export interface GateInputs {
  status: OperatorStatus | null
  wsConnected: boolean
  preflightComplete: boolean
}

/** Combine the supervisor's control gate with client-side evidence checks. */
export function gateFor(name: ActionName, { status, wsConnected, preflightComplete }: GateInputs): ControlState {
  if (!status) return { enabled: false, reason: 'Supervisor status is not loaded.' }
  if (isReadOnlyPresentation(status)) {
    return { enabled: false, reason: 'Recorded playback is read-only. Command authority is already zero.' }
  }
  const backendGate = status.controls[name] ?? { enabled: false, reason: 'Supervisor did not expose this control.' }
  if (!wsConnected && !isSafetyAction(name)) {
    return { enabled: false, reason: 'Supervisor event channel is unavailable; readiness evidence may be stale.' }
  }
  if (name === 'confirm-preflight' && backendGate.enabled && !preflightComplete) {
    return { enabled: false, reason: 'Enter both names, confirm the clear area and physical support, and rehearse the independent stop path.' }
  }
  return backendGate
}

/**
 * Whether ``action`` may be submitted while ``pending`` requests are in flight.
 * Routine actions are serialized behind every pending request; safety actions
 * are blocked only by an in-flight request for the same action.
 */
export function canSubmit(action: ActionName, pending: ReadonlySet<ActionName>): boolean {
  if (pending.has(action)) return false
  if (isSafetyAction(action)) return true
  return pending.size === 0
}

export function keyUnavailable(action: ActionName, gate: ControlState, pending: ReadonlySet<ActionName>): boolean {
  return !gate.enabled || !canSubmit(action, pending)
}

/**
 * Accept a status only if it is not older than the one already shown.
 * Responses for concurrent requests can arrive out of order; a stale
 * "tracking" response must never overwrite a newer disengaged status.
 */
export function isNewerStatus(current: OperatorStatus | null, next: OperatorStatus): boolean {
  if (!current) return true
  if (current.instance_id && next.instance_id && current.instance_id !== next.instance_id) return true
  return next.last_event_sequence >= current.last_event_sequence
}

export function keyLed(name: ActionName, enabled: boolean, status: OperatorStatus): Led {
  const fault = status.state === 'fault_latched' || Boolean(status.active_fault)
  if (enabled) return 'next'
  const reaches = ACTIONS.find((action) => action.name === name)?.reaches
  if (fault || !reaches) return 'off'
  if (name === 'start-tracking' && status.state === 'tracking') return 'active'
  return stateIndex(status.state) >= stateIndex(reaches) ? 'passed' : 'off'
}
