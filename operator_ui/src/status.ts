import type { ActionName, HealthItem, OperatorStatus, SessionState } from './types'

export type Led = 'passed' | 'next' | 'active' | 'off' | 'fault'

export const STATE_LABELS: Record<SessionState, string> = {
  setup_required: 'Setup required',
  idle: 'Idle / authority disabled',
  checking: 'Checking system',
  mocap_accepted: 'Mocap accepted',
  sim_validated: 'Simulation validated',
  robot_connected: 'Robot connected',
  preflight_passed: 'Preflight passed',
  armed_disabled: 'Armed / authority disabled',
  tracking: 'Tracking active',
  disengaging: 'Disengaging',
  fault_latched: 'Fault latched',
}

export const STATE_WORDS: Record<SessionState, string> = {
  setup_required: 'SETUP REQUIRED',
  idle: 'IDLE',
  checking: 'CHECKING',
  mocap_accepted: 'INPUT ACCEPTED',
  sim_validated: 'SIM VALIDATED',
  robot_connected: 'G1 CONNECTED',
  preflight_passed: 'PREFLIGHT PASSED',
  armed_disabled: 'ARMED · OUTPUT OFF',
  tracking: 'TRACKING',
  disengaging: 'DISENGAGING',
  fault_latched: 'FAULT LATCHED',
}

export const STATE_ORDER: SessionState[] = [
  'setup_required', 'idle', 'checking', 'mocap_accepted', 'sim_validated', 'robot_connected',
  'preflight_passed', 'armed_disabled', 'tracking',
]

export const ACTIONS: { name: ActionName; label: string; compact: string; group: 'evidence' | 'authority' | 'end' | 'stop'; reaches?: SessionState }[] = [
  { name: 'check', label: 'Check system', compact: 'Check', group: 'evidence', reaches: 'checking' },
  { name: 'mocap-accept', label: 'Test mocap', compact: 'Mocap', group: 'evidence', reaches: 'mocap_accepted' },
  { name: 'start-simulation', label: 'Start simulation', compact: 'Simulate', group: 'evidence', reaches: 'sim_validated' },
  { name: 'connect-robot', label: 'Connect robot', compact: 'Robot', group: 'evidence', reaches: 'robot_connected' },
  { name: 'confirm-preflight', label: 'Confirm preflight', compact: 'Preflight', group: 'evidence', reaches: 'preflight_passed' },
  { name: 'arm', label: 'Arm disabled', compact: 'Arm', group: 'authority', reaches: 'armed_disabled' },
  { name: 'start-tracking', label: 'Start tracking', compact: 'Track', group: 'authority', reaches: 'tracking' },
  { name: 'rearm', label: 'Rearm after fault', compact: 'Rearm', group: 'authority' },
  { name: 'shutdown', label: 'End session', compact: 'End session', group: 'end' },
  { name: 'disengage', label: 'Disengage', compact: 'Disengage', group: 'stop' },
]

const REQUIRED_STATE_COPY: Record<string, string> = {
  idle: 'End the current session before running a new system check.',
  checking: 'Run the system check first.',
  mocap_accepted: 'Accept the input stream first.',
  sim_validated: 'Validate the simulation first.',
  robot_connected: 'Connect the G1 or simulated G1 first.',
  preflight_passed: 'Complete the crew preflight first.',
  armed_disabled: 'Arm with command output disabled first.',
  fault_latched: 'Rearm is available only after a recoverable fault is latched.',
}

export const LED_WORDS: Record<Led, string> = {
  passed: 'Passed',
  next: 'Available',
  active: 'Active',
  off: 'Locked',
  fault: 'Fault',
}

export function isRecordedSimulation(status: OperatorStatus): boolean {
  return status.presentation_mode === 'recorded_simulation'
}

export function isReadOnlyPresentation(status: OperatorStatus): boolean {
  return status.presentation_mode === 'recorded_playback'
}

export function isDemoStatus(status: OperatorStatus): boolean {
  return status.presentation_mode === 'demo' || status.runtime_mode === 'fake'
}

export function humanizeToken(value: string): string {
  const words = value.replaceAll('_', ' ').replaceAll('-', ' ').trim()
  return words ? `${words.charAt(0).toUpperCase()}${words.slice(1)}` : 'Not reported'
}

export function stateLabel(status: OperatorStatus): string {
  if (isRecordedSimulation(status)) {
    return `Recorded sim · ${STATE_LABELS[status.state]}`
  }
  if (status.presentation_mode === 'recorded_playback') return 'Recorded review / authority disabled'
  if (isDemoStatus(status)) {
    if (status.state === 'tracking') return 'Simulation tracking active'
    if (status.state === 'robot_connected') return 'Simulated G1 connected'
    if (status.state === 'armed_disabled') return 'Simulation armed / output disabled'
  }
  return STATE_LABELS[status.state]
}

export function stateWord(status: OperatorStatus): string {
  if (isReadOnlyPresentation(status)) return 'RECORDED REVIEW'
  const simulated = isRecordedSimulation(status) || isDemoStatus(status)
  if (simulated && status.state === 'tracking') return 'SIM TRACKING'
  if (simulated && status.state === 'armed_disabled') return 'SIM ARMED · OUTPUT OFF'
  if (status.state === 'robot_connected') {
    if (isRecordedSimulation(status)) return 'VIRTUAL G1 READY'
    if (isDemoStatus(status)) return 'SIM G1 CONNECTED'
  }
  return STATE_WORDS[status.state]
}

export function stateScope(status: OperatorStatus): string {
  if (isReadOnlyPresentation(status)) return 'Passive playback of recorded media. Command authority is zero and controls are disabled.'
  if (isRecordedSimulation(status)) return 'Virtual G1 only. No DDS transport is open; no motor receives a command.'
  if (isDemoStatus(status)) return 'Backend demo runtime. Values are synthetic; no hardware is connected.'
  if (status.active_fault) return status.active_fault.reason || 'The supervisor latched a fault.'
  return status.health.command_authority
    ? 'Software command authority is active. The independent physical stop remains separate.'
    : 'No software command authority. Hardware control stays behind its signed capability.'
}

export function actionCopy(action: ActionName, status: OperatorStatus): { label: string; compact: string } {
  const base = ACTIONS.find((item) => item.name === action)
  if (!base) return { label: humanizeToken(action), compact: humanizeToken(action) }
  if (isRecordedSimulation(status)) {
    const recordedCopy: Partial<Record<ActionName, { label: string; compact: string }>> = {
      check: { label: 'Check recorded source', compact: 'Check' },
      'mocap-accept': { label: 'Accept recorded input', compact: 'Input' },
      'start-simulation': { label: 'Validate recorded simulation', compact: 'Validate' },
      'connect-robot': { label: 'Connect virtual G1', compact: 'Virtual G1' },
      'confirm-preflight': { label: 'Confirm simulation preflight', compact: 'Preflight' },
      arm: { label: 'Arm simulation', compact: 'Arm sim' },
      'start-tracking': { label: 'Start simulation tracking', compact: 'Track sim' },
      rearm: { label: 'Rearm simulation', compact: 'Rearm' },
      shutdown: { label: 'End simulated session', compact: 'End session' },
      disengage: { label: 'Stop simulation', compact: 'Stop sim' },
    }
    return recordedCopy[action] ?? base
  }
  if (!isDemoStatus(status)) return base
  const demoCopy: Partial<Record<ActionName, { label: string; compact: string }>> = {
    check: { label: 'Check demo runtime', compact: 'Check' },
    'mocap-accept': { label: 'Accept synthetic input', compact: 'Input' },
    'start-simulation': { label: 'Validate simulation', compact: 'Validate' },
    'connect-robot': { label: 'Connect simulated G1', compact: 'Sim G1' },
    'confirm-preflight': { label: 'Confirm simulated preflight', compact: 'Preflight' },
    arm: { label: 'Arm simulation', compact: 'Arm sim' },
    'start-tracking': { label: 'Start simulated tracking', compact: 'Track sim' },
    rearm: { label: 'Rearm simulation', compact: 'Rearm' },
    shutdown: { label: 'End demo session', compact: 'End session' },
    disengage: { label: 'Stop simulation', compact: 'Stop sim' },
  }
  return demoCopy[action] ?? base
}

export function friendlyGateReason(reason: string | null | undefined): string {
  if (!reason) return 'Available now.'
  const required = reason.match(/^Requires ([a-z_]+)$/)?.[1]
  if (required && REQUIRED_STATE_COPY[required]) return REQUIRED_STATE_COPY[required]
  return reason.replaceAll('_', ' ')
}

export function stateIndex(state: SessionState): number {
  if (state === 'fault_latched') return -1
  if (state === 'disengaging') return STATE_ORDER.indexOf('robot_connected')
  return STATE_ORDER.indexOf(state)
}

export function derivedHealth(status: OperatorStatus): Record<string, HealthItem> {
  const health = status.health
  const notReported = (label: string): HealthItem => ({ status: 'offline', label, detail: 'Not reported by the v1 supervisor', value: 'NOT REPORTED' })
  const message = health.messages[0] || 'No supervisor detail reported'
  const workstationTone = status.state === 'fault_latched' ? 'fault' : status.state === 'checking' ? 'checking' : health.overall === 'offline' ? 'offline' : 'ok'
  return {
    workstation: { status: workstationTone, label: 'Workstation', detail: message, value: health.overall.toUpperCase() },
    xsens: { status: health.mocap_online ? 'ok' : 'offline', label: 'Xsens stream', detail: health.mocap_online ? (health.mocap_accepted ? 'Stream accepted by supervisor' : 'Online; acceptance pending') : 'Supervisor reports mocap offline', value: health.mocap_online ? 'ONLINE' : 'OFFLINE' },
    retarget: notReported('Retargeting'),
    simulation: { status: health.simulation_ready ? 'ok' : 'offline', label: 'Simulation', detail: health.simulation_ready ? 'Supervisor reports validation ready' : 'Simulation not validated', value: health.simulation_ready ? 'VALIDATED' : 'NOT READY' },
    robot: { status: health.robot_connected && health.robot_state_fresh ? 'ok' : health.robot_connected ? 'warning' : 'offline', label: 'Unitree G1', detail: health.robot_connected ? (health.robot_state_fresh ? 'Connected with fresh state' : 'Connected; state is stale') : 'Supervisor reports robot disconnected', value: health.robot_connected ? 'CONNECTED' : 'OFFLINE' },
    controller: { status: health.controller_ready ? 'ok' : 'offline', label: 'Whole-body control', detail: health.controller_ready ? 'Controller reports ready' : 'Controller not ready', value: health.controller_ready ? 'READY' : 'NOT READY' },
    safety: { status: health.command_authority ? 'warning' : 'ok', label: 'Safety supervisor', detail: health.command_authority ? 'Command authority is active' : 'No command authority', value: health.command_authority ? 'ACTIVE' : 'DISABLED' },
    stream_rate: { status: health.mocap_online ? (health.mocap_rate_hz >= 50 ? 'ok' : 'warning') : 'offline', label: 'Mocap rate', detail: health.mocap_online ? 'Supervisor measurement' : 'Not reported while source is offline', value: health.mocap_online ? health.mocap_rate_hz.toFixed(1) : 'NOT REPORTED', unit: health.mocap_online ? 'Hz' : undefined },
    packet_loss: { status: health.mocap_online ? (health.packet_loss_percent <= 1 ? 'ok' : 'fault') : 'offline', label: 'Packet loss', detail: health.mocap_online ? 'Acceptance maximum 1%' : 'Not reported while source is offline', value: health.mocap_online ? health.packet_loss_percent.toFixed(2) : 'NOT REPORTED', unit: health.mocap_online ? '%' : undefined },
    input_age: { status: health.source_age_ms === null ? 'offline' : health.source_age_ms <= 100 ? 'ok' : 'fault', label: 'Input age', detail: health.source_age_ms === null ? 'Not reported by supervisor' : 'Stale limit 100 ms', value: health.source_age_ms === null ? 'NOT REPORTED' : health.source_age_ms.toFixed(1), unit: health.source_age_ms === null ? undefined : 'ms' },
    robot_age: notReported('Robot state age'),
    battery: notReported('Battery'),
    temperature: notReported('Peak joint temp'),
    clamp: notReported('Safety clamps'),
  }
}

export function healthTone(item?: HealthItem): string {
  return item?.status ?? 'offline'
}

export function isReported(item?: HealthItem): boolean {
  return Boolean(item) && item?.value !== 'NOT REPORTED' && item?.value !== null && item?.value !== undefined && item?.value !== ''
}

export function valueText(item?: HealthItem): string {
  if (!item || item.value === null || item.value === undefined || item.value === '') return '—'
  if (typeof item.value === 'boolean') return item.value ? 'YES' : 'NO'
  return `${item.value}${item.unit ? ` ${item.unit}` : ''}`
}

export function ledForTone(tone: string): Led {
  if (tone === 'ok') return 'passed'
  if (tone === 'fault') return 'fault'
  if (tone === 'warning' || tone === 'checking') return 'next'
  return 'off'
}

const FAULT_TITLES: Record<string, string> = {
  health_stale: 'Runtime health stale',
  tracking_health_lost: 'Tracking health lost',
  runtime_adapter_timeout: 'Runtime adapter timed out',
  runtime_adapter_unavailable: 'Runtime adapter blocked',
  runtime_watchdog_error: 'Runtime watchdog error',
  watchdog_overdue: 'Supervisor watchdog overdue',
  watchdog_tick_failed: 'Supervisor watchdog failed',
  watchdog_stopped: 'Supervisor watchdog stopped',
}

/** Operator-facing name for a latched fault code. */
export function faultTitle(code: string | null | undefined): string {
  if (!code) return 'Fault'
  return FAULT_TITLES[code] ?? humanizeToken(code)
}

function formatAge(seconds: number): string {
  if (seconds < 60) return `${seconds}s`
  if (seconds < 3600) return `${Math.floor(seconds / 60)}m ${seconds % 60}s`
  return `${Math.floor(seconds / 3600)}h ${Math.floor((seconds % 3600) / 60)}m`
}

/** "Last update 14:03:22 (12s ago)" from the supervisor's ``health.updated_at`` (epoch seconds). */
export function lastUpdateText(updatedAt: number, nowMs: number): string {
  if (!Number.isFinite(updatedAt) || updatedAt <= 0) return 'Last update time not reported.'
  const at = new Date(updatedAt * 1000)
  const ageS = Math.max(0, Math.floor(nowMs / 1000 - updatedAt))
  return `Last update ${at.toLocaleTimeString([], { hour12: false })} (${formatAge(ageS)} ago).`
}

/** Supervisor self-diagnostics that the operator must see (audit chain, adapter, watchdog). */
export function supervisorWarnings(status: OperatorStatus): string[] {
  const warnings: string[] = []
  const integrity = status.audit_integrity
  if (integrity && (!integrity.chain_verified || integrity.tail_matches_last_write === false)) {
    const breaks = integrity.chain_breaks === 1 ? '1 chain break' : `${integrity.chain_breaks} chain breaks`
    const tail = integrity.tail_matches_last_write === false ? '; the file changed after the last write' : ''
    warnings.push(`Audit log integrity not verified (${breaks}${tail}). Export diagnostics before continuing.`)
  }
  const blocked = status.runtime_adapter?.blocked_call
  if (blocked) {
    warnings.push(`Runtime adapter is blocked in an overdue ${blocked} call; runtime actions are refused until it returns.`)
  }
  if (status.watchdog && !status.watchdog.running) {
    warnings.push('Supervisor watchdog is not running; time-based safety checks are not enforced.')
  } else if (status.watchdog?.overdue) {
    warnings.push('Supervisor watchdog tick is overdue.')
  }
  return warnings
}

export function auditIntegrityText(status: OperatorStatus): string {
  const integrity = status.audit_integrity
  if (!integrity) return 'Hash chain: not reported.'
  if (!integrity.persistent) return 'Hash chain: in-memory audit only (not persisted).'
  if (!integrity.chain_verified || integrity.tail_matches_last_write === false) {
    return `Hash chain: NOT VERIFIED · ${integrity.chain_breaks} break${integrity.chain_breaks === 1 ? '' : 's'}.`
  }
  return `Hash chain: verified · ${integrity.chained_records} chained record${integrity.chained_records === 1 ? '' : 's'}.`
}
