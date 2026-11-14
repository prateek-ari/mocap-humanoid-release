/**
 * Runtime guards for supervisor payloads.  A status drives which keys are
 * enabled, so anything that does not match the v1 shape is rejected instead
 * of being cast into place.  Unknown extra fields are tolerated.
 */
import type { OperatorEvent, OperatorStatus, SessionState } from './types'

const SESSION_STATES: ReadonlySet<string> = new Set<SessionState>([
  'setup_required', 'idle', 'checking', 'mocap_accepted', 'sim_validated', 'robot_connected',
  'preflight_passed', 'armed_disabled', 'tracking', 'disengaging', 'fault_latched',
])
const PRESENTATION_MODES: ReadonlySet<string> = new Set(['live', 'demo', 'recorded_playback', 'recorded_simulation'])
const TOPOLOGIES: ReadonlySet<string> = new Set(['all_in_one', 'split'])
const RUNTIME_MODES: ReadonlySet<string> = new Set(['real', 'fake'])

export function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === 'object' && value !== null && !Array.isArray(value)
}

const isBool = (value: unknown): value is boolean => typeof value === 'boolean'
const isString = (value: unknown): value is string => typeof value === 'string'
const isFiniteNumber = (value: unknown): value is number => typeof value === 'number' && Number.isFinite(value)
const isCount = (value: unknown): value is number => isFiniteNumber(value) && value >= 0
const isStringOrNull = (value: unknown): value is string | null => value === null || isString(value)
const isStringArray = (value: unknown): value is string[] => Array.isArray(value) && value.every(isString)
const isNumberArrayOrNull = (value: unknown) => value === undefined || value === null || (Array.isArray(value) && value.every(isFiniteNumber))

function hasFields(value: Record<string, unknown>, checks: Record<string, (field: unknown) => boolean>): boolean {
  return Object.entries(checks).every(([name, check]) => check(value[name]))
}

function isCapabilityLock(value: unknown): boolean {
  return isRecord(value) && hasFields(value, {
    whole_body_unlocked: isBool,
    manifest_verified: isBool,
    reason: isString,
    manifest_id: isStringOrNull,
    expires_at: isStringOrNull,
    capabilities: isStringArray,
  })
}

function isHealth(value: unknown): boolean {
  return isRecord(value) && hasFields(value, {
    mocap_online: isBool,
    mocap_accepted: isBool,
    mocap_rate_hz: isFiniteNumber,
    packet_loss_percent: isFiniteNumber,
    source_age_ms: (field) => field === null || isFiniteNumber(field),
    simulation_ready: isBool,
    robot_connected: isBool,
    robot_state_fresh: isBool,
    controller_ready: isBool,
    preflight_confirmed: isBool,
    command_authority: isBool,
    overall: isString,
    updated_at: isFiniteNumber,
    messages: isStringArray,
  })
}

function isFault(value: unknown): boolean {
  return value === null || (isRecord(value) && hasFields(value, {
    code: isString,
    reason: isString,
    source: isString,
    timestamp: isFiniteNumber,
    latched: isBool,
    recoverable: isBool,
  }))
}

function isControls(value: unknown): boolean {
  return isRecord(value) && Object.values(value).every((control) => (
    isRecord(control) && isBool(control.enabled) && isStringOrNull(control.reason)
  ))
}

function isSimulation(value: unknown): boolean {
  if (value === undefined || value === null) return true
  return isRecord(value)
    && isNumberArrayOrNull(value.reference_qpos)
    && isNumberArrayOrNull(value.bounded_qpos)
    && (value.source === undefined || value.source === null || isRecord(value.source))
    && (value.telemetry === undefined || value.telemetry === null || isRecord(value.telemetry))
}

function isOptional(value: unknown, check: (field: unknown) => boolean): boolean {
  return value === undefined || check(value)
}

function isAuditIntegrity(value: unknown): boolean {
  return value === null || (isRecord(value) && hasFields(value, {
    persistent: isBool,
    chain_verified: isBool,
    chain_breaks: isCount,
    chained_records: isCount,
    unchained_records: isCount,
    unreadable_lines: isCount,
    checked_at: (field) => isOptional(field, isFiniteNumber),
    tail_matches_last_write: (field) => isOptional(field, isBool),
  }))
}

/** The validated status, or ``null`` when ``value`` is not a v1 status. */
export function parseOperatorStatus(value: unknown): OperatorStatus | null {
  if (!isRecord(value)) return null
  const valid = hasFields(value, {
    schema_version: isFiniteNumber,
    instance_id: (field) => isOptional(field, isString),
    presentation_mode: (field) => isString(field) && PRESENTATION_MODES.has(field),
    topology: (field) => isString(field) && TOPOLOGIES.has(field),
    runtime_mode: (field) => isString(field) && RUNTIME_MODES.has(field),
    state: (field) => isString(field) && SESSION_STATES.has(field),
    capability_lock: isCapabilityLock,
    health: isHealth,
    active_fault: isFault,
    simulation: isSimulation,
    fault_condition_cleared: isBool,
    last_event_sequence: isCount,
    controls: isControls,
    runtime_adapter: (field) => isOptional(field, (adapter) => isRecord(adapter) && isStringOrNull(adapter.blocked_call)),
    audit_integrity: (field) => isOptional(field, isAuditIntegrity),
    watchdog: (field) => isOptional(field, (watchdog) => isRecord(watchdog) && isBool(watchdog.running) && isBool(watchdog.overdue)),
  })
  return valid ? value as unknown as OperatorStatus : null
}

/**
 * The validated event, or ``null``.  An event that carries a ``status`` is
 * rejected as a whole when that status is malformed; a snapshot must carry one.
 */
export function parseOperatorEvent(value: unknown): OperatorEvent | null {
  if (!isRecord(value)) return null
  const valid = hasFields(value, {
    schema_version: isFiniteNumber,
    instance_id: (field) => isOptional(field, isString),
    sequence: isCount,
    audit_sequence: (field) => field === undefined || field === null || isCount(field),
    timestamp: (field) => isOptional(field, isFiniteNumber),
    type: isString,
    action: (field) => isOptional(field, isString),
    outcome: (field) => isOptional(field, isString),
    state: (field) => isOptional(field, (state) => isString(state) && SESSION_STATES.has(state)),
    actor: (field) => isOptional(field, isString),
    details: (field) => isOptional(field, isRecord),
  })
  if (!valid) return null
  if (value.status === undefined) {
    return value.type === 'snapshot' ? null : value as unknown as OperatorEvent
  }
  return parseOperatorStatus(value.status) ? value as unknown as OperatorEvent : null
}
