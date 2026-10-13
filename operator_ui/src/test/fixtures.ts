import type { ActionName, OperatorStatus } from '../types'

const ACTION_NAMES: ActionName[] = [
  'check', 'mocap-accept', 'start-simulation', 'connect-robot', 'confirm-preflight',
  'arm', 'start-tracking', 'disengage', 'rearm', 'shutdown',
]

export function makeStatus(overrides: Partial<OperatorStatus> = {}): OperatorStatus {
  const controls: OperatorStatus['controls'] = Object.fromEntries(
    ACTION_NAMES.map((name) => [name, { enabled: true, reason: null }]),
  )
  controls['edit-config'] = { enabled: true, reason: null }
  return {
    schema_version: 1,
    instance_id: 'instance-a',
    presentation_mode: 'live',
    topology: 'all_in_one',
    runtime_mode: 'real',
    state: 'tracking',
    capability_lock: {
      whole_body_unlocked: true,
      manifest_verified: true,
      reason: 'Verified',
      manifest_id: 'qualification-001',
      expires_at: '2035-01-01T00:00:00Z',
      capabilities: ['whole_body'],
    },
    health: {
      mocap_online: true,
      mocap_accepted: true,
      mocap_rate_hz: 60,
      packet_loss_percent: 0,
      source_age_ms: 4,
      simulation_ready: true,
      robot_connected: true,
      robot_state_fresh: true,
      controller_ready: true,
      preflight_confirmed: true,
      command_authority: true,
      overall: 'tracking',
      updated_at: 0,
      messages: [],
    },
    active_fault: null,
    simulation: null,
    fault_condition_cleared: false,
    last_event_sequence: 10,
    controls,
    runtime_adapter: { blocked_call: null },
    audit_integrity: {
      persistent: true,
      chain_verified: true,
      chain_breaks: 0,
      chained_records: 12,
      unchained_records: 0,
      unreadable_lines: 0,
      checked_at: 0,
    },
    watchdog: { running: true, overdue: false },
    ...overrides,
  }
}
