import { describe, expect, it } from 'vitest'
import { parseOperatorEvent, parseOperatorStatus } from './protocol'
import { makeStatus } from './test/fixtures'

const clone = <T>(value: T): T => JSON.parse(JSON.stringify(value)) as T

describe('parseOperatorStatus', () => {
  it('accepts a full v1 status, including the supervisor diagnostics', () => {
    const status = clone(makeStatus())
    expect(parseOperatorStatus(status)).toEqual(status)
  })

  it('accepts a status from a supervisor that predates the diagnostics fields', () => {
    const status: Record<string, unknown> = { ...clone(makeStatus()) }
    delete status.runtime_adapter
    delete status.audit_integrity
    delete status.watchdog
    expect(parseOperatorStatus(status)).not.toBeNull()
    expect(parseOperatorStatus({ ...status, audit_integrity: null })).not.toBeNull()
  })

  it.each([
    ['an unknown session state', { state: 'warp_speed' }],
    ['an unknown presentation mode', { presentation_mode: 'hud' }],
    ['a missing control map', { controls: undefined }],
    ['a control without an enabled flag', { controls: { disengage: { reason: null } } }],
    ['a string sequence', { last_event_sequence: '10' }],
    ['a negative sequence', { last_event_sequence: -1 }],
    ['a malformed fault', { active_fault: { code: 'x' } }],
    ['a non-boolean authority flag', { health: { ...makeStatus().health, command_authority: 'yes' } }],
    ['a non-numeric qpos', { simulation: { reference_qpos: [0, 'a'] } }],
    ['a malformed audit integrity block', { audit_integrity: { chain_verified: 'yes' } }],
    ['a malformed watchdog block', { watchdog: { running: 1, overdue: false } }],
    ['a malformed runtime adapter block', { runtime_adapter: { blocked_call: 3 } }],
  ])('rejects %s', (_label, override) => {
    expect(parseOperatorStatus({ ...clone(makeStatus()), ...override })).toBeNull()
  })

  it('rejects non-objects', () => {
    for (const value of [null, undefined, 'status', 3, [makeStatus()]]) expect(parseOperatorStatus(value)).toBeNull()
  })
})

describe('parseOperatorEvent', () => {
  const status = makeStatus()

  it('accepts snapshots and audit events', () => {
    expect(parseOperatorEvent({ schema_version: 1, instance_id: 'instance-a', sequence: 10, type: 'snapshot', status })).not.toBeNull()
    expect(parseOperatorEvent({
      schema_version: 1, instance_id: 'instance-a', sequence: 11, audit_sequence: null, timestamp: 1, type: 'control',
      action: 'check', outcome: 'accepted', state: 'checking', actor: 'operator', details: {},
    })).not.toBeNull()
  })

  it('rejects a snapshot without a valid status', () => {
    expect(parseOperatorEvent({ schema_version: 1, sequence: 10, type: 'snapshot' })).toBeNull()
    expect(parseOperatorEvent({ schema_version: 1, sequence: 10, type: 'snapshot', status: { ...status, state: 'nope' } })).toBeNull()
  })

  it('rejects any event whose attached status is malformed', () => {
    expect(parseOperatorEvent({ schema_version: 1, sequence: 11, type: 'control', status: { state: 'tracking' } })).toBeNull()
  })

  it.each([
    ['a missing sequence', { sequence: undefined }],
    ['a non-finite sequence', { sequence: Number.POSITIVE_INFINITY }],
    ['a missing type', { type: undefined }],
    ['an unknown state', { state: 'warp_speed' }],
    ['array details', { details: [] }],
  ])('rejects %s', (_label, override) => {
    expect(parseOperatorEvent({ schema_version: 1, sequence: 11, type: 'control', ...override })).toBeNull()
  })
})
