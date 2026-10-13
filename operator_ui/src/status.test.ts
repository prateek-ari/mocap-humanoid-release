import { describe, expect, it } from 'vitest'
import {
  actionCopy,
  auditIntegrityText,
  derivedHealth,
  faultTitle,
  friendlyGateReason,
  lastUpdateText,
  supervisorWarnings,
} from './status'
import { makeStatus } from './test/fixtures'
import type { AuditIntegrity } from './types'

describe('actionCopy', () => {
  it('uses the hardware wording in a live session', () => {
    expect(actionCopy('disengage', makeStatus())).toMatchObject({ label: 'Disengage', compact: 'Disengage' })
    expect(actionCopy('arm', makeStatus()).label).toBe('Arm disabled')
  })

  it('says simulation in demo and recorded-simulation sessions', () => {
    expect(actionCopy('disengage', makeStatus({ presentation_mode: 'demo' })).label).toBe('Stop simulation')
    expect(actionCopy('connect-robot', makeStatus({ runtime_mode: 'fake' })).label).toBe('Connect simulated G1')
    expect(actionCopy('connect-robot', makeStatus({ presentation_mode: 'recorded_simulation' })).label).toBe('Connect virtual G1')
    expect(actionCopy('shutdown', makeStatus({ presentation_mode: 'recorded_simulation' })).label).toBe('End simulated session')
  })
})

describe('friendlyGateReason', () => {
  it('explains the missing prerequisite instead of the raw state token', () => {
    expect(friendlyGateReason('Requires sim_validated')).toBe('Validate the simulation first.')
    expect(friendlyGateReason('Requires fault_latched')).toBe('Rearm is available only after a recoverable fault is latched.')
  })

  it('reports an open gate and falls back to readable text', () => {
    expect(friendlyGateReason(null)).toBe('Available now.')
    expect(friendlyGateReason(undefined)).toBe('Available now.')
    expect(friendlyGateReason('Requires unknown_state')).toBe('Requires unknown state')
    expect(friendlyGateReason('Runtime health reports a fault')).toBe('Runtime health reports a fault')
  })
})

describe('derivedHealth', () => {
  it('never fabricates telemetry the supervisor does not report', () => {
    const health = derivedHealth(makeStatus())
    for (const key of ['retarget', 'robot_age', 'battery', 'temperature', 'clamp']) {
      expect(health[key]).toMatchObject({ status: 'offline', value: 'NOT REPORTED' })
    }
  })

  it('hides stream readouts while the source is offline', () => {
    const status = makeStatus()
    const health = derivedHealth({ ...status, health: { ...status.health, mocap_online: false, source_age_ms: null } })
    expect(health.stream_rate).toMatchObject({ status: 'offline', value: 'NOT REPORTED', unit: undefined })
    expect(health.packet_loss.value).toBe('NOT REPORTED')
    expect(health.input_age.value).toBe('NOT REPORTED')
  })

  it('grades measured values against the acceptance limits', () => {
    const status = makeStatus()
    const health = derivedHealth({ ...status, health: { ...status.health, mocap_rate_hz: 30, packet_loss_percent: 2.5, source_age_ms: 140 } })
    expect(health.stream_rate).toMatchObject({ status: 'warning', value: '30.0', unit: 'Hz' })
    expect(health.packet_loss).toMatchObject({ status: 'fault', value: '2.50' })
    expect(health.input_age).toMatchObject({ status: 'fault', value: '140.0', unit: 'ms' })
  })

  it('flags stale robot state, active authority and a latched fault', () => {
    const status = makeStatus({ state: 'fault_latched' })
    const health = derivedHealth({ ...status, health: { ...status.health, robot_state_fresh: false } })
    expect(health.robot).toMatchObject({ status: 'warning', detail: 'Connected; state is stale' })
    expect(health.safety).toMatchObject({ status: 'warning', value: 'ACTIVE' })
    expect(health.workstation.status).toBe('fault')
  })
})

describe('faultTitle', () => {
  it('names the supervisor fault codes', () => {
    expect(faultTitle('health_stale')).toBe('Runtime health stale')
    expect(faultTitle('runtime_adapter_timeout')).toBe('Runtime adapter timed out')
    expect(faultTitle('runtime_adapter_unavailable')).toBe('Runtime adapter blocked')
    expect(faultTitle('watchdog_overdue')).toBe('Supervisor watchdog overdue')
    expect(faultTitle('some_new_code')).toBe('Some new code')
    expect(faultTitle('')).toBe('Fault')
  })
})

describe('lastUpdateText', () => {
  it('shows the time and age of the last supervisor health update', () => {
    const updatedAt = 1_760_000_000
    expect(lastUpdateText(updatedAt, (updatedAt + 12.9) * 1000)).toMatch(/^Last update \S+ \(12s ago\)\.$/)
    expect(lastUpdateText(updatedAt, (updatedAt + 125) * 1000)).toContain('(2m 5s ago)')
    expect(lastUpdateText(updatedAt, (updatedAt + 7260) * 1000)).toContain('(2h 1m ago)')
  })

  it('never shows a negative age and admits a missing timestamp', () => {
    expect(lastUpdateText(1_760_000_000, 1_759_999_990 * 1000)).toContain('(0s ago)')
    expect(lastUpdateText(0, Date.now())).toBe('Last update time not reported.')
    expect(lastUpdateText(Number.NaN, Date.now())).toBe('Last update time not reported.')
  })
})

describe('supervisor diagnostics', () => {
  const integrity = (overrides: Partial<AuditIntegrity>): AuditIntegrity => ({
    persistent: true, chain_verified: true, chain_breaks: 0, chained_records: 4, unchained_records: 0, unreadable_lines: 0, ...overrides,
  })

  it('stays quiet when everything verifies', () => {
    expect(supervisorWarnings(makeStatus())).toEqual([])
    expect(auditIntegrityText(makeStatus())).toBe('Hash chain: verified · 12 chained records.')
  })

  it('warns about a broken audit chain or a changed audit file', () => {
    const broken = makeStatus({ audit_integrity: integrity({ chain_verified: false, chain_breaks: 2 }) })
    expect(supervisorWarnings(broken)).toEqual([expect.stringContaining('2 chain breaks')])
    expect(auditIntegrityText(broken)).toBe('Hash chain: NOT VERIFIED · 2 breaks.')
    const truncated = makeStatus({ audit_integrity: integrity({ tail_matches_last_write: false }) })
    expect(supervisorWarnings(truncated)[0]).toContain('the file changed after the last write')
  })

  it('warns about a blocked adapter and the watchdog', () => {
    expect(supervisorWarnings(makeStatus({ runtime_adapter: { blocked_call: 'disengage' } }))[0]).toContain('overdue disengage call')
    expect(supervisorWarnings(makeStatus({ watchdog: { running: false, overdue: false } }))[0]).toContain('not running')
    expect(supervisorWarnings(makeStatus({ watchdog: { running: true, overdue: true } }))[0]).toContain('overdue')
  })

  it('reports missing or in-memory integrity without warning', () => {
    expect(auditIntegrityText(makeStatus({ audit_integrity: null }))).toBe('Hash chain: not reported.')
    expect(auditIntegrityText(makeStatus({ audit_integrity: integrity({ persistent: false }) }))).toContain('in-memory')
    expect(supervisorWarnings(makeStatus({ audit_integrity: undefined, watchdog: undefined, runtime_adapter: undefined }))).toEqual([])
  })
})
