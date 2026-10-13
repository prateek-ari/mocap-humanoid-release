import { describe, expect, it } from 'vitest'
import { canSubmit, gateFor, isNewerStatus, keyLed, keyUnavailable } from './gating'
import { makeStatus } from './test/fixtures'
import type { ActionName } from './types'

const enabled = { enabled: true, reason: null }
const pendingOf = (...actions: ActionName[]) => new Set<ActionName>(actions)

describe('safety keys are never blocked by another pending action', () => {
  it('keeps DISENGAGE available while a routine action is in flight', () => {
    for (const routine of ['start-tracking', 'arm', 'check', 'rearm'] as ActionName[]) {
      expect(canSubmit('disengage', pendingOf(routine))).toBe(true)
      expect(keyUnavailable('disengage', enabled, pendingOf(routine))).toBe(false)
    }
  })

  it('keeps End session available while a routine action or DISENGAGE is in flight', () => {
    expect(keyUnavailable('shutdown', enabled, pendingOf('start-tracking'))).toBe(false)
    expect(keyUnavailable('shutdown', enabled, pendingOf('disengage'))).toBe(false)
    expect(keyUnavailable('disengage', enabled, pendingOf('shutdown'))).toBe(false)
  })

  it('guards each safety key against its own double submit', () => {
    expect(canSubmit('disengage', pendingOf('disengage'))).toBe(false)
    expect(canSubmit('shutdown', pendingOf('shutdown'))).toBe(false)
  })

  it('serializes routine actions behind any pending request, including a stop', () => {
    expect(canSubmit('start-tracking', pendingOf())).toBe(true)
    expect(canSubmit('start-tracking', pendingOf('disengage'))).toBe(false)
    expect(canSubmit('arm', pendingOf('check'))).toBe(false)
    expect(keyUnavailable('arm', enabled, pendingOf('shutdown'))).toBe(true)
  })

  it('still honours a disabled supervisor gate', () => {
    expect(keyUnavailable('disengage', { enabled: false, reason: 'no' }, pendingOf())).toBe(true)
  })
})

describe('gateFor', () => {
  it('keeps stop and End session available when the event stream is lost', () => {
    const status = makeStatus()
    const inputs = { status, wsConnected: false, preflightComplete: true }
    expect(gateFor('disengage', inputs).enabled).toBe(true)
    expect(gateFor('shutdown', inputs).enabled).toBe(true)
    expect(gateFor('start-tracking', inputs).enabled).toBe(false)
  })

  it('requires local crew evidence before confirming preflight', () => {
    const status = makeStatus({ state: 'robot_connected' })
    expect(gateFor('confirm-preflight', { status, wsConnected: true, preflightComplete: false }).enabled).toBe(false)
    expect(gateFor('confirm-preflight', { status, wsConnected: true, preflightComplete: true }).enabled).toBe(true)
  })

  it('disables every control in recorded playback', () => {
    const status = makeStatus({ presentation_mode: 'recorded_playback' })
    expect(gateFor('disengage', { status, wsConnected: true, preflightComplete: true }).enabled).toBe(false)
  })
})

describe('isNewerStatus', () => {
  it('rejects a stale response that arrives after a newer status', () => {
    const disengaged = makeStatus({ state: 'armed_disabled', last_event_sequence: 14 })
    const lateTracking = makeStatus({ state: 'tracking', last_event_sequence: 12 })
    expect(isNewerStatus(disengaged, lateTracking)).toBe(false)
    expect(isNewerStatus(lateTracking, disengaged)).toBe(true)
    expect(isNewerStatus(disengaged, { ...disengaged })).toBe(true)
  })

  it('accepts any status from a new supervisor instance', () => {
    const old = makeStatus({ last_event_sequence: 400 })
    const restarted = makeStatus({ instance_id: 'instance-b', last_event_sequence: 2 })
    expect(isNewerStatus(old, restarted)).toBe(true)
    expect(isNewerStatus(null, restarted)).toBe(true)
  })
})

describe('keyLed', () => {
  it('lights the available key as next', () => {
    expect(keyLed('arm', true, makeStatus({ state: 'preflight_passed' }))).toBe('next')
  })

  it('marks gates already passed and leaves later ones off', () => {
    const status = makeStatus({ state: 'robot_connected' })
    expect(keyLed('check', false, status)).toBe('passed')
    expect(keyLed('connect-robot', false, status)).toBe('passed')
    expect(keyLed('confirm-preflight', false, status)).toBe('off')
    expect(keyLed('start-tracking', false, status)).toBe('off')
  })

  it('shows tracking as active while tracking', () => {
    expect(keyLed('start-tracking', false, makeStatus({ state: 'tracking' }))).toBe('active')
  })

  it('turns every unavailable key off in a fault', () => {
    const latched = makeStatus({ state: 'fault_latched' })
    expect(keyLed('check', false, latched)).toBe('off')
    const fault = { code: 'health_stale', reason: 'stale', source: 'controller', timestamp: 1, latched: true, recoverable: true }
    expect(keyLed('arm', false, makeStatus({ state: 'tracking', active_fault: fault }))).toBe('off')
    expect(keyLed('rearm', true, latched)).toBe('next')
  })

  it('never lights keys that do not reach a state', () => {
    expect(keyLed('rearm', false, makeStatus({ state: 'tracking' }))).toBe('off')
    expect(keyLed('disengage', false, makeStatus({ state: 'tracking' }))).toBe('off')
  })
})
