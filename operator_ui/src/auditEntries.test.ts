import { describe, expect, it } from 'vitest'
import { auditSequenceLabel, entryFromEvent, initialAuditEntries, mergeAuditEntry } from './auditEntries'
import type { AuditEvent } from './types'

const record = (sequence: number, timestamp: number): AuditEvent => ({
  sequence,
  timestamp,
  category: 'control',
  action: 'check',
  outcome: 'accepted',
  state: 'checking',
  actor: 'local-console',
  details: { claimed_operator: 'alice' },
})

describe('audit entries', () => {
  it('keeps durable audit numbers separate from event-stream cursors', () => {
    const entry = entryFromEvent({
      schema_version: 1,
      instance_id: 'instance-a',
      sequence: 3,
      audit_sequence: 412,
      timestamp: 100,
      type: 'control',
      action: 'check',
      actor: 'local-console',
      details: {},
    }, 'idle')

    expect(entry.auditSequence).toBe(412)
    expect(entry.eventSequence).toBe(3)
    expect(auditSequenceLabel(entry)).toBe('audit #412')
  })

  it('labels events whose audit write failed as not persisted', () => {
    const entry = entryFromEvent({ schema_version: 1, sequence: 7, audit_sequence: null, timestamp: 5, type: 'fault' }, 'tracking')
    expect(entry.auditSequence).toBeNull()
    expect(auditSequenceLabel(entry)).toBe('event #7 · not persisted')
  })

  it('deduplicates a streamed event already loaded from the audit log and orders newest first', () => {
    const loaded = initialAuditEntries([record(410, 90), record(411, 95)])
    expect(loaded.map((entry) => entry.auditSequence)).toEqual([411, 410])
    expect(loaded[0].claimedOperator).toBe('alice')

    const streamed = entryFromEvent({ schema_version: 1, sequence: 2, audit_sequence: 411, timestamp: 95, type: 'control', action: 'check' }, 'idle')
    const merged = mergeAuditEntry(loaded, streamed)
    expect(merged.map((entry) => entry.auditSequence)).toEqual([411, 410])

    const newer = entryFromEvent({ schema_version: 1, sequence: 3, audit_sequence: 412, timestamp: 99, type: 'control', action: 'arm' }, 'idle')
    expect(mergeAuditEntry(merged, newer).map((entry) => entry.auditSequence)).toEqual([412, 411, 410])
  })
})
