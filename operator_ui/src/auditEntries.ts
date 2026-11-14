import type { AuditEvent, OperatorEvent, SessionState } from './types'

/**
 * One row in the audit panel.  ``auditSequence`` is the durable AuditLog
 * number, which continues across service restarts; ``eventSequence`` is the
 * in-memory event-stream cursor, which restarts with each service instance.
 * The two are never compared or used interchangeably.
 */
export interface AuditEntry {
  key: string
  auditSequence: number | null
  eventSequence: number | null
  timestamp: number
  category: string
  action: string
  outcome: string
  /** A `SessionState` for current records; older records may carry other strings. */
  state: string
  actor: string
  claimedOperator: string | null
  details: Record<string, unknown>
}

export const AUDIT_PANEL_LIMIT = 100

function claimedOperator(details: Record<string, unknown>): string | null {
  return typeof details.claimed_operator === 'string' ? details.claimed_operator : null
}

export function entryFromRecord(record: AuditEvent): AuditEntry {
  const details = record.details ?? {}
  return {
    key: `audit-${record.sequence}`,
    auditSequence: record.sequence,
    eventSequence: null,
    timestamp: record.timestamp,
    category: record.category,
    action: record.action,
    outcome: record.outcome,
    state: record.state,
    actor: record.actor,
    claimedOperator: claimedOperator(details),
    details,
  }
}

export function entryFromEvent(event: OperatorEvent, fallbackState: SessionState): AuditEntry {
  const details = event.details ?? {}
  const auditSequence = typeof event.audit_sequence === 'number' && Number.isFinite(event.audit_sequence) ? event.audit_sequence : null
  return {
    key: auditSequence !== null ? `audit-${auditSequence}` : `event-${event.instance_id ?? 'unknown'}-${event.sequence}`,
    auditSequence,
    eventSequence: event.sequence,
    timestamp: event.timestamp ?? Date.now() / 1000,
    category: event.type,
    action: event.action ?? event.type,
    outcome: event.outcome ?? 'accepted',
    state: event.state ?? fallbackState,
    actor: event.actor ?? 'event-stream',
    claimedOperator: claimedOperator(details),
    details,
  }
}

/** Newest first by supervisor timestamp; durable audit number breaks ties. */
function compareEntries(a: AuditEntry, b: AuditEntry): number {
  if (b.timestamp !== a.timestamp) return b.timestamp - a.timestamp
  return (b.auditSequence ?? Number.MAX_SAFE_INTEGER) - (a.auditSequence ?? Number.MAX_SAFE_INTEGER)
}

export function initialAuditEntries(records: AuditEvent[], limit = AUDIT_PANEL_LIMIT): AuditEntry[] {
  return records.map(entryFromRecord).sort(compareEntries).slice(0, limit)
}

/** Add one entry, replacing any row with the same durable audit number. */
export function mergeAuditEntry(current: AuditEntry[], entry: AuditEntry, limit = AUDIT_PANEL_LIMIT): AuditEntry[] {
  const remaining = current.filter((row) => row.key !== entry.key)
  return [entry, ...remaining].sort(compareEntries).slice(0, limit)
}

export function auditSequenceLabel(entry: AuditEntry): string {
  if (entry.auditSequence !== null) return `audit #${entry.auditSequence}`
  return entry.eventSequence !== null ? `event #${entry.eventSequence} · not persisted` : 'not persisted'
}
