import { AlertTriangle, ChevronDown, Download, RefreshCw, Save } from 'lucide-react'
import { auditSequenceLabel, type AuditEntry } from './auditEntries'
import { Lamp } from './components'
import { auditIntegrityText, humanizeToken, isReadOnlyPresentation, isRecordedSimulation, type Led } from './status'
import type { OperatorConfig, OperatorStatus, QualificationRun } from './types'

export interface EngineeringViewProps {
  status: OperatorStatus
  config: OperatorConfig
  audit: AuditEntry[]
  qualificationRuns: QualificationRun[] | null
  qualificationLoading: boolean
  qualificationError: string | null
  demo: boolean
  exporting: boolean
  saving: boolean
  onConfig: (next: OperatorConfig) => void
  onSave: () => void
  onExport: () => void
  onRetryQualifications: () => void
}

function qualificationTime(value: string | number): string {
  const date = typeof value === 'number'
    ? new Date(value > 10_000_000_000 ? value : value * 1000)
    : new Date(value)
  if (!Number.isFinite(date.getTime())) return String(value)
  return new Intl.DateTimeFormat(undefined, { dateStyle: 'medium', timeStyle: 'short' }).format(date)
}

function qualificationEvidence(value: unknown): string | null {
  if (typeof value === 'string' || typeof value === 'number' || typeof value === 'boolean') return String(value)
  if (!value || typeof value !== 'object' || Array.isArray(value)) return null
  const preferred = ['verdict', 'status', 'score', 'value', 'reason']
  const entries = Object.entries(value as Record<string, unknown>)
  const selected = preferred
    .map((key) => entries.find(([entryKey]) => entryKey === key))
    .filter((entry): entry is [string, unknown] => Boolean(entry))
  const usable = (selected.length ? selected : entries)
    .filter(([, entryValue]) => ['string', 'number', 'boolean'].includes(typeof entryValue))
    .slice(0, 2)
  return usable.length ? usable.map(([key, entryValue]) => `${humanizeToken(key)}: ${String(entryValue)}`).join(' / ') : null
}

function qualificationTone(value: string): string {
  const normalized = value.toLowerCase()
  if (normalized.includes('pass') || normalized.includes('accept') || normalized.includes('complete')) return 'passed'
  if (normalized.includes('fail') || normalized.includes('reject') || normalized.includes('error')) return 'failed'
  if (normalized.includes('run') || normalized.includes('progress')) return 'running'
  return 'neutral'
}

function stageLed(status: string): Led {
  const tone = qualificationTone(status)
  return tone === 'passed' ? 'passed' : tone === 'failed' ? 'fault' : tone === 'running' ? 'next' : 'off'
}

function QualificationRunsPanel({ runs, loading, error, onRetry }: { runs: QualificationRun[] | null; loading: boolean; error: string | null; onRetry: () => void }) {
  return (
    <fieldset className="group qualification">
      <legend className="silk">Qualification runs</legend>
      <p className="group-note">Latest simulator and reference acceptance evidence.</p>
      {loading ? (
        <div className="empty-state" role="status"><RefreshCw className="spin" aria-hidden="true" /><span>Loading qualification history</span></div>
      ) : runs === null ? (
        <div className="empty-state is-fault" role="status">
          <AlertTriangle aria-hidden="true" />
          <strong>Qualification history unavailable</strong>
          <span>{error || 'This supervisor does not expose qualification runs.'}</span>
          <button className="panel-key" onClick={onRetry}><RefreshCw aria-hidden="true" />Retry</button>
        </div>
      ) : runs.length === 0 ? (
        <div className="empty-state"><strong>No qualification runs reported</strong><span>The supervisor returned an empty run list.</span></div>
      ) : (
        <div className="runs">
          {runs.slice(0, 3).map((run, index) => {
            const reference = qualificationEvidence(run.reference_quality)
            const execution = qualificationEvidence(run.simulator_execution)
            return (
              <details key={run.id} open={index === 0}>
                <summary>
                  <span className="run-name"><strong>{run.name}</strong><small className="mono">{qualificationTime(run.created_at)}</small></span>
                  <b className={`verdict verdict-${qualificationTone(run.verdict)}`}>{humanizeToken(run.verdict)}</b>
                  <ChevronDown aria-hidden="true" />
                </summary>
                <div className="run-body">
                  {(reference || execution) && <dl className="run-metrics">{reference && <div><dt>Reference quality</dt><dd>{reference}</dd></div>}{execution && <div><dt>Simulator execution</dt><dd>{execution}</dd></div>}</dl>}
                  {run.stages.length ? (
                    <ol className="stages">{run.stages.map((stage, stageIndex) => (
                      <li key={`${stage.name}-${stageIndex}`}>
                        <Lamp state={stageLed(stage.status)} label={stage.status} />
                        <span><strong>{stage.name}</strong>{stage.reason && <small>{stage.reason}</small>}</span>
                        <b className="mono">{humanizeToken(stage.status)}{typeof stage.margin === 'number' ? ` · margin ${stage.margin}` : ''}{typeof stage.worst_time_s === 'number' ? ` · ${stage.worst_time_s.toFixed(3)} s` : ''}</b>
                      </li>
                    ))}</ol>
                  ) : <p className="group-note">No stage detail was reported for this run.</p>}
                </div>
              </details>
            )
          })}
        </div>
      )}
    </fieldset>
  )
}

function auditTime(timestamp: number): { label: string; iso: string; title: string } {
  const date = new Date(timestamp * 1000)
  if (!Number.isFinite(date.getTime())) return { label: '—', iso: '', title: 'Time not reported' }
  return {
    label: date.toLocaleTimeString([], { hour12: false }),
    iso: date.toISOString(),
    title: new Intl.DateTimeFormat(undefined, { dateStyle: 'medium', timeStyle: 'long' }).format(date),
  }
}

export function EngineeringView({ status, config, audit, qualificationRuns, qualificationLoading, qualificationError, demo, exporting, saving, onConfig, onSave, onExport, onRetryQualifications }: EngineeringViewProps) {
  const recordedSimulation = isRecordedSimulation(status)
  const settingsLocked = recordedSimulation || !status.controls['edit-config']?.enabled
  const playbackReadOnly = isReadOnlyPresentation(status)
  const locked = !status.capability_lock.whole_body_unlocked
  const auditDetail = (event: AuditEntry) => {
    const reason = event.details.reason
    if (typeof reason === 'string') return reason
    const keys = Object.keys(event.details)
    return keys.length ? `${event.outcome}; ${keys.join(', ')}` : event.outcome
  }

  return (
    <main className="engineering-workspace">
      <header className="engineering-header">
        <div>
          <h1>Engineering</h1>
          <p>Topology, safety envelope, controller selection, capability evidence, and audit history.</p>
        </div>
        <div className="engineering-actions">
          {settingsLocked && <span className="lock-note"><Lamp state="fault" />{recordedSimulation ? 'Recorded source and config are read-only' : playbackReadOnly ? 'Recorded review is read-only' : 'Editing locked after checks begin'}</span>}
          <button className="panel-key" disabled={exporting} onClick={onExport}>
            {exporting ? <RefreshCw className="spin" aria-hidden="true" /> : <Download aria-hidden="true" />}
            {exporting ? 'Exporting' : 'Export diagnostics'}
          </button>
          <button className="panel-key is-primary" disabled={settingsLocked || saving} onClick={onSave}>
            {saving ? <RefreshCw className="spin" aria-hidden="true" /> : <Save aria-hidden="true" />}
            {saving ? 'Saving' : 'Save configuration'}
          </button>
        </div>
      </header>

      <div className="engineering-columns">
        <div className="engineering-settings">
          <fieldset className="group" disabled={settingsLocked}>
            <legend className="silk">Topology and network</legend>
            <div className="topology" role="radiogroup" aria-label="Deployment topology">
              <label className={config.topology === 'all_in_one' ? 'selected' : ''}>
                <input type="radio" name="topology" value="all_in_one" checked={config.topology === 'all_in_one'} onChange={() => onConfig({ ...config, topology: 'all_in_one' })} />
                <span><strong>All-in-one Ubuntu</strong><small>Xsens, control runtime, and GUI on this workstation</small></span>
              </label>
              <label className={config.topology === 'split' ? 'selected' : ''}>
                <input type="radio" name="topology" value="split" checked={config.topology === 'split'} onChange={() => onConfig({ ...config, topology: 'split' })} />
                <span><strong>Windows capture + Ubuntu control</strong><small>Paired Xsens source on a trusted LAN</small></span>
              </label>
            </div>
            <div className="form-grid">
              <label>Xsens source IPv4<input className="mono" value={config.mocap.source_ip ?? ''} placeholder={config.topology === 'split' ? 'Required for split topology' : 'Any accepted source'} onChange={(event) => onConfig({ ...config, mocap: { ...config.mocap, source_ip: event.target.value.trim() || null } })} /></label>
              <label>UDP port<input className="mono" type="number" value={config.mocap.port} onChange={(event) => onConfig({ ...config, mocap: { ...config.mocap, port: Number(event.target.value) } })} /></label>
              <label>Listen address<input className="mono" value={config.mocap.listen_address} onChange={(event) => onConfig({ ...config, mocap: { ...config.mocap, listen_address: event.target.value } })} /></label>
              <label>Unitree interface<input className="mono" value={config.robot.interface ?? ''} placeholder="Not configured" onChange={(event) => onConfig({ ...config, robot: { interface: event.target.value.trim() || null } })} /></label>
            </div>
            <ol className="route" aria-label="Configured data flow">
              <li>Xsens source<small className="mono">{config.mocap.source_ip ?? 'any'} → :{config.mocap.port}</small></li>
              <li>Ingest + control<small className="mono">{config.mocap.listen_address}</small></li>
              <li>G1 DDS<small className="mono">{config.robot.interface ?? 'not configured'}</small></li>
            </ol>
          </fieldset>

          <fieldset className="group">
            <legend className="silk">Safety envelope</legend>
            <dl className="spec-table">
              <div><dt>Stale input watchdog</dt><dd className="mono">100 ms baseline</dd><small>Runtime-owned; not reported by v1 API</small></div>
              <div><dt>Hard timeout</dt><dd className="mono">500 ms baseline</dd><small>Runtime-owned; not reported by v1 API</small></div>
              <div><dt>Motion scale / action clamp</dt><dd className="unreported">Not reported</dd><small>No configurable v1 interface</small></div>
              <div><dt>Gains and calibration</dt><dd className="unreported">Not reported</dd><small>No configurable v1 interface</small></div>
            </dl>
            <p className="group-note">The console does not invent or persist controller values that the safety supervisor cannot validate.</p>
          </fieldset>

          <fieldset className="group" disabled={settingsLocked}>
            <legend className="silk">Controller and recording</legend>
            <dl className="spec-table">
              <div><dt>Runtime mode</dt><dd>{config.runtime_mode === 'fake' ? 'Backend demo' : 'Real / passive'}</dd><small>Set by the service launcher</small></div>
              <div><dt>Whole-body policy and filter</dt><dd className="unreported">Not reported</dd><small>Capability stays locked until the backend exposes verified controller identity</small></div>
            </dl>
            <label className="consent"><input type="checkbox" checked={config.privacy.record_raw_mocap} onChange={(event) => onConfig({ ...config, privacy: { record_raw_mocap: event.target.checked } })} /><span><strong>Record raw mocap for this session</strong><small>Body trajectories are opt-in and reset off on End session or console restart. Safety telemetry and operator actions are always retained.</small></span></label>
          </fieldset>
        </div>

        <aside className="engineering-evidence">
          <fieldset className={`group authority ${locked ? 'is-locked' : 'is-verified'}`}>
            <legend className="silk">Capability manifest</legend>
            <div className="authority-line">
              <Lamp state={locked ? 'fault' : 'passed'} />
              <strong>{locked ? 'Hardware locked' : 'Capability verified'}</strong>
            </div>
            <ul className="chain">
              <li><Lamp state={status.capability_lock.manifest_verified ? 'passed' : 'off'} /><span>Manifest signature</span><strong>{status.capability_lock.manifest_verified ? 'VERIFIED' : 'NONE'}</strong></li>
              <li><Lamp state={status.capability_lock.whole_body_unlocked ? 'passed' : 'off'} /><span>Whole-body hardware</span><strong>{status.capability_lock.whole_body_unlocked ? 'UNLOCKED' : 'LOCKED'}</strong></li>
              <li><Lamp state="off" /><span>Declared capabilities</span><strong>{status.capability_lock.capabilities.length ? status.capability_lock.capabilities.join(', ') : 'NONE'}</strong></li>
              <li><Lamp state="off" /><span>Manifest expiry</span><strong>{status.capability_lock.expires_at ?? 'NOT REPORTED'}</strong></li>
            </ul>
            <code className="manifest-id">{status.capability_lock.manifest_id ?? 'manifest_id: none'}</code>
            <p>{status.capability_lock.reason}</p>
          </fieldset>

          <QualificationRunsPanel runs={qualificationRuns} loading={qualificationLoading} error={qualificationError} onRetry={onRetryQualifications} />

          <fieldset className="group audit">
            <legend className="silk">Audit events</legend>
            <p className="group-note">{demo ? 'Backend demo history' : 'Supervisor event history'} · newest first</p>
            <p className="group-note mono">{auditIntegrityText(status)}</p>
            <div className="audit-list" role="log" aria-live="polite">
              {audit.length === 0 ? (
                <div className="empty-state"><span>No recorded events yet.</span></div>
              ) : audit.map((event) => {
                const time = auditTime(event.timestamp)
                return (
                  <article key={event.key} className={`audit-event ${event.outcome === 'rejected' ? 'is-rejected' : ''}`}>
                    <time className="mono" dateTime={time.iso || undefined} title={time.title}>{time.label}</time>
                    <div><strong>{event.category} / {event.action}</strong><p>{auditDetail(event)}</p><span className="mono">{event.actor}{event.claimedOperator ? ` (claimed: ${event.claimedOperator})` : ''} · {event.state} · {auditSequenceLabel(event)}</span></div>
                  </article>
                )
              })}
            </div>
          </fieldset>
        </aside>
      </div>
    </main>
  )
}
