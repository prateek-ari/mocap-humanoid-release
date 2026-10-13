import { Download, Moon, RefreshCw, Sun, XOctagon } from 'lucide-react'
import { faultTitle, healthTone, isReported, type Led } from './status'
import type { HealthItem, OperatorStatus } from './types'

export function Lamp({ state, label }: { state: Led; label?: string }) {
  return <span className={`lamp lamp-${state}`} role={label ? 'img' : undefined} aria-label={label} aria-hidden={label ? undefined : true} />
}

export function ThemeButton({ theme, onToggle }: { theme: 'dark' | 'light'; onToggle: () => void }) {
  return (
    <button className="panel-key icon-key" onClick={onToggle} aria-label={`Switch to ${theme === 'dark' ? 'light' : 'dark'} panel`}>
      {theme === 'dark' ? <Sun aria-hidden="true" /> : <Moon aria-hidden="true" />}
    </button>
  )
}

export function Readout({ item }: { item: HealthItem }) {
  const reported = isReported(item)
  const tone = healthTone(item)
  return (
    <div className={`readout tone-${tone} ${reported ? '' : 'is-unreported'}`} title={item.detail}>
      <span className="silk">{item.label}</span>
      {reported ? (
        <strong>{String(item.value)}{item.unit ? <small>{item.unit}</small> : null}</strong>
      ) : (
        <strong className="unreported">— —<small>not reported</small></strong>
      )}
      <span className="sr-only">{item.detail}</span>
    </div>
  )
}

export function FaultAlarm({ status, exporting, onExport }: { status: OperatorStatus; exporting: boolean; onExport: () => void }) {
  const fault = status.active_fault
  if (!fault) return null
  const occurredAt = new Date(fault.timestamp * 1000)
  const validTime = Number.isFinite(occurredAt.getTime())
  const timeLabel = validTime
    ? new Intl.DateTimeFormat(undefined, { dateStyle: 'medium', timeStyle: 'medium' }).format(occurredAt)
    : 'Time not reported'
  const authorityDisabled = status.health.command_authority === false
  return (
    <section className="fault-alarm" role="alert" aria-labelledby="active-fault-title">
      <div className="fault-head">
        <Lamp state="fault" />
        <div>
          <span className="silk">Alarm · {faultTitle(fault.code)}</span>
          <strong id="active-fault-title">Fault latched</strong>
          <p>{fault.reason || 'The supervisor did not report a fault reason.'}</p>
        </div>
      </div>
      <dl className="fault-meta">
        <div><dt>Source</dt><dd>{fault.source || 'Not reported'}</dd></div>
        <div><dt>Time</dt><dd><time dateTime={validTime ? occurredAt.toISOString() : undefined}>{timeLabel}</time></dd></div>
        <div><dt>Recovery</dt><dd>{fault.recoverable ? 'Rearm after the condition clears' : 'Manual intervention required'}</dd></div>
        <div><dt>Authority report</dt><dd className={authorityDisabled ? 'is-zero' : 'is-active'}>{authorityDisabled ? 'Supervisor reports zero software authority' : 'Supervisor still reports command authority'}</dd></div>
      </dl>
      <p className="fault-stop"><XOctagon aria-hidden="true" /><span><strong>Keep the independent physical stop engaged.</strong> Confirm the Unitree remote or damping path before recovery.</span></p>
      <button className="panel-key" disabled={exporting} onClick={onExport}>
        {exporting ? <RefreshCw className="spin" aria-hidden="true" /> : <Download aria-hidden="true" />}
        {exporting ? 'Exporting' : 'Export diagnostics'}
      </button>
    </section>
  )
}
