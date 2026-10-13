import { useCallback, useEffect, useEffectEvent, useMemo, useRef, useState } from 'react'
import { AlertTriangle, RefreshCw, X } from 'lucide-react'
import { ActionDock, KeyBank } from './ActionDock'
import { OperatorClient } from './api'
import { entryFromEvent, initialAuditEntries, mergeAuditEntry, type AuditEntry } from './auditEntries'
import { FaultAlarm, Lamp, ThemeButton } from './components'
import { EngineeringView } from './EngineeringView'
import { canSubmit, gateFor as computeGate, isNewerStatus, keyUnavailable } from './gating'
import { isDisengageShortcut } from './shortcuts'
import { SessionView } from './SessionView'
import { SetupClient, type SetupState } from './setupApi'
import { SetupView, parseSetupHash } from './SetupView'
import {
  actionCopy,
  isDemoStatus,
  isReadOnlyPresentation,
  isRecordedSimulation,
  lastUpdateText,
  supervisorWarnings,
  type Led,
} from './status'
import type {
  ActionName,
  ControlState,
  OperatorConfig,
  OperatorEvent,
  OperatorStatus,
  PlaybackManifest,
  QualificationRun,
} from './types'

type Workspace = 'setup' | 'session' | 'engineering'

const defaultClient = new OperatorClient()
const defaultSetupClient = new SetupClient()
const REBOOTSTRAP_RETRY_MS = 2000

export function App({ client = defaultClient, setupClient = defaultSetupClient }: { client?: OperatorClient; setupClient?: SetupClient } = {}) {
  const [workspace, setWorkspace] = useState<Workspace>('session')
  // 'pending' until GET /setup settles; null when the workbench is off (404)
  // or unreachable, which keeps the Setup tab hidden.
  const [setupProbe, setSetupProbe] = useState<SetupState | null | 'pending'>('pending')
  const [theme, setTheme] = useState<'dark' | 'light'>(() => {
    try {
      const saved = localStorage.getItem('mocap-g1-theme')
      if (saved === 'dark' || saved === 'light') return saved
    } catch {
      // Storage may be unavailable; the light panel is the default.
    }
    return 'light'
  })
  const [status, setStatusState] = useState<OperatorStatus | null>(null)
  const [config, setConfig] = useState<OperatorConfig | null>(null)
  const [audit, setAudit] = useState<AuditEntry[]>([])
  const [playback, setPlayback] = useState<PlaybackManifest | null>(null)
  const [qualificationRuns, setQualificationRuns] = useState<QualificationRun[] | null>(null)
  const [qualificationLoading, setQualificationLoading] = useState(true)
  const [qualificationError, setQualificationError] = useState<string | null>(null)
  const [qualificationGeneration, setQualificationGeneration] = useState(0)
  const [wsConnected, setWsConnected] = useState(false)
  const [bootGeneration, setBootGeneration] = useState(0)
  const pendingRef = useRef<Set<ActionName>>(new Set())
  const [pending, setPending] = useState<ReadonlySet<ActionName>>(() => new Set())
  const [saving, setSaving] = useState(false)
  const [exporting, setExporting] = useState(false)
  const [notice, setNotice] = useState<string | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [initialError, setInitialError] = useState<string | null>(null)
  const [clock, setClock] = useState(new Date())
  const [operatorName, setOperatorName] = useState('')
  const [spotterName, setSpotterName] = useState('')
  const [stopChecked, setStopChecked] = useState(false)
  const [areaChecked, setAreaChecked] = useState(false)
  const [harnessChecked, setHarnessChecked] = useState(false)
  const [inspect, setInspect] = useState<string | null>(null)

  // Concurrent requests (for example DISENGAGE while a routine action is
  // pending) can complete out of order; never display an older status.
  const setStatus = useCallback((next: OperatorStatus) => {
    setStatusState((current) => isNewerStatus(current, next) ? next : current)
  }, [])

  const clearCrewEvidence = useCallback(() => {
    setOperatorName('')
    setSpotterName('')
    setStopChecked(false)
    setAreaChecked(false)
    setHarnessChecked(false)
  }, [])

  useEffect(() => {
    document.documentElement.dataset.theme = theme
    try {
      localStorage.setItem('mocap-g1-theme', theme)
    } catch {
      // Theme persistence is a convenience only.
    }
  }, [theme])

  useEffect(() => {
    let active = true
    let unsubscribe: () => void = () => {}
    let retryTimer: ReturnType<typeof setTimeout> | null = null
    const rebootstrap = bootGeneration > 0
    // Raw-mocap consent is reset by the supervisor at startup and on End
    // session. Loading the console is read-only: it never writes config.
    client.bootstrap().then((data) => {
      if (!active) return
      setStatusState(data.status)
      setConfig(data.config)
      setAudit(initialAuditEntries(data.audit))
      setPlayback(data.playback)
      unsubscribe = client.subscribe((event: OperatorEvent) => {
        if (event.status) setStatus(event.status)
        if (event.type !== 'snapshot') {
          setAudit((current) => mergeAuditEntry(current, entryFromEvent(event, data.status.state)))
        }
      }, {
        onConnection: setWsConnected,
        afterSequence: data.status.last_event_sequence,
        instanceId: data.status.instance_id ?? null,
        onReset: () => {
          if (!active) return
          // The supervisor restarted: its session, sequence counter, and
          // preflight evidence are gone. Reload instead of resuming.
          clearCrewEvidence()
          setNotice('Supervisor restarted. Console state was reloaded; crew preflight evidence was cleared.')
          setBootGeneration((generation) => generation + 1)
        },
      })
    }).catch((reason: unknown) => {
      if (!active) return
      if (rebootstrap) {
        setWsConnected(false)
        retryTimer = setTimeout(() => setBootGeneration((generation) => generation + 1), REBOOTSTRAP_RETRY_MS)
        return
      }
      setInitialError(reason instanceof Error ? reason.message : 'Unable to initialize operator console.')
    })
    return () => {
      active = false
      if (retryTimer !== null) clearTimeout(retryTimer)
      unsubscribe()
    }
  }, [client, bootGeneration, setStatus, clearCrewEvidence])

  useEffect(() => {
    let active = true
    setupClient.getSetup().then((setup) => {
      if (!active) return
      setSetupProbe(setup)
      // The Setup workbench is the default workspace when it is enabled.
      if (setup) setWorkspace('setup')
    }, () => {
      if (active) setSetupProbe(null)
    })
    return () => {
      active = false
    }
  }, [setupClient])

  const setupAvailable = setupProbe !== null && setupProbe !== 'pending'

  useEffect(() => {
    if (!setupAvailable) return
    const onHash = () => {
      if (parseSetupHash(window.location.hash)) setWorkspace('setup')
    }
    window.addEventListener('hashchange', onHash)
    return () => window.removeEventListener('hashchange', onHash)
  }, [setupAvailable])

  function selectWorkspace(next: Workspace) {
    setWorkspace(next)
    if (next !== 'setup' && parseSetupHash(window.location.hash)) {
      window.history.replaceState(null, '', `${window.location.pathname}${window.location.search}`)
    }
  }

  // Each request generation loads the qualification history once; a retry
  // bumps the generation.  State is only set once the request settles.
  useEffect(() => {
    let active = true
    client.getQualificationRuns().then((runs) => {
      if (!active) return
      setQualificationRuns(runs)
      setQualificationError(null)
    }, (reason: unknown) => {
      if (!active) return
      setQualificationRuns(null)
      setQualificationError(reason instanceof Error ? reason.message : 'Qualification history could not be loaded.')
    }).finally(() => {
      if (active) setQualificationLoading(false)
    })
    return () => {
      active = false
    }
  }, [client, qualificationGeneration])

  function retryQualificationRuns() {
    setQualificationLoading(true)
    setQualificationError(null)
    setQualificationGeneration((generation) => generation + 1)
  }

  useEffect(() => {
    const timer = window.setInterval(() => setClock(new Date()), 1000)
    return () => window.clearInterval(timer)
  }, [])

  useEffect(() => {
    if (!notice) return
    const timer = window.setTimeout(() => setNotice(null), 4500)
    return () => window.clearTimeout(timer)
  }, [notice])

  const preflightComplete = Boolean(operatorName.trim() && spotterName.trim() && stopChecked && areaChecked && harnessChecked)

  const gateFor = useMemo(() => (name: ActionName): ControlState => (
    computeGate(name, { status, wsConnected, preflightComplete })
  ), [status, wsConnected, preflightComplete])

  async function handleAction(action: ActionName) {
    // The ref is updated synchronously so a double click cannot submit twice
    // before React re-renders the disabled key.
    if (!canSubmit(action, pendingRef.current)) return
    pendingRef.current.add(action)
    setPending(new Set(pendingRef.current))
    setError(null)
    try {
      const operator = operatorName.trim()
      const payload = action === 'confirm-preflight'
        ? { operator, spotter: spotterName.trim(), metadata: { clear_area: true as const, harness_or_support: true as const, stop_drill: true as const } }
        : operator ? { operator } : {}
      const next = await client.action(action, payload)
      setStatus(next)
      if (action === 'shutdown') {
        clearCrewEvidence()
        // The supervisor resets raw-mocap consent on End session; re-read it
        // rather than issuing another configuration write.
        let recordingOff = true
        try {
          const latest = await client.getConfig()
          setConfig(latest)
          recordingOff = !latest.privacy.record_raw_mocap
        } catch (reason) {
          setError(reason instanceof Error ? `Session ended, but recording reset was not confirmed: ${reason.message}` : 'Session ended, but recording reset was not confirmed.')
          return
        }
        if (recordingOff) {
          setNotice('Session ended. Crew and preflight evidence cleared; raw mocap recording reset off.')
        } else {
          setError('Session ended, but the supervisor still reports raw mocap recording enabled.')
        }
      } else {
        setNotice(action === 'disengage'
          ? isDemoStatus(next) || isRecordedSimulation(next)
            ? 'Simulation stopped. Simulated command authority is disabled.'
            : 'Disengage accepted. Confirm the supervisor reports zero software authority.'
          : `${actionCopy(action, next).label} accepted.`)
      }
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : 'The supervisor rejected this action.')
    } finally {
      pendingRef.current.delete(action)
      setPending(new Set(pendingRef.current))
    }
  }

  async function saveConfiguration() {
    if (!config) return
    setSaving(true)
    setError(null)
    try {
      const next = await client.saveConfig(config, operatorName.trim())
      setConfig(next)
      setNotice(next.runtime_mode === 'fake' ? 'Configuration saved to the backend demo runtime.' : 'Configuration saved and audited.')
      // Never patch status locally: take the supervisor's own status through
      // the same ordering guard as every other update.  If this read fails,
      // the event stream's next snapshot carries the saved topology.
      try {
        setStatus(await client.getStatus())
      } catch {
        // Covered by the event stream.
      }
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : 'Configuration was not saved.')
    } finally {
      setSaving(false)
    }
  }

  async function exportDiagnostics() {
    setExporting(true)
    setError(null)
    try {
      await client.downloadDiagnostics()
      setNotice('Diagnostics export downloaded.')
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : 'Diagnostics export failed.')
    } finally {
      setExporting(false)
    }
  }

  // DISENGAGE shortcut: identical gating to the on-screen key, never from a
  // text field (see shortcuts.ts).
  const onDisengageShortcut = useEffectEvent(() => {
    if (keyUnavailable('disengage', gateFor('disengage'), pendingRef.current)) return
    void handleAction('disengage')
  })

  useEffect(() => {
    const onKeyDown = (event: KeyboardEvent) => {
      if (!isDisengageShortcut(event)) return
      event.preventDefault()
      onDisengageShortcut()
    }
    window.addEventListener('keydown', onKeyDown)
    return () => window.removeEventListener('keydown', onKeyDown)
  }, [])

  if (!status || !config || setupProbe === 'pending') {
    if (initialError) {
      return (
        <div className="boot-screen" role="alert">
          <Lamp state="fault" />
          <strong>Safety supervisor unavailable</strong>
          <p>{initialError}</p>
          <span>No readiness evidence is available. Robot actions are not exposed.</span>
          <button className="panel-key is-primary" onClick={() => window.location.reload()}><RefreshCw aria-hidden="true" />Retry connection</button>
        </div>
      )
    }
    return (
      <div className="boot-screen" aria-live="polite">
        <Lamp state="next" />
        <strong>Starting operator console</strong>
        <span>Contacting the safety supervisor</span>
        <div className="boot-track"><i /></div>
      </div>
    )
  }

  const recordedSimulation = isRecordedSimulation(status)
  const playbackMode = status.presentation_mode === 'recorded_playback'
  const readOnlyPresentation = isReadOnlyPresentation(status)
  const demo = isDemoStatus(status) && !recordedSimulation
  const sourceLed: Led = recordedSimulation || readOnlyPresentation || demo ? 'next' : wsConnected ? 'passed' : 'fault'
  const runtimeCompact = recordedSimulation ? 'RECORDED SIM' : playbackMode ? 'RECORDED REVIEW' : demo ? 'BACKEND DEMO' : wsConnected ? 'SUPERVISOR LIVE' : 'EVENT STREAM LOST'
  const runtimeMeta = recordedSimulation
    ? `${status.simulation?.source?.name?.trim() || 'Recorded input'} · interactive simulation · hardware locked`
    : playbackMode
    ? 'Passive playback · 0% authority'
    : demo
    ? 'Synthetic hardware · explicit demo mode'
    : `Event #${status.last_event_sequence}`
  const hardwareLocked = recordedSimulation || !status.capability_lock.whole_body_unlocked
  const stale = !wsConnected && !readOnlyPresentation
  const warnings = supervisorWarnings(status)

  const keyBankFor = (idPrefix: string) => (
    <KeyBank status={status} pending={pending} gateFor={gateFor} onAction={handleAction} onInspect={setInspect} idPrefix={idPrefix} />
  )

  return (
    <div className="app-shell">
      <header className="app-header">
        <div className="brand">
          <strong>MOCAP / G1</strong>
          <span className="silk">Operator console · G1 EDU 29-DOF</span>
        </div>
        <nav className="workspace-switch" aria-label="Workspace">
          {setupAvailable && (
            <button className={workspace === 'setup' ? 'active' : ''} aria-current={workspace === 'setup' ? 'page' : undefined} onClick={() => selectWorkspace('setup')}>Setup</button>
          )}
          <button className={workspace === 'session' ? 'active' : ''} aria-current={workspace === 'session' ? 'page' : undefined} onClick={() => selectWorkspace('session')}>Session</button>
          <button className={workspace === 'engineering' ? 'active' : ''} aria-current={workspace === 'engineering' ? 'page' : undefined} onClick={() => selectWorkspace('engineering')}>Engineering</button>
        </nav>
        <div className="header-annunciators">
          <span className={`annunciator-chip ${sourceLed === 'fault' ? 'is-fault' : ''}`} title={runtimeMeta}>
            <Lamp state={sourceLed} />
            <span><strong>{runtimeCompact}</strong><small>{runtimeMeta}</small></span>
          </span>
          <span className={`annunciator-chip ${hardwareLocked ? 'is-fault' : 'is-ok'}`}>
            <Lamp state={hardwareLocked ? 'fault' : 'passed'} />
            <strong>{hardwareLocked ? 'HW AUTHORITY · LOCKED' : 'HW CAPABILITY · SIGNED'}</strong>
          </span>
          <time className="clock mono" dateTime={clock.toISOString()} title="Local time">{clock.toLocaleTimeString([], { hour12: false })}</time>
          <ThemeButton theme={theme} onToggle={() => setTheme((current) => current === 'dark' ? 'light' : 'dark')} />
        </div>
      </header>

      {stale && (
        <div className="stale-banner">
          <Lamp state="fault" />
          <span role="alert"><strong>Supervisor connection lost.</strong> Displayed data may be stale; routine keys are locked. Stop and End session remain available.</span>
          {/* Outside the alert so the ticking age is not re-announced every second. */}
          <span className="mono stale-age">{lastUpdateText(status.health.updated_at, clock.getTime())}</span>
        </div>
      )}

      {warnings.length > 0 && (
        <div className="advisory-banner" role="status">
          <AlertTriangle aria-hidden="true" />
          <ul>{warnings.map((warning) => <li key={warning}>{warning}</li>)}</ul>
        </div>
      )}

      <div className={`workspace-frame ${stale ? 'is-stale' : ''}`}>
        <FaultAlarm status={status} exporting={exporting} onExport={exportDiagnostics} />
        {setupAvailable && <SetupView client={setupClient} initial={setupProbe} hidden={workspace !== 'setup'} />}
        {workspace === 'setup' ? null : workspace === 'session' ? (
          <SessionView
            status={status}
            demo={demo}
            playback={playback}
            readOnly={readOnlyPresentation}
            operatorName={operatorName}
            spotterName={spotterName}
            stopChecked={stopChecked}
            areaChecked={areaChecked}
            harnessChecked={harnessChecked}
            onOperatorName={setOperatorName}
            onSpotterName={setSpotterName}
            onStopChecked={setStopChecked}
            onAreaChecked={setAreaChecked}
            onHarnessChecked={setHarnessChecked}
            keyBank={keyBankFor('inline')}
          />
        ) : (
          <EngineeringView status={status} config={config} audit={audit} qualificationRuns={qualificationRuns} qualificationLoading={qualificationLoading} qualificationError={qualificationError} demo={demo} exporting={exporting} saving={saving} onConfig={setConfig} onSave={saveConfiguration} onExport={exportDiagnostics} onRetryQualifications={retryQualificationRuns} />
        )}
      </div>

      {(notice || error) && (
        <div className={`notice ${error ? 'is-error' : ''}`} role={error ? 'alert' : 'status'}>
          <Lamp state={error ? 'fault' : 'passed'} />
          <span>{error ?? notice}</span>
          <button className="panel-key icon-key" onClick={() => { setError(null); setNotice(null) }} aria-label="Dismiss message"><X aria-hidden="true" /></button>
        </div>
      )}

      {workspace !== 'setup' && <ActionDock status={status} pending={pending} preflightComplete={preflightComplete} gateFor={gateFor} inspect={inspect} keyBank={keyBankFor('dock')} onAction={handleAction} />}
    </div>
  )
}
