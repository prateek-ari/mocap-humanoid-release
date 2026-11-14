import { useCallback, useEffect, useRef, useState, type ReactNode } from 'react'
import {
  AlertTriangle,
  ChevronDown,
  Crosshair,
  FolderOpen,
  Play,
  Power,
  RefreshCw,
  RotateCcw,
  Square,
  Stethoscope,
  Video,
  Wrench,
  X,
} from 'lucide-react'
import { Lamp, ThemeButton } from './components'
import {
  SetupApiError,
  isLoopbackIp,
  normalizeField,
  tildePath,
  type ControlCommand,
  type JobKind,
  type JobSummary,
  type LiveStatus,
  type Prefs,
  type SessionValues,
  type SetupClient,
  type SetupState,
  type StreamSnapshot,
} from './setupApi'
import { JOB_POLL_MS, LOST_JOB_SUMMARY } from './SetupView'
import type { Led } from './status'

/** The idle stream monitor is read this often while no job owns the port. */
export const STREAM_POLL_MS = 500
/** Settings are written to the session this long after the last edit. */
export const SETTINGS_SAVE_MS = 700

const LOOPBACK_IP = '127.0.0.1'
const MIN_HEIGHT_M = 0.8
const MAX_HEIGHT_M = 2.5
const MIN_CALIBRATE_S = 0.5
const MAX_CALIBRATE_S = 60
/** Thresholds of the stream check (docs/windows-setup.md §4): below them a readout is amber. */
const GOOD_RATE_HZ = 50
/** A running simulation whose frame count stays the same this long has lost Xsens (STATUS comes every 2 s). */
export const LIVE_STALE_MS = 5000
const GOOD_LOSS = 0.01

const ROBOTS: { value: SessionValues['robot']; name: string; detail: string }[] = [
  { value: 'booster_k1', name: 'Booster K1', detail: '22-DOF humanoid' },
  { value: 'unitree_g1', name: 'Unitree G1', detail: '29-DOF humanoid' },
  { value: 'booster_t1_29dof', name: 'Booster T1', detail: '29-DOF humanoid' },
]

type XsensHost = 'this' | 'other'
type Tone = Led | 'warning'

interface Draft {
  robot: SessionValues['robot']
  height: string
  host: XsensHost
  /** The Xsens PC's IPv4 (Another PC only). */
  xsensIp: string
  /** This PC's IPv4 on the Xsens network (Another PC only). */
  thisIp: string
}

type DraftField = 'human_height_m' | 'windows_xsens_ip' | 'linux_xsens_ip'
type FieldErrors = Partial<Record<DraftField, string>>

interface LogEntry {
  job: JobSummary
  lines: string[]
}

// ---------- Pure helpers ----------

function draftFrom(setup: SetupState): Draft {
  const values = setup.session ?? setup.defaults
  const ip = values.windows_xsens_ip.trim()
  const onThisPc = ip ? isLoopbackIp(ip) : setup.xsens_on_this_pc
  return {
    robot: values.robot,
    height: String(values.human_height_m),
    host: onThisPc ? 'this' : 'other',
    xsensIp: onThisPc ? '' : ip,
    thisIp: onThisPc ? '' : values.linux_xsens_ip || setup.network.detected_linux_ip || '',
  }
}

function sameDraft(a: Draft, b: Draft): boolean {
  return a.robot === b.robot && Number(a.height) === Number(b.height) && a.host === b.host
    && (a.host === 'this' || (a.xsensIp.trim() === b.xsensIp.trim() && a.thisIp === b.thisIp))
}

/** Session values for the draft, or the field errors that stop it being saved. */
function valuesFrom(draft: Draft, base: SessionValues, sessionDir: string, requireIp: boolean): { values: SessionValues | null; errors: FieldErrors } {
  const errors: FieldErrors = {}
  const height = Number(draft.height.trim())
  if (!draft.height.trim() || !Number.isFinite(height)) errors.human_height_m = 'Enter the height in metres.'
  else if (height < MIN_HEIGHT_M || height > MAX_HEIGHT_M) errors.human_height_m = `Height must be ${MIN_HEIGHT_M}–${MAX_HEIGHT_M} m.`
  if (draft.host === 'other' && requireIp && !draft.xsensIp.trim()) errors.windows_xsens_ip = "Enter the Xsens PC's IPv4 address."
  if (Object.keys(errors).length) return { values: null, errors }
  const onThisPc = draft.host === 'this'
  return {
    errors,
    values: {
      ...base,
      route: 'live',
      session_dir: sessionDir,
      robot: draft.robot,
      human_height_m: height,
      windows_xsens_ip: onThisPc ? LOOPBACK_IP : draft.xsensIp.trim(),
      linux_xsens_ip: onThisPc ? '' : draft.thisIp,
    },
  }
}

/** Seconds for auto-calibration, or ``null`` when the text is not a usable value. */
function calibrateSeconds(text: string): number | null {
  const value = Number(text.trim())
  return text.trim() && Number.isFinite(value) && value >= MIN_CALIBRATE_S && value <= MAX_CALIBRATE_S ? value : null
}

/** The setup list and polled jobs merged (as in the Setup view): a finished state always wins. */
function mergeJobs(listed: JobSummary[], logs: Record<string, LogEntry>, lost: ReadonlySet<string>): JobSummary[] {
  const seen = new Set<string>()
  const merged = listed.map((job) => {
    seen.add(job.id)
    const polled = logs[job.id]?.job
    return job.state !== 'running' || !polled ? job : polled
  })
  for (const entry of Object.values(logs)) {
    if (!seen.has(entry.job.id)) merged.push(entry.job)
  }
  return merged
    .map((job) => job.state === 'running' && lost.has(job.id) ? { ...job, state: 'stopped' as const, summary: LOST_JOB_SUMMARY } : job)
    .sort((a, b) => b.started_at - a.started_at)
}

function errorMessage(reason: unknown, fallback: string): string {
  return reason instanceof Error && reason.message ? reason.message : fallback
}

function elapsedSeconds(job: JobSummary, nowMs: number): number {
  const end = job.state === 'running' ? nowMs / 1000 : job.finished_at ?? nowMs / 1000
  return job.started_at > 0 ? Math.max(0, end - job.started_at) : 0
}

function clockText(seconds: number): string {
  const whole = Math.floor(seconds)
  const minutes = Math.floor(whole / 60)
  return minutes ? `${minutes}:${String(whole % 60).padStart(2, '0')}` : `${whole} s`
}

/** The session folder has hosted a live run: Start then opens a fresh one. */
function folderUsed(setup: SetupState, jobs: JobSummary[]): boolean {
  return jobs.some((job) => job.kind === 'live') || setup.files.some((file) => file.kind === 'video')
}

interface StreamView {
  tone: Tone
  word: string
  detail: string
}

function streamView(stream: StreamSnapshot | null, port: number): StreamView {
  switch (stream?.state) {
    case 'receiving':
      return { tone: 'passed', word: 'Receiving', detail: stream.source ? `Body frames from ${stream.source}` : 'Body frames are arriving.' }
    case 'none':
      return { tone: 'off', word: 'Waiting for Xsens', detail: `Listening on UDP ${port}. Start the Network Streamer in Xsens Humanoid.` }
    case 'stale': {
      const age = stream.last_packet_age_s
      return { tone: 'warning', word: 'Stale', detail: `Xsens stopped sending${age !== null ? ` ${clockText(age)} ago` : ''}. Is the stream or playback paused?` }
    }
    case 'port_busy':
      return { tone: 'fault', word: 'Port in use', detail: `Another program is receiving on UDP ${port}. Close it (an old viewer or a second console).` }
    case 'paused':
      return { tone: 'off', word: 'Paused', detail: `A check is using UDP ${port}.` }
    default:
      return { tone: 'off', word: 'Not monitored', detail: 'The stream monitor is off.' }
  }
}

function ToneLamp({ tone, label }: { tone: Tone; label?: string }) {
  if (tone === 'warning') {
    return <span className="lamp lamp-warning" role={label ? 'img' : undefined} aria-label={label} aria-hidden={label ? undefined : true} />
  }
  return <Lamp state={tone} label={label} />
}

function Meter({ label, value, unit, tone, wide = false }: { label: string; value: string | null; unit?: string; tone?: 'warning' | 'fault'; wide?: boolean }) {
  return (
    <div className={`readout ${tone ? `tone-${tone}` : ''} ${value === null ? 'is-unreported' : ''} ${wide ? 'wide' : ''}`}>
      <span className="silk">{label}</span>
      {value === null
        ? <strong className="unreported">— —<small>not reported</small></strong>
        : <strong>{value}{unit ? <small>{unit}</small> : null}</strong>}
    </div>
  )
}

function InlineError({ message }: { message: string | null }) {
  if (!message) return null
  return <p className="setup-error" role="alert"><AlertTriangle aria-hidden="true" />{message}</p>
}

const VERDICT: Record<JobSummary['state'], string> = { running: 'Running', passed: 'Pass', failed: 'Fail', stopped: 'Stopped' }

function jobTone(job: JobSummary): Tone {
  if (job.state === 'running') return 'active'
  if (job.state === 'passed') return 'passed'
  if (job.state === 'failed') return 'fault'
  return job.kind === 'live' ? 'passed' : 'off'
}

function JobResult({ job, nowMs }: { job: JobSummary; nowMs: number }) {
  return (
    <div className={`tool-result is-${job.state}`}>
      <ToneLamp tone={jobTone(job)} />
      <span>
        <strong>{VERDICT[job.state]}</strong>
        <small className="mono">{job.summary ?? `${job.label} · ${clockText(elapsedSeconds(job, nowMs))}`}</small>
      </span>
    </div>
  )
}

// ---------- View ----------

export interface LiveAppProps {
  client: SetupClient
  initial: SetupState
  theme: 'dark' | 'light'
  onToggleTheme: () => void
}

/** The one-screen Windows desktop app: Xsens stream, session settings, Start/Stop and live controls. */
export function LiveApp({ client, initial, theme, onToggleTheme }: LiveAppProps) {
  const [setup, setSetup] = useState(initial)
  const [stream, setStream] = useState<StreamSnapshot | null>(initial.stream)
  const [streamError, setStreamError] = useState<string | null>(null)
  const [draft, setDraft] = useState<Draft>(() => draftFrom(initial))
  const [fieldErrors, setFieldErrors] = useState<FieldErrors>({})
  const [settingsState, setSettingsState] = useState<'idle' | 'saving' | 'saved'>('idle')
  const [settingsError, setSettingsError] = useState<string | null>(null)
  const [recordVideo, setRecordVideo] = useState(initial.prefs.record_video)
  const [autoCalibrate, setAutoCalibrate] = useState(initial.prefs.calibrate_after_s !== null)
  const [calibrateText, setCalibrateText] = useState(String(initial.prefs.calibrate_after_s ?? 5))
  const [prefsError, setPrefsError] = useState<string | null>(null)
  const [logs, setLogs] = useState<Record<string, LogEntry>>({})
  const [lostJobs, setLostJobs] = useState<ReadonlySet<string>>(() => new Set())
  const [stoppingIds, setStoppingIds] = useState<ReadonlySet<string>>(() => new Set())
  const [starting, setStarting] = useState<JobKind | null>(null)
  const [actionError, setActionError] = useState<string | null>(null)
  const [toolsError, setToolsError] = useState<string | null>(null)
  const [controlPending, setControlPending] = useState<ControlCommand | null>(null)
  const [controlError, setControlError] = useState<string | null>(null)
  const [refreshError, setRefreshError] = useState<string | null>(null)
  const [toolsOpen, setToolsOpen] = useState(false)
  // The streaming guide starts open until Xsens has been seen.
  const [howtoOpen, setHowtoOpen] = useState(() => initial.stream?.state !== 'receiving')
  const [nowMs, setNowMs] = useState(() => Date.now())
  const cursorRef = useRef(new Map<string, number>())
  const inflightRef = useRef(new Map<string, Promise<unknown>>())
  const requestedRef = useRef(new Set<string>())
  const saveTimerRef = useRef<number | undefined>(undefined)
  const prefsTimerRef = useRef<number | undefined>(undefined)
  const toolsHeadingRef = useRef<HTMLHeadingElement>(null)
  const toolsButtonRef = useRef<HTMLButtonElement>(null)
  const aliveRef = useRef(true)

  useEffect(() => {
    aliveRef.current = true
    return () => {
      aliveRef.current = false
      window.clearTimeout(saveTimerRef.current)
      window.clearTimeout(prefsTimerRef.current)
    }
  }, [])

  useEffect(() => {
    const previous = document.title
    document.title = 'Mocap Humanoid'
    return () => { document.title = previous }
  }, [])

  const jobs = mergeJobs(setup.jobs, logs, lostJobs)
  const runningJob = jobs.find((job) => job.state === 'running') ?? null
  const runningId = runningJob?.id ?? null
  const liveJob = jobs.find((job) => job.kind === 'live') ?? null
  const liveRunning = liveJob?.state === 'running'
  const checkJob = jobs.find((job) => job.kind === 'acceptance') ?? null
  const doctorJob = jobs.find((job) => job.kind === 'doctor') ?? null
  /** A stream check or the simulation owns the UDP port while it runs. */
  const portOwner = runningJob && (runningJob.kind === 'live' || runningJob.kind === 'acceptance') ? runningJob : null
  const busy = runningJob !== null || starting !== null
  const port = stream?.port ?? 5555
  const savedDraft = draftFrom(setup)
  const dirty = !setup.session || !sameDraft(savedDraft, draft)

  const refresh = useCallback(async () => {
    try {
      const next = await client.getSetup()
      if (!aliveRef.current) return
      if (next) {
        setSetup(next)
        if (next.stream) setStream(next.stream)
        setRefreshError(null)
      } else {
        setRefreshError('The console no longer offers the app controls. Close and reopen Mocap Humanoid.')
      }
    } catch (reason) {
      if (aliveRef.current) setRefreshError(errorMessage(reason, 'The console could not be reached.'))
    }
  }, [client])

  /** Append a job's new log lines; reads for one job are chained so no range is appended twice. */
  const readMore = useCallback((id: string) => {
    const previous = inflightRef.current.get(id) ?? Promise.resolve()
    const next = previous.catch(() => undefined).then(async () => {
      const log = await client.getJobLog(id, cursorRef.current.get(id) ?? 0)
      cursorRef.current.set(id, log.next)
      if (aliveRef.current) {
        setLogs((current) => ({
          ...current,
          [id]: { job: log.job, lines: [...(current[id]?.lines ?? []), ...log.lines].slice(-2000) },
        }))
      }
      return log
    })
    inflightRef.current.set(id, next)
    const release = () => {
      if (inflightRef.current.get(id) === next) inflightRef.current.delete(id)
    }
    next.then(release, release)
    return next
  }, [client])

  // Poll the running job (its log and, for the simulation, its live status).
  useEffect(() => {
    if (!runningId) return
    let active = true
    let timer: number | undefined
    const tick = async () => {
      timer = undefined
      let finished = false
      try {
        const log = await readMore(runningId)
        if (!active) return
        setNowMs(Date.now())
        finished = log.job.state !== 'running'
      } catch (reason) {
        if (!active) return
        if (reason instanceof SetupApiError && reason.status === 404) {
          setLostJobs((current) => new Set(current).add(runningId))
          finished = true
        }
      }
      if (finished) {
        void refresh()
        return
      }
      timer = window.setTimeout(() => void tick(), JOB_POLL_MS)
    }
    void tick()
    return () => {
      active = false
      if (timer !== undefined) window.clearTimeout(timer)
    }
  }, [runningId, readMore, refresh])

  // While nothing owns the port, read the stream monitor quickly.
  const monitorPolling = portOwner === null
  useEffect(() => {
    if (!monitorPolling) return
    let active = true
    let timer: number | undefined
    const tick = async () => {
      timer = undefined
      try {
        const next = await client.getStream()
        if (!active) return
        setStream(next)
        setStreamError(null)
      } catch (reason) {
        if (!active) return
        setStreamError(reason instanceof SetupApiError && reason.status === 404
          ? 'This console does not report the stream. Update Mocap Humanoid.'
          : errorMessage(reason, 'The stream monitor could not be read.'))
      }
      if (active) timer = window.setTimeout(() => void tick(), STREAM_POLL_MS)
    }
    timer = window.setTimeout(() => void tick(), STREAM_POLL_MS)
    return () => {
      active = false
      if (timer !== undefined) window.clearTimeout(timer)
    }
  }, [client, monitorPolling])

  // The latest finished job's log, read once for the Tools log view.
  const lastJob = jobs[0] ?? null
  const lastJobId = lastJob && lastJob.state !== 'running' && !logs[lastJob.id] ? lastJob.id : null
  useEffect(() => {
    if (!lastJobId || !lastJob || requestedRef.current.has(lastJobId)) return
    requestedRef.current.add(lastJobId)
    client.getJobLog(lastJobId, 0).then((log) => {
      if (!aliveRef.current) return
      cursorRef.current.set(lastJobId, log.next)
      setLogs((current) => current[lastJobId] ? current : { ...current, [lastJobId]: { job: log.job, lines: log.lines } })
    }, () => {
      if (aliveRef.current) setLogs((current) => current[lastJobId] ? current : { ...current, [lastJobId]: { job: lastJob, lines: [] } })
    })
  }, [client, lastJobId, lastJob])

  // ----- Session settings -----

  function applyError(reason: unknown, setGeneral: (message: string) => void, fallback: string) {
    const message = errorMessage(reason, fallback)
    const field = reason instanceof SetupApiError ? normalizeField(reason.field) : null
    if (field === 'human_height_m' || field === 'windows_xsens_ip' || field === 'linux_xsens_ip') {
      setFieldErrors({ [field]: message })
    } else {
      setGeneral(message)
    }
  }

  /** Write the draft into the current session folder (or the default one when there is none yet). */
  const saveInPlace = useCallback(async (next: Draft, current: SetupState): Promise<SetupState | null> => {
    const base = current.session ?? current.defaults
    const { values, errors } = valuesFrom(next, base, base.session_dir, false)
    if (!values) {
      setFieldErrors(errors)
      return null
    }
    const saved = await client.saveSession(values)
    if (aliveRef.current) {
      setSetup(saved)
      setFieldErrors({})
    }
    return saved
  }, [client])

  // Settings persist through the session: saved shortly after the last edit.
  useEffect(() => {
    window.clearTimeout(saveTimerRef.current)
    if (!dirty || busy || !setup.session && sameDraft(draft, savedDraft)) return
    saveTimerRef.current = window.setTimeout(() => {
      setSettingsState('saving')
      saveInPlace(draft, setup).then((saved) => {
        if (!aliveRef.current) return
        setSettingsState(saved ? 'saved' : 'idle')
        if (saved) setSettingsError(null)
      }, (reason: unknown) => {
        if (!aliveRef.current) return
        setSettingsState('idle')
        applyError(reason, setSettingsError, 'The settings were not saved.')
      })
    }, SETTINGS_SAVE_MS)
    return () => window.clearTimeout(saveTimerRef.current)
    // savedDraft is derived from setup; draft and setup are the real inputs.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [draft, setup, busy, saveInPlace])

  function edit(changes: Partial<Draft>, field?: DraftField) {
    setDraft((current) => ({ ...current, ...changes }))
    setSettingsState('idle')
    setSettingsError(null)
    if (field && fieldErrors[field]) setFieldErrors((current) => ({ ...current, [field]: undefined }))
  }

  function selectHost(host: XsensHost) {
    setFieldErrors((current) => ({ ...current, windows_xsens_ip: undefined, linux_xsens_ip: undefined }))
    edit(host === 'other' && !draft.thisIp ? { host, thisIp: setup.network.detected_linux_ip ?? '' } : { host })
  }

  // ----- Preferences -----

  function savePrefs(changes: Partial<Omit<Prefs, 'schema'>>) {
    setPrefsError(null)
    client.putPrefs(changes).then((saved) => {
      if (aliveRef.current) setSetup((current) => ({ ...current, prefs: saved }))
    }, (reason: unknown) => {
      if (aliveRef.current) setPrefsError(`Not saved: ${errorMessage(reason, 'the preference was rejected.')}`)
    })
  }

  function changeRecordVideo(checked: boolean) {
    setRecordVideo(checked)
    savePrefs({ record_video: checked })
  }

  function changeAutoCalibrate(checked: boolean) {
    setAutoCalibrate(checked)
    window.clearTimeout(prefsTimerRef.current)
    const seconds = calibrateSeconds(calibrateText)
    if (!checked) savePrefs({ calibrate_after_s: null })
    else if (seconds !== null) savePrefs({ calibrate_after_s: seconds })
  }

  function changeCalibrateText(text: string) {
    setCalibrateText(text)
    window.clearTimeout(prefsTimerRef.current)
    const seconds = calibrateSeconds(text)
    if (autoCalibrate && seconds !== null) {
      prefsTimerRef.current = window.setTimeout(() => savePrefs({ calibrate_after_s: seconds }), SETTINGS_SAVE_MS)
    }
  }

  const calibrateValue = calibrateSeconds(calibrateText)
  const calibrateInvalid = autoCalibrate && calibrateValue === null

  // ----- Jobs -----

  function track(job: JobSummary) {
    cursorRef.current.set(job.id, 0)
    requestedRef.current.add(job.id)
    setLogs((current) => ({ ...current, [job.id]: { job, lines: [] } }))
  }

  async function start() {
    window.clearTimeout(saveTimerRef.current)
    setActionError(null)
    setControlError(null)
    const base = setup.session ?? setup.defaults
    const check = valuesFrom(draft, base, base.session_dir, true)
    if (!check.values || calibrateInvalid) {
      setFieldErrors(check.errors)
      if (calibrateInvalid) setActionError(`Auto-calibrate needs ${MIN_CALIBRATE_S} to ${MAX_CALIBRATE_S} seconds.`)
      return
    }
    setStarting('live')
    try {
      // A folder that already holds a run is left as it is: this run gets a fresh, dated one.
      let current = setup
      const fresh = !current.session || folderUsed(current, jobs)
      if (fresh && current.session) {
        const latest = await client.getSetup()
        if (latest) current = latest
      }
      const values = valuesFrom(draft, current.session ?? current.defaults, fresh ? current.defaults.session_dir : current.session!.session_dir, true).values!
      const saved = await client.saveSession(values)
      if (!aliveRef.current) return
      setSetup(saved)
      setFieldErrors({})
      const job = await client.startJob('live', { record_video: recordVideo, calibrate_after_s: autoCalibrate ? calibrateValue : null })
      if (!aliveRef.current) return
      track(job)
    } catch (reason) {
      if (!aliveRef.current) return
      applyError(reason, setActionError, 'The simulation did not start.')
      if (reason instanceof SetupApiError && (reason.code === 'job_running' || reason.code === 'no_session')) void refresh()
    } finally {
      if (aliveRef.current) setStarting(null)
    }
  }

  async function runTool(kind: 'acceptance' | 'doctor') {
    window.clearTimeout(saveTimerRef.current)
    setToolsError(null)
    setStarting(kind)
    try {
      // The checks write their reports into the session folder, so it must exist.
      if (dirty) {
        const saved = await saveInPlace(draft, setup)
        if (!saved) {
          setToolsError('Fix the session settings first.')
          return
        }
      }
      const job = await client.startJob(kind)
      if (!aliveRef.current) return
      track(job)
    } catch (reason) {
      if (!aliveRef.current) return
      setToolsError(errorMessage(reason, 'The check did not start.'))
      if (reason instanceof SetupApiError && reason.code === 'job_running') void refresh()
    } finally {
      if (aliveRef.current) setStarting(null)
    }
  }

  /** The console answers at once; the job shows as stopping until polling reports its final state. */
  async function stopJob(job: JobSummary, onError: (message: string) => void) {
    setStoppingIds((current) => new Set(current).add(job.id))
    try {
      const stopped = await client.stopJob(job.id)
      if (!aliveRef.current || stopped.state === 'running') return
      try {
        await readMore(job.id)
      } catch {
        setLogs((current) => ({ ...current, [job.id]: { job: stopped, lines: current[job.id]?.lines ?? [] } }))
      }
      void refresh()
    } catch (reason) {
      if (!aliveRef.current) return
      setStoppingIds((current) => {
        const next = new Set(current)
        next.delete(job.id)
        return next
      })
      onError(errorMessage(reason, 'The job could not be stopped.'))
    }
  }
  const stopping = (job: JobSummary | null) => job !== null && stoppingIds.has(job.id)

  async function control(command: ControlCommand) {
    if (!liveJob || !liveRunning) return
    setControlError(null)
    setControlPending(command)
    try {
      await client.controlJob(liveJob.id, command)
    } catch (reason) {
      if (!aliveRef.current) return
      setControlError(reason instanceof SetupApiError && reason.status === 409
        ? 'The simulation is no longer running.'
        : errorMessage(reason, 'The simulation did not accept the command.'))
      if (reason instanceof SetupApiError && reason.status === 409) void refresh()
    } finally {
      if (aliveRef.current) setControlPending(null)
    }
  }

  // ----- Tools drawer -----

  function openTools() {
    setToolsOpen(true)
  }

  const closeTools = useCallback(() => {
    setToolsOpen(false)
    toolsButtonRef.current?.focus()
  }, [])

  useEffect(() => {
    if (!toolsOpen) return
    toolsHeadingRef.current?.focus()
    const onKey = (event: KeyboardEvent) => {
      if (event.key === 'Escape') closeTools()
    }
    window.addEventListener('keydown', onKey)
    return () => window.removeEventListener('keydown', onKey)
  }, [toolsOpen, closeTools])

  // ----- Derived display -----

  const live: LiveStatus | null = liveRunning ? liveJob?.live ?? null : null
  // STATUS keeps arriving while Xsens is silent, but its fps is a run average
  // that decays slowly: an unchanged frame count is the prompt signal.
  const liveKey = live && liveJob ? `${liveJob.id}:${live.frames}` : null
  const [staleKey, setStaleKey] = useState<string | null>(null)
  useEffect(() => {
    if (liveKey === null) return
    const timer = window.setTimeout(() => setStaleKey(liveKey), LIVE_STALE_MS)
    return () => window.clearTimeout(timer)
  }, [liveKey])
  const liveStale = liveKey !== null && staleKey === liveKey
  const noDisplay = setup.readiness.display === false
  const runtimeMissing = setup.readiness.live_runtime === false
  const idleView = streamError ? { tone: 'fault' as Tone, word: 'No reading', detail: streamError } : streamView(stream, port)
  const receiving = !streamError && stream?.state === 'receiving'

  let overall: { tone: Tone; text: string }
  if (liveRunning) {
    overall = stopping(liveJob) ? { tone: 'active', text: 'Stopping simulation' }
      : !live ? { tone: 'next', text: 'Simulation waiting for Xsens' }
      : liveStale ? { tone: 'warning', text: 'Simulation · no Xsens frames' }
      : live.enabled ? { tone: 'active', text: 'Tracking' }
      : { tone: 'passed', text: 'Simulation running · tracking off' }
  } else if (runningJob?.kind === 'acceptance') {
    overall = { tone: 'active', text: 'Checking the stream' }
  } else if (runningJob) {
    overall = { tone: 'active', text: `${runningJob.label} running` }
  } else if (refreshError) {
    overall = { tone: 'fault', text: 'Console unreachable' }
  } else if (receiving) {
    overall = { tone: 'passed', text: 'Ready · Xsens receiving' }
  } else {
    overall = { tone: idleView.tone, text: idleView.word }
  }

  const rate = stream?.rate_hz ?? null
  const loss = stream?.loss_fraction ?? null
  const streamReadouts = !streamError && stream && stream.state !== 'off' && stream.state !== 'paused'
  const checkSeconds = Math.round(setup.stream_check_s)
  const robotName = ROBOTS.find((robot) => robot.value === draft.robot)?.name ?? 'Robot'
  const sessionDir = setup.session?.session_dir ?? ''
  const sessionsRoot = (setup.session ?? setup.defaults).session_dir.replace(/[\\/][^\\/]+[\\/]?$/, '')
  const logJob = runningJob ?? lastJob
  const logLines = logJob ? logs[logJob.id]?.lines : undefined

  function renderStreamBody(): ReactNode {
    if (portOwner?.kind === 'acceptance') {
      const elapsed = Math.min(checkSeconds, Math.floor(elapsedSeconds(portOwner, nowMs)))
      return (
        <>
          <div className="live-state">
            <Lamp state="active" />
            <div><strong>Checking</strong><span>The {checkSeconds}-second stream check is listening. Keep Xsens streaming.</span></div>
          </div>
          <div className="progress-track live-progress" role="progressbar" aria-label="Stream check progress" aria-valuemin={0} aria-valuemax={checkSeconds} aria-valuenow={elapsed}>
            <i style={{ width: `${Math.min(100, (elapsed / Math.max(1, checkSeconds)) * 100)}%` }} />
          </div>
          <p className="group-note mono">{elapsed} / {checkSeconds} s</p>
        </>
      )
    }
    if (portOwner?.kind === 'live') {
      return (
        <>
          <div className="live-state">
            {liveStale ? <ToneLamp tone="warning" /> : live ? <Lamp state="passed" /> : <Lamp state="next" />}
            <div>
              <strong>{liveStale ? 'No new frames' : live ? 'Receiving' : 'Waiting'}</strong>
              <span>{liveStale ? `No Xsens frame for ${LIVE_STALE_MS / 1000} s. Is the stream or playback paused in Xsens Humanoid?`
                : live ? 'The simulation is receiving Xsens. Its readings are below.' : 'Waiting for the first Xsens frame…'}</span>
            </div>
          </div>
          <div className="readout-grid live-readouts">
            <Meter label="Simulation rate" value={live ? live.fps.toFixed(1) : null} unit="fps" />
            <Meter label="Frames" value={live ? String(live.frames) : null} />
            <Meter label="Dropped" value={live ? String(live.drops) : null} tone={live && live.drops > 0 ? 'warning' : undefined} />
            <Meter label="Failures" value={live ? String(live.failures) : null} tone={live && live.failures > 0 ? 'fault' : undefined} />
          </div>
        </>
      )
    }
    return (
      <>
        <div className="live-state">
          <ToneLamp tone={idleView.tone} />
          <div><strong>{idleView.word}</strong><span>{idleView.detail}</span></div>
        </div>
        <div className="readout-grid live-readouts is-stream">
          <Meter label="Rate" value={streamReadouts && rate !== null ? rate.toFixed(1) : null} unit="Hz" tone={streamReadouts && receiving && rate !== null && rate < GOOD_RATE_HZ ? 'warning' : undefined} />
          <Meter label="Loss" value={streamReadouts && loss !== null ? (loss * 100).toFixed(1) : null} unit="%" tone={streamReadouts && receiving && loss !== null && loss >= GOOD_LOSS ? 'warning' : undefined} />
          <Meter label="Joint angles" value={streamReadouts && stream.body_frames > 0 ? stream.joint_angles ? 'Yes' : 'No' : null} tone={streamReadouts && receiving && !stream.joint_angles ? 'warning' : undefined} />
          <Meter label="Source" value={streamReadouts ? stream.source : null} wide />
        </div>
        {streamReadouts && receiving && !stream.joint_angles && (
          <p className="group-note">No joint angles: choose the preset with <strong>Joint Angles</strong> in the Network Streamer.</p>
        )}
      </>
    )
  }

  const destination = draft.host === 'this' ? LOOPBACK_IP : draft.thisIp || "this PC's IPv4"

  return (
    <div className="app-shell live-app">
      <header className="app-header live-header">
        <div className="brand">
          <strong>MOCAP HUMANOID</strong>
          {setup.app?.version && <span className="live-version mono">v{setup.app.version}</span>}
        </div>
        <div className="header-annunciators">
          <span className={`annunciator-chip live-overall tone-${overall.tone}`} role="status" aria-live="polite">
            <ToneLamp tone={overall.tone} />
            <strong>{overall.text}</strong>
          </span>
          <button ref={toolsButtonRef} type="button" className="panel-key" aria-expanded={toolsOpen} aria-controls="live-tools" onClick={() => toolsOpen ? closeTools() : openTools()}>
            <Wrench aria-hidden="true" />Tools
          </button>
          <ThemeButton theme={theme} onToggle={onToggleTheme} />
        </div>
      </header>

      <div className="workspace-frame">
        <main className="live-workspace" aria-label="Mocap Humanoid">
          {refreshError && <InlineError message={refreshError} />}

          <section className="live-stream" aria-labelledby="live-stream-title">
            <div className="display-bezel">
              <div className="display live-display">
                <header className="live-panel-head">
                  <span className="silk" id="live-stream-title">Xsens stream</span>
                  <span className="mono live-port">UDP {port}{stream?.bind ? ` · ${stream.bind}` : ''}</span>
                </header>
                <div className="live-display-body">{renderStreamBody()}</div>
              </div>
            </div>
            <details className="setup-details live-howto" open={howtoOpen} onToggle={(event) => setHowtoOpen(event.currentTarget.open)}>
              <summary><span>How to stream from Xsens Humanoid</span><small /><ChevronDown aria-hidden="true" /></summary>
              <ol className="setup-checklist">
                <li>Open <strong>Options → Network Streamer</strong> and add a stream.</li>
                <li>Destination <code className="mono">{destination}</code>, port <code className="mono">{port}</code>, protocol <strong>UDP</strong>.</li>
                <li>Preset <strong>ROS: Position + Orientation (Quaternion) and Joint Angles</strong>.</li>
                <li>Prop and finger segments <strong>off</strong> (the receiver needs the 23-segment body).</li>
                <li>Keep the stream enabled. After editing it, disable and re-enable it so the change applies.</li>
              </ol>
              {draft.host === 'other' && <p className="group-note">On another PC, allow Python on <strong>Private</strong> networks when Windows Firewall asks.</p>}
            </details>
          </section>

          <section className="live-control" aria-label="Session and simulation">
            {liveRunning ? (
              <section className="group live-summary" aria-labelledby="live-summary-title">
                <h2 className="silk" id="live-summary-title">Session · locked while the simulation runs</h2>
                <dl>
                  <div><dt>Robot</dt><dd>{robotName}</dd></div>
                  <div><dt>Height</dt><dd className="mono">{draft.height} m</dd></div>
                  <div><dt>Xsens on</dt><dd>{draft.host === 'this' ? 'This PC' : <span className="mono">{draft.xsensIp}</span>}</dd></div>
                  <div><dt>Video</dt><dd>{recordVideo ? 'Recording' : 'Off'}</dd></div>
                  <div><dt>Auto-calibrate</dt><dd className="mono">{autoCalibrate && calibrateValue !== null ? `${calibrateValue} s` : 'Off'}</dd></div>
                </dl>
              </section>
            ) : (
              <fieldset className="group live-settings" disabled={busy}>
                <legend className="silk">Session</legend>
                <div className="choice-cards live-robots" role="radiogroup" aria-label="Robot">
                  {ROBOTS.map((robot) => (
                    <label key={robot.value} className={draft.robot === robot.value ? 'selected' : ''}>
                      <input type="radio" name="live-robot" value={robot.value} checked={draft.robot === robot.value} onChange={() => edit({ robot: robot.value })} />
                      <span><strong>{robot.name}</strong><small>{robot.detail}</small></span>
                    </label>
                  ))}
                </div>
                <div className="live-fields">
                  <div className="live-field">
                    <label className="silk" htmlFor="live-height">Performer height (m)</label>
                    <input
                      id="live-height"
                      className="mono"
                      type="number"
                      step="0.01"
                      min={MIN_HEIGHT_M}
                      max={MAX_HEIGHT_M}
                      inputMode="decimal"
                      value={draft.height}
                      aria-invalid={fieldErrors.human_height_m ? true : undefined}
                      aria-describedby={fieldErrors.human_height_m ? 'live-error-height' : undefined}
                      onChange={(event) => edit({ height: event.target.value }, 'human_height_m')}
                    />
                    {fieldErrors.human_height_m && <span className="field-error" id="live-error-height" role="alert">{fieldErrors.human_height_m}</span>}
                  </div>
                  <div className="live-field">
                    <span className="silk" id="live-host-label">Xsens runs on</span>
                    <div className="route-toggle live-host" role="radiogroup" aria-labelledby="live-host-label">
                      {([{ value: 'this', label: 'This PC' }, { value: 'other', label: 'Another PC' }] as const).map((option) => (
                        <label key={option.value} className={draft.host === option.value ? 'selected' : ''}>
                          <input type="radio" name="live-host" value={option.value} checked={draft.host === option.value} onChange={() => selectHost(option.value)} />
                          {option.label}
                        </label>
                      ))}
                    </div>
                  </div>
                </div>
                {draft.host === 'other' && (
                  <div className="live-fields">
                    <div className="live-field">
                      <label className="silk" htmlFor="live-xsens-ip">Xsens PC IP</label>
                      <input
                        id="live-xsens-ip"
                        className="mono"
                        type="text"
                        value={draft.xsensIp}
                        placeholder="e.g. 192.168.1.246"
                        spellCheck={false}
                        aria-invalid={fieldErrors.windows_xsens_ip ? true : undefined}
                        aria-describedby={fieldErrors.windows_xsens_ip ? 'live-error-xsens-ip' : undefined}
                        onChange={(event) => edit({ xsensIp: event.target.value }, 'windows_xsens_ip')}
                      />
                      {fieldErrors.windows_xsens_ip && <span className="field-error" id="live-error-xsens-ip" role="alert">{fieldErrors.windows_xsens_ip}</span>}
                    </div>
                    <div className="live-field">
                      <label className="silk" htmlFor="live-this-ip">This PC's IP</label>
                      <select
                        id="live-this-ip"
                        className="mono"
                        value={draft.thisIp}
                        aria-invalid={fieldErrors.linux_xsens_ip ? true : undefined}
                        aria-describedby={fieldErrors.linux_xsens_ip ? 'live-error-this-ip' : undefined}
                        onChange={(event) => edit({ thisIp: event.target.value }, 'linux_xsens_ip')}
                      >
                        <option value="">Choose…</option>
                        {draft.thisIp && !setup.network.interfaces.some((item) => item.ipv4 === draft.thisIp) && <option value={draft.thisIp}>{draft.thisIp}</option>}
                        {setup.network.interfaces.map((item) => (
                          <option key={`${item.name}-${item.ipv4}`} value={item.ipv4}>{item.ipv4} · {item.name}</option>
                        ))}
                      </select>
                      {fieldErrors.linux_xsens_ip && <span className="field-error" id="live-error-this-ip" role="alert">{fieldErrors.linux_xsens_ip}</span>}
                    </div>
                  </div>
                )}
                <div className="setup-options live-options">
                  <label className="check-option">
                    <input type="checkbox" checked={recordVideo} onChange={(event) => changeRecordVideo(event.target.checked)} />
                    <Video aria-hidden="true" />
                    <span>Record video</span>
                  </label>
                  <label className="check-option">
                    <input type="checkbox" checked={autoCalibrate} onChange={(event) => changeAutoCalibrate(event.target.checked)} />
                    <span>Auto-calibrate after</span>
                  </label>
                  <span className="check-option">
                    <input
                      className="mono short-input"
                      type="number"
                      step="0.5"
                      min={MIN_CALIBRATE_S}
                      max={MAX_CALIBRATE_S}
                      value={calibrateText}
                      disabled={!autoCalibrate}
                      aria-label="Seconds before auto-calibration"
                      aria-invalid={calibrateInvalid || undefined}
                      onChange={(event) => changeCalibrateText(event.target.value)}
                    />
                    <span>s</span>
                  </span>
                </div>
                {calibrateInvalid && <span className="field-error" role="alert">Use {MIN_CALIBRATE_S} to {MAX_CALIBRATE_S} seconds.</span>}
                <p className="live-saved" aria-live="polite">
                  {settingsState === 'saving' ? <><RefreshCw className="spin" aria-hidden="true" />Saving settings…</>
                    : busy ? <>Settings are locked while {runningJob?.kind === 'live' ? 'the simulation runs' : 'a check runs'}.</>
                    : settingsState === 'saved' ? <><Lamp state="passed" />Settings saved</>
                    : null}
                </p>
                <InlineError message={settingsError} />
                <InlineError message={prefsError} />
              </fieldset>
            )}

            <section className={`live-sim ${liveRunning ? 'is-running' : ''}`} aria-labelledby="live-sim-title">
              <header className="live-panel-head">
                <span className="silk" id="live-sim-title">Simulation · {robotName}</span>
                {liveRunning && liveJob && <span className="mono">{clockText(elapsedSeconds(liveJob, nowMs))}</span>}
              </header>
              {noDisplay && <p className="setup-callout is-warning"><AlertTriangle aria-hidden="true" /><span>{setup.readiness.display_hint ?? 'No desktop display: the MuJoCo window cannot open.'}</span></p>}
              {runtimeMissing && !liveRunning && <p className="setup-callout is-warning"><AlertTriangle aria-hidden="true" /><span>The simulation runtime is missing. Reinstall Mocap Humanoid.</span></p>}
              {liveRunning && liveJob ? (
                <>
                  <div className="live-running">
                    <div className="live-state">
                      <Lamp state="active" />
                      <div>
                        <strong>Running</strong>
                        <span>Simulation running in the MuJoCo window.{recordVideo ? ' Recording video into the session folder.' : ''}</span>
                      </div>
                    </div>
                    <button type="button" className="panel-key live-primary is-stop" disabled={stopping(liveJob)} onClick={() => void stopJob(liveJob, setActionError)}>
                      {stopping(liveJob) ? <RefreshCw className="spin" aria-hidden="true" /> : <Square aria-hidden="true" />}
                      <span>{stopping(liveJob) ? 'Stopping…' : 'Stop'}</span>
                    </button>
                  </div>
                  <div className="live-tracking">
                    <span className={`live-flag ${live?.enabled ? 'is-on' : ''}`}>
                      <Lamp state={live?.enabled ? 'active' : 'off'} />
                      <span>Tracking <strong>{live ? live.enabled ? 'on' : 'off' : '—'}</strong>{live?.reason ? <small className="mono">{live.reason}</small> : null}</span>
                    </span>
                    <span className={`live-flag ${live?.calibrated ? 'is-on' : ''}`}>
                      <Lamp state={live?.calibrated ? 'passed' : 'off'} />
                      <span>Calibrated <strong>{live ? live.calibrated ? 'yes' : 'no' : '—'}</strong></span>
                    </span>
                  </div>
                  <p className="live-calibrate-note">To calibrate, the performer stands still with arms relaxed at the sides, then press Calibrate.</p>
                  <div className="live-keys" role="group" aria-label="Simulation controls">
                    <button type="button" className="panel-key" disabled={controlPending !== null || !live} onClick={() => void control('calibrate')}>
                      {controlPending === 'calibrate' ? <RefreshCw className="spin" aria-hidden="true" /> : <Crosshair aria-hidden="true" />}Calibrate<kbd>N</kbd>
                    </button>
                    <button type="button" className="panel-key" aria-pressed={live?.enabled ?? false} disabled={controlPending !== null || !live} onClick={() => void control('toggle_tracking')}>
                      {controlPending === 'toggle_tracking' ? <RefreshCw className="spin" aria-hidden="true" /> : <Power aria-hidden="true" />}{live?.enabled ? 'Tracking off' : 'Tracking on'}<kbd>Space</kbd>
                    </button>
                    <button type="button" className="panel-key" disabled={controlPending !== null || !live} onClick={() => void control('rearm')}>
                      {controlPending === 'rearm' ? <RefreshCw className="spin" aria-hidden="true" /> : <RotateCcw aria-hidden="true" />}Rearm<kbd>R</kbd>
                    </button>
                  </div>
                  {!live && <p className="group-note">The controls unlock with the first Xsens frame.</p>}
                  <InlineError message={controlError} />
                </>
              ) : (
                <>
                  <div className="live-running">
                    <div className="live-state">
                      <ToneLamp tone={receiving ? 'next' : 'off'} />
                      <div>
                        <strong>{runningJob ? 'Busy' : 'Ready'}</strong>
                        <span>{runningJob
                          ? `Wait for ${runningJob.label.toLowerCase()} to finish.`
                          : receiving ? 'Press Start. The robot opens in a separate MuJoCo window.'
                          : 'You can start now: the simulation waits for the first Xsens frame.'}</span>
                      </div>
                    </div>
                    <button type="button" className="panel-key is-primary live-primary" disabled={busy || noDisplay || calibrateInvalid} onClick={() => void start()}>
                      {starting === 'live' ? <RefreshCw className="spin" aria-hidden="true" /> : <Play aria-hidden="true" />}
                      <span>{starting === 'live' ? 'Starting…' : 'Start'}</span>
                    </button>
                  </div>
                  {liveJob && (
                    <div className="live-last">
                      <span className="silk">Last run</span>
                      <JobResult job={liveJob} nowMs={nowMs} />
                      {liveJob.outputs.length > 0 && <small className="mono">{liveJob.outputs.join(', ')}</small>}
                    </div>
                  )}
                </>
              )}
              <InlineError message={actionError} />
            </section>
          </section>
        </main>
      </div>

      <aside id="live-tools" className="live-tools" hidden={!toolsOpen} aria-labelledby="live-tools-title">
        <header className="live-tools-head">
          <h2 id="live-tools-title" ref={toolsHeadingRef} tabIndex={-1}>Tools</h2>
          <button type="button" className="panel-key icon-key" aria-label="Close tools" onClick={closeTools}><X aria-hidden="true" /></button>
        </header>
        <fieldset className="group">
          <legend className="silk">Checks</legend>
          <div className="tool-row">
            <button type="button" className="panel-key" disabled={busy} onClick={() => void runTool('acceptance')}>
              {starting === 'acceptance' ? <RefreshCw className="spin" aria-hidden="true" /> : <Play aria-hidden="true" />}Run {checkSeconds}-second stream check
            </button>
            {checkJob ? <JobResult job={checkJob} nowMs={nowMs} /> : <small className="group-note">Not run yet. Pass: 50 Hz, under 1 % loss, joint angles.</small>}
          </div>
          <div className="tool-row">
            <button type="button" className="panel-key" disabled={busy} onClick={() => void runTool('doctor')}>
              {starting === 'doctor' ? <RefreshCw className="spin" aria-hidden="true" /> : <Stethoscope aria-hidden="true" />}Run system check
            </button>
            {doctorJob ? <JobResult job={doctorJob} nowMs={nowMs} /> : <small className="group-note">Not run yet. Checks Python, the runtime, OpenGL and UDP {port}.</small>}
          </div>
          {runningJob && runningJob.kind !== 'live' && (
            <button type="button" className="panel-key" disabled={stopping(runningJob)} onClick={() => void stopJob(runningJob, setToolsError)}>
              {stopping(runningJob) ? <RefreshCw className="spin" aria-hidden="true" /> : <Square aria-hidden="true" />}{stopping(runningJob) ? 'Stopping…' : `Stop ${runningJob.label.toLowerCase()}`}
            </button>
          )}
          <InlineError message={toolsError} />
        </fieldset>
        <fieldset className="group">
          <legend className="silk">This PC</legend>
          <dl className="tool-facts">
            <div><dt>Xsens port</dt><dd className="mono">UDP {port}{stream?.bind ? ` on ${stream.bind}` : ''}</dd></div>
            <div><dt><FolderOpen aria-hidden="true" />Sessions folder</dt><dd className="mono">{tildePath(sessionsRoot, setup.home) || '—'}</dd></div>
            <div><dt>Current session</dt><dd className="mono">{sessionDir ? tildePath(sessionDir, setup.home) : 'None yet'}</dd></div>
            {setup.app?.version && <div><dt>Version</dt><dd className="mono">{setup.app.version}</dd></div>}
          </dl>
        </fieldset>
        <details className="setup-details live-log">
          <summary><span>Log</span><small className="mono">{logJob ? `${logJob.label} · ${VERDICT[logJob.state]}` : 'No job yet'}</small><ChevronDown aria-hidden="true" /></summary>
          <pre className="job-log mono" role="log" aria-label={logJob ? `${logJob.label} log` : 'Job log'} tabIndex={0}>
            {logLines?.length ? logLines.join('\n') : logJob ? logJob.state === 'running' ? 'Waiting for output…' : logLines ? 'No output.' : 'Loading log…' : 'Nothing has run yet.'}
          </pre>
        </details>
      </aside>
    </div>
  )
}
