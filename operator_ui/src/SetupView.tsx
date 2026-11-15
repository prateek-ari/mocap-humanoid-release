import { useCallback, useEffect, useRef, useState, type ReactNode } from 'react'
import { AlertTriangle, BookOpen, Bot, Check, ChevronDown, Columns2, Copy, Crosshair, Download, FilePlus, FileSearch, Film, Play, Power, RefreshCw, Save, ShieldCheck, Square, Stethoscope, Video } from 'lucide-react'
import { Lamp } from './components'
import {
  SESSION_FIELDS,
  SetupApiError,
  asSetupRobot,
  isLoopbackIp,
  normalizeField,
  tildePath,
  type ControlCommand,
  type JobKind,
  type JobOptions,
  type JobSummary,
  type K1OpenOutcome,
  type K1Status,
  type PrivilegedAction,
  type PrivilegedOutcome,
  type SessionField,
  type SessionValues,
  type SetupClient,
  type SetupFile,
  type SetupRoute,
  type SetupState,
} from './setupApi'
import type { Led } from './status'

export type SetupStep = 'session' | 'windows' | 'stream' | 'live' | 'bvh' | 'retarget' | 'render' | 'finish' | 'k1'

export const SETUP_STEPS: readonly SetupStep[] = ['session', 'windows', 'stream', 'live', 'bvh', 'retarget', 'render', 'finish', 'k1']

/** Job polling interval while a job is running. */
export const JOB_POLL_MS = 500

/** Shown for a job that 404s while polled: the console restarted and lost it. */
export const LOST_JOB_SUMMARY = 'The console restarted; this job is no longer tracked.'
const STREAM_PORT = '5555'

interface StepMeta {
  /** Step number(s) in docs/engineer-setup-guide.md. */
  guide: string
  title: string
  route: SetupRoute | null
  /** Job kinds whose latest run is shown in this step. */
  kinds: readonly string[]
  instruction: string
}

const STEP_META: Record<SetupStep, StepMeta> = {
  session: { guide: '03', title: 'Session', route: null, kinds: ['doctor'], instruction: 'Fill in the values for this take, then click Save session.' },
  windows: { guide: '04–05', title: 'Windows: suit and streaming', route: 'live', kinds: [], instruction: 'On the Windows PC, set up Xsens with these values. Here, click only Allow in firewall if it is shown.' },
  stream: { guide: '06', title: 'Check the stream', route: 'live', kinds: ['acceptance'], instruction: 'Start streaming in Xsens, then click Run {n}-second check.' },
  live: { guide: '07', title: 'Live simulation', route: 'live', kinds: ['live', 'compare'], instruction: 'Click Start live simulation, then start streaming in Xsens within 60 seconds.' },
  bvh: { guide: '08', title: 'BVH files', route: 'recorded', kinds: ['bvh_check'], instruction: 'Copy both BVH files to the paths below, then click Check files.' },
  retarget: { guide: '09', title: 'Retarget', route: 'recorded', kinds: ['retarget'], instruction: 'Click Make preview to turn the human motion into robot motion.' },
  render: { guide: '10', title: 'Watch and record', route: 'recorded', kinds: ['render'], instruction: 'Choose a motion, then click Record MP4. Let it finish.' },
  finish: { guide: '11', title: 'Finish', route: null, kinds: ['save'], instruction: 'Stop anything still running, then click Save report.' },
  k1: { guide: '13', title: 'Real K1', route: null, kinds: ['k1_install'], instruction: 'Install the K1 test tools once, then click Open K1 test window.' },
}

/** Live-route jobs that show the viewer in the gmr-xsens-live container; Stop ends them normally. */
const VIEWER_KINDS: readonly string[] = ['live', 'compare']
/** Where the live tool sends the K1 arm references; the K1 window's Live arms step listens here. */
const ROBOT_STREAM_URL = 'udp://127.0.0.1:47011'

/** Wording that differs on a Windows console; the Linux text above is the default. */
const WINDOWS_STEP_META: Partial<Record<SetupStep, Pick<StepMeta, 'title' | 'instruction'>>> = {
  windows: { title: 'Xsens: suit and streaming', instruction: 'In Xsens Humanoid, set up the suit and streaming with these values. Nothing to click here.' },
}

const LOOPBACK_IP = '127.0.0.1'

type XsensHost = 'this' | 'other'

const ROBOTS: { value: SessionValues['robot']; name: string; detail: string }[] = [
  { value: 'booster_k1', name: 'Booster K1', detail: '22-DOF humanoid' },
  { value: 'unitree_g1', name: 'Unitree G1', detail: '29-DOF humanoid' },
  { value: 'booster_t1_29dof', name: 'Booster T1', detail: '29-DOF humanoid' },
]

/** ``#setup`` or ``#setup/<step>``; ``null`` for any other hash. */
export function parseSetupHash(hash: string): { step: SetupStep | null } | null {
  const match = /^#setup(?:\/([a-z0-9]+))?\/?$/.exec(hash.trim())
  if (!match) return null
  const step = match[1]
  return { step: step && (SETUP_STEPS as readonly string[]).includes(step) ? step as SetupStep : null }
}

/** ``desktopTools``: the Linux checkout console, which also has step 13 (Real K1). */
export function stepsForRoute(route: SetupRoute, desktopTools = false): SetupStep[] {
  return SETUP_STEPS.filter((step) => (STEP_META[step].route === null || STEP_META[step].route === route) && (desktopTools || step !== 'k1'))
}

// ---------- Session draft ----------

type Draft = Record<SessionField, string>
type FieldErrors = Partial<Record<SessionField, string>>

const NUMERIC_FIELDS: ReadonlySet<SessionField> = new Set<SessionField>([
  'human_height_m', 'bvh_scale', 'start_frame', 'end_frame', 'frame_step', 'npose_start_s', 'npose_duration_s',
])
const FRAME_FIELDS: ReadonlySet<SessionField> = new Set<SessionField>(['start_frame', 'end_frame', 'frame_step', 'npose_start_s', 'npose_duration_s'])
const COMMON_FIELDS: readonly SessionField[] = ['route', 'robot', 'human_height_m', 'session_dir']
const LIVE_FIELDS: readonly SessionField[] = ['windows_xsens_ip', 'linux_xsens_ip']
const RECORDED_FIELDS: readonly SessionField[] = ['mocap_ws', 'motion_bvh', 'npose_bvh', 'bvh_scale', 'start_frame', 'end_frame', 'frame_step', 'npose_start_s', 'npose_duration_s']

/** On Windows the live viewer runs from the runtime folder, so the live route shows it too. */
function visibleFields(route: SetupRoute, windows = false): ReadonlySet<SessionField> {
  const live: readonly SessionField[] = windows ? [...LIVE_FIELDS, 'mocap_ws'] : LIVE_FIELDS
  return new Set([...COMMON_FIELDS, ...(route === 'live' ? live : RECORDED_FIELDS)])
}

// offsets_file is not edited here (guide step 09 uses --allow-zero-offsets); it is round-tripped unchanged.
const PATH_FIELDS: ReadonlySet<SessionField> = new Set<SessionField>(['session_dir', 'mocap_ws', 'motion_bvh', 'npose_bvh'])

/** Server values for the form: paths under home are shown as ``~/…``. */
function toDraft(values: SessionValues, home: string | null): Draft {
  return Object.fromEntries(SESSION_FIELDS.map((field) => [
    field,
    PATH_FIELDS.has(field) ? tildePath(String(values[field]), home) : String(values[field]),
  ])) as Draft
}

/** Windows: where Xsens Humanoid runs, from the draft's source IP (or the console's hint while it is empty). */
function xsensHostOf(setup: SetupState, draft: Draft): XsensHost {
  const ip = draft.windows_xsens_ip.trim()
  return (ip ? isLoopbackIp(ip) : setup.xsens_on_this_pc) ? 'this' : 'other'
}

function initialDraft(setup: SetupState, route: SetupRoute | null): Draft {
  const draft = toDraft(setup.session ?? setup.defaults, setup.home)
  if (route) draft.route = route
  if (setup.platform === 'windows' && xsensHostOf(setup, draft) === 'this') {
    // Same PC: the source is loopback and there is no receiver IP to choose.
    draft.windows_xsens_ip = LOOPBACK_IP
    draft.linux_xsens_ip = ''
    return draft
  }
  // Prefill only a route-detected address; a lone interface may be Wi-Fi.
  if (!draft.linux_xsens_ip && setup.network.detected_linux_ip) draft.linux_xsens_ip = setup.network.detected_linux_ip
  return draft
}

/**
 * Only fields shown for the route are checked here.  A hidden numeric field
 * that does not parse keeps its server value (``fallback``); the backend
 * still validates everything, and its errors on hidden fields are shown at
 * form level.
 */
function fromDraft(draft: Draft, fallback: SessionValues, windows = false): { values: SessionValues | null; errors: FieldErrors } {
  const errors: FieldErrors = {}
  const numbers: Partial<Record<SessionField, number>> = {}
  const shownFields = visibleFields(draft.route === 'recorded' ? 'recorded' : 'live', windows)
  for (const field of NUMERIC_FIELDS) {
    const text = draft[field].trim()
    const value = Number(text)
    if (text && Number.isFinite(value)) numbers[field] = value
    else if (shownFields.has(field)) errors[field] = 'Enter a number.'
    else numbers[field] = fallback[field] as number
  }
  if (Object.keys(errors).length) return { values: null, errors }
  const text = (field: SessionField) => draft[field].trim()
  return {
    errors,
    values: {
      route: draft.route === 'recorded' ? 'recorded' : 'live',
      robot: asSetupRobot(draft.robot),
      session_dir: text('session_dir'),
      human_height_m: numbers.human_height_m!,
      mocap_ws: text('mocap_ws'),
      motion_bvh: text('motion_bvh'),
      npose_bvh: text('npose_bvh'),
      offsets_file: draft.offsets_file,
      bvh_scale: numbers.bvh_scale!,
      start_frame: numbers.start_frame!,
      end_frame: numbers.end_frame!,
      frame_step: numbers.frame_step!,
      npose_start_s: numbers.npose_start_s!,
      npose_duration_s: numbers.npose_duration_s!,
      windows_xsens_ip: text('windows_xsens_ip'),
      linux_xsens_ip: text('linux_xsens_ip'),
    },
  }
}

function sameDraft(a: Draft, b: Draft): boolean {
  return SESSION_FIELDS.every((field) => a[field].trim() === b[field].trim())
}

// ---------- Jobs ----------

interface LogEntry {
  job: JobSummary
  lines: string[]
}

/**
 * The setup list and polled jobs merged: a finished state always wins.  A
 * job the console no longer knows (``lost``) is shown as stopped so it can
 * never hold the UI busy.
 */
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

function jobLed(job: JobSummary | undefined): Led {
  if (!job) return 'off'
  if (job.state === 'running') return 'active'
  if (job.state === 'passed') return 'passed'
  if (job.state === 'failed') return 'fault'
  // Stopping the live simulation is how it normally ends.
  return VIEWER_KINDS.includes(job.kind) ? 'passed' : 'off'
}

function hasFile(files: SetupFile[], name: string): boolean {
  return files.some((file) => file.name === name)
}

function stepLeds(setup: SetupState, jobs: JobSummary[], route: SetupRoute): Record<SetupStep, Led> {
  const latest = (kind: string) => jobs.find((job) => job.kind === kind)
  const robot = setup.session?.robot ?? setup.defaults.robot
  const orFile = (led: Led, ...names: string[]): Led => led === 'off' && names.some((name) => hasFile(setup.files, name)) ? 'passed' : led
  const leds: Record<SetupStep, Led> = {
    session: setup.session ? 'passed' : 'off',
    windows: jobLed(latest('acceptance')) === 'passed' ? 'passed' : 'off',
    stream: jobLed(latest('acceptance')),
    live: jobLed(jobs.find((job) => VIEWER_KINDS.includes(job.kind))),
    bvh: jobLed(latest('bvh_check')),
    retarget: orFile(jobLed(latest('retarget')), `${robot}-preview.pkl`, `${robot}-npose.pkl`),
    render: orFile(jobLed(latest('render')), `${robot}-preview.mp4`, `${robot}-npose.mp4`),
    finish: jobLed(latest('save')),
    k1: jobLed(latest('k1_install')),
  }
  // The first unstarted step is next, unless a step failed: then the failed
  // step is what needs attention.  The optional K1 step is never "next".
  const routeSteps = stepsForRoute(route)
  if (!routeSteps.some((step) => leds[step] === 'fault')) {
    const next = routeSteps.find((step) => leds[step] === 'off')
    if (next) leds[next] = 'next'
  }
  return leds
}

const LED_WORD: Record<Led, string> = { passed: 'Done', next: 'Next', active: 'Running', fault: 'Failed', off: '' }

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

function sizeText(bytes: number): string {
  if (bytes >= 1024 * 1024) return `${(bytes / (1024 * 1024)).toFixed(1)} MB`
  if (bytes >= 1024) return `${Math.round(bytes / 1024)} KB`
  return `${bytes} B`
}

function servable(name: string): boolean {
  return /\.(mp4|json|txt|log)$/i.test(name)
}

// ---------- Small parts ----------

export function CopyButton({ value, label }: { value: string; label: string }) {
  const [state, setState] = useState<'idle' | 'copied' | 'failed'>('idle')

  useEffect(() => {
    if (state === 'idle') return
    const timer = window.setTimeout(() => setState('idle'), 2500)
    return () => window.clearTimeout(timer)
  }, [state])

  async function copy() {
    try {
      if (!navigator.clipboard) throw new Error('Clipboard unavailable')
      await navigator.clipboard.writeText(value)
      setState('copied')
    } catch {
      setState('failed')
    }
  }

  return (
    <button type="button" className={`panel-key copy-key ${state === 'failed' ? 'is-failed' : ''}`} onClick={copy} aria-label={`Copy ${label}`} disabled={!value}>
      {state === 'copied' ? <Check aria-hidden="true" /> : <Copy aria-hidden="true" />}
      <span aria-live="polite">{state === 'copied' ? 'Copied' : state === 'failed' ? 'Select and copy' : 'Copy'}</span>
    </button>
  )
}

function ValuePlate({ label, value, note, copy = true }: { label: string; value: string; note?: string; copy?: boolean }) {
  return (
    <div className="value-plate">
      <span className="silk">{label}</span>
      <strong className="mono">{value || '— not set'}</strong>
      {note && <small>{note}</small>}
      {copy && value && <CopyButton value={value} label={label} />}
    </div>
  )
}

function Verdict({ job, stopping = false }: { job: JobSummary; stopping?: boolean }) {
  const word = job.state === 'running' ? stopping ? 'STOPPING' : 'RUNNING' : job.state === 'passed' ? 'PASS' : job.state === 'failed' ? 'FAIL' : 'STOPPED'
  return (
    <div className={`verdict-plate is-${job.state}`} role="status" aria-label={`${job.label}: ${word}`}>
      <strong>{word}</strong>
      <span>
        <b>{job.label}</b>
        {job.summary && <small className="mono">{job.summary}</small>}
        {job.state === 'failed' && job.exit_code !== null && job.exit_code !== 0 && <small className="mono">Exit code {job.exit_code}</small>}
      </span>
    </div>
  )
}

function JobConsole({ job, lines, nowMs, stopping, onStop, progressSeconds, showStop = true }: {
  job: JobSummary
  lines: string[] | undefined
  nowMs: number
  stopping: boolean
  onStop: (job: JobSummary) => void
  progressSeconds?: number
  /** False when the step already shows its own prominent Stop key. */
  showStop?: boolean
}) {
  const logRef = useRef<HTMLPreElement>(null)
  const count = lines?.length ?? 0
  useEffect(() => {
    const element = logRef.current
    if (element) element.scrollTop = element.scrollHeight
  }, [count])
  const elapsed = elapsedSeconds(job, nowMs)
  const running = job.state === 'running'
  return (
    <div className="job-console">
      <Verdict job={job} stopping={stopping && running} />
      <div className="job-meta">
        <Lamp state={jobLed(job) === 'off' ? 'off' : jobLed(job)} />
        <span className="mono">{progressSeconds && running ? `${Math.min(progressSeconds, Math.floor(elapsed))} / ${progressSeconds} s` : clockText(elapsed)}</span>
        {progressSeconds && running ? (
          <div className="progress-track job-progress" role="progressbar" aria-label="Check progress" aria-valuemin={0} aria-valuemax={progressSeconds} aria-valuenow={Math.min(progressSeconds, Math.floor(elapsed))}>
            <i style={{ width: `${Math.min(100, (elapsed / progressSeconds) * 100)}%` }} />
          </div>
        ) : <span className="job-spacer" />}
        {running && showStop && (
          <button type="button" className="panel-key" disabled={stopping} onClick={() => onStop(job)}>
            {stopping ? <RefreshCw className="spin" aria-hidden="true" /> : <Square aria-hidden="true" />}
            {stopping ? 'Stopping…' : 'Stop'}
          </button>
        )}
      </div>
      <pre ref={logRef} className="job-log mono" role="log" aria-label={`${job.label} log`} tabIndex={0}>
        {count ? lines!.join('\n') : running ? 'Waiting for output…' : lines ? 'No output.' : 'Loading log…'}
      </pre>
      {job.outputs.length > 0 && <p className="group-note">Saved: <span className="mono">{job.outputs.join(', ')}</span></p>}
    </div>
  )
}

function ActionError({ message }: { message: string | null }) {
  if (!message) return null
  return <p className="setup-error" role="alert"><AlertTriangle aria-hidden="true" />{message}</p>
}

function FieldError({ id, message }: { id: string; message?: string }) {
  return message ? <span className="field-error" id={id} role="alert">{message}</span> : null
}

// ---------- View ----------

export interface SetupViewProps {
  client: SetupClient
  initial: SetupState
  hidden?: boolean
}

export function SetupView({ client, initial, hidden = false }: SetupViewProps) {
  const [setup, setSetup] = useState(initial)
  const [step, setStep] = useState<SetupStep>(() => {
    const hashed = parseSetupHash(window.location.hash)?.step
    if (hashed) return hashed
    const route = initial.session?.route ?? initial.defaults.route
    const leds = stepLeds(initial, initial.jobs, route)
    const routeSteps = stepsForRoute(route)
    return routeSteps.find((candidate) => leds[candidate] === 'next') ?? routeSteps.find((candidate) => leds[candidate] === 'fault') ?? 'session'
  })
  const [draft, setDraft] = useState<Draft>(() => initialDraft(initial, STEP_META[parseSetupHash(window.location.hash)?.step ?? 'session'].route))
  const [xsensHost, setXsensHost] = useState<XsensHost>(() => xsensHostOf(initial, initialDraft(initial, null)))
  const [fieldErrors, setFieldErrors] = useState<FieldErrors>({})
  const [formError, setFormError] = useState<string | null>(null)
  const [confirmReplace, setConfirmReplace] = useState<string | null>(null)
  const [saving, setSaving] = useState(false)
  const [savedNotice, setSavedNotice] = useState<string | null>(null)
  const [framesOpen, setFramesOpen] = useState(false)
  const [logs, setLogs] = useState<Record<string, LogEntry>>({})
  const [starting, setStarting] = useState<string | null>(null)
  // Jobs whose stop was requested; shown as stopping until polling reports a final state.
  const [stoppingIds, setStoppingIds] = useState<ReadonlySet<string>>(() => new Set())
  const [lostJobs, setLostJobs] = useState<ReadonlySet<string>>(() => new Set())
  const [actionError, setActionError] = useState<{ step: SetupStep; message: string; code: string | null } | null>(null)
  const [stoppingViewer, setStoppingViewer] = useState(false)
  const [viewerNotice, setViewerNotice] = useState<string | null>(null)
  const [refreshError, setRefreshError] = useState<string | null>(null)
  const [nowMs, setNowMs] = useState(() => Date.now())
  const [autoCalibrate, setAutoCalibrate] = useState(false)
  const [calibrateAfter, setCalibrateAfter] = useState('5')
  const [liveVideo, setLiveVideo] = useState(false)
  // null until the operator ticks or unticks it: then the default follows the K1 tools.
  const [robotStream, setRobotStream] = useState<boolean | null>(null)
  const [controlPending, setControlPending] = useState<ControlCommand | null>(null)
  const [controlError, setControlError] = useState<string | null>(null)
  const [renderMotion, setRenderMotion] = useState<'preview' | 'npose'>('preview')
  const [playing, setPlaying] = useState<string | null>(null)
  const [k1, setK1] = useState<K1Status | null>(null)
  const [k1Error, setK1Error] = useState<string | null>(null)
  const [k1Opening, setK1Opening] = useState(false)
  const [k1Result, setK1Result] = useState<{ outcome: K1OpenOutcome; message: string } | null>(null)
  // One desktop password prompt at a time (firewall rule or Tk).
  const [adminBusy, setAdminBusy] = useState<PrivilegedAction | null>(null)
  const [adminResult, setAdminResult] = useState<{ action: PrivilegedAction; outcome: PrivilegedOutcome | 'error'; message: string } | null>(null)
  const [confirmShutdown, setConfirmShutdown] = useState(false)
  const [shutdownState, setShutdownState] = useState<'idle' | 'sending' | 'done'>('idle')
  const [shutdownError, setShutdownError] = useState<string | null>(null)
  const cursorRef = useRef(new Map<string, number>())
  // One log read per job at a time, so each line range is appended once.
  const inflightRef = useRef(new Map<string, Promise<unknown>>())
  const requestedRef = useRef(new Set<string>())
  const aliveRef = useRef(true)

  useEffect(() => {
    aliveRef.current = true
    return () => { aliveRef.current = false }
  }, [])

  const refresh = useCallback(async () => {
    try {
      const next = await client.getSetup()
      if (!aliveRef.current) return
      if (next) {
        setSetup(next)
        setRefreshError(null)
      } else {
        setRefreshError('The Setup workbench is no longer available. Restart the console with --setup.')
      }
    } catch (reason) {
      if (aliveRef.current) setRefreshError(errorMessage(reason, 'Setup state could not be refreshed.'))
    }
  }, [client])

  const windows = setup.platform === 'windows'
  const route: SetupRoute = draft.route === 'recorded' ? 'recorded' : 'live'
  const steps = stepsForRoute(route, setup.desktop_tools)
  const jobs = mergeJobs(setup.jobs, logs, lostJobs)
  const runningJob = jobs.find((job) => job.state === 'running') ?? null
  const runningId = runningJob?.id ?? null
  const leds = stepLeds(setup, jobs, route)
  const session = setup.session
  const savedDraft = session ? toDraft(session, setup.home) : null
  const shown = (path: string) => tildePath(path, setup.home)
  const dirty = !savedDraft || !sameDraft(savedDraft, draft)

  // Keep the URL on the visible step so a reload (or a screenshot harness)
  // returns to it.  replaceState never fires hashchange.
  useEffect(() => {
    if (hidden) return
    const target = `#setup/${step}`
    if (window.location.hash !== target) window.history.replaceState(null, '', target)
  }, [hidden, step])

  useEffect(() => {
    const onHash = () => {
      const parsed = parseSetupHash(window.location.hash)
      if (!parsed?.step) return
      const selected = parsed.step
      setStep(selected)
      const stepRoute = STEP_META[selected].route
      if (stepRoute) setDraft((current) => current.route === stepRoute ? current : { ...current, route: stepRoute })
    }
    window.addEventListener('hashchange', onHash)
    return () => window.removeEventListener('hashchange', onHash)
  }, [])

  /**
   * Append the job's new log lines.  Reads for one job are chained: each
   * starts from the cursor the previous one left, so a poll tick and the
   * read after Stop never append the same range twice.
   */
  const readMore = useCallback((id: string) => {
    const previous = inflightRef.current.get(id) ?? Promise.resolve()
    const next = previous.catch(() => undefined).then(async () => {
      const log = await client.getJobLog(id, cursorRef.current.get(id) ?? 0)
      cursorRef.current.set(id, log.next)
      if (aliveRef.current) {
        setLogs((current) => ({
          ...current,
          [id]: { job: log.job, lines: [...(current[id]?.lines ?? []), ...log.lines].slice(-5000) },
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

  // Poll the running job.  The loop ends when the job reports a final state;
  // the setup state is then re-read for files, outputs and readiness.
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
          // The console restarted and no longer knows this job.
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

  const stepJob = jobs.find((job) => STEP_META[step].kinds.includes(job.kind))
  const stepJobId = stepJob && stepJob.state !== 'running' && !logs[stepJob.id] ? stepJob.id : null

  // A finished job from an earlier visit: load its log once.
  useEffect(() => {
    if (!stepJobId || requestedRef.current.has(stepJobId)) return
    requestedRef.current.add(stepJobId)
    client.getJobLog(stepJobId, 0).then((log) => {
      if (!aliveRef.current) return
      cursorRef.current.set(stepJobId, log.next)
      setLogs((current) => current[stepJobId] ? current : { ...current, [stepJobId]: { job: log.job, lines: log.lines } })
    }, () => {
      if (aliveRef.current) setLogs((current) => current[stepJobId] ? current : { ...current, [stepJobId]: { job: stepJob!, lines: [] } })
    })
  }, [client, stepJobId, stepJob])

  const refreshK1 = useCallback(async () => {
    try {
      const status = await client.getK1()
      if (!aliveRef.current) return
      setK1(status)
      setK1Error(null)
    } catch (reason) {
      if (aliveRef.current) setK1Error(errorMessage(reason, 'The K1 tools could not be checked.'))
    }
  }, [client])

  // Check the K1 tools when the step opens and each time an install ends.
  const k1InstallState = jobs.find((job) => job.kind === 'k1_install')?.state ?? null
  useEffect(() => {
    if ((step !== 'k1' && step !== 'live') || !setup.desktop_tools || k1InstallState === 'running') return
    let active = true
    client.getK1().then((status) => {
      if (!active || !aliveRef.current) return
      setK1(status)
      setK1Error(null)
    }, (reason: unknown) => {
      if (active && aliveRef.current) setK1Error(errorMessage(reason, 'The K1 tools could not be checked.'))
    })
    return () => { active = false }
  }, [client, step, setup.desktop_tools, k1InstallState])

  async function openK1Window() {
    setK1Opening(true)
    setK1Result(null)
    try {
      const result = await client.openK1()
      if (!aliveRef.current) return
      setK1Result(result)
      void refreshK1()
    } catch (reason) {
      if (aliveRef.current) setK1Result({ outcome: 'failed', message: errorMessage(reason, 'The K1 test window did not open.') })
    } finally {
      if (aliveRef.current) setK1Opening(false)
    }
  }

  async function runAdmin(action: PrivilegedAction) {
    setAdminBusy(action)
    setAdminResult(null)
    try {
      const result = await client.runPrivileged(action)
      if (!aliveRef.current) return
      setAdminResult({ action, ...result })
      if (action === 'install_tk' && result.outcome === 'done') void refreshK1()
    } catch (reason) {
      if (aliveRef.current) setAdminResult({ action, outcome: 'error', message: errorMessage(reason, 'The password prompt could not be opened. Use Copy and a terminal.') })
    } finally {
      if (aliveRef.current) setAdminBusy(null)
    }
  }

  async function shutdownConsole() {
    setShutdownState('sending')
    setShutdownError(null)
    try {
      await client.shutdown()
      if (aliveRef.current) setShutdownState('done')
    } catch (reason) {
      if (!aliveRef.current) return
      setShutdownState('idle')
      setShutdownError(errorMessage(reason, 'The console did not shut down.'))
    }
  }

  function selectStep(next: SetupStep) {
    setStep(next)
    setActionError(null)
    setViewerNotice(null)
  }

  function selectRoute(next: SetupRoute) {
    setDraft((current) => ({ ...current, route: next }))
    const shownFields = visibleFields(next, windows)
    setFieldErrors((current) => Object.fromEntries(Object.entries(current).filter(([name]) => shownFields.has(name as SessionField))))
    if (STEP_META[step].route && STEP_META[step].route !== next) setStep('session')
  }

  function setField(field: SessionField, value: string) {
    setDraft((current) => ({ ...current, [field]: value }))
    setSavedNotice(null)
    if (fieldErrors[field]) setFieldErrors((current) => ({ ...current, [field]: undefined }))
  }

  /** Windows: choose where Xsens Humanoid runs and set the two IP fields to match. */
  function selectXsensHost(next: XsensHost) {
    setXsensHost(next)
    setSavedNotice(null)
    setFieldErrors((current) => ({ ...current, windows_xsens_ip: undefined, linux_xsens_ip: undefined }))
    setDraft((current) => {
      if (next === 'this') return { ...current, windows_xsens_ip: LOOPBACK_IP, linux_xsens_ip: '' }
      const sourceIp = isLoopbackIp(current.windows_xsens_ip) ? '' : current.windows_xsens_ip
      return { ...current, windows_xsens_ip: sourceIp, linux_xsens_ip: current.linux_xsens_ip || setup.network.detected_linux_ip || '' }
    })
  }

  async function saveSession(replace = false) {
    const { values, errors } = fromDraft(draft, setup.session ?? setup.defaults, windows)
    setFormError(null)
    setSavedNotice(null)
    setConfirmReplace(null)
    if (!values) {
      setFieldErrors(errors)
      if (Object.keys(errors).some((field) => FRAME_FIELDS.has(field as SessionField))) setFramesOpen(true)
      return
    }
    setFieldErrors({})
    setSaving(true)
    try {
      const next = await client.saveSession(values, replace)
      if (!aliveRef.current) return
      setSetup(next)
      setDraft(next.session ? toDraft(next.session, next.home) : draft)
      if (next.session && next.platform === 'windows') setXsensHost(xsensHostOf(next, toDraft(next.session, next.home)))
      setSavedNotice(`Saved ${tildePath((next.session ?? values).session_dir, next.home)}/session.env`)
    } catch (reason) {
      if (!aliveRef.current) return
      const message = errorMessage(reason, 'The session was not saved.')
      const field = reason instanceof SetupApiError ? normalizeField(reason.field) : null
      if (reason instanceof SetupApiError && reason.code === 'session_exists') {
        setConfirmReplace(message)
      } else if (field && visibleFields(route, windows).has(field)) {
        setFieldErrors({ [field]: message })
        if (FRAME_FIELDS.has(field)) setFramesOpen(true)
      } else {
        setFormError(message)
      }
    } finally {
      if (aliveRef.current) setSaving(false)
    }
  }

  function newSession() {
    const next = toDraft(setup.defaults, setup.home)
    next.route = draft.route
    if (windows) {
      const host = xsensHostOf(setup, next)
      if (host === 'this') {
        next.windows_xsens_ip = LOOPBACK_IP
        next.linux_xsens_ip = ''
      } else if (!next.linux_xsens_ip) {
        next.linux_xsens_ip = draft.linux_xsens_ip
      }
      setXsensHost(host)
    } else if (!next.linux_xsens_ip) {
      next.linux_xsens_ip = draft.linux_xsens_ip
    }
    setDraft(next)
    setFieldErrors({})
    setFormError(null)
    setConfirmReplace(null)
    setSavedNotice(null)
  }

  async function stopLeftoverViewer(owner: SetupStep) {
    setStoppingViewer(true)
    try {
      const stopped = await client.stopViewer()
      if (!aliveRef.current) return
      setActionError(null)
      setViewerNotice(stopped ? 'Leftover viewer stopped. Try again.' : 'No leftover viewer was running. Try again.')
    } catch (reason) {
      if (aliveRef.current) setActionError({ step: owner, message: errorMessage(reason, 'The leftover viewer could not be stopped.'), code: null })
    } finally {
      if (aliveRef.current) setStoppingViewer(false)
    }
  }

  async function startJob(owner: SetupStep, kind: JobKind, options: JobOptions = {}) {
    setActionError(null)
    setViewerNotice(null)
    setStarting(kind)
    try {
      const job = await client.startJob(kind, options)
      if (!aliveRef.current) return
      cursorRef.current.set(job.id, 0)
      requestedRef.current.add(job.id)
      setLogs((current) => ({ ...current, [job.id]: { job, lines: [] } }))
    } catch (reason) {
      if (!aliveRef.current) return
      setActionError({ step: owner, message: errorMessage(reason, 'The job did not start.'), code: reason instanceof SetupApiError ? reason.code : null })
      if (reason instanceof SetupApiError && (reason.code === 'job_running' || reason.code === 'no_session')) void refresh()
    } finally {
      if (aliveRef.current) setStarting(null)
    }
  }

  /**
   * The console answers a stop request at once; the job can keep running
   * for a while as it shuts down.  It shows as stopping, and polling carries
   * on until the job reports a final state.
   */
  async function stopJob(job: JobSummary) {
    setStoppingIds((current) => new Set(current).add(job.id))
    try {
      const stopped = await client.stopJob(job.id)
      if (!aliveRef.current || stopped.state === 'running') return
      // Already final: polling has ended, so read the last lines here.
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
      setActionError({ step, message: errorMessage(reason, 'The job could not be stopped.'), code: null })
    }
  }
  const isStopping = (id: string) => stoppingIds.has(id)

  const busy = runningJob !== null || starting !== null
  const checkSeconds = Math.round(setup.stream_check_s)
  const meta: StepMeta = { ...STEP_META[step], ...(windows ? WINDOWS_STEP_META[step] : undefined) }
  // The Linux engineer-guide numbers mean nothing on Windows: number the visible steps instead.
  const stepLabel = (item: SetupStep) => windows ? String(steps.indexOf(item) + 1) : STEP_META[item].guide
  const stepWord = windows && steps.includes(step) ? String(steps.indexOf(step) + 1) : meta.guide
  const stepActionError = actionError?.step === step ? actionError : null
  const stepError = stepActionError && (
    <>
      <ActionError message={stepActionError.message} />
      {stepActionError.code === 'recorded_runtime_missing' && setup.desktop_tools && (
        <div className="setup-actions">
          <button type="button" className="panel-key" onClick={() => selectStep('k1')}>
            <Download aria-hidden="true" />Install it in step 13 · Real K1
          </button>
        </div>
      )}
      {stepActionError.code === 'viewer_running' && (
        <div className="setup-actions">
          <button type="button" className="panel-key" disabled={stoppingViewer} onClick={() => void stopLeftoverViewer(step)}>
            {stoppingViewer ? <RefreshCw className="spin" aria-hidden="true" /> : <Square aria-hidden="true" />}
            Stop leftover viewer
          </button>
        </div>
      )}
    </>
  )
  const stepNotice = viewerNotice && <p className="saved-note" role="status"><Lamp state="passed" /><span>{viewerNotice}</span></p>
  const noDisplay = setup.readiness.display === false
  const displayNote = noDisplay && (
    <p className="setup-callout is-warning"><AlertTriangle aria-hidden="true" /><span>{setup.readiness.display_hint ?? (windows ? 'No desktop display: viewer windows cannot open.' : 'No desktop display: open the console on this PC, not over SSH. Viewer windows cannot open.')}</span></p>
  )
  const robotName = ROBOTS.find((robot) => robot.value === (session?.robot ?? draft.robot))?.name ?? 'Robot'
  const robotId = session?.robot ?? asSetupRobot(draft.robot)
  const routeMismatch = session !== null && session.route !== route

  const consoleFor = (job: JobSummary | undefined, progressSeconds?: number, showStop = true) => job ? (
    <JobConsole job={job} lines={logs[job.id]?.lines} nowMs={nowMs} stopping={isStopping(job.id)} onStop={stopJob} progressSeconds={progressSeconds} showStop={showStop} />
  ) : null

  const runKey = (owner: SetupStep, kind: JobKind, label: string, icon: ReactNode, options: JobOptions = {}, primary = true, disabled = false) => (
    <button type="button" className={`panel-key setup-run ${primary ? 'is-primary' : ''}`} disabled={busy || !session || disabled} onClick={() => void startJob(owner, kind, options)}>
      {starting === kind ? <RefreshCw className="spin" aria-hidden="true" /> : icon}
      {label}
    </button>
  )

  const noSessionNote = !session && step !== 'session' && step !== 'windows' && step !== 'k1' && (
    <p className="setup-callout is-warning"><AlertTriangle aria-hidden="true" /><span>Save a session first ({windows ? 'Session step' : 'step 03'}). The buttons below stay locked until then.</span></p>
  )

  const field = (name: SessionField, label: string, input: ReactNode) => (
    <label data-field={name}>
      {label}
      {input}
      <FieldError id={`setup-error-${name}`} message={fieldErrors[name]} />
    </label>
  )

  const textInput = (name: SessionField, props: { mono?: boolean; placeholder?: string; type?: string; step?: string; inputMode?: 'decimal' | 'numeric' } = {}) => (
    <input
      className={props.mono ? 'mono' : undefined}
      type={props.type ?? 'text'}
      step={props.step}
      inputMode={props.inputMode}
      value={draft[name]}
      placeholder={props.placeholder}
      aria-invalid={fieldErrors[name] ? true : undefined}
      aria-describedby={fieldErrors[name] ? `setup-error-${name}` : undefined}
      spellCheck={false}
      onChange={(event) => setField(name, event.target.value)}
    />
  )

  const interfaces = setup.network.interfaces
  const linuxIpListed = interfaces.some((item) => item.ipv4 === draft.linux_xsens_ip)

  const ipSelect = (
    <select
      className="mono"
      value={draft.linux_xsens_ip}
      aria-invalid={fieldErrors.linux_xsens_ip ? true : undefined}
      aria-describedby={fieldErrors.linux_xsens_ip ? 'setup-error-linux_xsens_ip' : undefined}
      onChange={(event) => setField('linux_xsens_ip', event.target.value)}
    >
      <option value="">Not set yet</option>
      {!linuxIpListed && draft.linux_xsens_ip && <option value={draft.linux_xsens_ip}>{draft.linux_xsens_ip}</option>}
      {interfaces.map((item) => (
        <option key={`${item.name}-${item.ipv4}`} value={item.ipv4}>
          {item.ipv4} · {item.name}{item.ipv4 === setup.network.detected_linux_ip ? ' (detected)' : ''}
        </option>
      ))}
    </select>
  )

  function renderSession() {
    return (
      <>
        <form className="setup-form" onSubmit={(event) => { event.preventDefault(); void saveSession() }} noValidate>
          <fieldset className="group">
            <legend className="silk">Robot and performer</legend>
            <div className="choice-cards" role="radiogroup" aria-label="Robot">
              {ROBOTS.map((robot) => (
                <label key={robot.value} className={draft.robot === robot.value ? 'selected' : ''}>
                  <input type="radio" name="setup-robot" value={robot.value} checked={draft.robot === robot.value} onChange={() => setField('robot', robot.value)} />
                  <span><strong>{robot.name}</strong><small>{robot.detail}</small></span>
                </label>
              ))}
            </div>
            <div className="form-grid">
              {field('human_height_m', 'Performer height (m)', textInput('human_height_m', { mono: true, type: 'number', step: '0.01', inputMode: 'decimal' }))}
              {field('session_dir', 'Session folder', textInput('session_dir', { mono: true, placeholder: '~/mocap-sessions/<date-time>' }))}
            </div>
            <p className="group-note">Measure the performer. The session folder is created for you.</p>
          </fieldset>

          {route === 'live' ? (
            windows ? (
              <fieldset className="group">
                <legend className="silk">Network · live route</legend>
                <div className="units" role="radiogroup" aria-label="Xsens Humanoid runs on">
                  <span className="silk">Xsens Humanoid runs on</span>
                  {([{ value: 'this', label: 'This PC' }, { value: 'other', label: 'Another PC' }] as const).map((option) => (
                    <label key={option.value} className={xsensHost === option.value ? 'selected' : ''}>
                      <input type="radio" name="setup-xsens-host" value={option.value} checked={xsensHost === option.value} onChange={() => selectXsensHost(option.value)} />
                      {option.label}
                    </label>
                  ))}
                </div>
                {xsensHost === 'this' ? (
                  <p className="group-note">In Xsens Humanoid set Network Streamer destination <code>{LOOPBACK_IP}</code>, port <code>{STREAM_PORT}</code>.</p>
                ) : (
                  <>
                    <div className="form-grid">
                      {field('windows_xsens_ip', 'Xsens PC IP', textInput('windows_xsens_ip', { mono: true, placeholder: 'e.g. 192.168.1.246' }))}
                      {field('linux_xsens_ip', "This PC's IP", ipSelect)}
                    </div>
                    <p className="group-note">Run <code>ipconfig</code> on each PC: use the IPv4 address of the adapter on the network both PCs share. Both may stay empty until you know them.</p>
                  </>
                )}
                <div className="form-grid wide-fields">
                  {field('mocap_ws', 'Runtime folder', textInput('mocap_ws', { mono: true, placeholder: '~/mocap-runtime' }))}
                </div>
              </fieldset>
            ) : (
            <fieldset className="group">
              <legend className="silk">Network · live route</legend>
              <div className="form-grid">
                {field('windows_xsens_ip', 'Windows Xsens IP', textInput('windows_xsens_ip', { mono: true, placeholder: 'e.g. 192.168.1.246' }))}
                {field('linux_xsens_ip', 'Linux Xsens IP', ipSelect)}
              </div>
              <p className="group-note">Windows IP: run <code>ipconfig</code> on Windows and use the Xsens Ethernet adapter. Linux IP: the adapter on the Xsens router. Both may stay empty until you know them.</p>
            </fieldset>
            )
          ) : (
            <fieldset className="group">
              <legend className="silk">Recorded files</legend>
              <div className="form-grid wide-fields">
                {field('mocap_ws', 'Runtime folder', textInput('mocap_ws', { mono: true, placeholder: '~/mocap-runtime' }))}
                {field('motion_bvh', 'Motion BVH', textInput('motion_bvh', { mono: true, placeholder: '~/mocap-data/motion.bvh' }))}
                {field('npose_bvh', 'Neutral-pose BVH', textInput('npose_bvh', { mono: true, placeholder: '~/mocap-data/neutral.bvh' }))}
              </div>
              <div className="units" role="radiogroup" aria-label="BVH units">
                <span className="silk">BVH units</span>
                {[{ value: '0.01', label: 'Centimetres (0.01)' }, { value: '1', label: 'Metres (1)' }].map((unit) => (
                  <label key={unit.value} className={Number(draft.bvh_scale) === Number(unit.value) ? 'selected' : ''}>
                    <input type="radio" name="setup-units" checked={Number(draft.bvh_scale) === Number(unit.value)} onChange={() => setField('bvh_scale', unit.value)} />
                    {unit.label}
                  </label>
                ))}
                <FieldError id="setup-error-bvh_scale" message={fieldErrors.bvh_scale} />
              </div>
              <details className="setup-details" open={framesOpen} onToggle={(event) => setFramesOpen(event.currentTarget.open)}>
                <summary><span>Frames and neutral window</span><small className="mono">frames {draft.start_frame}–{draft.end_frame}, step {draft.frame_step} · neutral {draft.npose_start_s} s for {draft.npose_duration_s} s</small><ChevronDown aria-hidden="true" /></summary>
                <div className="form-grid">
                  {field('start_frame', 'Start frame', textInput('start_frame', { mono: true, type: 'number', step: '1', inputMode: 'numeric' }))}
                  {field('end_frame', 'End frame', textInput('end_frame', { mono: true, type: 'number', step: '1', inputMode: 'numeric' }))}
                  {field('frame_step', 'Frame step', textInput('frame_step', { mono: true, type: 'number', step: '1', inputMode: 'numeric' }))}
                  {field('npose_start_s', 'Neutral start (s)', textInput('npose_start_s', { mono: true, type: 'number', step: '0.1', inputMode: 'decimal' }))}
                  {field('npose_duration_s', 'Neutral length (s)', textInput('npose_duration_s', { mono: true, type: 'number', step: '0.1', inputMode: 'decimal' }))}
                </div>
                <p className="group-note">Change these only if your take needs it.</p>
              </details>
            </fieldset>
          )}

          {confirmReplace && (
            <div className="setup-callout is-warning" role="alert">
              <AlertTriangle aria-hidden="true" />
              <span>{confirmReplace}</span>
              <button type="button" className="panel-key" disabled={saving} onClick={() => void saveSession(true)}>Replace it</button>
            </div>
          )}
          <ActionError message={formError} />

          <div className="setup-actions">
            <button type="submit" className="panel-key is-primary setup-run" disabled={saving}>
              {saving ? <RefreshCw className="spin" aria-hidden="true" /> : <Save aria-hidden="true" />}
              {saving ? 'Saving' : 'Save session'}
            </button>
            <button type="button" className="panel-key" onClick={newSession}><FilePlus aria-hidden="true" />New session</button>
            {savedNotice ? (
              <span className="saved-note" role="status"><Lamp state="passed" /><span className="mono">{savedNotice}</span></span>
            ) : session && !dirty ? (
              <span className="saved-note"><Lamp state="passed" /><span>Saved · <span className="mono">{shown(session.session_dir)}/session.env</span></span></span>
            ) : session ? (
              <span className="saved-note is-dirty"><Lamp state="next" /><span>Unsaved changes</span></span>
            ) : (
              <span className="saved-note is-dirty"><Lamp state="off" /><span>No session saved yet</span></span>
            )}
          </div>
        </form>

        <fieldset className="group">
          <legend className="silk">This PC</legend>
          <ul className="chain readiness">
            {route === 'recorded' ? (
              <>
                <li><Lamp state={readyLed(setup.readiness.recorded_runtime)} /><span>Recorded runtime<small className="mono">{draft.mocap_ws || 'not set'}</small></span><strong>{readyWord(setup.readiness.recorded_runtime)}</strong></li>
                {setup.readiness.recorded_runtime === false && setup.desktop_tools && (
                  <li><span /><span>Install it with the K1 test tools in step 13.</span><button type="button" className="panel-key" onClick={() => selectStep('k1')}><Download aria-hidden="true" />Go to step 13</button></li>
                )}
                <li><Lamp state={readyLed(setup.readiness.motion_bvh)} /><span>Motion BVH<small className="mono">{draft.motion_bvh || 'not set'}</small></span><strong>{readyWord(setup.readiness.motion_bvh)}</strong></li>
                <li><Lamp state={readyLed(setup.readiness.npose_bvh)} /><span>Neutral-pose BVH<small className="mono">{draft.npose_bvh || 'not set'}</small></span><strong>{readyWord(setup.readiness.npose_bvh)}</strong></li>
              </>
            ) : windows ? (
              <li><Lamp state={readyLed(setup.readiness.live_runtime)} /><span>Live runtime<small className="mono">{draft.mocap_ws || 'not set'}</small></span><strong>{readyWord(setup.readiness.live_runtime)}</strong></li>
            ) : (
              <li><Lamp state={readyLed(setup.readiness.live_image)} /><span>Live image<small>Checked by Run system check</small></span><strong>{readyWord(setup.readiness.live_image)}</strong></li>
            )}
          </ul>
          <div className="setup-actions">
            {runKey('session', 'doctor', 'Run system check', <Stethoscope aria-hidden="true" />, {}, false)}
            <span className="group-note">Runs the project checks for the {route} route.</span>
          </div>
          {stepError}{stepNotice}
          {consoleFor(stepJob)}
        </fieldset>
      </>
    )
  }

  function renderWindows() {
    const onThisPc = windows && (session && session.windows_xsens_ip ? isLoopbackIp(session.windows_xsens_ip) : xsensHost === 'this')
    const linuxIp = onThisPc ? LOOPBACK_IP : session?.linux_xsens_ip ?? ''
    const destinationLabel = windows ? "This PC's IP" : 'Linux Xsens IP'
    return (
      <>
        <fieldset className="group">
          <legend className="silk">Before streaming{windows ? '' : ' · guide step 04'}</legend>
          <ol className="setup-checklist">
            <li>Power the router, Body Pack and sensors. Check all sensors are healthy in Xsens.</li>
            <li>Enter the performer's <strong>measured</strong> body dimensions.</li>
            <li>Run the suit calibration. Stand still, raise both arms, bend the knees, walk and turn. Fix fit or calibration here first.</li>
          </ol>
        </fieldset>
        <fieldset className="group">
          <legend className="silk">Network Streamer{windows ? '' : ' · guide step 05'}</legend>
          <p className="group-note">In Xsens Humanoid open <strong>Options → Network Streamer</strong> (labels vary by version).</p>
          <div className="plate-row">
            <ValuePlate
              label="Destination"
              value={linuxIp}
              note={onThisPc ? 'Xsens runs on this PC' : linuxIp ? destinationLabel : `Set ${windows ? "This PC's IP in the Session step" : 'the Linux Xsens IP in step 03'}`}
            />
            <ValuePlate label="Port" value={STREAM_PORT} note="UDP" />
            <ValuePlate label="Protocol" value="UDP" copy={false} />
          </div>
          <ol className="setup-checklist">
            <li>Turn on <strong>Position + Orientation (Quaternion)</strong> with 23 body segments.</li>
            <li>Turn on the <strong>joint angles</strong> stream.</li>
            <li>Turn off props and fingers. Stream one character only.</li>
            <li>Start live preview or play back a recorded take.</li>
          </ol>
        </fieldset>
        {windows ? (
          <div className="setup-callout">
            <Lamp state="next" />
            <div>
              <span className="silk">Windows Firewall</span>
              {onThisPc ? (
                <p>Streaming to {LOOPBACK_IP} stays on this PC, so no firewall change is needed.</p>
              ) : (
                <p>When Windows asks, allow Python on <strong>Private networks</strong>. Or add an inbound rule for <strong>UDP {STREAM_PORT}</strong> in Windows Defender Firewall.</p>
              )}
            </div>
          </div>
        ) : (
        <div className="setup-callout">
          <Lamp state="next" />
          <div>
            <span className="silk">Linux firewall</span>
            {setup.firewall_command ? (
              <>
                <p>{FIREWALL_NOTE[setup.firewall_state ?? 'unknown']}</p>
                {setup.desktop_tools && (setup.firewall_state === 'active' || setup.firewall_state === 'unknown') && (
                  <div className="setup-actions">
                    <button type="button" className="panel-key is-primary" disabled={adminBusy !== null} onClick={() => void runAdmin('firewall')}>
                      {adminBusy === 'firewall' ? <RefreshCw className="spin" aria-hidden="true" /> : <ShieldCheck aria-hidden="true" />}
                      {adminBusy === 'firewall' ? 'Waiting for your password…' : 'Allow in firewall'}
                    </button>
                    <span className="group-note">Asks for your password, then adds the rule below.</span>
                  </div>
                )}
                {adminResult?.action === 'firewall' && <AdminOutcome result={adminResult} />}
                <p>Or run it in a terminal (needs your password):</p>
                <div className="command-line">
                  <code className="mono">{setup.firewall_command}</code>
                  <CopyButton value={setup.firewall_command} label="firewall command" />
                </div>
              </>
            ) : (
              <p>Save both IP addresses in step 03 to get the firewall command.</p>
            )}
          </div>
        </div>
        )}
      </>
    )
  }

  function renderStream() {
    // Same PC on Windows: the receiver is reached through loopback.
    const receiverIp = session?.linux_xsens_ip || (windows && session && isLoopbackIp(session.windows_xsens_ip) ? LOOPBACK_IP : '')
    return (
      <>
        <fieldset className="group">
          <legend className="silk">30-second stream check</legend>
          <div className="plate-row">
            <ValuePlate label={windows ? 'From Xsens' : 'From Windows'} value={session?.windows_xsens_ip ?? ''} copy={false} />
            <ValuePlate label={windows ? 'To this PC' : 'To Linux'} value={receiverIp ? `${receiverIp}:${STREAM_PORT}` : ''} copy={false} />
          </div>
          <div className="setup-actions">
            {runKey('stream', 'acceptance', `Run ${checkSeconds}-second check`, <Play aria-hidden="true" />)}
            <span className="group-note">Keep Xsens streaming until the verdict shows. The report is saved in the session folder.</span>
          </div>
          {stepError}{stepNotice}
          {consoleFor(stepJob, setup.stream_check_s)}
        </fieldset>
        <fieldset className="group">
          <legend className="silk">If it fails</legend>
          <table className="hint-table">
            <thead><tr><th>You see</th><th>Fix</th></tr></thead>
            <tbody>
              <tr><td className="mono">No UDP traffic</td><td>{windows ? 'Is Xsens streaming? Check the destination IP, port 5555 and the Windows Firewall note in the Xsens step.' : 'Is Xsens streaming? Check the destination IP, port 5555, the Ethernet cable and the firewall rule (step 05).'}</td></tr>
              <tr><td className="mono">Only MXTP12 / MXTP13</td><td>Turn on the quaternion and joint-angle streams, then start preview or playback.</td></tr>
              <tr><td className="mono">unexpected source</td><td>{windows
                ? "The Xsens source IP changed. Update it in the Session step and save. If Xsens runs on this PC, make sure its destination is 127.0.0.1, not this PC's LAN IP."
                : 'The Windows IP changed. Update it in step 03 and save.'}</td></tr>
              <tr><td className="mono">Address already in use</td><td>Another receiver is running. Stop it; run one receiver at a time.</td></tr>
            </tbody>
          </table>
          <p className="group-note">Do not continue on a failed check.</p>
        </fieldset>
      </>
    )
  }

  function renderLive() {
    const calibrate = Number(calibrateAfter)
    const calibrateInvalid = autoCalibrate && (!Number.isFinite(calibrate) || calibrate < 0.5 || calibrate > 60)
    const videos = setup.files.filter((file) => file.kind === 'video' && /-live/.test(file.name))
    const liveJob = jobs.find((job) => VIEWER_KINDS.includes(job.kind))
    const liveRunning = liveJob?.state === 'running'
    // The K1 window (Booster SDK) runs on the Ubuntu PC only.
    const streamable = robotId === 'booster_k1' && !windows
    const sendArms = streamable && (robotStream ?? k1?.installed === true)
    const live = liveRunning && liveJob?.kind === 'live' ? liveJob.live ?? null : null
    async function control(command: ControlCommand) {
      if (!liveJob || !liveRunning) return
      setControlError(null)
      setControlPending(command)
      try {
        await client.controlJob(liveJob.id, command)
      } catch (reason) {
        if (!aliveRef.current) return
        setControlError(reason instanceof SetupApiError && reason.status === 409
          ? 'The simulation is no longer running or reading commands.'
          : errorMessage(reason, 'The simulation did not accept the command.'))
      } finally {
        if (aliveRef.current) setControlPending(null)
      }
    }
    return (
      <>
        <fieldset className="group">
          <legend className="silk">Live simulation</legend>
          {displayNote}
          <div className="plate-row">
            <ValuePlate label="Robot" value={robotName} copy={false} />
            <ValuePlate label="Performer height" value={session ? `${session.human_height_m} m` : ''} copy={false} />
          </div>
          <div className="setup-options">
            <label className="check-option">
              <input type="checkbox" checked={autoCalibrate} onChange={(event) => setAutoCalibrate(event.target.checked)} />
              <span>Auto-calibrate after</span>
              <input className="mono short-input" type="number" step="0.5" min="0.5" max="60" value={calibrateAfter} disabled={!autoCalibrate} aria-label="Seconds before auto-calibration" aria-invalid={calibrateInvalid || undefined} onChange={(event) => setCalibrateAfter(event.target.value)} />
              <span>s</span>
            </label>
            <label className="check-option">
              <input type="checkbox" checked={liveVideo} onChange={(event) => setLiveVideo(event.target.checked)} />
              <span>Record video</span>
            </label>
            {streamable && (
              <label className="check-option">
                <input type="checkbox" checked={sendArms} onChange={(event) => setRobotStream(event.target.checked)} />
                <span>Send the arms to the K1 window <small>The K1 window's Live arms step moves the robot from this simulation.</small> <small className="mono">{ROBOT_STREAM_URL}</small></span>
              </label>
            )}
          </div>
          {calibrateInvalid && <span className="field-error" role="alert">Use 0.5 to 60 seconds.</span>}
          <div className="setup-actions">
            {runKey('live', 'live', 'Start live simulation', <Play aria-hidden="true" />, { calibrate_after_s: autoCalibrate ? calibrate : null, record_video: liveVideo, ...(streamable ? { robot_stream: sendArms } : {}) }, true, calibrateInvalid || noDisplay)}
            {setup.desktop_tools && runKey('live', 'compare', 'Start G1 + T1 side by side', <Columns2 aria-hidden="true" />, { calibrate_after_s: autoCalibrate ? calibrate : null }, false, calibrateInvalid || noDisplay)}
            {liveRunning && liveJob && (
              <button type="button" className="panel-key setup-run is-stop" disabled={isStopping(liveJob.id)} onClick={() => void stopJob(liveJob)}>
                {isStopping(liveJob.id) ? <RefreshCw className="spin" aria-hidden="true" /> : <Square aria-hidden="true" />}{isStopping(liveJob.id) ? 'Stopping…' : 'Stop'}
              </button>
            )}
          </div>
          {stepError}{stepNotice}
          {liveRunning && liveJob?.kind === 'live' && (
            <>
              <div className="live-tracking">
                <span className={`live-flag ${live?.calibrated ? 'is-on' : ''}`}>
                  <Lamp state={live?.calibrated ? 'passed' : 'off'} />
                  <span>Calibrated <strong>{live ? live.calibrated ? 'yes' : 'no' : '—'}</strong></span>
                </span>
                <span className={`live-flag ${live?.enabled ? 'is-on' : ''}`}>
                  <Lamp state={live?.enabled ? 'active' : 'off'} />
                  <span>Tracking on <strong>{live ? live.enabled ? 'yes' : 'no' : '—'}</strong>{live?.reason ? <small className="mono">{live.reason}</small> : null}</span>
                </span>
              </div>
              {sendArms && <p className="live-calibrate-note">Calibrate with the performer standing still, arms down, looking straight ahead: the K1 head takes its zero here. A new Calibrate stops a running K1 arm run; start it again in the K1 window.</p>}
              <div className="live-keys" role="group" aria-label="Simulation controls">
                <button type="button" className="panel-key" disabled={controlPending !== null || !live} onClick={() => void control('calibrate')}>
                  {controlPending === 'calibrate' ? <RefreshCw className="spin" aria-hidden="true" /> : <Crosshair aria-hidden="true" />}Calibrate<kbd>N</kbd>
                </button>
                <button type="button" className="panel-key" aria-pressed={live?.enabled ?? false} disabled={controlPending !== null || !live} onClick={() => void control('toggle_tracking')}>
                  {controlPending === 'toggle_tracking' ? <RefreshCw className="spin" aria-hidden="true" /> : <Power aria-hidden="true" />}{live?.enabled ? 'Tracking off' : 'Tracking on'}<kbd>Space</kbd>
                </button>
              </div>
              {!live && <p className="group-note">Calibrate and Tracking unlock with the first Xsens frame.</p>}
              {controlError && <span className="field-error" role="alert">{controlError}</span>}
            </>
          )}
          {consoleFor(stepJob, undefined, !(liveRunning && stepJob?.id === liveJob?.id))}
        </fieldset>
        <div className="live-columns">
          <fieldset className="group">
            <legend className="silk">MuJoCo window keys</legend>
            <p className="group-note">Use the Calibrate and Tracking buttons above, or click the MuJoCo window first and press a key. Calibrate once per session: the performer stands still, arms relaxed at the sides.</p>
            <dl className="key-sheet">
              <div><dt><kbd>N</kbd></dt><dd>Calibrate. Stand still in the neutral pose first.</dd></div>
              <div><dt><kbd>Space</kbd></dt><dd>Arm and waist tracking on or off.</dd></div>
              <div><dt><kbd>R</kbd></dt><dd>Reset from the current pose after a stream drop.</dd></div>
            </dl>
          </fieldset>
          <fieldset className="group">
            <legend className="silk">Recorded videos</legend>
            {videos.length ? (
              <ul className="file-list">
                {videos.map((file) => (
                  <li key={file.name}>
                    <button type="button" className="panel-key" aria-pressed={playing === file.name} onClick={() => setPlaying(playing === file.name ? null : file.name)}><Film aria-hidden="true" />{file.name}</button>
                    <small className="mono">{sizeText(file.size)}</small>
                  </li>
                ))}
              </ul>
            ) : <div className="empty-state"><span>No live videos yet. Tick Record video before you start.</span></div>}
            {playing && videos.some((file) => file.name === playing) && <video className="setup-video" controls src={client.fileUrl(playing)} />}
          </fieldset>
        </div>
      </>
    )
  }

  function renderBvh() {
    return (
      <>
        <fieldset className="group">
          <legend className="silk">{windows ? 'Export from Xsens' : 'Export on Windows'}</legend>
          <ol className="setup-checklist">
            <li>In Xsens, export the take as <strong>BVH</strong> with the <strong>3DS Max</strong> preset.</li>
            <li>Frame rate <strong>60 Hz</strong>. Turn <strong>Overwrite first frame</strong> off, if shown. Note the units.</li>
            <li>Export the neutral-pose take the same way.</li>
            <li>Copy both files to this PC at the paths below.</li>
          </ol>
        </fieldset>
        <fieldset className="group">
          <legend className="silk">Files on this PC</legend>
          <ul className="chain readiness">
            <li><Lamp state={readyLed(setup.readiness.motion_bvh)} /><span>Motion BVH<small className="mono">{session ? shown(session.motion_bvh) || 'not set' : 'not set'}</small></span><strong>{readyWord(setup.readiness.motion_bvh)}</strong></li>
            <li><Lamp state={readyLed(setup.readiness.npose_bvh)} /><span>Neutral-pose BVH<small className="mono">{session ? shown(session.npose_bvh) || 'not set' : 'not set'}</small></span><strong>{readyWord(setup.readiness.npose_bvh)}</strong></li>
          </ul>
          <div className="setup-actions">
            {runKey('bvh', 'bvh_check', 'Check files', <FileSearch aria-hidden="true" />)}
            <span className="group-note"><span className="mono">Frame Time: 0.0166667</span> means 60 Hz. The motion needs at least {session?.end_frame ?? 600} frames.</span>
          </div>
          {stepError}{stepNotice}
          {consoleFor(stepJob)}
        </fieldset>
      </>
    )
  }

  function renderRetarget() {
    return (
      <fieldset className="group">
        <legend className="silk">Robot motion</legend>
        <ul className="chain readiness">
          <li><Lamp state={hasFile(setup.files, `${robotId}-preview.pkl`) ? 'passed' : 'off'} /><span>Preview<small className="mono">{robotId}-preview.pkl</small></span><strong>{hasFile(setup.files, `${robotId}-preview.pkl`) ? 'MADE' : 'NOT YET'}</strong></li>
          <li><Lamp state={hasFile(setup.files, `${robotId}-npose.pkl`) ? 'passed' : 'off'} /><span>Arm-aligned (optional)<small className="mono">{robotId}-npose.pkl · neutral pose {session?.npose_start_s ?? 2} s for {session?.npose_duration_s ?? 1} s</small></span><strong>{hasFile(setup.files, `${robotId}-npose.pkl`) ? 'MADE' : 'NOT YET'}</strong></li>
        </ul>
        <div className="setup-actions">
          {runKey('retarget', 'retarget', 'Make preview', <Play aria-hidden="true" />, { mode: 'preview' })}
          {runKey('retarget', 'retarget', 'Make arm-aligned (N-pose)', <Play aria-hidden="true" />, { mode: 'npose' }, false)}
        </div>
        <p className="group-note">Both outputs are preview quality and replace earlier ones in the session folder. For arm alignment, the neutral window must be a moment where the performer stands still with arms down.</p>
        {stepError}{stepNotice}
        {consoleFor(stepJob)}
      </fieldset>
    )
  }

  function renderRender() {
    const video = `${robotId}-${renderMotion}.mp4`
    const ready = hasFile(setup.files, `${robotId}-${renderMotion}.pkl`)
    return (
      <>
        <fieldset className="group">
          <legend className="silk">Simulation</legend>
          {displayNote}
          <div className="choice-cards" role="radiogroup" aria-label="Motion">
            {(['preview', 'npose'] as const).map((motion) => (
              <label key={motion} className={renderMotion === motion ? 'selected' : ''}>
                <input type="radio" name="setup-motion" checked={renderMotion === motion} onChange={() => setRenderMotion(motion)} />
                <span><strong>{motion === 'preview' ? 'Preview' : 'Arm-aligned'}</strong><small className="mono">{robotId}-{motion}.pkl{hasFile(setup.files, `${robotId}-${motion}.pkl`) ? '' : ' · not made yet'}</small></span>
              </label>
            ))}
          </div>
          <div className="setup-actions">
            {runKey('render', 'render', 'Record MP4', <Video aria-hidden="true" />, { motion: renderMotion, record_video: true }, true, !ready)}
            {runKey('render', 'render', 'Open viewer', <Play aria-hidden="true" />, { motion: renderMotion, record_video: false }, false, !ready || noDisplay)}
          </div>
          <p className="group-note">Check neutral stance, both arms, knee bends, steps and turns. Watch for tilted bodies, sliding or sinking feet, and hands passing through the body.</p>
          {stepError}{stepNotice}
          {consoleFor(stepJob)}
        </fieldset>
        <fieldset className="group">
          <legend className="silk">Video</legend>
          {hasFile(setup.files, video)
            ? <video className="setup-video" controls src={client.fileUrl(video)} aria-label={video} />
            : <div className="empty-state"><span>No <span className="mono">{video}</span> yet. Click Record MP4.</span></div>}
        </fieldset>
      </>
    )
  }

  function renderFinish() {
    return (
      <>
        {route === 'live' && (
          <p className="setup-callout"><Lamp state="next" /><span><strong>{windows ? 'In Xsens:' : 'On Windows:'}</strong> stop Network Streamer and save the take.</span></p>
        )}
        <fieldset className="group">
          <legend className="silk">Save report</legend>
          <p className="group-note">Saves the project version and a check report in the session folder.</p>
          <div className="setup-actions">
            {runKey('finish', 'save', 'Save report', <Save aria-hidden="true" />)}
          </div>
          {stepError}{stepNotice}
          {consoleFor(stepJob)}
        </fieldset>
        <fieldset className="group">
          <legend className="silk">Session files</legend>
          {session && <p className="group-note mono">{shown(session.session_dir)}</p>}
          {setup.files.length ? (
            <table className="hint-table file-table">
              <thead><tr><th>File</th><th>Kind</th><th>Size</th></tr></thead>
              <tbody>
                {setup.files.map((file) => (
                  <tr key={file.name}>
                    <td className="mono">{servable(file.name) ? <a href={client.fileUrl(file.name)} target="_blank" rel="noreferrer">{file.name}</a> : file.name}</td>
                    <td>{file.kind}</td>
                    <td className="mono">{sizeText(file.size)}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          ) : <div className="empty-state"><span>No files in the session folder yet.</span></div>}
        </fieldset>
        {setup.desktop_tools && (
          <fieldset className="group">
            <legend className="silk">Shut down</legend>
            {shutdownState === 'done' ? (
              <p className="saved-note" role="status"><Lamp state="passed" /><span>The console is shutting down. To start it again, open <strong>Mocap Setup</strong> from the applications menu.</span></p>
            ) : confirmShutdown ? (
              <div className="setup-callout is-warning" role="alertdialog" aria-label="Shut down the console?">
                <AlertTriangle aria-hidden="true" />
                <div>
                  <p>Shut down the Setup console? A running job{runningJob ? ` (${runningJob.label})` : ''} stops first. An open K1 test window stays open.</p>
                  <div className="setup-actions">
                    <button type="button" className="panel-key setup-run is-stop" disabled={shutdownState === 'sending'} onClick={() => void shutdownConsole()}>
                      {shutdownState === 'sending' ? <RefreshCw className="spin" aria-hidden="true" /> : <Power aria-hidden="true" />}Shut down
                    </button>
                    <button type="button" className="panel-key" disabled={shutdownState === 'sending'} onClick={() => setConfirmShutdown(false)}>Cancel</button>
                  </div>
                </div>
              </div>
            ) : (
              <div className="setup-actions">
                <button type="button" className="panel-key" onClick={() => setConfirmShutdown(true)}><Power aria-hidden="true" />Shut down the console</button>
                <span className="group-note">Open it again from the applications menu: Mocap Setup.</span>
              </div>
            )}
            {shutdownError && <ActionError message={shutdownError} />}
          </fieldset>
        )}
        <p className="group-note">Nothing here approves physical robot use. For the next take, click New session in {windows ? 'the Session step' : 'step 03'}.</p>
      </>
    )
  }

  function renderK1() {
    const row = (state: boolean | null, label: string, detail: string, word?: string) => (
      <li><Lamp state={readyLed(state)} /><span>{label}<small className="mono">{detail}</small></span><strong>{word ?? readyWord(state)}</strong></li>
    )
    const status = k1
    const known = status?.available === true
    const installJob = jobs.find((job) => job.kind === 'k1_install')
    return (
      <>
        <fieldset className="group">
          <legend className="silk">K1 test tools on this PC</legend>
          {k1Error && <ActionError message={k1Error} />}
          <ul className="chain readiness">
            {row(known ? status.sdk_version !== null : null, 'Booster K1 SDK', known && status.sdk_version ? `version ${status.sdk_version}` : 'project .venv')}
            {row(known ? status.tk : null, 'Tk (window toolkit)', 'python3-tk')}
            {row(known ? status.runtime : null, 'GMR runtime', known ? status.runtime_dir : '~/mocap-runtime')}
            {row(known ? status.model_found : null, 'K1 model', known ? status.model : 'K1_serial.xml')}
            {row(known ? (status.window_open ? true : null) : null, 'K1 test window', known && status.window_open ? 'open, or an install is running' : 'not open', known && status.window_open ? 'OPEN' : 'CLOSED')}
          </ul>
          <div className="setup-actions">
            {runKey('k1', 'k1_install', known && status.installed ? 'Reinstall K1 test tools' : 'Install K1 test tools', <Download aria-hidden="true" />, {}, !(known && status.installed), known && status.window_open)}
            {known && !status.tk && (
              <button type="button" className="panel-key" disabled={adminBusy !== null} onClick={() => void runAdmin('install_tk')}>
                {adminBusy === 'install_tk' ? <RefreshCw className="spin" aria-hidden="true" /> : <Download aria-hidden="true" />}
                {adminBusy === 'install_tk' ? 'Waiting for your password…' : 'Install Tk'}
              </button>
            )}
            <button type="button" className="panel-key" onClick={() => void refreshK1()}><RefreshCw aria-hidden="true" />Check again</button>
          </div>
          <p className="group-note">{session ? `Installs the recorded runtime (with the K1 model) into ${shown(session.mocap_ws)} and the Booster SDK into the project, without sudo. Safe to run again.` : 'Save a session in step 03 first: the install uses its runtime folder.'}{known && status.window_open ? ' Close the K1 test window before you install.' : ''}</p>
          {adminResult?.action === 'install_tk' && <AdminOutcome result={adminResult} copy="sudo apt-get install -y python3-tk" />}
          {stepError}
          {consoleFor(stepJob ?? installJob)}
        </fieldset>
        <fieldset className="group">
          <legend className="silk">K1 test window</legend>
          <p className="group-note">The twelve hardware-test steps, each from a button. It opens on this desktop as its own window. Shutting down this console leaves it, and its safety handling, running.</p>
          <div className="setup-actions">
            <button type="button" className="panel-key setup-run is-primary" disabled={k1Opening || (known && !status.display)} onClick={() => void openK1Window()}>
              {k1Opening ? <RefreshCw className="spin" aria-hidden="true" /> : <Bot aria-hidden="true" />}Open K1 test window
            </button>
            <a className="panel-key" href={client.guideUrl('step-13')} target="_blank" rel="noreferrer"><BookOpen aria-hidden="true" />Open the guide</a>
          </div>
          {known && !status.display && displayNote}
          {k1Result && (
            <p className={k1Result.outcome === 'failed' ? 'setup-callout is-warning' : 'saved-note'} role="status">
              {k1Result.outcome === 'failed' ? <AlertTriangle aria-hidden="true" /> : <Lamp state={k1Result.outcome === 'opened' ? 'passed' : 'next'} />}
              <span>{k1Result.outcome === 'already_open' ? 'Already open, or an install is running. Look for the K1 window on this desktop.' : k1Result.message}</span>
            </p>
          )}
          {known && status.log && <p className="group-note">Window log: <span className="mono">{status.log}</span></p>}
        </fieldset>
      </>
    )
  }

  const panels: Record<SetupStep, () => ReactNode> = {
    session: renderSession,
    windows: renderWindows,
    stream: renderStream,
    live: renderLive,
    bvh: renderBvh,
    retarget: renderRetarget,
    render: renderRender,
    finish: renderFinish,
    k1: renderK1,
  }

  return (
    <main className="setup-workspace" hidden={hidden} aria-label="Setup">
      <nav className="setup-stepper" aria-label="Setup steps">
        <div className="route-toggle" role="radiogroup" aria-label="Route">
          {(['live', 'recorded'] as const).map((value) => (
            <label key={value} className={route === value ? 'selected' : ''}>
              <input type="radio" name="setup-route" value={value} checked={route === value} onChange={() => selectRoute(value)} />
              {value === 'live' ? 'Live' : 'Recorded'}
            </label>
          ))}
        </div>
        <ol>
          {steps.map((item) => {
            const led = leds[item]
            return (
              <li key={item} data-step={item}>
                <button type="button" className={step === item ? 'active' : ''} aria-current={step === item ? 'step' : undefined} onClick={() => selectStep(item)}>
                  <span className="step-number mono">{stepLabel(item)}</span>
                  <span className="step-title">{(windows && WINDOWS_STEP_META[item]?.title) || STEP_META[item].title}</span>
                  <Lamp state={led} label={LED_WORD[led] || 'Not started'} />
                </button>
              </li>
            )
          })}
        </ol>
        <p className="setup-repo"><span className="silk">Project</span><span className="mono">{setup.repo_root}</span></p>
      </nav>

      <section className="setup-panel" data-step={step} aria-labelledby="setup-step-title">
        <header className="setup-panel-head">
          <span className="silk">Step {stepWord}{meta.route ? ` · ${meta.route} route` : ''}</span>
          <h1 id="setup-step-title">{meta.title}</h1>
          <p className="setup-instruction">{meta.instruction.replace('{n}', String(checkSeconds))}</p>
        </header>
        {refreshError && <ActionError message={refreshError} />}
        {routeMismatch && step !== 'session' && (
          <p className="setup-callout is-warning"><AlertTriangle aria-hidden="true" /><span>The saved session uses the {session.route} route. Save the session in {windows ? 'the Session step' : 'step 03'} to switch.</span></p>
        )}
        {runningJob && !STEP_META[step].kinds.includes(runningJob.kind) && (
          <div className="setup-callout running-strip">
            <Lamp state="active" />
            <span>Running: <strong>{runningJob.label}</strong> · <span className="mono">{clockText(elapsedSeconds(runningJob, nowMs))}</span></span>
            <button type="button" className="panel-key" disabled={isStopping(runningJob.id)} onClick={() => void stopJob(runningJob)}>
              {isStopping(runningJob.id) ? <RefreshCw className="spin" aria-hidden="true" /> : <Square aria-hidden="true" />}{isStopping(runningJob.id) ? 'Stopping…' : 'Stop'}
            </button>
          </div>
        )}
        {noSessionNote}
        {panels[step]()}
      </section>
    </main>
  )
}

const FIREWALL_NOTE: Record<NonNullable<SetupState['firewall_state']>, string> = {
  active: 'UFW is active on this PC: allow the Xsens stream through it.',
  unknown: 'Only needed if UFW is active (this console cannot tell).',
  inactive: 'UFW is off on this PC: no firewall change is needed.',
  not_installed: 'UFW is not installed on this PC: no firewall change is needed.',
}

function AdminOutcome({ result, copy }: { result: { outcome: PrivilegedOutcome | 'error'; message: string }; copy?: string }) {
  const ok = result.outcome === 'done'
  return (
    <div className={ok ? 'saved-note' : 'setup-callout is-warning'} role="status">
      {ok ? <Lamp state="passed" /> : result.outcome === 'cancelled' ? <Lamp state="off" /> : <AlertTriangle aria-hidden="true" />}
      <span>{result.message}</span>
      {copy && result.outcome !== 'done' && result.outcome !== 'cancelled' && (
        <span className="command-line"><code className="mono">{copy}</code><CopyButton value={copy} label="command" /></span>
      )}
    </div>
  )
}

function readyLed(value: boolean | null): Led {
  return value === true ? 'passed' : value === false ? 'fault' : 'off'
}

function readyWord(value: boolean | null): string {
  return value === true ? 'FOUND' : value === false ? 'MISSING' : 'NOT CHECKED'
}
