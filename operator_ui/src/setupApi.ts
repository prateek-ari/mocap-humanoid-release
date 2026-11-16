/**
 * Client and runtime guards for the Setup workbench API (``/api/v1/setup``).
 * The workbench only exists when the console was started with ``--setup``;
 * otherwise every route is 404 and the Setup workspace stays hidden.
 *
 * Like ``protocol.ts``, nothing here trusts the payload shape: fields are
 * checked one by one and malformed entries are dropped or replaced by a safe
 * fallback instead of being cast into place.
 */
import { isRecord } from './protocol'

const SETUP_API = '/api/v1/setup'
const SETUP_ROBOTS = ['booster_k1', 'unitree_g1', 'booster_t1_29dof'] as const

export type SetupRoute = 'live' | 'recorded'
/** The OS the console runs on; decides which guide wording and checks apply. */
export type SetupPlatform = 'linux' | 'windows'
export type SetupRobot = typeof SETUP_ROBOTS[number]

/** Unknown or missing robot keys fall back to the Booster K1, the default robot. */
export function asSetupRobot(value: unknown): SetupRobot {
  return typeof value === 'string' && (SETUP_ROBOTS as readonly string[]).includes(value) ? value as SetupRobot : 'booster_k1'
}
export type JobState = 'running' | 'passed' | 'failed' | 'stopped'
export type JobKind = 'doctor' | 'acceptance' | 'live' | 'bvh_check' | 'retarget' | 'render' | 'save' | 'compare' | 'k1_install'
/** UFW as the console reads it without privileges; ``unknown`` never means off. */
export type FirewallState = 'active' | 'inactive' | 'not_installed' | 'unknown'
export type PrivilegedAction = 'firewall' | 'install_tk'

/** ``GET /api/v1/setup/k1``: the K1 test tools on this PC (Linux checkout console only). */
export interface K1Status {
  available: boolean
  sdk_version: string | null
  tk: boolean
  runtime: boolean
  runtime_dir: string
  model: string
  model_found: boolean
  installed: boolean
  /** The K1 lock is held: a K1 test window is open, or an install is running. */
  window_open: boolean
  display: boolean
  log: string
}

export type K1OpenOutcome = 'opened' | 'already_open' | 'failed'

/** Step 13's live robot panel: ``GET /api/v1/setup/k1/live``. */
export type K1LiveAction = 'check' | 'go'
export type K1LiveOutcome = 'ready' | 'passed' | 'stopped' | 'refused' | 'failed'
export interface K1LiveCheck { key: string; ok: boolean | null; text: string }
export interface K1LiveRun {
  action: K1LiveAction
  running: boolean
  started_at: number
  finished_at: number | null
  settings: { net: string; operator: string; head: boolean; time_limit_s: number }
  checks: K1LiveCheck[]
  /** The bridge's last status line: ``state`` (waiting, aligning, following, hold, realign, stopping, …), ``following_s``, ``time_limit_s``, ``head``, ``lag_holds``. */
  live: Record<string, unknown> | null
  result: { outcome: K1LiveOutcome; reason: string | null } | null
  stop_requested: boolean
  robot_data: { folder: string | null; state: string } | null
  session: string | null
  lines: string[]
  log: string
}
export interface K1LiveStatus {
  /** Other programs on the ROS 2 ports (7400-7499) of this PC: they can crash the robot's software. */
  other_dds: { name: string; pid: number }[]
  run: K1LiveRun | null
  limits_s: number[]
  /** The practice robot, not the real K1 (``bash k1-demo.sh --practice``). */
  practice: boolean
}
export interface K1LiveRequest { action: K1LiveAction; net: string; operator: string; head: boolean; time_limit_s: number }
/** done (exit 0), cancelled (prompt dismissed) or unavailable (no prompt, or the command failed). */
export type PrivilegedOutcome = 'done' | 'cancelled' | 'unavailable'

export interface ActionResult<T extends string> {
  outcome: T
  message: string
}

export interface SessionValues {
  route: SetupRoute
  session_dir: string
  robot: SetupRobot
  human_height_m: number
  mocap_ws: string
  motion_bvh: string
  npose_bvh: string
  offsets_file: string
  bvh_scale: number
  start_frame: number
  end_frame: number
  frame_step: number
  npose_start_s: number
  npose_duration_s: number
  windows_xsens_ip: string
  linux_xsens_ip: string
}

export type SessionField = keyof SessionValues

export interface NetworkInterface {
  name: string
  ipv4: string
}

export interface SetupFile {
  name: string
  size: number
  modified: number
  kind: string
}

/** The live simulation's last machine STATUS line (Windows desktop app). */
export interface LiveStatus {
  /** Arm and waist tracking is on. */
  enabled: boolean
  reason: string
  fps: number
  frames: number
  drops: number
  failures: number
  calibrated: boolean
}

/** One ``calibration_result`` STATUS event of a live job (``request`` null: N key or timer). */
export interface CalibrationResult {
  request: number | null
  ok: boolean
  epoch: number | null
  epoch_changed: boolean
  warning: string | null
  reason: string | null
}

export interface JobSummary {
  id: string
  kind: string
  label: string
  state: JobState
  started_at: number
  finished_at: number | null
  exit_code: number | null
  summary: string | null
  outputs: string[]
  /** Live jobs only, after their first STATUS line; ``null`` (or absent) otherwise. */
  live?: LiveStatus | null
  /** Live jobs: ``calibrate`` requests sent by the console and the results received. */
  calibration?: { sent: number; results: CalibrationResult[] } | null
}

/** The checkout's commit and the last Mocap Update result (Ubuntu console only). */
export interface SoftwareInfo {
  commit: string | null
  /** ``null``: never updated with Mocap Update. */
  update_last: { line: string; result: 'PASS' | 'FAIL' | null; details: string[] } | null
}

/** The console runs inside the Windows desktop app (``MOCAP_APP=1``). */
export interface AppInfo {
  mode: 'windows_app'
  version: string
}

export type StreamState = 'off' | 'none' | 'receiving' | 'stale' | 'port_busy' | 'paused'

/** The idle Xsens UDP monitor (``GET /api/v1/setup/stream``). */
export interface StreamSnapshot {
  state: StreamState
  bind: string
  port: number
  /** Last body-packet sender, ``host:port``. */
  source: string | null
  /** IPv4 addresses that sent body packets in the last 2 s. */
  senders: string[]
  rate_hz: number | null
  loss_fraction: number | null
  body_frames: number
  joint_angles: boolean
  last_packet_age_s: number | null
  malformed: number
}

/** Per-user app preferences (``GET/PUT /api/v1/setup/prefs``). */
export interface Prefs {
  schema: 1
  record_video: boolean
  /** Seconds after the first frame before automatic calibration; ``null`` for manual. */
  calibrate_after_s: number | null
}

export type ControlCommand = 'calibrate' | 'toggle_tracking' | 'rearm'

export interface SetupReadiness {
  live_image: boolean | null
  /** Windows only: the live/retarget runtime (venv python and GMR) exists. ``null`` on Linux. */
  live_runtime: boolean | null
  recorded_runtime: boolean | null
  motion_bvh: boolean | null
  npose_bvh: boolean | null
  /** False when the console has no desktop display for viewer windows. */
  display: boolean | null
  display_hint: string | null
}

export interface SetupState {
  /** Non-null when the console runs as the Windows desktop app. */
  app: AppInfo | null
  /** The Xsens UDP monitor; ``null`` when it is off or not reported. */
  stream: StreamSnapshot | null
  prefs: Prefs
  /** Missing or unknown values are treated as ``linux``. */
  platform: SetupPlatform
  repo_root: string
  /** The console user's home directory; paths under it are displayed as ``~/…``. */
  home: string | null
  /** True when the session's (or default) Xsens source IP is a loopback address. */
  xsens_on_this_pc: boolean
  session: SessionValues | null
  defaults: SessionValues
  network: { interfaces: NetworkInterface[]; detected_linux_ip: string | null }
  firewall_command: string | null
  /** ``null`` on Windows. */
  firewall_state: FirewallState | null
  /** Linux checkout console: the Real K1 step, Allow in firewall and Shut down. */
  desktop_tools: boolean
  readiness: SetupReadiness
  files: SetupFile[]
  jobs: JobSummary[]
  /** Length of the stream check (``acceptance``) in seconds. */
  stream_check_s: number
  software: SoftwareInfo | null
}

export interface JobLog {
  job: JobSummary
  lines: string[]
  next: number
}

/** The console returns at most this many log lines per job request. */
export const LOG_PAGE_LINES = 2000

const DRIVE_PATH = /^[A-Za-z]:(?:[\\/]|$)/

/** True for a Windows drive path such as ``C:/Users/me`` or ``c:\\Users\\me``. */
export function isDrivePath(path: string): boolean {
  return DRIVE_PATH.test(path)
}

/**
 * Display a server path under ``home`` as ``~/…``; the backend expands ``~/`` input.
 * Windows drive paths match case-insensitively and accept ``\\`` as well as ``/``.
 */
export function tildePath(path: string, home: string | null): string {
  if (!home || !path) return path
  if (isDrivePath(home) && isDrivePath(path)) {
    const slashed = path.replace(/\\/g, '/')
    const base = home.replace(/\\/g, '/').replace(/\/+$/, '')
    if (slashed.toLowerCase() === base.toLowerCase()) return '~'
    return slashed.toLowerCase().startsWith(`${base.toLowerCase()}/`) ? `~${slashed.slice(base.length)}` : path
  }
  if (path === home) return '~'
  return path.startsWith(`${home}/`) ? `~${path.slice(home.length)}` : path
}

/** True for an IPv4 loopback address such as ``127.0.0.1``. */
export function isLoopbackIp(ip: string): boolean {
  return /^127\.\d{1,3}\.\d{1,3}\.\d{1,3}$/.test(ip.trim())
}

/** The home directory the console reports: a POSIX path or a Windows drive path, with ``/`` separators. */
function parseHome(value: unknown): string | null {
  if (typeof value !== 'string') return null
  if (isDrivePath(value)) return value.replace(/\\/g, '/').replace(/\/+$/, '') || null
  return value.startsWith('/') ? value.replace(/\/+$/, '') || null : null
}

export type JobOptions = Record<string, string | number | boolean | null>

/** A rejected Setup request, with the backend's error code and field when given. */
export class SetupApiError extends Error {
  readonly status: number
  readonly code: string | null
  readonly field: string | null
  readonly jobId: string | null

  constructor(message: string, status: number, code: string | null = null, field: string | null = null, jobId: string | null = null) {
    super(message)
    this.name = 'SetupApiError'
    this.status = status
    this.code = code
    this.field = field
    this.jobId = jobId
  }
}

export const SESSION_FIELDS: readonly SessionField[] = [
  'route', 'session_dir', 'robot', 'human_height_m', 'mocap_ws', 'motion_bvh', 'npose_bvh',
  'offsets_file', 'bvh_scale', 'start_frame', 'end_frame', 'frame_step', 'npose_start_s',
  'npose_duration_s', 'windows_xsens_ip', 'linux_xsens_ip',
]

const FIREWALL_STATES: ReadonlySet<string> = new Set<FirewallState>(['active', 'inactive', 'not_installed', 'unknown'])
const K1_OUTCOMES: ReadonlySet<string> = new Set<K1OpenOutcome>(['opened', 'already_open', 'failed'])
const PRIVILEGED_OUTCOMES: ReadonlySet<string> = new Set<PrivilegedOutcome>(['done', 'cancelled', 'unavailable'])
/** The console waits up to 10 minutes for the password prompt (PRIVILEGED_TIMEOUT_S). */
const PRIVILEGED_TIMEOUT_MS = 630_000

const JOB_STATES: ReadonlySet<string> = new Set<JobState>(['running', 'passed', 'failed', 'stopped'])

const FALLBACK_SESSION: SessionValues = {
  route: 'live',
  session_dir: '',
  robot: 'booster_k1',
  human_height_m: 1.75,
  mocap_ws: '',
  motion_bvh: '',
  npose_bvh: '',
  offsets_file: '',
  bvh_scale: 0.01,
  start_frame: 0,
  end_frame: 600,
  frame_step: 1,
  npose_start_s: 2,
  npose_duration_s: 1,
  windows_xsens_ip: '',
  linux_xsens_ip: '',
}

export const DEFAULT_PREFS: Prefs = { schema: 1, record_video: false, calibrate_after_s: null }

const STREAM_STATES: ReadonlySet<string> = new Set<StreamState>(['off', 'none', 'receiving', 'stale', 'port_busy', 'paused'])
const CONTROL_COMMANDS: ReadonlySet<string> = new Set<ControlCommand>(['calibrate', 'toggle_tracking', 'rearm'])

const str = (value: unknown, fallback = ''): string => typeof value === 'string' ? value : fallback
const num = (value: unknown, fallback: number): number => typeof value === 'number' && Number.isFinite(value) ? value : fallback
const numOrNull = (value: unknown): number | null => typeof value === 'number' && Number.isFinite(value) ? value : null
const boolOrNull = (value: unknown): boolean | null => typeof value === 'boolean' ? value : null

/** Field names may arrive as the session.env key (``HUMAN_HEIGHT_M``) or snake_case. */
export function normalizeField(value: unknown): SessionField | null {
  if (typeof value !== 'string') return null
  const lowered = value.trim().toLowerCase()
  return (SESSION_FIELDS as readonly string[]).includes(lowered) ? lowered as SessionField : null
}

export function parseSessionValues(value: unknown): SessionValues | null {
  if (!isRecord(value)) return null
  const f = FALLBACK_SESSION
  return {
    route: value.route === 'recorded' ? 'recorded' : 'live',
    session_dir: str(value.session_dir),
    robot: asSetupRobot(value.robot),
    human_height_m: num(value.human_height_m, f.human_height_m),
    mocap_ws: str(value.mocap_ws),
    motion_bvh: str(value.motion_bvh),
    npose_bvh: str(value.npose_bvh),
    offsets_file: str(value.offsets_file),
    bvh_scale: num(value.bvh_scale, f.bvh_scale),
    start_frame: num(value.start_frame, f.start_frame),
    end_frame: num(value.end_frame, f.end_frame),
    frame_step: num(value.frame_step, f.frame_step),
    npose_start_s: num(value.npose_start_s, f.npose_start_s),
    npose_duration_s: num(value.npose_duration_s, f.npose_duration_s),
    windows_xsens_ip: str(value.windows_xsens_ip),
    linux_xsens_ip: str(value.linux_xsens_ip),
  }
}

/** ``null`` for anything but ``{"mode": "windows_app", ...}``. */
export function parseAppInfo(value: unknown): AppInfo | null {
  if (!isRecord(value) || value.mode !== 'windows_app') return null
  return { mode: 'windows_app', version: typeof value.version === 'string' ? value.version.trim() : '' }
}

/** Unknown states read as ``off``; counters never go negative. */
export function parseStreamSnapshot(value: unknown): StreamSnapshot | null {
  if (!isRecord(value)) return null
  const nonNegative = (item: unknown): number | null => {
    const parsed = numOrNull(item)
    return parsed !== null && parsed >= 0 ? parsed : null
  }
  const port = num(value.port, 5555)
  const loss = nonNegative(value.loss_fraction)
  return {
    state: typeof value.state === 'string' && STREAM_STATES.has(value.state) ? value.state as StreamState : 'off',
    bind: str(value.bind),
    port: Number.isInteger(port) && port > 0 && port < 65536 ? port : 5555,
    source: typeof value.source === 'string' && value.source.trim() ? value.source.trim() : null,
    senders: Array.isArray(value.senders) ? value.senders.filter((item): item is string => typeof item === 'string' && item !== '') : [],
    rate_hz: nonNegative(value.rate_hz),
    loss_fraction: loss === null ? null : Math.min(1, loss),
    body_frames: Math.floor(nonNegative(value.body_frames) ?? 0),
    joint_angles: value.joint_angles === true,
    last_packet_age_s: nonNegative(value.last_packet_age_s),
    malformed: Math.floor(nonNegative(value.malformed) ?? 0),
  }
}

/** Missing or invalid fields fall back to the defaults (no video, manual calibration). */
export function parsePrefs(value: unknown): Prefs {
  if (!isRecord(value)) return { ...DEFAULT_PREFS }
  const after = numOrNull(value.calibrate_after_s)
  return {
    schema: 1,
    record_video: value.record_video === true,
    calibrate_after_s: after !== null && after > 0 ? after : null,
  }
}

/** ``null`` unless every field has the contract type. */
export function parseLiveStatus(value: unknown): LiveStatus | null {
  if (!isRecord(value) || typeof value.enabled !== 'boolean') return null
  const count = (item: unknown) => Math.max(0, Math.floor(num(item, 0)))
  return {
    enabled: value.enabled,
    reason: str(value.reason),
    fps: Math.max(0, num(value.fps, 0)),
    frames: count(value.frames),
    drops: count(value.drops),
    failures: count(value.failures),
    calibrated: value.calibrated === true,
  }
}

function parseCalibrationResult(value: unknown): CalibrationResult | null {
  if (!isRecord(value) || typeof value.ok !== 'boolean') return null
  const whole = (item: unknown) => typeof item === 'number' && Number.isInteger(item) ? item : null
  const text = (item: unknown) => typeof item === 'string' && item.trim() ? item.trim() : null
  return {
    request: whole(value.request),
    ok: value.ok,
    epoch: whole(value.epoch),
    epoch_changed: value.epoch_changed === true,
    warning: text(value.warning),
    reason: text(value.reason),
  }
}

export function parseSoftware(value: unknown): SoftwareInfo | null {
  if (!isRecord(value)) return null
  const last = isRecord(value.update_last) ? value.update_last : null
  return {
    commit: typeof value.commit === 'string' && value.commit.trim() ? value.commit.trim() : null,
    update_last: last === null ? null : {
      line: str(last.line),
      result: last.result === 'PASS' || last.result === 'FAIL' ? last.result : null,
      details: Array.isArray(last.details) ? last.details.filter((item): item is string => typeof item === 'string') : [],
    },
  }
}

export function parseJobSummary(value: unknown): JobSummary | null {
  if (!isRecord(value) || typeof value.id !== 'string' || !value.id || typeof value.kind !== 'string') return null
  if (typeof value.state !== 'string' || !JOB_STATES.has(value.state)) return null
  return {
    id: value.id,
    kind: value.kind,
    label: str(value.label, value.kind),
    state: value.state as JobState,
    started_at: num(value.started_at, 0),
    finished_at: numOrNull(value.finished_at),
    exit_code: numOrNull(value.exit_code),
    summary: typeof value.summary === 'string' && value.summary.trim() ? value.summary : null,
    outputs: Array.isArray(value.outputs) ? value.outputs.filter((item): item is string => typeof item === 'string') : [],
    live: value.kind === 'live' ? parseLiveStatus(value.live) : null,
    calibration: value.kind === 'live' && isRecord(value.calibration) ? {
      sent: Math.max(0, Math.floor(num(value.calibration.sent, 0))),
      results: list(value.calibration.results, parseCalibrationResult),
    } : null,
  }
}

function parseFile(value: unknown): SetupFile | null {
  if (!isRecord(value) || typeof value.name !== 'string' || !value.name) return null
  return { name: value.name, size: num(value.size, 0), modified: num(value.modified, 0), kind: str(value.kind, 'other') }
}

function parseInterface(value: unknown): NetworkInterface | null {
  if (!isRecord(value) || typeof value.name !== 'string' || typeof value.ipv4 !== 'string' || !value.ipv4) return null
  return { name: value.name, ipv4: value.ipv4 }
}

function list<T>(value: unknown, parse: (item: unknown) => T | null): T[] {
  return Array.isArray(value) ? value.map(parse).filter((item): item is T => item !== null) : []
}

export function parseSetupState(value: unknown): SetupState {
  if (!isRecord(value)) throw new Error('Setup response did not match the expected format.')
  const defaults = parseSessionValues(value.defaults)
  if (!defaults) throw new Error('Setup response did not include default session values.')
  const network = isRecord(value.network) ? value.network : {}
  const readiness = isRecord(value.readiness) ? value.readiness : {}
  return {
    app: parseAppInfo(value.app),
    stream: parseStreamSnapshot(value.stream),
    prefs: parsePrefs(value.prefs),
    platform: value.platform === 'windows' ? 'windows' : 'linux',
    repo_root: str(value.repo_root),
    home: parseHome(value.home),
    xsens_on_this_pc: value.xsens_on_this_pc === true,
    session: parseSessionValues(value.session),
    defaults,
    network: {
      interfaces: list(network.interfaces, parseInterface),
      detected_linux_ip: typeof network.detected_linux_ip === 'string' && network.detected_linux_ip ? network.detected_linux_ip : null,
    },
    firewall_command: typeof value.firewall_command === 'string' && value.firewall_command.trim() ? value.firewall_command : null,
    firewall_state: typeof value.firewall_state === 'string' && FIREWALL_STATES.has(value.firewall_state) ? value.firewall_state as FirewallState : null,
    desktop_tools: value.desktop_tools === true,
    readiness: {
      live_image: boolOrNull(readiness.live_image),
      live_runtime: boolOrNull(readiness.live_runtime),
      recorded_runtime: boolOrNull(readiness.recorded_runtime),
      motion_bvh: boolOrNull(readiness.motion_bvh),
      npose_bvh: boolOrNull(readiness.npose_bvh),
      display: boolOrNull(readiness.display),
      display_hint: typeof readiness.display_hint === 'string' && readiness.display_hint.trim() ? readiness.display_hint : null,
    },
    files: list(value.files, parseFile),
    jobs: list(value.jobs, parseJobSummary),
    stream_check_s: typeof value.stream_check_s === 'number' && Number.isFinite(value.stream_check_s) && value.stream_check_s > 0 ? value.stream_check_s : 30,
    software: parseSoftware(value.software),
  }
}

const K1_OUTCOME_SET: ReadonlySet<string> = new Set<K1LiveOutcome>(['ready', 'passed', 'stopped', 'refused', 'failed'])

/** Unknown fields read as "nothing running"; a malformed run is dropped. */
export function parseK1Live(value: unknown): K1LiveStatus {
  const record = isRecord(value) ? value : {}
  const other = Array.isArray(record.other_dds) ? record.other_dds.filter(isRecord).map((item) => ({ name: str(item.name), pid: typeof item.pid === 'number' ? item.pid : 0 })) : []
  const limits = Array.isArray(record.limits_s) ? record.limits_s.filter((item): item is number => typeof item === 'number') : [60, 120, 300, 600]
  const raw = isRecord(record.run) ? record.run : null
  let run: K1LiveRun | null = null
  if (raw && (raw.action === 'check' || raw.action === 'go')) {
    const settings = isRecord(raw.settings) ? raw.settings : {}
    const result = isRecord(raw.result) && typeof raw.result.outcome === 'string' && K1_OUTCOME_SET.has(raw.result.outcome)
      ? { outcome: raw.result.outcome as K1LiveOutcome, reason: typeof raw.result.reason === 'string' ? raw.result.reason : null }
      : null
    run = {
      action: raw.action,
      running: raw.running === true,
      started_at: typeof raw.started_at === 'number' ? raw.started_at : 0,
      finished_at: typeof raw.finished_at === 'number' ? raw.finished_at : null,
      settings: {
        net: str(settings.net), operator: str(settings.operator), head: settings.head !== false,
        time_limit_s: typeof settings.time_limit_s === 'number' ? settings.time_limit_s : 120,
      },
      checks: Array.isArray(raw.checks) ? raw.checks.filter(isRecord).map((item) => ({ key: str(item.key), ok: typeof item.ok === 'boolean' ? item.ok : null, text: str(item.text) })) : [],
      live: isRecord(raw.live) ? raw.live : null,
      result,
      stop_requested: raw.stop_requested === true,
      robot_data: isRecord(raw.robot_data) ? { folder: typeof raw.robot_data.folder === 'string' ? raw.robot_data.folder : null, state: str(raw.robot_data.state) } : null,
      session: typeof raw.session === 'string' ? raw.session : null,
      lines: Array.isArray(raw.lines) ? raw.lines.filter((line): line is string => typeof line === 'string') : [],
      log: str(raw.log),
    }
  }
  return { other_dds: other, run, limits_s: limits.length ? limits : [60, 120, 300, 600], practice: record.practice === true }
}

/** Missing or malformed fields read as not installed / not open. */
export function parseK1Status(value: unknown): K1Status {
  const record = isRecord(value) ? value : {}
  return {
    available: record.available === true,
    sdk_version: typeof record.sdk_version === 'string' && record.sdk_version ? record.sdk_version : null,
    tk: record.tk === true,
    runtime: record.runtime === true,
    runtime_dir: str(record.runtime_dir),
    model: str(record.model),
    model_found: record.model_found === true,
    installed: record.installed === true,
    window_open: record.window_open === true,
    display: record.display === true,
    log: str(record.log),
  }
}

function parseResult<T extends string>(value: unknown, allowed: ReadonlySet<string>, fallback: T): ActionResult<T> {
  const record = isRecord(value) ? value : {}
  return {
    outcome: typeof record.outcome === 'string' && allowed.has(record.outcome) ? record.outcome as T : fallback,
    message: str(record.message),
  }
}

export function parseJobLog(value: unknown, after: number): JobLog {
  const job = isRecord(value) ? parseJobSummary(value.job) : null
  if (!isRecord(value) || !job) throw new Error('Job response did not match the expected format.')
  const lines = Array.isArray(value.lines) ? value.lines.filter((line): line is string => typeof line === 'string') : []
  const next = typeof value.next === 'number' && Number.isInteger(value.next) && value.next >= after ? value.next : after + lines.length
  return { job, lines, next }
}

async function setupRequest(path: string, init: RequestInit & { timeoutMs?: number } = {}): Promise<Response> {
  const { timeoutMs = 5000, ...rest } = init
  try {
    return await fetch(`${SETUP_API}${path}`, {
      ...rest,
      headers: rest.body !== undefined ? { 'Content-Type': 'application/json', ...rest.headers } : rest.headers,
      signal: rest.signal ?? AbortSignal.timeout(timeoutMs),
    })
  } catch (error) {
    const detail = error instanceof Error ? error.message : 'network request failed'
    throw new SetupApiError(`Console unavailable: ${detail}`, 0)
  }
}

async function failure(response: Response): Promise<SetupApiError> {
  const body = await response.json().catch(() => null) as unknown
  const error = isRecord(body) && isRecord(body.error) ? body.error : null
  // Setup errors use {"error": {...}}; FastAPI's own 403/404/415/422 use {"detail": ...}.
  const detail = isRecord(body) ? body.detail : undefined
  const detailText = typeof detail === 'string'
    ? detail
    : Array.isArray(detail) && isRecord(detail[0]) && typeof detail[0].msg === 'string' ? detail[0].msg : ''
  const message = (error && typeof error.message === 'string' && error.message)
    || detailText
    || `${response.status} ${response.statusText}`.trim()
  const code = error && typeof error.code === 'string' ? error.code : null
  const field = error && typeof error.field === 'string' ? error.field : null
  const jobId = error && typeof error.job_id === 'string' ? error.job_id : null
  return new SetupApiError(message, response.status, code, field, jobId)
}

async function json(response: Response): Promise<unknown> {
  if (!response.ok) throw await failure(response)
  return response.json().catch(() => {
    throw new SetupApiError('The console returned an unreadable response.', response.status)
  }) as Promise<unknown>
}

export class SetupClient {
  /** ``null`` when the console runs without ``--setup`` (404). */
  async getSetup(): Promise<SetupState | null> {
    const response = await setupRequest('')
    if (response.status === 404) return null
    return parseSetupState(await json(response))
  }

  async saveSession(values: SessionValues, replace = false): Promise<SetupState> {
    const response = await setupRequest('/session', {
      method: 'PUT',
      body: JSON.stringify(replace ? { ...values, replace: true } : values),
      timeoutMs: 15000,
    })
    return parseSetupState(await json(response))
  }

  async startJob(kind: JobKind, options: JobOptions = {}): Promise<JobSummary> {
    const response = await setupRequest('/jobs', {
      method: 'POST',
      body: JSON.stringify({ kind, options }),
      timeoutMs: 15000,
    })
    const job = parseJobSummary(await json(response))
    if (!job) throw new SetupApiError('The console started the job, but its response was not readable.', response.status)
    return job
  }

  async getJob(id: string, after = 0): Promise<JobLog> {
    const response = await setupRequest(`/jobs/${encodeURIComponent(id)}?after=${Math.max(0, Math.floor(after))}`)
    return parseJobLog(await json(response), after)
  }

  async stopJob(id: string): Promise<JobSummary> {
    const response = await setupRequest(`/jobs/${encodeURIComponent(id)}/stop`, {
      method: 'POST',
      body: '{}',
      timeoutMs: 20000,
    })
    const job = parseJobSummary(await json(response))
    if (!job) throw new SetupApiError('The console stopped the job, but its response was not readable.', response.status)
    return job
  }

  async getStream(): Promise<StreamSnapshot> {
    const stream = parseStreamSnapshot(await json(await setupRequest('/stream', { timeoutMs: 3000 })))
    if (!stream) throw new SetupApiError('Stream response did not match the expected format.', 200)
    return stream
  }

  async getPrefs(): Promise<Prefs> {
    return parsePrefs(await json(await setupRequest('/prefs')))
  }

  /** Partial update; the console merges, validates and returns the saved preferences. */
  async putPrefs(changes: Partial<Omit<Prefs, 'schema'>>): Promise<Prefs> {
    const response = await setupRequest('/prefs', { method: 'PUT', body: JSON.stringify(changes) })
    return parsePrefs(await json(response))
  }

  /** Send a key-equivalent command to the running live simulation (202 accepted). */
  async controlJob(id: string, command: ControlCommand): Promise<number | null> {
    if (!CONTROL_COMMANDS.has(command)) throw new SetupApiError(`Unknown command: ${String(command)}`, 400)
    const response = await setupRequest(`/jobs/${encodeURIComponent(id)}/control`, {
      method: 'POST',
      body: JSON.stringify({ command }),
    })
    const body = await json(response)
    // ``request``: this calibrate's number in the job; its result carries the same number.
    return isRecord(body) && typeof body.request === 'number' ? body.request : null
  }

  /** Stop a leftover live viewer container (409 ``viewer_running``). */
  async stopViewer(): Promise<boolean> {
    const response = await setupRequest('/viewer/stop', { method: 'POST', body: '{}', timeoutMs: 20000 })
    const body = await json(response)
    return isRecord(body) && body.stopped === true
  }

  /** Read a job's log from ``after`` until caught up, following ``next`` across pages. */
  async getJobLog(id: string, after = 0): Promise<JobLog> {
    let log = await this.getJob(id, after)
    const lines = [...log.lines]
    for (let page = 0; page < 50 && log.lines.length >= LOG_PAGE_LINES && log.next > after; page += 1) {
      after = log.next
      log = await this.getJob(id, after)
      lines.push(...log.lines)
    }
    return { job: log.job, lines, next: log.next }
  }

  async getK1(): Promise<K1Status> {
    return parseK1Status(await json(await setupRequest('/k1', { timeoutMs: 30000 })))
  }

  /** Start the K1 test window on this desktop (the console waits about 1 s for it). */
  async openK1(): Promise<ActionResult<K1OpenOutcome>> {
    const response = await setupRequest('/k1/open', { method: 'POST', body: '{}', timeoutMs: 15000 })
    return parseResult(await json(response), K1_OUTCOMES, 'failed')
  }

  async getK1Live(): Promise<K1LiveStatus> {
    return parseK1Live(await json(await setupRequest('/k1/live', { timeoutMs: 15000 })))
  }

  /** Step 13: ``check`` (read-only) or ``go`` (the robot's arms and head follow). */
  async startK1Live(request: K1LiveRequest): Promise<void> {
    await json(await setupRequest('/k1/live', { method: 'POST', body: JSON.stringify(request), timeoutMs: 15000 }))
  }

  /** STOP: the arms slow down and are handed back; the head turns straight. */
  async stopK1Live(): Promise<void> {
    await json(await setupRequest('/k1/live/stop', { method: 'POST', body: '{}', timeoutMs: 15000 }))
  }

  /** One fixed admin command behind the desktop password prompt; resolves when it is answered. */
  async runPrivileged(action: PrivilegedAction): Promise<ActionResult<PrivilegedOutcome>> {
    const response = await setupRequest('/privileged', { method: 'POST', body: JSON.stringify({ action }), timeoutMs: PRIVILEGED_TIMEOUT_MS })
    return parseResult(await json(response), PRIVILEGED_OUTCOMES, 'unavailable')
  }

  /** Stop running jobs and exit the console (202). */
  async shutdown(): Promise<void> {
    await json(await setupRequest('/shutdown', { method: 'POST', body: '{}', timeoutMs: 15000 }))
  }

  guideUrl(section = ''): string {
    return `${SETUP_API}/guide${section ? `#${section}` : ''}`
  }

  fileUrl(name: string): string {
    return `${SETUP_API}/files/${encodeURIComponent(name)}`
  }
}
