// @vitest-environment jsdom
import { act, cleanup, fireEvent, render, screen, within } from '@testing-library/react'
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { App } from './App'
import { OperatorClient, type SubscribeOptions } from './api'
import { JOB_POLL_MS, LOST_JOB_SUMMARY, parseSetupHash } from './SetupView'
import {
  SetupApiError,
  SetupClient,
  parseSetupState,
  type JobKind,
  type JobLog,
  type JobOptions,
  type ActionResult,
  type JobSummary,
  type K1OpenOutcome,
  type K1Status,
  type PrivilegedOutcome,
  type SessionValues,
  type SetupState,
  type StreamSnapshot,
} from './setupApi'
import { makeStatus, makeWindowsSetupPayload } from './test/fixtures'
import type { BootstrapData, OperatorConfig, OperatorEvent, OperatorStatus, QualificationRun } from './types'

const NOW_MS = Date.UTC(2026, 9, 13, 10, 15, 0)

const config: OperatorConfig = {
  schema_version: 1,
  topology: 'all_in_one',
  runtime_mode: 'fake',
  mocap: { listen_address: '0.0.0.0', port: 5555, source_ip: null },
  robot: { interface: null },
  privacy: { record_raw_mocap: false },
}

class FakeClient extends OperatorClient {
  override bootstrap(): Promise<BootstrapData> {
    return Promise.resolve({ status: makeStatus({ health: { ...makeStatus().health, updated_at: NOW_MS / 1000 } }), config, audit: [], playback: null })
  }
  override getStatus(): Promise<OperatorStatus> { return Promise.resolve(makeStatus()) }
  override getConfig(): Promise<OperatorConfig> { return Promise.resolve(config) }
  override getQualificationRuns(): Promise<QualificationRun[] | null> { return Promise.resolve([]) }
  override subscribe(_handler: (event: OperatorEvent) => void, options: SubscribeOptions = {}): () => void {
    options.onConnection?.(true)
    return () => {}
  }
}

const values: SessionValues = {
  route: 'live',
  session_dir: '/home/u/mocap-sessions/2026-10-13-1015',
  robot: 'unitree_g1',
  human_height_m: 1.75,
  mocap_ws: '/home/u/mocap-runtime',
  motion_bvh: '/home/u/mocap-data/motion.bvh',
  npose_bvh: '/home/u/mocap-data/neutral.bvh',
  offsets_file: '',
  bvh_scale: 0.01,
  start_frame: 0,
  end_frame: 600,
  frame_step: 1,
  npose_start_s: 2,
  npose_duration_s: 1,
  windows_xsens_ip: '192.168.1.246',
  linux_xsens_ip: '192.168.1.20',
}

function makeSetup(overrides: Partial<SetupState> = {}): SetupState {
  return parseSetupState({
    repo_root: '~/prat/mocap_humanoid',
    home: '/home/u',
    session: null,
    defaults: values,
    network: { interfaces: [{ name: 'enp3s0', ipv4: '192.168.1.20' }], detected_linux_ip: '192.168.1.20' },
    firewall_command: 'sudo ufw allow from 192.168.1.246 to 192.168.1.20 port 5555 proto udp',
    readiness: { live_image: null, recorded_runtime: true, motion_bvh: false, npose_bvh: false },
    files: [],
    jobs: [],
    ...overrides,
  })
}

const runningJob: JobSummary = {
  id: 'j-20261013-101500-1',
  kind: 'acceptance',
  label: 'Stream check',
  state: 'running',
  started_at: NOW_MS / 1000,
  finished_at: null,
  exit_code: null,
  summary: null,
  outputs: [],
}

class FakeSetupClient extends SetupClient {
  state: SetupState | null
  readonly reads = vi.fn((): Promise<SetupState | null> => Promise.resolve(this.state))
  readonly saves = vi.fn<(next: SessionValues, replace: boolean) => Promise<SetupState>>((next) => {
    this.state = makeSetup({ session: next })
    return Promise.resolve(this.state)
  })
  readonly starts = vi.fn<(kind: JobKind, options: JobOptions) => Promise<JobSummary>>(() => Promise.resolve(runningJob))
  readonly polls = vi.fn<(id: string, after: number) => Promise<JobLog>>(() => Promise.reject(new Error('unexpected poll')))

  constructor(state: SetupState | null) {
    super()
    this.state = state
  }

  override getSetup() { return this.reads() }
  override saveSession(next: SessionValues, replace = false) { return this.saves(next, replace) }
  override startJob(kind: JobKind, options: JobOptions = {}) { return this.starts(kind, options) }
  override getJob(id: string, after = 0) { return this.polls(id, after) }
  readonly viewerStops = vi.fn(() => Promise.resolve(true))
  readonly stops = vi.fn<(id: string) => Promise<JobSummary>>(() => Promise.resolve(runningJob))
  override stopJob(id: string) { return this.stops(id) }
  override stopViewer() { return this.viewerStops() }
  readonly k1Reads = vi.fn((): Promise<K1Status> => Promise.resolve(k1Status()))
  readonly k1Opens = vi.fn<() => Promise<ActionResult<K1OpenOutcome>>>(() => Promise.resolve({ outcome: 'opened', message: 'The K1 test window is open on this desktop.' }))
  readonly admins = vi.fn<(action: string) => Promise<ActionResult<PrivilegedOutcome>>>(() => Promise.resolve({ outcome: 'done', message: 'Firewall rule added.' }))
  readonly shutdowns = vi.fn(() => Promise.resolve())
  readonly controls = vi.fn<(id: string, command: string) => Promise<number | null>>(() => Promise.resolve(1))
  readonly streams = vi.fn((): Promise<StreamSnapshot> => Promise.resolve(stream({ state: 'none' })))
  override getStream() { return this.streams() }
  override getK1() { return this.k1Reads() }
  override controlJob(id: string, command: 'calibrate' | 'toggle_tracking' | 'rearm') { return this.controls(id, command) }
  override openK1() { return this.k1Opens() }
  override runPrivileged(action: 'firewall' | 'install_tk') { return this.admins(action) }
  override shutdown() { return this.shutdowns() }
}

function stream(overrides: Partial<StreamSnapshot> = {}): StreamSnapshot {
  return {
    state: 'receiving', bind: '0.0.0.0', port: 5555, source: null, senders: [], rate_hz: 60, loss_fraction: 0,
    body_frames: 0, joint_angles: true, last_packet_age_s: null, malformed: 0, ...overrides,
  }
}

function k1Status(overrides: Partial<K1Status> = {}): K1Status {
  return {
    available: true, sdk_version: '1.6.3', tk: true, runtime: true, runtime_dir: '~/mocap-runtime',
    model: '~/mocap-runtime/GMR/assets/booster_k1/K1_serial.xml', model_found: true, installed: true,
    window_open: false, display: true, log: '~/.local/state/mocap-g1/k1-tests.log', ...overrides,
  }
}

async function flush() {
  await act(() => vi.advanceTimersByTimeAsync(0))
}

async function renderApp(setup: SetupState | null) {
  const setupClient = new FakeSetupClient(setup)
  render(<App client={new FakeClient()} setupClient={setupClient} />)
  await flush()
  return setupClient
}

function workspaceButtons() {
  return within(screen.getByRole('navigation', { name: 'Workspace' })).getAllByRole('button')
}

function panel() {
  return document.querySelector('.setup-panel')
}

beforeEach(() => {
  vi.useFakeTimers()
  vi.setSystemTime(NOW_MS)
  window.history.replaceState(null, '', '/')
})

afterEach(() => {
  cleanup()
  vi.useRealTimers()
  window.history.replaceState(null, '', '/')
})

describe('Setup workspace availability', () => {
  it('hides the Setup tab when the workbench returns 404', async () => {
    await renderApp(null)
    expect(workspaceButtons().map((button) => button.textContent)).toEqual(['Session', 'Engineering'])
    expect(screen.getByRole('button', { name: 'Session' }).getAttribute('aria-current')).toBe('page')
    expect(panel()).toBeNull()
  })

  it('shows Setup first and opens it by default when available', async () => {
    await renderApp(makeSetup())
    const buttons = workspaceButtons()
    expect(buttons.map((button) => button.textContent)).toEqual(['Demo', 'Session', 'Engineering'])
    expect(buttons[0].getAttribute('aria-current')).toBe('page')
    expect(panel()?.getAttribute('data-step')).toBe('session')
    // Live route steps only; the action dock belongs to the Session workspace.
    const steps = [...document.querySelectorAll('.setup-stepper li')].map((item) => item.getAttribute('data-step'))
    expect(steps).toEqual(['session', 'windows', 'stream', 'live', 'finish'])
    expect(screen.queryByRole('contentinfo', { name: 'Session actions' })).toBeNull()

    fireEvent.click(screen.getByRole('button', { name: 'Session' }))
    expect(screen.getByRole('contentinfo', { name: 'Session actions' })).toBeTruthy()
  })

  it('shows the recorded steps when the route is switched', async () => {
    await renderApp(makeSetup())
    fireEvent.click(screen.getByRole('radio', { name: 'Recorded' }))
    const steps = [...document.querySelectorAll('.setup-stepper li')].map((item) => item.getAttribute('data-step'))
    expect(steps).toEqual(['session', 'bvh', 'retarget', 'render', 'finish'])
  })
})

describe('Setup session form', () => {
  it('shows a validation error at its field, then saves', async () => {
    const setupClient = await renderApp(makeSetup())
    setupClient.saves.mockImplementationOnce(() => Promise.reject(
      new SetupApiError('Height must be 0.8–2.5 m', 422, 'invalid_session', 'human_height_m'),
    ))
    const height = screen.getByLabelText('Performer height (m)')
    fireEvent.change(height, { target: { value: '3' } })
    fireEvent.click(screen.getByRole('button', { name: 'Save session' }))
    await flush()

    expect(setupClient.saves).toHaveBeenCalledWith(expect.objectContaining({ human_height_m: 3, route: 'live', robot: 'unitree_g1' }), false)
    const fieldLabel = document.querySelector('[data-field="human_height_m"]') as HTMLElement
    expect(within(fieldLabel).getByText('Height must be 0.8–2.5 m')).toBeTruthy()
    expect(height.getAttribute('aria-invalid')).toBe('true')

    fireEvent.change(height, { target: { value: '1.8' } })
    expect(height.getAttribute('aria-invalid')).toBeNull()
    fireEvent.click(screen.getByRole('button', { name: 'Save session' }))
    await flush()

    expect(setupClient.saves).toHaveBeenLastCalledWith(expect.objectContaining({ human_height_m: 1.8 }), false)
    // Paths under the home folder are shown (and sent back) as ~/...
    expect(screen.getByText('Saved ~/mocap-sessions/2026-10-13-1015/session.env')).toBeTruthy()
    expect(setupClient.saves).toHaveBeenLastCalledWith(expect.objectContaining({ session_dir: '~/mocap-sessions/2026-10-13-1015' }), false)
    expect(screen.getByRole('img', { name: 'Done' })).toBeTruthy()
  })

  it('asks before replacing an existing session folder', async () => {
    const setupClient = await renderApp(makeSetup())
    setupClient.saves.mockImplementationOnce(() => Promise.reject(
      new SetupApiError('A session already exists in this folder.', 409, 'session_exists'),
    ))
    fireEvent.click(screen.getByRole('button', { name: 'Save session' }))
    await flush()
    fireEvent.click(screen.getByRole('button', { name: 'Replace it' }))
    await flush()
    expect(setupClient.saves).toHaveBeenLastCalledWith(expect.anything(), true)
  })
})

describe('Setup jobs', () => {
  it('starts a job, polls its log and shows the verdict', async () => {
    const setupClient = await renderApp(makeSetup({ session: values }))
    window.history.replaceState(null, '', '#setup/stream')
    act(() => { window.dispatchEvent(new HashChangeEvent('hashchange')) })
    expect(panel()?.getAttribute('data-step')).toBe('stream')

    setupClient.polls
      .mockResolvedValueOnce({ job: runningJob, lines: ['Listening on 192.168.1.20:5555'], next: 1 })
      .mockResolvedValueOnce({
        job: { ...runningJob, state: 'passed', finished_at: NOW_MS / 1000 + 30, exit_code: 0, summary: 'PASS: 1800 body frames at 60.0 Hz', outputs: ['stream-acceptance-20261013-101500.json'] },
        lines: ['PASS: 1800 body frames at 60.0 Hz'],
        next: 2,
      })
    const readsBefore = setupClient.reads.mock.calls.length

    fireEvent.click(screen.getByRole('button', { name: 'Run 30-second check' }))
    await flush()
    expect(setupClient.starts).toHaveBeenCalledWith('acceptance', {})
    expect(setupClient.polls).toHaveBeenCalledWith(runningJob.id, 0)
    expect(screen.getByRole('log', { name: 'Stream check log' }).textContent).toContain('Listening on 192.168.1.20:5555')
    expect(screen.getByRole('status', { name: 'Stream check: RUNNING' })).toBeTruthy()
    expect(screen.getByRole('button', { name: 'Run 30-second check' })).toHaveProperty('disabled', true)

    await act(() => vi.advanceTimersByTimeAsync(JOB_POLL_MS))
    expect(setupClient.polls).toHaveBeenLastCalledWith(runningJob.id, 1)
    const log = screen.getByRole('log', { name: 'Stream check log' }).textContent
    expect(log).toContain('Listening on 192.168.1.20:5555')
    expect(log).toContain('PASS: 1800 body frames at 60.0 Hz')
    expect(screen.getByRole('status', { name: 'Stream check: PASS' })).toBeTruthy()
    // The finished job triggers a setup refresh, and polling stops.
    expect(setupClient.reads.mock.calls.length).toBe(readsBefore + 1)
    const polls = setupClient.polls.mock.calls.length
    await act(() => vi.advanceTimersByTimeAsync(JOB_POLL_MS * 4))
    expect(setupClient.polls.mock.calls.length).toBe(polls)
  })

  it('shows a rejected start beside the button', async () => {
    window.history.replaceState(null, '', '#setup/session')
    const setupClient = await renderApp(makeSetup({ session: values }))
    setupClient.starts.mockRejectedValueOnce(new SetupApiError('Another job is running.', 409, 'job_running'))
    fireEvent.click(screen.getByRole('button', { name: 'Run system check' }))
    await flush()
    expect(within(panel() as HTMLElement).getByRole('alert').textContent).toContain('Another job is running.')
  })
})

function deferred<T>() {
  let resolve: (value: T) => void = () => {}
  const promise = new Promise<T>((done) => { resolve = done })
  return { promise, resolve }
}

const stoppedJob: JobSummary = { ...runningJob, state: 'stopped', finished_at: NOW_MS / 1000 + 5, exit_code: -2, summary: 'Stopped by the operator.' }

async function startStreamCheck(setupClient: FakeSetupClient) {
  window.history.replaceState(null, '', '#setup/stream')
  act(() => { window.dispatchEvent(new HashChangeEvent('hashchange')) })
  fireEvent.click(screen.getByRole('button', { name: 'Run 30-second check' }))
  await flush()
  expect(setupClient.starts).toHaveBeenCalledTimes(1)
}

describe('Setup stop and lost jobs', () => {
  it('shows a stop as stopping and keeps polling until the job ends', async () => {
    const setupClient = await renderApp(makeSetup({ session: values }))
    setupClient.polls
      .mockResolvedValueOnce({ job: runningJob, lines: ['Listening'], next: 1 })
      .mockResolvedValueOnce({ job: runningJob, lines: ['Stopping receiver'], next: 2 })
      .mockResolvedValueOnce({ job: stoppedJob, lines: ['Stopped by the operator.'], next: 3 })
    await startStreamCheck(setupClient)

    fireEvent.click(within(document.querySelector('.job-console') as HTMLElement).getByRole('button', { name: 'Stop' }))
    await flush()
    expect(setupClient.stops).toHaveBeenCalledWith(runningJob.id)
    // The console accepted the stop but the job is still shutting down.
    expect(screen.getByRole('status', { name: 'Stream check: STOPPING' })).toBeTruthy()
    expect(screen.getByRole('button', { name: 'Stopping…' })).toHaveProperty('disabled', true)

    await act(() => vi.advanceTimersByTimeAsync(JOB_POLL_MS))
    expect(screen.getByRole('status', { name: 'Stream check: STOPPING' })).toBeTruthy()
    await act(() => vi.advanceTimersByTimeAsync(JOB_POLL_MS))
    expect(screen.getByRole('status', { name: 'Stream check: STOPPED' })).toBeTruthy()
    expect(screen.queryByText(/Exit code/)).toBeNull()
    expect(within(panel() as HTMLElement).queryByRole('alert')).toBeNull()
    expect(screen.getByRole('button', { name: 'Run 30-second check' })).toHaveProperty('disabled', false)
  })

  it('appends each log range once when Stop races a poll', async () => {
    const setupClient = await renderApp(makeSetup({ session: values }))
    const slowPoll = deferred<JobLog>()
    setupClient.polls
      .mockResolvedValueOnce({ job: runningJob, lines: ['one'], next: 1 })
      .mockReturnValueOnce(slowPoll.promise)
      .mockResolvedValueOnce({ job: stoppedJob, lines: ['three'], next: 3 })
    setupClient.stops.mockResolvedValueOnce(stoppedJob)
    await startStreamCheck(setupClient)

    await act(() => vi.advanceTimersByTimeAsync(JOB_POLL_MS))
    expect(setupClient.polls).toHaveBeenCalledTimes(2)
    fireEvent.click(within(document.querySelector('.job-console') as HTMLElement).getByRole('button', { name: 'Stop' }))
    await flush()
    slowPoll.resolve({ job: runningJob, lines: ['two'], next: 2 })
    await flush()

    expect(setupClient.polls.mock.calls.map(([, after]) => after)).toEqual([0, 1, 2])
    expect(screen.getByRole('log', { name: 'Stream check log' }).textContent).toBe('one\ntwo\nthree')
  })

  it('releases the UI when a polled job is gone after a console restart', async () => {
    const setupClient = await renderApp(makeSetup({ session: values }))
    setupClient.polls.mockRejectedValueOnce(new SetupApiError('Job not found', 404, 'job_not_found'))
    await startStreamCheck(setupClient)

    expect(screen.getByRole('status', { name: 'Stream check: STOPPED' }).textContent).toContain(LOST_JOB_SUMMARY)
    expect(screen.getByRole('button', { name: 'Run 30-second check' })).toHaveProperty('disabled', false)
    const polls = setupClient.polls.mock.calls.length
    await act(() => vi.advanceTimersByTimeAsync(JOB_POLL_MS * 3))
    expect(setupClient.polls.mock.calls.length).toBe(polls)
  })
})

describe('Setup step state', () => {
  it('uses the console stream check length', async () => {
    window.history.replaceState(null, '', '#setup/stream')
    await renderApp({ ...makeSetup({ session: values }), stream_check_s: 20 })
    expect(screen.getByRole('button', { name: 'Run 20-second check' })).toBeTruthy()
    expect(document.querySelector('.setup-instruction')?.textContent).toContain('Run 20-second check')
  })

  it('draws attention to a failed step instead of a later one', async () => {
    const failed: JobSummary = { ...runningJob, state: 'failed', finished_at: NOW_MS / 1000 + 30, exit_code: 1, summary: 'FAIL: no UDP traffic' }
    await renderApp(makeSetup({ session: values, jobs: [failed] }))
    expect(panel()?.getAttribute('data-step')).toBe('stream')
    const lamp = (step: string) => document.querySelector(`.setup-stepper li[data-step="${step}"] .lamp`)?.getAttribute('aria-label')
    expect(lamp('stream')).toBe('Failed')
    expect(lamp('live')).toBe('Not started')
    expect(document.querySelectorAll('.setup-stepper .lamp-next')).toHaveLength(0)
  })
})

describe('Setup validation by route', () => {
  it('checks only fields shown for the route and reports hidden server errors at form level', async () => {
    const setupClient = await renderApp(makeSetup())
    fireEvent.click(screen.getByRole('radio', { name: 'Recorded' }))
    fireEvent.change(screen.getByLabelText('End frame'), { target: { value: '' } })
    fireEvent.click(screen.getByRole('button', { name: 'Save session' }))
    await flush()
    expect(setupClient.saves).not.toHaveBeenCalled()
    expect(within(document.querySelector('[data-field="end_frame"]') as HTMLElement).getByText('Enter a number.')).toBeTruthy()

    // Switching to Live hides the field and its error; the saved value is kept.
    fireEvent.click(screen.getByRole('radio', { name: 'Live' }))
    expect(screen.queryByText('Enter a number.')).toBeNull()
    setupClient.saves.mockImplementationOnce(() => Promise.reject(
      new SetupApiError('End frame must be after the start frame', 422, 'invalid_session', 'end_frame'),
    ))
    fireEvent.click(screen.getByRole('button', { name: 'Save session' }))
    await flush()
    expect(setupClient.saves).toHaveBeenCalledWith(expect.objectContaining({ route: 'live', end_frame: 600 }), false)
    expect(within(panel() as HTMLElement).getByRole('alert').textContent).toContain('End frame must be after the start frame')
  })
})

describe('Setup recovery and readiness', () => {
  it('offers to stop a leftover viewer, then lets the user retry', async () => {
    window.history.replaceState(null, '', '#setup/live')
    const setupClient = await renderApp(makeSetup({ session: values }))
    setupClient.starts.mockRejectedValueOnce(new SetupApiError('A leftover live viewer is still running.', 409, 'viewer_running'))
    fireEvent.click(screen.getByRole('button', { name: 'Start live simulation' }))
    await flush()
    expect(within(panel() as HTMLElement).getByRole('alert').textContent).toContain('A leftover live viewer is still running.')

    fireEvent.click(screen.getByRole('button', { name: 'Stop leftover viewer' }))
    await flush()
    expect(setupClient.viewerStops).toHaveBeenCalledTimes(1)
    expect(screen.getByText('Leftover viewer stopped. Try again.')).toBeTruthy()
    expect(screen.queryByRole('button', { name: 'Stop leftover viewer' })).toBeNull()
    expect(screen.getByRole('button', { name: 'Start live simulation' })).toHaveProperty('disabled', false)
  })

  it('locks viewer buttons without a desktop display', async () => {
    window.history.replaceState(null, '', '#setup/live')
    const hint = 'No desktop display: open the console from the Mocap Setup menu entry on this PC, not over SSH'
    const base = makeSetup({ session: values })
    await renderApp({ ...base, readiness: { ...base.readiness, display: false, display_hint: hint } })
    expect(screen.getByRole('button', { name: 'Start live simulation' })).toHaveProperty('disabled', true)
    expect(screen.getByText(hint)).toBeTruthy()
  })

  it('round-trips the offsets file it does not show', async () => {
    const setupClient = await renderApp(makeSetup({ session: { ...values, offsets_file: '/home/u/offsets.json' } }))
    fireEvent.click(within(document.querySelector('.setup-stepper li[data-step="session"]') as HTMLElement).getByRole('button'))
    fireEvent.click(screen.getByRole('button', { name: 'Save session' }))
    await flush()
    expect(setupClient.saves).toHaveBeenLastCalledWith(expect.objectContaining({ offsets_file: '/home/u/offsets.json', motion_bvh: '~/mocap-data/motion.bvh' }), false)
  })
})

describe('Setup hash navigation', () => {
  it('parses setup hashes', () => {
    expect(parseSetupHash('#setup')).toEqual({ step: null })
    expect(parseSetupHash('#setup/render')).toEqual({ step: 'render' })
    expect(parseSetupHash('#setup/unknown')).toEqual({ step: null })
    expect(parseSetupHash('#session')).toBeNull()
  })

  it('opens the step named in the URL and switches route to match', async () => {
    window.history.replaceState(null, '', '#setup/bvh')
    await renderApp(makeSetup({ session: values }))
    expect(panel()?.getAttribute('data-step')).toBe('bvh')
    expect(screen.getByRole('radio', { name: 'Recorded' })).toHaveProperty('checked', true)
    expect(document.querySelector('.setup-stepper li[data-step="bvh"] button')?.getAttribute('aria-current')).toBe('step')
  })

  it('switches back to Setup on a hash change from another workspace', async () => {
    await renderApp(makeSetup({ session: values }))
    fireEvent.click(screen.getByRole('button', { name: 'Engineering' }))
    expect(window.location.hash).toBe('')

    window.history.replaceState(null, '', '#setup/windows')
    act(() => { window.dispatchEvent(new HashChangeEvent('hashchange')) })
    expect(screen.getByRole('button', { name: 'Demo' }).getAttribute('aria-current')).toBe('page')
    expect(panel()?.getAttribute('data-step')).toBe('windows')
    expect(screen.getByText('192.168.1.20', { selector: '.value-plate strong' })).toBeTruthy()
  })

  it('opens the next unfinished step and writes the selected step to the URL', async () => {
    await renderApp(makeSetup({ session: values }))
    expect(panel()?.getAttribute('data-step')).toBe('windows')
    expect(window.location.hash).toBe('#setup/windows')
    fireEvent.click(within(document.querySelector('.setup-stepper li[data-step="live"]') as HTMLElement).getByRole('button'))
    expect(window.location.hash).toBe('#setup/live')
    expect(panel()?.getAttribute('data-step')).toBe('live')
  })
})

function makeWindowsSetup(overrides: Record<string, unknown> = {}): SetupState {
  return parseSetupState(makeWindowsSetupPayload(overrides))
}

const windowsValues = makeWindowsSetupPayload().defaults as SessionValues
const windowsOtherPc: SessionValues = { ...windowsValues, windows_xsens_ip: '192.168.1.246', linux_xsens_ip: '192.168.1.20' }

function openStep(step: string) {
  window.history.replaceState(null, '', `#setup/${step}`)
  act(() => { window.dispatchEvent(new HashChangeEvent('hashchange')) })
}

describe('Setup on Linux without platform fields', () => {
  it('keeps the Linux wording, guide numbers and UFW block', async () => {
    window.history.replaceState(null, '', '#setup/session')
    await renderApp(makeSetup({ session: values }))
    expect([...document.querySelectorAll('.step-number')].map((item) => item.textContent)).toEqual(['01', '02', '03', '04', '05'])
    expect(screen.getByLabelText('Windows Xsens IP')).toBeTruthy()
    expect(screen.getByLabelText('Linux Xsens IP')).toBeTruthy()
    expect(screen.queryByRole('radiogroup', { name: 'Xsens Humanoid runs on' })).toBeNull()
    expect(screen.getByText('Live image')).toBeTruthy()
    expect(screen.queryByText('Live runtime')).toBeNull()

    openStep('windows')
    expect(document.querySelector('#setup-step-title')?.textContent).toBe('Windows: suit and streaming')
    expect(screen.getByText('Linux firewall')).toBeTruthy()
    expect(screen.getByText(/sudo ufw allow/)).toBeTruthy()
    expect(screen.queryByText('Windows Firewall')).toBeNull()

    openStep('stream')
    expect(screen.getByText('From Windows')).toBeTruthy()
    expect(screen.getByText('To Linux')).toBeTruthy()
  })

  it('runs Xsens MVN on this Ubuntu PC: 127.0.0.1, no receiver IP, no firewall rule', async () => {
    window.history.replaceState(null, '', '#setup/session')
    const setupClient = await renderApp(makeSetup({ session: values }))
    expect(screen.getByRole('radio', { name: 'Another PC' })).toHaveProperty('checked', true)
    fireEvent.click(screen.getByRole('radio', { name: 'This PC' }))
    expect(screen.queryByLabelText('Windows Xsens IP')).toBeNull()
    expect(screen.queryByLabelText('Linux Xsens IP')).toBeNull()
    expect(screen.getByText(/Network Streamer destination/).textContent).toContain('In Xsens MVN set Network Streamer destination 127.0.0.1')

    fireEvent.click(screen.getByRole('button', { name: 'Save session' }))
    await flush()
    expect(setupClient.saves).toHaveBeenLastCalledWith(
      expect.objectContaining({ windows_xsens_ip: '127.0.0.1', linux_xsens_ip: '' }), false)

    openStep('windows')
    expect(screen.getByText('Streaming to 127.0.0.1 stays on this PC, so no firewall change is needed.')).toBeTruthy()
    expect(screen.queryByText(/sudo ufw allow/)).toBeNull()
    openStep('stream')
    expect(screen.getByText('127.0.0.1:5555')).toBeTruthy()
    expect(screen.getByText('To this PC')).toBeTruthy()
  })
})

describe('Setup on Windows', () => {
  it('uses plain step numbers instead of the Linux guide numbers', async () => {
    await renderApp(makeWindowsSetup({ session: windowsValues }))
    expect([...document.querySelectorAll('.step-number')].map((item) => item.textContent)).toEqual(['01', '02', '03', '04', '05'])
    expect([...document.querySelectorAll('.step-title')].map((item) => item.textContent)).toContain('Xsens: suit and streaming')
    expect(document.querySelector('.setup-panel-head .silk')?.textContent).toBe('Step 02 · live route')
    expect(document.body.textContent).not.toMatch(/guide step|04–05|ufw|sudo|bash |SSH/i)
  })

  it('writes 127.0.0.1 and an empty receiver IP when Xsens runs on this PC', async () => {
    const setupClient = await renderApp(makeWindowsSetup())
    setupClient.saves.mockImplementation((next) => {
      setupClient.state = makeWindowsSetup({ session: next })
      return Promise.resolve(setupClient.state)
    })
    expect(screen.getByRole('radio', { name: 'This PC' })).toHaveProperty('checked', true)
    expect(screen.queryByLabelText('Xsens PC IP')).toBeNull()
    expect(screen.queryByLabelText("This PC's IP")).toBeNull()
    expect(screen.getByText(/Network Streamer destination/).textContent).toContain('127.0.0.1')
    expect(screen.getByText(/Network Streamer destination/).textContent).toContain('port 5555')

    fireEvent.click(screen.getByRole('button', { name: 'Save session' }))
    await flush()
    expect(setupClient.saves).toHaveBeenLastCalledWith(
      expect.objectContaining({ windows_xsens_ip: '127.0.0.1', linux_xsens_ip: '', session_dir: '~/mocap-sessions/2026-10-13-1400' }),
      false,
    )
    expect(screen.getByText('Saved ~/mocap-sessions/2026-10-13-1400/session.env')).toBeTruthy()
  })

  it('shows the runtime folder on the live route and saves an edited value', async () => {
    const setupClient = await renderApp(makeWindowsSetup())
    setupClient.saves.mockImplementation((next) => {
      setupClient.state = makeWindowsSetup({ session: next })
      return Promise.resolve(setupClient.state)
    })
    const runtime = screen.getByLabelText<HTMLInputElement>('Runtime folder')
    fireEvent.change(runtime, { target: { value: '~/other-runtime' } })
    fireEvent.click(screen.getByRole('button', { name: 'Save session' }))
    await flush()
    expect(setupClient.saves).toHaveBeenLastCalledWith(expect.objectContaining({ mocap_ws: '~/other-runtime' }), false)
  })

  it('shows both IP fields for another PC and saves their values', async () => {
    const setupClient = await renderApp(makeWindowsSetup())
    setupClient.saves.mockImplementation((next) => {
      setupClient.state = makeWindowsSetup({ session: next, xsens_on_this_pc: false })
      return Promise.resolve(setupClient.state)
    })
    fireEvent.click(screen.getByRole('radio', { name: 'Another PC' }))
    const sourceIp = screen.getByLabelText<HTMLInputElement>('Xsens PC IP')
    // The loopback source is cleared; the receiver select offers the OS interfaces.
    expect(sourceIp.value).toBe('')
    const receiver = screen.getByLabelText<HTMLSelectElement>("This PC's IP")
    expect(within(receiver).getByRole('option', { name: '192.168.1.20 · Ethernet' })).toBeTruthy()
    expect(screen.getByText(/ipconfig/)).toBeTruthy()

    fireEvent.change(sourceIp, { target: { value: '192.168.1.246' } })
    fireEvent.change(receiver, { target: { value: '192.168.1.20' } })
    fireEvent.click(screen.getByRole('button', { name: 'Save session' }))
    await flush()
    expect(setupClient.saves).toHaveBeenLastCalledWith(expect.objectContaining({ windows_xsens_ip: '192.168.1.246', linux_xsens_ip: '192.168.1.20' }), false)

    fireEvent.click(screen.getByRole('radio', { name: 'This PC' }))
    expect(screen.queryByLabelText('Xsens PC IP')).toBeNull()
    fireEvent.click(screen.getByRole('button', { name: 'Save session' }))
    await flush()
    expect(setupClient.saves).toHaveBeenLastCalledWith(expect.objectContaining({ windows_xsens_ip: '127.0.0.1', linux_xsens_ip: '' }), false)
  })

  it('starts on another PC when the saved source is not loopback, and prefills the detected receiver IP', async () => {
    window.history.replaceState(null, '', '#setup/session')
    await renderApp(makeWindowsSetup({
      session: { ...windowsValues, windows_xsens_ip: '192.168.1.246', linux_xsens_ip: '' },
      xsens_on_this_pc: false,
      network: { interfaces: [{ name: 'Ethernet', ipv4: '192.168.1.20' }], detected_linux_ip: '192.168.1.20' },
    }))
    expect(screen.getByRole('radio', { name: 'Another PC' })).toHaveProperty('checked', true)
    expect(screen.getByLabelText<HTMLInputElement>('Xsens PC IP').value).toBe('192.168.1.246')
    expect(screen.getByLabelText<HTMLSelectElement>("This PC's IP").value).toBe('192.168.1.20')
  })

  it('takes the radio state from xsens_on_this_pc when no source IP is set', async () => {
    await renderApp(makeWindowsSetup({ defaults: { ...windowsValues, windows_xsens_ip: '' }, xsens_on_this_pc: true }))
    expect(screen.getByRole('radio', { name: 'This PC' })).toHaveProperty('checked', true)
  })

  it('shows the loopback destination and a no-change firewall note on this PC', async () => {
    await renderApp(makeWindowsSetup({ session: windowsValues }))
    openStep('windows')
    expect(document.querySelector('#setup-step-title')?.textContent).toBe('Xsens: suit and streaming')
    expect(screen.getByText('127.0.0.1', { selector: '.value-plate strong' })).toBeTruthy()
    expect(screen.getByText('Windows Firewall')).toBeTruthy()
    expect(screen.getByText(/no firewall change is needed/)).toBeTruthy()
    expect(screen.queryByText('Linux firewall')).toBeNull()
  })

  it('shows this PC\'s saved IP and the Private-network note for another PC', async () => {
    await renderApp(makeWindowsSetup({ session: windowsOtherPc, xsens_on_this_pc: false }))
    openStep('windows')
    expect(screen.getByText('192.168.1.20', { selector: '.value-plate strong' })).toBeTruthy()
    expect(screen.getByText("This PC's IP", { selector: '.value-plate small' })).toBeTruthy()
    expect(screen.getByText(/Private networks/)).toBeTruthy()
    expect(screen.getByText(/UDP 5555/)).toBeTruthy()
    expect(screen.queryByText(/ufw/i)).toBeNull()
  })

  it('never renders the UFW block, even if a firewall command is sent', async () => {
    await renderApp(makeWindowsSetup({ session: windowsOtherPc, firewall_command: 'sudo ufw allow 5555' }))
    openStep('windows')
    expect(screen.queryByText(/ufw/i)).toBeNull()
    expect(screen.queryByText('Linux firewall')).toBeNull()
  })

  it('relabels the stream check and its troubleshooting', async () => {
    await renderApp(makeWindowsSetup({ session: windowsValues }))
    openStep('stream')
    expect(screen.getByText('From Xsens')).toBeTruthy()
    const toPc = screen.getByText('To this PC').closest('.value-plate') as HTMLElement
    expect(within(toPc).getByText('127.0.0.1:5555')).toBeTruthy()
    const row = screen.getByText('unexpected source').closest('tr') as HTMLElement
    expect(row.textContent).toContain('Xsens source IP changed')
    expect(row.textContent).toContain('not this PC\'s LAN IP')
    expect(screen.queryByText('From Windows')).toBeNull()
  })

  it('shows the live runtime readiness instead of the live image', async () => {
    const setup = makeWindowsSetup()
    await renderApp(setup)
    const item = screen.getByText('Live runtime').closest('li') as HTMLElement
    expect(item.textContent).toContain('C:/Users/me/mocap-runtime'.replace('C:/Users/me', '~'))
    expect(item.textContent).toContain('FOUND')
    expect(screen.queryByText('Live image')).toBeNull()
    cleanup()
    await renderApp(makeWindowsSetup({ readiness: { live_runtime: false, display: true } }))
    expect((screen.getByText('Live runtime').closest('li') as HTMLElement).textContent).toContain('MISSING')
    cleanup()
    await renderApp(makeWindowsSetup({ readiness: { display: true } }))
    expect((screen.getByText('Live runtime').closest('li') as HTMLElement).textContent).toContain('NOT CHECKED')
  })

  it('accepts a Windows home with different casing and shows saved paths with a tilde', async () => {
    window.history.replaceState(null, '', '#setup/session')
    await renderApp(makeWindowsSetup({ home: 'c:/users/ME', session: windowsValues }))
    expect(screen.getByText('~/mocap-sessions/2026-10-13-1400/session.env', { exact: false })).toBeTruthy()
  })

  it('uses a Windows-neutral message when there is no display', async () => {
    const setup = makeWindowsSetup({ session: windowsValues, readiness: { display: false } })
    window.history.replaceState(null, '', '#setup/live')
    await renderApp(setup)
    expect(screen.getByText('No desktop display: viewer windows cannot open.')).toBeTruthy()
    expect(document.body.textContent).not.toMatch(/SSH/)
  })
})

describe('Setup desktop tools (Linux checkout console)', () => {
  const desktop = (overrides: Partial<SetupState> = {}) => makeSetup({ session: values, desktop_tools: true, firewall_state: 'active', ...overrides })

  function goTo(step: string) {
    window.history.replaceState(null, '', `#setup/${step}`)
    act(() => { window.dispatchEvent(new HashChangeEvent('hashchange')) })
  }

  it('adds step 13 only on the desktop console', async () => {
    await renderApp(desktop())
    const steps = [...document.querySelectorAll('.setup-stepper li')].map((item) => item.getAttribute('data-step'))
    expect(steps).toEqual(['session', 'windows', 'stream', 'live', 'k1', 'finish'])
    // 01, 02, 03; Live simulation and Real K1 share 04 (choose one); Finish 05
    expect([...document.querySelectorAll('.setup-stepper .step-number')].map((item) => item.textContent)).toEqual(['01', '02', '03', '04', '04', '05'])
    // Optional: never marked as the next step.
    expect(document.querySelector('[data-step="k1"]')?.textContent).not.toContain('Next')
    cleanup()
    await renderApp(makeSetup({ session: values }))
    expect(document.querySelector('.setup-stepper li[data-step="k1"]')).toBeNull()
  })

  it('shows the K1 tools and opens the K1 test window', async () => {
    const setupClient = await renderApp(desktop())
    goTo('k1')
    await flush()
    expect(setupClient.k1Reads).toHaveBeenCalled()
    expect(screen.getByText('version 1.6.3')).toBeTruthy()
    expect(screen.getByRole('button', { name: 'Reinstall K1 test tools' })).toBeTruthy()
    expect(screen.queryByRole('button', { name: 'Install Tk' })).toBeNull()
    fireEvent.click(screen.getByRole('button', { name: 'Open K1 test window' }))
    await flush()
    expect(setupClient.k1Opens).toHaveBeenCalledTimes(1)
    expect(screen.getByText('The K1 test window is open on this desktop.')).toBeTruthy()

    setupClient.k1Opens.mockResolvedValueOnce({ outcome: 'already_open', message: '' })
    fireEvent.click(screen.getByRole('button', { name: 'Open K1 test window' }))
    await flush()
    expect(screen.getByText('Already open, or an install is running. Look for the K1 window on this desktop.')).toBeTruthy()
    expect(screen.getByRole('link', { name: 'Open the guide' }).getAttribute('href')).toBe('/api/v1/setup/guide#step-13')
  })

  it('installs the K1 tools as a job, locks it while the window is open, and offers Install Tk', async () => {
    const setupClient = await renderApp(desktop())
    setupClient.k1Reads.mockResolvedValue(k1Status({ tk: false, installed: false, window_open: true }))
    goTo('k1')
    await flush()
    expect(screen.getByRole('button', { name: 'Install K1 test tools' })).toHaveProperty('disabled', true)
    expect(screen.getByText(/Close the K1 test window before you install/)).toBeTruthy()
    fireEvent.click(screen.getByRole('button', { name: 'Install Tk' }))
    await flush()
    expect(setupClient.admins).toHaveBeenCalledWith('install_tk')

    setupClient.k1Reads.mockResolvedValue(k1Status({ installed: false, runtime: false }))
    fireEvent.click(screen.getByRole('button', { name: 'Check again' }))
    await flush()
    fireEvent.click(screen.getByRole('button', { name: 'Install K1 test tools' }))
    await flush()
    expect(setupClient.starts).toHaveBeenCalledWith('k1_install', {})
  })

  it('allows the stream in the firewall and reports each outcome', async () => {
    const setupClient = await renderApp(desktop())
    goTo('windows')
    expect(screen.getByText('UFW is active on this PC: allow the Xsens stream through it.')).toBeTruthy()
    fireEvent.click(screen.getByRole('button', { name: 'Allow in firewall' }))
    await flush()
    expect(setupClient.admins).toHaveBeenCalledWith('firewall')
    expect(screen.getByText('Firewall rule added.')).toBeTruthy()

    setupClient.admins.mockResolvedValueOnce({ outcome: 'cancelled', message: 'Cancelled: nothing was changed.' })
    fireEvent.click(screen.getByRole('button', { name: 'Allow in firewall' }))
    await flush()
    expect(screen.getByText('Cancelled: nothing was changed.')).toBeTruthy()
    // The Copy fallback stays.
    expect(screen.getByRole('button', { name: 'Copy firewall command' })).toBeTruthy()
  })

  it('hides Allow in firewall when UFW is off, and keeps it when the state is unknown', async () => {
    await renderApp(desktop({ firewall_state: 'inactive' }))
    goTo('windows')
    expect(screen.queryByRole('button', { name: 'Allow in firewall' })).toBeNull()
    expect(screen.getByText('UFW is off on this PC: no firewall change is needed.')).toBeTruthy()
    cleanup()
    await renderApp(desktop({ firewall_state: 'unknown' }))
    goTo('windows')
    expect(screen.getByRole('button', { name: 'Allow in firewall' })).toBeTruthy()
  })

  it('offers the K1 arm stream on step 07 for the K1 only, with its port', async () => {
    let setupClient = await renderApp(desktop())
    goTo('live')
    expect(screen.queryByRole('checkbox', { name: /Send the arms to the K1 window/ })).toBeNull()
    fireEvent.click(screen.getByRole('button', { name: 'Start live simulation' }))
    await flush()
    expect(setupClient.starts).toHaveBeenCalledWith('live', { calibrate_after_s: null, record_video: false })
    cleanup()
    setupClient = await renderApp(desktop({ session: { ...values, robot: 'booster_k1' } }))
    goTo('live')
    await flush()
    // K1 tools installed: ticked by default.
    const box = screen.getByRole<HTMLInputElement>('checkbox', { name: /Send the arms to the K1 window/ })
    expect(box.checked).toBe(true)
    expect(screen.getByText('udp://127.0.0.1:47011')).toBeTruthy()
    fireEvent.click(screen.getByRole('button', { name: 'Start live simulation' }))
    await flush()
    expect(setupClient.starts).toHaveBeenLastCalledWith('live', { calibrate_after_s: null, record_video: false, robot_stream: true })
    fireEvent.click(box)
    expect(box.checked).toBe(false)
    cleanup()
    setupClient = new FakeSetupClient(desktop({ session: { ...values, robot: 'booster_k1' } }))
    setupClient.k1Reads.mockResolvedValue(k1Status({ installed: false }))
    window.history.replaceState(null, '', '#setup/live')
    render(<App client={new FakeClient()} setupClient={setupClient} />)
    await flush()
    expect((screen.getByRole<HTMLInputElement>('checkbox', { name: /Send the arms to the K1 window/ })).checked).toBe(false)
  })

  it('calibrates and toggles tracking of the running live simulation with buttons', async () => {
    const live = { enabled: false, reason: 'waiting for calibration', fps: 60, frames: 120, drops: 0, failures: 0, calibrated: false }
    const liveJob: JobSummary = { ...runningJob, kind: 'live', label: 'Live simulation', live }
    const setupClient = new FakeSetupClient(desktop({ session: { ...values, robot: 'booster_k1' }, jobs: [liveJob] }))
    setupClient.polls.mockResolvedValue({ job: liveJob, lines: [], next: 0 })
    window.history.replaceState(null, '', '#setup/live')
    render(<App client={new FakeClient()} setupClient={setupClient} />)
    await flush()
    await flush()
    expect(screen.getByText(/The K1 head takes its zero at Calibrate/)).toBeTruthy()
    const flags = document.querySelector('.live-tracking')?.textContent
    expect(flags).toContain('Calibrated no')
    expect(flags).toContain('Tracking on no')
    fireEvent.click(screen.getByRole('button', { name: /^Calibrate/ }))
    fireEvent.click(screen.getByRole('button', { name: 'Calibrate now' }))
    await flush()
    expect(setupClient.controls).toHaveBeenCalledWith(liveJob.id, 'calibrate')
    setupClient.controls.mockRejectedValueOnce(new SetupApiError('The live simulation is no longer reading commands', 409, 'job_not_controllable'))
    fireEvent.click(screen.getByRole('button', { name: /^Tracking on/ }))
    await flush()
    expect(setupClient.controls).toHaveBeenLastCalledWith(liveJob.id, 'toggle_tracking')
    expect(screen.getByRole('alert').textContent).toContain('no longer running or reading commands')
  })

  it('starts G1 + T1 side by side with the auto-calibrate value', async () => {
    const setupClient = await renderApp(desktop())
    goTo('live')
    fireEvent.click(screen.getByRole('checkbox', { name: /Auto-calibrate after/ }))
    fireEvent.click(screen.getByRole('button', { name: 'Start G1 + T1 side by side' }))
    await flush()
    expect(setupClient.starts).toHaveBeenCalledWith('compare', { calibrate_after_s: 5 })
  })

  it('points a missing recorded runtime at step 13', async () => {
    const setupClient = await renderApp(desktop({ session: { ...values, route: 'recorded' } }))
    setupClient.starts.mockRejectedValueOnce(new SetupApiError('Recorded runtime not found', 409, 'recorded_runtime_missing'))
    goTo('retarget')
    fireEvent.click(screen.getByRole('button', { name: 'Make preview' }))
    await flush()
    fireEvent.click(screen.getByRole('button', { name: 'Install it in the Real K1 step' }))
    expect(panel()?.getAttribute('data-step')).toBe('k1')
  })

  it('shuts down the console after a confirmation', async () => {
    const setupClient = await renderApp(desktop())
    goTo('finish')
    fireEvent.click(screen.getByRole('button', { name: 'Shut down the console' }))
    expect(setupClient.shutdowns).not.toHaveBeenCalled()
    fireEvent.click(screen.getByRole('button', { name: 'Cancel' }))
    fireEvent.click(screen.getByRole('button', { name: 'Shut down the console' }))
    fireEvent.click(screen.getByRole('button', { name: 'Shut down' }))
    await flush()
    expect(setupClient.shutdowns).toHaveBeenCalledTimes(1)
    expect(screen.getByText(/The console is shutting down/)).toBeTruthy()
  })
})

describe('Live K1 helpers (plan C1, C2, C3)', () => {
  const desktop = (overrides: Partial<SetupState> = {}) => makeSetup({ session: values, desktop_tools: true, firewall_state: 'active', ...overrides })
  const k1Session = { ...values, robot: 'booster_k1' as const }
  const liveStatus = { enabled: true, reason: '', fps: 60, frames: 120, drops: 0, failures: 0, calibrated: true }

  async function openLive(job: JobSummary, setup = desktop({ session: k1Session, jobs: [job] })) {
    const setupClient = new FakeSetupClient(setup)
    setupClient.polls.mockResolvedValue({ job, lines: [], next: 0 })
    window.history.replaceState(null, '', '#setup/live')
    render(<App client={new FakeClient()} setupClient={setupClient} />)
    await flush()
    await flush()
    return setupClient
  }

  async function calibrateAndAnswer(results: NonNullable<JobSummary['calibration']>['results'], jobOverrides: Partial<JobSummary> = {}) {
    const liveJob: JobSummary = { ...runningJob, kind: 'live', label: 'Live simulation', live: liveStatus, calibration: { sent: 0, results: [] } }
    const setupClient = await openLive(liveJob)
    setupClient.controls.mockResolvedValue(2)
    fireEvent.click(screen.getByRole('button', { name: /^Calibrate/ }))
    fireEvent.click(screen.getByRole('button', { name: 'Calibrate now' }))
    await flush()
    expect(screen.getByRole('status', { name: 'Calibration result' }).textContent).toBe('Calibrating…')
    setupClient.polls.mockResolvedValue({ job: { ...liveJob, calibration: { sent: 2, results }, ...jobOverrides }, lines: [], next: 0 })
    await act(() => vi.advanceTimersByTimeAsync(JOB_POLL_MS))
    await flush()
    return screen.queryByRole('status', { name: 'Calibration result' })?.textContent ?? null
  }

  const result = (request: number | null, overrides = {}) => ({ request, ok: true, epoch: 2, epoch_changed: true, warning: null, reason: null, ...overrides })

  it('shows the performer steps first and sends nothing on Cancel', async () => {
    const liveJob: JobSummary = { ...runningJob, kind: 'live', label: 'Live simulation', live: liveStatus }
    const setupClient = await openLive(liveJob)
    fireEvent.click(screen.getByRole('button', { name: /^Calibrate/ }))
    const ask = screen.getByRole('alertdialog', { name: 'Calibrate now?' })
    expect(ask.textContent).toContain('Stand up straight.')
    expect(ask.textContent).toContain('Arms down at the sides.')
    expect(ask.textContent).toContain('Look straight ahead.')
    expect(ask.textContent).toContain('Hold still')
    fireEvent.click(screen.getByRole('button', { name: 'Cancel' }))
    await flush()
    expect(setupClient.controls).not.toHaveBeenCalled()
    expect(screen.queryByRole('alertdialog', { name: 'Calibrate now?' })).toBeNull()
  })

  it('shows Calibrated for its own request only', async () => {
    expect(await calibrateAndAnswer([result(1, { ok: false, reason: 'old' }), result(2)])).toBe('Calibrated.')
  })

  it('shows the pose warning with the result', async () => {
    const text = await calibrateAndAnswer([result(2, { warning: 'the performer is not standing (thigh 80° from vertical).' })])
    expect(text).toBe('Calibrated, but the performer is not standing (thigh 80° from vertical).')
  })

  it('shows a partial and a full failure', async () => {
    expect(await calibrateAndAnswer([result(2, { ok: false, reason: 'floor calibration failed: no feet' })]))
      .toBe('Calibrated, but floor calibration failed: no feet')
    cleanup()
    expect(await calibrateAndAnswer([result(2, { ok: false, epoch_changed: false, reason: 'no frame yet' })]))
      .toBe('Calibration failed: no frame yet. The earlier calibration stays. Calibrate again.')
  })

  it('says when the job ended before it answered', async () => {
    const text = await calibrateAndAnswer([], { state: 'stopped', finished_at: NOW_MS / 1000 + 1 })
    expect(text).toBe('The live simulation stopped before it answered the Calibrate.')
  })

  it('ignores a result from another job and says No answer after 5 s', async () => {
    const liveJob: JobSummary = { ...runningJob, kind: 'live', label: 'Live simulation', live: liveStatus, calibration: { sent: 0, results: [] } }
    const setupClient = await openLive(liveJob)
    setupClient.controls.mockResolvedValue(1)
    fireEvent.click(screen.getByRole('button', { name: /^Calibrate/ }))
    fireEvent.click(screen.getByRole('button', { name: 'Calibrate now' }))
    await flush()
    // Same request number, other job: a new live job replaced the one we asked.
    const other: JobSummary = { ...liveJob, id: 'j-20261013-101600-2', calibration: { sent: 1, results: [result(1)] } }
    setupClient.polls.mockResolvedValue({ job: { ...liveJob, calibration: { sent: 1, results: [] } }, lines: [], next: 0 })
    setupClient.state = desktop({ session: k1Session, jobs: [liveJob, other] })
    await act(() => vi.advanceTimersByTimeAsync(5000))
    await flush()
    expect(screen.getByRole('status', { name: 'Calibration result' }).textContent).toContain('No answer from the live simulation')
    // A late answer still shows.
    setupClient.polls.mockResolvedValue({ job: { ...liveJob, calibration: { sent: 1, results: [result(1)] } }, lines: [], next: 0 })
    await act(() => vi.advanceTimersByTimeAsync(JOB_POLL_MS))
    await flush()
    expect(screen.getByRole('status', { name: 'Calibration result' }).textContent).toBe('Calibrated.')
  })

  it('never offers auto-calibration for the K1 arm stream', async () => {
    const setupClient = new FakeSetupClient(desktop({ session: k1Session }))
    window.history.replaceState(null, '', '#setup/live')
    render(<App client={new FakeClient()} setupClient={setupClient} />)
    await flush()
    await flush()
    expect(screen.queryByRole('checkbox', { name: /Auto-calibrate after/ })).toBeNull()
    fireEvent.click(screen.getByRole('button', { name: 'Start live simulation' }))
    await flush()
    expect(setupClient.starts).toHaveBeenLastCalledWith('live', { calibrate_after_s: null, record_video: false, robot_stream: true })
  })

  async function openStream(snapshot: StreamSnapshot) {
    const setupClient = new FakeSetupClient(desktop())
    setupClient.streams.mockResolvedValue(snapshot)
    window.history.replaceState(null, '', '#setup/stream')
    render(<App client={new FakeClient()} setupClient={setupClient} />)
    await flush()
    await flush()
    return setupClient
  }

  const senderText = () => screen.getByRole('status', { name: 'Xsens sender' }).textContent

  it('offers Use <address> for data from another sender and saves it', async () => {
    const setupClient = await openStream(stream({ senders: ['192.168.88.11'], source: '192.168.88.11:50000', body_frames: 300 }))
    expect(senderText()).toContain('arrives from 192.168.88.11, but the session expects 192.168.1.246')
    fireEvent.click(screen.getByRole('button', { name: 'Use 192.168.88.11' }))
    await flush()
    expect(setupClient.saves).toHaveBeenCalledWith({ ...values, windows_xsens_ip: '192.168.88.11' }, false)
    expect(senderText()).toContain('stop it and start it again')
    expect(senderText()).toContain('does not fix network routing or a firewall')
  })

  it('saves a sender on this PC (127.0.0.1) with an empty Linux IP', async () => {
    const setupClient = await openStream(stream({ senders: ['127.0.0.1'], source: '127.0.0.1:50000', body_frames: 300 }))
    fireEvent.click(screen.getByRole('button', { name: 'Use 127.0.0.1' }))
    await flush()
    expect(setupClient.saves).toHaveBeenCalledWith({ ...values, windows_xsens_ip: '127.0.0.1', linux_xsens_ip: '' }, false)
  })

  it('shows the configured sender, two senders, stale and no data without a button', async () => {
    await openStream(stream({ senders: ['192.168.1.246'], body_frames: 300 }))
    expect(senderText()).toContain('arrives from 192.168.1.246, the source in this session')
    expect(screen.queryByRole('button', { name: /^Use / })).toBeNull()
    cleanup()
    await openStream(stream({ senders: ['192.168.1.246', '192.168.88.11'], body_frames: 300 }))
    expect(senderText()).toContain('2 senders: 192.168.1.246, 192.168.88.11')
    expect(screen.queryByRole('button', { name: /^Use / })).toBeNull()
    cleanup()
    await openStream(stream({ state: 'stale', senders: [], body_frames: 300, last_packet_age_s: 7.2, source: '192.168.88.11:50000' }))
    expect(senderText()).toContain('No new Xsens data for 7 s (last from 192.168.88.11)')
    expect(screen.queryByRole('button', { name: /^Use / })).toBeNull()
    cleanup()
    await openStream(stream({ state: 'none' }))
    expect(senderText()).toContain('No Xsens data on UDP port 5555')
    cleanup()
    await openStream(stream({ state: 'port_busy' }))
    expect(senderText()).toContain('Another program uses UDP port 5555')
  })

  it('shows the running version and the last update in step 03', async () => {
    const software = { commit: 'e3962ae 2026-11-16', update_last: { line: '2026-11-16T09:00:00+01:00 FAIL e3962ae -> e3962ae', result: 'FAIL' as const, details: ['git pull failed: run Mocap Update again.'] } }
    await renderApp(desktop({ software }))
    window.history.replaceState(null, '', '#setup/session')
    act(() => { window.dispatchEvent(new HashChangeEvent('hashchange')) })
    const group = screen.getByRole('group', { name: 'Software' })
    expect(group.textContent).toContain('e3962ae 2026-11-16')
    expect(group.textContent).toContain('2026-11-16T09:00:00+01:00 FAIL e3962ae -> e3962ae')
    expect(group.textContent).toContain('The last update failed. git pull failed: run Mocap Update again.')
    expect(group.textContent).toContain('To update: stop the console')
    expect(group.textContent).toContain('Mocap Update')
    cleanup()
    await renderApp(desktop({ software: { commit: 'e3962ae 2026-11-16', update_last: null } }))
    expect(screen.getByRole('group', { name: 'Software' }).textContent).toContain('Mocap Update has not run on this PC')
  })
})
