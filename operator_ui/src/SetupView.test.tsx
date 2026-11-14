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
  type JobSummary,
  type SessionValues,
  type SetupState,
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
    expect(buttons.map((button) => button.textContent)).toEqual(['Setup', 'Session', 'Engineering'])
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
    expect(screen.getByRole('button', { name: 'Setup' }).getAttribute('aria-current')).toBe('page')
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
    expect([...document.querySelectorAll('.step-number')].map((item) => item.textContent)).toEqual(['03', '04–05', '06', '07', '11'])
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
})

describe('Setup on Windows', () => {
  it('uses plain step numbers instead of the Linux guide numbers', async () => {
    await renderApp(makeWindowsSetup({ session: windowsValues }))
    expect([...document.querySelectorAll('.step-number')].map((item) => item.textContent)).toEqual(['1', '2', '3', '4', '5'])
    expect([...document.querySelectorAll('.step-title')].map((item) => item.textContent)).toContain('Xsens: suit and streaming')
    expect(document.querySelector('.setup-panel-head .silk')?.textContent).toBe('Step 2 · live route')
    expect(document.body.textContent).not.toMatch(/guide step|04–05|step 0\d|ufw|sudo|bash |SSH/i)
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
