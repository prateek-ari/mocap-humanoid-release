// @vitest-environment jsdom
import { act, cleanup, fireEvent, render, screen, within } from '@testing-library/react'
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { App } from './App'
import { OperatorClient, type SubscribeOptions } from './api'
import { LIVE_STALE_MS, SETTINGS_SAVE_MS, STREAM_POLL_MS } from './LiveApp'
import { JOB_POLL_MS } from './SetupView'
import {
  SetupApiError,
  SetupClient,
  parseSetupState,
  parseStreamSnapshot,
  type ControlCommand,
  type JobKind,
  type JobLog,
  type JobOptions,
  type JobSummary,
  type Prefs,
  type SessionValues,
  type SetupState,
  type StreamSnapshot,
} from './setupApi'
import { makeAppSetupPayload, makeStatus, makeStreamSnapshot } from './test/fixtures'
import type { BootstrapData, OperatorConfig, OperatorEvent, OperatorStatus, QualificationRun } from './types'

const NOW_MS = Date.UTC(2026, 9, 14, 9, 0, 0)

const config: OperatorConfig = {
  schema_version: 1,
  topology: 'all_in_one',
  runtime_mode: 'fake',
  mocap: { listen_address: '0.0.0.0', port: 5555, source_ip: null },
  robot: { interface: null },
  privacy: { record_raw_mocap: false },
}

class FakeClient extends OperatorClient {
  readonly actions = vi.fn((): Promise<OperatorStatus> => Promise.resolve(makeStatus()))
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
  override action(): Promise<OperatorStatus> { return this.actions() }
}

const SESSION_DIR = 'C:/Users/me/mocap-sessions/2026-10-13-1400'
const FRESH_DIR = 'C:/Users/me/mocap-sessions/2026-10-14-0905'

function makeApp(overrides: Record<string, unknown> = {}): SetupState {
  return parseSetupState(makeAppSetupPayload(overrides))
}

function liveJob(overrides: Partial<JobSummary> = {}): JobSummary {
  return {
    id: 'j-live-1',
    kind: 'live',
    label: 'Live simulation',
    state: 'running',
    started_at: NOW_MS / 1000,
    finished_at: null,
    exit_code: null,
    summary: null,
    outputs: [],
    live: null,
    ...overrides,
  }
}

class FakeSetupClient extends SetupClient {
  state: SetupState
  stream: StreamSnapshot = parseStreamSnapshot(makeStreamSnapshot())!
  readonly reads = vi.fn((): Promise<SetupState | null> => Promise.resolve(this.state))
  readonly streams = vi.fn((): Promise<StreamSnapshot> => Promise.resolve(this.stream))
  readonly saves = vi.fn<(next: SessionValues) => Promise<SetupState>>((next) => {
    this.state = { ...this.state, session: next }
    return Promise.resolve(this.state)
  })
  readonly starts = vi.fn<(kind: JobKind, options: JobOptions) => Promise<JobSummary>>((kind) => Promise.resolve(
    kind === 'live' ? liveJob() : liveJob({ id: `j-${kind}`, kind, label: kind === 'doctor' ? 'System check' : 'Stream check' }),
  ))
  readonly polls = vi.fn<(id: string, after: number) => Promise<JobLog>>((id, after) => Promise.resolve({ job: liveJob({ id }), lines: [], next: after }))
  readonly stops = vi.fn<(id: string) => Promise<JobSummary>>((id) => Promise.resolve(liveJob({ id })))
  readonly prefsWrites = vi.fn<(changes: Partial<Prefs>) => Promise<Prefs>>((changes) => Promise.resolve({ ...this.state.prefs, ...changes }))
  readonly controls = vi.fn<(id: string, command: ControlCommand) => Promise<void>>(() => Promise.resolve())

  constructor(state: SetupState) {
    super()
    this.state = state
  }

  override getSetup() { return this.reads() }
  override getStream() { return this.streams() }
  override saveSession(next: SessionValues) { return this.saves(next) }
  override startJob(kind: JobKind, options: JobOptions = {}) { return this.starts(kind, options) }
  override getJob(id: string, after = 0) { return this.polls(id, after) }
  override stopJob(id: string) { return this.stops(id) }
  override putPrefs(changes: Partial<Omit<Prefs, 'schema'>>) { return this.prefsWrites(changes) }
  override controlJob(id: string, command: ControlCommand) { return this.controls(id, command) }
}

async function flush(ms = 0) {
  await act(() => vi.advanceTimersByTimeAsync(ms))
}

async function renderApp(setup: SetupState) {
  const setupClient = new FakeSetupClient(setup)
  const client = new FakeClient()
  render(<App client={client} setupClient={setupClient} />)
  await flush()
  return { setupClient, client }
}

function streamPanel() {
  return screen.getByRole('region', { name: 'Xsens stream' })
}

beforeEach(() => {
  vi.useFakeTimers()
  vi.setSystemTime(NOW_MS)
})

afterEach(() => {
  cleanup()
  vi.useRealTimers()
})

describe('Desktop app screen', () => {
  it('replaces the workspaces with one screen', async () => {
    await renderApp(makeApp())
    expect(screen.getByText('MOCAP HUMANOID')).toBeTruthy()
    expect(screen.getByText('v0.4.0')).toBeTruthy()
    expect(screen.queryByRole('navigation', { name: 'Workspace' })).toBeNull()
    expect(document.querySelector('.setup-workspace')).toBeNull()
    expect(document.title).toBe('Mocap Humanoid')
    expect(screen.getByRole('button', { name: 'Start' })).toBeTruthy()
  })

  it('keeps the Setup workbench when the console is not the app', async () => {
    await renderApp(makeApp({ app: null }))
    expect(screen.getByRole('navigation', { name: 'Workspace' })).toBeTruthy()
    expect(screen.queryByText('MOCAP HUMANOID')).toBeNull()
  })

  it('shows the stream state with text and follows the monitor', async () => {
    const { setupClient } = await renderApp(makeApp())
    const panel = streamPanel()
    expect(within(panel).getByText('Receiving')).toBeTruthy()
    expect(within(panel).getByText('60.0')).toBeTruthy()
    expect(within(panel).getByText('127.0.0.1:50123')).toBeTruthy()
    expect(screen.getByRole('status').textContent).toContain('Ready · Xsens receiving')

    setupClient.stream = parseStreamSnapshot(makeStreamSnapshot({ state: 'none', source: null, rate_hz: null, loss_fraction: null, body_frames: 0, joint_angles: false }))!
    await flush(STREAM_POLL_MS)
    expect(within(panel).getByText('Waiting for Xsens')).toBeTruthy()
    expect(within(panel).getAllByText('not reported').length).toBeGreaterThan(0)

    setupClient.stream = parseStreamSnapshot(makeStreamSnapshot({ state: 'port_busy' }))!
    await flush(STREAM_POLL_MS)
    expect(within(panel).getByText('Port in use')).toBeTruthy()

    setupClient.streams.mockImplementation(() => Promise.reject(new SetupApiError('Console unavailable: offline', 0)))
    await flush(STREAM_POLL_MS)
    expect(within(panel).getByText('Console unavailable: offline')).toBeTruthy()
  })

  it('flags a stream without joint angles', async () => {
    const { setupClient } = await renderApp(makeApp())
    setupClient.stream = parseStreamSnapshot(makeStreamSnapshot({ joint_angles: false, rate_hz: 40 }))!
    await flush(STREAM_POLL_MS)
    expect(within(streamPanel()).getByText('No')).toBeTruthy()
    expect(document.querySelectorAll('.live-readouts .tone-warning').length).toBe(2)
  })
})

describe('Desktop app simulation', () => {
  it('saves the session, starts the simulation and drives its controls', async () => {
    const { setupClient } = await renderApp(makeApp({ prefs: { schema: 1, record_video: true, calibrate_after_s: 4 } }))
    fireEvent.click(screen.getByRole('radio', { name: /Booster T1/ }))
    fireEvent.change(screen.getByLabelText('Performer height (m)'), { target: { value: '1.68' } })
    fireEvent.click(screen.getByRole('button', { name: 'Start' }))
    await flush()

    expect(setupClient.saves).toHaveBeenCalledTimes(1)
    expect(setupClient.saves.mock.calls[0][0]).toMatchObject({ route: 'live', robot: 'booster_t1_29dof', human_height_m: 1.68, windows_xsens_ip: '127.0.0.1', linux_xsens_ip: '', session_dir: SESSION_DIR })
    expect(setupClient.starts).toHaveBeenCalledWith('live', { record_video: true, calibrate_after_s: 4 })
    expect(screen.getByText('Waiting for the first Xsens frame…')).toBeTruthy()
    expect(screen.getByRole('button', { name: /Calibrate/ }).hasAttribute('disabled')).toBe(true)
    // Settings are locked: a summary replaces the form while the simulation runs.
    expect(screen.queryByLabelText('Performer height (m)')).toBeNull()
    const summary = screen.getByRole('region', { name: /Session · locked/ })
    expect(within(summary).getByText('Booster T1')).toBeTruthy()
    expect(within(summary).getByText('1.68 m')).toBeTruthy()

    setupClient.polls.mockImplementation((id, after) => Promise.resolve({
      job: liveJob({ id, live: { enabled: false, reason: 'waiting for calibration', fps: 59.6, frames: 900, drops: 0, failures: 0, calibrated: false } }),
      lines: ['live 900 frames'],
      next: after + 1,
    }))
    await flush(JOB_POLL_MS)
    expect(screen.getByText('59.6')).toBeTruthy()
    expect(screen.getByText('Simulation running in the MuJoCo window.', { exact: false })).toBeTruthy()
    expect(screen.getByText('waiting for calibration')).toBeTruthy()
    expect(within(streamPanel()).getByText('Receiving')).toBeTruthy()

    // Xsens goes quiet: STATUS keeps coming, but the frame count stops moving.
    await flush(LIVE_STALE_MS)
    expect(within(streamPanel()).getByText('No new frames')).toBeTruthy()
    expect(screen.getByText('Simulation · no Xsens frames')).toBeTruthy()
    setupClient.polls.mockImplementation((id, after) => Promise.resolve({
      job: liveJob({ id, live: { enabled: false, reason: 'waiting for calibration', fps: 59.6, frames: 960, drops: 0, failures: 0, calibrated: false } }),
      lines: [],
      next: after,
    }))
    await flush(JOB_POLL_MS)
    expect(within(streamPanel()).getByText('Receiving')).toBeTruthy()

    fireEvent.click(screen.getByRole('button', { name: /Calibrate/ }))
    await flush()
    expect(setupClient.controls).toHaveBeenCalledWith('j-live-1', 'calibrate')
    fireEvent.click(screen.getByRole('button', { name: /Tracking on/ }))
    await flush()
    expect(setupClient.controls).toHaveBeenLastCalledWith('j-live-1', 'toggle_tracking')

    setupClient.controls.mockImplementationOnce(() => Promise.reject(new SetupApiError('not running', 409)))
    fireEvent.click(screen.getByRole('button', { name: /Rearm/ }))
    await flush()
    expect(screen.getByRole('alert').textContent).toContain('The simulation is no longer running.')

    fireEvent.click(screen.getByRole('button', { name: 'Stop' }))
    await flush()
    expect(setupClient.stops).toHaveBeenCalledWith('j-live-1')
    expect(screen.getByRole('button', { name: 'Stopping…' })).toBeTruthy()
  })

  it('starts a run in a fresh folder once the current one holds a run', async () => {
    const finished = liveJob({ state: 'stopped', finished_at: NOW_MS / 1000 + 60 })
    const setup = makeApp({ session: { ...makeApp().defaults, session_dir: SESSION_DIR }, jobs: [finished] })
    const { setupClient } = await renderApp(setup)
    setupClient.state = { ...setupClient.state, defaults: { ...setupClient.state.defaults, session_dir: FRESH_DIR } }
    fireEvent.click(screen.getByRole('button', { name: 'Start' }))
    await flush()
    expect(setupClient.saves.mock.calls[0][0].session_dir).toBe(FRESH_DIR)
  })

  it('shows start errors inline, with field errors at their field', async () => {
    const { setupClient } = await renderApp(makeApp())
    setupClient.saves.mockImplementationOnce(() => Promise.reject(new SetupApiError('Height must be 0.8–2.5 m', 422, 'invalid_session', 'HUMAN_HEIGHT_M')))
    fireEvent.click(screen.getByRole('button', { name: 'Start' }))
    await flush()
    const height = screen.getByLabelText('Performer height (m)')
    expect(height.getAttribute('aria-invalid')).toBe('true')
    expect(screen.getByText('Height must be 0.8–2.5 m')).toBeTruthy()

    setupClient.starts.mockImplementationOnce(() => Promise.reject(new SetupApiError('No desktop display', 409, 'no_display')))
    fireEvent.click(screen.getByRole('button', { name: 'Start' }))
    await flush()
    expect(screen.getByRole('alert').textContent).toContain('No desktop display')
  })

  it('needs the Xsens PC IP when Xsens runs on another PC', async () => {
    const { setupClient } = await renderApp(makeApp())
    fireEvent.click(screen.getByRole('radio', { name: 'Another PC' }))
    expect(screen.getByRole('option', { name: '192.168.1.20 · Ethernet' })).toBeTruthy()
    fireEvent.click(screen.getByRole('button', { name: 'Start' }))
    await flush()
    expect(setupClient.starts).not.toHaveBeenCalled()
    expect(screen.getByText("Enter the Xsens PC's IPv4 address.")).toBeTruthy()

    fireEvent.change(screen.getByLabelText('Xsens PC IP'), { target: { value: '192.168.1.246' } })
    fireEvent.change(screen.getByLabelText("This PC's IP"), { target: { value: '192.168.1.20' } })
    fireEvent.click(screen.getByRole('button', { name: 'Start' }))
    await flush()
    expect(setupClient.saves).toHaveBeenLastCalledWith(expect.objectContaining({ windows_xsens_ip: '192.168.1.246', linux_xsens_ip: '192.168.1.20' }))
  })
})

describe('Desktop app settings', () => {
  it('saves edited settings into the session after a pause', async () => {
    const { setupClient } = await renderApp(makeApp({ session: { ...makeApp().defaults, session_dir: SESSION_DIR } }))
    fireEvent.change(screen.getByLabelText('Performer height (m)'), { target: { value: '1.9' } })
    await flush(SETTINGS_SAVE_MS - 100)
    expect(setupClient.saves).not.toHaveBeenCalled()
    await flush(200)
    expect(setupClient.saves).toHaveBeenCalledWith(expect.objectContaining({ human_height_m: 1.9, session_dir: SESSION_DIR }))
    expect(screen.getByText('Settings saved')).toBeTruthy()
  })

  it('saves preferences and checks the calibration delay', async () => {
    const { setupClient } = await renderApp(makeApp())
    fireEvent.click(screen.getByRole('checkbox', { name: 'Record video' }))
    await flush()
    expect(setupClient.prefsWrites).toHaveBeenCalledWith({ record_video: true })

    fireEvent.click(screen.getByRole('checkbox', { name: 'Auto-calibrate after' }))
    await flush()
    expect(setupClient.prefsWrites).toHaveBeenLastCalledWith({ calibrate_after_s: 5 })
    fireEvent.change(screen.getByLabelText('Seconds before auto-calibration'), { target: { value: '90' } })
    expect(screen.getByText('Use 0.5 to 60 seconds.')).toBeTruthy()
    expect(screen.getByRole('button', { name: 'Start' }).hasAttribute('disabled')).toBe(true)

    setupClient.prefsWrites.mockImplementationOnce(() => Promise.reject(new SetupApiError('calibrate_after_s must be 0.5–60', 400)))
    fireEvent.change(screen.getByLabelText('Seconds before auto-calibration'), { target: { value: '3' } })
    await flush(SETTINGS_SAVE_MS)
    expect(screen.getByText('Not saved: calibrate_after_s must be 0.5–60')).toBeTruthy()
  })
})

describe('Desktop app tools', () => {
  it('opens the drawer, runs the stream check and closes on Escape without touching the supervisor', async () => {
    const { setupClient, client } = await renderApp(makeApp())
    setupClient.polls.mockImplementation((id, after) => Promise.resolve({ job: liveJob({ id, kind: 'acceptance', label: 'Stream check' }), lines: [], next: after }))
    const toolsButton = screen.getByRole('button', { name: 'Tools' })
    fireEvent.click(toolsButton)
    const drawer = screen.getByRole('complementary', { name: 'Tools' })
    expect(document.activeElement).toBe(within(drawer).getByRole('heading', { name: 'Tools' }))
    fireEvent.click(within(drawer).getByRole('button', { name: 'Run 30-second stream check' }))
    await flush()
    // No session yet: the check's folder is saved first.
    expect(setupClient.saves).toHaveBeenCalledTimes(1)
    expect(setupClient.starts).toHaveBeenCalledWith('acceptance', {})
    expect(within(streamPanel()).getByText('Checking')).toBeTruthy()

    fireEvent.keyDown(window, { key: 'Escape' })
    expect(drawer.hidden).toBe(true)
    expect(document.activeElement).toBe(toolsButton)
    expect(client.actions).not.toHaveBeenCalled()
  })
})
