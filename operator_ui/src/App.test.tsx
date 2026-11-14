// @vitest-environment jsdom
import { act, cleanup, fireEvent, render, screen } from '@testing-library/react'
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { App } from './App'
import { OperatorClient, type SubscribeOptions } from './api'
import { makeStatus } from './test/fixtures'
import type { ActionName, BootstrapData, OperatorConfig, OperatorEvent, OperatorStatus, QualificationRun } from './types'

const NOW_MS = Date.UTC(2026, 9, 12, 12, 0, 0)

const config: OperatorConfig = {
  schema_version: 1,
  topology: 'all_in_one',
  runtime_mode: 'real',
  mocap: { listen_address: '0.0.0.0', port: 5555, source_ip: null },
  robot: { interface: null },
  privacy: { record_raw_mocap: false },
}

/** In-memory supervisor: no network, and the test drives the event stream. */
class FakeClient extends OperatorClient {
  status: OperatorStatus
  handler: ((event: OperatorEvent) => void) | null = null
  options: SubscribeOptions = {}
  readonly actions = vi.fn((action: ActionName): Promise<OperatorStatus> => Promise.resolve({ ...this.status, state: action === 'disengage' ? 'armed_disabled' : this.status.state, last_event_sequence: this.status.last_event_sequence + 1 }))
  readonly statusReads = vi.fn((): Promise<OperatorStatus> => Promise.resolve(this.status))
  readonly configWrites = vi.fn((next: OperatorConfig): Promise<OperatorConfig> => Promise.resolve(next))

  constructor(status: OperatorStatus) {
    super()
    this.status = status
  }

  override bootstrap(): Promise<BootstrapData> {
    return Promise.resolve({ status: this.status, config, audit: [], playback: null })
  }

  override getStatus(): Promise<OperatorStatus> {
    return this.statusReads()
  }

  override getConfig(): Promise<OperatorConfig> {
    return Promise.resolve(config)
  }

  override getQualificationRuns(): Promise<QualificationRun[] | null> {
    return Promise.resolve([])
  }

  override subscribe(handler: (event: OperatorEvent) => void, options: SubscribeOptions = {}): () => void {
    this.handler = handler
    this.options = options
    return () => { this.handler = null }
  }

  override action(action: ActionName): Promise<OperatorStatus> {
    return this.actions(action)
  }

  override saveConfig(next: OperatorConfig): Promise<OperatorConfig> {
    return this.configWrites(next)
  }

  connect(connected: boolean) {
    act(() => this.options.onConnection?.(connected))
  }
}

async function flush() {
  await act(() => vi.advanceTimersByTimeAsync(0))
}

async function renderApp(status: OperatorStatus = makeStatus({ health: { ...makeStatus().health, updated_at: NOW_MS / 1000 - 5 } })) {
  const client = new FakeClient(status)
  render(<App client={client} />)
  await flush()
  return client
}

beforeEach(() => {
  vi.useFakeTimers()
  vi.setSystemTime(NOW_MS)
})

afterEach(() => {
  cleanup()
  vi.useRealTimers()
})

describe('App', () => {
  it('renders the session from the bootstrap status', async () => {
    const client = await renderApp()
    client.connect(true)

    expect(screen.getByText('TRACKING')).toBeTruthy()
    expect(screen.getByRole('button', { name: /^Disengage\./ }).getAttribute('aria-disabled')).toBe('false')
    expect(screen.queryByText(/Supervisor connection lost/)).toBeNull()
  })

  it('shows a ticking age for the last update while the stream is lost', async () => {
    const client = await renderApp()
    client.connect(false)

    const banner = screen.getByText(/Supervisor connection lost/).closest('.stale-banner')
    expect(banner?.textContent).toMatch(/Last update \S+ \(5s ago\)/)
    await act(() => vi.advanceTimersByTimeAsync(3000))
    expect(banner?.textContent).toMatch(/\(8s ago\)/)
  })

  it('applies statuses from the event stream', async () => {
    const client = await renderApp()
    client.connect(true)
    act(() => client.handler?.({ schema_version: 1, sequence: 11, type: 'snapshot', status: makeStatus({ state: 'armed_disabled', last_event_sequence: 11 }) }))
    expect(screen.getByText('ARMED · OUTPUT OFF')).toBeTruthy()
  })

  it('surfaces supervisor diagnostics', async () => {
    await renderApp(makeStatus({ runtime_adapter: { blocked_call: 'start_tracking' }, watchdog: { running: false, overdue: false } }))
    expect(screen.getByText(/blocked in an overdue start_tracking call/)).toBeTruthy()
    expect(screen.getByText(/watchdog is not running/)).toBeTruthy()
  })
})

describe('DISENGAGE shortcut', () => {
  it('disengages on Escape, even while the stream is lost', async () => {
    const client = await renderApp()
    client.connect(false)

    fireEvent.keyDown(document.body, { key: 'Escape' })
    await flush()

    expect(client.actions).toHaveBeenCalledTimes(1)
    expect(client.actions).toHaveBeenCalledWith('disengage')
  })

  it('never fires from a text field, a modified key or a key repeat', async () => {
    const client = await renderApp()
    client.connect(true)

    fireEvent.keyDown(screen.getByLabelText('Operator'), { key: 'Escape' })
    fireEvent.keyDown(document.body, { key: 'Escape', ctrlKey: true })
    fireEvent.keyDown(document.body, { key: 'Escape', repeat: true })
    await flush()

    expect(client.actions).not.toHaveBeenCalled()
  })

  it('submits once while the first request is still pending', async () => {
    const client = await renderApp()
    client.connect(true)
    let resolve: (status: OperatorStatus) => void = () => {}
    client.actions.mockImplementationOnce(() => new Promise((done) => { resolve = done }))

    fireEvent.keyDown(document.body, { key: 'Escape' })
    fireEvent.keyDown(document.body, { key: 'Escape' })
    await flush()
    expect(client.actions).toHaveBeenCalledTimes(1)

    resolve(makeStatus({ state: 'armed_disabled', last_event_sequence: 11 }))
    await flush()
    expect(screen.getByText('ARMED · OUTPUT OFF')).toBeTruthy()
  })

  it('does nothing in read-only recorded playback', async () => {
    const client = await renderApp(makeStatus({ presentation_mode: 'recorded_playback' }))
    client.connect(true)
    fireEvent.keyDown(document.body, { key: 'Escape' })
    await flush()
    expect(client.actions).not.toHaveBeenCalled()
  })
})

describe('saving configuration', () => {
  it('re-reads status through the ordering guard instead of patching it', async () => {
    const client = await renderApp(makeStatus({ state: 'idle', last_event_sequence: 20 }))
    client.connect(true)
    client.statusReads.mockResolvedValueOnce(makeStatus({ state: 'checking', topology: 'split', last_event_sequence: 19 }))

    fireEvent.click(screen.getByRole('button', { name: 'Engineering' }))
    fireEvent.click(screen.getByRole('button', { name: 'Save configuration' }))
    await flush()

    expect(client.configWrites).toHaveBeenCalledTimes(1)
    expect(client.statusReads).toHaveBeenCalledTimes(1)
    fireEvent.click(screen.getByRole('button', { name: 'Session' }))
    // The older status (sequence 19) must not replace the shown one (20).
    expect(screen.getByText('IDLE')).toBeTruthy()
  })
})
