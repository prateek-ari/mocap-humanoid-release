import { afterEach, beforeEach, describe, expect, it, vi, type Mock } from 'vitest'
import { DOWNLOAD_URL_REVOKE_DELAY_MS, MAX_CONSECUTIVE_STREAM_FAILURES, OperatorClient, STATUS_POLL_DEBOUNCE_MS } from './api'
import { makeStatus } from './test/fixtures'
import type { OperatorConfig, OperatorEvent, OperatorStatus } from './types'

type Listener = (event: { data?: string }) => void

class FakeSocket {
  static instances: FakeSocket[] = []
  readonly listeners = new Map<string, Listener[]>()
  closed = false

  constructor(readonly url: string) {
    FakeSocket.instances.push(this)
  }

  addEventListener(type: string, listener: Listener) {
    this.listeners.set(type, [...(this.listeners.get(type) ?? []), listener])
  }

  close() {
    if (this.closed) return
    this.closed = true
    this.emit('close', {})
  }

  emit(type: string, event: { data?: string }) {
    for (const listener of this.listeners.get(type) ?? []) listener(event)
  }

  open() {
    this.emit('open', {})
  }

  message(payload: unknown) {
    this.emit('message', { data: JSON.stringify(payload) })
  }

  raw(data: string) {
    this.emit('message', { data })
  }
}

const config: OperatorConfig = {
  schema_version: 1,
  topology: 'all_in_one',
  runtime_mode: 'real',
  mocap: { listen_address: '0.0.0.0', port: 5555, source_ip: null },
  robot: { interface: null },
  privacy: { record_raw_mocap: true },
}

function json(body: unknown, status = 200): Response {
  return new Response(JSON.stringify(body), { status, headers: { 'content-type': 'application/json' } })
}

function snapshot(status: OperatorStatus, sequence = status.last_event_sequence): OperatorEvent {
  return { schema_version: 1, instance_id: status.instance_id, sequence, type: 'snapshot', status }
}

let fetchMock: Mock<(input: string | URL, init?: RequestInit) => Promise<Response>>
let liveStatus: OperatorStatus

beforeEach(() => {
  FakeSocket.instances = []
  liveStatus = makeStatus({ state: 'idle', last_event_sequence: 10 })
  fetchMock = vi.fn((input: string | URL) => {
    const url = input.toString()
    if (url.endsWith('/api/v1/status')) return Promise.resolve(json(liveStatus))
    if (url.endsWith('/api/v1/config')) return Promise.resolve(json(config))
    if (url.endsWith('/api/v1/audit?newest=true')) return Promise.resolve(json({ records: [], last_sequence: 0 }))
    if (url.endsWith('/api/v1/playback')) return Promise.resolve(json({ detail: 'none' }, 404))
    return Promise.resolve(json({ detail: 'unexpected' }, 500))
  })
  vi.stubGlobal('fetch', fetchMock)
  vi.stubGlobal('WebSocket', FakeSocket)
  vi.stubGlobal('window', { location: { protocol: 'http:', host: '127.0.0.1:8766' } })
})

afterEach(() => {
  vi.unstubAllGlobals()
  vi.useRealTimers()
})

describe('bootstrap', () => {
  it('never writes configuration, even when raw mocap recording is reported on', async () => {
    const data = await new OperatorClient().bootstrap()

    expect(data.config.privacy.record_raw_mocap).toBe(true)
    expect(fetchMock).toHaveBeenCalled()
    for (const [, init] of fetchMock.mock.calls as [unknown, RequestInit | undefined][]) {
      expect(init?.method ?? 'GET').toBe('GET')
      expect(init?.body).toBeUndefined()
    }
  })
})

describe('subscribe restart detection', () => {
  it('re-bootstraps instead of resuming when the supervisor instance changes', () => {
    const events: OperatorEvent[] = []
    const onReset = vi.fn()
    const onConnection = vi.fn()
    const stop = new OperatorClient().subscribe((event) => events.push(event), {
      onConnection,
      onReset,
      afterSequence: 10,
      instanceId: 'instance-a',
    })
    const socket = FakeSocket.instances[0]
    expect(socket.url).toContain('after_sequence=10')

    socket.message(snapshot(liveStatus))
    expect(events).toHaveLength(1)

    const restarted = makeStatus({ instance_id: 'instance-b', state: 'idle', last_event_sequence: 2 })
    socket.message(snapshot(restarted, 2))

    expect(onReset).toHaveBeenCalledTimes(1)
    expect(onConnection).toHaveBeenLastCalledWith(false)
    expect(events).toHaveLength(1)
    expect(socket.closed).toBe(true)
    expect(FakeSocket.instances).toHaveLength(1)
    stop()
  })

  it('re-bootstraps when the supervisor counter is behind the client cursor', () => {
    const onReset = vi.fn()
    new OperatorClient().subscribe(() => {}, { onReset, afterSequence: 50 })
    const socket = FakeSocket.instances[0]

    socket.message(snapshot(makeStatus({ instance_id: undefined, last_event_sequence: 3 }), 3))

    expect(onReset).toHaveBeenCalledTimes(1)
    expect(socket.closed).toBe(true)
  })

  it('resumes normally on the same instance', () => {
    const onReset = vi.fn()
    const events: OperatorEvent[] = []
    const stop = new OperatorClient().subscribe((event) => events.push(event), { onReset, afterSequence: 10, instanceId: 'instance-a' })
    const socket = FakeSocket.instances[0]

    socket.message(snapshot(liveStatus))
    socket.message({ schema_version: 1, instance_id: 'instance-a', sequence: 11, audit_sequence: 340, type: 'control', action: 'check' })

    expect(onReset).not.toHaveBeenCalled()
    expect(events.map((event) => event.sequence)).toEqual([10, 11])
    stop()
  })
})

function statusCalls(): number {
  return fetchMock.mock.calls.filter(([input]) => input.toString().endsWith('/api/v1/status')).length
}

function controlEvent(sequence: number, extra: Partial<OperatorEvent> = {}): OperatorEvent {
  return { schema_version: 1, instance_id: 'instance-a', sequence, audit_sequence: 300 + sequence, type: 'control', action: 'check', outcome: 'accepted', ...extra }
}

function openStream(afterSequence = 10) {
  const events: OperatorEvent[] = []
  const onConnection = vi.fn()
  const onReset = vi.fn()
  const stop = new OperatorClient().subscribe((event) => events.push(event), { onConnection, onReset, afterSequence, instanceId: 'instance-a' })
  const socket = FakeSocket.instances[0]
  socket.open()
  return { events, onConnection, onReset, stop, socket }
}

describe('event payload guard', () => {
  it('drops and logs malformed payloads without applying or closing the stream', () => {
    const warn = vi.spyOn(console, 'warn').mockImplementation(() => {})
    const { events, onConnection, socket, stop } = openStream()

    socket.raw('{not json')
    socket.message({ ...snapshot(liveStatus), status: { ...liveStatus, state: 'warp_speed' } })

    expect(events).toHaveLength(0)
    expect(socket.closed).toBe(false)
    expect(onConnection).toHaveBeenLastCalledWith(true)
    expect(warn).toHaveBeenCalled()

    socket.message(snapshot(liveStatus))
    expect(events).toHaveLength(1)
    stop()
    warn.mockRestore()
  })

  it('reports the stream lost after consecutive malformed payloads and recovers on a valid one', () => {
    const warn = vi.spyOn(console, 'warn').mockImplementation(() => {})
    const { events, onConnection, socket, stop } = openStream()

    for (let i = 0; i < MAX_CONSECUTIVE_STREAM_FAILURES - 1; i += 1) socket.message({ sequence: -1 })
    expect(onConnection).toHaveBeenLastCalledWith(true)
    socket.message({ type: 'snapshot', sequence: 11, schema_version: 1 })
    expect(onConnection).toHaveBeenLastCalledWith(false)
    expect(events).toHaveLength(0)

    socket.message(snapshot(liveStatus))
    expect(onConnection).toHaveBeenLastCalledWith(true)
    expect(events).toHaveLength(1)
    stop()
    warn.mockRestore()
  })
})

describe('status polling', () => {
  it('trusts a status carried by the event and never polls for it', async () => {
    vi.useFakeTimers()
    const { events, socket, stop } = openStream()
    const tracking = makeStatus({ state: 'tracking', last_event_sequence: 11 })

    socket.message(controlEvent(11, { status: tracking }))
    await vi.advanceTimersByTimeAsync(STATUS_POLL_DEBOUNCE_MS * 4)

    expect(statusCalls()).toBe(0)
    expect(events.map((event) => event.status?.state)).toEqual(['tracking'])
    stop()
  })

  it('polls once, after the debounce, for a burst of events without a status', async () => {
    vi.useFakeTimers()
    const { events, socket, stop } = openStream()
    liveStatus = makeStatus({ state: 'checking', last_event_sequence: 13 })

    socket.message(controlEvent(11))
    socket.message(controlEvent(12))
    socket.message(controlEvent(13))
    expect(statusCalls()).toBe(0)
    await vi.advanceTimersByTimeAsync(STATUS_POLL_DEBOUNCE_MS)

    expect(statusCalls()).toBe(1)
    expect(events.at(-1)).toMatchObject({ type: 'snapshot', sequence: 13, status: { state: 'checking' } })
    stop()
  })

  it('skips the poll when a newer snapshot arrives first', async () => {
    vi.useFakeTimers()
    const { socket, stop } = openStream()

    socket.message(controlEvent(11))
    socket.message(snapshot(makeStatus({ last_event_sequence: 11 }), 11))
    await vi.advanceTimersByTimeAsync(STATUS_POLL_DEBOUNCE_MS * 4)

    expect(statusCalls()).toBe(0)
    stop()
  })

  it('does not report the stream lost on a single failed poll', async () => {
    vi.useFakeTimers()
    const warn = vi.spyOn(console, 'warn').mockImplementation(() => {})
    fetchMock.mockImplementation(() => Promise.reject(new Error('timeout')))
    const { onConnection, socket, stop } = openStream()

    socket.message(controlEvent(11))
    await vi.advanceTimersByTimeAsync(STATUS_POLL_DEBOUNCE_MS)
    expect(statusCalls()).toBe(1)
    expect(onConnection).not.toHaveBeenCalledWith(false)

    for (let sequence = 12; statusCalls() < MAX_CONSECUTIVE_STREAM_FAILURES; sequence += 1) {
      socket.message(controlEvent(sequence))
      await vi.advanceTimersByTimeAsync(STATUS_POLL_DEBOUNCE_MS)
    }
    expect(onConnection).toHaveBeenLastCalledWith(false)
    stop()
    warn.mockRestore()
  })

  it('rejects a malformed status response instead of applying it', async () => {
    fetchMock.mockImplementation(() => Promise.resolve(json({ ...liveStatus, controls: null })))
    await expect(new OperatorClient().getStatus()).rejects.toThrow(/status format/)
  })
})

describe('saveConfig', () => {
  it('sends the typed operator name only as a claim header', async () => {
    fetchMock.mockImplementation(() => Promise.resolve(json(config)))
    const client = new OperatorClient()

    await client.saveConfig(config, '  alice ')
    await client.saveConfig(config, '')

    const [first, second] = fetchMock.mock.calls as [unknown, RequestInit][]
    expect(first[1].method).toBe('PUT')
    expect((first[1].headers as Record<string, string>)['x-operator-name']).toBe('alice')
    expect((second[1].headers as Record<string, string>)['x-operator-name']).toBeUndefined()
  })
})

describe('downloadDiagnostics', () => {
  it('revokes the object URL only after the download has started', async () => {
    vi.useFakeTimers()
    fetchMock.mockImplementation(() => Promise.resolve(new Response('{}', {
      status: 200,
      headers: { 'content-disposition': 'attachment; filename="diag.json"' },
    })))
    const anchor = { href: '', download: '', click: vi.fn(), remove: vi.fn() }
    vi.stubGlobal('document', {
      createElement: vi.fn(() => anchor),
      body: { appendChild: vi.fn() },
    })
    const createObjectURL = vi.spyOn(URL, 'createObjectURL').mockReturnValue('blob:diag')
    const revokeObjectURL = vi.spyOn(URL, 'revokeObjectURL').mockImplementation(() => {})

    await new OperatorClient().downloadDiagnostics()

    expect(anchor.click).toHaveBeenCalledTimes(1)
    expect(anchor.download).toBe('diag.json')
    expect(revokeObjectURL).not.toHaveBeenCalled()
    vi.advanceTimersByTime(DOWNLOAD_URL_REVOKE_DELAY_MS)
    expect(revokeObjectURL).toHaveBeenCalledWith('blob:diag')
    createObjectURL.mockRestore()
    revokeObjectURL.mockRestore()
  })
})
