import { isRecord, parseOperatorEvent, parseOperatorStatus } from './protocol'
import type {
  ActionName,
  AuditResponse,
  BootstrapData,
  OperatorConfig,
  OperatorEvent,
  OperatorStatus,
  PlaybackManifest,
  PreflightPayload,
  QualificationRun,
  QualificationStage,
} from './types'

const API = '/api/v1'

/** Long enough for every browser to start the download before revocation. */
export const DOWNLOAD_URL_REVOKE_DELAY_MS = 30_000

/**
 * A non-snapshot event without a status waits this long for a periodic
 * snapshot (the supervisor sends one every 100 ms) before ``GET /status``.
 */
export const STATUS_POLL_DEBOUNCE_MS = 250

/**
 * Consecutive malformed payloads or failed status polls after which the
 * stream is reported as lost even though the socket is still open.  A single
 * failure never changes the connection state.
 */
export const MAX_CONSECUTIVE_STREAM_FAILURES = 3

export interface SubscribeOptions {
  onConnection?: (connected: boolean) => void
  /** Called once, after the stream closes, when the supervisor has restarted. */
  onReset?: () => void
  afterSequence?: number
  instanceId?: string | null
}

function withTimeout(ms = 3000): AbortSignal {
  return AbortSignal.timeout(ms)
}

function optionalFiniteNumber(value: unknown): number | undefined {
  return typeof value === 'number' && Number.isFinite(value) ? value : undefined
}

function requireStatus(value: unknown, context: string): OperatorStatus {
  const status = parseOperatorStatus(value)
  if (!status) throw new Error(`${context} did not match the supervisor status format.`)
  return status
}

function parseQualificationStage(value: unknown): QualificationStage | null {
  if (!isRecord(value) || typeof value.name !== 'string' || typeof value.status !== 'string') return null
  return {
    name: value.name,
    status: value.status,
    reason: typeof value.reason === 'string' ? value.reason : undefined,
    margin: optionalFiniteNumber(value.margin),
    worst_time_s: optionalFiniteNumber(value.worst_time_s),
  }
}

function parseQualificationRun(value: unknown): QualificationRun | null {
  if (!isRecord(value)
    || typeof value.id !== 'string'
    || typeof value.name !== 'string'
    || (typeof value.created_at !== 'string' && typeof value.created_at !== 'number')
    || typeof value.verdict !== 'string') return null
  const stages = Array.isArray(value.stages)
    ? value.stages.map(parseQualificationStage).filter((stage): stage is QualificationStage => stage !== null)
    : []
  return {
    id: value.id,
    name: value.name,
    created_at: value.created_at,
    verdict: value.verdict,
    reference_quality: value.reference_quality,
    simulator_execution: value.simulator_execution,
    stages,
  }
}

async function jsonRequest<T>(url: string, init?: RequestInit): Promise<T> {
  let response: Response
  try {
    response = await fetch(url, {
      ...init,
      headers: { 'Content-Type': 'application/json', ...init?.headers },
      signal: init?.signal ?? withTimeout(),
    })
  } catch (error) {
    const detail = error instanceof Error ? error.message : 'network request failed'
    throw new Error(`Safety supervisor unavailable: ${detail}`, { cause: error })
  }
  if (!response.ok) {
    const body = await response.json().catch(() => null) as { error?: { message?: string }; detail?: string } | null
    throw new Error(body?.error?.message || body?.detail || `${response.status} ${response.statusText}`)
  }
  return response.json() as Promise<T>
}

export class OperatorClient {
  async getStatus(): Promise<OperatorStatus> {
    return requireStatus(await jsonRequest<unknown>(`${API}/status`), 'Supervisor status')
  }

  async bootstrap(): Promise<BootstrapData> {
    const [status, config, audit, playback] = await Promise.all([
      this.getStatus(),
      this.getConfig(),
      jsonRequest<AuditResponse>(`${API}/audit?newest=true`),
      this.getPlayback(),
    ])
    if (status.presentation_mode === 'recorded_playback' && playback === null) {
      throw new Error('Recorded review is unavailable: the supervisor did not provide its configured media manifest.')
    }
    if (status.presentation_mode !== 'recorded_playback' && playback !== null) {
      throw new Error('Supervisor mode mismatch: recorded media was exposed outside recorded playback mode.')
    }
    return { status, config, audit: audit.records, playback }
  }

  async getConfig(): Promise<OperatorConfig> {
    return jsonRequest<OperatorConfig>(`${API}/config`)
  }

  async getPlayback(): Promise<PlaybackManifest | null> {
    let response: Response
    try {
      response = await fetch(`${API}/playback`, { signal: withTimeout() })
    } catch (error) {
      const detail = error instanceof Error ? error.message : 'network request failed'
      throw new Error(`Recorded review availability is unknown: ${detail}`, { cause: error })
    }
    if (response.status === 404) return null
    if (!response.ok) throw new Error(`Recorded review lookup failed: ${response.status} ${response.statusText}`)
    return response.json() as Promise<PlaybackManifest>
  }

  async getQualificationRuns(): Promise<QualificationRun[] | null> {
    let response: Response
    try {
      response = await fetch(`${API}/qualification-runs`, { signal: withTimeout(5000) })
    } catch (error) {
      const detail = error instanceof Error ? error.message : 'network request failed'
      throw new Error(`Qualification history is unavailable: ${detail}`, { cause: error })
    }
    if (response.status === 404) return null
    if (!response.ok) {
      const body = await response.json().catch(() => null) as { error?: { message?: string }; detail?: string } | null
      throw new Error(body?.error?.message || body?.detail || `Qualification history request failed: ${response.status} ${response.statusText}`)
    }
    const body = await response.json() as unknown
    if (!isRecord(body) || !Array.isArray(body.runs)) {
      throw new Error('Qualification history response did not include a runs list.')
    }
    return body.runs.map(parseQualificationRun).filter((run): run is QualificationRun => run !== null)
  }

  /**
   * Follow the supervisor event stream.  The cursor is only meaningful for one
   * supervisor instance: when the instance id changes, or the supervisor's
   * counter is behind the client's cursor, the stream is closed and
   * ``onReset`` asks the caller to re-bootstrap instead of resuming.
   *
   * Every payload is validated; malformed ones are logged and dropped, never
   * handed to ``handler``.  A status carried by an event is trusted as is; an
   * event without one schedules a single debounced ``GET /status`` that a
   * newer snapshot cancels.  The connection is reported lost when the socket
   * closes or after ``MAX_CONSECUTIVE_STREAM_FAILURES`` failures in a row.
   */
  subscribe(handler: (event: OperatorEvent) => void, options: SubscribeOptions = {}): () => void {
    const { onConnection, onReset } = options
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
    let closed = false
    let socket: WebSocket | null = null
    let socketOpen = false
    let reconnectAttempts = 0
    let reconnectTimer: ReturnType<typeof setTimeout> | null = null
    let lastSequence = Math.max(0, Math.floor(options.afterSequence ?? 0))
    let instanceId = options.instanceId ?? null
    // Malformed payloads are cleared by any valid payload; failed polls only
    // by a status (a successful poll or an event that carries one).
    const failures = { payload: 0, poll: 0 }
    let degraded = false
    // Highest event sequence whose resulting status has not been seen yet.
    let awaitingStatusFor: number | null = null
    let pollTimer: ReturnType<typeof setTimeout> | null = null

    const cancelPoll = () => {
      if (pollTimer !== null) clearTimeout(pollTimer)
      pollTimer = null
      awaitingStatusFor = null
    }

    const stop = () => {
      closed = true
      cancelPoll()
      if (reconnectTimer !== null) {
        clearTimeout(reconnectTimer)
        reconnectTimer = null
      }
      const current = socket
      socket = null
      current?.close(1000, 'subscription closed')
    }

    const updateDegraded = () => {
      const next = Math.max(failures.payload, failures.poll) >= MAX_CONSECUTIVE_STREAM_FAILURES
      if (next === degraded) return
      degraded = next
      if (degraded) console.warn('Supervisor event stream: repeated failures; treating the stream as lost.')
      if (socketOpen) onConnection?.(!degraded)
    }

    const recordFailure = (kind: keyof typeof failures, detail: string) => {
      failures[kind] += 1
      if (failures[kind] === 1) console.warn(`Supervisor event stream: ${detail}`)
      updateDegraded()
    }

    const recordSuccess = (kind: keyof typeof failures) => {
      failures[kind] = 0
      updateDegraded()
    }

    const restarted = (eventInstance: string | undefined, supervisorSequence: number | undefined): boolean => {
      if (eventInstance) {
        if (instanceId === null) instanceId = eventInstance
        else if (eventInstance !== instanceId) return true
      }
      return typeof supervisorSequence === 'number' && supervisorSequence < lastSequence
    }

    const reset = () => {
      if (closed) return
      stop()
      onConnection?.(false)
      onReset?.()
    }

    const pollStatus = () => {
      pollTimer = null
      if (closed || awaitingStatusFor === null) return
      this.getStatus().then((status) => {
        if (closed) return
        if (restarted(status.instance_id, status.last_event_sequence)) {
          reset()
          return
        }
        recordSuccess('poll')
        if (awaitingStatusFor !== null && status.last_event_sequence >= awaitingStatusFor) awaitingStatusFor = null
        lastSequence = Math.max(lastSequence, status.last_event_sequence)
        handler({ schema_version: 1, instance_id: status.instance_id, sequence: status.last_event_sequence, type: 'snapshot', status })
      }).catch((error: unknown) => {
        if (closed) return
        recordFailure('poll', `status poll failed: ${error instanceof Error ? error.message : String(error)}`)
      })
    }

    const awaitStatus = (sequence: number) => {
      awaitingStatusFor = Math.max(awaitingStatusFor ?? 0, sequence)
      if (pollTimer === null) pollTimer = setTimeout(pollStatus, STATUS_POLL_DEBOUNCE_MS)
    }

    const scheduleReconnect = () => {
      if (closed || reconnectTimer !== null) return
      const delay = Math.min(30000, Math.round(1000 * Math.pow(2, reconnectAttempts)))
      reconnectAttempts += 1
      reconnectTimer = setTimeout(() => {
        reconnectTimer = null
        connect()
      }, delay)
    }

    const connect = () => {
      if (closed) return
      const current = new WebSocket(`${protocol}//${window.location.host}${API}/events?after_sequence=${lastSequence}`)
      socket = current

      current.addEventListener('open', () => {
        if (closed || socket !== current) return
        reconnectAttempts = 0
        socketOpen = true
        failures.payload = 0
        failures.poll = 0
        degraded = false
        onConnection?.(true)
      })
      current.addEventListener('close', () => {
        if (socket === current) {
          socket = null
          socketOpen = false
          cancelPoll()
        }
        if (closed) return
        onConnection?.(false)
        scheduleReconnect()
      })
      current.addEventListener('error', () => {
        if (closed || socket !== current) return
        onConnection?.(false)
      })
      current.addEventListener('message', (message: MessageEvent<unknown>) => {
        if (closed || socket !== current) return
        let payload: unknown
        try {
          payload = typeof message.data === 'string' ? JSON.parse(message.data) : undefined
        } catch {
          payload = undefined
        }
        const event = parseOperatorEvent(payload)
        if (!event) {
          recordFailure('payload', 'dropped a malformed supervisor event')
          return
        }
        if (restarted(event.instance_id ?? event.status?.instance_id, event.status?.last_event_sequence)) {
          reset()
          return
        }
        recordSuccess('payload')
        lastSequence = Math.max(lastSequence, Math.floor(event.sequence))
        handler(event)
        if (event.status) {
          recordSuccess('poll')
          if (awaitingStatusFor !== null && event.status.last_event_sequence >= awaitingStatusFor) cancelPoll()
        } else {
          awaitStatus(event.sequence)
        }
      })
    }

    connect()

    return stop
  }

  async action(action: ActionName, payload?: Partial<PreflightPayload> & { operator?: string }): Promise<OperatorStatus> {
    const body = await jsonRequest<unknown>(`${API}/actions/${action}`, {
      method: 'POST',
      body: JSON.stringify(payload ?? {}),
    })
    return requireStatus(body, 'The supervisor accepted the action, but its response')
  }

  /**
   * ``operator`` is sent as a self-asserted claim.  The supervisor records it
   * as ``claimed_operator``; only a trusted identity proxy sets the actor.
   */
  async saveConfig(config: OperatorConfig, operator: string): Promise<OperatorConfig> {
    const claim = operator.trim()
    return jsonRequest<OperatorConfig>(`${API}/config`, {
      method: 'PUT',
      headers: claim ? { 'x-operator-name': claim } : {},
      body: JSON.stringify(config),
    })
  }

  async downloadDiagnostics(): Promise<void> {
    let response: Response
    try {
      response = await fetch(`${API}/diagnostics`, { signal: withTimeout(10000) })
    } catch (error) {
      const detail = error instanceof Error ? error.message : 'network request failed'
      throw new Error(`Diagnostics export unavailable: ${detail}`, { cause: error })
    }
    if (!response.ok) {
      const body = await response.json().catch(() => null) as { error?: { message?: string }; detail?: string } | null
      throw new Error(body?.error?.message || body?.detail || `Diagnostics export failed: ${response.status} ${response.statusText}`)
    }
    const blob = await response.blob()
    const disposition = response.headers.get('content-disposition') ?? ''
    const matchedName = disposition.match(/filename\*?=(?:UTF-8''|")?([^";]+)/i)?.[1]
    const filename = matchedName ? decodeURIComponent(matchedName.trim()) : 'mocap-g1-diagnostics.json'
    const url = URL.createObjectURL(blob)
    const anchor = document.createElement('a')
    anchor.href = url
    anchor.download = filename
    document.body.appendChild(anchor)
    anchor.click()
    anchor.remove()
    // Revoking synchronously can cancel the download before the browser has
    // read the blob; release it once the download has had time to start.
    setTimeout(() => URL.revokeObjectURL(url), DOWNLOAD_URL_REVOKE_DELAY_MS)
  }
}
