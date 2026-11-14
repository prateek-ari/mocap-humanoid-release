import { afterEach, describe, expect, it, vi } from 'vitest'
import { LOG_PAGE_LINES, SetupApiError, SetupClient, isLoopbackIp, parseJobSummary, parseK1Status, parseSetupState, parseStreamSnapshot, tildePath } from './setupApi'
import { makeAppSetupPayload, makeStreamSnapshot, makeWindowsSetupPayload } from './test/fixtures'

function respond(status: number, body: unknown): Response {
  return new Response(JSON.stringify(body), { status, headers: { 'Content-Type': 'application/json' } })
}

const job = { id: 'j-1', kind: 'doctor', label: 'System check', state: 'passed', started_at: 1, finished_at: 2, exit_code: 0, summary: 'PASS: ok', outputs: [] }

afterEach(() => {
  vi.unstubAllGlobals()
})

describe('SetupClient', () => {
  it('treats 404 as a console without --setup', async () => {
    vi.stubGlobal('fetch', vi.fn(() => Promise.resolve(respond(404, { detail: 'Not Found' }))))
    expect(await new SetupClient().getSetup()).toBeNull()
  })

  it('reports setup errors with code and field', async () => {
    vi.stubGlobal('fetch', vi.fn(() => Promise.resolve(respond(422, { error: { code: 'invalid_session', message: 'Height must be 0.8–2.5 m', field: 'HUMAN_HEIGHT_M' } }))))
    const error = await new SetupClient().startJob('doctor').catch((reason: unknown) => reason)
    expect(error).toBeInstanceOf(SetupApiError)
    expect(error).toMatchObject({ status: 422, code: 'invalid_session', field: 'HUMAN_HEIGHT_M', message: 'Height must be 0.8–2.5 m' })
  })

  it('reads FastAPI detail errors', async () => {
    vi.stubGlobal('fetch', vi.fn()
      .mockResolvedValueOnce(respond(415, { detail: 'Use application/json' }))
      .mockResolvedValueOnce(respond(422, { detail: [{ msg: 'Field required' }] })))
    const client = new SetupClient()
    await expect(client.stopJob('j-1')).rejects.toThrow('Use application/json')
    await expect(client.stopJob('j-1')).rejects.toThrow('Field required')
  })

  it('follows next until a log is caught up', async () => {
    const page = Array.from({ length: LOG_PAGE_LINES }, (_, index) => `line ${index}`)
    const fetchMock = vi.fn()
      .mockResolvedValueOnce(respond(200, { job, lines: page, next: LOG_PAGE_LINES }))
      .mockResolvedValueOnce(respond(200, { job, lines: ['last'], next: LOG_PAGE_LINES + 1 }))
    vi.stubGlobal('fetch', fetchMock)
    const log = await new SetupClient().getJobLog('j-1', 0)
    expect(log.lines).toHaveLength(LOG_PAGE_LINES + 1)
    expect(log.next).toBe(LOG_PAGE_LINES + 1)
    expect(String(fetchMock.mock.calls[1][0])).toContain(`after=${LOG_PAGE_LINES}`)
  })
})

describe('setup parsing', () => {
  it('drops malformed jobs and files and rejects a payload without defaults', () => {
    const state = parseSetupState({
      defaults: { route: 'recorded', robot: 'booster_t1_29dof', human_height_m: 'tall' },
      jobs: [job, { id: 'j-2', kind: 'doctor', state: 'exploded' }, null],
      files: [{ name: 'a.mp4', size: 3, kind: 'video' }, { size: 1 }],
      home: '/home/u/',
    })
    expect(state.jobs.map((item) => item.id)).toEqual(['j-1'])
    expect(state.files.map((item) => item.name)).toEqual(['a.mp4'])
    expect(state.defaults).toMatchObject({ route: 'recorded', robot: 'booster_t1_29dof', human_height_m: 1.75 })
    expect(state.home).toBe('/home/u')
    expect(state.stream_check_s).toBe(30)
    expect(parseSetupState({ defaults: {}, stream_check_s: 20 }).stream_check_s).toBe(20)
    expect(parseSetupState({ defaults: {}, stream_check_s: -1 }).stream_check_s).toBe(30)
    expect(() => parseSetupState({ jobs: [] })).toThrow()
  })

  it('shows paths under home with a tilde', () => {
    expect(tildePath('/home/u/mocap-data/motion.bvh', '/home/u')).toBe('~/mocap-data/motion.bvh')
    expect(tildePath('/home/user2/x', '/home/u')).toBe('/home/user2/x')
    expect(tildePath('/srv/x', null)).toBe('/srv/x')
  })

  it('shows Windows drive paths under home with a tilde, ignoring case and backslashes', () => {
    expect(tildePath('C:/Users/me/mocap-data/motion.bvh', 'C:/Users/me')).toBe('~/mocap-data/motion.bvh')
    expect(tildePath('c:/users/ME/x', 'C:/Users/me')).toBe('~/x')
    expect(tildePath('C:\\Users\\me\\mocap-data\\motion.bvh', 'C:/Users/me')).toBe('~/mocap-data/motion.bvh')
    expect(tildePath('C:/Users/me', 'C:/Users/me')).toBe('~')
    expect(tildePath('C:/Users/me2/x', 'C:/Users/me')).toBe('C:/Users/me2/x')
    expect(tildePath('D:/data/x', 'C:/Users/me')).toBe('D:/data/x')
    // A POSIX home never matches case-insensitively.
    expect(tildePath('/HOME/u/x', '/home/u')).toBe('/HOME/u/x')
  })

  it('recognises loopback addresses', () => {
    expect(isLoopbackIp('127.0.0.1')).toBe(true)
    expect(isLoopbackIp(' 127.1.2.3 ')).toBe(true)
    expect(isLoopbackIp('192.168.1.2')).toBe(false)
    expect(isLoopbackIp('')).toBe(false)
  })
})

describe('Windows setup payload', () => {
  it('reads the platform fields and the drive-letter home', () => {
    const state = parseSetupState(makeWindowsSetupPayload({ home: 'C:\\Users\\me\\' }))
    expect(state.platform).toBe('windows')
    expect(state.xsens_on_this_pc).toBe(true)
    expect(state.readiness.live_runtime).toBe(true)
    expect(state.home).toBe('C:/Users/me')
    expect(state.firewall_command).toBeNull()
    expect(state.network.detected_linux_ip).toBeNull()
  })

  it('falls back to Linux defaults when the new fields are missing or malformed', () => {
    const state = parseSetupState({ defaults: {}, home: '/home/u' })
    expect(state).toMatchObject({ platform: 'linux', xsens_on_this_pc: false })
    expect(state.readiness.live_runtime).toBeNull()
    const odd = parseSetupState({ defaults: {}, platform: 'plan9', xsens_on_this_pc: 'yes', readiness: { live_runtime: 'yes' }, home: 'relative/path' })
    expect(odd).toMatchObject({ platform: 'linux', xsens_on_this_pc: false, home: null })
    expect(odd.readiness.live_runtime).toBeNull()
  })
})

describe('desktop app fields', () => {
  it('reads app, stream and prefs', () => {
    const state = parseSetupState(makeAppSetupPayload({ prefs: { schema: 1, record_video: true, calibrate_after_s: 4 } }))
    expect(state.app).toEqual({ mode: 'windows_app', version: '0.4.0' })
    expect(state.stream).toMatchObject({ state: 'receiving', port: 5555, source: '127.0.0.1:50123', rate_hz: 60, joint_angles: true })
    expect(state.prefs).toEqual({ schema: 1, record_video: true, calibrate_after_s: 4 })
  })

  it('falls back safely when the app fields are missing or malformed', () => {
    const plain = parseSetupState(makeWindowsSetupPayload())
    expect(plain.app).toBeNull()
    expect(plain.stream).toBeNull()
    expect(plain.prefs).toEqual({ schema: 1, record_video: false, calibrate_after_s: null })
    const odd = parseSetupState(makeWindowsSetupPayload({
      app: { mode: 'kiosk', version: '1' },
      stream: 'receiving',
      prefs: { record_video: 'yes', calibrate_after_s: -3 },
    }))
    expect(odd.app).toBeNull()
    expect(odd.stream).toBeNull()
    expect(odd.prefs).toEqual({ schema: 1, record_video: false, calibrate_after_s: null })
    expect(parseSetupState(makeWindowsSetupPayload({ app: { mode: 'windows_app', version: 7 } })).app).toEqual({ mode: 'windows_app', version: '' })
  })

  it('guards stream snapshot values', () => {
    expect(parseStreamSnapshot(makeStreamSnapshot({ state: 'exploded', port: 99999, rate_hz: -1, loss_fraction: 4, body_frames: 'many', source: '  ', joint_angles: 1 }))).toMatchObject({
      state: 'off', port: 5555, rate_hz: null, loss_fraction: 1, body_frames: 0, source: null, joint_angles: false,
    })
    expect(parseStreamSnapshot(null)).toBeNull()
  })

  it('reads live status only on live jobs with a well-formed status', () => {
    const base = { id: 'j-9', kind: 'live', label: 'Live simulation', state: 'running', started_at: 1, finished_at: null, exit_code: null, summary: null, outputs: [] }
    const status = { enabled: true, reason: 'tracking', fps: 59.8, frames: 1200, drops: 2, failures: 0, calibrated: true }
    expect(parseJobSummary({ ...base, live: status })?.live).toEqual(status)
    expect(parseJobSummary({ ...base, live: { ...status, enabled: 'yes' } })?.live).toBeNull()
    expect(parseJobSummary({ ...base, live: { enabled: false, frames: -5, fps: 'fast' } })?.live).toEqual({ enabled: false, reason: '', fps: 0, frames: 0, drops: 0, failures: 0, calibrated: false })
    expect(parseJobSummary({ ...base, kind: 'doctor', live: status })?.live).toBeNull()
    expect(parseJobSummary(base)?.live).toBeNull()
  })
})

describe('desktop app client calls', () => {
  it('reads the stream and prefs and saves a partial prefs update', async () => {
    const fetchMock = vi.fn()
      .mockResolvedValueOnce(respond(200, makeStreamSnapshot({ state: 'stale' })))
      .mockResolvedValueOnce(respond(200, { schema: 1, record_video: false, calibrate_after_s: null }))
      .mockResolvedValueOnce(respond(200, { schema: 1, record_video: true, calibrate_after_s: null }))
    vi.stubGlobal('fetch', fetchMock)
    const client = new SetupClient()
    expect((await client.getStream()).state).toBe('stale')
    expect((await client.getPrefs()).record_video).toBe(false)
    expect((await client.putPrefs({ record_video: true })).record_video).toBe(true)
    expect(String(fetchMock.mock.calls[0][0])).toBe('/api/v1/setup/stream')
    expect(fetchMock.mock.calls[2][1]).toMatchObject({ method: 'PUT', body: JSON.stringify({ record_video: true }) })
  })

  it('rejects an unreadable stream body', async () => {
    vi.stubGlobal('fetch', vi.fn(() => Promise.resolve(respond(200, ['nope']))))
    await expect(new SetupClient().getStream()).rejects.toBeInstanceOf(SetupApiError)
  })

  it('posts control commands and reports a 409', async () => {
    const fetchMock = vi.fn()
      .mockResolvedValueOnce(respond(202, { accepted: true }))
      .mockResolvedValueOnce(respond(409, { error: { code: 'job_not_running', message: 'No live simulation is running' } }))
    vi.stubGlobal('fetch', fetchMock)
    const client = new SetupClient()
    await client.controlJob('j 1', 'calibrate')
    expect(String(fetchMock.mock.calls[0][0])).toBe('/api/v1/setup/jobs/j%201/control')
    expect(fetchMock.mock.calls[0][1]).toMatchObject({ method: 'POST', body: JSON.stringify({ command: 'calibrate' }) })
    await expect(client.controlJob('j-1', 'rearm')).rejects.toMatchObject({ status: 409, message: 'No live simulation is running' })
  })
})

describe('Desktop tools', () => {
  it('reads the firewall state and desktop tools, treating anything unknown as absent', () => {
    expect(parseSetupState(makeWindowsSetupPayload())).toMatchObject({ firewall_state: null, desktop_tools: false })
    expect(parseSetupState(makeWindowsSetupPayload({ firewall_state: 'active', desktop_tools: true }))).toMatchObject({ firewall_state: 'active', desktop_tools: true })
    expect(parseSetupState(makeWindowsSetupPayload({ firewall_state: 'maybe', desktop_tools: 'yes' }))).toMatchObject({ firewall_state: null, desktop_tools: false })
  })

  it('reads a malformed K1 status as not installed and not open', () => {
    expect(parseK1Status(null)).toMatchObject({ available: false, installed: false, window_open: false, sdk_version: null })
    expect(parseK1Status({ available: true, sdk_version: '1.6.3', tk: 'yes', window_open: true })).toMatchObject({ available: true, sdk_version: '1.6.3', tk: false, window_open: true })
  })

  it('posts the fixed actions and maps unknown outcomes to failure', async () => {
    const fetchMock = vi.fn<(url: string, init?: RequestInit) => Promise<Response>>(() => Promise.resolve(respond(200, { outcome: 'weird', message: 'x' })))
    vi.stubGlobal('fetch', fetchMock)
    const client = new SetupClient()
    expect(await client.runPrivileged('firewall')).toEqual({ outcome: 'unavailable', message: 'x' })
    expect(fetchMock.mock.calls[0][0]).toBe('/api/v1/setup/privileged')
    expect(fetchMock.mock.calls[0][1]).toMatchObject({ method: 'POST', body: JSON.stringify({ action: 'firewall' }) })
    expect(await client.openK1()).toEqual({ outcome: 'failed', message: 'x' })
    expect(fetchMock.mock.calls[1][0]).toBe('/api/v1/setup/k1/open')
  })
})
