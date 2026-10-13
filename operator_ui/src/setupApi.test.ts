import { afterEach, describe, expect, it, vi } from 'vitest'
import { LOG_PAGE_LINES, SetupApiError, SetupClient, parseSetupState, tildePath } from './setupApi'

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
})
