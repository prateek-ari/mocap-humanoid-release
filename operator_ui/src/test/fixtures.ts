import type { ActionName, OperatorStatus } from '../types'

const ACTION_NAMES: ActionName[] = [
  'check', 'mocap-accept', 'start-simulation', 'connect-robot', 'confirm-preflight',
  'arm', 'start-tracking', 'disengage', 'rearm', 'shutdown',
]

export function makeStatus(overrides: Partial<OperatorStatus> = {}): OperatorStatus {
  const controls: OperatorStatus['controls'] = Object.fromEntries(
    ACTION_NAMES.map((name) => [name, { enabled: true, reason: null }]),
  )
  controls['edit-config'] = { enabled: true, reason: null }
  return {
    schema_version: 1,
    instance_id: 'instance-a',
    presentation_mode: 'live',
    topology: 'all_in_one',
    runtime_mode: 'real',
    state: 'tracking',
    capability_lock: {
      whole_body_unlocked: true,
      manifest_verified: true,
      reason: 'Verified',
      manifest_id: 'qualification-001',
      expires_at: '2035-01-01T00:00:00Z',
      capabilities: ['whole_body'],
    },
    health: {
      mocap_online: true,
      mocap_accepted: true,
      mocap_rate_hz: 60,
      packet_loss_percent: 0,
      source_age_ms: 4,
      simulation_ready: true,
      robot_connected: true,
      robot_state_fresh: true,
      controller_ready: true,
      preflight_confirmed: true,
      command_authority: true,
      overall: 'tracking',
      updated_at: 0,
      messages: [],
    },
    active_fault: null,
    simulation: null,
    fault_condition_cleared: false,
    last_event_sequence: 10,
    controls,
    runtime_adapter: { blocked_call: null },
    audit_integrity: {
      persistent: true,
      chain_verified: true,
      chain_breaks: 0,
      chained_records: 12,
      unchained_records: 0,
      unreadable_lines: 0,
      checked_at: 0,
    },
    watchdog: { running: true, overdue: false },
    ...overrides,
  }
}

/** Raw ``GET /api/v1/setup`` payload for a Windows console (forward-slash paths, Xsens on this PC). */
export function makeWindowsSetupPayload(overrides: Record<string, unknown> = {}): Record<string, unknown> {
  const session = {
    route: 'live',
    session_dir: 'C:/Users/me/mocap-sessions/2026-10-13-1400',
    robot: 'unitree_g1',
    human_height_m: 1.75,
    mocap_ws: 'C:/Users/me/mocap-runtime',
    motion_bvh: 'C:/Users/me/mocap-data/motion.bvh',
    npose_bvh: 'C:/Users/me/mocap-data/neutral.bvh',
    offsets_file: '',
    bvh_scale: 0.01,
    start_frame: 0,
    end_frame: 600,
    frame_step: 1,
    npose_start_s: 2,
    npose_duration_s: 1,
    windows_xsens_ip: '127.0.0.1',
    linux_xsens_ip: '',
  }
  return {
    platform: 'windows',
    repo_root: 'C:/Users/me/mocap_humanoid',
    home: 'C:/Users/me',
    xsens_on_this_pc: true,
    session: null,
    defaults: session,
    network: { interfaces: [{ name: 'Ethernet', ipv4: '192.168.1.20' }], detected_linux_ip: null },
    firewall_command: null,
    readiness: { live_image: null, live_runtime: true, recorded_runtime: true, motion_bvh: false, npose_bvh: false, display: true, display_hint: null },
    files: [],
    jobs: [],
    ...overrides,
  }
}

/** Raw ``StreamSnapshot`` from the idle Xsens monitor, receiving by default. */
export function makeStreamSnapshot(overrides: Record<string, unknown> = {}): Record<string, unknown> {
  return {
    state: 'receiving',
    bind: '127.0.0.1',
    port: 5555,
    source: '127.0.0.1:50123',
    rate_hz: 60,
    loss_fraction: 0,
    body_frames: 1200,
    joint_angles: true,
    last_packet_age_s: 0.01,
    malformed: 0,
    ...overrides,
  }
}

/** Raw ``GET /api/v1/setup`` payload for the Windows desktop app (``MOCAP_APP=1``). */
export function makeAppSetupPayload(overrides: Record<string, unknown> = {}): Record<string, unknown> {
  return makeWindowsSetupPayload({
    app: { mode: 'windows_app', version: '0.4.0' },
    stream: makeStreamSnapshot(),
    prefs: { schema: 1, record_video: false, calibrate_after_s: null },
    ...overrides,
  })
}
