export type SessionState =
  | 'setup_required'
  | 'idle'
  | 'checking'
  | 'mocap_accepted'
  | 'sim_validated'
  | 'robot_connected'
  | 'preflight_passed'
  | 'armed_disabled'
  | 'tracking'
  | 'disengaging'
  | 'fault_latched'

export interface CapabilityLock {
  whole_body_unlocked: boolean
  manifest_verified: boolean
  reason: string
  manifest_id: string | null
  expires_at: string | null
  capabilities: string[]
}

export interface HealthSnapshot {
  mocap_online: boolean
  mocap_accepted: boolean
  mocap_rate_hz: number
  packet_loss_percent: number
  source_age_ms: number | null
  simulation_ready: boolean
  robot_connected: boolean
  robot_state_fresh: boolean
  controller_ready: boolean
  preflight_confirmed: boolean
  command_authority: boolean
  overall: string
  updated_at: number
  messages: string[]
}

export interface FaultEvent {
  code: string
  reason: string
  source: string
  timestamp: number
  latched: boolean
  recoverable: boolean
}

export interface ControlState {
  enabled: boolean
  reason: string | null
}

export interface OperatorConfig {
  schema_version: number
  topology: 'all_in_one' | 'split'
  runtime_mode: 'real' | 'fake'
  mocap: {
    listen_address: string
    port: number
    source_ip: string | null
  }
  robot: {
    interface: string | null
  }
  privacy: {
    record_raw_mocap: boolean
  }
}

export interface SimulationSource {
  name?: string
  frame_count?: number
  fps?: number
  duration_s?: number
  sha256?: string
}

export interface SimulationTelemetry {
  source_age_ms?: number | null
  source_rate_hz?: number | null
  command_rate_hz?: number | null
  safety_reason?: string | null
  blend_weight?: number | null
  reset_required?: boolean | null
  clamped_joint_count?: number | null
}

export interface SimulationStatus {
  mode?: string
  state?: string
  source?: SimulationSource | null
  frame_index?: number | null
  progress_pct?: number | null
  reference_qpos?: number[] | null
  bounded_qpos?: number[] | null
  telemetry?: SimulationTelemetry | null
  limitations?: string[]
}

/** Summary of the durable audit log's hash-chain verification. */
export interface AuditIntegrity {
  persistent: boolean
  chain_verified: boolean
  chain_breaks: number
  chained_records: number
  unchained_records: number
  unreadable_lines: number
  checked_at?: number
  /** Present after an on-demand re-read; false means the file changed underneath the supervisor. */
  tail_matches_last_write?: boolean
}

export interface RuntimeAdapterStatus {
  /** Label of a runtime-adapter call that overran its deadline and has not returned. */
  blocked_call: string | null
}

export interface WatchdogStatus {
  running: boolean
  overdue: boolean
}

export interface OperatorStatus {
  schema_version: number
  /** Per-process supervisor id; it changes whenever the backend restarts. */
  instance_id?: string
  presentation_mode: 'live' | 'demo' | 'recorded_playback' | 'recorded_simulation'
  topology: OperatorConfig['topology']
  runtime_mode: OperatorConfig['runtime_mode']
  state: SessionState
  capability_lock: CapabilityLock
  health: HealthSnapshot
  active_fault: FaultEvent | null
  simulation?: SimulationStatus | null
  fault_condition_cleared: boolean
  last_event_sequence: number
  controls: Record<string, ControlState>
  runtime_adapter?: RuntimeAdapterStatus
  /** Null when the supervisor's audit log does not report integrity. */
  audit_integrity?: AuditIntegrity | null
  watchdog?: WatchdogStatus
}

export interface QualificationStage {
  name: string
  status: string
  reason?: string
  margin?: number
  worst_time_s?: number
}

export interface QualificationRun {
  id: string
  name: string
  created_at: string | number
  verdict: string
  reference_quality?: unknown
  simulator_execution?: unknown
  stages: QualificationStage[]
}

export interface AuditEvent {
  sequence: number
  timestamp: number
  category: string
  action: string
  outcome: string
  state: SessionState
  actor: string
  details: Record<string, unknown>
}

export interface AuditResponse {
  records: AuditEvent[]
  last_sequence: number
}

export interface BootstrapData {
  status: OperatorStatus
  config: OperatorConfig
  audit: AuditEvent[]
  playback: PlaybackManifest | null
}

export interface PlaybackManifest {
  schema_version: 1
  available: true
  kind: 'recorded_review_video'
  presentation_mode: 'recorded_playback'
  filename: string
  bytes: number
  video_url: string
  review_mode: 'simulation_preview'
  hardware_status: 'blocked'
  command_authority: false
}

export interface OperatorEvent {
  schema_version: number
  instance_id?: string
  /** In-memory event-stream cursor; restarts at 0 with each supervisor instance. */
  sequence: number
  /** Durable AuditLog number, or null when the record could not be persisted. */
  audit_sequence?: number | null
  timestamp?: number
  /** ``snapshot`` carries a full ``status``; other types are audit categories. */
  type: string
  status?: OperatorStatus
  action?: string
  outcome?: string
  state?: SessionState
  actor?: string
  details?: Record<string, unknown>
}

export interface PreflightPayload {
  operator: string
  spotter: string
  metadata: {
    clear_area: true
    harness_or_support: true
    stop_drill: true
  }
}

export type ActionName =
  | 'check'
  | 'mocap-accept'
  | 'start-simulation'
  | 'connect-robot'
  | 'confirm-preflight'
  | 'arm'
  | 'start-tracking'
  | 'disengage'
  | 'rearm'
  | 'shutdown'

export type HealthLevel = 'ok' | 'warning' | 'fault' | 'offline' | 'checking'

export interface HealthItem {
  status: HealthLevel
  label: string
  detail: string
  value?: string | number | boolean | null
  unit?: string
}
