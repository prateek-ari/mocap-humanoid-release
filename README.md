# mocap-humanoid-release — Xsens MVN motion capture → humanoid simulation

Full-body Xsens MVN motion capture retargeted through [GMR](https://github.com/YanjieZe/GMR) onto
the **Unitree G1** (29-DOF, full-wrist model) and the **Booster T1** in MuJoCo. Two routes:

- **Live:** Xsens Humanoid on a Windows PC streams MXTP02 body poses (and MXTP20 joint angles) over
  UDP to a Linux PC; the simulated robot follows the performer in a MuJoCo window.
- **Recorded:** a take exported as 3DS Max BVH is retargeted on Linux, played back and recorded as
  MP4.

Everything in this repository is simulation, replay or synthetic-traffic evidence. No physical robot
control path is enabled: the operator console keeps hardware authority locked, and the G1 upper-body
bridge is exercised only against fake transports. Connecting a real robot is optional and later; see
the [hardware bring-up runbook](docs/g1-bringup-runbook-2026-08-16.md).

## Start here

Open the [step-by-step setup guide](docs/engineer-setup.html) (interactive) or its
[Markdown source](docs/engineer-setup-guide.md). It goes one step at a time: install, enter session
values, calibrate the suit, then live suit → MuJoCo for the G1 or Booster T1 (first route), or
recorded BVH export (second route). Edge cases, terminal equivalents, Docker image details and the
full troubleshooting table are in the [engineering reference](docs/engineer-reference.md).

For a repeatable installation check before connecting the suit, follow
[Testing without an Xsens suit](docs/testing-without-suit.md): GPU rendering, synthetic UDP-to-G1
video, replaying a real Xsens recording into the live G1/T1 viewer and the optional ROS 2 recording
check.

On a fresh Ubuntu 22.04/24.04 PC, one line installs Git, downloads the project and starts the
setup questions (press Enter to accept each default):

```bash
sudo apt-get update && sudo apt-get install -y git && git clone https://github.com/prateek-ari/mocap-humanoid-release.git && cd mocap-humanoid-release && bash tools/setup/install.sh
```

The installer ends by opening the **Setup console** in your browser and adds **Mocap Setup** to
the applications menu. Every later step (session values, stream check, live simulation, BVH
retargeting, videos, session report) is a button there. To open it from a terminal instead:

```bash
bash tools/setup/open_console.sh
```

Prefer the terminal? Enter the session values with prefilled defaults and load them in each new
terminal:

```bash
bash tools/setup/new_session.sh
source session.env
```

Scripts can skip the questions: `bash tools/setup/install.sh core|recorded|live [--system-packages]`
(`recorded` defaults to `--workspace ~/mocap-runtime`; add `--dry-run` to only print commands), and
`bash tools/setup/new_session.sh --defaults`.

From the repository root, check the prerequisites without changing the host:

```bash
python3 tools/checks/setup_doctor.py core --with-operator
python3 tools/checks/setup_doctor.py recorded --workspace "$MOCAP_WS"
XSENS_SOURCE_IP='<current Windows Xsens-facing IPv4>' python3 tools/checks/setup_doctor.py live
```

Run `core` in your project venv. The checker returns nonzero for missing prerequisites; it does not
certify real motion or physical robot readiness.

## How it fits together

```text
Windows Xsens Humanoid + MVN Link (source IPv4 confirmed per session)
        │
        └── MXTP02 quaternion + MXTP20 joint angles
            UDP 5555 ──> Linux receiver (destination IPv4 detected per session)
                         │
                         ├── stream acceptance gate (tools/xsens/stream_acceptance.py)
                         ├── optional xsens_mvn_ros2 ──> TF + joint/link states + rosbag
                         └── GMR Xsens adapter ──> G1 / T1 retargeting ──> MuJoCo viewer
```

Acceptance, ROS 2 and the GMR viewer each bind UDP 5555, so `tools/xsens/live_operator.sh` runs
them one stage at a time. The direct GMR path stays outside ROS for minimum latency. Live viewer
behaviour (arm abduction, neutral-pose calibration, command screening) is explained in the
[engineering reference](docs/engineer-reference.md#live-viewer-behaviour).

The recorded route uses `tools/retarget/retarget_robot_recorded.py` (robot-generic Xsens BVH
retargeting with optional neutral-pose arm alignment), `tools/retarget/project_robot_arm_collisions.py`
(arm/body clearance gate) and `tools/retarget/render_robot_motion.py` (MuJoCo playback and MP4
export). The G1 uses GMR's upstream Xsens mapping; the T1 mapping is this project's
(`retarget_configs/`).

## Repository map

| Path | Purpose |
|---|---|
| [docs/engineer-setup-guide.md](docs/engineer-setup-guide.md) ([interactive](docs/engineer-setup.html)) | Step-by-step installation and operation for both routes. |
| [docs/engineer-reference.md](docs/engineer-reference.md) | Edge cases, terminal equivalents, Docker images, calibration, troubleshooting. |
| [docs/testing-without-suit.md](docs/testing-without-suit.md) | Reproducible installation checks with synthetic traffic or a replayed recording. |
| [docs/windows-xsens-humanoid-setup.md](docs/windows-xsens-humanoid-setup.md) | Native Windows capture, System License activation, streaming and BVH handoff. |
| [docs/operator-console.md](docs/operator-console.md) | Operator console modes (status, recorded simulation, Setup workspace), safety boundaries and deployment. |
| [docs/architecture.html](docs/architecture.html) ([guide](docs/architecture-map.md)) | Interactive codebase map. **Predates the operator console** (`operator_ui/`, `mocap_g1/operator/`), which it does not draw. |
| [docs/qualify-pipeline-2026-08-16.md](docs/qualify-pipeline-2026-08-16.md) | One-command qualification chain (`make qualify`) for a recorded G1 motion and its ledger. |
| [docs/balance-screen-2026-08-16.md](docs/balance-screen-2026-08-16.md) | Standing-balance and torque-feasibility screen: thresholds and rationale. |
| [docs/live-latency-2026-08-16.md](docs/live-latency-2026-08-16.md) | Live causal filter, command upsampler and measured latency bounds (`mocap_g1/live_filter.py`). |
| [docs/g1-bringup-runbook-2026-08-16.md](docs/g1-bringup-runbook-2026-08-16.md) | Robot-side hardware bring-up runbook (gates, pass conditions, hard stops). |
| [docs/pkl-schema-note.md](docs/pkl-schema-note.md) | G1 `.pkl` data contract and safe joint-name lookup rule. |
| `docs/vendor/` | Xsens ROS 2 guide PDF (Network Streamer reference). |
| `mocap_g1/` | Canonical G1 schema, Xsens codecs, live retargeting support, safety envelope, qualification modules and the operator console service. |
| `operator_ui/` | React/TypeScript source of the operator console; the built bundle is committed under `mocap_g1/operator/static/`. |
| `tools/` | Installer and session helpers, setup doctor, stream acceptance, live/replay tools, retargeting, qualification and control screens, guide builder. |
| `eval/` | Retarget-quality scoring harness (PHUMA-style metrics, gate v2) and the official 29-DOF URDF. |
| `retarget_configs/` | G1 and Booster T1 mappings, GMR patches and calibration notes. |
| `docker/gmr-live/` | Ubuntu 24.04 GMR/MuJoCo live viewer image (host NVIDIA GPU and X11). |
| `docker/xsens-ros2/` | Ubuntu 24.04 / ROS 2 Jazzy build of the Xsens ROS 2 driver. |
| `docker/xsens-humanoid/` | Experimental Ubuntu 24.04 runtime for the Xsens Humanoid AppImage on an Ubuntu 22.04 host. |
| `deploy/operator-console/` | systemd unit and desktop entry for the status-only console. |
| `configs/` | Example dataset job for `tools/dataset/g1_dataset_pipeline.py`. |

## Development

```bash
python3 -m pip install -c constraints/requirements-ci.txt -e '.[runtime,operator,test,lint]'
make hardware-free-checks
make frontend-check
```

- Scripts under `tools/` and `eval/` are run **from the repository root** (a checkout or the Docker
  images). They bootstrap `sys.path` so `mocap_g1` imports resolve without installation; only the
  `mocap_g1` package is installed by `pip install -e .`.
- Set `MOCAP_WS` to the pinned GMR workspace (`tools/bootstrap_g1_workspace.py`) to run the MuJoCo
  gate tests. Set `MOCAP_REQUIRE_ALL_TESTS=1` to make any skipped test fail the run.
- `make lint` runs `ruff check .`; `make typecheck` runs mypy on the safety modules.
- `.github/workflows/hardware-free.yml` runs the Python suite on 3.10–3.12 with dependencies pinned
  by `constraints/requirements-ci.txt` (refresh with `make constraints`, needs `uv`), plus the
  frontend lint/typecheck/test/build and the committed-bundle check. It needs a self-hosted runner
  that carries the GMR workspace.
- The `numpy>=2.2,<2.3` pin is mirrored in `tools/checks/setup_doctor.py`, its test and
  `docker/gmr-live/Dockerfile`; change them together.
- The ROS 2 synthetic smoke test is manual: it needs Docker and the `xsens-ros2:jazzy` image and
  binds host UDP 9763 (`docker build -t xsens-ros2:jazzy docker/xsens-ros2 && tools/checks/ros2_synthetic_smoke.sh`).

### Qualifying a recorded G1 motion

Audit a retargeted recording without modifying it, condition a time window into a separate
artifact, then screen it:

```bash
python3 tools/xsens/audit_g1_recording.py /path/to/g1-preview.npz \
  --window-seconds 10 --output /path/to/window-audit.json
python3 tools/condition_g1_motion.py /path/to/g1-preview.npz /path/to/conditioned.npz \
  --start-seconds 165 --end-seconds 170
python3 tools/control/audit_g1_self_collisions.py /path/to/g1-preview.npz \
  --model "$MOCAP_WS/GMR/assets/unitree_g1/g1_mocap_29dof.xml" --output /path/to/self-collision-audit.json
python3 tools/control/qualify_g1_upper_body.py /path/to/conditioned.npz \
  --model "$MOCAP_WS/GMR/assets/unitree_g1/g1_mocap_29dof.xml" --output /path/to/controller-screen.json
python3 tools/control/project_g1_self_collisions.py /path/to/conditioned.npz \
  --model "$MOCAP_WS/GMR/assets/unitree_g1/g1_mocap_29dof.xml" \
  --output /path/to/collision-gated.npz --report /path/to/collision-gate.json
python3 tools/control/replay_g1_faults.py /path/to/conditioned.npz \
  --source-stall 3.0:0.20 --disable 1.5:0.50 --control-gap 10.0:0.65 \
  --require-reason tracking --require-reason disabled \
  --require-reason stale_source --require-reason hard_timeout \
  --output /path/to/fault-replay.json
```

`make qualify RECORDING=path/to.npz` runs the whole chain and writes one ledger; see
[docs/qualify-pipeline-2026-08-16.md](docs/qualify-pipeline-2026-08-16.md). The audit exits `1` when
motion fails its kinematic gate while still writing the report. The conditioner is offline, preserves
the source and does not perform or prove calibration. The controller screen fixes the simulated
pelvis and tests bounded upper-body tracking, not balance. Collision projection writes no derivative
unless every frame and the correction dynamics pass. Even a recording that passes every screen stays
`BLOCKED` as hardware evidence until it carries a reviewed (non-`zero-preview`) Xsens calibration.

Runtime data and the GMR checkout live outside this repository in `$MOCAP_WS`; keep datasets,
recordings and generated `.pkl`/`.npz`/MP4 files out of Git. LAFAN1-derived motion and results are
research-only (CC BY-NC-ND).

## Third-party components

- [GMR](https://github.com/YanjieZe/GMR) (MIT), pinned by commit and patched by
  `retarget_configs/gmr-*.patch`.
- Unitree G1 29-DOF URDF from
  [unitree_ros](https://github.com/unitreerobotics/unitree_ros) (`eval/assets/`, mirrored as package
  data in `mocap_g1/assets/`), unmodified except a provenance comment.
- [xsens/xsens_mvn_ros2](https://github.com/xsens/xsens_mvn_ros2), built from a pinned commit in
  `docker/xsens-ros2/`.
- Booster T1 MuJoCo asset ships with GMR (Apache-2.0).
