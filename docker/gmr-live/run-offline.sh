#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<EOF
Usage: $0 [--detach] /path/to/motion.bvh [loop_bvh_g1.py options]

Examples:
  $0 /path/to/motion.bvh
  $0 /path/to/motion.bvh --loop-mode repeat --loops 1 \\
    --record-video --video-path /data/motion-g1.mp4

Paths passed to --video-path and --calibration-output must be below /data,
which maps to the ignored artifacts/xsens-offline directory by default.
EOF
}

detach=false
if [[ ${1:-} == -h || ${1:-} == --help ]]; then
  usage
  exit 0
fi
if [[ ${1:-} == --detach ]]; then
  detach=true
  shift
fi
if (( $# < 1 )); then
  usage
  exit 2
fi

bvh_file=$(realpath -- "$1")
shift
if [[ ! -f $bvh_file ]]; then
  echo "BVH file was not found: $bvh_file" >&2
  exit 2
fi

display=${DISPLAY:-:0}
xauthority=${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}
image=${GMR_LIVE_IMAGE:-gmr-live:bb1bbe4}
container_name=${GMR_OFFLINE_CONTAINER_NAME:-gmr-bvh-loop}

if [[ ! -f $xauthority ]]; then
  echo "X11 authority file was not found: $xauthority" >&2
  exit 2
fi
display_number=${display##*:}
display_number=${display_number%%.*}
if [[ ! -S /tmp/.X11-unix/X${display_number} ]]; then
  echo "X11 socket for DISPLAY=$display was not found" >&2
  exit 2
fi
if docker container inspect "$container_name" >/dev/null 2>&1; then
  echo "Container $container_name already exists; stop it before starting another loop" >&2
  exit 2
fi

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd -- "$script_dir/../.." && pwd)
runtime_root=${GMR_OFFLINE_RUNTIME_ROOT:-$repo_root/artifacts/xsens-offline}
mkdir -p "$runtime_root"

base_name=$(basename -- "$bvh_file")
safe_name=${base_name%.*}
safe_name=${safe_name//[^A-Za-z0-9_.-]/_}
cache_name=${GMR_OFFLINE_CACHE_NAME:-${safe_name}-g1-preview.npz}
if [[ $cache_name == */* ]]; then
  echo "GMR_OFFLINE_CACHE_NAME must be a file name, not a path" >&2
  exit 2
fi

docker_args=(run --rm --name "$container_name")
if [[ $detach == true ]]; then
  docker_args+=(-d)
else
  docker_args+=(-i)
  if [[ -t 0 && -t 1 ]]; then
    docker_args+=(-t)
  fi
fi

offset_args=()
offset_mount=()
if [[ -n ${GMR_OFFSETS_FILE:-} ]]; then
  offsets_file=$(realpath -- "$GMR_OFFSETS_FILE")
  if [[ ! -f $offsets_file ]]; then
    echo "GMR_OFFSETS_FILE was not found: $offsets_file" >&2
    exit 2
  fi
  offset_mount+=(--volume "$offsets_file":/input/offsets.json:ro)
  offset_args+=(--offsets-file /input/offsets.json)
else
  offset_args+=(--allow-zero-offsets)
fi

calibration_args=()
calibration_mount=()
if [[ -n ${GMR_CALIBRATION_BVH_FILE:-} ]]; then
  if [[ ${GMR_NPOSE_REVIEWED:-0} != 1 ]]; then
    echo "GMR_NPOSE_REVIEWED=1 is required after visually reviewing the selected N-pose window" >&2
    exit 2
  fi
  calibration_file=$(realpath -- "$GMR_CALIBRATION_BVH_FILE")
  if [[ ! -f $calibration_file ]]; then
    echo "GMR_CALIBRATION_BVH_FILE was not found: $calibration_file" >&2
    exit 2
  fi
  npose_start_seconds=${GMR_NPOSE_START_SECONDS:-0}
  npose_seconds=${GMR_NPOSE_SECONDS:-4}
  calibration_mount+=(--volume "$calibration_file":/input/calibration.bvh:ro)
  calibration_args+=(
    --calibration-bvh-file /input/calibration.bvh
    --calibration-start-seconds "$npose_start_seconds"
    --calibration-seconds "$npose_seconds"
    --calibration-output "/data/${safe_name}-npose-arm-calibration.json"
    --arm-mode npose
    --accept-reviewed-calibration-window
  )
fi

docker "${docker_args[@]}" \
  --hostname gmr-bvh-loop \
  --network none \
  --gpus all \
  --shm-size 1g \
  --user "$(id -u):$(id -g)" \
  --cap-drop ALL \
  --security-opt no-new-privileges \
  --env NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility \
  --env DISPLAY="$display" \
  --env XAUTHORITY=/tmp/xauthority \
  --env HOME=/tmp/gmr-home \
  --tmpfs /tmp/gmr-home:rw,nosuid,nodev,size=256m,uid="$(id -u)",gid="$(id -g)" \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:ro \
  --volume "$xauthority":/tmp/xauthority:ro \
  --volume "$bvh_file":/input/motion.bvh:ro \
  --volume "$runtime_root":/data \
  "${offset_mount[@]}" \
  "${calibration_mount[@]}" \
  --entrypoint python \
  "$image" \
  /opt/mocap_humanoid/tools/xsens/loop_bvh_g1.py \
  --bvh-file /input/motion.bvh \
  --cache-file "/data/$cache_name" \
  --scale 0.01 \
  --start 1 \
  --frame-step 4 \
  "${offset_args[@]}" \
  "${calibration_args[@]}" \
  "$@"
