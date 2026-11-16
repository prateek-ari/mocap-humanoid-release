#!/usr/bin/env bash
set -euo pipefail

action=${1:-help}
if (( $# > 0 )); then
  shift
fi

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd -- "$script_dir/../.." && pwd)
# shellcheck source=tools/lib/xsens_source_ip.sh
source "$repo_root/tools/lib/xsens_source_ip.sh"
udp_port=${XSENS_UDP_PORT:-5555}
source_ip=${XSENS_SOURCE_IP:-}
duration=${XSENS_DURATION_S:-30}
ros_image=${XSENS_ROS_IMAGE:-xsens-ros2:jazzy}

# Every stage holds update.lock shared while it runs (exec keeps the descriptor), so it cannot
# start during Mocap Update (tools/setup/update.sh) and the update cannot start meanwhile.
case $action in
  acceptance|ros2|gmr)
    update_lock=${XDG_STATE_HOME:-~/.local/state}/mocap-g1/update.lock  # ~ works without HOME
    mkdir -p -- "${update_lock%/*}"
    exec {update_fd}>>"$update_lock"
    if ! flock -s -n "$update_fd"; then
      echo "An update is running. Wait until Mocap Update has finished." >&2
      exit 2
    fi
    ;;
esac

require_source_ip() {
  require_xsens_source_ip "$source_ip"
}

case "$action" in
  acceptance)
    require_source_ip
    # A caller-supplied --output replaces the default dated report folder.
    output_args=()
    caller_output=0
    for arg in "$@"; do
      case $arg in
        --output|--output=*) caller_output=1 ;;
      esac
    done
    if (( ! caller_output )); then
      artifact_dir="$repo_root/artifacts/xsens-live/$(date +%Y%m%d-%H%M%S)"
      mkdir -p "$artifact_dir"
      output_args=(--output "$artifact_dir/stream-acceptance.json")
    fi
    exec python3 "$repo_root/tools/xsens/stream_acceptance.py" \
      --bind 0.0.0.0 \
      --port "$udp_port" \
      --expected-source "$source_ip" \
      --duration "$duration" \
      --minimum-body-frames 300 \
      --minimum-rate 50 \
      --maximum-loss 0.01 \
      --require-joint-angles \
      ${output_args[@]+"${output_args[@]}"} \
      "$@"
    ;;
  ros2)
    if (( $# > 0 )); then
      echo "The ros2 stage does not accept additional arguments" >&2
      exit 2
    fi
    docker_args=(run --rm -i)
    if [[ -t 0 && -t 1 ]]; then
      docker_args+=(-t)
    fi
    exec docker "${docker_args[@]}" \
      --name "xsens-stream-${udp_port}" \
      --network host \
      --cap-drop ALL \
      --security-opt no-new-privileges \
      --env XSENS_UDP_PORT="$udp_port" \
      --volume "$repo_root/docker/xsens-ros2/run-stream.sh":/opt/run-stream.sh:ro \
      "$ros_image" /opt/run-stream.sh
    ;;
  gmr)
    require_source_ip
    export XSENS_UDP_PORT="$udp_port"
    export XSENS_SOURCE_IP="$source_ip"
    exec "$repo_root/docker/gmr-live/run.sh" "$@"
    ;;
  help|-h|--help)
    cat <<'EOF'
Usage: tools/xsens/live_operator.sh <acceptance|ros2|gmr> [arguments]

Run stages sequentially; every stage binds the same UDP port.

  acceptance  Grade 30 seconds of MXTP02+MXTP20 from the configured Windows IP
  ros2        Publish the accepted stream as ROS 2 topics
  gmr         Show the live retargeted robot in MuJoCo (--robot booster_k1 (default),
              unitree_g1 or booster_t1_29dof; N or --calibrate-after SECONDS: neutral
              calibration)

Environment:
  XSENS_SOURCE_IP=<current Windows Xsens-facing IPv4> (required for acceptance/GMR)
  XSENS_UDP_PORT=5555
  XSENS_DURATION_S=30
EOF
    ;;
  *)
    echo "Unknown action: $action" >&2
    exit 2
    ;;
esac
