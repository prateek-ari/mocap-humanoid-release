#!/usr/bin/env bash
# One live Xsens receiver and one window for G1 and T1.
set -euo pipefail
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd -- "$script_dir/../.." && pwd)
source "$repo_root/tools/lib/xsens_source_ip.sh"
source_ip=${XSENS_SOURCE_IP:-}
require_xsens_source_ip "$source_ip"
udp_port=${XSENS_UDP_PORT:-5555}
[[ $udp_port =~ ^[0-9]+$ ]] && ((udp_port >= 1 && udp_port <= 65535)) || {
  echo 'XSENS_UDP_PORT must be between 1 and 65535' >&2; exit 2;
}
display=${DISPLAY:-:0}
authority=${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}
[[ -f $authority ]] || { echo "Missing Xauthority: $authority" >&2; exit 2; }
display_number=${display##*:}; display_number=${display_number%%.*}
[[ -S /tmp/.X11-unix/X${display_number} ]] || { echo "Missing X11 socket: $display" >&2; exit 2; }
entry=$repo_root/tools/xsens/live_compare_sim.py
[[ -f $entry ]] || { echo "Missing comparison tool: $entry" >&2; exit 2; }
height_args=()
height_given=false
for arg in "$@"; do
  if [[ $arg == --human-height || $arg == --human-height=* ]]; then
    height_given=true
  fi
done
if [[ $height_given == false ]]; then
  [[ -n ${HUMAN_HEIGHT_M:-} ]] || {
    echo 'Set HUMAN_HEIGHT_M or pass --human-height in metres' >&2; exit 2;
  }
  height_args=(--human-height "$HUMAN_HEIGHT_M")
fi
image=${GMR_LIVE_IMAGE:-gmr-live:bb1bbe4}
exec docker run --rm -i --name gmr-xsens-live --network host --gpus all \
  --shm-size 1g --user "$(id -u):$(id -g)" --cap-drop ALL \
  --security-opt no-new-privileges \
  -e NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility,compute \
  -e DISPLAY="$display" -e XAUTHORITY=/tmp/xauthority -e HOME=/tmp/gmr-home \
  --tmpfs "/tmp/gmr-home:rw,nosuid,nodev,size=256m,uid=$(id -u),gid=$(id -g)" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v "$authority":/tmp/xauthority:ro \
  -v "$entry":/opt/mocap_humanoid/tools/xsens/live_compare_sim.py:ro \
  -v "$repo_root/tools/xsens/live_g1_sim.py":/opt/mocap_humanoid/tools/xsens/live_g1_sim.py:ro \
  --entrypoint python "$image" /opt/mocap_humanoid/tools/xsens/live_compare_sim.py \
  --port "$udp_port" --source-ip "$source_ip" \
  "${height_args[@]}" "$@"
