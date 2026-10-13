#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd -- "$script_dir/../.." && pwd)
# shellcheck source=tools/lib/xsens_source_ip.sh
source "$repo_root/tools/lib/xsens_source_ip.sh"

display=${DISPLAY:-:0}
xauthority=${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}
udp_port=${XSENS_UDP_PORT:-5555}
source_ip=${XSENS_SOURCE_IP:-}
image=${GMR_LIVE_IMAGE:-gmr-live:bb1bbe4}

if [[ ! $udp_port =~ ^[0-9]+$ ]] || (( udp_port < 1 || udp_port > 65535 )); then
  echo "XSENS_UDP_PORT must be between 1 and 65535" >&2
  exit 2
fi
require_xsens_source_ip "$source_ip"
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

runtime_root=${GMR_LIVE_RUNTIME_ROOT:-$repo_root/artifacts/xsens-live}
mkdir -p "$runtime_root"

# Name the default recording after the robot (--robot passes through to the viewer).
video_name=g1-live.mp4
previous=''
for arg in "$@"; do
  if [[ $previous == --robot && $arg == booster_t1_29dof ]] || [[ $arg == --robot=booster_t1_29dof ]]; then
    video_name=t1-live.mp4
  fi
  previous=$arg
done

docker_args=(run --rm -i)
if [[ -t 0 && -t 1 ]]; then
  docker_args+=(-t)
fi

exec docker "${docker_args[@]}" \
  --name gmr-xsens-live \
  --hostname gmr-xsens-live \
  --network host \
  --gpus all \
  --shm-size 1g \
  --user "$(id -u):$(id -g)" \
  --cap-drop ALL \
  --security-opt no-new-privileges \
  --env NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility,compute \
  --env DISPLAY="$display" \
  --env XAUTHORITY=/tmp/xauthority \
  --env HOME=/tmp/gmr-home \
  --tmpfs /tmp/gmr-home:rw,nosuid,nodev,size=256m,uid="$(id -u)",gid="$(id -g)" \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:ro \
  --volume "$xauthority":/tmp/xauthority:ro \
  --volume "$runtime_root":/data \
  "$image" \
  --port "$udp_port" \
  --source-ip "$source_ip" \
  --video-path "/data/$video_name" \
  "$@"
