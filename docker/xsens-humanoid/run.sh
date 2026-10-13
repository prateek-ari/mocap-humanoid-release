#!/usr/bin/env bash
set -euo pipefail

if (( $# != 1 )); then
  echo "Usage: $0 /absolute/path/to/MVN-Studio-2026.6.0.appimage" >&2
  exit 2
fi

appimage=$1
if [[ $appimage != /* || ! -f $appimage ]]; then
  echo "AppImage must be an existing absolute path: $appimage" >&2
  exit 2
fi

display=${DISPLAY:-:0}
xauthority=${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}
display_number=${display##*:}
display_number=${display_number%%.*}
if [[ ! -S /tmp/.X11-unix/X${display_number} ]]; then
  echo "X11 socket for DISPLAY=$display was not found" >&2
  exit 2
fi
if [[ ! -f $xauthority ]]; then
  echo "X11 authority file was not found: $xauthority" >&2
  exit 2
fi

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd -- "$script_dir/../.." && pwd)
runtime_root=${XSENS_RUNTIME_ROOT:-$repo_root/artifacts/xsens-humanoid}
mkdir -p "$runtime_root/home" "$runtime_root/data"

docker_args=(run --rm -i)
if [[ -t 0 && -t 1 ]]; then
  docker_args+=(-t)
fi

exec docker "${docker_args[@]}" \
  --name xsens-humanoid \
  --hostname xsens-humanoid \
  --network host \
  --gpus all \
  --shm-size 1g \
  --user "$(id -u):$(id -g)" \
  --cap-drop ALL \
  --security-opt no-new-privileges \
  --env NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility,compute \
  --env DISPLAY="$display" \
  --env XAUTHORITY=/tmp/xauthority \
  --env QT_QPA_PLATFORM=xcb \
  --env QT_X11_NO_MITSHM=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:ro \
  --volume "$xauthority":/tmp/xauthority:ro \
  --volume "$runtime_root/home":/home/xsens \
  --volume "$runtime_root/data":/data \
  --volume "$appimage":/opt/xsens/MVN-Studio.appimage:ro \
  xsens-humanoid:2026.6
