#!/usr/bin/env bash
set -euo pipefail

udp_port=${XSENS_UDP_PORT:-5555}
if [[ ! $udp_port =~ ^[0-9]+$ ]] || (( udp_port < 1 || udp_port > 65535 )); then
  echo "XSENS_UDP_PORT must be between 1 and 65535" >&2
  exit 2
fi

ros2 run xsens_mvn_ros2_stream xsens_mvn_ros2_stream_node \
  --ros-args -p udp_port:="$udp_port" &
node_pid=$!

cleanup() {
  kill -INT "$node_pid" 2>/dev/null || true
  wait "$node_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

until ros2 lifecycle get /xsens_mvn_ros2_stream_node >/dev/null 2>&1; do
  if ! kill -0 "$node_pid" 2>/dev/null; then
    wait "$node_pid"
    exit $?
  fi
  sleep 0.2
done

ros2 lifecycle set /xsens_mvn_ros2_stream_node configure
ros2 lifecycle set /xsens_mvn_ros2_stream_node activate
ros2 param get /xsens_mvn_ros2_stream_node udp_port
wait "$node_pid"
