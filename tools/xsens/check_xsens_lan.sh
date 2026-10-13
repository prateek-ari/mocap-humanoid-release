#!/usr/bin/env bash
set -euo pipefail

umask 077

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd -- "$script_dir/../.." && pwd)

windows_ip=${XSENS_SOURCE_IP:-}
gateway_ip=${XSENS_GATEWAY_IP:-192.168.1.1}
expected_gateway_mac=${XSENS_GATEWAY_MAC:-}
udp_port=${XSENS_UDP_PORT:-5555}
duration=${XSENS_DURATION_S:-30}
non_interactive=false
self_test=false

usage() {
  cat <<'EOF'
Usage: tools/xsens/check_xsens_lan.sh [--non-interactive] [--self-test]

Run this after Linux has a direct route to Windows through the Xsens router.
It verifies the selected router/interface, prints the Linux destination to enter
in MVN, then grades 30 seconds of MXTP02+MXTP20 traffic on UDP 5555. For the
supported dual-network setup, run setup_xsens_dual_network.sh instead.

Environment overrides:
  XSENS_SOURCE_IP       Confirmed Windows Xsens-LAN address (required)
  XSENS_GATEWAY_IP      Xsens router address (default: 192.168.1.1)
  XSENS_GATEWAY_MAC     Xsens router MAC from its label/admin page (required)
  XSENS_UDP_PORT        UDP destination port (default: 5555)
  XSENS_DURATION_S      capture duration in whole seconds (default: 30)

This script never installs software, changes routes, kills listeners, or changes
firewall rules. Reports contain aggregate metrics, not motion packet payloads.
EOF
}

while (( $# > 0 )); do
  case "$1" in
    --non-interactive)
      non_interactive=true
      ;;
    --self-test)
      self_test=true
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

for command_name in ip ss python3 mktemp tee; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    echo "ERROR: required local command is missing: $command_name" >&2
    exit 2
  fi
done

if [[ ! $udp_port =~ ^[0-9]+$ ]] || (( udp_port < 1 || udp_port > 65535 )); then
  echo "ERROR: XSENS_UDP_PORT must be between 1 and 65535" >&2
  exit 2
fi
if [[ ! $duration =~ ^[1-9][0-9]*$ ]]; then
  echo "ERROR: XSENS_DURATION_S must be a positive whole number" >&2
  exit 2
fi

artifact_root="$repo_root/artifacts/xsens-live"
mkdir -p "$artifact_root"
artifact_dir=$(mktemp -d "$artifact_root/xsens-lan-$(date +%Y%m%d-%H%M%S)-XXXXXX")
preflight_log="$artifact_dir/preflight.txt"
acceptance_log="$artifact_dir/acceptance.log"
acceptance_json="$artifact_dir/stream-acceptance.json"

log() {
  printf '%s\n' "$*" | tee -a "$preflight_log"
}

log "Xsens isolated-LAN check"
log "timestamp=$(date --iso-8601=seconds)"
log "repo=$repo_root"
log "git_commit=$(git -C "$repo_root" rev-parse HEAD 2>/dev/null || printf unknown)"
log "windows_source=$windows_ip"
log "udp_port=$udp_port"
log "artifact_dir=$artifact_dir"

if $self_test; then
  log "mode=self-test"
  python3 "$repo_root/tools/xsens/stream_acceptance.py" \
    --self-test \
    --require-joint-angles \
    --output "$acceptance_json" | tee "$acceptance_log"
  log "PASS: offline decoder self-test completed"
  exit 0
fi

if [[ -z $windows_ip ]]; then
  log "ERROR: set XSENS_SOURCE_IP to the current Windows Ethernet IPv4 from ipconfig"
  exit 2
fi
if [[ -z $expected_gateway_mac ]] ||
  ! [[ ${expected_gateway_mac//-/:} =~ ^([[:xdigit:]]{2}:){5}[[:xdigit:]]{2}$ ]]; then
  log "ERROR: set XSENS_GATEWAY_MAC to the Xsens router MAC as aa:bb:cc:dd:ee:ff"
  exit 2
fi

route_line=$(ip -4 route get "$windows_ip" 2>&1) || {
  log "ERROR: Linux has no IPv4 route to Windows $windows_ip"
  exit 3
}
log "route=$route_line"

if [[ $route_line == *" via "* ]]; then
  log "ERROR: route to $windows_ip uses a gateway; the Xsens link must be directly on-link"
  log "Check that the Windows host route selects the Xsens Ethernet interface."
  exit 3
fi

interface=$(awk '{for (i=1; i<=NF; i++) if ($i == "dev") {print $(i+1); exit}}' <<<"$route_line")
receiver_ip=$(awk '{for (i=1; i<=NF; i++) if ($i == "src") {print $(i+1); exit}}' <<<"$route_line")
if [[ -z $interface || -z $receiver_ip ]]; then
  log "ERROR: could not determine the Linux interface/source address from the route"
  exit 3
fi
case "$interface" in
  lo|docker*|br-*|tun*|tap*|wg*)
    log "ERROR: route selected non-physical interface $interface"
    exit 3
    ;;
esac
if [[ $receiver_ip == "$windows_ip" ]]; then
  log "ERROR: Linux and Windows cannot use the same address $receiver_ip"
  exit 3
fi
if ! ip -o -4 address show dev "$interface" | awk '{print $4}' | cut -d/ -f1 | grep -Fqx "$receiver_ip"; then
  log "ERROR: detected source $receiver_ip is not assigned to $interface"
  exit 3
fi

log "interface=$interface"
log "linux_receiver=$receiver_ip"
ip -br -4 address show dev "$interface" | tee -a "$preflight_log"

if command -v ping >/dev/null 2>&1; then
  ping -c 1 -W 1 -I "$interface" "$gateway_ip" >/dev/null 2>&1 || true
fi
gateway_neighbor=$(ip neigh show to "$gateway_ip" dev "$interface" || true)
gateway_mac=$(awk '{for (i=1; i<=NF; i++) if ($i == "lladdr") {print tolower($(i+1)); exit}}' \
  <<<"$gateway_neighbor")
expected_gateway_mac=${expected_gateway_mac//-/:}
expected_gateway_mac=${expected_gateway_mac,,}
log "gateway_neighbor=${gateway_neighbor:-unavailable}"
if [[ -z $gateway_mac ]]; then
  log "ERROR: could not resolve the Xsens router $gateway_ip on $interface"
  exit 3
fi
if [[ $gateway_mac != "$expected_gateway_mac" ]]; then
  log "ERROR: wrong router detected: gateway MAC is $gateway_mac"
  log "Expected the Xsens router MAC $expected_gateway_mac. Check the Xsens Ethernet route."
  exit 3
fi

if command -v ping >/dev/null 2>&1; then
  ping -c 1 -W 1 -I "$interface" "$windows_ip" >/dev/null 2>&1 || true
fi
windows_neighbor=$(ip neigh show to "$windows_ip" dev "$interface" || true)
log "windows_neighbor=${windows_neighbor:-unavailable}"
if [[ $windows_neighbor == *" FAILED"* || $windows_neighbor == *" INCOMPLETE"* || -z $windows_neighbor ]]; then
  log "WARNING: Windows did not answer ARP yet; the UDP test will be the final check."
fi

listener_output=$(ss -H -lunp "( sport = :$udp_port )" 2>/dev/null || true)
if [[ -n $listener_output ]]; then
  log "ERROR: UDP $udp_port is already owned by another process:"
  printf '%s\n' "$listener_output" | tee -a "$preflight_log"
  log "Stop that exact receiver and rerun; this script will not kill it."
  exit 4
fi

ufw_state=$(systemctl is-active ufw 2>/dev/null || true)
log "ufw_service=${ufw_state:-unknown}"
if [[ $ufw_state == active ]]; then
  if sudo -n true >/dev/null 2>&1; then
    sudo -n ufw status numbered 2>&1 | tee -a "$preflight_log" || true
  else
    log "WARNING: UFW is active but its rules require your sudo password to inspect."
  fi
  log "If no equivalent rule exists, run this in another terminal:"
  log "sudo ufw allow from $windows_ip to $receiver_ip port $udp_port proto udp"
fi

cat <<EOF

Linux is on the verified Xsens router.

In Windows Xsens Humanoid Network Streamer set:
  Preset:       ROS 2
  Destination:  $receiver_ip
  UDP port:     $udp_port
  Body data:    Position + Orientation (Quaternion), strict 23 segments
  Joint data:   Joint Angles (MXTP20)
  Props/fingers: disabled for this check

Start live preview or play an .mvn recording before continuing.
EOF

if ! $non_interactive; then
  read -r -p "Press Enter when MVN is actively streaming... "
fi

route_recheck=$(ip -4 route get "$windows_ip" 2>&1) || true
if [[ $route_recheck != *"dev $interface"* || $route_recheck != *"src $receiver_ip"* ]]; then
  log "ERROR: route changed before capture: $route_recheck"
  exit 3
fi
listener_output=$(ss -H -lunp "( sport = :$udp_port )" 2>/dev/null || true)
if [[ -n $listener_output ]]; then
  log "ERROR: UDP $udp_port became busy before capture: $listener_output"
  exit 4
fi

minimum_body_frames=$(( duration * 40 ))
log "Starting strict UDP acceptance for ${duration}s; minimum body frames=$minimum_body_frames"
set +e
python3 "$repo_root/tools/xsens/stream_acceptance.py" \
  --bind "$receiver_ip" \
  --port "$udp_port" \
  --expected-source "$windows_ip" \
  --duration "$duration" \
  --minimum-body-frames "$minimum_body_frames" \
  --minimum-rate 50 \
  --maximum-loss 0.01 \
  --require-joint-angles \
  --output "$acceptance_json" 2>&1 | tee "$acceptance_log"
acceptance_status=${PIPESTATUS[0]}
set -e

if (( acceptance_status != 0 )); then
  log "FAIL: no acceptable Xsens stream; inspect $acceptance_json and $acceptance_log"
  exit "$acceptance_status"
fi

log "PASS: accepted Windows Xsens traffic at $receiver_ip:$udp_port"
cat <<EOF

The stream passed. To continue sequentially on this same network:

  XSENS_SOURCE_IP=$windows_ip XSENS_UDP_PORT=$udp_port tools/xsens/live_operator.sh ros2

Stop ROS with Ctrl+C, verify UDP $udp_port is free, then run GMR with the measured height:

  HUMAN_HEIGHT_M=1.75  # example only; replace with the measured height
  XSENS_SOURCE_IP=$windows_ip XSENS_UDP_PORT=$udp_port \\
    tools/xsens/live_operator.sh gmr --human-height "\$HUMAN_HEIGHT_M"

Report: $acceptance_json
EOF
