#!/usr/bin/env bash
set -euo pipefail

# Interface names are site-specific, so there are deliberately no defaults:
# guessing the wrong adapter would rewrite the wrong NetworkManager profile.
wifi_interface=${XSENS_WIFI_INTERFACE:-}
ethernet_interface=${XSENS_ETHERNET_INTERFACE:-}
ethernet_profile=${XSENS_ETHERNET_PROFILE:-Xsens Ethernet}
windows_ip=${XSENS_SOURCE_IP:-}
udp_port=${XSENS_UDP_PORT:-5555}
configure_only=false
non_interactive=false
dry_run=false

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
checker="$script_dir/check_xsens_lan.sh"

usage() {
  cat <<'EOF'
Usage: tools/xsens/setup_xsens_dual_network.sh [OPTIONS]

Keep Internet/DNS on Wi-Fi and route only the Windows Xsens host through
Ethernet, then run the Xsens UDP acceptance checker.

Options:
  --configure-only   Configure and verify networking without starting capture
  --non-interactive  Do not wait for Enter before the 30-second UDP check
  --dry-run          Print the persistent NetworkManager change without applying it
  -h, --help         Show this help

Environment overrides:
  XSENS_WIFI_INTERFACE      Internet interface, e.g. from `ip -br link` (required)
  XSENS_ETHERNET_INTERFACE  Xsens interface, e.g. from `ip -br link` (required)
  XSENS_ETHERNET_PROFILE    Dedicated NetworkManager profile (default: Xsens Ethernet)
  XSENS_SOURCE_IP           Confirmed Windows Xsens address (required)
  XSENS_GATEWAY_MAC         Xsens router MAC required by the capture checker
  XSENS_UDP_PORT            Stream destination port (default: 5555)

The change is persistent but limited to the named Ethernet profile. The script
does not bridge networks, enable Internet sharing, modify firewall rules, or
stop processes. It may prompt once for sudo to update NetworkManager.
EOF
}

while (( $# > 0 )); do
  case "$1" in
    --configure-only)
      configure_only=true
      ;;
    --non-interactive)
      non_interactive=true
      ;;
    --dry-run)
      dry_run=true
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

for command_name in ip nmcli grep awk python3; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    echo "ERROR: required command is missing: $command_name" >&2
    exit 2
  fi
done
if [[ ! -x $checker ]]; then
  echo "ERROR: checker is missing or not executable: $checker" >&2
  exit 2
fi
if [[ ! $udp_port =~ ^[0-9]+$ ]] || (( udp_port < 1 || udp_port > 65535 )); then
  echo "ERROR: XSENS_UDP_PORT must be between 1 and 65535" >&2
  exit 2
fi
if [[ -z $wifi_interface || -z $ethernet_interface ]]; then
  echo "ERROR: set XSENS_WIFI_INTERFACE (Internet) and XSENS_ETHERNET_INTERFACE (Xsens router)" >&2
  echo "Find the adapter names with: ip -br link; nmcli device status" >&2
  echo "Example: XSENS_WIFI_INTERFACE=wlan0 XSENS_ETHERNET_INTERFACE=enp3s0 XSENS_SOURCE_IP=<windows ip> $0" >&2
  exit 2
fi
if [[ -z $windows_ip ]]; then
  echo "ERROR: set XSENS_SOURCE_IP to the current Windows Ethernet IPv4 from ipconfig" >&2
  exit 2
fi
if ! python3 - "$windows_ip" <<'PY'
import ipaddress
import sys

try:
    address = ipaddress.IPv4Address(sys.argv[1])
except ipaddress.AddressValueError:
    raise SystemExit(1)
raise SystemExit(address.is_unspecified or address.is_multicast or address.is_loopback)
PY
then
  echo "ERROR: XSENS_SOURCE_IP must be a usable IPv4 host address" >&2
  exit 2
fi

if ! ip link show dev "$wifi_interface" >/dev/null 2>&1; then
  echo "ERROR: Wi-Fi interface does not exist: $wifi_interface" >&2
  exit 3
fi
if ! ip link show dev "$ethernet_interface" >/dev/null 2>&1; then
  echo "ERROR: Ethernet interface does not exist: $ethernet_interface" >&2
  exit 3
fi
if [[ $wifi_interface == "$ethernet_interface" ]]; then
  echo "ERROR: Wi-Fi and Ethernet interfaces must be different" >&2
  exit 3
fi

wifi_ip=$(ip -o -4 address show dev "$wifi_interface" scope global |
  awk 'NR == 1 {split($4, address, "/"); print address[1]}')
wifi_default=$(ip -4 route show default dev "$wifi_interface" | head -n 1)
if [[ -z $wifi_ip || -z $wifi_default ]]; then
  echo "ERROR: $wifi_interface must be connected to the Internet Wi-Fi first." >&2
  echo "Connect Wi-Fi, leave it enabled, and rerun this script." >&2
  exit 3
fi

carrier_file="/sys/class/net/$ethernet_interface/carrier"
if [[ ! -r $carrier_file || $(<"$carrier_file") != 1 ]]; then
  echo "ERROR: no Ethernet carrier on $ethernet_interface." >&2
  echo "Connect the cable from this Linux PC to the Xsens router and retry." >&2
  exit 3
fi

echo "Dual-network configuration"
echo "  Internet Wi-Fi: $wifi_interface ($wifi_ip)"
echo "  Xsens Ethernet: $ethernet_interface"
echo "  Windows target: $windows_ip"
echo "  UDP port:       $udp_port"

profile_settings=(
  connection.autoconnect yes
  connection.autoconnect-priority 100
  ipv4.method auto
  ipv4.never-default yes
  ipv4.ignore-auto-dns yes
  ipv4.ignore-auto-routes yes
  ipv4.route-metric 700
  ipv4.routes "$windows_ip/32"
  ipv6.method disabled
)

if $dry_run; then
  printf 'Would create or update dedicated profile %q with:\n  ' "$ethernet_profile"
  printf '%q ' connection.interface-name "$ethernet_interface"
  printf '%q ' "${profile_settings[@]}"
  printf '\nWould activate profile %q on %q and verify both routes.\n' \
    "$ethernet_profile" "$ethernet_interface"
  exit 0
fi

if (( EUID == 0 )); then
  nmcli_admin=(nmcli)
else
  if ! command -v sudo >/dev/null 2>&1; then
    echo "ERROR: sudo is required to modify the system NetworkManager profile" >&2
    exit 2
  fi
  sudo -v
  nmcli_admin=(sudo nmcli)
fi

network_changed=false
network_ready=false
profile_uuid=""
rollback_failed_setup() {
  status=$?
  trap - EXIT
  if (( status != 0 )) && $network_changed && ! $network_ready; then
    echo "Rolling back the failed Xsens Ethernet activation..." >&2
    if [[ -n $profile_uuid ]]; then
      "${nmcli_admin[@]}" --wait 10 connection down uuid "$profile_uuid" >/dev/null 2>&1 || true
      # The named profile is tool-owned. Remove a failed configuration rather
      # than leave persistent partial routing state behind.
      "${nmcli_admin[@]}" connection delete uuid "$profile_uuid" >/dev/null 2>&1 || true
    fi
  fi
  exit "$status"
}
trap rollback_failed_setup EXIT

if nmcli -g connection.uuid connection show "$ethernet_profile" >/dev/null 2>&1; then
  profile_uuid=$(nmcli -g connection.uuid connection show "$ethernet_profile")
  if [[ $profile_uuid == *$'\n'* ]]; then
    echo "ERROR: more than one NetworkManager profile is named $ethernet_profile" >&2
    exit 3
  fi
  profile_type=$(nmcli -g connection.type connection show uuid "$profile_uuid")
  if [[ $profile_type != 802-3-ethernet ]]; then
    echo "ERROR: $ethernet_profile exists but is not an Ethernet profile" >&2
    exit 3
  fi
  existing_interface=$(nmcli -g connection.interface-name connection show uuid "$profile_uuid")
  if [[ -n $existing_interface && $existing_interface != "$ethernet_interface" ]]; then
    echo "ERROR: $ethernet_profile is already assigned to $existing_interface" >&2
    exit 3
  fi
  existing_dns=$(nmcli -g ipv4.dns connection show uuid "$profile_uuid")
  if [[ -n $existing_dns ]]; then
    echo "ERROR: dedicated profile $ethernet_profile contains manual DNS: $existing_dns" >&2
    echo "Remove that DNS explicitly before using this safety-focused setup." >&2
    exit 3
  fi
  network_changed=true
  "${nmcli_admin[@]}" connection modify uuid "$profile_uuid" \
    connection.interface-name "$ethernet_interface" "${profile_settings[@]}"
else
  "${nmcli_admin[@]}" connection add type ethernet ifname "$ethernet_interface" \
    con-name "$ethernet_profile" "${profile_settings[@]}"
  profile_uuid=$(nmcli -g connection.uuid connection show "$ethernet_profile")
  network_changed=true
fi

if ! "${nmcli_admin[@]}" --wait 30 connection up uuid "$profile_uuid" \
  ifname "$ethernet_interface"; then
  echo "ERROR: Xsens Ethernet did not obtain a DHCP lease within 30 seconds" >&2
  exit 4
fi

internet_route=$(ip -4 route get 1.1.1.1 2>&1) || {
  echo "ERROR: no IPv4 Internet route remains" >&2
  exit 4
}
windows_route=$(ip -4 route get "$windows_ip" 2>&1) || {
  echo "ERROR: no route to Windows $windows_ip" >&2
  exit 4
}
ethernet_ip=$(awk '{for (i=1; i<=NF; i++) if ($i == "src") {print $(i+1); exit}}' \
  <<<"$windows_route")
if [[ -z $ethernet_ip ]]; then
  echo "ERROR: the Windows route has no Ethernet source address" >&2
  exit 4
fi
if ! python3 - "$ethernet_ip" "$windows_ip" <<'PY'
import ipaddress
import sys

receiver = ipaddress.IPv4Address(sys.argv[1])
source_network = ipaddress.IPv4Network(f"{sys.argv[2]}/24", strict=False)
raise SystemExit(receiver not in source_network)
PY
then
  echo "ERROR: Ethernet lease $ethernet_ip is outside the Windows /24 network" >&2
  exit 4
fi
if [[ $ethernet_ip == "$wifi_ip" ]]; then
  echo "ERROR: both interfaces received the same IPv4 address: $ethernet_ip" >&2
  echo "The script has deactivated the Xsens profile; reserve distinct DHCP addresses." >&2
  exit 4
fi
if [[ $ethernet_ip == "$windows_ip" ]]; then
  echo "ERROR: Linux and Windows both use $ethernet_ip" >&2
  exit 4
fi
if ! ip -o -4 address show dev "$ethernet_interface" scope global |
  awk '{print $4}' | grep -Eq "^${ethernet_ip//./\\.}/24$"; then
  echo "ERROR: Xsens Ethernet did not receive the expected 192.168.1.0/24-style lease" >&2
  exit 4
fi

echo
echo "Route verification"
echo "  Internet: $internet_route"
echo "  Windows:  $windows_route"

if [[ $internet_route != *"dev $wifi_interface"* ]]; then
  echo "ERROR: Internet traffic is not using $wifi_interface" >&2
  exit 4
fi
if [[ $internet_route != *"src $wifi_ip"* ]]; then
  echo "ERROR: Internet traffic is not using Wi-Fi address $wifi_ip" >&2
  exit 4
fi
if [[ $windows_route != *"dev $ethernet_interface"* ]]; then
  echo "ERROR: Windows Xsens traffic is not using $ethernet_interface" >&2
  exit 4
fi
if [[ $windows_route == *" via "* ]]; then
  echo "ERROR: Windows Xsens traffic unexpectedly uses a gateway" >&2
  exit 4
fi
if [[ $windows_route != *"src $ethernet_ip"* ]]; then
  echo "ERROR: Windows Xsens traffic is not using Ethernet address $ethernet_ip" >&2
  exit 4
fi
if ip -4 route show default dev "$ethernet_interface" | grep -q .; then
  echo "ERROR: Xsens Ethernet still installed an IPv4 default route" >&2
  exit 4
fi
ethernet_dns=$(nmcli -g IP4.DNS device show "$ethernet_interface")
if [[ -n $ethernet_dns ]]; then
  echo "ERROR: Xsens Ethernet unexpectedly supplied DNS: $ethernet_dns" >&2
  exit 4
fi

network_ready=true
trap - EXIT

echo
echo "PASS: Internet remains on $wifi_interface; $windows_ip uses $ethernet_interface."
if command -v ping >/dev/null 2>&1 && ping -c 1 -W 2 -I "$wifi_interface" 1.1.1.1 >/dev/null 2>&1; then
  echo "PASS: Internet IPv4 connectivity responded through $wifi_interface."
else
  echo "WARNING: Internet routing is correct, but the external ping did not answer."
fi
if getent ahostsv4 example.com >/dev/null 2>&1; then
  echo "PASS: DNS resolution is working."
else
  echo "WARNING: DNS resolution did not answer; check the Wi-Fi connection."
fi
echo "Set the Windows Xsens Network Streamer destination to $ethernet_ip:$udp_port."

if $configure_only; then
  exit 0
fi

checker_args=()
if $non_interactive; then
  checker_args+=(--non-interactive)
fi
export XSENS_SOURCE_IP="$windows_ip"
export XSENS_UDP_PORT="$udp_port"
exec "$checker" "${checker_args[@]}"
