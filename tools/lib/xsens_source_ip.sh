# shellcheck shell=bash
# Shared validation for XSENS_SOURCE_IP. Source this file, then call
#   require_xsens_source_ip "${XSENS_SOURCE_IP:-}"
# It exits the calling script with status 2 when the value is unset or is not a
# usable IPv4 host address (unspecified, multicast and loopback are rejected).

require_xsens_source_ip() {
  local source_ip=${1:-}
  if [[ -z $source_ip ]]; then
    echo "Set XSENS_SOURCE_IP to the current Windows Xsens-facing IPv4 from ipconfig" >&2
    exit 2
  fi
  if ! python3 - "$source_ip" <<'PY'
import ipaddress
import sys

try:
    address = ipaddress.IPv4Address(sys.argv[1])
except ipaddress.AddressValueError:
    raise SystemExit(1)
raise SystemExit(address.is_unspecified or address.is_multicast or address.is_loopback)
PY
  then
    echo "XSENS_SOURCE_IP must be a usable IPv4 host address" >&2
    exit 2
  fi
}
