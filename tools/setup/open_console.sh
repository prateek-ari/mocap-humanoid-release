#!/usr/bin/env bash
# Open the Setup console in the browser, starting it in the background first if needed.
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage: bash tools/setup/open_console.sh [--install-shortcut | --stop] [--no-browser]

  (no option)         Start the Setup console if it is not running, then open it
  --install-shortcut  Add a "Mocap Setup" launcher to the applications menu (right-click
                      it for "Stop the Setup console"), and "K1 Hardware Tests" when the
                      K1 test tools are installed, "K1 Live Demo" and "Mocap Update"
  --stop              Stop a Setup console started by this script
  --no-browser        Start it without opening a browser

The console listens on http://127.0.0.1:8784/ only (this PC); set MOCAP_SETUP_PORT to change it.
Its log is ~/.local/state/mocap-g1/setup-console.log.
EOF
}

# The menu shortcut has no terminal, so also show errors as a desktop notification.
die() {
  printf 'ERROR: %s\n' "$*" >&2
  [[ -t 2 ]] || notify-send 'Mocap Setup' "$*" >/dev/null 2>&1 || true
  exit 2
}

script_dir=$(cd -- "${BASH_SOURCE[0]%/*}" && pwd -P)
repo_root=$(cd -- "$script_dir/../.." && pwd -P)
venv_python=$repo_root/.venv/bin/python
port=${MOCAP_SETUP_PORT:-8784}
[[ $port =~ ^[0-9]+$ ]] && (( port >= 1 && port <= 65535 )) || die "MOCAP_SETUP_PORT must be 1-65535"
url=http://127.0.0.1:$port/
state_dir=${XDG_STATE_HOME:-$HOME/.local/state}/mocap-g1
pid_file=$state_dir/setup-console.pid
log_file=$state_dir/setup-console.log

action=open browser=1
while (($#)); do
  case $1 in
    --install-shortcut) action=shortcut ;;
    --stop) action=stop ;;
    --no-browser) browser=0 ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown option: $1" ;;
  esac
  shift
done

# Prints "200 display" or "200 nodisplay" for a Setup console (whether it can open
# viewer windows), another HTTP code or "busy" for any other program on the
# port, and 000 when nothing listens.
probe() {
  "$venv_python" - "$url" <<'PY'
import json, sys, urllib.error, urllib.request
try:
    with urllib.request.urlopen(sys.argv[1] + "api/v1/setup", timeout=1) as response:
        ready = json.load(response).get("readiness", {}).get("display")
        print(response.status, "nodisplay" if ready is False else "display")
except urllib.error.HTTPError as exc:
    print(exc.code)
except urllib.error.URLError as exc:
    print("000" if isinstance(exc.reason, ConnectionRefusedError) else "busy")
except Exception:  # answered, but not with HTTP/JSON
    print("busy")
PY
}

# Stop the console this script started. The console stops a running job (such
# as a live viewer) on the way out, which can take up to about 40 s.
stop_console() {
  [[ -f $pid_file ]] || return 1
  local pid
  pid=$(<"$pid_file")
  if [[ $pid =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
    printf 'Stopping the Setup console and any running job...\n'
    kill -INT "$pid"
    for _ in {1..600}; do kill -0 "$pid" 2>/dev/null || break; sleep 0.1; done
    ! kill -0 "$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  fi
  rm -f -- "$pid_file"
}

start_console() {
  mkdir -p -- "$state_dir"
  nohup "$venv_python" "$repo_root/tools/operator_console.py" --setup --port "$port" \
    >> "$log_file" 2>&1 < /dev/null &
  printf '%s\n' "$!" > "$pid_file"
  for _ in {1..100}; do
    [[ $(probe) == 200* ]] && return 0
    kill -0 "$(<"$pid_file")" 2>/dev/null || { rm -f -- "$pid_file"; die "the console stopped; see $log_file"; }
    sleep 0.1
  done
  die "the console did not start; see $log_file"
}

case $action in
  shortcut)
    applications=${XDG_DATA_HOME:-$HOME/.local/share}/applications
    mkdir -p -- "$applications"
    entry=$applications/mocap-setup.desktop
    # Desktop Entry quoting: escape \ " ` $ inside a double-quoted argument.
    quote() {
      local value=$1
      value=${value//\\/\\\\}; value=${value//\"/\\\"}
      value=${value//\`/\\\`}; value=${value//\$/\\\$}
      printf '"%s"' "$value"
    }
    target=$(quote "$script_dir/open_console.sh")
    {
      printf '[Desktop Entry]\nType=Application\nVersion=1.0\nName=Mocap Setup\n'
      printf 'Comment=Open the mocap session setup console\n'
      printf 'Exec=bash %s\n' "$target"
      printf 'Icon=applications-engineering\nTerminal=false\nStartupNotify=true\n'
      printf 'Categories=Utility;Engineering;\nActions=stop;\n\n'
      printf '[Desktop Action stop]\nName=Stop the Setup console\n'
      printf 'Exec=bash %s --stop\n' "$target"
    } > "$entry.tmp"
    mv -- "$entry.tmp" "$entry"
    printf 'Added "Mocap Setup" to the applications menu (%s).\n' "$entry"
    k1_entry=$applications/mocap-k1-tests.desktop
    if [[ -x $venv_python ]] &&
        "$venv_python" -c 'import booster_robotics_sdk_python, tkinter' >/dev/null 2>&1; then
      {
        printf '[Desktop Entry]\nType=Application\nVersion=1.0\nName=K1 Hardware Tests\n'
        printf 'Comment=Booster K1 step-by-step hardware tests\n'
        printf 'Exec=%s %s\n' "$(quote "$venv_python")" "$(quote "$repo_root/tools/control/k1_commission_gui.py")"
        printf 'Path=%s\n' "$repo_root"
        printf 'Icon=applications-science\nTerminal=false\nStartupNotify=true\n'
        printf 'Categories=Utility;Engineering;\n'
      } > "$k1_entry.tmp"
      mv -- "$k1_entry.tmp" "$k1_entry"
      printf 'Added "K1 Hardware Tests" to the applications menu (%s).\n' "$k1_entry"
    else
      rm -f -- "$k1_entry"
      printf 'K1 test tools not installed: no "K1 Hardware Tests" menu entry.\n'
    fi
    demo_entry=$applications/mocap-k1-demo.desktop
    {
      printf '[Desktop Entry]\nType=Application\nVersion=1.0\nName=K1 Live Demo\n'
      printf 'Comment=The robot copies the performer: the Setup console on its live-demo page\n'
      printf 'Exec=bash %s\n' "$(quote "$repo_root/k1-demo.sh")"
      printf 'Icon=media-playback-start\nTerminal=false\nStartupNotify=true\n'
      printf 'Categories=Utility;Engineering;\n'
    } > "$demo_entry.tmp"
    mv -- "$demo_entry.tmp" "$demo_entry"
    printf 'Added "K1 Live Demo" to the applications menu (%s).\n' "$demo_entry"
    update_entry=$applications/mocap-update.desktop
    {
      printf '[Desktop Entry]\nType=Application\nVersion=1.0\nName=Mocap Update\n'
      printf 'Comment=Download and install the newest version (stop the Setup console first)\n'
      printf 'Exec=bash %s\n' "$(quote "$script_dir/update.sh")"
      printf 'Icon=system-software-update\nTerminal=true\nStartupNotify=true\n'
      printf 'Categories=Utility;Engineering;\n'
    } > "$update_entry.tmp"
    mv -- "$update_entry.tmp" "$update_entry"
    printf 'Added "Mocap Update" to the applications menu (%s).\n' "$update_entry"
    exit 0 ;;
  stop)
    if stop_console; then
      message='Stopped the Setup console.'
    else
      message='No Setup console was started by this script.'
    fi
    printf '%s\n' "$message"
    # The menu's "Stop the Setup console" has no terminal: show the outcome.
    [[ -t 1 ]] || notify-send 'Mocap Setup' "$message" >/dev/null 2>&1 || true
    exit 0 ;;
esac

[[ -x $venv_python ]] || die "project environment missing; run: bash tools/setup/install.sh"
# The console backend holds update.lock shared while it runs (it inherits this descriptor), so
# Mocap Update cannot run meanwhile and the console cannot start during an update.
mkdir -p -- "$state_dir"
exec {update_fd}>>"$state_dir/update.lock"
flock -s -n "$update_fd" || die 'An update is running. Wait until Mocap Update has finished.'
case $(probe) in
  '200 nodisplay')
    # Started without a desktop (for example over SSH). Restart it from this
    # desktop session so viewer windows can open; never stop a console we did not start.
    if [[ -n ${DISPLAY:-} ]] && stop_console; then
      start_console
    else
      printf 'Note: the running console has no desktop display, so viewer windows cannot open.\n'
    fi ;;
  200*) ;;
  000) start_console ;;
  *) die "port $port is used by another program; stop it or set MOCAP_SETUP_PORT" ;;
esac

exec {update_fd}>&-  # only the backend keeps it, never the browser
printf 'Setup console: %s\n' "$url"
if (( browser )); then
  xdg-open "$url" >/dev/null 2>&1 || printf 'Open %s in your browser.\n' "$url"
fi
