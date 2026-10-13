#!/usr/bin/env bash
# Ask for one session's values step by step and save them as session.env.
# Every question shows a standard default (or your previous answer); Enter accepts it.
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage: bash tools/setup/new_session.sh [options]

Options:
  --route live|recorded   Workflow for this session (asked if omitted)
  --robot ROBOT           unitree_g1 or booster_t1_29dof
  --workspace ABS_PATH    GMR runtime folder (default: ~/mocap-runtime)
  --defaults              Accept every default without asking
  -h, --help              Show this help

Writes <session folder>/session.env and points ./session.env in this checkout
at it. Load it in each new terminal with:  source session.env
Defaults come from the previous session.env when there is one.
EOF
}

die() { printf 'ERROR: %s\n' "$*" >&2; exit 2; }

script_dir=$(cd -- "${BASH_SOURCE[0]%/*}" && pwd -P)
repo_root=$(cd -- "$script_dir/../.." && pwd -P)
current_link=$repo_root/session.env
[[ -n ${HOME:-} ]] || die "HOME is not set"

route='' robot='' workspace='' defaults=0
while (($#)); do
  case $1 in
    --route|--robot|--workspace)
      (($# >= 2)) && [[ -n $2 && $2 != --* ]] || die "$1 needs a value"
      case $1 in
        --route) route=$2 ;;
        --robot) robot=$2 ;;
        --workspace) workspace=$2 ;;
      esac
      shift 2 ;;
    --defaults) defaults=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown option: $1" ;;
  esac
done
[[ -z $route || $route == recorded || $route == live ]] || die "--route must be recorded or live"
[[ -z $robot || $robot == unitree_g1 || $robot == booster_t1_29dof ]] || die "unsupported robot: $robot"

# Prefill from the previous session. The file is written by this script.
if [[ -f $current_link ]]; then
  # shellcheck disable=SC1090
  source "$current_link"
fi

ask() {  # ask VARIABLE "Question" default
  local _answer=''
  if (( ! defaults )); then
    read -r -p "$2 [$3]: " _answer || die "no answer; nothing was saved"
  fi
  printf -v "$1" '%s' "${_answer:-$3}"
}
ask_yes() {  # ask_yes "Question" y|n
  local reply hint=y/N
  [[ $2 == y ]] && hint=Y/n
  (( ! defaults )) || { [[ $2 == y ]]; return; }
  while true; do
    read -r -p "$1 [$hint]: " reply || die "no answer; nothing was saved"
    case ${reply:-$2} in
      [Yy]|[Yy]es) return 0 ;;
      [Nn]|[Nn]o) return 1 ;;
    esac
  done
}
step() { (( defaults )) || printf '\n%s\n' "$*"; }
choose() {  # choose VARIABLE default-number value1 value2: numbered menu
  local -n target=$1
  local reply
  while true; do
    ask reply 'Choice' "$2"
    case $reply in
      1) target=$3; return ;;
      2) target=$4; return ;;
    esac
  done
}
absolute() {  # expand a leading ~ and require an absolute path
  local value=$1
  [[ $value == '~' ]] && value=$HOME
  [[ $value == '~/'* ]] && value=$HOME/${value#'~/'}
  [[ $value == /* ]] || die "use an absolute path (or ~/...): $1"
  printf '%s' "$value"
}
number_between() {  # number_between VALUE MIN MAX
  [[ $1 =~ ^[0-9]+([.][0-9]+)?$ ]] && awk -v v="$1" -v lo="$2" -v hi="$3" 'BEGIN { exit !(v >= lo && v <= hi) }'
}
ipv4() {
  local IFS=. part
  local -a parts
  [[ $1 =~ ^[0-9]{1,3}([.][0-9]{1,3}){3}$ ]] || return 1
  read -r -a parts <<< "$1"
  for part in "${parts[@]}"; do (( 10#$part <= 255 )) || return 1; done
  (( 10#${parts[0]} != 0 && 10#${parts[0]} != 127 && 10#${parts[0]} < 224 ))
}
tilde() { [[ $1 == "$HOME"/* ]] && printf '~/%s' "${1#"$HOME"/}" || printf '%s' "$1"; }

(( defaults )) || printf 'New mocap session. Press Enter to accept the [default] answer.\n'

if [[ -z $route ]]; then
  step 'Which workflow is this session for?
  1) Live Xsens suit -> G1 or Booster T1 simulation
  2) Recorded BVH export -> G1 or Booster T1 simulation'
  default=1
  [[ ${SESSION_ROUTE:-} != recorded ]] || default=2
  choose route "$default" live recorded
fi
if [[ -z $robot ]]; then
  step 'Which robot?
  1) Unitree G1
  2) Booster T1'
  default=1
  [[ ${ROBOT:-} != booster_t1_29dof ]] || default=2
  choose robot "$default" unitree_g1 booster_t1_29dof
fi

step 'Session folder: a new folder for every take keeps results apart.'
default=$HOME/mocap-sessions/$(date +%Y-%m-%d-%H%M)
candidate=$default suffix=1
while [[ -e $candidate ]]; do suffix=$((suffix + 1)); candidate=$default-$suffix; done
ask session_dir 'Folder' "$(tilde "$candidate")"
session_dir=$(absolute "$session_dir")
if [[ -f $session_dir/session.env ]] && ! ask_yes "$(tilde "$session_dir")/session.env exists. Replace it?" n; then
  die "kept the existing session; choose another folder"
fi

step 'Performer height in metres. Measure the performer; do not guess.'
while true; do
  ask height 'Height' "${HUMAN_HEIGHT_M:-1.75}"
  number_between "$height" 0.8 2.5 && break
  (( ! defaults )) || die "height must be 0.8-2.5 m"
  printf '  Enter a number from 0.8 to 2.5.\n'
done

# Recorded values. Live sessions keep them too, so a later recorded session is prefilled.
mocap_ws=$(absolute "${workspace:-${MOCAP_WS:-~/mocap-runtime}}")
motion=$(absolute "${MOTION_BVH:-~/mocap-data/motion.bvh}")
npose=$(absolute "${NPOSE_BVH:-~/mocap-data/neutral.bvh}")
scale=${BVH_SCALE:-0.01}
start=${START_FRAME:-0} end=${END_FRAME:-600} frame_step=${FRAME_STEP:-1}
npose_start=${NPOSE_START_S:-2.0} npose_duration=${NPOSE_DURATION_S:-1.0}
offsets=${OFFSETS_FILE:-}
windows_ip=${WINDOWS_XSENS_IP:-} linux_ip=${LINUX_XSENS_IP:-}

if [[ $route == recorded ]]; then
  if [[ -z $workspace ]]; then
    step 'GMR runtime folder (created by the installer; outside this checkout).'
    ask mocap_ws 'Folder' "$(tilde "$mocap_ws")"
    mocap_ws=$(absolute "$mocap_ws")
  fi
  step 'Exported BVH files (3DS Max preset). They can be copied there later.'
  ask motion 'Motion take' "$(tilde "$motion")"
  motion=$(absolute "$motion")
  ask npose 'Neutral-pose take' "$(tilde "$npose")"
  npose=$(absolute "$npose")
  step 'BVH units chosen at export:
  1) Centimetres (scale 0.01, the Xsens default)
  2) Metres (scale 1)'
  default=1
  [[ $scale != 1 && $scale != 1.0 ]] || default=2
  choose scale "$default" 0.01 1
  step "Frames: $start to $end, every ${frame_step}. (about $(( (end - start) / 60 )) s of a 60 Hz export)
Neutral pose: from ${npose_start} s for ${npose_duration} s. (check this window in Xsens playback)"
  if ask_yes 'Change frames, neutral-pose window or offsets file?' n; then
    while true; do
      ask start 'First frame (inclusive; 1 if frame 0 is a reference pose)' "$start"
      ask end 'End frame (exclusive)' "$end"
      [[ $start =~ ^[0-9]+$ && $end =~ ^[0-9]+$ ]] && (( end > start )) && break
      printf '  Use whole numbers, with the end frame after the first frame.\n'
    done
    while true; do
      ask frame_step 'Frame step (1 for 60 Hz, 4 for 240 Hz)' "$frame_step"
      [[ $frame_step =~ ^[1-9][0-9]*$ ]] && break
      printf '  Use a positive whole number.\n'
    done
    while true; do
      ask npose_start 'Neutral pose starts at (seconds)' "$npose_start"
      ask npose_duration 'Neutral pose lasts (seconds)' "$npose_duration"
      number_between "$npose_start" 0 100000 && number_between "$npose_duration" 0.01 100000 && break
      printf '  Use positive numbers of seconds.\n'
    done
    ask offsets 'Reviewed offsets JSON (Enter to skip)' "${offsets:-none}"
    [[ $offsets != none ]] || offsets=''
    [[ -z $offsets ]] || offsets=$(absolute "$offsets")
  fi
else
  step 'Network. On Windows, run ipconfig and read the IPv4 of the adapter on the Xsens router.'
  while true; do
    ask windows_ip 'Windows Xsens IP (Enter to skip until you know it)' "${windows_ip:-skip}"
    [[ $windows_ip != skip ]] || { windows_ip=''; break; }
    ipv4 "$windows_ip" && break
    (( ! defaults )) || die "invalid Windows IP: $windows_ip"
    printf '  Enter an IPv4 address such as 192.168.1.246.\n'
  done
  detected=''
  if [[ -n $windows_ip ]] && command -v ip >/dev/null 2>&1; then
    detected=$(ip -4 route get "$windows_ip" 2>/dev/null | sed -n 's/.* src \([0-9.]*\).*/\1/p' | head -n 1) || true
  fi
  if (( ! defaults )) && command -v ip >/dev/null 2>&1; then
    printf 'This PC'"'"'s addresses:\n'
    ip -br -4 address | awk '$1 != "lo" { print "  " $1 "  " $3 }' || true
  fi
  while true; do
    ask linux_ip 'Linux Xsens IP (enter it in Xsens Network Streamer)' "${detected:-${linux_ip:-skip}}"
    [[ $linux_ip != skip ]] || { linux_ip=''; break; }
    ipv4 "$linux_ip" && [[ $linux_ip != "$windows_ip" ]] && break
    (( ! defaults )) || die "invalid Linux IP: $linux_ip"
    printf '  Enter this PC'"'"'s IPv4 on the Xsens network (not the Windows one).\n'
  done
fi

if [[ $robot == booster_t1_29dof ]]; then
  ik_config=$repo_root/retarget_configs/bvh_xsens_to_booster_t1_29dof.json
else
  ik_config=$mocap_ws/GMR/general_motion_retargeting/ik_configs/bvh_xsens_to_g1.json
fi

mkdir -p -- "$session_dir"
env_file=$session_dir/session.env
{
  printf '# Mocap session created %s by tools/setup/new_session.sh\n' "$(date '+%Y-%m-%d %H:%M')"
  printf '# Load in each new terminal from the checkout: source session.env\n'
  printf '# Rerun tools/setup/new_session.sh to change values.\n'
  printf 'export %s=%q\n' \
    SESSION_ROUTE "$route" SESSION_DIR "$session_dir" ROBOT "$robot" HUMAN_HEIGHT_M "$height" \
    MOCAP_WS "$mocap_ws" IK_CONFIG "$ik_config" \
    MOTION_BVH "$motion" NPOSE_BVH "$npose" OFFSETS_FILE "$offsets" BVH_SCALE "$scale" \
    START_FRAME "$start" END_FRAME "$end" FRAME_STEP "$frame_step" \
    NPOSE_START_S "$npose_start" NPOSE_DURATION_S "$npose_duration" \
    WINDOWS_XSENS_IP "$windows_ip" LINUX_XSENS_IP "$linux_ip" \
    XSENS_SOURCE_IP "$windows_ip" XSENS_UDP_PORT 5555
} > "$env_file.tmp"
mv -- "$env_file.tmp" "$env_file"
if [[ -e $current_link && ! -L $current_link ]]; then
  die "$current_link exists and is not a link; move it aside and rerun"
fi
ln -sfn -- "$env_file" "$current_link"

printf '\nSaved %s\n' "$env_file"
printf '  Robot %s · height %s m' "$robot" "$height"
if [[ $route == recorded ]]; then
  printf ' · motion %s\n' "$(tilde "$motion")"
  [[ -f $motion ]] || printf '  Note: the motion BVH is not there yet; copy it before retargeting.\n'
else
  printf ' · Windows %s -> Linux %s, UDP 5555\n' "${windows_ip:-not set}" "${linux_ip:-not set}"
  [[ -n $windows_ip && -n $linux_ip ]] ||
    printf '  Note: rerun this script once both IP addresses are known.\n'
fi
printf '\nLoad it in every new terminal (from %s):\n  source session.env\n' "$(tilde "$repo_root")"
