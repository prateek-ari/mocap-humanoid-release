#!/usr/bin/env bash
# Install a selected simulation workflow without changing the caller's shell.
set -Eeuo pipefail
stage=validation
trap 'printf "FAIL: %s stage stopped; fix the error and rerun the same command. Completed stages are reusable.\n" "$stage" >&2' ERR

usage() {
  cat <<'EOF'
Usage: bash tools/setup/install.sh                      (step-by-step questions)
       bash tools/setup/install.sh <core|recorded|live> [options]

Options:
  --workspace ABS_PATH  External GMR runtime workspace (recorded; default: ~/mocap-runtime)
  --robot ROBOT         unitree_g1 (default) or booster_t1_29dof (recorded only)
  --python COMMAND      Python >=3.10 to create the project venv (default: python3)
  --with-ros2           Build xsens-ros2:jazzy too (live only)
  --system-packages     Install missing Ubuntu 22.04/24.04 apt prerequisites (uses sudo)
  --dry-run             Print commands without executing or changing anything
  -h, --help            Show this help

The project venv is .venv in the checkout. Recorded GMR uses a separate venv
inside --workspace, as managed by bootstrap_g1_workspace.py.
EOF
}

die() { printf 'ERROR: %s\n' "$*" >&2; exit 2; }

# Bash %q gives commands that can be copied into a Bash shell, including paths
# with spaces. Keep the printed command and executed argv in one function.
run() {
  local arg
  printf '+'
  for arg in "$@"; do printf ' %q' "$arg"; done
  printf '\n'
  if (( ! dry_run )); then "$@"; fi
}

# Lexically normalize an absolute path using Bash builtins. In dry-run this
# deliberately avoids launching even a read-only helper process.
normalize_path() {
  local input=$1 part result=() joined
  local -a parts
  [[ $input == /* ]] || die "--workspace must be an absolute path"
  IFS='/' read -r -a parts <<< "$input"
  for part in "${parts[@]}"; do
    case $part in
      ''|.) ;;
      ..) if ((${#result[@]})); then unset 'result[${#result[@]}-1]'; fi ;;
      *) result+=("$part") ;;
    esac
  done
  joined=''
  for part in "${result[@]}"; do joined+="/$part"; done
  printf '%s\n' "${joined:-/}"
}

script_dir=$(cd -- "${BASH_SOURCE[0]%/*}" && pwd -P)
repo_root=$(cd -- "$script_dir/../.." && pwd -P)
profile=${1:-}
if [[ $profile == -h || $profile == --help ]]; then usage; exit 0; fi
wizard=0
if [[ $profile == core || $profile == recorded || $profile == live ]]; then
  shift
elif [[ ( -z $profile || $profile == --* ) && -t 0 && -t 1 ]]; then
  # No profile in a terminal: ask step by step (options such as --dry-run still apply).
  profile='' wizard=1
else
  usage >&2; die "choose core, recorded, or live"
fi

workspace=''
robot=unitree_g1
python=python3
with_ros2=0
system_packages=0
dry_run=0
robot_given=0
while (($#)); do
  case $1 in
    --workspace|--robot|--python)
      option=$1
      (($# >= 2)) || die "$option needs a value"
      [[ -n $2 && $2 != --* ]] || die "$option needs a value"
      case $option in
        --workspace) workspace=$2 ;;
        --robot) robot=$2; robot_given=1 ;;
        --python) python=$2 ;;
      esac
      shift 2 ;;
    --with-ros2) with_ros2=1; shift ;;
    --system-packages) system_packages=1; shift ;;
    --dry-run) dry_run=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown option: $1" ;;
  esac
done

# Interactive setup: every question has a standard default; Enter accepts it.
ask() {
  local reply
  read -r -p "$1 [$2]: " reply || die "no answer; setup cancelled"
  REPLY=${reply:-$2}
}
ask_yes() {
  local reply hint=y/N
  [[ $2 == y ]] && hint=Y/n
  while true; do
    read -r -p "$1 [$hint]: " reply || die "no answer; setup cancelled"
    case ${reply:-$2} in
      [Yy]|[Yy]es) return 0 ;;
      [Nn]|[Nn]o) return 1 ;;
    esac
  done
}
ubuntu_supported() {
  local key value os_id='' os_version=''
  [[ -r /etc/os-release ]] || return 1
  while IFS='=' read -r key value; do
    case $key in
      ID) os_id=${value//\"/} ;;
      VERSION_ID) os_version=${value//\"/} ;;
    esac
  done < /etc/os-release
  [[ $os_id == ubuntu && ( $os_version == 22.04 || $os_version == 24.04 ) ]]
}

if (( wizard )); then
  printf '\nMocap humanoid setup. Press Enter to accept the [default] answer.\n\n'
  printf 'Step 1 · What do you want to set up?\n'
  printf '  1) Live Xsens suit -> G1 or Booster T1 simulation (needs Docker + NVIDIA)\n'
  printf '  2) Recorded BVH export -> G1 or Booster T1 simulation\n'
  printf '  3) Project tools only (console and checks)\n'
  while true; do
    ask 'Choice' 1
    case $REPLY in
      1) profile=live; break ;;
      2) profile=recorded; break ;;
      3) profile=core; break ;;
    esac
  done
  if [[ $profile == recorded ]]; then
    printf '\nStep 2 · Which robot?\n  1) Unitree G1\n  2) Booster T1\n'
    while true; do
      ask 'Choice' 1
      case $REPLY in
        1) robot=unitree_g1; break ;;
        2) robot=booster_t1_29dof; robot_given=1; break ;;
      esac
    done
    printf '\nStep 3 · GMR runtime folder (large download, kept outside this checkout)\n'
    ask 'Folder' "${HOME:-}/mocap-runtime"
    workspace=$REPLY
  elif [[ $profile == live ]]; then
    if ! command -v docker >/dev/null 2>&1 || ! command -v nvidia-smi >/dev/null 2>&1; then
      printf '\nDocker or the NVIDIA driver is missing. Install them first (guide step 02),\n'
      printf 'then rerun this setup.\n'
      ask_yes 'Continue anyway?' n || exit 0
    fi
    printf '\nStep 2 · The live image runs both robots. ROS 2 topics are optional.\n'
    if ask_yes 'Also build the ROS 2 image?' n; then with_ros2=1; fi
  fi
  printf '\nLast step · '
  if ubuntu_supported; then
    if ask_yes 'Install missing Ubuntu packages with sudo apt?' y; then system_packages=1; fi
  else
    printf 'Not Ubuntu 22.04/24.04: install git, python3-venv and build tools yourself.\n'
  fi
  printf '\nAbout to run: bash tools/setup/install.sh %s' "$profile"
  [[ $profile != recorded ]] || printf ' --workspace %q --robot %s' "$workspace" "$robot"
  (( ! with_ros2 )) || printf ' --with-ros2'
  (( ! system_packages )) || printf ' --system-packages'
  (( ! dry_run )) || printf ' --dry-run'
  printf '\n'
  ask_yes 'Start?' y || exit 0
fi

[[ $robot == unitree_g1 || $robot == booster_t1_29dof ]] || die "unsupported robot: $robot"
if [[ $profile == recorded ]]; then
  if [[ -z $workspace ]]; then
    [[ -n ${HOME:-} ]] || die "recorded needs --workspace /absolute/external/path"
    workspace=$HOME/mocap-runtime
    printf 'Using default GMR runtime folder: %s\n' "$workspace"
  fi
else
  [[ -z $workspace ]] || die "--workspace is valid only for recorded"
  (( ! robot_given )) || die "--robot is valid only for recorded"
fi
if (( with_ros2 )) && [[ $profile != live ]]; then
  die "--with-ros2 is valid only for live"
fi

if [[ $profile == recorded ]]; then
  workspace=$(normalize_path "$workspace")
  # Reject symlinked path components without launching realpath. This prevents
  # a path that looks external from resolving into the source checkout.
  candidate=''
  IFS='/' read -r -a workspace_parts <<< "$workspace"
  for part in "${workspace_parts[@]}"; do
    [[ -n $part ]] || continue
    candidate+="/$part"
    [[ ! -L $candidate ]] || die "--workspace contains a symlink: $candidate"
  done
  [[ $workspace != / && $workspace != "$repo_root" && $workspace != "$repo_root"/* ]] ||
    die "runtime workspace must be outside this source checkout"
  [[ $repo_root != "$workspace"/* ]] || die "workspace must not contain the source checkout"
  IFS='/' read -r -a workspace_parts <<< "$workspace"
  ((${#workspace_parts[@]} >= 4)) || die "refusing broad workspace path: $workspace"
fi
if (( EUID == 0 && ! dry_run )); then
  die "run setup as a normal user; --system-packages uses sudo only for apt"
fi

if (( system_packages )); then
  # Read OS metadata with Bash builtins so dry-run launches no commands.
  ubuntu_supported || die "--system-packages supports Ubuntu 22.04/24.04 only"
fi

if (( system_packages )); then
  stage='Ubuntu packages'
  packages=(git python3 python3-venv python3-pip build-essential ripgrep iproute2)
  missing=("${packages[@]}")
  if (( ! dry_run )); then
    # Skip apt (and its sudo prompt) when everything is already installed.
    missing=()
    for package in "${packages[@]}"; do
      dpkg-query -W -f='${Status}' "$package" 2>/dev/null | grep -q 'install ok installed' ||
        missing+=("$package")
    done
  fi
  if ((${#missing[@]})); then
    run sudo apt-get update
    run sudo apt-get install -y "${missing[@]}"
  else
    printf 'Ubuntu packages already installed.\n'
  fi
fi

if (( ! dry_run )); then
  stage='project revision'
  revision=''
  if [[ -e $repo_root/.git ]]; then
    revision=$(git -C "$repo_root" rev-parse --short HEAD)
  elif [[ -r $repo_root/.git_archival.txt ]]; then
    # A GitHub "Download ZIP" has no .git; its .git_archival.txt names the commit.
    revision=$(sed -n 's/^node: \([0-9a-f]\{7\}\).*/\1 (ZIP download)/p' "$repo_root/.git_archival.txt")
  fi
  printf 'Project revision: %s\n' "${revision:-unknown (no Git checkout)}"
else
  printf 'Dry run: no commands will execute. Project revision command: git -C %q rev-parse --short HEAD\n' "$repo_root"
fi

if (( ! dry_run )); then
  stage='Python selection'
  command -v "$python" >/dev/null 2>&1 || die "Python command unavailable: $python"
  "$python" -c 'import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)' ||
    die "--python must select Python >=3.10"
fi

stage='project environment'
venv_python="$repo_root/.venv/bin/python"
if [[ ! -x $venv_python ]]; then
  run "$python" -m venv "$repo_root/.venv"
fi
if (( ! dry_run )); then
  [[ -x $venv_python ]] || die "project venv Python is unavailable: $venv_python"
  "$venv_python" -c 'import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)' ||
    die "existing project venv uses Python <3.10; move .venv aside and rerun setup"
fi
run "$venv_python" -m pip install -e "${repo_root}[runtime,operator]"
run "$venv_python" "$repo_root/tools/checks/setup_doctor.py" core --with-operator
run "$venv_python" "$repo_root/tools/xsens/stream_acceptance.py" --self-test --require-joint-angles

if [[ $profile == recorded ]]; then
  stage='recorded runtime'
  run "$venv_python" "$repo_root/tools/bootstrap_g1_workspace.py" bootstrap --workspace "$workspace"
  run "$venv_python" "$repo_root/tools/bootstrap_g1_workspace.py" check --workspace "$workspace"
  run "$venv_python" "$repo_root/tools/checks/setup_doctor.py" recorded --workspace "$workspace" --robot "$robot"
fi

if [[ $profile == live ]]; then
  stage='live host preflight'
  if (( dry_run )); then
    printf '# Preflight: Docker CLI/daemon, NVIDIA driver/runtime, local X11 socket and Xauthority\n'
    run docker info --format '{{json .Runtimes}}'
    run nvidia-smi -L
  else
    command -v docker >/dev/null 2>&1 || die "Docker CLI missing; install Docker Engine first"
    command -v nvidia-smi >/dev/null 2>&1 || die "nvidia-smi missing; install a working NVIDIA driver first"
    runtimes=$(docker info --format '{{json .Runtimes}}') || die "Docker daemon is unavailable to this user"
    [[ $runtimes == *'"nvidia"'* ]] || die "NVIDIA Container Toolkit runtime is not registered with Docker"
    nvidia-smi -L || die "NVIDIA driver/GPU check failed"
    display=${DISPLAY:-:0}
    [[ $display =~ ^(unix/)?:([0-9]+)(\.[0-9]+)?$ ]] || die "a local X11 DISPLAY is required (found: $display)"
    [[ -S /tmp/.X11-unix/X${BASH_REMATCH[2]} ]] || die "local X11 socket for $display is missing"
    authority=${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}
    [[ -f $authority ]] || die "Xauthority file missing: $authority"
  fi
  stage='live GMR image'
  run docker build -t gmr-live:bb1bbe4 -f "$repo_root/docker/gmr-live/Dockerfile" "$repo_root"
  run docker run --rm --gpus all --entrypoint nvidia-smi gmr-live:bb1bbe4 -L
  if (( with_ros2 )); then
    stage='ROS 2 image'
    run docker build -t xsens-ros2:jazzy "$repo_root/docker/xsens-ros2"
    run docker run --rm xsens-ros2:jazzy ros2 pkg prefix xsens_mvn_ros2
  fi
  if [[ -n ${XSENS_SOURCE_IP:-} ]]; then
    run "$venv_python" "$repo_root/tools/checks/setup_doctor.py" live
  else
    printf 'Before live operation, set the current Windows Xsens-facing XSENS_SOURCE_IP and run:\n'
    printf '  %q %q live\n' "$venv_python" "$repo_root/tools/checks/setup_doctor.py"
  fi
  printf 'Image and GPU readiness do not verify X11 viewer operation or real suit traffic.\n'
fi

if (( dry_run )); then
  printf 'DRY RUN: NOT VERIFIED; no install or checks executed.\n'
else
  printf 'PASS: %s installation checks completed.\n' "$profile"
fi
printf 'This script cannot activate its caller shell. Activate the project environment with:\n'
printf '  source %q\n' "$repo_root/.venv/bin/activate"

if (( wizard && ! dry_run )) && [[ $profile != core ]]; then
  bash "$repo_root/tools/setup/open_console.sh" --install-shortcut ||
    printf 'Could not add the menu shortcut; open the console with: bash tools/setup/open_console.sh\n'
  printf '\nNext: everything else happens in the Setup console in your browser.\n'
  printf 'Open it any time from the applications menu: Mocap Setup.\n'
  if ask_yes 'Open the Setup console now?' y; then
    exec bash "$repo_root/tools/setup/open_console.sh"
  fi
  printf 'Prefer the terminal? Run: bash tools/setup/new_session.sh\n'
fi
