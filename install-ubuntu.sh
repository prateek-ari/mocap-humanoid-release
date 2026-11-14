#!/usr/bin/env bash
# One command for a new Ubuntu PC: check what the live simulation needs,
# install only what is missing, then set up this project. Safe to run again.
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage: bash install-ubuntu.sh [--check] [--with-ros2] [--with-k1 | --no-k1] [--no-console]

Checks this Ubuntu 22.04/24.04 PC and installs only what is missing:
  1. Helper packages (curl, gnupg, pciutils, ubuntu-drivers-common)
  2. NVIDIA driver (then asks you to restart and run this script again)
  3. Docker Engine, and Docker access for your user
  4. NVIDIA Container Toolkit, registered with Docker
  5. This project: tools/setup/install.sh live (Python environment, live image)
  6. Optional (it asks; default yes): the recorded route and the Booster K1 test
     tools (GMR runtime in ~/mocap-runtime, Booster SDK, Tk)
Then it adds "Mocap Setup" (and "K1 Hardware Tests") to the applications menu
and opens the Setup console.

  --check       Only report what is installed or missing; change nothing
  --with-ros2   Also build the optional ROS 2 image
  --with-k1     Install step 6 without asking
  --no-k1       Skip step 6 without asking
  --no-console  Do not open the Setup console at the end
  -h, --help    Show this help

Run it as your normal user from a terminal on this PC's desktop (not over SSH).
It asks for your password when it needs administrator rights.
EOF
}

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
user=${USER:-$(id -un)}
check_only=0 with_ros2=0 open_console=1 with_k1=ask
while (($#)); do
  case $1 in
    --check) check_only=1 ;;
    --with-ros2) with_ros2=1 ;;
    --with-k1) with_k1=yes ;;
    --no-k1) with_k1=no ;;
    --no-console) open_console=0 ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; printf 'ERROR: unknown option: %s\n' "$1" >&2; exit 2 ;;
  esac
  shift
done

stage='start'
trap 'printf "\nFAILED during: %s\nFix the problem shown above, then run again: bash install-ubuntu.sh\nFinished parts are skipped.\n" "$stage" >&2' ERR

die() { printf '\nERROR: %s\n' "$*" >&2; exit 2; }
step() { stage=$*; printf '\n== %s\n' "$*"; }
ok() { printf '  [ok]       %s\n' "$*"; }
missing_count=0
missing() { printf '  [missing]  %s\n' "$*"; missing_count=$((missing_count + 1)); }
doing() { printf '  [install]  %s\n' "$*"; }
optional() { printf '  [optional] %s\n' "$*"; }

ask_yes() {
  local reply
  [[ -t 0 ]] || return 1
  read -r -p "$1 [Y/n]: " reply || return 1
  [[ -z $reply || $reply == [Yy]* ]]
}

# --- Checks (no changes, no sudo) -------------------------------------------

os_codename=''
ubuntu_supported() {
  local key value os_id='' os_version=''
  [[ -r /etc/os-release ]] || return 1
  while IFS='=' read -r key value; do
    value=${value//\"/}
    case $key in
      ID) os_id=$value ;;
      VERSION_ID) os_version=$value ;;
      UBUNTU_CODENAME) os_codename=$value ;;
      VERSION_CODENAME) [[ -n $os_codename ]] || os_codename=$value ;;
    esac
  done < /etc/os-release
  [[ $os_id == ubuntu && ( $os_version == 22.04 || $os_version == 24.04 ) ]]
}

pkg_installed() { dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q 'install ok installed'; }
nvidia_gpu_present() { lspci 2>/dev/null | grep -iE 'vga|3d|display' | grep -qi nvidia; }
driver_working() { nvidia-smi -L >/dev/null 2>&1; }
driver_package_installed() {
  dpkg-query -W -f='${Package} ${Status}\n' 'nvidia-driver-*' 2>/dev/null | grep -q 'install ok installed'
}
docker_installed() { command -v docker >/dev/null 2>&1; }
docker_from_snap() { [[ $(command -v docker 2>/dev/null) == /snap/* ]]; }
docker_running() { systemctl is-active --quiet docker 2>/dev/null; }
# Group membership as saved (takes effect in new logins), not of this shell.
user_in_docker_group() { id -nG "$user" 2>/dev/null | tr ' ' '\n' | grep -qx docker; }
docker_usable_now() { docker info >/dev/null 2>&1; }
toolkit_installed() { command -v nvidia-ctk >/dev/null 2>&1; }
# Docker as this user if possible, else through sudo (never prompts in --check).
docker_any() {
  if docker_usable_now; then docker "$@"
  elif (( check_only )); then sudo -n docker "$@" 2>/dev/null
  else sudo docker "$@"
  fi
}
nvidia_runtime_registered() {
  local runtimes
  if runtimes=$(docker_any info --format '{{range $k, $v := .Runtimes}}{{$k}} {{end}}'); then
    [[ " $runtimes " == *' nvidia '* ]]
  else
    # Daemon unreachable from here: fall back to the saved configuration.
    grep -qs '"nvidia"' /etc/docker/daemon.json
  fi
}

# Run a command with Docker access even if the docker group was added in
# this session (new group membership only applies after logging in again).
with_docker_group() {
  if docker_usable_now; then
    "$@"
  else
    sg docker -c "$(printf '%q ' "$@")"
  fi
}

check_report() {
  step 'Checking this PC'
  if ubuntu_supported; then ok "Ubuntu ${os_codename}"; else missing 'Ubuntu 22.04 or 24.04 (this script supports only these)'; fi
  if driver_working; then
    ok "NVIDIA driver: $(nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>/dev/null | head -n 1)"
  elif driver_package_installed; then
    missing 'NVIDIA driver is installed but not running (restart the PC)'
  elif nvidia_gpu_present; then
    missing 'NVIDIA driver'
  else
    missing 'NVIDIA graphics card (none found, or pciutils is missing)'
  fi
  if docker_from_snap; then missing 'Docker Engine from Docker apt repository (found the Snap version)'
  elif docker_installed; then ok "Docker: $(docker --version 2>/dev/null)"
  else missing 'Docker Engine'; fi
  if docker_installed && ! docker_running; then missing 'Docker service running'; fi
  if user_in_docker_group; then ok "Docker access for $user"; else missing "Docker access for $user (docker group)"; fi
  if toolkit_installed; then ok 'NVIDIA Container Toolkit'; else missing 'NVIDIA Container Toolkit'; fi
  if docker_installed && toolkit_installed; then
    if nvidia_runtime_registered; then ok 'NVIDIA runtime registered with Docker'; else missing 'NVIDIA runtime registered with Docker'; fi
  fi
  if [[ -x $repo_root/.venv/bin/python ]]; then ok 'Project Python environment (.venv)'; else missing 'Project Python environment (.venv)'; fi
  if docker_usable_now && [[ -n $(docker image ls -q gmr-live 2>/dev/null) ]]; then
    ok 'Live simulation image (gmr-live)'
  else
    missing 'Live simulation image (gmr-live)'
  fi
  if [[ -n ${DISPLAY:-} ]]; then ok "Desktop display: $DISPLAY"; else missing 'Desktop display (run from a terminal on the PC desktop, not SSH)'; fi
  # Step 6 is optional: report it, but do not count it as missing.
  local runtime=$HOME/mocap-runtime venv_python=$repo_root/.venv/bin/python
  if [[ -x $runtime/venv/bin/python && -f $runtime/GMR/assets/booster_k1/K1_serial.xml ]]; then
    ok "Recorded route runtime with the K1 model ($runtime)"
  else
    optional "Recorded route runtime with the K1 model ($runtime)"
  fi
  if [[ -x $venv_python ]] && "$venv_python" -c 'import booster_robotics_sdk_python' >/dev/null 2>&1; then
    ok 'Booster K1 SDK (K1 test window)'
  else
    optional 'Booster K1 SDK (K1 test window)'
  fi
  if [[ -x $venv_python ]] && "$venv_python" -c 'import tkinter' >/dev/null 2>&1; then
    ok 'Tk (K1 test window)'
  else
    optional 'Tk (K1 test window)'
  fi
}

if (( check_only )); then
  check_report
  if (( missing_count )); then
    printf '\n%d item(s) missing. Run without --check to install them: bash install-ubuntu.sh\n' "$missing_count"
    exit 1
  fi
  printf '\nEverything is installed.\n'
  exit 0
fi

# --- Install -----------------------------------------------------------------

(( EUID != 0 )) || die 'run as your normal user, not with sudo; it asks for your password when needed'
ubuntu_supported || die 'this script supports Ubuntu 22.04 and 24.04 only'
command -v sudo >/dev/null 2>&1 || die 'sudo is missing'
printf 'Mocap Humanoid setup for Ubuntu. Some steps need administrator rights.\n'
sudo -v

apt_updated=0
apt_install() {
  if (( ! apt_updated )); then sudo apt-get update; apt_updated=1; fi
  sudo apt-get install -y "$@"
}

step '1/6 Helper packages'
helpers=(ca-certificates curl gnupg pciutils ubuntu-drivers-common)
need=()
for package in "${helpers[@]}"; do pkg_installed "$package" || need+=("$package"); done
if ((${#need[@]})); then doing "${need[*]}"; apt_install "${need[@]}"; fi
ok 'helper packages'

step '2/6 NVIDIA driver'
offer_restart() {
  printf '\nRestart the PC, then run this script again:  bash install-ubuntu.sh\n'
  if ask_yes 'Restart now?'; then sudo reboot; fi
  exit 3
}
if driver_working; then
  ok "$(nvidia-smi --query-gpu=name,driver_version --format=csv,noheader | head -n 1)"
elif ! nvidia_gpu_present; then
  die 'no NVIDIA graphics card found. The live simulation on Ubuntu needs one.'
elif driver_package_installed; then
  printf '  The NVIDIA driver is installed but not running yet.\n'
  offer_restart
else
  doing 'recommended NVIDIA driver (ubuntu-drivers)'
  printf '  If a screen asks about Secure Boot, choose a password and enter it again\n'
  printf '  after the restart (choose "Enroll MOK").\n'
  sudo ubuntu-drivers install || sudo ubuntu-drivers autoinstall
  offer_restart
fi

step '3/6 Docker'
if docker_from_snap; then
  die 'Docker is installed as a Snap, which this setup does not support. Remove it with: sudo snap remove docker, then run this script again.'
fi
if ! docker_installed; then
  doing 'Docker Engine (from download.docker.com)'
  sudo install -m 0755 -d /etc/apt/keyrings
  sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
  sudo chmod a+r /etc/apt/keyrings/docker.asc
  printf 'deb [arch=%s signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu %s stable\n' \
    "$(dpkg --print-architecture)" "$os_codename" | sudo tee /etc/apt/sources.list.d/docker.list >/dev/null
  apt_updated=0
  apt_install docker-ce docker-ce-cli containerd.io docker-buildx-plugin
fi
docker_running || sudo systemctl enable --now docker
ok "$(docker --version)"
relogin=0
if ! user_in_docker_group; then
  doing "Docker access for $user"
  sudo usermod -aG docker "$user"
  relogin=1
fi
ok "Docker access for $user"

step '4/6 NVIDIA Container Toolkit'
if ! toolkit_installed; then
  doing 'NVIDIA Container Toolkit (from nvidia.github.io)'
  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey |
    sudo gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
  curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list |
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' |
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list >/dev/null
  apt_updated=0
  apt_install nvidia-container-toolkit
fi
if ! nvidia_runtime_registered; then
  doing 'register the NVIDIA runtime with Docker'
  sudo nvidia-ctk runtime configure --runtime=docker
  sudo systemctl restart docker
fi
nvidia_runtime_registered || die 'Docker does not list the nvidia runtime after configuration'
ok 'NVIDIA runtime registered with Docker'

step '5/6 Project (Python environment and live simulation image)'
[[ -n ${DISPLAY:-} ]] ||
  die 'no desktop display. Steps 1-4 are done: open a terminal on the PC desktop (not SSH) and run this script again.'
install_args=(live --system-packages)
(( ! with_ros2 )) || install_args+=(--with-ros2)
with_docker_group bash "$repo_root/tools/setup/install.sh" "${install_args[@]}"

step '6/6 Recorded route and K1 test tools (optional)'
if [[ $with_k1 == ask ]]; then
  if ask_yes 'Also install the recorded route and the Booster K1 test tools?'; then with_k1=yes; else with_k1=no; fi
fi
if [[ $with_k1 == yes ]]; then
  bash "$repo_root/tools/setup/install.sh" recorded --with-booster --system-packages \
    --workspace "$HOME/mocap-runtime" --robot booster_k1
else
  printf '  Skipped. Install them later from the Setup console (step 13 · Real K1).\n'
fi
bash "$repo_root/tools/setup/open_console.sh" --install-shortcut ||
  printf 'Could not add the menu shortcut; open the console with: bash tools/setup/open_console.sh\n'

stage='finish'
printf '\nPASS: this PC is ready for the live simulation.\n'
if (( relogin )); then
  printf 'Log out and log in once, so Docker also works in new terminals and in Mocap Setup.\n'
fi
printf 'Next: open "Mocap Setup" from the applications menu and follow it from step 03.\n'
if (( open_console )); then
  with_docker_group bash "$repo_root/tools/setup/open_console.sh"
fi
