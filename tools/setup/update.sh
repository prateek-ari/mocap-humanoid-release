#!/usr/bin/env bash
# Mocap Update: pull the newest version of this checkout and run its installer in this
# terminal. It never runs while the Setup console, a live simulation or the K1 test window
# runs, and none of them can start while it runs (docs/k1-live-ui-plan-2026-11-16.md, C3).
set -uo pipefail

# Everything runs inside main: the pull may replace this file while it runs.
main() {
  script_dir=$(cd -- "${BASH_SOURCE[0]%/*}" && pwd -P)
  repo_root=$(cd -- "$script_dir/../.." && pwd -P)
  state_dir=${XDG_STATE_HOME:-$HOME/.local/state}/mocap-g1
  result=$state_dir/update-last.txt
  again="Run Mocap Update again, or in a terminal: bash $repo_root/tools/setup/update.sh"
  mkdir -p -- "$state_dir"

  # Lock order everywhere: update.lock, then k1-tests.lock. The console, the live
  # simulation and the K1 window hold update.lock shared while they run.
  exec {update_fd}>>"$state_dir/update.lock"
  flock -x -n "$update_fd" || refuse \
    'Mocap Update did not start: one of our programs is running.' \
    'Close it, then run Mocap Update again:' \
    '  - Setup console (and a live simulation it runs): the Shut down button in the console,' \
    '    or right-click Mocap Setup in the applications menu > Stop the Setup console' \
    '  - a live simulation started in a terminal: press Ctrl+C there' \
    '  - K1 Hardware Tests window: close it'
  exec {k1_fd}>>"$state_dir/k1-tests.lock"
  flock -x -n "$k1_fd" || refuse \
    'Mocap Update did not start: an install is running, or the K1 Hardware Tests window is open.' \
    'Wait for the install to finish or close the window, then run Mocap Update again.'

  old=$(git -C "$repo_root" rev-parse --short HEAD 2>/dev/null) || old=unknown
  new=$old
  printf 'Mocap Update: %s, now at %s\n' "$repo_root" "$old"

  branch=$(git -C "$repo_root" symbolic-ref -q --short HEAD) ||
    fail 'Step: check the branch. The checkout is not on a branch. Nothing was changed.' \
      'Ask a developer to put it back on the release branch.' "$again"
  upstream=$(git -C "$repo_root" rev-parse -q --abbrev-ref --symbolic-full-name '@{u}' 2>/dev/null) ||
    fail "Step: check the branch. Branch $branch has no upstream to update from. Nothing was changed." \
      'Ask a developer to set it up.' "$again"
  remote=${upstream%%/*}
  release=$(git -C "$repo_root" symbolic-ref -q --short "refs/remotes/$remote/HEAD") || release=$upstream
  [[ $upstream == "$release" ]] ||
    fail "Step: check the branch. The checkout follows $upstream, not the release branch $release. Nothing was changed." \
      "Switch back: git -C $repo_root switch ${release#"$remote"/}" "$again"
  dirty=$(git -C "$repo_root" status --porcelain --untracked-files=no) ||
    fail 'Step: check for local changes. git status failed. Nothing was changed.' "$again"
  [[ -z $dirty ]] ||
    fail 'Step: check for local changes. The checkout has changed files, so nothing was changed:' \
      "$dirty" "Show them: git -C $repo_root status. Ask a developer before you undo them." "$again"

  printf 'Downloading from %s ...\n' "$upstream"
  git -C "$repo_root" fetch --quiet "$remote" ||
    fail "Step: download. Could not reach $remote. Nothing was changed." \
      'Check the network connection.' "$again"
  git -C "$repo_root" merge-base --is-ancestor HEAD "$upstream" ||
    fail "Step: check the download. This checkout has commits that $upstream does not have, so it cannot update by fast-forward. Nothing was changed." \
      'Ask a developer.' "$again"
  git -C "$repo_root" pull --ff-only ||
    fail 'Step: git pull failed (see above).' "$again"
  new=$(git -C "$repo_root" rev-parse --short HEAD)

  if [[ -f $repo_root/install-ubuntu.sh ]]; then
    installer=(bash install-ubuntu.sh --no-console)
    if "$repo_root/.venv/bin/python" -c 'import booster_robotics_sdk_python' >/dev/null 2>&1; then
      installer+=(--with-k1)
    else
      installer+=(--no-k1)
    fi
  else
    installer=(bash tools/setup/install.sh live --system-packages)
  fi
  printf '\nNow at %s. Running the installer: %s\n' "$new" "${installer[*]}"
  # The installer takes k1-tests.lock through this open file (MOCAP_K1_LOCK_FD). It does not
  # get update.lock: a program it leaves running must not keep every start refused.
  (cd -- "$repo_root" && MOCAP_K1_LOCK_FD=$k1_fd "${installer[@]}" {update_fd}>&-) ||
    fail "Step: installer. The new version $new is downloaded but not installed." "$again"

  if [[ $old == "$new" ]]; then
    record PASS "Already at the newest version $new; the installer ran again."
  else
    record PASS "Updated from $old to $new and installed."
  fi
  printf '\nUPDATE PASSED: %s -> %s\n' "$old" "$new"
  exec {k1_fd}>&- {update_fd}>&-  # release both: the console must be able to start now
  if [[ -t 0 ]]; then
    read -r -p 'Open the Setup console now? [Y/n] ' reply || reply=n
    if [[ ${reply:-y} == [Yy]* ]]; then
      exec bash "$repo_root/tools/setup/open_console.sh"
    fi
  fi
}

# A refusal changes nothing, not even the last result.
refuse() {
  printf '\n' >&2
  printf '%s\n' "$@" 'Nothing was changed.' >&2
  pause
  exit 2
}

record() {  # status, then free-text lines; first line is the interface the console reads
  local status=$1
  shift
  { printf '%s %s %s -> %s\n' "$(date -Iseconds)" "$status" "$old" "$new"; printf '%s\n' "$@"; } \
    > "$result.tmp" && mv -f -- "$result.tmp" "$result"
}

fail() {
  record FAIL "$@"
  printf '\nUPDATE FAILED: %s -> %s\n' "$old" "$new" >&2
  printf '%s\n' "$@" >&2
  printf '(Saved in %s)\n' "$result" >&2
  pause
  exit 1
}

pause() {
  if [[ -t 0 ]]; then read -r -p 'Press Enter to close this window. ' _ || true; fi
}

main "$@"
exit
