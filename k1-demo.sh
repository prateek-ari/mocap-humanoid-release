#!/usr/bin/env bash
# K1 live demo: open the Setup console on its live-demo page (step 13 · Real K1), where the
# robot copies the performer in four steps. Run it in this folder:  bash k1-demo.sh
# Rehearsal without a robot (practice robot):  bash k1-demo.sh --practice  (console not running yet)
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if [[ ${1:-} == --practice ]]; then
  # Rehearsal: the practice robot instead of the real K1 (only when this starts the console).
  export MOCAP_K1_TRANSPORT=fake
fi
port=${MOCAP_SETUP_PORT:-8784}
url="http://127.0.0.1:$port/#setup/k1"
bash tools/setup/open_console.sh --no-browser
cat <<TEXT

K1 live demo: $url

On the page, top to bottom:
  1  Start the suit stream   (Xsens streaming to this laptop; the 3D view opens)
  2  Calibrate the performer (stand straight, arms down, look ahead, hold still)
  3  Check the robot         (robot on the hoist, standing, walking mode; nothing moves)
  4  Start the robot         (performer holds still until the page says FOLLOWING)
STOP ROBOT on the page stops it. The real stop is always the remote: LT + BACK.
Do not run Isaac Sim or other ROS 2 programs on this laptop at the same time.
TEXT
xdg-open "$url" >/dev/null 2>&1 || printf 'Open %s in your browser.\n' "$url"
