#!/bin/sh
# Launch the ROS Python replay program while retaining the caller's ROS paths.
set -eu

SCRIPT_DIR="$(CDPATH= cd "$(dirname "$0")" && pwd -P)"
MCAP_SITE="/home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/lib/python3.12/site-packages"

if [ "${1:-}" = "--dry-run" ]; then
  shift
  printf 'PYTHONPATH=%s:%s python3 %s/replay_left_hand_mcap.py' "$MCAP_SITE" "${PYTHONPATH:-}" "$SCRIPT_DIR"
  for argument in "$@"; do
    printf ' %s' "$argument"
  done
  printf '\n'
  exit 0
fi

export PYTHONPATH="$MCAP_SITE${PYTHONPATH:+:$PYTHONPATH}"
exec python3 "$SCRIPT_DIR/replay_left_hand_mcap.py" "$@"
