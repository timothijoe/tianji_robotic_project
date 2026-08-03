#!/bin/sh
set -eu
SCRIPT_DIR="$(CDPATH= cd "$(dirname "$0")" && pwd -P)"
PYTHON="/home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python"
if [ "${1:-}" = "--dry-run" ]; then
  shift
  printf '%s %s/replay_left_hand_mcap_sdk.py' "$PYTHON" "$SCRIPT_DIR"
  for argument in "$@"; do printf ' %s' "$argument"; done
  printf '\n'
  exit 0
fi
exec "$PYTHON" "$SCRIPT_DIR/replay_left_hand_mcap_sdk.py" "$@"
