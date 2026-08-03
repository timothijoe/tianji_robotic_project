#!/bin/sh
# Convert a right Wuji Glove Studio MCAP recording to left Wuji Hand joint
# positions, then open the result in the offline MuJoCo viewer.
set -eu

SCRIPT_DIR="$(CDPATH= cd "$(dirname "$0")" && pwd -P)"
SDK_PYTHON="/home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python"
MUJOCO_PYTHON="/home/zhoutong/catkin_robotic_ws/august_ws/tianji_robotic_project/.venv/bin/python"

# Change only this line when visualizing a different Studio recording.
#DEFAULT_MCAP_PATH="/home/zhoutong/catkin_robotic_ws/august_ws/wuji-record-data/august_02/session_20260802_162909_764.mcap"
DEFAULT_MCAP_PATH="/tmp/august_02/session_20260802_174450_282.mcap"
DEFAULT_MCAP_PATH="/tmp/august_02/session_20260802_174440_936.mcap"
# DEFAULT_MCAP_PATH="/tmp/august_02/session_20260802_174450_282.mcap"
# DEFAULT_MCAP_PATH="/tmp/august_02/session_20260802_174450_282.mcap"
# DEFAULT_MCAP_PATH="/tmp/august_02/session_20260802_174450_282.mcap"
# DEFAULT_MCAP_PATH="/tmp/august_02/session_20260802_174450_282.mcap"
# DEFAULT_MCAP_PATH="/tmp/august_02/session_20260802_174450_282.mcap"
usage() {
  cat <<'EOF'
Usage: visualize_right_glove_to_left.sh [--dry-run] [/absolute/path/to/session.mcap]

Converts a right-glove recording into a mirrored left Wuji Hand trajectory,
writes the generated .npz and .mcap beside the source recording, and opens the
.npz in MuJoCo. With no MCAP argument, it uses DEFAULT_MCAP_PATH at the top of
this file. This script is offline only; it never connects to the hand.
EOF
}

dry_run=0
if [ "${1:-}" = "--dry-run" ]; then
  dry_run=1
  shift
fi

if [ "${1:-}" = "--help" ] || [ "${1:-}" = "-h" ]; then
  usage
  exit 0
fi

if [ "$#" -gt 1 ]; then
  usage >&2
  exit 2
fi

source_mcap="$(realpath "${1:-$DEFAULT_MCAP_PATH}")"
if [ ! -f "$source_mcap" ]; then
  echo "MCAP file not found: $source_mcap" >&2
  exit 2
fi
case "$source_mcap" in
  *.mcap) ;;
  *)
    echo "Expected a .mcap recording: $source_mcap" >&2
    exit 2
    ;;
esac

if [ ! -x "$SDK_PYTHON" ] || [ ! -x "$MUJOCO_PYTHON" ]; then
  echo "Required Python environment is missing; see this script's SDK_PYTHON and MUJOCO_PYTHON settings." >&2
  exit 2
fi

output_prefix="${source_mcap%.mcap}_right_to_left_wuji_hand"
trajectory_npz="${output_prefix}.npz"
left_hand_mcap="${output_prefix}.mcap"

print_command() {
  printf '  '
  printf '%s ' "$@"
  printf '\n'
}

echo "Input MCAP:       $source_mcap"
echo "Left-hand NPZ:    $trajectory_npz"
echo "Left-hand MCAP:   $left_hand_mcap"
echo
echo "1/3 Mirror right glove coordinates and retarget to left Wuji Hand:"
print_command "$SDK_PYTHON" "$SCRIPT_DIR/mcap_to_left_qpos.py" "$source_mcap" "$trajectory_npz"
echo "2/3 Export the left-hand trajectory as a standard JointState MCAP:"
print_command "$SDK_PYTHON" "$SCRIPT_DIR/left_qpos_to_mcap.py" "$trajectory_npz" "$left_hand_mcap"
echo "3/3 Open the trajectory in the offline MuJoCo viewer:"
print_command "$MUJOCO_PYTHON" "$SCRIPT_DIR/mujoco_left_replay.py" "$trajectory_npz"

if [ "$dry_run" -eq 1 ]; then
  echo
  echo "Dry run complete; no files were written and MuJoCo was not started."
  exit 0
fi

"$SDK_PYTHON" "$SCRIPT_DIR/mcap_to_left_qpos.py" "$source_mcap" "$trajectory_npz"
"$SDK_PYTHON" "$SCRIPT_DIR/left_qpos_to_mcap.py" "$trajectory_npz" "$left_hand_mcap"
exec "$MUJOCO_PYTHON" "$SCRIPT_DIR/mujoco_left_replay.py" "$trajectory_npz"
