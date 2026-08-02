#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
COMMON_GIT_DIR="$(git -C "${ROOT}" rev-parse --path-format=absolute --git-common-dir)"
PRIMARY_ROOT="$(dirname -- "${COMMON_GIT_DIR}")"
PARENT="$(cd -- "${PRIMARY_ROOT}/.." && pwd -P)"
PYTHON="${ROOT}/.venv-ros2/bin/python"
MSG_SOURCE="${PARENT}/wujihandros2/wujihand_msgs"
[[ -x "${PYTHON}" ]] || { echo "Run setup_ubuntu24_wuji_env.sh first" >&2; exit 1; }
[[ -f "${MSG_SOURCE}/package.xml" ]] || { echo "Missing ${MSG_SOURCE}" >&2; exit 1; }
source /opt/ros/jazzy/setup.bash
export PATH="${ROOT}/.venv-ros2/bin:${PATH}"
cd -- "${ROOT}"
"${PYTHON}" -m colcon build \
  --base-paths "${MSG_SOURCE}" ros2_ws/src/twin_wuji_sim \
  --build-base ros2_ws/build --install-base ros2_ws/install --log-base ros2_ws/log \
  --symlink-install --packages-select wujihand_msgs twin_wuji_sim \
  --cmake-args -DPython3_EXECUTABLE="${PYTHON}"
