#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
COMMON_GIT_DIR="$(git -C "${ROOT}" rev-parse --path-format=absolute --git-common-dir)"
PRIMARY_ROOT="$(dirname -- "${COMMON_GIT_DIR}")"
PARENT="$(cd -- "${PRIMARY_ROOT}/.." && pwd -P)"
ROS_ROOT="/opt/ros/jazzy"
WUJI_ROS="${PARENT}/wujihandros2"
WUJI_HAND_PY="${PARENT}/wujihandpy"
WUJI_ROS_PACKAGE="${PARENT}/wujihandros2/wujihand_msgs/package.xml"
WUJI_HAND_PY_MANIFEST="${PARENT}/wujihandpy/pyproject.toml"

[[ -x /usr/bin/python3.12 ]] || {
    echo "Python 3.12 is required" >&2
    exit 1
}
[[ -f "${ROS_ROOT}/setup.bash" ]] || {
    echo "ROS 2 Jazzy is required" >&2
    exit 1
}
[[ -f "${WUJI_ROS_PACKAGE}" ]] || {
    echo "Missing ${WUJI_ROS}" >&2
    exit 1
}
[[ -f "${WUJI_HAND_PY_MANIFEST}" ]] || {
    echo "Missing ${WUJI_HAND_PY}" >&2
    exit 1
}

cd -- "${ROOT}"
/usr/bin/python3.12 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements-sim.lock
.venv/bin/python -m pip install -e .
/usr/bin/python3.12 -m venv --system-site-packages .venv-ros2
.venv-ros2/bin/python -m pip install mujoco==3.10.0 numpy==2.5.1
/usr/bin/python3.12 -m venv .venv-wujihand
.venv-wujihand/bin/python -m pip install "${WUJI_HAND_PY}"
