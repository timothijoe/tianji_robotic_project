# Ubuntu 24 Wuji Environment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Configure reproducible, hardware-free MuJoCo, ROS 2 Jazzy, and official Wuji SDK environments on Ubuntu 24.04.

**Architecture:** Repository-local shell entry points create three isolated virtual environments. The ROS overlay builds only the upstream message package and this repository's simulation bridge, without the upstream USB driver.

**Tech Stack:** Ubuntu 24.04, CPython 3.12, ROS 2 Jazzy, colcon, MuJoCo 3.10.0, NumPy 2.5.1, pytest 9.1.1, wujihandpy, Bash.

## Global Constraints

- Use `/opt/ros/jazzy`; do not install Humble, Kilted, a container, or a virtual machine.
- Use sibling checkouts of the primary Git worktree, `../wujihandros2` and
  `../wujihandpy`, without modifying either. Resolve the primary worktree from
  `git rev-parse --path-format=absolute --git-common-dir` so scripts also work
  from a linked worktree.
- Build only `wujihand_msgs` and `twin_wuji_sim`; never build or launch `wujihand_driver` or `wujihand_bringup`.
- Never run `sudo`, create USB rules, probe USB, instantiate `wujihandpy.Hand`, or send hardware commands.
- Local `.venv`, `.venv-ros2`, `.venv-wujihand`, and ROS build output remain Git-ignored.

---

## File structure

| Path | Responsibility |
| --- | --- |
| `scripts/setup_ubuntu24_wuji_env.sh` | Validate local prerequisites and install all three environments. |
| `scripts/build_ros2_jazzy_wuji_sim.sh` | Build the restricted two-package Jazzy overlay. |
| `docs/simulation/setup.md` | Core setup and validation instructions. |
| `docs/simulation/ros2_wuji_hand_bridge.md` | Jazzy-specific ROS build and smoke-test instructions. |
| `tests/simulation/test_guarded_chop_launcher.py` | Source-level safety tests for setup entry points. |

### Task 1: Add the hardware-free environment setup entry point

**Files:**
- Create: `scripts/setup_ubuntu24_wuji_env.sh`
- Modify: `tests/simulation/test_guarded_chop_launcher.py`

**Interfaces:**
- Consumes: `requirements-sim.lock`, `pyproject.toml`, `../wujihandros2/wujihand_msgs/package.xml`, and `../wujihandpy/pyproject.toml`.
- Produces: `.venv`, `.venv-ros2`, `.venv-wujihand`.

- [ ] **Step 1: Write failing source-level safety test**

```python
def test_ubuntu24_wuji_setup_is_hardware_free_and_validates_siblings():
    script = _read_script("setup_ubuntu24_wuji_env.sh")
    assert 'wujihandros2/wujihand_msgs/package.xml' in script
    assert 'wujihandpy/pyproject.toml' in script
    assert '.venv-wujihand/bin/python -m pip install "${WUJI_HAND_PY}"' in script
    assert 'pip install -e "${WUJI_HAND_PY}"' not in script
    assert "wujihandpy.Hand" not in script
    assert "sudo" not in script
```

- [ ] **Step 2: Run the test and confirm it fails**

Run: `.venv/bin/python -m pytest tests/simulation/test_guarded_chop_launcher.py -k ubuntu24_wuji -q`

Expected: `FAIL` because `scripts/setup_ubuntu24_wuji_env.sh` does not exist.

- [ ] **Step 3: Implement the setup script**

```bash
#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
COMMON_GIT_DIR="$(git -C "${ROOT}" rev-parse --path-format=absolute --git-common-dir)"
PRIMARY_ROOT="$(dirname -- "${COMMON_GIT_DIR}")"
PARENT="$(cd -- "${PRIMARY_ROOT}/.." && pwd -P)"
ROS_ROOT="/opt/ros/jazzy"
WUJI_ROS="${PARENT}/wujihandros2"
WUJI_HAND_PY="${PARENT}/wujihandpy"
[[ -x /usr/bin/python3.12 ]] || { echo "Python 3.12 is required" >&2; exit 1; }
[[ -f "${ROS_ROOT}/setup.bash" ]] || { echo "ROS 2 Jazzy is required" >&2; exit 1; }
[[ -f "${WUJI_ROS}/wujihand_msgs/package.xml" ]] || { echo "Missing ${WUJI_ROS}" >&2; exit 1; }
[[ -f "${WUJI_HAND_PY}/pyproject.toml" ]] || { echo "Missing ${WUJI_HAND_PY}" >&2; exit 1; }
cd -- "${ROOT}"
/usr/bin/python3.12 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements-sim.lock
.venv/bin/python -m pip install -e .
/usr/bin/python3.12 -m venv --system-site-packages .venv-ros2
.venv-ros2/bin/python -m pip install mujoco==3.10.0 numpy==2.5.1
/usr/bin/python3.12 -m venv .venv-wujihand
.venv-wujihand/bin/python -m pip install "${WUJI_HAND_PY}"
```

- [ ] **Step 4: Verify focused test and shell syntax**

Run: `.venv/bin/python -m pytest tests/simulation/test_guarded_chop_launcher.py -k ubuntu24_wuji -q && bash -n scripts/setup_ubuntu24_wuji_env.sh`

Expected: `1 passed` and no shell-check output.

- [ ] **Step 5: Commit**

Run: `git add scripts/setup_ubuntu24_wuji_env.sh tests/simulation/test_guarded_chop_launcher.py && git commit -m "feat: add Ubuntu 24 Wuji environment setup"`

### Task 2: Add the restricted Jazzy overlay build entry point

**Files:**
- Create: `scripts/build_ros2_jazzy_wuji_sim.sh`
- Modify: `tests/simulation/test_guarded_chop_launcher.py`

**Interfaces:**
- Consumes: `.venv-ros2/bin/python`, `/opt/ros/jazzy/setup.bash`, `../wujihandros2/wujihand_msgs`, and `ros2_ws/src/twin_wuji_sim`.
- Produces: `ros2_ws/install/setup.bash` for `wujihand_msgs` and `twin_wuji_sim` only.

- [ ] **Step 1: Write failing source-level safety test**

```python
def test_jazzy_build_script_limits_colcon_scope_to_message_and_sim_packages():
    script = _read_script("build_ros2_jazzy_wuji_sim.sh")
    assert "source /opt/ros/jazzy/setup.bash" in script
    assert "--packages-select wujihand_msgs twin_wuji_sim" in script
    assert "wujihand_driver" not in script
    assert "wujihand_bringup" not in script
```

- [ ] **Step 2: Run the test and confirm it fails**

Run: `.venv/bin/python -m pytest tests/simulation/test_guarded_chop_launcher.py -k jazzy_build -q`

Expected: `FAIL` because `scripts/build_ros2_jazzy_wuji_sim.sh` does not exist.

- [ ] **Step 3: Implement the restricted build script**

```bash
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
```

- [ ] **Step 4: Verify focused test and shell syntax**

Run: `.venv/bin/python -m pytest tests/simulation/test_guarded_chop_launcher.py -k jazzy_build -q && bash -n scripts/build_ros2_jazzy_wuji_sim.sh`

Expected: `1 passed` and no shell-check output.

- [ ] **Step 5: Commit**

Run: `git add scripts/build_ros2_jazzy_wuji_sim.sh tests/simulation/test_guarded_chop_launcher.py && git commit -m "feat: add restricted Jazzy ROS build"`

### Task 3: Update the documentation and run hardware-free validation

**Files:**
- Modify: `docs/simulation/setup.md`
- Modify: `docs/simulation/ros2_wuji_hand_bridge.md`

**Interfaces:**
- Consumes: Task 1 and Task 2 scripts.
- Produces: Ubuntu 24/Jazzy instructions with no stale developer-machine paths.

- [ ] **Step 1: Replace the documented environment and build commands**

Add these exact commands and explain the sibling-checkout, message-only, and import-only constraints:

```bash
./scripts/setup_ubuntu24_wuji_env.sh
./scripts/build_ros2_jazzy_wuji_sim.sh
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export PYTHONPATH="$PWD/src:${PYTHONPATH:-}"
```

- [ ] **Step 2: Run complete validation**

```bash
sha256sum --check docs/simulation/protected-files.sha256
.venv/bin/python -c "import mujoco, numpy, twin_sim; print(mujoco.__version__, numpy.__version__)"
.venv/bin/python -m pytest -q
.venv-wujihand/bin/python -c "import importlib.metadata as m; import wujihandpy; print(m.version('wujihandpy'))"
./scripts/build_ros2_jazzy_wuji_sim.sh
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export PYTHONPATH="$PWD/src:${PYTHONPATH:-}"
.venv-ros2/bin/python tests/ros2/ros2_bridge_smoke.py
```

Expected: checksum entries are `OK`; core prints `3.10.0 2.5.1`; pytest has zero failures; SDK prints a version; smoke prints `ROS2_WUJI_SIM_SMOKE_OK`.

- [ ] **Step 3: Validate text and shell integrity**

Run: `git diff --check && bash -n scripts/setup_ubuntu24_wuji_env.sh scripts/build_ros2_jazzy_wuji_sim.sh`

Expected: no output and a zero exit status.

- [ ] **Step 4: Commit**

Run: `git add docs/simulation/setup.md docs/simulation/ros2_wuji_hand_bridge.md && git commit -m "docs: add Ubuntu 24 Jazzy Wuji setup"`
