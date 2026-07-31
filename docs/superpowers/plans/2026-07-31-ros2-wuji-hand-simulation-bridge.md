# ROS 2 Wuji Hand Simulation Bridge Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Install ROS 2 Humble and provide tested wujihandpy-style and ROS 2 interfaces that control the existing MuJoCo left hand without physical hardware.

**Architecture:** Preserve the existing Python 3.12 simulation environment, install ROS 2 Humble for Ubuntu's Python 3.10, and create a separate `.venv-ros2` with system site packages for MuJoCo. A `SimWujiHand` adapter wraps `RightArmRobot`; a project-owned ROS package hosts one bridge node that subscribes to upstream-compatible commands and publishes states and diagnostics.

**Tech Stack:** Ubuntu 22.04, ROS 2 Humble, Python 3.10/3.12, MuJoCo 3.10.0, NumPy, rclpy, sensor_msgs, wujihand_msgs, colcon, pytest

## Global Constraints

- Do not instantiate the official USB-backed `wujihandpy.Hand` without hardware.
- Do not enable or command any physical actuator.
- Do not modify `/home/linux/august_folder/wujihandpy` or `/home/linux/august_folder/wujihandros2`.
- Keep all flat arrays in the canonical 20-joint finger-major order.
- ROS uses `/usr/bin/python3` 3.10; the existing simulation environment remains Python 3.12.
- Preserve protected real-robot files.

---

### Task 1: Install and Verify ROS 2 Humble

**Files:**
- System: `/etc/apt/sources.list.d/ros2.list`
- System: `/usr/share/keyrings/ros-archive-keyring.gpg`
- Create: `.venv-ros2/` (ignored local environment)

**Produces:** `/opt/ros/humble`, working `ros2`, `colcon`, `rosdep`, and a Python 3.10 environment that can import both `rclpy` and `mujoco`

- [ ] Add the official ROS apt signing key and Humble repository for Jammy.
- [ ] Run `apt update` and install the exact packages named in the design.
- [ ] Source `/opt/ros/humble/setup.bash` and verify `ros2 --help`,
  `ros2 doctor --report`, and an isolated talker/listener smoke test.
- [ ] Create `.venv-ros2` using
  `/usr/bin/python3 -m venv --system-site-packages .venv-ros2`.
- [ ] Install `mujoco==3.10.0` and a Python-3.10-compatible NumPy into that
  environment; verify imports of `rclpy`, `mujoco`, and `numpy`.

### Task 2: Install and Inspect the Official wujihandpy SDK

**Files:**
- Create: `.venv-wujihand/` (ignored local environment)

**Produces:** official SDK import and no-device diagnostic evidence

- [ ] Create a Python 3.12 virtual environment and install released
  `wujihandpy`.
- [ ] Verify import, package version, and exposed `Hand`/filter APIs.
- [ ] Run a discovery-only construction attempt with no motor-enable or write
  calls; require the expected no-device failure while `lsusb` lacks
  `0483:2000`.
- [ ] Record SDK logs and never treat no hardware as a failed installation.

### Task 3: Implement the SimWujiHand Backend with TDD

**Files:**
- Create: `src/twin_sim/wuji_hand_backend.py`
- Create: `tests/simulation/test_wuji_hand_backend.py`

**Produces:** `SimWujiHand`, `SimRealtimeController`, `(5,4)` read/write API,
enable state, and finite simulated effort

- [ ] Write tests for shape conversion, default enabled matrix, enable/disable,
target validation, stepping, realtime context entry/exit, and unsupported API
errors.
- [ ] Run tests and confirm failure because the module is absent.
- [ ] Implement the minimal adapter around `RightArmRobot` and
`LeftHandController`.
- [ ] Run focused tests and the existing hand tests.
- [ ] Commit as `feat: add wujihandpy-style simulation backend`.

### Task 4: Build ROS Messages and the MuJoCo Bridge

**Files:**
- Create: `ros2_ws/src/twin_wuji_sim/package.xml`
- Create: `ros2_ws/src/twin_wuji_sim/setup.py`
- Create: `ros2_ws/src/twin_wuji_sim/setup.cfg`
- Create: `ros2_ws/src/twin_wuji_sim/resource/twin_wuji_sim`
- Create: `ros2_ws/src/twin_wuji_sim/twin_wuji_sim/__init__.py`
- Create: `ros2_ws/src/twin_wuji_sim/twin_wuji_sim/bridge_node.py`
- Create: `ros2_ws/src/twin_wuji_sim/twin_wuji_sim/demo_node.py`
- Create: `ros2_ws/src/twin_wuji_sim/launch/sim_hand.launch.py`
- Create: `tests/ros2/test_bridge_validation.py`

**Produces:** `twin-wuji-sim-bridge`, ROS topic/service compatibility, and a
public-topic demo

- [ ] Write pure callback/command-decoding tests that cover named and positional
messages plus atomic rejection.
- [ ] Build upstream `wujihand_msgs` and the project package into
`ros2_ws/install` using colcon without building the hardware driver.
- [ ] Implement the node at `/hand_left`, subscribe `joint_commands`, publish
`joint_states` and `hand_diagnostics`, and provide `set_enabled`.
- [ ] Implement a conservative open/relaxed-close demo publisher.
- [ ] Add launch parameters `viewer`, `hand_name`, `publish_rate`, and
`start_enabled`.
- [ ] Rebuild and run package tests.
- [ ] Commit as `feat: add ROS2 MuJoCo Wuji hand bridge`.

### Task 5: Verify End-to-End ROS 2 Control

**Files:**
- Create: `tests/ros2/ros2_bridge_smoke.py`

**Produces:** evidence that ROS commands change MuJoCo hand state without USB

- [ ] Launch the bridge headlessly in its own process and wait conditionally for
`/hand_left/joint_states`.
- [ ] Echo one state message and assert 20 exact names and finite positions.
- [ ] Publish a small named command through `/hand_left/joint_commands`.
- [ ] Observe successive states and assert the commanded joint trends toward
the target while other targets remain unchanged.
- [ ] Call `set_enabled` false, publish another command, and prove it is
rejected/ignored; re-enable and repeat.
- [ ] Stop all ROS processes cleanly and verify no bridge process remains.
- [ ] Commit as `test: verify ROS2 Wuji hand simulation bridge`.

### Task 6: Document Environments and Tomorrow's Hardware Procedure

**Files:**
- Create: `docs/simulation/ros2_wuji_hand_bridge.md`
- Modify: `README.md`
- Modify: `docs/simulation/wuji_hand_control.md`
- Modify: `.gitignore`

**Produces:** reproducible setup/source/build/run commands and explicit
simulation-versus-hardware instructions

- [ ] Document ROS installation, the two Python environments, colcon build,
bridge launch, topic publication, services, demo, and shutdown.
- [ ] Document the no-device result and tomorrow's read-only-first hardware
checklist from the design.
- [ ] Link the new guide from README and the existing hand guide.
- [ ] Ignore `.venv-ros2`, `.venv-wujihand`, and ROS build/install/log outputs.
- [ ] Run full simulation tests, ROS package tests, end-to-end smoke,
`sha256sum --check docs/simulation/protected-files.sha256`, and
`git diff --check`.
- [ ] Commit as `docs: add ROS2 Wuji hand integration guide`.
