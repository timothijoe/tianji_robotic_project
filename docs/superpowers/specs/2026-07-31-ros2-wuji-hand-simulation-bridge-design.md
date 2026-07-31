# ROS 2 Wuji Hand Simulation Bridge Design

## Objective

Install ROS 2 Humble on Ubuntu 22.04 and add a simulation-only Wuji Hand
integration that lets wujihandpy-style Python code and wujihandros2-compatible
ROS 2 topics control the left hand already mounted in the MuJoCo scene.

No physical hand is currently present. The implementation must provide a clean
backend boundary so tomorrow's hardware bring-up can replace MuJoCo with the
official driver without changing application-level joint ordering or ROS topic
contracts.

## System Installation

Install ROS 2 Humble from the official ROS apt repository using the supported
Ubuntu 22.04 binary packages. Install:

- `ros-humble-ros-base`;
- `ros-humble-robot-state-publisher`;
- `ros-humble-rviz2`;
- `ros-humble-sensor-msgs`;
- `ros-humble-std-msgs`;
- `python3-colcon-common-extensions`;
- `python3-rosdep`.

Install the official `wujihandpy` Python distribution into a dedicated project
environment, not the system Python. The official SDK remains available for
tomorrow's hardware diagnostics, but simulation code must not instantiate its
USB-backed `Hand` class.

The existing `/home/linux/august_folder/wujihandros2` checkout will be built as
an upstream reference workspace after its submodules and dependencies are
validated. Project-specific simulation packages remain in this repository so
the upstream checkout is not modified.

## Architecture

The integration has three layers:

1. `LeftHandController` remains the low-level MuJoCo position-target owner.
2. `SimWujiHand` exposes a wujihandpy-shaped simulation API and converts between
   flat `(20,)` vectors and SDK-style `(5, 4)` arrays.
3. A ROS 2 bridge node owns `RightArmRobot`, subscribes to hand commands,
   advances MuJoCo, and publishes hand state.

The ROS node is the only component that owns and advances a MuJoCo instance in
ROS mode. Callback functions update validated targets; a timer callback applies
the current target, steps physics, and publishes state. This prevents concurrent
callbacks from independently stepping the same model.

## Simulated wujihandpy-Compatible API

The project will not create a top-level package named `wujihandpy`, because
that would shadow the official SDK. It will provide
`twin_sim.wuji_hand_backend.SimWujiHand`.

The supported subset is:

- `read_joint_actual_position() -> np.ndarray` with shape `(5, 4)`;
- `read_joint_target_position() -> np.ndarray` with shape `(5, 4)`;
- `write_joint_target_position(target: np.ndarray) -> None`;
- `write_joint_enabled(enabled: bool) -> None`;
- `read_joint_enabled() -> np.ndarray` with shape `(5, 4)`;
- `step(control_dt_s: float) -> None`;
- `realtime_controller(...)` as a context manager exposing
  `set_joint_target_position`, `get_joint_actual_position`, and
  `get_joint_actual_effort`.

The simulation effort output is explicitly marked non-calibrated. Unsupported
official SDK methods raise a clear `NotImplementedError`; no silent dummy
success is allowed.

When disabled, new position commands are rejected and stepping holds the last
accepted target. Enabling simulation does not correspond to physical motor
power.

## ROS 2 Interface

Default namespace: `/hand_left`.

The bridge will provide:

- subscription `/hand_left/joint_commands`, type
  `sensor_msgs/msg/JointState`;
- publisher `/hand_left/joint_states`, type
  `sensor_msgs/msg/JointState`;
- service `/hand_left/set_enabled`, using the upstream
  `wujihand_msgs/srv/SetEnabled` when the upstream message package is
  available;
- publisher `/hand_left/diagnostics`, using the upstream
  `wujihand_msgs/msg/HandDiagnostics` when available.

Commands may be either:

- named: `name` and `position` arrays, updated by joint name; or
- positional: exactly 20 positions in the canonical finger-major order.

Unknown names, duplicate names, non-finite values, wrong array lengths, and
out-of-range positions are rejected without partially changing the active
target.

The bridge publishes the exact 20 names used by the current MuJoCo hand model.
The default simulation and publish rate is 100 Hz. MuJoCo timestep remains
0.002 seconds, so every timer interval maps to five physics substeps.

## Process and Launch Model

A ROS package in this repository will provide:

- a bridge executable;
- a command-line launch file with `viewer`, `hand_name`, `publish_rate`, and
  `start_enabled` parameters;
- a small wave/open-close publisher that uses only the public ROS topic;
- a launch test/smoke command that runs without hardware and without Viewer.

The bridge will source the ROS base environment and this project's colcon
overlay. It will use the current repository's Python package rather than copying
MuJoCo model logic into the ROS workspace.

## Verification Without Hardware

Verification is staged:

1. ROS 2 CLI and core daemon can start.
2. The upstream `wujihand_msgs` package and project bridge build with colcon.
3. `SimWujiHand` unit tests verify shape conversion, enable state, validation,
   realtime context behavior, and finite stepping.
4. ROS tests launch the bridge headlessly, observe one 20-joint state message,
   publish a small valid command, and confirm actual positions trend toward it.
5. Invalid commands do not change the target.
6. The complete existing MuJoCo suite continues to pass.
7. Protected physical-robot files remain byte-for-byte unchanged.

The official USB driver is not launched as a success criterion because there is
no `0483:2000` device today.

## Tomorrow's Hardware Bring-Up Gate

Hardware testing must follow this order:

1. Confirm `lsusb` detects `0483:2000` and record the serial number.
2. Install the udev rule and verify non-root read access.
3. Use official `wujihandpy` for discovery and read-only position/error reads.
4. Confirm left/right handedness, all 20 encoder names, zero positions, and
   positive directions against simulation.
5. Read existing effort limits before changing them.
6. Enable only for a low-amplitude, single-joint command near the current
   position, with a person ready to cut power.
7. Disable all joints in `finally`.
8. Only after that validation, launch the official ROS driver and compare its
   `/joint_states` ordering with the simulation bridge.

Simulation gains, ranges, relaxed-close targets, and wave demonstrations are
not authorized as initial physical-hand commands.

## Non-Goals

- Emulating a USB hand device.
- Modifying or forking the official `wujihandpy` or `wujihandros2` checkouts.
- Running reinforcement learning or Isaac Lab.
- Claiming simulated efforts are physical torque measurements.
- Automatically switching to hardware when a USB device appears.
- Sending any physical motor command during this hardware-free phase.
