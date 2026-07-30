# Twin Right-Arm Chopping Design

## Context

`twin_joint_ws` will become the development workspace for a dual-arm table-chopping simulation. The existing `cook_ws` project is a single-arm MuJoCo and ROS2 project that can chop a tabletop with one arm. The new project should follow the useful boundaries from `cook_ws`, but stay simpler and start with a pure MuJoCo implementation before adding ROS integration.

The current `twin_joint_ws` assets contain `MarvinCCS/marvin_final_fixed.xml`. This model already loads in MuJoCo and exposes 14 hinge joints:

- `left_joint1` through `left_joint7`
- `right_joint1` through `right_joint7`

The model has a shared `robot_base` body. Both arms are mounted under that body with different positions and opposite base quaternions. There is no separately named `shoulder` body, but `robot_base` is the shared upper-body or shoulder-like mounting body that connects both arms. This means the right arm's local base frame must be measured from the MuJoCo model and cannot reuse the single-arm `cook_ws` home pose, board position, or tool orientation unchanged.

The original model currently has no tool site, force sensor site, force or torque sensor, knife, chopping board, or task scene.

## Selected Approach

Use a ROS2 workspace-style layout from the start, but implement only the pure MuJoCo packages in the first phase:

```text
twin_joint_ws/
  src/
    twin_core/
    twin_description/
    twin_mujoco/
  examples/
  tests/
  docs/
```

This approach puts the long-term dual-arm abstraction first. The first deliverable is still narrow: right-arm force-controlled chopping in pure MuJoCo. ROS2 nodes, launch files, and RViz integration are explicitly out of scope for this phase.

## Package Responsibilities

### `twin_core`

`twin_core` defines backend-independent data structures and arm selection concepts. It must not depend on MuJoCo, ROS2, or asset paths.

Responsibilities:

- Define arm identifiers such as `left` and `right`.
- Define `ArmSpec` with arm name, joint names, actuator names, and optional metadata.
- Define joint state and joint trajectory data structures.
- Provide small validation helpers for finite values, joint ordering, and arm selection.

### `twin_description`

`twin_description` owns robot assets and model paths.

Responsibilities:

- Keep `MarvinCCS/marvin_final_fixed.xml` as an unchanged source asset.
- Add `src/twin_description/assets/robot/mujoco/right_chopping_scene.xml`.
- Provide path helpers for the source model and right-arm chopping scene.
- Document the right-arm board, tool, and sensor conventions.

The original source XML must remain unchanged. The chopping scene is a new MJCF file that copies the dual-arm structure and adds task-specific elements.

### `twin_mujoco`

`twin_mujoco` owns MuJoCo loading, arm views, force control, and demos.

Responsibilities:

- Load the full 14-joint dual-arm model.
- Build joint, dof, qpos, actuator, site, and sensor mappings.
- Expose `ArmView("left")` and `ArmView("right")` over the shared runtime.
- Provide a right-arm force-control runtime that performs 7-DoF Cartesian impedance and hybrid force control while the underlying MuJoCo model remains 14-DoF.
- Provide a CLI for viewer and headless chopping demos.

## Scene Design

`right_chopping_scene.xml` will include:

- The complete dual-arm robot from the source MJCF.
- A floor.
- A chopping board placed on the right-arm side of the workspace, in world `y < 0`.
- A tool and knife mounted under `right_link7`.
- `right_force_sensor_body`.
- `right_force_sensor_site`.
- `right_tool_body`.
- `right_tool_tip_site`.
- MuJoCo force and torque sensors attached to the right force sensor site.

Initial measured facts from the current source model:

- The right arm is mounted at `right_link1` under `robot_base`.
- At zero joint position, `right_link7` is near world `y=-0.8155`, `z=0.5196`.
- The left arm is symmetric in positive `y`.

The board position and right-arm safe home pose will be chosen by MuJoCo validation, not copied from `cook_ws`. Tests must verify the final result.

The first tool mounting transform will follow the same intent as `cook_ws`: place a force sensor and tool under the wrist so the tool-tip local Z axis points approximately toward world `-Z` in the safe chopping pose. The exact transform may be adjusted during implementation until the tests pass.

## Runtime and Arm Views

`TwinMujocoRuntime` loads the complete model and owns the shared MuJoCo `model` and `data`.

It exposes:

- All joint names.
- All actuator names.
- `arm_view("left")`.
- `arm_view("right")`.
- Body and site pose queries.
- Shared stepping and reset.

`ArmView` is a lightweight 7-axis projection over the shared runtime. For the right arm it maps:

- joints: `right_joint1` through `right_joint7`
- actuators: `act_right_joint1` through `act_right_joint7`

`ArmView` provides:

- joint positions and velocities for one arm.
- actuator effort limits for one arm.
- site pose and Jacobian for one arm's tool site.
- torque application for that arm only.

The right-arm force controller uses only the right-arm 7-DoF Jacobian and actuator set. The shared runtime still steps the full 14-DoF model.

## Left Arm Behavior

The left arm is present in the model but not task-active in phase one.

Default behavior:

- Reset the left arm to its default or configured home position.
- Do not run IK or force control for the left arm.
- Do not include the left arm in chopping state transitions.
- During right-arm torque control, keep left-arm actuators at zero torque unless a simple hold mode is needed to prevent drift.

If drift becomes a problem in headless tests, a simple joint-space hold controller may be added for the left arm. This hold controller must remain separate from right-arm chopping logic.

## Chopping Control

The first task uses the right arm and follows the force-control pattern from `cook_ws`:

1. `APPROACH`: move the right tool tip to a safe height above the board.
2. `DESCEND`: descend toward the board until the search height is reached.
3. `FORCE_HOLD`: enable hybrid force control along tool Z and maintain the target force.
4. `RETRACT`: move back to safe height.
5. `SHIFT`: move laterally to the next chopping position.
6. `COMPLETE` or `FAULT`: finish or stop safely.

Default demo settings:

- right arm only.
- at least 3 chopping cycles.
- configurable `--cycles`, `--force`, `--hold`, `--viewer`, `--headless`, and `--log`.
- CSV samples include phase, control mode, target pose, actual pose, measured force, raw wrench, compensated wrench, joint positions, joint velocities, applied torques, and fault text.

The controller uses:

- Cartesian impedance for free-space motion.
- MuJoCo force and torque sensors for measured wrench.
- Wrench calibration at startup.
- Hybrid force control along the tool Z axis during force hold.
- Torque limits from the selected right-arm actuators.

## Error Handling

Startup must fail with clear errors if required model elements are missing:

- right-arm joints or actuators.
- `right_tool_tip_site`.
- `right_force_sensor_site`.
- right force or torque sensor.
- `chopping_board`.

Runtime safety failures enter `FAULT`, disable force control, and attempt a safe retract:

- IK does not converge.
- tool Z axis is not sufficiently downward at safe home.
- contact search times out.
- measured force exceeds the configured maximum.
- position tracking error exceeds the configured maximum.
- MuJoCo state contains NaN or infinity.

CSV write failures should be reported clearly. They should not hide the controller fault or success state.

## Testing

Tests will be written before implementation where practical.

Required tests:

- Source model loads with MuJoCo and exposes 14 joints.
- Chopping scene loads with MuJoCo and exposes 14 joints, 14 actuators, force sensor, torque sensor, tool site, sensor site, and chopping board.
- Arm specs select exactly 7 joints and 7 actuators for each arm.
- `ArmView("right")` exposes right-arm qpos, dof, actuator limits, and a 6x7 tool Jacobian.
- Right-arm safe home keeps the tool tip above the board.
- Right tool Z axis is approximately world `-Z` at safe home.
- The force sensor site is above the tool tip at safe home.
- Cartesian impedance produces finite torque and respects right-arm actuator limits.
- Headless chopping runs at least 3 cycles and reaches `COMPLETE`.
- Headless chopping produces a CSV log with samples from all expected phases.

## Phase-One Acceptance Criteria

Phase one is complete when:

- `twin_joint_ws` has a git repository rooted in `twin_joint_ws`.
- The pure MuJoCo workspace packages are present: `twin_core`, `twin_description`, and `twin_mujoco`.
- `MarvinCCS/marvin_final_fixed.xml` remains unchanged.
- `right_chopping_scene.xml` loads in MuJoCo.
- A headless right-arm chopping command completes at least 3 cycles.
- Tests verify the scene, right-arm mappings, tool direction, force-control controller, and 3-cycle chopping behavior.
- No ROS2 nodes or launch files are required for this phase.

## Out of Scope

- ROS2 nodes, launch files, RViz visualization, and topic adapters.
- Dual-arm cooperative chopping.
- Real robot SDK integration.
- Editing the original source MJCF in `MarvinCCS`.
- Full collision-aware planning.
- GUI teach pendant support.
