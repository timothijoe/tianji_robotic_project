# Wuji Hand Control Guide Design

## Objective

Create a Chinese-language control guide for the simulated left Wuji Hand. The
guide must let a new developer understand the 20-joint layout, send safe
position targets in MuJoCo, read state, and run the existing demonstration
without reverse-engineering the implementation.

## Deliverables

The primary guide will be
`docs/simulation/wuji_hand_control.md`. `README.md` and
`docs/simulation/usage.md` will link to it rather than duplicating its detailed
content.

## Audience and Scope

The guide targets developers familiar with basic Python and NumPy but not with
this repository or the Wuji Hand. It documents only behavior present in the
current feature branch as executable functionality.

A final future-hardware section will describe the interface shape observed in
the downloaded `wuji-mjlab` reference—20 radians in finger-major order,
`write_target`, `read_encoders`, and a `(5, 4)` SDK boundary—but will explicitly
state that this repository does not install `wujihandpy`, open a USB device, or
provide a real-hand driver. Reference code from the external repository will
not be copied into this project.

## Content Structure

The guide will contain:

1. Model origin, left-wrist mounting point, license, and simulation-only status.
2. A concise explanation of five fingers, four actuated hinge joints per
   finger, 20 joints total, and 34 actuators in the complete robot scene.
3. An exact ordered table for all 20 joint names, matching actuator names,
   control ranges in radians, and semantic grouping. Values will be read from
   the vendored MJCF rather than estimated.
4. The exact `DEFAULT_OPEN_RAD` and `RELAXED_CLOSE_RAD` vectors with their
   intended use.
5. CLI commands for headless validation and slow Viewer demonstration,
   including the known Viewer-close exit-code limitation.
6. A minimal Python example that creates `RightArmRobot`, commands a valid
   20-value vector, calls `step`, reads actual hand qpos through
   `robot.sim.hand.qpos_ids`, and closes the robot in `finally`.
7. A smooth interpolation example that sends small position increments at a
   control interval which is an integer multiple of the 0.002-second MuJoCo
   timestep.
8. An API reference for `LeftHandController.target`, `command`, `apply`, and
   `open`, plus the relationship between `RightArmRobot.step` and
   `LeftHandController.apply`.
9. Failure behavior for wrong shape, non-finite values, out-of-range targets,
   and invalid control periods.
10. A current-versus-future table separating simulated `data.ctrl` writes from
    unimplemented physical-hand transport.

## Terminology and Ordering

The guide will use “finger 1” through “finger 5,” because the vendored original
model exposes numeric finger names and does not authoritatively label them as
thumb/index/middle/ring/little. It will not invent anatomical mappings.

All flat vectors use finger-major order:

`finger1_joint1 ... finger1_joint4, finger2_joint1 ... finger5_joint4`.

Angles are radians. Every example uses a NumPy array of shape `(20,)`.

## Safety and Accuracy Rules

- Do not imply that assigning `robot.hand.target` directly is supported; it is
  a copy-returning property.
- Explain that `command()` validates and stores a target but does not advance
  simulation time.
- Explain that `apply()` writes the target into `data.ctrl`, while
  `RightArmRobot.step()` calls `apply()` and advances MuJoCo.
- Do not recommend direct writes to numeric actuator offsets.
- Do not present a 20-value jump as the preferred motion method; use
  interpolation for nontrivial pose changes.
- Do not claim collision avoidance, grasp planning, tactile feedback, or
  physical-hand safety.
- Explicitly warn that simulation joint ranges and gains are not sufficient
  safety parameters for real hardware.

## Validation

The documentation will be checked by:

1. Parsing the vendored MJCF and comparing all table names/ranges with the
   compiled MuJoCo model.
2. Executing the minimal and interpolated examples as headless smoke tests.
3. Running `pytest -q` to ensure documentation-link edits do not accompany
   accidental code regressions.
4. Running the protected real-code checksum to prove that the entity-machine
   files remain unchanged.
5. Running `git diff --check` and scanning the guide for placeholders and
   unsupported hardware claims.
