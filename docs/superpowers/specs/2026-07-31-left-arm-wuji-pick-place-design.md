# Left Arm + Wuji Hand Pick-and-Place Design

## Goal

Add a MuJoCo task in which the left arm and the mounted Wuji Hand cooperate to
pick up a movable cube, carry it laterally, and place it in a target region.
The cube must be held by modeled contact and friction; the task must not weld,
attach, teleport, or otherwise hide a failed grasp.

This is phase A. A later phase B will reuse the same left-arm and grasping
interfaces to hold a vegetable while the right arm performs the existing
chopping task. Phase B is not implemented in this scope.

## Current State

- `RightArmRobot` owns one MuJoCo instance.
- The right arm has a position-target command API and a right-tool TCP.
- The left arm is reset and then held at its initial joint configuration.
- `Kinematics` is hard-coded to the right arm and `right_tool_tip_site`.
- The Wuji Hand has 20 position actuators and a tested `LeftHandController`.
- The scene does not yet contain a free grasp object, a left-palm TCP site, or
  a left-arm task controller.

## Chosen Approach

Generalize the existing position-control architecture instead of introducing
impedance control or mocap-based motion:

1. Generalize arm kinematics so a caller selects the arm indices and TCP site.
2. Give the left palm a stable `left_palm_tcp_site`.
3. Maintain independent left-arm, right-arm, and hand position targets.
4. Plan Cartesian left-palm motion through continuous IK and execute
   minimum-jerk position trajectories.
5. Close the fingers gradually and determine grasp readiness from modeled
   contacts, object motion, and conservative actuator-force limits.
6. Execute a staged pick-and-place state machine.

This architecture preserves the existing right-arm chopping behavior and
creates reusable boundaries for phase B.

## Scene and Object

Add one approximately 0.05 m cube with:

- a MuJoCo free joint;
- visible and collision geometry;
- nonzero mass and inertia;
- contact friction suitable for finger grasping;
- a deterministic initial pose on the chopping board;
- no equality constraint connecting it to the hand.

Add a visible target region on the table. The initial and target placements
must be reachable by the left arm without entering the right-arm chopping
workspace.

The left palm receives a named TCP site whose pose represents the grasp frame.
Its axes and offset are documented and validated in a model test.

## Arm Abstractions

Replace right-only assumptions in `Kinematics` with explicit construction from:

- an `ArmIndices` instance;
- a TCP site name.

FK, Jacobian, IK, and path solving operate only on the selected arm. Temporary
FK/IK calculations must restore all MuJoCo state exactly.

Extend the robot controller with:

- independent left and right joint targets;
- validated left-arm commands;
- left-palm pose access;
- unchanged right-arm public behavior.

Stepping writes both arm targets and the hand target before advancing physics.
Commands remain position targets; this feature does not introduce a custom
joint or Cartesian impedance controller.

## Pick-and-Place State Machine

The task uses these ordered phases:

1. `INITIALIZE`: reset the scene and verify finite state.
2. `OPEN_HAND`: establish the safe open posture.
3. `PREGRASP`: move the palm to an offset pose near the cube.
4. `APPROACH`: move linearly to the grasp pose.
5. `CLOSE_HAND`: interpolate finger targets toward a conservative enclosure.
6. `STABILIZE`: hold the arm and hand while evaluating grasp readiness.
7. `LIFT`: raise the palm while preserving orientation.
8. `TRANSFER`: move laterally to the target region.
9. `LOWER`: descend until the cube is just above or contacting the table.
10. `RELEASE`: open the fingers gradually.
11. `RETREAT`: move the palm away from the placed cube.
12. `COMPLETE`: hold the final pose for inspection.

The task records the phase associated with every sample. Phase B can later
reuse the approach, close, stabilize, and hold portions without depending on
the pick-and-place CLI.

## Grasp Detection and Finger Control

Finger closing is a time-parameterized position command, not a target jump.
During closure the task monitors:

- contacts between hand geoms and the cube;
- whether contacts involve opposing parts of the hand rather than one isolated
  touch;
- cube linear and angular speed during stabilization;
- simulated hand actuator force.

Closure stops tightening when a stable multi-contact enclosure is detected or
the conservative simulated-force threshold is reached. Simulated actuator
force is a safety signal for this model only and is not treated as calibrated
physical torque.

The lift begins only after the readiness criteria pass for a continuous dwell
interval. The task must not infer success solely because the hand reached its
requested closed pose.

## Failure Handling

The task aborts before transfer when:

- IK fails or a solved path violates the joint-step limit;
- a requested arm or hand target is invalid;
- the grasp readiness dwell is not achieved within its timeout;
- the cube is dropped after lift;
- the cube or robot penetrates the table beyond a small numerical tolerance;
- contact/actuator force exceeds its abort threshold;
- any controlled MuJoCo state becomes non-finite.

On abort, the controller keeps the last accepted arm and hand targets and stops
increasing finger closure. It reports the phase and reason. It does not
automatically open the hand while the cube may still be elevated.

## Trajectory Visualization and Logging

The Viewer displays:

- planned left-palm path;
- actual left-palm path;
- actual cube path;
- initial cube and target-region markers;
- current task phase and grasp/contact status.

Headless execution records equivalent numeric samples so tests do not depend on
the Viewer. The final Viewer hold is long enough for inspection and remains
configurable.

## Success Criteria

A run succeeds only if:

- the cube rises at least 0.08 m above its initial table height;
- its horizontal displacement reaches at least 0.15 m;
- it is released within the target region;
- it settles without being attached to the hand;
- no table penetration, numerical failure, force abort, or IK/path error occurs;
- the right-arm target and state remain unchanged within simulation tolerance.

Exact poses and phase durations may be tuned from headless results, but the
thresholds above are acceptance requirements rather than visual impressions.

## Testing

### Model and unit tests

- cube free-joint, mass, collision, and absence of hidden constraints;
- left-palm TCP existence and documented frame;
- left/right kinematics isolation and state restoration;
- left-arm command validation and right-arm regression behavior;
- phase ordering and failure transitions;
- contact classification and grasp-readiness dwell;
- trajectory continuity and finite output.

### Headless integration test

Run the complete task and assert all success criteria, including cube lift,
transfer, placement, right-arm isolation, and no hidden attachment.

### Viewer acceptance

Run a slow visualization and inspect palm, cube, and target trajectories,
finger contact behavior, phase changes, placement, and final hold.

## ROS 2 Boundary

The first implementation exposes a local task/CLI and does not expand the
existing ROS hand bridge into an arm motion protocol. Its controller and state
machine must remain independent of the CLI so a later ROS action or service can
invoke the same task without duplicating motion logic.

## Phase B Extension

After phase A is accepted, phase B will:

- replace the cube goal with a vegetable hold pose;
- keep the left arm and hand in a monitored hold state;
- coordinate readiness with the right-arm chopping task;
- abort right-arm descent if the left grasp or hold becomes unsafe.

Phase B requires a separate design review because hand placement, knife
clearance, and two-arm failure coordination add safety constraints not needed
for the standalone pick-and-place task.
