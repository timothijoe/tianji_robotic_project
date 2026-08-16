# Keyboard Cartesian Jog Design

## Goal

Provide a deliberately conservative, keyboard-operated Cartesian jog utility
for the existing physical Tianji robot SDK.  An operator can move one selected
arm in base-frame X, Y, or Z by a small, discrete distance per key press.
This is for commissioning and debugging, not for autonomous operation.

## Scope and Boundaries

The implementation adds `real_robot_debug/keyboard_cartesian_jog.py` and its
offline tests.  It does not modify chopping, recorded-trajectory playback,
ROS 2, hand control, or MuJoCo code.

The initial target is SDK arm `A` (the left arm).  The arm selector remains an
explicit CLI option so that support for `B` does not require duplicate code.
The utility does not command both arms at once.

## Operator Interface

The default coordinate frame is the robot base frame.  The key mapping is
configured and printed at startup:

| Key | Requested base-frame delta |
| --- | --- |
| `W` / `S` | `+X` / `-X` |
| `A` / `D` | `+Y` / `-Y` |
| `R` / `F` | `+Z` / `-Z` |
| `Space` | Stop: disable the selected arm and terminate |
| `Q` | Terminate and disable the selected arm |

The physical meaning of positive axes is displayed but must be verified by a
2 mm dry commissioning move at the physical workcell.  Existing project notes
describe `+Y` as downward for the A-arm setup; the program does not infer a
human-oriented front/left/up convention from that fact.

Each valid movement key requests exactly one displacement with default
`--step-mm 2`.  It is an edge-triggered terminal program, not a press-and-hold
velocity controller.

## Runtime Flow

1. Parse and validate configuration before opening the robot connection.
2. In real execution mode require an explicit workspace box:
   `--workspace-min x,y,z` and `--workspace-max x,y,z` in SDK millimetres.
3. Connect to the existing vendor SDK, clear errors, verify fresh feedback, and
   read current feedback joints.
4. Use the existing kinematics configuration to compute the feedback TCP pose.
5. For every movement key, form a proposed position by adding one base-frame
   delta while retaining the current TCP orientation.
6. Reject the proposal when it is outside the workspace, when a previous
   trajectory has not returned idle, or when MOVLA planning fails.
7. For an accepted proposal, create one `MOVLA` segment from the feedback pose
   to the proposed pose.  With `--execute`, issue the SDK `setPln_Cart` command
   and wait for trajectory idle.  Without it, print the planned target only and
   do not issue a motion command.
8. After a completed segment, obtain fresh feedback and use it as the start
   point for the next key press.  This avoids accumulating idealized targets
   when the hardware has not reached them.
9. On `Q`, `Space`, errors, EOF, or interruption, clear the command buffer,
   disable the selected arm unless `--keep-enabled` was explicitly selected,
   and release the SDK connection.

## Safety Contract

- `--execute` is required before any motion command is sent.
- `--step-mm` must be positive and no greater than 5 mm.
- Velocity and acceleration ratios must be in `[0, 100]`; defaults are 10.
- The workspace box is mandatory for `--execute`, finite, and strictly ordered
  on all axes.  Each Cartesian proposal is checked before planning and sending.
- A stale feedback stream, non-idle trajectory, SDK planning error, or an
  unexpected controller state aborts the requested step without a fallback
  joint command.
- `Space` is software stop/disable, not a substitute for the physical E-stop.
- The startup banner states that a physical E-stop and a clear workspace are
  prerequisites for a real run.

There are no assumed universal workspace limits in source code because the
safe volume depends on the installed tool, table, and workcell.  Operators must
supply the vetted bounds for the current setup.

## Architecture

Pure, offline-testable helpers own configuration validation, parsing the
three-value box, mapping key bytes to named base-frame deltas, producing a
candidate pose, and containment checks.  A small SDK adapter owns connection,
feedback, `MOVLA`, `setPln_Cart`, idle waiting, and cleanup.  Raw terminal mode
is confined to a context manager so terminal settings are restored on all exit
paths.

The SDK adapter may reuse the established helper semantics in
`real_ik_cart_impedance_lateral.py`, but it must not import a CLI entry point or
cause a command merely by being imported.

## Validation

Offline pytest tests will cover:

- all six movement key mappings and non-motion keys;
- 2 mm candidate pose generation preserving orientation;
- configuration rejection for unsafe step, malformed/inverted workspace, and
  execute mode without workspace bounds;
- boundary inclusion and rejection just outside the box;
- that dry-run produces a plan but does not call the SDK command sender;
- that execute mode refuses to command when planning fails or feedback/idle
  checks fail, using a fake SDK adapter.

No automated test connects to a robot.  The implementation handoff includes a
dry-run command and a separately labelled, operator-reviewed `--execute`
example; it will not start the latter automatically.

## Acceptance Criteria

- The utility accepts the documented keys and never sends a command without
  `--execute`.
- Every executed step is at most 5 mm, lies inside the operator-provided
  workspace, and starts from refreshed feedback.
- Failures leave the selected arm disabled by default and report why the step
  was refused.
- The complete offline test suite passes without vendor SDK hardware access.
