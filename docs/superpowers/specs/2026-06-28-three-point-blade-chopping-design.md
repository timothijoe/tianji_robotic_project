# Two-Point Blade Chopping Design

## Context

The current right-arm chopping demo is a pure MuJoCo program. It runs through
`twin-chop` and does not connect to ROS2, a real robot SDK, or external
kinematics.

The existing chopping state machine treats `right_tool_tip_site` as the single
task point. Safe height, descent target height, logged actual position, and the
Cartesian controller all use that site. This matched the previous tool
assumption, where the robot pushed downward along the tool/flange direction.

The tool and connection have changed. The quick version should now model a knife
face/blade moving down as a whole. The motion direction is world `-Z`, which is
perpendicular to the old stabbing direction relative to the flange. The flange
and blade extension should be planned in a horizontal posture before the blade
is pressed downward.

For this quick phase, only two blade reference sites are constrained:

- `right_blade_edge_top`
- `right_blade_edge_bot`

`right_tool_tip_site` remains the controlled MuJoCo site because the existing
Cartesian controller and Jacobian are built around it, but it is not a cutting
completion constraint in this phase.

## Selected Approach

Use a small two-point blade geometry layer while keeping the existing
single-site Cartesian controller.

`RightArmChopper` will read the two blade-edge site poses from MuJoCo and
compute their world positions and board-relative heights. Planning will still
command `right_tool_tip_site`, but the commanded position and rotation will be
derived from the two blade-edge sites instead of from the tip alone.

This is the fast MuJoCo-only implementation. It changes the demo semantics from
"tool tip moves toward a board-relative height" to "the blade edge line is made
horizontal, then the blade edge is planned down to the board." It does not add a
full multi-site IK controller.

## Blade Geometry

Add a focused internal representation for the blade edge sites:

- site names: top edge and bottom edge.
- world positions for both sites.
- Z values for both sites.
- minimum and maximum Z values.
- board-relative clearances.

The geometry layer should answer:

- Are both blade-edge sites above the board by at least a safe margin?
- What target rotation makes the blade-edge line horizontal in world Z?
- What world-Z shift is needed for the predicted blade-edge sites to align with
  `board_top` after the horizontal target posture is applied?
- Are both blade-edge sites within a configurable tolerance of `board_top`?

The quick implementation uses the existing tool-tip controller as the only task
controller. It predicts blade-edge target positions by applying a rigid transform
from the current tool-tip pose to the target tool-tip pose and target rotation.

## Chopping State Machine

The public phases remain unchanged:

1. `APPROACH`
2. `DESCEND`
3. `FORCE_HOLD`
4. `RETRACT`
5. `SHIFT`
6. `COMPLETE` or `FAULT`

The target calculation changes:

- Target rotation: choose a horizontal blade posture by projecting the current
  tool local Z axis into the world XY plane and using it as the target tool Z
  axis. The target tool X axis is chosen from the current tool X axis projected
  orthogonal to that Z axis. This keeps the posture close to the current pose
  while making the blade extension horizontal.
- Safe target: with the horizontal target rotation, command the tip so both
  blade-edge sites clear `board_top` by `safe_height_m`.
- Descend target: with the same horizontal target rotation, command the tip
  along world `-Z` so both predicted blade-edge site Z values are at
  `board_top`.
- Retract target: return to the safe target for the same cycle.
- Shift target: move laterally as before, using the same two-point safe and
  descend height rules at the shifted X position.

The Cartesian controller continues to use `right_tool_tip_site` and its
Jacobian. The target position and rotation change, but the controller interface
does not.

## Logging

Keep the current CSV schema compatible. Existing fields such as `target_x`,
`actual_x`, and `measured_force_n` remain.

Add in-memory sample data for the two blade-edge site positions so tests can
assert the new behavior without parsing MuJoCo state after the run.

Do not expand the CSV schema in this quick version. Keeping the file format
unchanged preserves compatibility with existing CLI users and tests.

## Error Handling

Startup must fail clearly if either required blade-edge site is missing.

Runtime should keep the existing safety checks. This phase does not add
collision-aware planning, full multi-site IK convergence checks, or real robot
safety handling.

## Testing

Tests will be updated before production changes.

Required test coverage:

- The scene exposes `right_blade_edge_top`, `right_blade_edge_bot`, and
  `right_tool_tip_site`.
- The blade geometry helper returns two finite blade-edge world positions.
- The horizontal target rotation predicts the two blade-edge points at equal
  world Z.
- The safe target keeps both blade-edge points above the board by
  `safe_height_m`.
- The descent target predicts both blade-edge points near `board_top`.
- The headless chopping demo still completes and logs CSV.
- The viewer mode remains compatible with the changed state machine.

## Acceptance Criteria

This phase is accepted when:

- `twin-chop --cycles 1 --headless` completes in MuJoCo.
- Existing headless and viewer tests still pass.
- New tests prove that chopping target planning uses the two blade-edge sites
  and horizontal blade posture.
- The implementation remains MuJoCo-only and does not add ROS2 or real robot
  integration.

## Out of Scope

- Real robot SDK integration.
- ROS2 nodes, launch files, or RViz.
- Full multi-site IK or a 6D/9D multi-site controller.
- Editing the upstream source asset in `MarvinCCS/`.
- Constraining `right_tool_tip_site` to reach `board_top` in this quick phase.
