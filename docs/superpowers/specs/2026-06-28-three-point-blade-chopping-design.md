# Three-Point Blade Chopping Design

## Context

The current right-arm chopping demo is a pure MuJoCo program. It runs through
`twin-chop` and does not connect to ROS2, a real robot SDK, or external
kinematics.

The existing chopping state machine treats `right_tool_tip_site` as the single
task point. Safe height, descent target height, logged actual position, and the
Cartesian controller all use that site. This was acceptable for the previous
tool assumption, where one point's distance to the board was the main cutting
condition.

The tool has changed. The new quick version should use three tool sites as the
cutting geometry:

- `right_blade_edge_top`
- `right_blade_edge_bot`
- `right_tool_tip_site`

For this phase, a cut is considered geometrically in-place when all three sites
are near the top surface of `chopping_board`. Descent is along world `-Z`, not
along the tool frame.

## Selected Approach

Use a small three-point blade geometry layer while keeping the existing
single-site Cartesian controller.

`RightArmChopper` will read the three site poses from MuJoCo and compute their
world positions and board-relative heights. Planning will still command
`right_tool_tip_site`, but the commanded target will be derived from all three
sites instead of from the tip alone.

This is the fast MuJoCo-only implementation. It changes the demo semantics from
"tool tip moves to a board-relative height" to "the three blade reference points
are planned to reach the board top together." It does not add a full three-site
IK controller.

## Blade Geometry

Add a focused internal representation for the blade sites:

- site names: top edge, bottom edge, and tip.
- world positions for all three sites.
- Z values for all three sites.
- minimum and maximum Z values.
- board-relative clearances.

The geometry layer should answer:

- Are all three sites above the board by at least a safe margin?
- What world-Z shift is needed for all three current site points to align with
  `board_top` under the current tool orientation?
- Are all three sites within a configurable tolerance of `board_top`?

The quick implementation assumes the relative Z offsets from
`right_tool_tip_site` to the other two sites remain valid during a single
planned chop segment. This matches the current approach, where the controller
tracks a fixed rotation per run segment.

## Chopping State Machine

The public phases remain unchanged:

1. `APPROACH`
2. `DESCEND`
3. `FORCE_HOLD`
4. `RETRACT`
5. `SHIFT`
6. `COMPLETE` or `FAULT`

The target calculation changes:

- Safe target: compute the current blade site's highest and lowest points, then
  command the tip so every blade site clears `board_top` by `safe_height_m`.
- Descend target: command the tip along world `-Z` so the predicted three site
  Z values are all at `board_top` within the configured tolerance.
- Retract target: return to the safe target for the same cycle.
- Shift target: move laterally as before, using the same three-point safe and
  descend height rules at the shifted X position.

The Cartesian controller continues to use `right_tool_tip_site` and its
Jacobian. The target position changes, but the controller interface does not.

## Logging

Keep the current CSV schema compatible. Existing fields such as `target_x`,
`actual_x`, and `measured_force_n` remain.

Add in-memory sample data for the three blade site positions so tests can assert
the new behavior without parsing MuJoCo state after the run.

Do not expand the CSV schema in this quick version. Keeping the file format
unchanged preserves compatibility with existing CLI users and tests.

## Error Handling

Startup must fail clearly if any of the three required blade sites are missing.

Runtime should keep the existing safety checks. This phase does not add
collision-aware planning, full three-point IK convergence checks, or real robot
safety handling.

## Testing

Tests will be updated before production changes.

Required test coverage:

- The scene exposes `right_blade_edge_top`, `right_blade_edge_bot`, and
  `right_tool_tip_site`.
- The blade geometry helper returns three finite world positions.
- The safe target keeps all three blade points above the board by
  `safe_height_m`.
- The descent target predicts all three blade points near `board_top`.
- The headless chopping demo still completes and logs CSV.
- The viewer mode remains compatible with the changed state machine.

## Acceptance Criteria

This phase is accepted when:

- `twin-chop --cycles 1 --headless` completes in MuJoCo.
- Existing headless and viewer tests still pass.
- New tests prove that chopping target planning uses all three blade sites.
- The implementation remains MuJoCo-only and does not add ROS2 or real robot
  integration.

## Out of Scope

- Real robot SDK integration.
- ROS2 nodes, launch files, or RViz.
- Full three-site IK or a 9D multi-site controller.
- Editing the upstream source asset in `MarvinCCS/`.
- Replacing the current right-arm home pose unless required by tests.
