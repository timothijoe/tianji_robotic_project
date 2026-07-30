# Right Chopping Scene

`right_chopping_scene.xml` is the phase-one MuJoCo task scene. It copies the
dual-arm model from `MarvinCCS/marvin_final_fixed.xml` and adds only task
objects: floor, right-side chopping board, right wrist force sensor body, tool,
tool-tip site, and force/torque sensors.

The source XML in `MarvinCCS/` is treated as an upstream asset and is not edited.
The board is placed in front of the robot at the shared chopping workspace (`x > 0`, `y = 0`). Runtime
tests verify the final safe home pose, tool Z direction, sensor-site placement,
and board clearance before the chopping demo is accepted.

## Tool-frame lesson

The knife mount went through several incorrect iterations. The initial mistake
was treating the visible wrist/flange mesh, the force-sensor body frame, and the
control tool frame as the same coordinate frame. That led to a sequence of
compensating quaternions: moving the sensor body to the visible mesh origin,
rotating the knife visual to offset the sensor body's rotation, then rotating
the connector to offset both of those. Each local fix made one part look better
while leaving another part visibly perpendicular to the flange.

The accepted convention is to separate mechanical visual alignment from task
control frames:

- `right_force_sensor_body` and `right_tool_body` stay parallel to `right_link7`
  (`quat="1 0 0 0"` relative to their parent frames).
- `right_force_sensor_adapter`, `right_cleaver_handle_visual`, and
  `right_cleaver_visual` use the same visual axis as the flange/Link7 chain, so
  the visible sequence is flange -> connector -> knife handle -> blade.
- `right_force_sensor_site` and `right_tool_tip_site` are allowed to carry their
  own task-frame quaternion. They preserve the downward force-control/tool-tip
  convention without forcing the visual geometry to be rotated.
- The hidden collision blade stays task-oriented for contact and force control;
  it is not a visual source of truth.

Do not fix future knife alignment problems by stacking compensating quaternions
on the visual meshes. First identify whether the issue is visual alignment
(flange/connector/knife axis) or task-frame alignment (sensor/tool-tip site).
Tests should check both: visual axes remain parallel to `right_link7`, and the
safe-home tool-tip site remains above the board with its Z axis downward.
