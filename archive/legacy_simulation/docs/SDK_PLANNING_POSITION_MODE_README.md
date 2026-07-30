# SDK Planning Position Mode Notes

This note records the current direction for improving
`examples/position_mode_chop_demo.py` by referencing the SDK planning-mode
position control path.

## Current conclusion

The real SDK planning path and the current MuJoCo position demo are different.

On the real robot, planned Cartesian position mode is:

1. Enter position/planning state.
2. Use `movLA(start_xyzabc, end_xyzabc, ref_joints, vel, acc, freq_hz)` to
   generate a straight Cartesian segment and a `pset`.
3. Send the planned trajectory with `setPln_Cart(arm=..., pset=pset)`.
4. Wait until `traj_state == b"\x00"`.

The local real-robot wrapper already follows this idea in
`real_robot_debug/real_ik_cart_impedance_lateral.py`, through
`_set_pln_cart_position_mode()` and `_run_pln_cart_chop()`.

The MuJoCo demo currently does not have a real SDK planning executor. It builds
Cartesian targets in Python, solves IK point by point, and sends each result
with `set_joint_position_cmd()`. Tracking quality is therefore limited by the
MuJoCo torque-PD joint position controller.

## Why planning mode may help

Planning mode can improve the trajectory command side:

- The Cartesian path is generated as SDK-style straight segments.
- Velocity and sampling frequency are defined at the planner level.
- Joint targets come from one continuous planned segment instead of independent
  IK calls for hand-sampled waypoints.

It will not fully solve contact-control behavior by itself. The MuJoCo
position mode still tracks through torque PD, so large acceleration, high tool
inertia, torque-rate limits, or contact with the board can still cause lag or
overshoot.

## Proposed MuJoCo implementation

Add a new trajectory mode:

```bash
--trajectory-mode sdk-planning
```

Expected data flow:

1. Reuse the existing blade-aligned endpoint sequence:
   safe tip -> descend tip -> safe tip, repeated by cycle and lateral offset.
2. Convert each endpoint pose to SDK `XYZABC`.
3. For each adjacent segment, call local `kine.movLA(...)`.
4. Convert returned planning points into a list of joint targets.
5. Replay those joint targets through existing MuJoCo position mode with
   `set_joint_position_cmd()`.
6. Keep the existing target/actual viewer markers and trace arrays.

This is closer to SDK planning than the current `waypoint` mode, while keeping
the demo MuJoCo-only and not touching the real robot scripts.

## Needed code changes

Planned changes:

- `src/twin_control/sdk_kine.py`
  - Add `MujocoKine.movLA(...)`.
  - It should linearly interpolate between SDK `XYZABC` poses, run IK for each
    sample, and return `(points, pset)` in the same broad shape as the vendor
    SDK.

- `examples/position_mode_chop_demo.py`
  - Add a small planned-command data structure.
  - Add `build_sdk_planning_joint_targets(...)`.
  - Extend `--trajectory-mode` choices with `sdk-planning`.
  - In `run_demo()`, use planned joint targets directly when this mode is
    selected instead of doing per-command IK.

- `tests/test_position_mode_chop_demo.py`
  - Tests have already been added for the CLI and planned-target helper.
  - These tests currently fail because the implementation is not done yet.

## Current test status

The last focused test run was:

```bash
python3 -m pytest tests/test_position_mode_chop_demo.py -q
```

Expected current result:

- Existing tests pass.
- New `sdk-planning` tests fail because:
  - `--trajectory-mode sdk-planning` is not accepted yet.
  - `build_sdk_planning_joint_targets(...)` does not exist yet.

This is intentional TDD state.

## Candidate command after implementation

One-line command for viewer testing:

```bash
PYTHONPATH=src:src/twin_core:src/twin_description:src/twin_mujoco python3 examples/position_mode_chop_demo.py --backend mujoco --viewer --control-hz 250 --dz-mm -60 --hold-s 1.5 --cycles 10 --lateral --lateral-mm 10 --contact-clearance-mm 30 --trajectory-mode sdk-planning --viewer-trace --viewer-trace-stride 1
```

If tracking still lags, first try increasing motion time instead of increasing
gains:

```bash
PYTHONPATH=src:src/twin_core:src/twin_description:src/twin_mujoco python3 examples/position_mode_chop_demo.py --backend mujoco --viewer --control-hz 250 --dz-mm -60 --hold-s 2.5 --cycles 10 --lateral --lateral-mm 10 --contact-clearance-mm 30 --trajectory-mode sdk-planning --viewer-trace --viewer-trace-stride 1
```

## Important caveat

This `sdk-planning` mode will improve trajectory generation and make the demo
closer to the SDK planning path. It is still not identical to real
`setPln_Cart()`, because MuJoCo does not currently have the vendor controller's
internal trajectory executor.
