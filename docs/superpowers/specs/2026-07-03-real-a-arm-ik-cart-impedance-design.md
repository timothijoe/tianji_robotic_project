# Real A-Arm IK Cartesian Impedance Debug Design

## Goal

Create a dedicated real-machine debug folder for A-arm Cartesian impedance experiments. The script should reproduce the core trajectory logic from `examples/DEMO_PYTHON_STYLE/showcase_ik_cart_impedance.py` with `dz=-20 mm`, `hold_s=2.0`, `cycles=5`, lateral motion enabled, and `lateral_mm=10`, while using the vendor `SDK_PYTHON` real robot and kinematics APIs.

## Scope

The new code lives under `real_robot_debug/` and is intentionally separate from simulation examples. It does not modify `SDK_PYTHON/`, `test/`, `src/twin_control/`, or MuJoCo runtime code.

The default arm is `A`, matching `test/v1_test_torque_eefcart.py`. The script starts from the robot's current feedback pose and applies a relative down-up trajectory, instead of sending the MuJoCo B-arm chopping home pose to the real robot.

## Runtime Behavior

The main script connects to the robot, clears existing errors, verifies frame feedback updates, initializes the kinematics SDK from an `.MvKDCfg` file, reads A-arm feedback joints, computes the current TCP pose, and configures Cartesian impedance plus end-effector Cartesian orientation control.

For each control step, it computes the target TCP pose in SDK units, solves IK using the previous target as the reference, sends the resulting joint command, samples feedback, and optionally writes a CSV trace. The motion profile is the same down-up cycle shape used by the MuJoCo debug script: half-cycle descent and half-cycle retract. When lateral motion is enabled, each cycle shifts along Y by `lateral_mm`, with the shift applied during retract.

## Safety Defaults

The script defaults to `--dry-run`, which computes IK and writes trace data without sending joint commands. Real command output requires `--execute`.

Default limits are conservative and explicit:

- `--arm A`
- `--control-hz 250`
- `--dz-mm -20`
- `--hold-s 2.0`
- `--cycles 5`
- `--lateral`
- `--lateral-mm 10`
- `--vel-ratio 10`
- `--acc-ratio 10`

The script rejects invalid arm names, non-positive frequency, non-positive cycle count, excessive vertical motion over 80 mm, and excessive lateral step over 50 mm.

## Testing

Automated tests do not connect to hardware. They validate pure helpers: cycle progress, trajectory generation, argument defaults, safety validation, and importability of the new real-machine script.
