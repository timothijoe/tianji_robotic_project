# Real A-Arm Planned Cartesian Position Chop Design

## Goal

Create a real-machine A-arm planned Cartesian position chopping entrypoint that mirrors the useful parts of `examples/position_mode_chop_demo.py` without depending on MuJoCo. The script should initialize the real A arm to a known starting joint pose, then plan and execute the same relative chopping pattern:

- `dz_mm = -20`
- `hold_s = 2.0`
- `cycles = 5`
- lateral motion enabled
- `lateral_mm = 10`
- `control_hz = 250`

## Entry Point

Add a dedicated wrapper under `real_robot_debug/`:

```bash
python3 real_robot_debug/real_a_pln_cart_position_chop.py \
  --robot-ip 192.168.1.190
```

The wrapper delegates to `real_robot_debug.real_ik_cart_impedance_lateral` with A-arm defaults and forces `--command-mode pln-cart`.

Real movement remains opt-in:

```bash
python3 real_robot_debug/real_a_pln_cart_position_chop.py \
  --robot-ip 192.168.1.190 \
  --execute
```

## Defaults

The A-arm wrapper sets:

- `arm = A`
- `init_joints = 0,0,0,-5,0,0,0`
- `control_hz = 250`
- `dz_mm = -20`
- `hold_s = 2.0`
- `cycles = 5`
- `lateral = true`
- `lateral_mm = 10`
- `vel_ratio = 10`
- `acc_ratio = 10`

The initial pose is intentionally conservative. It is not derived from the right-arm MuJoCo chopping home, because that pose belongs to arm B and should not be mirrored onto arm A without validation.

## Runtime Flow

The execution path is:

1. Connect to the real robot SDK.
2. Clear controller errors and verify A-arm feedback frames update.
3. Initialize the kinematics SDK from the configured `.MvKDCfg`.
4. If `--init-joints` is not `none`, switch A arm to position/planning mode and move to the initial joint pose.
5. Wait until the initialization target is reached within tolerance.
6. Read real A-arm feedback joints.
7. Compute the real current TCP pose with FK.
8. Build planned Cartesian chopping segments from that measured TCP pose.
9. For each segment, call `movLA(...)` and send the returned plan with `setPln_Cart(...)` when `--execute` is set.
10. Write optional trace CSV rows with target/actual pose and joint data.
11. Disable the arm at the end unless `--keep-enabled` is set.

Dry-run mode follows the same planning path but does not send motion commands.

## Safety

The wrapper keeps the existing real-machine safeguards:

- default dry-run
- explicit `--execute` required for motion
- maximum vertical motion magnitude of `80 mm`
- maximum lateral step of `50 mm`
- positive control frequency, hold time, cycle count, and initialization timeout
- initialization failure diagnostics include joint error, controller state, error code, trajectory state, target joints, and final joints

The wrapper does not expose MuJoCo-only options such as `--viewer`, `--viewer-trace`, `--position-k-scale`, `--position-d-scale`, or `--torque-rate-limit`.

## Testing

Automated tests should avoid connecting to hardware. They should validate:

- the A-arm wrapper appends or overrides arguments to force `--arm A` and `--command-mode pln-cart`
- the wrapper default initialization joints are `0,0,0,-5,0,0,0`
- the existing pure trajectory helpers still generate descend/retract/lateral planned segments
- invalid motion limits are rejected by existing validation

