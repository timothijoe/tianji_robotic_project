# Real Robot Debug

This folder is for real-machine experiments only. It is intentionally separate
from MuJoCo examples and starts from the robot's current feedback pose.

## Recommended Real Run

Use this command for the current left-arm sampled IK joint-impedance chopping
run. It initializes SDK arm `A`, chops along real-robot `+Y` downward, shifts on
`Z`, prints Cartesian and six-axis force feedback, and writes the full telemetry
CSV.

```bash
PYTHONPATH=. python3 real_robot_debug/real_sampled_joint_impedance_chop.py \
  --robot-ip 192.168.1.190 \
  --arm A \
  --init-joints "112.46,-51.20,-85.44,-72.70, 47.48, -12.84, 35.14" \
  --execute \
  --control-hz 250 \
  --dz-mm -40 \
  --hold-s 2.0 \
  --cycles 2 \
  --lateral \
  --lateral-mm 10 \
  --chop-axis y \
  --lateral-axis z \
  --lateral-phase separate \
  --joint-k "8,8,8,4,2,1.5,1" \
  --joint-d "0.8,0.8,0.8,0.6,0.4,0.3,0.2" \
  --print-feedback \
  --feedback-stride 25 \
  --print-force-feedback \
  --force-feedback-stride 25 \
  --trace-csv /home/zhoutong/catkin_robotic_ws/cook_proj/tianji_robotic_project/data/tmp/real_left_arm_joint_impedance_trace.csv
```

The trace CSV records timestamped telemetry:

- `timestamp_s`, `elapsed_s`
- target and actual Cartesian pose: `target_x/y/z/a/b/c`, `actual_x/y/z/a/b/c`
- target and actual joint positions: `target_q_0..6`, `actual_q_0..6`
- actual joint velocities and torques: `actual_qd_0..6`, `actual_tau_0..6`
- six-axis force/torque feedback: `force_fx/fy/fz`, `torque_tx/ty/tz`

## Joint-Impedance Variants

Same controller as the recommended command, but with the previous initial pose,
`Y` chopping and `X` lateral shift.

```bash
PYTHONPATH=. python3 real_robot_debug/real_sampled_joint_impedance_chop.py \
  --robot-ip 192.168.1.190 \
  --arm A \
  --init-joints "109.81,-62.66,-95.69,-93.79,63.32,-2.76,12.42" \
  --execute \
  --control-hz 250 \
  --dz-mm -20 \
  --hold-s 2.0 \
  --cycles 2 \
  --lateral \
  --lateral-mm 10 \
  --chop-axis y \
  --lateral-axis x \
  --lateral-phase separate \
  --joint-k "8,8,8,4,2,1.5,1" \
  --joint-d "0.8,0.8,0.8,0.6,0.4,0.3,0.2" \
  --print-feedback \
  --feedback-stride 25 \
  --print-force-feedback \
  --force-feedback-stride 25 \
  --trace-csv /tmp/real_left_arm_joint_impedance_trace.csv
```

Use `--lateral-phase retract` if you intentionally want lateral motion during
the upward/retract stroke. The default `separate` mode returns to the upper
position first, then shifts laterally.

## Sampled Position Mode

This is useful for comparing against the joint-impedance run. It uses the same
sampled Cartesian target generation and IK, but sends sampled joint targets in
position mode.

```bash
PYTHONPATH=. python3 real_robot_debug/real_sampled_position_chop.py \
  --robot-ip 192.168.1.190 \
  --arm A \
  --init-joints "109.81,-62.66,-95.69,-93.79,63.32,-2.76,12.42" \
  --execute \
  --control-hz 250 \
  --dz-mm -20 \
  --hold-s 2.0 \
  --cycles 2 \
  --lateral \
  --lateral-mm 10 \
  --chop-axis y \
  --lateral-axis x \
  --lateral-phase separate \
  --print-feedback \
  --feedback-stride 25 \
  --print-force-feedback \
  --force-feedback-stride 25 \
  --trace-csv /tmp/real_left_arm_sampled_position_trace.csv
```

## Planned Cartesian MOVLA Mode

This is the coarser planned Cartesian path. It uses MOVLA segments rather than
sampled IK targets at every control tick.

```bash
PYTHONPATH=. python3 real_robot_debug/real_pln_cart_position_chop.py \
  --robot-ip 192.168.1.190 \
  --arm A \
  --execute \
  --control-hz 250 \
  --dz-mm -20 \
  --hold-s 2.0 \
  --cycles 5 \
  --lateral \
  --lateral-mm 10 \
  --chop-axis y \
  --lateral-axis x \
  --trace-csv /tmp/real_left_arm_pln_cart_trace.csv
```

## Offline Planning Check

This does not connect to the robot. It uses FK from the initial joint pose,
generates sampled Cartesian targets, runs IK, and prints/writes the theoretical
Cartesian and joint trajectory.

```bash
PYTHONPATH=. python3 real_robot_debug/plan_sampled_ik_chop.py \
  --init-joints "112.46,-51.20,-85.44,-72.70,47.48,-12.84,35.14" \
  --control-hz 250 \
  --dz-mm -40 \
  --hold-s 2.0 \
  --cycles 2 \
  --lateral \
  --lateral-mm 10 \
  --chop-axis y \
  --lateral-axis z \
  --lateral-phase separate \
  --stride 25 \
  --output-csv /tmp/planned_left_arm_joint_impedance_shape.csv
```

## Keyboard Cartesian Jog

`keyboard_cartesian_jog.py` is a discrete, base-frame debugging utility for
one arm. Each movement key requests one Cartesian step while preserving the
current tool orientation. It does not provide continuous press-and-hold
control.

The default is dry-run: it still connects to the controller to read the current
feedback pose, then plans and prints requested 2 mm steps, but never sends
`setPln_Cart` or enables a motion command.

```bash
PYTHONPATH=. python3 real_robot_debug/keyboard_cartesian_jog.py --arm A
```

Keys: `W`/`S` request `+X`/`-X`; `A`/`D` request `+Y`/`-Y`; `R`/`F` request
`+Z`/`-Z`; `Q` and Space exit and disable the selected arm. The physical
direction of each positive axis must be confirmed at the installed workcell;
the project notes currently describe A-arm `+Y` as downward.

Real execution requires `--execute` plus a site-specific, vetted workspace
box in millimetres. Do not copy the following placeholders as values:

```bash
PYTHONPATH=. python3 real_robot_debug/keyboard_cartesian_jog.py \
  --arm A --execute --step-mm 2 \
  --workspace-min "XMIN,YMIN,ZMIN" \
  --workspace-max "XMAX,YMAX,ZMAX"
```

Replace all six bounds with the current tool/table/workcell's verified limits.
The program rejects a step beyond the box, a step above 5 mm, stale feedback,
a non-idle trajectory, or a failed MOVLA plan. Keep the physical E-stop
reachable: Space is a software disable, not an E-stop.

Before the first real movement, keep `--step-mm 2`, verify each positive axis
with a clear workspace, and use a physical E-stop that is reachable by the
operator. The script is intentionally one-key/one-step; holding a key never
creates continuous motion.

## Notes

- SDK arm `A` is the left arm on this robot.
- In the real robot frame used here, `+Y` is downward. The scripts treat
  `--dz-mm -40` as a 40 mm downward chop when `--chop-axis y` is used.
- `--lateral-axis z` shifts along the arm-direction axis; `--lateral-axis x`
  shifts forward/backward.
- `--execute` sends commands to the real robot. Omit it for dry-run planning
  and logging setup only.
- The script rejects vertical motion above 80 mm and lateral step above 50 mm.
