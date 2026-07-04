# Real Robot Debug

This folder is for real-machine experiments only. It is intentionally separate
from MuJoCo examples and starts from the robot's current feedback pose.

Default planned Cartesian MOVLA chopping script:

```bash
python3 real_robot_debug/real_pln_cart_position_chop.py \
  --robot-ip 192.168.1.190 \
  --arm A \
  --control-hz 250 \
  --dz-mm -20 \
  --hold-s 2.0 \
  --cycles 5 \
  --lateral \
  --lateral-mm 10 \
  --trace-csv /tmp/real_left_arm_trace.csv
```


MuJoCo-like sampled IK position-mode chopping script:

```bash
python3 real_robot_debug/real_sampled_position_chop.py \
  --robot-ip 192.168.1.190 \
  --arm A \
  --control-hz 250 \
  --dz-mm -20 \
  --hold-s 2.0 \
  --cycles 5 \
  --lateral \
  --lateral-mm 10 \
  --print-trajectory \
  --trajectory-stride 10 \
  --trace-csv /tmp/real_left_arm_sampled_trace.csv
```

This sampled entrypoint is closer to the MuJoCo Cartesian impedance demo: every
control tick builds a TCP target, solves IK, and sends a joint position command
when `--execute` is present. Use it when checking the down-up chopping shape.

Sampled IK joint-impedance chopping script:

```bash
python3 real_robot_debug/real_sampled_joint_impedance_chop.py \
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
  --trace-csv /tmp/real_left_arm_joint_impedance_trace.csv
```

This mode uses the same Cartesian target generation and IK as the sampled
position script, but switches the SDK to joint impedance and sends each IK
solution with `set_joint_position_cmd`. The initialization move still uses
position mode before joint impedance is enabled.

By default this is a dry run: it connects, initializes the planned Cartesian
position-mode path with MOVLA, and writes trace rows, but does not send motion
commands. Add `--execute` only when the workspace is clear and the robot is ready.
The execute path first moves the arm to `--init-joints` and then repeats the
descend/retract chopping motion.

```bash
python3 real_robot_debug/real_pln_cart_position_chop.py \
  --robot-ip 192.168.1.190 \
  --execute
```

Safety defaults:

- arm: `A` (left arm)
- control frequency: `250 Hz`
- vertical motion: `-20 mm`
- cycle hold time: `2.0 s`
- cycles: `5`
- lateral step: `10 mm`
- velocity/acceleration ratio: `10`

The script rejects vertical motion above 80 mm and lateral step above 50 mm.
