# Real Robot Debug

This folder is for real-machine experiments only. It is intentionally separate
from MuJoCo examples and starts from the robot's current feedback pose.

Default position-mode chopping script:

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
  --trace-csv /tmp/real_a_arm_trace.csv
```

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

- arm: `A`
- control frequency: `250 Hz`
- vertical motion: `-20 mm`
- cycle hold time: `2.0 s`
- cycles: `5`
- lateral step: `10 mm`
- velocity/acceleration ratio: `10`

The script rejects vertical motion above 80 mm and lateral step above 50 mm.
