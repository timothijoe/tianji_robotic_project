# SDK-style MuJoCo demos

These examples intentionally follow the vendor `DEMO_PYTHON` style: create a
robot object, connect, set a control mode, send commands, wait, and release.

By default they run against MuJoCo:

```bash
python3 examples/DEMO_PYTHON_STYLE/showcase_position.py --backend mujoco --viewer
python3 examples/DEMO_PYTHON_STYLE/showcase_joint_impedance.py --backend mujoco --headless
```

The same scripts can target the real robot by switching the backend and IP:

```bash
python3 examples/DEMO_PYTHON_STYLE/showcase_position.py \
  --backend real --robot-ip 192.168.1.190
```

Keep business logic in calls shared by both backends:

- `connect(...)`
- `set_position_state(...)`
- `set_imp_joint_state(...)`
- `set_joint_position_cmd(...)`
- `set_force_cmd(...)`
- `wait(...)`
- `release_robot()`

MuJoCo maps SDK arm `A` to the left simulated arm and SDK arm `B` to the right
simulated arm.  The current chopping setup usually uses arm `B`.
