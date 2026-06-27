# twin_joint_ws

Pure MuJoCo phase-one workspace for a dual-arm Marvin model. The source robot
asset is kept under `MarvinCCS/marvin_final_fixed.xml`; phase-one task geometry
lives in `src/twin_description/twin_description/assets/robot/mujoco/right_chopping_scene.xml`.

## Setup

```bash
python3 -m pip install -e .[test]
```

## Test

```bash
# 进入项目目录后运行全部测试（45 项）
PYTHONPATH="src/twin_core:src/twin_description:src/twin_mujoco" python3 -m pytest -v
```

测试覆盖：

- 源模型加载（14 关节 / 14 执行器）
- 切菜场景加载（砧板、刀、力传感器、site）
- 刀-法兰朝向一致性（quat 对齐验证）
- 双臂运行时（ArmView、力矩施加）
- 力控制器（Cartesian impedance、力矩限幅）
- 切菜状态机（3 周期 headless demo、CSV 日志）

## Headless right-arm chopping

```bash
PYTHONPATH="src/twin_core:src/twin_description:src/twin_mujoco" twin-chop --cycles 3 --headless --log /tmp/twin_right_chop.csv
```

## Visual right-arm chopping

```bash
PYTHONPATH="src/twin_core:src/twin_description:src/twin_mujoco" twin-chop --cycles 3 --viewer --log /tmp/twin_right_chop.csv
```

The first phase is pure MuJoCo. ROS2 nodes and launch files are intentionally
out of scope.
