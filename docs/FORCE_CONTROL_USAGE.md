# MuJoCo 力控 Demo 使用说明

## 环境

从 `cook_ws` 目录构建并加载工作区：

```bash
colcon build --symlink-install
source install/setup.bash
```

## 静态恒力测试

```bash
ros2 run cook_mujoco cook_force_control_demo \
  --mode static --force 10 --hold 5 --viewer --realtime \
  --log static_force.csv
```

## 五次剁切测试

```bash
ros2 run cook_mujoco cook_force_control_demo \
  --mode chopping --cycles 5 --force 10 --hold 0.5 \
  --viewer --realtime --log chopping.csv
```

无显示环境中去掉 `--viewer`；希望尽快完成离线仿真时再去掉 `--realtime`。

## Python API

```python
from cook_description.paths import CHOPPING_MJCF_PATH
from cook_mujoco.chopping import ChoppingConfig, MujocoForceRobot

robot = MujocoForceRobot(CHOPPING_MJCF_PATH, viewer=True, realtime=True)
robot.connect()
robot.initialize()
robot.set_force_control_params(target_force_n=10.0)
robot.execute_chopping_trajectory(
    config=ChoppingConfig(cycles=5, force_hold_s=0.5),
    log_path="chopping.csv",
)
print(robot.summary())
robot.close()
```

控制接口全部使用 SI 单位：米、弧度、牛顿和牛顿米。正目标力表示工具沿自身 Z 轴压向砧板。

## 实时查看六维力和控制模式

CLI 默认以 5 Hz 输出状态，也可以通过 `--status-hz` 修改：

```bash
ros2 run cook_mujoco cook_force_control_demo \
  --mode chopping --cycles 5 --force 10 --hold 0.5 \
  --viewer --realtime --status-hz 10 --log chopping.csv
```

终端中的模式含义：

- `CARTESIAN_IMPEDANCE`：六维笛卡尔阻抗轨迹跟踪；
- `HYBRID_FORCE_Z`：X/Y 和姿态保持阻抗控制，工具 Z 轴切换为力控制。

`F=(Fx,Fy,Fz)` 和 `M=(Mx,My,Mz)` 是工具坐标系下补偿后的六维传感器值，`Fz_ctrl` 是力控制器使用的低通反馈。CSV 中的 `raw_wrench_0..5` 为原始值，`wrench_0..5` 为补偿值，`control_mode` 为当前控制模式。

Python 中可读取：

```python
raw = robot.get_raw_wrench()  # [Fx, Fy, Fz, Mx, My, Mz]
wrench = robot.get_wrench()  # 去皮及重力补偿后
mode = robot.control_mode
```

## 补充说明

六维传感器字段、第七轴安装关系和控制模式切换详见 [`FORCE_SENSOR_AND_CONTROL_MODES.md`](FORCE_SENSOR_AND_CONTROL_MODES.md)。
