# MuJoCo 机械臂力控与剁切 Demo

本功能在 `cook_ws` 中实现了单臂七自由度机械臂的 MuJoCo 力矩控制、笛卡尔阻抗控制、工具 Z 轴力控制、腕部六维力传感和重复剁切 Demo。

## 已实现功能

- 七个关节力矩 actuator；
- 笛卡尔阻抗轨迹跟踪；
- 工具 Z 轴导纳式恒力控制；
- `CARTESIAN_IMPEDANCE` 与 `HYBRID_FORCE_Z` 自动切换；
- 腕部六维力/力矩传感器；
- 传感器去皮和工具重力补偿；
- 落刀、接触、恒力保持、起刀和移位状态机；
- 最大接触力、下探距离和接触超时保护；
- MuJoCo Viewer、终端状态显示和 CSV 日志；
- SDK 风格 Python API。

## 模型结构

末端运动链为：

```text
Joint6_L
→ Link6_L
→ Joint7_L
→ Link7_L
→ 绿色末端法兰
→ 六维力传感器/转接盘
→ 工具杆
→ 工具尖端
```

厂商法兰矩阵不能直接作为 URDF `Link7_L` 的局部变换。通过对比厂商 FK 和 MuJoCo FK，反求得到：

```text
Link7 → flange translation = [0, -0.095, 0] m
Link7 → flange quaternion  = [0.5, 0.5, -0.5, 0.5]  # wxyz
```

剁切场景将机器人底座抬高 `90 mm`，使工具近似垂直向下时能够在砧板上方保留约 `81 mm` 的初始安全间距。

## 环境构建

```bash
cd /home/zhoutong/catkin_robotic_ws/cook_proj/cook_ws
colcon build --symlink-install
source install/setup.bash
```

每次打开新终端后，需要重新执行：

```bash
cd /home/zhoutong/catkin_robotic_ws/cook_proj/cook_ws
source install/setup.bash
```

## 静态恒力测试

```bash
ros2 run cook_mujoco cook_force_control_demo \
  --mode static \
  --force 10 \
  --hold 5 \
  --viewer \
  --realtime \
  --status-hz 10 \
  --log static_force.csv
```

## 五次剁切 Demo

```bash
ros2 run cook_mujoco cook_force_control_demo \
  --mode chopping \
  --cycles 5 \
  --force 10 \
  --hold 0.5 \
  --viewer \
  --realtime \
  --status-hz 10 \
  --log chopping.csv
```

常用参数：

| 参数 | 含义 |
| --- | --- |
| `--mode static` | 单次静态恒力测试 |
| `--mode chopping` | 重复剁切测试 |
| `--cycles 5` | 剁切次数 |
| `--force 10` | 目标工具 Z 轴力，单位 N |
| `--hold 0.5` | 每次恒力保持时间，单位 s |
| `--viewer` | 打开 MuJoCo Viewer |
| `--realtime` | 按真实时间运行 |
| `--status-hz 10` | 终端状态刷新频率 |
| `--status-hz 0` | 关闭终端实时状态 |
| `--log FILE.csv` | CSV 输出路径 |

没有图形界面时去掉 `--viewer`。希望尽快完成离线仿真时也可以去掉 `--realtime`。

## 控制模式

终端会显示当前阶段和控制模式，例如：

```text
phase=DESCEND    mode=CARTESIAN_IMPEDANCE
phase=FORCE_HOLD mode=HYBRID_FORCE_Z
phase=RETRACT    mode=CARTESIAN_IMPEDANCE
```

模式含义：

- `CARTESIAN_IMPEDANCE`：X/Y/Z 和工具姿态均使用笛卡尔阻抗跟踪；
- `HYBRID_FORCE_Z`：X/Y 和姿态继续使用阻抗控制，工具 Z 轴改用导纳式力控制。

总体控制链：

```text
笛卡尔轨迹
→ 笛卡尔阻抗/工具 Z 轴力控制
→ 末端 wrench
→ Jacobian 转置
→ 关节力矩
→ MuJoCo 动力学
→ 六维力传感器反馈
```

## 六维力传感器

传感器输出顺序：

```text
[Fx, Fy, Fz, Mx, My, Mz]
```

单位：

- 力：N；
- 力矩：N·m。

终端状态示例：

```text
F=(Fx,Fy,Fz)N Fz_ctrl=...N target_Fz=...N M=(Mx,My,Mz)Nm
```

其中：

- `F`、`M`：补偿后的实时六维传感器数据；
- `Fz_ctrl`：力控制器使用的低通反馈；
- `target_Fz`：目标工具 Z 轴力。

Python 读取接口：

```python
raw_wrench = robot.get_raw_wrench()
wrench = robot.get_wrench()
mode = robot.control_mode
```

- `get_raw_wrench()`：MuJoCo 原始六维传感器数据；
- `get_wrench()`：去皮和工具重力补偿后的六维数据。

## CSV 数据

日志至少包括：

- `phase`：当前轨迹阶段；
- `control_mode`：当前控制模式；
- `target_x/y/z`、`actual_x/y/z`：目标和实际工具位置；
- `target_force_n`、`measured_force_n`：目标力和控制反馈；
- `raw_wrench_0..5`：原始六维传感器数据；
- `wrench_0..5`：补偿后的六维数据；
- `q_0..6`：七个关节位置；
- `qd_0..6`：七个关节速度；
- `tau_0..6`：七个关节控制力矩；
- `contact`：接触状态；
- `fault`：安全故障信息。

六维字段映射：

```text
0=Fx, 1=Fy, 2=Fz, 3=Mx, 4=My, 5=Mz
```

## Python API 示例

```python
from cook_description.paths import CHOPPING_MJCF_PATH
from cook_mujoco.chopping import ChoppingConfig, MujocoForceRobot

robot = MujocoForceRobot(
    CHOPPING_MJCF_PATH,
    viewer=True,
    realtime=True,
)
robot.connect()
robot.initialize()

robot.set_force_control_params(target_force_n=10.0)
robot.execute_chopping_trajectory(
    config=ChoppingConfig(
        cycles=5,
        target_force_n=10.0,
        force_hold_s=0.5,
    ),
    log_path="chopping.csv",
)

print(robot.get_wrench())
print(robot.control_mode)
print(robot.summary())
robot.close()
```

接口使用 SI 单位：米、弧度、牛顿和牛顿米。

## 安全保护

当前主要保护包括：

- 最大接触力：`30 N`；
- 最大接触搜索下探距离：`50 mm`；
- 接触建立超时：`2 s`；
- 关节力矩限幅；
- 力矩变化率限制；
- 关节限位；
- NaN/Inf 检测；
- Viewer 关闭和用户停止处理；
- 故障后的安全回撤。

真实补偿传感器值用于接触判断、峰值统计和 30 N 安全保护；低通反馈仅用于控制，不能掩盖接触冲击。

## 测试

运行全量测试：

```bash
cd /home/zhoutong/catkin_robotic_ws/cook_proj/cook_ws

PYTHONPATH=src/cook_mujoco:src/cook_description:src/cook_core:src/cook_bringup \
pytest -q
```

当前结果：

```text
50 passed, 6 skipped
```

当前五次剁切验证结果约为：

```text
真实力峰值：11.23 N
恒力阶段 RMSE：1.28 N
非接触轨迹 RMSE：2.66 mm
五次剁切：完成
```

具体结果会随控制参数、接触模型和运行平台略有变化。

## 关键文件

```text
src/cook_description/assets/robot/mujoco/chopping_scene.xml
src/cook_mujoco/cook_mujoco/control/force_control.py
src/cook_mujoco/cook_mujoco/chopping.py
src/cook_mujoco/cook_mujoco/chopping_cli.py
tests/test_force_control.py
```

更多设计和历史记录：

- [`docs/FORCE_CONTROL_REQUIREMENTS.md`](docs/FORCE_CONTROL_REQUIREMENTS.md)
- [`docs/FORCE_CONTROL_USAGE.md`](docs/FORCE_CONTROL_USAGE.md)
- [`docs/FORCE_SENSOR_AND_CONTROL_MODES.md`](docs/FORCE_SENSOR_AND_CONTROL_MODES.md)
- [`docs/ORIGINAL_FORCE_CONTROL_REQUEST.md`](docs/ORIGINAL_FORCE_CONTROL_REQUEST.md)
