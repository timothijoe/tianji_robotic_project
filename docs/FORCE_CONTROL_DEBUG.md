# 力控 Debug 链路

工作目录：

```bash
cd /home/zhoutong/catkin_robotic_ws/cook_proj/cook_ws
source install/setup.bash
```

## 1. 一条控制周期的数据链

```text
CLI 参数
  chopping_cli.py
      ↓
剁切状态机：APPROACH / DESCEND / FORCE_HOLD / RETRACT / SHIFT
  chopping.py
      ↓
MuJoCo 原始传感器 [Fx,Fy,Fz,Mx,My,Mz]
  ForceControlRuntime.raw_wrench()
      ↓
去皮与工具重力补偿
  WrenchCalibrator.compensate()
      ↓
Fz 低通滤波
      ↓
一维 Z 导纳：F_target - Fz → Δz
      ↓
六维笛卡尔阻抗：位姿误差 → wrench_cmd
      ↓
Jacobian 转置：tau = J.T @ wrench_cmd + qfrc_bias
      ↓
7 轴力矩限幅与变化率限制
      ↓
MuJoCo actuator ctrl
      ↓
MuJoCo 动力学和下一周期传感器反馈
```

## 2. 使用 VS Code Debug（推荐）

从终端打开正确的工作区根目录：

```bash
cd /home/zhoutong/catkin_robotic_ws/cook_proj/cook_ws
code .
```

VS Code 需要安装扩展：

- `Python`（`ms-python.python`）；
- `Python Debugger`（`ms-python.debugpy`）。

项目已提供本机配置：

```text
.vscode/launch.json
.vscode/settings.json
.vscode/extensions.json
```

操作步骤：

1. 打开左侧 **Run and Debug**；
2. 在顶部下拉框选择一个配置；
3. 在源码行号左侧点击设置红色断点；
4. 按 `F5` 启动；
5. 使用 Variables、Watch、Call Stack 查看数据。

推荐配置：

- `Force Control: Pipeline Debug`：单次剁切，输出完整控制链；
- `Force Control: Static + Viewer`：静态 10 N 力控和 Viewer；
- `Force Control: Chopping + Viewer`：五次剁切和 Viewer；
- `Force Control: Current Python File`：调试当前打开的 Python 文件。

推荐首先在以下代码处打断点：

```text
src/cook_mujoco/cook_mujoco/chopping.py
  _search_contact_and_hold()
  _control_step()

src/cook_mujoco/cook_mujoco/control/force_control.py
  CartesianForceController.compute()
  ForceControlRuntime.raw_wrench()
  ForceControlRuntime.step()
```

进入 `CartesianForceController.compute()` 后，建议加入 Watch：

```text
self.force_enabled
wrench_tool[2]
self._filtered_force
error
self._force_position_offset
effective_target
position_error
task_wrench
torque
```

注意：`error`、`task_wrench` 等变量只在对应代码执行后才存在。断点应放在变量赋值行之后。

`.vscode/` 当前被项目 `.gitignore` 忽略，所以这些配置保存在本机，但不会自动提交到 Git。

## 3. 直接运行 Debug 脚本

```bash
/usr/bin/python3 examples/force_control_debug.py \
  --force 10 \
  --hold 1 \
  --debug-hz 20 \
  --log force_control_debug.csv
```

带 Viewer 和真实时间：

```bash
/usr/bin/python3 examples/force_control_debug.py \
  --force 10 --hold 1 --debug-hz 20 \
  --viewer --realtime \
  --log force_control_debug.csv
```

输出依次显示：

```text
sensor: raw_Fz → compensated_Fz → filtered_Fz
admittance: target → force_error → delta_z
cartesian: position_error → rotation_error
wrench_cmd_world: force → torque
joint torque: tau_1 ... tau_7
```

## 4. 在进入力控时停住

```bash
/usr/bin/python3 examples/force_control_debug.py \
  --force 10 --hold 1 \
  --break-on-force
```

进入 `FORCE_HOLD` 后会打开 Python debugger。可执行：

```text
p sample
p state
p state["force_error_n"]
p state["admittance_offset_m"]
p state["position_error"]
p state["task_wrench_world"]
p state["joint_torque_command"]
p robot.runtime.joint_positions
p robot.runtime.site_jacobian()
c
```

常用 debugger 命令：

- `n`：执行下一行；
- `s`：进入函数；
- `p expr`：打印表达式；
- `pp expr`：格式化打印；
- `c`：继续运行；
- `q`：退出。

## 5. 推荐断点

使用 IDE 或 `breakpoint()` 时，按以下顺序检查：

1. CLI 和参数入口：`src/cook_mujoco/cook_mujoco/chopping_cli.py:25`
2. 状态机入口：`src/cook_mujoco/cook_mujoco/chopping.py:174`
3. 接触判断：`src/cook_mujoco/cook_mujoco/chopping.py:257`
4. 单周期控制：`src/cook_mujoco/cook_mujoco/chopping.py:313`
5. 传感器原始数据：`src/cook_mujoco/cook_mujoco/control/force_control.py:217`
6. 去皮与重力补偿：`src/cook_mujoco/cook_mujoco/control/force_control.py:291`
7. 导纳和六维阻抗：`src/cook_mujoco/cook_mujoco/control/force_control.py:335`
8. MuJoCo 力矩写入：`src/cook_mujoco/cook_mujoco/control/force_control.py:220`
9. 模型传感器和 actuator：`src/cook_description/assets/robot/mujoco/chopping_scene.xml`

行号会随代码修改变化；优先按函数名搜索。

## 6. 关键变量解释

| 变量 | 含义 |
| --- | --- |
| `raw_input_wrench_tool` | 补偿后输入控制器的六维 wrench |
| `filtered_force_n` | 低通后的工具 Fz |
| `force_error_n` | `目标 Fz - filtered Fz` |
| `admittance_offset_m` | 导纳外环生成的 Z 位置偏移 |
| `desired_position` | 状态机给定的原始位置目标 |
| `effective_target_position` | 加入导纳偏移后的目标 |
| `position_error` | 六维阻抗中的平移误差 |
| `orientation_error` | 六维阻抗中的旋转向量误差 |
| `task_wrench_world` | 六维阻抗生成的世界系 wrench |
| `joint_torque_command` | Jacobian 转置、补偿和限幅后的 7 轴力矩 |

## 7. CSV 离线检查

```bash
python3 - <<'PY'
import pandas as pd

log = pd.read_csv("force_control_debug.csv")
print(log.groupby(["phase", "control_mode"])[
    ["wrench_2", "measured_force_n", "target_force_n"]
].agg(["min", "max", "mean"]))
PY
```

如果没有 pandas，可直接查看：

```bash
head -n 3 force_control_debug.csv
column -s, -t force_control_debug.csv | less -S
```

## 8. 判断问题属于哪一层

- `raw_wrench_2` 异常：检查 MJCF 传感器、接触模型和安装坐标；
- 原始正常、补偿异常：检查标定和工具质量；
- 补偿正常、`filtered_force_n` 延迟大：检查低通系数；
- 力误差正常、`delta_z` 不变化：检查导纳增益和限幅；
- `delta_z` 正常、位置误差大：检查阻抗刚度、关节限位和姿态可达性；
- wrench 正常、关节力矩异常：检查 Jacobian、奇异点和偏置力矩；
- 力矩正常、模型运动异常：检查 actuator、接触参数和 MuJoCo timestep。
