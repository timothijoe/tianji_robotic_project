# 六维力传感器、第七轴安装关系与控制模式说明

## 1. 六维力传感器数据读取

当前剁切场景包含一组 MuJoCo 力/力矩传感器：

```xml
<force name="tool_force" site="force_sensor_site" />
<torque name="tool_torque" site="force_sensor_site" />
```

两者组合为工具坐标系下的六维 wrench：

```text
[Fx, Fy, Fz, Mx, My, Mz]
```

单位为：

- `Fx, Fy, Fz`：牛顿（N）；
- `Mx, My, Mz`：牛顿米（N·m）。

### 1.1 Python 接口

```python
raw = robot.get_raw_wrench()
wrench = robot.get_wrench()
```

含义：

- `get_raw_wrench()`：MuJoCo 传感器原始六维数据；
- `get_wrench()`：完成静态去皮和工具重力补偿后的六维数据。

返回顺序均为：

```text
[Fx, Fy, Fz, Mx, My, Mz]
```

### 1.2 CSV 字段

运行日志中包含：

```text
raw_wrench_0 ... raw_wrench_5
wrench_0 ... wrench_5
```

字段对应关系：

| 字段 | 数据 |
| --- | --- |
| `*_0` | Fx |
| `*_1` | Fy |
| `*_2` | Fz |
| `*_3` | Mx |
| `*_4` | My |
| `*_5` | Mz |

其中：

- `raw_wrench_*` 为原始值；
- `wrench_*` 为补偿后值；
- Z 轴力控制主要使用补偿后的 `Fz`；
- 安全峰值和 30 N 保护使用真实补偿传感器值，不使用低通结果掩盖冲击。

### 1.3 终端实时显示

CLI 会显示：

```text
F=(Fx,Fy,Fz)N
M=(Mx,My,Mz)Nm
Fz_ctrl=...
target_Fz=...
```

其中：

- `F` 和 `M` 为当前补偿后的六维传感器读数；
- `Fz_ctrl` 为力控制器使用的低通反馈；
- `target_Fz` 为当前目标力。

## 2. 传感器与第七轴的安装关系

当前模型运动链为：

```text
Joint6_L
→ Link6_L
→ Joint7_L
→ Link7_L
→ force_sensor_body
→ force_sensor_site
→ tool_body
→ tool_tip_site
```

因此传感器和工具在拓扑上安装于 `Joint7_L` 之后，不是安装在第六轴之后。

对应 MJCF 结构：

```xml
<body name="Link7_L">
  <joint name="Joint7_L" ... />
  <geom mesh="mesh_0014" ... />
  <body name="force_sensor_body">
    <site name="force_sensor_site" ... />
    <body name="tool_body">
      ...
    </body>
  </body>
</body>
```

五次剁切日志中，第七轴的运动范围约为：

```text
0.2005 rad ≈ 11.5°
```

所以第七轴确实参与了笛卡尔阻抗控制，只是它主要绕工具轴旋转，从 Viewer 中不容易观察。

### 2.1 真实法兰固定变换

截图检查发现，早期版本虽然把工具放在 `Joint7_L` 的子树中，但错误地安装在第七轴关节原点，即红色关节位置，而不是绿色末端法兰。

随后从厂商 `ccs_m6_40.MvKDCfg` 和 `kinematicsSDK/FxRobot.cpp` 中提取出精确参数：

```text
DH[7] = [90°, 0, 95 mm, 90°]
Flan = DH[7][2] = 95 mm
```

厂商法兰矩阵不能直接作为 URDF `Link7_L` 局部变换。通过三组不同关节角对比厂商 FK 与 MuJoCo Link7 位姿，反求得到恒定变换：

```text
translation = [0, -0.095, 0] m
rotation =
[0 -1  0
 0  0 -1
 1  0  0]
```

对应 MuJoCo 四元数（wxyz）为：

```text
[0.5, 0.5, -0.5, 0.5]
```

当前 `force_sensor_body` 已按反求固定变换安装，并增加实体传感器/转接盘连接绿色法兰与工具。剁切场景将机器人底座抬高 90 mm，使正确法兰下的工具能够近似垂直向下，并在砧板上方保留约 81 mm 安全间距。

## 3. 笛卡尔阻抗与力控模式切换

当前控制器包含两个可观察模式。

### 3.1 `CARTESIAN_IMPEDANCE`

完整六维笛卡尔阻抗控制：

- X、Y、Z 位置执行阻抗跟踪；
- 工具姿态执行旋转阻抗跟踪；
- 输出通过 Jacobian 转置映射为七个关节力矩。

典型阶段：

```text
APPROACH
DESCEND
RETRACT
SHIFT
```

### 3.2 `HYBRID_FORCE_Z`

混合位置/力控制：

- X、Y 方向继续使用笛卡尔阻抗；
- 工具姿态继续使用旋转阻抗；
- 工具 Z 轴切换为恒力闭环；
- 最终仍输出七个关节力矩。

典型阶段：

```text
FORCE_HOLD
```

### 3.3 切换过程

典型终端输出：

```text
phase=DESCEND    mode=CARTESIAN_IMPEDANCE
phase=FORCE_HOLD mode=HYBRID_FORCE_Z
phase=RETRACT    mode=CARTESIAN_IMPEDANCE
```

切换逻辑为：

1. `DESCEND` 阶段使用笛卡尔阻抗缓慢搜索接触；
2. 真实补偿 `Fz` 超过接触阈值；
3. 进入 `FORCE_HOLD`；
4. Z 轴切换为目标恒力控制；
5. 保持完成后关闭力控；
6. `RETRACT` 恢复完整笛卡尔阻抗。

CSV 中的 `control_mode` 字段记录每个控制周期的模式。

Python 中可以读取：

```python
mode = robot.control_mode
```

返回值为：

```text
CARTESIAN_IMPEDANCE
```

或：

```text
HYBRID_FORCE_Z
```

## 4. 推荐运行命令

```bash
cd /home/zhoutong/catkin_robotic_ws/cook_proj/cook_ws
source install/setup.bash

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

参数说明：

- `--status-hz 10`：每秒显示十次六维传感器和模式信息；
- `--status-hz 0`：关闭实时终端状态；
- `--log chopping.csv`：保存六维力、模式、轨迹和关节数据。

## 5. 当前验证结果

五次剁切测试结果：

```text
真实补偿力峰值：约 14.5 N
力控低通反馈峰值：约 10.1 N
非接触轨迹 RMSE：约 4.4 mm
五次剁切：完成
安全保护：未触发
```

当前测试结果：

```text
50 passed, 6 skipped
```
