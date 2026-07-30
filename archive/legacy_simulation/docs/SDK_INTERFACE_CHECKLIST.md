# SDK 同名接口速查

目标：用尽量同一套 Python 调用，同时控制 MuJoCo 和真机。

统一入口：

```python
from twin_control.sdk_compat import create_ik_param, create_kine, create_robot

robot = create_robot(backend, arm="B", sdk_root=sdk_root)
kine = create_kine(backend, arm_type=1, sdk_root=sdk_root)
sp = create_ik_param(backend, sdk_root=sdk_root)
```

其中：

```text
backend="mujoco" -> 本地 MuJoCo 兼容实现
backend="real"   -> 厂商 SDK
```

## 1. Robot 控制接口

这些函数在 MuJoCo backend 和真机 `Concise_Marvin_Robot` 里基本同名。

| 函数 | 输入 | 输出 | MuJoCo 实现 | 真机 SDK 对应 | 是否可统一调用 |
|---|---|---|---|---|---|
| `connect(robot_ip, log_switch=0)` | `robot_ip: str`，`log_switch: int` | `bool` | 加载 MuJoCo 模型，初始化 `TwinRobot` | 连接控制柜 IP | 可以 |
| `release_robot()` | 无 | `bool` 或状态码 | 关闭仿真/viewer | 断开真机连接 | 可以 |
| `set_position_state(arm, velRatio, AccRatio)` | `arm: "A"/"B"`，速度/加速度百分比 `0..100` | `bool` | 进入 position mode | 进入关节位置模式 | 可以 |
| `set_imp_joint_state(arm, velRatio, AccRatio, K, D)` | `K: 7`，`D: 7` | `bool` | 进入 joint impedance | 进入关节阻抗 | 可以 |
| `set_imp_cart_state(arm, velRatio, AccRatio, K, D, rot_type, cart_ctrl_para)` | `K: 7`，`D: 7`，`rot_type: int`，`cart_ctrl_para: 7` | `bool` | 进入 cartesian impedance | 进入笛卡尔阻抗 | 可以 |
| `set_imp_force_state(arm, fx_dir, fc_adj_lmt)` | `fx_dir: 6`，`fc_adj_lmt: float` | `bool` | 进入 force mode | 进入力控阻抗 | 可以，但第一版先不用 |
| `set_joint_position_cmd(arm, joint)` | `joint: 7`，单位 degree | `bool` | 发送 joint target；cart impedance 下会 FK 成 TCP target | 发送关节目标 | 可以 |
| `set_force_cmd(arm, force)` | `force: float`，单位 N | `bool` | 设置目标力 | 设置目标力 | 可以，但第一版先不用 |
| `subscribe(dcss=None)` | 可不传；真机 adapter 内部持有 `DCSS()` | `dict` 或 `None` | 返回仿真反馈 dict | 返回真机反馈 dict | 可以 |
| `disable(arm)` | `arm: "A"/"B"` | `bool` | disable 本地控制器 | 下使能/复位 | 可以 |

## 2. Robot 反馈字段

`subscribe()` 返回的 dict 里，MuJoCo 和真机都尽量使用这些字段：

| 字段 | 含义 | 单位 |
|---|---|---|
| `data["states"][i]["cur_state"]` | 当前模式/状态 | SDK 状态码 |
| `data["states"][i]["err_code"]` | 错误码 | int |
| `data["outputs"][i]["fb_joint_pos"]` | 反馈关节位置 | degree |
| `data["outputs"][i]["fb_joint_vel"]` | 反馈关节速度 | degree/s |
| `data["outputs"][i]["fb_joint_cmd"]` | 当前/最近关节命令 | degree |
| `data["outputs"][i]["fb_joint_sToq"]` | 关节力矩反馈 | Nm 或 SDK 原始定义 |
| `data["outputs"][i]["est_cart_fn"]` | 末端估计力/力矩 | N / Nm |
| `data["outputs"][i]["low_speed_flag"]` | 低速/停止标志 | bytes / int |
| `data["outputs"][i]["traj_state"]` | 轨迹状态 | SDK 状态 |
| `data["inputs"][i]["joint_cmd_pos"]` | 输入侧关节命令 | degree |
| `data["inputs"][i]["joint_vel_ratio"]` | 输入侧速度比例 | 0..100 |
| `data["inputs"][i]["joint_acc_ratio"]` | 输入侧加速度比例 | 0..100 |

`i=0` 是 A 臂，`i=1` 是 B 臂。

注意：厂商原始 `Concise_Marvin_Robot.subscribe()` 不能传 `None`，需要：

```python
from SDK_PYTHON.fx_robot import DCSS

dcss = DCSS()
data = robot.subscribe(dcss)
```

现在 `create_robot("real")` 已经返回 `RealSdkRobotAdapter`，内部自动持有 `DCSS()`。业务脚本可以统一写：

```python
data = robot.subscribe(None)
```

## 3. Kinematics 运动学接口

这些函数在 MuJoCo `MujocoKine` 和真机 `Marvin_Kine` 中基本同名。

| 函数 | 输入 | 输出 | MuJoCo 实现 | 真机 SDK 对应 | 是否可统一调用 |
|---|---|---|---|---|---|
| `load_config(arm_type, config_path)` | `arm_type: 0/1`，配置文件路径 | `dict` | 返回兼容形状的 placeholder | 读取 `.MvKDCfg` | 可以，但 MuJoCo 不真解析配置 |
| `initial_kine(robot_type, dh, pnva, j67)` | config 中的参数 | `bool` | no-op，返回 True | 初始化厂商运动学 | 可以 |
| `fk(joints)` | `joints: 7`，degree | `4x4 pose`，位置 mm | MuJoCo site FK | 厂商 FK | 可以 |
| `ik(sp)` | `FX_InvKineSolvePara` | IK 结果结构体 | MuJoCo 数值 IK | 厂商 IK | 可以，配合 `create_ik_param()` |
| `joints2JacobMatrix(joints)` | `joints: 7`，degree | `6x7` Jacobian | MuJoCo Jacobian | 厂商 Jacobian | 可以 |
| `mat4x4_to_mat1x16(pose_mat)` | `4x4 pose` | 长度 16 list | 本地转换 | 厂商转换 | 可以 |
| `mat4x4_to_xyzabc(pose_mat)` | `4x4 pose`，位置 mm | `[x,y,z,a,b,c]`，mm/degree | 本地转换 | 厂商转换 | 可以 |
| `xyzabc_to_mat4x4(xyzabc)` | `[x,y,z,a,b,c]`，mm/degree | `4x4 pose` | 本地转换 | 厂商转换 | 可以 |
| `set_tool_kine(tool_mat)` | `4x4 tool transform` | `bool` | 设置本地 TCP offset | 设置厂商 tool kine | 可以，第一版先不用 |
| `remove_tool_kine()` | 无 | `bool` | 清除本地 TCP offset | 清除厂商 tool kine | 可以，第一版先不用 |

## 4. IK 参数结构

MuJoCo 和真机的 IK 参数结构体类型不同，现在统一通过 `create_ik_param()` 创建：

```python
sp = create_ik_param(backend, sdk_root=sdk_root)
```

两边方法名基本一致：

| 方法 | 输入 | 输出 |
|---|---|---|
| `set_input_ik_target_tcp(matrix)` | 长度 16，4x4 pose 展平，位置 mm | `None` |
| `set_input_ik_ref_joint(values)` | 7 维参考关节，degree | `None` |
| `set_input_ik_zsp_type(value)` | `int` | `None` |
| `set_input_ik_zsp_para(values)` | 6 维参数 | `None` |
| `set_input_zsp_angle(value)` | degree | `None` |
| `get_output_ret_joint()` | 无 | 7 维 IK 结果，degree |
| `get_output_result_num()` | 无 | IK 结果数量 |
| `get_output_is_out_range()` | 无 | 是否超可达范围 |
| `get_output_is_jnt_exd()` | 无 | 是否超关节限制 |
| `get_output_jnt_exd_tags()` | 无 | 7 维关节限制标志 |

这样业务代码不用关心本地 dataclass 还是真机 ctypes Structure。

## 5. 第一版建议只用这些函数

为了先把 MuJoCo 和真机都跑通，第一版起伏切菜实验只用：

```text
create_robot
create_kine
create_ik_param
connect
release_robot
set_position_state
set_imp_cart_state
set_joint_position_cmd
subscribe
load_config
initial_kine
fk
ik
mat4x4_to_mat1x16
```

暂时不用：

```text
force control
PVT
RunPlnJoint / RunPlnCart
tool dynamic identification
485/tool communication
data collection SDK
```

## 6. 当前结论

大部分核心控制函数已经同名。`RealSdkRobotAdapter` 和 `create_ik_param()` 已补上，起伏切菜实验脚本可以基本写成：

```python
robot = create_robot(backend, arm="B", sdk_root=sdk_root)
kine = create_kine(backend, arm_type=1, sdk_root=sdk_root)
sp = create_ik_param(backend, sdk_root=sdk_root)
```

后面的控制逻辑保持一致。
