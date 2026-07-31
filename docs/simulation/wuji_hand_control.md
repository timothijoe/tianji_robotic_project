# Wuji Hand 左手控制说明

本文说明当前项目中左侧 Wuji Hand 的模型结构、关节顺序以及 MuJoCo
位置控制方法。这里的可运行代码只控制仿真模型，不会向实体灵巧手发送数据。

## 1. 模型概览

- 模型：原版 Wuji Hand 左手。
- 来源：`wuji-description/hand/body`。
- 项目内资产：`robot_assets/mujoco/wuji_hand/`。
- 许可证：MIT，见资产目录中的 `LICENSE`。
- 安装链：`left_link7 → left_hand_mount → left_palm_link`。
- 手部自由度：5 根手指 × 每指 4 个主动旋转关节，共 20 个。
- 完整场景：双臂 14 个执行器 + 左手 20 个执行器，共 34 个位置执行器。

模型只使用 `finger1` 至 `finger5` 的数字名称。上游文件没有给出足以确认的解剖学
名称，因此本文不擅自把它们标成拇指、食指等。

## 2. 关节顺序和控制范围

所有 20 维向量都采用“手指优先”的固定顺序：

```text
finger1 的 joint1..joint4,
finger2 的 joint1..joint4,
...
finger5 的 joint1..joint4
```

角度单位全部为弧度。下表是当前编译模型的 actuator 控制范围：

| 索引 | 分组 | 关节名称 | 执行器名称 | 最小值 | 最大值 |
|---:|---|---|---|---:|---:|
| 0 | finger1 | `left_finger1_joint1` | `left_finger1_joint1_actuator` | 0.0475 | 1.6030 |
| 1 | finger1 | `left_finger1_joint2` | `left_finger1_joint2_actuator` | -0.1387 | 0.9324 |
| 2 | finger1 | `left_finger1_joint3` | `left_finger1_joint3_actuator` | -0.4642 | 1.5620 |
| 3 | finger1 | `left_finger1_joint4` | `left_finger1_joint4_actuator` | -0.4699 | 1.5568 |
| 4 | finger2 | `left_finger2_joint1` | `left_finger2_joint1_actuator` | -0.1585 | 1.5600 |
| 5 | finger2 | `left_finger2_joint2` | `left_finger2_joint2_actuator` | -0.3700 | 0.3700 |
| 6 | finger2 | `left_finger2_joint3` | `left_finger2_joint3_actuator` | -0.4777 | 1.5480 |
| 7 | finger2 | `left_finger2_joint4` | `left_finger2_joint4_actuator` | -0.4683 | 1.5750 |
| 8 | finger3 | `left_finger3_joint1` | `left_finger3_joint1_actuator` | -0.1644 | 1.5516 |
| 9 | finger3 | `left_finger3_joint2` | `left_finger3_joint2_actuator` | -0.3700 | 0.3700 |
| 10 | finger3 | `left_finger3_joint3` | `left_finger3_joint3_actuator` | -0.4739 | 1.5510 |
| 11 | finger3 | `left_finger3_joint4` | `left_finger3_joint4_actuator` | -0.4684 | 1.5745 |
| 12 | finger4 | `left_finger4_joint1` | `left_finger4_joint1_actuator` | -0.1554 | 1.5580 |
| 13 | finger4 | `left_finger4_joint2` | `left_finger4_joint2_actuator` | -0.3700 | 0.3700 |
| 14 | finger4 | `left_finger4_joint3` | `left_finger4_joint3_actuator` | -0.4765 | 1.5487 |
| 15 | finger4 | `left_finger4_joint4` | `left_finger4_joint4_actuator` | -0.4777 | 1.5630 |
| 16 | finger5 | `left_finger5_joint1` | `left_finger5_joint1_actuator` | -0.1626 | 1.5580 |
| 17 | finger5 | `left_finger5_joint2` | `left_finger5_joint2_actuator` | -0.3700 | 0.3700 |
| 18 | finger5 | `left_finger5_joint3` | `left_finger5_joint3_actuator` | -0.4768 | 1.5490 |
| 19 | finger5 | `left_finger5_joint4` | `left_finger5_joint4_actuator` | -0.4683 | 1.5730 |

代码中的权威顺序定义在 `src/twin_sim/hand_names.py`。不要假定手执行器总是
位于 `data.ctrl[14:34]`；应使用 `sim.hand.actuator_ids` 按名称解析。

## 3. 内置姿态

自然张开姿态 `DEFAULT_OPEN_RAD`：

```python
np.array([
    0.15, 0.00, 0.05, 0.05,
    0.05, 0.00, 0.05, 0.05,
    0.05, 0.00, 0.05, 0.05,
    0.05, 0.00, 0.05, 0.05,
    0.05, 0.00, 0.05, 0.05,
])
```

演示使用的放松闭合姿态 `RELAXED_CLOSE_RAD`：

```python
np.array([
    0.80, 0.30, 0.80, 0.80,
    0.80, 0.00, 0.80, 0.80,
    0.80, 0.00, 0.80, 0.80,
    0.80, 0.00, 0.80, 0.80,
    0.80, 0.00, 0.80, 0.80,
])
```

这两个姿态都只是仿真示例，不代表抓取规划结果，也不是实体手的安全标定姿态。

## 4. 命令行仿真控制

无窗口运行完整的张开—闭合—张开动作：

```bash
.venv/bin/twin-sim hand-demo --headless
```

打开 Viewer 并慢速展示：

```bash
.venv/bin/twin-sim hand-demo --slow
```

慢速模式先保持张开 5 秒，再用 4 秒闭合、4 秒重新张开，最后停留 10 秒。
当前 Linux 图形环境中，动作展示完成后销毁 MuJoCo Viewer 偶尔会以退出码 139
结束；同一轨迹的 Headless 模式能够正常完成。

## 5. Python API：发送一个目标

下面的代码把完整的 20 维目标发送到仿真控制器，并推进 10 ms：

```python
import numpy as np

from twin_sim.robot import RightArmRobot
from twin_sim.tasks.hand_demo import RELAXED_CLOSE_RAD

robot = RightArmRobot(viewer=False)
try:
    target = np.asarray(RELAXED_CLOSE_RAD, dtype=float)
    robot.hand.command(target)
    robot.step(0.01)

    actual = robot.sim.data.qpos[robot.sim.hand.qpos_ids].copy()
    print(actual.shape)  # (20,)
    print(actual)
finally:
    robot.close()
```

### 控制调用的数据流

```text
20 维弧度目标
    ↓ robot.hand.command(target)
校验长度、有限性和每个关节范围，并保存目标
    ↓ robot.step(control_dt_s)
robot.hand.apply() 写入手部对应的 data.ctrl
    ↓ mujoco.mj_step(...)
推进 MuJoCo 动力学
    ↓ data.qpos[sim.hand.qpos_ids]
读取实际关节角
```

`command()` 本身不推进仿真时间，也不会立即改变实际关节位置。通常直接调用
`robot.step()` 即可，因为它会在每个控制周期自动调用 `hand.apply()`。只有在编写
更底层的仿真循环时，才需要单独调用 `apply()`。

## 6. 平滑发送

不要把相差很大的姿态一步跳变后当作推荐控制方式。下面用 2 秒、10 ms 控制周期
从张开姿态平滑移动到放松闭合姿态：

```python
import numpy as np

from twin_sim.hand import DEFAULT_OPEN_RAD
from twin_sim.robot import RightArmRobot
from twin_sim.tasks.hand_demo import RELAXED_CLOSE_RAD

control_dt_s = 0.01
duration_s = 2.0
steps = round(duration_s / control_dt_s)

robot = RightArmRobot(viewer=False)
try:
    start = DEFAULT_OPEN_RAD.copy()
    goal = RELAXED_CLOSE_RAD.copy()

    for fraction in np.linspace(0.0, 1.0, steps + 1)[1:]:
        target = start + fraction * (goal - start)
        robot.hand.command(target)
        robot.step(control_dt_s)

    actual = robot.sim.data.qpos[robot.sim.hand.qpos_ids].copy()
    print(actual)
finally:
    robot.close()
```

当前模型 timestep 为 0.002 秒，因此 `control_dt_s` 必须是 0.002 的正整数倍；
例如 0.002、0.01、0.02 均可，0.015 不可。

## 7. 读取状态

常用数据如下：

```python
# 当前保存的控制目标；返回副本，修改它不会修改控制器
target = robot.hand.target

# 实际 20 个手关节角，单位 rad
position = robot.sim.data.qpos[robot.sim.hand.qpos_ids].copy()

# 实际 20 个手关节速度，单位 rad/s
velocity = robot.sim.data.qvel[robot.sim.hand.dof_ids].copy()

# MuJoCo 中当前 20 个手执行器控制值
control = robot.sim.data.ctrl[robot.sim.hand.actuator_ids].copy()
```

不要写 `robot.hand.target[0] = ...` 来发送命令，因为 `target` 属性返回的是副本。
请构造完整的 20 维数组并调用 `robot.hand.command(...)`。

## 8. `LeftHandController` 接口

| 接口 | 行为 |
|---|---|
| `target` | 返回当前 20 维目标的副本 |
| `command(joints_rad)` | 完整校验后原子替换目标，不推进仿真 |
| `apply()` | 将当前目标写入这 20 个手执行器的 `data.ctrl` |
| `open()` | 把保存的目标恢复为 `DEFAULT_OPEN_RAD`，不推进仿真 |

“原子替换”表示 20 个值必须全部有效，控制器才会更新目标。任何一个值无效时，
原目标和 `data.ctrl` 都保持不变。

## 9. 输入错误和限制

以下输入会抛出 `ValueError`：

- 不是恰好 20 个值；
- 含有 NaN 或无穷大；
- 任一值超出对应 actuator 的控制范围；
- `step()` 的控制周期不是 0.002 秒的正整数倍。

当前控制器不提供：

- 单指或单关节的增量命令；
- 自动速度、加速度或 jerk 限制；
- 自碰撞规避和抓取规划；
- 触觉反馈或接触力闭环；
- 实体手通信。

如果只想改变一个关节，应先复制完整目标，再修改其中一项：

```python
target = robot.hand.target
target[4] = 0.30
robot.hand.command(target)
robot.step(0.01)
```

## 10. 未来实体手接入

当前项目和未来实体接口的边界如下：

| 能力 | 当前 MuJoCo 仿真 | 未来实体手 |
|---|---|---|
| 目标形状 | `(20,)`，弧度 | 预计保持 `(20,)`，弧度 |
| 顺序 | finger1_joint1 至 finger5_joint4 | 必须在启动时核对编码器顺序 |
| 发送入口 | `robot.hand.command()` + `robot.step()` | 尚未在本项目实现 |
| 实际位置 | MuJoCo `data.qpos` | 预计由编码器读取 |
| 底层排列 | 一维 20 元素 | Wuji SDK 参考使用 `(5, 4)` |
| 通信设备 | 无 | USB/驱动和实时控制器，尚未接入 |

下载的 `wuji-mjlab` 中可以看到一种参考抽象：

```python
write_target(joint_targets: np.ndarray)  # 输入 shape (20,)
read_encoders() -> np.ndarray            # 输出 shape (20,)
```

其 SDK 边界将一维目标 reshape 为 `(5, 4)` 后发送。不过这只是未来适配时可参考的
接口形状，不是本项目当前可调用的功能。本项目没有安装 `wujihandpy`，没有打开
实体设备，也没有验证电机使能、力矩上限、回零、通信频率或急停流程。

在接入实体手之前，必须单独完成：

1. 确认左手硬件与仿真的 20 关节顺序、正方向和零位。
2. 从硬件规范确定位置、速度、加速度、力矩和通信频率限制。
3. 实现平滑回零、使能/失能、超时、断连和急停流程。
4. 先在低速、低力矩限制下逐关节验证，不能直接复用仿真增益。

