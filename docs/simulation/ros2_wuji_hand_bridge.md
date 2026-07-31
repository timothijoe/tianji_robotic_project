# ROS 2 与 Wuji Hand 仿真桥接

本文说明如何通过 ROS 2 Humble 或类似 `wujihandpy` 的 Python API 控制
MuJoCo 左手。当前链路只连接仿真，不会搜索、使能或写入实体电机。

## 1. 当前安装状态

- 操作系统：Ubuntu 22.04，ROS 2 Humble 安装于 `/opt/ros/humble`。
- 仿真开发环境：项目根目录 `.venv`，Python 3.12。
- ROS 仿真环境：项目根目录 `.venv-ros2`，系统 Python 3.10、
  `--system-site-packages`、MuJoCo 3.10.0。
- 厂商 SDK 环境：项目根目录 `.venv-wujihand`，Python 3.12、
  `wujihandpy 1.8.0`。
- ROS 消息源码：`/home/linux/august_folder/wujihandros2/wujihand_msgs`。

安装 ROS 时发现无关的 ToDesk apt 源已经失效，因此将
`/etc/apt/sources.list.d/todesk.list` 可恢复地改名为
`todesk.list.disabled`。ROS 官方源仍位于
`/etc/apt/sources.list.d/ros2.list`。

当前机器未检测到 USB VID/PID `0483:2000`。只执行 SDK 构造检查时，
`wujihandpy.Hand(side="left")` 按预期返回
`ConnectionError: No device found`；这说明 SDK 已安装，但不代表硬件异常。

## 2. 重建两个隔离环境

ROS 2 Humble 的 `rclpy` 面向 Ubuntu 的 Python 3.10，不能直接装入项目的
Python 3.12 环境。请保持两个环境分离：

```bash
/usr/bin/python3 -m venv --system-site-packages .venv-ros2
.venv-ros2/bin/python -m pip install mujoco==3.10.0

python3.12 -m venv .venv-wujihand
.venv-wujihand/bin/python -m pip install wujihandpy
```

验证：

```bash
source /opt/ros/humble/setup.bash
.venv-ros2/bin/python -c \
  "import rclpy, mujoco, numpy; print(mujoco.__version__, numpy.__version__)"
.venv-wujihand/bin/python -c \
  "import importlib.metadata as m; print(m.version('wujihandpy'))"
```

## 3. 构建 ROS 工作区

只构建上游 `wujihand_msgs` 和本项目仿真节点，不构建或启动 USB 硬件驱动：

```bash
source /opt/ros/humble/setup.bash
export PATH="$PWD/.venv-ros2/bin:/usr/bin:/bin"
python -m colcon --log-base ros2_ws/log build \
  --base-paths \
    /home/linux/august_folder/wujihandros2/wujihand_msgs \
    ros2_ws/src/twin_wuji_sim \
  --build-base ros2_ws/build \
  --install-base ros2_ws/install \
  --symlink-install \
  --cmake-args -DPython3_EXECUTABLE="$PWD/.venv-ros2/bin/python"
```

显式设置 `PATH` 和 `Python3_EXECUTABLE` 很重要；否则 Anaconda Python 可能被
CMake 误选，导致 ROS 消息生成器与 Python ABI 不匹配。

## 4. 启动 MuJoCo 桥接

每个新终端先执行：

```bash
cd /home/linux/august_folder/tianji_robotic_project
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export PYTHONPATH="$PWD/src:$PYTHONPATH"
```

Headless 启动：

```bash
ros2 launch twin_wuji_sim sim_hand.launch.py viewer:=false
```

打开 MuJoCo Viewer：

```bash
ros2 launch twin_wuji_sim sim_hand.launch.py viewer:=true
```

可用 launch 参数：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `viewer` | `false` | 是否打开 MuJoCo Viewer |
| `hand_name` | `hand_left` | ROS namespace |
| `publish_rate` | `100.0` | 仿真控制和状态发布频率，Hz |
| `start_enabled` | `true` | 仿真关节初始逻辑使能状态 |

100 Hz 对应每周期 0.01 秒；当前 MuJoCo timestep 为 0.002 秒，因此每个 ROS
周期推进 5 个物理 substep。

## 5. Topic 和 service

默认接口：

| 名称 | 类型 | 方向 |
|---|---|---|
| `/hand_left/joint_commands` | `sensor_msgs/msg/JointState` | 订阅 |
| `/hand_left/joint_states` | `sensor_msgs/msg/JointState` | 发布 |
| `/hand_left/hand_diagnostics` | `wujihand_msgs/msg/HandDiagnostics` | 发布 |
| `/hand_left/set_enabled` | `wujihand_msgs/srv/SetEnabled` | service |

读取状态：

```bash
ros2 topic echo /hand_left/joint_states --once
```

按名称只移动一个关节：

```bash
ros2 topic pub --once /hand_left/joint_commands sensor_msgs/msg/JointState \
  "{name: ['left_finger1_joint2'], position: [0.25]}"
```

也可以令 `name: []` 并提供恰好 20 个 position，按 finger-major 顺序发送完整
目标。未知名称、重复名称、数量不匹配、NaN/Inf 和越界目标会整条原子拒绝。

全手逻辑失能和重新使能：

```bash
ros2 service call /hand_left/set_enabled wujihand_msgs/srv/SetEnabled \
  "{finger_id: 255, joint_id: 255, enabled: false}"
ros2 service call /hand_left/set_enabled wujihand_msgs/srv/SetEnabled \
  "{finger_id: 255, joint_id: 255, enabled: true}"
```

`finger_id` 为 0–4，`joint_id` 为 0–3；任一字段的 255 表示全部。这里的使能
只是仿真命令门控，不代表实体电机上电。

启动保守的张开/放松闭合演示：

```bash
ros2 run twin_wuji_sim demo --ros-args -r __ns:=/hand_left
```

## 6. Python 仿真 SDK 接口

`SimWujiHand` 使用厂商 SDK 风格的 `(5, 4)` 数组，但不会遮蔽或伪造顶层
`wujihandpy` 包：

```python
from twin_sim.wuji_hand_backend import SimWujiHand

hand = SimWujiHand(viewer=False)
try:
    target = hand.read_joint_target_position()
    target[0, 1] = 0.25
    hand.write_joint_target_position(target)
    hand.step(0.01)
    print(hand.read_joint_actual_position())
finally:
    hand.close()
```

支持实际/目标位置、逻辑使能、推进仿真和 realtime controller 的位置/力读取
子集。模拟 effort 来自 MuJoCo actuator force，未经实体标定，不能当作真实扭矩。
温度等未模拟的厂商接口会显式抛出 `NotImplementedError`。

## 7. 自动验证

```bash
.venv/bin/pytest tests/simulation tests/ros2/test_bridge_validation.py -q

source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export PYTHONPATH="$PWD/src:$PYTHONPATH"
.venv-ros2/bin/python tests/ros2/ros2_bridge_smoke.py
```

端到端测试会启动 Headless 桥接、校验 20 个名称、发送命令、观察实际位置朝目标
变化，并验证失能时拒绝新命令，最后清理桥接进程。

## 8. 明天接实体手的安全顺序

当前不启动 `wujihand_driver`，也不把仿真目标直接转发给 USB。实体调试按以下
门控顺序进行：

1. 机械断电或电机保持失能，只检查 `lsusb` 是否出现 `0483:2000`、序列号和左右手。
2. 只读位置、固件、温度、电压、错误码和现有 effort limit，并保存日志。
3. 逐项核对实体与仿真的 20 关节顺序、零位、单位和正方向。
4. 确认急停、通信超时、断连失能和软件 `finally` 失能路径。
5. 降低速度、幅度和 effort limit，只使能一个关节，发送远离机械限位的小幅目标。
6. 确认方向正确后逐关节扩大覆盖；任何方向、温度、电流或错误码异常立即失能。
7. 完成上述检查后，才考虑切换到上游硬件 ROS 驱动。

实体首测不能直接运行仿真 demo，也不能假设 MuJoCo 的 effort、零位或范围等同于
硬件标定值。
