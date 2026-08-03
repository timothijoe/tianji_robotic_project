# 04. 将转换后的 MCAP 安全回放到实体左手

本教程只处理已经完成“右手骨架 → 左手关节”转换的：

```text
*_right_to_left_wuji_hand.mcap
```

它不会读取原始 `/right_glove/hand_skeleton`；原始手套 MCAP 必须先按 [03_右手到左手与MuJoCo.md](03_右手到左手与MuJoCo.md) 转换并在 MuJoCo 中检查。

## 1. 工具与职责

```text
wuji-glove-recorder/replay_left_hand_mcap.py
wuji-glove-recorder/replay_left_hand_mcap.sh
```

工具通过 ROS 2 向：

```text
/hand_0/joint_commands
```

发布 20 个具名 `sensor_msgs/msg/JointState` 位置。它不直接使用 Wuji Hand SDK，也不启动驱动、不使能、不复位、不回零。

## 2. 内置安全检查

回放脚本默认是**只读预检**，必须显式加入 `--arm` 才会动作。预检会拒绝：

- 不存在或名称不含 `_right_to_left_wuji_hand` 的 MCAP；
- 没有 `/joint_states` 的文件；
- 不是完整 20 个左手关节名、重复名、非有限数；
- 非单调时间戳；
- 超出左手 MuJoCo MJCF 关节范围的值；
- 相邻帧任一关节变化超过 `--max-step-rad`（默认 `0.08 rad`）。

实体开始前还会：

1. 等待 `/hand_0/joint_states` 的真实反馈；
2. 从当前实际姿态缓慢插值到录制第一帧；
3. 按 MCAP 时间戳和 `--speed` 回放；
4. 反馈丢失或运行期发现异常时停止继续发布；
5. 结束或 `Ctrl+C` 后不发任何额外姿态，机械手保持最后目标。

这不是硬件急停，也不能替代现场监督。

## 3. 首次运行前的准备

1. 用 [01_机械手连接与控制.md](01_机械手连接与控制.md) 启动驱动；
2. 检查 `/hand_0/joint_states` 和 `/hand_0/hand_diagnostics`；
3. 用 `wave_demo.py` 验证实体手确实响应 ROS 命令；
4. 在 MuJoCo 检查同一 MCAP/NPZ；
5. 清空手周围，保持人工监看和硬件急停可用。

## 4. 只读预检

进入工具目录：

```bash
cd ~/catkin_robotic_ws/august_ws/wuji-technology/wuji-glove-recorder
```

对当前样例进行预检：

```bash
sh replay_left_hand_mcap.sh \
  /tmp/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap
```

成功时会打印帧数、录制时长、每个关节范围、第一帧和最大帧间变化，并提示：

```text
Preflight passed. Add --arm only when the physical hand is ready.
```

当前样例已验证为 499 帧、4.150 秒、最大帧间变化 `0.02534 rad`。

## 5. 实体回放命令

首次用 5% 速度和 5 秒缓入：

```bash
sh replay_left_hand_mcap.sh --arm --speed 0.05 --ramp-seconds 5 \
  /tmp/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap
```

参数说明：

| 参数 | 默认 | 作用 |
| --- | --- | --- |
| `--arm` | 缺省关闭 | 必须显式提供，否则只预检 |
| `--speed` | `0.1` | 回放速度，范围 `(0, 1]`；0.2 即原始录制速度 20% |
| `--ramp-seconds` | `3.0` | 当前手姿到第一帧的缓入时间 |
| `--max-step-rad` | `0.08` | 允许的最大相邻单关节变化 |
| `--hand-name` | `hand_0` | ROS 接口前缀 |
| `--state-timeout` | `5.0` | 等待初始状态反馈时间 |

例如确认稳定后以 20% 回放：

```bash
sh replay_left_hand_mcap.sh --arm --speed 0.2 --ramp-seconds 5 \
  /tmp/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap
```

程序刻意不允许超过 `1.0` 倍，也不提供跳过限位/强制执行参数。

## 6. 回放没有动作时怎么办

曾遇到过“命令 Topic 有目标、驱动 enabled/error code 正常，但手完全不动”。处理流程：

1. 停止回放，不要反复发送同一 MCAP；
2. 重启 `wujihand.launch.py`；
3. 运行官方 `wave_demo.py`；
4. 只有确认 wave demo 让实体手实际运动后，再回到本脚本。

此前正是通过此流程恢复基础执行链路。这个问题说明 Topic 可见、诊断无错误并不足以证明执行器在响应。

## 7. 绝对不要做的事

- 不要对原始右手手套 MCAP 直接加 `--arm`；
- 不要在 Studio/SDK/ROS 多个程序同时直接控制实体手；
- 不要因为驱动显示 enabled 就跳过 MuJoCo 和低速首测；
- 不要把 `Ctrl+C` 当紧急停止；
- 不要修改设备 IP 来解决实体手回放问题；
- 不要使用早期未镜像的 `*_left_wuji_hand.mcap`。

## 8. 每次实体回放的标准操作卡

按顺序逐项勾选；任何一项失败都停止在该项排查。

- [ ] 驱动终端仍在运行，且日志显示 `Connected to WujiHand (left)`。
- [ ] `/hand_0/joint_states` 有 20 个实时位置。
- [ ] `/hand_0/hand_diagnostics` 的 error code 都是 0，enabled 都是 true。
- [ ] 官方 `wave_demo.py` 已被观察到能够实际运动，且现已停止。
- [ ] 输入文件名含 `_right_to_left_wuji_hand.mcap`。
- [ ] 已运行无 `--arm` 预检，人工看过首帧、范围与最大变化。
- [ ] MuJoCo 已查看同一轨迹，动作方向合理。
- [ ] 现场无人接触机械手，硬件急停/断电方式清楚。
- [ ] 首次速度使用 0.05，缓入至少 5 秒。

通过后才执行：

```bash
sh replay_left_hand_mcap.sh --arm --speed 0.05 --ramp-seconds 5 \
  /绝对路径/xxx_right_to_left_wuji_hand.mcap
```

## 9. 回放期间的观察点

- 开始的 `ramp-seconds` 时间内，手应从当前姿态平滑过渡；若出现突跳，立即停止并回到 MuJoCo/关节范围检查。
- 手指应与 MuJoCo 中同一段动作有相同的大致屈伸趋势；机械模型外观不同是允许的。
- 若手不动，先停止回放并走第 6 节恢复流程；不要直接提高 `--speed`。
- 若方向反了或某根手指明显不对，停止回放，保留 MCAP/NPZ/截图，回查右转左 Y 镜像和关节顺序。

## 10. 回放脚本依赖与运行环境

`replay_left_hand_mcap.sh` 会：

1. 保留当前终端的 ROS `PYTHONPATH`，以便系统 `python3` 导入 `rclpy`；
2. 补充 `wuji-teleop-venv` 中的 `mcap` 包；
3. 启动 `replay_left_hand_mcap.py`。

因此运行实体回放前，终端至少应能找到 ROS：

```bash
source /opt/ros/jazzy/setup.bash
source ~/catkin_robotic_ws/august_ws/wuji-technology/wujihandros2/install/setup.bash
```

如果脚本提示无法导入 `rclpy`，说明当前 shell 没有 source ROS 环境；如果提示无法导入 `mcap`，检查 `/home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv` 是否仍存在。

## 11. SDK 直控版本

SDK 版本为 `replay_left_hand_mcap_sdk.sh`。它与 ROS 版互斥：必须先停止 ROS 驱动，SDK 版会检查到 `/hand_0/wujihand_driver` 或 `wujihand_driver_node` 后拒绝启动。无 `--arm` 时仍只预检：

```bash
sh replay_left_hand_mcap_sdk.sh /tmp/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap
```

SDK 版通过官方 `SdkManager`、`LowPass` 和 20 个 `JointCommand` 控制左手；退出时会关闭发布器、禁用手并断开 SDK。因此 SDK 回放结束后若要继续使用 ROS，必须重新启动 ROS driver。

两个 `.sh` 文件在同一目录中，但它们是不同入口：`replay_left_hand_mcap.sh` 是 ROS 版，`replay_left_hand_mcap_sdk.sh` 是 SDK 版。本次 SDK 实测使用后者。SDK 脚本仅复用了前者的离线 MCAP 解析/预检函数，不会启动 ROS 节点或发布 ROS Topic。

已于 2026-08-02 以样例 MCAP、`--arm --speed 0.05` 实机完整验证：SDK 扫描到左手 SN `LQSQJL.260706.001`、固件 `1.2.1`，回放进程以退出码 0 结束。SDK 的 `handedness_name()` 规范返回值是 `"Left"`（首字母大写）；不要写成小写 `"left"`，否则程序会在使能前安全拒绝。安全检查还会忽略回放程序自身的 PID，但会拒绝 ROS 驱动或另一份 SDK 回放进程。

## 12. 两种实体回放模式的退出与切换

ROS 和 SDK 的完整控制边界见 [01_机械手连接与控制.md](01_机械手连接与控制.md) 第 12 节。摘要如下：

- ROS 回放终端的 `Ctrl+C` 只停止 Topic 发布；还要在 ROS launch 终端 `Ctrl+C`，才能释放 driver 对机械手的占有。
- SDK 回放会在自然结束或普通 `Ctrl+C` 时关闭 publisher、禁用手并断开；不要使用 `kill -9`，它会跳过清理。
- 两种模式都不自动回零；最后姿态可能保持。异常危险时应使用硬件急停/断电。
- ROS → SDK 前检查 `ros2 node list` 中不存在 `/hand_0/wujihand_driver`；SDK → ROS 前确认 SDK 回放进程已退出，然后再启动 ROS driver。
