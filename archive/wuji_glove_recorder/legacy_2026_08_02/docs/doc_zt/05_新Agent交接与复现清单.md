# 05. 新 Agent 交接与复现清单

本文件面向“重新打开一个 Agent 会话后，希望不重新踩坑”的场景。按顺序读取和执行，不要把离线可视化、手套通信和实体手控制混为一件事。

## A. 先读哪些文件

1. 本文件；
2. [01_机械手连接与控制.md](01_机械手连接与控制.md)；
3. [02_Wuji_Studio与手套教程.md](02_Wuji_Studio与手套教程.md)；
4. [03_右手到左手与MuJoCo.md](03_右手到左手与MuJoCo.md)；
5. [04_MCAP实体左手回放.md](04_MCAP实体左手回放.md)；
6. 如需了解理由与历史证据，再读 [问题与决策日志.md](问题与决策日志.md)。

## B. 已知、经过实际验证的状态

| 项目 | 当前已验证结论 |
| --- | --- |
| 实体手 | 左手，ROS 接口 `/hand_0/*`，SDK 1.8.0，固件 1.2.1 |
| 手套 | 右手，`192.168.10.151:50001`，SN `WG1KA03260510008`，固件 0.10.1 |
| 主机手套网段 | 曾使用 `enp129s0 = 192.168.10.10/24` |
| Studio 与 SDK | 不能同时连同一手套，冲突错误为 `Session already exists` |
| 右转左规则 | 关键点进入 Left retargeter 前必须 `Y *= -1` |
| 左手模型 | `mujoco-sim/wuji_hand_description/mjcf/left.xml` |
| 实体回放输入 | 只能使用 `*_right_to_left_wuji_hand.mcap` 的 `/joint_states` |
| 实体回放安全默认 | 先预检；`--arm` 才动作；速度默认 0.1，首次推荐 0.05；SDK 直控已以样例文件完整实测并正常退出 |
| SDK 实测问题与修复 | 冲突检查须排除自身 PID；SDK 左手返回值严格为 `Left`，不是 `left`；修复后 28 项测试通过 |

## C. 不要重新做的错误尝试

1. 不要把右手 21 点原始骨架直接交给 Left retargeter；会造成末端关节异常过弯。
2. 不要让 Studio 和独立 SDK 同时连接手套。
3. 不要随意改手套 IP；让主机加入当前网段。
4. 不要依赖 Studio Hand Skeleton 自动显示左手 `/joint_states` MCAP；用 MuJoCo。
5. 不要在 ROS 驱动运行时开 SDK 直接控制实体手。
6. 不要跳过 MuJoCo 就直接 `--arm`。
7. 不要仅因 `enabled=true`、`error=0` 就相信实体手可动；先跑官方 wave demo。
8. 不要对手不动的问题反复加速或重复发送轨迹；先重启驱动并做基础 demo。

## D. 典型工作流

### 采集新动作

```text
配置手套网络 → Studio 显示右手 → Studio 录制原始 MCAP → 保存原始文件
```

### 离线验证

```text
修改 visualize_right_glove_to_left.sh 的 DEFAULT_MCAP_PATH
→ sh visualize_right_glove_to_left.sh
→ MuJoCo 观察
```

### 实体回放（SDK 直控，当前推荐）

```text
停止 ROS 驱动 → SDK MCAP 预检 → --arm --speed 0.05
→ 程序结束时自动 disable/disconnect
```

SDK 直控时 `handedness_name()` 的正确左手值为 `Left`，不可写成 `left`。若要改用 ROS 回放，先确保 SDK 回放已结束，再启动 ROS 驱动。

## E. 关键绝对路径

```text
工程根目录:
/home/zhoutong/catkin_robotic_ws/august_ws/wuji-technology

离线工具目录:
/home/zhoutong/catkin_robotic_ws/august_ws/wuji-technology/wuji-glove-recorder

ROS 驱动目录:
/home/zhoutong/catkin_robotic_ws/august_ws/wuji-technology/wujihandros2

SDK Python:
/home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python

MuJoCo Python:
/home/zhoutong/catkin_robotic_ws/august_ws/tianji_robotic_project/.venv/bin/python

当前实体回放样例:
/tmp/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap
```

## F. 新 Agent 的第一轮只读检查命令

这些命令不会移动手：

```bash
cd ~/catkin_robotic_ws/august_ws/wuji-technology
rg --files doc_zt wuji-glove-recorder wujihandros2 | head -n 80

cd wuji-glove-recorder
sh replay_left_hand_mcap.sh \
  /tmp/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap

cd ../wujihandros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo /hand_0/joint_states --once
ros2 topic echo /hand_0/hand_diagnostics --once
```

若某一前提不满足，应报告是哪一层失败（文件、转换、MuJoCo、ROS Topic、驱动诊断或实体执行），而不要直接修改坐标映射或把危险命令发送到硬件。

## G. ROS / SDK 控制源切换

```text
ROS → SDK：停止 ROS 回放 → Ctrl+C 停止 ROS launch → ros2 node list 无 wujihand_driver → SDK 预检/--arm
SDK → ROS：SDK 自然结束或 Ctrl+C → 确认进程退出 → 启动 ROS launch → 检查诊断 → ROS 回放
```

ROS 回放停止不等于 driver 退出；SDK 回放结束或普通 `Ctrl+C` 会执行 disable/disconnect；两种模式都不自动回零。完整的退出语义和 `kill -9` 风险见 [01_机械手连接与控制.md](01_机械手连接与控制.md) 第 12 节。
