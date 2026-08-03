# Wuji Glove 到左手 Wuji Hand：文档索引

更新日期：2026-08-02

## 快速入口

| 文档 | 适用场景 |
| --- | --- |
| [01_机械手连接与控制.md](01_机械手连接与控制.md) | 连接、启动、检查和手动控制实体左手 Wuji Hand |
| [02_Wuji_Studio与手套教程.md](02_Wuji_Studio与手套教程.md) | 配置右手手套、Studio 可视化和录制 MCAP |
| [03_右手到左手与MuJoCo.md](03_右手到左手与MuJoCo.md) | 右手骨架镜像、官方重定向和 MuJoCo 验证 |
| [04_MCAP实体左手回放.md](04_MCAP实体左手回放.md) | 将转换后的 MCAP 安全回放到实体左手 |
| [05_新Agent交接与复现清单.md](05_新Agent交接与复现清单.md) | 新会话/新 Agent 从零接手时的顺序、已知状态与验收清单 |
| [问题与决策日志.md](问题与决策日志.md) | 按时间线记录排查证据、失败现象和技术决策 |
| [运行与排障.md](运行与排障.md) | 常用命令、可视化操作和快速故障索引 |

下面保留项目状态摘要；日常操作请优先使用上表四份教程。

## 1. 项目目标

使用 **右手 Wuji Glove** 采集人的手部动作，将其离线转换为 **左手第一代 Wuji Hand** 的 20 个关节位置，并先在 MuJoCo 中可视化。当前阶段的重点是验证坐标转换和关节映射；离线流程不会连接或控制实体手。

最终的预期链路如下：

```text
右手 Wuji Glove
  └─ Wuji Studio 录制 .mcap
       └─ 右手坐标镜像到左手坐标
            └─ 官方 RetargetSession 映射为 20 个左手关节
                 ├─ .npz 轨迹（MuJoCo 回放）
                 └─ JointState .mcap（通用关节数据）
```

## 2. 已确认的硬件与软件状态

### 实体左手 Wuji Hand

- 已安装官方 `wujihandcpp 1.8.0`。
- `wujihandros2` 的 ROS 2 驱动已能连接左手实体手。
- 已以低速、连续小幅正弦命令验证关节确实会运动；硬件温度由现场人工确认正常。
- 避免同时使用 ROS 驱动和 SDK 直接向同一只手发命令，以免产生控制冲突。

### 右手 Wuji Glove

- 实际可用网络端点：`192.168.10.151:50001`。
- 设备侧：右手，序列号 `WG1KA03260510008`，固件 `0.10.1`。
- SDK 可成功获取 21 个手部骨架关键点。
- PC 曾在网口 `enp129s0` 增加 `192.168.10.10/24` 地址；此操作只配置电脑，不会修改手套 IP。
- Wuji Studio 与自行编写的 SDK 客户端会争用同一个手套会话。两者不能同时连接；同时连接时设备会返回 `Session already exists`。Studio 录制时应关闭独立 SDK 客户端。

### Wuji Studio

- 已能连接并可视化右手手套。
- Studio 可以一边可视化、一边录制 MCAP。
- 本项目使用的 Studio MCAP 包含 `/right_glove/hand_skeleton`（JSON，21 个关键点）等 Topic。
- Studio 的 Hand Skeleton 面板针对手套骨架；仅含 `/joint_states` 的自定义左手 MCAP 不会自动显示成左手模型。因此左手结果的主要可视化工具改为 MuJoCo。

## 3. 关键技术结论：右手转左手必须镜像 Y 轴

最早的版本将右手骨架直接传给 `Handedness.Left` 的 retargeter。MuJoCo 中可看到指尖附近关节弯曲明显过度：例如食指到小指的第三、四关节大量贴近上限。

原因是左右手腕坐标系并不完全相同。根据官方坐标约定，手腕坐标的 X 轴和 Z 轴方向可保持，而 Y 轴含义相反：右手 Y 正方向朝掌面，左手 Y 正方向朝手背。因此，右手骨架进入左手映射器前必须执行：

```python
left_keypoints = right_keypoints.copy()
left_keypoints[:, 1] *= -1
```

修正后，原来贴上限的食指到小指末端关节均回到正常范围，说明坐标镜像是造成视觉差异的根本原因。该变换位于：

[`wuji-glove-recorder/mcap_to_left_qpos.py`](../wuji-glove-recorder/mcap_to_left_qpos.py)

函数名为 `mirror_right_to_left`，并在输出元数据中记录：

```text
input_hand_side: right
input_transform: mirror_y_right_wrist_to_left_wrist
```

## 4. 已开发的工具

| 文件 | 用途 | 是否连接硬件 |
| --- | --- | --- |
| [`glove_recorder.py`](../wuji-glove-recorder/glove_recorder.py) | Tkinter 开始/结束按钮，直接从 SDK 将骨架保存为 NPZ | 会连接手套 |
| [`mcap_to_left_qpos.py`](../wuji-glove-recorder/mcap_to_left_qpos.py) | Studio 的右手 MCAP → 镜像 → 官方左手 20 关节 NPZ | 否 |
| [`left_qpos_to_mcap.py`](../wuji-glove-recorder/left_qpos_to_mcap.py) | 左手 20 关节 NPZ → 标准 `sensor_msgs/msg/JointState` MCAP | 否 |
| [`mujoco_left_replay.py`](../wuji-glove-recorder/mujoco_left_replay.py) | 在左手 MuJoCo MJCF 模型回放 NPZ | 否 |
| [`visualize_right_glove_to_left.sh`](../wuji-glove-recorder/visualize_right_glove_to_left.sh) | 一键执行上述离线转换并打开 MuJoCo | 否 |

MuJoCo 回放器只将离线轨迹写入仿真 `data.qpos` 后调用 `mj_forward`；不含 ROS、SDK 或实体手控制代码。

## 5. 当前样例数据与产物

输入录制：

```text
/home/zhoutong/catkin_robotic_ws/august_ws/wuji-record-data/august_02/session_20260802_162909_764.mcap
```

转换产物（与输入在同一目录）：

```text
session_20260802_162909_764_right_to_left_wuji_hand.npz
session_20260802_162909_764_right_to_left_wuji_hand.mcap
```

早期、未镜像的文件名为 `session_20260802_162909_764_left_wuji_hand.*`，仅用于对比问题现象；后续应使用带 `right_to_left` 后缀的修正版本。

## 6. Python 环境与依赖

一键脚本内部已固定使用以下解释器，运行脚本前无需手动 `source` 或激活虚拟环境：

| 环节 | Python 解释器 | 关键依赖 |
| --- | --- | --- |
| MCAP 解析、官方 retargeting、JointState MCAP 导出 | `/home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python` | `wuji-sdk 2026.7.21`、`numpy`、`mcap` |
| MuJoCo 可视化与测试 | `/home/zhoutong/catkin_robotic_ws/august_ws/tianji_robotic_project/.venv/bin/python` | `mujoco 3.10.0`、`numpy`、`mcap`、`pytest` |

## 7. 验证记录

- 右手骨架的左右手 Y 轴镜像已具有自动化测试。
- MCAP → 左手 20 关节、关节 NPZ → MCAP、MuJoCo 轨迹加载、录制器数据校验均有测试。
- `visualize_right_glove_to_left.sh` 已分别使用 Bash 和 `sh` 干跑验证。
- 最近一次完整测试：13 项通过。
- 已实际使用 `sh visualize_right_glove_to_left.sh` 成功转换样例 MCAP 并启动 MuJoCo。

## 8. 当前限制与后续建议

1. 当前完成的是**离线可视化**，不是实时遥操作。将轨迹下发实体手前，需加入关节限位、速率限制、急停和小幅度人工验收。
2. 目前使用官方 `RetargetSession` 输出固件关节顺序的 20 维位置；这适合后续发送到手部控制接口，但实体手首次使用前必须先以低速、单帧或小范围命令验证每个手指映射。
3. MuJoCo 中的几何手形与手套 Studio 中的骨架显示不是同一模型；应优先比较关节弯曲趋势，不要以视觉外形完全一致作为唯一判据。
4. 若手套 IP 或网段更改，应先在 Studio 或 SDK 中重新确认可达性；不要随意修改设备 IP，以免影响其他电脑的配置。

## 9. 相关已有设计文档

- [`docs/superpowers/specs/2026-08-02-left-hand-mcap-hardware-replay-design.md`](../docs/superpowers/specs/2026-08-02-left-hand-mcap-hardware-replay-design.md)：离线 MCAP 到实体左手的 `--arm` 回放安全设计。
- [`doc_zt/问题与决策日志.md`](问题与决策日志.md)：按实际推进顺序记录现象、证据、决策和失败尝试。
- [`docs/superpowers/specs/2026-08-02-wuji-glove-recorder-design.md`](../docs/superpowers/specs/2026-08-02-wuji-glove-recorder-design.md)
- [`docs/superpowers/specs/2026-08-02-left-hand-mujoco-replay-design.md`](../docs/superpowers/specs/2026-08-02-left-hand-mujoco-replay-design.md)
- [`wuji-glove-recorder/README.md`](../wuji-glove-recorder/README.md)
