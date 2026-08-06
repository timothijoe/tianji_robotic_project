# MuJoCo 轨迹导出与右手 Retarget 进展（2026-08-06）

## 摘要

本轮恢复了 MuJoCo GLFW Viewer，并完成了 Tianji 双臂与 Wuji 左手录制切菜的
Viewer 验收。随后导出了 200 Hz 的双臂加左手离线轨迹。为实现左右镜像，最初尝试
从已 retarget 的左手 20 维关节角直接数值生成右手；MuJoCo Viewer 证明该方法的
右手动作严重错误，现已标记为不可用。当前正确候选改为从原始右手手套骨架直接经过
官方 `Handedness.Right` retargeter 生成右手关节角，再与镜像双臂对齐到 200 Hz。

本记录仅覆盖离线文件、MuJoCo 与官方手部模型；没有启动 ROS 2、没有连接 SDK
设备、没有命令 Tianji 或 Wuji 真机。

## 1. MuJoCo Viewer 恢复

### 问题

重启前，headless 仿真正常，但 GLFW Viewer 报：

```text
GLX: Failed to create context: BadValue
ERROR: could not create window
```

原因是 NVIDIA 用户态库为 `595.84`，内核已加载模块为 `595.71.05`，导致
`nvidia-smi` 报 NVML driver/library version mismatch。

### 解决与验收

系统重启后，`nvidia-smi` 显示 Driver Version `595.84`，桌面 Xorg/gnome-shell
正常使用 GPU。以下 Viewer 命令完成并正常退出：

```bash
env -u PYTHONPATH .venv-wuji-teleop/bin/twin-sim recorded-hand-guarded-chop \
  --final-hold 60 \
  --hand-mcap recordings/wuji/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap
```

输出为 5 次切削、5 次护手循环，计划最小刀手距离 `0.022 m`，优选安全层
`0.020 m`，最大桌面穿透 `0.000000 m`，拇指净间隙 `0.003 m`。

## 2. 原始双臂加左手 200 Hz 导出

已将当前录制切菜仿真的命令轨迹导出为 5 ms（200 Hz）离线数据：

- `recordings/recorded_hand_guarded_chop_200hz_latest.npz`
- `recordings/recorded_hand_guarded_chop_200hz_latest.csv`

共有 831 帧，时长 4.150 s。数据包含：

- `time_s`：`(831,)`，步长 0.005 s；
- `right_arm_target_rad`：`(831, 7)`；
- `left_arm_target_rad`：`(831, 7)`；
- `left_hand_target_rad`：`(831, 20)`。

`.npz` 是权威数据格式，保留完整数组、形状和安全元数据；`.csv` 便于人工查看和
外部脚本适配。CSV 的可读列名使用 1-based 仿真命名，例如
`left_finger1_joint1_rad`；若 SDK 使用 0-based 索引，则该列对应
`finger[0].joint[0]`，20 维平铺顺序为五根手指、每指四关节。

## 3. 失败的右手数值镜像方法

### 原始假设

为镜像双臂，采用用户指定的规则：交换左右臂后，关节 1、3、5、7 取反，
关节 2、4、6 保持不变。该规则用于离线臂轨迹变换。

初版右手尝试比较左右手 MJCF 的关节轴：20 个关节的编号和限位一一对应，只有
拇指第二关节的轴符号不同；因此初版错误地将左手 20 维输出复制为右手，仅将
`finger1_joint2` 取反。

### 发现的问题与根因

在官方 `right.xml` MuJoCo Viewer 中播放后，右手动作严重失真。根因不是 Viewer，
而是数据流错误：

```text
原始右手 21 点骨架
  → 镜像 wrist Y
  → 官方 Left retargeter
  → 左手 20 维关节角
```

左手 20 维关节角是 retargeter 的非线性输出，不能通过逐关节取反或复制可靠反推
官方右手 20 维关节角。MJCF 轴方向比较只能说明模型坐标差异，不能证明 SDK 索引、
零位、retargeter 内部约束、关节耦合或电机正方向。

旧文件已保留但改名为不可用，禁止作为真机输入：

- `recordings/recorded_hand_guarded_chop_200hz_UNUSABLE_numeric_mirror.npz`
- `recordings/recorded_hand_guarded_chop_200hz_UNUSABLE_numeric_mirror.csv`

## 4. 当前正确的右手候选链路

当前候选使用原始右手录制，而非旧左手关节角：

```text
recordings/wuji/august_02_origin/session_20260802_174440_936.mcap
  (/right_glove/hand_skeleton，499 帧)
  → 官方 RetargetSession.for_hand(WujiHand, Handedness.Right)
  → 官方 right.xml 的 20 路控制范围裁剪
  → 线性重采样到 5 ms / 200 Hz
  → 与镜像后的双臂目标合并
```

官方 SDK 环境确认支持 `Handedness.Right`。原始录制被识别为
`right_glove_skeleton`。直接 retarget 输出为 `(499, 20)` 且全为有限数值；有最大
`4.77e-9 rad` 的浮点边界超限，因此仅裁剪到官方右手 MJCF `ctrlrange`。

新候选包：

- `recordings/recorded_hand_guarded_chop_200hz_official_right_retarget_candidate.npz`
- `recordings/recorded_hand_guarded_chop_200hz_official_right_retarget_candidate.csv`

候选包为 831 帧、5 ms。双臂仍按指定的交换与奇数轴取反规则生成；右手来自官方
Right retargeter，并非从左手关节数据数值镜像而来。候选文件的 `right_hand_target_rad`
在官方 `right.xml` 控制范围内，最小范围余量为 0（存在触及限位的帧）。

## 5. MuJoCo 可视化已执行的内容

已使用官方 Wuji MJCF 依次查看：

1. 原始左手关节轨迹在官方 `left.xml` 中的动作；
2. 错误数值镜像包在官方 `right.xml` 中的动作（发现严重错误）；
3. 原始右手骨架经官方 Right retargeter 的 `(499, 20)` 临时轨迹在官方 `right.xml`
   中的动作；
4. 新 200 Hz 候选包 `right_hand_target_rad` 在官方 `right.xml` 中的动作，并按需要
   连续播放 3 次。

这些 Viewer 仅写 MuJoCo qpos 并调用 `mj_forward`，没有调用 ROS、Wuji SDK 设备连接
或真实手控制。

## 6. 已验证命令与回归

200 Hz 导出功能的完整项目回归曾通过：`272 passed, 1 warning`；warning 为既有切菜
接触力观察告警（39.611 N 超过 30 N），不属于录制手势问题。

镜像候选生成时已检查：

```text
samples=831
dt_s=0.005000
right_hand_range_margin_rad=0
```

旧数值镜像与原始导出之间的数值身份检查曾显示臂变换、拇指符号和其他 19 路手关节
误差均为零；这只证明旧文件按其错误规则生成正确，不证明其右手动作正确。

## 7. 当前限制与真机前风险

新 `official_right_retarget_candidate` 比旧数值镜像方法有正确的数据来源，但仍是离线
候选，不可直接认定为真机安全程序。MuJoCo `right.xml` 可验证模型关节范围和视觉
屈伸方向，不能验证以下项目：

- 真机 SDK 的 0-based 索引、数组顺序和命名约定；
- 真机零位偏置、方向、齿隙、传动耦合和固件限位；
- 与 Tianji 右臂安装后的坐标系、时钟同步和碰撞风险；
- 真实硬件的速度、加速度、电流、力矩和急停策略。

在任何真机右手动作前，应先在隔离、无负载、低速条件下逐关节进行 SDK 索引和正方向
核验；再验证零位、硬件限位与反馈闭环。不要将旧 `UNUSABLE_numeric_mirror` 文件或
当前 `candidate` 文件直接批量下发给真机。

## 8. 建议下一步

1. 以官方 Right retarget 候选为唯一右手离线来源，继续在 MuJoCo 观察拇指及各手指
   屈伸方向；
2. 编写仅检查、不发布命令的 SDK 数组索引/反馈读取工具，确认 0-based 顺序；
3. 完成逐关节低速正方向与零位验证后，再单独设计真机播放的限速、限幅、超时和急停
   流程；
4. 在右手验证完成前，保持双臂/右手候选仅用于离线分析与 MuJoCo Viewer。
