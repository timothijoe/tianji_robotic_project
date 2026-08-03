# Wuji Glove Recorder

一个只读的 Wuji Glove 骨骼数据采集工具。窗口只有“开始记录”和“停止并保存”两个采集操作；它只订阅右手手套的 `hand_skeleton` 数据，**不会控制、使能或连接机械手，也不会修改手套 IP、端口或固件**。

## 启动

手套连接到本机网络并能在 Wuji Studio 中正常显示后，在此目录执行：

```bash
/home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python -m pip install -r requirements.txt
/home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python glove_recorder.py
```

程序默认连接 `192.168.10.151:50001`。状态显示“已连接”后，点击“开始记录”，完成动作后点击“停止并保存”。保存路径会显示在窗口中。

录制时请关闭 Wuji Studio，避免两个程序同时订阅时的界面或网络干扰。

## 录制文件

每次录制会在 `recordings/` 生成一个压缩 NPZ 文件，文件名形如 `glove_20260802_153000.npz`。

- `timestamps_ns`：每一帧的单调时钟纳秒时间戳，形状 `(N,)`。
- `keypoints_m`：每帧 21 个手部骨骼关键点的米制 XYZ 坐标，形状 `(N, 21, 3)`。
- `metadata`：JSON 字符串，包含手套端点、序列号、左右手信息、帧数和坐标单位。

这些数据可作为下一步“右手套到左机械手”离线映射的输入；本工具本身不做映射，也不向 ROS 发布消息。

## Studio MCAP 转左手关节角

Studio 录制的 MCAP 已包含 `/right_glove/hand_skeleton`。以下命令只读取该文件，并调用官方 SDK 的 `RetargetSession`，生成目标为**第一代左 Wuji Hand**的 20 个关节角；它不会连接或控制手套/机械手。

右手套的 `r_wrist` 与左手 `l_wrist` 坐标系 Y 轴方向相反，因此转换器会先翻转所有关键点的 Y 坐标，再送入左手 retargeter。这是右手套映射到左机械手所必需的镜像步骤：

```bash
/home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python \
  mcap_to_left_qpos.py \
  /absolute/path/to/session.mcap \
  /absolute/path/to/session_left_wuji_hand.npz
```

输出 NPZ 含：

- `timestamps_ns`：对应 Studio 骨骼帧的纳秒时间戳；
- `left_joint_positions_rad`：形状 `(N, 20)` 的左机械手目标关节角（弧度）；
- `metadata`：输入文件、源 Topic、源坐标系、目标模型 `WujiHand`、目标侧 `left`。

这仍是离线检查结果。未经逐指方向检查、速度限制和显式使能保护，不应将该文件直接发布到实体手。

## 左手 MCAP 回放

将上一步的左手关节 NPZ 写成 Studio 可回放的 `/joint_states` MCAP：

```bash
/home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python \
  left_qpos_to_mcap.py \
  /absolute/path/to/session_left_wuji_hand.npz \
  /absolute/path/to/session_left_wuji_hand.mcap
```

输出文件中每帧都有 20 个左手 URDF 关节名（`left_finger1_joint1` 至 `left_finger5_joint4`）和同序位置值。Wuji Studio 中进入 PLAYER，打开该 MCAP；在 3D 面板选择左手 `wujihand` 模型或本仓库的 `wuji-description/hand/body/urdf/left-ros.urdf` 作为 Custom URDF，然后播放。

该 MCAP 是回放数据，不会对任何真实设备发送命令。

## MuJoCo 左机械手回放

Tianji 项目已有 MuJoCo 3.10.0 环境。以下命令将 NPZ 中的 20 个左手关节角加载到本仓库的 `left.xml` 机械手模型中；只改变仿真状态，不使用 ROS、SDK 或任何实体设备：

```bash
cd /home/zhoutong/catkin_robotic_ws/august_ws/wuji-technology/wuji-glove-recorder
/home/zhoutong/catkin_robotic_ws/august_ws/tianji_robotic_project/.venv/bin/python \
  mujoco_left_replay.py \
  /home/zhoutong/catkin_robotic_ws/august_ws/wuji-record-data/august_02/session_20260802_162909_764_left_wuji_hand.npz
```

快捷键：空格播放/暂停；暂停时 `←` / `→` 单帧；`R` 回到第一帧；`+` / `-` 调整 0.25× 至 4× 播放速度；`Esc` 退出。MuJoCo 窗口自身支持鼠标旋转、平移和缩放视角。

## 测试

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  /home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python \
  -m pytest tests/test_glove_recorder.py -v
```
# Right glove MCAP to left-hand MuJoCo visualization

For a Studio recording from the right Wuji Glove, edit `DEFAULT_MCAP_PATH` near
the top of `visualize_right_glove_to_left.sh`, then run:

```bash
./visualize_right_glove_to_left.sh
```

The script mirrors the glove's wrist-frame Y coordinate for a left hand,
retargets the motion to the 20 left Wuji Hand joints, writes
`*_right_to_left_wuji_hand.npz` and `*_right_to_left_wuji_hand.mcap` beside the
original recording, then opens the NPZ trajectory in MuJoCo. It is an offline
visualization tool and never connects to or commands the physical hand.

To check the paths and generated filenames without writing files or starting a
viewer:

```bash
./visualize_right_glove_to_left.sh --dry-run
```

You may also provide an MCAP path as an argument to temporarily override the
configured path.

## 实体左手回放（危险：会运动硬件）

`replay_left_hand_mcap.sh` 只接受已完成右转左镜像的
`*_right_to_left_wuji_hand.mcap`。它先验证 20 个具名关节、MJCF 范围、时间戳和帧间变化；**没有 `--arm` 时只读检查，不会初始化 ROS 或发送命令**：

```bash
sh replay_left_hand_mcap.sh \
  /tmp/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap
```

首次实体执行前，先在另一终端启动已验证的 `/hand_0` 左手驱动，清空手周围区域并保持人工监看和硬件急停可用。首次建议使用更慢的 5% 速度和 5 秒缓入：

```bash
sh replay_left_hand_mcap.sh --arm --speed 0.05 --ramp-seconds 5 \
  /tmp/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap
```

默认 `--speed` 是 0.1，最大只能为 1.0。`Ctrl+C` 只会停止本程序继续发布，不是硬件急停；手将保持最后一个已命令姿态，不会自动回零或张开。
