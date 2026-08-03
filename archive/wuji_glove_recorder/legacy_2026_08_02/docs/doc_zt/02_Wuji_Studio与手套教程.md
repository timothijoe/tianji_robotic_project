# 02. Wuji Studio 与 Wuji Glove：连接、可视化和录制

适用组合：右手 Wuji Glove → 本机 Wuji Studio。当前实际验证的设备端点为 `192.168.10.151:50001`，设备序列号为 `WG1KA03260510008`，固件 `0.10.1`。

## 1. 网络与物理连接

手套有 USB 和网线接口。本次稳定的数据连接通过**网线和 IP 网络**建立；USB 可以同时连接，但不应仅依据 USB 或指示灯状态判断数据通信成功。

主机网卡曾配置同网段地址：

```text
enp129s0: 192.168.10.10/24
```

手套服务端实际可达地址：

```text
192.168.10.151:50001
```

主机地址只影响本机网络，不会改变手套 IP。手套曾在其他电脑的 Studio 配置过，因此应优先让电脑适配现有网段，**不要随意修改手套 IP**。

## 2. Studio 的正确用途

Studio 适合做三件事：

1. 验证手套是否在线；
2. 实时显示右手骨架/手形；
3. 一边可视化，一边录制 MCAP。

在 Studio 中连接右手设备后，确认右手模型随手动作。不同 Studio 版本的面板文案可能不同；若找不到旧教程中的 `Hand model source`，不要据此判断失败。重点是找到 3D/Hand Skeleton 类面板，并选择右手手套骨架 Topic。

常见源 Topic：

```text
/right_glove/hand_skeleton
```

该 Topic 是 JSON 编码的 21 个骨架关键点，手腕 frame 为 `r_wrist`。录制通常还会有 `tip_poses`、`hand_joint_angles` 等 Topic；本项目用 `hand_skeleton` 做官方 retargeting 输入。

## 3. Studio 录制步骤

1. 确认右手模型实时跟随；
2. 在 Studio 的录制/Player 功能中创建录制；
3. 开始录制，完成动作后停止；
4. 保存得到 `.mcap` 文件；
5. 用本项目的离线工具转换，而不要在录制时同时启动独立手套 SDK 程序。

Studio 可以同时可视化和录制，因此这是推荐的采集方式。

## 4. 最重要的会话限制：Studio 与独立 SDK 不能并用

本次实际收到过设备错误：

```text
Session already exists
```

原因是 Studio 与独立 SDK 客户端争用同一个手套通信会话。操作约定：

| 目的 | 应运行的程序 |
| --- | --- |
| Studio 观察/录制 | 只运行 Studio，关闭独立 SDK 采集器 |
| 自定义 SDK 采集 | 关闭 Studio 或先断开设备 |
| 已录制 MCAP 的转换/MuJoCo/实体回放 | 不连接手套，可不受影响 |

不要因为 SDK 收不到数据就更改 IP；先检查 Studio 是否还占用设备。

## 5. 直接 SDK 采集器（可选）

工程中的：

```text
wuji-glove-recorder/glove_recorder.py
```

是一个 Tkinter 开始/结束按钮录制器，将 21 个关键点保存为 NPZ。它只读手套，不控制机械手。运行前关闭 Studio：

```bash
cd ~/catkin_robotic_ws/august_ws/wuji-technology/wuji-glove-recorder
/home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python glove_recorder.py
```

但若目标是后续转左手，优先使用 Studio 录制 MCAP，后续流程更完整、可复现。

## 6. Studio 显示与左手机械手 MCAP 的区别

Studio 可以很好地显示手套的 `/right_glove/hand_skeleton`。但是项目输出的左手 `.mcap` 是标准 ROS `sensor_msgs/msg/JointState` 的 `/joint_states` 数据，不一定会自动显示成左手机械手。

原因不是文件必然错误，而是 Studio 的 Hand Skeleton 面板针对手套骨架；机械手关节数据还需要相应 URDF、模型源和关节解释。当前推荐使用 MuJoCo 验证左手机械手，而不是强行以 Studio Hand Skeleton 面板显示它。

## 7. 官方资料应如何使用

本仓库的 `wuji-studio/README.md` 和 Studio 的内置帮助是安装与界面入口。实际项目应额外记录：

- 设备当前 IP、主机网卡和已验证端口；
- 左右手设备实际物理侧；
- Studio 与 SDK 的会话互斥；
- Studio UI 在版本间会变化，按 Topic/数据流定位，不要死找旧教程的按钮名称；
- 录制后一定保留原始 MCAP，它是今后重新映射的依据。

## 8. 从零连接的逐步验收

按顺序做，前一项未通过不要跳到下一项：

1. 手套网线接入已配置 `192.168.10.10/24` 的主机网口；USB 可同时接入，但不以 USB 当数据成功依据。
2. 打开 Studio，选择/连接右手设备，确认目标端点为 `192.168.10.151:50001`。
3. 在 Studio 中观察模型；实际移动手套，模型应连续更新。
4. 启动录制前确认没有独立 `glove_recorder.py` 或 SDK 订阅程序在运行。
5. 录制一次 5–10 秒的慢动作：张手、握拳、食指屈伸和拇指动作至少各出现一次。
6. 停止录制，记录生成文件的绝对路径；不要只记录文件名。
7. 保留原始 MCAP，不覆盖、不改后缀；之后的转换产物使用新后缀保存。

推荐将录制文件放在稳定目录，例如：

```text
/home/zhoutong/catkin_robotic_ws/august_ws/wuji-record-data/YYYY_MM/
```

本次也使用过临时目录 `/tmp/august_02/`；它适合短期实验，但 `/tmp` 可能被系统清理，重要原始数据应复制到工作区数据目录。

## 9. 指示灯、IP 与“没有数据”的排障决策树

### 指示灯亮一下又灭

这不足以推断供电不足或网络已断。先检查 Studio 是否能连接、是否有骨架帧；只有 Studio/SDK 都无法建立连接时，再检查供电、线缆和主机网段。

### Studio 能连接、SDK 报 `Session already exists`

这是会话占用，不是 IP 错误。关闭 Studio 的设备连接或完全退出 Studio，再重试 SDK；不要修改手套 IP。

### Studio 也无法连接

依次检查：网线是否接好 → 本机是否仍有 `192.168.10.10/24` → 手套地址是否仍为 `.151` → 是否被另一台电脑/Studio 占用。设备曾在其他电脑配置过，优先确认而非重置。

### Studio 有画面但转换器找不到骨架

打开 MCAP 后先检查是否包含 `/right_glove/hand_skeleton`。本项目转换器只读取这个 Topic；只包含 tip pose 或关节角的录制不符合当前转换器输入。

## 10. 录制质量建议

- 动作尽量慢且连续，避免突然甩手；这减少重定向后相邻帧跳变。
- 录制开始和结束保留约 1 秒自然静止，方便实体回放从第一帧缓入。
- 记录手套是右手还是左手。当前工具链明确假设**右手手套**输入，输出左手。
- 采集多个短任务文件优于一个很长的混合文件；每个文件应有清楚的动作名称和日期。
- 不要删除“看起来不好”的原始录制；它有助于判断是手套数据、坐标转换还是机械手限制问题。
