# Wuji 真机接口边界

项目把仿真、SDK 真机和 ROS 2 真机路径明确分开：

- `tianji_robotics.simulation` 只负责 MuJoCo；
- `tianji_robotics.hardware.wuji_hand.SdkWujiHand` 定义 SDK 生命周期；
- `ros2_ws/` 负责 ROS 2 消息、节点与 launch，不应直接拥有 SDK 连接；
- SDK 与 ROS 2 未来必须通过单一设备所有权机制互斥，不能同时控制一只手。

当前唯一开放的硬件域命令是：

```bash
.venv-wuji-teleop/bin/tianji-robot hardware wuji-sdk preflight trajectory.npz
```

该命令只读取并验证轨迹文件，**不会连接、搜索、使能或控制实体手**。CLI 没有
`--arm` 或执行轨迹的选项。`SdkWujiHand` 也不会在构造时连接；调用方未来必须
显式执行 `connect → arm → command → disarm → close`，且需要先完成上机安全评审。

官方 SDK 的具体 runtime、Wuji ROS 2 实机驱动，以及天机机械臂实机代码仍是后续
工作。现阶段 `real_robot_debug/` 保持为隔离的调试区，不参与离线仿真入口。

## A 臂两阶段阻抗播放器

`real_robot_debug/a_arm_impedance_two_stage.py` 只面向 SDK A 臂（本机左臂）。它读取
`recordings/recorded_hand_guarded_chop_200hz_official_right_retarget_candidate.npz` 的
`left_arm_target_rad`，并转换为 SDK 的角度单位。离线模式可用明确给定的起始关节角
生成 20 秒、200 Hz、两端静止的五次平滑接入段，再显示按 0.1× 速度调度的 831 点
播放计划：

```bash
PYTHONPATH=. python3 real_robot_debug/a_arm_impedance_two_stage.py \
  --start-deg=-90,-90,90,-90,0,0,0
```

该命令不连接、不使能、不发送机器人命令。程序内部已经使用现有 SDK 的关节阻抗序列
（`state=3`、`impedance_type=1`、K/D 参数和逐点目标命令），但目前 `--execute` 会
明确拒绝：独立的 A 臂碰撞预检尚未可用。不得通过修改参数或直接调用内部函数绕过该
安全门；恢复执行前还必须完成现场空间/急停确认、连续反馈、零错误码、真实限位和
运行期跟踪误差检查。
