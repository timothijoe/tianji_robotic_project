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
