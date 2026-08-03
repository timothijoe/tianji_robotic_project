# Wuji 手套录制离线回放

本流程读取 Wuji Studio 的右手骨架 MCAP，经官方 `wuji-sdk` 离线重定向为
第一代左手 20 关节轨迹，再由项目内 MuJoCo 后端回放。它不会发现或连接真机。

回放使用 hand-only 场景，只加载 `robot_assets/mujoco/wuji_hand_standalone/`
中按 MIT 许可证迁入的官方左手模型，**不会加载机械臂**。双臂及机械臂挂载手的
联合仿真仍由独立的 `twin-sim` 命令负责。

## 环境

```bash
./scripts/setup_wuji_teleop_env.sh
```

脚本只创建项目内 `.venv-wuji-teleop`，不激活或修改当前 shell。官方源码仓库
保持在相邻的 `../wuji-technology/`；它们不复制进本项目。

## 回放

录制文件保存在被 Git 忽略的 `recordings/wuji/`。无界面复现命令为：

```bash
.venv-wuji-teleop/bin/tianji-robot sim wuji-replay \
  recordings/wuji/august_02/session_20260802_162909_764.mcap \
  --headless \
  --npz recordings/wuji/august_02/session_20260802_162909_764_refactored.npz \
  --joint-state-mcap recordings/wuji/august_02/session_20260802_162909_764_refactored.mcap
```

去掉 `--headless` 可打开 Viewer。`--npz` 保存项目的规范轨迹；
`--joint-state-mcap` 输出 ROS 2 `sensor_msgs/msg/JointState` 语义的 MCAP。
两个输出均为本地生成物，不应提交 Git。

旧的自研脚本和决策记录只作为历史材料归档在
`archive/wuji_glove_recorder/legacy_2026_08_02/`；新功能以 `src/tianji_robotics`
及上述 CLI 为准。
