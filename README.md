# tianji_robotic_project

这是 Marvin 双臂机器人的简化 MuJoCo 仿真工作区。当前仿真使用 MuJoCo 原生
`position` actuator，不再维护自研关节/Cartesian 阻抗控制器。

主要能力：

- 模型加载与 Viewer；
- 右臂关节位置运动；
- Cartesian 路径采样、连续 IK 和关节位置执行；
- 左臂安装 20 自由度 Wuji Hand，并提供安全的位置目标接口；
- 左臂与 Wuji Hand 通过真实 MuJoCo 接触完成方块抓取、搬运和放置；
- 一次“接近—下压—停留—抬刀”切菜任务；
- 双臂耦合完成 5 刀、4 次猫爪倒手的平面切菜演示；
- 正常速度状态录制，以及不重新运行控制器的独立二倍速回放；
- 接触力观测、阈值 warning 和 CSV 日志。

## 快速开始

```bash
python3.12 -m venv .venv
.venv/bin/python -m pip install -r requirements-sim.lock
.venv/bin/python -m pip install -e .
.venv/bin/python -m pytest -q
.venv/bin/twin-sim hand-demo --headless
.venv/bin/twin-sim pick-place --headless
.venv/bin/twin-sim chop --headless --log /tmp/twin-sim-chop.csv
```

使用 `.venv/bin/twin-sim hand-demo --slow` 可以在 Viewer 中查看左手缓慢
闭合后重新张开。左手模型来自 `wuji-description` 的原版左手，按 MIT 许可证
收录在 `robot_assets/mujoco/wuji_hand/`；本项目没有引入 `wuji-mjlab` 的
Isaac Lab/强化学习依赖，也没有连接实体灵巧手。

详细说明：

- [Wuji 手套录制离线回放](docs/wuji/offline_replay.md)
- [Wuji SDK 与 ROS 2 真机接口边界](docs/wuji/hardware_interfaces.md)
- [Tianji + Wuji 集成进展、问题与后续计划](docs/wuji/development_status.md)
- [当前版本复现与智能体交接](docs/simulation/current_version_handoff.md)
- [环境安装](docs/simulation/setup.md)
- [新架构](docs/simulation/architecture.md)
- [使用方法](docs/simulation/usage.md)
- [Wuji Hand 控制手册](docs/simulation/wuji_hand_control.md)
- [左臂 + Wuji Hand 抓取放置](docs/simulation/left_arm_wuji_pick_place.md)
- [ROS 2 与 Wuji Hand 仿真桥接](docs/simulation/ros2_wuji_hand_bridge.md)
- [迁移说明](docs/simulation/migration.md)
- [旧仿真归档](archive/legacy_simulation/README.md)

`SDK_PYTHON/`、`test/` 和 `real_robot_debug/` 属于实体机器人/厂商代码，本轮未
连接实体机器人，也未验证这些脚本的运行行为。
