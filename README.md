# tianji_robotic_project

这是 Marvin 双臂机器人的简化 MuJoCo 仿真工作区。当前仿真使用 MuJoCo 原生
`position` actuator，不再维护自研关节/Cartesian 阻抗控制器。

主要能力：

- 模型加载与 Viewer；
- 右臂关节位置运动；
- Cartesian 路径采样、连续 IK 和关节位置执行；
- 一次“接近—下压—停留—抬刀”切菜任务；
- 接触力观测、阈值 warning 和 CSV 日志。

## 快速开始

```bash
python3.12 -m venv .venv
.venv/bin/python -m pip install -r requirements-sim.lock
.venv/bin/python -m pip install -e .
.venv/bin/python -m pytest -q
.venv/bin/twin-sim chop --headless --log /tmp/twin-sim-chop.csv
```

详细说明：

- [环境安装](docs/simulation/setup.md)
- [新架构](docs/simulation/architecture.md)
- [使用方法](docs/simulation/usage.md)
- [迁移说明](docs/simulation/migration.md)
- [旧仿真归档](archive/legacy_simulation/README.md)

`SDK_PYTHON/`、`test/` 和 `real_robot_debug/` 属于实体机器人/厂商代码，本轮未
连接实体机器人，也未验证这些脚本的运行行为。
