# 旧仿真迁移说明

旧仿真保存在 `archive/legacy_simulation/`，不参与安装、导入或 pytest。

| 旧功能 | 当前实现 |
|---|---|
| Joint impedance / torque PD | MuJoCo 原生 position actuator |
| Cartesian impedance | Cartesian 采样、连续 IK、关节位置轨迹 |
| Force control | 仅接触力观测和 warning |
| Replay 与全量轨迹演示 | 已归档，不迁移 |
| SDK 风格仿真兼容层 | 已归档，不兼容 |
| 实体机器人 SDK | 保留原文件，本轮未修改或验证 |

代码迁移时不要从归档目录导入模块。需要历史实现时只读查看，恢复方法见
`archive/legacy_simulation/README.md`。

导纳控制是预计的后续功能：测量力只修正位置目标，并保持与实体控制实现隔离。

