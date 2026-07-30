# 新仿真架构

`twin_sim` 只负责 Linux 上的 MuJoCo 仿真，内部统一使用 rad、m、s 和 N。

```text
任务/CLI
  → minimum-jerk Cartesian 或关节轨迹
  → Cartesian 路径连续 IK
  → 7 轴关节位置目标
  → MuJoCo position actuator
  → 状态、接触力、Viewer、CSV
```

模块职责：

- `model.py`：加载模型并校验关节、执行器、site、geom 和 sensor。
- `robot.py`：写入右臂位置目标、保持左臂、仿真步进和数值安全检查。
- `kinematics.py`：SI-only FK、Jacobian、DLS IK 和整条路径预检。
- `trajectory.py`：minimum-jerk 与 Cartesian 姿态插值。
- `force_monitor.py`：读取力传感器、低通滤波和阈值 warning。
- `logging.py`：记录目标/实际关节、TCP pose 和接触力。
- `tasks/chop.py`：编排一次切菜，不包含控制律。
- `cli.py`：稳定命令入口。

位置执行器的 `kp`、`kv` 和力限制属于 MJCF 模型参数，不是对外阻抗 API。
执行器力限制与 `MarvinCCS/marvin_final_fixed.xml` 的关节额定范围一致；腕部
关节 5–7 使用 ±18 N·m，不为追求轨迹跟踪而放宽。
力阈值只产生 warning 和日志标记，不修改轨迹。NaN/Inf 状态则立即停止。

后续导纳控制应作为独立上层模块，根据测量力小幅修正位置目标；它不属于本版本。
新仿真不得导入实体 SDK。
