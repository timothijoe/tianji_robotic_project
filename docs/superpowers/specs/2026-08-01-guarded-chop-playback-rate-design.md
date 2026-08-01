# Guarded Chop Viewer 1.5× 播放设计

## 目标

将 `./scripts/run_guarded_chop.sh` 的 MuJoCo Viewer 默认播放速度设为 1.5×。仿真仍执行
完全相同数量的控制步和物理子步；轨迹几何、阶段时长所代表的仿真时间、位置伺服目标、
接触计算、安全互锁、marker 更新顺序及最终结果均不改变。

本次只影响 guarded-chop 的有窗口播放。headless 运行、其他仿真任务、ROS 2、实体机械臂
和实体 Wuji Hand 不受影响。

## 方案

采用“物理步长与 Viewer 等待解耦”的方案。`RightArmRobot.step()` 新增可选的
`viewer_sleep_s` 关键字参数：

- 未传入时仍等待 `control_dt_s`，保持所有现有调用者行为；
- 传入时仅用该值控制 Viewer 的墙钟等待；
- MuJoCo 子步数始终由原始 `control_dt_s` 计算，绝不使用缩短后的等待值。

`GuardedChopConfig` 新增 `viewer_playback_rate: float = 1.5`。guarded-chop 每个控制步继续
调用 `robot.step(control_dt_s)` 进行物理推进，同时仅在 Viewer 存在时传入
`viewer_sleep_s = control_dt_s / viewer_playback_rate`。因此 `0.01 s` 的仿真控制步对应约
`0.00667 s` 的墙钟等待，画面约以 1.5× 播放。

没有采用缩小 `control_dt_s`，因为那会改变控制采样和轨迹点数；也没有再次缩短动作阶段
时长，因为那会改变关节速度与安全预检，而不只是播放速度。

## 配置与兼容性

`viewer_playback_rate` 必须是有限正数。默认启动脚本无需新增参数，会自然使用 1.5×。
Python 调用者可以显式构造 `GuardedChopConfig(viewer_playback_rate=1.0)` 恢复实时播放。
本次不新增 CLI 参数，避免为当前单一默认需求扩展接口。

`RightArmRobot.step()` 的新参数为仅限关键字且有兼容默认值。现有 `step(dt)` 和
`step(dt, sync_viewer=...)` 调用保持原义。传入的 `viewer_sleep_s` 必须是有限非负数；
无 Viewer 时不执行等待。

## 测试与验收

自动测试验证：

- guarded-chop 默认播放倍率为 1.5，非正数、NaN 和无穷值被拒绝；
- `RightArmRobot.step(0.01, viewer_sleep_s=0.01 / 1.5)` 的 MuJoCo 仿真时间仍只前进
  `0.01 s`，并使用约 `0.00667 s` 的 Viewer 等待；
- 未传入新参数时仍按 `control_dt_s` 等待；
- guarded-chop 向 Viewer 路径传递缩放后的等待值，headless 路径不产生墙钟等待；
- 既有 5 刀、4 次倒手、安全距离以及完整测试继续通过。

人工运行启动脚本，确认动作顺序和 marker 轨迹不变，整体播放明显快于上一版本，最终
仍返回 5 刀、4 次倒手和相同安全指标。
