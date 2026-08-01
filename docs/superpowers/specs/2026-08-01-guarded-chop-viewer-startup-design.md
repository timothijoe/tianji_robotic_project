# Guarded Chop Viewer 无旧场景启动设计

## 问题与根因

`run_guarded_chop(viewer=True)` 当前直接构造 `RightArmRobot(viewer=True)`。
机器人构造器先 reset 到通用默认姿态并立即打开 MuJoCo Viewer；切菜任务随后才进行
轨迹预检、隐藏抓取任务的红色方块、绿色目标区和两个灰色底座，并 reset 到切菜
ready 姿态。因此 Viewer 的首屏会短暂显示旧抓取场景和默认举刀姿态，之后突然
切换到正确切菜画面。模型中并不存在运行时复制出的第二套物体，问题是显示时序。

## 目标与范围

默认 `guarded-chop` Viewer 只有在以下工作全部完成后才打开：

- 轨迹预检成功；
- 当前 scene mode 的物体显示和碰撞配置完成；
- 右臂、左臂和 Wuji Hand 已设为最终切菜初始姿态；
- MuJoCo 正向计算已经更新几何位置。

Viewer 第一帧必须直接显示正确的砧板、刀具、猫爪护手和切菜姿态，不显示抓取任务
物体或通用 home 姿态。窗口打开后保持正确初始姿态约 1 秒，再开始原有动作。

本次只改变 `guarded-chop` 的 Viewer 打开时机。无窗口模式、其他仿真任务、ROS 2、
实体机械臂、实体 Wuji Hand 和硬件发送接口均不改变。

## 设计

为 `RightArmRobot` 增加幂等的 `open_viewer()` 方法，把构造器中已有的 passive
Viewer 启动和线程识别逻辑移动到该方法。`RightArmRobot(viewer=True)` 仍在 reset
后调用 `open_viewer()`，保持其他调用者的既有行为；重复调用不创建第二个窗口。

`run_guarded_chop` 改为始终先构造 `RightArmRobot(viewer=False)`。完成预检、scene
配置、双臂与手指状态设置以及 `mj_forward` 后，如果调用者请求 Viewer，则：

1. 调用 `robot.open_viewer()`；
2. 设置 guarded-chop 专用相机；
3. 创建 `GuardedChopTrace` 并设置 5 个规划切点；
4. 以 `GUARD_READY` 记录并显示约 1 秒正确初始姿态；
5. 继续执行原有稳定等待和切菜循环。

现有 `guard_ready_duration_s=2.0` 已包含足够的初始保持时间，不新增配置项；关键是
该保持阶段必须发生在 Viewer 打开之后。外部传入 trace 时继续使用该 trace，不重复
创建默认 trace。

## 错误处理

- 预检失败时 Viewer 尚未打开，任务返回原有 `ABORTED` 结果，不闪现窗口。
- Viewer 启动失败时异常按现有调用路径传播，`finally` 仍调用 `robot.close()`。
- `open_viewer()` 重复调用时直接返回，避免重复窗口和线程引用泄漏。
- headless 路径永远不调用 `open_viewer()`。

## 测试与验收

自动测试验证：

- `RightArmRobot.open_viewer()` 幂等，并保留构造参数 `viewer=True` 的兼容行为。
- guarded-chop 在预检、scene 配置和最终姿态设置完成之前不调用 `open_viewer()`。
- Viewer/trace 创建发生在最终 `mj_forward` 之后，并在动作前收到 `GUARD_READY`。
- headless 运行不打开 Viewer。
- 现有 5 刀、4 次倒手、安全距离、object 模式和完整仿真回归继续通过。

人工验收通过 `./scripts/run_guarded_chop.sh` 连续启动两次：窗口首帧不再出现红色、
绿色或两个灰色抓取物体，右刀和左手从正确切菜初始姿态开始，保持后再执行动作。

## 实现进展

2026-08-01 已完成幂等 `RightArmRobot.open_viewer()` 和 guarded-chop 延迟开窗。
自动启动顺序测试在 `open_viewer()` 调用时直接验证四个抓取物体均已隐藏、Wuji
Hand 已处于猫爪姿态，并验证相机准备及 trace 创建发生在开窗之后。待完整回归和
两次真实 Viewer 首帧检查完成后记录最终验收结果。

独立审查补充验证了预检计划的左右臂 ready 关节、控制量与最终 `mj_forward` 均在
开窗前完成，并确认 Viewer 启动异常会向上传播且仍清理资源，结论为 `READY`。

完整仿真回归结果为 `187 passed`，另有 1 条既有接触力观测告警；平面 headless
仍完成 5 刀、4 次倒手和 `0.080 m` 总退让，最小刀手距离 `0.039 m`，返回
`success=True`。真实 Viewer 首帧检查仍待人工确认。
