# MuJoCo 原生位置控制与巡线切菜开发日志

日期：2026-07-30 至 2026-07-31  
开发分支：`feature/mujoco-position-rebuild`  
隔离工作区：`.worktrees/mujoco-position-rebuild`

## 当前结论

本轮开发将仿真主线从应用层阻抗/力矩控制收敛为 MuJoCo 原生位置执行器，并完成了
可重复安装的 Python 3.12 环境、旧仿真归档、单刀切菜、预切初始化、五刀左右巡线、
CSV 日志、SVG 轨迹图以及 Viewer 内目标/实际 TCP marker。

当前巡线任务从刀面竖直、刀尖朝前的预切姿态启动。每刀执行下降、接触停留和抬起，
前四刀抬起后沿刀面法向向机器人侧方移动 3 cm；第五刀抬起后结束。青色 Viewer
marker 表示完整目标轨迹，橙色 marker 表示运行时实际轨迹。

实体机器人代码和厂商 SDK 始终保持冻结。最终保护清单中的 91 个文件哈希全部一致。

## 一、仿真架构重建

### 背景

旧仿真使用 MuJoCo `motor` 执行器，并在 Python 层维护关节 PD、Cartesian
impedance 和 force control。其控制语义与实体机器不完全一致，开发精力主要消耗在
仿真阻抗参数和控制律维护上。

### 方案

新主线使用 MuJoCo 原生 `position` actuator：

```text
任务目标
  → Cartesian 路径采样
  → 连续 IK
  → 关节位置轨迹
  → MuJoCo position actuator
  → 状态、接触力、Viewer、CSV
```

应用层不再实现阻抗控制。MuJoCo 动力学、碰撞和接触力仍然保留，因此能够观察接触
力的定性趋势。未来若实现导纳控制，应作为修改位置目标的上层模块加入，而不是恢复
当前已移除的仿真阻抗控制器。

### 仓库整理

- 新仿真包：`src/twin_sim/`
- 新场景：`robot_assets/mujoco/right_chopping_scene.xml`
- 新测试：`tests/simulation/`
- 新文档：`docs/simulation/`
- 旧仿真归档：`archive/legacy_simulation/`
- 实体机保护清单：`docs/simulation/protected-files.sha256`

环境版本：

- CPython `3.12`
- MuJoCo `3.10.0`
- NumPy `2.5.1`
- pytest `9.1.1`

## 二、刀具碰撞与切菜姿态调试

### 问题 1：刀片明显插入案板

初始慢速演示中，机械臂关节实际有运动，但刀片视觉上深插案板。诊断发现任务使用
`right_blade_edge_bot` 作为接触参考，而该 site 与刀片碰撞盒的真实最低边缘不
一致。

直接把关节设置到目标接触姿态后，错误碰撞几何的穿透约为 85 mm；受动力学接触
阻挡时仍测得约 57 mm。

### 解决

- 校准刀片视觉体和碰撞体位置。
- 让接触参考 site 落在真实切削边缘。
- 增加回归测试，要求接触目标处刀片穿透不超过 5 mm。
- 修正位置执行器增益，消除旧错误碰撞意外“托住”机械臂后暴露的重力静差。

### 问题 2：刀刃方向不符合切菜观察

最初刀刃长轴与竖直方向仅相差约 25°，刀口看起来接近竖着。曾尝试通过虚拟刀具
安装旋转直接调平，但该方案改变了刀具相对法兰的安装关系，视觉效果不自然。

### 解决

恢复原始刀具安装关系，通过机器人关节姿态调整刀具。随后为满足“刀尖朝前、刀面
竖直”，重新搜索完整可达关节解，而不是强制保持旧 TCP 位置：

- 刀尖方向接近世界 `+X`。
- 刀面高度方向接近世界 `+Z`。
- 刀刃水平和刀面竖直误差均限制在 2° 内。
- 预切姿态直接位于案板上方约 8 cm。

为使上下运动清晰可见，切削行程由约 3 cm 增大到约 8.3 cm。

## 三、marker 与预切初始化

### marker 越界

刀片视觉几何沿刀身方向的有效范围为 `-0.01–0.19 m`，旧 marker 坐标为
`0.10、0.19、0.28 m`，最后一个点超出刀片约 90 mm。

### 解决

三个刀刃 marker 被重新分布到视觉刀片边界内。相关测试直接比较 MJCF 中 site
位置与刀片 box 的局部包围范围，防止后续资产调整再次产生越界。

### 预切初始化

`chop` 默认不再播放站立 HOME、转腕和长距离接近阶段，而是直接把 MuJoCo 状态
重置到预切安全姿态，只执行：

```text
READY → DESCEND → HOLD → RETRACT → COMPLETE
```

旧完整动作通过 `--full-motion` 保留，用于调试 HOME 到预切姿态的连续运动。

## 四、五刀巡线切菜

设计文档：

- `docs/superpowers/specs/2026-07-31-line-chop-trajectory-design.md`
- `docs/superpowers/plans/2026-07-31-line-chop-trajectory.md`

CLI：

```bash
.venv/bin/twin-sim line-chop \
  --cuts 5 \
  --spacing-m 0.03 \
  --log /tmp/twin-sim-line-chop.csv \
  --plot /tmp/twin-sim-line-chop.svg
```

默认状态机：

```text
READY
  → DESCEND_1 → HOLD_1 → RETRACT_1 → SHIFT_1
  → DESCEND_2 → HOLD_2 → RETRACT_2 → SHIFT_2
  → DESCEND_3 → HOLD_3 → RETRACT_3 → SHIFT_3
  → DESCEND_4 → HOLD_4 → RETRACT_4 → SHIFT_4
  → DESCEND_5 → HOLD_5 → RETRACT_5
  → COMPLETE
```

### 问题 1：最初巡线变成前后移动

早期实现把“沿刀刃方向”直接解释成刀身/刀尖方向。刀尖校准到世界 `+X` 后，该
方向就成为机器人本体的前后方向，不符合连续切片需要的左右移动。

### 解决

把平移方向改为刀面法向在 XY 平面的投影：

- 刀尖继续朝世界 `+X`。
- 巡线沿刀面法向世界 `-Y`。
- 每刀侧移 3 cm。
- 5 个切点从 Y=0 附近移动到 Y=-0.12 m 附近。
- 前后 X 偏移测试容差为 1 mm。

`-Y` 和 `+Y` 都通过了 IK 可达性诊断；选择 `-Y` 是因为五刀结束后的最小关节限位
余量约为 0.55 rad，高于另一侧约 0.29 rad。

### 问题 2：冗余姿态与 IK 奇异性

在保持刀尖严格朝前和刀面严格竖直时，第一组关节解使腕部第 6 轴接近限位，3 cm
横移的 IK 残差约 `4.5e-5`，高于项目 `1e-5` 的标准。

### 解决

没有放宽全局 IK 精度，而是在同一 TCP 位姿下搜索冗余关节解。新解为后续左右巡线
保留了更大的腕部与关节限位余量，五刀全部路径能够在执行前完成 IK。

### 预检规则

执行前会完成：

- 刀数、间距和时长参数校验。
- 所有切点和刀刃 marker 的案板 XY 边界检查。
- 五次下降、五次抬起和四次横移的连续 IK。
- 关节范围、有限值和相邻关节跳变检查。

任一段失败时，机器人不会开始运动。

## 五、轨迹日志与可视化

### CSV

`SimulationSample` 增加 `cut_index`：

- `0`：READY 或全局阶段。
- `1..5`：对应刀次。
- `SHIFT` 归入刚完成的刀次。

一次默认快速运行包含：

- `READY`：1 个样本
- `DESCEND`：500 个样本
- `HOLD`：100 个样本
- `RETRACT`：500 个样本
- `SHIFT`：400 个样本
- `COMPLETE`：1 个样本

总计 1502 个样本。

### SVG

`src/twin_sim/trajectory_plot.py` 不依赖 Matplotlib 或 SciPy，直接生成标准 SVG：

- 左侧：X-Y 俯视目标/实际轨迹和切点编号。
- 右侧：水平路径距离-Z 侧视轨迹。
- 目标轨迹为青黄色虚线。
- 实际轨迹为蓝色实线。

### MuJoCo Viewer marker

从旧归档的 `archive/legacy_simulation/src/twin_control/trail.py` 提取了
`user_scn` 球形 geom 注入方式，并在新架构中实现为
`src/twin_sim/viewer_trail.py`：

- 青色大球：完整目标 TCP 轨迹，动画开始前一次性绘制。
- 橙色小球：实际 TCP 轨迹，运行中逐步追加。
- 蓝色球：刀刃自身定位 site。
- 追踪点：`right_tool_tip_site`。
- 默认每 10 个控制周期采样一次，避免耗尽 Viewer geom 容量。
- Headless 模式为 no-op，不改变控制、CSV 或 SVG。

## 六、主要提交记录

与最终功能直接相关的提交：

| 提交 | 内容 |
|---|---|
| `1b1f7fc` | 校准刀片碰撞与 Viewer 慢速展示 |
| `4be19f4` | 使用腕部调平阶段保持真实刀具安装关系 |
| `cffe744` | 默认从预切姿态开始 |
| `80eaff7` | 对齐 marker、刀尖方向和可见切削行程 |
| `9131f74` | CSV 增加刀次索引 |
| `e161fbb` | 实现预检后的五刀巡线状态机 |
| `912b9b0` | 生成无依赖 SVG 轨迹图 |
| `d3609fd` | 暴露 `line-chop` CLI |
| `b94d459` | Viewer 显示目标/实际 TCP marker |
| `8fe57a3` | 将巡线方向修正为刀面法向左右平移 |

## 验证与证据

最终验证命令：

```bash
.venv/bin/python -m pytest tests/simulation -q
sha256sum --check docs/simulation/protected-files.sha256
```

最终结果：

- 仿真测试：`74 passed`
- 预期接触力 warning：1 个测试路径中触发；warning 不改变或中断轨迹
- 保护文件：`91/91 OK`
- Git 工作区：提交后干净

慢速五刀演示：

- 仿真动作时间约 47 s
- 加上 Viewer 开始/结束停留后约 60 s
- 日志约 4702 个数据样本，最后阶段为 `COMPLETE`、`cut_index=5`

## 未完成项与风险

1. MuJoCo Viewer 在部分运行中完成动作后会停在关闭阶段，进程退出码可能为 1；
   CSV、SVG 和任务状态均已完整到 `COMPLETE`。当前通过结束残留 Viewer 进程处理，
   尚未定位底层 GLX/Viewer 生命周期原因。
2. 接触力只用于趋势、日志和阈值 warning，未与实体机器标定，不能把当前牛顿值直接
   用作实体控制参数。
3. 当前巡线是固定 3 cm、固定 5 刀的开环位置轨迹。尚未加入视觉巡线、食材识别、
   导纳控制或依据接触力修正位置目标。
4. 本轮只验证 Linux 仿真环境，未运行或连接实体机器人。

## 下一步

1. 若继续仿真开发，优先解决 Viewer 关闭阶段偶发卡住的问题。
2. 为巡线任务增加可配置的起始切点、左右方向和案板安全裕量。
3. 在保持位置执行器架构的前提下，评估上层导纳位置偏置。
4. 在进入实体机器人验证前，单独制定速度、加速度、工作空间和碰撞安全检查方案。

## 后续开发：左侧 Wuji Hand 接入

在巡线切菜版本之后，将 `wuji-description` 中的原版 20 自由度左手模型接入
`left_link7`。模型通过单一 `left_hand_mount` 安装层连接，手掌、手指、碰撞体、
惯性和位置执行器保持上游数据。所需 MJCF、26 个 STL 和 MIT 许可证均复制到
`robot_assets/mujoco/wuji_hand/`，运行时不依赖下载目录。

主要解决的问题：

1. 上游手模型自带全局 solver 选项，直接组合会改变切菜场景。接入版本移除了
   子模型的全局 option，仅让父场景决定积分器和求解器。
2. 手执行器的控制范围相对关节范围存在小量舍入收缩，模型契约因此对机械臂保持
   严格相等，对手要求控制范围位于关节范围内部。
3. 旧控制路径只认识 14 个机械臂执行器。新增 `LeftHandController` 后，机械臂和
   灵巧手均按名称选择执行器，20 维手目标在完整检查长度、有限性和范围后才原子更新。
4. 默认全零姿态对拇指第一关节越界。新的自然张开姿态使用范围内的非零目标，并在
   reset 时同步写入 qpos、qvel 和 ctrl。

新增 `twin-sim hand-demo` 演示张开、放松闭合、重新张开；`--slow` 将每个运动
阶段延长至 4 秒，并在动作前后分别停留 5 秒和 10 秒，便于检查安装方向、腕部
间隙和五指运动。Viewer 能完成显示和动作，但在自动销毁窗口时仍可能以退出码
139 结束；同一模型的 Headless 演示正常完成，这与前文记录的既有 GLX/Viewer
关闭问题一致。
