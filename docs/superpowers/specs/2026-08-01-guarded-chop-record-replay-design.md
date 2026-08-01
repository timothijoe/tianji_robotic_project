# Guarded Chop 1× 录制与 2× 回放设计

## 用户体验

保留现有实时入口：

```bash
./scripts/run_guarded_chop.sh
```

它继续以 1× 墙钟速度运行仿真，与真实环境的时间尺度一致，不录制、不回放。

新增组合入口：

```bash
./scripts/run_guarded_chop_record_replay.sh
```

它先在 Viewer 中以 1× 完整执行并观看切菜，同时把状态帧保存在内存；执行成功后，在
同一 Viewer 中自动从起点按 2× 回放。第一次是正常控制和物理仿真，第二次只恢复已经
记录的状态，不重新运行控制器、安全决策或接触求解。

组合入口默认不写文件。需要保存时使用：

```bash
./scripts/run_guarded_chop_record_replay.sh --record
./scripts/run_guarded_chop_record_replay.sh --record recordings/demo_01.npz
```

无路径的 `--record` 使用 `recordings/guarded_chop_latest.npz`。相同路径再次录制时原子
覆盖旧文件，不生成序号副本；显式给出不同路径时保留不同记录。

## 架构与组件

新增 `twin_sim.guarded_chop_recording`，负责三件事：

1. 以不可变帧记录每个控制步之后的 MuJoCo `time`、`qpos`、`qvel`、`ctrl`，以及回放
   marker 和状态栏所需的 phase、cut index、刀刃点、护手点、最小距离和允许状态；
2. 使用带版本号的压缩 NPZ 格式保存和读取记录；
3. 把记录帧恢复到现有机器人模型，并调用 `mj_forward` 和 Viewer/trace 更新完成回放。

录制数据不包含模型 XML。文件保存模型状态维度和格式版本；读取时必须与当前模型的
`nq`、`nv`、`nu` 匹配，否则给出明确错误，不尝试部分加载。

`run_guarded_chop` 增加可选的 recording/replay 配置。只有请求回放或落盘时才复制状态，
因此现有实时入口和 headless 集成测试不会承担录制内存开销。正常执行成功后先完成结果
校验和可选原子保存，再在仍然打开的同一 Viewer 中回放；失败或安全中止时不自动回放，
但如果请求落盘则保存截至中止点的帧并标记结果未完成。

CLI 为 `guarded-chop` 增加：

- `--replay-rate RATE`：成功运行后按指定倍率回放；组合脚本固定传 `2.0`；
- `--record [PATH]`：可选保存，省略 PATH 时使用固定 latest 路径。

倍率必须为有限正数。回放要求 Viewer；`--headless --replay-rate` 被 CLI 明确拒绝，
headless 仍允许只录制到文件。

## 时间与控制语义

1× 执行阶段保持当前 `control_dt_s=0.01`：MuJoCo 每步前进 0.01 秒，Viewer 每步按现有
逻辑等待 0.01 秒。左右臂和 Wuji Hand 仍是关节位置伺服，轨迹、力/距离阈值和安全
互锁均不改变。

2× 回放不调用 `mj_step`，而是依次恢复所有记录帧并执行 `mj_forward`。相邻帧的墙钟
等待为记录仿真时间差除以 2；Viewer 同步仍限制在当前显示刷新频率附近，不能因为倍率
提高而改变记录帧顺序。回放结束后保持最后一帧，随后按正常清理关闭窗口。

## Marker 与可见状态

计划切点继续使用蓝色，实际刀轨迹继续使用青色，护手轨迹继续使用紫色。回放开始时
清空运行阶段的实际轨迹缓存，再从记录帧重建，以免两遍轨迹叠加造成误判；计划轨迹
保留。状态栏在 phase 前显示 `replay 2.0x`，让用户能明确分辨实时执行和回放阶段。

## 文件安全

保存时先在目标目录创建临时文件，完整写入并关闭后用 `os.replace` 原子替换目标路径。
父目录不存在时自动创建。写入失败时清理临时文件并保留旧记录；错误向上传播，不能把
仿真成功误报为录制成功。默认 `recordings/` 加入 `.gitignore`，避免二进制记录进入仓库。

## 测试与验收

自动测试覆盖：

- 记录帧复制数据而不是持有 MuJoCo 可变数组引用；
- NPZ 保存、加载和模型维度校验，默认路径覆盖且无冗余文件；
- 2× 回放恢复所有状态，调用 `mj_forward`，等待时间为帧间隔的一半且不调用 `mj_step`；
- marker 在回放前清理并按记录重建，状态栏标明 2×；
- CLI 的可选 `--record` 路径、固定 latest 路径、倍率校验和 headless 限制；
- 两个 shell 入口分别传递 1× 实时模式和“1× 执行后 2× 回放”模式；
- 既有 5 刀、4 次倒手、`0.080 m` 退让、安全距离和完整测试继续通过。

人工验收运行组合脚本，确认第一遍与当前已验收的真实速度一致，结束后同一 Viewer 从
起点明确进入 2× 回放；两遍姿态、物体和 marker 轨迹一致。连续两次使用默认 `--record`
后目录中只有一个 `guarded_chop_latest.npz`，指定其他文件名时两个文件同时保留。

## 实现进展

2026-08-01 已完成不可变状态帧、版本 1 压缩 NPZ、模型维度校验、原子覆盖保存、纯状态
Viewer 回放、回放轨迹重置与 `replay 2.0x` 状态栏。`run_guarded_chop` 仅在请求保存或
回放时捕获帧；普通 1× 入口不承担录制开销。CLI 已支持 `--replay-rate` 和可选路径的
`--record`，并拒绝无 Viewer 的回放请求。

新增脚本为：

```bash
# 只看与真实时间一致的 1× 在线仿真
./scripts/run_guarded_chop.sh

# 先看 1× 在线仿真，再在同一窗口看 2× 状态回放；不落盘
./scripts/run_guarded_chop_record_replay.sh

# 同上，并原子覆盖默认 latest 文件
./scripts/run_guarded_chop_record_replay.sh --record

# 同上，保存为独立名称
./scripts/run_guarded_chop_record_replay.sh --record recordings/demo_01.npz
```

完整自动回归结果为 `202 passed`，另有 1 条既有的 39.611 N 接触力观测告警。默认路径
连续录制两次后只有一个 `guarded_chop_latest.npz`；随后指定不同路径得到第二个文件。
两份文件均可重新加载，各包含 3115 帧且末帧 phase 为 `complete`。

真实组合 Viewer 已完成一遍 1× 在线执行和同窗口 2× 状态回放，程序最终返回
`success=True`、5 刀、4 次倒手、`0.080 m` 总退让和 `0.039 m` 最小刀手距离。自动与
程序运行验收通过；两遍视觉节奏和状态栏辨识度等待用户确认。
