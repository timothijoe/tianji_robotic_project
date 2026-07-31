# 巡线切菜与轨迹图设计

日期：2026-07-31

## 目标

在现有 MuJoCo 原生位置控制仿真上增加巡线切菜任务。机器人从当前预切姿态开始，
连续切 5 刀；每刀抬起后沿刀面法向在案板平面内向机器人侧方横移 3 cm，再执行
下一刀。
任务同时输出完整 CSV 和无需额外绘图库即可查看的 SVG 轨迹图。

实体机器人、厂商 SDK 和现有单刀 `chop` 行为不在本次修改范围内。

## 任务状态机

默认循环为：

```text
READY
  ↓
DESCEND_1 → HOLD_1 → RETRACT_1 → SHIFT_1
  ↓
DESCEND_2 → HOLD_2 → RETRACT_2 → SHIFT_2
  ↓
DESCEND_3 → HOLD_3 → RETRACT_3 → SHIFT_3
  ↓
DESCEND_4 → HOLD_4 → RETRACT_4 → SHIFT_4
  ↓
DESCEND_5 → HOLD_5 → RETRACT_5
  ↓
COMPLETE
```

第五刀抬起后结束，不执行多余平移。所有阶段保持当前已校准的刀尖朝前、刀面竖直
姿态。

## 几何与轨迹

- 默认刀数：5。
- 默认切点间距：`0.03 m`。
- 平移方向：预切姿态下刀面法向在世界 XY 平面的单位投影，选择关节限位余量更大
  的 `-Y` 方向。刀尖继续指向世界 `+X`，因此平移与刀尖方向垂直。
- 下切方向：世界 `-Z`。
- 安全高度、接触压入量、控制周期和力阈值复用 `ChopConfig` 的现有语义。
- 每次 `SHIFT` 只改变 XY，不改变 Z 或姿态。
- 全部切点必须落在案板 XY 边界内，并保留刀片碰撞几何所需的边缘裕量。
- 所有 Cartesian 段在执行前完成连续 IK、关节范围和相邻点跳变校验。任一段失败，
  整个任务不得开始运动。

实现采用显式状态机编排各刀轨迹，不把每刀重置为独立仿真，也不把阶段信息压平成
无法诊断的单一 waypoint 列表。

## 接口

Python API：

```python
run_line_chop(
    config: LineChopConfig,
    *,
    log_path: Path,
    plot_path: Path,
    viewer: bool = False,
) -> LineChopResult
```

`LineChopConfig` 组合现有 `ChopConfig`，并增加：

- `cuts: int = 5`
- `spacing_m: float = 0.03`
- `shift_duration_s: float`

CLI：

```bash
twin-sim line-chop \
  --cuts 5 \
  --spacing-m 0.03 \
  --log /tmp/line-chop.csv \
  --plot /tmp/line-chop.svg
```

慢速 Viewer 沿用 `--slow`。Headless 模式不进行墙钟等待。

## 日志

现有 `SimulationSample` 和 CSV 增加可选 `cut_index`：

- `0` 表示 `READY` 或不属于某一刀的全局阶段；
- `1..cuts` 表示对应刀次；
- `SHIFT_n` 归入刚完成的第 `n` 刀。

阶段字段继续使用 `READY`、`DESCEND`、`HOLD`、`RETRACT`、`SHIFT` 和
`COMPLETE`，刀次由 `cut_index` 区分，避免动态阶段名破坏既有分析代码。

## SVG 轨迹图

SVG 包含两个并排视图：

1. 俯视图（X-Y）：目标路径、实际路径、5 个切点和刀次编号。
2. 侧视图（巡线距离-Z）：目标与实际的下降、保持、回升锯齿轨迹。

目标轨迹使用虚线，实际轨迹使用实线；不同阶段通过颜色或图例说明。绘图器直接生成
标准 SVG，不新增 Matplotlib 或 SciPy 依赖。即使 Viewer 不可用，headless 运行也
必须生成 CSV 与 SVG。

## 校验与错误处理

- `cuts` 必须是正整数。
- `spacing_m` 和各阶段时长必须为正有限值。
- 轨迹越出案板、IK 失败、关节越界或数值异常时，执行前失败。
- 力超过阈值只产生 warning 和日志标记，不改变轨迹。
- 日志或 SVG 目标路径不可写时，在创建机器人或执行运动前失败。

## 验收标准

- 默认任务完整记录 5 次 `DESCEND/HOLD/RETRACT` 和 4 次 `SHIFT`。
- 相邻切点沿刀面法向间距为 `0.03 m`，误差不超过 `1 mm`；前后方向偏移不超过
  `1 mm`。
- 每次下切和回升的实际 Z 行程清晰可见。
- 第五刀后无额外 `SHIFT`。
- SVG 同时包含俯视图、侧视图、目标/实际轨迹、切点编号和图例。
- 全量仿真测试通过。
- `SDK_PYTHON/`、`test/`、`real_robot_debug/` 保护哈希保持一致。
