# 仿真使用方法

以下命令均从仓库根目录运行。

## Viewer

```bash
.venv/bin/twin-sim view
```

## 关节位置运动

```bash
.venv/bin/twin-sim joint --joint 1 --delta-rad 0.05 --headless
```

去掉 `--headless` 可以打开 Viewer。`--joint` 范围是 1–7。

## Cartesian IK 运动

```bash
.venv/bin/twin-sim cartesian --dz-m 0.02 --headless
```

命令保持 TCP 朝向，以连续 IK 计算完整路径后再开始执行。任一点不可达时整条路径
在运动前失败。

## 单次切菜

```bash
.venv/bin/twin-sim chop --headless --log /tmp/twin-sim-chop.csv
```

任务依次执行 `APPROACH`、`DESCEND`、`HOLD`、`RETRACT` 和 `COMPLETE`。
CSV 包含目标/实际关节、目标/实际 TCP pose、原始/滤波接触力和阈值标志。
超过力阈值时任务继续；这只是观测告警，不是力控。

Python API 的最小示例位于 `examples/simulation_demo.py`。

## 巡线切菜与轨迹图

```bash
.venv/bin/twin-sim line-chop \
  --cuts 5 \
  --spacing-m 0.03 \
  --headless \
  --log /tmp/twin-sim-line-chop.csv \
  --plot /tmp/twin-sim-line-chop.svg
```

任务默认沿刀面法向向机器人侧方连续切 5 刀，相邻切点间距 3 cm，刀尖继续朝前。
每刀依次执行
`DESCEND`、`HOLD`、`RETRACT`，前 4 刀之后追加 `SHIFT`。CSV 的
`cut_index` 标明刀次；SVG 同时展示目标/实际轨迹的俯视图和侧视图。
去掉 `--headless` 可打开 Viewer，增加 `--slow` 可慢速播放。

## 灵巧手控制

左臂安装了 20 自由度 Wuji Hand。运行无窗口开合验证：

```bash
.venv/bin/twin-sim hand-demo --headless
```

关节顺序、控制范围、Python 发送与状态读取示例见
[Wuji Hand 左手控制说明](wuji_hand_control.md)。
