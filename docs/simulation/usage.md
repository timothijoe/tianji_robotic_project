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

