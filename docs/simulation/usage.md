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

## 双臂猫爪倒手切菜

直接从任意工作目录打开默认平面场景 Viewer：

```bash
./scripts/run_guarded_chop.sh
```

该脚本会自动定位仓库路径，并使用仓库内的 `.venv`。它只运行 MuJoCo 仿真，
不会向实体机械臂、Wuji Hand 或 ROS 2 硬件接口发送命令。

该入口只播放一遍 1× 在线仿真。若要先观看相同的 1× 在线执行，再在同一窗口
自动观看一遍 2× 状态回放，运行：

```bash
./scripts/run_guarded_chop_record_replay.sh
```

第二遍只恢复第一遍记录的 MuJoCo `qpos`、`qvel`、`ctrl` 和时间戳并执行
`mj_forward`，不会重新运行位置控制器、接触求解或安全决策。Viewer 状态栏会显示
`replay 2.0x`，用于区分在线执行与状态回放。

录制默认只保存在内存。需要保存时可以使用：

```bash
# 保存到 recordings/guarded_chop_latest.npz；重复运行时原子覆盖
./scripts/run_guarded_chop_record_replay.sh --record

# 指定不同名称时保留为独立文件
./scripts/run_guarded_chop_record_replay.sh \
  --record recordings/demo_01.npz
```

NPZ 保存仿真时间、完整 `qpos/qvel/ctrl`、动作阶段、刀次和 marker 数据。同一路径
不会生成冗余副本；模型状态维度与当前 MuJoCo 模型不一致的录制会被拒绝加载。
`recordings/` 已加入 `.gitignore`，不会被误提交。

其他命令行模式：

```bash
.venv/bin/twin-sim guarded-chop
.venv/bin/twin-sim guarded-chop --headless --final-hold 0
.venv/bin/twin-sim guarded-chop --scene object
.venv/bin/twin-sim guarded-chop --replay-rate 2.0
.venv/bin/twin-sim guarded-chop --replay-rate 2.0 --record
```

默认 `plane` 场景在砧板平面上从屏幕右侧向左侧完成 5 刀。每次右刀下切并稳定
后保持在低刀位，Wuji Hand 依次开手、左臂后退 2 cm、合手并稳定；随后左手
保持不动，右刀通过一段斜向轨迹同时抬升并横移到下一切点，再次下落。前 4 刀
之后各完成一次倒手，总倒手 0.08 m。低刀位倒手期间右刀必须静止，斜向换刀
期间左手必须静止。

Viewer 会在轨迹预检、无关物体隐藏和双臂初始姿态设置完成后才打开。窗口首帧
直接显示正确切菜场景，并以 `GUARD_READY` 保持初始姿态后开始动作，不会先闪现
抓取任务的红/绿物体、灰色底座或通用举刀姿态。

Viewer 中蓝线是右刀规划路径，青线是右刀实际轨迹，紫线是左手护手实际轨迹，
并在砧板上保留 5 个紧凑的 cut marks；右上角显示刀数、阶段、刀手距离和下切
许可。`--scene object` 保留先前固定红色方块的接触标定，供回归验证使用，
但不是默认演示。方块不会被切分、抓起或推动。

该命令只运行 MuJoCo，不会向实体机械臂或 Wuji Hand 发送命令。

## 灵巧手控制

左臂安装了 20 自由度 Wuji Hand。运行无窗口开合验证：

```bash
.venv/bin/twin-sim hand-demo --headless
```

关节顺序、控制范围、Python 发送与状态读取示例见
[Wuji Hand 左手控制说明](wuji_hand_control.md)。
