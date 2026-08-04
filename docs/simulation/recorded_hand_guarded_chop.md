# 录制手势联动切菜仿真

该案例在同一个 MuJoCo 场景和时钟中运行天机双臂与 Wuji 左手：左手安装在
天机左臂末端，每次完整执行一次 MCAP 中掌心向下、贴桌后退的动作；到达安全
后退位后，右臂完成一次切菜。共执行 5 轮。原有 `guarded-chop` 案例保持不变。

## Viewer 运行

在项目根目录执行：

```bash
./scripts/run_recorded_hand_guarded_chop.sh
```

也可以传入另一个兼容的 MCAP：

```bash
./scripts/run_recorded_hand_guarded_chop.sh recordings/wuji/august_02/example.mcap
```

等价的直接命令是：

```bash
.venv-wuji-teleop/bin/twin-sim recorded-hand-guarded-chop \
  --hand-mcap recordings/wuji/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap
```

程序完成后默认保持最终姿势 3 秒；可用 `--final-hold 10` 延长观察时间。

## Headless 验证

```bash
.venv-wuji-teleop/bin/twin-sim recorded-hand-guarded-chop \
  --headless --final-hold 0 \
  --hand-mcap recordings/wuji/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap
```

成功输出包含 `cuts=5`、`hand_cycles=5`、自动选择的桌面抬升量，以及刀手距离、
最大桌面穿透和拇指间隙。

## 场景与安全边界

- 默认搜索 4–12 cm 的共享桌面抬升量，并选择最低可行值；当前真实录制选择
  4 cm。
- 每一刀必须晚于对应手势的 `HAND_SAFE` 事件。
- 预检要求刀与手至少相距 20 mm，整只手穿透桌面不超过 0.5 mm，拇指始终
  离桌至少 10 mm。
- MCAP 的 20 个手关节会裁剪到模型声明的执行器范围，以消除边界浮点误差；
  左臂逐帧逆解，复位段至少 0.5 秒。
- 此命令仅使用 MuJoCo 和离线 MCAP，不导入或访问 Tianji/Wuji 真机驱动、SDK
  连接或 ROS 2 硬件节点。

若预检无法找到安全桌面高度，Viewer 不会打开，命令会返回非零状态并打印限制
条件。忽略目录中的 MCAP 不随 Git 提交，因此换机后需自行放回相同相对路径或
通过 `--hand-mcap` 指定文件。
