# 录制手势联动切菜仿真

该案例在同一个 MuJoCo 场景和时钟中运行天机双臂与 Wuji 左手：左手安装在
天机左臂末端，一次连续执行 MCAP 中掌心向下、贴桌后退的动作；右刀不再等待
左手完成，而是在同一个 10 ms 控制采样中同时下发左右臂和 20 路手关节命令。
共执行 5 个同步切削周期。原有 `guarded-chop` 案例保持不变。

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

成功输出包含 `cuts=5`、`hand_cycles=5`、自动选择的桌面抬升量、刀手全几何
距离、最近长指垫相对刀刃的侧向距离范围、最大桌面穿透和拇指间隙。

## 场景与安全边界

- 默认搜索 4–12 cm 的共享桌面抬升量，并选择最低可行值；当前真实录制选择
  4 cm。
- 记录手势的完整掌部坐标系绕桌面法向旋转 `-90°`：手腕、手指朝向和 30 mm
  后退位移一起旋转。后退方向由原来的场景前后 `-X` 改为与五个切点一致的
  左右 `+Y`。
- 新任务为纯桌面模式：刀直接接触砧板顶面；切菜方块、两个拾取底座、拾取
  方块和目标标记仅在该任务实例中隐藏并禁用碰撞，不影响其他任务。
- 机器人自身左侧是世界 `+Y`。预飞用四个长指中最靠刀的 pad 与
  `right_blade_edge_bot` 作为侧向基准，把距离逐帧保持在 `30±3 mm`；右刀的
  `+Y` 位置直接跟踪该指垫，因此刀和左手并行后退而不是分别走独立 shift。
  左腕仍连续后退 32 mm。为同时满足整手安全包络，掌部沿世界 `X` 使用
  `-180 mm` 纵向偏置；当前全几何最近距离为 32 mm。
- 五刀共享一次连续录制，不再重复回放五遍或执行向右 RESET。四个长指 MCP
  屈曲限制在 0.30 rad，主要屈曲转移到 PIP（至少比 MCP 多 0.25 rad）；腕部
  根据指垫真实位移自适应分配 32 mm 后退量，使指垫单步向刀侧波动不超过
  0.5 mm，且四指首末净位移均向机器人左侧。
- RETREAT/HOLD 中通过逐帧 DIP 求解让远节指骨朝向桌面。实测食指到小指的
  最大偏角依次为 7.63°、1.02°、2.86°、5.64°，均低于 15°。PREPARE 最后
  0.3 秒平滑进入压持姿态；PIP 峰峰值为 0.26–0.38 rad，DIP 为
  0.83–1.32 rad，因此保留了比上一版更明显的录制动作变化。
- 旧的 `HAND_SAFE → CUT_DOWN` 串行互锁已经删除。每个同步周期把刀的下降/回升
  波形重采样到对应的连续手势分段，三个命令向量在同一仿真 tick 前进。
- 预检要求刀与手至少相距 20 mm，整只手穿透桌面不超过 0.5 mm，拇指始终
  离桌至少 10 mm。
- MCAP 的 20 个手关节会裁剪到模型声明的执行器范围，以消除边界浮点误差；
  左臂逐帧逆解，复位段至少 0.5 秒。
- 此命令仅使用 MuJoCo 和离线 MCAP，不导入或访问 Tianji/Wuji 真机驱动、SDK
  连接或 ROS 2 硬件节点。

同步近距离版本的真实 Headless 验收结果为 5 次切菜、一次录制的 5 个连续
分段，桌面抬高 40 mm，侧向距离范围 `30..30 mm`（CLI 三位小数显示）、
计划最小全几何刀手距离 32 mm、手部桌面穿透为 0、拇指最小离桌距离 32 mm。

若预检无法找到安全桌面高度，Viewer 不会打开，命令会返回非零状态并打印限制
条件。忽略目录中的 MCAP 不随 Git 提交，因此换机后需自行放回相同相对路径或
通过 `--hand-mcap` 指定文件。
