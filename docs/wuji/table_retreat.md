# Wuji 录制驱动的贴桌后退手势

该离线工具在 hand-only MuJoCo 场景中回放用户录制的 20 关节动作，并只对手掌
位姿做贴桌和后退修正。它只运行仿真，**不会连接、搜索或控制真机**。

## 推荐 Viewer 命令

```bash
.venv-wuji-teleop/bin/tianji-robot sim wuji-table-retreat \
  recordings/wuji/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap \
  --loops 3
```

`--loops` 接受正整数，省略时默认循环 3 次。每次正向动作严格使用 MCAP 的
相对时间戳（该录制约 4.15 秒）；相邻两次之间生成平滑 RESET，使关节和手掌
连续回到起点。三次动作只有两段 RESET，最后一次停在最终姿势。

Viewer 完成全部循环后继续显示最终姿势，不会自动关闭；手动关闭窗口后命令会
正常退出。如果播放过程中提前手动关闭窗口，回放也会立即停止。Headless 使用
相同的默认循环 3 次，但完成后直接退出，不等待窗口。

推荐输入已经是标准 `/joint_states` MCAP，程序会直接读取，不再经过手套重定向。
原始右手套文件也可作为回退输入：

```bash
.venv-wuji-teleop/bin/tianji-robot sim wuji-table-retreat \
  recordings/wuji/august_02/session_20260802_174440_936.mcap
```

程序按 MCAP topic 判别 `joint_states` 或 `right_glove_skeleton`，不依赖文件名；
后者才调用官方 Wuji 离线重定向器。

## Headless 验证与保存

```bash
.venv-wuji-teleop/bin/tianji-robot sim wuji-table-retreat \
  recordings/wuji/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap \
  --headless \
  --npz recordings/verification/wuji_recorded_retreat.npz \
  --report recordings/verification/wuji_recorded_retreat.json
```

2026-08-04 的两条真实输入验证结果相同：保留全部 499 帧和原始相对时间戳，自动
识别动作区间 `311:464`，仅在该区间将手掌后退 30 mm。掌心向下，准备阶段逐帧
下降到首次几何接触；后退阶段允许手指逐渐离桌，但整只手的数值穿透不超过
0.5 mm。拇指验收下限为 10 mm，本次最小间隙为 34.7 mm；最大穿透为
0.498 mm，关节修正为 0 rad，因此录制中的食指独立动作以及中指、无名指、
小指的相关运动均被保留。

## 行为边界

- 手掌主体以腕部到四个长指根部的固定轴保持水平，不再根据弯曲指尖估算掌轴；
- 掌侧方向同时用官方模型参考点与“屈指朝桌面”运动学检查，避免掌背翻转；
- PREPARE 保持贴桌，RETREAT/HOLD 只沿 Z 轴投影以防穿桌，不修改相对指姿；
- 原始 MCAP 不会被修改；输出 NPZ/JSON 默认放在被 Git 忽略的 `recordings/`。

可用 `--retreat-distance` 和 `--table-height` 调整后退距离与桌高。人工参数不会
绕过掌心方向、拇指间隙、关节步长或整手碰撞预检。用 `--loops N` 修改循环
次数；`N` 必须大于零。
