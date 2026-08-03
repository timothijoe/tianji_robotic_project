# Wuji 四指落桌后退手势

该离线工具从手套 MCAP 自动选择一个可行姿态，让四个长指落在桌面上，保持拇指
抬起，再收指并带动手掌后退。它只运行 MuJoCo，**不会连接、搜索或控制真机**。

## Viewer

```bash
.venv-wuji-teleop/bin/tianji-robot sim wuji-table-retreat \
  recordings/wuji/august_02/session_20260802_162909_764.mcap
```

## Headless 与保存结果

```bash
.venv-wuji-teleop/bin/tianji-robot sim wuji-table-retreat \
  recordings/wuji/august_02/session_20260802_162909_764.mcap \
  --headless \
  --npz recordings/verification/wuji_table_retreat.npz \
  --report recordings/verification/wuji_table_retreat.json
```

默认动作是 `PLACE → RETREAT → HOLD`：落桌 1 秒、后退 2 秒、保持 2 秒。
手掌后退 30 mm，拇指至少离桌面 10 mm；四个长指允许的最大接触点滑移为
3 mm，高度误差和穿透上限为 2 mm。完整轨迹在 Viewer 打开前完成预检。

真实示例会自动选择第 410 帧，生成 2501 帧修正动作。当前验证结果为后退
30 mm、拇指最小间隙 10.8 mm、最大指尖高度误差 0.39 mm、最大滑移 1.41 mm。

## 人工覆盖

```bash
# 指定源帧和后退距离
.venv-wuji-teleop/bin/tianji-robot sim wuji-table-retreat INPUT.mcap \
  --source-frame 410 --retreat-distance 0.04

# 修改桌高和三个阶段时间
.venv-wuji-teleop/bin/tianji-robot sim wuji-table-retreat INPUT.mcap \
  --table-height 0.02 \
  --place-duration 1.5 --retreat-duration 2.5 --hold-duration 3.0
```

人工指定帧不会绕过可行性和安全检查。原始 MCAP 不会被修改，原始动作仍使用
`tianji-robot sim wuji-replay` 查看。
