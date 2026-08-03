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
PLACE 将掌面调整为与桌面平行，并让四个长指的实际碰撞表面贴近桌面；RETREAT
解除指尖固定约束，允许指尖随收指动作逐渐离开桌面。手掌后退 30 mm，拇指至少
离桌面 10 mm。PLACE 先使用 3 mm 指尖参考高度进行表面标定，再以真实碰撞面
修正接触；任意手部碰撞几何的数值穿透容差为 0.5 mm。完整轨迹在 Viewer 打开
前完成预检。

真实示例会自动选择第 410 帧。缩短阶段时间的 Headless 验证生成 851 帧，结果
为后退 30 mm、拇指最小间隙 83.1 mm、最大真实穿透 0.327 mm、后退阶段最大
指尖抬升 58.5 mm。侧视 Viewer 已分别检查 PLACE 与 HOLD：初始掌面水平、长指
贴近桌面，最终掌根后退且四指弯曲抬起，桌面没有穿过掌根或指节。

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
