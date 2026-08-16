# Wuji 灵巧手半握拳控制脚本

## 文件

- `scripts/half_fist_wuji_hand_hold.py` — 半握拳并保持使能（不自动恢复）

## 用途

从当前姿态缓入到半握拳，然后保持使能，让手停在半握拳状态。需要手动运行 `reset_wuji_hand_fast.py` 恢复张开。

## 命令

```bash
# 3s 缓入到半握拳，到达后立即退出（保持使能）
.venv-wujihand/bin/python scripts/half_fist_wuji_hand_hold.py

# 缓入后保持 10s 再退出（仍保持使能）
.venv-wujihand/bin/python scripts/half_fist_wuji_hand_hold.py --ramp 3 --hold 10

# 长时间保持（例如 60 秒）
.venv-wujihand/bin/python scripts/half_fist_wuji_hand_hold.py --ramp 3 --hold 60
```

## 恢复张开

```bash
.venv-wujihand/bin/python scripts/reset_wuji_hand_fast.py
```

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--ramp` | 3.0s | 缓入到半握拳的时间 |
| `--hold` | 0s | 到达后保持时间（0=到达即退出，仍保持使能） |

## 安全机制

- 目标 clamped 在硬件限位内（0.02 rad 余量）
- 错误码非零时拒绝执行
- Ctrl+C 中断后强制回张开再去使能
- 程序退出后手保持使能，USB 连接释放，可被 reset 脚本接管

## 半握拳定义

```text
拇指: [0.45, 0.30, 0.50, 0.45]
食指: [0.60, 0.13, 0.60, 0.55]
中指: [0.60, 0.12, 0.60, 0.55]
无名指: [0.60, 0.11, 0.60, 0.50]
小指: [0.55, 0.11, 0.55, 0.50]
```

约全握拳的 50%，五指自然弯曲。

## 创建记录

- 创建日期：2026-08-16
- 起因：需要半握拳姿势进行手指协调性和碰撞测试
- 与 `half_fist_wuji_hand.py` 的区别：`half_fist_wuji_hand_hold.py` 不自动恢复张开，需要手动 reset