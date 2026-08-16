# Wuji 灵巧手快速恢复脚本说明

## 文件位置

`scripts/reset_wuji_hand_fast.py`

## 用途

从任意姿态快速返回张开（零位）安全姿势，特别是在手指间碰撞、手势异常或需要紧急停止时。

## 命令

```bash
# 快速恢复（默认 0.8s 缓入）
.venv-wujihand/bin/python scripts/reset_wuji_hand_fast.py

# 自定义缓入/保持时间
.venv-wujihand/bin/python scripts/reset_wuji_hand_fast.py --ramp 1.5 --hold 1.0
```

## 与 `scripts/reset_wuji_hand.py` 的区别

| 特性 | 原 `reset_wuji_hand.py` | 新 `reset_wuji_hand_fast.py` |
|---|---|---|
| 默认缓入 | 1.0s | **0.8s**（`--ramp` 可调） |
| 中断处理 | 直接去使能，手停在半途 | **Ctrl+C 后强制下张开目标再退出** |
| 写入方式 | 逐关节串行调用 | **批量 `write_joint_target_position` 五指同时运动** |
| 可复用性 | 仅 main 入口 | 导出 `run()` 函数，可被其他脚本 import |

## 安全机制

- 目标 clamped 在硬件限位内（0.02 rad 余量）
- 错误码非零时拒绝执行并报错
- 全程使能，执行完自动去使能
- Ctrl+C 中断后：先强制下发张开目标 → 等 0.3s → 再去使能（避免停在碰撞姿态）

## 典型场景

```bash
# 场景 1：刚从握拳恢复，但感觉手指间有卡碰
.venv-wujihand/bin/python scripts/reset_wuji_hand_fast.py

# 场景 2：正在播放轨迹，发现动作不对 → Ctrl+C → 立即跑此脚本
.venv-wujihand/bin/python scripts/reset_wuji_hand_fast.py

# 场景 3：在其他 Python 脚本中作为安全兜底直接调用
from reset_wuji_hand_fast import run
run(ramp=1.0, hold=0.5)
```

## 创建记录

- 创建日期：2026-08-16
- 起因：握拳手势后五指间出现碰撞，需要比原版更快的恢复机制
- 验证：从张开位恢复，最大跟踪误差 3.9 mrad