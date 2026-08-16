# 双臂 + Wuji 手联合回放

## 程序文件

`real_robot_debug/dual_arm_hand_batch_stream.py`

## 用途

同时控制 Tianji 双臂（A 臂 + B 臂）和 Wuji 灵巧手，从同一个 200Hz NPZ 同步回放。

- 双臂使用底层低延迟路径（`clear_set()` + `set_joint_cmd_pose()` A/B + 一次 `send_cmd()`），已验证 200 Hz
- 手部使用批量 `write_joint_target_position(5x4)`，五指同时
- 双臂和手在同一个 wall-clock 时间轴上同步

## Python 环境

手部 SDK `wujihandpy` 在 `.venv-wujihand` 里，**必须用 `.venv-wujihand/bin/python` 运行**。

## 测试步骤

### 1. 先干跑（不连接任何设备，只看规划）

```bash
PYTHONPATH=. .venv-wujihand/bin/python real_robot_debug/dual_arm_hand_batch_stream.py \
  --source-npz recordings/recorded_hand_guarded_chop_200hz_official_right_retarget_candidate_forward_x_plus_80mm_right_z_plus_130mm.npz
```

### 2. 真机联合回放（双臂 + 手）

```bash
PYTHONPATH=. .venv-wujihand/bin/python real_robot_debug/dual_arm_hand_batch_stream.py \
  --source-npz recordings/recorded_hand_guarded_chop_200hz_official_right_retarget_candidate_forward_x_plus_80mm_right_z_plus_130mm.npz \
  --execute \
  --entry-duration-s 15 \
  --speed-scale 0.05
```

> 注意：高速率要控制机器人，`--speed-scale 0.05` 是最安全的首测速度。

### 3. 只控制双手臂（不加手）

```bash
PYTHONPATH=. .venv-wujihand/bin/python real_robot_debug/dual_arm_hand_batch_stream.py \
  --source-npz recordings/recorded_hand_guarded_chop_200hz_official_right_retarget_candidate_forward_x_plus_80mm_right_z_plus_130mm.npz \
  --execute --arm-only
```

### 4. 只控制手

```bash
PYTHONPATH=. .venv-wujihand/bin/python real_robot_debug/dual_arm_hand_batch_stream.py \
  --source-npz recordings/recorded_hand_guarded_chop_200hz_official_right_retarget_candidate_forward_x_plus_80mm_right_z_plus_130mm.npz \
  --execute --hand-only
```

### 5. 只读探针（不移动，验证双臂反馈）

```bash
PYTHONPATH=. .venv-wujihand/bin/python real_robot_debug/dual_arm_hand_batch_stream.py \
  --source-npz recordings/recorded_hand_guarded_chop_200hz_official_right_retarget_candidate_forward_x_plus_80mm_right_z_plus_130mm.npz \
  --execute --probe-current
```

## 参数说明

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--source-npz` | (默认路径) | 轨迹 NPZ 文件 |
| `--speed-scale` | 0.05 | 回放倍速 (0.01~1.0) |
| `--entry-duration-s` | 15.0 | 双臂缓入时长（秒） |
| `--execute` | (不加) | 加此参数才连接真机 |
| `--hand-only` | (不加) | 只控制手，不碰双臂 |
| `--arm-only` | (不加) | 只控制双臂，不碰手 |
| `--probe-current` | (不加) | 只读探针：在当前姿态保持 200 帧验证反馈 |

## 安全机制

| 情况 | 行为 |
|---|---|
| 不加 `--execute` | 只干跑，不连接设备 |
| 双臂关节超限 | 拒绝执行（1° 余量） |
| 手指关节超限 | 自动裁剪到硬件限位内 |
| 反馈帧不递增 / state≠3 | 报错停止 |
| 跟踪误差 >5° | 报错停止 |
| 程序结束 | 手去使能 + 双臂禁用 + 释放机器人 |

## 恢复张开（任何时候）

```bash
.venv-wujihand/bin/python scripts/reset_wuji_hand_fast.py
```

## 快速命令速查

```bash
# 干跑
PYTHONPATH=. .venv-wujihand/bin/python real_robot_debug/dual_arm_hand_batch_stream.py --source-npz 你的文件.npz

# 真机双臂+手
PYTHONPATH=. .venv-wujihand/bin/python real_robot_debug/dual_arm_hand_batch_stream.py --source-npz 你的文件.npz --execute --entry-duration-s 15 --speed-scale 0.05

# 恢复张开
.venv-wujihand/bin/python scripts/reset_wuji_hand_fast.py
```