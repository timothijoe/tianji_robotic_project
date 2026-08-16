# Wuji 灵巧手轨迹回放测试

## 程序文件

`real_robot_debug/hand_only_stream.py`

## Python 环境

**必须使用 `.venv-wujihand/bin/python`**，因为 `wujihandpy` SDK 只装在这个虚拟环境里。

```bash
.venv-wujihand/bin/python -c "from wujihandpy import Hand; print('SDK ok')"
```

如果直接用系统 `python3` 会报错 `ModuleNotFoundError: No module named 'wujihandpy'`。

## 用途

只控制 Wuji 灵巧手，不连接机械臂。从 200Hz NPZ 读取手部轨迹，在真机上慢速回放，用于验证手部动作效果。

## 测试流程

### 1. 先干跑（只看规划，不连设备）

```bash
PYTHONPATH=. .venv-wujihand/bin/python real_robot_debug/hand_only_stream.py \
  --source-npz recordings/recorded_hand_guarded_chop_200hz_official_right_retarget_candidate_forward_x_plus_80mm_right_z_plus_130mm.npz
```

### 2. 真机回放（默认 0.2x 慢速）

```bash
PYTHONPATH=. .venv-wujihand/bin/python real_robot_debug/hand_only_stream.py \
  --source-npz recordings/recorded_hand_guarded_chop_200hz_official_right_retarget_candidate_forward_x_plus_80mm_right_z_plus_130mm.npz \
  --execute
```

### 3. 更慢速（0.1x）方便观察

```bash
PYTHONPATH=. .venv-wujihand/bin/python real_robot_debug/hand_only_stream.py \
  --source-npz recordings/recorded_hand_guarded_chop_200hz_official_right_retarget_candidate_forward_x_plus_80mm_right_z_plus_130mm.npz \
  --execute --speed-scale 0.1
```

### 4. 自定义缓入时长

```bash
PYTHONPATH=. .venv-wujihand/bin/python real_robot_debug/hand_only_stream.py \
  --source-npz 你的文件.npz \
  --execute --speed-scale 0.2 --entry-duration-s 8
```

## 参数说明

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--source-npz` | (默认路径) | 轨迹 NPZ 文件路径 |
| `--speed-scale` | 0.2 | 回放倍速 (0.01~1.0) |
| `--entry-duration-s` | 5.0 | 缓入到首帧的时长（秒） |
| `--execute` | (不加) | 加此参数才连接真机 |

## 安全机制

| 情况 | 行为 |
|---|---|
| 不加 `--execute` | 只干跑，不连接设备 |
| 手指关节超限 | 自动裁剪到硬件限位内 |
| 正常播放完成 | 手停在末帧姿态，保持使能 |
| **Ctrl+C 中断** | 手停在当前姿态，保持使能 |
| 手指卡碰 / 需要恢复 | 运行下方 reset 命令 |

## 恢复张开（任何时候）

```bash
.venv-wujihand/bin/python scripts/reset_wuji_hand_fast.py
```

程序运行后手会保持使能状态，方便观察。观察完毕后用 reset 命令安全恢复张开并自动去使能。

## 快速命令速查

```bash
# 干跑
PYTHONPATH=. .venv-wujihand/bin/python real_robot_debug/hand_only_stream.py --source-npz 你的文件.npz

# 真机测试
PYTHONPATH=. .venv-wujihand/bin/python real_robot_debug/hand_only_stream.py --source-npz 你的文件.npz --execute

# 恢复张开
.venv-wujihand/bin/python scripts/reset_wuji_hand_fast.py
```