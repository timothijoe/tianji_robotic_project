# Wuji 左手轨迹回放与复位操作手册

更新日期：2026-08-05

本文档记录当前 Wuji 左手（SN: `LQSQJR.260616.005`，USB: `365939643134`，固件 `1.2.1`）
的 SDK 直控命令。所有命令从仓库根目录 `/home/zhoutong/august_folder/tianji_robotic_project` 运行。

## 硬件状态速查

| 属性 | 值 |
|---|---|
| 物理连接 | `/dev/ttyACM0` (USB) |
| USB 序列号 | `365939643134` |
| 产品序列号 | `LQSQJR.260616.005` |
| 手性 | 左手 (handedness=0) |
| 固件 | `1.2.1` |
| SDK | `1.8.0` |
| 关节限位 | `[-0.688, 1.684]` rad（各关节不同，见下方详细限位表）|
| 控制方式 | SDK 直控（非 ROS） |
| Python 环境 | `.venv-wujihand/bin/python` |

## 各关节硬件限位 (rad)

| 手指 | Joint 1 (MCP) | Joint 2 (PIP) | Joint 3 (DIP) | Joint 4 |
|---|---|---|---|---|
| 拇指 F1 | [-0.119, 1.679] | [-0.257, 0.950] | [-0.550, 1.663] | [-0.582, 1.641] |
| 食指 F2 | [-0.262, 1.646] | [-0.439, 0.342] | [-0.529, 1.672] | [-0.505, 1.640] |
| 中指 F3 | [-0.234, 1.644] | [-0.448, 0.330] | [-0.569, 1.644] | [-0.572, 1.662] |
| 无名指 F4 | [-0.242, 1.644] | [-0.478, 0.275] | [-0.576, 1.679] | [-0.543, 1.684] |
| 小指 F5 | [-0.258, 1.632] | [-0.471, 0.292] | [-0.688, 1.536] | [-0.572, 1.646] |

## 1. 播放轨迹

```bash
# 默认 0.2x 速度，3 秒缓入
.venv-wujihand/bin/python scripts/play_wuji_trajectory.py /tmp/wuji_right_retarget_WuzTwX.npz

# 更慢（0.1x）
.venv-wujihand/bin/python scripts/play_wuji_trajectory.py /tmp/wuji_right_retarget_WuzTwX.npz --speed 0.1

# 更长缓入时间
.venv-wujihand/bin/python scripts/play_wuji_trajectory.py /tmp/wuji_right_retarget_WuzTwX.npz --speed 0.2 --ramp 5.0

# 指定其他 NPZ 文件
.venv-wujihand/bin/python scripts/play_wuji_trajectory.py 任意轨迹文件.npz --speed 0.2
```

**脚本行为**：
1. 加载 NPZ 轨迹，对比硬件限位（自动裁剪超限关节并打印 warning）
2. 检查关节错误码（非零则拒绝）
3. 使能 → 缓入到首帧 → 按倍速逐帧回放 → 去使能

**Ctrl+C 安全**：
- 按 `Ctrl+C` 会立即停止播放并自动去使能
- 屏幕会提示复位命令
- 手可能停在半途姿态

## 2. 复位到张开手

```bash
.venv-wujihand/bin/python scripts/reset_wuji_hand.py

# 更慢的缓入
.venv-wujihand/bin/python scripts/reset_wuji_hand.py --ramp 2.0
```

**脚本行为**：
1. 读取当前关节位置
2. 目标：全零位（手张开）
3. 使能 → 缓入到零位 → 保持 1 秒 → 去使能

## 3. 典型操作流程

```bash
# Step 1: 播放轨迹
.venv-wujihand/bin/python scripts/play_wuji_trajectory.py /tmp/wuji_right_retarget_WuzTwX.npz

# 如果动作不对或需要紧急停止 → Ctrl+C

# Step 2: 立即复位
.venv-wujihand/bin/python scripts/reset_wuji_hand.py
```

## 4. 连接到设备（手动诊断）

```bash
.venv-wujihand/bin/python -c "
from wujihandpy import Hand
import numpy as np

hand = Hand(serial_number='365939643134')
print('SN:', hand.get_product_sn())
print('FW:', hand.get_firmware_version())
print('Handedness:', hand.get_handedness())

pos = np.asarray(hand.read_joint_actual_position(), dtype=float)
print('Positions:', pos)

errors = np.asarray(hand.read_joint_error_code())
print('Errors sum:', errors.sum())

temps = np.asarray(hand.read_joint_temperature(), dtype=float)
print('Temp max:', temps.max())
"
```

## 5. 常见问题

### 连接失败 `ERROR_BUSY` / `Failed to init`

有另一个进程正占用手部。执行：

```bash
pkill -f wujihandpy; sleep 1
```

然后重试。

### 错误码非零

检查错误码：

```bash
.venv-wujihand/bin/python -c "
from wujihandpy import Hand
hand = Hand(serial_number='365939643134')
import numpy as np
print(np.asarray(hand.read_joint_error_code()))
"
```

如有错误，尝试：

```bash
.venv-wujihand/bin/python -c "
from wujihandpy import Hand
hand = Hand(serial_number='365939643134')
hand.write_joint_reset_error()
print('error reset done')
"
```

### ROS 驱动也在运行

SDK 和 ROS 不能同时控制。确认无 ROS 驱动：

```bash
ps aux | grep wujihand_driver
```

如有则先停止 ROS launch。

---

# Tianji 真机双臂 joint impedance NPZ 回放

从离线 NPZ 轨迹读取左臂 + 右臂各 7 关节目标，在真机上以 joint impedance 模式**双臂同时**回放。

**Python 环境**：系统 Python 3（非虚拟环境），需要在项目根目录设置 `PYTHONPATH=.`。

**机器人**：IP `192.168.1.190`，A 臂 = 左臂，B 臂 = 右臂。

## 1. 干跑（不连机器人，只看规划）

```bash
PYTHONPATH=. python3 real_robot_debug/a_arm_impedance_two_stage.py
```

输出示例：
```
Loading: recordings/...npz
Total frames:      831
Source DT:         5 ms (200 Hz)
Play DT:           100.0 ms (speed=0.05x)
Source duration:   4.2s
Playback duration: 83.0s
✅ Both arms — joint limits OK

Left  (A):
  Joint 1: [   68.6,    81.3]
  ...
  Max frame step: 0.45 deg

Right (B):
  Joint 1: [  -52.0,   -51.0]
  ...
  Max frame step: 0.21 deg

[Dry-run] Both arms planned. No robot commands sent.
To execute on the real robot, add --execute
```

## 2. 真机执行

```bash
# 首测：0.05x 速度，15 秒缓入（最安全，双臂同时动）
PYTHONPATH=. python3 real_robot_debug/a_arm_impedance_two_stage.py \
    --execute --speed-scale 0.05 --entry-duration-s 15.0

# 0.1x 速度
PYTHONPATH=. python3 real_robot_debug/a_arm_impedance_two_stage.py \
    --execute --speed-scale 0.1 --entry-duration-s 10.0

# 播放完后禁用双臂
PYTHONPATH=. python3 real_robot_debug/a_arm_impedance_two_stage.py \
    --execute --speed-scale 0.1 --no-keep-enabled
```

**脚本行为**：
1. 连接机器人 → 初始化运动学（双臂） → 验证反馈帧（双臂）
2. 读取双臂当前关节位置
3. 配置双臂 joint impedance 模式（相同 K/D 参数）
4. **Phase 1**：双臂各自从当前位置 quintic 平滑缓入到各自首帧
5. **Phase 2**：双臂每帧**同时**下发目标（一次 `send_cmd` 发双臂命令）
6. 默认保持使能；`--no-keep-enabled` 则禁用双臂

**双臂同时通信**：每帧执行一次 `clear_set()` + `set_joint_cmd_pose(arm="A", ...)` + `set_joint_cmd_pose(arm="B", ...)` + `send_cmd()`，确保双臂在同一个控制周期内收到目标。

**Ctrl+C 安全**：
- 任意阶段按 `Ctrl+C` 会立即停止 → disable 双臂 → release robot

## 3. 关键参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--source-npz` | `recordings/...candidate.npz` | 轨迹 NPZ 文件（需同时含 left/right arm）|
| `--execute` | false | 不加就是干跑 |
| `--speed-scale` | 0.05 | 速度倍率 (0.01–1.0) |
| `--entry-duration-s` | 15.0 | Phase 1 缓入时长 |
| `--no-keep-enabled` | false | 播放完后禁用双臂 |
| `--robot-ip` | `192.168.1.190` | 机器人 IP |
| `--kine-config` | `test/ccs_m6_40.MvKDCfg` | 运动学配置 |

## 4. 关节阻抗参数

双臂使用相同参数（可在代码中修改 `DEFAULT_JOINT_K` / `DEFAULT_JOINT_D`）：

| 关节 | K (刚度) | D (阻尼) |
|---|---|---|
| J1 | 8.0 | 0.8 |
| J2 | 8.0 | 0.8 |
| J3 | 8.0 | 0.8 |
| J4 | 4.0 | 0.6 |
| J5 | 2.0 | 0.4 |
| J6 | 1.5 | 0.3 |
| J7 | 1.0 | 0.2 |

## 5. 关节限位（双臂通用）

| 关节 | 下限 (deg) | 上限 (deg) |
|---|---|---|
| J1 | -170 | 170 |
| J2 | -100 | 120 |
| J3 | -170 | 170 |
| J4 | -130 | 130 |
| J5 | -170 | 170 |
| J6 | -90 | 220 |
| J7 | -170 | 170 |

轨迹超出限位 1° 余量会自动拒绝（双臂独立检查）。

---

## 文件索引

| 文件 | 用途 |
|---|---|
| `scripts/play_wuji_trajectory.py` | SDK 直控 Wuji 手轨迹回放 |
| `scripts/reset_wuji_hand.py` | SDK 直控 Wuji 手复位到张开（1s 缓入） |
| `scripts/reset_wuji_hand_fast.py` | **SDK 直控 Wuji 手快速恢复张开（0.8s，Ctrl+C 强制回零兜底）** |
| `scripts/half_fist_wuji_hand.py` | **SDK 直控 Wuji 手半握拳（自动恢复张开版本）** |
| `scripts/half_fist_wuji_hand_hold.py` | **SDK 直控 Wuji 手半握拳（保持使能，需手动 reset 恢复；详见 doc_zt/half_fist_wuji_hand.md）** |
| `scripts/play_mirrored_hand.sh` | 播放镜像右手 200Hz NPZ（旧版 shell 包装）|
| `real_robot_debug/a_arm_impedance_two_stage.py` | **真机双臂 joint impedance NPZ 回放** |
