# Tianji 双臂 + Wuji 灵巧手 真机操作流程手册

更新日期：2026-08-16

本文档汇总 Tianji 双臂和 Wuji 灵巧手的真机调用流程、Python 环境、安全门控和常见问题。

---

## 一、硬件概览

### 1.1 Wuji 灵巧手（左手）

| 属性 | 值 |
|---|---|
| 物理连接 | `/dev/ttyACM0` (USB) |
| USB 序列号 | `365939643134` |
| 产品序列号 | `LQSQJR.260616.005` |
| 手性 | 左手 (handedness=0) |
| 固件 | 1.2.1 |
| SDK 版本 | 1.8.0 |
| 控制方式 | SDK 直控（非 ROS） |
| Python 环境 | `.venv-wujihand/bin/python` |

### 1.2 Tianji 双臂

| 属性 | 值 |
|---|---|
| 机器人 IP | `192.168.1.190` |
| A 臂（左臂） | 对应 SDK arm="A" |
| B 臂（右臂） | 对应 SDK arm="B" |
| SDK 版本 | 100343009 |
| Python 环境 | 系统 Python 3 + `PYTHONPATH=.` |

### 1.3 关节限位

**双臂（度）：**

| 关节 | 下限 | 上限 |
|---|---|---|
| J1 | -170 | 170 |
| J2 | -100 | 120 |
| J3 | -170 | 170 |
| J4 | -130 | 130 |
| J5 | -170 | 170 |
| J6 | -90 | 220 |
| J7 | -170 | 170 |

**Wuji 手（rad）—— 各指不同，详见 doc_zt/wuji_sdk_playback_guide.md**

---

## 二、Python 环境说明

项目有三套 Python 环境，各管各的设备，**不能混用**：

| 用途 | Python 解释器 | 关键包 | 控制对象 |
|---|---|---|---|
| Wuji 手 SDK 直控 | `.venv-wujihand/bin/python` | `wujihandpy` 1.8.0 | Wuji 灵巧手 |
| 仿真 + 数据加载 | `.venv-wuji-teleop/bin/python` | `mujoco 3.10.0`, `numpy`, `mcap` | 仅仿真 |
| Tianji 真机控制 | 系统 `python3` + `PYTHONPATH=.` | `SDK_PYTHON.fx_robot` | Tianji 双臂 |

**同时控制双臂 + 手时，必须用 `.venv-wujihand/bin/python`**（它不包含 Tianji SDK，但系统路径可以访问）。

---

## 三、Wuji 灵巧手操作流程

### 3.1 检查设备在线

```bash
ls -l /dev/ttyACM0
```

输出应为 `crw-rw-rw-`。如果不存在，检查 USB 连接。

### 3.2 读取手部状态（不使能，只读）

```bash
.venv-wujihand/bin/python -c "
from wujihandpy import Hand
import numpy as np
hand = Hand(serial_number='365939643134')
print('SN:', hand.get_product_sn())
print('FW:', hand.get_firmware_version())
print('Handedness:', hand.get_handedness())
print('Errors:', np.asarray(hand.read_joint_error_code()).sum())
print('Temp max:', np.asarray(hand.read_joint_temperature(), dtype=float).max())
print('Positions:', np.asarray(hand.read_joint_actual_position(), dtype=float))
"
```

### 3.3 复位到张开（安全姿势）

```bash
.venv-wujihand/bin/python scripts/reset_wuji_hand.py          # 1s 缓入
.venv-wujihand/bin/python scripts/reset_wuji_hand_fast.py      # 0.8s 快速恢复
```

### 3.4 半握拳测试

```bash
# 保持使能，停在半握拳，观察后手动 reset
.venv-wujihand/bin/python scripts/half_fist_wuji_hand_hold.py --ramp 3

# 观察完恢复张开
.venv-wujihand/bin/python scripts/reset_wuji_hand_fast.py
```

### 3.5 播放 NPZ 手部轨迹

```bash
# 仅手回放（推荐先测这个）
PYTHONPATH=. .venv-wujihand/bin/python real_robot_debug/hand_only_stream.py \
  --source-npz recordings/你的文件.npz --execute
```

### 3.6 SDK 控制流程（代码层面）

```python
from wujihandpy import Hand
import numpy as np

hand = Hand(serial_number="365939643134")

# 1. 检查错误
errors = np.asarray(hand.read_joint_error_code())
if errors.any():  # 先清除错误
    hand.write_joint_reset_error()

# 2. 使能
hand.write_joint_enabled(True)

# 3. 下发目标（5 指 × 4 关节 = 20 个值）
target = np.array([
    [0.45, 0.30, 0.50, 0.45],  # 拇指
    [0.60, 0.13, 0.60, 0.55],  # 食指
    [0.60, 0.12, 0.60, 0.55],  # 中指
    [0.60, 0.11, 0.60, 0.50],  # 无名指
    [0.55, 0.11, 0.55, 0.50],  # 小指
], dtype=float)
hand.write_joint_target_position(target)

# 4. 去使能
hand.write_joint_enabled(False)
```

### 3.7 常见问题

| 问题 | 解决方法 |
|---|---|
| `ERROR_BUSY` / `Failed to init` | 另一个进程占用了 USB：`pkill -f wujihandpy; sleep 1` 再试 |
| 错误码非零 | `hand.write_joint_reset_error()` 清除 |
| ROS 驱动也在运行 | `ps aux \| grep wujihand_driver`，如有则停止 ROS launch |
| 手部温度过高（>75°C） | 暂停使用，等冷却后再操作 |

---

## 四、Tianji 双臂操作流程

### 4.1 检查机器人连接

```bash
ping -c 1 192.168.1.190
```

### 4.2 读取双臂状态（只读诊断）

```bash
PYTHONPATH=. python3 -c "
from SDK_PYTHON.fx_robot import DCSS, Marvin_Robot
dcss = DCSS()
robot = Marvin_Robot()
robot.connect('192.168.1.190')
sub = robot.subscribe(dcss)
for idx, name in [(0, 'A'), (1, 'B')]:
    s = sub['states'][idx]
    fb = sub['outputs'][idx]['fb_joint_pos']
    print(f'{name}: state={s[\"cur_state\"]}, err={s[\"err_code\"]}, pos={[round(v,1) for v in fb]}')
robot.release_robot()
"
```

**状态码含义：**
- 0 = 下伺服（未上电）
- 1 = 位置跟随
- 2 = PVT
- 3 = 扭矩（阻抗）← 播放轨迹需要此状态
- 4 = 协作释放
- 100 = 报错

### 4.3 双臂 NPZ 轨迹回放

```bash
# 干跑
PYTHONPATH=. python3 real_robot_debug/dual_arm_batch_stream.py \
  --source-npz recordings/你的文件.npz

# 真机（0.05x 首测）
PYTHONPATH=. python3 real_robot_debug/dual_arm_batch_stream.py \
  --source-npz recordings/你的文件.npz \
  --execute --entry-duration-s 15 --speed-scale 0.05
```

### 4.4 只读探针（验证反馈，不移动）

```bash
PYTHONPATH=. python3 real_robot_debug/dual_arm_batch_stream.py \
  --source-npz recordings/你的文件.npz \
  --execute --probe-current
```

在当前姿态保持 200 帧（约 1 秒），验证反馈帧是否递增、state=3、误差 ≤5°。

### 4.5 SDK 控制流程（双臂，底层 API）

```python
from SDK_PYTHON.fx_robot import DCSS, Marvin_Robot

robot = Marvin_Robot()
dcss = DCSS()
robot.connect("192.168.1.190")

# 1. 配置 joint impedance
robot.clear_set()
robot.set_state(arm="A", state=3)
robot.set_state(arm="B", state=3)
robot.set_impedance_type(arm="A", type=1)
robot.set_impedance_type(arm="B", type=1)
robot.set_vel_acc(arm="A", velRatio=100, AccRatio=100)
robot.set_vel_acc(arm="B", velRatio=100, AccRatio=100)
robot.send_cmd()
time.sleep(0.2)

# 2. 配置 K/D
robot.clear_set()
robot.set_joint_kd_params(arm="A", K=[8,8,8,4,2,1.5,1], D=[0.8,0.8,0.8,0.6,0.4,0.3,0.2])
robot.set_joint_kd_params(arm="B", K=[8,8,8,4,2,1.5,1], D=[0.8,0.8,0.8,0.6,0.4,0.3,0.2])
robot.send_cmd()
time.sleep(0.2)

# 3. 每帧：暂存 A/B 目标 → 一次发送
robot.clear_set()
robot.set_joint_cmd_pose(arm="A", joints=[...])  # 7 个关节角度，度
robot.set_joint_cmd_pose(arm="B", joints=[...])  # 7 个关节角度，度
robot.send_cmd()

# 4. 清理
robot.release_robot()
```

### 4.6 常见问题

| 问题 | 可能原因 | 解决 |
|---|---|---|
| state=0 | 臂未上电 | 检查控制柜急停、伺服使能 |
| state=100 | 报错 | `robot.check_error_and_clear(dcss)` 清除 |
| 反馈恒为 [-90,-90,90,-90,0,0,0] | 未上电或编码器未就绪 | 检查硬件上电 |
| 帧号不递增 | 控制器未运行 | 确认 control-system 服务正常 |
| 版本不匹配告警 | 控制柜固件与 SDK 版本不一致 | 检查控制柜 control-system 服务 |

---

## 五、双臂 + 手联合操作流程

### 5.1 完整流程（推荐顺序）

```
Step 1: 确认硬件就绪
  ├── Wuji 手：ls /dev/ttyACM0 ✓
  └── Tianji 臂：ping 192.168.1.190 ✓

Step 2: 只读诊断
  ├── Wuji 手状态读取（不使能）
  └── Tianji 双臂状态读取（不运动）

Step 3: 干跑检查轨迹
  └── PYTHONPATH=. .venv-wujihand/bin/python real_robot_debug/dual_arm_hand_batch_stream.py

Step 4: 只读探针（确认双臂反馈正常）
  └── 加 --probe-current

Step 5: 联合回放（0.05x 首测）
  └── 加 --execute --entry-duration-s 15 --speed-scale 0.05

Step 6: 恢复张开
  └── .venv-wujihand/bin/python scripts/reset_wuji_hand_fast.py
```

### 5.2 联合回放命令

```bash
# 干跑
PYTHONPATH=. .venv-wujihand/bin/python real_robot_debug/dual_arm_hand_batch_stream.py \
  --source-npz recordings/你的文件.npz

# 真机（双臂 + 手）
PYTHONPATH=. .venv-wujihand/bin/python real_robot_debug/dual_arm_hand_batch_stream.py \
  --source-npz recordings/你的文件.npz \
  --execute --entry-duration-s 15 --speed-scale 0.05
```

### 5.3 模式选择

| 参数 | 控制对象 | 不控制 |
|---|---|---|
| （不加参数） | 双臂 + 手 | — |
| `--hand-only` | 仅手 | 双臂 |
| `--arm-only` | 仅双臂 | 手 |

### 5.4 验证标准

回放过程中每 200 帧自动检查：
- 双臂反馈帧号是否递增
- 双臂 state=3（impedance 模式）
- 双臂跟踪误差 ≤5°
- 手部关节自动裁剪到限位内

---

## 六、安全门控

### 6.1 操作前必查清单

- [ ] 机器人周围净空，无人或障碍物
- [ ] 急停按钮可触及
- [ ] Wuji 手 USB 已连接（`ls /dev/ttyACM0`）
- [ ] 机器人可 ping 通（`ping -c 1 192.168.1.190`）
- [ ] 双臂状态正常（state=3，err=0）
- [ ] 轨迹已干跑验证通过
- [ ] 已知恢复张开命令（`reset_wuji_hand_fast.py`）

### 6.2 运行中安全

| 情况 | 操作 |
|---|---|
| 动作异常 | 按 Ctrl+C 中断程序 |
| 手指卡碰 | 运行 `reset_wuji_hand_fast.py` 恢复张开 |
| 双臂异常 | 按急停按钮 |
| 程序退出后 | 手自动去使能，双臂自动禁用 |

### 6.3 速度策略

| 阶段 | 推荐 speed-scale | 说明 |
|---|---|---|
| 首次测试 | 0.05 | 最慢速，20 倍慢放 |
| 常规验证 | 0.1 ~ 0.2 | 5~10 倍慢放 |
| 全速 | 1.0 | 仅限充分验证后 |

---

## 七、文件索引

### 7.1 Wuji 手脚本

| 文件 | 用途 |
|---|---|
| `scripts/reset_wuji_hand.py` | SDK 直控复位到张开（1s 缓入） |
| `scripts/reset_wuji_hand_fast.py` | 快速恢复张开（0.8s，Ctrl+C 强制回零） |
| `scripts/half_fist_wuji_hand.py` | 半握拳（自动恢复张开） |
| `scripts/half_fist_wuji_hand_hold.py` | 半握拳（保持使能，手动 reset） |
| `scripts/play_wuji_trajectory.py` | SDK 直控播放 NPZ 轨迹 |
| `real_robot_debug/hand_only_stream.py` | 手部轨迹回放（不碰臂） |

### 7.2 Tianji 臂脚本

| 文件 | 用途 |
|---|---|
| `real_robot_debug/dual_arm_batch_stream.py` | 双臂轨迹回放（底层 API，200 Hz） |
| `real_robot_debug/arm_command_diagnostic.py` | 单臂只读诊断工具 |

### 7.3 联合脚本

| 文件 | 用途 |
|---|---|
| `real_robot_debug/dual_arm_hand_batch_stream.py` | 双臂 + 手联合回放（推荐） |
| `real_robot_debug/dual_arm_hand_playback.py` | 双臂 + 手联合回放（旧版，Concise API） |

### 7.4 文档

| 文件 | 用途 |
|---|---|
| `doc_zt/wuji_sdk_playback_guide.md` | Wuji 手 + Tianji 臂操作手册 |
| `doc_zt/2026-08-05-sdk-control-summary.md` | SDK 控制总结 |
| `doc_zt/2026-08-05-agent-handoff.md` | 智能体交接文档 |
| `doc_zt/2026-08-16-tianji-dual-arm-handoff.md` | 双臂轨迹交接 |
| `doc_zt/hand_only_stream_readme.md` | 手部单独回放 README |
| `doc_zt/dual_arm_hand_batch_stream_readme.md` | 双臂+手联合回放 README |
| `doc_zt/half_fist_wuji_hand.md` | 半握拳脚本说明 |
| `doc_zt/reset_wuji_hand_fast.md` | 快速恢复脚本说明 |
| `docs/simulation/current_version_handoff.md` | 仿真版本交接 |

---

## 八、快速命令速查

```bash
# === 环境检查 ===
ls /dev/ttyACM0                            # Wuji 手在线
ping -c 1 192.168.1.190                    # 机器人可通

# === Wuji 手 ===
.venv-wujihand/bin/python scripts/reset_wuji_hand_fast.py     # 快速恢复张开
.venv-wujihand/bin/python scripts/half_fist_wuji_hand_hold.py  # 半握拳
PYTHONPATH=. .venv-wujihand/bin/python real_robot_debug/hand_only_stream.py \
  --source-npz 文件.npz --execute                               # 手部回放

# === Tianji 双臂 ===
PYTHONPATH=. python3 real_robot_debug/dual_arm_batch_stream.py \
  --source-npz 文件.npz --execute --entry-duration-s 15 --speed-scale 0.05

# === 双臂 + 手联合 ===
PYTHONPATH=. .venv-wujihand/bin/python real_robot_debug/dual_arm_hand_batch_stream.py \
  --source-npz 文件.npz --execute --entry-duration-s 15 --speed-scale 0.05
```