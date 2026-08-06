# 2026-08-05 工作日志：Tianji 双臂 + Wuji 手 SDK 真机控制

更新日期：2026-08-05

本日记录今天完成的 Wuji 左手 SDK 直控、Tianji 双臂真机联合阻抗回放、以及相关脚本和文档的完整开发过程。

---

## 一、Wuji 左手 SDK 直控

### 1.1 硬件连接确认

| 属性 | 值 |
|---|---|
| 物理连接 | `/dev/ttyACM0` (USB)，`crw-rw-rw-` |
| USB 序列号 | `365939643134` |
| 产品序列号 | `LQSQJR.260616.005` |
| 手性 | 左手 (handedness=0) |
| 固件 | `1.2.1` |
| SDK 版本 | `1.8.0` |
| Python 环境 | `.venv-wujihand/bin/python` |

### 1.2 控制能力验证

- ✅ SDK 连接/断开
- ✅ 读取关节位置、错误码（全零）、温度、电压（12.32V）、关节限位
- ✅ 使能/去使能
- ✅ **批量写入**：`hand.write_joint_target_position((5,4) array)` 一次写入 20 个关节，五指同时运动
- ✅ 单关节微动测试：食指 MCP 往返 ±0.03 rad，跟踪误差 < 2.3 mrad
- ✅ 正弦摆动测试：食指 MCP，±0.08 rad / 1 Hz / 10 秒，归位误差 1.2 mrad
- ✅ Ctrl+C 安全退出 + finally 去使能

### 1.3 创建的脚本

| 文件 | 用途 |
|---|---|
| `scripts/reset_wuji_hand.py` | SDK 直控复位到张开手（全零位，1 秒缓入） |
| `scripts/play_wuji_trajectory.py` | SDK 直控播放重定向 NPZ 轨迹（支持 --speed、--ramp、--verbose、Ctrl+C 安全） |

### 1.4 发现的 SDK API 关键信息

- **逐指写入**（串行，手指分时动）：`hand.finger(f).joint(j).write_joint_target_position(val)`
- **批量写入**（并行，五指同时动）：`hand.write_joint_target_position(5×4 array)`
- 批量写入消除了分时复用控制的问题
- `ERROR_BUSY` 错误：上一个进程未正常释放 USB claim，等待几秒后重试即可

### 1.5 各关节硬件限位 (rad)

| 手指 | Joint 1 (MCP) | Joint 2 (PIP) | Joint 3 (DIP) | Joint 4 |
|---|---|---|---|---|
| 拇指 F1 | [-0.119, 1.679] | [-0.257, 0.950] | [-0.550, 1.663] | [-0.582, 1.641] |
| 食指 F2 | [-0.262, 1.646] | [-0.439, 0.342] | [-0.529, 1.672] | [-0.505, 1.640] |
| 中指 F3 | [-0.234, 1.644] | [-0.448, 0.330] | [-0.569, 1.644] | [-0.572, 1.662] |
| 无名指 F4 | [-0.242, 1.644] | [-0.478, 0.275] | [-0.576, 1.679] | [-0.543, 1.684] |
| 小指 F5 | [-0.258, 1.632] | [-0.471, 0.292] | [-0.688, 1.536] | [-0.572, 1.646] |

---

## 二、Tianji 双臂真机关节阻抗回放

### 2.1 硬件连接

| 属性 | 值 |
|---|---|
| 机器人 IP | `192.168.1.190` |
| A 臂（左臂）| 正常，state=3（impedance），反馈正常 |
| B 臂（右臂）| ⚠️ 反馈始终为 `[-90, -90, 90, -90, 0, 0, 0]`，疑似未上电或编码器未就绪 |
| SDK 版本 | `100343009` |
| Python 环境 | 系统 Python 3 + `PYTHONPATH=.` |

### 2.2 NPZ 数据源

文件：`recordings/recorded_hand_guarded_chop_200hz_official_right_retarget_candidate.npz`

| 字段 | 形状 | 说明 |
|---|---|---|
| `time_s` | (831,) | 5ms 均匀采样，共 4.15s |
| `left_arm_target_rad` | (831, 7) | A 臂（左臂）目标 |
| `right_arm_target_rad` | (831, 7) | B 臂（右臂）目标 |
| `right_hand_target_rad` | (831, 20) | 右手 20 关节目标 |

轨迹各关节范围验证：全部在 [-170,170] 限位 1° 余量内通过。

### 2.3 开发历程与遇到的问题

#### 阶段 1：单臂 → 双臂
- 初始脚本 `a_arm_impedance_two_stage.py` 只支持 A 臂
- 扩展为双臂同时回放：加载左右臂轨迹 → 各自缓入 → 每帧同时下发双臂命令

#### 阶段 2：API 名称错误
- 报错：`'Marvin_Robot' object has no attribute 'set_joint_position_cmd'`
- 原因：SDK 原版 API 是 `set_joint_cmd_pose(arm="A", joints=...)`，不是 `set_joint_position_cmd`
- 修复：Phase 1 和 Phase 2 统一使用 `set_joint_cmd_pose`

#### 阶段 3：B 臂反馈异常
- B 臂反馈恒为 `[-90, -90, 90, -90, 0, 0, 0]` ——这是 state=0 下的占位值
- A 臂正常（state=3，反馈有真实编码器值）
- 诊断：B 臂可能未上电或未使能

#### 阶段 4：切换到 Concise_Marvin_Robot
- 阅读 SDK 文档 `python_doc_contrl.md` 发现简明式 SDK
- 简明式 API：`set_imp_joint_state`（一步配置 impedance）、`set_joint_position_cmd`（直接下发）、`set_position_state`（position 模式）
- 不需要 `clear_set()/send_cmd()` 包裹

#### 阶段 5：B 臂 state=100 错误
- 直接用 `set_imp_joint_state` 让 B 臂从 state=0 切到 state=3，B 臂进入 state=100（报错）
- `Concise_Marvin_Robot.connect()` 遇到 B 臂报错直接拒绝连接
- 改用 `Marvin_Robot` 先连接清错，再切 `Concise_Marvin_Robot`
- 清错后 B 臂 state=0，但 `set_imp_joint_state` 后再次 state=100

#### 阶段 6：两步切换策略
- Step 1：`set_position_state` 让 B 臂进入 state=1（position 模式），读真实反馈
- Step 2：`set_imp_joint_state` 切到 state=3（impedance）
- 结果：B 臂 position 模式下反馈仍为占位值 → **B 臂可能未上电**

### 2.4 当前结论

| 组件 | 状态 | 备注 |
|---|---|---|
| Wuji 左手 SDK 控制 | ✅ 完全可用 | 五指同时运动，批量写入 |
| Tianji A 臂（左）真机 | ✅ 可用 | state=3 impedance 正常 |
| Tianji B 臂（右）真机 | ❌ 待排查 | 反馈异常，疑似未上电或硬件禁用 |
| 双臂联合 NPZ 回放 | ⚠️ 代码就绪 | 等待 B 臂硬件就绪即可运行 |

---

## 三、SDK API 速查

### 3.1 Wuji SDK (wujihandpy)

```python
from wujihandpy import Hand

hand = Hand(serial_number="365939643134")

# 读取
hand.get_product_sn()               # 产品序列号
hand.get_firmware_version()          # 固件版本
hand.get_handedness()                # 手性 (0=Left, 1=Right)
hand.read_joint_actual_position()    # 当前 20 关节位置 (5×4)
hand.read_joint_error_code()         # 错误码
hand.read_joint_temperature()        # 温度
hand.read_joint_lower_limit() / hand.read_joint_upper_limit()

# 控制
hand.write_joint_enabled(True/False) # 使能/去使能
hand.write_joint_target_position((5,4) array)  # 批量写入全部关节
```

### 3.2 Tianji 控制 SDK

**原版 SDK (`Marvin_Robot`)**：
```python
robot.clear_set()
robot.set_state(arm="A", state=3)           # 0=下伺服, 1=位置, 2=PVT, 3=扭矩, 4=协作
robot.set_impedance_type(arm="A", type=1)    # 1=关节阻抗, 2=坐标阻抗
robot.set_joint_kd_params(arm="A", K=[...], D=[...])
robot.set_joint_cmd_pose(arm="A", joints=[...])  # 关节位置指令（度）
robot.send_cmd()
```

**简明版 SDK (`Concise_Marvin_Robot`)**：
```python
# 一键配置
robot.set_position_state(arm="A", velRatio=50, AccRatio=50)
robot.set_imp_joint_state(arm="A", velRatio=100, AccRatio=100, K=[...], D=[...])

# 下发目标（位置/扭矩模式通用）
robot.set_joint_position_cmd(arm="A", joint=[...])  # 七个关节角度（度）

# 去使能
robot.disable(arm="A")
```

**状态码含义**：
- 0 = 下伺服
- 1 = 位置跟随
- 2 = PVT
- 3 = 扭矩（阻抗）
- 4 = 协作释放
- 100 = 报错

---

## 四、创建/修改的文件清单

| 文件 | 操作 | 说明 |
|---|---|---|
| `scripts/reset_wuji_hand.py` | 新建 | Wuji 手 SDK 复位到张开手 |
| `scripts/play_wuji_trajectory.py` | 新建 | Wuji 手 SDK 播放 NPZ 轨迹 |
| `scripts/play_mirrored_hand.sh` | 新建 | 播放镜像右手 200Hz NPZ（旧版） |
| `real_robot_debug/a_arm_impedance_two_stage.py` | 重写 | 真机双臂 NPZ 回放（dry-run/execute） |
| `real_robot_debug/dual_arm_hand_playback.py` | 新建 | **双臂 + 手联合回放**（同一时间轴同步） |
| `doc_zt/wuji_sdk_playback_guide.md` | 新建 | Wuji 手 + Tianji 臂操作手册 |
| `doc_zt/2026-08-05-sdk-control-summary.md` | 新建 | 本文件 — 今日工作总 |

---

## 五、下一步建议

1. **排查 B 臂**：确认右臂是否上电、伺服是否使能、急停是否释放
2. **B 臂确认后**：运行双臂回放验证
3. **PVT 方案**：如果 impedance 模式对 B 臂不稳定，可考虑用 SDK 的 `send_pvt` + `run_pvt` 离线轨迹模式
4. ~~Wuji 手联动~~ ✅ **已完成**：`real_robot_debug/dual_arm_hand_playback.py` 将双臂 + 手整合到同一脚本

---

## 六、快速命令索引

```bash
# === Wuji 手 ===
# 播放轨迹
.venv-wujihand/bin/python scripts/play_wuji_trajectory.py /tmp/wuji_right_retarget_WuzTwX.npz --speed 0.2

# 复位到张开
.venv-wujihand/bin/python scripts/reset_wuji_hand.py

# === Tianji 双臂 ===
# 干跑（不连机器人）
PYTHONPATH=. python3 real_robot_debug/a_arm_impedance_two_stage.py

# 真机执行（0.05x 首测）
PYTHONPATH=. python3 real_robot_debug/a_arm_impedance_two_stage.py \
    --execute --speed-scale 0.05 --entry-duration-s 15.0

# 播放完禁用双臂
PYTHONPATH=. python3 real_robot_debug/a_arm_impedance_two_stage.py \
    --execute --speed-scale 0.1 --no-keep-enabled
```

---

## 七、联合回放（双臂 + 手同步）

新增 `real_robot_debug/dual_arm_hand_playback.py`，将 Tianji 双臂（A 臂 + B 臂）和 Wuji 手整合到同一个时间轴同步回放。

### 设计重点

- **同一时间轴**：每个控制周期同时下发双臂 + 手部命令，共享 `time.sleep(play_dt)` wall-clock 同步
- **安全检查**：双臂限位 1° 余量 + 手部限位 0.02 rad 余量（手部超出自动裁剪）
- **Phase 1**：双臂 + 手各自从当前位置 quintic 缓入到 NPZ 首帧
- **Phase 2**：双臂 + 手同步逐帧回放
- **Ctrl+C 安全**：任意阶段中断 → 手去使能 + 臂禁用/释放

### 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--execute` | false | 不加就是干跑 |
| `--speed-scale` | 0.05 | 速度倍率 |
| `--entry-duration-s` | 15.0 | 缓入时长 |
| `--hand-only` | false | 只控制手，不碰臂 |
| `--arm-only` | false | 只控制臂，不碰手 |
| `--no-keep-enabled` | false | 播放完后禁用双臂 |

### 命令

```bash
# 干跑（只看规划，不连任何设备）
PYTHONPATH=. python3 real_robot_debug/dual_arm_hand_playback.py

# 全开（双臂 + 手）
PYTHONPATH=. python3 real_robot_debug/dual_arm_hand_playback.py --execute

# 仅手
PYTHONPATH=. python3 real_robot_debug/dual_arm_hand_playback.py --execute --hand-only

# 仅臂
PYTHONPATH=. python3 real_robot_debug/dual_arm_hand_playback.py --execute --arm-only
```
