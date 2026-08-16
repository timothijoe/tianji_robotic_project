# 智能体交接文档

更新日期：2026-08-05 22:54
上一会话 Agent：GLM-5.2
交接人：zhoutong

---

## 0. 快速上手

```bash
# 工作目录
cd /home/zhoutong/august_folder/tianji_robotic_project

# Wuji 手复位到张开（首次连接可用此确认手在线）
.venv-wujihand/bin/python scripts/reset_wuji_hand.py

# Wuji 手播放 NPZ 轨迹
.venv-wujihand/bin/python scripts/play_wuji_trajectory.py /tmp/wuji_right_retarget_WuzTwX.npz --speed 0.2

# 真机双臂回放（0.05x 首测）
PYTHONPATH=. python3 real_robot_debug/a_arm_impedance_two_stage.py --execute --speed-scale 0.05

# 双臂 + 手联合回放（干跑）
PYTHONPATH=. python3 real_robot_debug/dual_arm_hand_playback.py

# 全量回归测试
.venv-wuji-teleop/bin/pytest tests/simulation -q
```

---

## 1. 当前工作目录

```
/home/zhoutong/august_folder/
├── tianji_robotic_project/      # 主项目仓库 (Git)
│   ├── src/                     # 仿真源码
│   ├── real_robot_debug/        # 真机控制脚本 ← 本日主要工作
│   ├── scripts/                 # 常用 shell/python 入口
│   ├── recordings/              # NPZ/MCAP 录制文件 (Git 忽略)
│   ├── doc_zt/                  # 交接/操作文档
│   └── .venv-wujihand/          # Wuji SDK Python 环境
│
└── wuji-technology/             # Wuji 官方源码
    ├── doc_zt/                  # 旧交接文档（重要背景知识）
    ├── wuji-glove-recorder/     # 手套录制/转换工具
    └── wujihandros2/            # ROS 2 驱动
```

---

## 2. 硬件状态

### 2.1 Wuji 左手

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
| **控制状态** | ✅ 已验证可用，五指批量写入 |

### 2.2 Tianji 双臂

| 组件 | 状态 | 详情 |
|---|---|---|
| 机器人 IP | `192.168.1.190` | 已 ping 通 |
| A 臂（左臂） | ✅ **可用** | state=3 (impedance)，反馈正常 |
| B 臂（右臂） | ❌ **待排查** | 反馈恒为 `[-90, -90, 90, -90, 0, 0, 0]`，疑似未上电 |
| SDK 版本 | 100343009 | |
| Python 环境 | 系统 Python 3 + `PYTHONPATH=.` | |

---

## 3. 本日完成的工作

### 3.1 Wuji 手 SDK 控制

- ✅ SDK 连接/断开、读取状态、使能/去使能
- ✅ **批量写入**：`hand.write_joint_target_position((5,4) array)` 五指同时运动
- ✅ 单指微动测试 + 正弦摆动测试
- ✅ Ctrl+C 安全退出 + finally 去使能

### 3.2 创建的脚本

| 文件 | 用途 | 状态 |
|---|---|---|
| `scripts/reset_wuji_hand.py` | Wuji 手复位到张开 | ✅ 可用 |
| `scripts/play_wuji_trajectory.py` | Wuji 手播放 NPZ 轨迹 | ✅ 可用 |
| `real_robot_debug/a_arm_impedance_two_stage.py` | 真机双臂 joint impedance 回放 | ⚠️ 仅 A 臂可用 |
| `real_robot_debug/dual_arm_hand_playback.py` | **双臂 + 手联合回放** | ⚠️ 仅 A 臂+手可用 |
| `doc_zt/wuji_sdk_playback_guide.md` | 操作手册 | ✅ |
| `doc_zt/2026-08-05-sdk-control-summary.md` | 本日工作总结 | ✅ |

### 3.3 关键发现

- **SDK 简明版** (`Concise_Marvin_Robot`) 比原版好用：`set_imp_joint_state` 一步配置 impedance，`set_joint_position_cmd` 直接下发，不需要 `clear_set()/send_cmd()` 包裹
- **Wuji 手批量写入**：`hand.write_joint_target_position(5×4 array)` 而非逐指调用
- **B 臂问题**：state=0（未上电），`set_position_state` 和 `set_imp_joint_state` 后反馈仍为占位值

---

## 4. 已知问题与待办

### 4.1 高优先级

| 问题 | 详情 | 建议 |
|---|---|---|
| **B 臂（右臂）不动** | 反馈恒为 `[-90, -90, 90, -90, 0, 0, 0]`，state=0 | 检查右臂是否上电、伺服使能、急停是否释放 |
| **B 臂 state=100** | 尝试 `set_imp_joint_state` 时报错 | 请先确认 B 臂硬件正常再试 |

### 4.2 中优先级

- `dual_arm_hand_playback.py` 的 B 臂部分需要 B 臂就绪后才能验证
- 手部温度偏高（49–69°C），但 SDK 读数可能不准，以人工触摸为准

### 4.3 低优先级

- 考虑 PVT 离线轨迹作为 impedance 的备选方案
- 将 Wuji 手复位逻辑集成到联合回放脚本的清理阶段

---

## 5. 重要文档索引

### 5.1 本仓库

| 文档 | 位置 | 说明 |
|---|---|---|
| 本日工作总结 | `doc_zt/2026-08-05-sdk-control-summary.md` | 当前任务的完整记录 |
| 操作手册 | `doc_zt/wuji_sdk_playback_guide.md` | Wuji 手 + Tianji 臂命令速查 |
| 仿真交接 | `docs/simulation/current_version_handoff.md` | 旧仿真版本交接 |
| 跨机器迁移 | `docs/deployment/cross_machine_migration.md` | 环境安装与迁移 |

### 5.2 wuji-technology/doc_zt/（重要背景知识）

| 文档 | 说明 |
|---|---|
| `01_机械手连接与控制.md` | 实体手连接、ROS 驱动、SDK |
| `02_Wuji_Studio与手套教程.md` | 手套录制、Studio 使用 |
| `03_右手到左手与MuJoCo.md` | 右手骨架→左手关节的 Y 轴镜像 |
| `04_MCAP实体左手回放.md` | 实体手回放安全流程 |
| `05_新Agent交接与复现清单.md` | 旧交接文档 |
| `问题与决策日志.md` | 排查历史 |

### 5.3 SDK 文档

| 文档 | 位置 |
|---|---|
| Python 控制 SDK | `~/august_folder/TJ_FX_ROBOT_CONTRL_SDK/python_doc_contrl.md` |
| Python 运动学 SDK | `~/august_folder/TJ_FX_ROBOT_CONTRL_SDK/python_doc_kine.md` |

---

## 6. Python 环境说明

| 用途 | 环境 | 关键包 |
|---|---|---|
| Wuji 手 SDK 直控 | `.venv-wujihand/bin/python` | `wujihandpy` 1.8.0 |
| 仿真 + 数据加载 | `.venv-wuji-teleop/bin/python` | `mujoco 3.10.0`, `numpy`, `mcap` |
| Tianji 真机控制 | 系统 Python 3 + `PYTHONPATH=.` | `SDK_PYTHON.fx_robot` |

---

## 7. 快速命令索引

```bash
# === Wuji 手 ===
# 复位到张开
.venv-wujihand/bin/python scripts/reset_wuji_hand.py

# 播放 NPZ 轨迹（0.2x 默认）
.venv-wujihand/bin/python scripts/play_wuji_trajectory.py /tmp/wuji_right_retarget_WuzTwX.npz

# 播放 NPZ 轨迹（0.1x 慢速）
.venv-wujihand/bin/python scripts/play_wuji_trajectory.py /tmp/wuji_right_retarget_WuzTwX.npz --speed 0.1

# === Tianji 双臂（仅 A 臂可用） ===
# 干跑
PYTHONPATH=. python3 real_robot_debug/a_arm_impedance_two_stage.py

# 真机执行
PYTHONPATH=. python3 real_robot_debug/a_arm_impedance_two_stage.py --execute --speed-scale 0.05

# === 双臂 + 手联合 ===
# 干跑
PYTHONPATH=. python3 real_robot_debug/dual_arm_hand_playback.py

# 真机执行（B 臂就绪后）
PYTHONPATH=. python3 real_robot_debug/dual_arm_hand_playback.py --execute

# 仅手
PYTHONPATH=. python3 real_robot_debug/dual_arm_hand_playback.py --execute --hand-only

# 仅臂
PYTHONPATH=. python3 real_robot_debug/dual_arm_hand_playback.py --execute --arm-only

# === 手动诊断 ===
# Wuji 手状态
.venv-wujihand/bin/python -c "
from wujihandpy import Hand
hand = Hand(serial_number='365939643134')
import numpy as np
print('SN:', hand.get_product_sn())
print('FW:', hand.get_firmware_version())
print('Handedness:', hand.get_handedness())
print('Errors:', np.asarray(hand.read_joint_error_code()).sum())
print('Temp max:', np.asarray(hand.read_joint_temperature(), dtype=float).max())
"

# 机器人状态
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

---

## 8. 新 Agent 接手清单

- [ ] 阅读本交接文档
- [ ] 阅读 `doc_zt/2026-08-05-sdk-control-summary.md`（本日工作详情）
- [ ] 阅读 `doc_zt/wuji_sdk_playback_guide.md`（操作手册）
- [ ] 运行 `git status --short` 确认当前分支和修改
- [ ] 确认 Wuji 手在线：`ls /dev/ttyACM0`
- [ ] 确认机器人可 ping：`ping -c 1 192.168.1.190`
- [ ] **排查 B 臂**：检查右臂上电/伺服/急停
- [ ] B 臂就绪后验证 `dual_arm_hand_playback.py --execute`