# Sim-Style 手柄笛卡尔 Jog 真机实现（2026-08-23）

## 目标与动机

实物遥操作此前使用 `kine.movLA()` 逐段规划直线轨迹：每次只能规划一段，且必须等
`setPln_Cart` 发出的整段轨迹执行完（`_wait_until_traj_idle`）才能做下一件事。当需要
编排多个动作（如切菜中刀走一段、手走一段、刀手同时走）时非常不灵活。

本次目标是：**按照仿真（MuJoCo `twin_sim/gamepad_teleop.py`）的思路实现一版实物遥
操作**，用 SDK 自带的 `Marvin_Kine.fk()/ik()` 替代 `movLA`，把控制循环、运动幅度
参数都与仿真对齐，为后续多动作编排打基础。

## 当前结论

- 新增 `real_robot_debug/gamepad_cartesian_jog_simstyle.py`：
  50 Hz FK/IK 连续速度控制，参数与仿真一致（100 mm/s、±350 mm、0.15 死区）。
- 新增 `tests/hardware/test_gamepad_cartesian_jog_simstyle.py`：35 项测试全部通过。
- 文档同步更新至 `real_robot_debug/README.md:195`。

## 设计思路（对照）

| 对比项 | 旧版 `gamepad_cartesian_jog.py` | 新版 `_simstyle.py` |
|---|---|---|
| 控制方式 | `movLA` 规划 → `setPln_Cart` 整段发送 → 等轨迹空闲 | 每周期：手柄速度 → 积分目标 TCP → SDK IK → `set_joint_cmd_pose` 直发 |
| 控制频率 | 5 Hz（0.2 s） | 50 Hz（0.02 s）← 匹配仿真 |
| 默认速度 | 10 mm/s | 100 mm/s ← 匹配仿真 |
| 默认工作空间 | ±50 mm | ±350 mm ← 匹配仿真 |
| 速度上限 | 10 mm/s | 300 mm/s（`--max-speed-mm-s` 可配） |
| 运动学 | SDK MOVLA 内部规划 | SDK FK/IK（与仿真架构一致） |
| 动作编排 | 每段轨迹必须等完才能发下一段 | 可任意串联动作，不等待 |

### 控制循环（仿真思路）

```text
每 20 ms（50 Hz）:
  ├── 轮询手柄事件
  ├── Start(7) → 退出；RB(5) 未按住 → 保持，不发命令
  ├── 手柄轴 → 笛卡尔速度 v (mm/s)（左杆 X/Y，右杆垂直 Z）
  ├── target_xyz = current_xyz + v * dt（积分，保持当前姿态 ABC）
  ├── 按工作空间边界 clip
  ├── SDK IK：target_xyzabc → 目标关节角（参考上一帧关节角，光滑连续）
  ├── 检查 IK 结果有效性与关节超限标志
  ├── robot.set_joint_cmd_pose() 发送目标关节角（度）
  └── 更新当前位姿
```

### 关键参数收敛（与仿真相同）

| 参数 | 仿真 `gamepad_teleop.py` | 新版实物 |
|---|---|---|
| 控制周期 | 0.02 s | 0.02 s |
| 默认速度 | 100 mm/s | 100 mm/s |
| 速度上限 | 300 mm/s | 300 mm/s |
| 工作空间 | ±350 mm | ±350 mm |
| 死区 | 0.15 | 0.15 |

## 关键实现细节

- **使用 `Marvin_Kine` 的 FK/IK，不导入仿真 IK**：
  - FK：`kine.fk(joints_deg)` → 4×4 位姿；`kine.mat4x4_to_xyzabc()` → XYZABC（mm/度）
  - IK：`FX_InvKineSolvePara` 结构体 → `kine.ik(params)` →
    `params.get_output_ret_joint()`（度）
  - 参考关节角用**上一帧已发送的关节角**，与仿真 `ik(pose, joints)` 用上一帧
    关节角作 reference 的模式一致，保证解平滑不跳变。
- **控制状态**：`_configure_position_mode()` 把目标臂设到 `state=1`（位置跟随），
  而非旧版的规划模式。关闭时 `state=0` 下使能。
- **干跑默认**：不带 `--execute` 只打印请求，不发送任何命令、不下使能。
- **安全保留**：RB 死曼、Start 退出、`_verify_frame_updates` 反馈帧校验、
  数值有限性校验、速度上限拒绝。

## 真机 FK/IK 离线验证（不连机器人）

用真实 SDK + `test/ccs_m6_40.MvKDCfg`（TYPE=1017 CCS 机型）验证：

- 左臂（A）参考位姿 `[17.970, -35.197, 11.414, -73.344, -9.154, -17.035, 7.086]`：
  FK → XYZABC → IK 回解，关节误差全零，4 组解，`is_out_range=False`。
- 右臂（B）起点 `[-90, -90, 90, -90, 0, 0, 0]`（占位值）：FK XYZABC =
  `[427.0, -305.0, 174.5, 90, 0, 90]`；向 `+X` 走 10 mm 的 IK 成功，Re-FK
  位置误差 ~2e-5 mm。
- 较大位移（+100/+50/+30 mm）IK 成功，Re-FK 最大位置误差 ~8e-6 mm。

## 关键诊断：SDK 解析 IK 的局限性

- **`ik()` 要求参考关节第 4 关节（J4）不能为零**（SDK 文档原文「参考角第四关节不能
  为零」）。已加防护：如果 ref[3] 为 0，微调到 0.001。
- 在右臂从 `j4=-13.379°` 继续沿 `+X` 走时，IK 开始持续失败；进一步验证**从更远
  起点（如 startup 位姿）重新 IK 到同一目标位置也失败**，说明 SDK 解析 IK 在部分
  工作空间区域（靠近某些奇异构型）无解，即使目标点物理可达。
- 对比：仿真自实现的 DLS 数值 IK（`twin_sim/kinematics.py`，阻尼最小二乘）在同样
  区域内可以继续求解。**结论：SDK 解析 IK 的求解稳定性弱于数值 IK**。

### 对真机使用的影响

- 手柄走到 IK 失败的姿态时，程序**保持上一有效位姿**（不发新命令），不会崩溃。
- 用户需要手动退回已知稳定区域。这是 SDK IK 的固有约束，不是代码 bug。
- 已知脆弱的起点示例（右臂沿 `+X`）：`[-6.799, -66.376, 2.975, -13.379, 10.122, -4.737, 12.069]` 之后继续前移即开始失败。

## 文件清单

| 路径 | 职责 |
|---|---|
| `real_robot_debug/gamepad_cartesian_jog_simstyle.py` | 仿真风格 50 Hz FK/IK 手柄 Jog（新建） |
| `tests/hardware/test_gamepad_cartesian_jog_simstyle.py` | 35 项单元测试（新建，全部通过） |
| `real_robot_debug/README.md` | 新增 Sim-Style 章节（`:195`） |
| `real_robot_debug/keyboard_cartesian_jog.py` | 复用 `_verify_frame_updates`、`workspace_bounds_from_center`、`inside_workspace` |

## 使用方式

```bash
# 干跑（默认，不连真机运动）
PYTHONPATH=. python3 real_robot_debug/gamepad_cartesian_jog_simstyle.py

# 真机首测（低速度、小工作空间）
PYTHONPATH=. python3 real_robot_debug/gamepad_cartesian_jog_simstyle.py \
  --arm right --execute \
  --speed-mm-s 20 \
  --workspace-around-current-mm 50

# 完全仿真参数
PYTHONPATH=. python3 real_robot_debug/gamepad_cartesian_jog_simstyle.py \
  --arm right --execute \
  --speed-mm-s 100 \
  --workspace-around-current-mm 350
```

安全提示：物理 E-stop 必须可触及；RB 只是软件死曼；工作空间是启动 TCP 为中心的
固定盒，越界请求被 clip；IK 失败时保持位姿。

## 验证

```bash
PYTHONPATH=. python3 -m pytest tests/hardware/test_gamepad_cartesian_jog_simstyle.py -q
# 结果：35 passed
```

## 尚未完成 / 下一步建议

1. **真机首测**：先以 `--speed-mm-s 20` 验证位置模式下关节跟踪效果与反馈
   帧稳定性；确认 SDK FK 与真机反馈 TCP 一致。
2. **IK 失败兜底**：若 SDK 解析 IK 稳定性不足，可做混合方案 —— IK 成功走连续
   模式，失败时回退到一段 `movLA` 直线兜底。
3. **双臂手柄**：把该模式扩展为双臂同时控制，参考
   `dual_arm_batch_stream.py` 的 `clear_set()` + A/B 暂存 + 单次 `send_cmd()`。
4. **多动作编排**：基于本脚本（不依赖 MOVLA 单段规划）编排刀/手串联动作序列。
5. **更接近仿真**：后续可在仿真中引入阻抗行为（按真机 K/D 参数），使两端的
   动力学响应更接近。

## 相关文档

- `doc_zt/2026-08-16-tianji-dual-arm-handoff.md` —— 双臂轨迹回放交接
- `doc_zt/tianji_wuji_operation_guide.md` —— 真机操作手册（状态码、环境、安全门控）
- `docs/simulation/xbox_gamepad_teleop.md` —— MuJoCo 手柄遥操作说明（对照参考）
- `docs/simulation/current_version_handoff.md` —— 仿真版本交接