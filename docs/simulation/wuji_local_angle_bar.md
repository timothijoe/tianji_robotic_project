# Wuji 灵巧手本地角度条仿真

这个入口在 MuJoCo 中打开一个本地角度条面板，用鼠标实时控制一只 20 自由度
Wuji 灵巧手。它只改变仿真模型，不连接 ROS、USB 或真实手 SDK。

## 启动

在仓库根目录运行：

```bash
.venv/bin/tianji-robot sim wuji-angle-bar
```

默认载入左手。启动时直接选择右手：

```bash
.venv/bin/tianji-robot sim wuji-angle-bar --hand right
```

面板顶部也可以在左手和右手之间切换。切换会关闭当前 MuJoCo Viewer，载入对应
手模型，并用该模型经过校验的张开姿态重新初始化角度条。

每根滑条对应一个关节，五指各有四个关节。显示值和滑条范围均为弧度（rad）；
范围取自当前模型的 position actuator，输入不会超出模型允许的控制范围。点击
“Reset open hand”可将当前手恢复为安全的张开姿态。

## 运行条件

- 已按[仿真环境设置](setup.md)创建并安装依赖的 `.venv`。
- 需要有效的图形会话（`DISPLAY`/XWayland）和可用的 OpenGL，才能显示 Tk 面板
  和 MuJoCo Viewer。
- 无图形会话时可以继续运行自动化测试，但不能使用交互式角度条窗口。

## 面板功能

### Save Pose（保存姿态）

将当前 20 个滑条角度保存为单帧 NPZ 文件：

- 默认目录：`recordings/wuji/`（可通过保存对话框选择其他位置）
- 文件名：`wuji_pose_{side}_YYYYMMDD_HHMMSS.npz`
- 内容：
  - `joint_positions_rad`: shape `(1, 20)` 弧度
  - `timestamps_ns`: shape `(1,)`
  - `side`: `"left"` / `"right"`
  - `joint_names`: 20 个关节名称

### Record / Stop（录制动作序列）

录制一段连续动作：

- 点击 **Record** 开始录制（按钮变为红色 ■ Stop）
- 每个 Tk tick 记录当前 20 个关节角度
- 点击 **Stop** 停止并弹出保存对话框
- 文件名：`wuji_trajectory_{side}_YYYYMMDD_HHMMSS.npz`
- 内容：
  - `joint_positions_rad`: shape `(N, 20)`
  - `timestamps_ns`: shape `(N,)` 严格递增
  - `side`, `joint_names`, `frame_count`

### Send to Hand（发送到真机）

将当前滑条姿态发送到真实 Wuji 手：

- 点击后弹出确认对话框
- 确认后将当前姿态写入临时 NPZ，通过 subprocess 调用
  `scripts/send_pose_to_wuji_hand.py`
- 脚本运行在 `.venv-wujihand` 环境中（内置 `wujihandpy` SDK）
- 执行流程：使能 → 缓入 → 保持 → 自动去使能
- 面板状态栏显示执行结果

## 关节映射

仿真 MuJoCo 模型的 20 个 actuator 与真实 Wuji 手 SDK 的 20 个关节**直接
一一对应**，无需坐标变换：

| 仿真 flat 索引 | 仿真 actuator | 手指 | 真机 SDK |
|--------------|--------------|------|---------|
| 0–3 | `finger1_joint1..4` | 拇指 | `finger[0].joint[0..3]` |
| 4–7 | `finger2_joint1..4` | 食指 | `finger[1].joint[0..3]` |
| 8–11 | `finger3_joint1..4` | 中指 | `finger[2].joint[0..3]` |
| 12–15 | `finger4_joint1..4` | 无名指 | `finger[3].joint[0..3]` |
| 16–19 | `finger5_joint1..4` | 小指 | `finger[4].joint[0..3]` |

## 真机回放命令

```bash
# 发送单帧姿态到真机
.venv-wujihand/bin/python scripts/send_pose_to_wuji_hand.py \
    recordings/wuji/wuji_pose_left_YYYYMMDD_HHMMSS.npz

# 回放多帧轨迹到真机（默认 0.2 倍速）
.venv-wujihand/bin/python scripts/play_wuji_trajectory.py \
    recordings/wuji/wuji_trajectory_left_YYYYMMDD_HHMMSS.npz

# 复位真机到张开姿态
.venv-wujihand/bin/python scripts/reset_wuji_hand.py
```

## 边界与安全

`local_angle_bar.py` 本身是 MuJoCo-only 的离线仿真模块，不导入 `wujihandpy`。
真机控制通过独立子进程 `scripts/send_pose_to_wuji_hand.py` 完成，该脚本：

- 检测关节错误码，非零时拒绝执行
- 目标值裁剪到硬件限位（留 0.02 rad 余量）
- 使能后缓入（默认 2 s）→ 保持（默认 3 s）→ 自动去使能
- 支持 Ctrl+C 中断并立即去使能

## 生命周期与步进

MuJoCo 后端的每次 `step()` 只推进一个模型 timestep，不自行等待；Tk 面板按该
timestep 重新调度下一次步进，因此它是唯一的实时节拍拥有者。关闭 MuJoCo Viewer
或关闭面板都会执行同一套幂等清理：关闭后端并退出、销毁 Tk 主循环。可运行下列
无窗口回归测试验证左右手切换、窗口关闭和步进调度契约：

```bash
python3 -m pytest tests/simulation/test_local_angle_bar.py -q
```

（如环境较旧缺少 pytest 插件，可追加 `-p no:launch_testing`）