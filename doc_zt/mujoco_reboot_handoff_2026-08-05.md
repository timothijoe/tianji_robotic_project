# Tianji MuJoCo 重启后交接记录

记录时间：2026-08-05（Asia/Shanghai）

## 当前目标

恢复 MuJoCo Viewer 图形窗口，并演示 Tianji + Wuji 的录制手势联动切菜。
ROS 2 仍不在本轮范围内，禁止连接或命令任何真机。

## 已完成的离线复现

- 工作目录：`/home/zhoutong/august_folder/tianji_robotic_project`
- Git 分支/提交：`develop_12_kinematic_branch`，`d5d1392d74d2f8522a5cc6ace0213cbf4984c7f8`
- 已本地重建（Git 忽略）：`.venv-wuji-teleop/`
- 运行时已固定：Python 3.12.3、MuJoCo 3.10.0、NumPy 2.5.1、MCAP 1.4.0、Wuji SDK 2026.7.21。
- `sha256sum --check docs/simulation/protected-files.sha256`：全部 `OK`。
- 两份关键 MCAP hash 与迁移手册一致：
  - `session_20260802_174440_936_right_to_left_wuji_hand.mcap`：`8b3afb35cddffe54cc2b804221cf328dd28b6a93f877e773296e79cf5896a6af`
  - `session_20260802_174440_936.mcap`：`0d8ef1225e8f01854436b421660e071033c816ea715caeb38076fe6436b915a4`
- 快速 CLI + 架构回归：`27 passed`。
- 真实 MCAP 聚焦回归：`32 passed in 27.77s`。
- Headless 联动案例成功：`cuts=5 hand_cycles=5 surface_offset_m=0.040 min_distance_m=0.022 clearance_tier_m=0.020 max_penetration_m=0.000000 thumb_clearance_m=0.003`。
- 完整仿真回归：`261 passed, 1 warning in 164.43s`。
  唯一 warning 是文档已知的旧 `chop` 接触力观察告警：39.611 N 超过 30 N；不是本次录制切菜问题。

## Viewer 阻塞原因

Headless 正常，但 GLFW Viewer 失败：

```text
GLX: Failed to create context: BadValue
ERROR: could not create window
```

诊断结果：NVIDIA 用户态库/已安装包为 `595.84`，当前内核已加载模块为 `595.71.05`：

```text
nvidia-smi
Failed to initialize NVML: Driver/library version mismatch
NVML library version: 595.84

/proc/driver/nvidia/version
NVRM version: ... 595.71.05
```

系统自 2026-07-04 未重启。优先处理方式是重启，让内核加载已安装的 595.84 模块；不要在未确认重启效果前重装驱动或修改项目代码。

## 重启后执行顺序

在项目根目录执行：

```bash
cd /home/zhoutong/august_folder/tianji_robotic_project
nvidia-smi
env -u PYTHONPATH .venv-wuji-teleop/bin/twin-sim recorded-hand-guarded-chop \
  --final-hold 60 \
  --hand-mcap recordings/wuji/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap
```

成功标准：

1. `nvidia-smi` 不再报告 `Driver/library version mismatch`；
2. MuJoCo Viewer 窗口打开；
3. 观察到刀和 Wuji 左手同步完成 5 次切削/护手动作，刀始终在机器人坐标意义的左手右侧；
4. 若 Viewer 仍失败，保存完整报错，不要更改切菜轨迹或安全阈值。

## 重启后对 Codex 说什么

直接发送：

> 我已经重启了。请读取 `doc_zt/mujoco_reboot_handoff_2026-08-05.md`，先运行 `nvidia-smi`，然后继续修复并演示 MuJoCo 的 `recorded-hand-guarded-chop` Viewer；不要处理 ROS 2 或真机。

## 工作区注意事项

不要删除或覆盖用户已有的未跟踪项：`doc_zt/`、`recordings (2)/`、`recordings.zip`。本轮还产生了 MuJoCo 的本地日志 `MUJOCO_LOG.TXT`；它不是版本库修改。

## Suggested skills

- `superpowers:systematic-debugging`：若重启后 `nvidia-smi` 或 Viewer 仍失败。
- `superpowers:verification-before-completion`：宣称 Viewer 恢复或演示成功之前。
