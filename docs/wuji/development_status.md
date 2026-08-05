# Tianji + Wuji 集成开发状态

更新时间：2026-08-05

> 新智能体请先阅读
> [current_version_handoff.md](../simulation/current_version_handoff.md) 第 0 节。
> 其中记录当前录制联动任务的动机、机器人坐标、数据流、实测证据、已知风险和
> 代码入口；本文件后续早期参数是历史演进，不代表当前默认值。

## 录制手势与双臂切菜联动（2026-08-04）

新增独立的 `recorded-hand-guarded-chop` 仿真案例，原 `guarded-chop` 未改为
记录模式。Wuji 左手继续刚性安装在天机左臂末端，真实 499 帧、4.150116 秒的
掌心向下桌面后退动作被重采样到 10 ms 联合控制时钟，并在 5 次右臂切菜前各
执行一次。每轮均先产生 `HAND_SAFE`，右刀才进入 `CUT_DOWN`；轮间使用不少于
0.5 秒的最小加加速度复位。

联合预检会同步抬高砧板与被切物体，并搜索最低可行高度。当前录制在桌面抬高
40 mm 时完成 5 个左手周期与 5 条右刀轨迹。根据 Viewer 反馈，完整掌部坐标系
已绕桌面法向旋转 `-90°`，手腕方向同步旋转，30 mm 后退由前后 `-X` 改为与
切点一致的左右 `+Y`；安全锚点相应移到刀路靠左臂的 `-X` 一侧 120 mm。
修正后的计划刀手最小距离约 29 mm，最大桌面穿透约 0.358 mm，拇指最小间隙
约 32 mm，分别满足 20 mm、0.5 mm 和
10 mm 的约束。真实 Headless 状态机测试已完成 `5/5` 刀和 `5/5` 手势周期。
运行方式见 [recorded_hand_guarded_chop.md](../simulation/recorded_hand_guarded_chop.md)。

后续纯桌面修正移除了新任务画面中的切菜方块、两个拾取底座、拾取方块和目标
标记，右刀改为直接接触砧板表面。机器人自身左侧经双臂基座确认是世界 `+Y`；
左手掌心参考点比每个刀位领先 160 mm，并靠近机器人 80 mm，录制动作继续向
`+Y` 后退约 30 mm。160 mm 是为长手指实体留出空间后的掌心间隔；实际全几何
最小刀手距离为 28 mm。Headless 完成 5 刀/5 手势，最大手部桌面穿透
0.356 mm，拇指最小间隙 32 mm；相关新旧任务回归为 `44 passed`。

连续倒手修正将五次重复回放改成一次录制的五个连续时间分段，删除轮间左臂
向右 RESET。长指 MCP 屈曲限制为 0.30 rad，PIP 至少比 MCP 多屈曲 0.25 rad；
指垫实际运动用于分配左腕总计 32 mm 的单调 `+Y` 后退，四个长指单步向刀侧
波动不超过 0.5 mm，首末净位移均向机器人左侧。PIP 整形扩大了手部包络，首
刀掌心领先量提高到 240 mm；五刀后仍领先约 188 mm，实际全几何最小刀手距离
40 mm。Headless 与 Viewer 均完成 5 刀/5 连续分段，手部桌面穿透为 0，拇指
间隙 32 mm；本轮聚焦回归为 `36 passed`。

远节压持修正使用 MuJoCo link4 局部 `+Z` 逐帧求解 DIP，并在 PREPARE 最后
0.3 秒平滑进入目标。RETREAT/HOLD 中食指、中指、无名指、小指的最大远节偏角
分别为 7.63°、1.02°、2.86°、5.64°；PIP 峰峰值分别为 0.385、0.261、
0.310、0.358 rad，DIP 峰峰值为 1.141、0.831、0.903、1.323 rad。四指指垫
净后退 15.1–19.4 mm，最差单步向刀侧波动 0.481 mm。Viewer/Headless 均完成，
最小刀手距离 40.4 mm、手部穿透 0、拇指间隙 31.7 mm；聚焦回归为
`36 passed`。

同步近距离切削修正删除了 `HAND_SAFE → CUT_DOWN` 串行阶段。一次录制仍连续分为
5 段，但每段现在与右刀的下降/回升波形共用 10 ms 时钟；每个采样同时命令右臂、
左臂和 Wuji 手。右刀 `+Y` 位置逐帧跟踪最靠刀的长指 pad，并以
`right_blade_edge_bot` 为基准保持 `30±3 mm` 侧向距离。为兼顾整手碰撞包络，
掌部纵向偏置调整为 `-180 mm`。真实 Headless 完成 `5/5` 同步周期，桌面抬高
40 mm，侧向距离为 `30..30 mm`，全几何最小距离 32 mm，桌面穿透 0，拇指
间隙 32 mm；聚焦回归为 `31 passed in 67.55s`。

深度对齐与指腹接触修正将右刀世界 `X` 逐帧移动到四指接触区，删除视觉上的
约 182 mm 前后错位。侧向搜索先尝试 20 mm 全几何层、再尝试 10 mm 回退层；
真实录制选择 45 mm 固定侧向间距并在首选层得到 22 mm 最近距离，最大 `X`
误差小于 0.5 mm。PIP 峰峰值缩小为 0.208–0.308 rad，四指 DIP 均为
0.35 rad；四指最低离桌为 0–4.998 mm，拇指 2.98 mm，桌面穿透为 0。
Headless 完成 `5/5` 同步周期，聚焦回归为 `32 passed in 96.55s`。
完整仿真回归为 `261 passed, 1 warning in 468.28s`；唯一 warning 属于旧
`chop` 接触力观察阈值。本轮实现提交为 `6b99835`。当前 Viewer 仍只播放一次，
后续计划增加相机准备停留、可配置循环次数和平滑循环复位。

循环回放开发进展：`wuji-table-retreat` 已增加正整数 `--loops`，默认循环 3 次；
正向动作改为按 MCAP 累计时间戳调度，相邻动作间使用通过桌面安全预检的平滑
RESET，每段至少 0.5 秒。Viewer 完成循环后保持最终姿势，用户关闭窗口后退出；
Headless 完成后立即退出。Viewer 的关闭路径已改为状态感知，避免窗口已关闭后
再次调用 MuJoCo `close()` 导致 Python 进程停在 futex。真实 499 帧录制的默认
三循环 Headless 与 Viewer 均完成 `3/3`，计划时长 13.454 秒；Viewer 在最终
姿势持续保持，手动关闭后命令立即以退出码 0 返回，未留下运行中的新进程。

最新录制驱动手势：`wuji-table-retreat` 现在优先直接读取
`session_20260802_174440_936_right_to_left_wuji_hand.mcap` 的 `/joint_states`，并对
原始 `session_20260802_174440_936.mcap` 保留官方重定向回退路径。两条路径均验证
为 499 帧、活动区间 `311:464`、后退 30 mm、掌心向下、拇指最小间隙
34.7 mm、最大穿透 0.498 mm、关节修正为 0。准备阶段逐帧贴桌，后退阶段允许
逐渐离桌但整只手不得穿透。掌侧标定已加入“长指必须朝桌面屈曲”的回归检查，
修复了仅靠局部轴符号造成的掌背翻转。

桌面手势进展：`wuji-table-retreat` 已改用解剖掌面标定和真实 MuJoCo 碰撞距离，
不再把指尖 mesh 原点误当成实际接触面。PLACE 保持掌面与桌面平行并让四个长指
贴近桌面；RETREAT 允许指尖逐渐离桌，同时逐帧抬高任何可能穿透的姿态。真实
录制仍选择第 410 帧；缩短时长的 851 帧 Headless 验收为后退 30 mm、拇指间隙
83.1 mm、最大真实穿透 0.327 mm、最大指尖抬升 58.5 mm。侧视 PLACE/HOLD
截图确认桌面不再切入手掌或指节。原始 MCAP 与 `wuji-replay` 均保持不变。
本次修复后的全量回归为 `246 passed, 1 warning in 386.07s`；唯一 warning 仍为
既有切菜仿真的 39.611 N 接触力观察阈值。

最新修正：Wuji MCAP 默认回放已从双臂组合场景切换为官方 hand-only 左手模型；
模型严格包含 20 个关节/actuator，不再显示或加载天机机械臂。

hand-only 修正当时的基线为 `240 passed, 1 warning in 391.29s`。真实 1676 帧录制已在
Headless 与 Viewer 两条 hand-only 路径完成，CLI 均返回 0；Viewer 在窗口失效后
不会继续执行 `sync()`。唯一 warning 仍是原有切菜仿真的接触力观察阈值。

## 本轮目标与结论

本轮完成的是“Wuji 手套录制到左手 MuJoCo 回放”的本地闭环，以及后续真机
SDK/ROS 2 接入所需的安全接口。所有自研实现归入同一个
`tianji_robotic_project` Git 仓库；相邻 `../wuji-technology/` 中的 10 个官方
仓库保持独立、未修改，远端仍为 `https://github.com/wuji-technology/...`。

已实现：

- 建立 `src/tianji_robotics`，按 `wuji_hand`、`simulation`、`hardware`、
  `data`、`workflows` 分层，并以静态测试禁止领域层直接依赖厂商 SDK；
- 解析 Wuji Studio 右手骨架 MCAP，镜像到左手坐标并调用官方离线重定向；
- 使用不可变、有限值、严格时间戳的 20 关节轨迹模型；
- 保存安全 NPZ，输出 ROS 2 JointState 语义 MCAP；
- 使用项目内 vendored 的官方 hand-only 左手 MJCF 实现独立 MuJoCo 后端，完成
  按录制时间戳的 Headless/Viewer 回放和关节范围、单步变化量预检；该后端不
  导入或加载机械臂；
- 提供 `tianji-robot sim wuji-replay`；
- 提供注入式 `SdkWujiHand` 生命周期契约，以及只读的
  `tianji-robot hardware wuji-sdk preflight`；
- ROS 2 保持在同一仓库的 `ros2_ws/`，但与直接 SDK 路径分目录；
- 提供幂等环境脚本 `scripts/setup_wuji_teleop_env.sh`；
- 将 `wuji-record-data` 的 24 个文件迁至被 Git 忽略的
  `recordings/wuji/`，迁移前后均为 88,810,615 字节且逐文件 SHA-256 一致；
- 将原 `wuji-glove-recorder`、`doc_zt` 和自研设计记录迁至
  `archive/wuji_glove_recorder/legacy_2026_08_02/`。

## 最新验证证据

- 最新全量测试：`246 passed, 1 warning in 386.07s`；警告为已有切菜仿真接触力
  39.611 N 超过 30 N 观察阈值，不是 Wuji 回放失败；
- Wuji 专用测试：`54 passed in 0.19s`；
- 真实录制 `session_20260802_162909_764.mcap`：成功回放 1676 帧、
  13.958 秒；生成轨迹形状 `(1676, 20)`，所有数值有限；
- 在 `DISPLAY=:0` 下 Viewer 路径也完整执行 1676 帧并正常退出；视觉方向仍需由
  操作者在目标显示器前做人工确认；
- 硬件预检输出明确为 `no device accessed`，代码路径不导入物理 runtime；
- 10 个官方嵌套 Git 仓库均为 clean，remote 未变化。

## 当前有效的 shell 脚本

以下 5 个文件真实存在、具有可执行权限，并已通过 `bash -n` 语法检查：

| 脚本 | 功能 | 窗口 | 真机访问 |
|---|---|---|---|
| `scripts/setup_wuji_teleop_env.sh` | 创建/更新 `.venv-wuji-teleop` 并安装 Wuji 离线依赖 | 无 | 否 |
| `scripts/run_guarded_chop.sh` | 双臂猫爪倒手切菜在线仿真 | Viewer | 否 |
| `scripts/run_guarded_chop_record.sh` | 双臂切菜并保存 MuJoCo 状态 NPZ | Viewer | 否 |
| `scripts/replay_guarded_chop_2x.sh` | 读取已有状态 NPZ 并以二倍速回放 | Viewer | 否 |
| `scripts/run_guarded_chop_record_replay.sh` | 在线执行后在同一 Viewer 二倍速状态回放 | Viewer | 否 |

脚本依赖的 `.venv/bin/twin-sim` 与 `.venv-wuji-teleop/bin/tianji-robot` 当前均
可执行，默认 `recordings/guarded_chop_latest.npz` 也存在。此前讨论的
`scripts/verification/...` 尚未创建，因此不能列为有效脚本。

Wuji 手套 MCAP hand-only 回放目前使用已验证的 CLI，而不是 shell 包装：

```bash
.venv-wuji-teleop/bin/tianji-robot sim wuji-replay \
  recordings/wuji/august_02/session_20260802_162909_764.mcap
```

该命令只显示官方左 Wuji Hand。真实录制已完成 1676 帧、13.958 秒的
Headless 与 Viewer 验证，两个路径均返回 0。

## 已知问题与边界

- 本轮没有连接手套、Wuji 实体手或天机机械臂，因此不能声称真机通信、使能、
  急停或运动已经上机验证；
- `SdkWujiHand` 目前是稳定的安全契约，官方 SDK physical runtime 尚未实现；
- Wuji ROS 2 真机驱动尚未接入统一设备所有权锁；
- `real_robot_debug/` 中的天机机械臂代码尚未按新架构拆分；
- `twin_sim` 暂时作为兼容包保留，尚未全部迁入 `tianji_robotics`；
- 全量测试中的接触力 warning 应在机械臂仿真后续工作中单独评估。

## 后续计划

1. 在实体设备旁完成官方 Wuji SDK runtime 适配，加入连接超时、设备身份、
   使能确认、命令限速、故障撤使能和急停检查，再做低速上机验收。
2. 增加 Wuji ROS 2 physical adapter，并实现 SDK/ROS 2 互斥设备所有权。
3. 将 `real_robot_debug/` 的天机机械臂代码拆为领域模型、厂商适配器和受保护
   workflow，仿真与真机继续使用不同 CLI 域。
4. 完成所有调用方迁移后移除 `twin_sim` 兼容层。
5. 单独处理切菜仿真接触力阈值警告，并建立机械臂/灵巧手联合真机测试矩阵。

离线复现命令和真机接口约束分别见 [offline_replay.md](offline_replay.md) 与
[hardware_interfaces.md](hardware_interfaces.md)。
