# Tianji + Wuji 集成开发状态

更新时间：2026-08-03

最新修正：Wuji MCAP 默认回放已从双臂组合场景切换为官方 hand-only 左手模型；
模型严格包含 20 个关节/actuator，不再显示或加载天机机械臂。

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
- 用现有 `SimWujiHand` 封装 MuJoCo 后端，完成按录制时间戳的 Headless/Viewer
  回放和关节范围、单步变化量预检；
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

- 全量测试：`234 passed, 1 warning in 364.56s`；警告为已有切菜仿真接触力
  39.611 N 超过 30 N 观察阈值，不是 Wuji 回放失败；
- Wuji 专用测试：`54 passed in 0.19s`；
- 真实录制 `session_20260802_162909_764.mcap`：成功回放 1676 帧、
  13.958 秒；生成轨迹形状 `(1676, 20)`，所有数值有限；
- 在 `DISPLAY=:0` 下 Viewer 路径也完整执行 1676 帧并正常退出；视觉方向仍需由
  操作者在目标显示器前做人工确认；
- 硬件预检输出明确为 `no device accessed`，代码路径不导入物理 runtime；
- 10 个官方嵌套 Git 仓库均为 clean，remote 未变化。

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
