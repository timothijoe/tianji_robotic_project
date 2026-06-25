# 完成进度与缺漏

## 已完成

- [x] MuJoCo-only 控制器。
- [x] URDF/STL 示例资产迁入通用 `assets/robot`。
- [x] 离线 URDF 到 MJCF/XML 转换。
- [x] ROS2 `ament_python` 包配置。
- [x] RViz launch 链路。
- [x] 内部统一接口类型。
- [x] ROS 标准消息 Adapter。
- [x] 轨迹执行 Module。
- [x] ROS2 workspace 拆分为 `cook_core`、`cook_description`、`cook_mujoco`、`cook_bringup`。
- [x] GUI 示教器随可视化 launch 启动，并对齐内部 `JointTrajectoryData` 与 ROS `JointTrajectory`。
- [x] 行为测试覆盖核心链路。
- [x] 精炼背景意图文档、PRD、研发流程文档。

## 待优化

- [x] 增加资产配置 Module，统一本地路径、ROS share 路径和 package URI。
- [x] 增加规划器注册 Module，支持按配置选择线性规划、OMPL Adapter 或其他算法。
- [x] 增加多机器人接口样例测试，覆盖不同自由度和不同 joint order。
- [ ] 增加 ROS 控制节点集成测试，降低 launch 回归风险。
- [ ] 配置 issue tracker 后发布 PRD 并打 `ready-for-agent` 标签。

## 当前风险

- ROS launch 依赖本机 ROS2 Humble 环境。
- GUI 示教器依赖本机 Tk 环境；无显示环境请使用 `use_teach_pendant:=false`。
- RViz 展示使用 URDF，控制使用 MJCF/XML，需要持续验证两者 joint 名一致。
- 当前只有一个机器人样例，多自由度泛化还需要更多模型验收。
