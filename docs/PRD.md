# PRD：机器人可视化与控制链路优化

## Problem Statement

当前项目已经具备 MuJoCo 控制、ROS2 可视化和基础规划能力，但链路仍需要进一步统一接口、整理文档、收敛职责，确保未来接入不同自由度机器人模型和新规划算法库时不扩散复杂度。

## Solution

以内部通用接口作为规划、控制、ROS 适配之间的稳定交换形式；以 MuJoCo 控制器作为唯一控制后端；以 ROS2/RViz 提供展示与外部消息入口；以文档、测试和架构评审持续约束可维护性。

## User Stories

1. 作为机器人开发者，我想通过统一关节状态表达机器人状态，以便不同自由度模型共享同一控制链路。
2. 作为规划算法开发者，我想通过规划器 Interface 返回关节轨迹，以便未来接入 OMPL 或其他算法库。
3. 作为控制链路维护者，我想让 ROS 消息只在 Adapter 内转换，以便核心控制代码不依赖 ROS。
4. 作为可视化用户，我想用 ROS2 launch 一键启动 RViz 与 MuJoCo 控制节点，以便检查机器人运动。
5. 作为测试维护者，我想通过公共 Interface 编写测试，以便重构内部实现时测试仍然有效。
6. 作为项目维护者，我想看到完成进度和缺漏清单，以便逐步推进开发流程。
7. 作为新机器人接入者，我想复用资产解析和关节元数据加载，以便降低模型接入成本。
8. 作为系统集成者，我想保留标准 ROS JointState/JointTrajectory 消息，以便与 ROS 工具链兼容。

## Implementation Decisions

- 内部接口以 `JointStateData`、`TrajectoryPointData`、`JointTrajectoryData`、`PlanRequest`、`PlanResult`、`ControlCommand`、`ControlState` 为核心。
- 轨迹执行从 ROS 节点中抽成独立 Module，使时间推进、完成状态和命令生成集中在一个可测试 Interface 后。
- MuJoCo runtime 继续拒绝 URDF，只加载 MJCF/XML。
- ROS2 使用 `ament_python`，并发布标准 `/joint_states`，订阅标准 `JointTrajectory`。
- 项目按 ROS2 workspace 组织为 `cook_core`、`cook_description`、`cook_mujoco`、`cook_bringup`，避免根目录伪装成 ROS package。
- PRD 暂存为仓库文档；当前未配置 issue tracker，无法发布 `ready-for-agent` issue。

## Testing Decisions

- 测试只验证公共 Interface 行为，不断言内部调用顺序。
- 覆盖接口校验、轨迹序列化、轨迹执行、规划器输出、MuJoCo 控制器、URDF 转 MJCF、ROS 消息转换。
- ROS 相关测试在未 source ROS 环境时跳过；在 ROS 环境中应通过。

## Out of Scope

- 不实现自定义 ROS `.msg`。
- 不在运行时做 URDF 到 MJCF 转换。
- 不实现 OMPL Adapter，只保留清晰接入点。
- 不引入新的控制后端。

## Further Notes

当前代码重点是小规模可维护链路。后续优化优先级：资产配置 Module、规划器注册 Module、ROS launch 参数收敛、更多机器人模型验收样例。
