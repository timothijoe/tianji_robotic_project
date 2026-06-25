# 研发流程文档

## 当前链路

1. `cook_description`：URDF/STL 作为源资产，离线转换生成 MJCF/XML。
2. `cook_description`：从 URDF 读取 link、joint、limit。
3. `cook_core`：规划器接收 `PlanRequest`，输出 `PlanResult` / `JointTrajectoryData`。
4. `cook_core`：任务流通过 `RobotCommandPort` 发出后端无关的关节指令。
5. `cook_core`：`TrajectoryExecutor` 按时间把轨迹点转换为 `ControlCommand`。
6. `cook_mujoco`：MuJoCo controller 应用关节目标并发布当前关节状态。
7. `cook_bringup`：Adapter 在标准 ROS 消息和内部接口之间转换。
8. `cook_bringup`：GUI 示教器读取 `/joint_states`，生成 `JointTrajectoryData` 后发布标准 `JointTrajectory`。
9. `cook_bringup`：MuJoCo 控制节点发布虚拟 `tcp_link` TF、实际 TCP 路径和预测 TCP 路径。
10. `cook_bringup`：robot_state_publisher + RViz 使用 URDF、`/joint_states`、TF 和 TCP Path topic 展示运动。

## 开发阶段

- 阶段 1：新增或更新测试，明确一个可观察行为。
- 阶段 2：实现最小代码使测试通过。
- 阶段 3：在 GREEN 状态重构，深化 Module，减少浅 Interface。
- 阶段 4：运行单元测试、ROS 转换测试、MuJoCo headless demo。
- 阶段 5：需要 ROS2 时运行 `colcon build` 和 launch 参数检查。

## 新机器人接入

- 放入 URDF、STL、生成 MJCF/XML。
- 确认 URDF 中 movable joints 和 MJCF joint 名一致。
- 使用同一 `JointStateData` 与 `JointTrajectoryData` 链路。
- 示教器会从 URDF movable joints 和 limit 自动生成关节滑块。
- 为模型增加元数据测试：关节数量、关节名、limit、MJCF 可加载。

## 新规划算法接入

- 实现 `Planner.plan(request) -> PlanResult`。
- 输入输出只使用内部接口类型。
- 算法库私有配置留在 Adapter 内，不泄漏到控制器。
- OMPL 作为可选依赖放在 `cook_mujoco.planning`，因为它复用 MuJoCo contact 做状态有效性检查。
- 碰撞过滤基于 body pair；当前默认模型零位已有接触，规划时默认允许起点已有接触。
- 规划失败返回 `PlanResult(success=False, message=...)` 或明确异常，不让控制节点崩溃。
- 用行为测试验证轨迹起点、终点、时间单调、维度完整。
- 碰撞规划测试至少覆盖：起点已有接触过滤、禁用过滤后的碰撞失败、OMPL 未安装时的 `OmplUnavailableError`、fake/mock OMPL 的轨迹转换。
- README 只保留规划 demo 和安装入口；完整设计说明维护在 `docs/ARCHITECTURE.md`。

## 验证命令

```bash
conda run -n gmp env PYTHONPATH=src/cook_core:src/cook_description:src/cook_mujoco:src/cook_bringup python -m pytest -q
source /opt/ros/humble/setup.bash
PYTHONPATH=src/cook_core:src/cook_description:src/cook_mujoco:src/cook_bringup:$PYTHONPATH python -m pytest -q tests/test_ros_conversions.py
colcon build --packages-select cook_core cook_description cook_mujoco cook_bringup
ROS_LOG_DIR=/tmp/ros_log ros2 launch cook_bringup visualization.launch.py --show-args
```
