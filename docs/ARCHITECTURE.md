# Cook 项目整体链路文档

## 1. 项目定位

Cook 是一个最小完整、可扩展的机器人可视化与控制框架。当前目标不是做完整工业控制栈，而是把机器人资产、规划、控制、仿真、ROS2 可视化和任务脚本之间的链路打通，并保持后续扩展边界清晰。

核心约束：

- MuJoCo 是当前唯一仿真控制运行时。
- ROS2/RViz 负责消息接入、状态发布、GUI 接入和可视化。
- 核心数据接口放在 `cook_core`，不直接依赖 ROS2 或 MuJoCo。
- URDF 服务 ROS/RViz 和关节元数据解析，MJCF/XML 服务 MuJoCo 运行时。
- 任务脚本不直接依赖 ROS topic、ROS message、MuJoCo API 或硬件 SDK。
- 规划器接口保留为可替换模块，未来可接入 OMPL 或其他算法库。
- 目录、变量、接口命名保持通用语义，不绑定特定机器人或项目代号。

整体设计原则是：上层表达“要机器人做什么”，中间层负责“接口和消息对齐”，底层负责“如何在仿真或真机中执行”。

## 2. 包结构与职责

当前 workspace 是标准 ROS2 多 package 结构，功能包统一放在 `src/` 下。

```text
src/
  cook_core/
  cook_description/
  cook_mujoco/
  cook_bringup/
examples/
tests/
docs/
```

### `cook_core`

项目核心接口层。

职责：

- 定义通用数据类型：关节状态、轨迹点、关节轨迹、规划请求、规划结果、控制命令、控制状态。
- 定义规划器协议和默认线性规划器。
- 提供轨迹执行器和轨迹插值采样。
- 提供示教器内部模型。
- 提供后端无关的机器人任务控制端口。

设计要求：

- 不依赖 `rclpy`。
- 不依赖 MuJoCo。
- 不知道 ROS topic 名称。
- 不知道 RViz 或 GUI。
- 不知道真机通信协议。

### `cook_description`

机器人资产与模型描述层。

职责：

- 管理默认机器人资产目录。
- 提供 URDF、MJCF/XML、mesh 路径。
- 解析 URDF 中的 link、joint、joint limit。
- 提供离线 URDF 到 MJCF 的转换工具。
- 为 ROS launch 提供可用的 `robot_description`。

运行边界：

- URDF 用于 ROS/RViz 和元数据。
- MJCF/XML 用于 MuJoCo 运行时。
- 控制节点启动时只加载预生成 MJCF/XML，不在线转换 URDF。

### `cook_mujoco`

MuJoCo 仿真控制后端。

职责：

- 加载 MuJoCo MJCF/XML 模型。
- 建立关节名到 MuJoCo qpos/dof 的映射。
- 读取关节位置。
- 读取 MuJoCo body 位姿。
- 执行最小位置控制。
- 提供本地 demo CLI。

设计要求：

- 不承担 ROS 通信。
- 不发布 topic。
- 不知道 RViz 配置。
- 不承担任务脚本 API。

### `cook_bringup`

ROS2 集成、启动和可视化层。

职责：

- 提供 ROS message 与 `cook_core` 数据结构之间的转换。
- 启动 MuJoCo controller node。
- 启动 GUI 示教器。
- 启动 `robot_state_publisher` 和 RViz。
- 发布 `/joint_states`。
- 发布 TCP TF 和 TCP path。
- 提供 ROS2 后端控制适配器。

设计要求：

- ROS 相关依赖集中在这一层。
- 不把 ROS message 泄漏到 `cook_core`。
- 不让 GUI 自己定义独立轨迹格式。

### `examples`

独立任务脚本示例。

当前重点是 `examples/function_control_demo.py`，展示任务代码如何通过函数调用控制机器人，并且不直接 import ROS 相关模块。

### `tests`

项目测试目录。

覆盖范围包括：

- workspace package 布局。
- 核心接口类型。
- 轨迹插值与执行。
- 线性规划器。
- MuJoCo runtime/controller。
- ROS message 转换。
- TCP path 计算。
- ROS 控制客户端。
- 示例脚本 import 与 dry-run。

## 3. 总体运行链路

完整控制和可视化链路如下：

```text
任务脚本 / GUI 示教器 / demo publisher
        |
        v
统一控制接口或示教模型
        |
        v
JointTrajectoryData
        |
        v
ROS2 适配层
        |
        v
trajectory_msgs/JointTrajectory
        |
        v
MuJoCo Controller Node
        |
        v
TrajectoryExecutor 时间插值采样
        |
        v
MujocoPositionController
        |
        v
MujocoRuntime / MJCF 模型
        |
        v
joint_states + tcp_link TF + TCP Path
        |
        v
RViz 可视化
```

其中，任务脚本和 GUI 是控制输入来源，MuJoCo controller node 是控制执行中心，RViz 是状态观察端。

## 4. 核心数据接口

核心类型位于 `src/cook_core/cook_core/interfaces.py`。

### `JointStateData`

表示一组关节状态。

字段：

- `names`: 关节名顺序。
- `positions`: 关节位置。
- `velocities`: 可选速度。
- `efforts`: 可选力矩。
- `timestamp_sec`: 可选时间戳。

常用方法：

- `from_mapping()`: 从 `{joint_name: position}` 创建状态。
- `as_mapping()`: 转为 `{joint_name: position}`。
- `ordered_positions(joint_names)`: 按指定关节顺序输出位置。

主要用途：

- 表示当前机器人状态。
- 表示控制目标。
- 表示规划起点和终点。

### `TrajectoryPointData`

表示轨迹中的一个时间点。

字段：

- `positions`: 每个关节的位置。
- `time_from_start_sec`: 从轨迹开始计时的时间。
- `velocities`: 可选速度。
- `accelerations`: 可选加速度。

约束：

- 时间必须非负。
- 数值必须是有限数。

### `JointTrajectoryData`

表示完整关节轨迹。

字段：

- `joint_names`: 轨迹关节顺序。
- `points`: 轨迹点序列。
- `trajectory_id`: 轨迹标识。
- `frame_id`: 坐标系标识。
- `source`: 来源，例如 `linear`、`teach_pendant`、`robot_command`、`ros`。
- `metadata`: 扩展元数据。

约束：

- `joint_names` 不能为空。
- 每个轨迹点必须包含所有关节。
- 轨迹点不能包含未知关节。
- 时间必须单调不下降。

这是项目内最重要的消息传递格式。规划器、示教器、任务控制端口和 ROS adapter 都围绕它对齐。

### `PlanRequest` / `PlanResult`

规划器输入输出。

`PlanRequest` 包含：

- 起点 `start_state`。
- 终点 `goal_state`。
- 参与规划的 `joint_names`。
- 轨迹时长 `duration_sec`。
- 采样点数 `waypoint_count`。
- 规划超时 `planning_time_sec`。
- 状态有效性检查分辨率 `collision_check_resolution`。
- 扩展元数据 `metadata`。

`PlanResult` 包含：

- `trajectory`: 统一的 `JointTrajectoryData`。
- `success`: 是否规划成功。
- `message`: 失败或诊断信息。
- `planner_name`: 实际规划器名称。
- `metadata`: 规划器扩展信息。

未来接入 OMPL 时，应保持 `PlanRequest -> Planner -> PlanResult` 这条外部接口不变。

### `ControlCommand` / `ControlState`

控制层输入输出。

`ControlCommand` 表示控制器要执行的目标状态。

`ControlState` 表示控制器执行后的状态反馈。

控制器内部可以替换，但对外仍围绕这两个接口表达目标和结果。

## 5. 任务脚本控制链路

任务脚本 API 位于 `src/cook_core/cook_core/robot.py`。

公开协议是 `RobotCommandPort`：

```python
class RobotCommandPort(Protocol):
    def move_joints(self, target_positions, *, duration_sec=1.0) -> JointTrajectoryData:
        ...

    def move_joint_sequence(self, waypoints, *, duration_sec) -> JointTrajectoryData:
        ...

    def hold_current(self, *, duration_sec=0.0) -> JointTrajectoryData:
        ...

    def close(self) -> None:
        ...
```

### 使用方式

```python
from cook_core.robot import create_robot_command_port

robot = create_robot_command_port(backend="ros2")

robot.move_joints(
    {
        "Joint1_L": 0.2,
        "Joint2_L": -0.1,
        "Joint3_L": 0.15,
    },
    duration_sec=1.5,
)

robot.close()
```

任务代码只 import `cook_core.robot`，不需要知道：

- ROS topic 名称。
- ROS node 生命周期。
- `trajectory_msgs/JointTrajectory` 字段。
- MuJoCo runtime API。
- 当前后端是仿真还是真机。

### `RobotCommandBuilder`

`RobotCommandBuilder` 是任务指令对齐层。

职责：

- 维护统一关节顺序。
- 支持部分关节目标。
- 未指定关节保持当前内部位置。
- 校验未知关节。
- 校验非有限数。
- 校验非负 duration。
- 生成 `JointTrajectoryData`。

行为：

- `move_joints()` 生成两点轨迹：当前状态到目标状态。
- `move_joint_sequence()` 生成多点轨迹：按总时长均匀分配时间。
- `hold_current()` 生成保持当前位置的两点轨迹。

### 当前后端

`create_robot_command_port()` 当前支持：

- `backend="ros2"`：创建 `cook_bringup.client.RosRobotCommandPort`。
- `backend="fake"`：创建 `RecordingRobotCommandPort`，用于 dry-run 和测试。

真机接入时应新增另一个 `RobotCommandPort` 实现，而不是修改任务脚本。

## 6. ROS2 控制客户端链路

ROS2 控制客户端位于 `src/cook_bringup/cook_bringup/client.py`。

`RosRobotCommandPort` 的职责：

- 在内部初始化或复用 `rclpy` context。
- 自动加载默认 URDF 并读取 movable joints。
- 使用 `RobotCommandBuilder` 生成 `JointTrajectoryData`。
- 转换成 `trajectory_msgs/JointTrajectory`。
- 发布到控制节点的轨迹 topic。
- 提供可重复调用的 `close()`。

默认 topic：

```text
/mujoco_controller_node/joint_trajectory
```

这个类位于 `cook_bringup`，因此可以依赖 ROS；但任务脚本只通过 `cook_core.robot.create_robot_command_port()` 间接使用它。

## 7. ROS message 转换链路

转换逻辑位于 `src/cook_bringup/cook_bringup/ros/conversions.py`。

转换关系：

```text
sensor_msgs/JointState
    <-> JointStateData

trajectory_msgs/JointTrajectory
    <-> JointTrajectoryData
```

关键规则：

- `JointState.header.stamp` 转为 `timestamp_sec`。
- `JointTrajectory.header.frame_id` 映射到 `trajectory_id/frame_id`。
- ROS 轨迹点的 `time_from_start` 转为秒。
- 输出 ROS message 时按 `JointTrajectoryData.joint_names` 固定顺序写入。

设计目的：

- ROS message 只存在于 adapter 层。
- 核心规划、控制和任务代码使用普通 Python 数据类型。
- 将来真机协议转换也可以复用同一类 adapter 设计。

## 8. ROS2 控制节点链路

MuJoCo 控制节点位于 `src/cook_bringup/cook_bringup/ros/mujoco_controller_node.py`。

节点名：

```text
mujoco_controller_node
```

订阅：

```text
~/joint_trajectory
```

在默认节点名下展开为：

```text
/mujoco_controller_node/joint_trajectory
```

发布：

```text
/joint_states
/tcp/actual_path
/tcp/predicted_path
TF: Base_L -> tcp_link
```

主要启动步骤：

1. 读取 `model_path` 和 `urdf_path`。
2. 从 URDF 解析 movable joints 和 joint limits。
3. 使用 MJCF 创建 `MujocoRuntime`。
4. 使用 runtime 创建 `MujocoPositionController`。
5. 创建 `TrajectoryExecutor`。
6. 创建 ROS publisher/subscriber/timer。
7. 如果启用 TCP path，则创建独立预测 runtime 和 TF broadcaster。

运行时逻辑：

1. 收到 `JointTrajectory`。
2. 转为 `JointTrajectoryData`。
3. 校验关节顺序是否等于 controller joint order。
4. 启动 `TrajectoryExecutor`。
5. 清空实际 TCP path。
6. 使用预测 runtime 生成 `/tcp/predicted_path`。
7. 控制定时器按当前时间插值出 `ControlCommand`。
8. `MujocoPositionController` 应用目标关节位置。
9. 发布 `/joint_states`。
10. 追加 `/tcp/actual_path` 并广播 `tcp_link` TF。

## 9. 轨迹执行链路

轨迹执行逻辑位于 `src/cook_core/cook_core/trajectory.py`。

`TrajectoryExecutor` 的职责：

- 保存当前活动轨迹。
- 记录轨迹启动时间。
- 按 `now_sec - start_time_sec` 计算 elapsed。
- 调用 `sample_trajectory_positions()` 得到当前目标关节位置。
- 输出 `ControlCommand`。
- 轨迹结束后自动清空活动状态。

`sample_trajectory_positions()` 的行为：

- elapsed 小于首段时取首段插值。
- elapsed 位于两个轨迹点之间时线性插值。
- elapsed 超过最后一个点时返回最后一个点。
- 相邻点时间相同或倒退已在数据结构层约束；零时长段按末端点处理。

这个设计避免了控制节点只在离散 waypoint 上跳变，提高了关节指令响应和轨迹平滑性。

## 10. MuJoCo runtime 链路

MuJoCo runtime 位于 `src/cook_mujoco/cook_mujoco/control/runtime.py`。

`MujocoRuntime` 的职责：

- 校验模型路径后缀，只接受 `.xml` 和 `.mjcf`。
- 明确拒绝运行时直接加载 `.urdf`。
- 使用 `mujoco.MjModel.from_xml_path()` 加载模型。
- 创建 `mujoco.MjData`。
- 建立每个关节到 qpos/dof 地址的映射。
- 限制当前支持一自由度 hinge/slide joint。
- 支持 reset、set_joint_positions、step。
- 支持读取 MuJoCo body pose。

重要约束：

- 运行时只加载预生成 MJCF/XML。
- 关节名必须同时存在于 URDF movable joints 和 MJCF joints 中。
- 当前只支持每个控制关节对应一个 qpos 的模型。

## 11. MuJoCo position controller 链路

控制器位于 `src/cook_mujoco/cook_mujoco/control/controller.py`。

`MujocoPositionController` 的职责：

- 接收 `ControlCommand`。
- 将目标关节位置按 controller joint order 排序。
- 未指定目标关节时保持当前值。
- 按 URDF joint limit clamp。
- 调用 runtime 写入 MuJoCo qpos。
- 返回新的 `ControlState`。

当前控制策略是最小位置控制：

- 不做动力学力矩控制。
- 不做速度/加速度约束。
- 不做碰撞检查。
- 不做实时控制保证。

它的价值是让完整链路可运行，并为未来更复杂控制器保留接口边界。

## 12. 模型资产链路

资产层位于 `src/cook_description`。

默认资产根目录：

```text
src/cook_description/assets/robot/
```

典型结构：

```text
assets/robot/
  urdf/
  mujoco/
  meshes/
```

链路：

```text
URDF + meshes
        |
        | 离线转换 / 校验
        v
MJCF/XML
        |
        v
MuJoCo Runtime
```

URDF 职责：

- 给 `robot_state_publisher` 使用。
- 给 RViz 显示 RobotModel。
- 提供 link、joint、joint limit 元数据。
- 提供 movable joint 顺序。

MJCF/XML 职责：

- 给 MuJoCo 加载。
- 提供实际仿真 body、joint、mesh、collision/visual 信息。
- 提供 TCP parent body 位姿查询来源。

为什么不在运行时使用 URDF：

- MuJoCo 原生运行时需要 MJCF/XML。
- URDF 到 MJCF 涉及 mesh 路径、package URI、惯量和 joint 映射等转换问题。
- 在线转换会增加启动不确定性。
- 控制节点应尽量小、确定、可测试。

## 13. 规划链路

规划接口位于 `src/cook_core/cook_core/planning/interface.py`。

协议：

```text
PlanRequest -> Planner.plan() -> PlanResult
```

当前默认实现是 `LinearJointPlanner`，碰撞感知实现是 `cook_mujoco.planning.OmplJointPlanner`。`cook_core.planning.PlannerRegistry` 负责把配置名称映射到 planner factory；MuJoCo 集成层通过 `create_mujoco_planner_registry()` 注册 `linear` 和 `ompl`，任务示例与 ROS demo publisher 统一调用 `create_planner()`，不再各自维护算法分支。

线性规划流程：

1. 校验 `duration_sec > 0`。
2. 校验 `waypoint_count >= 2`。
3. 按 `joint_names` 从 start/goal 查找位置。
4. 在关节空间线性插值。
5. 生成 `JointTrajectoryData(source="linear")`。

OMPL 规划链路：

```text
PlanRequest
    |
    v
OmplJointPlanner
    |
    v
OMPL RealVectorStateSpace / RRTConnect
    |
    v
MujocoCollisionChecker
    |
    v
PlanResult / JointTrajectoryData(source="ompl")
```

OMPL 作为可选依赖接入，未安装时会抛出 `OmplUnavailableError` 并提示 `pip install ompl`。普通线性规划、MuJoCo 控制和 ROS2 可视化不强制依赖 OMPL。

`MujocoCollisionChecker` 使用独立 MuJoCo runtime 设置关节状态并读取 `data.ncon/contact`。当前默认模型零位存在 `world-Link1_L` 和 `Link5_L-Link7_L` 接触，因此默认开启 `allow_initial_contacts`，会把起点已有 body pair 记录为允许接触；否则零位会直接被判为碰撞状态。由于当前 MJCF 的 geom 未命名，过滤逻辑基于 contact 两端 geom 所属 body，而不是 geom name。

其他算法库接入要求：

- 新增 planner adapter。
- 外部输入仍是 `PlanRequest`。
- 外部输出仍是 `PlanResult`。
- 算法库内部状态、空间定义、碰撞检查对象不能泄漏到任务、控制和 GUI 层。
- 如果需要额外约束，应通过兼容字段或扩展请求类型引入，而不是破坏主接口。

规划器测试要求：

- 基础行为测试覆盖轨迹起点、终点、时间单调和关节维度完整。
- 碰撞测试覆盖默认允许起点已有接触，以及禁用该过滤后能够检测到碰撞。
- OMPL 测试覆盖未安装依赖时的 `OmplUnavailableError`，以及 fake/mock OMPL 下的轨迹转换逻辑。
- ROS demo 和独立任务脚本只验证接入链路，不把 OMPL 类型暴露给外层调用者。

## 14. GUI 示教器链路

GUI 示教器位于 `src/cook_bringup/cook_bringup/ros/teach_pendant_node.py`。

内部模型位于 `src/cook_core/cook_core/teaching.py`。

链路：

```text
Tk GUI 输入
    |
    v
TeachPendantModel
    |
    v
JointTrajectoryData(source="teach_pendant")
    |
    v
trajectory_msgs/JointTrajectory
    |
    v
/mujoco_controller_node/joint_trajectory
```

职责拆分：

- GUI node 负责窗口、输入控件、ROS 订阅发布、生命周期。
- `TeachPendantModel` 负责当前关节状态、limit clamp 和轨迹构造。

这样 GUI 不需要自己拼 ROS message 语义，也不会绕过统一轨迹类型。

## 15. TCP 轨迹可视化链路

TCP 轨迹逻辑位于 `src/cook_bringup/cook_bringup/ros/tcp_path.py`。

当前 RViz 只绘制 TCP 轨迹，不绘制其他 link 的运行轨迹。

控制节点参数：

| 参数 | 节点默认值 | 可视化 launch 默认值 | 说明 |
| --- | --- | --- | --- |
| `publish_tcp_paths` | `true` | `true` | 是否发布 TCP path 和 TF |
| `tcp_parent_body_name` | `Link7_L` | `Link7_L` | TCP 依附的 MuJoCo body |
| `tcp_frame_id` | `tcp_link` | `tcp_link` | TF child frame |
| `tcp_offset_xyz` | `[0.0, 0.0, 0.0]` | `[0.0, -0.1, 0.0]` | TCP 在 parent body 局部坐标系下的位置偏移 |
| `tcp_offset_rpy` | `[0.0, 0.0, 0.0]` | `[0.0, 0.0, 0.0]` | TCP 在 parent body 局部坐标系下的姿态偏移 |
| `max_tcp_actual_path_points` | `1000` | `1000` | 实际轨迹最大保存点数 |
| `tcp_predicted_path_sample_count` | `120` | `120` | 预测轨迹采样点数 |

TCP 位姿计算：

```text
tcp_pose = parent_body_pose * local_tcp_offset
```

实现细节：

- `offset_xyz` 在 parent body 局部坐标系下定义。
- `offset_xyz` 会先按 parent body quaternion 旋转到世界坐标，再叠加到 parent body 世界位置。
- `offset_rpy` 会转成 quaternion 后与 parent body quaternion 相乘。
- MuJoCo 内部 quaternion 顺序是 `(w, x, y, z)`。
- ROS message 写入 orientation 时字段仍是 `x/y/z/w` 属性，但值来源保持正确。

发布内容：

```text
TF: Base_L -> tcp_link
/tcp/actual_path
/tcp/predicted_path
```

实际轨迹：

- 控制 tick 每次执行 command 后读取当前 TCP pose。
- 追加到 `/tcp/actual_path`。
- 超过 `max_tcp_actual_path_points` 后裁剪旧点。

预测轨迹：

- 收到新轨迹时生成。
- 使用独立 MuJoCo runtime。
- 对轨迹按时间采样。
- 每个采样点设置预测 runtime 的关节位置。
- 读取 parent body pose 并组合 TCP offset。
- 发布 `/tcp/predicted_path`。

使用独立预测 runtime 的原因：

- 不污染正在执行控制的 MuJoCo runtime。
- 避免预测采样改变当前机器人实际状态。
- 便于未来替换为更复杂的前向运动学或碰撞预测模块。

## 16. RViz 可视化链路

启动文件：

```text
src/cook_bringup/launch/visualization.launch.py
```

RViz 配置：

```text
src/cook_bringup/rviz/cook_visualization.rviz
```

launch 默认启动：

- `robot_state_publisher`
- `cook_mujoco_controller_node`
- `cook_teach_pendant`
- `rviz2`

可选启动：

- `cook_demo_trajectory_publisher`，通过 `publish_demo:=true` 启用。

RViz 主要显示：

- RobotModel。
- TF。
- `tcp_link`。
- `/tcp/actual_path`。
- `/tcp/predicted_path`。

当前不显示：

- `/end_effector/actual_path`
- `/end_effector/predicted_path`

这是为了保证运行轨迹语义统一为 TCP，而不是混用 EEF 和 TCP。

## 17. Launch 参数

`visualization.launch.py` 主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `use_rviz` | `true` | 是否启动 RViz |
| `use_teach_pendant` | `true` | 是否启动 GUI 示教器 |
| `publish_demo` | `false` | 是否启动 demo 轨迹发布器 |
| `demo_planner_type` | `linear` | demo publisher 使用的规划器，支持 `linear` 或 `ompl` |
| `demo_collision_check` | `true` | OMPL demo 是否启用 MuJoCo 碰撞检查 |
| `demo_allow_initial_contacts` | `true` | 是否允许起点已有接触 body pair |
| `demo_planning_time_sec` | `1.0` | OMPL demo 规划时间上限 |
| `demo_collision_check_resolution` | `0.01` | OMPL 状态有效性检查分辨率 |
| `control_rate_hz` | `100.0` | 控制 tick 频率 |
| `publish_rate_hz` | `30.0` | joint state/path 发布频率 |
| `publish_tcp_paths` | `true` | 是否发布 TCP path |
| `tcp_parent_body_name` | `Link7_L` | TCP 依附的 MuJoCo body |
| `tcp_frame_id` | `tcp_link` | TCP TF 名称 |
| `tcp_offset_xyz` | `[0.0, -0.1, 0.0]` | launch 默认 TCP 位置偏移 |
| `tcp_offset_rpy` | `[0.0, 0.0, 0.0]` | launch 默认 TCP 姿态偏移 |
| `max_tcp_actual_path_points` | `1000` | 实际 path 点数上限 |
| `tcp_predicted_path_sample_count` | `120` | 预测 path 采样数 |

示例：

```bash
ros2 launch cook_bringup visualization.launch.py \
  tcp_offset_xyz:="[0.0, -0.12, 0.03]" \
  tcp_offset_rpy:="[0.0, 0.0, 1.5708]"
```

无 GUI/headless 示例：

```bash
ros2 launch cook_bringup visualization.launch.py \
  use_rviz:=false \
  use_teach_pendant:=false
```

## 18. 启动流程

标准启动：

```bash
conda activate gmp
source /opt/ros/humble/setup.bash
colcon build --packages-select cook_core cook_description cook_mujoco cook_bringup
source install/setup.bash
ros2 launch cook_bringup visualization.launch.py
```

启动后发生：

1. launch 通过 `ament_index` 找到 package share 目录。
2. `RobotAssetContext` 定位 URDF、MJCF 和 RViz 配置。
3. `robot_state_publisher` 使用 URDF 发布机器人 TF。
4. `mujoco_controller_node` 加载 MJCF 和 URDF 元数据。
5. `teach_pendant` 读取 URDF joint 信息并打开 GUI。
6. RViz 加载显示配置。
7. controller node 发布 `/joint_states`。
8. controller node 发布 TCP TF 和 path。
9. GUI 或外部任务脚本发布轨迹。
10. MuJoCo controller 执行轨迹并刷新可视化。

## 19. Demo 与任务脚本

完整安装与“规划 -> 执行”操作指南见 `docs/PLANNING_USAGE.md`。

### 本地 MuJoCo demo

```bash
cook_model_convert --overwrite --validate
cook_mujoco_demo demo --headless --steps 120
```

用途：

- 验证 MJCF 可以加载。
- 验证 controller 可以执行简单轨迹。
- 不依赖 ROS launch 和 RViz。

### ROS demo publisher

```bash
ros2 launch cook_bringup visualization.launch.py publish_demo:=true
```

用途：

- 自动发布一条线性关节轨迹。
- 验证 ROS trajectory topic 到 MuJoCo controller 的链路。

OMPL demo：

```bash
pip install ompl
ros2 launch cook_bringup visualization.launch.py \
  publish_demo:=true \
  demo_planner_type:=ompl
```

用途：

- 使用 OMPL 生成关节空间轨迹。
- 使用 MuJoCo contact 做碰撞状态有效性检查。
- 保持输出仍为标准 `JointTrajectoryData` / ROS `JointTrajectory`。

### 函数式控制 demo

```bash
python examples/function_control_demo.py
```

dry-run：

```bash
python examples/function_control_demo.py --backend fake
```

用途：

- 展示独立任务脚本如何通过 `cook_core.robot` 控制机器人。
- 验证任务流程不直接依赖 ROS message。
- 为未来技能脚本、任务流水线和真机后端提供形态参考。

### 规划控制 demo

```bash
python examples/planning_control_demo.py --planner linear --backend fake
```

OMPL dry-run：

```bash
pip install ompl
python examples/planning_control_demo.py --planner ompl --backend fake
```

用途：

- 展示规划器输出如何通过统一任务控制端口发送。
- 验证任务脚本不直接依赖 OMPL、ROS message 或 MuJoCo API。

## 20. 退出与生命周期

安全退出工具位于 `src/cook_bringup/cook_bringup/ros/shutdown.py`。

目标：

- Ctrl-C 后不重复调用已关闭的 `rclpy.shutdown()`。
- 节点销毁失败时不影响整体退出。
- GUI 进程收到 shutdown 后能够退出主循环。
- launch 退出时减少 traceback 和僵住进程。

控制节点主函数流程：

```text
rclpy.init()
create node
rclpy.spin(node)
catch KeyboardInterrupt / ExternalShutdownException
safe_destroy_node(node)
safe_shutdown()
```

GUI 节点应同样遵守：

- ROS spin 和 Tk mainloop 都要有退出路径。
- 不应在 SIGINT 后阻塞 launch。
- close/destroy/shutdown 应允许重复调用。

## 21. 错误处理与安全边界

当前已经具备的校验：

- 关节名不能为空。
- 关节名不能重复。
- 轨迹点必须包含所有关节。
- 轨迹点不能包含未知关节。
- 轨迹时间必须单调。
- duration 必须非负或正数，取决于接口语义。
- 关节目标必须是有限数。
- MuJoCo 模型后缀必须是 `.xml` 或 `.mjcf`。
- 运行时拒绝直接加载 `.urdf`。
- MuJoCo joint 必须存在。
- 当前控制 joint 必须是一自由度 joint。
- 控制目标会按 URDF joint limit clamp。
- TCP offset 必须是 3 维向量。
- TCP path 失败时会禁用 path 发布并输出 warning，避免控制节点持续抛异常。
- OMPL 是可选依赖，未安装时只影响 `OmplJointPlanner`。
- MuJoCo 碰撞检查支持按 body pair 过滤起点已有接触。

当前尚未覆盖的工业级能力：

- 动力学力矩控制。
- 速度/加速度/jerk 限制。
- 实时控制优先级。
- 急停和安全区域。
- 真机通信异常恢复。
- 多机器人命名空间隔离。
- 复杂任务编排和恢复策略。

## 22. 扩展指南

### 新增机器人模型

推荐步骤：

1. 将 URDF、mesh、MJCF 放入 `cook_description` 资产目录。
2. 确保 URDF 中 movable joint 名称稳定、语义明确。
3. 确保 MJCF 中 joint 名称与 URDF movable joints 对齐。
4. 确认 TCP parent body 名称存在于 MJCF。
5. 使用 `cook_model_convert --overwrite --validate` 做离线转换和校验。
6. 启动 RViz 检查 RobotModel 与 MuJoCo joint state 是否一致。

### 新增规划器

推荐方式：

1. 新建 planner adapter。
2. 实现 `Planner.plan(request: PlanRequest) -> PlanResult`。
3. 内部转换到算法库需要的状态空间和约束。
4. 输出统一 `JointTrajectoryData`。
5. 将 factory 注册到 `PlannerRegistry`；需要 MuJoCo 资产的算法在 `create_mujoco_planner_registry()` 中注册。
6. 添加 planner 单元测试。

如果规划器需要碰撞检测，优先复用 `MujocoCollisionChecker` 或实现同等的 `is_state_valid()` 抽象，不要让控制节点直接承担规划职责。

禁止做法：

- 让任务脚本直接依赖 OMPL 类型。
- 让 MuJoCo controller 直接调用规划库。
- 让 ROS message 成为规划器输入输出主类型。

### 新增真机后端

推荐方式：

1. 实现新的 `RobotCommandPort`。
2. 复用 `RobotCommandBuilder` 对齐关节顺序和轨迹格式。
3. 在后端内部完成协议转换、网络通信和硬件状态同步。
4. 任务脚本继续调用 `move_joints()`、`move_joint_sequence()`、`hold_current()`。
5. 增加 fake/mock 测试，避免所有测试依赖真实硬件。

### 新增控制模式

推荐方式：

1. 先在 `cook_core` 明确通用控制语义。
2. 再在 MuJoCo 或硬件后端做具体实现。
3. 如需 ROS topic，放在 `cook_bringup` adapter 中。
4. 保持任务 API 不直接暴露后端细节。

## 23. 验证命令

常用验证：

```bash
conda run -n gmp pytest -q
```

ROS2 环境验证：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select cook_core cook_description cook_mujoco cook_bringup
source install/setup.bash
pytest -q
ros2 launch cook_bringup visualization.launch.py --show-args
```

headless smoke：

```bash
ros2 launch cook_bringup visualization.launch.py \
  use_rviz:=false \
  use_teach_pendant:=false
```

任务脚本 dry-run：

```bash
python examples/function_control_demo.py --backend fake
```

规划 demo：

```bash
python examples/planning_control_demo.py --planner linear --backend fake
```

OMPL 可用时额外验证：

```bash
python examples/planning_control_demo.py --planner ompl --backend fake
```

## 24. 当前架构边界

当前项目已经具备：

- ROS2 package 化目录结构。
- MuJoCo 控制后端。
- RViz 可视化。
- GUI 示教器。
- TCP 实际轨迹和预测轨迹显示。
- 后端无关的函数式任务控制 API。
- 可替换规划器接口和可选 OMPL 碰撞感知规划器。
- 基础单元测试和 ROS2 集成测试。
- 安全退出处理。

当前仍需持续完善：

- 更完整的控制器策略。
- 更严格的速度/加速度限制。
- 真机后端。
- 更完整的碰撞场景和路径规划约束。
- 多模型配置管理。
- 多机器人命名空间。
- 更完整的任务编排层。
- 更系统的错误恢复和安全策略。
