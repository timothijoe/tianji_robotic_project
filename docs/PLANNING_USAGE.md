# 规划到执行使用指南

本文说明当前项目的安装、仿真启动、规划轨迹生成和执行方式。完整链路是：

```text
PlanRequest
  -> Planner
  -> PlanResult / JointTrajectoryData
  -> RobotCommandPort
  -> ROS2 JointTrajectory
  -> MuJoCo controller
```

规划输出统一是 `JointTrajectoryData`，执行仍走现有 `RobotCommandPort -> ROS2 JointTrajectory -> MuJoCo controller` 链路。

## 1. 基础安装

```bash
cd /home/alen/projects/cook
conda activate gmp
source /opt/ros/humble/setup.bash

colcon build --packages-select cook_core cook_description cook_mujoco cook_bringup
source install/setup.bash
```

验证基础链路：

```bash
cook_model_convert --overwrite --validate
cook_mujoco_demo demo --headless --steps 120
```

## 2. OMPL 安装

OMPL 是可选依赖。未安装 OMPL 时，线性规划、MuJoCo 控制、RViz 可视化仍可用。

```bash
pip install ompl
```

如果不安装 OMPL，可以先用内置线性规划器：

```bash
python examples/planning_control_demo.py --planner linear --backend fake
```

## 3. 启动仿真与可视化

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch cook_bringup visualization.launch.py
```

无 GUI/RViz 的 headless 方式：

```bash
ROS_LOG_DIR=/tmp/ros_log ros2 launch cook_bringup visualization.launch.py \
  use_rviz:=false \
  use_teach_pendant:=false
```

## 4. 规划到执行 Demo

先启动 `visualization.launch.py`，然后另开终端：

```bash
cd /home/alen/projects/cook
conda activate gmp
source /opt/ros/humble/setup.bash
source install/setup.bash

python examples/planning_control_demo.py --planner linear --backend ros2
```

使用 OMPL：

```bash
python examples/planning_control_demo.py --planner ompl --backend ros2
```

dry-run 不连接 ROS：

```bash
python examples/planning_control_demo.py --planner linear --backend fake
python examples/planning_control_demo.py --planner ompl --backend fake
```

## 5. 代码调用形态

```python
from cook_core.planning import PlanRequest
from cook_core.robot import create_robot_command_port
from cook_description.assets import RobotAssetContext
from cook_description.models import load_robot_definition
from cook_mujoco.planning import create_mujoco_ompl_planner

assets = RobotAssetContext.local_default()
definition = load_robot_definition(assets.urdf_path)
joint_names = definition.movable_joint_names

request = PlanRequest.from_position_mappings(
    start_positions={name: 0.0 for name in joint_names},
    goal_positions={name: 0.1 for name in joint_names},
    joint_names=joint_names,
    duration_sec=3.0,
    waypoint_count=80,
    planning_time_sec=1.0,
)

planner = create_mujoco_ompl_planner(
    model_path=assets.mjcf_path,
    urdf_path=assets.urdf_path,
)

result = planner.plan(request)
if not result.success:
    raise RuntimeError(result.message)

robot = create_robot_command_port(backend="ros2")
try:
    robot.move_joints(result.trajectory)
finally:
    robot.close()
```

## 6. 使用边界

- `--planner linear` 不依赖 OMPL，适合验证任务和控制链路。
- `--planner ompl` 需要安装 OMPL Python bindings。
- OMPL planner 使用 MuJoCo contact 做碰撞状态有效性检查。
- 当前默认模型零位已有接触，规划器默认允许起点已有 body pair。
- 任务脚本不直接依赖 ROS message、OMPL 类型或 MuJoCo API。

## 7. 常见问题

### 安装态找不到 URDF/MJCF

如果看到类似 `site-packages/assets/robot/urdf/test1.urdf` 不存在，说明当前 shell 使用了旧的 install 产物。重新构建并 source：

```bash
colcon build --packages-select cook_description cook_mujoco cook_bringup
source install/setup.bash
```

资产路径应解析到：

```text
install/cook_description/share/cook_description/assets/robot/
```

### OMPL binding 接口差异

不同 OMPL Python binding 暴露的 API 不完全一致。当前 planner 已兼容：

- `StateValidityCheckerFn` 函数式 checker。
- 仅有 `StateValidityChecker` 的子类 checker。
- `ob.State(space)` 和 `space.allocState()` 两种 state 创建方式。

如果更新 OMPL 后再次出现 binding API 错误，优先运行：

```bash
python examples/planning_control_demo.py --planner ompl --backend fake
```

先确认规划器本身可用，再切换到 `--backend ros2`。

### ROS 日志目录不可写

如果 ROS 尝试写入 `~/.ros/log` 失败，可临时指定日志目录：

```bash
ROS_LOG_DIR=/tmp/ros_log python examples/planning_control_demo.py --planner ompl --backend ros2
```
