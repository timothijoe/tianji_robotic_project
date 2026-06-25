# Robot Visualization

基于 MuJoCo、ROS 2 和 RViz 的七自由度机械臂仿真项目，支持模型转换、轨迹规划、位置控制、笛卡尔阻抗、末端力控和可视化。

当前包含两条主要控制链：

- 关节轨迹规划与 ROS 2/RViz 可视化；
- MuJoCo 力矩驱动、腕部六维力传感和重复剁切。

## 环境要求

当前验证环境：

- Ubuntu
- ROS 2 Jazzy：`/opt/ros/jazzy`
- Python 3.12
- MuJoCo 3.9

确认系统 Python 能同时导入 ROS 和 MuJoCo：

```bash
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 -c "import rclpy, mujoco; print(mujoco.__version__)"
```

如果 MuJoCo 尚未安装：

```bash
sudo apt-get install -y python3-pip
/usr/bin/python3 -m pip install --user --break-system-packages mujoco
```

## 构建

```bash
cd ~/catkin_robotic_ws/cook_proj/cook_ws
source /opt/ros/jazzy/setup.bash

colcon build \
  --packages-select cook_core cook_description cook_mujoco cook_bringup \
  --symlink-install

source install/setup.bash
```

本机已经配置 `sc` alias。在项目根目录可以用它代替最后一条命令：

```bash
sc
```

## 模型转换

将默认 URDF 转换为 MJCF/XML，并使用 MuJoCo 验证：

```bash
ros2 run cook_description cook_model_convert --overwrite --validate
```

默认输出：

```text
src/cook_description/assets/robot/mujoco/robot.xml
```

## MuJoCo Demo

打开 MuJoCo Viewer 并播放规划轨迹：

```bash
ros2 run cook_mujoco cook_mujoco_demo demo
```

无界面验证：

```bash
ros2 run cook_mujoco cook_mujoco_demo demo --headless --steps 120
```

`--headless` 不会打开窗口，只验证模型、规划器和控制器能否运行。

## 力控与剁切 Demo

静态保持 `10 N`：

```bash
ros2 run cook_mujoco cook_force_control_demo \
  --mode static --force 10 --hold 5 \
  --viewer --realtime --status-hz 10 \
  --log static_force.csv
```

执行五次剁切：

```bash
ros2 run cook_mujoco cook_force_control_demo \
  --mode chopping --cycles 5 --force 10 --hold 0.5 \
  --viewer --realtime --status-hz 10 \
  --log chopping.csv
```

终端会显示六维力和控制模式：

- `CARTESIAN_IMPEDANCE`：笛卡尔阻抗轨迹跟踪；
- `HYBRID_FORCE_Z`：X/Y 与姿态保持阻抗控制，工具 Z 轴切换为力控制。

详细说明见 [`FORCE_CONTROL_README.md`](FORCE_CONTROL_README.md)。

逐周期调试链路见 [`docs/FORCE_CONTROL_DEBUG.md`](docs/FORCE_CONTROL_DEBUG.md)。

## ROS 2 与 RViz

启动 RViz、MuJoCo 控制器、`robot_state_publisher` 和示教器：

```bash
ros2 launch cook_bringup visualization.launch.py
```

如果不需要示教器：

```bash
ros2 launch cook_bringup visualization.launch.py \
  use_teach_pendant:=false
```

主要话题：

- `/joint_states`：MuJoCo 控制器发布的当前关节状态。
- `/mujoco_controller_node/joint_trajectory`：控制器接收的关节轨迹。
- `/tcp/actual_path`：实际执行的 TCP 路径。
- `/tcp/predicted_path`：收到轨迹时计算的预测 TCP 路径。
- `/tf`、`/tf_static`：RViz 使用的坐标变换。

RViz 中黄色路径表示预测 TCP 路径，蓝色路径表示实际 TCP 路径。RobotModel 根据 `/joint_states` 更新姿态。

## Function Control Demo

终端 1：

```bash
cd ~/catkin_robotic_ws/cook_proj/cook_ws
sc
ros2 launch cook_bringup visualization.launch.py
```

等终端出现 `MuJoCo controller ready` 后，在终端 2 运行：

```bash
cd ~/catkin_robotic_ws/cook_proj/cook_ws
sc
/usr/bin/python3 examples/function_control_demo.py
```

该示例依次发送单组关节目标、多路点轨迹和保持当前位置命令。

不连接 ROS 的 dry-run：

```bash
/usr/bin/python3 examples/function_control_demo.py --backend fake
```

## Planning Demo

### 内置线性规划器

保持终端 1 中 RViz 和控制器运行，在终端 2 执行：

```bash
cd ~/catkin_robotic_ws/cook_proj/cook_ws
sc
/usr/bin/python3 examples/planning_control_demo.py \
  --planner linear \
  --backend ros2
```

只验证规划结果，不发送到 ROS：

```bash
/usr/bin/python3 examples/planning_control_demo.py \
  --planner linear \
  --backend fake
```

### OMPL 碰撞感知规划器

OMPL 是可选依赖，需安装到 ROS Jazzy 使用的系统 Python：

```bash
/usr/bin/python3 -m pip install --user --break-system-packages ompl
# 通用 Python 环境可使用：pip install ompl
```

验证安装：

```bash
/usr/bin/python3 -c "import ompl; print(ompl.__file__)"
```

推荐使用两个终端运行，以确保 RViz 和控制器已经就绪。

终端 1：

```bash
cd ~/catkin_robotic_ws/cook_proj/cook_ws
sc
ros2 launch cook_bringup visualization.launch.py
```

看到 `MuJoCo controller ready` 后，在终端 2 执行：

```bash
cd ~/catkin_robotic_ws/cook_proj/cook_ws
sc
/usr/bin/python3 examples/planning_control_demo.py \
  --planner ompl \
  --backend ros2
```

正常情况下，终端会显示 `RRTConnect` 找到解。在 RViz 中：

- 黄色路径：OMPL 规划得到的预测 TCP 路径。
- 蓝色路径：MuJoCo 控制器实际执行的 TCP 路径。

控制器会持续发布最后一条预测路径，因此 RViz 即使加载较慢，也应能显示黄色路径。

不连接 ROS 的 OMPL dry-run：

```bash
/usr/bin/python3 examples/planning_control_demo.py \
  --planner ompl \
  --backend fake
```

也可以由 launch 自动发布一次规划轨迹：

```bash
ros2 launch cook_bringup visualization.launch.py \
  publish_demo:=true \
  demo_planner_type:=ompl
```

自动 demo 可能在 RViz 完成加载前开始。需要观察完整动画时，推荐先启动 RViz，再在第二个终端运行规划脚本。

## 常用检查命令

```bash
# 确认工作区已加载
ros2 pkg prefix cook_bringup

# 查看节点
ros2 node list

# 检查轨迹话题的发布和订阅关系
ros2 topic info -v /mujoco_controller_node/joint_trajectory

# 查看当前关节状态
ros2 topic echo --once /joint_states

# 查看 TCP 路径
ros2 topic echo --once /tcp/actual_path
ros2 topic echo --once /tcp/predicted_path
```

## 常见问题

### `command not found`

项目入口安装在 ROS 包目录中，应通过 `ros2 run` 调用：

```bash
ros2 run cook_description cook_model_convert --help
ros2 run cook_mujoco cook_mujoco_demo --help
```

不要直接运行裸命令 `cook_model_convert` 或 `cook_mujoco_demo`。

### 找不到 ROS 包

```bash
cd ~/catkin_robotic_ws/cook_proj/cook_ws
sc
```

如果尚未构建，先运行“构建”一节中的 `colcon build`。

### RViz 打开但机器人不运动

只启动 `visualization.launch.py` 不会自动发布轨迹。需要运行 Function Control Demo、Planning Demo，或启动时设置 `publish_demo:=true`。

发送轨迹前确认控制器已输出：

```text
MuJoCo controller ready
```

### OMPL 脚本提示未安装

如果出现：

```text
OMPL Python bindings are required for OmplJointPlanner
```

说明 OMPL 没有安装到 `/usr/bin/python3`。执行：

```bash
/usr/bin/python3 -m pip install --user --break-system-packages ompl
# 通用 Python 环境可使用：pip install ompl
```

不要只把 OMPL 安装到另一个 Conda 环境，否则系统 ROS Python 仍然无法导入。

### RViz 中没有黄色规划路径

先确认路径话题存在：

```bash
ros2 topic echo --once /tcp/predicted_path
```

正常消息应包含约 120 个 `poses`。如果没有消息：

1. 确认控制器已经输出 `MuJoCo controller ready`。
2. 在第二个终端重新运行 Planning Demo。
3. 确认启动参数没有设置 `publish_tcp_paths:=false`。
4. 重新执行 `colcon build --packages-select cook_bringup --symlink-install`，然后 `sc`。

### Conda 与系统 ROS

本项目控制器和 ROS 2 节点推荐使用系统 Python 3.12，因为 ROS Jazzy 的 `rclpy` 由系统包提供。

机器学习节点可以使用独立 Conda 环境，并直接运行：

```bash
conda activate learning
source /opt/ros/jazzy/setup.bash
python learning_node.py
```

Python 脚本不需要通过 `ros2 run` 才能成为 ROS 节点。只要能够导入 `rclpy` 并创建节点，就可以正常使用 topic、service 和 action。

## 项目结构

- `src/cook_core`：通用接口、线性规划器、轨迹执行器和机器人命令端口。
- `src/cook_description`：URDF、STL、MJCF 资产及离线转换工具。
- `src/cook_mujoco`：MuJoCo runtime、位置/阻抗/力控制器、剁切状态机、碰撞检测和 OMPL Adapter。
- `src/cook_bringup`：ROS 2 消息转换、控制节点、launch、RViz 配置和示教器。
- `examples`：函数控制和规划控制示例。
- `tests`：接口、规划、控制、模型和 ROS Adapter 测试。
- `docs/ARCHITECTURE.md`：完整架构说明。
- `FORCE_CONTROL_README.md`：力控、六维传感器和剁切 Demo 快速说明。
- `docs/PLANNING_USAGE.md`：规划到执行的详细说明。

## 完整数据链路

```text
PlanRequest
  -> Planner
  -> JointTrajectoryData
  -> ROS 2 JointTrajectory
  -> MuJoCo controller
  -> JointState
  -> robot_state_publisher
  -> TF / RViz
```

新规划算法应实现 `cook_core.planning.Planner` 并返回统一的 `JointTrajectoryData`，从而复用现有 ROS 2、MuJoCo 和 RViz 链路。
