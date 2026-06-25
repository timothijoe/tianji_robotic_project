# 快速运行

## 1. 第一次构建

```bash
cd ~/catkin_robotic_ws/cook_proj/cook_ws

colcon build \
  --packages-select cook_core cook_description cook_mujoco cook_bringup \
  --symlink-install

sc
```

## 2. 运行 OMPL 规划

终端 1：启动控制器和 RViz。

```bash
cd ~/catkin_robotic_ws/cook_proj/cook_ws
sc
ros2 launch cook_bringup visualization.launch.py
```

等待终端显示：

```text
MuJoCo controller ready
```

终端 2：发送 OMPL 规划轨迹。

```bash
cd ~/catkin_robotic_ws/cook_proj/cook_ws
sc

/usr/bin/python3 examples/planning_control_demo.py \
  --planner ompl \
  --backend ros2
```

RViz 中：

- 黄色：规划路径
- 蓝色：实际执行路径

脚本会读取 `/joint_states` 作为真实规划起点，并在两组目标姿态之间自动切换。因此重复运行该命令时，机械臂应来回运动，而不是停在同一目标。

发送成功时终端会显示：

```text
Trajectory sent to MuJoCo controller.
```

## 3. 运行线性规划

保持终端 1 运行，在终端 2 执行：

```bash
/usr/bin/python3 examples/planning_control_demo.py \
  --planner linear \
  --backend ros2
```

## 4. 打开 MuJoCo Viewer

```bash
cd ~/catkin_robotic_ws/cook_proj/cook_ws
sc
ros2 run cook_mujoco cook_mujoco_demo demo
```

## 5. 模型转换

```bash
cd ~/catkin_robotic_ws/cook_proj/cook_ws
sc
ros2 run cook_description cook_model_convert --overwrite --validate
```

## 常见问题

如果 `sc` 不存在：

```bash
source install/setup.bash
```

如果找不到 ROS 包，重新构建后再执行 `sc`。

如果 RViz 中出现路径闪烁，检查是否启动了重复节点：

```bash
ros2 node list
ros2 topic info /tcp/actual_path
```

`/tcp/actual_path` 的 `Publisher count` 应为 `1`。
