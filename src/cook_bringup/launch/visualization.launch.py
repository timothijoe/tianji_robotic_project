from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from cook_description.assets import RobotAssetContext


def generate_launch_description() -> LaunchDescription:
    bringup_package = "cook_bringup"
    description_package = "cook_description"
    bringup_share_dir = Path(get_package_share_directory(bringup_package))
    description_share_dir = Path(get_package_share_directory(description_package))
    assets = RobotAssetContext.from_share_path(
        description_share_dir,
        package_name=description_package,
    )
    model_path = assets.mjcf_path
    urdf_path = assets.urdf_path
    rviz_path = bringup_share_dir / "rviz" / "cook_visualization.rviz"
    robot_description = assets.robot_description_for_ros()

    use_rviz = LaunchConfiguration("use_rviz")
    use_teach_pendant = LaunchConfiguration("use_teach_pendant")
    publish_demo = LaunchConfiguration("publish_demo")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_teach_pendant", default_value="true"),
            DeclareLaunchArgument("publish_demo", default_value="false"),
            DeclareLaunchArgument("demo_planner_type", default_value="linear"),
            DeclareLaunchArgument("demo_collision_check", default_value="true"),
            DeclareLaunchArgument("demo_allow_initial_contacts", default_value="true"),
            DeclareLaunchArgument("demo_planning_time_sec", default_value="1.0"),
            DeclareLaunchArgument("demo_collision_check_resolution", default_value="0.01"),
            DeclareLaunchArgument("control_rate_hz", default_value="100.0"),
            DeclareLaunchArgument("publish_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("publish_tcp_paths", default_value="true"),
            DeclareLaunchArgument("tcp_parent_body_name", default_value="Link7_L"),
            DeclareLaunchArgument("tcp_frame_id", default_value="tcp_link"),
            DeclareLaunchArgument("tcp_offset_xyz", default_value="[0.0, -0.1, 0.0]"),
            DeclareLaunchArgument("tcp_offset_rpy", default_value="[0.0, 0.0, 0.0]"),
            DeclareLaunchArgument("max_tcp_actual_path_points", default_value="1000"),
            DeclareLaunchArgument("tcp_predicted_path_sample_count", default_value="120"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package=bringup_package,
                executable="cook_mujoco_controller_node",
                name="mujoco_controller_node",
                parameters=[
                    {
                        "model_path": str(model_path),
                        "urdf_path": str(urdf_path),
                        "control_rate_hz": LaunchConfiguration("control_rate_hz"),
                        "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                        "publish_tcp_paths": LaunchConfiguration(
                            "publish_tcp_paths"
                        ),
                        "tcp_parent_body_name": LaunchConfiguration(
                            "tcp_parent_body_name"
                        ),
                        "tcp_frame_id": LaunchConfiguration("tcp_frame_id"),
                        "tcp_offset_xyz": LaunchConfiguration("tcp_offset_xyz"),
                        "tcp_offset_rpy": LaunchConfiguration("tcp_offset_rpy"),
                        "max_tcp_actual_path_points": LaunchConfiguration(
                            "max_tcp_actual_path_points"
                        ),
                        "tcp_predicted_path_sample_count": LaunchConfiguration(
                            "tcp_predicted_path_sample_count"
                        ),
                    }
                ],
                output="screen",
            ),
            Node(
                package=bringup_package,
                executable="cook_demo_trajectory_publisher",
                name="demo_trajectory_publisher",
                parameters=[
                    {
                        "model_path": str(model_path),
                        "urdf_path": str(urdf_path),
                        "planner_type": LaunchConfiguration("demo_planner_type"),
                        "collision_check": LaunchConfiguration("demo_collision_check"),
                        "allow_initial_contacts": LaunchConfiguration(
                            "demo_allow_initial_contacts"
                        ),
                        "planning_time_sec": LaunchConfiguration(
                            "demo_planning_time_sec"
                        ),
                        "collision_check_resolution": LaunchConfiguration(
                            "demo_collision_check_resolution"
                        ),
                        "publish_once": True,
                    }
                ],
                condition=IfCondition(publish_demo),
                output="screen",
            ),
            Node(
                package=bringup_package,
                executable="cook_teach_pendant",
                name="teach_pendant",
                parameters=[{"urdf_path": str(urdf_path)}],
                condition=IfCondition(use_teach_pendant),
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", str(rviz_path)],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
