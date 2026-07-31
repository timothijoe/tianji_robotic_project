from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("viewer", default_value="false"),
            DeclareLaunchArgument("hand_name", default_value="hand_left"),
            DeclareLaunchArgument("publish_rate", default_value="100.0"),
            DeclareLaunchArgument("start_enabled", default_value="true"),
            Node(
                package="twin_wuji_sim",
                executable="bridge",
                namespace=LaunchConfiguration("hand_name"),
                output="screen",
                parameters=[
                    {
                        "viewer": LaunchConfiguration("viewer"),
                        "publish_rate": LaunchConfiguration("publish_rate"),
                        "start_enabled": LaunchConfiguration("start_enabled"),
                    }
                ],
            ),
        ]
    )
