from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

from cook_description.models import load_robot_definition
from cook_core.planning import PlanRequest
from cook_bringup.ros.conversions import trajectory_data_to_msg
from cook_bringup.ros.paths import default_model_path, default_urdf_path
from cook_bringup.ros.shutdown import (
    ignore_shutdown_signals,
    safe_destroy_node,
    safe_shutdown,
)


class DemoTrajectoryPublisher(Node):
    def __init__(self) -> None:
        super().__init__("demo_trajectory_publisher")
        self.declare_parameter("model_path", str(default_model_path()))
        self.declare_parameter("urdf_path", str(default_urdf_path()))
        self.declare_parameter("planner_type", "linear")
        self.declare_parameter("collision_check", True)
        self.declare_parameter("allow_initial_contacts", True)
        self.declare_parameter("planning_time_sec", 1.0)
        self.declare_parameter("collision_check_resolution", 0.01)
        self.declare_parameter("duration_sec", 3.0)
        self.declare_parameter("waypoint_count", 120)
        self.declare_parameter("publish_once", True)
        self.publisher = self.create_publisher(
            JointTrajectory,
            "/mujoco_controller_node/joint_trajectory",
            10,
        )
        self._published = False
        self.create_timer(0.5, self._timer_callback)

    def _timer_callback(self) -> None:
        if self._published and _bool_parameter(self, "publish_once"):
            return
        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        urdf_path = self.get_parameter("urdf_path").get_parameter_value().string_value
        definition = load_robot_definition(urdf_path)
        joint_names = definition.movable_joint_names
        start = {name: 0.0 for name in joint_names}
        offsets = [
            0.2 - (0.4 * index / max(1, len(joint_names) - 1))
            for index in range(len(joint_names))
        ]
        goal = {name: offsets[index] for index, name in enumerate(joint_names)}
        planner_type = (
            self.get_parameter("planner_type")
            .get_parameter_value()
            .string_value
            .strip()
            .lower()
        )
        planner = self._create_planner(
            planner_type=planner_type,
            model_path=model_path,
            urdf_path=urdf_path,
        )
        result = planner.plan(
            PlanRequest.from_position_mappings(
                start_positions=start,
                goal_positions=goal,
                joint_names=joint_names,
                duration_sec=_float_parameter(self, "duration_sec"),
                waypoint_count=_int_parameter(self, "waypoint_count"),
                planning_time_sec=_float_parameter(self, "planning_time_sec"),
                collision_check_resolution=_float_parameter(
                    self,
                    "collision_check_resolution",
                ),
            )
        )
        if not result.success:
            self.get_logger().warning(f"demo planner failed: {result.message}")
            self._published = True
            return
        self.publisher.publish(trajectory_data_to_msg(result.trajectory, node=self))
        self._published = True
        self.get_logger().info(
            "published demo trajectory: "
            f"planner={planner_type}, joints={len(joint_names)}, points={len(result.points)}"
        )

    def _create_planner(self, *, planner_type: str, model_path: str, urdf_path: str):
        from cook_mujoco.planning import create_planner

        return create_planner(
            planner_type,
            model_path=model_path,
            urdf_path=urdf_path,
            collision_check_enabled=_bool_parameter(self, "collision_check"),
            allow_initial_contacts=_bool_parameter(
                self,
                "allow_initial_contacts",
            ),
        )


def _parameter_value(node: Node, name: str):
    return node.get_parameter(name).value


def _float_parameter(node: Node, name: str) -> float:
    return float(_parameter_value(node, name))


def _int_parameter(node: Node, name: str) -> int:
    return int(_parameter_value(node, name))


def _bool_parameter(node: Node, name: str) -> bool:
    value = _parameter_value(node, name)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def main(argv: list[str] | None = None) -> int:
    node = None
    rclpy.init(args=argv)
    try:
        node = DemoTrajectoryPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        ignore_shutdown_signals()
        pass
    finally:
        safe_destroy_node(node)
        safe_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
