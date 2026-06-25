from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from trajectory_msgs.msg import JointTrajectory

from cook_mujoco.control import MujocoPositionController, MujocoRuntime
from cook_core.interfaces import JointStateData, JointTrajectoryData
from cook_description.models import load_robot_definition
from cook_bringup.ros.conversions import (
    joint_state_data_to_msg,
    trajectory_data_from_msg,
)
from cook_bringup.ros.tcp_path import (
    append_body_pose_to_path,
    body_pose_to_transform_stamped,
    build_predicted_tcp_path,
    compose_tcp_pose,
)
from cook_bringup.ros.paths import default_model_path, default_urdf_path
from cook_bringup.ros.shutdown import (
    ignore_shutdown_signals,
    safe_destroy_node,
    safe_shutdown,
)
from cook_core.trajectory import TrajectoryExecutor


class MujocoControllerNode(Node):
    def __init__(self, *, context=None) -> None:
        super().__init__("mujoco_controller_node", context=context)
        self.declare_parameter("model_path", str(default_model_path()))
        self.declare_parameter("urdf_path", str(default_urdf_path()))
        self.declare_parameter("control_rate_hz", 100.0)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("publish_tcp_paths", True)
        self.declare_parameter("tcp_parent_body_name", "Link7_L")
        self.declare_parameter("tcp_frame_id", "tcp_link")
        self.declare_parameter("tcp_offset_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("tcp_offset_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("max_tcp_actual_path_points", 1000)
        self.declare_parameter("tcp_predicted_path_sample_count", 120)

        model_path = str(_parameter_value(self, "model_path"))
        urdf_path = str(_parameter_value(self, "urdf_path"))
        self.publish_tcp_paths = _bool_parameter(
            self,
            "publish_tcp_paths",
        )
        self.tcp_parent_body_name = str(
            _parameter_value(self, "tcp_parent_body_name")
        )
        self.tcp_frame_id = str(_parameter_value(self, "tcp_frame_id"))
        self.tcp_offset_xyz = _vector3_parameter(self, "tcp_offset_xyz")
        self.tcp_offset_rpy = _vector3_parameter(self, "tcp_offset_rpy")
        self.path_frame_id = "Base_L"
        self.max_tcp_actual_path_points = max(
            1,
            _int_parameter(self, "max_tcp_actual_path_points"),
        )
        self.tcp_predicted_path_sample_count = max(
            2,
            _int_parameter(self, "tcp_predicted_path_sample_count"),
        )

        self.definition = load_robot_definition(urdf_path)
        runtime = MujocoRuntime.load(model_path, self.definition.movable_joint_names)
        self.controller = MujocoPositionController(
            runtime,
            joint_limits=self.definition.joint_limits,
        )
        self.current_state = JointStateData(
            names=self.controller.joint_names,
            positions=self.controller.reset().positions,
        )
        self.trajectory_executor = TrajectoryExecutor(
            expected_joint_names=self.controller.joint_names
        )
        self.prediction_runtime = (
            MujocoRuntime.load(model_path, self.definition.movable_joint_names)
            if self.publish_tcp_paths
            else None
        )

        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.tcp_actual_path_pub = None
        self.tcp_predicted_path_pub = None
        self.tcp_transform_broadcaster = None
        self.tcp_actual_path = Path()
        self.tcp_actual_path.header.frame_id = self.path_frame_id
        self.tcp_predicted_path = Path()
        self.tcp_predicted_path.header.frame_id = self.path_frame_id
        if self.publish_tcp_paths:
            self.tcp_actual_path_pub = self.create_publisher(
                Path,
                "/tcp/actual_path",
                10,
            )
            self.tcp_predicted_path_pub = self.create_publisher(
                Path,
                "/tcp/predicted_path",
                10,
            )
            self.tcp_transform_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            JointTrajectory,
            "~/joint_trajectory",
            self._trajectory_callback,
            10,
        )
        control_period = 1.0 / max(
            1e-6,
            _float_parameter(self, "control_rate_hz"),
        )
        publish_period = 1.0 / max(
            1e-6,
            _float_parameter(self, "publish_rate_hz"),
        )
        self.create_timer(control_period, self._control_tick)
        self.create_timer(publish_period, self._publish_joint_state)
        self.get_logger().info(
            "MuJoCo controller ready: "
            f"joints={len(self.controller.joint_names)}, model={model_path}"
        )

    def _trajectory_callback(self, message: JointTrajectory) -> None:
        trajectory = trajectory_data_from_msg(message)
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if not self.trajectory_executor.start(trajectory, now_sec=now_sec):
            incoming = ", ".join(trajectory.joint_names)
            expected = ", ".join(self.controller.joint_names)
            self.get_logger().warning(
                "ignore invalid trajectory: "
                f"incoming=[{incoming}], expected=[{expected}], empty={trajectory.is_empty}"
            )
            return
        self._reset_tcp_actual_path()
        self._publish_tcp_predicted_path(trajectory)

    def _control_tick(self) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        command = self.trajectory_executor.command_at(now_sec=now_sec)
        if command is None:
            return
        state = self.controller.apply_command(command)
        self.current_state = state.joints
        self._append_tcp_actual_path_point()

    def _publish_joint_state(self) -> None:
        self.joint_state_pub.publish(joint_state_data_to_msg(self.current_state, node=self))
        if self.publish_tcp_paths and self.tcp_actual_path_pub is not None:
            self.tcp_actual_path_pub.publish(self.tcp_actual_path)
        if (
            self.publish_tcp_paths
            and self.tcp_predicted_path_pub is not None
            and self.tcp_predicted_path.poses
        ):
            self.tcp_predicted_path_pub.publish(self.tcp_predicted_path)

    def _reset_tcp_actual_path(self) -> None:
        if not self.publish_tcp_paths:
            return
        self.tcp_actual_path = Path()
        self.tcp_actual_path.header.frame_id = self.path_frame_id

    def _append_tcp_actual_path_point(self) -> None:
        if not self.publish_tcp_paths:
            return
        stamp = self.get_clock().now().to_msg()
        try:
            tcp_pose = self._current_tcp_pose()
            append_body_pose_to_path(
                self.tcp_actual_path,
                tcp_pose,
                frame_id=self.path_frame_id,
                stamp=stamp,
                max_points=self.max_tcp_actual_path_points,
            )
            self._broadcast_tcp_transform(tcp_pose, stamp=stamp)
        except Exception as exc:
            self.publish_tcp_paths = False
            self.get_logger().warning(
                f"disable TCP path publishing: {exc}"
            )

    def _publish_tcp_predicted_path(self, trajectory: JointTrajectoryData) -> None:
        if (
            not self.publish_tcp_paths
            or self.tcp_predicted_path_pub is None
            or self.prediction_runtime is None
        ):
            return
        try:
            self.tcp_predicted_path = build_predicted_tcp_path(
                runtime=self.prediction_runtime,
                trajectory=trajectory,
                parent_body_name=self.tcp_parent_body_name,
                frame_id=self.path_frame_id,
                offset_xyz=self.tcp_offset_xyz,
                offset_rpy=self.tcp_offset_rpy,
                sample_count=self.tcp_predicted_path_sample_count,
                stamp=self.get_clock().now().to_msg(),
            )
        except Exception as exc:
            self.get_logger().warning(
                f"failed to publish predicted TCP path: {exc}"
            )
            return
        self.tcp_predicted_path_pub.publish(self.tcp_predicted_path)

    def _current_tcp_pose(self):
        return compose_tcp_pose(
            self.controller.runtime.get_body_pose(self.tcp_parent_body_name),
            offset_xyz=self.tcp_offset_xyz,
            offset_rpy=self.tcp_offset_rpy,
        )

    def _broadcast_tcp_transform(self, tcp_pose, *, stamp) -> None:
        if self.tcp_transform_broadcaster is None:
            return
        self.tcp_transform_broadcaster.sendTransform(
            body_pose_to_transform_stamped(
                tcp_pose,
                parent_frame_id=self.path_frame_id,
                child_frame_id=self.tcp_frame_id,
                stamp=stamp,
            )
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


def _vector3_parameter(node: Node, name: str) -> tuple[float, float, float]:
    value = _parameter_value(node, name)
    if isinstance(value, str):
        text = value.strip().strip("[]")
        values = [item.strip() for item in text.split(",") if item.strip()]
    else:
        values = list(value)
    if len(values) != 3:
        raise ValueError(f"{name} must contain exactly 3 values")
    return (float(values[0]), float(values[1]), float(values[2]))


def main(argv: list[str] | None = None) -> int:
    node = None
    rclpy.init(args=argv)
    try:
        node = MujocoControllerNode()
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
