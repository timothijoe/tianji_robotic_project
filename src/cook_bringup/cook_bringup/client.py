from __future__ import annotations

import time
from typing import Mapping, Sequence

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from cook_bringup.ros.conversions import trajectory_data_to_msg
from cook_bringup.ros.paths import default_urdf_path
from cook_bringup.ros.shutdown import safe_destroy_node, safe_shutdown
from cook_core.interfaces import JointStateData, JointTrajectoryData
from cook_core.robot import RobotCommandBuilder
from cook_description.models import load_robot_definition


class RosRobotCommandPort:
    def __init__(
        self,
        *,
        joint_names: Sequence[str] | None = None,
        current_positions: Mapping[str, float] | JointStateData | None = None,
        topic: str = "/mujoco_controller_node/joint_trajectory",
        urdf_path: str | None = None,
        node_name: str = "robot_command_client",
        publish_warmup_sec: float = 0.1,
        connection_timeout_sec: float = 3.0,
        publish_settle_sec: float = 0.1,
        context=None,
    ):
        self._owns_context = context is None and not rclpy.ok()
        if context is None and self._owns_context:
            rclpy.init()
        self.node = Node(str(node_name), context=context)
        self.executor = SingleThreadedExecutor(context=self.node.context)
        self.executor.add_node(self.node)
        self.topic = str(topic)
        if joint_names is None:
            definition = load_robot_definition(urdf_path or default_urdf_path())
            joint_names = definition.movable_joint_names
        self.builder = RobotCommandBuilder(
            joint_names=tuple(joint_names),
            current_positions=current_positions,
        )
        self.publisher = self.node.create_publisher(JointTrajectory, self.topic, 10)
        self.connection_timeout_sec = max(0.0, float(connection_timeout_sec))
        self.publish_settle_sec = max(0.0, float(publish_settle_sec))
        if publish_warmup_sec > 0.0:
            time.sleep(float(publish_warmup_sec))
        self._closed = False

    def wait_for_joint_state(
        self,
        *,
        topic: str = "/joint_states",
        timeout_sec: float = 3.0,
    ) -> JointStateData:
        received = None

        def callback(message: JointState) -> None:
            nonlocal received
            positions = dict(zip(message.name, message.position))
            if all(name in positions for name in self.builder.joint_names):
                received = JointStateData(
                    names=self.builder.joint_names,
                    positions=tuple(
                        float(positions[name]) for name in self.builder.joint_names
                    ),
                )

        subscription = self.node.create_subscription(JointState, topic, callback, 10)
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        try:
            while received is None and time.monotonic() < deadline:
                self.executor.spin_once(timeout_sec=0.05)
        finally:
            self.node.destroy_subscription(subscription)
        if received is None:
            raise RuntimeError(
                f"timed out waiting for joint state on {topic}; "
                "is the MuJoCo controller running?"
            )
        self.builder.update_current_positions(received)
        return received

    def move_joints(
        self,
        target_positions: Mapping[str, float] | JointStateData | JointTrajectoryData,
        *,
        duration_sec: float = 1.0,
    ) -> JointTrajectoryData:
        trajectory = self.builder.build_move_joints(
            target_positions,
            duration_sec=duration_sec,
        )
        self._publish(trajectory)
        return trajectory

    def move_joint_sequence(
        self,
        waypoints: Sequence[Mapping[str, float] | JointStateData],
        *,
        duration_sec: float,
    ) -> JointTrajectoryData:
        trajectory = self.builder.build_joint_sequence(
            waypoints,
            duration_sec=duration_sec,
        )
        self._publish(trajectory)
        return trajectory

    def hold_current(self, *, duration_sec: float = 0.0) -> JointTrajectoryData:
        trajectory = self.builder.build_hold_current(duration_sec=duration_sec)
        self._publish(trajectory)
        return trajectory

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.executor.remove_node(self.node)
        self.executor.shutdown()
        safe_destroy_node(self.node)
        if self._owns_context:
            safe_shutdown()

    def _publish(self, trajectory: JointTrajectoryData) -> None:
        if self._closed:
            raise RuntimeError("robot command client is closed")
        deadline = time.monotonic() + self.connection_timeout_sec
        while (
            self.publisher.get_subscription_count() <= 0
            and time.monotonic() < deadline
        ):
            self.executor.spin_once(timeout_sec=0.05)
        if self.publisher.get_subscription_count() <= 0:
            raise RuntimeError(
                f"no trajectory subscriber found on {self.topic}; "
                "is the MuJoCo controller running?"
            )
        self.publisher.publish(trajectory_data_to_msg(trajectory, node=self.node))
        if self.publish_settle_sec > 0.0:
            self.executor.spin_once(timeout_sec=self.publish_settle_sec)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()
