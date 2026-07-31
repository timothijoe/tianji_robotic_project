import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from twin_sim.hand import DEFAULT_OPEN_RAD
from twin_sim.ros2_bridge import LEFT_HAND_JOINT_NAMES


class WujiSimDemo(Node):
    def __init__(self) -> None:
        super().__init__("wuji_sim_demo")
        self._publisher = self.create_publisher(
            JointState, "joint_commands", 10
        )
        self._closed = False
        self.create_timer(2.0, self._publish_pose)
        self._publish_pose()

    def _publish_pose(self) -> None:
        self._closed = not self._closed
        target = DEFAULT_OPEN_RAD.copy()
        if self._closed:
            target[[1, 2, 3, 5, 6, 7, 9, 10, 11, 13, 14, 15, 17, 18, 19]] += 0.25
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(LEFT_HAND_JOINT_NAMES)
        message.position = target.tolist()
        self._publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WujiSimDemo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
