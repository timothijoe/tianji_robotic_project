import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("nav_msgs.msg")
pytest.importorskip("tf2_ros")

from rclpy.context import Context

from cook_bringup.ros.conversions import trajectory_data_to_msg
from cook_bringup.ros.mujoco_controller_node import MujocoControllerNode
from cook_core.interfaces import JointTrajectoryData, TrajectoryPointData


@pytest.fixture
def ros_context():
    context = Context()
    context.init(initialize_logging=False)
    try:
        yield context
    finally:
        context.shutdown()


def test_mujoco_controller_node_publishes_tcp_paths_and_tf(ros_context):
    node = MujocoControllerNode(context=ros_context)
    predicted_pub = _Publisher()
    actual_pub = _Publisher()
    tf_broadcaster = _TransformBroadcaster()
    try:
        node.tcp_predicted_path_pub = predicted_pub
        node.tcp_actual_path_pub = actual_pub
        node.tcp_transform_broadcaster = tf_broadcaster
        trajectory = _trajectory_for(node.controller.joint_names)

        node._trajectory_callback(trajectory_data_to_msg(trajectory, node=node))

        assert len(predicted_pub.messages) == 1
        assert len(predicted_pub.messages[0].poses) == node.tcp_predicted_path_sample_count
        assert predicted_pub.messages[0].header.frame_id == "Base_L"
        assert len(node.tcp_actual_path.poses) == 0

        node._control_tick()
        node._publish_joint_state()

        assert len(node.tcp_actual_path.poses) == 1
        assert len(actual_pub.messages) == 1
        assert len(actual_pub.messages[0].poses) == 1
        assert len(predicted_pub.messages) == 2
        assert len(tf_broadcaster.transforms) == 1
        assert tf_broadcaster.transforms[0].child_frame_id == "tcp_link"

        node._trajectory_callback(trajectory_data_to_msg(trajectory, node=node))

        assert len(node.tcp_actual_path.poses) == 0
        assert len(predicted_pub.messages) == 3
        assert not hasattr(node, "actual_path_pub")
        assert not hasattr(node, "predicted_path_pub")
    finally:
        node.destroy_node()


def _trajectory_for(joint_names: tuple[str, ...]) -> JointTrajectoryData:
    start = {name: 0.0 for name in joint_names}
    goal = {name: 0.1 for name in joint_names}
    return JointTrajectoryData(
        joint_names=joint_names,
        points=(
            TrajectoryPointData(start, 0.0),
            TrajectoryPointData(goal, 0.2),
        ),
    )


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _TransformBroadcaster:
    def __init__(self):
        self.transforms = []

    def sendTransform(self, transform):
        self.transforms.append(transform)
