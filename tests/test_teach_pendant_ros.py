import pytest

rclpy = pytest.importorskip("rclpy")
trajectory_msgs = pytest.importorskip("trajectory_msgs.msg")
JointTrajectory = trajectory_msgs.JointTrajectory
from rclpy.context import Context

from cook_bringup.ros.teach_pendant_node import TeachPendantNode


@pytest.fixture
def ros_context():
    context = Context()
    context.init(initialize_logging=False)
    try:
        yield context
    finally:
        context.shutdown()


def test_teach_pendant_node_publishes_standard_joint_trajectory(ros_context):
    node = TeachPendantNode(context=ros_context)
    published = []

    class Publisher:
        def publish(self, message):
            published.append(message)

    try:
        node.publisher = Publisher()
        node.publish_targets({node.model.joint_names[0]: 0.25}, duration_sec=0.5)
    finally:
        node.destroy_node()

    assert len(published) == 1
    message = published[0]
    assert isinstance(message, JointTrajectory)
    assert message.header.frame_id == "teach_pendant"
    assert message.joint_names == list(node.model.joint_names)
    assert len(message.points) == 2
    assert message.points[0].time_from_start.sec == 0
    assert message.points[1].time_from_start.nanosec == 500_000_000
