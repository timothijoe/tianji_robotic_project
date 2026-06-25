import pytest

rclpy = pytest.importorskip("rclpy")
trajectory_msgs = pytest.importorskip("trajectory_msgs.msg")
JointTrajectory = trajectory_msgs.JointTrajectory

from rclpy.context import Context

from cook_bringup.client import RosRobotCommandPort


@pytest.fixture
def ros_context():
    context = Context()
    context.init(initialize_logging=False)
    try:
        yield context
    finally:
        context.shutdown()


def test_ros_robot_command_port_publishes_standard_joint_trajectory(ros_context):
    client = RosRobotCommandPort(
        joint_names=("joint_1", "joint_2"),
        context=ros_context,
        publish_warmup_sec=0.0,
    )
    publisher = _Publisher()
    client.publisher = publisher

    try:
        trajectory = client.move_joints({"joint_2": 0.4}, duration_sec=0.75)
        client.close()
        client.close()
    finally:
        client.close()

    assert client.topic == "/mujoco_controller_node/joint_trajectory"
    assert trajectory.points[-1].positions == {"joint_1": 0.0, "joint_2": 0.4}
    assert len(publisher.messages) == 1
    assert isinstance(publisher.messages[0], JointTrajectory)
    assert publisher.messages[0].joint_names == ["joint_1", "joint_2"]
    assert list(publisher.messages[0].points[-1].positions) == [0.0, 0.4]


class _Publisher:
    def __init__(self):
        self.messages = []

    def get_subscription_count(self):
        return 1

    def publish(self, message):
        self.messages.append(message)
