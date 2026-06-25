import math

import pytest

pytest.importorskip("geometry_msgs.msg")
pytest.importorskip("nav_msgs.msg")

from cook_bringup.ros.tcp_path import (
    append_body_pose_to_path,
    body_pose_to_pose_stamped,
    body_pose_to_transform_stamped,
    build_predicted_tcp_path,
    compose_tcp_pose,
)
from cook_core.interfaces import JointTrajectoryData, TrajectoryPointData
from cook_mujoco.control.runtime import BodyPose


def test_body_pose_to_pose_stamped_uses_ros_quaternion_order():
    message = body_pose_to_pose_stamped(
        BodyPose(position=(1.0, 2.0, 3.0), quaternion=(0.5, 0.1, 0.2, 0.3)),
        frame_id="Base_L",
    )

    assert message.header.frame_id == "Base_L"
    assert message.pose.position.x == pytest.approx(1.0)
    assert message.pose.orientation.x == pytest.approx(0.1)
    assert message.pose.orientation.y == pytest.approx(0.2)
    assert message.pose.orientation.z == pytest.approx(0.3)
    assert message.pose.orientation.w == pytest.approx(0.5)


def test_zero_tcp_offset_matches_parent_pose():
    parent = BodyPose(position=(1.0, 2.0, 3.0), quaternion=(1.0, 0.0, 0.0, 0.0))

    tcp = compose_tcp_pose(
        parent,
        offset_xyz=(0.0, 0.0, 0.0),
        offset_rpy=(0.0, 0.0, 0.0),
    )

    assert tcp == parent


def test_tcp_offset_is_applied_in_parent_frame():
    yaw_90 = (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))
    parent = BodyPose(position=(1.0, 2.0, 3.0), quaternion=yaw_90)

    tcp = compose_tcp_pose(
        parent,
        offset_xyz=(1.0, 0.0, 0.0),
        offset_rpy=(0.0, 0.0, 0.0),
    )

    assert tcp.position == pytest.approx((1.0, 3.0, 3.0))
    assert tcp.quaternion == pytest.approx(yaw_90)


def test_tcp_rpy_offset_composes_orientation():
    parent = BodyPose(position=(0.0, 0.0, 0.0), quaternion=(1.0, 0.0, 0.0, 0.0))

    tcp = compose_tcp_pose(
        parent,
        offset_xyz=(0.0, 0.0, 0.0),
        offset_rpy=(0.0, 0.0, math.pi / 2.0),
    )

    assert tcp.quaternion == pytest.approx(
        (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))
    )


def test_transform_uses_tcp_child_frame():
    transform = body_pose_to_transform_stamped(
        BodyPose(position=(1.0, 2.0, 3.0), quaternion=(1.0, 0.0, 0.0, 0.0)),
        parent_frame_id="Base_L",
        child_frame_id="tcp_link",
    )

    assert transform.header.frame_id == "Base_L"
    assert transform.child_frame_id == "tcp_link"
    assert transform.transform.translation.z == pytest.approx(3.0)


def test_append_body_pose_to_path_limits_actual_history():
    from nav_msgs.msg import Path

    path = Path()

    for index in range(4):
        append_body_pose_to_path(
            path,
            BodyPose(position=(float(index), 0.0, 0.0), quaternion=(1.0, 0.0, 0.0, 0.0)),
            frame_id="Base_L",
            max_points=2,
        )

    assert path.header.frame_id == "Base_L"
    assert len(path.poses) == 2
    assert [pose.pose.position.x for pose in path.poses] == pytest.approx([2.0, 3.0])


def test_predicted_tcp_path_samples_joint_trajectory():
    runtime = _FakeRuntime()
    trajectory = JointTrajectoryData(
        joint_names=("joint_1",),
        points=(
            TrajectoryPointData({"joint_1": 0.0}, 0.0),
            TrajectoryPointData({"joint_1": 1.0}, 1.0),
        ),
    )

    path = build_predicted_tcp_path(
        runtime=runtime,
        trajectory=trajectory,
        parent_body_name="Link7_L",
        frame_id="Base_L",
        offset_xyz=(0.0, 1.0, 0.0),
        offset_rpy=(0.0, 0.0, 0.0),
        sample_count=3,
    )

    assert [pose.pose.position.x for pose in path.poses] == pytest.approx(
        [0.0, 0.5, 1.0]
    )
    assert [pose.pose.position.y for pose in path.poses] == pytest.approx(
        [1.0, 1.0, 1.0]
    )
    assert runtime.body_names == ["Link7_L", "Link7_L", "Link7_L"]


class _FakeRuntime:
    def __init__(self):
        self.positions = (0.0,)
        self.body_names = []

    def set_joint_positions(self, positions, *, forward=True):
        assert forward
        self.positions = tuple(float(value) for value in positions)

    def get_body_pose(self, name: str) -> BodyPose:
        self.body_names.append(name)
        return BodyPose(
            position=(self.positions[0], 0.0, 0.0),
            quaternion=(1.0, 0.0, 0.0, 0.0),
        )
