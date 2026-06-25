import pytest

sensor_msgs = pytest.importorskip("sensor_msgs.msg")
trajectory_msgs = pytest.importorskip("trajectory_msgs.msg")
JointState = sensor_msgs.JointState
JointTrajectory = trajectory_msgs.JointTrajectory

from cook_core.interfaces import JointStateData, JointTrajectoryData, TrajectoryPointData
from cook_bringup.ros.conversions import (
    joint_state_data_from_msg,
    joint_state_data_to_msg,
    trajectory_data_from_msg,
    trajectory_data_to_msg,
)


def test_joint_state_round_trip():
    data = JointStateData(
        names=("joint_1", "joint_2"),
        positions=(0.1, 0.2),
        timestamp_sec=1.25,
    )

    message = joint_state_data_to_msg(data)
    parsed = joint_state_data_from_msg(message)

    assert isinstance(message, JointState)
    assert parsed.names == data.names
    assert parsed.positions == pytest.approx(data.positions)
    assert parsed.timestamp_sec == pytest.approx(1.25)


def test_trajectory_round_trip():
    data = JointTrajectoryData(
        joint_names=("joint_1", "joint_2"),
        points=(
            TrajectoryPointData({"joint_1": 0.0, "joint_2": 1.0}, 0.0),
            TrajectoryPointData({"joint_1": 0.5, "joint_2": 1.5}, 1.25),
        ),
        frame_id="demo",
    )

    message = trajectory_data_to_msg(data)
    parsed = trajectory_data_from_msg(message)

    assert isinstance(message, JointTrajectory)
    assert parsed.joint_names == data.joint_names
    assert len(parsed.points) == 2
    assert parsed.points[-1].positions == {"joint_1": 0.5, "joint_2": 1.5}
    assert parsed.points[-1].time_from_start_sec == pytest.approx(1.25)
