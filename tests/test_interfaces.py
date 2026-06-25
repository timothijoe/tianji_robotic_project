import pytest

from cook_core.interfaces import (
    InterfaceDataError,
    JointStateData,
    JointTrajectoryData,
    TrajectoryPointData,
)


def test_joint_state_requires_matching_lengths():
    with pytest.raises(InterfaceDataError, match="length mismatch"):
        JointStateData(names=("joint_1",), positions=(0.0, 1.0))


def test_joint_trajectory_requires_monotonic_time():
    with pytest.raises(InterfaceDataError, match="monotonic"):
        JointTrajectoryData(
            joint_names=("joint_1",),
            points=(
                TrajectoryPointData({"joint_1": 0.0}, 1.0),
                TrajectoryPointData({"joint_1": 0.1}, 0.5),
            ),
        )


def test_joint_trajectory_requires_all_declared_joints():
    with pytest.raises(InterfaceDataError, match="missing joint"):
        JointTrajectoryData(
            joint_names=("joint_1", "joint_2"),
            points=(TrajectoryPointData({"joint_1": 0.0}, 0.0),),
        )


def test_joint_trajectory_round_trips_through_dict():
    trajectory = JointTrajectoryData(
        joint_names=("joint_1", "joint_2"),
        points=(
            TrajectoryPointData({"joint_1": 0.0, "joint_2": 1.0}, 0.0),
            TrajectoryPointData({"joint_1": 0.5, "joint_2": 1.5}, 1.0),
        ),
        trajectory_id="demo",
        frame_id="base",
        source="test",
        metadata={"planner": "linear"},
    )

    restored = JointTrajectoryData.from_dict(trajectory.to_dict())

    assert restored == trajectory
