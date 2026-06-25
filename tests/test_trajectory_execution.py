import pytest

from cook_core.interfaces import JointTrajectoryData, TrajectoryPointData
from cook_core.trajectory import TrajectoryExecutor


def _trajectory():
    return JointTrajectoryData(
        joint_names=("joint_1", "joint_2"),
        points=(
            TrajectoryPointData({"joint_1": 0.0, "joint_2": 0.5}, 0.0),
            TrajectoryPointData({"joint_1": 1.0, "joint_2": 1.5}, 1.0),
            TrajectoryPointData({"joint_1": 2.0, "joint_2": 2.5}, 2.0),
        ),
        trajectory_id="demo",
    )


def test_executor_interpolates_between_points_and_finishes():
    executor = TrajectoryExecutor(expected_joint_names=("joint_1", "joint_2"))
    executor.start(_trajectory(), now_sec=10.0)

    first = executor.command_at(now_sec=10.2)
    second = executor.command_at(now_sec=11.2)
    third = executor.command_at(now_sec=12.1)
    after_finish = executor.command_at(now_sec=12.2)

    assert first is not None
    assert first.target.as_mapping() == pytest.approx(
        {"joint_1": 0.2, "joint_2": 0.7}
    )
    assert second is not None
    assert second.target.as_mapping() == pytest.approx(
        {"joint_1": 1.2, "joint_2": 1.7}
    )
    assert third is not None
    assert third.target.as_mapping() == {"joint_1": 2.0, "joint_2": 2.5}
    assert third.trajectory_id == "demo"
    assert after_finish is None
    assert not executor.has_active_trajectory


def test_executor_immediately_applies_repeated_time_points():
    trajectory = JointTrajectoryData(
        joint_names=("joint_1",),
        points=(
            TrajectoryPointData({"joint_1": 0.0}, 0.0),
            TrajectoryPointData({"joint_1": 1.0}, 0.0),
        ),
    )
    executor = TrajectoryExecutor(expected_joint_names=("joint_1",))
    executor.start(trajectory, now_sec=5.0)

    command = executor.command_at(now_sec=5.0)
    after_finish = executor.command_at(now_sec=5.01)

    assert command is not None
    assert command.target.as_mapping() == {"joint_1": 1.0}
    assert after_finish is None


def test_executor_rejects_joint_order_mismatch():
    executor = TrajectoryExecutor(expected_joint_names=("joint_2", "joint_1"))

    accepted = executor.start(_trajectory(), now_sec=0.0)

    assert not accepted
    assert not executor.has_active_trajectory
