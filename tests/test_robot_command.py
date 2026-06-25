import pytest

from cook_core.interfaces import JointStateData, JointTrajectoryData, TrajectoryPointData
from cook_core.robot import (
    RecordingRobotCommandPort,
    RobotCommandBuilder,
    RobotCommandError,
    create_robot_command_port,
)


def test_move_joints_builds_complete_two_point_trajectory():
    builder = RobotCommandBuilder(
        joint_names=("joint_1", "joint_2"),
        current_positions={"joint_1": 0.1},
    )

    trajectory = builder.build_move_joints({"joint_2": 0.4}, duration_sec=1.5)

    assert trajectory.joint_names == ("joint_1", "joint_2")
    assert trajectory.points[0].positions == {"joint_1": 0.1, "joint_2": 0.0}
    assert trajectory.points[1].positions == {"joint_1": 0.1, "joint_2": 0.4}
    assert trajectory.points[1].time_from_start_sec == pytest.approx(1.5)


def test_move_joint_sequence_aligns_partial_waypoints():
    builder = RobotCommandBuilder(
        joint_names=("joint_1", "joint_2"),
        current_positions=JointStateData(
            names=("joint_1", "joint_2"),
            positions=(0.0, 0.2),
        ),
    )

    trajectory = builder.build_joint_sequence(
        [{"joint_1": 0.5}, {"joint_2": 0.7}, {"joint_1": -0.1}],
        duration_sec=3.0,
    )

    assert [point.time_from_start_sec for point in trajectory.points] == pytest.approx(
        [0.0, 1.5, 3.0]
    )
    assert trajectory.points[0].positions == {"joint_1": 0.5, "joint_2": 0.2}
    assert trajectory.points[1].positions == {"joint_1": 0.5, "joint_2": 0.7}
    assert trajectory.points[2].positions == {"joint_1": -0.1, "joint_2": 0.7}


def test_builder_accepts_ready_trajectory_and_rejects_wrong_order():
    builder = RobotCommandBuilder(joint_names=("joint_1", "joint_2"))
    trajectory = JointTrajectoryData(
        joint_names=("joint_1", "joint_2"),
        points=(TrajectoryPointData({"joint_1": 0.1, "joint_2": 0.2}, 0.0),),
    )

    assert builder.build_move_joints(trajectory) is trajectory

    wrong = JointTrajectoryData(
        joint_names=("joint_2", "joint_1"),
        points=(TrajectoryPointData({"joint_2": 0.2, "joint_1": 0.1}, 0.0),),
    )
    with pytest.raises(RobotCommandError, match="joint order mismatch"):
        builder.build_move_joints(wrong)


def test_builder_validates_inputs():
    builder = RobotCommandBuilder(joint_names=("joint_1",))

    with pytest.raises(RobotCommandError, match="must not be empty"):
        builder.build_move_joints({})
    with pytest.raises(RobotCommandError, match="non-negative"):
        builder.build_move_joints({"joint_1": 0.1}, duration_sec=-1.0)
    with pytest.raises(RobotCommandError, match="finite"):
        builder.build_move_joints({"joint_1": float("nan")})
    with pytest.raises(RobotCommandError, match="unknown joints"):
        builder.build_move_joints({"joint_2": 0.1})


def test_fake_robot_command_port_runs_task_without_ros():
    robot = create_robot_command_port(
        backend="fake",
        joint_names=("joint_1", "joint_2"),
    )

    trajectory = robot.move_joints({"joint_1": 0.2}, duration_sec=0.5)
    robot.hold_current()
    robot.close()

    assert isinstance(robot, RecordingRobotCommandPort)
    assert trajectory.points[-1].positions == {"joint_1": 0.2, "joint_2": 0.0}
    assert len(robot.published) == 2
    assert robot.closed
