import pytest

from cook_core.interfaces import JointTrajectoryData
from cook_core.planning import LinearJointPlanner, PlanRequest, PlanResult


def test_linear_planner_outputs_endpoint_trajectory():
    planner = LinearJointPlanner()

    result = planner.plan(
        PlanRequest.from_position_mappings(
            start_positions={"joint_1": 0.0, "joint_2": 1.0},
            goal_positions={"joint_1": 1.0, "joint_2": 3.0},
            joint_names=("joint_1", "joint_2"),
            duration_sec=2.0,
            waypoint_count=3,
        )
    )

    assert len(result.points) == 3
    assert result.points[0].positions == {"joint_1": 0.0, "joint_2": 1.0}
    assert result.points[-1].positions == {"joint_1": 1.0, "joint_2": 3.0}
    assert [point.time_from_start_sec for point in result.points] == pytest.approx(
        [0.0, 1.0, 2.0]
    )


def test_linear_planner_requires_all_goal_joints():
    planner = LinearJointPlanner()

    with pytest.raises(ValueError, match="goal_positions missing joint"):
        planner.plan(
            PlanRequest.from_position_mappings(
                start_positions={"joint_1": 0.0},
                goal_positions={},
                joint_names=("joint_1",),
            )
        )


def test_plan_request_supports_common_planner_options():
    request = PlanRequest.from_position_mappings(
        start_positions={"joint_1": 0.0},
        goal_positions={"joint_1": 1.0},
        joint_names=("joint_1",),
        planning_time_sec=2.5,
        collision_check_resolution=0.02,
        metadata={"planner": "ompl"},
    )

    assert request.planning_time_sec == pytest.approx(2.5)
    assert request.collision_check_resolution == pytest.approx(0.02)
    assert request.metadata == {"planner": "ompl"}


def test_plan_result_can_represent_failure():
    trajectory = JointTrajectoryData(joint_names=("joint_1",), points=())
    result = PlanResult(
        trajectory=trajectory,
        success=False,
        message="no path",
        planner_name="ompl_rrt_connect",
    )

    assert result.is_empty
    assert not result.success
    assert result.message == "no path"
    assert result.planner_name == "ompl_rrt_connect"


@pytest.mark.parametrize(
    ("joint_names", "start_values", "goal_values"),
    [
        (("joint_c", "joint_a", "joint_b"), (0.3, 0.1, 0.2), (1.3, 1.1, 1.2)),
        (
            ("j6", "j2", "j5", "j1", "j4", "j3"),
            (0.6, 0.2, 0.5, 0.1, 0.4, 0.3),
            (-0.6, -0.2, -0.5, -0.1, -0.4, -0.3),
        ),
    ],
)
def test_linear_planner_preserves_robot_specific_joint_order(
    joint_names,
    start_values,
    goal_values,
):
    start = dict(zip(joint_names, start_values))
    goal = dict(zip(joint_names, goal_values))

    result = LinearJointPlanner().plan(
        PlanRequest.from_position_mappings(
            start_positions=start,
            goal_positions=goal,
            joint_names=joint_names,
            waypoint_count=4,
        )
    )

    assert result.trajectory.joint_names == joint_names
    assert result.points[0].ordered_positions(joint_names) == start_values
    assert result.points[-1].ordered_positions(joint_names) == goal_values
