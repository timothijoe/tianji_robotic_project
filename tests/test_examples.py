import importlib.util
from pathlib import Path

from cook_core.robot import create_robot_command_port


def test_function_control_demo_task_runs_with_fake_port():
    module = _load_example("function_control_demo.py")
    module.time.sleep = lambda _duration: None
    robot = create_robot_command_port(
        backend="fake",
        joint_names=tuple(f"Joint{index}_L" for index in range(1, 8)),
    )

    module.run_task(robot)

    assert len(robot.published) == 3
    assert robot.published[0].points[-1].positions["Joint1_L"] == 0.2
    assert robot.published[1].points[-1].positions["Joint2_L"] == 0.2


def test_planning_control_demo_builds_linear_plan():
    module = _load_example("planning_control_demo.py")

    result = module.build_plan(planner_type="linear")

    assert result.success
    assert result.trajectory.source == "linear"
    assert len(result.points) == 80
    assert result.trajectory.joint_names[0] == "Joint1_L"


def test_planning_control_demo_chooses_a_different_goal_on_repeated_runs():
    module = _load_example("planning_control_demo.py")
    first = module.build_plan(planner_type="linear")
    first_goal = first.points[-1].positions

    second = module.build_plan(
        planner_type="linear",
        start_positions=first_goal,
    )

    assert second.success
    assert second.points[0].positions == first_goal
    assert second.points[-1].positions != first_goal


def _load_example(filename: str):
    path = Path(__file__).resolve().parents[1] / "examples" / filename
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module
