from __future__ import annotations

import argparse
from collections.abc import Mapping
from math import sqrt

from cook_core.planning import PlanRequest
from cook_core.robot import create_robot_command_port
from cook_description.assets import RobotAssetContext
from cook_description.models import load_robot_definition
from cook_mujoco.planning import create_planner


def build_plan(
    *,
    planner_type: str,
    start_positions: Mapping[str, float] | None = None,
):
    assets = RobotAssetContext.local_default()
    definition = load_robot_definition(assets.urdf_path)
    joint_names = definition.movable_joint_names
    start = (
        {name: 0.0 for name in joint_names}
        if start_positions is None
        else {name: float(start_positions[name]) for name in joint_names}
    )
    positive_goal = {
        name: 0.15 - (0.3 * index / max(1, len(joint_names) - 1))
        for index, name in enumerate(joint_names)
    }
    negative_goal = {name: -value for name, value in positive_goal.items()}
    goal = max(
        (positive_goal, negative_goal),
        key=lambda candidate: _joint_distance(start, candidate, joint_names),
    )
    request = PlanRequest.from_position_mappings(
        start_positions=start,
        goal_positions=goal,
        joint_names=joint_names,
        duration_sec=3.0,
        waypoint_count=80,
        planning_time_sec=1.0,
    )
    planner = create_planner(
        planner_type,
        model_path=assets.mjcf_path,
        urdf_path=assets.urdf_path,
    )
    return planner.plan(request)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Plan a joint trajectory and send it through the robot command port."
    )
    parser.add_argument("--planner", choices=("linear", "ompl"), default="linear")
    parser.add_argument("--backend", choices=("ros2", "fake"), default="ros2")
    args = parser.parse_args()

    assets = RobotAssetContext.local_default()
    definition = load_robot_definition(assets.urdf_path)
    robot = create_robot_command_port(
        backend=args.backend,
        joint_names=definition.movable_joint_names,
    )
    try:
        if args.backend == "ros2":
            current_state = robot.wait_for_joint_state(timeout_sec=3.0)
            start_positions = current_state.as_mapping()
            print(f"Current joint positions: {current_state.positions}")
        else:
            start_positions = None

        result = build_plan(
            planner_type=args.planner,
            start_positions=start_positions,
        )
        if not result.success:
            raise RuntimeError(result.message or "planning failed")

        goal_positions = result.points[-1].ordered_positions(
            result.trajectory.joint_names
        )
        print(
            f"Plan ready: planner={args.planner}, points={len(result.points)}, "
            f"goal={goal_positions}"
        )
        executed = robot.move_joints(result.trajectory)
        if args.backend == "ros2":
            print("Trajectory sent to MuJoCo controller.")
        else:
            final_positions = executed.points[-1].ordered_positions(
                executed.joint_names
            )
            print(
                "Fake backend accepted trajectory: "
                f"planner={args.planner}, joints={len(executed.joint_names)}, "
                f"points={len(executed.points)}, "
                f"duration={executed.points[-1].time_from_start_sec:.3f}s"
            )
            print(f"Final joint positions: {final_positions}")
    finally:
        robot.close()
    return 0


def _joint_distance(
    first: Mapping[str, float],
    second: Mapping[str, float],
    joint_names: tuple[str, ...],
) -> float:
    return sqrt(
        sum(
            (float(first[name]) - float(second[name])) ** 2
            for name in joint_names
        )
    )


if __name__ == "__main__":
    raise SystemExit(main())
