from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np

from cook_mujoco.control import MujocoPositionController, MujocoRuntime
from cook_description.models import load_robot_definition
from cook_description.paths import DEFAULT_MJCF_PATH, DEFAULT_URDF_PATH
from cook_core.planning import LinearJointPlanner, PlanRequest
from cook_description.tools.convert_model import main as convert_main


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="MuJoCo robot visualization tools.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("convert", help="Convert the sample URDF to MJCF/XML.")

    run_parser = subparsers.add_parser("run", help="Open the MuJoCo viewer.")
    _add_runtime_args(run_parser)

    demo_parser = subparsers.add_parser("demo", help="Play a planned joint demo.")
    _add_runtime_args(demo_parser)
    demo_parser.add_argument("--duration", type=float, default=3.0)
    demo_parser.add_argument("--waypoints", type=int, default=120)
    demo_parser.add_argument("--steps", type=int, default=0)
    demo_parser.add_argument("--headless", action="store_true")

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args, remaining = parser.parse_known_args(argv)
    if args.command == "convert":
        return convert_main(remaining or ["--overwrite", "--validate"])
    if args.command == "run":
        return run_viewer(args)
    if args.command == "demo":
        return run_demo(args)
    parser.error(f"unsupported command: {args.command}")
    return 2


def run_viewer(args) -> int:
    controller = _load_controller(args.model, args.urdf)
    controller.reset()
    _launch_viewer(controller.runtime)
    return 0


def run_demo(args) -> int:
    controller = _load_controller(args.model, args.urdf)
    start_state = controller.reset().as_mapping()
    goal_state = _demo_goal(start_state, controller.joint_names)
    planner = LinearJointPlanner()
    plan = planner.plan(
        PlanRequest.from_position_mappings(
            start_positions=start_state,
            goal_positions=goal_state,
            joint_names=controller.joint_names,
            duration_sec=args.duration,
            waypoint_count=args.waypoints,
        )
    )

    if args.headless:
        max_steps = args.steps if args.steps > 0 else len(plan.points)
        for point in plan.points[:max_steps]:
            controller.set_joint_targets(point.positions)
        return 0

    _play_plan_in_viewer(controller, plan.points, args.duration)
    return 0


def _add_runtime_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--model", default=str(DEFAULT_MJCF_PATH), help="MJCF/XML path.")
    parser.add_argument("--urdf", default=str(DEFAULT_URDF_PATH), help="URDF metadata path.")


def _load_controller(model_path: str | Path, urdf_path: str | Path) -> MujocoPositionController:
    definition = load_robot_definition(urdf_path)
    runtime = MujocoRuntime.load(model_path, joint_names=definition.movable_joint_names)
    return MujocoPositionController(runtime, joint_limits=definition.joint_limits)


def _demo_goal(
    start_state: dict[str, float],
    joint_names: tuple[str, ...],
) -> dict[str, float]:
    goal = dict(start_state)
    offsets = np.linspace(0.2, -0.2, num=len(joint_names))
    for name, offset in zip(joint_names, offsets):
        goal[name] = float(start_state[name] + offset)
    return goal


def _launch_viewer(runtime: MujocoRuntime) -> None:
    import mujoco.viewer

    with mujoco.viewer.launch_passive(runtime.model, runtime.data) as viewer:
        while viewer.is_running():
            runtime._mujoco.mj_forward(runtime.model, runtime.data)
            viewer.sync()
            time.sleep(float(runtime.model.opt.timestep))


def _play_plan_in_viewer(controller, points, duration_sec: float) -> None:
    import mujoco.viewer

    dt = max(1e-6, duration_sec / max(1, len(points) - 1))
    with mujoco.viewer.launch_passive(controller.runtime.model, controller.runtime.data) as viewer:
        for point in points:
            if not viewer.is_running():
                break
            controller.set_joint_targets(point.positions)
            viewer.sync()
            time.sleep(dt)
        while viewer.is_running():
            viewer.sync()
            time.sleep(float(controller.runtime.model.opt.timestep))


if __name__ == "__main__":
    raise SystemExit(main())
