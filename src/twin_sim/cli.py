import argparse
from pathlib import Path
import time

from twin_sim.kinematics import Kinematics
from twin_sim.robot import RIGHT_HOME_RAD, RightArmRobot
from twin_sim.tasks.chop import ChopConfig, run_chop
from twin_sim.tasks.line_chop import LineChopConfig, run_line_chop
from twin_sim.tasks.hand_demo import HandDemoConfig, run_hand_demo
from twin_sim.trajectory import cartesian_trajectory, joint_trajectory


def main(argv: list[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    if args.command == "hand-demo":
        run_hand_demo(
            HandDemoConfig(
                close_duration_s=4.0 if args.slow else 2.0,
                open_duration_s=4.0 if args.slow else 2.0,
                viewer_start_hold_s=5.0 if args.slow else 0.0,
                viewer_end_hold_s=10.0 if args.slow else 0.0,
            ),
            viewer=not args.headless,
        )
        return 0
    if args.command == "chop":
        config = (
            ChopConfig(
                control_dt_s=args.control_dt,
                orient_duration_s=3.0,
                approach_duration_s=3.0,
                descent_duration_s=3.0,
                hold_duration_s=1.0,
                retract_duration_s=3.0,
                viewer_start_hold_s=5.0,
                viewer_end_hold_s=8.0,
                full_motion=args.full_motion,
            )
            if args.slow
            else ChopConfig(
                control_dt_s=args.control_dt,
                full_motion=args.full_motion,
            )
        )
        run_chop(
            config,
            log_path=args.log,
            viewer=not args.headless,
        )
        return 0
    if args.command == "line-chop":
        chop_config = (
            ChopConfig(
                control_dt_s=args.control_dt,
                descent_duration_s=3.0,
                hold_duration_s=1.0,
                retract_duration_s=3.0,
                viewer_start_hold_s=5.0,
                viewer_end_hold_s=8.0,
            )
            if args.slow
            else ChopConfig(control_dt_s=args.control_dt)
        )
        run_line_chop(
            LineChopConfig(
                chop=chop_config,
                cuts=args.cuts,
                spacing_m=args.spacing_m,
                shift_duration_s=3.0 if args.slow else 1.0,
            ),
            log_path=args.log,
            plot_path=args.plot,
            viewer=not args.headless,
        )
        return 0
    if args.command == "view":
        robot = RightArmRobot(viewer=True)
        try:
            while robot._viewer is not None and robot._viewer.is_running():
                robot.step(0.01)
                time.sleep(0.01)
        finally:
            robot.close()
        return 0

    robot = RightArmRobot(viewer=not args.headless)
    try:
        if args.command == "joint":
            if not 1 <= args.joint <= 7:
                parser.error("--joint must be between 1 and 7")
            goal = RIGHT_HOME_RAD.copy()
            goal[args.joint - 1] += args.delta_rad
            points = joint_trajectory(RIGHT_HOME_RAD, goal, 0.5, 0.01)
        else:
            kinematics = Kinematics(robot.sim)
            start = kinematics.fk(RIGHT_HOME_RAD)
            goal = start.copy()
            goal[2, 3] += args.dz_m
            points = cartesian_trajectory(
                kinematics, start, goal, RIGHT_HOME_RAD, 0.5, 0.01
            )
        robot.validate_targets([point.joints_rad for point in points])
        for point in points:
            robot.command(point.joints_rad)
            robot.step(0.01)
        return 0
    finally:
        robot.close()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="twin-sim")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("view")
    hand_demo = commands.add_parser("hand-demo")
    hand_demo.add_argument("--headless", action="store_true")
    hand_demo.add_argument("--slow", action="store_true")
    joint = commands.add_parser("joint")
    joint.add_argument("--joint", type=int, required=True)
    joint.add_argument("--delta-rad", type=float, required=True)
    joint.add_argument("--headless", action="store_true")
    cartesian = commands.add_parser("cartesian")
    cartesian.add_argument("--dz-m", type=float, required=True)
    cartesian.add_argument("--headless", action="store_true")
    chop = commands.add_parser("chop")
    chop.add_argument("--headless", action="store_true")
    chop.add_argument("--log", type=Path, required=True)
    chop.add_argument("--control-dt", type=float, default=0.01)
    chop.add_argument("--slow", action="store_true")
    chop.add_argument("--full-motion", action="store_true")
    line_chop = commands.add_parser("line-chop")
    line_chop.add_argument("--cuts", type=int, default=5)
    line_chop.add_argument("--spacing-m", type=float, default=0.03)
    line_chop.add_argument("--headless", action="store_true")
    line_chop.add_argument("--log", type=Path, required=True)
    line_chop.add_argument("--plot", type=Path, required=True)
    line_chop.add_argument("--control-dt", type=float, default=0.01)
    line_chop.add_argument("--slow", action="store_true")
    return parser


if __name__ == "__main__":
    raise SystemExit(main())
