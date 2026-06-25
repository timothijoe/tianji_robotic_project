from __future__ import annotations

import argparse
import time

from cook_core.robot import RobotCommandPort, create_robot_command_port


def run_task(robot: RobotCommandPort) -> None:
    robot.move_joints(
        {
            "Joint1_L": 0.2,
            "Joint2_L": -0.1,
            "Joint3_L": 0.15,
        },
        duration_sec=1.5,
    )
    time.sleep(1.7)

    robot.move_joint_sequence(
        [
            {"Joint1_L": 0.0, "Joint2_L": 0.0, "Joint3_L": 0.0},
            {"Joint1_L": 0.3, "Joint2_L": -0.2, "Joint3_L": 0.1},
            {"Joint1_L": -0.2, "Joint2_L": 0.2, "Joint3_L": -0.1},
        ],
        duration_sec=3.0,
    )
    time.sleep(3.2)

    robot.hold_current(duration_sec=0.2)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Function-call robot control demo for simulation or hardware ports."
    )
    parser.add_argument(
        "--backend",
        default="ros2",
        choices=("ros2", "fake"),
        help="Command backend. Use ros2 after visualization.launch.py is running.",
    )
    args = parser.parse_args()

    config = {}
    if args.backend == "fake":
        config["joint_names"] = tuple(f"Joint{index}_L" for index in range(1, 8))

    robot = create_robot_command_port(backend=args.backend, **config)
    try:
        run_task(robot)
    finally:
        robot.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
