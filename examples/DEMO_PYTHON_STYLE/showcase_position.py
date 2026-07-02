#!/usr/bin/env python3
"""SDK-style position mode example for MuJoCo or the real Marvin robot."""

from __future__ import annotations

import argparse

from _common import add_backend_args, make_robot


READY_B = [-75.627, -67.572, 52.390, -124.574, -90.421, 42.952, 41.374]
TARGET_B = [-68.0, -70.0, 48.0, -118.0, -88.0, 38.0, 36.0]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    add_backend_args(parser)
    parser.add_argument("--hold-s", type=float, default=1.5)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    robot = make_robot(args)
    robot.connect(args.robot_ip)

    try:
        robot.set_position_state(args.arm, velRatio=30, AccRatio=30)
        robot.set_joint_position_cmd(args.arm, READY_B)
        robot.wait(args.hold_s)

        robot.set_joint_position_cmd(args.arm, TARGET_B)
        robot.wait(args.hold_s)

        robot.set_joint_position_cmd(args.arm, READY_B)
        robot.wait(args.hold_s)

        if hasattr(robot, "subscribe"):
            data = robot.subscribe(None)
            arm_index = 0 if args.arm == "A" else 1
            print("final joints [deg]:", data["outputs"][arm_index]["fb_joint_pos"])
        return 0
    finally:
        robot.release_robot()


if __name__ == "__main__":
    raise SystemExit(main())
