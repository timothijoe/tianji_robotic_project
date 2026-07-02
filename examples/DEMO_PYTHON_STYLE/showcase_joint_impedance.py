#!/usr/bin/env python3
"""SDK-style joint impedance example for MuJoCo or the real Marvin robot."""

from __future__ import annotations

import argparse

from _common import add_backend_args, make_robot


READY_B = [-75.627, -67.572, 52.390, -124.574, -90.421, 42.952, 41.374]
TARGET_B = [-73.0, -72.0, 55.0, -119.0, -86.0, 37.0, 45.0]

# Conservative stiffness/damping for simulation.  On the real robot these are
# only starting values; validate at low speed before increasing.
JOINT_K = [8.0, 8.0, 8.0, 4.0, 2.0, 1.5, 1.0]
JOINT_D = [1.5, 1.5, 1.5, 1.0, 0.6, 0.4, 0.3]


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
        robot.set_imp_joint_state(
            args.arm,
            velRatio=30,
            AccRatio=30,
            K=JOINT_K,
            D=JOINT_D,
        )
        robot.set_joint_position_cmd(args.arm, READY_B)
        robot.wait(args.hold_s)

        robot.set_joint_position_cmd(args.arm, TARGET_B)
        robot.wait(args.hold_s)

        robot.set_joint_position_cmd(args.arm, READY_B)
        robot.wait(args.hold_s)
        return 0
    finally:
        robot.release_robot()


if __name__ == "__main__":
    raise SystemExit(main())
