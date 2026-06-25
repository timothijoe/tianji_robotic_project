from __future__ import annotations

import argparse
import json
from pathlib import Path

from cook_description.paths import CHOPPING_MJCF_PATH
from cook_mujoco.chopping import ChoppingConfig, MujocoForceRobot


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="MuJoCo Cartesian impedance and chopping force-control demo")
    parser.add_argument("--model", type=Path, default=CHOPPING_MJCF_PATH)
    parser.add_argument("--mode", choices=("static", "chopping"), default="chopping")
    parser.add_argument("--cycles", type=int, default=5)
    parser.add_argument("--force", type=float, default=10.0)
    parser.add_argument("--hold", type=float, default=0.5)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--realtime", action="store_true")
    parser.add_argument("--log", type=Path, default=Path("force_control_run.csv"))
    parser.add_argument("--status-hz", type=float, default=5.0, help="terminal status refresh rate; use 0 to disable")
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    robot = MujocoForceRobot(args.model, viewer=args.viewer, realtime=args.realtime)
    robot.connect()
    last_print_time = -1e9
    last_phase = None

    def print_status(sample):
        nonlocal last_print_time, last_phase
        period = 1.0 / args.status_hz if args.status_hz > 0 else float("inf")
        changed = sample.phase != last_phase
        if changed or sample.time_s - last_print_time >= period:
            wrench = sample.compensated_wrench
            print(
                f"t={sample.time_s:7.3f}s "
                f"phase={sample.phase.value:10s} "
                f"mode={sample.control_mode:19s} "
                f"F=({wrench[0]:+.2f},{wrench[1]:+.2f},{wrench[2]:+.2f})N "
                f"Fz_ctrl={sample.measured_force_n:+6.2f}N target_Fz={sample.target_force_n:5.1f}N "
                f"M=({wrench[3]:+.3f},{wrench[4]:+.3f},{wrench[5]:+.3f})Nm",
                flush=True,
            )
            last_print_time = sample.time_s
            last_phase = sample.phase

    callback = print_status if args.status_hz > 0 else None
    try:
        robot.initialize()
        if args.mode == "static":
            robot.execute_chopping_trajectory(
                config=ChoppingConfig(cycles=1, target_force_n=args.force, force_hold_s=max(args.hold, 0.1)),
                log_path=args.log,
                status_callback=callback,
            )
        else:
            robot.execute_chopping_trajectory(
                config=ChoppingConfig(cycles=args.cycles, target_force_n=args.force, force_hold_s=args.hold),
                log_path=args.log,
                status_callback=callback,
            )
        print(json.dumps(robot.summary(), indent=2, ensure_ascii=False))
        return 0
    finally:
        robot.close()


if __name__ == "__main__":
    raise SystemExit(main())
