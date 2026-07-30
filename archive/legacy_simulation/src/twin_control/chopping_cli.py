"""CLI entry point for the SDK-based chopping demo.

Usage::

    PYTHONPATH="src:src/twin_core:src/twin_description:src/twin_mujoco" \\
        python3 -m twin_control.chopping_cli --cycles 5 --viewer --log /tmp/chop.csv
"""

from __future__ import annotations

import argparse
from pathlib import Path

from twin_control.chopping import ChoppingConfig, TwinRobotChopper


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="twin-chop-sdk")
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--force", type=float, default=10.0)
    parser.add_argument("--hold", type=float, default=0.15)
    parser.add_argument("--control-hz", type=float, default=ChoppingConfig().control_hz)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--log", type=Path, default=None)
    args = parser.parse_args(argv)

    config = ChoppingConfig(
        cycles=args.cycles,
        target_force_n=args.force,
        force_hold_s=args.hold,
        control_hz=args.control_hz,
    )

    chopper = TwinRobotChopper()
    chopper.run(config, log_path=args.log, headless=args.headless or not args.viewer)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
