from __future__ import annotations

import argparse
import time
from pathlib import Path

import mujoco.viewer

from twin_mujoco.chopping import ChoppingConfig, RightArmChopper


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="twin-chop")
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--force", type=float, default=10.0)
    parser.add_argument("--hold", type=float, default=0.15)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--log", type=Path, default=None)
    args = parser.parse_args(argv)

    chopper = RightArmChopper()
    config = ChoppingConfig(
        cycles=args.cycles,
        target_force_n=args.force,
        force_hold_s=args.hold,
    )
    if args.viewer:
        viewer = mujoco.viewer.launch_passive(chopper.runtime.model, chopper.runtime.data)
        try:
            _configure_viewer_camera(viewer)
            chopper.run(config, log_path=args.log, viewer_sync=viewer.sync)
        finally:
            viewer.close()
            time.sleep(0.5)
    else:
        chopper.run(config, log_path=args.log)
    return 0


def _configure_viewer_camera(viewer) -> None:
    viewer.cam.lookat[:] = (0.3, 0.0, 0.36)
    viewer.cam.distance = 1.15
    viewer.cam.azimuth = 180
    viewer.cam.elevation = -20

if __name__ == "__main__":
    raise SystemExit(main())
