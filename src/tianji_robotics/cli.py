"""Top-level command line boundary for simulation and real hardware."""

import argparse
from pathlib import Path
import sys


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="tianji-robot")
    domains = parser.add_subparsers(dest="domain", required=True)

    sim = domains.add_parser("sim", help="offline and simulated execution")
    sim_commands = sim.add_subparsers(dest="sim_command", required=True)
    replay = sim_commands.add_parser("wuji-replay", help="replay a Wuji Studio recording")
    replay.add_argument("source", type=Path)
    replay.add_argument("--headless", action="store_true")
    replay.add_argument("--npz", type=Path)
    replay.add_argument("--joint-state-mcap", type=Path)
    replay.set_defaults(handler=_run_wuji_replay)

    hardware = domains.add_parser("hardware", help="guarded real-device tools")
    vendors = hardware.add_subparsers(dest="hardware_vendor", required=True)
    wuji = vendors.add_parser("wuji-sdk", help="Wuji SDK boundary")
    wuji_commands = wuji.add_subparsers(dest="wuji_command", required=True)
    preflight = wuji_commands.add_parser("preflight", help="validate an offline NPZ only")
    preflight.add_argument("trajectory", type=Path)
    preflight.set_defaults(handler=lambda args: _preflight_trajectory(args.trajectory))
    return parser


def _run_wuji_replay(args: argparse.Namespace) -> int:
    try:
        from tianji_robotics.data.mcap import StudioMcapSkeletonSource, write_joint_state_mcap
        from tianji_robotics.data.npz import save_trajectory_npz
        from tianji_robotics.simulation.replay import replay_trajectory
        from tianji_robotics.simulation.wuji_hand import MujocoWujiHand
        from tianji_robotics.workflows.wuji_glove_replay import retarget_recording
        from tianji_robotics.wuji_sdk.retargeter import OfficialWujiRetargeter

        trajectory = retarget_recording(
            StudioMcapSkeletonSource(args.source),
            OfficialWujiRetargeter.create_left_first_generation(),
        )
        if args.npz:
            save_trajectory_npz(trajectory, args.npz)
        if args.joint_state_mcap:
            write_joint_state_mcap(trajectory, args.joint_state_mcap)
        backend = MujocoWujiHand(viewer=not args.headless)
        try:
            summary = replay_trajectory(trajectory, backend, realtime=not args.headless)
        finally:
            backend.close()
        print(f"replayed {summary.frame_count} frames ({summary.duration_s:.3f}s)")
        return 0
    except ModuleNotFoundError as exc:
        if exc.name in {"mcap", "wuji_sdk"}:
            print(
                "missing Wuji offline dependency; install with "
                "'pip install -e .[wuji-offline]'",
                file=sys.stderr,
            )
            return 2
        raise


def _preflight_trajectory(path: Path) -> int:
    from tianji_robotics.data.npz import load_trajectory_npz

    trajectory = load_trajectory_npz(path)
    duration_s = float(trajectory.timestamps_ns[-1] - trajectory.timestamps_ns[0]) / 1e9
    print(f"trajectory valid: {trajectory.positions_rad.shape[0]} frames, {duration_s:.3f}s; no device accessed")
    return 0


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    return int(args.handler(args))
