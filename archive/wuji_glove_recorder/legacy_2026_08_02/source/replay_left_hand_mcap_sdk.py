"""Fail-closed, SDK-direct replay for converted left Wuji Hand MCAP files."""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import os
from pathlib import Path
import subprocess
import time
from typing import Sequence
from types import SimpleNamespace

import numpy as np

from replay_left_hand_mcap import load_validated_trajectory, preflight_report, ramp_positions


@dataclass(frozen=True)
class SdkReplayConfig:
    path: Path
    arm: bool
    speed: float
    ramp_seconds: float
    max_step_rad: float
    effort_limit: float
    cutoff_hz: float


def parse_args(argv: Sequence[str] | None = None) -> SdkReplayConfig:
    parser = argparse.ArgumentParser(description="SDK-direct replay of a converted left-hand MCAP.")
    parser.add_argument("mcap", type=Path)
    parser.add_argument("--arm", action="store_true")
    parser.add_argument("--speed", type=float, default=0.1)
    parser.add_argument("--ramp-seconds", type=float, default=3.0)
    parser.add_argument("--max-step-rad", type=float, default=0.08)
    parser.add_argument("--effort-limit", type=float, default=1.5)
    parser.add_argument("--cutoff-hz", type=float, default=5.0)
    a = parser.parse_args(argv)
    if not 0 < a.speed <= 1 or min(a.ramp_seconds, a.max_step_rad, a.effort_limit, a.cutoff_hz) <= 0 or a.effort_limit > 1.5 or a.cutoff_hz > 20:
        parser.error("invalid safety parameter")
    return SdkReplayConfig(a.mcap, a.arm, a.speed, a.ramp_seconds, a.max_step_rad, a.effort_limit, a.cutoff_hz)


def assert_sdk_control_available(node_names: Sequence[str], process_lines: Sequence[str]) -> None:
    if any(name.endswith("/wujihand_driver") for name in node_names) or any("wujihand_driver_node" in line or "replay_left_hand_mcap_sdk.py" in line for line in process_lines):
        raise RuntimeError("stop ROS driver and other SDK replay processes before SDK control")


def exclude_current_process(process_lines: Sequence[str], pid: int | None = None) -> list[str]:
    """Remove this replay process from pgrep output before conflict checking."""
    current_pid = str(os.getpid() if pid is None else pid)
    return [line for line in process_lines if line.split(maxsplit=1)[0] != current_pid]


def make_sdk_symbols(DeviceType, JointCommand, LowPass) -> SimpleNamespace:
    """Bundle SDK classes without relying on a function-local class body."""
    return SimpleNamespace(DeviceType=DeviceType, JointCommand=JointCommand, LowPass=LowPass)


def is_left_handedness(value: str) -> bool:
    """Match the exact handedness spelling declared by wuji_sdk's type stub."""
    return value == "Left"


def _system_lines(command: list[str]) -> list[str]:
    result = subprocess.run(command, capture_output=True, text=True, check=False)
    return result.stdout.splitlines()


def _send(publisher, JointCommand, positions: np.ndarray) -> None:
    publisher.send([JointCommand(float(value), 0.0, 0.0) for value in positions])


def run_sdk_replay(config: SdkReplayConfig, trajectory, manager, sdk, sleep=time.sleep) -> int:
    devices = [d for d in manager.scan() if d.device_type == sdk.DeviceType.WujiHand]
    if len(devices) != 1:
        raise RuntimeError(f"expected exactly one first-generation WujiHand, found {len(devices)}")
    hand = manager.connect(sn=devices[0].sn, device_name="wuji_hand_mcap_replay")
    publisher = None
    enabled = False
    try:
        if not is_left_handedness(hand.handedness_name()):
            raise RuntimeError("SDK connected device is not the left hand")
        start = np.asarray(hand.read_joint_state().position, dtype=float)
        if start.shape != (20,) or not np.isfinite(start).all():
            raise RuntimeError("SDK did not return 20 finite joint positions")
        hand.set_all_effort_limit(config.effort_limit)
        hand.enable(); enabled = True
        with hand.realtime_controller(sdk.LowPass(cutoff_hz=config.cutoff_hz)):
            publisher = hand.joint_command().publish()
            for target in ramp_positions(start, trajectory.frames[0].positions_rad, config.ramp_seconds, 100.0, config.max_step_rad)[1:]:
                _send(publisher, sdk.JointCommand, target); sleep(0.01)
            previous = trajectory.frames[0]
            _send(publisher, sdk.JointCommand, previous.positions_rad)
            for frame in trajectory.frames[1:]:
                interval = max((frame.timestamp_ns - previous.timestamp_ns) / 1e9 / config.speed, 1e-4)
                steps = max(1, math.ceil(interval * 100))
                for i in range(1, steps + 1):
                    _send(publisher, sdk.JointCommand, previous.positions_rad + (frame.positions_rad - previous.positions_rad) * (i / steps)); sleep(interval / steps)
                previous = frame
        return 0
    finally:
        if publisher is not None: publisher.close()
        if enabled:
            try: hand.disable()
            except Exception: pass
        manager.disconnect_all()


def main(argv: Sequence[str] | None = None) -> int:
    config = parse_args(argv)
    trajectory = load_validated_trajectory(config.path, config.max_step_rad)
    print(preflight_report(trajectory))
    if not config.arm:
        print("Preflight passed. SDK was not loaded; add --arm only after ROS driver is stopped.")
        return 0
    process_lines = exclude_current_process(
        _system_lines(["pgrep", "-af", "[w]ujihand_driver_node|[r]eplay_left_hand_mcap_sdk.py"])
    )
    assert_sdk_control_available(_system_lines(["ros2", "node", "list"]), process_lines)
    from wuji_sdk import DeviceType, JointCommand, LowPass, SdkManager
    return run_sdk_replay(config, trajectory, SdkManager.instance(), make_sdk_symbols(DeviceType, JointCommand, LowPass))


if __name__ == "__main__":
    raise SystemExit(main())
