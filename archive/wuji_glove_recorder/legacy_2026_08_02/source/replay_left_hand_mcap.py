"""Fail-closed offline preflight and optional ROS replay for left-hand MCAPs.

Importing this module never imports ROS, connects to hardware, or publishes a
command.  The ROS runtime is deliberately loaded only by the explicit armed
execution path added below.
"""

from __future__ import annotations

from dataclasses import dataclass
import argparse
import json
import math
from pathlib import Path
import sys
import time
from typing import Sequence
from xml.etree import ElementTree

import numpy as np
from mcap.reader import make_reader


CANONICAL_LEFT_JOINT_NAMES = tuple(
    f"left_finger{finger}_joint{joint}"
    for finger in range(1, 6)
    for joint in range(1, 5)
)
JOINT_STATES_TOPIC = "/joint_states"
MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "mujoco-sim"
    / "wuji_hand_description"
    / "mjcf"
    / "left.xml"
)


@dataclass(frozen=True)
class Frame:
    timestamp_ns: int
    positions_rad: np.ndarray


@dataclass(frozen=True)
class Trajectory:
    frames: tuple[Frame, ...]


@dataclass(frozen=True)
class ReplayConfig:
    path: Path
    arm: bool
    speed: float
    ramp_seconds: float
    hand_name: str
    state_timeout: float
    max_step_rad: float


def validate_replay_path(path: Path) -> Path:
    """Reject files that cannot be known converted right-to-left trajectories."""
    if "_right_to_left_wuji_hand" not in path.stem:
        raise ValueError("refusing a file not named *_right_to_left_wuji_hand.mcap")
    resolved = path.resolve()
    if not resolved.is_file() or resolved.suffix != ".mcap":
        raise ValueError(f"expected an existing .mcap file: {resolved}")
    return resolved


def normalize_joint_state(names: Sequence[str], positions: Sequence[float]) -> np.ndarray:
    """Return 20 named left-hand positions in firmware finger-major order."""
    if len(names) != 20 or len(positions) != 20 or len(set(names)) != 20:
        raise ValueError("expected 20 unique named left-hand joints")
    by_name = dict(zip(names, positions))
    if set(by_name) != set(CANONICAL_LEFT_JOINT_NAMES):
        raise ValueError("joint names do not match the 20 canonical left-hand joints")
    result = np.asarray([by_name[name] for name in CANONICAL_LEFT_JOINT_NAMES], dtype=float)
    if not np.isfinite(result).all():
        raise ValueError("joint positions must be finite")
    return result


def joint_limits_from_mjcf(path: Path = MODEL_PATH) -> dict[str, tuple[float, float]]:
    """Read exactly the 20 native left-hand position ranges from the MJCF."""
    root = ElementTree.parse(path).getroot()
    limits: dict[str, tuple[float, float]] = {}
    for joint in root.findall(".//joint"):
        name, raw_range = joint.get("name"), joint.get("range")
        if name and raw_range:
            lower, upper = map(float, raw_range.split())
            limits[f"left_{name}"] = (lower, upper)
    if set(limits) != set(CANONICAL_LEFT_JOINT_NAMES):
        raise ValueError("left MJCF does not define exactly 20 canonical joint ranges")
    return limits


def load_validated_trajectory(path: Path, max_step_rad: float) -> Trajectory:
    """Load only a complete, finite, bounded converted left-hand trajectory."""
    if max_step_rad <= 0:
        raise ValueError("max_step_rad must be positive")
    source = validate_replay_path(path)
    limits = joint_limits_from_mjcf()
    frames: list[Frame] = []
    previous_timestamp: int | None = None
    previous_positions: np.ndarray | None = None
    with source.open("rb") as stream:
        reader = make_reader(stream)
        for _, channel, message in reader.iter_messages(topics=[JOINT_STATES_TOPIC]):
            if channel.topic != JOINT_STATES_TOPIC:
                continue
            payload = json.loads(message.data)
            positions = normalize_joint_state(payload.get("name", []), payload.get("position", []))
            timestamp_ns = int(message.log_time)
            if previous_timestamp is not None and timestamp_ns <= previous_timestamp:
                raise ValueError("MCAP timestamps must be strictly increasing")
            for name, value in zip(CANONICAL_LEFT_JOINT_NAMES, positions):
                lower, upper = limits[name]
                if value < lower or value > upper:
                    raise ValueError(f"{name} is outside MJCF range [{lower}, {upper}]")
            if previous_positions is not None and np.max(np.abs(positions - previous_positions)) > max_step_rad:
                raise ValueError("interframe step exceeds max_step_rad")
            frames.append(Frame(timestamp_ns=timestamp_ns, positions_rad=positions))
            previous_timestamp, previous_positions = timestamp_ns, positions
    if not frames:
        raise ValueError(f"no {JOINT_STATES_TOPIC} frames found in {source}")
    return Trajectory(frames=tuple(frames))


def parse_args(argv: Sequence[str] | None = None) -> ReplayConfig:
    """Parse only conservative replay settings; --arm remains opt-in."""
    parser = argparse.ArgumentParser(description="Preflight or replay a converted left-hand MCAP.")
    parser.add_argument("mcap", type=Path)
    parser.add_argument("--arm", action="store_true", help="allow physical ROS command publication")
    parser.add_argument("--speed", type=float, default=0.1)
    parser.add_argument("--ramp-seconds", type=float, default=3.0)
    parser.add_argument("--hand-name", default="hand_0")
    parser.add_argument("--state-timeout", type=float, default=5.0)
    parser.add_argument("--max-step-rad", type=float, default=0.08)
    args = parser.parse_args(argv)
    if not 0 < args.speed <= 1:
        parser.error("--speed must be in (0, 1]")
    if args.ramp_seconds <= 0 or args.state_timeout <= 0 or args.max_step_rad <= 0:
        parser.error("ramp-seconds, state-timeout, and max-step-rad must be positive")
    if not args.hand_name.strip():
        parser.error("--hand-name must not be blank")
    return ReplayConfig(args.mcap, args.arm, args.speed, args.ramp_seconds,
                        args.hand_name, args.state_timeout, args.max_step_rad)


def ramp_positions(start: np.ndarray, goal: np.ndarray, duration_s: float,
                   rate_hz: float, max_step_rad: float) -> tuple[np.ndarray, ...]:
    """Create a fixed-rate ramp whose adjacent positions never exceed the limit."""
    start = np.asarray(start, dtype=float)
    goal = np.asarray(goal, dtype=float)
    if start.shape != (20,) or goal.shape != (20,) or duration_s <= 0 or rate_hz <= 0 or max_step_rad <= 0:
        raise ValueError("invalid ramp inputs")
    steps_by_time = max(1, math.ceil(duration_s * rate_hz))
    steps_by_delta = max(1, math.ceil(float(np.max(np.abs(goal - start))) / max_step_rad))
    steps = max(steps_by_time, steps_by_delta)
    return tuple(start + (goal - start) * (index / steps) for index in range(steps + 1))


def preflight_report(trajectory: Trajectory) -> str:
    """Render the complete offline safety summary before any hardware action."""
    values = np.asarray([frame.positions_rad for frame in trajectory.frames])
    duration = (trajectory.frames[-1].timestamp_ns - trajectory.frames[0].timestamp_ns) / 1e9
    delta = 0.0 if len(values) == 1 else float(np.max(np.abs(np.diff(values, axis=0))))
    lines = [f"Preflight passed: {len(values)} frames, {duration:.3f} s, max frame step {delta:.5f} rad.",
             "Joint ranges (rad):"]
    lines.extend(
        f"  {name}: {low:.4f} .. {high:.4f}"
        for name, low, high in zip(CANONICAL_LEFT_JOINT_NAMES, values.min(axis=0), values.max(axis=0))
    )
    lines.append("First frame: " + np.array2string(values[0], precision=4, separator=", "))
    return "\n".join(lines)


def run_offline(config: ReplayConfig) -> int:
    """Execute file-only validation and reporting with no ROS dependency."""
    trajectory = load_validated_trajectory(config.path, config.max_step_rad)
    print(preflight_report(trajectory))
    print("Preflight passed. Add --arm only when the physical hand is ready.")
    return 0


def _joint_state_message(joint_state_type, positions: np.ndarray):
    message = joint_state_type()
    message.name = list(CANONICAL_LEFT_JOINT_NAMES)
    message.position = np.asarray(positions, dtype=float).tolist()
    return message


def run_armed(config: ReplayConfig, trajectory: Trajectory) -> int:
    """Replay only after validated preflight and explicit arming by the operator."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = Node("left_hand_mcap_replay")
    latest_state: np.ndarray | None = None
    last_feedback_time = 0.0

    def on_state(message: JointState) -> None:
        nonlocal latest_state, last_feedback_time
        values = np.asarray(message.position, dtype=float)
        if values.shape == (20,) and np.isfinite(values).all():
            latest_state = values
            last_feedback_time = time.monotonic()

    try:
        publisher = node.create_publisher(JointState, f"/{config.hand_name}/joint_commands", qos_profile_sensor_data)
        node.create_subscription(JointState, f"/{config.hand_name}/joint_states", on_state, qos_profile_sensor_data)
        deadline = time.monotonic() + config.state_timeout
        while latest_state is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if latest_state is None:
            raise RuntimeError(f"no valid /{config.hand_name}/joint_states before timeout")

        last_command = latest_state
        for target in ramp_positions(latest_state, trajectory.frames[0].positions_rad,
                                     config.ramp_seconds, 50.0, config.max_step_rad)[1:]:
            if time.monotonic() - last_feedback_time > 1.0:
                raise RuntimeError("joint-state feedback was lost")
            publisher.publish(_joint_state_message(JointState, target))
            last_command = target
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(1.0 / 50.0)

        for previous, frame in zip(trajectory.frames, trajectory.frames[1:]):
            interval = max((frame.timestamp_ns - previous.timestamp_ns) / 1e9 / config.speed, 1e-4)
            time.sleep(interval)
            rclpy.spin_once(node, timeout_sec=0.0)
            if time.monotonic() - last_feedback_time > 1.0:
                raise RuntimeError("joint-state feedback was lost")
            if np.max(np.abs(frame.positions_rad - last_command)) > config.max_step_rad:
                raise RuntimeError("runtime command step exceeds max_step_rad")
            publisher.publish(_joint_state_message(JointState, frame.positions_rad))
            last_command = frame.positions_rad
        print("Replay completed; no additional pose was published.")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: Sequence[str] | None = None) -> int:
    config = parse_args(argv)
    trajectory = load_validated_trajectory(config.path, config.max_step_rad)
    print(preflight_report(trajectory))
    if not config.arm:
        print("Preflight passed. Add --arm only when the physical hand is ready.")
        return 0
    return run_armed(config, trajectory)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("Stopped; no additional pose was published.")
        raise SystemExit(130)
    except (ValueError, RuntimeError) as error:
        print(f"Replay refused: {error}", file=sys.stderr)
        raise SystemExit(2)
