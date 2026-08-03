from pathlib import Path
import json
import sys
import subprocess

import numpy as np
import pytest
from mcap.writer import Writer

from replay_left_hand_mcap import (
    CANONICAL_LEFT_JOINT_NAMES,
    load_validated_trajectory,
    normalize_joint_state,
    parse_args,
    ramp_positions,
    ReplayConfig,
    run_offline,
    validate_replay_path,
)


LAUNCHER = Path(__file__).parents[1] / "replay_left_hand_mcap.sh"


def write_joint_state_mcap(path: Path, frames, topic: str = "/joint_states") -> Path:
    with path.open("wb") as stream:
        writer = Writer(stream)
        writer.start(profile="ros2", library="test")
        schema_id = writer.register_schema("sensor_msgs/msg/JointState", "jsonschema", b"{}")
        channel_id = writer.register_channel(topic, "json", schema_id)
        for sequence, (timestamp_ns, names, positions) in enumerate(frames):
            writer.add_message(
                channel_id,
                log_time=timestamp_ns,
                publish_time=timestamp_ns,
                sequence=sequence,
                data=json.dumps({"name": list(names), "position": list(positions)}).encode(),
            )
        writer.finish()
    return path


def test_normalize_joint_state_reorders_named_left_joints():
    names = list(reversed(CANONICAL_LEFT_JOINT_NAMES))
    positions = list(range(20))

    normalized = normalize_joint_state(names, positions)

    assert np.array_equal(normalized, np.arange(19, -1, -1, dtype=float))


@pytest.mark.parametrize("path", [Path("capture.mcap"), Path("right_to_left.mcap")])
def test_replay_filename_must_identify_right_to_left_output(path: Path):
    with pytest.raises(ValueError, match="right_to_left_wuji_hand"):
        validate_replay_path(path)


def test_load_validated_trajectory_reorders_two_valid_frames(tmp_path: Path):
    path = write_joint_state_mcap(
        tmp_path / "case_right_to_left_wuji_hand.mcap",
        [
            (100, tuple(reversed(CANONICAL_LEFT_JOINT_NAMES)), np.zeros(20)),
            (200, tuple(reversed(CANONICAL_LEFT_JOINT_NAMES)), np.full(20, 0.01)),
        ],
    )

    trajectory = load_validated_trajectory(path, max_step_rad=0.08)

    assert len(trajectory.frames) == 2
    assert np.array_equal(trajectory.frames[0].positions_rad, np.zeros(20))


def test_load_validated_trajectory_rejects_a_joint_outside_mjcf_range(tmp_path: Path):
    path = write_joint_state_mcap(
        tmp_path / "case_right_to_left_wuji_hand.mcap",
        [(100, CANONICAL_LEFT_JOINT_NAMES, np.full(20, 99.0))],
    )

    with pytest.raises(ValueError, match="outside MJCF range"):
        load_validated_trajectory(path, max_step_rad=0.08)


def test_load_validated_trajectory_rejects_a_large_interframe_step(tmp_path: Path):
    path = write_joint_state_mcap(
        tmp_path / "case_right_to_left_wuji_hand.mcap",
        [
            (100, CANONICAL_LEFT_JOINT_NAMES, np.zeros(20)),
            (200, CANONICAL_LEFT_JOINT_NAMES, np.full(20, 0.1)),
        ],
    )

    with pytest.raises(ValueError, match="interframe step"):
        load_validated_trajectory(path, max_step_rad=0.08)


def test_parse_args_defaults_to_offline_ten_percent_replay(tmp_path: Path):
    config = parse_args([str(tmp_path / "sample_right_to_left_wuji_hand.mcap")])

    assert config.arm is False
    assert config.speed == 0.1
    assert config.ramp_seconds == 3.0
    assert config.hand_name == "hand_0"


def test_ramp_has_exact_endpoints_and_bounded_steps():
    frames = ramp_positions(
        np.zeros(20), np.full(20, 0.3), duration_s=1.0, rate_hz=50.0, max_step_rad=0.08
    )

    assert np.array_equal(frames[0], np.zeros(20))
    assert np.array_equal(frames[-1], np.full(20, 0.3))
    assert np.abs(np.diff(np.asarray(frames), axis=0)).max() <= 0.08


def test_run_offline_never_imports_rclpy(monkeypatch, tmp_path: Path):
    path = write_joint_state_mcap(
        tmp_path / "case_right_to_left_wuji_hand.mcap",
        [(100, CANONICAL_LEFT_JOINT_NAMES, np.zeros(20))],
    )
    monkeypatch.setitem(sys.modules, "rclpy", None)
    config = ReplayConfig(path, False, 0.1, 3.0, "hand_0", 5.0, 0.08)

    assert run_offline(config) == 0


def test_launcher_dry_run_preserves_ros_pythonpath_and_calls_replayer(tmp_path: Path):
    source = tmp_path / "sample_right_to_left_wuji_hand.mcap"
    source.touch()

    result = subprocess.run(
        ["sh", str(LAUNCHER), "--dry-run", str(source)],
        capture_output=True,
        text=True,
        check=True,
    )

    assert "replay_left_hand_mcap.py" in result.stdout
    assert "PYTHONPATH" in result.stdout
