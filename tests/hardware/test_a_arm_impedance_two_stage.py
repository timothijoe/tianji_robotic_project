from __future__ import annotations

import numpy as np
import pytest

from real_robot_debug.a_arm_impedance_two_stage import (
    PlayerConfig,
    build_entry_targets_deg,
    build_playback_schedule_s,
    execute_impedance_targets,
    load_offline_trajectory,
    validate_config,
)


def _candidate_npz(path, *, time_s=None, targets=None):
    np.savez(
        path,
        time_s=np.array([0.0, 0.005, 0.010]) if time_s is None else time_s,
        left_arm_target_rad=np.array(
            [[0.0] * 7, [0.1] * 7, [0.2] * 7], dtype=float
        )
        if targets is None
        else targets,
    )


def test_loads_left_arm_radians_and_converts_sdk_targets_to_degrees(tmp_path):
    source = tmp_path / "candidate.npz"
    _candidate_npz(source)

    trajectory = load_offline_trajectory(source)

    assert trajectory.time_s.shape == (3,)
    assert trajectory.target_deg.shape == (3, 7)
    assert np.array_equal(trajectory.target_deg[0], np.zeros(7))
    assert np.allclose(trajectory.target_deg[-1], np.rad2deg(np.full(7, 0.2)))


def test_rejects_nonuniform_or_nonfinite_source_trajectory(tmp_path):
    source = tmp_path / "bad_candidate.npz"
    _candidate_npz(source, time_s=np.array([0.0, 0.005, 0.011]))

    with pytest.raises(ValueError, match="uniform"):
        load_offline_trajectory(source)


def test_builds_20_second_quintic_entry_with_exact_degree_endpoints():
    start_deg = np.array([-90.0, -90.0, 90.0, -90.0, 0.0, 0.0, 0.0])
    target_deg = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0])

    entry = build_entry_targets_deg(start_deg, target_deg, duration_s=20.0, control_hz=200.0)

    assert entry.shape == (4001, 7)
    assert np.array_equal(entry[0], start_deg)
    assert np.array_equal(entry[-1], target_deg)
    assert np.allclose(np.diff(entry[[0, 1, -2, -1]], axis=0)[[0, -1]], 0.0, atol=1e-5)


def test_scales_five_millisecond_source_clock_to_tenth_speed():
    schedule = build_playback_schedule_s(np.array([0.0, 0.005, 0.010]), speed_scale=0.1)

    assert np.allclose(schedule, np.array([0.0, 0.05, 0.10]), rtol=0.0, atol=1e-12)


def test_execute_mode_refuses_without_an_independent_collision_preflight(tmp_path):
    source = tmp_path / "candidate.npz"
    _candidate_npz(source)
    config = PlayerConfig(source_npz=source, execute=True)

    with pytest.raises(ValueError, match="collision preflight"):
        validate_config(config)


def test_impedance_sender_configures_joint_mode_and_sends_each_degree_target():
    class Robot:
        def __init__(self):
            self.calls = []

        def clear_set(self):
            self.calls.append(("clear_set",))

        def set_state(self, **kwargs):
            self.calls.append(("set_state", kwargs))

        def set_impedance_type(self, **kwargs):
            self.calls.append(("set_impedance_type", kwargs))

        def set_vel_acc(self, **kwargs):
            self.calls.append(("set_vel_acc", kwargs))

        def set_joint_kd_params(self, **kwargs):
            self.calls.append(("set_joint_kd_params", kwargs))
            return True

        def set_joint_position_cmd(self, arm, joints):
            self.calls.append(("set_joint_position_cmd", arm, joints))

        def send_cmd(self):
            self.calls.append(("send_cmd",))

    robot = Robot()
    targets = np.array([[0.0] * 7, [1.0] * 7])

    execute_impedance_targets(
        robot,
        arm="A",
        targets_deg=targets,
        joint_k=[8.0] * 7,
        joint_d=[0.5] * 7,
        vel_ratio=10,
        acc_ratio=10,
    )

    assert ("set_state", {"arm": "A", "state": 3}) in robot.calls
    assert ("set_impedance_type", {"arm": "A", "type": 1}) in robot.calls
    sent = [call for call in robot.calls if call[0] == "set_joint_position_cmd"]
    assert sent == [
        ("set_joint_position_cmd", "A", [0.0] * 7),
        ("set_joint_position_cmd", "A", [1.0] * 7),
    ]
