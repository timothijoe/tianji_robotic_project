import numpy as np
import pytest

from mujoco_left_replay import clamp_frame, load_trajectory


def test_load_trajectory_accepts_20_joint_data(tmp_path):
    path = tmp_path / "left.npz"
    np.savez(
        path,
        timestamps_ns=[10, 20],
        left_joint_positions_rad=np.zeros((2, 20)),
    )

    trajectory = load_trajectory(path)

    assert trajectory.positions_rad.shape == (2, 20)


def test_load_trajectory_rejects_nonfinite_joint_data(tmp_path):
    path = tmp_path / "bad.npz"
    np.savez(
        path,
        timestamps_ns=[10],
        left_joint_positions_rad=np.full((1, 20), np.nan),
    )

    with pytest.raises(ValueError, match="finite"):
        load_trajectory(path)


def test_clamp_frame_stays_inside_trajectory():
    assert clamp_frame(-1, 4) == 0
    assert clamp_frame(8, 4) == 3


def test_load_trajectory_rejects_mismatched_timestamps(tmp_path):
    path = tmp_path / "bad-time.npz"
    np.savez(
        path,
        timestamps_ns=[10],
        left_joint_positions_rad=np.zeros((2, 20)),
    )

    with pytest.raises(ValueError, match="one timestamp"):
        load_trajectory(path)
