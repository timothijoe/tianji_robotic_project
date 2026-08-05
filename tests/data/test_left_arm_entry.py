import numpy as np
import pytest

from tianji_robotics.data.left_arm_entry import generate_left_arm_entry


def test_generates_200hz_20_second_quintic_entry_with_exact_endpoints():
    start = np.deg2rad(np.array([-90, -90, 90, -90, 0, 0, 0], dtype=float))
    target = np.deg2rad(
        np.array([71.431, -66.107, -49.733, -128.492, 94.122, 45.648, -49.911])
    )

    result = generate_left_arm_entry(start, target)

    assert result.time_s.shape == (4001,)
    assert result.left_arm_target_rad.shape == (4001, 7)
    assert result.time_s[0] == 0.0
    assert result.time_s[-1] == 20.0
    assert np.allclose(np.diff(result.time_s), 0.005)
    assert np.array_equal(result.left_arm_target_rad[0], start)
    assert np.array_equal(result.left_arm_target_rad[-1], target)


@pytest.mark.parametrize("bad", [np.zeros(6), np.full(7, np.nan)])
def test_rejects_nonfinite_or_nonseven_joint_vector(bad):
    with pytest.raises(ValueError, match="seven finite"):
        generate_left_arm_entry(bad, np.zeros(7))


def test_has_near_zero_boundary_velocity_and_acceleration():
    result = generate_left_arm_entry(
        np.zeros(7), np.ones(7), duration_s=2.0, sample_rate_hz=200.0
    )

    velocity = np.gradient(result.left_arm_target_rad, result.time_s, axis=0)
    acceleration = np.gradient(velocity, result.time_s, axis=0)

    assert np.all(np.abs(velocity[[0, -1]]) < 2e-4)
    assert np.all(np.abs(acceleration[[0, -1]]) < 0.05)
