import numpy as np
import pytest

from twin_sim.ros2_bridge import LEFT_HAND_JOINT_NAMES, decode_joint_command


def test_positional_command_requires_all_twenty_positions():
    current = np.zeros(20)
    command = np.linspace(0.0, 0.19, 20)
    np.testing.assert_array_equal(
        decode_joint_command([], command, current), command
    )
    with pytest.raises(ValueError, match="exactly 20"):
        decode_joint_command([], command[:-1], current)


def test_named_command_updates_only_named_joints():
    current = np.arange(20, dtype=float)
    result = decode_joint_command(
        [LEFT_HAND_JOINT_NAMES[7], LEFT_HAND_JOINT_NAMES[0]],
        [0.7, 0.1],
        current,
    )
    expected = current.copy()
    expected[[7, 0]] = [0.7, 0.1]
    np.testing.assert_array_equal(result, expected)


@pytest.mark.parametrize(
    ("names", "positions", "message"),
    (
        (["unknown"], [0.1], "unknown joint"),
        ([LEFT_HAND_JOINT_NAMES[0]] * 2, [0.1, 0.2], "duplicate"),
        ([LEFT_HAND_JOINT_NAMES[0]], [], "same length"),
        ([LEFT_HAND_JOINT_NAMES[0]], [np.nan], "finite"),
    ),
)
def test_invalid_named_command_is_rejected_atomically(names, positions, message):
    current = np.arange(20, dtype=float)
    before = current.copy()
    with pytest.raises(ValueError, match=message):
        decode_joint_command(names, positions, current)
    np.testing.assert_array_equal(current, before)
