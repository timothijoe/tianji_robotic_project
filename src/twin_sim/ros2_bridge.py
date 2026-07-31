"""ROS-independent validation shared by the ROS 2 bridge and unit tests."""

from collections.abc import Sequence

import numpy as np


LEFT_HAND_JOINT_NAMES = tuple(
    f"left_finger{finger}_joint{joint}"
    for finger in range(1, 6)
    for joint in range(1, 5)
)


def decode_joint_command(
    names: Sequence[str],
    positions: Sequence[float],
    current_target: Sequence[float],
) -> np.ndarray:
    """Validate a JointState command and return a new flat target.

    An empty name list denotes a positional 20-joint command. Named commands
    may update any subset and leave the remaining targets unchanged.
    """
    current = np.asarray(current_target, dtype=float)
    if current.shape != (20,):
        raise ValueError("current target must contain exactly 20 values")
    try:
        values = np.asarray(positions, dtype=float)
    except (TypeError, ValueError) as error:
        raise ValueError("joint positions must be finite") from error
    if values.ndim != 1 or not np.isfinite(values).all():
        raise ValueError("joint positions must be finite")

    if not names:
        if values.shape != (20,):
            raise ValueError("positional command must contain exactly 20 positions")
        return values.copy()

    if len(names) != len(values):
        raise ValueError("joint names and positions must have the same length")
    if len(set(names)) != len(names):
        raise ValueError("joint command contains a duplicate joint name")

    indices = {name: index for index, name in enumerate(LEFT_HAND_JOINT_NAMES)}
    unknown = next((name for name in names if name not in indices), None)
    if unknown is not None:
        raise ValueError(f"unknown joint name: {unknown}")

    result = current.copy()
    for name, value in zip(names, values, strict=True):
        result[indices[name]] = value
    return result
