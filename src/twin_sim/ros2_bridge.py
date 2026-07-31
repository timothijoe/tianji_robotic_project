"""ROS-independent validation shared by the ROS 2 bridge and unit tests."""

from collections.abc import Sequence

import numpy as np


LEFT_HAND_JOINT_NAMES = tuple(
    f"left_finger{finger}_joint{joint}"
    for finger in range(1, 6)
    for joint in range(1, 5)
)


def control_period(publish_rate: float, timestep: float) -> float:
    rate = float(publish_rate)
    step = float(timestep)
    if not np.isfinite(rate) or rate <= 0.0:
        raise ValueError("publish_rate must be positive and finite")
    period = 1.0 / rate
    ratio = period / step
    if round(ratio) < 1 or not np.isclose(ratio, round(ratio), rtol=0.0, atol=1e-10):
        raise ValueError(
            "publish period must be a positive integer multiple of MuJoCo timestep"
        )
    return round(ratio) * step


def enabled_indices(finger_id: int, joint_id: int) -> tuple[int, ...]:
    """Resolve exactly the selector combinations supported by upstream."""
    if finger_id == 255 and joint_id == 255:
        return tuple(range(20))
    if not 0 <= finger_id < 5:
        raise ValueError("finger_id must be 0-4, or 255 when joint_id is also 255")
    if joint_id == 255:
        start = finger_id * 4
        return tuple(range(start, start + 4))
    if not 0 <= joint_id < 4:
        raise ValueError("joint_id must be 0-3 or 255")
    return (finger_id * 4 + joint_id,)


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
