"""Cartesian trajectory helpers for high-level motion planning."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np


@dataclass(frozen=True)
class CartesianTrajectoryPoint:
    """One planned Cartesian target for a control cycle."""

    time_s: float
    pose_matrix: np.ndarray
    phase: Any = None
    target_force_n: float = 0.0


def minimum_jerk_scalar(ratio: float) -> float:
    """Return the normalized minimum-jerk progress for ``ratio`` in [0, 1]."""
    r = float(np.clip(ratio, 0.0, 1.0))
    return float(10.0 * r**3 - 15.0 * r**4 + 6.0 * r**5)


def cartesian_minimum_jerk_trajectory(
    start_pos: Sequence[float],
    target_pos: Sequence[float],
    rotation: np.ndarray,
    steps: int,
    phase: Any = None,
    start_time_s: float = 0.0,
    dt_s: float = 0.002,
    target_force_n: float = 0.0,
) -> list[CartesianTrajectoryPoint]:
    """Generate fixed-orientation Cartesian targets with minimum-jerk position.

    The first point is exactly ``start_pos`` and the last point is exactly
    ``target_pos``.  This makes the sequence suitable for replacing direct
    endpoint interpolation without changing the surrounding state machine.
    """
    count = int(steps)
    if count < 1:
        raise ValueError("steps must be >= 1")

    start = np.asarray(start_pos, dtype=float).reshape(3)
    target = np.asarray(target_pos, dtype=float).reshape(3)
    rot = np.asarray(rotation, dtype=float).reshape(3, 3)
    if not np.all(np.isfinite(start)):
        raise ValueError("start_pos must be finite")
    if not np.all(np.isfinite(target)):
        raise ValueError("target_pos must be finite")
    if not np.all(np.isfinite(rot)):
        raise ValueError("rotation must be finite")
    if not np.isfinite(start_time_s):
        raise ValueError("start_time_s must be finite")
    if not np.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("dt_s must be positive finite")

    points: list[CartesianTrajectoryPoint] = []
    denominator = max(count - 1, 1)
    for index in range(count):
        progress = minimum_jerk_scalar(index / denominator)
        pos = start + (target - start) * progress
        pose = np.eye(4)
        pose[:3, :3] = rot
        pose[:3, 3] = pos
        points.append(CartesianTrajectoryPoint(
            time_s=float(start_time_s + index * dt_s),
            pose_matrix=pose,
            phase=phase,
            target_force_n=float(target_force_n),
        ))
    return points
