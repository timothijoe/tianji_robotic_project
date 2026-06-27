from __future__ import annotations

from typing import Sequence

import numpy as np


def finite_vector(values: Sequence[float], size: int, label: str) -> np.ndarray:
    result = np.asarray(values, dtype=float)
    if result.ndim != 1 or result.shape != (int(size),) or not np.all(np.isfinite(result)):
        raise ValueError(f"{label} must contain {int(size)} finite values")
    return result


def validate_waypoints(waypoints: Sequence[Sequence[float]], dof: int) -> np.ndarray:
    result = np.asarray(waypoints, dtype=float)
    if result.ndim != 2 or result.shape[1] != int(dof) or not np.all(np.isfinite(result)):
        raise ValueError(f"waypoints must be a finite N x {int(dof)} array")
    if result.shape[0] == 0:
        raise ValueError("waypoints must contain at least one row")
    return result
