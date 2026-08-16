#!/usr/bin/env python3
"""Conservative base-frame keyboard Cartesian jog for one physical robot arm."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


KEY_DIRECTIONS: dict[str, tuple[float, float, float]] = {
    "w": (1.0, 0.0, 0.0),
    "s": (-1.0, 0.0, 0.0),
    "a": (0.0, 1.0, 0.0),
    "d": (0.0, -1.0, 0.0),
    "r": (0.0, 0.0, 1.0),
    "f": (0.0, 0.0, -1.0),
}


@dataclass(frozen=True)
class JogConfig:
    """Safety-critical configuration for a keyboard jog session."""

    arm: str = "A"
    step_mm: float = 2.0
    execute: bool = False
    workspace_min: tuple[float, float, float] | None = None
    workspace_max: tuple[float, float, float] | None = None
    vel_ratio: int = 10
    acc_ratio: int = 10
    keep_enabled: bool = False


def validate_config(config: JogConfig) -> None:
    """Reject invalid motion limits before any SDK connection is opened."""
    if config.arm not in ("A", "B"):
        raise ValueError("arm must be 'A' or 'B'")
    if not math.isfinite(float(config.step_mm)) or not 0.0 < float(config.step_mm) <= 5.0:
        raise ValueError("step-mm must be in (0, 5]")
    if not (0 <= int(config.vel_ratio) <= 100):
        raise ValueError("vel-ratio must be in [0, 100]")
    if not (0 <= int(config.acc_ratio) <= 100):
        raise ValueError("acc-ratio must be in [0, 100]")
    if config.execute and (config.workspace_min is None or config.workspace_max is None):
        raise ValueError("--execute requires both --workspace-min and --workspace-max")
    if (config.workspace_min is None) != (config.workspace_max is None):
        raise ValueError("workspace-min and workspace-max must be supplied together")
    if config.workspace_min is not None and config.workspace_max is not None:
        lower = np.asarray(config.workspace_min, dtype=float)
        upper = np.asarray(config.workspace_max, dtype=float)
        if lower.shape != (3,) or upper.shape != (3,) or not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)):
            raise ValueError("workspace bounds must contain 3 finite values")
        if not np.all(lower < upper):
            raise ValueError("workspace-min must be strictly below workspace-max")


def inside_workspace(
    point_xyz: np.ndarray,
    lower_xyz: tuple[float, float, float],
    upper_xyz: tuple[float, float, float],
) -> bool:
    """Return whether a point lies within an inclusive Cartesian workspace box."""
    point = np.asarray(point_xyz, dtype=float)
    lower = np.asarray(lower_xyz, dtype=float)
    upper = np.asarray(upper_xyz, dtype=float)
    return bool(point.shape == (3,) and np.all(point >= lower) and np.all(point <= upper))


def key_to_delta(key: str, step_mm: float) -> tuple[float, float, float] | None:
    """Map a movement key to one base-frame Cartesian step in millimetres."""
    direction = KEY_DIRECTIONS.get(str(key).lower())
    if direction is None:
        return None
    return tuple(float(step_mm) * axis for axis in direction)


def candidate_pose(current_xyzabc: np.ndarray, delta_xyz: tuple[float, float, float]) -> np.ndarray:
    """Translate XYZ while retaining the supplied TCP ABC orientation."""
    target = np.asarray(current_xyzabc, dtype=float).copy()
    if target.shape != (6,):
        raise ValueError("current_xyzabc must contain exactly 6 values")
    target[:3] += np.asarray(delta_xyz, dtype=float)
    return target
