"""Immutable, validated data crossing the Wuji hand domain boundary."""

from dataclasses import dataclass
from types import MappingProxyType
from typing import Literal, Mapping

import numpy as np
from numpy.typing import NDArray

from .names import HAND_JOINT_NAMES


def _copied_readonly_array(value: object) -> NDArray[np.generic]:
    result = np.array(value, copy=True)
    result.setflags(write=False)
    return result


@dataclass(frozen=True)
class SkeletonFrame:
    timestamp_ns: int
    frame_id: str
    side: Literal["left", "right"]
    keypoints_m: NDArray[np.floating]

    def __post_init__(self) -> None:
        if not self.frame_id:
            raise ValueError("frame_id must be non-empty")
        if self.side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        keypoints = _copied_readonly_array(self.keypoints_m)
        if keypoints.shape != (21, 3):
            raise ValueError("keypoints_m must have shape (21, 3)")
        if not np.issubdtype(keypoints.dtype, np.number) or not np.isfinite(keypoints).all():
            raise ValueError("keypoints_m must be finite")
        object.__setattr__(self, "keypoints_m", keypoints)


@dataclass(frozen=True)
class HandTrajectory:
    timestamps_ns: NDArray[np.integer]
    positions_rad: NDArray[np.floating]
    joint_names: tuple[str, ...]
    metadata: Mapping[str, object]

    def __post_init__(self) -> None:
        timestamps = _copied_readonly_array(self.timestamps_ns)
        positions = _copied_readonly_array(self.positions_rad)
        joint_names = tuple(self.joint_names)
        if timestamps.ndim != 1 or timestamps.size == 0:
            raise ValueError("timestamps_ns must be a non-empty one-dimensional array")
        if not np.issubdtype(timestamps.dtype, np.integer):
            raise ValueError("timestamps_ns must contain integers")
        if positions.shape != (timestamps.size, 20):
            raise ValueError("positions_rad must have shape (N, 20)")
        if not np.issubdtype(positions.dtype, np.number) or not np.isfinite(positions).all():
            raise ValueError("positions_rad must be finite")
        if not np.all(np.diff(timestamps) > 0):
            raise ValueError("timestamps_ns must be strictly increasing")
        if joint_names != HAND_JOINT_NAMES or len(set(joint_names)) != 20:
            raise ValueError("joint_names must be the 20 unique canonical joint names")
        object.__setattr__(self, "timestamps_ns", timestamps)
        object.__setattr__(self, "positions_rad", positions)
        object.__setattr__(self, "joint_names", joint_names)
        object.__setattr__(self, "metadata", MappingProxyType(dict(self.metadata)))
