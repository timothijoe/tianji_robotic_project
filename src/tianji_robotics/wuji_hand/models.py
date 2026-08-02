"""Immutable, validated data crossing the Wuji hand domain boundary."""

from dataclasses import dataclass
from types import MappingProxyType
from collections.abc import Mapping
from typing import Literal

import numpy as np
from numpy.typing import NDArray

from .names import HAND_JOINT_NAMES


def _copied_readonly_array(value: object) -> NDArray[np.generic]:
    """Copy into an ndarray backed by immutable bytes.

    A normal non-writeable ndarray can be made writeable again by callers when it
    owns its memory. A bytes-backed array cannot, which preserves value-object
    immutability while retaining the public ndarray API.
    """
    normalized = np.array(value, copy=True)
    return np.frombuffer(normalized.tobytes(), dtype=normalized.dtype).reshape(normalized.shape)


def _is_real_numeric_dtype(dtype: np.dtype[np.generic]) -> bool:
    return np.issubdtype(dtype, np.number) and not np.issubdtype(dtype, np.complexfloating)


def _freeze_metadata(value: object) -> object:
    if isinstance(value, Mapping):
        frozen: dict[str, object] = {}
        for key, nested_value in value.items():
            if not isinstance(key, str):
                raise ValueError("metadata keys must be strings")
            frozen[key] = _freeze_metadata(nested_value)
        return MappingProxyType(frozen)
    if isinstance(value, (list, tuple)):
        return tuple(_freeze_metadata(item) for item in value)
    if isinstance(value, (set, frozenset)):
        return frozenset(_freeze_metadata(item) for item in value)
    return value


@dataclass(frozen=True)
class SkeletonFrame:
    timestamp_ns: int
    frame_id: str
    side: Literal["left", "right"]
    keypoints_m: NDArray[np.floating]

    def __post_init__(self) -> None:
        if (
            isinstance(self.timestamp_ns, bool)
            or not isinstance(self.timestamp_ns, (int, np.integer))
            or self.timestamp_ns < 0
        ):
            raise ValueError("timestamp_ns must be a non-negative integer")
        if not self.frame_id:
            raise ValueError("frame_id must be non-empty")
        if self.side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        keypoints = _copied_readonly_array(self.keypoints_m)
        if keypoints.shape != (21, 3):
            raise ValueError("keypoints_m must have shape (21, 3)")
        if not _is_real_numeric_dtype(keypoints.dtype):
            raise ValueError("keypoints_m must contain real numbers")
        if not np.isfinite(keypoints).all():
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
            raise ValueError("timestamps_ns must contain real integers")
        if positions.shape != (timestamps.size, 20):
            raise ValueError("positions_rad must have shape (N, 20)")
        if not _is_real_numeric_dtype(positions.dtype):
            raise ValueError("positions_rad must contain real numbers")
        if not np.isfinite(positions).all():
            raise ValueError("positions_rad must be finite")
        if not np.all(np.diff(timestamps) > 0):
            raise ValueError("timestamps_ns must be strictly increasing")
        if joint_names != HAND_JOINT_NAMES or len(set(joint_names)) != 20:
            raise ValueError("joint_names must be the 20 unique canonical joint names")
        object.__setattr__(self, "timestamps_ns", timestamps)
        object.__setattr__(self, "positions_rad", positions)
        object.__setattr__(self, "joint_names", joint_names)
        frozen_metadata = _freeze_metadata(self.metadata)
        if not isinstance(frozen_metadata, Mapping):
            raise ValueError("metadata must be a mapping")
        object.__setattr__(self, "metadata", frozen_metadata)
