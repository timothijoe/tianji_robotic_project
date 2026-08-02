"""Backend-neutral Wuji hand capability contracts."""

from collections.abc import Iterator
from typing import Protocol, runtime_checkable

import numpy as np
from numpy.typing import NDArray

from .models import SkeletonFrame


@runtime_checkable
class SkeletonSource(Protocol):
    def frames(self) -> Iterator[SkeletonFrame]: ...


@runtime_checkable
class Retargeter(Protocol):
    def step(self, keypoints_m: NDArray[np.floating]) -> NDArray[np.floating]: ...


@runtime_checkable
class WujiHandBackend(Protocol):
    def read_position_rad(self) -> NDArray[np.floating]: ...

    def command_position_rad(self, target: NDArray[np.floating]) -> None: ...

    def step(self, duration_s: float) -> None: ...

    def close(self) -> None: ...
