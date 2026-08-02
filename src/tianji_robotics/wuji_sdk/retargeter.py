"""Lazy, hardware-free adapter for the official Wuji retargeting session."""

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class OfficialWujiRetargeter:
    """Adapt an official ``RetargetSession`` to the backend-neutral contract."""

    _session: Any

    @classmethod
    def create_left_first_generation(cls) -> "OfficialWujiRetargeter":
        """Create the first-generation left-hand retargeter without device access."""
        from wuji_sdk import HandModel, Handedness, RetargetSession

        return cls(RetargetSession.for_hand(HandModel.WujiHand, Handedness.Left))

    def step(self, keypoints_m: np.ndarray) -> np.ndarray:
        """Retarget keypoints and reject malformed SDK commands."""
        command = np.asarray(self._session.step(keypoints_m))
        if command.shape != (20,):
            raise ValueError("official Wuji retargeter must return 20 joint commands")
        if not np.issubdtype(command.dtype, np.number) or np.issubdtype(
            command.dtype, np.complexfloating
        ):
            raise ValueError("official Wuji retargeter returned non-numeric commands")
        if not np.isfinite(command).all():
            raise ValueError("official Wuji retargeter returned non-finite commands")
        return np.array(command, dtype=np.float64, copy=True)
