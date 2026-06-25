from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping

from cook_core.interfaces import (
    JointStateData,
    JointTrajectoryData,
    TrajectoryPointData,
)


@dataclass
class TeachPendantModel:
    joint_names: tuple[str, ...]
    joint_limits: Mapping[str, tuple[float | None, float | None]] = field(
        default_factory=dict
    )
    _current_positions: dict[str, float] = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self.joint_names = tuple(str(name) for name in self.joint_names)
        self.joint_limits = dict(self.joint_limits)
        self._current_positions = {name: 0.0 for name in self.joint_names}

    def update_current_state(self, state: JointStateData) -> None:
        values = state.as_mapping()
        for name in self.joint_names:
            if name in values:
                self._current_positions[name] = self._clamp(name, values[name])

    def current_positions(self) -> dict[str, float]:
        return dict(self._current_positions)

    def build_trajectory(
        self,
        *,
        target_positions: Mapping[str, float],
        duration_sec: float,
    ) -> JointTrajectoryData:
        duration = max(0.0, float(duration_sec))
        target = {}
        for name in self.joint_names:
            value = target_positions.get(name, self._current_positions[name])
            target[name] = self._clamp(name, float(value))
        return JointTrajectoryData(
            joint_names=self.joint_names,
            points=(
                TrajectoryPointData(dict(self._current_positions), 0.0),
                TrajectoryPointData(target, duration),
            ),
            trajectory_id="teach_pendant",
            source="teach_pendant",
        )

    def _clamp(self, name: str, value: float) -> float:
        lower, upper = self.joint_limits.get(name, (None, None))
        if lower is not None:
            value = max(float(lower), value)
        if upper is not None:
            value = min(float(upper), value)
        return value
