from __future__ import annotations

from typing import Mapping, Protocol

from cook_core.interfaces import (
    JointTrajectoryData,
    PlanRequest,
    PlanResult,
    TrajectoryPointData,
)

JointTrajectoryPoint = TrajectoryPointData


class Planner(Protocol):
    def plan(self, request: PlanRequest) -> PlanResult:
        """Return a joint trajectory for the requested start and goal state."""


class StateValidityChecker(Protocol):
    def is_state_valid(self, positions: Mapping[str, float]) -> bool:
        """Return whether a joint state is valid for planning."""


class PlanningError(RuntimeError):
    pass


__all__ = [
    "JointTrajectoryData",
    "JointTrajectoryPoint",
    "PlanRequest",
    "PlanResult",
    "PlanningError",
    "Planner",
    "StateValidityChecker",
    "TrajectoryPointData",
]
