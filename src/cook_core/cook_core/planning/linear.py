from __future__ import annotations

import numpy as np

from cook_core.interfaces import JointTrajectoryData
from cook_core.planning.interface import (
    PlanRequest,
    PlanResult,
    TrajectoryPointData,
)


class LinearJointPlanner:
    def plan(self, request: PlanRequest) -> PlanResult:
        if request.duration_sec <= 0.0:
            raise ValueError("duration_sec must be positive")
        if request.waypoint_count < 2:
            raise ValueError("waypoint_count must be at least 2")
        joint_names = tuple(str(name) for name in request.joint_names)
        if not joint_names:
            raise ValueError("joint_names must not be empty")

        start_positions = request.start_state.as_mapping()
        goal_positions = request.goal_state.as_mapping()
        start = np.array(
            [_lookup_position(start_positions, name, "start") for name in joint_names],
            dtype=float,
        )
        goal = np.array(
            [_lookup_position(goal_positions, name, "goal") for name in joint_names],
            dtype=float,
        )

        points = []
        for index in range(request.waypoint_count):
            ratio = index / (request.waypoint_count - 1)
            positions = start + (goal - start) * ratio
            points.append(
                TrajectoryPointData(
                    positions={
                        name: float(value) for name, value in zip(joint_names, positions)
                    },
                    time_from_start_sec=float(request.duration_sec * ratio),
                )
            )
        return PlanResult(
            trajectory=JointTrajectoryData(
                joint_names=joint_names,
                points=tuple(points),
                source="linear",
            )
        )


def _lookup_position(values, name: str, label: str) -> float:
    if name not in values:
        raise ValueError(f"{label}_positions missing joint: {name}")
    value = float(values[name])
    if not np.isfinite(value):
        raise ValueError(f"{label}_positions contains non-finite value: {name}")
    return value
