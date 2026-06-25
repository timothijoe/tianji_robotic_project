from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

from cook_core.interfaces import (
    ControlCommand,
    JointStateData,
    JointTrajectoryData,
)


@dataclass
class _ActiveTrajectory:
    trajectory: JointTrajectoryData
    start_time_sec: float
    point_index: int = 0
    finished_command_returned: bool = False


class TrajectoryExecutor:
    """Time-indexed trajectory playback behind a small control interface."""

    def __init__(self, *, expected_joint_names: tuple[str, ...]):
        self.expected_joint_names = tuple(expected_joint_names)
        self._active: _ActiveTrajectory | None = None

    @property
    def has_active_trajectory(self) -> bool:
        return self._active is not None

    def start(self, trajectory: JointTrajectoryData, *, now_sec: float) -> bool:
        if trajectory.joint_names != self.expected_joint_names:
            return False
        if trajectory.is_empty:
            return False
        self._active = _ActiveTrajectory(
            trajectory=trajectory,
            start_time_sec=float(now_sec),
        )
        return True

    def command_at(self, *, now_sec: float) -> ControlCommand | None:
        if self._active is None:
            return None
        active = self._active
        if active.finished_command_returned:
            self._active = None
            return None

        elapsed_sec = max(0.0, float(now_sec) - active.start_time_sec)
        points = active.trajectory.points
        while (
            active.point_index + 1 < len(points)
            and points[active.point_index + 1].time_from_start_sec <= elapsed_sec
        ):
            active.point_index += 1
        if active.point_index == len(points) - 1:
            active.finished_command_returned = True
        else:
            active.finished_command_returned = False
        target_positions = sample_trajectory_positions(active.trajectory, elapsed_sec)
        return ControlCommand(
            target=JointStateData.from_mapping(target_positions),
            trajectory_id=active.trajectory.trajectory_id,
        )


def sample_trajectory_positions(
    trajectory: JointTrajectoryData,
    elapsed_sec: float,
) -> dict[str, float]:
    points = trajectory.points
    if not points:
        raise ValueError("trajectory must contain at least one point")
    elapsed = max(0.0, float(elapsed_sec))
    point_index = 0
    while (
        point_index + 1 < len(points)
        and points[point_index + 1].time_from_start_sec <= elapsed
    ):
        point_index += 1
    if point_index == len(points) - 1:
        return {
            name: points[point_index].positions[name]
            for name in trajectory.joint_names
        }
    start = points[point_index]
    end = points[point_index + 1]
    return _interpolate_positions(
        joint_names=trajectory.joint_names,
        start_positions=start.positions,
        end_positions=end.positions,
        start_time_sec=start.time_from_start_sec,
        end_time_sec=end.time_from_start_sec,
        elapsed_sec=elapsed,
    )


def _interpolate_positions(
    *,
    joint_names: tuple[str, ...],
    start_positions: Mapping[str, float],
    end_positions: Mapping[str, float],
    start_time_sec: float,
    end_time_sec: float,
    elapsed_sec: float,
) -> dict[str, float]:
    duration = float(end_time_sec) - float(start_time_sec)
    if duration <= 0.0:
        ratio = 1.0
    else:
        ratio = (float(elapsed_sec) - float(start_time_sec)) / duration
        ratio = min(1.0, max(0.0, ratio))
    return {
        name: float(start_positions[name])
        + (float(end_positions[name]) - float(start_positions[name])) * ratio
        for name in joint_names
    }
