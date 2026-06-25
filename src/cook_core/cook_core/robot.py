from __future__ import annotations

from dataclasses import dataclass
from importlib import import_module
from math import isfinite
from typing import Mapping, Protocol, Sequence

from cook_core.interfaces import (
    JointStateData,
    JointTrajectoryData,
    TrajectoryPointData,
)


class RobotCommandError(ValueError):
    pass


class RobotCommandPort(Protocol):
    def move_joints(
        self,
        target_positions: Mapping[str, float] | JointStateData | JointTrajectoryData,
        *,
        duration_sec: float = 1.0,
    ) -> JointTrajectoryData:
        ...

    def move_joint_sequence(
        self,
        waypoints: Sequence[Mapping[str, float] | JointStateData],
        *,
        duration_sec: float,
    ) -> JointTrajectoryData:
        ...

    def hold_current(self, *, duration_sec: float = 0.0) -> JointTrajectoryData:
        ...

    def close(self) -> None:
        ...


@dataclass
class RobotCommandBuilder:
    joint_names: tuple[str, ...]
    current_positions: Mapping[str, float] | JointStateData | None = None

    def __post_init__(self) -> None:
        self.joint_names = _joint_names(self.joint_names)
        if self.current_positions is None:
            self.current_positions = {name: 0.0 for name in self.joint_names}
        else:
            self.current_positions = self._complete_positions(self.current_positions)

    def update_current_positions(
        self,
        positions: Mapping[str, float] | JointStateData,
    ) -> None:
        self.current_positions = self._complete_positions(positions)

    def build_move_joints(
        self,
        target_positions: Mapping[str, float] | JointStateData | JointTrajectoryData,
        *,
        duration_sec: float = 1.0,
    ) -> JointTrajectoryData:
        if isinstance(target_positions, JointTrajectoryData):
            return self._validate_trajectory(target_positions)
        duration = _non_negative_duration(duration_sec)
        start = dict(self.current_positions or {})
        target = self._complete_positions(target_positions, base=start)
        self.current_positions = dict(target)
        return JointTrajectoryData(
            joint_names=self.joint_names,
            points=(
                TrajectoryPointData(start, 0.0),
                TrajectoryPointData(target, duration),
            ),
            source="robot_command",
        )

    def build_joint_sequence(
        self,
        waypoints: Sequence[Mapping[str, float] | JointStateData],
        *,
        duration_sec: float,
    ) -> JointTrajectoryData:
        duration = _non_negative_duration(duration_sec)
        if not waypoints:
            raise RobotCommandError("waypoints must not be empty")
        base = dict(self.current_positions or {})
        count = len(waypoints)
        points = []
        for index, waypoint in enumerate(waypoints):
            positions = self._complete_positions(waypoint, base=base)
            time_from_start = duration if count == 1 else duration * index / (count - 1)
            points.append(TrajectoryPointData(positions, time_from_start))
            base = positions
        self.current_positions = dict(base)
        return JointTrajectoryData(
            joint_names=self.joint_names,
            points=tuple(points),
            source="robot_command",
        )

    def build_hold_current(self, *, duration_sec: float = 0.0) -> JointTrajectoryData:
        duration = _non_negative_duration(duration_sec)
        current = dict(self.current_positions or {})
        return JointTrajectoryData(
            joint_names=self.joint_names,
            points=(
                TrajectoryPointData(current, 0.0),
                TrajectoryPointData(dict(current), duration),
            ),
            source="robot_command",
        )

    def _complete_positions(
        self,
        positions: Mapping[str, float] | JointStateData,
        *,
        base: Mapping[str, float] | None = None,
    ) -> dict[str, float]:
        raw = _position_mapping(positions)
        if not raw:
            raise RobotCommandError("target positions must not be empty")
        unknown = sorted(name for name in raw if name not in self.joint_names)
        if unknown:
            raise RobotCommandError(f"unknown joints: {', '.join(unknown)}")
        baseline = {name: 0.0 for name in self.joint_names}
        if base is not None:
            baseline.update(_position_mapping(base))
        result = dict(baseline)
        for name, value in raw.items():
            result[name] = _finite(value, f"joint target {name}")
        return {name: result[name] for name in self.joint_names}

    def _validate_trajectory(
        self,
        trajectory: JointTrajectoryData,
    ) -> JointTrajectoryData:
        if trajectory.joint_names != self.joint_names:
            raise RobotCommandError(
                "trajectory joint order mismatch: "
                f"expected={self.joint_names}, got={trajectory.joint_names}"
            )
        if trajectory.is_empty:
            raise RobotCommandError("trajectory must not be empty")
        self.current_positions = dict(trajectory.points[-1].positions)
        return trajectory


class RecordingRobotCommandPort:
    def __init__(
        self,
        *,
        joint_names: Sequence[str],
        current_positions: Mapping[str, float] | JointStateData | None = None,
    ):
        self.builder = RobotCommandBuilder(
            joint_names=tuple(joint_names),
            current_positions=current_positions,
        )
        self.published: list[JointTrajectoryData] = []
        self.closed = False

    def move_joints(
        self,
        target_positions: Mapping[str, float] | JointStateData | JointTrajectoryData,
        *,
        duration_sec: float = 1.0,
    ) -> JointTrajectoryData:
        trajectory = self.builder.build_move_joints(
            target_positions,
            duration_sec=duration_sec,
        )
        self.published.append(trajectory)
        return trajectory

    def move_joint_sequence(
        self,
        waypoints: Sequence[Mapping[str, float] | JointStateData],
        *,
        duration_sec: float,
    ) -> JointTrajectoryData:
        trajectory = self.builder.build_joint_sequence(
            waypoints,
            duration_sec=duration_sec,
        )
        self.published.append(trajectory)
        return trajectory

    def hold_current(self, *, duration_sec: float = 0.0) -> JointTrajectoryData:
        trajectory = self.builder.build_hold_current(duration_sec=duration_sec)
        self.published.append(trajectory)
        return trajectory

    def close(self) -> None:
        self.closed = True


def create_robot_command_port(
    *,
    backend: str = "ros2",
    **config,
) -> RobotCommandPort:
    if backend == "fake":
        return RecordingRobotCommandPort(**config)
    if backend == "ros2":
        module = import_module("cook_bringup.client")
        return module.RosRobotCommandPort(**config)
    raise RobotCommandError(f"unsupported robot command backend: {backend}")


def _joint_names(values: Sequence[str]) -> tuple[str, ...]:
    names = tuple(str(name).strip() for name in values)
    if not names:
        raise RobotCommandError("joint_names must not be empty")
    if any(not name for name in names):
        raise RobotCommandError("joint_names must not contain empty names")
    if len(set(names)) != len(names):
        raise RobotCommandError("joint_names must not contain duplicates")
    return names


def _position_mapping(
    positions: Mapping[str, float] | JointStateData,
) -> dict[str, float]:
    if isinstance(positions, JointStateData):
        return positions.as_mapping()
    return {str(name): value for name, value in dict(positions).items()}


def _non_negative_duration(value: float) -> float:
    duration = _finite(value, "duration_sec")
    if duration < 0.0:
        raise RobotCommandError("duration_sec must be non-negative")
    return duration


def _finite(value, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise RobotCommandError(f"{label} must be a finite number") from exc
    if not isfinite(result):
        raise RobotCommandError(f"{label} must be a finite number")
    return result
