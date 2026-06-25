from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

import numpy as np


class InterfaceDataError(ValueError):
    pass


@dataclass(frozen=True)
class JointStateData:
    names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...] | None = None
    efforts: tuple[float, ...] | None = None
    timestamp_sec: float | None = None

    def __post_init__(self) -> None:
        names = tuple(_name(name, "joint name") for name in self.names)
        positions = tuple(_finite(value, "joint position") for value in self.positions)
        if len(names) != len(positions):
            raise InterfaceDataError(
                "joint names and positions length mismatch: "
                f"names={len(names)}, positions={len(positions)}"
            )
        object.__setattr__(self, "names", names)
        object.__setattr__(self, "positions", positions)
        object.__setattr__(
            self,
            "velocities",
            _optional_vector(self.velocities, len(names), "joint velocity"),
        )
        object.__setattr__(
            self,
            "efforts",
            _optional_vector(self.efforts, len(names), "joint effort"),
        )
        object.__setattr__(
            self,
            "timestamp_sec",
            None
            if self.timestamp_sec is None
            else _finite(self.timestamp_sec, "timestamp_sec"),
        )

    @classmethod
    def from_mapping(
        cls,
        positions: Mapping[str, float],
        *,
        timestamp_sec: float | None = None,
    ) -> "JointStateData":
        names = tuple(_name(name, "joint name") for name in positions)
        return cls(
            names=names,
            positions=tuple(float(positions[name]) for name in names),
            timestamp_sec=timestamp_sec,
        )

    def as_mapping(self) -> dict[str, float]:
        return dict(zip(self.names, self.positions))

    def ordered_positions(self, joint_names: tuple[str, ...] | list[str]) -> tuple[float, ...]:
        values = self.as_mapping()
        ordered = []
        for name in joint_names:
            key = _name(name, "joint name")
            if key not in values:
                raise InterfaceDataError(f"missing joint position: {key}")
            ordered.append(values[key])
        return tuple(ordered)

    def to_dict(self) -> dict[str, Any]:
        return {
            "names": list(self.names),
            "positions": list(self.positions),
            "velocities": None if self.velocities is None else list(self.velocities),
            "efforts": None if self.efforts is None else list(self.efforts),
            "timestamp_sec": self.timestamp_sec,
        }


@dataclass(frozen=True)
class TrajectoryPointData:
    positions: Mapping[str, float]
    time_from_start_sec: float
    velocities: Mapping[str, float] | None = None
    accelerations: Mapping[str, float] | None = None

    def __post_init__(self) -> None:
        positions = _numeric_mapping(self.positions, "position")
        velocities = (
            None if self.velocities is None else _numeric_mapping(self.velocities, "velocity")
        )
        accelerations = (
            None
            if self.accelerations is None
            else _numeric_mapping(self.accelerations, "acceleration")
        )
        timestamp = _finite(self.time_from_start_sec, "time_from_start_sec")
        if timestamp < 0.0:
            raise InterfaceDataError("time_from_start_sec must be non-negative")
        object.__setattr__(self, "positions", positions)
        object.__setattr__(self, "velocities", velocities)
        object.__setattr__(self, "accelerations", accelerations)
        object.__setattr__(self, "time_from_start_sec", timestamp)

    def ordered_positions(self, joint_names: tuple[str, ...] | list[str]) -> tuple[float, ...]:
        return _ordered_mapping_values(self.positions, joint_names, "position")

    def to_dict(self) -> dict[str, Any]:
        return {
            "positions": dict(self.positions),
            "velocities": None if self.velocities is None else dict(self.velocities),
            "accelerations": (
                None if self.accelerations is None else dict(self.accelerations)
            ),
            "time_from_start_sec": self.time_from_start_sec,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "TrajectoryPointData":
        raw = _mapping(data, "TrajectoryPointData")
        return cls(
            positions=_mapping(raw.get("positions"), "positions"),
            velocities=(
                None
                if raw.get("velocities") is None
                else _mapping(raw.get("velocities"), "velocities")
            ),
            accelerations=(
                None
                if raw.get("accelerations") is None
                else _mapping(raw.get("accelerations"), "accelerations")
            ),
            time_from_start_sec=raw.get("time_from_start_sec"),
        )


@dataclass(frozen=True)
class JointTrajectoryData:
    joint_names: tuple[str, ...]
    points: tuple[TrajectoryPointData, ...]
    trajectory_id: str = ""
    frame_id: str = ""
    source: str = ""
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        joint_names = tuple(_name(name, "joint name") for name in self.joint_names)
        if not joint_names:
            raise InterfaceDataError("joint_names must not be empty")
        points = tuple(self.points)
        previous_time = -np.inf
        for point in points:
            for name in joint_names:
                if name not in point.positions:
                    raise InterfaceDataError(f"trajectory point is missing joint: {name}")
            for name in point.positions:
                if name not in joint_names:
                    raise InterfaceDataError(f"trajectory point has unknown joint: {name}")
            if point.time_from_start_sec < previous_time:
                raise InterfaceDataError("trajectory point times must be monotonic")
            previous_time = point.time_from_start_sec
        object.__setattr__(self, "joint_names", joint_names)
        object.__setattr__(self, "points", points)
        object.__setattr__(self, "trajectory_id", str(self.trajectory_id))
        object.__setattr__(self, "frame_id", str(self.frame_id))
        object.__setattr__(self, "source", str(self.source))
        object.__setattr__(self, "metadata", dict(self.metadata))

    @classmethod
    def from_positions(
        cls,
        *,
        joint_names: tuple[str, ...] | list[str],
        positions: list[Mapping[str, float]] | tuple[Mapping[str, float], ...],
        times_sec: list[float] | tuple[float, ...],
        trajectory_id: str = "",
        frame_id: str = "",
        source: str = "",
    ) -> "JointTrajectoryData":
        if len(positions) != len(times_sec):
            raise InterfaceDataError(
                "positions and times length mismatch: "
                f"positions={len(positions)}, times={len(times_sec)}"
            )
        return cls(
            joint_names=tuple(joint_names),
            points=tuple(
                TrajectoryPointData(position, time_sec)
                for position, time_sec in zip(positions, times_sec)
            ),
            trajectory_id=trajectory_id,
            frame_id=frame_id,
            source=source,
        )

    @property
    def is_empty(self) -> bool:
        return len(self.points) == 0

    def to_dict(self) -> dict[str, Any]:
        return {
            "joint_names": list(self.joint_names),
            "points": [point.to_dict() for point in self.points],
            "trajectory_id": self.trajectory_id,
            "frame_id": self.frame_id,
            "source": self.source,
            "metadata": dict(self.metadata),
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "JointTrajectoryData":
        raw = _mapping(data, "JointTrajectoryData")
        return cls(
            joint_names=tuple(raw.get("joint_names", ())),
            points=tuple(
                TrajectoryPointData.from_dict(point)
                for point in raw.get("points", ())
            ),
            trajectory_id=str(raw.get("trajectory_id", "")),
            frame_id=str(raw.get("frame_id", "")),
            source=str(raw.get("source", "")),
            metadata=dict(_mapping(raw.get("metadata", {}), "metadata")),
        )


@dataclass(frozen=True)
class PlanRequest:
    start_state: JointStateData
    goal_state: JointStateData
    joint_names: tuple[str, ...]
    duration_sec: float = 3.0
    waypoint_count: int = 120
    planning_time_sec: float = 1.0
    collision_check_resolution: float = 0.01
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        joint_names = tuple(_name(name, "joint name") for name in self.joint_names)
        if not joint_names:
            raise InterfaceDataError("joint_names must not be empty")
        duration = _finite(self.duration_sec, "duration_sec")
        planning_time = _finite(self.planning_time_sec, "planning_time_sec")
        resolution = _finite(
            self.collision_check_resolution,
            "collision_check_resolution",
        )
        if duration <= 0.0:
            raise InterfaceDataError("duration_sec must be positive")
        if int(self.waypoint_count) < 2:
            raise InterfaceDataError("waypoint_count must be at least 2")
        if planning_time <= 0.0:
            raise InterfaceDataError("planning_time_sec must be positive")
        if resolution <= 0.0:
            raise InterfaceDataError("collision_check_resolution must be positive")
        object.__setattr__(self, "joint_names", joint_names)
        object.__setattr__(self, "duration_sec", duration)
        object.__setattr__(self, "waypoint_count", int(self.waypoint_count))
        object.__setattr__(self, "planning_time_sec", planning_time)
        object.__setattr__(self, "collision_check_resolution", resolution)
        object.__setattr__(self, "metadata", dict(self.metadata))

    @classmethod
    def from_position_mappings(
        cls,
        *,
        start_positions: Mapping[str, float],
        goal_positions: Mapping[str, float],
        joint_names: tuple[str, ...] | list[str],
        duration_sec: float = 3.0,
        waypoint_count: int = 120,
        planning_time_sec: float = 1.0,
        collision_check_resolution: float = 0.01,
        metadata: Mapping[str, Any] | None = None,
    ) -> "PlanRequest":
        return cls(
            start_state=JointStateData.from_mapping(start_positions),
            goal_state=JointStateData.from_mapping(goal_positions),
            joint_names=tuple(joint_names),
            duration_sec=duration_sec,
            waypoint_count=waypoint_count,
            planning_time_sec=planning_time_sec,
            collision_check_resolution=collision_check_resolution,
            metadata={} if metadata is None else metadata,
        )


@dataclass(frozen=True)
class PlanResult:
    trajectory: JointTrajectoryData
    success: bool = True
    message: str = ""
    planner_name: str = ""
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "success", bool(self.success))
        object.__setattr__(self, "message", str(self.message))
        object.__setattr__(self, "planner_name", str(self.planner_name))
        object.__setattr__(self, "metadata", dict(self.metadata))

    @property
    def points(self) -> tuple[TrajectoryPointData, ...]:
        return self.trajectory.points

    @property
    def is_empty(self) -> bool:
        return self.trajectory.is_empty


@dataclass(frozen=True)
class ControlCommand:
    target: JointStateData
    trajectory_id: str = ""


@dataclass(frozen=True)
class ControlState:
    joints: JointStateData
    controller_name: str = ""
    trajectory_id: str = ""

    def as_mapping(self) -> dict[str, float]:
        return self.joints.as_mapping()


def _name(value: str, label: str) -> str:
    text = str(value).strip()
    if not text:
        raise InterfaceDataError(f"{label} must not be empty")
    return text


def _finite(value, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise InterfaceDataError(f"{label} must be finite") from exc
    if not np.isfinite(result):
        raise InterfaceDataError(f"{label} must be finite")
    return result


def _optional_vector(values, expected_len: int, label: str) -> tuple[float, ...] | None:
    if values is None:
        return None
    result = tuple(_finite(value, label) for value in values)
    if len(result) != expected_len:
        raise InterfaceDataError(
            f"{label} length mismatch: expected={expected_len}, got={len(result)}"
        )
    return result


def _numeric_mapping(values: Mapping[str, float], label: str) -> dict[str, float]:
    return {
        _name(name, f"{label} joint name"): _finite(value, f"{label} {name}")
        for name, value in values.items()
    }


def _mapping(value, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise InterfaceDataError(f"{label} must be a mapping")
    return value


def _ordered_mapping_values(
    values: Mapping[str, float],
    joint_names: tuple[str, ...] | list[str],
    label: str,
) -> tuple[float, ...]:
    ordered = []
    for name in joint_names:
        key = _name(name, "joint name")
        if key not in values:
            raise InterfaceDataError(f"missing trajectory {label}: {key}")
        ordered.append(float(values[key]))
    return tuple(ordered)
