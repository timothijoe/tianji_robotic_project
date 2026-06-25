from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

import numpy as np

from cook_mujoco.control.runtime import MujocoRuntime, MujocoJointError
from cook_core.interfaces import (
    ControlCommand,
    ControlState,
    InterfaceDataError,
    JointStateData,
)


@dataclass(frozen=True)
class ControllerState:
    joint_names: tuple[str, ...]
    positions: tuple[float, ...]

    def as_mapping(self) -> dict[str, float]:
        return dict(zip(self.joint_names, self.positions))


class MujocoPositionController:
    def __init__(
        self,
        runtime: MujocoRuntime,
        *,
        joint_limits: Mapping[str, tuple[float | None, float | None]] | None = None,
    ):
        self.runtime = runtime
        self.joint_limits = dict(joint_limits or {})

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self.runtime.joint_names

    def set_joint_targets(self, targets: Mapping[str, float]) -> ControllerState:
        try:
            command = ControlCommand(target=JointStateData.from_mapping(targets))
        except InterfaceDataError as exc:
            raise MujocoJointError(str(exc)) from exc
        state = self.apply_command(command)
        return ControllerState(
            joint_names=state.joints.names,
            positions=state.joints.positions,
        )

    def apply_command(self, command: ControlCommand) -> ControlState:
        current = self.runtime.get_joint_positions()
        targets = command.target.as_mapping()
        ordered = []
        for index, name in enumerate(self.joint_names):
            value = targets.get(name, current[index])
            ordered.append(self._clamp_joint(name, _finite(value, f"target {name}")))
        positions = self.runtime.advance_position_targets(ordered)
        return ControlState(
            joints=JointStateData(
                names=self.joint_names,
                positions=tuple(float(value) for value in positions),
            ),
            controller_name="mujoco_position",
            trajectory_id=command.trajectory_id,
        )

    def reset(self, positions: Mapping[str, float] | None = None) -> ControllerState:
        if positions is None:
            values = np.zeros(len(self.joint_names), dtype=float)
        else:
            values = [
                self._clamp_joint(name, _finite(positions.get(name, 0.0), name))
                for name in self.joint_names
            ]
        state = self.runtime.reset(values)
        return ControllerState(
            joint_names=self.joint_names,
            positions=tuple(float(value) for value in state),
        )

    def _clamp_joint(self, name: str, value: float) -> float:
        lower, upper = self.joint_limits.get(name, (None, None))
        if lower is not None:
            value = max(float(lower), value)
        if upper is not None:
            value = min(float(upper), value)
        return value


def _finite(value, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise MujocoJointError(f"{label} must be a finite number") from exc
    if not np.isfinite(result):
        raise MujocoJointError(f"{label} must be a finite number")
    return result
