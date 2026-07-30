from __future__ import annotations

from dataclasses import dataclass
from typing import Literal


ArmId = Literal["left", "right"]


@dataclass(frozen=True)
class ArmSpec:
    name: ArmId
    joint_names: tuple[str, ...]
    actuator_names: tuple[str, ...]
    home: tuple[float, ...] | None = None

    def __post_init__(self) -> None:
        if len(self.joint_names) != 7:
            raise ValueError(f"{self.name} arm must define 7 joints")
        if len(self.actuator_names) != 7:
            raise ValueError(f"{self.name} arm must define 7 actuators")
        if self.home is not None and len(self.home) != 7:
            raise ValueError(f"{self.name} arm home must contain 7 values")


_SPECS: dict[str, ArmSpec] = {
    "left": ArmSpec(
        name="left",
        joint_names=tuple(f"left_joint{i}" for i in range(1, 8)),
        actuator_names=tuple(f"act_left_joint{i}" for i in range(1, 8)),
    ),
    "right": ArmSpec(
        name="right",
        joint_names=tuple(f"right_joint{i}" for i in range(1, 8)),
        actuator_names=tuple(f"act_right_joint{i}" for i in range(1, 8)),
    ),
}


def arm_spec(name: str) -> ArmSpec:
    try:
        return _SPECS[str(name)]
    except KeyError as exc:
        allowed = ", ".join(sorted(_SPECS))
        raise ValueError(f"unknown arm '{name}'; expected one of: {allowed}") from exc


def all_arm_specs() -> dict[str, ArmSpec]:
    return dict(_SPECS)
