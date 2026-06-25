from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping, Sequence

import numpy as np

from cook_mujoco.control import MujocoRuntime
from cook_mujoco.control.runtime import MujocoJointError


BodyPair = tuple[str, str]


@dataclass
class MujocoCollisionChecker:
    runtime: MujocoRuntime
    joint_names: Sequence[str]
    reference_positions: Mapping[str, float] | Sequence[float] | None = None
    allow_initial_contacts: bool = True
    allowed_body_pairs: set[BodyPair] | None = None
    initial_contact_body_pairs: set[BodyPair] = field(init=False)

    def __post_init__(self) -> None:
        self.joint_names = tuple(str(name) for name in self.joint_names)
        if not self.joint_names:
            raise MujocoJointError("joint_names must not be empty")
        self.allowed_body_pairs = set(self.allowed_body_pairs or set())
        self.initial_contact_body_pairs = set()
        if self.allow_initial_contacts and self.reference_positions is not None:
            self.runtime.set_joint_positions(
                self._ordered_positions(self.reference_positions),
                forward=True,
            )
            self.initial_contact_body_pairs = self.contact_body_pairs()
            self.allowed_body_pairs.update(self.initial_contact_body_pairs)

    def is_state_valid(
        self,
        positions: Mapping[str, float] | Sequence[float],
    ) -> bool:
        self.runtime.set_joint_positions(self._ordered_positions(positions), forward=True)
        return not self.has_disallowed_collision()

    def has_disallowed_collision(self) -> bool:
        return any(pair not in self.allowed_body_pairs for pair in self.contact_body_pairs())

    def contact_body_pairs(self) -> set[BodyPair]:
        pairs = set()
        for index in range(int(self.runtime.data.ncon)):
            contact = self.runtime.data.contact[index]
            pairs.add(
                _ordered_body_pair(
                    self._body_name_for_geom(int(contact.geom1)),
                    self._body_name_for_geom(int(contact.geom2)),
                )
            )
        return pairs

    def _ordered_positions(
        self,
        positions: Mapping[str, float] | Sequence[float],
    ) -> tuple[float, ...]:
        if isinstance(positions, Mapping):
            missing = [name for name in self.joint_names if name not in positions]
            if missing:
                raise MujocoJointError(f"missing joint position: {', '.join(missing)}")
            values = [positions[name] for name in self.joint_names]
        else:
            values = list(positions)
            if len(values) != len(self.joint_names):
                raise MujocoJointError(
                    "joint position length mismatch: "
                    f"expected={len(self.joint_names)}, got={len(values)}"
                )
        result = tuple(float(value) for value in values)
        if not np.all(np.isfinite(result)):
            raise MujocoJointError("joint positions must be finite")
        return result

    def _body_name_for_geom(self, geom_id: int) -> str:
        body_id = int(self.runtime.model.geom_bodyid[geom_id])
        name = self.runtime._mujoco.mj_id2name(
            self.runtime.model,
            self.runtime._mujoco.mjtObj.mjOBJ_BODY,
            body_id,
        )
        return "world" if name is None else str(name)


def _ordered_body_pair(first: str, second: str) -> BodyPair:
    left, right = str(first), str(second)
    if left <= right:
        return (left, right)
    return (right, left)
