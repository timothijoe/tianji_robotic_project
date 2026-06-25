from __future__ import annotations

import importlib
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np


MJCF_SUFFIXES = frozenset({".xml", ".mjcf"})


class MujocoRuntimeError(RuntimeError):
    pass


class MujocoUnavailableError(MujocoRuntimeError):
    pass


class MujocoModelError(MujocoRuntimeError):
    pass


class MujocoJointError(MujocoRuntimeError):
    pass


@dataclass(frozen=True)
class BodyPose:
    position: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]


def require_mujoco():
    try:
        return importlib.import_module("mujoco")
    except ImportError as exc:
        raise MujocoUnavailableError(
            "MuJoCo is required at runtime. Install it with `pip install mujoco`."
        ) from exc


class MujocoRuntime:
    def __init__(self, model, data, mujoco_module, joint_names: Sequence[str]):
        self.model = model
        self.data = data
        self._mujoco = mujoco_module
        self.joint_names = tuple(str(name) for name in joint_names)
        self._joint_qpos_addr = self._build_joint_qpos_mapping(self.joint_names)
        self._joint_dof_addr = self._build_joint_dof_mapping(self.joint_names)
        self._joint_qpos_index = np.array(
            [self._joint_qpos_addr[name] for name in self.joint_names],
            dtype=int,
        )
        self._joint_dof_index = np.array(
            [self._joint_dof_addr[name] for name in self.joint_names],
            dtype=int,
        )

    @classmethod
    def load(cls, model_path: str | Path, joint_names: Sequence[str]) -> "MujocoRuntime":
        path = _validate_mjcf_path(model_path)
        mujoco = require_mujoco()
        try:
            model = mujoco.MjModel.from_xml_path(str(path))
            data = mujoco.MjData(model)
        except Exception as exc:
            raise MujocoModelError(f"failed to load MuJoCo model: {path}") from exc
        return cls(model, data, mujoco, joint_names)

    def reset(self, joint_positions: Sequence[float] | None = None) -> np.ndarray:
        self._mujoco.mj_resetData(self.model, self.data)
        if joint_positions is not None:
            self.set_joint_positions(joint_positions, forward=False)
        self._mujoco.mj_forward(self.model, self.data)
        return self.get_joint_positions()

    def set_joint_positions(
        self,
        joint_positions: Sequence[float],
        *,
        forward: bool = True,
        zero_velocities: bool = True,
    ) -> None:
        positions = np.asarray(joint_positions, dtype=float).reshape(-1)
        if len(positions) != len(self.joint_names):
            raise MujocoJointError(
                "joint position length mismatch: "
                f"expected={len(self.joint_names)}, got={len(positions)}"
            )
        if not np.all(np.isfinite(positions)):
            raise MujocoModelError("joint positions must be finite")
        self.data.qpos[self._joint_qpos_index] = positions
        if zero_velocities:
            self.data.qvel[self._joint_dof_index] = 0.0
        if forward:
            self._mujoco.mj_forward(self.model, self.data)

    def advance_position_targets(self, joint_positions: Sequence[float]) -> np.ndarray:
        self.set_joint_positions(joint_positions, forward=True, zero_velocities=True)
        return self.get_joint_positions()

    def step(self, substeps: int = 1) -> np.ndarray:
        for _ in range(max(1, int(substeps))):
            self._mujoco.mj_step(self.model, self.data)
        return self.get_joint_positions()

    def get_joint_positions(self) -> np.ndarray:
        return np.array(self.data.qpos[self._joint_qpos_index], dtype=float)

    def get_body_pose(self, name: str) -> BodyPose:
        body_id = self._name_to_id(self._mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id < 0:
            raise MujocoModelError(f"MuJoCo body not found: {name}")
        return BodyPose(
            position=tuple(float(v) for v in self.data.xpos[body_id]),
            quaternion=tuple(float(v) for v in self.data.xquat[body_id]),
        )

    def _build_joint_qpos_mapping(self, joint_names: Sequence[str]) -> dict[str, int]:
        mapping = {}
        for name in joint_names:
            joint_id = self._name_to_id(self._mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                raise MujocoJointError(f"MuJoCo joint not found: {name}")
            qpos_width = self._joint_qpos_width(joint_id)
            if qpos_width != 1:
                raise MujocoJointError(
                    "only one-DoF hinge/slide joints are supported: "
                    f"name={name}, qpos_width={qpos_width}"
                )
            mapping[str(name)] = int(self.model.jnt_qposadr[joint_id])
        return mapping

    def _build_joint_dof_mapping(self, joint_names: Sequence[str]) -> dict[str, int]:
        mapping = {}
        for name in joint_names:
            joint_id = self._name_to_id(self._mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                raise MujocoJointError(f"MuJoCo joint not found: {name}")
            mapping[str(name)] = int(self.model.jnt_dofadr[joint_id])
        return mapping

    def _joint_qpos_width(self, joint_id: int) -> int:
        qpos_addr = int(self.model.jnt_qposadr[joint_id])
        if joint_id + 1 < int(self.model.njnt):
            return int(self.model.jnt_qposadr[joint_id + 1]) - qpos_addr
        return int(self.model.nq) - qpos_addr

    def _name_to_id(self, obj_type, name: str) -> int:
        return int(self._mujoco.mj_name2id(self.model, obj_type, str(name)))


def _validate_mjcf_path(path: str | Path) -> Path:
    model_path = Path(path).expanduser()
    suffix = model_path.suffix.lower()
    if suffix == ".urdf":
        raise MujocoModelError(
            "runtime only loads pre-generated MJCF/XML files; convert URDF offline"
        )
    if suffix not in MJCF_SUFFIXES:
        supported = ", ".join(sorted(MJCF_SUFFIXES))
        raise MujocoModelError(
            f"unsupported MuJoCo model suffix '{suffix}'; supported: {supported}"
        )
    if not model_path.is_file():
        raise MujocoModelError(f"MuJoCo model file does not exist: {model_path}")
    return model_path
