from __future__ import annotations

from pathlib import Path
from typing import Sequence

import mujoco
import numpy as np

from twin_core import arm_spec
from twin_description import right_chopping_scene_path

from .errors import MujocoModelError, SafetyStop


class TwinMujocoRuntime:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self.model = model
        self.data = data
        self.joint_names = self._names(mujoco.mjtObj.mjOBJ_JOINT, model.njnt)
        self.actuator_names = self._names(mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu)
        self.timestep = float(model.opt.timestep)

    @classmethod
    def load(cls, model_path: str | Path | None = None) -> "TwinMujocoRuntime":
        path = Path(model_path) if model_path is not None else right_chopping_scene_path()
        if not path.is_file():
            raise MujocoModelError(f"MuJoCo model file does not exist: {path}")
        try:
            model = mujoco.MjModel.from_xml_path(str(path))
            data = mujoco.MjData(model)
        except Exception as exc:
            raise MujocoModelError(f"failed to load MuJoCo model: {path}") from exc
        mujoco.mj_forward(model, data)
        return cls(model, data)

    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def step(self) -> None:
        mujoco.mj_step(self.model, self.data)
        if not np.all(np.isfinite(self.data.qpos)) or not np.all(np.isfinite(self.data.qvel)):
            raise SafetyStop("MuJoCo state contains NaN or infinity")

    def set_arm_positions(self, arm_name: str, joint_positions: Sequence[float]) -> None:
        arm = self.arm_view(arm_name)
        q = np.asarray(joint_positions, dtype=float).reshape(-1)
        if q.shape != (7,) or not np.all(np.isfinite(q)):
            raise ValueError("joint_positions must contain 7 finite values")
        self.data.qpos[arm._qpos] = q
        self.data.qvel[arm._dofs] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def arm_view(self, name: str) -> "ArmView":
        return ArmView(self, arm_spec(name))

    def site_pose(self, name: str) -> tuple[np.ndarray, np.ndarray]:
        site_id = self._id(mujoco.mjtObj.mjOBJ_SITE, name)
        return self.data.site_xpos[site_id].copy(), self.data.site_xmat[site_id].reshape(3, 3).copy()

    def _id(self, object_type: mujoco.mjtObj, name: str) -> int:
        value = int(mujoco.mj_name2id(self.model, object_type, name))
        if value < 0:
            raise MujocoModelError(f"MuJoCo object not found: {name}")
        return value

    def _names(self, object_type: mujoco.mjtObj, count: int) -> tuple[str, ...]:
        names: list[str] = []
        for index in range(count):
            name = mujoco.mj_id2name(self.model, object_type, index)
            names.append("" if name is None else name)
        return tuple(names)


class ArmView:
    def __init__(self, runtime: TwinMujocoRuntime, spec) -> None:
        self.runtime = runtime
        self.spec = spec
        self.joint_names = spec.joint_names
        self.actuator_names = spec.actuator_names
        self._joint_ids = np.array([runtime._id(mujoco.mjtObj.mjOBJ_JOINT, name) for name in spec.joint_names], dtype=int)
        self._qpos = np.array([runtime.model.jnt_qposadr[joint_id] for joint_id in self._joint_ids], dtype=int)
        self._dofs = np.array([runtime.model.jnt_dofadr[joint_id] for joint_id in self._joint_ids], dtype=int)
        self._actuators = np.array([runtime._id(mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in spec.actuator_names], dtype=int)
        self.effort_limits = np.max(np.abs(runtime.model.actuator_ctrlrange[self._actuators]), axis=1)

    @property
    def joint_positions(self) -> np.ndarray:
        return self.runtime.data.qpos[self._qpos].copy()

    @property
    def joint_velocities(self) -> np.ndarray:
        return self.runtime.data.qvel[self._dofs].copy()

    @property
    def bias_torque(self) -> np.ndarray:
        return self.runtime.data.qfrc_bias[self._dofs].copy()

    def site_pose(self, name: str) -> tuple[np.ndarray, np.ndarray]:
        return self.runtime.site_pose(name)

    def site_jacobian(self, name: str) -> np.ndarray:
        site_id = self.runtime._id(mujoco.mjtObj.mjOBJ_SITE, name)
        jacp = np.zeros((3, self.runtime.model.nv))
        jacr = np.zeros((3, self.runtime.model.nv))
        mujoco.mj_jacSite(self.runtime.model, self.runtime.data, jacp, jacr, site_id)
        return np.vstack((jacp[:, self._dofs], jacr[:, self._dofs]))

    def apply_torque(self, torque_nm: Sequence[float]) -> np.ndarray:
        torque = np.asarray(torque_nm, dtype=float).reshape(-1)
        if torque.shape != (7,) or not np.all(np.isfinite(torque)):
            raise ValueError("torque_nm must contain 7 finite values")
        applied = np.clip(torque, -self.effort_limits, self.effort_limits)
        self.runtime.data.ctrl[self._actuators] = applied
        return applied.copy()
