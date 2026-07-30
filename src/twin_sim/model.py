from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from twin_sim.names import LEFT_ARM, RIGHT_ARM, ArmNames
from twin_sim.paths import scene_path


class ModelValidationError(RuntimeError):
    pass


@dataclass(frozen=True)
class ArmIndices:
    joint_ids: np.ndarray
    qpos_ids: np.ndarray
    dof_ids: np.ndarray
    actuator_ids: np.ndarray


@dataclass
class SimulationModel:
    model: mujoco.MjModel
    data: mujoco.MjData
    left: ArmIndices
    right: ArmIndices

    @classmethod
    def load(cls, path: Path | None = None) -> "SimulationModel":
        model_path = scene_path() if path is None else Path(path)
        model = mujoco.MjModel.from_xml_path(str(model_path))
        data = mujoco.MjData(model)
        simulation = cls(
            model=model,
            data=data,
            left=cls._arm_indices(model, LEFT_ARM),
            right=cls._arm_indices(model, RIGHT_ARM),
        )
        simulation.require_site("right_tool_tip_site")
        simulation.require_sensor("right_tool_force")
        simulation.require_sensor("right_tool_torque")
        simulation.require_geom("right_knife_blade")
        simulation.require_geom("chopping_board")
        mujoco.mj_forward(model, data)
        return simulation

    @classmethod
    def _arm_indices(cls, model: mujoco.MjModel, names: ArmNames) -> ArmIndices:
        joint_ids = np.asarray(
            [cls._require_id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint", name) for name in names.joints],
            dtype=np.int32,
        )
        actuator_ids = np.asarray(
            [
                cls._require_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "actuator", name)
                for name in names.actuators
            ],
            dtype=np.int32,
        )
        return ArmIndices(
            joint_ids=joint_ids,
            qpos_ids=model.jnt_qposadr[joint_ids].copy(),
            dof_ids=model.jnt_dofadr[joint_ids].copy(),
            actuator_ids=actuator_ids,
        )

    @staticmethod
    def _require_id(
        model: mujoco.MjModel, object_type: mujoco.mjtObj, label: str, name: str
    ) -> int:
        object_id = mujoco.mj_name2id(model, object_type, name)
        if object_id < 0:
            raise ModelValidationError(f"missing {label}: {name}")
        return object_id

    def require_site(self, name: str) -> int:
        return self._require_id(self.model, mujoco.mjtObj.mjOBJ_SITE, "site", name)

    def require_geom(self, name: str) -> int:
        return self._require_id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "geom", name)

    def require_sensor(self, name: str) -> int:
        return self._require_id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "sensor", name)
