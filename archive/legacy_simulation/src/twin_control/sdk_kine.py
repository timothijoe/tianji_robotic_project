"""Vendor-style kinematics compatibility backed by MuJoCo."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence

import numpy as np

from twin_control.kinematics import MarvinKinematics, matrix_to_xyzabc, xyzabc_to_matrix
from twin_description.paths import right_chopping_scene_path
from twin_mujoco.runtime import TwinMujocoRuntime


def _arm_name_from_type(arm_type: int) -> str:
    if int(arm_type) == 0:
        return "left"
    if int(arm_type) == 1:
        return "right"
    raise ValueError(f"arm_type must be 0 or 1, got {arm_type!r}")


@dataclass
class FX_InvKineSolvePara:
    """Small Python equivalent of the vendor IK parameter/result structure."""

    m_Input_Target_TCP: list[float] = field(default_factory=lambda: [0.0] * 16)
    m_Input_Ref_Joint: list[float] = field(default_factory=lambda: [0.0] * 7)
    m_Input_IK_ZSP_Type: int = 0
    m_Input_IK_ZSP_Para: list[float] = field(default_factory=lambda: [0.0] * 6)
    m_Input_ZSP_Angle: float = 0.0
    m_Output_RetJoint: list[float] = field(default_factory=lambda: [0.0] * 7)
    m_OutPut_AllJoint: list[list[float]] = field(default_factory=list)
    m_OutPut_Result_Num: int = 0
    m_Output_IsOutRange: bool = False
    m_Output_IsDeg: bool = False
    m_Output_IsJntExd: bool = False
    m_Output_JntExdTags: list[int] = field(default_factory=lambda: [0] * 7)

    def set_input_ik_target_tcp(self, matrix: Sequence[float]) -> None:
        values = [float(v) for v in matrix]
        if len(values) != 16:
            raise ValueError("target TCP matrix must contain 16 values")
        self.m_Input_Target_TCP = values

    def set_input_ik_ref_joint(self, values: Sequence[float]) -> None:
        ref = [float(v) for v in values]
        if len(ref) != 7:
            raise ValueError("reference joints must contain 7 values")
        self.m_Input_Ref_Joint = ref

    def set_input_ik_zsp_type(self, value: int) -> None:
        self.m_Input_IK_ZSP_Type = int(value)

    def set_input_ik_zsp_para(self, values: Sequence[float]) -> None:
        para = [float(v) for v in values]
        if len(para) != 6:
            raise ValueError("zsp parameters must contain 6 values")
        self.m_Input_IK_ZSP_Para = para

    def set_input_zsp_angle(self, value: float) -> None:
        self.m_Input_ZSP_Angle = float(value)

    def get_output_ret_joint(self) -> list[float]:
        return list(self.m_Output_RetJoint)

    def get_output_all_joint(self) -> list[list[float]]:
        return [list(v) for v in self.m_OutPut_AllJoint]

    def get_output_result_num(self) -> int:
        return int(self.m_OutPut_Result_Num)

    def get_output_is_out_range(self) -> bool:
        return bool(self.m_Output_IsOutRange)

    def get_output_is_deg(self) -> bool:
        return bool(self.m_Output_IsDeg)

    def get_output_jnt_exd_tags(self) -> list[int]:
        return list(self.m_Output_JntExdTags)

    def get_output_is_jnt_exd(self) -> bool:
        return bool(self.m_Output_IsJntExd)


class MujocoKine:
    """Subset of vendor ``Marvin_Kine`` backed by the local MuJoCo model."""

    def __init__(
        self,
        *,
        arm_type: int = 1,
        model_path: str | Path | None = None,
        tcp_site_name: str | None = None,
    ) -> None:
        self.arm_type = int(arm_type)
        self.arm = _arm_name_from_type(self.arm_type)
        self.model_path = Path(model_path) if model_path else right_chopping_scene_path()
        self.runtime = TwinMujocoRuntime.load(str(self.model_path))
        self.runtime.reset()
        self.kinematics = MarvinKinematics(
            self.arm,
            unit_mode="sdk",
            tcp_site_name=tcp_site_name,
        )
        self.kinematics.set_runtime(self.runtime)
        self._config: dict | None = None

    def log_switch(self, switch: int) -> None:
        del switch

    def load_config(self, arm_type: int, config_path: str) -> dict:
        """Return a vendor-shaped config dictionary for compatibility."""
        self.arm_type = int(arm_type)
        self.arm = _arm_name_from_type(self.arm_type)
        self._config = {
            "TYPE": [1007, 1007],
            "DH": [[0.0] * 4 for _ in range(2)],
            "PNVA": [[0.0] * 4 for _ in range(2)],
            "BD": [[0.0] * 3 for _ in range(2)],
            "CONFIG_PATH": str(config_path),
        }
        return self._config

    def initial_kine(self, robot_type: int, dh: list, pnva: list, j67: list) -> bool:
        del robot_type, dh, pnva, j67
        return True

    def set_tool_kine(self, tool_mat: Sequence[Sequence[float]]) -> bool:
        xyzabc = self.mat4x4_to_xyzabc(tool_mat)
        self.kinematics.set_tool(xyzabc)
        return True

    def remove_tool_kine(self) -> bool:
        self.kinematics.remove_tool()
        return True

    def fk(self, joints: Sequence[float]) -> list[list[float]]:
        matrix_si, _ = self.kinematics.fk(np.asarray(joints, dtype=float))
        matrix_sdk = matrix_si.copy()
        matrix_sdk[:3, 3] *= 1000.0
        return matrix_sdk.tolist()

    def ik(self, structure_data: FX_InvKineSolvePara) -> FX_InvKineSolvePara:
        target_sdk = np.asarray(structure_data.m_Input_Target_TCP, dtype=float).reshape(4, 4)
        target_si = target_sdk.copy()
        target_si[:3, 3] *= 0.001
        ref = np.asarray(structure_data.m_Input_Ref_Joint, dtype=float)
        result = self.kinematics.ik(
            target_si,
            ref,
            zsp_type=structure_data.m_Input_IK_ZSP_Type,
            zsp_para=np.asarray(structure_data.m_Input_IK_ZSP_Para, dtype=float),
            zsp_angle_deg=structure_data.m_Input_ZSP_Angle,
        )
        joints_deg = np.rad2deg(result.joints_rad).tolist()
        structure_data.m_Output_RetJoint = joints_deg
        structure_data.m_OutPut_AllJoint = [joints_deg]
        structure_data.m_OutPut_Result_Num = 1 if result.success else 0
        structure_data.m_Output_IsOutRange = not result.success
        structure_data.m_Output_IsDeg = bool(result.is_singular)
        structure_data.m_Output_IsJntExd = False
        structure_data.m_Output_JntExdTags = [0] * 7
        return structure_data

    def joints2JacobMatrix(self, joints: Sequence[float]) -> list[list[float]]:
        return self.kinematics.jacobian(np.asarray(joints, dtype=float)).tolist()

    def mat4x4_to_mat1x16(self, pose_mat: Sequence[Sequence[float]]) -> list[float]:
        return np.asarray(pose_mat, dtype=float).reshape(4, 4).reshape(16).tolist()

    def mat4x4_to_xyzabc(self, pose_mat: Sequence[Sequence[float]]) -> list[float]:
        matrix = np.asarray(pose_mat, dtype=float).reshape(4, 4).copy()
        matrix[:3, 3] *= 0.001
        xyzabc_si = matrix_to_xyzabc(matrix)
        xyzabc_sdk = xyzabc_si.copy()
        xyzabc_sdk[:3] *= 1000.0
        xyzabc_sdk[3:] = np.rad2deg(xyzabc_sdk[3:])
        return xyzabc_sdk.tolist()

    def xyzabc_to_mat4x4(self, xyzabc: Sequence[float]) -> list[list[float]]:
        values = np.asarray(xyzabc, dtype=float).reshape(6).copy()
        values[:3] *= 0.001
        values[3:] = np.deg2rad(values[3:])
        matrix = xyzabc_to_matrix(values)
        matrix[:3, 3] *= 1000.0
        return matrix.tolist()
