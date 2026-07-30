from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np

from twin_mujoco.runtime import ArmView


@dataclass(frozen=True)
class CartesianImpedanceConfig:
    translational_stiffness: tuple[float, float, float] = (2500.0, 2500.0, 2800.0)
    translational_damping: tuple[float, float, float] = (105.0, 105.0, 115.0)
    rotational_stiffness: tuple[float, float, float] = (45.0, 45.0, 35.0)
    rotational_damping: tuple[float, float, float] = (5.5, 5.5, 4.5)
    nullspace_stiffness: float = 4.0
    nullspace_damping: float = 1.5
    torque_rate_limit: float = 1500.0


@dataclass(frozen=True)
class ForceControlConfig:
    target_force_n: float = 10.0
    feedback_alpha: float = 0.18
    admittance_gain_m_per_ns: float = 0.03
    maximum_position_offset_m: float = 0.04


class WrenchCalibrator:
    def __init__(self, *, tool_mass_kg: float = 0.22, gravity: Sequence[float] = (0.0, 0.0, -9.81)):
        self.tool_mass_kg = float(tool_mass_kg)
        self.gravity_world = _vector(gravity, 3, "gravity")
        self._residual = np.zeros(6)
        self._calibrated = False

    def modeled_gravity_wrench(self, sensor_rotation_world: np.ndarray) -> np.ndarray:
        rotation = np.asarray(sensor_rotation_world, dtype=float).reshape(3, 3)
        force = -(rotation.T @ (self.tool_mass_kg * self.gravity_world))
        return np.concatenate((force, np.zeros(3)))

    def calibrate(self, samples: Iterable[Sequence[float]], sensor_rotation_world: np.ndarray) -> None:
        values = np.asarray(list(samples), dtype=float)
        if values.ndim != 2 or values.shape[1] != 6 or len(values) == 0 or not np.all(np.isfinite(values)):
            raise ValueError("calibration requires finite six-axis samples")
        self._residual = np.mean(values, axis=0) - self.modeled_gravity_wrench(sensor_rotation_world)
        self._calibrated = True

    def compensate(self, raw_wrench: Sequence[float], sensor_rotation_world: np.ndarray) -> np.ndarray:
        if not self._calibrated:
            raise ValueError("wrench sensor has not been calibrated")
        raw = _vector(raw_wrench, 6, "raw_wrench")
        return raw - self._residual - self.modeled_gravity_wrench(sensor_rotation_world)


class CartesianForceController:
    def __init__(
        self,
        arm: ArmView,
        *,
        impedance: CartesianImpedanceConfig | None = None,
        force: ForceControlConfig | None = None,
        tool_site: str = "right_tool_tip_site",
    ) -> None:
        self.arm = arm
        self.tool_site = tool_site
        self.impedance = impedance or CartesianImpedanceConfig()
        self.force = force or ForceControlConfig()
        self.desired_position, self.desired_rotation = arm.site_pose(tool_site)
        self.nullspace_reference = arm.joint_positions
        self.force_enabled = False
        self.force_axis_world: np.ndarray | None = None
        self._filtered_force = 0.0
        self._force_position_offset = 0.0
        self._previous_torque = np.zeros(7)

    def set_target(self, position: Sequence[float], rotation: np.ndarray) -> None:
        self.desired_position = _vector(position, 3, "position")
        self.desired_rotation = np.asarray(rotation, dtype=float).reshape(3, 3).copy()

    def set_force_axis_world(self, axis: Sequence[float] | None) -> None:
        if axis is None:
            self.force_axis_world = None
            return
        vector = _vector(axis, 3, "axis")
        norm = float(np.linalg.norm(vector))
        if norm < 1e-12:
            raise ValueError("axis must be non-zero")
        self.force_axis_world = vector / norm

    def _effective_force_axis_world(self, tool_rotation_world: np.ndarray) -> np.ndarray:
        if self.force_axis_world is not None:
            return self.force_axis_world.copy()
        rotation = np.asarray(tool_rotation_world, dtype=float).reshape(3, 3)
        return rotation[:, 2].copy()

    def _measured_force_along_axis(self, wrench_tool: np.ndarray, tool_rotation_world: np.ndarray) -> float:
        rotation = np.asarray(tool_rotation_world, dtype=float).reshape(3, 3)
        force_world = rotation @ wrench_tool[:3]
        return float(np.dot(force_world, self._effective_force_axis_world(rotation)))

    def enable_force(self, enabled: bool, *, target_force_n: float | None = None) -> None:
        if target_force_n is not None:
            if target_force_n < 0:
                raise ValueError("target force must be non-negative")
            self.force = ForceControlConfig(
                target_force_n=float(target_force_n),
                feedback_alpha=self.force.feedback_alpha,
                admittance_gain_m_per_ns=self.force.admittance_gain_m_per_ns,
                maximum_position_offset_m=self.force.maximum_position_offset_m,
            )
        self.force_enabled = bool(enabled)
        if not self.force_enabled:
            self._filtered_force = 0.0
            self._force_position_offset = 0.0

    def compute(self, compensated_wrench_tool: Sequence[float]) -> np.ndarray:
        wrench_tool = _vector(compensated_wrench_tool, 6, "compensated_wrench_tool")
        position, rotation = self.arm.site_pose(self.tool_site)
        jacobian = self.arm.site_jacobian(self.tool_site)
        velocity = jacobian @ self.arm.joint_velocities
        orientation_error = rotation_error(rotation, self.desired_rotation)
        timestep = self.arm.runtime.timestep
        force_axis_world = self._effective_force_axis_world(rotation)
        alpha = self.force.feedback_alpha
        measured_force = self._measured_force_along_axis(wrench_tool, rotation)
        self._filtered_force = (1.0 - alpha) * self._filtered_force + alpha * measured_force
        effective_target = self.desired_position.copy()
        if self.force_enabled:
            error = self.force.target_force_n - self._filtered_force
            self._force_position_offset = float(np.clip(
                self._force_position_offset + self.force.admittance_gain_m_per_ns * error * timestep,
                -self.force.maximum_position_offset_m,
                self.force.maximum_position_offset_m,
            ))
            effective_target += self._force_position_offset * force_axis_world
        kp = np.asarray(self.impedance.translational_stiffness)
        kd = np.asarray(self.impedance.translational_damping)
        kr = np.asarray(self.impedance.rotational_stiffness)
        dr = np.asarray(self.impedance.rotational_damping)
        task_wrench = np.concatenate((
            kp * (effective_target - position) - kd * velocity[:3],
            kr * orientation_error - dr * velocity[3:],
        ))
        torque = jacobian.T @ task_wrench + self.arm.bias_torque
        null_projector = np.eye(7) - jacobian.T @ np.linalg.pinv(jacobian.T)
        torque += null_projector @ (
            -self.impedance.nullspace_stiffness * (self.arm.joint_positions - self.nullspace_reference)
            - self.impedance.nullspace_damping * self.arm.joint_velocities
        )
        max_delta = self.impedance.torque_rate_limit * timestep
        torque = np.clip(torque, self._previous_torque - max_delta, self._previous_torque + max_delta)
        torque = np.clip(torque, -self.arm.effort_limits, self.arm.effort_limits)
        self._previous_torque = torque.copy()
        return torque


def rotation_error(current: np.ndarray, desired: np.ndarray) -> np.ndarray:
    current = np.asarray(current, dtype=float).reshape(3, 3)
    desired = np.asarray(desired, dtype=float).reshape(3, 3)
    qc = np.empty(4)
    qd = np.empty(4)
    qc[:] = _matrix_to_quaternion(current)
    qd[:] = _matrix_to_quaternion(desired)
    qerr = _quaternion_multiply(qd, np.array((qc[0], -qc[1], -qc[2], -qc[3])))
    if qerr[0] < 0.0:
        qerr = -qerr
    vector_norm = float(np.linalg.norm(qerr[1:]))
    if vector_norm < 1e-12:
        return np.zeros(3)
    angle = 2.0 * np.arctan2(vector_norm, float(np.clip(qerr[0], -1.0, 1.0)))
    return qerr[1:] * (angle / vector_norm)


def _quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return np.array((
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    ))


def _matrix_to_quaternion(matrix: np.ndarray) -> np.ndarray:
    matrix = np.asarray(matrix, dtype=float).reshape(3, 3)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        quat = np.array((
            0.25 * scale,
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
        ))
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = 2.0 * np.sqrt(max(0.0, 1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]))
            quat = np.array((
                (matrix[2, 1] - matrix[1, 2]) / scale,
                0.25 * scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
            ))
        elif index == 1:
            scale = 2.0 * np.sqrt(max(0.0, 1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]))
            quat = np.array((
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                0.25 * scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
            ))
        else:
            scale = 2.0 * np.sqrt(max(0.0, 1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]))
            quat = np.array((
                (matrix[1, 0] - matrix[0, 1]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                0.25 * scale,
            ))
    norm = np.linalg.norm(quat)
    if norm < 1e-12:
        raise ValueError("rotation matrix produced an invalid quaternion")
    return quat / norm


def downward_tool_rotation(x_axis_world: Sequence[float] = (1.0, 0.0, 0.0)) -> np.ndarray:
    z_axis = np.array((0.0, 0.0, -1.0))
    x_axis = _vector(x_axis_world, 3, "x_axis_world")
    x_axis = x_axis - z_axis * np.dot(z_axis, x_axis)
    norm = np.linalg.norm(x_axis)
    if norm < 1e-9:
        raise ValueError("tool x axis cannot be parallel to tool z axis")
    x_axis /= norm
    y_axis = np.cross(z_axis, x_axis)
    return np.column_stack((x_axis, y_axis, z_axis))


def _vector(values: Sequence[float], size: int, label: str) -> np.ndarray:
    result = np.asarray(values, dtype=float).reshape(-1)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{label} must contain {size} finite values")
    return result
