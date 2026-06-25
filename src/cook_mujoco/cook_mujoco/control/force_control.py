from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np

from cook_mujoco.control.runtime import (
    BodyPose,
    MujocoJointError,
    MujocoModelError,
    require_mujoco,
)


class ForceControlError(RuntimeError):
    pass


class SafetyStop(ForceControlError):
    pass


class ChoppingPhase(str, Enum):
    APPROACH = "APPROACH"
    DESCEND = "DESCEND"
    FORCE_HOLD = "FORCE_HOLD"
    RETRACT = "RETRACT"
    SHIFT = "SHIFT"
    COMPLETE = "COMPLETE"
    FAULT = "FAULT"


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
    contact_enter_n: float = 2.0
    contact_exit_n: float = 1.0
    proportional_gain: float = 0.35
    integral_gain: float = 1.2
    integral_limit_n: float = 8.0
    maximum_command_n: float = 30.0
    feedback_alpha: float = 0.18
    admittance_gain_m_per_ns: float = 0.03
    maximum_position_offset_m: float = 0.04


@dataclass(frozen=True)
class SafetyConfig:
    maximum_force_n: float = 30.0
    maximum_descent_m: float = 0.05
    contact_timeout_s: float = 2.0
    maximum_position_error_m: float = 0.12


@dataclass(frozen=True)
class ChoppingConfig:
    cycles: int = 5
    safe_height_m: float = 0.08
    spacing_m: float = 0.02
    descent_speed_m_s: float = 0.05
    retract_speed_m_s: float = 0.08
    force_hold_s: float = 0.5
    target_force_n: float = 10.0
    control_hz: float = 500.0


@dataclass(frozen=True)
class ForceControlSample:
    time_s: float
    phase: ChoppingPhase
    control_mode: str
    target_position: tuple[float, float, float]
    actual_position: tuple[float, float, float]
    target_force_n: float
    measured_force_n: float
    raw_wrench: tuple[float, ...]
    compensated_wrench: tuple[float, ...]
    joint_positions: tuple[float, ...]
    joint_velocities: tuple[float, ...]
    joint_torques: tuple[float, ...]
    contact: bool
    fault: str = ""


class ForceControlRuntime:
    """Torque-level MuJoCo runtime for one seven-axis arm."""

    def __init__(
        self,
        model,
        data,
        mujoco_module,
        *,
        joint_names: Sequence[str],
        actuator_names: Sequence[str],
        tool_site: str = "tool_tip_site",
        sensor_site: str = "force_sensor_site",
        force_sensor: str = "tool_force",
        torque_sensor: str = "tool_torque",
    ) -> None:
        self.model = model
        self.data = data
        self._mujoco = mujoco_module
        self.joint_names = tuple(joint_names)
        self.actuator_names = tuple(actuator_names)
        if len(self.joint_names) != 7 or len(self.actuator_names) != 7:
            raise MujocoModelError("force control requires seven joints and actuators")
        self._joint_ids = np.array([self._id(mujoco_module.mjtObj.mjOBJ_JOINT, n) for n in self.joint_names])
        self._qpos = np.array([model.jnt_qposadr[i] for i in self._joint_ids], dtype=int)
        self._dofs = np.array([model.jnt_dofadr[i] for i in self._joint_ids], dtype=int)
        self._actuators = np.array([self._id(mujoco_module.mjtObj.mjOBJ_ACTUATOR, n) for n in self.actuator_names])
        self.tool_site_name = tool_site
        self.sensor_site_name = sensor_site
        self._tool_site = self._id(mujoco_module.mjtObj.mjOBJ_SITE, tool_site)
        self._sensor_site = self._id(mujoco_module.mjtObj.mjOBJ_SITE, sensor_site)
        self._force_sensor = self._id(mujoco_module.mjtObj.mjOBJ_SENSOR, force_sensor)
        self._torque_sensor = self._id(mujoco_module.mjtObj.mjOBJ_SENSOR, torque_sensor)
        self._force_slice = self._sensor_slice(self._force_sensor, 3)
        self._torque_slice = self._sensor_slice(self._torque_sensor, 3)
        self.effort_limits = np.max(np.abs(model.actuator_ctrlrange[self._actuators]), axis=1)
        self.timestep = float(model.opt.timestep)

    @classmethod
    def load(
        cls,
        model_path: str | Path,
        joint_names: Sequence[str] | None = None,
    ) -> "ForceControlRuntime":
        path = Path(model_path).expanduser()
        if not path.is_file():
            raise MujocoModelError(f"MuJoCo model file does not exist: {path}")
        mujoco = require_mujoco()
        try:
            model = mujoco.MjModel.from_xml_path(str(path))
            data = mujoco.MjData(model)
        except Exception as exc:
            raise MujocoModelError(f"failed to load force-control model: {path}") from exc
        names = tuple(joint_names or (f"Joint{i}_L" for i in range(1, 8)))
        actuators = tuple(f"{name}_motor" for name in names)
        runtime = cls(model, data, mujoco, joint_names=names, actuator_names=actuators)
        runtime.reset()
        return runtime

    def _id(self, object_type, name: str) -> int:
        value = int(self._mujoco.mj_name2id(self.model, object_type, name))
        if value < 0:
            raise MujocoModelError(f"MuJoCo object not found: {name}")
        return value

    def _sensor_slice(self, sensor_id: int, width: int) -> slice:
        start = int(self.model.sensor_adr[sensor_id])
        actual = int(self.model.sensor_dim[sensor_id])
        if actual != width:
            raise MujocoModelError(f"sensor width mismatch: expected={width}, got={actual}")
        return slice(start, start + width)

    def reset(self, joint_positions: Sequence[float] | None = None) -> np.ndarray:
        self._mujoco.mj_resetData(self.model, self.data)
        if joint_positions is not None:
            q = _vector(joint_positions, 7, "joint_positions")
            self.data.qpos[self._qpos] = q
        self.data.qvel[self._dofs] = 0.0
        self.data.ctrl[self._actuators] = 0.0
        self._mujoco.mj_forward(self.model, self.data)
        return self.joint_positions.copy()

    @property
    def joint_positions(self) -> np.ndarray:
        return np.asarray(self.data.qpos[self._qpos], dtype=float).copy()

    @property
    def joint_velocities(self) -> np.ndarray:
        return np.asarray(self.data.qvel[self._dofs], dtype=float).copy()

    @property
    def bias_torque(self) -> np.ndarray:
        self._mujoco.mj_forward(self.model, self.data)
        return np.asarray(self.data.qfrc_bias[self._dofs], dtype=float).copy()

    def site_pose(self, name: str | None = None) -> tuple[np.ndarray, np.ndarray]:
        site_id = self._tool_site if name is None else self._id(self._mujoco.mjtObj.mjOBJ_SITE, name)
        return (
            np.asarray(self.data.site_xpos[site_id], dtype=float).copy(),
            np.asarray(self.data.site_xmat[site_id], dtype=float).reshape(3, 3).copy(),
        )

    def tool_pose(self) -> BodyPose:
        position, rotation = self.site_pose()
        quaternion = np.empty(4)
        self._mujoco.mju_mat2Quat(quaternion, rotation.reshape(-1))
        return BodyPose(tuple(position), tuple(quaternion))

    def site_jacobian(self, name: str | None = None) -> np.ndarray:
        site_id = self._tool_site if name is None else self._id(self._mujoco.mjtObj.mjOBJ_SITE, name)
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        self._mujoco.mj_jacSite(self.model, self.data, jacp, jacr, site_id)
        return np.vstack((jacp[:, self._dofs], jacr[:, self._dofs]))

    def tool_velocity(self) -> np.ndarray:
        return self.site_jacobian() @ self.joint_velocities

    def raw_wrench(self) -> np.ndarray:
        return np.concatenate((self.data.sensordata[self._force_slice], self.data.sensordata[self._torque_slice])).copy()

    def step(self, torque_nm: Sequence[float]) -> np.ndarray:
        torque = _vector(torque_nm, 7, "torque_nm")
        torque = np.clip(torque, -self.effort_limits, self.effort_limits)
        self.data.ctrl[:] = 0.0
        self.data.ctrl[self._actuators] = torque
        self._mujoco.mj_step(self.model, self.data)
        if not np.all(np.isfinite(self.data.qpos)) or not np.all(np.isfinite(self.data.qvel)):
            raise SafetyStop("MuJoCo state contains NaN or infinity")
        return torque.copy()

    def solve_ik(
        self,
        target_position: Sequence[float],
        target_rotation: np.ndarray,
        *,
        initial: Sequence[float] | None = None,
        max_iterations: int = 500,
        tolerance: float = 2e-4,
        damping: float = 0.04,
    ) -> np.ndarray:
        target_p = _vector(target_position, 3, "target_position")
        target_r = np.asarray(target_rotation, dtype=float).reshape(3, 3)
        q = self.joint_positions if initial is None else _vector(initial, 7, "initial")
        lower = self.model.jnt_range[self._joint_ids, 0]
        upper = self.model.jnt_range[self._joint_ids, 1]
        original = self.joint_positions
        try:
            for _ in range(max_iterations):
                self.data.qpos[self._qpos] = q
                self.data.qvel[self._dofs] = 0.0
                self._mujoco.mj_forward(self.model, self.data)
                position, rotation = self.site_pose()
                error = np.concatenate((target_p - position, rotation_error(rotation, target_r)))
                if np.linalg.norm(error[:3]) < tolerance and np.linalg.norm(error[3:]) < 5 * tolerance:
                    return q.copy()
                jacobian = self.site_jacobian()
                lhs = jacobian @ jacobian.T + (damping**2) * np.eye(6)
                dq = jacobian.T @ np.linalg.solve(lhs, error)
                scale = max(1.0, np.max(np.abs(dq)) / 0.08)
                q = np.clip(q + dq / scale, lower + 0.01, upper - 0.01)
        finally:
            self.data.qpos[self._qpos] = original
            self.data.qvel[self._dofs] = 0.0
            self._mujoco.mj_forward(self.model, self.data)
        raise MujocoJointError("inverse kinematics did not converge")


class WrenchCalibrator:
    """Remove static sensor bias and compensate gravity as tool orientation changes."""

    def __init__(self, *, tool_mass_kg: float = 0.22, gravity: Sequence[float] = (0.0, 0.0, -9.81)):
        self.tool_mass_kg = float(tool_mass_kg)
        self.gravity_world = _vector(gravity, 3, "gravity")
        self._residual = np.zeros(6)
        self._calibrated = False

    def modeled_gravity_wrench(self, sensor_rotation_world: np.ndarray) -> np.ndarray:
        rotation = np.asarray(sensor_rotation_world, dtype=float).reshape(3, 3)
        # MuJoCo force/torque sensors report the child-on-parent wrench.
        force = -(rotation.T @ (self.tool_mass_kg * self.gravity_world))
        return np.concatenate((force, np.zeros(3)))

    def calibrate(self, samples: Iterable[Sequence[float]], sensor_rotation_world: np.ndarray) -> None:
        values = np.asarray(list(samples), dtype=float)
        if values.ndim != 2 or values.shape[1] != 6 or len(values) == 0:
            raise ForceControlError("calibration requires one or more six-axis samples")
        if not np.all(np.isfinite(values)):
            raise ForceControlError("calibration samples contain NaN or infinity")
        self._residual = np.mean(values, axis=0) - self.modeled_gravity_wrench(sensor_rotation_world)
        self._calibrated = True

    def compensate(self, raw_wrench: Sequence[float], sensor_rotation_world: np.ndarray) -> np.ndarray:
        if not self._calibrated:
            raise ForceControlError("wrench sensor has not been calibrated")
        raw = _vector(raw_wrench, 6, "raw_wrench")
        return raw - self._residual - self.modeled_gravity_wrench(sensor_rotation_world)


class CartesianForceController:
    def __init__(
        self,
        runtime: ForceControlRuntime,
        *,
        impedance: CartesianImpedanceConfig | None = None,
        force: ForceControlConfig | None = None,
    ) -> None:
        self.runtime = runtime
        self.impedance = impedance or CartesianImpedanceConfig()
        self.force = force or ForceControlConfig()
        self.desired_position, self.desired_rotation = runtime.site_pose()
        self.nullspace_reference = runtime.joint_positions
        self.force_enabled = False
        self._force_integral = 0.0
        self._filtered_force = 0.0
        self._previous_torque = np.zeros(7)
        self._force_position_offset = 0.0
        self._last_debug: dict[str, object] = {}

    @property
    def measured_force_n(self) -> float:
        return float(self._filtered_force)

    def debug_state(self) -> dict[str, object]:
        """Return a copy of the latest controller pipeline values."""
        result = {}
        for key, value in self._last_debug.items():
            result[key] = value.copy() if isinstance(value, np.ndarray) else value
        return result

    def set_target(self, position: Sequence[float], rotation: np.ndarray) -> None:
        self.desired_position = _vector(position, 3, "position")
        self.desired_rotation = np.asarray(rotation, dtype=float).reshape(3, 3).copy()

    def enable_force(self, enabled: bool, *, target_force_n: float | None = None) -> None:
        if target_force_n is not None:
            if target_force_n < 0:
                raise ForceControlError("target force must be non-negative")
            self.force = ForceControlConfig(**{**self.force.__dict__, "target_force_n": float(target_force_n)})
        if bool(enabled) != self.force_enabled:
            self._force_integral = 0.0
            self._force_position_offset = 0.0
        self.force_enabled = bool(enabled)

    def compute(self, compensated_wrench_tool: Sequence[float]) -> np.ndarray:
        wrench_tool = _vector(compensated_wrench_tool, 6, "compensated_wrench_tool")
        position, rotation = self.runtime.site_pose()
        jacobian = self.runtime.site_jacobian()
        velocity = jacobian @ self.runtime.joint_velocities
        orientation_error = rotation_error(rotation, self.desired_rotation)
        kp = np.asarray(self.impedance.translational_stiffness)
        kd = np.asarray(self.impedance.translational_damping)
        kr = np.asarray(self.impedance.rotational_stiffness)
        dr = np.asarray(self.impedance.rotational_damping)

        tool_axis_world = rotation[:, 2]
        measured = float(wrench_tool[2])
        alpha = self.force.feedback_alpha
        self._filtered_force = (1.0 - alpha) * self._filtered_force + alpha * measured
        effective_target = self.desired_position.copy()
        if self.force_enabled:
            error = self.force.target_force_n - self._filtered_force
            self._force_position_offset = float(np.clip(
                self._force_position_offset
                + self.force.admittance_gain_m_per_ns * error * self.runtime.timestep,
                -self.force.maximum_position_offset_m,
                self.force.maximum_position_offset_m,
            ))
            effective_target += self._force_position_offset * tool_axis_world

        position_error = effective_target - position
        force_world = kp * position_error - kd * velocity[:3]
        torque_world = kr * orientation_error - dr * velocity[3:]

        task_wrench = np.concatenate((force_world, torque_world))
        torque = jacobian.T @ task_wrench + self.runtime.bias_torque
        q = self.runtime.joint_positions
        qd = self.runtime.joint_velocities
        null_projector = np.eye(7) - jacobian.T @ np.linalg.pinv(jacobian.T)
        null_torque = -self.impedance.nullspace_stiffness * (q - self.nullspace_reference) - self.impedance.nullspace_damping * qd
        torque += null_projector @ null_torque
        max_delta = self.impedance.torque_rate_limit * self.runtime.timestep
        torque = np.clip(torque, self._previous_torque - max_delta, self._previous_torque + max_delta)
        torque = np.clip(torque, -self.runtime.effort_limits, self.runtime.effort_limits)
        self._previous_torque = torque.copy()
        self._last_debug = {
            "force_enabled": self.force_enabled,
            "raw_input_wrench_tool": wrench_tool.copy(),
            "filtered_force_n": float(self._filtered_force),
            "target_force_n": float(self.force.target_force_n),
            "force_error_n": float(self.force.target_force_n - self._filtered_force),
            "admittance_offset_m": float(self._force_position_offset),
            "desired_position": self.desired_position.copy(),
            "effective_target_position": effective_target.copy(),
            "actual_position": position.copy(),
            "position_error": position_error.copy(),
            "orientation_error": orientation_error.copy(),
            "tool_axis_world": tool_axis_world.copy(),
            "commanded_force_world": force_world.copy(),
            "commanded_torque_world": torque_world.copy(),
            "task_wrench_world": task_wrench.copy(),
            "joint_torque_command": torque.copy(),
        }
        return torque


def rotation_error(current: np.ndarray, desired: np.ndarray) -> np.ndarray:
    """World-frame rotation vector from current orientation to desired.

    Quaternion differencing avoids the zero error produced by the common
    column-cross-product formula at rotations close to 180 degrees.
    """
    current = np.asarray(current, dtype=float).reshape(3, 3)
    desired = np.asarray(desired, dtype=float).reshape(3, 3)
    qc = np.empty(4)
    qd = np.empty(4)
    # Matrix-to-quaternion is implemented locally so this helper remains
    # independent of a runtime instance.
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
        quat = np.array((0.25 * scale, (matrix[2, 1] - matrix[1, 2]) / scale, (matrix[0, 2] - matrix[2, 0]) / scale, (matrix[1, 0] - matrix[0, 1]) / scale))
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = 2.0 * np.sqrt(max(0.0, 1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]))
            quat = np.array(((matrix[2, 1] - matrix[1, 2]) / scale, 0.25 * scale, (matrix[0, 1] + matrix[1, 0]) / scale, (matrix[0, 2] + matrix[2, 0]) / scale))
        elif index == 1:
            scale = 2.0 * np.sqrt(max(0.0, 1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]))
            quat = np.array(((matrix[0, 2] - matrix[2, 0]) / scale, (matrix[0, 1] + matrix[1, 0]) / scale, 0.25 * scale, (matrix[1, 2] + matrix[2, 1]) / scale))
        else:
            scale = 2.0 * np.sqrt(max(0.0, 1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]))
            quat = np.array(((matrix[1, 0] - matrix[0, 1]) / scale, (matrix[0, 2] + matrix[2, 0]) / scale, (matrix[1, 2] + matrix[2, 1]) / scale, 0.25 * scale))
    norm = np.linalg.norm(quat)
    if norm < 1e-12:
        raise ForceControlError("rotation matrix produced an invalid quaternion")
    return quat / norm


def downward_tool_rotation(x_axis_world: Sequence[float] = (1.0, 0.0, 0.0)) -> np.ndarray:
    z_axis = np.array((0.0, 0.0, -1.0))
    x_axis = _vector(x_axis_world, 3, "x_axis_world")
    x_axis = x_axis - z_axis * np.dot(z_axis, x_axis)
    norm = np.linalg.norm(x_axis)
    if norm < 1e-9:
        raise ForceControlError("tool x axis cannot be parallel to tool z axis")
    x_axis /= norm
    y_axis = np.cross(z_axis, x_axis)
    return np.column_stack((x_axis, y_axis, z_axis))


def interpolate_linear(start: Sequence[float], end: Sequence[float], fraction: float) -> np.ndarray:
    a = _vector(start, 3, "start")
    b = _vector(end, 3, "end")
    s = float(np.clip(fraction, 0.0, 1.0))
    smooth = s * s * (3.0 - 2.0 * s)
    return a + smooth * (b - a)


def _vector(values: Sequence[float], size: int, label: str) -> np.ndarray:
    result = np.asarray(values, dtype=float).reshape(-1)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ForceControlError(f"{label} must contain {size} finite values")
    return result
