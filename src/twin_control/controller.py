"""Unified torque-mode controller for 7-DOF robot arms.

Provides three control strategies behind a single ``compute()`` interface:

- **Position / joint impedance**:  τ = K·(q_des − q) + D·(0 − qd) + bias
- **Cartesian impedance**:  τ = Jᵀ·F_task + nullspace + bias
- **Force control**:  hybrid Cartesian impedance + admittance force tracking
  on one axis

All internal calculations use SI units.

Public API
----------
ControlMode              enum: POSITION / JOINT_IMPEDANCE / CARTESIAN_IMPEDANCE / FORCE
JointImpedanceParams     dataclass: stiffness (7,), damping (7,)
CartesianImpedanceParams dataclass: translational + rotational stiffness/damping, nullspace
ForceControlParams       dataclass: target_force_n, direction, admittance_gain, etc.
UnifiedController(joint_limits, torque_limits, rate_limit, dt_s)
    .set_mode(mode)
    .set_joint_impedance_params(params) / .set_cartesian_impedance_params(params) / .set_force_params(params)
    .set_joint_cmd(joints_rad) / .set_cart_cmd(pose_matrix) / .set_force_cmd(force_n)
    .compute(q, qd, J, pose, wrench, bias) → (7,) joint torques (N·m)
    .force_axis_world(pose) / .measured_force_along_axis(wrench, pose)
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Sequence

import numpy as np

from twin_control.rotation import matrix_to_quat as _matrix_to_quat
from twin_control.rotation import quat_mul as _quat_mul
from twin_control.rotation import rotation_error as _rotation_error


# ---------------------------------------------------------------------------
# Control mode enum
# ---------------------------------------------------------------------------

class ControlMode(Enum):
    """Torque-mode control strategies."""
    POSITION = 1             # joint position tracking
    JOINT_IMPEDANCE = 2      # τ = K·(q_des − q) + D·(0 − qd)
    CARTESIAN_IMPEDANCE = 3  # τ = Jᵀ·F_task + nullspace
    FORCE = 4                # hybrid Cartesian + admittance force


# ---------------------------------------------------------------------------
# Parameter dataclasses
# ---------------------------------------------------------------------------

@dataclass
class JointImpedanceParams:
    """Joint-space impedance parameters.

    Attributes
    ----------
    stiffness : (7,) tuple — joint stiffness (N·m/rad).
    damping : (7,) tuple — joint damping (N·m/(rad/s)).
    """
    stiffness: tuple[float, float, float, float, float, float, float]
    damping: tuple[float, float, float, float, float, float, float]


@dataclass
class CartesianImpedanceParams:
    """Cartesian-space impedance parameters.

    Attributes
    ----------
    translational_stiffness : (3,) tuple — N/m in X, Y, Z.
    translational_damping : (3,) tuple — N/(m/s) in X, Y, Z.
    rotational_stiffness : (3,) tuple — N·m/rad for RX, RY, RZ.
    rotational_damping : (3,) tuple — N·m/(rad/s) for RX, RY, RZ.
    nullspace_stiffness : float — joint-space nullspace stiffness (N·m/rad).
    nullspace_damping : float — joint-space nullspace damping (N·m/(rad/s)).
    """
    translational_stiffness: tuple[float, float, float] = (2500.0, 2500.0, 2800.0)
    translational_damping: tuple[float, float, float] = (105.0, 105.0, 115.0)
    rotational_stiffness: tuple[float, float, float] = (45.0, 45.0, 35.0)
    rotational_damping: tuple[float, float, float] = (5.5, 5.5, 4.5)
    nullspace_stiffness: float = 4.0
    nullspace_damping: float = 1.5


@dataclass
class ForceControlParams:
    """Hybrid force-control parameters.

    Attributes
    ----------
    direction : (6,) tuple — 6-D force/moment direction (e.g. (0,0,1,0,0,0) = Z force).
    admittance_gain_m_per_ns : float — position adjustment per force error (m/(N·s)).
    max_position_offset_m : float — maximum admittance position adjustment (m).
    feedback_alpha : float — low-pass filter coefficient (0, 1].
    """
    target_force_n: float = 10.0
    direction: tuple[float, float, float, float, float, float] = (0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
    admittance_gain_m_per_ns: float = 0.03
    max_position_offset_m: float = 0.04
    feedback_alpha: float = 0.18


# ---------------------------------------------------------------------------
# UnifiedController
# ---------------------------------------------------------------------------

class UnifiedController:
    """Unified torque-mode controller.

    Parameters
    ----------
    joint_limits_rad : (7, 2) ndarray — per-joint (lower, upper) limits.
    torque_limits_nm : (7,) ndarray — per-joint maximum torque (N·m).
    torque_rate_limit_nm_per_s : float — max torque change rate (N·m/s). Default 1500.
    dt_s : float — control timestep (s). Default 0.002 (500 Hz).
    """

    def __init__(
        self,
        joint_limits_rad: np.ndarray,
        torque_limits_nm: np.ndarray,
        torque_rate_limit_nm_per_s: float = 1500.0,
        dt_s: float = 0.002,
    ) -> None:
        self._joint_limits = np.asarray(joint_limits_rad, dtype=float).reshape(7, 2)
        self._torque_limits = np.asarray(torque_limits_nm, dtype=float).reshape(7)
        self._rate_limit = float(torque_rate_limit_nm_per_s)
        self._dt = float(dt_s)

        self._mode = ControlMode.POSITION
        self._joint_imp_params = JointImpedanceParams(stiffness=(8.0,) * 7, damping=(1.5,) * 7)
        self._cart_imp_params = CartesianImpedanceParams()
        self._force_params = ForceControlParams()

        self._joint_target_rad: np.ndarray | None = None
        self._cart_target_matrix: np.ndarray | None = None
        self._force_target_n: float = 0.0

        self._previous_torque = np.zeros(7)
        self._nullspace_ref_rad: np.ndarray | None = None
        self._force_offset_m = 0.0
        self._filtered_force_n = 0.0

    # ------------------------------------------------------------------
    # Mode & parameter setting
    # ------------------------------------------------------------------

    def set_mode(self, mode: ControlMode) -> None:
        """Switch control mode. Resets force state when leaving FORCE mode."""
        self._mode = mode
        if mode != ControlMode.FORCE:
            self._force_offset_m = 0.0
            self._filtered_force_n = 0.0

    def set_joint_impedance_params(self, params: JointImpedanceParams) -> None:
        _validate_7(params.stiffness, "stiffness")
        _validate_7(params.damping, "damping")
        self._joint_imp_params = params

    def set_cartesian_impedance_params(self, params: CartesianImpedanceParams) -> None:
        _validate_3(params.translational_stiffness, "translational_stiffness")
        _validate_3(params.translational_damping, "translational_damping")
        _validate_3(params.rotational_stiffness, "rotational_stiffness")
        _validate_3(params.rotational_damping, "rotational_damping")
        self._cart_imp_params = params

    def set_force_params(self, params: ForceControlParams) -> None:
        _validate_6(params.direction, "direction")
        axis = np.asarray(params.direction[:3], dtype=float).reshape(3)
        if float(np.linalg.norm(axis)) < 1e-12:
            raise ValueError("force direction translation axis must be non-zero")
        self._force_params = params

    # ------------------------------------------------------------------
    # Force axis helpers
    # ------------------------------------------------------------------

    def force_axis_world(self, pose_matrix: np.ndarray | None = None) -> np.ndarray:
        """Return the force-control axis expressed in world frame.

        Uses the configured direction if set; otherwise defaults to the
        tool Z axis from *pose_matrix*.
        """
        axis = np.asarray(self._force_params.direction[:3], dtype=float).reshape(3)
        norm = float(np.linalg.norm(axis))
        if norm < 1e-12:
            if pose_matrix is None:
                return np.array((0.0, 0.0, 1.0), dtype=float)
            return np.asarray(pose_matrix, dtype=float).reshape(4, 4)[:3, 2].copy()
        return axis / norm

    def measured_force_along_axis(self, wrench: np.ndarray, pose_matrix: np.ndarray) -> float:
        """Project the tool-frame wrench onto the configured force axis.

        Parameters
        ----------
        wrench : (6,) ndarray — force/torque in sensor (tool) frame.
        pose_matrix : (4, 4) ndarray — current end-effector pose.

        Returns
        -------
        float — force component along the configured axis (N).
        """
        w = np.asarray(wrench, dtype=float).reshape(6)
        rotation = np.asarray(pose_matrix, dtype=float).reshape(4, 4)[:3, :3]
        force_world = rotation @ w[:3]
        return float(np.dot(force_world, self.force_axis_world(pose_matrix)))

    # ------------------------------------------------------------------
    # Target commands
    # ------------------------------------------------------------------

    def set_joint_cmd(self, target_joints_rad: np.ndarray) -> None:
        """Set joint-space target (radians). Also updates nullspace reference."""
        q = np.asarray(target_joints_rad, dtype=float).reshape(7)
        if not np.all(np.isfinite(q)):
            raise ValueError("target joints must be finite")
        self._joint_target_rad = q.copy()
        self._nullspace_ref_rad = q.copy()

    def set_cart_cmd(self, target_pose_matrix: np.ndarray) -> None:
        """Set Cartesian target pose as a 4×4 homogeneous transform."""
        T = np.asarray(target_pose_matrix, dtype=float).reshape(4, 4)
        if not np.all(np.isfinite(T)):
            raise ValueError("target pose must be finite")
        self._cart_target_matrix = T.copy()

    def set_force_cmd(self, force_n: float) -> None:
        """Set force target (Newtons). Must be non-negative."""
        if not math.isfinite(force_n) or force_n < 0:
            raise ValueError("force target must be non-negative finite")
        self._force_target_n = float(force_n)

    # ------------------------------------------------------------------
    # Main compute
    # ------------------------------------------------------------------

    def compute(
        self,
        current_joints_rad: np.ndarray,
        current_velocities_rad_s: np.ndarray,
        jacobian: np.ndarray,
        current_pose_matrix: np.ndarray | None = None,
        wrench: np.ndarray | None = None,
        bias_torque: np.ndarray | None = None,
    ) -> np.ndarray:
        """Compute joint torques for the current control mode.

        Parameters
        ----------
        current_joints_rad : (7,) ndarray — joint positions (rad).
        current_velocities_rad_s : (7,) ndarray — joint velocities (rad/s).
        jacobian : (6, 7) ndarray — geometric Jacobian.
        current_pose_matrix : (4, 4) ndarray or None — end-effector pose (required
            for Cartesian/force modes).
        wrench : (6,) ndarray or None — compensated wrench (required for force mode).
        bias_torque : (7,) ndarray or None — gravity compensation (added to all modes).

        Returns
        -------
        (7,) ndarray — joint torques (N·m), clamped by torque and rate limits.
        """
        q = np.asarray(current_joints_rad, dtype=float).reshape(7)
        qd = np.asarray(current_velocities_rad_s, dtype=float).reshape(7)
        J = np.asarray(jacobian, dtype=float).reshape(6, 7)
        bias = (np.zeros(7) if bias_torque is None
                else np.asarray(bias_torque, dtype=float).reshape(7))

        _check_finite(q, "current_joints_rad")
        _check_finite(qd, "current_velocities_rad_s")

        # Dispatch to control law
        if self._mode == ControlMode.JOINT_IMPEDANCE:
            tau = self._joint_impedance(q, qd, bias)
        elif self._mode == ControlMode.CARTESIAN_IMPEDANCE:
            tau = self._cartesian_impedance(q, qd, J, current_pose_matrix, bias)
        elif self._mode == ControlMode.POSITION:
            tau = self._joint_impedance(q, qd, bias)
        else:  # FORCE
            tau = self._force_control(q, qd, J, current_pose_matrix, wrench, bias)

        # Safety: torque rate + magnitude limiting
        tau = _apply_torque_limits(tau, self._previous_torque,
                                   self._rate_limit * self._dt, self._torque_limits)
        self._previous_torque = tau.copy()
        return tau

    # ------------------------------------------------------------------
    # Control laws (private)
    # ------------------------------------------------------------------

    def _joint_impedance(self, q: np.ndarray, qd: np.ndarray, bias: np.ndarray) -> np.ndarray:
        """τ = K·(q_des − q) − D·qd + bias."""
        if self._joint_target_rad is None:
            return bias
        K = np.asarray(self._joint_imp_params.stiffness, dtype=float).reshape(7)
        D = np.asarray(self._joint_imp_params.damping, dtype=float).reshape(7)
        return K * (self._joint_target_rad - q) - D * qd + bias

    def _cartesian_impedance(
        self, q: np.ndarray, qd: np.ndarray, J: np.ndarray,
        pose_matrix: np.ndarray | None, bias: np.ndarray,
    ) -> np.ndarray:
        """τ = Jᵀ·F_task + nullspace + bias."""
        if self._cart_target_matrix is None or pose_matrix is None:
            return bias

        e_pos = self._cart_target_matrix[:3, 3] - pose_matrix[:3, 3]
        e_rot = _rotation_error(pose_matrix[:3, :3], self._cart_target_matrix[:3, :3])
        v = J @ qd

        Kp = np.asarray(self._cart_imp_params.translational_stiffness)
        Kd = np.asarray(self._cart_imp_params.translational_damping)
        Kr = np.asarray(self._cart_imp_params.rotational_stiffness)
        Dr = np.asarray(self._cart_imp_params.rotational_damping)

        F_task = np.concatenate((Kp * e_pos - Kd * v[:3], Kr * e_rot - Dr * v[3:]))
        tau = J.T @ F_task + bias
        tau += self._nullspace_torque(q, qd, J)
        return tau

    def _force_control(
        self, q: np.ndarray, qd: np.ndarray, J: np.ndarray,
        pose_matrix: np.ndarray | None, wrench: np.ndarray | None,
        bias: np.ndarray,
    ) -> np.ndarray:
        """Hybrid: Cartesian impedance + admittance-based force tracking."""
        if pose_matrix is None or self._cart_target_matrix is None:
            return bias

        # Low-pass filter force along the configured axis
        if wrench is not None:
            alpha = self._force_params.feedback_alpha
            measured = self.measured_force_along_axis(wrench, pose_matrix)
            self._filtered_force_n = ((1.0 - alpha) * self._filtered_force_n + alpha * measured)

        # Admittance: adjust target position along force axis
        f_err = self._force_target_n - self._filtered_force_n
        self._force_offset_m += self._force_params.admittance_gain_m_per_ns * f_err * self._dt
        self._force_offset_m = float(np.clip(
            self._force_offset_m, -self._force_params.max_position_offset_m,
            self._force_params.max_position_offset_m,
        ))

        force_axis = self.force_axis_world(pose_matrix)
        x_des_eff = self._cart_target_matrix[:3, 3] + self._force_offset_m * force_axis

        e_pos = x_des_eff - pose_matrix[:3, 3]
        e_rot = _rotation_error(pose_matrix[:3, :3], self._cart_target_matrix[:3, :3])
        v = J @ qd

        Kp = np.asarray(self._cart_imp_params.translational_stiffness)
        Kd = np.asarray(self._cart_imp_params.translational_damping)
        Kr = np.asarray(self._cart_imp_params.rotational_stiffness)
        Dr = np.asarray(self._cart_imp_params.rotational_damping)

        F_task = np.concatenate((Kp * e_pos - Kd * v[:3], Kr * e_rot - Dr * v[3:]))
        tau = J.T @ F_task + bias
        tau += self._nullspace_torque(q, qd, J)
        return tau

    # ------------------------------------------------------------------
    # Shared helpers
    # ------------------------------------------------------------------

    def _nullspace_torque(self, q: np.ndarray, qd: np.ndarray, J: np.ndarray) -> np.ndarray:
        """Compute null-space joint torque pulling toward the reference configuration.

        Projects the joint-space spring-damper through the nullspace of J
        so it does not affect the end-effector task.
        """
        if self._nullspace_ref_rad is None:
            return np.zeros(7)
        try:
            J_pinv = np.linalg.lstsq(J, np.eye(6), rcond=None)[0]  # (7, 6)
        except np.linalg.LinAlgError:
            return np.zeros(7)
        null_proj = np.eye(7) - J_pinv @ J  # (7, 7)
        Kn = self._cart_imp_params.nullspace_stiffness
        Dn = self._cart_imp_params.nullspace_damping
        return null_proj @ (-Kn * (q - self._nullspace_ref_rad) - Dn * qd)


# ---------------------------------------------------------------------------
# Safety: torque limiting
# ---------------------------------------------------------------------------

def _apply_torque_limits(
    tau: np.ndarray,
    prev_tau: np.ndarray,
    max_delta: float,
    torque_limits: np.ndarray,
) -> np.ndarray:
    """Clamp *tau* by rate and magnitude limits, with NaN guard."""
    tau = np.clip(tau, prev_tau - max_delta, prev_tau + max_delta)
    tau = np.clip(tau, -torque_limits, torque_limits)
    if not np.all(np.isfinite(tau)):
        tau = prev_tau.copy()
    return tau


# ---------------------------------------------------------------------------
# Validation helpers
# ---------------------------------------------------------------------------

def _validate_7(values: tuple[float, ...], name: str) -> None:
    if len(values) != 7:
        raise ValueError(f"{name} must have 7 values, got {len(values)}")
    if not all(math.isfinite(v) for v in values):
        raise ValueError(f"{name} must be finite")
    if not all(v >= 0 for v in values):
        raise ValueError(f"{name} must be non-negative")


def _validate_3(values: tuple[float, ...], name: str) -> None:
    if len(values) != 3:
        raise ValueError(f"{name} must have 3 values, got {len(values)}")
    if not all(math.isfinite(v) for v in values):
        raise ValueError(f"{name} must be finite")
    if not all(v >= 0 for v in values):
        raise ValueError(f"{name} must be non-negative")


def _validate_6(values: tuple[float, ...], name: str) -> None:
    if len(values) != 6:
        raise ValueError(f"{name} must have 6 values, got {len(values)}")
    if not all(math.isfinite(v) for v in values):
        raise ValueError(f"{name} must be finite")


def _check_finite(arr: np.ndarray, name: str) -> None:
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} contains NaN or Inf")
