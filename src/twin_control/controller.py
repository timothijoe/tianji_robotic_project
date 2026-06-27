"""Unified impedance/force controller for 7-DOF robot arms.

Provides three torque-mode control strategies behind a single ``compute()``
interface:

- Joint impedance:  τ = K·(q_des − q) + D·(0 − qd) + bias
- Cartesian impedance:  τ = Jᵀ·F_task + nullspace + bias
- Force control:  hybrid Cartesian impedance + admittance-based force tracking

All internal calculations use SI units.  Input/output conversion for SDK
display units (deg, mm) is handled at the ``MarvinKinematics`` boundary.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Sequence

import numpy as np


class ControlMode(Enum):
    POSITION = 1             # stiff Cartesian impedance (no special params)
    JOINT_IMPEDANCE = 2      # τ = K·(q_des − q) + D·(0 − qd)
    CARTESIAN_IMPEDANCE = 3  # τ = Jᵀ·(Kp·(x_des − x) − Kd·v) + nullspace
    FORCE = 4                # hybrid Cartesian + admittance force on one axis


# ---------------------------------------------------------------------------
# Parameter dataclasses
# ---------------------------------------------------------------------------

@dataclass
class JointImpedanceParams:
    stiffness: tuple[float, float, float, float, float, float, float]
    """7 joint stiffness values (N·m/rad)."""
    damping: tuple[float, float, float, float, float, float, float]
    """7 joint damping values (N·m/(rad/s))."""


@dataclass
class CartesianImpedanceParams:
    translational_stiffness: tuple[float, float, float] = (2500.0, 2500.0, 2800.0)
    """X, Y, Z translational stiffness (N/m)."""
    translational_damping: tuple[float, float, float] = (105.0, 105.0, 115.0)
    """X, Y, Z translational damping (N/(m/s))."""
    rotational_stiffness: tuple[float, float, float] = (45.0, 45.0, 35.0)
    """RX, RY, RZ rotational stiffness (N·m/rad)."""
    rotational_damping: tuple[float, float, float] = (5.5, 5.5, 4.5)
    """RX, RY, RZ rotational damping (N·m/(rad/s))."""
    nullspace_stiffness: float = 4.0
    """Null-space joint stiffness (N·m/rad)."""
    nullspace_damping: float = 1.5
    """Null-space joint damping (N·m/(rad/s))."""


@dataclass
class ForceControlParams:
    target_force_n: float = 10.0
    """Target force in Newtons."""
    direction: tuple[float, float, float, float, float, float] = (0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
    """6-D force/moment direction vector (unit).  E.g. (0,0,1,0,0,0) = Z force."""
    admittance_gain_m_per_ns: float = 0.03
    """Admittance gain (m/(N·s))."""
    max_position_offset_m: float = 0.04
    """Maximum admittance position offset (m)."""
    feedback_alpha: float = 0.18
    """Low-pass filter coefficient for force feedback (0, 1]."""


# ---------------------------------------------------------------------------
# UnifiedController
# ---------------------------------------------------------------------------

class UnifiedController:
    """Unified torque-mode controller supporting joint impedance, Cartesian
    impedance, and force control.

    Parameters
    ----------
    joint_limits_rad : (7, 2) ndarray
        Per-joint (lower, upper) limits in radians.
    torque_limits_nm : (7,) ndarray
        Per-joint maximum torque magnitude (N·m).
    torque_rate_limit_nm_per_s : float
        Maximum torque change rate (N·m/s).  Default 1500.
    dt_s : float
        Control timestep in seconds (used for rate limiting and admittance
        integration).  Default 0.002 (500 Hz).
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

        # Mode & parameters
        self._mode = ControlMode.POSITION
        self._joint_imp_params = JointImpedanceParams(
            stiffness=(8.0,) * 7, damping=(1.5,) * 7,
        )
        self._cart_imp_params = CartesianImpedanceParams()
        self._force_params = ForceControlParams()

        # Targets
        self._joint_target_rad: np.ndarray | None = None
        self._cart_target_matrix: np.ndarray | None = None
        self._force_target_n: float = 0.0

        # State
        self._previous_torque = np.zeros(7)
        self._nullspace_ref_rad: np.ndarray | None = None
        self._force_offset_m = 0.0
        self._filtered_force_n = 0.0

    # ------------------------------------------------------------------
    # Mode & parameter setting
    # ------------------------------------------------------------------

    def set_mode(self, mode: ControlMode) -> None:
        self._mode = mode
        # Reset force state when switching to non-force modes
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
        self._force_params = params

    # ------------------------------------------------------------------
    # Target commands
    # ------------------------------------------------------------------

    def set_joint_cmd(self, target_joints_rad: np.ndarray) -> None:
        """Set joint-space target (radians)."""
        q = np.asarray(target_joints_rad, dtype=float).reshape(7)
        if not np.all(np.isfinite(q)):
            raise ValueError("target joints must be finite")
        self._joint_target_rad = q.copy()
        self._nullspace_ref_rad = q.copy()

    def set_cart_cmd(self, target_pose_matrix: np.ndarray) -> None:
        """Set Cartesian target pose as 4×4 homogeneous transform."""
        T = np.asarray(target_pose_matrix, dtype=float).reshape(4, 4)
        if not np.all(np.isfinite(T)):
            raise ValueError("target pose must be finite")
        self._cart_target_matrix = T.copy()

    def set_force_cmd(self, force_n: float) -> None:
        """Set force target in Newtons."""
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
        current_joints_rad : (7,) ndarray
            Current joint positions (rad).
        current_velocities_rad_s : (7,) ndarray
            Current joint velocities (rad/s).
        jacobian : (6, 7) ndarray
            Geometric Jacobian at the current configuration.
        current_pose_matrix : (4, 4) ndarray or None
            Current end-effector pose (required for Cartesian/force modes).
        wrench : (6,) ndarray or None
            Compensated force-torque at end-effector (required for force mode).
        bias_torque : (7,) ndarray or None
            Gravity compensation torque (added to all modes).

        Returns
        -------
        (7,) ndarray — joint torques (N·m), clamped by limits and rate.
        """
        q = np.asarray(current_joints_rad, dtype=float).reshape(7)
        qd = np.asarray(current_velocities_rad_s, dtype=float).reshape(7)
        J = np.asarray(jacobian, dtype=float).reshape(6, 7)
        bias = (np.zeros(7) if bias_torque is None
                else np.asarray(bias_torque, dtype=float).reshape(7))

        _check_finite(q, "current_joints_rad")
        _check_finite(qd, "current_velocities_rad_s")

        if self._mode == ControlMode.JOINT_IMPEDANCE:
            tau = self._joint_impedance(q, qd, bias)
        elif self._mode == ControlMode.CARTESIAN_IMPEDANCE:
            tau = self._cartesian_impedance(q, qd, J, current_pose_matrix, bias)
        elif self._mode == ControlMode.FORCE:
            tau = self._force_control(q, qd, J, current_pose_matrix, wrench, bias)
        else:  # POSITION — stiff Cartesian
            tau = self._cartesian_impedance(q, qd, J, current_pose_matrix, bias)

        # Safety: torque rate limiting
        max_delta = self._rate_limit * self._dt
        tau = np.clip(tau, self._previous_torque - max_delta, self._previous_torque + max_delta)

        # Safety: torque magnitude limiting
        tau = np.clip(tau, -self._torque_limits, self._torque_limits)

        # Safety: NaN guard
        if not np.all(np.isfinite(tau)):
            tau = self._previous_torque.copy()

        self._previous_torque = tau.copy()
        return tau

    # ------------------------------------------------------------------
    # Control laws (private)
    # ------------------------------------------------------------------

    def _joint_impedance(
        self, q: np.ndarray, qd: np.ndarray, bias: np.ndarray,
    ) -> np.ndarray:
        if self._joint_target_rad is None:
            return bias
        K = np.asarray(self._joint_imp_params.stiffness, dtype=float).reshape(7)
        D = np.asarray(self._joint_imp_params.damping, dtype=float).reshape(7)
        return K * (self._joint_target_rad - q) - D * qd + bias

    def _cartesian_impedance(
        self, q: np.ndarray, qd: np.ndarray, J: np.ndarray,
        pose_matrix: np.ndarray | None, bias: np.ndarray,
    ) -> np.ndarray:
        if self._cart_target_matrix is None or pose_matrix is None:
            return bias

        # Position error
        x_des = self._cart_target_matrix[:3, 3]
        x_cur = pose_matrix[:3, 3]
        e_pos = x_des - x_cur

        # Orientation error
        e_rot = _rotation_error(pose_matrix[:3, :3], self._cart_target_matrix[:3, :3])

        # End-effector velocity
        v = J @ qd

        # Task wrench
        Kp = np.asarray(self._cart_imp_params.translational_stiffness)
        Kd = np.asarray(self._cart_imp_params.translational_damping)
        Kr = np.asarray(self._cart_imp_params.rotational_stiffness)
        Dr = np.asarray(self._cart_imp_params.rotational_damping)

        F_task = np.concatenate((
            Kp * e_pos - Kd * v[:3],
            Kr * e_rot - Dr * v[3:],
        ))

        tau = J.T @ F_task

        # Null-space projection
        if self._nullspace_ref_rad is not None:
            try:
                # J is (6,7). Compute J^+ (7,6) via lstsq: J @ X = I_6 → X = (7,6)
                J_pinv = np.linalg.lstsq(J, np.eye(6), rcond=None)[0]  # (7, 6)
            except np.linalg.LinAlgError:
                J_pinv = np.zeros((7, 6))
            null_proj = np.eye(7) - J_pinv @ J  # (7,7) - (7,6)@(6,7) = (7,7)
            Kn = self._cart_imp_params.nullspace_stiffness
            Dn = self._cart_imp_params.nullspace_damping
            tau += null_proj @ (-Kn * (q - self._nullspace_ref_rad) - Dn * qd)

        return tau + bias

    def _force_control(
        self, q: np.ndarray, qd: np.ndarray, J: np.ndarray,
        pose_matrix: np.ndarray | None, wrench: np.ndarray | None,
        bias: np.ndarray,
    ) -> np.ndarray:
        if pose_matrix is None or self._cart_target_matrix is None:
            return bias

        # Low-pass filter the force measurement
        if wrench is not None:
            w = np.asarray(wrench, dtype=float).reshape(6)
            alpha = self._force_params.feedback_alpha
            self._filtered_force_n = (
                (1.0 - alpha) * self._filtered_force_n
                + alpha * float(w[2])  # Z-axis force
            )

        # Admittance: adjust target position along tool Z axis
        f_err = self._force_target_n - self._filtered_force_n
        self._force_offset_m += (
            self._force_params.admittance_gain_m_per_ns * f_err * self._dt
        )
        self._force_offset_m = float(np.clip(
            self._force_offset_m,
            -self._force_params.max_position_offset_m,
            self._force_params.max_position_offset_m,
        ))

        # Adjust Cartesian target along end-effector Z axis
        z_ee = pose_matrix[:3, 2]
        x_des_eff = self._cart_target_matrix[:3, 3] + self._force_offset_m * z_ee

        x_cur = pose_matrix[:3, 3]
        e_pos = x_des_eff - x_cur
        e_rot = _rotation_error(pose_matrix[:3, :3], self._cart_target_matrix[:3, :3])
        v = J @ qd

        Kp = np.asarray(self._cart_imp_params.translational_stiffness)
        Kd = np.asarray(self._cart_imp_params.translational_damping)
        Kr = np.asarray(self._cart_imp_params.rotational_stiffness)
        Dr = np.asarray(self._cart_imp_params.rotational_damping)

        F_task = np.concatenate((
            Kp * e_pos - Kd * v[:3],
            Kr * e_rot - Dr * v[3:],
        ))

        tau = J.T @ F_task

        # Null-space
        if self._nullspace_ref_rad is not None:
            try:
                # J is (6,7). Compute J^+ (7,6): J @ X = I_6 → X = (7,6)
                J_pinv = np.linalg.lstsq(J, np.eye(6), rcond=None)[0]  # (7, 6)
            except np.linalg.LinAlgError:
                J_pinv = np.zeros((7, 6))
            null_proj = np.eye(7) - J_pinv @ J  # (7,7) - (7,6)@(6,7) = (7,7)
            Kn = self._cart_imp_params.nullspace_stiffness
            Dn = self._cart_imp_params.nullspace_damping
            tau += null_proj @ (-Kn * (q - self._nullspace_ref_rad) - Dn * qd)

        return tau + bias


# ---------------------------------------------------------------------------
# Rotation helpers (module-level to avoid circular import)
# ---------------------------------------------------------------------------

def _matrix_to_quat(matrix: np.ndarray) -> np.ndarray:
    m = np.asarray(matrix, dtype=float).reshape(3, 3)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
        return np.array((0.25 * s, (m[2, 1] - m[1, 2]) / s,
                         (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s))
    idx = int(np.argmax(np.diag(m)))
    if idx == 0:
        s = 2.0 * math.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2]))
        return np.array(((m[2, 1] - m[1, 2]) / s, 0.25 * s,
                         (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s))
    elif idx == 1:
        s = 2.0 * math.sqrt(max(0.0, 1.0 + m[1, 1] - m[0, 0] - m[2, 2]))
        return np.array(((m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s,
                         0.25 * s, (m[1, 2] + m[2, 1]) / s))
    else:
        s = 2.0 * math.sqrt(max(0.0, 1.0 + m[2, 2] - m[0, 0] - m[1, 1]))
        return np.array(((m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s,
                         (m[1, 2] + m[2, 1]) / s, 0.25 * s))


def _quat_mul(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return np.array((
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    ))


def _rotation_error(current: np.ndarray, desired: np.ndarray) -> np.ndarray:
    current = np.asarray(current, dtype=float).reshape(3, 3)
    desired = np.asarray(desired, dtype=float).reshape(3, 3)
    qc = _matrix_to_quat(current)
    qd = _matrix_to_quat(desired)
    qerr = _quat_mul(qd, np.array((qc[0], -qc[1], -qc[2], -qc[3])))
    if qerr[0] < 0.0:
        qerr = -qerr
    vnorm = float(np.linalg.norm(qerr[1:]))
    if vnorm < 1e-12:
        return np.zeros(3)
    angle = 2.0 * math.atan2(vnorm, float(np.clip(qerr[0], -1.0, 1.0)))
    return qerr[1:] * (angle / vnorm)


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
