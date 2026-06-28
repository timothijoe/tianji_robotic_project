"""Forward/inverse kinematics and Jacobian for Marvin CCS 7-DOF arm.

Forward kinematics uses MuJoCo for correctness — the MJCF kinematic chain
(body stacking with pos+quat) is the ground truth.  Inverse kinematics is
a pure-Python damped least-squares solver.  The analytical Jacobian uses
MuJoCo's ``mj_jacSite`` when available, falling back to finite differences.

Internal calculations always use SI units (radians, metres, N·m).
The ``unit_mode`` parameter controls input/output conversion at the public API
boundary only.

``unit_mode="si"`` (default): radians, metres, N·m.
``unit_mode="sdk"``: degrees, millimetres, N·m (SDK-compatible display).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np

# Conversion constants
_RAD_PER_DEG = math.pi / 180.0
_DEG_PER_RAD = 180.0 / math.pi
_MM_PER_M = 1000.0
_M_PER_MM = 0.001


# ---------------------------------------------------------------------------
# Rotation utilities
# ---------------------------------------------------------------------------

def _rotation_error(current: np.ndarray, desired: np.ndarray) -> np.ndarray:
    """Axis-angle rotation error (3-vector) from current to desired rotation."""
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


def _quat_mul(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return np.array((
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    ))


def _matrix_to_quat(matrix: np.ndarray) -> np.ndarray:
    m = np.asarray(matrix, dtype=float).reshape(3, 3)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
        return np.array((
            0.25 * s, (m[2, 1] - m[1, 2]) / s,
            (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s,
        ))
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


def matrix_to_xyzabc(matrix: np.ndarray) -> np.ndarray:
    """4×4 homogeneous transform → XYZABC (m, rad).  ZYX fixed-angle convention."""
    m = np.asarray(matrix, dtype=float).reshape(4, 4)
    pos = m[:3, 3].copy()
    r = m[:3, :3]
    sy = math.sqrt(r[0, 0] ** 2 + r[1, 0] ** 2)
    if sy > 1e-12:
        a = math.atan2(r[2, 1], r[2, 2])
        b = math.atan2(-r[2, 0], sy)
        c = math.atan2(r[1, 0], r[0, 0])
    else:
        a = math.atan2(-r[1, 2], r[1, 1])
        b = math.atan2(-r[2, 0], sy)
        c = 0.0
    return np.array((pos[0], pos[1], pos[2], a, b, c))


def xyzabc_to_matrix(xyzabc: Sequence[float]) -> np.ndarray:
    """XYZABC (m, rad) → 4×4 homogeneous transform (ZYX fixed-angle)."""
    x, y, z, a, b, c = (float(v) for v in xyzabc)
    ca, sa = math.cos(a), math.sin(a)
    cb, sb = math.cos(b), math.sin(b)
    cc, sc = math.cos(c), math.sin(c)
    return np.array((
        (cc * cb, cc * sb * sa - sc * ca, cc * sb * ca + sc * sa, x),
        (sc * cb, sc * sb * sa + cc * ca, sc * sb * ca - cc * sa, y),
        (-sb,     cb * sa,                cb * ca,                z),
        (0,       0,                      0,                      1),
    ))


# ---------------------------------------------------------------------------
# IK result
# ---------------------------------------------------------------------------

@dataclass
class IkResult:
    joints_rad: np.ndarray       # 7 joint angles (radians)
    success: bool
    iterations: int
    residual: float
    is_singular: bool


# ---------------------------------------------------------------------------
# MarvinKinematics
# ---------------------------------------------------------------------------

class MarvinKinematics:
    """Kinematics for one Marvin CCS arm (7-DOF), backed by MuJoCo FK.

    Parameters
    ----------
    arm : str
        ``"left"`` or ``"right"``.
    unit_mode : str
        ``"si"`` (default): radians, metres.
        ``"sdk"``: degrees, millimetres.
    """

    def __init__(self, arm: str, unit_mode: str = "si", tcp_site_name: str | None = None) -> None:
        if arm not in ("left", "right"):
            raise ValueError(f"arm must be 'left' or 'right', got '{arm}'")
        if unit_mode not in ("si", "sdk"):
            raise ValueError(f"unit_mode must be 'si' or 'sdk', got '{unit_mode}'")
        self.arm = arm
        self.unit_mode = unit_mode
        self._tcp_site_name = tcp_site_name or f"{self.arm}_force_sensor_site"
        self._tool_matrix: np.ndarray | None = None
        # MuJoCo objects (set by TwinRobot.connect)
        self._runtime: object | None = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def joint_limits_rad(self) -> np.ndarray:
        """7×2 array of (lower, upper) joint limits in radians."""
        return np.array((
            (-3.11, 3.11),
            (-2.093, 2.093),
            (-3.11, 3.11),
            (-2.531, 1.047),
            (-3.11, 3.11),
            (-1.0467, 1.0467),
            (-1.57, 1.57),
        ), dtype=float)

    @property
    def tcp_site_name(self) -> str:
        return self._tcp_site_name

    def set_runtime(self, runtime: object) -> None:
        """Attach a MuJoCo runtime for FK/Jacobian access."""
        self._runtime = runtime

    def set_tool(self, tool_xyzabc: Sequence[float]) -> None:
        xyzabc = _to_si_xyzabc(np.asarray(tool_xyzabc, dtype=float).reshape(6), self.unit_mode)
        self._tool_matrix = xyzabc_to_matrix(xyzabc)

    def remove_tool(self) -> None:
        self._tool_matrix = None

    def fk(self, joints: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Forward kinematics via MuJoCo.

        Returns (4×4 matrix, xyzabc) in units per ``unit_mode``.
        """
        q = _to_si_radians(np.asarray(joints, dtype=float).reshape(7), self.unit_mode)
        matrix = self._fk_matrix(q)
        xyzabc_si = matrix_to_xyzabc(matrix)
        xyzabc = _from_si_xyzabc(xyzabc_si, self.unit_mode)
        return matrix, xyzabc

    def ik(
        self,
        target_matrix: np.ndarray,
        ref_joints: np.ndarray,
        zsp_type: int = 0,
        zsp_para: np.ndarray | None = None,
        zsp_angle_deg: float = 0.0,
    ) -> IkResult:
        """Inverse kinematics via damped least squares.

        Uses MuJoCo FK + Jacobian at each iteration for correctness.
        """
        T_target = np.asarray(target_matrix, dtype=float).reshape(4, 4)
        q_ref = _to_si_radians(np.asarray(ref_joints, dtype=float).reshape(7), self.unit_mode)
        limits = self.joint_limits_rad

        q = q_ref.copy()
        lam = 0.1
        alpha_null = 0.01
        prev_residual = float("inf")
        is_singular = False

        for iteration in range(200):
            T_cur = self._fk_matrix(q)
            e_pos = T_target[:3, 3] - T_cur[:3, 3]
            e_rot = _rotation_error(T_cur[:3, :3], T_target[:3, :3])
            error = np.concatenate((e_pos, e_rot))
            residual = float(np.linalg.norm(error))

            if residual < 1e-6:
                break

            J = self._jacobian(q)

            JJt = J @ J.T
            damped = JJt + lam * lam * np.eye(6)
            try:
                dq = J.T @ np.linalg.solve(damped, error)
            except np.linalg.LinAlgError:
                is_singular = True
                dq = J.T @ np.linalg.lstsq(damped, error, rcond=None)[0]

            if zsp_type == 0:
                J_pinv = np.linalg.lstsq(J, np.eye(6), rcond=None)[0]  # (7,6)
                null_proj = np.eye(7) - J_pinv @ J
                dq += null_proj @ ((q_ref - q) * alpha_null)

            q = np.clip(q + dq, limits[:, 0], limits[:, 1])

            if residual < prev_residual:
                lam = max(lam * 0.7, 0.001)
            else:
                lam = min(lam * 2.0, 10.0)
            prev_residual = residual

        return IkResult(
            joints_rad=q,
            success=residual < 1e-4,
            iterations=iteration + 1,
            residual=residual,
            is_singular=is_singular,
        )

    def jacobian(self, joints: np.ndarray) -> np.ndarray:
        """6×7 geometric Jacobian via MuJoCo ``mj_jacSite``."""
        q = _to_si_radians(np.asarray(joints, dtype=float).reshape(7), self.unit_mode)
        return self._jacobian(q)

    # ------------------------------------------------------------------
    # Internal (SI only)
    # ------------------------------------------------------------------

    def _fk_matrix(self, q_rad: np.ndarray) -> np.ndarray:
        """Compute FK by setting joint positions in MuJoCo and reading the site pose."""
        if self._runtime is None:
            raise RuntimeError(
                "MuJoCo runtime not attached — call set_runtime() or use TwinRobot.connect()"
            )
        import mujoco as _mj
        rt = self._runtime
        arm_view = rt.arm_view(self.arm)
        # Save current state
        saved_qpos = rt.data.qpos.copy()
        saved_qvel = rt.data.qvel.copy()
        try:
            rt.set_arm_positions(self.arm, q_rad)
            site_id = _mj.mj_name2id(rt.model, _mj.mjtObj.mjOBJ_SITE, self.tcp_site_name)
            pos = rt.data.site_xpos[site_id].copy()
            rot = rt.data.site_xmat[site_id].reshape(3, 3).copy()
        finally:
            rt.data.qpos[:] = saved_qpos
            rt.data.qvel[:] = saved_qvel
            _mj.mj_forward(rt.model, rt.data)

        T = np.eye(4)
        T[:3, :3] = rot
        T[:3, 3] = pos
        if self._tool_matrix is not None:
            T = T @ self._tool_matrix
        return T

    def _jacobian(self, q_rad: np.ndarray) -> np.ndarray:
        """Jacobian via MuJoCo ``mj_jacSite``."""
        if self._runtime is None:
            raise RuntimeError("MuJoCo runtime not attached")
        import mujoco as _mj
        rt = self._runtime
        arm_view = rt.arm_view(self.arm)
        saved_qpos = rt.data.qpos.copy()
        saved_qvel = rt.data.qvel.copy()
        try:
            rt.set_arm_positions(self.arm, q_rad)
            J = arm_view.site_jacobian(self.tcp_site_name)
        finally:
            rt.data.qpos[:] = saved_qpos
            rt.data.qvel[:] = saved_qvel
            _mj.mj_forward(rt.model, rt.data)
        return J.copy()


# ---------------------------------------------------------------------------
# Unit conversion helpers
# ---------------------------------------------------------------------------

def _to_si_radians(joints: np.ndarray, unit_mode: str) -> np.ndarray:
    if unit_mode == "sdk":
        return joints * _RAD_PER_DEG
    return joints.copy()


def _from_si_radians(joints_si: np.ndarray, unit_mode: str) -> np.ndarray:
    if unit_mode == "sdk":
        return joints_si * _DEG_PER_RAD
    return joints_si.copy()


def _to_si_xyzabc(xyzabc: np.ndarray, unit_mode: str) -> np.ndarray:
    result = xyzabc.copy()
    if unit_mode == "sdk":
        result[:3] *= _M_PER_MM
        result[3:] *= _RAD_PER_DEG
    return result


def _from_si_xyzabc(xyzabc_si: np.ndarray, unit_mode: str) -> np.ndarray:
    result = xyzabc_si.copy()
    if unit_mode == "sdk":
        result[:3] *= _MM_PER_M
        result[3:] *= _DEG_PER_RAD
    return result
