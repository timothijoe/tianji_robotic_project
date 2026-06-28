"""Rotation utility functions shared by kinematics and controller modules.

These are pure-math, standalone helpers for SO(3) operations:
quaternion construction, quaternion multiplication, and rotation error
computation via the axis-angle representation.

All functions operate on numpy arrays and use SI conventions.
"""

from __future__ import annotations

import math

import numpy as np


def matrix_to_quat(matrix: np.ndarray) -> np.ndarray:
    """Convert a 3×3 rotation matrix to a unit quaternion (w, x, y, z).

    Parameters
    ----------
    matrix : (3, 3) ndarray
        Orthonormal rotation matrix.

    Returns
    -------
    (4,) ndarray — quaternion (w, x, y, z), guaranteed unit-norm.
    """
    m = np.asarray(matrix, dtype=float).reshape(3, 3)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
        return np.array((
            0.25 * s,
            (m[2, 1] - m[1, 2]) / s,
            (m[0, 2] - m[2, 0]) / s,
            (m[1, 0] - m[0, 1]) / s,
        ))
    idx = int(np.argmax(np.diag(m)))
    if idx == 0:
        s = 2.0 * math.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2]))
        return np.array((
            (m[2, 1] - m[1, 2]) / s, 0.25 * s,
            (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s,
        ))
    elif idx == 1:
        s = 2.0 * math.sqrt(max(0.0, 1.0 + m[1, 1] - m[0, 0] - m[2, 2]))
        return np.array((
            (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s,
            0.25 * s, (m[1, 2] + m[2, 1]) / s,
        ))
    else:
        s = 2.0 * math.sqrt(max(0.0, 1.0 + m[2, 2] - m[0, 0] - m[1, 1]))
        return np.array((
            (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s,
            (m[1, 2] + m[2, 1]) / s, 0.25 * s,
        ))


def quat_mul(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    """Multiply two quaternions (w, x, y, z convention).

    Parameters
    ----------
    left : (4,) ndarray — quaternion a.
    right : (4,) ndarray — quaternion b.

    Returns
    -------
    (4,) ndarray — a ⊗ b.
    """
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return np.array((
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    ))


def rotation_error(current: np.ndarray, desired: np.ndarray) -> np.ndarray:
    """Compute the axis-angle rotation error from *current* to *desired*.

    Parameters
    ----------
    current : (3, 3) ndarray — current rotation matrix.
    desired : (3, 3) ndarray — desired rotation matrix.

    Returns
    -------
    (3,) ndarray — axis-angle vector whose magnitude is the angular error
    in radians and whose direction is the rotation axis.
    """
    current = np.asarray(current, dtype=float).reshape(3, 3)
    desired = np.asarray(desired, dtype=float).reshape(3, 3)
    qc = matrix_to_quat(current)
    qd = matrix_to_quat(desired)
    # Quaternion error: q_err = q_des * conj(q_cur)
    qerr = quat_mul(qd, np.array((qc[0], -qc[1], -qc[2], -qc[3])))
    if qerr[0] < 0.0:
        qerr = -qerr
    vnorm = float(np.linalg.norm(qerr[1:]))
    if vnorm < 1e-12:
        return np.zeros(3)
    angle = 2.0 * math.atan2(vnorm, float(np.clip(qerr[0], -1.0, 1.0)))
    return qerr[1:] * (angle / vnorm)
