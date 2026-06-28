"""TCP trail visualisation for the MuJoCo viewer.

Renders the end-effector path as small coloured spheres in the viewer's
``user_scn`` scene, with separate colours for actual TCP positions
(default orange) and commanded target positions (default cyan).

Public API
----------
TcpTrail(viewer, tcp_site_name)     — attach to a MuJoCo viewer.
    .set_actual_color(rgba)          — change actual-path marker colour.
    .set_target_color(rgba)          — change commanded-path marker colour.
    .record(q, kinematics, controller) — sample current actual + target TCP.
    .render()                        — inject new markers into viewer scene.
    .clear()                         — reset trail.
"""

from __future__ import annotations

from typing import Sequence

import mujoco
import numpy as np

from twin_control.controller import ControlMode, UnifiedController
from twin_control.kinematics import MarvinKinematics


class TcpTrail:
    """Record and render TCP trails in the MuJoCo viewer.

    Parameters
    ----------
    viewer : mujoco viewer handle or None
        The passive viewer.  If None, all methods become no-ops.
    tcp_site_name : str
        Site name used for FK lookups (informational; actual positions come
        from the caller).
    """

    def __init__(self, viewer, tcp_site_name: str = "") -> None:
        self._viewer = viewer
        self._tcp_site_name = tcp_site_name
        self._actual: list[np.ndarray] = []
        self._target: list[np.ndarray] = []
        self._actual_rendered = 0
        self._target_rendered = 0
        self._step_count = 0
        self._sample_stride = 20

        self.actual_color = (1.0, 0.45, 0.0, 0.95)   # orange
        self.actual_size = 0.008
        self.target_color = (0.0, 0.85, 1.0, 0.95)    # cyan
        self.target_size = 0.012

    # ------------------------------------------------------------------
    # Recording
    # ------------------------------------------------------------------

    def record(self, q_rad: np.ndarray, kinematics: MarvinKinematics,
               controller: UnifiedController) -> None:
        """Sample one actual and one (optional) commanded TCP position.

        Parameters
        ----------
        q_rad : (7,) ndarray — current joint positions (rad).
        kinematics : MarvinKinematics — for FK computation.
        controller : UnifiedController — for reading Cartesian and joint targets.
        """
        self._step_count += 1
        if self._step_count % self._sample_stride != 0:
            return

        # Actual TCP
        matrix, _ = kinematics.fk(q_rad)
        self._actual.append(matrix[:3, 3].copy())

        # Commanded TCP
        target = self._resolve_target(kinematics, controller)
        if target is not None:
            self._target.append(target)

    def _resolve_target(self, kinematics: MarvinKinematics,
                        controller: UnifiedController) -> np.ndarray | None:
        """Compute the commanded TCP position from the controller's internal state."""
        cart_target = getattr(controller, "_cart_target_matrix", None)
        mode = getattr(controller, "_mode", ControlMode.POSITION)

        if cart_target is not None and mode in (ControlMode.CARTESIAN_IMPEDANCE,
                                                 ControlMode.POSITION):
            return np.asarray(cart_target[:3, 3], dtype=float).copy()

        if cart_target is not None and mode == ControlMode.FORCE:
            offset = getattr(controller, "_force_offset_m", 0.0)
            axis = controller.force_axis_world(None)
            return np.asarray(cart_target[:3, 3] + offset * axis, dtype=float).copy()

        joint_target = getattr(controller, "_joint_target_rad", None)
        if joint_target is not None and mode == ControlMode.JOINT_IMPEDANCE:
            return kinematics.fk(joint_target)[0][:3, 3].copy()

        return None

    # ------------------------------------------------------------------
    # Rendering
    # ------------------------------------------------------------------

    def render(self) -> None:
        """Add new trail points to the viewer's ``user_scn``.

        Safe to call every frame; only new points since last render are added.
        If the viewer is not available, this is a no-op.
        """
        if self._viewer is None:
            return
        scn = self._viewer.user_scn
        if scn is None:
            return

        with self._viewer.lock():
            self._actual_rendered = _append_geoms(
                scn, self._actual, self._actual_rendered,
                self.actual_color, self.actual_size,
            )
            self._target_rendered = _append_geoms(
                scn, self._target, self._target_rendered,
                self.target_color, self.target_size,
            )

    # ------------------------------------------------------------------
    # Clear
    # ------------------------------------------------------------------

    def clear(self) -> None:
        """Reset all trail data."""
        self._actual.clear()
        self._target.clear()
        self._actual_rendered = 0
        self._target_rendered = 0
        self._step_count = 0
        if self._viewer is not None and self._viewer.user_scn is not None:
            with self._viewer.lock():
                self._viewer.user_scn.ngeom = 0


# ---------------------------------------------------------------------------
# Internal: inject sphere geoms into the viewer scene
# ---------------------------------------------------------------------------

def _append_geoms(
    scn,
    trail: list[np.ndarray],
    rendered: int,
    color: tuple[float, ...],
    size: float,
) -> int:
    """Append new geoms from *trail[rendered:]* to *scn*."""
    new_pts = trail[rendered:]
    if not new_pts:
        return rendered

    stride = max(1, len(new_pts) // 100) if len(new_pts) > 100 else 1
    rgba = np.array(color, dtype=float)
    for pos in new_pts[::stride]:
        if scn.ngeom >= scn.maxgeom:
            return rendered
        g = scn.geoms[scn.ngeom]
        _init_sphere_geom(g, pos, size, rgba)
        scn.ngeom += 1

    return len(trail)


def _init_sphere_geom(geom, pos: np.ndarray, size: float, rgba: np.ndarray) -> None:
    """Initialise a MuJoCo geom struct as a sphere marker.

    Uses ``mjv_initGeom`` when available (MuJoCo 3.10+), falling back to
    manual field assignment for 3.9.
    """
    sz = np.array((size, size, size), dtype=float)
    p = np.asarray(pos, dtype=float).reshape(3)
    mat = np.eye(3)
    try:
        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_SPHERE, sz, p, mat, rgba)
    except TypeError:
        geom.type = mujoco.mjtGeom.mjGEOM_SPHERE
        geom.size[:] = sz
        geom.pos[:] = p
        try:
            geom.mat[:] = mat.ravel()
        except ValueError:
            geom.mat.flat = mat.ravel()   # handle both (3,3) and flat arrays
        geom.rgba[:] = rgba
    geom.category = mujoco.mjtCatBit.mjCAT_DECOR
    # Clear optional fields
    for attr in ("segid", "objtype", "objid", "dataid"):
        if hasattr(geom, attr):
            setattr(geom, attr, -1)
    for attr in ("emission", "specular", "shininess", "reflectance"):
        if hasattr(geom, attr):
            setattr(geom, attr, 0.0)
    if hasattr(geom, "label"):
        try:
            geom.label = ""
        except Exception:
            geom.label[:] = b"\x00" * len(geom.label)
