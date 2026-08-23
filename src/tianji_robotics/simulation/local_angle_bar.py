"""Local MuJoCo backend for interactive Wuji hand angle-bar control."""

from __future__ import annotations

import time

import mujoco
import numpy as np

from tianji_robotics.simulation.paths import official_wuji_hand_mjcf


_SIDES = frozenset({"left", "right"})
OPEN_TARGET_RAD = {
    # Explicit per-side poses, kept separate so a future handed model can
    # change one pose without silently changing the other.
    "left": np.asarray((0.15, 0.0, 0.05, 0.05) + (0.05, 0.0, 0.05, 0.05) * 4, dtype=float),
    "right": np.asarray((0.15, 0.0, 0.05, 0.05) + (0.05, 0.0, 0.05, 0.05) * 4, dtype=float),
}


def _validate_side(side: str) -> str:
    if side not in _SIDES:
        raise ValueError("hand side must be 'left' or 'right'")
    return side


class LocalWujiHand:
    """Control one official 20-DOF Wuji hand entirely within MuJoCo."""

    def __init__(self, side: str, *, viewer: bool = False) -> None:
        self.side = _validate_side(side)
        self.model = mujoco.MjModel.from_xml_path(str(official_wuji_hand_mjcf(self.side)))
        self.data = mujoco.MjData(self.model)
        self._actuator_ids = np.arange(self.model.nu, dtype=np.int32)
        if self.model.nu != 20:
            raise RuntimeError("official Wuji hand must expose 20 actuators")
        self.joint_names = tuple(
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            for i in self._actuator_ids
        )
        if any(name is None for name in self.joint_names):
            raise RuntimeError("official Wuji hand actuators must be named")
        self.control_ranges_rad = self.model.actuator_ctrlrange[self._actuator_ids].copy()
        self._open_target_rad = OPEN_TARGET_RAD[self.side].copy()
        if (
            np.any(self._open_target_rad < self.control_ranges_rad[:, 0])
            or np.any(self._open_target_rad > self.control_ranges_rad[:, 1])
        ):
            raise RuntimeError(f"{self.side} open target is outside model control ranges")
        self._closed = False
        self._viewer = None
        if viewer:
            from mujoco import viewer as mujoco_viewer

            self._viewer = mujoco_viewer.launch_passive(self.model, self.data)
            self._viewer.sync()
        self.command(self._open_target_rad)

    @property
    def target_rad(self) -> np.ndarray:
        self._require_open()
        return self.data.ctrl[self._actuator_ids].copy()

    def command(self, target_rad: np.ndarray) -> None:
        self._require_open()
        target = np.asarray(target_rad, dtype=float)
        if target.shape != (20,) or not np.isfinite(target).all():
            raise ValueError("hand target must contain 20 finite radians")
        if np.any(target < self.control_ranges_rad[:, 0]) or np.any(
            target > self.control_ranges_rad[:, 1]
        ):
            raise ValueError("hand target is outside range")
        self.data.ctrl[self._actuator_ids] = target

    def reset_open(self) -> None:
        self.command(self._open_target_rad)

    def step(self) -> None:
        self._require_open()
        mujoco.mj_step(self.model, self.data)
        if self._viewer is not None and self._viewer.is_running():
            self._viewer.sync()
            time.sleep(float(self.model.opt.timestep))

    def close(self) -> None:
        if self._closed:
            return
        if self._viewer is not None:
            self._viewer.close()
        self._closed = True

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError("local Wuji hand backend is closed")
