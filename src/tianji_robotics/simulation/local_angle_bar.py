"""Local MuJoCo backend for interactive Wuji hand angle-bar control."""

from __future__ import annotations

import time
import tkinter as tk
from collections.abc import Callable

import mujoco
import numpy as np

from tianji_robotics.simulation.paths import official_wuji_hand_mjcf


_SIDES = frozenset({"left", "right"})
_FINGER_LABELS = ("Thumb", "Index", "Middle", "Ring", "Little")
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

    @property
    def open_target_rad(self) -> np.ndarray:
        """Return the validated, side-specific open hand target in radians."""
        self._require_open()
        return self._open_target_rad.copy()

    @property
    def viewer_is_running(self) -> bool:
        """Whether this backend owns a passive Viewer that remains open."""
        return self._viewer is not None and self._viewer.is_running()

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


class WujiAngleBarPanel:
    """Tk controls that command one locally simulated Wuji hand at a time."""

    def __init__(
        self,
        root: tk.Tk,
        backend_factory: Callable[..., LocalWujiHand] = LocalWujiHand,
        initial_side: str = "left",
    ) -> None:
        self._root = root
        self._backend_factory = backend_factory
        self._backend = backend_factory(initial_side, viewer=True)
        self._side = tk.StringVar(value=initial_side)
        self._targets = [tk.DoubleVar(value=value) for value in self._backend.target_rad]
        self._slider_groups: tk.Frame | None = None
        self._value_labels: list[tk.Label] = []
        self._build_controls()
        self._root.protocol("WM_DELETE_WINDOW", self.close)
        self._root.after(0, self._tick)

    def _build_controls(self) -> None:
        self._root.title("Wuji hand local angle control")
        controls = tk.Frame(self._root, padx=10, pady=10)
        controls.pack(fill=tk.X)
        tk.Label(controls, text="Hand side:").pack(side=tk.LEFT)
        for side in ("left", "right"):
            tk.Radiobutton(
                controls,
                text=side.capitalize(),
                variable=self._side,
                value=side,
                command=lambda selected=side: self.switch_side(selected),
            ).pack(side=tk.LEFT)
        tk.Button(controls, text="Reset open hand", command=self.reset_open).pack(side=tk.RIGHT)
        self._slider_host = tk.Frame(self._root, padx=10, pady=10)
        self._slider_host.pack(fill=tk.BOTH, expand=True)
        self._rebuild_slider_groups()

    def _rebuild_slider_groups(self) -> None:
        if self._slider_groups is not None:
            self._slider_groups.destroy()
        self._slider_groups = tk.Frame(self._slider_host)
        self._slider_groups.pack(fill=tk.BOTH, expand=True)
        self._value_labels = []
        for finger_index, finger_name in enumerate(_FINGER_LABELS):
            group = tk.LabelFrame(self._slider_groups, text=finger_name, padx=6, pady=4)
            group.pack(fill=tk.X, pady=3)
            for joint_offset in range(4):
                index = finger_index * 4 + joint_offset
                self._build_slider_row(group, index)

    def _build_slider_row(self, parent: tk.Misc, index: int) -> None:
        lower, upper = self._backend.control_ranges_rad[index]
        tk.Label(parent, text=self._backend.joint_names[index], width=24, anchor=tk.W).grid(
            row=index % 4, column=0, sticky=tk.W
        )
        scale = tk.Scale(
            parent,
            from_=float(lower),
            to=float(upper),
            resolution=0.001,
            orient=tk.HORIZONTAL,
            variable=self._targets[index],
            command=lambda _value, selected=index: self._apply_slider(selected),
            length=300,
        )
        scale.grid(row=index % 4, column=1, sticky=tk.EW)
        value_label = tk.Label(parent, text=self._format_radians(self._targets[index].get()), width=10)
        value_label.grid(row=index % 4, column=2, sticky=tk.E)
        self._value_labels.append(value_label)
        parent.grid_columnconfigure(1, weight=1)

    def _apply_slider(self, index: int) -> None:
        target = self._backend.target_rad
        target[index] = self._targets[index].get()
        self._backend.command(target)
        self._value_labels[index].configure(text=self._format_radians(target[index]))

    def reset_open(self) -> None:
        self._backend.reset_open()
        for index, value in enumerate(self._backend.target_rad):
            self._targets[index].set(value)
            self._value_labels[index].configure(text=self._format_radians(value))

    def switch_side(self, side: str) -> None:
        old_backend = self._backend
        self._backend = self._backend_factory(side, viewer=True)
        old_backend.close()
        self._side.set(side)
        self._targets = [tk.DoubleVar(value=value) for value in self._backend.target_rad]
        self._rebuild_slider_groups()

    def _tick(self) -> None:
        try:
            if not self._root.winfo_exists():
                return
            self._backend.step()
            if self._backend.viewer_is_running:
                self._root.after(10, self._tick)
        except tk.TclError:
            return

    def close(self) -> None:
        self._backend.close()
        self._root.destroy()

    @staticmethod
    def _format_radians(value: float) -> str:
        return f"{value:.3f} rad"


def run_local_angle_bar(side: str) -> int:
    """Run a local MuJoCo hand angle-bar panel for the requested side."""
    _validate_side(side)
    root = tk.Tk()
    WujiAngleBarPanel(root, initial_side=side)
    root.mainloop()
    return 0
