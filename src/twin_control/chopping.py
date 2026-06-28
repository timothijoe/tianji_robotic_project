"""SDK-based chopping state machine for Marvin CCS right arm.

Uses ``TwinRobot`` (SDK-style interface) as the control backend, replacing
the original ``CartesianForceController``-based implementation.  State
machine logic and blade geometry calculations are preserved from the
reference implementation.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Sequence

import mujoco
import mujoco.viewer  # noqa: F401 — loads lazy submodule
import numpy as np

from twin_control.robot import (
    LEFT_HOME_RAD,
    RIGHT_HOME_RAD,
    TwinRobot,
    _DEFAULT_CART_D,
    _DEFAULT_CART_K,
)
from twin_description.paths import right_chopping_scene_path


# ---------------------------------------------------------------------------
# Phase / config / geometry types
# ---------------------------------------------------------------------------

class ChoppingPhase(str, Enum):
    APPROACH = "APPROACH"
    DESCEND = "DESCEND"
    FORCE_HOLD = "FORCE_HOLD"
    RETRACT = "RETRACT"
    SHIFT = "SHIFT"
    COMPLETE = "COMPLETE"
    FAULT = "FAULT"


@dataclass(frozen=True)
class ChoppingConfig:
    cycles: int = 3
    safe_height_m: float = 0.08
    spacing_m: float = 0.02
    descent_speed_m_s: float = 0.05
    retract_speed_m_s: float = 0.08
    force_hold_s: float = 0.15
    target_force_n: float = 10.0
    control_hz: float = 500.0


@dataclass(frozen=True)
class BladeGeometry:
    edge_positions: tuple[tuple[float, float, float], ...]
    tip_position: tuple[float, float, float]
    tip_rotation: tuple[tuple[float, float, float], ...]

    @property
    def min_z(self) -> float:
        return min(float(p[2]) for p in self.edge_positions)

    @property
    def max_z(self) -> float:
        return max(float(p[2]) for p in self.edge_positions)


@dataclass(frozen=True)
class ChoppingSample:
    time_s: float
    phase: ChoppingPhase
    control_mode: str
    target_position: tuple[float, float, float]
    actual_position: tuple[float, float, float]
    target_force_n: float
    measured_force_n: float
    raw_wrench: tuple[float, ...]
    joint_positions: tuple[float, ...]
    joint_velocities: tuple[float, ...]
    joint_torques: tuple[float, ...]
    contact: bool
    fault: str = ""


# ---------------------------------------------------------------------------
# Blade geometry constants
# ---------------------------------------------------------------------------

BLADE_EDGE_SITE_NAMES = ("right_blade_edge_top", "right_blade_edge_bot")
BLADE_REFERENCE_SITE_NAMES = (
    "right_blade_edge_top",
    "right_blade_edge_bot",
    "right_tool_tip_site",
)
WORLD_DOWN_AXIS = np.array((0.0, 0.0, -1.0), dtype=float)

_CSV_FIELDS = (
    "time_s", "phase", "control_mode",
    "target_x", "target_y", "target_z",
    "actual_x", "actual_y", "actual_z",
    "target_force_n", "measured_force_n",
    *(f"raw_wrench_{i}" for i in range(6)),
    *(f"q_{i}" for i in range(7)),
    *(f"qd_{i}" for i in range(7)),
    *(f"tau_{i}" for i in range(7)),
    "contact", "fault",
)


# ---------------------------------------------------------------------------
# TwinRobotChopper
# ---------------------------------------------------------------------------

class TwinRobotChopper:
    """SDK-style chopping state machine using ``TwinRobot`` for control.

    Parameters
    ----------
    robot : TwinRobot or None
        Pre-configured robot instance.  Created internally if None.
    """

    def __init__(self, robot: TwinRobot | None = None) -> None:
        self.robot = robot or TwinRobot(
            arm_name="right", unit_mode="si", tcp_site_name="right_tool_tip_site",
        )
        self.samples: list[ChoppingSample] = []

    # ------------------------------------------------------------------
    # Main entry point
    # ------------------------------------------------------------------

    def run(
        self,
        config: ChoppingConfig | None = None,
        log_path: str | Path | None = None,
        headless: bool = False,
    ) -> list[ChoppingSample]:
        cfg = config or ChoppingConfig()
        _validate_config(cfg)

        # Connect + reset
        self.robot.control_hz = float(cfg.control_hz)
        self.robot.connect(viewer=not headless, realtime=not headless)
        self.robot.runtime.reset()
        self.robot.runtime.set_arm_positions("left", LEFT_HOME_RAD)
        self.robot.runtime.set_arm_positions("right", RIGHT_HOME_RAD)
        self.samples.clear()

        # Compute blade geometry & Cartesian target parameters
        geometry = self._blade_geometry()
        sensor_pos, _ = self.robot.get_tcp_pose()
        tip_pos = np.array((float(geometry.tip_position[0]), float(geometry.tip_position[1]),
                            float(geometry.tip_position[2])))
        rotation = self._horizontal_blade_rotation(geometry)
        board_top = self._board_top()
        safe_tip = self._tip_target_for_blade_clearance(tip_pos, rotation, geometry, board_top, cfg.safe_height_m)
        descend_tip = self._tip_target_for_blade_on_board(tip_pos, rotation, geometry, board_top)
        control_dt = 1.0 / cfg.control_hz

        # Cartesian impedance parameters (same stiffness used throughout)
        K = _DEFAULT_CART_K
        D = _DEFAULT_CART_D

        # Force control along world down, matching the reference chopper.
        fx_dir = (0.0, 0.0, -1.0, 0.0, 0.0, 0.0)

        for cycle in range(cfg.cycles):
            # Shift target along X for each cycle
            shifted_safe = safe_tip.copy()
            shifted_safe[0] += cycle * cfg.spacing_m
            shifted_descend = descend_tip.copy()
            shifted_descend[0] += cycle * cfg.spacing_m

            phase = ChoppingPhase.APPROACH if cycle == 0 else ChoppingPhase.SHIFT

            # --- Approach / Shift ---
            self.robot.set_cart_impedance_state(0.5, 0.5, K, D)
            self._move_tip_to(shifted_safe, rotation, phase, 0.0, control_dt, cfg, 8)

            # --- Descend ---
            self.robot.set_cart_impedance_state(0.5, 0.5, K, D)
            descend_steps = _motion_steps(shifted_safe, shifted_descend, cfg.descent_speed_m_s, control_dt)
            self._move_tip_to(shifted_descend, rotation, ChoppingPhase.DESCEND, 0.0, control_dt, cfg,
                              max(1, descend_steps))

            # --- Force hold ---
            self.robot.set_force_state(0.5, 0.5, K, D, fx_dir, fc_adj_lmt=0.04)
            self.robot.set_force_cmd(cfg.target_force_n)
            T = np.eye(4)
            T[:3, :3] = rotation
            T[:3, 3] = shifted_descend
            self.robot._controller.set_cart_cmd(T)
            hold_steps = max(2, int(np.ceil(cfg.force_hold_s / control_dt)))
            for _ in range(hold_steps):
                self.robot.step(viewer_sync=True)
                wrench = self.robot.get_wrench()
                self._record_sample(ChoppingPhase.FORCE_HOLD, "FORCE", shifted_descend,
                                    cfg.target_force_n, wrench)
                if not headless and self.robot._viewer is not None:
                    self.robot._viewer.sync()

            # --- Retract ---
            self.robot.set_cart_impedance_state(0.5, 0.5, K, D)
            retract_steps = _motion_steps(shifted_descend, shifted_safe, cfg.retract_speed_m_s, control_dt)
            self._move_tip_to(shifted_safe, rotation, ChoppingPhase.RETRACT, 0.0, control_dt, cfg,
                              max(1, retract_steps))

        # Final sample
        self.samples.append(self._make_sample(
            self.robot.runtime.data.time, ChoppingPhase.COMPLETE, "IDLE",
            safe_tip, 0.0, np.zeros(6),
        ))

        if not headless and self.robot._viewer is not None:
            self.robot._viewer.sync()

        if log_path is not None:
            self.write_csv(log_path)

        self.robot.close()
        return list(self.samples)

    # ------------------------------------------------------------------
    # CSV
    # ------------------------------------------------------------------

    def write_csv(self, log_path: str | Path) -> None:
        path = Path(log_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=_CSV_FIELDS)
            writer.writeheader()
            for s in self.samples:
                writer.writerow(_sample_row(s))

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _move_tip_to(
        self,
        target_pos: np.ndarray,
        rotation: np.ndarray,
        phase: ChoppingPhase,
        target_force_n: float,
        control_dt: float,
        cfg: ChoppingConfig,
        steps: int,
    ) -> None:
        """Interpolate tip from current position to *target_pos* over *steps*."""
        current_pos, _ = self.robot.get_tcp_pose()
        current_pos = np.asarray(current_pos, dtype=float).reshape(3)
        T = np.eye(4)
        T[:3, :3] = rotation
        for idx in range(steps):
            alpha = (idx + 1) / steps
            interp_pos = current_pos + (target_pos - current_pos) * alpha
            T[:3, 3] = interp_pos
            self.robot._controller.set_cart_cmd(T)
            self.robot.step(viewer_sync=True)
            wrench = self.robot.get_wrench()
            self._record_sample(phase, "CARTESIAN_IMPEDANCE", interp_pos, target_force_n, wrench)

    def _record_sample(
        self,
        phase: ChoppingPhase,
        mode: str,
        target_pos: np.ndarray,
        target_force_n: float,
        wrench: np.ndarray,
    ) -> None:
        self.samples.append(self._make_sample(
            self.robot.runtime.data.time, phase, mode,
            np.asarray(target_pos, dtype=float).reshape(3), target_force_n,
            np.asarray(wrench, dtype=float).reshape(6),
        ))

    def _make_sample(
        self,
        t: float, phase: ChoppingPhase, mode: str,
        target_pos: np.ndarray, target_force_n: float,
        wrench: np.ndarray, fault: str = "",
    ) -> ChoppingSample:
        q = self.robot.get_joint_positions()
        qd = self.robot.get_joint_velocities()
        actual_pos, _ = self.robot.get_tcp_pose()
        raw = self.robot.get_raw_wrench()
        return ChoppingSample(
            time_s=float(t),
            phase=phase,
            control_mode=mode,
            target_position=_triple(target_pos),
            actual_position=_triple(actual_pos),
            target_force_n=float(target_force_n),
            measured_force_n=float(wrench[2]),
            raw_wrench=tuple(float(v) for v in raw),
            joint_positions=tuple(float(v) for v in q),
            joint_velocities=tuple(float(v) for v in qd),
            joint_torques=tuple(float(v) for v in self.robot._last_torque),
            contact=bool(abs(wrench[2]) > 1e-6),
            fault=fault,
        )

    # ------------------------------------------------------------------
    # Blade geometry (mirrored from reference implementation)
    # ------------------------------------------------------------------

    def _blade_geometry(self) -> BladeGeometry:
        edge = []
        for name in BLADE_EDGE_SITE_NAMES:
            pos, _ = self.robot.runtime.site_pose(name)
            edge.append(_triple(pos))
        tip_pos, tip_rot = self.robot.runtime.site_pose("right_tool_tip_site")
        rot_rows = tuple(tuple(float(v) for v in row) for row in tip_rot)
        return BladeGeometry(tuple(edge), _triple(tip_pos), rot_rows)

    def _horizontal_blade_rotation(self, geometry: BladeGeometry) -> np.ndarray:
        current_rot = np.asarray(geometry.tip_rotation, dtype=float).reshape(3, 3)
        target_z = current_rot[:, 2].copy()
        target_z[2] = 0.0
        norm = float(np.linalg.norm(target_z))
        if norm < 1e-9:
            target_z = np.array((1.0, 0.0, 0.0))
        else:
            target_z /= norm
        target_x = current_rot[:, 0] - target_z * float(np.dot(current_rot[:, 0], target_z))
        x_norm = float(np.linalg.norm(target_x))
        if x_norm < 1e-9:
            target_x = np.cross((0.0, 0.0, 1.0), target_z)
            x_norm = float(np.linalg.norm(target_x))
        target_x /= x_norm
        target_y = np.cross(target_z, target_x)
        target_y /= float(np.linalg.norm(target_y))
        return np.column_stack((target_x, target_y, target_z))

    def _predict_positions(
        self,
        positions: Sequence[Sequence[float]],
        tip_target: Sequence[float],
        target_rotation: np.ndarray,
        geometry: BladeGeometry,
    ) -> tuple[tuple[float, float, float], ...]:
        current_tip = np.array(geometry.tip_position, dtype=float).reshape(3)
        current_rot = np.array(geometry.tip_rotation, dtype=float).reshape(3, 3)
        target = np.asarray(tip_target, dtype=float).reshape(3)
        rotation = np.asarray(target_rotation, dtype=float).reshape(3, 3)
        pred = []
        for pos in positions:
            pt = np.asarray(pos, dtype=float).reshape(3)
            local = current_rot.T @ (pt - current_tip)
            pred.append(_triple(target + rotation @ local))
        return tuple(pred)

    def _tip_target_for_blade_clearance(
        self, tip: np.ndarray, rotation: np.ndarray,
        geometry: BladeGeometry, board_top: float, clearance_m: float,
    ) -> np.ndarray:
        target = tip.copy()
        pred = self._predict_positions(
            (*geometry.edge_positions, geometry.tip_position),
            target, rotation, geometry,
        )
        min_z = min(float(p[2]) for p in pred)
        target[2] += float(board_top) + float(clearance_m) - min_z
        return target

    def _tip_target_for_blade_on_board(
        self, tip: np.ndarray, rotation: np.ndarray,
        geometry: BladeGeometry, board_top: float,
    ) -> np.ndarray:
        target = tip.copy()
        pred = self._predict_positions(
            (*geometry.edge_positions, geometry.tip_position),
            target, rotation, geometry,
        )
        max_z = max(float(p[2]) for p in pred)
        target[2] += float(board_top) - max_z
        return target

    def _board_top(self) -> float:
        geom_id = int(mujoco.mj_name2id(
            self.robot.runtime.model, mujoco.mjtObj.mjOBJ_GEOM, "chopping_board",
        ))
        return float(
            self.robot.runtime.data.geom_xpos[geom_id, 2]
            + self.robot.runtime.model.geom_size[geom_id, 2],
        )


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _validate_config(cfg: ChoppingConfig) -> None:
    if cfg.cycles < 1:
        raise ValueError("cycles must be at least 1")
    if cfg.control_hz <= 0.0:
        raise ValueError("control_hz must be positive")
    if cfg.descent_speed_m_s <= 0.0:
        raise ValueError("descent_speed_m_s must be positive")
    if cfg.retract_speed_m_s <= 0.0:
        raise ValueError("retract_speed_m_s must be positive")


def _motion_steps(start: np.ndarray, end: np.ndarray, speed_m_s: float, dt: float) -> int:
    d = float(np.linalg.norm(np.asarray(end).reshape(3) - np.asarray(start).reshape(3)))
    return max(1, int(np.ceil(d / speed_m_s / dt)))


def _triple(v: np.ndarray | Sequence[float]) -> tuple[float, float, float]:
    a = np.asarray(v, dtype=float).reshape(3)
    return tuple(float(x) for x in a)


def _sample_row(s: ChoppingSample) -> dict[str, object]:
    row: dict[str, object] = {
        "time_s": f"{s.time_s:.6f}",
        "phase": s.phase.value,
        "control_mode": s.control_mode,
        "target_x": s.target_position[0], "target_y": s.target_position[1], "target_z": s.target_position[2],
        "actual_x": s.actual_position[0], "actual_y": s.actual_position[1], "actual_z": s.actual_position[2],
        "target_force_n": s.target_force_n,
        "measured_force_n": s.measured_force_n,
        "contact": int(s.contact),
        "fault": s.fault,
    }
    for i, v in enumerate(s.raw_wrench):
        row[f"raw_wrench_{i}"] = v
    for i, v in enumerate(s.joint_positions):
        row[f"q_{i}"] = v
        row[f"qd_{i}"] = s.joint_velocities[i] if i < len(s.joint_velocities) else 0.0
        row[f"tau_{i}"] = s.joint_torques[i] if i < len(s.joint_torques) else 0.0
    return row
