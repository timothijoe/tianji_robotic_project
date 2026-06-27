from __future__ import annotations

import csv
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Callable, Sequence

import mujoco
import numpy as np

from twin_mujoco.control import CartesianForceController
from twin_mujoco.runtime import TwinMujocoRuntime


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


LEFT_HOME_Q = np.zeros(7, dtype=float)
RIGHT_CHOPPING_HOME_Q = np.array((0.164118, -1.229373, 0.052513, -1.606525, 0.057176, 0.79256, 0.142978), dtype=float)


_CSV_FIELDS = (
    "time_s",
    "phase",
    "control_mode",
    "target_x",
    "target_y",
    "target_z",
    "actual_x",
    "actual_y",
    "actual_z",
    "target_force_n",
    "measured_force_n",
    *(f"raw_wrench_{index}" for index in range(6)),
    *(f"wrench_{index}" for index in range(6)),
    *(f"q_{index}" for index in range(7)),
    *(f"qd_{index}" for index in range(7)),
    *(f"tau_{index}" for index in range(7)),
    "contact",
    "fault",
)


class RightArmChopper:
    def __init__(self, runtime: TwinMujocoRuntime | None = None) -> None:
        self.runtime = runtime or TwinMujocoRuntime.load()
        self.left = self.runtime.arm_view("left")
        self.right = self.runtime.arm_view("right")
        self.controller = CartesianForceController(self.right)
        self.samples: list[ForceControlSample] = []

    def run(
        self,
        config: ChoppingConfig | None = None,
        log_path: str | Path | None = None,
        viewer_sync: Callable[[], None] | None = None,
    ) -> list[ForceControlSample]:
        cfg = config or ChoppingConfig()
        if cfg.cycles < 1:
            raise ValueError("cycles must be at least 1")
        if cfg.control_hz <= 0.0:
            raise ValueError("control_hz must be positive")
        if cfg.force_hold_s < 0.0:
            raise ValueError("force_hold_s must be non-negative")
        if cfg.descent_speed_m_s <= 0.0:
            raise ValueError("descent_speed_m_s must be positive")
        if cfg.retract_speed_m_s <= 0.0:
            raise ValueError("retract_speed_m_s must be positive")

        self.runtime.reset()
        self.runtime.set_arm_positions("left", LEFT_HOME_Q)
        self.runtime.set_arm_positions("right", RIGHT_CHOPPING_HOME_Q)
        self.samples.clear()
        self.controller = CartesianForceController(self.right)

        position, rotation = self.right.site_pose("right_tool_tip_site")
        board_top = self._board_top()
        safe = position.copy()
        safe[2] = max(float(safe[2]), board_top + cfg.safe_height_m)
        descend = safe.copy()
        descend[2] = board_top + 0.02
        control_dt = _control_interval(cfg.control_hz, self.runtime.timestep)
        substeps = _substeps_per_control(cfg.control_hz, self.runtime.timestep)

        elapsed = 0.0
        torque = np.zeros(7)
        final_target = safe.copy()
        for cycle in range(cfg.cycles):
            shifted_safe = safe.copy()
            shifted_safe[0] += cycle * cfg.spacing_m
            shifted_descend = descend.copy()
            shifted_descend[0] += cycle * cfg.spacing_m

            phase = ChoppingPhase.APPROACH if cycle == 0 else ChoppingPhase.SHIFT
            elapsed, torque = self._record_phase(
                elapsed,
                phase,
                shifted_safe,
                shifted_safe,
                rotation,
                8,
                0.0,
                torque,
                control_dt,
                substeps,
                viewer_sync,
            )
            descend_steps = _motion_steps(shifted_safe, shifted_descend, cfg.descent_speed_m_s, control_dt)
            elapsed, torque = self._record_phase(
                elapsed,
                ChoppingPhase.DESCEND,
                shifted_safe,
                shifted_descend,
                rotation,
                descend_steps,
                0.0,
                torque,
                control_dt,
                substeps,
                viewer_sync,
            )
            self.controller.enable_force(True, target_force_n=cfg.target_force_n)
            elapsed, torque = self._record_phase(
                elapsed,
                ChoppingPhase.FORCE_HOLD,
                shifted_descend,
                shifted_descend,
                rotation,
                max(2, int(np.ceil(cfg.force_hold_s / control_dt))),
                cfg.target_force_n,
                torque,
                control_dt,
                substeps,
                viewer_sync,
            )
            self.controller.enable_force(False)
            retract_steps = _motion_steps(shifted_descend, shifted_safe, cfg.retract_speed_m_s, control_dt)
            elapsed, torque = self._record_phase(
                elapsed,
                ChoppingPhase.RETRACT,
                shifted_descend,
                shifted_safe,
                rotation,
                retract_steps,
                0.0,
                torque,
                control_dt,
                substeps,
                viewer_sync,
            )
            final_target = shifted_safe

        self.samples.append(
            self._sample(self.runtime.data.time, ChoppingPhase.COMPLETE, "IDLE", final_target, 0.0, np.zeros(7), "")
        )
        if viewer_sync is not None:
            viewer_sync()
        if log_path is not None:
            self.write_csv(log_path)
        return list(self.samples)

    def write_csv(self, log_path: str | Path) -> None:
        path = Path(log_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=_CSV_FIELDS)
            writer.writeheader()
            for sample in self.samples:
                writer.writerow(_sample_row(sample))

    def _record_phase(
        self,
        elapsed: float,
        phase: ChoppingPhase,
        start: np.ndarray,
        target: np.ndarray,
        rotation: np.ndarray,
        steps: int,
        target_force_n: float,
        torque: np.ndarray,
        control_dt: float,
        substeps: int,
        viewer_sync: Callable[[], None] | None,
    ) -> tuple[float, np.ndarray]:
        mode = "FORCE" if phase == ChoppingPhase.FORCE_HOLD else "POSITION"
        control_wrench = _control_wrench(phase, target_force_n)
        count = max(1, steps)
        for index in range(count):
            phase_target = _interpolate(start, target, index + 1, count)
            self.controller.set_target(phase_target, rotation)
            torque = self.controller.compute(control_wrench)
            applied = self.right.apply_torque(torque)
            self._hold_left_arm()
            for _ in range(substeps):
                self.runtime.step()
                self.runtime.set_arm_positions("left", LEFT_HOME_Q)
            raw_wrench = self._measured_wrench()
            compensated_wrench = raw_wrench.copy()
            self.samples.append(
                self._sample(
                    self.runtime.data.time,
                    phase,
                    mode,
                    phase_target,
                    target_force_n,
                    applied,
                    "",
                    raw_wrench=raw_wrench,
                    compensated_wrench=compensated_wrench,
                )
            )
            if viewer_sync is not None:
                viewer_sync()
            elapsed += control_dt
        return elapsed, torque


    def _hold_left_arm(self) -> np.ndarray:
        return self.left.apply_torque(self.left.bias_torque)

    def _sample(
        self,
        elapsed: float,
        phase: ChoppingPhase,
        control_mode: str,
        target_position: Sequence[float],
        target_force_n: float,
        torque: Sequence[float],
        fault: str,
        *,
        raw_wrench: Sequence[float] | None = None,
        compensated_wrench: Sequence[float] | None = None,
    ) -> ForceControlSample:
        actual_position, _ = self.right.site_pose("right_tool_tip_site")
        raw = np.zeros(6) if raw_wrench is None else np.asarray(raw_wrench, dtype=float).reshape(6)
        wrench = np.zeros(6) if compensated_wrench is None else np.asarray(compensated_wrench, dtype=float).reshape(6)
        tau = np.asarray(torque, dtype=float).reshape(7)
        return ForceControlSample(
            time_s=float(elapsed),
            phase=phase,
            control_mode=control_mode,
            target_position=_triple(target_position),
            actual_position=_triple(actual_position),
            target_force_n=float(target_force_n),
            measured_force_n=float(wrench[2]),
            raw_wrench=tuple(float(value) for value in raw),
            compensated_wrench=tuple(float(value) for value in wrench),
            joint_positions=tuple(float(value) for value in self.right.joint_positions),
            joint_velocities=tuple(float(value) for value in self.right.joint_velocities),
            joint_torques=tuple(float(value) for value in tau),
            contact=bool(abs(wrench[2]) > 1e-6),
            fault=fault,
        )

    def _measured_wrench(self) -> np.ndarray:
        force = self._sensor_values("right_tool_force")
        torque = self._sensor_values("right_tool_torque")
        return np.concatenate((force, torque))

    def _sensor_values(self, name: str) -> np.ndarray:
        sensor_id = self.runtime._id(mujoco.mjtObj.mjOBJ_SENSOR, name)
        address = int(self.runtime.model.sensor_adr[sensor_id])
        dimension = int(self.runtime.model.sensor_dim[sensor_id])
        return self.runtime.data.sensordata[address : address + dimension].copy()

    def _board_top(self) -> float:
        geom_id = self.runtime._id(mujoco.mjtObj.mjOBJ_GEOM, "chopping_board")
        return float(self.runtime.data.geom_xpos[geom_id, 2] + self.runtime.model.geom_size[geom_id, 2])


def _motion_steps(start: Sequence[float], end: Sequence[float], speed_m_s: float, control_dt: float) -> int:
    distance = float(np.linalg.norm(np.asarray(end, dtype=float).reshape(3) - np.asarray(start, dtype=float).reshape(3)))
    return max(1, int(np.ceil(distance / speed_m_s / control_dt)))


def _control_interval(control_hz: float, runtime_timestep: float) -> float:
    return _substeps_per_control(control_hz, runtime_timestep) * runtime_timestep


def _substeps_per_control(control_hz: float, runtime_timestep: float) -> int:
    return max(1, int(round((1.0 / control_hz) / runtime_timestep)))


def _interpolate(start: Sequence[float], end: Sequence[float], step: int, steps: int) -> np.ndarray:
    start_array = np.asarray(start, dtype=float).reshape(3)
    end_array = np.asarray(end, dtype=float).reshape(3)
    return start_array + (end_array - start_array) * (float(step) / float(max(1, steps)))


def _control_wrench(phase: ChoppingPhase, target_force_n: float) -> np.ndarray:
    if phase == ChoppingPhase.FORCE_HOLD:
        return np.array((0.0, 0.0, float(target_force_n), 0.0, 0.0, 0.0), dtype=float)
    return np.zeros(6)


def _triple(values: Sequence[float]) -> tuple[float, float, float]:
    array = np.asarray(values, dtype=float).reshape(3)
    return tuple(float(value) for value in array)


def _sample_row(sample: ForceControlSample) -> dict[str, object]:
    row: dict[str, object] = {
        "time_s": f"{sample.time_s:.6f}",
        "phase": sample.phase.value,
        "control_mode": sample.control_mode,
        "target_x": sample.target_position[0],
        "target_y": sample.target_position[1],
        "target_z": sample.target_position[2],
        "actual_x": sample.actual_position[0],
        "actual_y": sample.actual_position[1],
        "actual_z": sample.actual_position[2],
        "target_force_n": sample.target_force_n,
        "measured_force_n": sample.measured_force_n,
        "contact": int(sample.contact),
        "fault": sample.fault,
    }
    for index, value in enumerate(sample.raw_wrench):
        row[f"raw_wrench_{index}"] = value
    for index, value in enumerate(sample.compensated_wrench):
        row[f"wrench_{index}"] = value
    for index, value in enumerate(sample.joint_positions):
        row[f"q_{index}"] = value
    for index, value in enumerate(sample.joint_velocities):
        row[f"qd_{index}"] = value
    for index, value in enumerate(sample.joint_torques):
        row[f"tau_{index}"] = value
    return row
