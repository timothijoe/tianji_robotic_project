from __future__ import annotations

import csv
import math
import time
from dataclasses import replace
from pathlib import Path
from typing import Callable, Sequence

import numpy as np

from cook_mujoco.control.force_control import (
    CartesianForceController,
    CartesianImpedanceConfig,
    ChoppingConfig,
    ChoppingPhase,
    ForceControlConfig,
    ForceControlRuntime,
    ForceControlSample,
    SafetyConfig,
    SafetyStop,
    WrenchCalibrator,
    downward_tool_rotation,
    interpolate_linear,
)


DEFAULT_HOME_Q = np.array((-0.0000831356, 0.0628633182, 0.0000762999, -1.6292067638, -1.5751170702, 1.0421541060, -1.5658))


class MujocoForceRobot:
    """SDK-style facade for Cartesian impedance and chopping force control."""

    def __init__(
        self,
        model_path: str | Path,
        *,
        viewer: bool = False,
        realtime: bool = False,
        safety: SafetyConfig | None = None,
    ) -> None:
        self.runtime = ForceControlRuntime.load(model_path)
        self.safety = safety or SafetyConfig()
        self.controller = CartesianForceController(self.runtime)
        self.calibrator = WrenchCalibrator()
        self.viewer_enabled = bool(viewer)
        self.realtime = bool(realtime)
        self._viewer = None
        self._stopped = False
        self._samples: list[ForceControlSample] = []
        self._last_compensated = np.zeros(6)
        self._phase = ChoppingPhase.APPROACH
        self._fault = ""
        self._target_force_n = 10.0

    def connect(self, robot_ip: str = "mujoco") -> bool:
        del robot_ip
        if self.viewer_enabled and self._viewer is None:
            from mujoco import viewer

            self._viewer = viewer.launch_passive(self.runtime.model, self.runtime.data)
            self._viewer.cam.lookat[:] = (0.25, 0.0, 0.35)
            self._viewer.cam.distance = 1.15
            self._viewer.cam.azimuth = 135
            self._viewer.cam.elevation = -25
        return True

    def close(self) -> None:
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None

    def stop(self) -> None:
        self._stopped = True
        self.controller.enable_force(False)

    def set_cartesian_impedance(
        self,
        *,
        translational_stiffness: Sequence[float] | None = None,
        translational_damping: Sequence[float] | None = None,
        rotational_stiffness: Sequence[float] | None = None,
        rotational_damping: Sequence[float] | None = None,
    ) -> None:
        config = self.controller.impedance
        self.controller.impedance = replace(
            config,
            translational_stiffness=tuple(translational_stiffness or config.translational_stiffness),
            translational_damping=tuple(translational_damping or config.translational_damping),
            rotational_stiffness=tuple(rotational_stiffness or config.rotational_stiffness),
            rotational_damping=tuple(rotational_damping or config.rotational_damping),
        )

    def set_force_control_params(
        self,
        *,
        target_force_n: float | None = None,
        contact_enter_n: float | None = None,
        contact_exit_n: float | None = None,
        proportional_gain: float | None = None,
        integral_gain: float | None = None,
    ) -> None:
        config = self.controller.force
        config = replace(
            config,
            target_force_n=config.target_force_n if target_force_n is None else float(target_force_n),
            contact_enter_n=config.contact_enter_n if contact_enter_n is None else float(contact_enter_n),
            contact_exit_n=config.contact_exit_n if contact_exit_n is None else float(contact_exit_n),
            proportional_gain=config.proportional_gain if proportional_gain is None else float(proportional_gain),
            integral_gain=config.integral_gain if integral_gain is None else float(integral_gain),
        )
        if config.target_force_n < 0 or config.contact_enter_n <= config.contact_exit_n:
            raise ValueError("invalid force-control parameters")
        self.controller.force = config
        self._target_force_n = config.target_force_n

    def set_force_cmd(self, *, force_n: float) -> None:
        self.set_force_control_params(target_force_n=force_n)
        self.controller.enable_force(True, target_force_n=force_n)

    def get_wrench(self) -> np.ndarray:
        """Return compensated [Fx, Fy, Fz, Mx, My, Mz] in the tool frame."""
        return self._last_compensated.copy()

    def get_raw_wrench(self) -> np.ndarray:
        """Return raw MuJoCo [Fx, Fy, Fz, Mx, My, Mz] sensor data."""
        return self.runtime.raw_wrench()

    @property
    def control_mode(self) -> str:
        return "HYBRID_FORCE_Z" if self.controller.force_enabled else "CARTESIAN_IMPEDANCE"

    def initialize(self, *, home_q: Sequence[float] = DEFAULT_HOME_Q, calibration_s: float = 1.0) -> None:
        self.runtime.reset(home_q)
        position, rotation = self.runtime.site_pose()
        self.controller = CartesianForceController(
            self.runtime,
            impedance=self.controller.impedance,
            force=self.controller.force,
        )
        self.controller.set_target(position, rotation)
        self.controller.nullspace_reference = self.runtime.joint_positions
        # Stabilize using Cartesian impedance. Calibration input is treated as zero
        # until a stationary sample window has been collected.
        raw_samples: list[np.ndarray] = []
        steps = max(10, int(calibration_s / self.runtime.timestep))
        initial_raw = self.runtime.raw_wrench()
        for index in range(steps):
            torque = self.controller.compute(np.zeros(6))
            self._step_runtime(torque)
            if index >= steps // 2:
                raw_samples.append(self.runtime.raw_wrench())
        if np.max(np.abs(self.runtime.joint_velocities)) > 0.08:
            raise SafetyStop("robot did not become stationary during wrench calibration")
        _, sensor_rotation = self.runtime.site_pose(self.runtime.sensor_site_name)
        # Use the measured stationary average as the exact zero. The calibrator's
        # gravity model then adjusts that zero if tool orientation changes.
        self.calibrator.calibrate(raw_samples or [initial_raw], sensor_rotation)
        self._last_compensated = self._compensated_wrench()
        self._samples.clear()
        self._stopped = False
        self._fault = ""

    def execute_static_force_test(
        self,
        *,
        target_force_n: float = 10.0,
        hold_s: float = 5.0,
        log_path: str | Path | None = None,
    ) -> list[ForceControlSample]:
        config = ChoppingConfig(cycles=1, force_hold_s=hold_s, target_force_n=target_force_n)
        return self.execute_chopping_trajectory(config=config, log_path=log_path)

    def execute_chopping_trajectory(
        self,
        *,
        config: ChoppingConfig | None = None,
        log_path: str | Path | None = None,
        status_callback: Callable[[ForceControlSample], None] | None = None,
    ) -> list[ForceControlSample]:
        cfg = config or ChoppingConfig()
        if cfg.cycles < 1 or cfg.safe_height_m <= 0 or cfg.descent_speed_m_s <= 0:
            raise ValueError("invalid chopping configuration")
        self.set_force_control_params(target_force_n=cfg.target_force_n)
        self._samples.clear()
        self._stopped = False
        self._fault = ""
        _, rotation = self.runtime.site_pose()
        if float(rotation[2, 2]) > -0.98:
            raise SafetyStop("tool Z axis is not sufficiently downward at the safe home pose")
        board_top = self._board_top()
        safe_z = board_top + cfg.safe_height_m
        search_start_z = board_top + min(0.02, cfg.safe_height_m * 0.5)
        start_position, _ = self.runtime.site_pose()
        base_x = float(start_position[0])
        y = float(start_position[1])

        try:
            self._move_cartesian(
                np.array((base_x, y, safe_z)),
                rotation,
                speed=cfg.retract_speed_m_s,
                phase=ChoppingPhase.APPROACH,
                target_force_n=0.0,
                callback=status_callback,
            )
            for cycle in range(cfg.cycles):
                x = base_x + cycle * cfg.spacing_m
                self._move_cartesian(
                    np.array((x, y, safe_z)),
                    rotation,
                    speed=cfg.retract_speed_m_s,
                    phase=ChoppingPhase.SHIFT if cycle else ChoppingPhase.APPROACH,
                    target_force_n=0.0,
                    callback=status_callback,
                )
                self._move_cartesian(
                    np.array((x, y, search_start_z)),
                    rotation,
                    speed=cfg.descent_speed_m_s,
                    phase=ChoppingPhase.DESCEND,
                    target_force_n=0.0,
                    callback=status_callback,
                )
                self._search_contact_and_hold(
                    x=x,
                    y=y,
                    rotation=rotation,
                    board_top=board_top,
                    config=cfg,
                    callback=status_callback,
                )
                current, _ = self.runtime.site_pose()
                self.controller.enable_force(False)
                self._move_cartesian(
                    np.array((x, y, safe_z)),
                    rotation,
                    speed=cfg.retract_speed_m_s,
                    phase=ChoppingPhase.RETRACT,
                    target_force_n=0.0,
                    callback=status_callback,
                    start=current,
                )
            self._phase = ChoppingPhase.COMPLETE
        except (SafetyStop, KeyboardInterrupt) as exc:
            self._fault = str(exc) or "interrupted"
            self._phase = ChoppingPhase.FAULT
            self.controller.enable_force(False)
            self._safe_retract(safe_z, rotation)
            if isinstance(exc, SafetyStop):
                raise
        finally:
            if log_path is not None:
                self.write_csv(log_path)
        return list(self._samples)

    def _search_contact_and_hold(
        self,
        *,
        x: float,
        y: float,
        rotation: np.ndarray,
        board_top: float,
        config: ChoppingConfig,
        callback: Callable[[ForceControlSample], None] | None,
    ) -> None:
        self._phase = ChoppingPhase.DESCEND
        start_position, _ = self.runtime.site_pose()
        start_z = float(start_position[2])
        minimum_z = start_z - self.safety.maximum_descent_m
        started = float(self.runtime.data.time)
        contact = False
        contact_search_speed = min(config.descent_speed_m_s, 0.005)
        while not contact:
            elapsed = float(self.runtime.data.time) - started
            if elapsed > self.safety.contact_timeout_s:
                raise SafetyStop("contact establishment timeout")
            next_z = max(minimum_z, start_z - contact_search_speed * elapsed)
            self.controller.set_target((x, y, next_z), rotation)
            sample = self._control_step(ChoppingPhase.DESCEND, 0.0, callback)
            contact = sample.compensated_wrench[2] >= self.controller.force.contact_enter_n
            if next_z <= minimum_z and not contact:
                raise SafetyStop("maximum contact-search descent exceeded")

        self._phase = ChoppingPhase.FORCE_HOLD
        self.controller.enable_force(True, target_force_n=config.target_force_n)
        hold_start = float(self.runtime.data.time)
        while float(self.runtime.data.time) - hold_start < config.force_hold_s:
            self._control_step(ChoppingPhase.FORCE_HOLD, config.target_force_n, callback)

    def _move_cartesian(
        self,
        target: np.ndarray,
        rotation: np.ndarray,
        *,
        speed: float,
        phase: ChoppingPhase,
        target_force_n: float,
        callback: Callable[[ForceControlSample], None] | None,
        start: np.ndarray | None = None,
    ) -> None:
        self.controller.enable_force(False)
        initial = self.runtime.site_pose()[0] if start is None else np.asarray(start, dtype=float)
        distance = float(np.linalg.norm(target - initial))
        duration = max(self.runtime.timestep, distance / max(speed, 1e-6))
        steps = max(1, int(math.ceil(duration / self.runtime.timestep)))
        self._phase = phase
        for index in range(steps):
            desired = interpolate_linear(initial, target, (index + 1) / steps)
            self.controller.set_target(desired, rotation)
            self._control_step(phase, target_force_n, callback)

    def _control_step(
        self,
        phase: ChoppingPhase,
        target_force_n: float,
        callback: Callable[[ForceControlSample], None] | None,
    ) -> ForceControlSample:
        if self._stopped:
            raise SafetyStop("stop requested")
        compensated = self._compensated_wrench()
        torque = self.controller.compute(compensated)
        applied = self._step_runtime(torque)
        compensated = self._compensated_wrench()
        self._last_compensated = compensated
        measured = float(self.controller.measured_force_n)
        position, _ = self.runtime.site_pose()
        position_error = float(np.linalg.norm(self.controller.desired_position - position))
        sensor_force_z = float(compensated[2])
        if abs(sensor_force_z) > self.safety.maximum_force_n:
            raise SafetyStop(f"force limit exceeded: {sensor_force_z:.3f} N")
        if position_error > self.safety.maximum_position_error_m:
            raise SafetyStop(f"Cartesian tracking error exceeded: {position_error:.4f} m")
        sample = ForceControlSample(
            time_s=float(self.runtime.data.time),
            phase=phase,
            control_mode=self.control_mode,
            target_position=tuple(float(v) for v in self.controller.desired_position),
            actual_position=tuple(float(v) for v in position),
            target_force_n=float(target_force_n),
            measured_force_n=measured,
            raw_wrench=tuple(float(v) for v in self.runtime.raw_wrench()),
            compensated_wrench=tuple(float(v) for v in compensated),
            joint_positions=tuple(float(v) for v in self.runtime.joint_positions),
            joint_velocities=tuple(float(v) for v in self.runtime.joint_velocities),
            joint_torques=tuple(float(v) for v in applied),
            contact=sensor_force_z >= self.controller.force.contact_enter_n,
            fault=self._fault,
        )
        self._samples.append(sample)
        if callback is not None:
            callback(sample)
        return sample

    def _compensated_wrench(self) -> np.ndarray:
        _, sensor_rotation = self.runtime.site_pose(self.runtime.sensor_site_name)
        return self.calibrator.compensate(self.runtime.raw_wrench(), sensor_rotation)

    def _step_runtime(self, torque: Sequence[float]) -> np.ndarray:
        wall_start = time.monotonic()
        applied = self.runtime.step(torque)
        if self._viewer is not None:
            if not self._viewer.is_running():
                raise SafetyStop("viewer closed")
            self._viewer.sync()
        if self.realtime:
            delay = self.runtime.timestep - (time.monotonic() - wall_start)
            if delay > 0:
                time.sleep(delay)
        return applied

    def _safe_retract(self, safe_z: float, rotation: np.ndarray) -> None:
        try:
            position, _ = self.runtime.site_pose()
            target = position.copy()
            target[2] = max(target[2], safe_z)
            self._move_cartesian(
                target,
                rotation,
                speed=0.05,
                phase=ChoppingPhase.RETRACT,
                target_force_n=0.0,
                callback=None,
            )
        except Exception:
            self._stopped = True

    def _board_top(self) -> float:
        mujoco = self.runtime._mujoco
        geom_id = int(mujoco.mj_name2id(self.runtime.model, mujoco.mjtObj.mjOBJ_GEOM, "chopping_board"))
        if geom_id < 0:
            raise ValueError("chopping_board geom is missing")
        return float(self.runtime.model.geom_pos[geom_id, 2] + self.runtime.model.geom_size[geom_id, 2])

    def write_csv(self, path: str | Path) -> Path:
        output = Path(path).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        fields = [
            "time_s", "phase", "control_mode", "target_x", "target_y", "target_z",
            "actual_x", "actual_y", "actual_z", "target_force_n", "measured_force_n",
            *[f"raw_wrench_{i}" for i in range(6)],
            *[f"wrench_{i}" for i in range(6)],
            *[f"q_{i}" for i in range(7)],
            *[f"qd_{i}" for i in range(7)],
            *[f"tau_{i}" for i in range(7)],
            "contact", "fault",
        ]
        with output.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for sample in self._samples:
                row = {
                    "time_s": sample.time_s,
                    "phase": sample.phase.value,
                    "control_mode": sample.control_mode,
                    "target_x": sample.target_position[0], "target_y": sample.target_position[1], "target_z": sample.target_position[2],
                    "actual_x": sample.actual_position[0], "actual_y": sample.actual_position[1], "actual_z": sample.actual_position[2],
                    "target_force_n": sample.target_force_n, "measured_force_n": sample.measured_force_n,
                    "contact": int(sample.contact), "fault": sample.fault,
                }
                row.update({f"raw_wrench_{i}": v for i, v in enumerate(sample.raw_wrench)})
                row.update({f"wrench_{i}": v for i, v in enumerate(sample.compensated_wrench)})
                row.update({f"q_{i}": v for i, v in enumerate(sample.joint_positions)})
                row.update({f"qd_{i}": v for i, v in enumerate(sample.joint_velocities)})
                row.update({f"tau_{i}": v for i, v in enumerate(sample.joint_torques)})
                writer.writerow(row)
        return output

    def summary(self) -> dict[str, float | int | str]:
        if not self._samples:
            return {"samples": 0, "phase": self._phase.value, "fault": self._fault}
        forces = np.array([sample.measured_force_n for sample in self._samples])
        sensor_forces = np.array([sample.compensated_wrench[2] for sample in self._samples])
        position_errors = np.array([
            np.linalg.norm(np.asarray(sample.target_position) - np.asarray(sample.actual_position))
            for sample in self._samples
        ])
        hold = np.array([sample.phase == ChoppingPhase.FORCE_HOLD for sample in self._samples])
        noncontact = ~hold
        force_rmse = float(np.sqrt(np.mean((forces[hold] - self._target_force_n) ** 2))) if np.any(hold) else math.nan
        noncontact_rmse = float(np.sqrt(np.mean(position_errors[noncontact] ** 2))) if np.any(noncontact) else math.nan
        return {
            "samples": len(self._samples),
            "phase": self._phase.value,
            "fault": self._fault,
            "peak_force_n": float(np.max(np.abs(sensor_forces))),
            "peak_filtered_force_n": float(np.max(np.abs(forces))),
            "force_hold_rmse_n": force_rmse,
            "position_rmse_m": float(np.sqrt(np.mean(position_errors**2))),
            "noncontact_position_rmse_m": noncontact_rmse,
        }
