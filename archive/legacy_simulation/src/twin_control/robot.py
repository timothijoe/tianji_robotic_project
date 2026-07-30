"""TwinRobot: SDK-style unified interface for MuJoCo-based control.

Aligns with the ``Concise_Marvin_Robot`` API from ``TJ_FX_ROBOT_CONTRL_SDK``:

1. ``connect(model_path)`` — load MJCF, initialise runtime + kinematics + controller.
2. ``set_*_state(...)`` — switch control mode and configure parameters.
3. ``set_joint_position_cmd(...)`` / ``set_force_cmd(...)`` — dispatch targets.
4. ``step()`` / ``spin(N)`` — advance simulation.
5. ``close()`` — release resources.

Internally uses ``TwinMujocoRuntime`` + ``ArmView`` from ``twin_mujoco``
and delegates torque computation to ``UnifiedController``.

TCP trail visualisation is handled by ``TcpTrail`` (``twin_control.trail``).

Public API
----------
RobotState                 enum: IDLE / POSITION / JOINT_IMPEDANCE / CARTESIAN_IMPEDANCE / FORCE
RIGHT_HOME_RAD / LEFT_HOME_RAD   (7,) ndarray — default home configurations
_DEFAULT_JOINT_K / _DEFAULT_JOINT_D  — default joint impedance (SI)
_DEFAULT_CART_K / _DEFAULT_CART_D   — default Cartesian impedance (SI)
TwinRobot(arm_name, unit_mode, control_hz, tcp_site_name)
    .connect(model_path, viewer, realtime)
    .set_position_state(vel, acc) / .set_joint_impedance_state(vel, acc, K, D)
    .set_cart_impedance_state(vel, acc, K, D) / .set_force_state(vel, acc, K, D, fx_dir, adj_lmt)
    .disable()
    .set_joint_position_cmd(joints) / .set_force_cmd(force_n)
    .set_tool(kine_para) / .remove_tool()
    .get_joint_positions() / .get_joint_velocities()
    .get_tcp_pose() / .get_raw_wrench() / .get_wrench() / .calibrate_wrench(samples)
    .step(viewer_sync) / .spin(steps, viewer_sync)
    .hold_viewer_open(hz)
    .clear_log() / .write_csv(path)
"""

from __future__ import annotations

import csv
import time
from enum import Enum
from pathlib import Path
from typing import Callable, Sequence

import mujoco
import mujoco.viewer  # loads lazy submodule (needed in MuJoCo 3.9)
import numpy as np

from twin_control.controller import (
    CartesianImpedanceParams,
    ControlMode,
    ForceControlParams,
    JointImpedanceParams,
    UnifiedController,
)
from twin_control.kinematics import MarvinKinematics
from twin_control.trail import TcpTrail
from twin_description.paths import right_chopping_scene_path
from twin_mujoco.runtime import ArmView, TwinMujocoRuntime


# ---------------------------------------------------------------------------
# Home configurations
# ---------------------------------------------------------------------------

RIGHT_HOME_RAD = np.array(
    (0.4, -1.3, 0.08, -1.606525, 0.057176, 0.79256, 1.5), dtype=float,
)
LEFT_HOME_RAD = np.zeros(7, dtype=float)


# ---------------------------------------------------------------------------
# CSV log fields
# ---------------------------------------------------------------------------

_CSV_FIELDS = (
    "time_s",
    "control_mode",
    "target_x", "target_y", "target_z",
    "actual_x", "actual_y", "actual_z",
    "target_force_n", "measured_force_n",
    *(f"raw_wrench_{i}" for i in range(6)),
    *(f"q_{i}" for i in range(7)),
    *(f"qd_{i}" for i in range(7)),
    *(f"tau_{i}" for i in range(7)),
)


class RobotState(str, Enum):
    IDLE = "IDLE"
    POSITION = "POSITION"
    JOINT_IMPEDANCE = "JOINT_IMPEDANCE"
    CARTESIAN_IMPEDANCE = "CARTESIAN_IMPEDANCE"
    FORCE = "FORCE"


# Default impedance parameters (SI units, tuned for stable MuJoCo simulation)
_DEFAULT_JOINT_K = (8.0, 8.0, 8.0, 4.0, 2.0, 1.5, 1.0)
_DEFAULT_JOINT_D = (1.5, 1.5, 1.5, 1.0, 0.6, 0.4, 0.3)
_DEFAULT_CART_K = (2000.0, 2000.0, 2500.0, 40.0, 40.0, 30.0, 4.0)
_DEFAULT_CART_D = (100.0, 100.0, 110.0, 5.0, 5.0, 4.0, 1.5)


# ---------------------------------------------------------------------------
# TwinRobot
# ---------------------------------------------------------------------------

class TwinRobot:
    """SDK-style unified robot interface backed by MuJoCo.

    Parameters
    ----------
    arm_name : str
        ``"left"`` or ``"right"``.
    unit_mode : str
        ``"si"`` (rad, m) or ``"sdk"`` (deg, mm).
    control_hz : float
        Control loop rate.  Default 500 Hz.
    tcp_site_name : str or None
        MuJoCo site used as end-effector.  Defaults to ``"{arm}_force_sensor_site"``.
    """

    def __init__(
        self,
        arm_name: str = "right",
        unit_mode: str = "si",
        control_hz: float = 500.0,
        tcp_site_name: str | None = None,
    ) -> None:
        if arm_name not in ("left", "right"):
            raise ValueError(f"arm_name must be 'left' or 'right', got '{arm_name}'")
        self.arm_name = arm_name
        self.unit_mode = unit_mode
        self.control_hz = float(control_hz)
        self._tcp_site_name_override = tcp_site_name

        # Set during connect()
        self.runtime: TwinMujocoRuntime | None = None
        self._arm: ArmView | None = None
        self._kinematics: MarvinKinematics | None = None
        self._controller: UnifiedController | None = None
        self._viewer = None
        self._trail: TcpTrail | None = None
        self._state = RobotState.IDLE
        self._samples: list[dict] = []
        self._wrench_bias = np.zeros(6)
        self._wrench_calibrated = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect(
        self,
        model_path: str | Path | None = None,
        viewer: bool = False,
        realtime: bool = True,
    ) -> None:
        """Load MJCF model and initialise runtime.

        Parameters
        ----------
        model_path : Path or None
            Path to MJCF/XML file.  Defaults to ``right_chopping_scene.xml``.
        viewer : bool
            If True, open a MuJoCo viewer window.
        realtime : bool
            If True, run the viewer in real-time mode.
        """
        path = Path(model_path) if model_path else right_chopping_scene_path()
        self.runtime = TwinMujocoRuntime.load(str(path))
        self.runtime.reset()
        self.runtime.set_arm_positions("left", LEFT_HOME_RAD)
        self.runtime.set_arm_positions("right", RIGHT_HOME_RAD)

        self._arm = self.runtime.arm_view(self.arm_name)
        self._kinematics = MarvinKinematics(
            self.arm_name, unit_mode=self.unit_mode,
            tcp_site_name=self._tcp_site_name,
        )
        self._kinematics.set_runtime(self.runtime)
        self._controller = UnifiedController(
            joint_limits_rad=self._kinematics.joint_limits_rad,
            torque_limits_nm=self._arm.effort_limits,
            torque_rate_limit_nm_per_s=1500.0,
            dt_s=self._control_dt,
        )

        self._trail = None
        if viewer:
            self._viewer = mujoco.viewer.launch_passive(
                self.runtime.model, self.runtime.data,
            )
            self._trail = TcpTrail(self._viewer, self._tcp_site_name)
            _configure_viewer_camera(self._viewer)
            print("[TwinRobot] viewer trail: actual=orange, target=cyan")

        self._state = RobotState.IDLE
        self._samples.clear()
        print(f"[TwinRobot] connected: {path.name}  arm={self.arm_name}")

    def close(self) -> None:
        """Release resources."""
        if self._viewer is not None:
            self._viewer.close()
            time.sleep(0.5)
            self._viewer = None
        self._trail = None
        self.runtime = None
        self._arm = None
        self._state = RobotState.IDLE
        print("[TwinRobot] disconnected")

    # ------------------------------------------------------------------
    # State switching
    # ------------------------------------------------------------------

    def set_position_state(
        self, vel_ratio: float = 0.5, acc_ratio: float = 0.5,
    ) -> None:
        """Switch to position mode (joint target tracking)."""
        self._ensure_connected()
        fv = float(np.clip(vel_ratio, 0.0, 1.0))
        self._controller.set_joint_impedance_params(
            JointImpedanceParams(
                stiffness=(35.0, 35.0, 30.0, 24.0, 16.0, 10.0, 8.0),
                damping=tuple(d * max(fv, 0.1) for d in (7.0, 7.0, 6.0, 5.0, 3.5, 2.5, 2.0)),
            )
        )
        self._controller.set_mode(ControlMode.POSITION)
        self._state = RobotState.POSITION
        print(f"[TwinRobot] state → POSITION (vel={vel_ratio})")

    def set_joint_impedance_state(
        self, vel_ratio: float, acc_ratio: float,
        K: Sequence[float], D: Sequence[float],
    ) -> None:
        """Switch to joint impedance mode."""
        self._ensure_connected()
        K_si = tuple(float(v) for v in K)
        D_si = tuple(float(v) for v in D)
        if self.unit_mode == "sdk":
            K_si = tuple(v * 57.29578 for v in K_si)
            D_si = tuple(v * 57.29578 for v in D_si)
        self._controller.set_joint_impedance_params(
            JointImpedanceParams(stiffness=K_si, damping=D_si),
        )
        self._controller.set_mode(ControlMode.JOINT_IMPEDANCE)
        self._state = RobotState.JOINT_IMPEDANCE
        print(f"[TwinRobot] state → JOINT_IMPEDANCE  K[0]={K_si[0]:.1f} D[0]={D_si[0]:.2f}")

    def set_cart_impedance_state(
        self, vel_ratio: float, acc_ratio: float,
        K: Sequence[float], D: Sequence[float],
        rot_type: int = 0, cart_ctrl_para: Sequence[float] | None = None,
    ) -> None:
        """Switch to Cartesian impedance mode."""
        self._ensure_connected()
        K_t, D_t = tuple(float(v) for v in K), tuple(float(v) for v in D)
        self._controller.set_cartesian_impedance_params(CartesianImpedanceParams(
            translational_stiffness=K_t[:3], translational_damping=D_t[:3],
            rotational_stiffness=K_t[3:6], rotational_damping=D_t[3:6],
            nullspace_stiffness=K_t[6], nullspace_damping=D_t[6],
        ))
        self._controller.set_mode(ControlMode.CARTESIAN_IMPEDANCE)
        if self._arm is not None:
            self._controller.set_joint_cmd(self._arm.joint_positions)
        self._state = RobotState.CARTESIAN_IMPEDANCE
        print(f"[TwinRobot] state → CARTESIAN_IMPEDANCE  K_trans[0]={K_t[0]:.0f}")

    def set_force_state(
        self, vel_ratio: float, acc_ratio: float,
        K: Sequence[float], D: Sequence[float],
        fx_dir: Sequence[float], fc_adj_lmt: float,
    ) -> None:
        """Switch to force control mode (hybrid Cartesian impedance + admittance)."""
        self._ensure_connected()
        self.set_cart_impedance_state(vel_ratio, acc_ratio, K, D)
        adj_m = fc_adj_lmt * 0.001 if self.unit_mode == "sdk" else float(fc_adj_lmt)
        self._controller.set_force_params(ForceControlParams(
            direction=tuple(float(v) for v in fx_dir),
            max_position_offset_m=abs(adj_m),
        ))
        self._controller.set_mode(ControlMode.FORCE)
        self._state = RobotState.FORCE
        print(f"[TwinRobot] state → FORCE  dir={fx_dir}  adj_lim={adj_m:.4f}m")

    def disable(self) -> None:
        """Disable torque output (set ctrl=0)."""
        self._ensure_connected()
        self._arm.apply_torque(np.zeros(7))
        self._state = RobotState.IDLE
        print("[TwinRobot] disabled")

    # ------------------------------------------------------------------
    # Command dispatch
    # ------------------------------------------------------------------

    def set_joint_position_cmd(self, joints: Sequence[float]) -> None:
        """Set joint-space position target (rad or deg per unit_mode)."""
        self._ensure_connected()
        q = np.asarray(joints, dtype=float).reshape(7)
        if self.unit_mode == "sdk":
            q = np.deg2rad(q)
        self._controller.set_joint_cmd(q)

    def set_force_cmd(self, force: float) -> None:
        """Set force target (Newtons)."""
        self._ensure_connected()
        self._controller.set_force_cmd(force)

    # ------------------------------------------------------------------
    # Tool
    # ------------------------------------------------------------------

    def set_tool(self, kine_para: Sequence[float]) -> None:
        self._ensure_connected()
        self._kinematics.set_tool(kine_para)

    def remove_tool(self) -> None:
        self._ensure_connected()
        self._kinematics.remove_tool()

    # ------------------------------------------------------------------
    # Data acquisition
    # ------------------------------------------------------------------

    def get_joint_positions(self) -> np.ndarray:
        """Return current joint positions (rad or deg per unit_mode)."""
        self._ensure_connected()
        q = self._arm.joint_positions
        return np.rad2deg(q) if self.unit_mode == "sdk" else q

    def get_joint_velocities(self) -> np.ndarray:
        """Return current joint velocities (rad/s or deg/s per unit_mode)."""
        self._ensure_connected()
        qd = self._arm.joint_velocities
        return np.rad2deg(qd) if self.unit_mode == "sdk" else qd

    def get_tcp_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """Return TCP position (m or mm) and 4×4 matrix."""
        self._ensure_connected()
        matrix, xyzabc = self._kinematics.fk(self._arm.joint_positions)
        return xyzabc[:3], matrix

    def get_raw_wrench(self) -> np.ndarray:
        """Return raw 6-D force/torque sensor reading (N, N·m)."""
        self._ensure_connected()
        return _sensor_reading(self.runtime, "right_tool_force", "right_tool_torque")

    def get_wrench(self) -> np.ndarray:
        """Return compensated 6-D wrench (N, N·m)."""
        raw = self.get_raw_wrench()
        if self._wrench_calibrated:
            pose = self._kinematics.fk(self._arm.joint_positions)[0]
            return raw - self._wrench_bias - _gravity_compensation(pose[:3, :3])
        return raw

    def calibrate_wrench(self, samples: int = 500) -> None:
        """Tare the wrench sensor. Arm must be stationary."""
        self._ensure_connected()
        readings = [self.get_raw_wrench() for _ in range(samples)]
        for _ in readings[1:]:  # step between readings
            self.runtime.step()
        self._wrench_bias = np.mean(readings, axis=0)
        self._wrench_calibrated = True
        print(f"[TwinRobot] wrench calibrated  bias={np.round(self._wrench_bias, 3)}")

    # ------------------------------------------------------------------
    # Stepping
    # ------------------------------------------------------------------

    def step(self, viewer_sync: bool = True) -> None:
        """Single control cycle: compute torque → apply → step physics."""
        self._ensure_connected()

        q = self._arm.joint_positions
        qd = self._arm.joint_velocities
        J = self._arm.site_jacobian(self._tcp_site_name)
        pose_matrix = self._kinematics.fk(q)[0]

        wrench = self.get_wrench() if self._state == RobotState.FORCE else None
        compute_pose = pose_matrix if self._state in (
            RobotState.CARTESIAN_IMPEDANCE, RobotState.FORCE, RobotState.POSITION,
        ) else None

        torque = self._controller.compute(
            current_joints_rad=q, current_velocities_rad_s=qd,
            jacobian=J, current_pose_matrix=compute_pose,
            wrench=wrench, bias_torque=self._arm.bias_torque,
        )

        self._arm.apply_torque(torque)
        self._hold_other_arm()
        for _ in range(self._substeps):
            self.runtime.step()
        self._hold_other_arm()

        self._samples.append(self._make_sample(torque, wrench))

        # Trail recording uses the post-step joint state so actual markers
        # align with the sampled robot state shown in the viewer/logs.
        if self._trail is not None:
            self._trail.record(self._arm.joint_positions, self._kinematics, self._controller)

        # Viewer sync
        if self._viewer is not None and self._trail is not None and viewer_sync:
            self._trail.render()
            self._viewer.sync()

    def spin(self, steps: int, viewer_sync: bool = True) -> None:
        """Run ``step()`` *steps* times."""
        for _ in range(steps):
            self.step(viewer_sync=viewer_sync)

    def hold_viewer_open(self, sync_hz: float = 30.0) -> None:
        """Keep the passive viewer open for inspecting the final scene."""
        if self._viewer is None:
            return
        delay = 1.0 / max(float(sync_hz), 1.0)
        try:
            while self._viewer is not None and self._viewer.is_running():
                self._viewer.sync()
                time.sleep(delay)
        except KeyboardInterrupt:
            pass

    # ------------------------------------------------------------------
    # CSV logging
    # ------------------------------------------------------------------

    def clear_log(self) -> None:
        self._samples.clear()

    def write_csv(self, path: str | Path) -> None:
        """Write accumulated samples to a CSV file."""
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        with p.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=_CSV_FIELDS)
            writer.writeheader()
            for s in self._samples:
                writer.writerow(s)
        print(f"[TwinRobot] log saved: {p}  ({len(self._samples)} samples)")

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def state(self) -> RobotState:
        return self._state

    @property
    def control_mode(self) -> str:
        return self._state.value

    @property
    def _tcp_site_name(self) -> str:
        return self._tcp_site_name_override or f"{self.arm_name}_force_sensor_site"

    @property
    def _control_dt(self) -> float:
        return 1.0 / self.control_hz

    @property
    def _substeps(self) -> int:
        if self.runtime is None:
            return 1
        return max(1, int(round(self._control_dt / self.runtime.timestep)))

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _ensure_connected(self) -> None:
        if self.runtime is None or self._arm is None:
            raise RuntimeError("TwinRobot not connected — call connect() first")

    def _hold_other_arm(self) -> None:
        other = "left" if self.arm_name == "right" else "right"
        other_arm = self.runtime.arm_view(other)
        other_arm.apply_torque(other_arm.bias_torque)

    def _make_sample(self, torque: np.ndarray, wrench: np.ndarray | None = None) -> dict:
        pos, _ = self.get_tcp_pose()
        raw = self.get_raw_wrench()
        q = self.get_joint_positions()
        qd = self.get_joint_velocities()
        w = np.zeros(6) if wrench is None else np.asarray(wrench, dtype=float).reshape(6)
        tau = np.asarray(torque, dtype=float).reshape(7)
        sample: dict = {
            "time_s": f"{self.runtime.data.time:.6f}",
            "control_mode": self.control_mode,
            "target_x": 0.0, "target_y": 0.0, "target_z": 0.0,
            "actual_x": float(pos[0]), "actual_y": float(pos[1]), "actual_z": float(pos[2]),
            "target_force_n": 0.0, "measured_force_n": float(w[2]),
        }
        for i in range(6):
            sample[f"raw_wrench_{i}"] = float(raw[i])
        for i in range(7):
            sample[f"q_{i}"] = float(q[i])
            sample[f"qd_{i}"] = float(qd[i])
            sample[f"tau_{i}"] = float(tau[i])
        return sample


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _sensor_reading(runtime: TwinMujocoRuntime, force_name: str, torque_name: str) -> np.ndarray:
    """Read 6-D wrench from named force + torque sensors."""
    fid = _sensor_id(runtime, force_name)
    tid = _sensor_id(runtime, torque_name)
    return np.concatenate((
        runtime.data.sensordata[fid : fid + 3],
        runtime.data.sensordata[tid : tid + 3],
    ))


def _sensor_id(runtime: TwinMujocoRuntime, name: str) -> int:
    sid = int(mujoco.mj_name2id(runtime.model, mujoco.mjtObj.mjOBJ_SENSOR, name))
    return int(runtime.model.sensor_adr[sid])


def _gravity_compensation(sensor_rotation: np.ndarray, tool_mass_kg: float = 0.22) -> np.ndarray:
    """Compute wrench from tool gravity (to be subtracted from raw readings)."""
    R = np.asarray(sensor_rotation, dtype=float).reshape(3, 3)
    gravity_world = np.array((0.0, 0.0, -9.81))
    force = -(R.T @ (tool_mass_kg * gravity_world))
    return np.concatenate((force, np.zeros(3)))


def _configure_viewer_camera(viewer) -> None:
    """Set default camera for the chopping scene."""
    if not hasattr(viewer, "cam"):
        return
    viewer.cam.lookat[:] = (0.38, 0.0, 0.48)
    viewer.cam.distance = 1.15
    viewer.cam.azimuth = 155
    viewer.cam.elevation = -25
