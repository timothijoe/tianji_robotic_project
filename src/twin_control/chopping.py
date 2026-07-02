"""SDK-based chopping state machine for Marvin CCS right arm.

Uses ``TwinRobot`` (SDK-style interface) as the control backend, replacing
the original ``CartesianForceController``-based implementation.  State
machine logic and blade geometry calculations are preserved from the
reference implementation.

The chopping cycle is:  APPROACH → DESCEND → FORCE_HOLD → RETRACT  (repeat).

Public API
----------
ChoppingPhase        enum: APPROACH / DESCEND / FORCE_HOLD / RETRACT / SHIFT / COMPLETE / FAULT
ChoppingConfig       dataclass: cycles, safe_height_m, spacing_m, speeds, force params
BladeGeometry        dataclass: edge_positions, tip_position, tip_rotation
ChoppingSample       dataclass: per-step telemetry (time, phase, position, wrench, joints, torque)
TwinRobotChopper(robot=None)
    .run(config, log_path, headless) → list[ChoppingSample]
    .write_csv(path)
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

from twin_control.rotation import rotation_error
from twin_control.robot import (
    LEFT_HOME_RAD,
    TwinRobot,
    _DEFAULT_CART_D,
    _DEFAULT_CART_K,
)
from twin_description.paths import right_chopping_scene_path
from twin_control.trajectory import (
    cartesian_minimum_jerk_trajectory,
    minimum_jerk_scalar,
)


# ---------------------------------------------------------------------------
# Phase / config / geometry types
# ---------------------------------------------------------------------------

class ChoppingPhase(str, Enum):
    """Phases of a single chopping cycle."""
    APPROACH = "APPROACH"        # move to safe height above board (first cycle only)
    DESCEND = "DESCEND"          # lower blade onto board under Cartesian impedance
    FORCE_HOLD = "FORCE_HOLD"    # maintain target force via admittance control
    RETRACT = "RETRACT"          # lift blade back to safe height
    SHIFT = "SHIFT"              # move to next cutting position (subsequent cycles)
    COMPLETE = "COMPLETE"        # all cycles finished
    FAULT = "FAULT"              # safety stop triggered


@dataclass(frozen=True)
class ChoppingConfig:
    """Parameters for a chopping run.

    Attributes
    ----------
    cycles : int — number of chop cycles (default 3).
    safe_height_m : float — blade clearance above board (m).
    spacing_m : float — X shift between successive chops (m).
    descent_speed_m_s : float — max Cartesian descent speed (m/s).
    retract_speed_m_s : float — max Cartesian retract speed (m/s).
    force_hold_s : float — duration of force-hold phase (s).
    target_force_n : float — desired contact force (N).
    control_hz : float — control loop rate (Hz).
    position_tolerance_m : float — max TCP tracking error before path advance pauses (m).
    orientation_tolerance_rad : float — max terminal TCP orientation error (rad).
    settle_timeout_s : float — max time spent catching up to one path target (s).
    """
    cycles: int = 3
    safe_height_m: float = 0.08
    spacing_m: float = 0.02
    descent_speed_m_s: float = 0.05
    retract_speed_m_s: float = 0.08
    force_hold_s: float = 0.15
    target_force_n: float = 10.0
    control_hz: float = 250.0
    position_tolerance_m: float = 0.005
    orientation_tolerance_rad: float = 0.08726646259971647
    settle_timeout_s: float = 1.0


@dataclass(frozen=True)
class BladeGeometry:
    """Static blade shape captured from MuJoCo site poses at initialisation.

    Attributes
    ----------
    edge_positions : tuple of (x,y,z) — blade edge sites in world frame.
    tip_position : (x,y,z) — tool tip site position.
    tip_rotation : 3×3 tuple-of-tuples — tool tip rotation matrix.
    """
    edge_positions: tuple[tuple[float, float, float], ...]
    tip_position: tuple[float, float, float]
    tip_rotation: tuple[tuple[float, float, float], ...]

    @property
    def min_z(self) -> float:
        """Lowest Z among edge positions."""
        return min(float(p[2]) for p in self.edge_positions)

    @property
    def max_z(self) -> float:
        """Highest Z among edge positions."""
        return max(float(p[2]) for p in self.edge_positions)


@dataclass(frozen=True)
class ChoppingSample:
    """One control-step worth of telemetry.

    Attributes
    ----------
    time_s : float — simulation time (s).
    phase : ChoppingPhase — current state machine phase.
    control_mode : str — "CARTESIAN_IMPEDANCE", "FORCE", or "IDLE".
    target_position : (x,y,z) — commanded TCP position (m).
    actual_position : (x,y,z) — measured TCP position (m).
    target_rotation : 3×3 tuple-of-tuples — commanded TCP rotation matrix.
    actual_rotation : 3×3 tuple-of-tuples — measured TCP rotation matrix.
    target_force_n : float — commanded force (N).
    measured_force_n : float — Z-axis force from wrench (N).
    raw_wrench : (6,) tuple — raw force/torque sensor reading.
    joint_positions : (7,) tuple — joint angles (rad).
    joint_velocities : (7,) tuple — joint velocities (rad/s).
    joint_torques : (7,) tuple — applied joint torques (N·m).
    contact : bool — True if |Fz| > 1e-6.
    fault : str — non-empty if a safety fault occurred.
    blade_reference_positions : three current blade reference positions.
    target_blade_reference_positions : three target blade reference positions.
    """
    time_s: float
    phase: ChoppingPhase
    control_mode: str
    target_position: tuple[float, float, float]
    actual_position: tuple[float, float, float]
    target_rotation: tuple[tuple[float, float, float], ...]
    actual_rotation: tuple[tuple[float, float, float], ...]
    target_force_n: float
    measured_force_n: float
    raw_wrench: tuple[float, ...]
    joint_positions: tuple[float, ...]
    joint_velocities: tuple[float, ...]
    joint_torques: tuple[float, ...]
    contact: bool
    fault: str = ""
    blade_edge_positions: tuple[tuple[float, float, float], ...] = ()
    target_blade_edge_positions: tuple[tuple[float, float, float], ...] = ()
    blade_reference_positions: tuple[tuple[float, float, float], ...] = ()
    target_blade_reference_positions: tuple[tuple[float, float, float], ...] = ()


# ---------------------------------------------------------------------------
# Blade geometry constants
# ---------------------------------------------------------------------------

BLADE_EDGE_SITE_NAMES = ("right_blade_edge_top", "right_blade_edge_bot")
"""MuJoCo site names for the two blade edge reference points."""

BLADE_REFERENCE_SITE_NAMES = (
    "right_blade_edge_top",
    "right_blade_edge_bot",
    "right_tool_tip_site",
)
"""MuJoCo site names for all blade reference points (edges + tip)."""

WORLD_DOWN_AXIS = np.array((0.0, 0.0, -1.0), dtype=float)
"""World-frame downward direction for force control."""

RIGHT_CHOPPING_HOME_RAD = np.array(
    (-1.319946, -1.179343, 0.914382, -2.174226, -1.57813, 0.749651, 0.722104),
    dtype=float,
)
"""Right-arm home configuration used by the reference chopping controller."""

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
        Defaults to right arm, SI units, tool-tip TCP site.
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
        """Execute the full chopping sequence.

        Parameters
        ----------
        config : ChoppingConfig or None — chopping parameters.
        log_path : Path or None — if set, write CSV log to this path.
        headless : bool — if True, run without MuJoCo viewer.

        Returns
        -------
        list[ChoppingSample] — per-step telemetry for the entire run.
        """
        cfg = config or ChoppingConfig()
        _validate_config(cfg)

        # Connect + reset
        self.robot.control_hz = float(cfg.control_hz)
        self.robot.connect(viewer=not headless, realtime=not headless)
        self.robot.runtime.reset()
        self.robot.runtime.set_arm_positions("left", LEFT_HOME_RAD)
        self.robot.runtime.set_arm_positions("right", RIGHT_CHOPPING_HOME_RAD)
        self.samples.clear()

        # Compute blade geometry & Cartesian target parameters
        geometry = self._blade_geometry()
        sensor_pos, _ = self.robot.get_tcp_pose()
        tip_pos = np.array((float(geometry.tip_position[0]), float(geometry.tip_position[1]),
                            float(geometry.tip_position[2])))
        target_rotation = self._horizontal_blade_rotation(geometry)
        board_top = self._board_top()
        safe_tip = self._tip_target_for_blade_clearance(
            tip_pos, target_rotation, geometry, board_top, cfg.safe_height_m,
        )
        descend_tip = self._tip_target_for_blade_on_board(tip_pos, target_rotation, geometry, board_top)
        control_dt = 1.0 / cfg.control_hz

        # Match the reference MuJoCo chopper's softer impedance. The SDK path
        # used to run stiffer gains at 200 Hz, which produced visible chatter
        # around the folded, low hand posture.
        K = (2500.0, 2500.0, 2800.0, 45.0, 45.0, 35.0, _DEFAULT_CART_K[6])
        D = (105.0, 105.0, 115.0, 5.5, 5.5, 4.5, _DEFAULT_CART_D[6])
        terminal_K = K
        terminal_D = D

        # Force control along world down, matching the reference chopper.
        fx_dir = (0.0, 0.0, -1.0, 0.0, 0.0, 0.0)

        for cycle in range(cfg.cycles):
            # Shift target along X for each cycle
            shifted_safe = safe_tip.copy()
            shifted_safe[0] += cycle * cfg.spacing_m
            shifted_descend = descend_tip.copy()
            shifted_descend[0] += cycle * cfg.spacing_m
            shifted_safe_edges = self._predict_blade_edge_positions(shifted_safe, target_rotation, geometry)
            shifted_descend_edges = self._predict_blade_edge_positions(shifted_descend, target_rotation, geometry)
            shifted_safe_refs = self._predict_blade_reference_positions(shifted_safe, target_rotation, geometry)
            shifted_descend_refs = self._predict_blade_reference_positions(shifted_descend, target_rotation, geometry)

            phase = ChoppingPhase.APPROACH if cycle == 0 else ChoppingPhase.SHIFT

            # --- Approach / Shift ---
            self.robot.set_cart_impedance_state(0.5, 0.5, K, D)
            current_tip, _ = self.robot.get_tcp_pose()
            approach_steps = _motion_steps(current_tip, shifted_safe, cfg.retract_speed_m_s, control_dt)
            self._move_tip_to(shifted_safe, target_rotation, phase, 0.0, control_dt, cfg,
                              approach_steps, terminal_K, terminal_D, shifted_safe_edges, shifted_safe_refs)

            # --- Descend ---
            self.robot.set_cart_impedance_state(0.5, 0.5, K, D)
            descend_steps = _motion_steps(shifted_safe, shifted_descend, cfg.descent_speed_m_s, control_dt)
            self._move_tip_to(shifted_descend, target_rotation, ChoppingPhase.DESCEND, 0.0, control_dt, cfg,
                              max(1, descend_steps), terminal_K, terminal_D,
                              shifted_descend_edges, shifted_descend_refs)

            # --- Force hold ---
            self.robot.set_force_state(0.5, 0.5, K, D, fx_dir, fc_adj_lmt=0.04)
            self.robot.set_force_cmd(cfg.target_force_n)
            T = np.eye(4)
            T[:3, :3] = target_rotation
            T[:3, 3] = shifted_descend
            self.robot._controller.set_cart_cmd(T)
            hold_steps = max(2, int(np.ceil(cfg.force_hold_s / control_dt)))
            for hold_index in range(hold_steps):
                force_progress = minimum_jerk_scalar((hold_index + 1) / hold_steps)
                force_target_n = cfg.target_force_n * force_progress
                self.robot.set_force_cmd(force_target_n)
                self.robot.step(viewer_sync=True)
                wrench = self.robot.get_wrench()
                self._record_sample(ChoppingPhase.FORCE_HOLD, "FORCE", shifted_descend,
                                    target_rotation, force_target_n, wrench,
                                    shifted_descend_edges, shifted_descend_refs)
                if not headless and self.robot._viewer is not None:
                    self.robot._viewer.sync()

            # --- Retract ---
            self.robot.set_cart_impedance_state(0.5, 0.5, K, D)
            retract_steps = _motion_steps(shifted_descend, shifted_safe, cfg.retract_speed_m_s, control_dt)
            self._move_tip_to(shifted_safe, target_rotation, ChoppingPhase.RETRACT, 0.0, control_dt, cfg,
                              max(1, retract_steps), terminal_K, terminal_D,
                              shifted_safe_edges, shifted_safe_refs)

        # Final sample
        self.samples.append(self._make_sample(
            self.robot.runtime.data.time, ChoppingPhase.COMPLETE, "IDLE",
            safe_tip, target_rotation, 0.0, np.zeros(6),
            target_blade_edge_positions=self._predict_blade_edge_positions(safe_tip, target_rotation, geometry),
            target_blade_reference_positions=self._predict_blade_reference_positions(safe_tip, target_rotation, geometry),
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
        """Write accumulated samples to a CSV file.

        Parameters
        ----------
        log_path : str or Path — output file path.  Parent directories are
            created if needed.
        """
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
        terminal_K: Sequence[float],
        terminal_D: Sequence[float],
        target_blade_edge_positions: tuple[tuple[float, float, float], ...],
        target_blade_reference_positions: tuple[tuple[float, float, float], ...],
    ) -> None:
        """Linearly interpolate the TCP from its current position to *target_pos*
        over *steps* control cycles, recording a sample at each step.

        Parameters
        ----------
        target_pos : (3,) ndarray — desired TCP position (m).
        rotation : (3,3) ndarray — desired TCP rotation matrix.
        phase : ChoppingPhase — phase label for recorded samples.
        target_force_n : float — force target for logging (0 during position moves).
        control_dt : float — control interval (s).
        cfg : ChoppingConfig — unused; reserved for future use.
        steps : int — number of control cycles for the move.
        """
        current_pos, _ = self.robot.get_tcp_pose()
        current_pos = np.asarray(current_pos, dtype=float).reshape(3)
        trajectory = cartesian_minimum_jerk_trajectory(
            start_pos=current_pos,
            target_pos=target_pos,
            rotation=rotation,
            steps=steps,
            phase=phase,
            start_time_s=self.robot.runtime.data.time,
            dt_s=control_dt,
            target_force_n=target_force_n,
        )
        T = np.eye(4)
        T[:3, :3] = rotation
        for point in trajectory:
            T = point.pose_matrix
            self.robot._controller.set_cart_cmd(T)
            self.robot.step(viewer_sync=True)
            wrench = self.robot.get_wrench()
            self._record_sample(phase, "CARTESIAN_IMPEDANCE", T[:3, 3], rotation, target_force_n, wrench,
                                target_blade_edge_positions, target_blade_reference_positions)
        self.robot.set_cart_impedance_state(0.5, 0.5, terminal_K, terminal_D)
        self.robot._controller.set_cart_cmd(T)
        self._settle_to_pose_tolerance(
            target_pos, rotation, phase, target_force_n, cfg, control_dt,
            target_blade_edge_positions, target_blade_reference_positions,
        )

    def _settle_to_pose_tolerance(
        self,
        target_pos: np.ndarray,
        target_rotation: np.ndarray,
        phase: ChoppingPhase,
        target_force_n: float,
        cfg: ChoppingConfig,
        control_dt: float,
        target_blade_edge_positions: tuple[tuple[float, float, float], ...],
        target_blade_reference_positions: tuple[tuple[float, float, float], ...],
    ) -> None:
        """Avoid direct IK state correction after Cartesian motion.

        The reference ``twin_mujoco`` chopper leaves the arm in the dynamically
        reached Cartesian posture.  Forcing an IK endpoint solution here moves
        the arm into a different null-space posture and makes the simulated
        chopping motion look like an awkward reconfiguration.
        """
        del target_pos, target_rotation, phase, target_force_n, cfg, control_dt
        del target_blade_edge_positions, target_blade_reference_positions
        return None

    def _record_sample(
        self,
        phase: ChoppingPhase,
        mode: str,
        target_pos: np.ndarray,
        target_rotation: np.ndarray,
        target_force_n: float,
        wrench: np.ndarray,
        target_blade_edge_positions: tuple[tuple[float, float, float], ...] | None = None,
        target_blade_reference_positions: tuple[tuple[float, float, float], ...] | None = None,
    ) -> None:
        """Convenience wrapper around ``_make_sample``."""
        self.samples.append(self._make_sample(
            self.robot.runtime.data.time, phase, mode,
            np.asarray(target_pos, dtype=float).reshape(3),
            np.asarray(target_rotation, dtype=float).reshape(3, 3),
            target_force_n,
            np.asarray(wrench, dtype=float).reshape(6),
            target_blade_edge_positions=target_blade_edge_positions,
            target_blade_reference_positions=target_blade_reference_positions,
        ))

    def _make_sample(
        self,
        t: float, phase: ChoppingPhase, mode: str,
        target_pos: np.ndarray, target_rotation: np.ndarray, target_force_n: float,
        wrench: np.ndarray, fault: str = "",
        target_blade_edge_positions: tuple[tuple[float, float, float], ...] | None = None,
        target_blade_reference_positions: tuple[tuple[float, float, float], ...] | None = None,
    ) -> ChoppingSample:
        """Build a ``ChoppingSample`` from the current robot state.

        Parameters
        ----------
        t : float — simulation time (s).
        phase : ChoppingPhase — current phase.
        mode : str — control mode label.
        target_pos : (3,) ndarray — commanded TCP position (m).
        target_force_n : float — commanded force (N).
        wrench : (6,) ndarray — compensated wrench (N, N·m).
        fault : str — fault description (empty if none).

        Returns
        -------
        ChoppingSample
        """
        q = self.robot.get_joint_positions()
        qd = self.robot.get_joint_velocities()
        actual_pos, actual_matrix = self.robot.get_tcp_pose()
        blade_geometry = self._blade_geometry()
        if target_blade_edge_positions is None:
            target_blade_edge_positions = self._predict_blade_edge_positions(
                target_pos, target_rotation, blade_geometry,
            )
        if target_blade_reference_positions is None:
            target_blade_reference_positions = self._predict_blade_reference_positions(
                target_pos, target_rotation, blade_geometry,
            )
        raw = self.robot.get_raw_wrench()
        return ChoppingSample(
            time_s=float(t),
            phase=phase,
            control_mode=mode,
            target_position=_triple(target_pos),
            actual_position=_triple(actual_pos),
            target_rotation=_matrix3(target_rotation),
            actual_rotation=_matrix3(actual_matrix[:3, :3]),
            target_force_n=float(target_force_n),
            measured_force_n=float(wrench[2]),
            raw_wrench=tuple(float(v) for v in raw),
            joint_positions=tuple(float(v) for v in q),
            joint_velocities=tuple(float(v) for v in qd),
            joint_torques=tuple(float(v) for v in self.robot._controller._previous_torque),
            contact=bool(abs(wrench[2]) > 1e-6),
            fault=fault,
            blade_edge_positions=blade_geometry.edge_positions,
            target_blade_edge_positions=target_blade_edge_positions,
            blade_reference_positions=(*blade_geometry.edge_positions, blade_geometry.tip_position),
            target_blade_reference_positions=target_blade_reference_positions,
        )

    # ------------------------------------------------------------------
    # Blade geometry (mirrored from reference implementation)
    # ------------------------------------------------------------------

    def _blade_geometry(self) -> BladeGeometry:
        """Capture the current blade pose from MuJoCo site positions.

        Returns
        -------
        BladeGeometry — edge positions, tip position, and tip rotation.
        """
        edge = []
        for name in BLADE_EDGE_SITE_NAMES:
            pos, _ = self.robot.runtime.site_pose(name)
            edge.append(_triple(pos))
        tip_pos, tip_rot = self.robot.runtime.site_pose("right_tool_tip_site")
        rot_rows = tuple(tuple(float(v) for v in row) for row in tip_rot)
        return BladeGeometry(tuple(edge), _triple(tip_pos), rot_rows)

    def _horizontal_blade_rotation(self, geometry: BladeGeometry) -> np.ndarray:
        """Compute a rotation matrix that keeps the blade horizontal (Z=0 in
        world) while preserving the blade's X-axis orientation as much as
        possible.

        Parameters
        ----------
        geometry : BladeGeometry — current blade pose.

        Returns
        -------
        (3, 3) ndarray — target rotation matrix with Z axis horizontal.
        """
        current_rot = np.asarray(geometry.tip_rotation, dtype=float).reshape(3, 3)
        # Project tool Z onto XY plane
        target_z = current_rot[:, 2].copy()
        target_z[2] = 0.0
        norm = float(np.linalg.norm(target_z))
        if norm < 1e-9:
            target_z = np.array((1.0, 0.0, 0.0))
        else:
            target_z /= norm
        # Orthogonalise X against new Z
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
        """Predict where blade reference points would be if the tip were moved
        to *tip_target* with orientation *target_rotation*.

        Uses the rigid-body assumption: local offsets in the current tip
        frame are re-applied in the target frame.

        Parameters
        ----------
        positions : sequence of (3,) — current world positions of reference points.
        tip_target : (3,) sequence — desired tip position.
        target_rotation : (3,3) ndarray — desired tip rotation.
        geometry : BladeGeometry — current blade pose.

        Returns
        -------
        tuple of (x,y,z) — predicted world positions.
        """
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

    def _predict_blade_edge_positions(
        self,
        tip_target: Sequence[float],
        target_rotation: np.ndarray,
        geometry: BladeGeometry,
    ) -> tuple[tuple[float, float, float], ...]:
        return self._predict_positions(geometry.edge_positions, tip_target, target_rotation, geometry)

    def _predict_blade_reference_positions(
        self,
        tip_target: Sequence[float],
        target_rotation: np.ndarray,
        geometry: BladeGeometry,
    ) -> tuple[tuple[float, float, float], ...]:
        return self._predict_positions(
            (*geometry.edge_positions, geometry.tip_position),
            tip_target,
            target_rotation,
            geometry,
        )

    def _tip_target_for_blade_clearance(
        self, tip: np.ndarray, rotation: np.ndarray,
        geometry: BladeGeometry, board_top: float, clearance_m: float,
    ) -> np.ndarray:
        """Adjust the tip Z so the lowest blade point is *clearance_m* above
        the board.

        Parameters
        ----------
        tip : (3,) ndarray — current tip position.
        rotation : (3,3) ndarray — desired tip rotation.
        geometry : BladeGeometry — current blade pose.
        board_top : float — Z coordinate of the board top surface.
        clearance_m : float — desired clearance (m).

        Returns
        -------
        (3,) ndarray — adjusted tip position.
        """
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
        """Adjust the tip Z so the highest blade point sits exactly on the board.

        Parameters
        ----------
        tip : (3,) ndarray — current tip position.
        rotation : (3,3) ndarray — desired tip rotation.
        geometry : BladeGeometry — current blade pose.
        board_top : float — Z coordinate of the board top surface.

        Returns
        -------
        (3,) ndarray — adjusted tip position.
        """
        target = tip.copy()
        pred = self._predict_positions(
            (*geometry.edge_positions, geometry.tip_position),
            target, rotation, geometry,
        )
        max_z = max(float(p[2]) for p in pred)
        target[2] += float(board_top) - max_z
        return target

    def _board_top(self) -> float:
        """Return the Z coordinate of the chopping board's top surface.

        Returns
        -------
        float — board top Z in world frame (m).
        """
        geom_id = int(mujoco.mj_name2id(
            self.robot.runtime.model, mujoco.mjtObj.mjOBJ_GEOM, "chopping_board",
        ))
        return float(
            self.robot.runtime.data.geom_xpos[geom_id, 2]
            + self.robot.runtime.model.geom_size[geom_id, 2],
        )

    def _board_top_from_model_path(self) -> float:
        """Read the chopping-board top height without requiring an active run."""
        runtime = self.robot.runtime
        if runtime is None:
            from twin_mujoco.runtime import TwinMujocoRuntime

            runtime = TwinMujocoRuntime.load(str(right_chopping_scene_path()))
        geom_id = int(mujoco.mj_name2id(runtime.model, mujoco.mjtObj.mjOBJ_GEOM, "chopping_board"))
        return float(runtime.data.geom_xpos[geom_id, 2] + runtime.model.geom_size[geom_id, 2])


# ---------------------------------------------------------------------------
# Module-level helpers
# ---------------------------------------------------------------------------

def _validate_config(cfg: ChoppingConfig) -> None:
    """Raise ``ValueError`` if any config field is invalid."""
    if cfg.cycles < 1:
        raise ValueError("cycles must be at least 1")
    if cfg.control_hz <= 0.0:
        raise ValueError("control_hz must be positive")
    if cfg.descent_speed_m_s <= 0.0:
        raise ValueError("descent_speed_m_s must be positive")
    if cfg.retract_speed_m_s <= 0.0:
        raise ValueError("retract_speed_m_s must be positive")
    if cfg.position_tolerance_m <= 0.0:
        raise ValueError("position_tolerance_m must be positive")
    if cfg.orientation_tolerance_rad <= 0.0:
        raise ValueError("orientation_tolerance_rad must be positive")
    if cfg.settle_timeout_s < 0.0:
        raise ValueError("settle_timeout_s must be non-negative")


def _motion_steps(start: np.ndarray, end: np.ndarray, speed_m_s: float, dt: float) -> int:
    """Return the number of control steps needed to move from *start* to *end*
    at the given Cartesian speed.

    Parameters
    ----------
    start : (3,) ndarray — start position (m).
    end : (3,) ndarray — end position (m).
    speed_m_s : float — desired Cartesian speed (m/s).
    dt : float — control interval (s).

    Returns
    -------
    int — number of steps (≥ 1).
    """
    d = float(np.linalg.norm(np.asarray(end).reshape(3) - np.asarray(start).reshape(3)))
    return max(1, int(np.ceil(d / speed_m_s / dt)))


def _triple(v: np.ndarray | Sequence[float]) -> tuple[float, float, float]:
    """Convert a 3-element array-like to a (float, float, float) tuple."""
    a = np.asarray(v, dtype=float).reshape(3)
    return tuple(float(x) for x in a)


def _matrix3(v: np.ndarray | Sequence[Sequence[float]]) -> tuple[tuple[float, float, float], ...]:
    """Convert a 3×3 array-like to nested float tuples."""
    a = np.asarray(v, dtype=float).reshape(3, 3)
    return tuple(tuple(float(x) for x in row) for row in a)


def _sample_row(s: ChoppingSample) -> dict[str, object]:
    """Convert a ``ChoppingSample`` to a flat dict for CSV writing.

    Parameters
    ----------
    s : ChoppingSample

    Returns
    -------
    dict — keys matching ``_CSV_FIELDS``.
    """
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
