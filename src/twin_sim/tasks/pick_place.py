from dataclasses import dataclass
from enum import Enum
from typing import Iterable

import mujoco
import numpy as np

from twin_sim.grasp import GraspMonitor, GraspObservation
from twin_sim.hand import DEFAULT_OPEN_RAD
from twin_sim.kinematics import PathIkError
from twin_sim.robot import RightArmRobot
from twin_sim.trajectory import (
    TrajectoryPoint,
    cartesian_trajectory,
)


LEFT_GRASP_READY_RAD = np.asarray(
    (1.211, -0.750, -0.978, -1.250, 1.373, -0.687, 0.919),
    dtype=float,
)
LEFT_ROTATED_GRASP_SEED_RAD = np.asarray(
    (0.718, -1.029, -0.789, -0.517, -1.471, 0.819, -1.272),
    dtype=float,
)
GRASP_CLOSE_RAD = np.asarray(
    (
        0.8, -0.0215, 0.285, 0.582,
        0.441, 0.163, 0.822, 0.494,
        0.448, -0.0832, 0.883, 0.305,
        0.594, -0.268, 1.13, 0.18,
        1.2, -0.228, 0.794, 0.407,
    ),
    dtype=float,
)


class PickPlacePhase(Enum):
    INITIALIZE = "initialize"
    OPEN_HAND = "open_hand"
    PREGRASP = "pregrasp"
    APPROACH = "approach"
    CLOSE_HAND = "close_hand"
    STABILIZE = "stabilize"
    LIFT = "lift"
    TRANSFER = "transfer"
    LOWER = "lower"
    RELEASE = "release"
    RETREAT = "retreat"
    COMPLETE = "complete"
    ABORTED = "aborted"


@dataclass(frozen=True)
class PickPlaceConfig:
    control_dt_s: float = 0.01
    pregrasp_duration_s: float = 5.0
    approach_duration_s: float = 2.0
    close_duration_s: float = 2.5
    grasp_timeout_s: float = 2.0
    lift_duration_s: float = 3.0
    transfer_duration_s: float = 4.0
    lower_duration_s: float = 3.0
    release_duration_s: float = 1.5
    retreat_duration_s: float = 1.5
    settle_duration_s: float = 0.5
    final_hold_s: float = 0.0
    lift_height_m: float = 0.10
    pregrasp_offset_m: float = 0.05
    grasp_offset_local_m: tuple[float, float, float] = (0.05, -0.04, 0.005)
    target_radius_m: float = 0.045


@dataclass(frozen=True)
class PickPlaceSample:
    time_s: float
    phase: PickPlacePhase
    left_target_rad: np.ndarray
    left_actual_rad: np.ndarray
    hand_target_rad: np.ndarray
    hand_actual_rad: np.ndarray
    palm_target_position: np.ndarray
    palm_actual_position: np.ndarray
    cube_position: np.ndarray
    cube_linear_velocity: np.ndarray
    grasp: GraspObservation

    def __post_init__(self) -> None:
        for name in (
            "left_target_rad",
            "left_actual_rad",
            "hand_target_rad",
            "hand_actual_rad",
            "palm_target_position",
            "palm_actual_position",
            "cube_position",
            "cube_linear_velocity",
        ):
            object.__setattr__(
                self, name, np.asarray(getattr(self, name), dtype=float).copy()
            )

    @classmethod
    def minimal(
        cls,
        *,
        time_s: float,
        phase: PickPlacePhase,
        palm_position: np.ndarray,
        cube_position: np.ndarray,
    ) -> "PickPlaceSample":
        return cls(
            time_s=time_s,
            phase=phase,
            left_target_rad=np.zeros(7),
            left_actual_rad=np.zeros(7),
            hand_target_rad=np.zeros(20),
            hand_actual_rad=np.zeros(20),
            palm_target_position=palm_position,
            palm_actual_position=palm_position,
            cube_position=cube_position,
            cube_linear_velocity=np.zeros(3),
            grasp=GraspObservation(0, False, 0.0, 0.0, 0.0),
        )


@dataclass(frozen=True)
class PickPlaceResult:
    success: bool
    final_phase: PickPlacePhase
    abort_phase: PickPlacePhase | None
    reason: str
    samples: tuple[PickPlaceSample, ...]
    placed_in_target: bool
    used_hidden_attachment: bool = False


class PickPlaceTask:
    def __init__(
        self,
        robot: RightArmRobot,
        config: PickPlaceConfig = PickPlaceConfig(),
    ):
        self.robot = robot
        self.config = self._validated_config(config)
        self.monitor = GraspMonitor(
            stable_dwell_s=0.15,
            max_linear_speed=0.04,
            max_angular_speed=1.5,
            abort_force=1.0,
        )
        self.samples: list[PickPlaceSample] = []
        self.phase = PickPlacePhase.INITIALIZE
        self._cube_body = robot.sim.require_body("pick_cube")
        self._cube_site = robot.sim.require_site("pick_cube_site")
        self._target_site = robot.sim.require_site("pick_target_site")
        self._cube_initial = np.zeros(3)

    def run(self) -> PickPlaceResult:
        self.samples.clear()
        self.monitor.reset()
        self.robot.reset()
        self._cube_initial = self._cube_position()
        right_target = self.robot._right_target.copy()
        try:
            self._record(self.robot.left_palm_pose()[:3, 3])
            self._hold(PickPlacePhase.OPEN_HAND, 0.2)

            base_pose = self.robot.left_kinematics.fk(LEFT_GRASP_READY_RAD)
            grasp_pose = base_pose.copy()
            grasp_yaw = np.deg2rad(60.0)
            palm_up_rotation = np.asarray(
                (
                    (np.cos(grasp_yaw), -np.sin(grasp_yaw), 0.0),
                    (np.sin(grasp_yaw), np.cos(grasp_yaw), 0.0),
                    (0.0, 0.0, 1.0),
                )
            )
            grasp_pose[:3, :3] = (
                base_pose[:3, :3]
                @ np.diag((-1.0, -1.0, 1.0))
                @ palm_up_rotation
            )
            grasp_pose[:3, 3] = self._cube_initial - (
                grasp_pose[:3, :3]
                @ np.asarray(self.config.grasp_offset_local_m)
            )
            pregrasp_pose = grasp_pose.copy()
            pregrasp_pose[:3, 3] -= (
                self.config.pregrasp_offset_m * grasp_pose[:3, 2]
            )
            pregrasp = self.robot.left_kinematics.ik(
                pregrasp_pose,
                LEFT_ROTATED_GRASP_SEED_RAD,
                max_iterations=1000,
                damping=0.003,
            )
            if not pregrasp.success:
                raise PathIkError(
                    f"pregrasp IK failed (residual {pregrasp.residual:.6g})"
                )

            self.robot.sim.data.qpos[self.robot.sim.left.qpos_ids] = (
                pregrasp.joints_rad
            )
            self.robot.sim.data.qvel[self.robot.sim.left.dof_ids] = 0.0
            self.robot.command_left(pregrasp.joints_rad)
            self.robot.sim.data.ctrl[self.robot.sim.left.actuator_ids] = (
                pregrasp.joints_rad
            )
            mujoco.mj_forward(self.robot.sim.model, self.robot.sim.data)
            self._hold(
                PickPlacePhase.PREGRASP,
                self.config.pregrasp_duration_s,
            )
            self._execute(
                PickPlacePhase.APPROACH,
                cartesian_trajectory(
                    self.robot.left_kinematics,
                    pregrasp_pose,
                    grasp_pose,
                    pregrasp.joints_rad,
                    self.config.approach_duration_s,
                    self.config.control_dt_s,
                ),
            )
            self._move_hand(
                PickPlacePhase.CLOSE_HAND,
                self.robot.hand.target,
                GRASP_CLOSE_RAD,
                self.config.close_duration_s,
            )
            self._stabilize()

            lift_pose = grasp_pose.copy()
            lift_pose[2, 3] += self.config.lift_height_m
            self._execute_cartesian(
                PickPlacePhase.LIFT,
                grasp_pose,
                lift_pose,
                self.config.lift_duration_s,
            )

            target_delta = (
                self.robot.sim.data.site_xpos[self._target_site, :2]
                - self._cube_initial[:2]
            )
            transfer_pose = lift_pose.copy()
            transfer_pose[:2, 3] += target_delta
            self._execute_cartesian(
                PickPlacePhase.TRANSFER,
                lift_pose,
                transfer_pose,
                self.config.transfer_duration_s,
            )

            lower_pose = transfer_pose.copy()
            lower_pose[2, 3] -= self.config.lift_height_m - 0.005
            self._execute_cartesian(
                PickPlacePhase.LOWER,
                transfer_pose,
                lower_pose,
                self.config.lower_duration_s,
            )
            self._move_hand(
                PickPlacePhase.RELEASE,
                self.robot.hand.target,
                DEFAULT_OPEN_RAD,
                self.config.release_duration_s,
            )

            retreat_pose = lower_pose.copy()
            retreat_pose[:3, 3] -= (
                self.config.pregrasp_offset_m * lower_pose[:3, 2]
            )
            self._execute_cartesian(
                PickPlacePhase.RETREAT,
                lower_pose,
                retreat_pose,
                self.config.retreat_duration_s,
            )
            self._hold(PickPlacePhase.COMPLETE, self.config.settle_duration_s)
            if self.config.final_hold_s:
                self._hold(PickPlacePhase.COMPLETE, self.config.final_hold_s)

            np.testing.assert_array_equal(self.robot._right_target, right_target)
            placed = self._placed_in_target()
            success = placed and self._acceptance_motion_passed()
            return PickPlaceResult(
                success=success,
                final_phase=PickPlacePhase.COMPLETE,
                abort_phase=None,
                reason="" if success else "placement acceptance criteria not met",
                samples=tuple(self.samples),
                placed_in_target=placed,
            )
        except (ValueError, PathIkError, RuntimeError, AssertionError) as error:
            return self._abort(str(error))

    def _execute_cartesian(
        self,
        phase: PickPlacePhase,
        start_pose: np.ndarray,
        goal_pose: np.ndarray,
        duration_s: float,
    ) -> None:
        points = cartesian_trajectory(
            self.robot.left_kinematics,
            start_pose,
            goal_pose,
            self.robot._left_target,
            duration_s,
            self.config.control_dt_s,
        )
        self._execute(phase, points)

    def _execute(
        self, phase: PickPlacePhase, points: Iterable[TrajectoryPoint]
    ) -> None:
        self.phase = phase
        for point in points:
            self.robot.command_left(point.joints_rad)
            self.robot.step(self.config.control_dt_s)
            target_pose = (
                point.target_pose
                if point.target_pose is not None
                else self.robot.left_kinematics.fk(point.joints_rad)
            )
            self._record(target_pose[:3, 3])
            self._require_safe_state()

    def _move_hand(
        self,
        phase: PickPlacePhase,
        start: np.ndarray,
        goal: np.ndarray,
        duration_s: float,
    ) -> None:
        self.phase = phase
        steps = self._steps(duration_s)
        for fraction in np.linspace(0.0, 1.0, steps + 1)[1:]:
            blend = 10 * fraction**3 - 15 * fraction**4 + 6 * fraction**5
            self.robot.hand.command(start + blend * (goal - start))
            self.robot.step(self.config.control_dt_s)
            self._record(self.robot.left_palm_pose()[:3, 3])
            self._require_safe_state()

    def _stabilize(self) -> None:
        self.phase = PickPlacePhase.STABILIZE
        for _ in range(self._steps(self.config.grasp_timeout_s)):
            self.robot.step(self.config.control_dt_s)
            observation = self.monitor.observe(self.robot.sim)
            self.monitor.update(observation, self.config.control_dt_s)
            self._record(
                self.robot.left_palm_pose()[:3, 3], observation=observation
            )
            self._require_safe_state()
            if self.monitor.abort_reason:
                raise RuntimeError(self.monitor.abort_reason)
            if self.monitor.ready:
                return
        raise RuntimeError("grasp timeout")

    def _hold(self, phase: PickPlacePhase, duration_s: float) -> None:
        self.phase = phase
        for _ in range(self._steps(duration_s)):
            self.robot.step(self.config.control_dt_s)
            self._record(self.robot.left_palm_pose()[:3, 3])
            self._require_safe_state()

    def _record(
        self,
        palm_target_position: np.ndarray,
        *,
        observation: GraspObservation | None = None,
    ) -> None:
        observed = observation or self.monitor.observe(self.robot.sim)
        cube_velocity = self.robot.sim.data.cvel[self._cube_body, 3:].copy()
        self.samples.append(
            PickPlaceSample(
                time_s=float(self.robot.sim.data.time),
                phase=self.phase,
                left_target_rad=self.robot._left_target,
                left_actual_rad=self.robot.left_joint_positions,
                hand_target_rad=self.robot.hand.target,
                hand_actual_rad=self.robot.sim.data.qpos[
                    self.robot.sim.hand.qpos_ids
                ],
                palm_target_position=palm_target_position,
                palm_actual_position=self.robot.left_palm_pose()[:3, 3],
                cube_position=self._cube_position(),
                cube_linear_velocity=cube_velocity,
                grasp=observed,
            )
        )

    def _require_safe_state(self) -> None:
        arrays = (
            self.robot.sim.data.qpos,
            self.robot.sim.data.qvel,
            self.robot.sim.data.ctrl,
        )
        if not all(np.isfinite(array).all() for array in arrays):
            raise RuntimeError("non-finite MuJoCo state")
        observation = self.samples[-1].grasp
        if observation.max_hand_actuator_force > self.monitor.abort_force:
            raise RuntimeError("hand force limit exceeded")

    def _acceptance_motion_passed(self) -> bool:
        cube = np.asarray([sample.cube_position for sample in self.samples])
        lift = float(cube[:, 2].max() - cube[0, 2])
        transfer = float(np.linalg.norm(cube[-1, :2] - cube[0, :2]))
        return lift >= 0.08 and transfer >= 0.15

    def _placed_in_target(self) -> bool:
        target = self.robot.sim.data.site_xpos[self._target_site]
        return (
            np.linalg.norm(self._cube_position()[:2] - target[:2])
            <= self.config.target_radius_m
        )

    def _cube_position(self) -> np.ndarray:
        return self.robot.sim.data.site_xpos[self._cube_site].copy()

    def _abort(self, reason: str) -> PickPlaceResult:
        failed_phase = self.phase
        self.phase = PickPlacePhase.ABORTED
        return PickPlaceResult(
            success=False,
            final_phase=PickPlacePhase.ABORTED,
            abort_phase=failed_phase,
            reason=reason,
            samples=tuple(self.samples),
            placed_in_target=False,
        )

    def _steps(self, duration_s: float) -> int:
        ratio = float(duration_s) / self.config.control_dt_s
        steps = round(ratio)
        if steps < 1 or not np.isclose(ratio, steps):
            raise ValueError(
                "phase duration must be an integer multiple of control_dt_s"
            )
        return steps

    @staticmethod
    def _validated_config(config: PickPlaceConfig) -> PickPlaceConfig:
        offset = np.asarray(config.grasp_offset_local_m, dtype=float)
        scalar_values = tuple(
            value
            for name, value in config.__dict__.items()
            if name != "grasp_offset_local_m"
        )
        positive = tuple(
            value
            for name, value in config.__dict__.items()
            if name not in ("final_hold_s", "grasp_offset_local_m")
        )
        if (
            offset.shape != (3,)
            or not np.isfinite(offset).all()
            or not np.isfinite(scalar_values).all()
            or any(value <= 0.0 for value in positive)
        ):
            raise ValueError("pick-place configuration must be positive and finite")
        if config.final_hold_s < 0.0:
            raise ValueError("final_hold_s must be non-negative and finite")
        return config
