from dataclasses import dataclass
from enum import Enum
from typing import Sequence

import mujoco
import numpy as np

from twin_sim.guarded_chop_safety import CAT_PAW_RAD, SafetyCoordinator
from twin_sim.kinematics import PathIkError
from twin_sim.robot import RightArmRobot
from twin_sim.tasks.chop import ChopConfig
from twin_sim.tasks.line_chop import (
    LineChopConfig,
    _CutTrajectories,
    _preflight_line_chop,
)
from twin_sim.tasks.pick_place import LEFT_GRASP_READY_RAD
from twin_sim.trajectory import TrajectoryPoint, cartesian_trajectory


class GuardedChopPhase(Enum):
    INITIALIZE = "initialize"
    GUARD_READY = "guard_ready"
    CUT_DOWN = "cut_down"
    KNIFE_UP = "knife_up"
    HAND_SHIFT = "hand_shift"
    COMPLETE = "complete"
    ABORTED = "aborted"


@dataclass(frozen=True)
class GuardedChopConfig:
    cuts: int = 5
    hand_shift_m: float = 0.02
    minimum_distance_m: float = 0.02
    control_dt_s: float = 0.01
    guard_ready_duration_s: float = 2.0
    cut_duration_s: float = 1.0
    knife_up_duration_s: float = 1.0
    hand_shift_duration_s: float = 0.8
    final_hold_s: float = 10.0

    def validated(self) -> "GuardedChopConfig":
        if (
            isinstance(self.cuts, bool)
            or not isinstance(self.cuts, (int, np.integer))
            or self.cuts != 5
        ):
            raise ValueError("cuts must equal 5 for guarded chopping")
        if not np.isclose(self.hand_shift_m, 0.02, rtol=0.0, atol=1e-12):
            raise ValueError("hand_shift_m must equal 0.02")
        if not np.isclose(
            self.minimum_distance_m, 0.02, rtol=0.0, atol=1e-12
        ):
            raise ValueError("minimum_distance_m must equal 0.02")
        durations = (
            self.control_dt_s,
            self.guard_ready_duration_s,
            self.cut_duration_s,
            self.knife_up_duration_s,
            self.hand_shift_duration_s,
        )
        if not all(np.isfinite(value) and value > 0.0 for value in durations):
            raise ValueError("control durations must be positive and finite")
        if not np.isfinite(self.final_hold_s) or self.final_hold_s < 0.0:
            raise ValueError("final_hold_s must be non-negative and finite")
        for duration in durations[1:] + (self.final_hold_s,):
            ratio = duration / self.control_dt_s
            if not np.isclose(ratio, round(ratio), rtol=0.0, atol=1e-9):
                raise ValueError(
                    "durations must be integer multiples of control_dt_s"
                )
        return self


@dataclass(frozen=True)
class GuardedChopSample:
    time_s: float
    phase: GuardedChopPhase
    cut_index: int
    left_target_rad: np.ndarray
    left_actual_rad: np.ndarray
    right_target_rad: np.ndarray
    right_actual_rad: np.ndarray
    hand_target_rad: np.ndarray
    knife_position: np.ndarray
    guard_position: np.ndarray
    knife_guard_distance_m: float
    knife_height_m: float
    cut_allowed: bool


@dataclass(frozen=True)
class GuardedChopResult:
    success: bool
    final_phase: GuardedChopPhase
    reason: str
    samples: tuple[GuardedChopSample, ...]
    completed_cuts: int
    completed_shifts: int
    total_shift_m: float
    minimum_distance_m: float


@dataclass(frozen=True)
class _GuardedPreflight:
    right_ready_rad: np.ndarray
    left_ready_rad: np.ndarray
    cuts: tuple[_CutTrajectories, ...]
    guard_shifts: tuple[tuple[TrajectoryPoint, ...], ...]
    guard_targets: np.ndarray
    minimum_planned_distances_m: np.ndarray
    safe_knife_height_m: float


def _point_to_box_distance(
    point: Sequence[float],
    center: Sequence[float],
    rotation: np.ndarray,
    half_size: Sequence[float],
) -> float:
    point_array = np.asarray(point, dtype=float)
    center_array = np.asarray(center, dtype=float)
    rotation_array = np.asarray(rotation, dtype=float)
    size_array = np.asarray(half_size, dtype=float)
    if (
        point_array.shape != (3,)
        or center_array.shape != (3,)
        or rotation_array.shape != (3, 3)
        or size_array.shape != (3,)
        or not all(
            np.isfinite(value).all()
            for value in (
                point_array,
                center_array,
                rotation_array,
                size_array,
            )
        )
        or np.any(size_array < 0.0)
    ):
        raise ValueError("box distance inputs must be finite and correctly shaped")
    local = rotation_array.T @ (point_array - center_array)
    closest = np.clip(local, -size_array, size_array)
    return float(np.linalg.norm(local - closest))


def _preflight_guarded_chop(
    robot: RightArmRobot,
    config: GuardedChopConfig,
) -> _GuardedPreflight:
    config = config.validated()
    saved = mujoco.MjData(robot.sim.model)
    mujoco.mj_copyData(saved, robot.sim.model, robot.sim.data)
    try:
        line = _preflight_line_chop(
            robot,
            robot.right_kinematics,
            LineChopConfig(
                chop=ChopConfig(
                    control_dt_s=config.control_dt_s,
                    descent_duration_s=config.cut_duration_s,
                    retract_duration_s=config.knife_up_duration_s,
                ),
                cuts=config.cuts,
                spacing_m=config.hand_shift_m,
                shift_duration_s=config.hand_shift_duration_s,
            ),
        )
        left_ready, guard_targets, guard_shifts = _guard_plan(
            robot, line.cut_points_xy, config
        )
        distances = _planned_clearances(
            robot, line.cuts, guard_targets, guard_shifts, left_ready
        )
        if np.any(distances < config.minimum_distance_m):
            raise ValueError(
                "guarded-chop plan violates knife-guard clearance: "
                f"{float(np.min(distances)):.6f} m"
            )
        safe_height = float(
            line.initial_safe.target_pose[2, 3]
        )
        return _GuardedPreflight(
            right_ready_rad=line.initial_safe.joints_rad.copy(),
            left_ready_rad=left_ready.copy(),
            cuts=line.cuts,
            guard_shifts=guard_shifts,
            guard_targets=guard_targets,
            minimum_planned_distances_m=distances,
            safe_knife_height_m=safe_height,
        )
    finally:
        mujoco.mj_copyData(robot.sim.data, robot.sim.model, saved)


def _guard_plan(
    robot: RightArmRobot,
    cut_points_xy: np.ndarray,
    config: GuardedChopConfig,
) -> tuple[np.ndarray, np.ndarray, tuple[tuple[TrajectoryPoint, ...], ...]]:
    model, data = robot.sim.model, robot.sim.data
    knuckle = robot.sim.require_site("left_guard_knuckle_site")
    data.qpos[robot.sim.left.qpos_ids] = LEFT_GRASP_READY_RAD
    data.qpos[robot.sim.hand.qpos_ids] = CAT_PAW_RAD
    mujoco.mj_forward(model, data)
    palm_pose = robot.left_kinematics.fk(LEFT_GRASP_READY_RAD)
    knuckle_local = palm_pose[:3, :3].T @ (
        data.site_xpos[knuckle] - palm_pose[:3, 3]
    )

    direction = cut_points_xy[1] - cut_points_xy[0]
    direction /= np.linalg.norm(direction)
    desired_knuckle = np.asarray(
        (cut_points_xy[0, 0], cut_points_xy[0, 1], 0.35), dtype=float
    )
    desired_knuckle[:2] -= (
        config.minimum_distance_m + 0.002
    ) * direction
    ready_pose = palm_pose.copy()
    ready_pose[:3, 3] = (
        desired_knuckle - ready_pose[:3, :3] @ knuckle_local
    )
    result = robot.left_kinematics.ik(
        ready_pose,
        LEFT_GRASP_READY_RAD,
        max_iterations=1000,
        damping=0.003,
    )
    if not result.success:
        raise PathIkError(
            f"guard-ready IK failed (residual {result.residual:.6g})"
        )

    targets = []
    for index in range(config.cuts):
        pose = ready_pose.copy()
        pose[:2, 3] += index * config.hand_shift_m * direction
        targets.append(pose)
    guard_targets = np.asarray(targets)
    shifts = []
    seed = result.joints_rad
    for start, goal in zip(
        guard_targets[:-1], guard_targets[1:], strict=True
    ):
        points = cartesian_trajectory(
            robot.left_kinematics,
            start,
            goal,
            seed,
            config.hand_shift_duration_s,
            config.control_dt_s,
        )
        shifts.append(tuple(points))
        seed = points[-1].joints_rad
    return result.joints_rad, guard_targets, tuple(shifts)


def _planned_clearances(
    robot: RightArmRobot,
    cuts: tuple[_CutTrajectories, ...],
    guard_targets: np.ndarray,
    guard_shifts: tuple[tuple[TrajectoryPoint, ...], ...],
    left_ready_rad: np.ndarray,
) -> np.ndarray:
    model, data = robot.sim.model, robot.sim.data
    blade = robot.sim.require_geom("right_knife_blade")
    knuckle = robot.sim.require_site("left_guard_knuckle_site")
    distances = []

    def measure(right: np.ndarray, left: np.ndarray) -> None:
        data.qpos[robot.sim.right.qpos_ids] = right
        data.qpos[robot.sim.left.qpos_ids] = left
        data.qpos[robot.sim.hand.qpos_ids] = CAT_PAW_RAD
        mujoco.mj_forward(model, data)
        distances.append(
            _point_to_box_distance(
                data.site_xpos[knuckle],
                data.geom_xpos[blade],
                data.geom_xmat[blade].reshape(3, 3),
                model.geom_size[blade],
            )
        )

    left = left_ready_rad
    for index, cut in enumerate(cuts):
        for point in (*cut.descent, *cut.retract, *cut.shift):
            measure(point.joints_rad, left)
        if index < len(guard_shifts):
            right = cut.shift[-1].joints_rad
            for point in guard_shifts[index]:
                measure(right, point.joints_rad)
            left = guard_shifts[index][-1].joints_rad
    return np.asarray(distances)


def run_guarded_chop(
    config: GuardedChopConfig = GuardedChopConfig(),
    *,
    viewer: bool = False,
    trace=None,
    coordinator: SafetyCoordinator | None = None,
) -> GuardedChopResult:
    raise NotImplementedError("guarded chopping execution is implemented in Task 4")
