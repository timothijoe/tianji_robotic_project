from dataclasses import dataclass
from enum import Enum
from typing import Sequence

import mujoco
import numpy as np

from twin_sim.guarded_chop_safety import (
    CAT_PAW_RAD,
    SafetyCoordinator,
    SafetyObservation,
)
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
    maximum_guard_penetration_m: float = 0.003
    maximum_guard_force_n: float = 35.0
    control_dt_s: float = 0.01
    guard_ready_duration_s: float = 2.0
    cut_duration_s: float = 1.0
    knife_up_duration_s: float = 1.0
    hand_shift_duration_s: float = 3.0
    interlock_settle_s: float = 0.3
    stability_timeout_s: float = 4.0
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
        if (
            not np.isfinite(self.maximum_guard_penetration_m)
            or self.maximum_guard_penetration_m <= 0.0
            or not np.isfinite(self.maximum_guard_force_n)
            or self.maximum_guard_force_n <= 0.0
        ):
            raise ValueError("guard contact limits must be positive and finite")
        durations = (
            self.control_dt_s,
            self.guard_ready_duration_s,
            self.cut_duration_s,
            self.knife_up_duration_s,
            self.hand_shift_duration_s,
            self.interlock_settle_s,
            self.stability_timeout_s,
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
    safety_allowed: bool
    guard_cube_contact_count: int
    guard_cube_penetration_m: float
    guard_cube_normal_force_n: float
    blade_hand_contact_count: int
    left_speed_rad_s: float
    right_speed_rad_s: float
    hand_speed_rad_s: float

    def __post_init__(self) -> None:
        for name in (
            "left_target_rad",
            "left_actual_rad",
            "right_target_rad",
            "right_actual_rad",
            "hand_target_rad",
            "knife_position",
            "guard_position",
        ):
            object.__setattr__(
                self, name, np.asarray(getattr(self, name), dtype=float).copy()
            )


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
        board = robot.sim.require_geom("chopping_board")
        cube = robot.sim.require_geom("guarded_chop_cube")
        board_top = float(
            robot.sim.data.geom_xpos[board, 2]
            + robot.sim.model.geom_size[board, 2]
        )
        cube_top = float(
            robot.sim.data.geom_xpos[cube, 2]
            + robot.sim.model.geom_size[cube, 2]
        )
        cuts = _translate_right_cuts(
            robot,
            line.cuts,
            cube_top - board_top + 0.008,
            config,
        )
        left_ready, guard_targets, guard_shifts = _guard_plan(
            robot, line.cut_points_xy, config
        )
        distances = _planned_clearances(
            robot, cuts, guard_targets, guard_shifts, left_ready
        )
        if np.any(distances < config.minimum_distance_m):
            raise ValueError(
                "guarded-chop plan violates knife-guard clearance: "
                f"{float(np.min(distances)):.6f} m"
            )
        right_ready = cuts[0].descent[0].joints_rad
        safe_height = _blade_bottom_height(robot, right_ready)
        return _GuardedPreflight(
            right_ready_rad=right_ready.copy(),
            left_ready_rad=left_ready.copy(),
            cuts=cuts,
            guard_shifts=guard_shifts,
            guard_targets=guard_targets,
            minimum_planned_distances_m=distances,
            safe_knife_height_m=safe_height,
        )
    finally:
        mujoco.mj_copyData(robot.sim.data, robot.sim.model, saved)


def _translate_right_cuts(
    robot: RightArmRobot,
    cuts: tuple[_CutTrajectories, ...],
    dz_m: float,
    config: GuardedChopConfig,
) -> tuple[_CutTrajectories, ...]:
    translated = []
    seed = cuts[0].descent[0].joints_rad
    for cut in cuts:
        safe = cut.descent[0].target_pose.copy()
        contact = cut.descent[-1].target_pose.copy()
        safe[2, 3] += dz_m
        contact[2, 3] += dz_m
        safe_result = robot.right_kinematics.ik(
            safe, seed, max_iterations=1000, damping=0.003
        )
        if not safe_result.success:
            raise PathIkError(
                "raised knife-safe IK failed "
                f"(residual {safe_result.residual:.6g})"
            )
        descent = cartesian_trajectory(
            robot.right_kinematics,
            safe,
            contact,
            safe_result.joints_rad,
            config.cut_duration_s,
            config.control_dt_s,
        )
        retract = cartesian_trajectory(
            robot.right_kinematics,
            contact,
            safe,
            descent[-1].joints_rad,
            config.knife_up_duration_s,
            config.control_dt_s,
        )
        shift = ()
        seed = retract[-1].joints_rad
        if cut.shift:
            next_safe = cut.shift[-1].target_pose.copy()
            next_safe[2, 3] += dz_m
            shift = tuple(
                cartesian_trajectory(
                    robot.right_kinematics,
                    safe,
                    next_safe,
                    seed,
                    config.hand_shift_duration_s,
                    config.control_dt_s,
                )
            )
            seed = shift[-1].joints_rad
        translated.append(
            _CutTrajectories(tuple(descent), tuple(retract), shift)
        )
    return tuple(translated)


def _blade_bottom_height(robot: RightArmRobot, joints_rad: np.ndarray) -> float:
    blade = robot.sim.require_geom("right_knife_blade")
    with robot.right_kinematics._configuration(joints_rad):
        rotation = robot.sim.data.geom_xmat[blade].reshape(3, 3)
        radius = float(
            np.abs(rotation[2]) @ robot.sim.model.geom_size[blade]
        )
        return float(robot.sim.data.geom_xpos[blade, 2] - radius)


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
        (cut_points_xy[0, 0], cut_points_xy[0, 1], 0.367), dtype=float
    )
    desired_knuckle[:2] -= (
        config.minimum_distance_m + 0.080
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
    distances = []

    def measure(right: np.ndarray, left: np.ndarray) -> None:
        data.qpos[robot.sim.right.qpos_ids] = right
        data.qpos[robot.sim.left.qpos_ids] = left
        data.qpos[robot.sim.hand.qpos_ids] = CAT_PAW_RAD
        mujoco.mj_forward(model, data)
        distances.append(_blade_hand_distance(robot))

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
    config = config.validated()
    safety = coordinator or SafetyCoordinator(
        config.minimum_distance_m,
        config.maximum_guard_penetration_m,
        config.maximum_guard_force_n,
    )
    robot = RightArmRobot(viewer=viewer)
    if trace is None and viewer:
        from twin_sim.guarded_chop_visualization import GuardedChopTrace

        trace = GuardedChopTrace(robot._viewer)
    samples: list[GuardedChopSample] = []
    phase = GuardedChopPhase.INITIALIZE
    completed_cuts = 0
    completed_shifts = 0
    try:
        plan = _preflight_guarded_chop(robot, config)
        _activate_guarded_scene(robot)
        robot.reset(plan.right_ready_rad)
        robot.sim.data.qpos[robot.sim.left.qpos_ids] = plan.left_ready_rad
        robot.sim.data.qvel[robot.sim.left.dof_ids] = 0.0
        robot.command_left(plan.left_ready_rad)
        robot.sim.data.qpos[robot.sim.hand.qpos_ids] = CAT_PAW_RAD
        robot.sim.data.qvel[robot.sim.hand.dof_ids] = 0.0
        robot.hand.command(CAT_PAW_RAD)
        robot.sim.data.ctrl[robot.sim.left.actuator_ids] = plan.left_ready_rad
        robot.sim.data.ctrl[robot.sim.hand.actuator_ids] = CAT_PAW_RAD
        mujoco.mj_forward(robot.sim.model, robot.sim.data)

        cube_site = robot.sim.require_site("guarded_chop_cube_center")
        cube_position = robot.sim.data.site_xpos[cube_site].copy()
        cube_rotation = robot.sim.data.site_xmat[cube_site].copy()
        last_left_target = robot._left_target.copy()
        last_right_target = robot._right_target.copy()

        def step_and_record(
            current_phase: GuardedChopPhase,
            cut_index: int,
        ) -> None:
            nonlocal last_left_target, last_right_target
            left_stationary = np.array_equal(
                robot._left_target, last_left_target
            )
            right_stationary = np.array_equal(
                robot._right_target, last_right_target
            )
            robot.step(config.control_dt_s)
            sample = _observe_sample(
                robot,
                current_phase,
                cut_index,
                plan.safe_knife_height_m - 0.005,
                left_stationary=left_stationary,
                right_stationary=right_stationary,
                safety=safety,
                trace=trace,
            )
            samples.append(sample)
            last_left_target = robot._left_target.copy()
            last_right_target = robot._right_target.copy()
            if not sample.safety_allowed:
                observation = _safety_observation(
                    robot,
                    current_phase,
                    plan.safe_knife_height_m - 0.005,
                    sample.knife_guard_distance_m,
                    left_stationary,
                    right_stationary,
                )
                decision = safety.evaluate(observation)
                raise RuntimeError(decision.reason)

        def latch_hand_pose() -> None:
            hand_position = robot.sim.data.qpos[
                robot.sim.hand.qpos_ids
            ].copy()
            hand_ranges = robot.sim.model.actuator_ctrlrange[
                robot.sim.hand.actuator_ids
            ]
            robot.hand.command(
                np.clip(hand_position, hand_ranges[:, 0], hand_ranges[:, 1])
            )

        def wait_for_guard_stability(
            current_phase: GuardedChopPhase,
            cut_index: int,
        ) -> None:
            latch_hand_pose()
            stable_time = 0.0
            maximum_steps = int(
                round(config.stability_timeout_s / config.control_dt_s)
            )
            for _ in range(maximum_steps):
                step_and_record(current_phase, cut_index)
                hand_speed = float(
                    np.linalg.norm(
                        robot.sim.data.qvel[robot.sim.hand.dof_ids]
                    )
                )
                if (
                    np.linalg.norm(robot.left_joint_velocities) <= 0.05
                    and hand_speed <= 0.05
                ):
                    stable_time += config.control_dt_s
                    if stable_time + 1e-12 >= config.interlock_settle_s:
                        return
                else:
                    stable_time = 0.0
            raise RuntimeError("left guard did not settle before cut")

        phase = GuardedChopPhase.GUARD_READY
        for _ in range(
            int(round(config.guard_ready_duration_s / config.control_dt_s))
        ):
            step_and_record(phase, 0)
        wait_for_guard_stability(phase, 0)

        for index, cut in enumerate(plan.cuts, start=1):
            phase = GuardedChopPhase.CUT_DOWN
            for point in cut.descent[1:]:
                robot.command(point.joints_rad)
                step_and_record(phase, index)
            completed_cuts += 1

            phase = GuardedChopPhase.KNIFE_UP
            for point in (*cut.retract[1:], *cut.shift[1:]):
                robot.command(point.joints_rad)
                step_and_record(phase, index)
            for _ in range(
                int(round(config.interlock_settle_s / config.control_dt_s))
            ):
                step_and_record(phase, index)

            if index <= len(plan.guard_shifts):
                phase = GuardedChopPhase.HAND_SHIFT
                for point in plan.guard_shifts[index - 1][1:]:
                    robot.command_left(point.joints_rad)
                    latch_hand_pose()
                    step_and_record(phase, index)
                wait_for_guard_stability(phase, index)
                completed_shifts += 1

        phase = GuardedChopPhase.COMPLETE
        complete_steps = max(
            1, int(round(config.final_hold_s / config.control_dt_s))
        )
        for _ in range(complete_steps):
            step_and_record(phase, config.cuts)

        np.testing.assert_array_equal(
            robot.sim.data.site_xpos[cube_site], cube_position
        )
        np.testing.assert_array_equal(
            robot.sim.data.site_xmat[cube_site], cube_rotation
        )
        minimum = min(
            sample.knife_guard_distance_m for sample in samples
        )
        if completed_cuts != 5 or completed_shifts != 4:
            raise RuntimeError("guarded chopping did not complete 5 cuts/4 shifts")
        if minimum < config.minimum_distance_m:
            raise RuntimeError("knife-guard distance below final limit")
        return GuardedChopResult(
            success=True,
            final_phase=phase,
            reason="",
            samples=tuple(samples),
            completed_cuts=completed_cuts,
            completed_shifts=completed_shifts,
            total_shift_m=completed_shifts * config.hand_shift_m,
            minimum_distance_m=minimum,
        )
    except (AssertionError, PathIkError, RuntimeError, ValueError) as error:
        if trace is not None and hasattr(trace, "set_abort"):
            trace.set_abort(str(error))
        minimum = min(
            (sample.knife_guard_distance_m for sample in samples),
            default=float("inf"),
        )
        return GuardedChopResult(
            success=False,
            final_phase=GuardedChopPhase.ABORTED,
            reason=str(error),
            samples=tuple(samples),
            completed_cuts=completed_cuts,
            completed_shifts=completed_shifts,
            total_shift_m=completed_shifts * config.hand_shift_m,
            minimum_distance_m=minimum,
        )
    finally:
        robot.close()


def _activate_guarded_scene(robot: RightArmRobot) -> None:
    cube = robot.sim.require_geom("guarded_chop_cube")
    cube_body = int(robot.sim.model.geom_bodyid[cube])
    robot.sim.model.geom_rgba[cube] = (0.75, 0.18, 0.12, 1.0)
    guard_contact_bit = 8
    robot.sim.model.geom_contype[cube] = guard_contact_bit
    robot.sim.model.geom_conaffinity[cube] = guard_contact_bit
    robot.sim.model.body_contype[cube_body] = guard_contact_bit
    robot.sim.model.body_conaffinity[cube_body] = guard_contact_bit
    for name in (
        "left_finger3_pad",
        "right_knife_blade",
    ):
        geom = robot.sim.require_geom(name)
        robot.sim.model.geom_contype[geom] |= guard_contact_bit
        robot.sim.model.geom_conaffinity[geom] |= guard_contact_bit
        body = int(robot.sim.model.geom_bodyid[geom])
        while body:
            robot.sim.model.body_contype[body] |= guard_contact_bit
            robot.sim.model.body_conaffinity[body] |= guard_contact_bit
            body = int(robot.sim.model.body_parentid[body])
    for name in (
        "pick_source_pedestal",
        "pick_target_pedestal",
        "pick_cube_geom",
        "pick_target_region",
    ):
        geom = robot.sim.require_geom(name)
        robot.sim.model.geom_rgba[geom, 3] = 0.0
        robot.sim.model.geom_contype[geom] = 0
        robot.sim.model.geom_conaffinity[geom] = 0
    palm = robot.sim.require_body("left_palm_link")
    for geom in range(robot.sim.model.ngeom):
        if (
            _geom_belongs_to_body_tree(robot, geom, palm)
            and mujoco.mj_id2name(
                robot.sim.model, mujoco.mjtObj.mjOBJ_GEOM, geom
            )
            is None
        ):
            robot.sim.model.geom_contype[geom] = 0
            robot.sim.model.geom_conaffinity[geom] = 0


def _safety_observation(
    robot: RightArmRobot,
    phase: GuardedChopPhase,
    safe_height_m: float,
    distance_m: float,
    left_stationary: bool,
    right_stationary: bool,
) -> SafetyObservation:
    _, penetration_m, normal_force_n = _hand_cube_contact_metrics(robot)
    finite = all(
        np.isfinite(values).all()
        for values in (
            robot.sim.data.qpos,
            robot.sim.data.qvel,
            robot.sim.data.ctrl,
        )
    )
    return SafetyObservation(
        phase=phase.value,
        knife_height_m=_current_blade_bottom(robot),
        safe_knife_height_m=safe_height_m,
        knife_guard_distance_m=distance_m,
        left_target_stationary=left_stationary,
        right_target_stationary=right_stationary,
        left_speed_rad_s=float(
            np.linalg.norm(robot.left_joint_velocities)
        ),
        right_speed_rad_s=float(np.linalg.norm(robot.joint_velocities)),
        hand_speed_rad_s=float(
            np.linalg.norm(
                robot.sim.data.qvel[robot.sim.hand.dof_ids]
            )
        ),
        guard_cube_penetration_m=penetration_m,
        guard_cube_normal_force_n=normal_force_n,
        finite_state=finite,
    )


def _observe_sample(
    robot: RightArmRobot,
    phase: GuardedChopPhase,
    cut_index: int,
    safe_height_m: float,
    *,
    left_stationary: bool,
    right_stationary: bool,
    safety: SafetyCoordinator,
    trace=None,
) -> GuardedChopSample:
    blade = robot.sim.require_geom("right_knife_blade")
    knuckle = robot.sim.require_site("left_guard_knuckle_site")
    guard_position = robot.sim.data.site_xpos[knuckle].copy()
    distance = _blade_hand_distance(robot)
    observation = _safety_observation(
        robot,
        phase,
        safe_height_m,
        distance,
        left_stationary,
        right_stationary,
    )
    decision = safety.evaluate(observation)
    sample = GuardedChopSample(
        time_s=float(robot.sim.data.time),
        phase=phase,
        cut_index=cut_index,
        left_target_rad=robot._left_target,
        left_actual_rad=robot.left_joint_positions,
        right_target_rad=robot._right_target,
        right_actual_rad=robot.joint_positions,
        hand_target_rad=robot.hand.target,
        knife_position=robot.sim.data.geom_xpos[blade],
        guard_position=guard_position,
        knife_guard_distance_m=distance,
        knife_height_m=observation.knife_height_m,
        cut_allowed=(
            decision.allowed and phase is GuardedChopPhase.CUT_DOWN
        ),
        safety_allowed=decision.allowed,
        guard_cube_contact_count=_hand_cube_contact_count(robot),
        guard_cube_penetration_m=observation.guard_cube_penetration_m,
        guard_cube_normal_force_n=observation.guard_cube_normal_force_n,
        blade_hand_contact_count=_blade_hand_contact_count(robot),
        left_speed_rad_s=observation.left_speed_rad_s,
        right_speed_rad_s=observation.right_speed_rad_s,
        hand_speed_rad_s=observation.hand_speed_rad_s,
    )
    if trace is not None:
        trace.append(
            planned_knife=_blade_center_for_joints(
                robot, robot._right_target
            ),
            actual_knife=sample.knife_position,
            actual_guard=sample.guard_position,
            guard_target=_knuckle_for_left_target(robot),
            phase=phase.value,
            cut_index=cut_index,
            minimum_distance_m=distance,
            cut_allowed=sample.cut_allowed,
        )
    return sample


def _current_blade_bottom(robot: RightArmRobot) -> float:
    blade = robot.sim.require_geom("right_knife_blade")
    rotation = robot.sim.data.geom_xmat[blade].reshape(3, 3)
    radius = float(
        np.abs(rotation[2]) @ robot.sim.model.geom_size[blade]
    )
    return float(robot.sim.data.geom_xpos[blade, 2] - radius)


def _blade_center_for_joints(
    robot: RightArmRobot, joints_rad: np.ndarray
) -> np.ndarray:
    blade = robot.sim.require_geom("right_knife_blade")
    with robot.right_kinematics._configuration(joints_rad):
        return robot.sim.data.geom_xpos[blade].copy()


def _knuckle_for_left_target(robot: RightArmRobot) -> np.ndarray:
    knuckle = robot.sim.require_site("left_guard_knuckle_site")
    with robot.left_kinematics._configuration(robot._left_target):
        return robot.sim.data.site_xpos[knuckle].copy()


def _hand_cube_contact_count(robot: RightArmRobot) -> int:
    return _hand_cube_contact_metrics(robot)[0]


def _hand_cube_contact_metrics(
    robot: RightArmRobot,
) -> tuple[int, float, float]:
    cube = robot.sim.require_geom("guarded_chop_cube")
    palm = robot.sim.require_body("left_palm_link")

    count = 0
    maximum_penetration = 0.0
    maximum_normal_force = 0.0
    for index in range(robot.sim.data.ncon):
        contact = robot.sim.data.contact[index]
        geom1, geom2 = int(contact.geom1), int(contact.geom2)
        if (
            geom1 == cube
            and _geom_belongs_to_body_tree(robot, geom2, palm)
        ) or (
            geom2 == cube
            and _geom_belongs_to_body_tree(robot, geom1, palm)
        ):
            count += 1
            maximum_penetration = max(
                maximum_penetration, max(0.0, -float(contact.dist))
            )
            contact_force = np.zeros(6)
            mujoco.mj_contactForce(
                robot.sim.model, robot.sim.data, index, contact_force
            )
            maximum_normal_force = max(
                maximum_normal_force, abs(float(contact_force[0]))
            )
    return count, maximum_penetration, maximum_normal_force


def _blade_hand_distance(robot: RightArmRobot) -> float:
    blade = robot.sim.require_geom("right_knife_blade")
    palm = robot.sim.require_body("left_palm_link")
    distances = []
    for geom in range(robot.sim.model.ngeom):
        if geom == blade or robot.sim.model.geom_contype[geom] == 0:
            continue
        name = mujoco.mj_id2name(
            robot.sim.model, mujoco.mjtObj.mjOBJ_GEOM, geom
        )
        if name is None or not _geom_belongs_to_body_tree(
            robot, geom, palm
        ):
            continue
        from_to = np.zeros(6)
        distance = mujoco.mj_geomDistance(
            robot.sim.model,
            robot.sim.data,
            blade,
            geom,
            1.0,
            from_to,
        )
        distances.append(float(distance))
    if not distances:
        raise RuntimeError("no collidable left-hand geometry found")
    return min(distances)


def _blade_hand_contact_count(robot: RightArmRobot) -> int:
    blade = robot.sim.require_geom("right_knife_blade")
    palm = robot.sim.require_body("left_palm_link")
    count = 0
    for index in range(robot.sim.data.ncon):
        contact = robot.sim.data.contact[index]
        geom1, geom2 = int(contact.geom1), int(contact.geom2)
        if (
            geom1 == blade
            and _geom_belongs_to_body_tree(robot, geom2, palm)
        ) or (
            geom2 == blade
            and _geom_belongs_to_body_tree(robot, geom1, palm)
        ):
            count += 1
    return count


def _geom_belongs_to_body_tree(
    robot: RightArmRobot, geom: int, root_body: int
) -> bool:
    body = int(robot.sim.model.geom_bodyid[geom])
    while body:
        if body == root_body:
            return True
        body = int(robot.sim.model.body_parentid[body])
    return False
