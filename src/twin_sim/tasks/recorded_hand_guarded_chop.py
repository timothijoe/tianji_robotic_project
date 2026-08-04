"""Preflight planning for recorded-hand guarded chopping."""

from dataclasses import dataclass
from enum import Enum
from pathlib import Path

import mujoco
import numpy as np

from twin_sim.kinematics import PathIkError
from twin_sim.raised_work_surface import (
    apply_work_surface_offset,
    restore_work_surface,
    work_surface_height_m,
)
from twin_sim.recorded_hand_guard import RecordedGuardCycle
from twin_sim.robot import RightArmRobot
from twin_sim.tasks.guarded_chop import (
    GuardedChopConfig,
    _blade_hand_distance,
    _geom_belongs_to_body_tree,
    _minimum_jerk_joint_trajectory,
    _preflight_guarded_chop,
)
from twin_sim.tasks.line_chop import _CutTrajectories


@dataclass(frozen=True)
class RecordedHandGuardedChopConfig:
    """Safety and placement policy for the combined five-cut task."""

    control_dt_s: float = 0.01
    cuts: int = 5
    surface_offsets_m: tuple[float, ...] = tuple(
        float(value) for value in np.arange(0.04, 0.121, 0.01)
    )
    minimum_distance_m: float = 0.02
    maximum_hand_penetration_m: float = 0.0005
    minimum_thumb_clearance_m: float = 0.01
    anchor_lateral_clearance_m: float = 0.12
    combined_pad_contact_lift_m: float = 0.0098
    ik_tolerance: float = 0.003
    maximum_arm_joint_step_rad: float = 0.12
    reset_duration_s: float = 0.5
    final_hold_s: float = 3.0

    def validated(self) -> "RecordedHandGuardedChopConfig":
        if self.cuts != 5:
            raise ValueError("cuts must equal 5")
        positive = (
            self.control_dt_s,
            self.minimum_distance_m,
            self.maximum_hand_penetration_m,
            self.minimum_thumb_clearance_m,
            self.anchor_lateral_clearance_m,
            self.ik_tolerance,
            self.maximum_arm_joint_step_rad,
            self.reset_duration_s,
        )
        if not all(np.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("recorded guarded-chop limits must be positive and finite")
        if (
            not np.isfinite(self.combined_pad_contact_lift_m)
            or self.combined_pad_contact_lift_m < 0.0
        ):
            raise ValueError("combined_pad_contact_lift_m must be non-negative")
        if not np.isfinite(self.final_hold_s) or self.final_hold_s < 0.0:
            raise ValueError("final_hold_s must be non-negative and finite")
        if not self.surface_offsets_m or not all(
            np.isfinite(value) and value >= 0.0
            for value in self.surface_offsets_m
        ):
            raise ValueError("surface_offsets_m must contain finite non-negative values")
        return self


@dataclass(frozen=True)
class RecordedLeftCycle:
    left: np.ndarray
    hand: np.ndarray
    phases: tuple[str, ...]
    palm_targets: np.ndarray


@dataclass(frozen=True)
class RecordedHandGuardedChopPlan:
    surface_offset_m: float
    right_ready_rad: np.ndarray
    left_ready_rad: np.ndarray
    right_cuts: tuple[_CutTrajectories, ...]
    knife_lift_shifts: tuple
    left_cycles: tuple[RecordedLeftCycle, ...]
    left_resets: tuple[RecordedLeftCycle, ...]
    safe_knife_height_m: float
    minimum_planned_distance_m: float
    maximum_hand_penetration_m: float
    minimum_thumb_clearance_m: float


class RecordedHandGuardedChopPhase(str, Enum):
    INITIALIZE = "initialize"
    HAND_MOTION = "hand_motion"
    HAND_SAFE = "hand_safe"
    CUT_DOWN = "cut_down"
    KNIFE_RETRACT = "knife_retract"
    KNIFE_SHIFT = "knife_shift"
    RESET = "reset"
    COMPLETE = "complete"
    ABORTED = "aborted"


@dataclass(frozen=True)
class RecordedHandGuardedChopResult:
    success: bool
    final_phase: RecordedHandGuardedChopPhase
    reason: str
    completed_cuts: int
    completed_hand_cycles: int
    surface_offset_m: float
    minimum_distance_m: float
    maximum_hand_penetration_m: float
    minimum_thumb_clearance_m: float
    events: tuple[tuple[int, str], ...]


def _chopping_aligned_palm_rotation(initial_rotation: np.ndarray) -> np.ndarray:
    angle = -np.pi / 2.0
    world_quarter_turn = np.asarray(
        (
            (np.cos(angle), -np.sin(angle), 0.0),
            (np.sin(angle), np.cos(angle), 0.0),
            (0.0, 0.0, 1.0),
        )
    )
    return (
        world_quarter_turn
        @ np.diag((-1.0, -1.0, 1.0))
        @ np.asarray(initial_rotation, dtype=float)
    )


def _lateral_guard_anchor_xy(
    cut_xy: np.ndarray,
    clearance_m: float,
) -> np.ndarray:
    cut = np.asarray(cut_xy, dtype=float)
    return np.asarray((cut[0] - clearance_m, cut[1]))


def _preflight_recorded_hand_guarded_chop(
    robot: RightArmRobot,
    cycle: RecordedGuardCycle,
    config: RecordedHandGuardedChopConfig,
) -> RecordedHandGuardedChopPlan:
    config = config.validated()
    failures = []
    for offset in config.surface_offsets_m:
        try:
            return _build_candidate_plan(robot, cycle, config, float(offset))
        except (PathIkError, ValueError) as error:
            failures.append(f"{float(offset):.3f} m: {error}")
    raise ValueError(
        "no safe shared work-surface height found; " + "; ".join(failures)
    )


def _build_candidate_plan(
    robot: RightArmRobot,
    cycle: RecordedGuardCycle,
    config: RecordedHandGuardedChopConfig,
    offset_m: float,
) -> RecordedHandGuardedChopPlan:
    snapshot = apply_work_surface_offset(robot.sim, offset_m)
    keep_surface = False
    saved = mujoco.MjData(robot.sim.model)
    mujoco.mj_copyData(saved, robot.sim.model, robot.sim.data)
    try:
        right = _preflight_guarded_chop(
            robot,
            GuardedChopConfig(
                scene_mode="plane",
                cuts=config.cuts,
                control_dt_s=config.control_dt_s,
                minimum_distance_m=config.minimum_distance_m,
            ),
        )
        surface_z = work_surface_height_m(robot.sim)
        cycles = []
        seed = right.left_ready_rad.copy()
        hand_limits = robot.sim.model.actuator_ctrlrange[
            robot.sim.hand.actuator_ids
        ]
        hand_positions = np.clip(
            cycle.hand_positions_rad,
            hand_limits[:, 0],
            hand_limits[:, 1],
        )
        tcp_from_palm = np.eye(4)
        tcp_from_palm[2, 3] = 0.07

        for cut_xy in right.cut_points_xy:
            anchor = cycle.initial_palm_transform.copy()
            anchor[:3, :3] = _chopping_aligned_palm_rotation(
                anchor[:3, :3]
            )
            anchor[:3, 3] = (
                *_lateral_guard_anchor_xy(
                    cut_xy, config.anchor_lateral_clearance_m
                ),
                float(
                    cycle.initial_palm_transform[2, 3]
                    + surface_z
                    + config.combined_pad_contact_lift_m
                ),
            )
            palm_targets = np.asarray(
                [anchor @ relative for relative in cycle.relative_palm_transforms]
            )
            tcp_targets = np.asarray(
                [target @ tcp_from_palm for target in palm_targets]
            )
            left = _solve_left_path(robot, tcp_targets, seed, config)
            seed = left[-1].copy()
            cycles.append(
                RecordedLeftCycle(
                    left=left,
                    hand=hand_positions.copy(),
                    phases=cycle.phases,
                    palm_targets=palm_targets,
                )
            )

        resets = []
        reset_count = int(round(config.reset_duration_s / config.control_dt_s)) + 1
        for current, following in zip(cycles[:-1], cycles[1:], strict=True):
            left = np.asarray(
                _minimum_jerk_joint_trajectory(
                    current.left[-1],
                    following.left[0],
                    config.reset_duration_s,
                    config.control_dt_s,
                )
            )
            hand = np.asarray(
                _minimum_jerk_joint_trajectory(
                    current.hand[-1],
                    following.hand[0],
                    config.reset_duration_s,
                    config.control_dt_s,
                )
            )
            resets.append(
                RecordedLeftCycle(
                    left=left,
                    hand=hand,
                    phases=("RESET",) * reset_count,
                    palm_targets=np.empty((reset_count, 4, 4)),
                )
            )

        minimum_distance, maximum_penetration, minimum_thumb_clearance = (
            _measure_plan_safety(robot, right.cuts, tuple(cycles), surface_z)
        )
        if minimum_distance < config.minimum_distance_m:
            raise ValueError(
                f"knife-hand clearance {minimum_distance:.6f} m is below "
                f"{config.minimum_distance_m:.6f} m"
            )
        if maximum_penetration > config.maximum_hand_penetration_m:
            raise ValueError(
                f"hand-board penetration {maximum_penetration:.6f} m exceeds "
                f"{config.maximum_hand_penetration_m:.6f} m"
            )
        if minimum_thumb_clearance < config.minimum_thumb_clearance_m:
            raise ValueError(
                f"thumb clearance {minimum_thumb_clearance:.6f} m is below "
                f"{config.minimum_thumb_clearance_m:.6f} m"
            )
        keep_surface = True
        return RecordedHandGuardedChopPlan(
            surface_offset_m=offset_m,
            right_ready_rad=right.right_ready_rad.copy(),
            left_ready_rad=cycles[0].left[0].copy(),
            right_cuts=right.cuts,
            knife_lift_shifts=right.knife_lift_shifts,
            left_cycles=tuple(cycles),
            left_resets=tuple(resets),
            safe_knife_height_m=right.safe_knife_height_m,
            minimum_planned_distance_m=minimum_distance,
            maximum_hand_penetration_m=maximum_penetration,
            minimum_thumb_clearance_m=minimum_thumb_clearance,
        )
    finally:
        mujoco.mj_copyData(robot.sim.data, robot.sim.model, saved)
        if not keep_surface:
            restore_work_surface(robot.sim, snapshot)


def _solve_left_path(
    robot: RightArmRobot,
    targets: np.ndarray,
    seed: np.ndarray,
    config: RecordedHandGuardedChopConfig,
) -> np.ndarray:
    previous = np.asarray(seed, dtype=float)
    solutions = []
    for index, target in enumerate(targets):
        result = robot.left_kinematics.ik(
            target,
            previous,
            max_iterations=150,
            tolerance=config.ik_tolerance,
            damping=0.003,
        )
        if not result.success:
            raise PathIkError(
                f"recorded left-arm IK failed at sample {index} "
                f"(residual {result.residual:.6g})"
            )
        step = float(np.max(np.abs(result.joints_rad - previous)))
        if solutions and step > config.maximum_arm_joint_step_rad:
            raise PathIkError(
                f"left-arm joint step {step:.6g} rad exceeds "
                f"{config.maximum_arm_joint_step_rad:.6g} rad at sample {index}"
            )
        solutions.append(result.joints_rad.copy())
        previous = result.joints_rad
    return np.asarray(solutions)


def _measure_plan_safety(
    robot: RightArmRobot,
    cuts: tuple[_CutTrajectories, ...],
    cycles: tuple[RecordedLeftCycle, ...],
    surface_z: float,
) -> tuple[float, float, float]:
    model, data = robot.sim.model, robot.sim.data
    board = robot.sim.require_geom("chopping_board")
    thumb = robot.sim.require_geom("left_finger1_pad")
    thumb_radius = float(model.geom_size[thumb, 0])
    minimum_distance = np.inf
    maximum_penetration = 0.0
    minimum_thumb_clearance = np.inf

    def measure(right_rad: np.ndarray, left_rad: np.ndarray, hand_rad: np.ndarray) -> None:
        nonlocal minimum_distance, maximum_penetration, minimum_thumb_clearance
        data.qpos[robot.sim.right.qpos_ids] = right_rad
        data.qpos[robot.sim.left.qpos_ids] = left_rad
        data.qpos[robot.sim.hand.qpos_ids] = hand_rad
        mujoco.mj_forward(model, data)
        minimum_distance = min(minimum_distance, _blade_hand_distance(robot))
        minimum_thumb_clearance = min(
            minimum_thumb_clearance,
            float(data.geom_xpos[thumb, 2] - thumb_radius - surface_z),
        )
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            if board in (int(contact.geom1), int(contact.geom2)):
                maximum_penetration = max(
                    maximum_penetration, max(0.0, -float(contact.dist))
                )

    for cut, cycle in zip(cuts, cycles, strict=True):
        for right_point in cut.descent:
            measure(right_point.joints_rad, cycle.left[-1], cycle.hand[-1])
        for left_rad, hand_rad in zip(cycle.left, cycle.hand, strict=True):
            measure(cut.descent[-1].joints_rad, left_rad, hand_rad)
    return (
        float(minimum_distance),
        float(maximum_penetration),
        float(minimum_thumb_clearance),
    )


def run_recorded_hand_guarded_chop(
    config: RecordedHandGuardedChopConfig,
    *,
    hand_mcap: Path,
    viewer: bool = False,
    trace=None,
    record_path: Path | None = None,
) -> RecordedHandGuardedChopResult:
    """Execute five recorded guard cycles and five right-arm cuts."""
    if record_path is not None:
        raise NotImplementedError("recording output is not implemented yet")
    config = config.validated()
    cycle = __import__(
        "twin_sim.recorded_hand_guard", fromlist=["load_recorded_guard_cycle"]
    ).load_recorded_guard_cycle(hand_mcap, control_dt_s=config.control_dt_s)
    robot = RightArmRobot(viewer=False)
    completed_cuts = 0
    completed_cycles = 0
    events: list[tuple[int, str]] = []
    phase = RecordedHandGuardedChopPhase.INITIALIZE
    plan = None
    reason = ""
    try:
        plan = _preflight_recorded_hand_guarded_chop(robot, cycle, config)
        robot.reset(plan.right_ready_rad)
        robot.sim.data.qpos[robot.sim.left.qpos_ids] = plan.left_ready_rad
        robot.sim.data.qpos[robot.sim.hand.qpos_ids] = plan.left_cycles[0].hand[0]
        robot.command_left(plan.left_ready_rad)
        robot.hand.command(plan.left_cycles[0].hand[0])
        mujoco.mj_forward(robot.sim.model, robot.sim.data)
        if viewer:
            robot.open_viewer()
            _prepare_viewer(robot)

        def execute(
            current_phase: RecordedHandGuardedChopPhase,
            right_rad: np.ndarray,
            left_rad: np.ndarray,
            hand_rad: np.ndarray,
            cycle_index: int,
        ) -> None:
            nonlocal phase
            phase = current_phase
            robot.command(right_rad)
            robot.command_left(left_rad)
            robot.hand.command(hand_rad)
            robot.step(config.control_dt_s)
            if trace is not None:
                trace.update(robot, current_phase.value, cycle_index)

        for index, (cut, left_cycle) in enumerate(
            zip(plan.right_cuts, plan.left_cycles, strict=True), start=1
        ):
            right_safe = cut.descent[0].joints_rad
            for left_rad, hand_rad in zip(
                left_cycle.left, left_cycle.hand, strict=True
            ):
                execute(
                    RecordedHandGuardedChopPhase.HAND_MOTION,
                    right_safe,
                    left_rad,
                    hand_rad,
                    index,
                )
            completed_cycles += 1
            events.append((index, "HAND_SAFE"))
            phase = RecordedHandGuardedChopPhase.HAND_SAFE
            events.append((index, "CUT_DOWN"))
            for point in cut.descent[1:]:
                execute(
                    RecordedHandGuardedChopPhase.CUT_DOWN,
                    point.joints_rad,
                    left_cycle.left[-1],
                    left_cycle.hand[-1],
                    index,
                )
            completed_cuts += 1
            for point in cut.retract[1:]:
                execute(
                    RecordedHandGuardedChopPhase.KNIFE_RETRACT,
                    point.joints_rad,
                    left_cycle.left[-1],
                    left_cycle.hand[-1],
                    index,
                )
            for point in cut.shift[1:]:
                execute(
                    RecordedHandGuardedChopPhase.KNIFE_SHIFT,
                    point.joints_rad,
                    left_cycle.left[-1],
                    left_cycle.hand[-1],
                    index,
                )
            if index <= len(plan.left_resets):
                reset = plan.left_resets[index - 1]
                next_right = plan.right_cuts[index].descent[0].joints_rad
                for left_rad, hand_rad in zip(reset.left[1:], reset.hand[1:], strict=True):
                    execute(
                        RecordedHandGuardedChopPhase.RESET,
                        next_right,
                        left_rad,
                        hand_rad,
                        index,
                    )

        phase = RecordedHandGuardedChopPhase.COMPLETE
        for _ in range(int(round(config.final_hold_s / config.control_dt_s))):
            robot.step(config.control_dt_s)
        return RecordedHandGuardedChopResult(
            success=True,
            final_phase=phase,
            reason="",
            completed_cuts=completed_cuts,
            completed_hand_cycles=completed_cycles,
            surface_offset_m=plan.surface_offset_m,
            minimum_distance_m=plan.minimum_planned_distance_m,
            maximum_hand_penetration_m=plan.maximum_hand_penetration_m,
            minimum_thumb_clearance_m=plan.minimum_thumb_clearance_m,
            events=tuple(events),
        )
    except Exception as error:
        reason = str(error)
        phase = RecordedHandGuardedChopPhase.ABORTED
        return RecordedHandGuardedChopResult(
            success=False,
            final_phase=phase,
            reason=reason,
            completed_cuts=completed_cuts,
            completed_hand_cycles=completed_cycles,
            surface_offset_m=float("nan") if plan is None else plan.surface_offset_m,
            minimum_distance_m=float("nan") if plan is None else plan.minimum_planned_distance_m,
            maximum_hand_penetration_m=float("nan") if plan is None else plan.maximum_hand_penetration_m,
            minimum_thumb_clearance_m=float("nan") if plan is None else plan.minimum_thumb_clearance_m,
            events=tuple(events),
        )
    finally:
        robot.close()


def _prepare_viewer(robot: RightArmRobot) -> None:
    viewer = robot._viewer
    if viewer is None:
        return
    viewer.cam.azimuth = 135.0
    viewer.cam.elevation = -22.0
    viewer.cam.distance = 1.7
    viewer.cam.lookat[:] = (0.5, 0.02, 0.5)
