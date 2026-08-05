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
from twin_sim.recorded_hand_guard import (
    RecordedGuardCycle,
    shape_pip_led_guard_hand,
)
from twin_sim.robot import RightArmRobot
from twin_sim.tasks.chop import ChopConfig
from twin_sim.tasks.guarded_chop import (
    _KNIFE_SAFE_TRACKING_HEADROOM_M,
    GuardedChopConfig,
    _blade_hand_distance,
    _blade_bottom_height,
    _diagonal_lift_shifts,
    _geom_belongs_to_body_tree,
    _right_to_left_indices,
    _translate_right_cuts,
)
from twin_sim.tasks.line_chop import (
    LineChopConfig,
    _CutTrajectories,
    _preflight_line_chop,
)
from twin_sim.tasks.pick_place import LEFT_GRASP_READY_RAD


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
    anchor_lateral_clearance_m: float = 0.24
    target_lateral_spacing_m: float = 0.030
    lateral_spacing_tolerance_m: float = 0.003
    anchor_longitudinal_offset_m: float = -0.18
    combined_pad_contact_lift_m: float = 0.0098
    ik_tolerance: float = 0.003
    maximum_arm_joint_step_rad: float = 0.12
    reset_duration_s: float = 0.5
    final_hold_s: float = 3.0
    total_wrist_retreat_m: float = 0.032

    def validated(self) -> "RecordedHandGuardedChopConfig":
        if self.cuts != 5:
            raise ValueError("cuts must equal 5")
        positive = (
            self.control_dt_s,
            self.minimum_distance_m,
            self.maximum_hand_penetration_m,
            self.minimum_thumb_clearance_m,
            self.anchor_lateral_clearance_m,
            self.target_lateral_spacing_m,
            self.lateral_spacing_tolerance_m,
            self.ik_tolerance,
            self.maximum_arm_joint_step_rad,
            self.reset_duration_s,
            self.total_wrist_retreat_m,
        )
        if not all(np.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("recorded guarded-chop limits must be positive and finite")
        if (
            not np.isfinite(self.combined_pad_contact_lift_m)
            or self.combined_pad_contact_lift_m < 0.0
        ):
            raise ValueError("combined_pad_contact_lift_m must be non-negative")
        if not np.isfinite(self.anchor_longitudinal_offset_m):
            raise ValueError("anchor_longitudinal_offset_m must be finite")
        if not np.isfinite(self.final_hold_s) or self.final_hold_s < 0.0:
            raise ValueError("final_hold_s must be non-negative and finite")
        if not 0.025 <= self.total_wrist_retreat_m <= 0.040:
            raise ValueError("total_wrist_retreat_m must be between 0.025 and 0.040")
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
class SynchronizedGuardCycle:
    """Equal-length right-arm, left-arm, and hand commands for one cut."""

    right: np.ndarray
    left: np.ndarray
    hand: np.ndarray
    knife_phases: tuple[str, ...]
    palm_targets: np.ndarray
    knife_targets: np.ndarray


@dataclass(frozen=True)
class RecordedHandGuardedChopPlan:
    surface_offset_m: float
    right_ready_rad: np.ndarray
    left_ready_rad: np.ndarray
    right_cuts: tuple[_CutTrajectories, ...]
    knife_lift_shifts: tuple
    left_cycles: tuple[RecordedLeftCycle, ...]
    synchronized_cycles: tuple[SynchronizedGuardCycle, ...]
    left_transitions: tuple[RecordedLeftCycle, ...]
    safe_knife_height_m: float
    minimum_planned_distance_m: float
    minimum_lateral_spacing_m: float
    maximum_lateral_spacing_m: float
    maximum_hand_penetration_m: float
    minimum_thumb_clearance_m: float
    minimum_pad_step_y_m: float
    pad_net_retreats_m: np.ndarray


@dataclass(frozen=True)
class _TableCutPlan:
    right_ready_rad: np.ndarray
    left_ready_rad: np.ndarray
    cuts: tuple[_CutTrajectories, ...]
    knife_lift_shifts: tuple
    cut_points_xy: np.ndarray
    safe_knife_height_m: float


class RecordedHandGuardedChopPhase(str, Enum):
    INITIALIZE = "initialize"
    HAND_MOTION = "hand_motion"
    HAND_SAFE = "hand_safe"
    CUT_DOWN = "cut_down"
    KNIFE_RETRACT = "knife_retract"
    KNIFE_SHIFT = "knife_shift"
    CONTINUOUS_TRANSITION = "continuous_transition"
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
    minimum_lateral_spacing_m: float
    maximum_lateral_spacing_m: float
    maximum_hand_penetration_m: float
    minimum_thumb_clearance_m: float
    events: tuple[tuple[int, str], ...]


_TABLE_ONLY_HIDDEN_GEOMS = (
    "guarded_chop_cube",
    "pick_source_pedestal",
    "pick_target_pedestal",
    "pick_cube_geom",
    "pick_target_region",
)


def _configure_table_only_scene(robot: RightArmRobot) -> None:
    model = robot.sim.model
    for name in _TABLE_ONLY_HIDDEN_GEOMS:
        geom = robot.sim.require_geom(name)
        model.geom_rgba[geom, 3] = 0.0
        model.geom_contype[geom] = 0
        model.geom_conaffinity[geom] = 0
    mujoco.mj_forward(model, robot.sim.data)


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
    return np.asarray((cut[0] - 0.08, cut[1] + clearance_m))


def _preflight_table_cuts(
    robot: RightArmRobot,
    config: RecordedHandGuardedChopConfig,
) -> _TableCutPlan:
    guarded_config = GuardedChopConfig(
        scene_mode="plane",
        cuts=config.cuts,
        control_dt_s=config.control_dt_s,
        minimum_distance_m=config.minimum_distance_m,
    )
    line = _preflight_line_chop(
        robot,
        robot.right_kinematics,
        LineChopConfig(
            chop=ChopConfig(
                control_dt_s=config.control_dt_s,
                descent_duration_s=guarded_config.cut_duration_s,
                retract_duration_s=guarded_config.knife_up_duration_s,
            ),
            cuts=config.cuts,
            spacing_m=guarded_config.hand_shift_m,
            shift_duration_s=guarded_config.hand_shift_duration_s,
        ),
    )
    ordered_indices = _right_to_left_indices(line.cut_points_xy)
    cut_points = line.cut_points_xy[ordered_indices].copy()
    ordered_cuts = tuple(line.cuts[index] for index in ordered_indices)
    cuts = _translate_right_cuts(
        robot,
        ordered_cuts,
        0.0,
        guarded_config,
    )
    lift_shifts = _diagonal_lift_shifts(robot, cuts, guarded_config)
    right_ready = cuts[0].descent[0].joints_rad.copy()
    return _TableCutPlan(
        right_ready_rad=right_ready,
        left_ready_rad=LEFT_GRASP_READY_RAD.copy(),
        cuts=cuts,
        knife_lift_shifts=lift_shifts,
        cut_points_xy=cut_points,
        safe_knife_height_m=(
            _blade_bottom_height(robot, right_ready)
            - _KNIFE_SAFE_TRACKING_HEADROOM_M
        ),
    )


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


def _long_finger_pad_offsets(
    robot: RightArmRobot,
    left_rad: np.ndarray,
    hand_positions: np.ndarray,
) -> np.ndarray:
    data = robot.sim.data
    pad_ids = np.asarray(
        [robot.sim.require_geom(f"left_finger{finger}_pad") for finger in range(2, 6)]
    )
    offsets = []
    for hand_rad in hand_positions:
        data.qpos[robot.sim.left.qpos_ids] = left_rad
        data.qpos[robot.sim.hand.qpos_ids] = hand_rad
        mujoco.mj_forward(robot.sim.model, data)
        palm = robot.left_palm_pose()
        offsets.append(
            (data.geom_xpos[pad_ids] - palm[:3, 3]) @ palm[:3, :3]
        )
    return np.asarray(offsets)


def _compensated_wrist_retreat(
    local_pad_y: np.ndarray,
    total_retreat_m: float,
) -> np.ndarray:
    delta = np.diff(np.asarray(local_pad_y, dtype=float), axis=0)
    required = np.maximum(0.0, np.max(-0.0005 - delta, axis=1))
    required_total = float(np.sum(required))
    if required_total > total_retreat_m:
        raise ValueError(
            f"finger-pad compensation requires {required_total:.6f} m, "
            f"exceeding wrist retreat {total_retreat_m:.6f} m"
        )
    increments = required + (total_retreat_m - required_total) / len(required)
    return np.concatenate(([0.0], np.cumsum(increments)))


def _shape_vertical_distal_guard(
    robot: RightArmRobot,
    hand_positions: np.ndarray,
    phases: tuple[str, ...],
    palm_rotation: np.ndarray,
    *,
    maximum_angle_deg: float = 15.0,
) -> np.ndarray:
    shaped = np.asarray(hand_positions, dtype=float).copy()
    active = np.asarray(phases) != "PREPARE"
    active_indices = np.flatnonzero(active)
    if active_indices.size == 0:
        raise ValueError("recorded guard has no RETREAT/HOLD samples")
    transition_start = max(0, int(active_indices[0]) - 30)
    solved_indices = range(transition_start, len(shaped))
    limits = robot.sim.model.actuator_ctrlrange[robot.sim.hand.actuator_ids]
    left_reference = robot.left_joint_positions
    saved = mujoco.MjData(robot.sim.model)
    mujoco.mj_copyData(saved, robot.sim.model, robot.sim.data)
    try:
        for index in solved_indices:
            for finger, dip in zip(range(2, 6), (7, 11, 15, 19), strict=True):
                body = robot.sim.require_body(f"left_finger{finger}_link4")
                original = shaped[index, dip]
                best = (np.inf, original, np.inf)
                for candidate in np.linspace(limits[dip, 0], limits[dip, 1], 61):
                    hand = shaped[index].copy()
                    hand[dip] = candidate
                    robot.sim.data.qpos[robot.sim.left.qpos_ids] = left_reference
                    robot.sim.data.qpos[robot.sim.hand.qpos_ids] = hand
                    mujoco.mj_forward(robot.sim.model, robot.sim.data)
                    palm = robot.left_palm_pose()
                    axis_world = robot.sim.data.xmat[body].reshape(3, 3)[:, 2]
                    axis_local = palm[:3, :3].T @ axis_world
                    target_axis = palm_rotation @ axis_local
                    angle = float(
                        np.degrees(
                            np.arccos(
                                np.clip(target_axis @ (0.0, 0.0, -1.0), -1.0, 1.0)
                            )
                        )
                    )
                    score = angle + 1e-4 * abs(candidate - original)
                    if score < best[0]:
                        best = (score, float(candidate), angle)
                if active[index] and best[2] > maximum_angle_deg:
                    raise ValueError(
                        f"finger {finger} distal angle {best[2]:.3f} degrees "
                        f"exceeds {maximum_angle_deg:.3f} at sample {index}"
                    )
                if active[index]:
                    weight = 1.0
                else:
                    progress = (index - transition_start + 1) / (
                        active_indices[0] - transition_start + 1
                    )
                    weight = progress * progress * (3.0 - 2.0 * progress)
                shaped[index, dip] = original + weight * (best[1] - original)
    finally:
        mujoco.mj_copyData(robot.sim.data, robot.sim.model, saved)
    if np.max(np.abs(np.diff(shaped, axis=0)), initial=0.0) > 0.12:
        raise ValueError("vertical distal shaping exceeds 0.12 rad joint-step limit")
    return shaped


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
        _configure_table_only_scene(robot)
        right = _preflight_table_cuts(robot, config)
        surface_z = work_surface_height_m(robot.sim)
        seed = right.left_ready_rad.copy()
        hand_limits = robot.sim.model.actuator_ctrlrange[
            robot.sim.hand.actuator_ids
        ]
        hand_positions = np.clip(
            shape_pip_led_guard_hand(
                cycle.hand_positions_rad, cycle.phases, hand_limits
            ),
            hand_limits[:, 0],
            hand_limits[:, 1],
        )
        tcp_from_palm = np.eye(4)
        tcp_from_palm[2, 3] = 0.07
        anchor = cycle.initial_palm_transform.copy()
        anchor[:3, :3] = _chopping_aligned_palm_rotation(anchor[:3, :3])
        anchor[:3, 3] = (
            *_lateral_guard_anchor_xy(
                right.cut_points_xy[0], config.anchor_lateral_clearance_m
            ),
            float(
                cycle.initial_palm_transform[2, 3]
                + surface_z
                + config.combined_pad_contact_lift_m
            ),
        )
        anchor[0, 3] = (
            float(right.cut_points_xy[0, 0]) + config.anchor_longitudinal_offset_m
        )
        hand_positions = _shape_vertical_distal_guard(
            robot,
            hand_positions,
            cycle.phases,
            anchor[:3, :3],
        )
        local_pads = _long_finger_pad_offsets(robot, seed, hand_positions)
        target_pad_offsets = local_pads @ anchor[:3, :3].T
        robot.sim.data.qpos[robot.sim.right.qpos_ids] = right.right_ready_rad
        mujoco.mj_forward(robot.sim.model, robot.sim.data)
        blade_reference_y = float(
            robot.sim.data.site_xpos[
                robot.sim.require_site("right_blade_edge_bot"), 1
            ]
        )
        anchor[1, 3] = (
            blade_reference_y
            + config.target_lateral_spacing_m
            - float(np.min(target_pad_offsets[0, :, 1]))
        )
        carrier = _compensated_wrist_retreat(
            target_pad_offsets[:, :, 1], config.total_wrist_retreat_m
        )
        palm_targets = np.repeat(anchor[None, :, :], len(hand_positions), axis=0)
        palm_targets[:, 1, 3] += carrier
        tcp_targets = np.asarray(
            [target @ tcp_from_palm for target in palm_targets]
        )
        left = _solve_left_path(robot, tcp_targets, seed, config)
        robot.sim.data.qpos[robot.sim.right.qpos_ids] = right.right_ready_rad
        robot.sim.data.qpos[robot.sim.left.qpos_ids] = left[0]
        robot.sim.data.qpos[robot.sim.hand.qpos_ids] = hand_positions[0]
        mujoco.mj_forward(robot.sim.model, robot.sim.data)
        first_pad_y = float(
            np.min(
                robot.sim.data.geom_xpos[
                    np.asarray(
                        [
                            robot.sim.require_geom(f"left_finger{finger}_pad")
                            for finger in range(2, 6)
                        ]
                    ),
                    1,
                ]
            )
        )
        actual_blade_y = float(
            robot.sim.data.site_xpos[
                robot.sim.require_site("right_blade_edge_bot"), 1
            ]
        )
        anchor_correction_y = (
            actual_blade_y + config.target_lateral_spacing_m - first_pad_y
        )
        anchor[1, 3] += anchor_correction_y
        palm_targets[:, 1, 3] += anchor_correction_y
        tcp_targets = np.asarray(
            [target @ tcp_from_palm for target in palm_targets]
        )
        left = _solve_left_path(robot, tcp_targets, seed, config)
        world_pad_y = (
            palm_targets[:, None, 1, 3] + target_pad_offsets[:, :, 1]
        )
        minimum_pad_step_y = float(np.min(np.diff(world_pad_y, axis=0)))
        pad_net_retreats = world_pad_y[-1] - world_pad_y[0]
        cycles = []
        for indices in np.array_split(np.arange(len(hand_positions)), config.cuts):
            cycles.append(
                RecordedLeftCycle(
                    left=left[indices].copy(),
                    hand=hand_positions[indices].copy(),
                    phases=tuple(cycle.phases[index] for index in indices),
                    palm_targets=palm_targets[indices].copy(),
                )
            )
        synchronized_cycles = _build_synchronized_cycles(
            robot,
            right.cuts,
            tuple(cycles),
            config.total_wrist_retreat_m,
            config,
        )

        (
            minimum_distance,
            maximum_penetration,
            minimum_thumb_clearance,
            minimum_lateral_spacing,
            maximum_lateral_spacing,
        ) = (
            _measure_plan_safety(robot, synchronized_cycles, surface_z)
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
        lower_spacing = (
            config.target_lateral_spacing_m - config.lateral_spacing_tolerance_m
        )
        upper_spacing = (
            config.target_lateral_spacing_m + config.lateral_spacing_tolerance_m
        )
        if minimum_lateral_spacing < lower_spacing:
            raise ValueError(
                f"knife-pad lateral spacing {minimum_lateral_spacing:.6f} m is "
                f"below {lower_spacing:.6f} m"
            )
        if maximum_lateral_spacing > upper_spacing:
            raise ValueError(
                f"knife-pad lateral spacing {maximum_lateral_spacing:.6f} m is "
                f"above {upper_spacing:.6f} m"
            )
        keep_surface = True
        return RecordedHandGuardedChopPlan(
            surface_offset_m=offset_m,
            right_ready_rad=right.right_ready_rad.copy(),
            left_ready_rad=cycles[0].left[0].copy(),
            right_cuts=right.cuts,
            knife_lift_shifts=right.knife_lift_shifts,
            left_cycles=tuple(cycles),
            synchronized_cycles=synchronized_cycles,
            left_transitions=(),
            safe_knife_height_m=right.safe_knife_height_m,
            minimum_planned_distance_m=minimum_distance,
            minimum_lateral_spacing_m=minimum_lateral_spacing,
            maximum_lateral_spacing_m=maximum_lateral_spacing,
            maximum_hand_penetration_m=maximum_penetration,
            minimum_thumb_clearance_m=minimum_thumb_clearance,
            minimum_pad_step_y_m=minimum_pad_step_y,
            pad_net_retreats_m=pad_net_retreats.copy(),
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


def _build_synchronized_cycles(
    robot: RightArmRobot,
    cuts: tuple[_CutTrajectories, ...],
    left_cycles: tuple[RecordedLeftCycle, ...],
    total_carrier_m: float,
    config: RecordedHandGuardedChopConfig,
) -> tuple[SynchronizedGuardCycle, ...]:
    """Retimes five knife down/up waves onto the continuous guard clock."""
    sample_counts = np.asarray([len(cycle.hand) for cycle in left_cycles], dtype=int)
    total_samples = int(np.sum(sample_counts))
    if total_samples < 2 or np.any(sample_counts < 2):
        raise ValueError("synchronized guard cycles require at least two samples")

    safe = cuts[0].descent[0].target_pose.copy()
    contact_z = float(cuts[0].descent[-1].target_pose[2, 3])
    if not np.isclose(
        left_cycles[-1].palm_targets[-1, 1, 3]
        - left_cycles[0].palm_targets[0, 1, 3],
        total_carrier_m,
        atol=1e-6,
    ):
        raise ValueError("left palm carrier does not match configured retreat")
    long_pads = np.asarray(
        [robot.sim.require_geom(f"left_finger{finger}_pad") for finger in range(2, 6)]
    )
    pad_y = []
    for cycle in left_cycles:
        for left_rad, hand_rad in zip(cycle.left, cycle.hand, strict=True):
            robot.sim.data.qpos[robot.sim.left.qpos_ids] = left_rad
            robot.sim.data.qpos[robot.sim.hand.qpos_ids] = hand_rad
            mujoco.mj_forward(robot.sim.model, robot.sim.data)
            pad_y.append(float(np.min(robot.sim.data.geom_xpos[long_pads, 1])))
    robot.sim.data.qpos[robot.sim.right.qpos_ids] = cuts[0].descent[0].joints_rad
    mujoco.mj_forward(robot.sim.model, robot.sim.data)
    blade_reference = robot.sim.require_site("right_blade_edge_bot")
    blade_to_tcp_y = float(
        robot.sim.data.site_xpos[blade_reference, 1]
        - cuts[0].descent[0].target_pose[1, 3]
    )
    targets = []
    phases = []
    cursor = 0
    for count in sample_counts:
        local = np.linspace(0.0, 1.0, int(count))
        down = local <= 0.5
        vertical = np.where(down, local * 2.0, (1.0 - local) * 2.0)
        for sample, depth in enumerate(vertical):
            target = safe.copy()
            target[1, 3] = (
                pad_y[cursor + sample]
                - config.target_lateral_spacing_m
                - blade_to_tcp_y
            )
            target[2, 3] += depth * (contact_z - safe[2, 3])
            targets.append(target)
            phases.append("CUT_DOWN" if down[sample] else "KNIFE_RETRACT")
        cursor += int(count)

    right = robot.right_kinematics.solve_path(
        targets,
        cuts[0].descent[0].joints_rad,
        max_joint_step_rad=config.maximum_arm_joint_step_rad,
    )
    synchronized = []
    cursor = 0
    for left_cycle, count in zip(left_cycles, sample_counts, strict=True):
        section = slice(cursor, cursor + int(count))
        synchronized.append(
            SynchronizedGuardCycle(
                right=right[section].copy(),
                left=left_cycle.left.copy(),
                hand=left_cycle.hand.copy(),
                knife_phases=tuple(phases[section]),
                palm_targets=left_cycle.palm_targets.copy(),
                knife_targets=np.asarray(targets[section]).copy(),
            )
        )
        cursor += int(count)
    return tuple(synchronized)


def _measure_plan_safety(
    robot: RightArmRobot,
    cycles: tuple[SynchronizedGuardCycle, ...],
    surface_z: float,
) -> tuple[float, float, float, float, float]:
    model, data = robot.sim.model, robot.sim.data
    board = robot.sim.require_geom("chopping_board")
    palm = robot.sim.require_body("left_palm_link")
    thumb = robot.sim.require_geom("left_finger1_pad")
    long_pads = np.asarray(
        [robot.sim.require_geom(f"left_finger{finger}_pad") for finger in range(2, 6)]
    )
    blade_reference = robot.sim.require_site("right_blade_edge_bot")
    thumb_radius = float(model.geom_size[thumb, 0])
    minimum_distance = np.inf
    maximum_penetration = 0.0
    minimum_thumb_clearance = np.inf
    minimum_lateral_spacing = np.inf
    maximum_lateral_spacing = -np.inf

    def measure(right_rad: np.ndarray, left_rad: np.ndarray, hand_rad: np.ndarray) -> None:
        nonlocal minimum_distance, maximum_penetration, minimum_thumb_clearance
        nonlocal minimum_lateral_spacing, maximum_lateral_spacing
        data.qpos[robot.sim.right.qpos_ids] = right_rad
        data.qpos[robot.sim.left.qpos_ids] = left_rad
        data.qpos[robot.sim.hand.qpos_ids] = hand_rad
        mujoco.mj_forward(model, data)
        minimum_distance = min(minimum_distance, _blade_hand_distance(robot))
        lateral_spacing = float(
            np.min(data.geom_xpos[long_pads, 1])
            - data.site_xpos[blade_reference, 1]
        )
        minimum_lateral_spacing = min(minimum_lateral_spacing, lateral_spacing)
        maximum_lateral_spacing = max(maximum_lateral_spacing, lateral_spacing)
        minimum_thumb_clearance = min(
            minimum_thumb_clearance,
            float(data.geom_xpos[thumb, 2] - thumb_radius - surface_z),
        )
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
            hand_geom = (
                geom2
                if geom1 == board
                else geom1 if geom2 == board else None
            )
            if hand_geom is not None and _geom_belongs_to_body_tree(
                robot, hand_geom, palm
            ):
                maximum_penetration = max(
                    maximum_penetration, max(0.0, -float(contact.dist))
                )

    for cycle in cycles:
        for right_rad, left_rad, hand_rad in zip(
            cycle.right, cycle.left, cycle.hand, strict=True
        ):
            measure(right_rad, left_rad, hand_rad)
    return (
        float(minimum_distance),
        float(maximum_penetration),
        float(minimum_thumb_clearance),
        float(minimum_lateral_spacing),
        float(maximum_lateral_spacing),
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

        for index, synchronized in enumerate(
            plan.synchronized_cycles, start=1
        ):
            events.append((index, "SYNC_CYCLE_START"))
            for right_rad, left_rad, hand_rad, knife_phase in zip(
                synchronized.right,
                synchronized.left,
                synchronized.hand,
                synchronized.knife_phases,
                strict=True,
            ):
                execute(
                    RecordedHandGuardedChopPhase[knife_phase],
                    right_rad,
                    left_rad,
                    hand_rad,
                    index,
                )
            completed_cycles += 1
            completed_cuts += 1
            events.append((index, "SYNC_CYCLE_COMPLETE"))
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
            minimum_lateral_spacing_m=plan.minimum_lateral_spacing_m,
            maximum_lateral_spacing_m=plan.maximum_lateral_spacing_m,
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
            minimum_lateral_spacing_m=(
                float("nan") if plan is None else plan.minimum_lateral_spacing_m
            ),
            maximum_lateral_spacing_m=(
                float("nan") if plan is None else plan.maximum_lateral_spacing_m
            ),
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
