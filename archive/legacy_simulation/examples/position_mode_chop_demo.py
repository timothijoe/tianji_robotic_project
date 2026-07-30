#!/usr/bin/env python3
"""MuJoCo position-mode relative Cartesian chopping demo.

This demo mirrors the CLI style of ``showcase_ik_cart_impedance.py`` while
executing with SDK-style POSITION commands. It is intentionally MuJoCo-only;
real-machine scripts live under ``real_robot_debug/``.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Sequence

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "src" / "twin_core"))
sys.path.insert(0, str(ROOT / "src" / "twin_description"))
sys.path.insert(0, str(ROOT / "src" / "twin_mujoco"))

from twin_control.chopping import RIGHT_CHOPPING_HOME_RAD, TwinRobotChopper
from twin_control.controller import JointImpedanceParams
from twin_control.robot import LEFT_HOME_RAD
from twin_control.sdk_compat import create_ik_param, create_kine, create_robot
from twin_control.trail import _init_sphere_geom
from twin_control.trajectory import cartesian_minimum_jerk_trajectory


POSITION_BASE_K = (35.0, 35.0, 30.0, 24.0, 16.0, 10.0, 8.0)
POSITION_BASE_D = (7.0, 7.0, 6.0, 5.0, 3.5, 2.5, 2.0)


def apply_position_gain_scaling(
    internal_robot,
    *,
    k_scale: float,
    d_scale: float,
    torque_rate_limit: float | None = None,
) -> JointImpedanceParams:
    """Apply stronger MuJoCo joint-position tracking gains for this demo."""
    k = tuple(float(v) * max(0.0, float(k_scale)) for v in POSITION_BASE_K)
    d = tuple(float(v) * max(0.0, float(d_scale)) for v in POSITION_BASE_D)
    params = JointImpedanceParams(stiffness=k, damping=d)
    internal_robot._controller.set_joint_impedance_params(params)
    if torque_rate_limit is not None:
        internal_robot._controller._rate_limit = max(0.0, float(torque_rate_limit))
    return params


def max_abs_joint_error_deg(target_joints_deg: Sequence[float], actual_joints_deg: Sequence[float]) -> float:
    target = np.asarray(target_joints_deg, dtype=float).reshape(7)
    actual = np.asarray(actual_joints_deg, dtype=float).reshape(7)
    return float(np.max(np.abs(target - actual)))


def needs_joint_settle(
    target_joints_deg: Sequence[float],
    actual_joints_deg: Sequence[float],
    *,
    tolerance_deg: float,
) -> bool:
    return max_abs_joint_error_deg(target_joints_deg, actual_joints_deg) > max(0.0, float(tolerance_deg))


def build_relative_tcp_targets(
    *,
    start_pos_m: Sequence[float],
    dz_mm: float,
    hold_s: float,
    control_hz: float,
    cycles: int,
    lateral: bool,
    lateral_mm: float,
) -> list[np.ndarray]:
    """Build per-control-step TCP targets for down/up chopping motion.

    Targets are expressed in metres in the MuJoCo world/base frame. Each cycle
    descends from the starting Z, retracts to the starting Z, then shifts in Y
    during the retract portion when lateral motion is enabled.
    """
    start = np.asarray(start_pos_m, dtype=float).reshape(3)
    steps_per_cycle = max(3, int(round(float(hold_s) * float(control_hz))))
    cycle_count = max(1, int(cycles))
    direction = -1.0 if float(dz_mm) <= 0.0 else 1.0
    dz_m = abs(float(dz_mm)) * 0.001
    lateral_step_m = abs(float(lateral_mm)) * 0.001 if lateral else 0.0

    targets: list[np.ndarray] = []
    for step_index in range(steps_per_cycle * cycle_count):
        cycle_index = min(step_index // steps_per_cycle, cycle_count - 1)
        cycle_step = step_index - cycle_index * steps_per_cycle
        z_progress, retract_progress = _cut_cycle_progress(cycle_step, steps_per_cycle)
        target = start.copy()
        target[2] = start[2] + direction * dz_m * z_progress
        target[1] = start[1] + lateral_step_m * (float(cycle_index) + retract_progress)
        targets.append(target)
    return targets


def build_blade_aligned_pose_targets(
    *,
    chopper,
    start_tip_m: Sequence[float],
    dz_mm: float,
    hold_s: float,
    control_hz: float,
    cycles: int,
    lateral: bool,
    lateral_mm: float,
    contact_clearance_mm: float = 30.0,
) -> list[np.ndarray]:
    """Build fixed-orientation TCP pose targets from the chopping blade geometry.

    ``dz_mm`` is interpreted as the requested vertical stroke from the initial
    tip position.  The lowest target is clamped above the board by
    ``contact_clearance_mm`` so position mode does not push into contact.
    """
    start = np.asarray(start_tip_m, dtype=float).reshape(3)
    geometry = chopper._blade_geometry()
    target_rotation = np.asarray(chopper._horizontal_blade_rotation(geometry), dtype=float).reshape(3, 3)
    board_top = float(chopper._board_top())
    geometry_tip = np.asarray(getattr(geometry, "tip_position", start), dtype=float).reshape(3)
    safe_tip = start.copy()
    requested_descend_tip = safe_tip.copy()
    requested_descend_tip[2] += float(dz_mm) * 0.001
    min_clearance_m = max(0.0, float(contact_clearance_mm)) * 0.001
    min_allowed_tip = np.asarray(
        chopper._tip_target_for_blade_clearance(
            geometry_tip,
            target_rotation,
            geometry,
            board_top,
            min_clearance_m,
        ),
        dtype=float,
    ).reshape(3)
    descend_tip = requested_descend_tip.copy()
    if float(dz_mm) < 0.0 and descend_tip[2] < min_allowed_tip[2]:
        descend_tip[2] = min_allowed_tip[2]

    steps_per_cycle = max(3, int(round(float(hold_s) * float(control_hz))))
    descent_steps = max(1, steps_per_cycle // 2)
    retract_steps = max(1, steps_per_cycle - descent_steps)
    cycle_count = max(1, int(cycles))
    lateral_step_m = abs(float(lateral_mm)) * 0.001 if lateral else 0.0

    poses: list[np.ndarray] = []
    current = start

    def append_segment(target: np.ndarray, steps: int) -> None:
        nonlocal current
        trajectory = cartesian_minimum_jerk_trajectory(
            start_pos=current,
            target_pos=target,
            rotation=target_rotation,
            steps=steps,
        )
        poses.extend(point.pose_matrix.copy() for point in trajectory)
        current = np.asarray(target, dtype=float).reshape(3)

    for cycle_index in range(cycle_count):
        offset = np.array((0.0, lateral_step_m * float(cycle_index), 0.0), dtype=float)
        cycle_safe = safe_tip + offset
        cycle_descend = descend_tip + offset
        append_segment(cycle_safe, descent_steps)
        append_segment(cycle_descend, descent_steps)
        append_segment(cycle_safe, retract_steps)


    return poses



def _blade_aligned_tip_targets(
    *,
    chopper,
    start_tip_m: Sequence[float],
    dz_mm: float,
    contact_clearance_mm: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    start = np.asarray(start_tip_m, dtype=float).reshape(3)
    geometry = chopper._blade_geometry()
    target_rotation = np.asarray(chopper._horizontal_blade_rotation(geometry), dtype=float).reshape(3, 3)
    board_top = float(chopper._board_top())
    geometry_tip = np.asarray(getattr(geometry, "tip_position", start), dtype=float).reshape(3)

    safe_tip = start.copy()
    descend_tip = safe_tip.copy()
    descend_tip[2] += float(dz_mm) * 0.001
    min_clearance_m = max(0.0, float(contact_clearance_mm)) * 0.001
    min_allowed_tip = np.asarray(
        chopper._tip_target_for_blade_clearance(
            geometry_tip,
            target_rotation,
            geometry,
            board_top,
            min_clearance_m,
        ),
        dtype=float,
    ).reshape(3)
    if float(dz_mm) < 0.0 and descend_tip[2] < min_allowed_tip[2]:
        descend_tip[2] = min_allowed_tip[2]
    return safe_tip, descend_tip, target_rotation


def _distribute_steps(total_steps: int, command_count: int) -> list[int]:
    count = max(1, int(command_count))
    total = max(count, int(total_steps))
    base = total // count
    remainder = total % count
    return [base + (1 if index < remainder else 0) for index in range(count)]


def build_blade_aligned_waypoint_targets(
    *,
    chopper,
    start_tip_m: Sequence[float],
    dz_mm: float,
    hold_s: float,
    control_hz: float,
    cycles: int,
    lateral: bool,
    lateral_mm: float,
    contact_clearance_mm: float = 30.0,
    waypoints_per_segment: int = 8,
) -> list[tuple[np.ndarray, int]]:
    """Build medium-density minimum-jerk waypoints for position mode.

    This keeps the Cartesian trajectory shape close to the impedance demo while
    avoiding the dense-point slowdown of commanding every control-cycle sample.
    """
    safe_tip, descend_tip, target_rotation = _blade_aligned_tip_targets(
        chopper=chopper,
        start_tip_m=start_tip_m,
        dz_mm=dz_mm,
        contact_clearance_mm=contact_clearance_mm,
    )
    steps_per_cycle = max(2, int(round(float(hold_s) * float(control_hz))))
    descent_steps = max(1, steps_per_cycle // 2)
    retract_steps = max(1, steps_per_cycle - descent_steps)
    waypoint_count = max(2, int(waypoints_per_segment))
    lateral_step_m = abs(float(lateral_mm)) * 0.001 if lateral else 0.0

    commands: list[tuple[np.ndarray, int]] = []
    for cycle_index in range(max(1, int(cycles))):
        offset = np.array((0.0, lateral_step_m * float(cycle_index), 0.0), dtype=float)
        cycle_safe = safe_tip + offset
        cycle_descend = descend_tip + offset
        segments = (
            (cycle_safe, cycle_descend, descent_steps),
            (cycle_descend, cycle_safe, retract_steps),
        )
        for start, target, segment_steps in segments:
            trajectory = cartesian_minimum_jerk_trajectory(
                start_pos=start,
                target_pos=target,
                rotation=target_rotation,
                steps=waypoint_count,
            )
            holds = _distribute_steps(segment_steps, waypoint_count)
            commands.extend((point.pose_matrix.copy(), hold) for point, hold in zip(trajectory, holds))
    return commands

def build_blade_aligned_endpoint_targets(
    *,
    chopper,
    start_tip_m: Sequence[float],
    dz_mm: float,
    hold_s: float,
    control_hz: float,
    cycles: int,
    lateral: bool,
    lateral_mm: float,
    contact_clearance_mm: float = 30.0,
) -> list[tuple[np.ndarray, int]]:
    """Build sparse endpoint targets for position mode tracking.

    Each cycle sends one descend endpoint and one retract endpoint. The caller
    holds each endpoint for the returned number of control steps so the joint
    position controller can chase a stable target instead of many tiny ones.
    """
    start = np.asarray(start_tip_m, dtype=float).reshape(3)
    geometry = chopper._blade_geometry()
    target_rotation = np.asarray(chopper._horizontal_blade_rotation(geometry), dtype=float).reshape(3, 3)
    board_top = float(chopper._board_top())
    geometry_tip = np.asarray(getattr(geometry, "tip_position", start), dtype=float).reshape(3)

    safe_tip = start.copy()
    descend_tip = safe_tip.copy()
    descend_tip[2] += float(dz_mm) * 0.001
    min_clearance_m = max(0.0, float(contact_clearance_mm)) * 0.001
    min_allowed_tip = np.asarray(
        chopper._tip_target_for_blade_clearance(
            geometry_tip,
            target_rotation,
            geometry,
            board_top,
            min_clearance_m,
        ),
        dtype=float,
    ).reshape(3)
    if float(dz_mm) < 0.0 and descend_tip[2] < min_allowed_tip[2]:
        descend_tip[2] = min_allowed_tip[2]

    steps_per_cycle = max(2, int(round(float(hold_s) * float(control_hz))))
    descend_steps = max(1, steps_per_cycle // 2)
    retract_steps = max(1, steps_per_cycle - descend_steps)
    lateral_step_m = abs(float(lateral_mm)) * 0.001 if lateral else 0.0

    endpoints: list[tuple[np.ndarray, int]] = []
    for cycle_index in range(max(1, int(cycles))):
        offset = np.array((0.0, lateral_step_m * float(cycle_index), 0.0), dtype=float)
        for target, steps in ((descend_tip + offset, descend_steps), (safe_tip + offset, retract_steps)):
            pose = np.eye(4)
            pose[:3, :3] = target_rotation
            pose[:3, 3] = target
            endpoints.append((pose, steps))
    return endpoints


def run_demo(
    *,
    backend: str = "mujoco",
    arm: str = "B",
    viewer: bool = False,
    realtime: bool = False,
    control_hz: float = 250.0,
    dz_mm: float = -20.0,
    hold_s: float = 2.0,
    cycles: int = 5,
    lateral: bool = False,
    lateral_mm: float = 10.0,
    viewer_trace: bool = False,
    viewer_trace_stride: int = 10,
    viewer_trace_size_mm: float = 6.0,
    plot: Path | None = None,
    csv_path: Path | None = None,
    position_substeps: int = 20,
    position_k_scale: float = 8.0,
    position_d_scale: float = 5.0,
    torque_rate_limit: float = 2000.0,
    settle_tolerance_deg: float = 5.0,
    settle_timeout_s: float = 0.15,
    contact_clearance_mm: float = 30.0,
    trajectory_mode: str = "waypoint",
    waypoints_per_segment: int = 8,
) -> dict:
    """Run the MuJoCo position-mode relative Cartesian chopping demo."""
    if backend != "mujoco":
        raise ValueError("position_mode_chop_demo is MuJoCo-only; use real_robot_debug for real hardware")
    if arm.upper() != "B":
        raise ValueError("position_mode_chop_demo currently supports MuJoCo B/right arm only")

    robot = create_robot(
        "mujoco",
        arm="B",
        viewer=viewer,
        realtime=realtime,
        control_hz=control_hz,
        tcp_site_name="right_tool_tip_site",
    )
    kine = create_kine("mujoco", arm_type=1, tcp_site_name="right_tool_tip_site")
    robot.connect("mujoco")
    try:
        config = kine.load_config(arm_type=1, config_path="ccs_m6_40.MvKDCfg")
        kine.initial_kine(
            robot_type=config["TYPE"][1],
            dh=config["DH"][1],
            pnva=config["PNVA"][1],
            j67=config["BD"][1],
        )

        internal = robot._require_robot()
        internal.runtime.reset()
        internal.runtime.set_arm_positions("left", LEFT_HOME_RAD)
        internal.runtime.set_arm_positions("right", RIGHT_CHOPPING_HOME_RAD)
        if viewer_trace:
            if not viewer or internal._viewer is None:
                print("viewer_trace: disabled because --viewer is not active")
            _clear_viewer_trace(internal._viewer)
        trace_size_m = max(0.001, float(viewer_trace_size_mm) * 0.001)

        data = robot.subscribe(None)
        if data is None:
            raise RuntimeError("subscribe failed after reset")
        arm_index = 1
        reference_joints = data["outputs"][arm_index]["fb_joint_pos"]
        start_pos_m, _start_rot = internal._arm.site_pose("right_tool_tip_site")
        chopper = TwinRobotChopper(internal)
        if trajectory_mode == "waypoint":
            target_commands = build_blade_aligned_waypoint_targets(
                chopper=chopper,
                start_tip_m=start_pos_m,
                dz_mm=dz_mm,
                hold_s=hold_s,
                control_hz=control_hz,
                cycles=cycles,
                lateral=lateral,
                lateral_mm=lateral_mm,
                contact_clearance_mm=contact_clearance_mm,
                waypoints_per_segment=waypoints_per_segment,
            )
        elif trajectory_mode == "endpoint":
            target_commands = build_blade_aligned_endpoint_targets(
                chopper=chopper,
                start_tip_m=start_pos_m,
                dz_mm=dz_mm,
                hold_s=hold_s,
                control_hz=control_hz,
                cycles=cycles,
                lateral=lateral,
                lateral_mm=lateral_mm,
                contact_clearance_mm=contact_clearance_mm,
            )
        elif trajectory_mode == "dense":
            target_commands = [
                (pose, max(1, int(position_substeps)))
                for pose in build_blade_aligned_pose_targets(
                    chopper=chopper,
                    start_tip_m=start_pos_m,
                    dz_mm=dz_mm,
                    hold_s=hold_s,
                    control_hz=control_hz,
                    cycles=cycles,
                    lateral=lateral,
                    lateral_mm=lateral_mm,
                    contact_clearance_mm=contact_clearance_mm,
                )
            ]
        else:
            raise ValueError("trajectory_mode must be 'waypoint', 'endpoint', or 'dense'")

        robot.set_position_state("B", velRatio=30, AccRatio=30)
        position_gains = apply_position_gain_scaling(
            internal,
            k_scale=position_k_scale,
            d_scale=position_d_scale,
            torque_rate_limit=torque_rate_limit,
        )
        trace_stride = max(1, int(viewer_trace_stride))
        substeps_per_target = max(1, int(position_substeps))
        ref_for_ik = list(reference_joints)
        target_joints = list(reference_joints)
        ik_success = True
        target_x_trace_mm: list[float] = []
        target_y_trace_mm: list[float] = []
        target_z_trace_mm: list[float] = []
        actual_x_trace_mm: list[float] = []
        actual_y_trace_mm: list[float] = []
        actual_z_trace_mm: list[float] = []
        viewer_trace_markers = 0
        settle_steps = 0
        final_joint_error_deg = 0.0

        sample_index = 0
        for command_index, (target_pose_m, hold_steps) in enumerate(target_commands):
            target_pos_m = np.asarray(target_pose_m[:3, 3], dtype=float).reshape(3)
            target_pose_sdk = np.asarray(target_pose_m, dtype=float).copy()
            target_pose_sdk[:3, 3] = target_pos_m * 1000.0
            sp = create_ik_param("mujoco")
            sp.set_input_ik_target_tcp(kine.mat4x4_to_mat1x16(target_pose_sdk))
            sp.set_input_ik_ref_joint(ref_for_ik)
            sp.set_input_ik_zsp_type(0)
            ik = kine.ik(sp)
            ik_success = ik_success and ik.get_output_result_num() >= 1
            target_joints = list(ik.get_output_ret_joint())
            ref_for_ik = target_joints

            robot.set_joint_position_cmd("B", target_joints)
            settle_budget_steps = max(0, int(round(float(settle_timeout_s) * float(control_hz))))
            command_steps = max(1, int(hold_steps))
            extra_steps = 0
            while True:
                is_base_step = sample_index < sum(max(1, int(steps)) for _, steps in target_commands)
                is_last_base_step = extra_steps == 0 and command_steps == 1
                robot.step(viewer_sync=False)
                command_steps -= 1
                actual_pos_m, _ = internal._arm.site_pose("right_tool_tip_site")
                data = robot.subscribe(None)
                final_joint_error_deg = max_abs_joint_error_deg(target_joints, data["outputs"][arm_index]["fb_joint_pos"])
                if viewer_trace and sample_index % trace_stride == 0:
                    if _append_viewer_trace_marker(internal._viewer, target_pos_m, (0.0, 0.95, 1.0, 1.0), trace_size_m):
                        viewer_trace_markers += 1
                    if _append_viewer_trace_marker(internal._viewer, actual_pos_m, (1.0, 0.20, 0.0, 1.0), trace_size_m * 0.8):
                        viewer_trace_markers += 1
                target_x_trace_mm.append(float(target_pos_m[0] * 1000.0))
                target_y_trace_mm.append(float(target_pos_m[1] * 1000.0))
                target_z_trace_mm.append(float(target_pos_m[2] * 1000.0))
                actual_x_trace_mm.append(float(actual_pos_m[0] * 1000.0))
                actual_y_trace_mm.append(float(actual_pos_m[1] * 1000.0))
                actual_z_trace_mm.append(float(actual_pos_m[2] * 1000.0))
                sample_index += 1

                if command_steps > 0:
                    continue
                if not needs_joint_settle(target_joints, data["outputs"][arm_index]["fb_joint_pos"], tolerance_deg=settle_tolerance_deg):
                    break
                if extra_steps >= settle_budget_steps:
                    break
                extra_steps += 1
                settle_steps += 1

            if viewer and internal._viewer is not None:
                if getattr(internal, "_trail", None) is not None:
                    internal._trail.render()
                internal._viewer.sync()

        if csv_path is not None:
            internal.write_csv(csv_path)
        result = {
            "ik_success": ik_success,
            "execution_mode": "POSITION_IK_OSCILLATION",
            "motion_steps": len(target_commands),
            "sim_steps": len(target_z_trace_mm),
            "position_substeps": substeps_per_target,
            "position_k_scale": float(position_k_scale),
            "position_d_scale": float(position_d_scale),
            "torque_rate_limit": float(torque_rate_limit),
            "settle_tolerance_deg": float(settle_tolerance_deg),
            "settle_timeout_s": float(settle_timeout_s),
            "settle_steps": int(settle_steps),
            "final_joint_error_deg": float(final_joint_error_deg),
            "position_stiffness": tuple(float(v) for v in position_gains.stiffness),
            "position_damping": tuple(float(v) for v in position_gains.damping),
            "contact_clearance_mm": float(contact_clearance_mm),
            "trajectory_mode": trajectory_mode,
            "waypoints_per_segment": int(waypoints_per_segment),
            "trajectory_source": {
                "waypoint": "BLADE_ALIGNED_WAYPOINT_AIR_CHOP",
                "endpoint": "BLADE_ALIGNED_ENDPOINT_AIR_CHOP",
                "dense": "BLADE_ALIGNED_DENSE_AIR_CHOP",
            }[trajectory_mode],
            "lateral": bool(lateral),
            "lateral_mm": float(lateral_mm),
            "target_joints": target_joints,
            "feedback": data["outputs"][arm_index],
            "viewer_trace_markers": viewer_trace_markers,
            "viewer_trace_size_mm": float(viewer_trace_size_mm),
            "target_x_trace_mm": target_x_trace_mm,
            "actual_x_trace_mm": actual_x_trace_mm,
            "target_y_trace_mm": target_y_trace_mm,
            "actual_y_trace_mm": actual_y_trace_mm,
            "target_z_trace_mm": target_z_trace_mm,
            "actual_z_trace_mm": actual_z_trace_mm,
        }
        if plot is not None:
            _write_trace_plot(result, plot, control_hz)
        if viewer:
            print("Viewer open - press Ctrl+C to exit.")
            internal.hold_viewer_open(sync_hz=30.0)
        return result
    finally:
        robot.release_robot()


def _cut_cycle_progress(cycle_step: int, steps_per_cycle: int) -> tuple[float, float]:
    descent_steps = max(1, int(steps_per_cycle) // 2)
    retract_steps = max(1, int(steps_per_cycle) - descent_steps)
    if int(cycle_step) < descent_steps:
        denominator = max(descent_steps - 1, 1)
        return float(cycle_step) / float(denominator), 0.0

    retract_step = min(int(cycle_step) - descent_steps, retract_steps - 1)
    denominator = max(retract_steps - 1, 1)
    retract_progress = float(retract_step) / float(denominator)
    return 1.0 - retract_progress, retract_progress


def _clear_viewer_trace(viewer) -> None:
    if viewer is None or getattr(viewer, "user_scn", None) is None:
        return
    with viewer.lock():
        viewer.user_scn.ngeom = 0


def _append_viewer_trace_marker(viewer, position_m, rgba, size_m: float) -> bool:
    if viewer is None or getattr(viewer, "user_scn", None) is None:
        return False
    scn = viewer.user_scn
    with viewer.lock():
        if scn.ngeom >= scn.maxgeom:
            return False
        geom = scn.geoms[scn.ngeom]
        _init_sphere_geom(
            geom,
            np.asarray(position_m, dtype=float).reshape(3),
            float(size_m),
            np.asarray(rgba, dtype=float),
        )
        scn.ngeom += 1
        return True


def _write_trace_plot(result: dict, path: Path, control_hz: float) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    target_z = np.asarray(result["target_z_trace_mm"], dtype=float)
    actual_z = np.asarray(result["actual_z_trace_mm"], dtype=float)
    count = min(target_z.size, actual_z.size)
    if count == 0:
        raise ValueError("trajectory trace is empty")
    time_s = np.arange(count, dtype=float) / max(float(control_hz), 1.0)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(9, 4.8), dpi=140)
    ax.plot(time_s, target_z[:count], label="target TCP Z", linewidth=2.0)
    ax.plot(time_s, actual_z[:count], label="actual TCP Z", linewidth=1.8)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("TCP Z (mm)")
    ax.set_title("Position-mode IK blade-aligned chopping trajectory")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("mujoco",), default="mujoco")
    parser.add_argument("--arm", choices=("B",), default="B")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--realtime", action="store_true")
    parser.add_argument("--control-hz", type=float, default=250.0)
    parser.add_argument("--dz-mm", type=float, default=-20.0)
    parser.add_argument("--hold-s", type=float, default=2.0)
    parser.add_argument("--cycles", type=int, default=5)
    parser.add_argument("--lateral", action="store_true")
    parser.add_argument("--lateral-mm", type=float, default=10.0)
    parser.add_argument("--viewer-trace", action="store_true")
    parser.add_argument("--viewer-trace-stride", type=int, default=10)
    parser.add_argument("--viewer-trace-size-mm", type=float, default=6.0, help="MuJoCo viewer trace sphere radius in mm")
    parser.add_argument("--position-substeps", type=int, default=20, help="MuJoCo sim steps to hold each IK position target")
    parser.add_argument("--position-k-scale", type=float, default=8.0, help="Scale MuJoCo position-mode joint stiffness")
    parser.add_argument("--position-d-scale", type=float, default=5.0, help="Scale MuJoCo position-mode joint damping")
    parser.add_argument("--torque-rate-limit", type=float, default=2000.0, help="MuJoCo torque slew-rate limit in Nm/s for position tracking")
    parser.add_argument("--settle-tolerance-deg", type=float, default=5.0, help="Extra tracking waits until max joint error is below this")
    parser.add_argument("--settle-timeout-s", type=float, default=0.15, help="Max extra settle time per waypoint")
    parser.add_argument("--contact-clearance-mm", type=float, default=30.0, help="Minimum blade clearance above the board in position mode")
    parser.add_argument("--trajectory-mode", choices=("waypoint", "endpoint", "dense"), default="waypoint", help="Use sampled waypoints, sparse held endpoints, or dense IK points")
    parser.add_argument("--waypoints-per-segment", type=int, default=8, help="Waypoint mode samples per descend/retract segment")
    parser.add_argument("--plot", type=Path, default=None, help="Save target/actual TCP Z trace plot as PNG")
    parser.add_argument("--csv", type=Path, default=None, help="Save internal TwinRobot CSV log")
    parser.add_argument("--execution-mode", choices=("POSITION_IK_OSCILLATION",), default="POSITION_IK_OSCILLATION")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    result = run_demo(
        backend=args.backend,
        arm=args.arm,
        viewer=args.viewer and not args.headless,
        realtime=args.realtime,
        control_hz=args.control_hz,
        dz_mm=args.dz_mm,
        hold_s=args.hold_s,
        cycles=args.cycles,
        lateral=args.lateral,
        lateral_mm=args.lateral_mm,
        viewer_trace=args.viewer_trace,
        viewer_trace_stride=args.viewer_trace_stride,
        viewer_trace_size_mm=args.viewer_trace_size_mm,
        plot=args.plot,
        csv_path=args.csv,
        position_substeps=args.position_substeps,
        position_k_scale=args.position_k_scale,
        position_d_scale=args.position_d_scale,
        torque_rate_limit=args.torque_rate_limit,
        settle_tolerance_deg=args.settle_tolerance_deg,
        settle_timeout_s=args.settle_timeout_s,
        contact_clearance_mm=args.contact_clearance_mm,
        trajectory_mode=args.trajectory_mode,
        waypoints_per_segment=args.waypoints_per_segment,
    )
    if args.plot is not None:
        print("plot:", args.plot)
    if args.csv is not None:
        print("csv:", args.csv)
    print("ik_success:", result["ik_success"])
    print("execution_mode:", result["execution_mode"])
    print("motion_steps:", result["motion_steps"])
    print("sim_steps:", result["sim_steps"])
    print("position_substeps:", result["position_substeps"])
    print("position_k_scale:", result["position_k_scale"])
    print("position_d_scale:", result["position_d_scale"])
    print("torque_rate_limit:", result["torque_rate_limit"])
    print("settle_tolerance_deg:", result["settle_tolerance_deg"])
    print("settle_timeout_s:", result["settle_timeout_s"])
    print("settle_steps:", result["settle_steps"])
    print("final_joint_error_deg:", round(result["final_joint_error_deg"], 3))
    print("contact_clearance_mm:", result["contact_clearance_mm"])
    print("trajectory_mode:", result["trajectory_mode"])
    print("waypoints_per_segment:", result["waypoints_per_segment"])
    print("trajectory_source:", result["trajectory_source"])
    print("viewer_trace_markers:", result["viewer_trace_markers"])
    print("viewer_trace_size_mm:", result["viewer_trace_size_mm"])
    print("target_z_range_mm:", [round(min(result["target_z_trace_mm"]), 3), round(max(result["target_z_trace_mm"]), 3)])
    print("actual_z_range_mm:", [round(min(result["actual_z_trace_mm"]), 3), round(max(result["actual_z_trace_mm"]), 3)])
    print("final_target_actual_z_mm:", [round(result["target_z_trace_mm"][-1], 3), round(result["actual_z_trace_mm"][-1], 3)])
    print("lateral:", result["lateral"])
    print("target_joints:", [round(v, 3) for v in result["target_joints"]])
    print("feedback_joints:", result["feedback"]["fb_joint_pos"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
