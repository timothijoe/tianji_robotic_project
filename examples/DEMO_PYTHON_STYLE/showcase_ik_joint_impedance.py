#!/usr/bin/env python3
"""SDK-style IK + joint impedance demo for MuJoCo or real backend."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "src" / "twin_core"))
sys.path.insert(0, str(ROOT / "src" / "twin_description"))
sys.path.insert(0, str(ROOT / "src" / "twin_mujoco"))

from twin_control.chopping import RIGHT_CHOPPING_HOME_RAD
from twin_control.robot import LEFT_HOME_RAD
from twin_control.sdk_compat import create_ik_param, create_kine, create_robot
from twin_control.trail import _init_sphere_geom


READY_B = [-75.627, -67.572, 52.390, -124.574, -90.421, 42.952, 41.374]
SDK_JOINT_K = [2.0, 2.0, 2.0, 1.6, 5.0, 5.0, 5.0]
SDK_JOINT_D = [0.3, 0.3, 0.3, 0.2, 0.5, 0.5, 0.5]


def run_demo(
    *,
    backend: str = "mujoco",
    arm: str = "B",
    robot_ip: str = "mujoco",
    sdk_root: str | Path | None = None,
    viewer: bool = False,
    realtime: bool = False,
    control_hz: float = 250.0,
    dz_mm: float = -20.0,
    hold_s: float = 1.0,
    cycles: int = 1,
    viewer_trace: bool = False,
    viewer_trace_stride: int = 20,
    lateral: bool = False,
    lateral_mm: float = 30.0,
    joint_k: list[float] | None = None,
    joint_d: list[float] | None = None,
    enable_self_collision: bool = False,
    velocity_ff: float = 0.8,
    tracking_mode: str = "trajectory",
    path_speed_mm_s: float = 80.0,
    lookahead_mm: float = 8.0,
    path_tolerance_mm: float = 6.0,
) -> dict:
    """Run a TCP chop trajectory by solving IK and tracking joints with impedance."""
    arm_type = 0 if arm.upper() == "A" else 1
    robot = create_robot(
        backend,
        arm=arm,
        viewer=viewer,
        realtime=realtime,
        control_hz=control_hz,
        tcp_site_name="right_tool_tip_site" if arm.upper() == "B" else None,
        sdk_root=sdk_root,
    )
    kine = create_kine(
        backend,
        arm_type=arm_type,
        tcp_site_name="right_tool_tip_site" if arm.upper() == "B" else None,
        sdk_root=sdk_root,
    )

    robot.connect(robot_ip)
    try:
        config = kine.load_config(arm_type=arm_type, config_path="ccs_m6_40.MvKDCfg")
        kine.initial_kine(
            robot_type=config["TYPE"][arm_type],
            dh=config["DH"][arm_type],
            pnva=config["PNVA"][arm_type],
            j67=config["BD"][arm_type],
        )

        if backend == "mujoco" and arm.upper() == "B":
            return _run_mujoco_ik_joint_impedance(
                robot=robot,
                kine=kine,
                arm=arm,
                arm_index=1,
                control_hz=control_hz,
                dz_mm=dz_mm,
                hold_s=hold_s,
                cycles=cycles,
                viewer=viewer,
                viewer_trace=viewer_trace,
                viewer_trace_stride=viewer_trace_stride,
                lateral=lateral,
                lateral_mm=lateral_mm,
                joint_k=joint_k or SDK_JOINT_K,
                joint_d=joint_d or SDK_JOINT_D,
                enable_self_collision=enable_self_collision,
                velocity_ff=velocity_ff,
                tracking_mode=tracking_mode,
                path_speed_mm_s=path_speed_mm_s,
                lookahead_mm=lookahead_mm,
                path_tolerance_mm=path_tolerance_mm,
                sdk_root=sdk_root,
            )

        return _run_sdk_endpoint_ik_joint_impedance(
            robot=robot,
            kine=kine,
            arm=arm,
            arm_index=arm_type,
            control_hz=control_hz,
            dz_mm=dz_mm,
            hold_s=hold_s,
            joint_k=joint_k or SDK_JOINT_K,
            joint_d=joint_d or SDK_JOINT_D,
            backend=backend,
            sdk_root=sdk_root,
            viewer=viewer,
        )
    finally:
        robot.release_robot()


def _run_mujoco_ik_joint_impedance(
    *,
    robot,
    kine,
    arm: str,
    arm_index: int,
    control_hz: float,
    dz_mm: float,
    hold_s: float,
    cycles: int,
    viewer: bool,
    viewer_trace: bool,
    viewer_trace_stride: int,
    lateral: bool,
    lateral_mm: float,
    joint_k: list[float],
    joint_d: list[float],
    enable_self_collision: bool,
    velocity_ff: float,
    tracking_mode: str,
    path_speed_mm_s: float,
    lookahead_mm: float,
    path_tolerance_mm: float,
    sdk_root: str | Path | None,
) -> dict:
    internal = robot._require_robot()
    if not enable_self_collision:
        _disable_robot_collision_geoms(internal)
    internal.runtime.reset()
    internal.runtime.set_arm_positions("left", LEFT_HOME_RAD)
    internal.runtime.set_arm_positions("right", RIGHT_CHOPPING_HOME_RAD)

    robot.set_imp_joint_state(arm, velRatio=40, AccRatio=40, K=joint_k, D=joint_d)

    data = robot.subscribe(None)
    if data is None:
        raise RuntimeError("subscribe failed after chopping-home reset")
    reference_joints = data["outputs"][arm_index]["fb_joint_pos"]
    chopping_home_joints = np.rad2deg(RIGHT_CHOPPING_HOME_RAD).tolist()

    start_pos_m, start_rot = internal._arm.site_pose("right_tool_tip_site")
    trace_stride = max(1, int(viewer_trace_stride))
    if viewer_trace:
        _clear_viewer_trace(internal._viewer)
    cycle_count = max(1, int(cycles))
    steps_per_cycle = max(3, int(round(float(hold_s) * float(control_hz))))
    motion_steps = steps_per_cycle * cycle_count
    amplitude_m = abs(float(dz_mm)) * 0.001
    lateral_step_m = abs(float(lateral_mm)) * 0.001 if lateral else 0.0

    target_x_trace_mm: list[float] = []
    actual_x_trace_mm: list[float] = []
    target_y_trace_mm: list[float] = []
    actual_y_trace_mm: list[float] = []
    target_z_trace_mm: list[float] = []
    actual_z_trace_mm: list[float] = []
    target_joint_trace_deg: list[list[float]] = []
    ref_for_ik = list(reference_joints)
    target_joints = list(reference_joints)
    previous_target_joints = np.asarray(reference_joints, dtype=float)
    ik_success = True
    stopped_reason = "completed"
    ik_failed_step = None
    ik_failed_target_mm = None

    preflight_ref = list(reference_joints)
    for target_index, target_pos_m in enumerate(_preflight_targets(
        start_pos_m=start_pos_m,
        start_rot=start_rot,
        direction=-1.0 if float(dz_mm) <= 0.0 else 1.0,
        amplitude_m=amplitude_m,
        lateral_step_m=lateral_step_m,
        cycles=cycle_count,
    )):
        target_pose_sdk = np.eye(4)
        target_pose_sdk[:3, :3] = start_rot
        target_pose_sdk[:3, 3] = target_pos_m * 1000.0
        sp = create_ik_param("mujoco", sdk_root=sdk_root)
        sp.set_input_ik_target_tcp(kine.mat4x4_to_mat1x16(target_pose_sdk))
        sp.set_input_ik_ref_joint(preflight_ref)
        sp.set_input_ik_zsp_type(0)
        ik = kine.ik(sp)
        if ik.get_output_result_num() < 1:
            ik_success = False
            stopped_reason = "preflight_ik_failed"
            ik_failed_step = target_index
            ik_failed_target_mm = (target_pos_m * 1000.0).tolist()
            break
        preflight_ref = ik.get_output_ret_joint()

    mode = str(tracking_mode).lower()
    if mode not in ("trajectory", "path"):
        raise ValueError("tracking_mode must be 'trajectory' or 'path'")
    direction = -1.0 if float(dz_mm) <= 0.0 else 1.0
    path_segments = _build_cut_path_segments(
        start_pos_m=start_pos_m,
        direction=direction,
        amplitude_m=amplitude_m,
        lateral_step_m=lateral_step_m,
        cycles=cycle_count,
    )
    path_segment_index = 0
    path_time_s = 0.0
    path_speed_m_s = max(1e-6, abs(float(path_speed_mm_s)) * 0.001)
    lookahead_m = max(0.0, abs(float(lookahead_mm)) * 0.001)
    path_tolerance_m = max(0.0, abs(float(path_tolerance_mm)) * 0.001)
    max_path_steps = max(motion_steps, int(np.ceil(_path_length(path_segments) / path_speed_m_s * float(control_hz))) * 2)
    loop_steps = motion_steps if mode == "trajectory" else max_path_steps

    for step_index in range(loop_steps if ik_success else 0):
        if mode == "trajectory":
            cycle_index = min(step_index // steps_per_cycle, cycle_count - 1)
            cycle_step = step_index - cycle_index * steps_per_cycle
            z_progress, retract_progress = _cut_cycle_progress(cycle_step, steps_per_cycle)
            target_pos_m = start_pos_m.copy()
            target_pos_m[2] = start_pos_m[2] + direction * amplitude_m * z_progress
            target_pos_m[1] = start_pos_m[1] + lateral_step_m * (
                float(cycle_index) + retract_progress
            )
        else:
            if path_segment_index >= len(path_segments):
                break
            actual_for_path_m, _ = internal._arm.site_pose("right_tool_tip_site")
            target_pos_m, path_time_s = _path_tracking_target(
                segments=path_segments,
                segment_index=path_segment_index,
                actual_pos_m=actual_for_path_m,
                path_time_s=path_time_s,
                dt_s=1.0 / max(float(control_hz), 1.0),
                speed_m_s=path_speed_m_s,
                lookahead_m=lookahead_m,
            )

        target_pose_sdk = np.eye(4)
        target_pose_sdk[:3, :3] = start_rot
        target_pose_sdk[:3, 3] = target_pos_m * 1000.0
        sp = create_ik_param("mujoco", sdk_root=sdk_root)
        sp.set_input_ik_target_tcp(kine.mat4x4_to_mat1x16(target_pose_sdk))
        sp.set_input_ik_ref_joint(ref_for_ik)
        sp.set_input_ik_zsp_type(0)
        ik = kine.ik(sp)
        if ik.get_output_result_num() < 1:
            ik_success = False
            stopped_reason = "ik_failed"
            ik_failed_step = step_index
            ik_failed_target_mm = (target_pos_m * 1000.0).tolist()
            break
        target_joints = ik.get_output_ret_joint()
        target_joints_array = np.asarray(target_joints, dtype=float)
        target_velocity_deg_s = (
            float(velocity_ff) * (target_joints_array - previous_target_joints) * float(control_hz)
        )
        previous_target_joints = target_joints_array.copy()
        ref_for_ik = target_joints
        target_joint_trace_deg.append(list(target_joints))

        robot.set_joint_position_cmd(arm, target_joints, target_velocity_deg_s)
        robot.step(viewer_sync=viewer)

        actual_pos_m, _ = internal._arm.site_pose("right_tool_tip_site")
        if viewer_trace and step_index % trace_stride == 0:
            _append_viewer_trace_marker(
                internal._viewer, target_pos_m, (0.0, 0.85, 1.0, 0.9), 0.004,
            )
            _append_viewer_trace_marker(
                internal._viewer, actual_pos_m, (1.0, 0.45, 0.0, 0.9), 0.004,
            )
        target_x_trace_mm.append(float(target_pos_m[0] * 1000.0))
        actual_x_trace_mm.append(float(actual_pos_m[0] * 1000.0))
        target_y_trace_mm.append(float(target_pos_m[1] * 1000.0))
        actual_y_trace_mm.append(float(actual_pos_m[1] * 1000.0))
        target_z_trace_mm.append(float(target_pos_m[2] * 1000.0))
        actual_z_trace_mm.append(float(actual_pos_m[2] * 1000.0))
        if mode == "path":
            while path_segment_index < len(path_segments):
                _, segment_end = path_segments[path_segment_index]
                if float(np.linalg.norm(actual_pos_m - segment_end)) > path_tolerance_m:
                    break
                path_segment_index += 1
                path_time_s = 0.0
        data = robot.subscribe(None)

    return {
        "ik_success": ik_success,
        "reference_joints": reference_joints,
        "target_joints": target_joints,
        "target_joint_trace_deg": target_joint_trace_deg,
        "feedback": data["outputs"][arm_index],
        "motion_steps": motion_steps,
        "executed_steps": len(target_joint_trace_deg),
        "stopped_reason": stopped_reason,
        "ik_failed_step": ik_failed_step,
        "ik_failed_target_mm": ik_failed_target_mm,
        "target_x_trace_mm": target_x_trace_mm,
        "actual_x_trace_mm": actual_x_trace_mm,
        "target_y_trace_mm": target_y_trace_mm,
        "actual_y_trace_mm": actual_y_trace_mm,
        "target_z_trace_mm": target_z_trace_mm,
        "actual_z_trace_mm": actual_z_trace_mm,
        "lateral": bool(lateral),
        "lateral_mm": float(lateral_mm),
        "joint_k": list(joint_k),
        "joint_d": list(joint_d),
        "self_collision_enabled": bool(enable_self_collision),
        "velocity_ff": float(velocity_ff),
        "tracking_mode": mode,
        "path_speed_mm_s": float(path_speed_mm_s),
        "lookahead_mm": float(lookahead_mm),
        "path_tolerance_mm": float(path_tolerance_mm),
        "control_mode": internal.state.name,
        "chopping_home_joints": chopping_home_joints,
        "execution_mode": "JOINT_IMPEDANCE_IK_OSCILLATION",
    }


def _run_sdk_endpoint_ik_joint_impedance(
    *,
    robot,
    kine,
    arm: str,
    arm_index: int,
    control_hz: float,
    dz_mm: float,
    hold_s: float,
    joint_k: list[float],
    joint_d: list[float],
    backend: str,
    sdk_root: str | Path | None,
    viewer: bool,
) -> dict:
    robot.set_position_state(arm, velRatio=30, AccRatio=30)
    robot.set_joint_position_cmd(arm, READY_B)
    robot.wait(hold_s, viewer_sync=viewer)

    data = robot.subscribe(None)
    if data is None:
        raise RuntimeError("subscribe failed after ready command")
    reference_joints = data["outputs"][arm_index]["fb_joint_pos"]
    current_pose = np.asarray(kine.fk(reference_joints), dtype=float)
    target_pose = current_pose.copy()
    target_pose[2, 3] += float(dz_mm)

    sp = create_ik_param(backend, sdk_root=sdk_root)
    sp.set_input_ik_target_tcp(kine.mat4x4_to_mat1x16(target_pose))
    sp.set_input_ik_ref_joint(reference_joints)
    sp.set_input_ik_zsp_type(0)
    ik = kine.ik(sp)
    target_joints = ik.get_output_ret_joint()

    robot.set_imp_joint_state(arm, velRatio=40, AccRatio=40, K=joint_k, D=joint_d)
    robot.set_joint_position_cmd(arm, target_joints)

    motion_steps = max(2, int(round(float(hold_s) * float(control_hz))))
    for _ in range(motion_steps):
        if hasattr(robot, "step"):
            robot.step(viewer_sync=viewer)
        else:
            robot.wait(1.0 / max(float(control_hz), 1.0), viewer_sync=viewer)
        data = robot.subscribe(None)

    actual_pose = np.asarray(kine.fk(data["outputs"][arm_index]["fb_joint_pos"]), dtype=float)
    return {
        "ik_success": ik.get_output_result_num() >= 1,
        "reference_joints": reference_joints,
        "target_joints": target_joints,
        "target_joint_trace_deg": [target_joints],
        "feedback": data["outputs"][arm_index],
        "motion_steps": motion_steps,
        "target_x_trace_mm": [float(target_pose[0, 3])],
        "actual_x_trace_mm": [float(actual_pose[0, 3])],
        "target_y_trace_mm": [float(target_pose[1, 3])],
        "actual_y_trace_mm": [float(actual_pose[1, 3])],
        "target_z_trace_mm": [float(target_pose[2, 3])],
        "actual_z_trace_mm": [float(actual_pose[2, 3])],
        "lateral": False,
        "lateral_mm": 0.0,
        "joint_k": list(joint_k),
        "joint_d": list(joint_d),
        "control_mode": "JOINT_IMPEDANCE",
        "chopping_home_joints": READY_B,
        "execution_mode": "JOINT_IMPEDANCE_IK_ENDPOINT",
    }


def _build_cut_path_segments(
    *,
    start_pos_m: np.ndarray,
    direction: float,
    amplitude_m: float,
    lateral_step_m: float,
    cycles: int,
) -> list[tuple[np.ndarray, np.ndarray]]:
    segments: list[tuple[np.ndarray, np.ndarray]] = []
    current = start_pos_m.copy()
    for cycle_index in range(max(1, int(cycles))):
        top = start_pos_m.copy()
        top[1] = start_pos_m[1] + lateral_step_m * float(cycle_index)
        bottom = top.copy()
        bottom[2] = start_pos_m[2] + direction * amplitude_m
        if np.linalg.norm(current - top) > 1e-9:
            segments.append((current.copy(), top.copy()))
        segments.append((top.copy(), bottom.copy()))
        next_top = start_pos_m.copy()
        next_top[1] = start_pos_m[1] + lateral_step_m * float(cycle_index + 1)
        segments.append((bottom.copy(), next_top.copy()))
        current = next_top.copy()
    return segments


def _path_length(segments: list[tuple[np.ndarray, np.ndarray]]) -> float:
    return float(sum(np.linalg.norm(end - start) for start, end in segments))


def _path_tracking_target(
    *,
    segments: list[tuple[np.ndarray, np.ndarray]],
    segment_index: int,
    actual_pos_m: np.ndarray,
    path_time_s: float,
    dt_s: float,
    speed_m_s: float,
    lookahead_m: float,
) -> tuple[np.ndarray, float]:
    start, end = segments[segment_index]
    vec = end - start
    length = float(np.linalg.norm(vec))
    if length < 1e-12:
        return end.copy(), 0.0
    direction = vec / length
    s_actual = float(np.clip(np.dot(actual_pos_m - start, direction), 0.0, length))
    next_time_s = min(path_time_s + speed_m_s * dt_s, length)
    target_s = min(next_time_s, s_actual + lookahead_m, length)
    return start + direction * target_s, next_time_s


def _preflight_targets(
    *,
    start_pos_m: np.ndarray,
    start_rot: np.ndarray,
    direction: float,
    amplitude_m: float,
    lateral_step_m: float,
    cycles: int,
) -> list[np.ndarray]:
    del start_rot
    targets: list[np.ndarray] = []
    for cycle_index in range(max(1, int(cycles))):
        top = start_pos_m.copy()
        top[1] = start_pos_m[1] + lateral_step_m * float(cycle_index)
        targets.append(top)

        bottom = top.copy()
        bottom[2] = start_pos_m[2] + direction * amplitude_m
        targets.append(bottom)

    final_top = start_pos_m.copy()
    final_top[1] = start_pos_m[1] + lateral_step_m * float(max(1, int(cycles)))
    targets.append(final_top)
    return targets


def _disable_robot_collision_geoms(internal) -> None:
    model = internal.runtime.model
    for geom_id in range(model.ngeom):
        body_id = int(model.geom_bodyid[geom_id])
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
        if (
            body_name == "robot_base"
            or body_name.startswith("left_link")
            or body_name.startswith("right_link")
        ):
            model.geom_contype[geom_id] = 0
            model.geom_conaffinity[geom_id] = 0


def _cut_cycle_progress(cycle_step: int, steps_per_cycle: int) -> tuple[float, float]:
    descent_steps = max(1, int(steps_per_cycle) // 2)
    retract_steps = max(1, int(steps_per_cycle) - descent_steps)
    if cycle_step < descent_steps:
        denominator = max(descent_steps - 1, 1)
        return _smoothstep(float(cycle_step) / float(denominator)), 0.0

    retract_step = min(int(cycle_step) - descent_steps, retract_steps - 1)
    denominator = max(retract_steps - 1, 1)
    retract_progress = _smoothstep(float(retract_step) / float(denominator))
    return 1.0 - retract_progress, retract_progress


def _smoothstep(value: float) -> float:
    u = float(np.clip(value, 0.0, 1.0))
    return u * u * (3.0 - 2.0 * u)


def _clear_viewer_trace(viewer) -> None:
    if viewer is None or getattr(viewer, "user_scn", None) is None:
        return
    with viewer.lock():
        viewer.user_scn.ngeom = 0


def _append_viewer_trace_marker(
    viewer,
    position_m: np.ndarray,
    rgba: tuple[float, float, float, float],
    size_m: float,
) -> None:
    if viewer is None or getattr(viewer, "user_scn", None) is None:
        return
    scn = viewer.user_scn
    with viewer.lock():
        if scn.ngeom >= scn.maxgeom:
            return
        geom = scn.geoms[scn.ngeom]
        _init_sphere_geom(
            geom,
            np.asarray(position_m, dtype=float).reshape(3),
            float(size_m),
            np.asarray(rgba, dtype=float),
        )
        scn.ngeom += 1


def _write_trace_plot(result: dict, path: Path, control_hz: float) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    target_z = np.asarray(result["target_z_trace_mm"], dtype=float)
    actual_z = np.asarray(result["actual_z_trace_mm"], dtype=float)
    if target_z.size == 0 or actual_z.size == 0:
        raise ValueError("trajectory trace is empty")
    count = min(target_z.size, actual_z.size)
    time_s = np.arange(count, dtype=float) / max(float(control_hz), 1.0)

    path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(9, 4.8), dpi=140)
    ax.plot(time_s, target_z[:count], label="target TCP Z", linewidth=2.0)
    ax.plot(time_s, actual_z[:count], label="actual TCP Z", linewidth=1.8)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("TCP Z (mm)")
    ax.set_title("IK joint impedance down-up trajectory")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def _parse_vector(text: str) -> list[float]:
    values = [float(item.strip()) for item in text.split(",") if item.strip()]
    if len(values) != 7:
        raise argparse.ArgumentTypeError("expected 7 comma-separated values")
    return values


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("mujoco", "real"), default="mujoco")
    parser.add_argument("--arm", choices=("A", "B"), default="B")
    parser.add_argument("--robot-ip", default="mujoco")
    parser.add_argument("--sdk-root", type=Path, default=None)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--realtime", action="store_true")
    parser.add_argument("--control-hz", type=float, default=250.0)
    parser.add_argument("--dz-mm", type=float, default=-20.0)
    parser.add_argument("--hold-s", type=float, default=1.0)
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--plot", type=Path, default=None, help="Save target/actual TCP Z trace plot as PNG")
    parser.add_argument("--viewer-trace", action="store_true", help="Draw target/actual TCP trail markers in the MuJoCo viewer")
    parser.add_argument("--viewer-trace-stride", type=int, default=20, help="Control-step interval between viewer trace markers")
    parser.add_argument("--lateral", action="store_true", help="Enable Y offset between consecutive vertical cuts")
    parser.add_argument("--lateral-mm", type=float, default=30.0, help="Y offset between consecutive vertical cuts in millimetres")
    parser.add_argument("--joint-k", type=_parse_vector, default=SDK_JOINT_K, help="SDK-style joint stiffness, comma-separated 7 values")
    parser.add_argument("--joint-d", type=_parse_vector, default=SDK_JOINT_D, help="SDK-style joint damping, comma-separated 7 values")
    parser.add_argument("--enable-self-collision", action="store_true", help="Keep robot self-collision enabled for debugging")
    parser.add_argument("--velocity-ff", type=float, default=0.8, help="Scale for joint target velocity feedforward")
    parser.add_argument("--tracking-mode", choices=("trajectory", "path"), default="trajectory")
    parser.add_argument("--path-speed-mm-s", type=float, default=80.0)
    parser.add_argument("--lookahead-mm", type=float, default=8.0)
    parser.add_argument("--path-tolerance-mm", type=float, default=6.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    result = run_demo(
        backend=args.backend,
        arm=args.arm,
        robot_ip=args.robot_ip,
        sdk_root=args.sdk_root,
        viewer=args.viewer and not args.headless,
        realtime=args.realtime,
        control_hz=args.control_hz,
        dz_mm=args.dz_mm,
        hold_s=args.hold_s,
        cycles=args.cycles,
        viewer_trace=args.viewer_trace,
        viewer_trace_stride=args.viewer_trace_stride,
        lateral=args.lateral,
        lateral_mm=args.lateral_mm,
        joint_k=args.joint_k,
        joint_d=args.joint_d,
        enable_self_collision=args.enable_self_collision,
        velocity_ff=args.velocity_ff,
        tracking_mode=args.tracking_mode,
        path_speed_mm_s=args.path_speed_mm_s,
        lookahead_mm=args.lookahead_mm,
        path_tolerance_mm=args.path_tolerance_mm,
    )
    if args.plot is not None:
        _write_trace_plot(result, args.plot, args.control_hz)
        print("plot:", args.plot)
    print("ik_success:", result["ik_success"])
    print("execution_mode:", result["execution_mode"])
    print("control_mode:", result["control_mode"])
    print("motion_steps:", result["motion_steps"])
    print("executed_steps:", result.get("executed_steps"))
    print("stopped_reason:", result.get("stopped_reason"))
    if result.get("ik_failed_target_mm") is not None:
        print("ik_failed_step:", result.get("ik_failed_step"))
        print("ik_failed_target_mm:", [round(v, 3) for v in result["ik_failed_target_mm"]])
    print("lateral:", result["lateral"])
    print("joint_k:", result["joint_k"])
    print("joint_d:", result["joint_d"])
    print("self_collision_enabled:", result.get("self_collision_enabled"))
    print("velocity_ff:", result.get("velocity_ff"))
    print("tracking_mode:", result.get("tracking_mode"))
    print("target_joints:", [round(v, 3) for v in result["target_joints"]])
    print("feedback_joints:", result["feedback"]["fb_joint_pos"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
