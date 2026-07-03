#!/usr/bin/env python3
"""SDK-style IK + Cartesian impedance demo for MuJoCo or real backend."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

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
from twin_mujoco.control import CartesianForceController


READY_B = [-75.627, -67.572, 52.390, -124.574, -90.421, 42.952, 41.374]


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
) -> dict:
    """Run an SDK-style IK + Cartesian impedance down-up motion."""
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

        arm_index = 0 if arm.upper() == "A" else 1
        if backend == "mujoco" and arm.upper() == "B":
            return _run_mujoco_down_up(
                robot=robot,
                kine=kine,
                arm=arm,
                arm_index=arm_index,
                control_hz=control_hz,
                dz_mm=dz_mm,
                hold_s=hold_s,
                cycles=cycles,
                viewer=viewer,
                viewer_trace=viewer_trace,
                viewer_trace_stride=viewer_trace_stride,
                lateral=lateral,
                lateral_mm=lateral_mm,
                sdk_root=sdk_root,
            )

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

        robot.set_imp_cart_state(
            arm,
            velRatio=40,
            AccRatio=40,
            K=[2500, 2500, 2800, 45, 45, 35, 4],
            D=[0.8, 0.8, 0.8, 0.4, 0.4, 0.4, 1],
            rot_type=0,
            cart_ctrl_para=[0] * 7,
        )
        robot.set_joint_position_cmd(arm, target_joints)

        motion_steps = max(2, int(round(float(hold_s) * float(control_hz))))
        for _ in range(motion_steps):
            if hasattr(robot, "step"):
                robot.step(viewer_sync=viewer)
            else:
                robot.wait(1.0 / max(float(control_hz), 1.0), viewer_sync=viewer)
            data = robot.subscribe(None)

        return {
            "ik_success": ik.get_output_result_num() >= 1,
            "reference_joints": reference_joints,
            "target_joints": target_joints,
            "feedback": data["outputs"][arm_index],
            "motion_steps": motion_steps,
            "target_x_trace_mm": [float(target_pose[0, 3])],
            "actual_x_trace_mm": [float(np.asarray(kine.fk(data["outputs"][arm_index]["fb_joint_pos"]), dtype=float)[0, 3])],
            "target_y_trace_mm": [float(target_pose[1, 3])],
            "actual_y_trace_mm": [float(np.asarray(kine.fk(data["outputs"][arm_index]["fb_joint_pos"]), dtype=float)[1, 3])],
            "target_z_trace_mm": [float(target_pose[2, 3])],
            "actual_z_trace_mm": [float(np.asarray(kine.fk(data["outputs"][arm_index]["fb_joint_pos"]), dtype=float)[2, 3])],
            "lateral": False,
            "lateral_mm": 0.0,
            "chopping_home_joints": READY_B,
            "execution_mode": "CARTESIAN_IMPEDANCE_ENDPOINT",
        }
    finally:
        robot.release_robot()


def _run_mujoco_down_up(
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
    sdk_root: str | Path | None,
) -> dict:
    internal = robot._require_robot()
    internal.runtime.reset()
    internal.runtime.set_arm_positions("left", LEFT_HOME_RAD)
    internal.runtime.set_arm_positions("right", RIGHT_CHOPPING_HOME_RAD)

    data = robot.subscribe(None)
    if data is None:
        raise RuntimeError("subscribe failed after chopping-home reset")
    reference_joints = data["outputs"][arm_index]["fb_joint_pos"]
    chopping_home_joints = np.rad2deg(RIGHT_CHOPPING_HOME_RAD).tolist()

    start_pos_m, start_rot = internal._arm.site_pose("right_tool_tip_site")
    controller = CartesianForceController(internal._arm, tool_site="right_tool_tip_site")
    trace_stride = max(1, int(viewer_trace_stride))
    if viewer_trace:
        _clear_viewer_trace(internal._viewer)
    cycle_count = max(1, int(cycles))
    steps_per_cycle = max(3, int(round(float(hold_s) * float(control_hz))))
    motion_steps = steps_per_cycle * cycle_count
    amplitude_m = abs(float(dz_mm)) * 0.001
    lateral_amplitude_m = abs(float(lateral_mm)) * 0.001 if lateral else 0.0

    target_x_trace_mm: list[float] = []
    actual_x_trace_mm: list[float] = []
    target_y_trace_mm: list[float] = []
    actual_y_trace_mm: list[float] = []
    target_z_trace_mm: list[float] = []
    actual_z_trace_mm: list[float] = []
    ref_for_ik = list(reference_joints)
    target_joints = list(reference_joints)
    ik_success = True

    for step_index in range(motion_steps):
        cycle_index = min(step_index // steps_per_cycle, cycle_count - 1)
        cycle_step = step_index - cycle_index * steps_per_cycle
        z_progress, retract_progress = _cut_cycle_progress(cycle_step, steps_per_cycle)
        target_pos_m = start_pos_m.copy()
        direction = -1.0 if float(dz_mm) <= 0.0 else 1.0
        target_pos_m[2] = start_pos_m[2] + direction * amplitude_m * z_progress
        target_pos_m[1] = start_pos_m[1] + lateral_amplitude_m * (
            float(cycle_index) + retract_progress
        )

        target_pose_sdk = np.eye(4)
        target_pose_sdk[:3, :3] = start_rot
        target_pose_sdk[:3, 3] = target_pos_m * 1000.0
        sp = create_ik_param("mujoco", sdk_root=sdk_root)
        sp.set_input_ik_target_tcp(kine.mat4x4_to_mat1x16(target_pose_sdk))
        sp.set_input_ik_ref_joint(ref_for_ik)
        sp.set_input_ik_zsp_type(0)
        ik = kine.ik(sp)
        ik_success = ik_success and ik.get_output_result_num() >= 1
        target_joints = ik.get_output_ret_joint()
        ref_for_ik = target_joints

        controller.set_target(target_pos_m, start_rot)
        torque = controller.compute(np.zeros(6))
        internal._arm.apply_torque(torque)
        internal._hold_other_arm()
        for _ in range(internal._substeps):
            internal.runtime.step()
        internal._hold_other_arm()
        if internal._viewer is not None and viewer:
            internal._viewer.sync()

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
        data = robot.subscribe(None)

    return {
        "ik_success": ik_success,
        "reference_joints": reference_joints,
        "target_joints": target_joints,
        "feedback": data["outputs"][arm_index],
        "motion_steps": motion_steps,
        "target_x_trace_mm": target_x_trace_mm,
        "actual_x_trace_mm": actual_x_trace_mm,
        "target_y_trace_mm": target_y_trace_mm,
        "actual_y_trace_mm": actual_y_trace_mm,
        "target_z_trace_mm": target_z_trace_mm,
        "actual_z_trace_mm": actual_z_trace_mm,
        "lateral": bool(lateral),
        "lateral_mm": float(lateral_mm),
        "chopping_home_joints": chopping_home_joints,
        "execution_mode": "CARTESIAN_IMPEDANCE_OSCILLATION",
    }


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
    ax.set_title("IK Cartesian impedance down-up trajectory")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)



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
    parser.add_argument("--lateral", action="store_true", help="Enable left-right Y motion while moving up and down")
    parser.add_argument("--lateral-mm", type=float, default=30.0, help="Y offset between consecutive vertical cuts in millimetres")
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
    )
    if args.plot is not None:
        _write_trace_plot(result, args.plot, args.control_hz)
        print("plot:", args.plot)
    print("ik_success:", result["ik_success"])
    print("execution_mode:", result["execution_mode"])
    print("motion_steps:", result["motion_steps"])
    print("lateral:", result["lateral"])
    print("target_joints:", [round(v, 3) for v in result["target_joints"]])
    print("feedback_joints:", result["feedback"]["fb_joint_pos"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
