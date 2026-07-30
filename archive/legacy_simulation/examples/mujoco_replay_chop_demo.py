#!/usr/bin/env python3
"""Replay sampled IK chopping targets in MuJoCo without torque control."""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path
from typing import Sequence

import mujoco
import mujoco.viewer
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / "src", ROOT / "src" / "twin_core", ROOT / "src" / "twin_description", ROOT / "src" / "twin_mujoco"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from twin_control.chopping import RIGHT_CHOPPING_HOME_RAD
from twin_control.replay import JointReplayConfig, JointReplayExecutor
from twin_control.sampled_planner import SampledChopConfig, build_sampled_cartesian_targets
from twin_control.sdk_kine import FX_InvKineSolvePara, MujocoKine
from twin_control.trail import _init_sphere_geom


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate sampled Cartesian chopping targets, solve IK with the "
            "MuJoCo model, and replay the resulting joint targets without "
            "applying actuator torques."
        )
    )
    parser.add_argument("--arm", choices=("A", "B"), default="B")
    parser.add_argument("--control-hz", type=float, default=250.0)
    parser.add_argument("--hold-s", type=float, default=2.0)
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--dz-mm", type=float, default=-20.0)
    parser.add_argument("--lateral", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--lateral-mm", type=float, default=10.0)
    parser.add_argument("--chop-axis", choices=("x", "y", "z"), default="y")
    parser.add_argument("--lateral-axis", choices=("x", "y", "z"), default="x")
    parser.add_argument("--lateral-phase", choices=("separate", "retract"), default="separate")
    parser.add_argument("--execution-model", choices=("ideal", "lagged", "noisy", "lagged-noisy"), default="lagged")
    parser.add_argument("--tracking-alpha", type=float, default=0.35)
    parser.add_argument("--joint-noise-std-deg", type=float, default=0.0)
    parser.add_argument("--command-delay-steps", type=int, default=0)
    parser.add_argument("--joint-velocity-limit-deg-s", type=float, default=None)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--csv", type=Path, default=None)
    parser.add_argument("--print-feedback", action="store_true")
    parser.add_argument("--feedback-stride", type=int, default=25)
    parser.add_argument("--viewer", action="store_true", help="Open a MuJoCo passive viewer and replay actual joints visually.")
    parser.add_argument("--viewer-sync-stride", type=int, default=1, help="Sync the viewer every N replay steps.")
    parser.add_argument("--realtime", action="store_true", help="Sleep by dt_s between viewer frames.")
    return parser.parse_args(argv)


def _planner_config(args: argparse.Namespace) -> SampledChopConfig:
    return SampledChopConfig(
        control_hz=float(args.control_hz),
        hold_s=float(args.hold_s),
        cycles=int(args.cycles),
        dz_mm=float(args.dz_mm),
        lateral=bool(args.lateral),
        lateral_mm=float(args.lateral_mm),
        chop_axis=args.chop_axis,
        lateral_axis=args.lateral_axis,
        lateral_phase=args.lateral_phase,
    )


def _replay_config(args: argparse.Namespace) -> JointReplayConfig:
    return JointReplayConfig(
        model=args.execution_model,
        dt_s=1.0 / float(args.control_hz),
        tracking_alpha=float(args.tracking_alpha),
        joint_noise_std=float(args.joint_noise_std_deg),
        command_delay_steps=int(args.command_delay_steps),
        joint_velocity_limit=args.joint_velocity_limit_deg_s,
        random_seed=args.seed,
    )


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    rows = run_replay(args)
    if args.csv is not None:
        _write_csv(args.csv, rows)
    print(f"steps: {len(rows)}")
    print(f"execution_model: {args.execution_model}")
    if rows:
        print(f"final_tcp_error_mm: {rows[-1]['tcp_error_mm']:.6f}")
    return 0


def run_replay(args: argparse.Namespace) -> list[dict[str, float | int | bool]]:
    arm_type = 0 if args.arm == "A" else 1
    kine = MujocoKine(arm_type=arm_type, tcp_site_name="right_tool_tip_site" if args.arm == "B" else None)
    start_joints = _start_joints_deg(args.arm)
    start_pose = np.asarray(kine.mat4x4_to_xyzabc(kine.fk(start_joints)), dtype=float).reshape(6)
    targets = build_sampled_cartesian_targets(start_pose, _planner_config(args))
    executor = JointReplayExecutor(
        initial_joints=start_joints,
        config=_replay_config(args),
        fk=lambda joints: np.asarray(kine.fk(joints), dtype=float).reshape(4, 4),
        jacobian=lambda joints: np.asarray(kine.joints2JacobMatrix(joints), dtype=float).reshape(6, 7),
    )

    viewer = None
    try:
        if args.viewer:
            _set_runtime_arm_positions(kine.runtime, args.arm, start_joints)
            viewer = mujoco.viewer.launch_passive(kine.runtime.model, kine.runtime.data)
            _configure_viewer_camera(viewer)
        return _run_replay_loop(args, kine, targets, executor, start_joints, viewer)
    finally:
        if viewer is not None:
            viewer.close()


def _run_replay_loop(
    args: argparse.Namespace,
    kine: MujocoKine,
    targets,
    executor: JointReplayExecutor,
    start_joints: np.ndarray,
    viewer,
) -> list[dict[str, float | int | bool]]:
    ref_joints = start_joints.copy()
    rows: list[dict[str, float | int | bool]] = []
    for target in targets:
        target_matrix = kine.xyzabc_to_mat4x4(target.xyzabc.tolist())
        ik = _solve_ik(kine, target_matrix, ref_joints)
        ik_success = ik.get_output_result_num() >= 1
        target_joints = np.asarray(ik.get_output_ret_joint(), dtype=float).reshape(7)
        if ik_success:
            ref_joints = target_joints.copy()
        sample = executor.step(target_joints)
        actual_pose = np.asarray(kine.mat4x4_to_xyzabc(sample.tcp_pose), dtype=float).reshape(6)
        row = _trace_row(target.step_index, target.xyzabc, actual_pose, target_joints, sample, ik_success)
        rows.append(row)
        if viewer is not None:
            _update_viewer(args, kine, viewer, sample, target.xyzabc, actual_pose)
        if args.print_feedback and target.step_index % max(1, int(args.feedback_stride)) == 0:
            print(
                "feedback: "
                f"step={target.step_index},"
                f"target=({target.xyzabc[0]:.3f},{target.xyzabc[1]:.3f},{target.xyzabc[2]:.3f}),"
                f"actual=({actual_pose[0]:.3f},{actual_pose[1]:.3f},{actual_pose[2]:.3f}),"
                f"tcp_error_mm={row['tcp_error_mm']:.3f}"
            )
    return rows


def _solve_ik(kine: MujocoKine, target_matrix: Sequence[Sequence[float]], ref_joints: np.ndarray):
    param = FX_InvKineSolvePara()
    param.set_input_ik_target_tcp(kine.mat4x4_to_mat1x16(target_matrix))
    param.set_input_ik_ref_joint(ref_joints.tolist())
    param.set_input_ik_zsp_type(0)
    return kine.ik(param)


def _start_joints_deg(arm: str) -> np.ndarray:
    if arm == "B":
        return np.rad2deg(RIGHT_CHOPPING_HOME_RAD).astype(float)
    return np.zeros(7, dtype=float)


def _set_runtime_arm_positions(runtime, arm: str, joints_deg: np.ndarray) -> None:
    arm_name = "left" if arm == "A" else "right"
    runtime.set_arm_positions(arm_name, np.deg2rad(np.asarray(joints_deg, dtype=float).reshape(7)))


def _update_viewer(args: argparse.Namespace, kine: MujocoKine, viewer, sample, target_xyzabc: np.ndarray, actual_xyzabc: np.ndarray) -> None:
    _set_runtime_arm_positions(kine.runtime, args.arm, sample.actual_joints)
    if sample.step_index % max(1, int(args.viewer_sync_stride)) == 0:
        _append_trace_markers(viewer, target_xyzabc[:3] * 0.001, actual_xyzabc[:3] * 0.001)
        viewer.sync()
        if args.realtime:
            time.sleep(1.0 / float(args.control_hz))


def _append_trace_markers(viewer, target_m: np.ndarray, actual_m: np.ndarray) -> None:
    if viewer is None or viewer.user_scn is None:
        return
    with viewer.lock():
        for position, rgba, size in (
            (np.asarray(target_m, dtype=float).reshape(3), np.array((0.0, 0.85, 1.0, 0.9)), 0.01),
            (np.asarray(actual_m, dtype=float).reshape(3), np.array((1.0, 0.45, 0.0, 0.9)), 0.008),
        ):
            if viewer.user_scn.ngeom >= viewer.user_scn.maxgeom:
                return
            geom = viewer.user_scn.geoms[viewer.user_scn.ngeom]
            _init_sphere_geom(geom, position, size, rgba)
            viewer.user_scn.ngeom += 1


def _configure_viewer_camera(viewer) -> None:
    if not hasattr(viewer, "cam"):
        return
    viewer.cam.lookat[:] = (0.4, 0.0, 0.45)
    viewer.cam.distance = 1.25
    viewer.cam.azimuth = 155
    viewer.cam.elevation = -25


def _trace_row(
    step_index: int,
    target_xyzabc: np.ndarray,
    actual_xyzabc: np.ndarray,
    target_joints: np.ndarray,
    sample,
    ik_success: bool,
) -> dict[str, float | int | bool]:
    row: dict[str, float | int | bool] = {
        "step": int(step_index),
        "time_s": float(sample.time_s),
        "ik_success": bool(ik_success),
        "tcp_error_mm": float(np.linalg.norm(target_xyzabc[:3] - actual_xyzabc[:3])),
        "joint_error_deg": float(np.linalg.norm(target_joints - sample.actual_joints)),
    }
    for index, name in enumerate(("x", "y", "z", "a", "b", "c")):
        row[f"target_{name}"] = float(target_xyzabc[index])
        row[f"actual_{name}"] = float(actual_xyzabc[index])
    for index in range(7):
        row[f"target_q_{index}"] = float(target_joints[index])
        row[f"actual_q_{index}"] = float(sample.actual_joints[index])
        row[f"actual_qd_{index}"] = float(sample.joint_velocities[index])
    return row


def _write_csv(path: Path, rows: list[dict[str, float | int | bool]]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    raise SystemExit(main())
