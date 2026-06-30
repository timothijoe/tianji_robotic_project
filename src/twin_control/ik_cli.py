"""Command-line inverse kinematics helper for the MuJoCo Marvin model."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Sequence

import numpy as np

from twin_control.kinematics import MarvinKinematics, xyzabc_to_matrix
from twin_control.robot import LEFT_HOME_RAD, RIGHT_HOME_RAD
from twin_description.paths import right_chopping_scene_path
from twin_mujoco.chopping import RIGHT_CHOPPING_HOME_Q
from twin_mujoco.runtime import TwinMujocoRuntime


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Solve 7-axis arm IK from x y z roll pitch yaw using the MuJoCo "
            "scene as FK/Jacobian ground truth."
        ),
    )
    parser.add_argument(
        "pose",
        nargs=6,
        type=float,
        metavar="POSE",
        help="x y z roll pitch yaw. Position is metres; angles are radians unless --degrees is set.",
    )
    parser.add_argument("--arm", choices=("right", "left"), default="right")
    parser.add_argument(
        "--site",
        default="right_force_sensor_site",
        help="MuJoCo site to solve for; use right_tool_tip_site for knife tip.",
    )
    parser.add_argument("--model", type=Path, default=None, help="MJCF/XML path; defaults to right_chopping_scene.xml.")
    parser.add_argument("--degrees", action="store_true", help="Interpret roll/pitch/yaw as degrees and print degrees too.")
    parser.add_argument(
        "--reference",
        choices=("chopping-home", "robot-home", "zero"),
        default="chopping-home",
        help="Reference posture used to choose one solution for the redundant 7-DOF arm.",
    )
    parser.add_argument(
        "--ref-joints",
        default=None,
        help="Comma-separated 7 joint angles in radians; overrides --reference.",
    )
    args = parser.parse_args(argv)

    xyzabc = np.asarray(args.pose, dtype=float)
    if args.degrees:
        xyzabc[3:] = np.deg2rad(xyzabc[3:])

    reference = _reference_joints(args.arm, args.reference, args.ref_joints)
    runtime = TwinMujocoRuntime.load(args.model or right_chopping_scene_path())
    runtime.reset()
    if args.arm == "right":
        runtime.set_arm_positions("left", LEFT_HOME_RAD)
    runtime.set_arm_positions(args.arm, reference)

    kinematics = MarvinKinematics(args.arm, unit_mode="si", tcp_site_name=args.site)
    kinematics.set_runtime(runtime)

    target = xyzabc_to_matrix(xyzabc)
    result = kinematics.ik(target, reference)

    runtime.set_arm_positions(args.arm, result.joints_rad)
    actual_matrix, actual_xyzabc = kinematics.fk(result.joints_rad)
    position_error = float(np.linalg.norm(target[:3, 3] - actual_matrix[:3, 3]))
    orientation_error = float(np.linalg.norm(actual_xyzabc[3:] - xyzabc[3:]))

    print(f"success: {result.success}")
    print(f"iterations: {result.iterations}")
    print(f"residual: {result.residual:.9g}")
    print(f"position_error_m: {position_error:.9g}")
    print(f"orientation_error_rad_approx: {orientation_error:.9g}")
    print("joints_rad:")
    print(_format_array(result.joints_rad))
    if args.degrees:
        print("joints_deg:")
        print(_format_array(np.rad2deg(result.joints_rad)))
    print("python:")
    print("np.array((" + ", ".join(f"{v:.9g}" for v in result.joints_rad) + "), dtype=float)")

    return 0 if result.success else 2


def _reference_joints(arm: str, reference_name: str, custom: str | None) -> np.ndarray:
    if custom:
        values = np.asarray([float(part.strip()) for part in custom.split(",")], dtype=float)
        if values.shape != (7,) or not np.all(np.isfinite(values)):
            raise ValueError("--ref-joints must contain 7 finite comma-separated values")
        return values
    if reference_name == "zero":
        return np.zeros(7, dtype=float)
    if arm == "left":
        return LEFT_HOME_RAD.copy()
    if reference_name == "robot-home":
        return RIGHT_HOME_RAD.copy()
    return RIGHT_CHOPPING_HOME_Q.copy()


def _format_array(values: np.ndarray) -> str:
    return "[" + ", ".join(f"{float(v): .9f}" for v in values) + "]"


if __name__ == "__main__":
    raise SystemExit(main())
