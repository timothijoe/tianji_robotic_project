#!/usr/bin/env python3
"""SDK-style IK + Cartesian impedance demo for MuJoCo or real backend."""

from __future__ import annotations

import argparse
import time
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "src" / "twin_core"))
sys.path.insert(0, str(ROOT / "src" / "twin_description"))
sys.path.insert(0, str(ROOT / "src" / "twin_mujoco"))

from twin_control.sdk_compat import create_ik_param, create_kine, create_robot


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
) -> dict:
    """Run one SDK-style IK target through Cartesian impedance."""
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

        robot.set_position_state(arm, velRatio=30, AccRatio=30)
        robot.set_joint_position_cmd(arm, READY_B)
        robot.wait(hold_s, viewer_sync=viewer)

        current_pose = np.asarray(kine.fk(READY_B), dtype=float)
        target_pose = current_pose.copy()
        target_pose[2, 3] += float(dz_mm)

        sp = create_ik_param(backend, sdk_root=sdk_root)
        sp.set_input_ik_target_tcp(kine.mat4x4_to_mat1x16(target_pose))
        sp.set_input_ik_ref_joint(READY_B)
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

        deadline = time.monotonic() + float(hold_s)
        data = robot.subscribe(None)
        while time.monotonic() < deadline:
            if hasattr(robot, "step"):
                robot.step(viewer_sync=viewer)
            else:
                time.sleep(1.0 / max(float(control_hz), 1.0))
            data = robot.subscribe(None)

        arm_index = 0 if arm.upper() == "A" else 1
        return {
            "ik_success": ik.get_output_result_num() >= 1,
            "target_joints": target_joints,
            "feedback": data["outputs"][arm_index],
        }
    finally:
        robot.release_robot()


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
    )
    print("ik_success:", result["ik_success"])
    print("target_joints:", [round(v, 3) for v in result["target_joints"]])
    print("feedback_joints:", result["feedback"]["fb_joint_pos"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
