#!/usr/bin/env python3
"""Conservative base-frame keyboard Cartesian jog for one physical robot arm."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable

import numpy as np


KEY_DIRECTIONS: dict[str, tuple[float, float, float]] = {
    "w": (1.0, 0.0, 0.0),
    "s": (-1.0, 0.0, 0.0),
    "a": (0.0, 1.0, 0.0),
    "d": (0.0, -1.0, 0.0),
    "r": (0.0, 0.0, 1.0),
    "f": (0.0, 0.0, -1.0),
}


@dataclass(frozen=True)
class JogConfig:
    """Safety-critical configuration for a keyboard jog session."""

    arm: str = "A"
    step_mm: float = 2.0
    execute: bool = False
    workspace_min: tuple[float, float, float] | None = None
    workspace_max: tuple[float, float, float] | None = None
    vel_ratio: int = 10
    acc_ratio: int = 10
    keep_enabled: bool = False
    robot_ip: str = "192.168.1.190"


def validate_config(config: JogConfig) -> None:
    """Reject invalid motion limits before any SDK connection is opened."""
    if config.arm not in ("A", "B"):
        raise ValueError("arm must be 'A' or 'B'")
    if not math.isfinite(float(config.step_mm)) or not 0.0 < float(config.step_mm) <= 5.0:
        raise ValueError("step-mm must be in (0, 5]")
    if not (0 <= int(config.vel_ratio) <= 100):
        raise ValueError("vel-ratio must be in [0, 100]")
    if not (0 <= int(config.acc_ratio) <= 100):
        raise ValueError("acc-ratio must be in [0, 100]")
    if config.execute and (config.workspace_min is None or config.workspace_max is None):
        raise ValueError("--execute requires both --workspace-min and --workspace-max")
    if (config.workspace_min is None) != (config.workspace_max is None):
        raise ValueError("workspace-min and workspace-max must be supplied together")
    if config.workspace_min is not None and config.workspace_max is not None:
        lower = np.asarray(config.workspace_min, dtype=float)
        upper = np.asarray(config.workspace_max, dtype=float)
        if lower.shape != (3,) or upper.shape != (3,) or not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)):
            raise ValueError("workspace bounds must contain 3 finite values")
        if not np.all(lower < upper):
            raise ValueError("workspace-min must be strictly below workspace-max")


def inside_workspace(
    point_xyz: np.ndarray,
    lower_xyz: tuple[float, float, float],
    upper_xyz: tuple[float, float, float],
) -> bool:
    """Return whether a point lies within an inclusive Cartesian workspace box."""
    point = np.asarray(point_xyz, dtype=float)
    lower = np.asarray(lower_xyz, dtype=float)
    upper = np.asarray(upper_xyz, dtype=float)
    return bool(point.shape == (3,) and np.all(point >= lower) and np.all(point <= upper))


def key_to_delta(key: str, step_mm: float) -> tuple[float, float, float] | None:
    """Map a movement key to one base-frame Cartesian step in millimetres."""
    direction = KEY_DIRECTIONS.get(str(key).lower())
    if direction is None:
        return None
    return tuple(float(step_mm) * axis for axis in direction)


def candidate_pose(current_xyzabc: np.ndarray, delta_xyz: tuple[float, float, float]) -> np.ndarray:
    """Translate XYZ while retaining the supplied TCP ABC orientation."""
    target = np.asarray(current_xyzabc, dtype=float).copy()
    if target.shape != (6,):
        raise ValueError("current_xyzabc must contain exactly 6 values")
    target[:3] += np.asarray(delta_xyz, dtype=float)
    return target


def _arm_index(arm: str) -> int:
    return 0 if arm == "A" else 1


def _feedback(robot, dcss, arm_index: int) -> dict:
    return robot.subscribe(dcss)


def _read_feedback_pose(robot, dcss, kine, arm_index: int) -> tuple[list[float], np.ndarray]:
    """Read feedback joints and convert their FK result to XYZABC."""
    feedback = _feedback(robot, dcss, arm_index)
    joints = [float(value) for value in feedback["outputs"][arm_index]["fb_joint_pos"]]
    pose = np.asarray(kine.mat4x4_to_xyzabc(kine.fk(joints)), dtype=float)
    if pose.shape != (6,) or not np.all(np.isfinite(pose)):
        raise RuntimeError("kinematics returned an invalid feedback pose")
    return joints, pose


def _verify_frame_updates(robot, dcss, arm_index: int) -> None:
    """Refuse execute mode when feedback frame serials are not advancing."""
    observed = {
        _feedback(robot, dcss, arm_index)["outputs"][arm_index].get("frame_serial", 0)
        for _ in range(3)
    }
    if not any(int(serial) != 0 for serial in observed) or len(observed) < 2:
        raise RuntimeError("robot feedback frame did not update")


def _trajectory_is_idle(robot, dcss, arm_index: int) -> bool:
    output = _feedback(robot, dcss, arm_index)["outputs"][arm_index]
    return output.get("traj_state") in (None, 0, "idle", "IDLE")


def _wait_until_traj_idle(robot, dcss, arm_index: int, timeout_s: float = 3.0) -> None:
    deadline = time.monotonic() + float(timeout_s)
    while time.monotonic() < deadline:
        if _trajectory_is_idle(robot, dcss, arm_index):
            return
        time.sleep(0.02)
    raise RuntimeError("selected arm trajectory did not return idle")


def plan_or_execute_step(
    robot,
    dcss,
    kine,
    arm_index: int,
    joints: list[float],
    current_pose: np.ndarray,
    target_pose: np.ndarray,
    config: JogConfig,
) -> np.ndarray:
    """Plan one Cartesian step and send it only when execute is enabled."""
    _points, pset = kine.movLA(
        start_xyzabc=current_pose.tolist(),
        end_xyzabc=target_pose.tolist(),
        ref_joints=list(joints),
        vel=10.0,
        acc=100.0,
        freq_hz=100,
    )
    if pset is None:
        raise RuntimeError("MOVLA planning failed")
    if not config.execute:
        return target_pose
    robot.setPln_Cart(arm=config.arm, pset=pset)
    _wait_until_traj_idle(robot, dcss, arm_index)
    _joints, actual_pose = _read_feedback_pose(robot, dcss, kine, arm_index)
    return actual_pose


def _shutdown(robot, config: JogConfig, connected: bool) -> None:
    if connected and not config.keep_enabled:
        try:
            robot.clear_set()
            robot.set_state(arm=config.arm, state=0)
            robot.send_cmd()
        except Exception:
            pass
    try:
        robot.release_robot()
    except Exception:
        pass


def run_jog_session(
    config: JogConfig,
    read_key: Callable[[], str],
    sdk_factory: Callable[[], tuple[object, object, object]],
) -> list[np.ndarray]:
    """Run a key-driven jog session using an injected SDK adapter.

    The factory injection makes this control flow testable without importing or
    connecting to the vendor SDK.  Production supplies the real factory.
    """
    validate_config(config)
    robot, dcss, kine = sdk_factory()
    arm_index = _arm_index(config.arm)
    connected = False
    poses: list[np.ndarray] = []
    try:
        connected = bool(robot.connect(config.robot_ip))
        if not connected:
            raise RuntimeError("failed to connect to the robot")
        robot.check_error_and_clear(dcss)
        if config.execute:
            _verify_frame_updates(robot, dcss, arm_index)
        joints, current_pose = _read_feedback_pose(robot, dcss, kine, arm_index)
        poses.append(current_pose)
        while True:
            try:
                key = read_key()
            except StopIteration:
                break
            if not key or key.lower() == "q" or key == " ":
                break
            delta = key_to_delta(key, config.step_mm)
            if delta is None:
                continue
            target_pose = candidate_pose(current_pose, delta)
            if config.execute:
                if config.workspace_min is None or config.workspace_max is None:
                    raise RuntimeError("execute workspace validation was bypassed")
                if not inside_workspace(target_pose[:3], config.workspace_min, config.workspace_max):
                    raise ValueError(f"requested pose is outside workspace: {target_pose[:3].tolist()}")
                if not _trajectory_is_idle(robot, dcss, arm_index):
                    raise RuntimeError("selected arm trajectory is not idle")
            current_pose = plan_or_execute_step(
                robot, dcss, kine, arm_index, joints, current_pose, target_pose, config,
            )
            if config.execute:
                joints, current_pose = _read_feedback_pose(robot, dcss, kine, arm_index)
            poses.append(current_pose)
        return poses
    finally:
        _shutdown(robot, config, connected)
