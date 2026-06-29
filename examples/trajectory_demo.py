#!/usr/bin/env python3
"""Trajectory demo: validate kinematics + control with 14 trajectory variants.

Usage::

    python examples/trajectory_demo.py --trajectory 1a        # single variant
    python examples/trajectory_demo.py --trajectory 2         # all group-2 variants
    python examples/trajectory_demo.py --all                  # all 14 variants
    python examples/trajectory_demo.py --all --headless --log /tmp/demo.csv
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Sequence

import numpy as np

# Ensure source packages are importable when the demo is run from a checkout.
_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT / "src"))
sys.path.insert(0, str(_ROOT / "src" / "twin_core"))
sys.path.insert(0, str(_ROOT / "src" / "twin_description"))
sys.path.insert(0, str(_ROOT / "src" / "twin_mujoco"))

from twin_control.controller import (
    CartesianImpedanceParams,
    ForceControlParams,
    JointImpedanceParams,
)
from twin_control.kinematics import MarvinKinematics
from twin_control.robot import (
    _DEFAULT_CART_D,
    _DEFAULT_CART_K,
    _DEFAULT_JOINT_D,
    _DEFAULT_JOINT_K,
    RIGHT_HOME_RAD,
    TwinRobot,
)

# ---------------------------------------------------------------------------
# Trajectory definitions
# ---------------------------------------------------------------------------

# Default impedance parameters (SI)
JOINT_K = _DEFAULT_JOINT_K
JOINT_D = _DEFAULT_JOINT_D
CART_K = _DEFAULT_CART_K
CART_D = _DEFAULT_CART_D


@dataclass
class TrajectorySpec:
    name: str
    group: int
    description: str
    steps_per_segment: int = 500  # control steps per segment


def _joint_waypoints(kin: MarvinKinematics, home: np.ndarray) -> list[TrajectorySpec]:
    """Trajectory group 1: Joint-space waypoints."""
    limits = kin.joint_limits_rad
    rng = np.random.default_rng(42)

    def random_config(scale: float) -> np.ndarray:
        mid = (limits[:, 0] + limits[:, 1]) / 2
        half = (limits[:, 1] - limits[:, 0]) / 2 * scale
        return np.clip(rng.uniform(mid - half, mid + half), limits[:, 0], limits[:, 1])

    specs = [
        TrajectorySpec("1a", 1, "Small sweep: home → 3 nearby configs (±10°) → home"),
        TrajectorySpec("1b", 1, "Full range: home → 4 configs spanning ±60° on J1-3, ±30° on J4-7"),
        TrajectorySpec("1c", 1, "Single joint: move each joint individually ±30° while others hold"),
        TrajectorySpec("1d", 1, "Random walk: home → 5 random feasible configs"),
    ]
    return specs


def _cartesian_waypoints(kin: MarvinKinematics, home: np.ndarray) -> list[TrajectorySpec]:
    """Trajectory group 2: Cartesian XYZ translations."""
    return [
        TrajectorySpec("2a", 2, "Axis-aligned: ±5 cm along X, Y, Z sequentially (6 segments)"),
        TrajectorySpec("2b", 2, "Diagonal: 5 cm along 4 body diagonals in XZ+YZ planes"),
        TrajectorySpec("2c", 2, "Step response: 2 cm step in +Z, return; same for +X"),
        TrajectorySpec("2d", 2, "Compound: sequential translations forming a box (8 corners)"),
    ]


def _curve_waypoints(kin: MarvinKinematics, home: np.ndarray) -> list[TrajectorySpec]:
    """Trajectory group 3: Cartesian curves."""
    return [
        TrajectorySpec("3a", 3, "Circle XZ: radius 8 cm, XZ plane, 3 revolutions", steps_per_segment=3000),
        TrajectorySpec("3b", 3, "Circle XY: radius 6 cm, XY plane, 2 revolutions", steps_per_segment=2000),
        TrajectorySpec("3c", 3, "Figure-8: lemniscate in XZ, 12×8 cm, 2 cycles", steps_per_segment=3000),
    ]


def _force_waypoints(kin: MarvinKinematics, home: np.ndarray) -> list[TrajectorySpec]:
    """Trajectory group 4: Hybrid force descent."""
    return [
        TrajectorySpec("4a", 4, "Single press: descend, hold 10 N for 1.5 s, retract"),
        TrajectorySpec("4b", 4, "Repeated press: 3 press-release cycles at 10 N"),
        TrajectorySpec("4c", 4, "Variable force: staircase 5→10→15 N, each 1 s"),
    ]


# ---------------------------------------------------------------------------
# Trajectory executors
# ---------------------------------------------------------------------------

def _run_joint_trajectory(
    robot: TwinRobot, spec: TrajectorySpec, kin: MarvinKinematics, home: np.ndarray,
) -> None:
    """Execute a joint-space trajectory variant."""
    limits = kin.joint_limits_rad
    rng = np.random.default_rng(42)
    robot.set_joint_impedance_state(0.5, 0.5, JOINT_K, JOINT_D)

    if spec.name == "1a":
        # Small sweep: 3 nearby configs
        targets = [home]
        for _ in range(3):
            dq = rng.uniform(-0.175, 0.175, 7)  # ±10°
            targets.append(np.clip(home + dq, limits[:, 0], limits[:, 1]))
        targets.append(home)
        _run_waypoint_sequence(robot, targets, spec.steps_per_segment)

    elif spec.name == "1b":
        # Full range: 4 wide configs
        targets = [home]
        for _ in range(4):
            q = home.copy()
            q[:3] = rng.uniform(-1.0, 1.0, 3)  # ±60° on J1-3
            q[3:] = rng.uniform(-0.5, 0.5, 4)  # ±30° on J4-7
            targets.append(np.clip(q, limits[:, 0], limits[:, 1]))
        targets.append(home)
        _run_waypoint_sequence(robot, targets, spec.steps_per_segment)

    elif spec.name == "1c":
        # Single joint moves
        for joint_idx in range(7):
            for sign in (+1, -1):
                target = home.copy()
                delta = 0.5236 * sign  # ±30°
                target[joint_idx] = np.clip(
                    home[joint_idx] + delta, limits[joint_idx, 0], limits[joint_idx, 1],
                )
                robot.set_joint_position_cmd(target)
                robot.spin(spec.steps_per_segment, viewer_sync=True)
            # Return to home
            robot.set_joint_position_cmd(home)
            robot.spin(spec.steps_per_segment, viewer_sync=True)

    elif spec.name == "1d":
        # Random walk
        targets = [home]
        for _ in range(5):
            q = rng.uniform(limits[:, 0], limits[:, 1], 7)
            targets.append(np.clip(q, limits[:, 0], limits[:, 1]))
        _run_waypoint_sequence(robot, targets, spec.steps_per_segment)


def _run_waypoint_sequence(
    robot: TwinRobot, targets: list[np.ndarray], steps: int,
) -> None:
    """Send a sequence of joint targets, holding each for ``steps``."""
    for target in targets:
        robot.set_joint_position_cmd(target)
        robot.spin(steps, viewer_sync=True)


def _run_cartesian_trajectory(
    robot: TwinRobot, spec: TrajectorySpec, kin: MarvinKinematics, home: np.ndarray,
) -> None:
    """Execute a Cartesian translation trajectory variant."""
    robot.set_cart_impedance_state(0.5, 0.5, CART_K, CART_D)

    # Get home TCP pose
    home_matrix, _ = kin.fk(home)
    p_home = home_matrix[:3, 3].copy()
    R_home = home_matrix[:3, :3].copy()

    def _target_at(offset: np.ndarray) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3] = R_home
        T[:3, 3] = p_home + offset
        return T

    d = 0.05  # 5 cm

    if spec.name == "2a":
        offsets = [
            np.array((d, 0, 0)), np.array((-d, 0, 0)),
            np.array((0, d, 0)), np.array((0, -d, 0)),
            np.array((0, 0, d)), np.array((0, 0, -d)),
        ]
        for off in offsets:
            robot.set_cartesian_pose_cmd(_target_at(off))
            robot.spin(spec.steps_per_segment, viewer_sync=True)

    elif spec.name == "2b":
        # 4 body diagonals
        offsets = [
            np.array((d, 0, d)), np.array((-d, 0, -d)),
            np.array((d, 0, -d)), np.array((-d, 0, d)),
        ]
        for off in offsets:
            robot.set_cartesian_pose_cmd(_target_at(off))
            robot.spin(spec.steps_per_segment, viewer_sync=True)

    elif spec.name == "2c":
        # Step response: 2 cm in Z, return; 2 cm in X, return
        for axis, delta in [(2, 0.02), (0, 0.02)]:
            off = np.zeros(3)
            off[axis] = delta
            robot.set_cartesian_pose_cmd(_target_at(off))
            robot.spin(spec.steps_per_segment, viewer_sync=True)
            robot.set_cartesian_pose_cmd(_target_at(np.zeros(3)))
            robot.spin(spec.steps_per_segment, viewer_sync=True)

    elif spec.name == "2d":
        # 8 corners of a 3 cm box
        s = 0.03
        corners = [
            np.array((sx, sy, sz))
            for sx in (s, -s) for sy in (s, -s) for sz in (s, -s)
        ]
        for off in corners:
            robot.set_cartesian_pose_cmd(_target_at(off))
            robot.spin(spec.steps_per_segment, viewer_sync=True)


def _run_curve_trajectory(
    robot: TwinRobot, spec: TrajectorySpec, kin: MarvinKinematics, home: np.ndarray,
) -> None:
    """Execute a Cartesian curve trajectory variant."""
    robot.set_cart_impedance_state(0.5, 0.5, CART_K, CART_D)

    home_matrix, _ = kin.fk(home)
    p_home = home_matrix[:3, 3].copy()
    R_home = home_matrix[:3, :3].copy()

    def _target_at(pos: np.ndarray) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3] = R_home
        T[:3, 3] = pos
        return T

    if spec.name == "3a":
        # Circle in XZ plane, radius 8 cm, 3 revs
        radius = 0.08
        for step in range(spec.steps_per_segment):
            angle = 3 * 2 * math.pi * step / spec.steps_per_segment
            pos = p_home + np.array((radius * math.cos(angle), 0.0, radius * math.sin(angle)))
            robot.set_cartesian_pose_cmd(_target_at(pos))
            robot.step(viewer_sync=(step % 10 == 0))

    elif spec.name == "3b":
        # Circle in XY plane, radius 6 cm, 2 revs
        radius = 0.06
        for step in range(spec.steps_per_segment):
            angle = 2 * 2 * math.pi * step / spec.steps_per_segment
            pos = p_home + np.array((radius * math.cos(angle), radius * math.sin(angle), 0.0))
            robot.set_cartesian_pose_cmd(_target_at(pos))
            robot.step(viewer_sync=(step % 10 == 0))

    elif spec.name == "3c":
        # Figure-8 (lemniscate) in XZ plane
        a, b = 0.06, 0.04  # half-width, half-height
        for step in range(spec.steps_per_segment):
            t = 2 * 2 * math.pi * step / spec.steps_per_segment
            x = a * math.sin(t)
            z = b * math.sin(2 * t)
            pos = p_home + np.array((x, 0.0, z))
            robot.set_cartesian_pose_cmd(_target_at(pos))
            robot.step(viewer_sync=(step % 10 == 0))


def _run_force_trajectory(
    robot: TwinRobot, spec: TrajectorySpec, kin: MarvinKinematics, home: np.ndarray,
) -> None:
    """Execute a hybrid force descent trajectory variant."""
    # Calibrate wrench sensor first
    robot.calibrate_wrench(samples=200)

    home_matrix, _ = kin.fk(home)
    p_home = home_matrix[:3, 3].copy()
    R_home = home_matrix[:3, :3].copy()

    # Board top is at Z ≈ 0.235 m (from chopping scene)
    board_z = 0.235
    safe_z = board_z + 0.06  # 6 cm above board
    contact_z = board_z + 0.005  # just above board surface

    def _cart_target(z: float) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3] = R_home
        T[:3, 3] = (p_home[0], p_home[1], z)
        return T

    # Switch to Cartesian impedance for approach
    robot.set_cart_impedance_state(0.3, 0.3, CART_K, CART_D)
    robot.set_cartesian_pose_cmd(_cart_target(safe_z))
    robot.spin(300, viewer_sync=True)

    if spec.name == "4a":
        # Single press
        _force_press(robot, kin, safe_z, contact_z, R_home, p_home, target_force=10.0, hold_steps=750)

    elif spec.name == "4b":
        # Repeated press: 3 cycles
        for cycle in range(3):
            print(f"\n  --- Press cycle {cycle + 1}/3 ---")
            _force_press(robot, kin, safe_z, contact_z, R_home, p_home, target_force=10.0, hold_steps=500)
            # Retract
            robot.set_cart_impedance_state(0.3, 0.3, CART_K, CART_D)
            robot.set_cartesian_pose_cmd(_cart_target(safe_z))
            robot.spin(300, viewer_sync=True)

    elif spec.name == "4c":
        # Variable force staircase
        for force_n in (5.0, 10.0, 15.0):
            print(f"\n  --- Force level: {force_n} N ---")
            _force_press(robot, kin, safe_z, contact_z, R_home, p_home, target_force=force_n, hold_steps=500)
            # Retract slightly between levels
            robot.set_cart_impedance_state(0.3, 0.3, CART_K, CART_D)
            robot.set_cartesian_pose_cmd(_cart_target(safe_z))
            robot.spin(200, viewer_sync=True)

    # Final retract
    robot.set_cart_impedance_state(0.3, 0.3, CART_K, CART_D)
    robot.set_cartesian_pose_cmd(_cart_target(safe_z))
    robot.spin(300, viewer_sync=True)


def _force_press(
    robot: TwinRobot, kin: MarvinKinematics,
    safe_z: float, contact_z: float,
    R_home: np.ndarray, p_home: np.ndarray,
    target_force: float, hold_steps: int,
) -> None:
    """Execute one press cycle: descend → contact → force hold → retract."""
    def _cart_target(z: float) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3] = R_home
        T[:3, 3] = (p_home[0], p_home[1], z)
        return T

    # Descend under Cartesian impedance
    robot.set_cart_impedance_state(0.3, 0.3, CART_K, CART_D)
    descend_steps = 500
    for step in range(descend_steps):
        alpha = (step + 1) / descend_steps
        z = safe_z + (contact_z - safe_z) * alpha
        robot.set_cartesian_pose_cmd(_cart_target(z))
        robot.step(viewer_sync=(step % 20 == 0))
        # Check for contact
        wrench = robot.get_wrench()
        if abs(wrench[2]) > 2.0:
            print(f"    Contact detected at step {step}, Fz={wrench[2]:.2f} N")
            break

    # Switch to force control
    fx_dir = (0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
    robot.set_force_state(0.3, 0.3, CART_K, CART_D, fx_dir, fc_adj_lmt=0.03)
    robot.set_force_cmd(target_force)
    print(f"    Force hold: target={target_force} N, {hold_steps} steps")
    robot.spin(hold_steps, viewer_sync=True)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def _all_specs() -> list[TrajectorySpec]:
    kin = MarvinKinematics("right", unit_mode="si")
    home = RIGHT_HOME_RAD
    return (
        _joint_waypoints(kin, home)
        + _cartesian_waypoints(kin, home)
        + _curve_waypoints(kin, home)
        + _force_waypoints(kin, home)
    )


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Twin Control Trajectory Demo")
    parser.add_argument("--trajectory", type=str, default=None,
                        help="Trajectory variant (e.g. '1a', '2') or 'all'")
    parser.add_argument("--all", action="store_true",
                        help="Run all 14 trajectory variants")
    parser.add_argument("--headless", action="store_true",
                        help="Run without MuJoCo viewer")
    parser.add_argument("--log", type=str, default=None,
                        help="CSV log output path")
    parser.add_argument("--hold-open", action="store_true",
                        help="Keep the MuJoCo viewer open after trajectories complete")
    args = parser.parse_args(argv)

    all_specs = _all_specs()

    # Select specs
    if args.all or args.trajectory == "all":
        selected = all_specs
    elif args.trajectory is not None:
        sel = args.trajectory.strip()
        # Group selector: "1", "2", "3", "4"
        if sel in ("1", "2", "3", "4"):
            group = int(sel)
            selected = [s for s in all_specs if s.group == group]
        else:
            selected = [s for s in all_specs if s.name == sel]
        if not selected:
            print(f"Unknown trajectory: {args.trajectory}")
            print(f"Available: {[s.name for s in all_specs]}")
            return 1
    else:
        parser.print_help()
        return 1

    print(f"Running {len(selected)} trajectory variant(s):")
    for s in selected:
        print(f"  {s.name}: {s.description}")

    robot = TwinRobot(arm_name="right", unit_mode="si", control_hz=500.0)
    kin = MarvinKinematics("right", unit_mode="si")
    home = RIGHT_HOME_RAD

    try:
        robot.connect(viewer=not args.headless, realtime=not args.headless)
        robot.runtime.set_arm_positions("right", home)
        kin.set_runtime(robot.runtime)  # Mount MuJoCo runtime for FK/IK

        for spec in selected:
            print(f"\n{'='*60}")
            print(f"  Trajectory {spec.name}: {spec.description}")
            print(f"{'='*60}")

            # Reset visual trail and robot state for this trajectory.
            robot.clear_trail()
            robot.runtime.set_arm_positions("right", home)
            robot.set_joint_position_cmd(home)

            if spec.group == 1:
                _run_joint_trajectory(robot, spec, kin, home)
            elif spec.group == 2:
                _run_cartesian_trajectory(robot, spec, kin, home)
            elif spec.group == 3:
                _run_curve_trajectory(robot, spec, kin, home)
            elif spec.group == 4:
                _run_force_trajectory(robot, spec, kin, home)

            print(f"  ✓ {spec.name} complete")

        if args.log:
            robot.write_csv(args.log)

        if args.hold_open and not args.headless:
            print("\nViewer held open. Close the MuJoCo window or press Ctrl-C to exit.")
            robot.hold_viewer_open()

    finally:
        robot.close()

    print("\nAll trajectories complete.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
