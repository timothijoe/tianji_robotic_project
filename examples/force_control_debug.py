#!/usr/bin/env python3
"""Trace the complete MuJoCo force-control pipeline.

Run after sourcing install/setup.bash. Use --break-on-force to enter pdb when
HYBRID_FORCE_Z starts; inspect the local variables `sample`, `state`, and
`robot`.
"""

from __future__ import annotations

import argparse

import numpy as np

from cook_description.paths import CHOPPING_MJCF_PATH
from cook_mujoco.chopping import ChoppingConfig, MujocoForceRobot
from cook_mujoco.control.force_control import ChoppingPhase


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--force", type=float, default=10.0)
    result.add_argument("--hold", type=float, default=1.0)
    result.add_argument("--debug-hz", type=float, default=20.0)
    result.add_argument("--viewer", action="store_true")
    result.add_argument("--realtime", action="store_true")
    result.add_argument("--break-on-force", action="store_true")
    result.add_argument("--log", default="force_control_debug.csv")
    return result


def vec(values, digits=3) -> str:
    return np.array2string(
        np.asarray(values, dtype=float),
        precision=digits,
        suppress_small=True,
        separator=",",
    )


def main(argv=None) -> int:
    args = parser().parse_args(argv)
    robot = MujocoForceRobot(
        CHOPPING_MJCF_PATH,
        viewer=args.viewer,
        realtime=args.realtime,
    )
    robot.connect()
    robot.initialize()

    last_time = -1e9
    previous_phase = None
    stopped = False

    def trace(sample):
        nonlocal last_time, previous_phase, stopped
        state = robot.controller.debug_state()
        changed = sample.phase != previous_phase
        period = 1.0 / max(args.debug_hz, 1e-6)
        if changed or sample.time_s - last_time >= period:
            raw = np.asarray(sample.raw_wrench)
            compensated = np.asarray(sample.compensated_wrench)
            print(
                f"\nt={sample.time_s:.3f}s phase={sample.phase.value} "
                f"mode={sample.control_mode}"
            )
            print(
                f"  sensor: raw_Fz={raw[2]:+.3f} N "
                f"comp_Fz={compensated[2]:+.3f} N "
                f"filtered_Fz={state.get('filtered_force_n', 0.0):+.3f} N"
            )
            print(
                f"  admittance: target={state.get('target_force_n', 0.0):+.3f} N "
                f"error={state.get('force_error_n', 0.0):+.3f} N "
                f"delta_z={state.get('admittance_offset_m', 0.0) * 1000:+.3f} mm"
            )
            print(
                "  cartesian: position_error="
                f"{vec(state.get('position_error', np.zeros(3)), 5)} m "
                "rotation_error="
                f"{vec(state.get('orientation_error', np.zeros(3)), 5)} rad"
            )
            print(
                "  wrench_cmd_world: F="
                f"{vec(state.get('commanded_force_world', np.zeros(3)))} N "
                "M="
                f"{vec(state.get('commanded_torque_world', np.zeros(3)))} Nm"
            )
            print(
                "  tau_cmd="
                f"{vec(state.get('joint_torque_command', np.zeros(7)))} Nm"
            )
            last_time = sample.time_s

        if (
            args.break_on_force
            and not stopped
            and sample.phase == ChoppingPhase.FORCE_HOLD
        ):
            stopped = True
            print("\nEntered FORCE_HOLD. Inspect: sample, state, robot")
            breakpoint()
        previous_phase = sample.phase

    try:
        robot.execute_chopping_trajectory(
            config=ChoppingConfig(
                cycles=1,
                target_force_n=args.force,
                force_hold_s=args.hold,
            ),
            log_path=args.log,
            status_callback=trace,
        )
        print("\nsummary:", robot.summary())
        return 0
    finally:
        robot.close()


if __name__ == "__main__":
    raise SystemExit(main())
