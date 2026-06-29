#!/usr/bin/env python3
"""Quick test: verify orange trail markers appear in the MuJoCo viewer.

Run this on a machine with a display::

    python3 test_trail_markers.py

You should see the right arm move through a few joint targets, with orange
spheres marking the TCP (force sensor) path.
"""

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent if __file__.endswith(".py") else Path.cwd()
sys.path.insert(0, str(_ROOT / "src"))
sys.path.insert(0, str(_ROOT / "src" / "twin_core"))
sys.path.insert(0, str(_ROOT / "src" / "twin_description"))
sys.path.insert(0, str(_ROOT / "src" / "twin_mujoco"))

import numpy as np
from twin_control.robot import TwinRobot, RIGHT_HOME_RAD, _DEFAULT_JOINT_K, _DEFAULT_JOINT_D

robot = TwinRobot(arm_name="right", unit_mode="si", control_hz=500.0)
robot.connect(viewer=True)
robot.runtime.set_arm_positions("right", RIGHT_HOME_RAD)

# Joint impedance — move through 3 targets
targets = [
    RIGHT_HOME_RAD + np.array((0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    RIGHT_HOME_RAD + np.array((0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0)),
    RIGHT_HOME_RAD + np.array((0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0)),
    RIGHT_HOME_RAD,
]

robot.set_joint_impedance_state(0.5, 0.5, _DEFAULT_JOINT_K, _DEFAULT_JOINT_D)

for i, target in enumerate(targets):
    print(f"Target {i+1}/{len(targets)}: {target.round(2)}")
    robot.set_joint_position_cmd(target)
    robot.spin(500, viewer_sync=True)

actual_count, target_count = robot.trail_point_counts
print(f"Trail markers: actual={actual_count}, target={target_count} points recorded")
print("You should see orange spheres along the TCP path in the viewer.")
print("Press Ctrl+C or close the viewer window to exit.")

try:
    robot.hold_viewer_open()
except KeyboardInterrupt:
    pass
finally:
    robot.close()
