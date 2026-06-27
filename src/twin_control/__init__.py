# twin_control: Unified kinematics and control for Marvin CCS dual-arm robot in MuJoCo.
#
# Layers (no circular dependencies):
#   kinematics.py   — numpy only
#   controller.py   — numpy + kinematics
#   robot.py        — numpy + mujoco + twin_mujoco.runtime + kinematics + controller
