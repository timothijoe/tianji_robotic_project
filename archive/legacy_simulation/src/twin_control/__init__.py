# twin_control: Unified kinematics and control for Marvin CCS dual-arm robot in MuJoCo.
#
# Modules
# -------
# rotation      — SO(3) utilities: quaternion, rotation error
# kinematics    — FK (MuJoCo), IK (DLS), Jacobian, unit_mode
# controller    — UnifiedController: joint / Cartesian impedance, force control
# robot         — TwinRobot: SDK-style interface, MuJoCo glue
# trail         — TcpTrail: viewer trail markers
# chopping      — TwinRobotChopper: SDK-driven chopping state machine
# chopping_cli  — CLI entry point for the chopping demo
