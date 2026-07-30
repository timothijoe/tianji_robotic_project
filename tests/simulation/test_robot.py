import numpy as np
import pytest

from twin_sim.robot import RIGHT_HOME_RAD, NumericalSafetyError, RightArmRobot


def test_position_target_is_tracked():
    robot = RightArmRobot()
    robot.reset(RIGHT_HOME_RAD)
    target = RIGHT_HOME_RAD + np.array([0.02, 0, 0, 0, 0, 0, 0])
    robot.command(target)
    for _ in range(250):
        robot.step(0.002)
    assert np.linalg.norm(robot.joint_positions - target) < 0.03
    robot.close()


def test_invalid_command_is_rejected():
    robot = RightArmRobot()
    with pytest.raises(ValueError, match="7 finite"):
        robot.command([float("nan")] * 7)
    robot.close()


def test_nonfinite_state_stops():
    robot = RightArmRobot()
    robot.sim.data.qpos[robot.sim.right.qpos_ids[0]] = np.nan
    with pytest.raises(NumericalSafetyError):
        robot.step(0.002)
    robot.close()


def test_step_requires_a_positive_integer_number_of_physics_ticks():
    robot = RightArmRobot()
    with pytest.raises(ValueError, match="positive integer multiple"):
        robot.step(0.003)
    robot.close()


def test_step_rejects_a_nonfinite_control_period():
    robot = RightArmRobot()
    with pytest.raises(ValueError, match="positive integer multiple"):
        robot.step(float("nan"))
    robot.close()


def test_step_rejects_a_near_multiple_control_period():
    robot = RightArmRobot()
    with pytest.raises(ValueError, match="positive integer multiple"):
        robot.step(0.002000000001)
    robot.close()


def test_reset_holds_the_left_arm_and_exposes_right_arm_state():
    robot = RightArmRobot()
    left_hold = robot.sim.data.qpos[robot.sim.left.qpos_ids].copy()
    robot.reset(RIGHT_HOME_RAD)

    np.testing.assert_allclose(robot.joint_positions, RIGHT_HOME_RAD)
    np.testing.assert_allclose(robot.joint_velocities, np.zeros(7))
    assert robot.tcp_pose().shape == (7,)
    assert np.isfinite(robot.tcp_pose()).all()

    robot.command(RIGHT_HOME_RAD)
    robot.step(0.002)

    np.testing.assert_allclose(robot.sim.data.ctrl[robot.sim.left.actuator_ids], left_hold)
    robot.close()
