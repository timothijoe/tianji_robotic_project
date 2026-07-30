import numpy as np

from twin_description import right_chopping_scene_path
from twin_mujoco import TwinMujocoRuntime
from twin_mujoco.control import CartesianForceController, WrenchCalibrator, downward_tool_rotation


def test_downward_tool_rotation_has_tool_z_aligned_to_world_negative_z():
    rotation = downward_tool_rotation()

    np.testing.assert_allclose(rotation[:, 2], [0.0, 0.0, -1.0])
    np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-8)


def test_wrench_calibrator_removes_stationary_bias():
    calibrator = WrenchCalibrator(tool_mass_kg=0.22)
    rotation = np.eye(3)
    raw = np.array([0.1, 0.2, 2.3, 0.01, 0.02, 0.03])

    calibrator.calibrate([raw, raw], rotation)
    compensated = calibrator.compensate(raw, rotation)

    np.testing.assert_allclose(compensated, np.zeros(6), atol=1e-9)


def test_cartesian_controller_produces_finite_limited_right_arm_torque():
    runtime = TwinMujocoRuntime.load(right_chopping_scene_path())
    right = runtime.arm_view("right")
    controller = CartesianForceController(right)
    position, rotation = right.site_pose("right_tool_tip_site")


    controller.set_target(position, rotation)
    torque = controller.compute(np.zeros(6))

    assert torque.shape == (7,)
    assert np.all(np.isfinite(torque))
    assert np.all(np.abs(torque) <= right.effort_limits + 1e-9)


def test_cartesian_controller_force_mode_updates_and_resets_force_state():
    runtime = TwinMujocoRuntime.load(right_chopping_scene_path())
    right = runtime.arm_view("right")
    controller = CartesianForceController(right)
    position, rotation = right.site_pose("right_tool_tip_site")

    controller.set_target(position, rotation)
    controller.enable_force(True, target_force_n=10.0)
    torque = controller.compute(np.array([0.0, 0.0, 4.0, 0.0, 0.0, 0.0]))

    assert torque.shape == (7,)
    assert controller._filtered_force > 0.0
    assert controller._force_position_offset > 0.0

    controller.enable_force(False)

    assert controller._force_position_offset == 0.0
    assert controller._filtered_force == 0.0
