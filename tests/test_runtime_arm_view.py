import numpy as np

from twin_description import right_chopping_scene_path
from twin_mujoco.chopping import LEFT_HOME_Q, RIGHT_CHOPPING_HOME_Q
from twin_mujoco import TwinMujocoRuntime


def test_runtime_loads_full_dual_arm_scene_and_exposes_arm_views():
    runtime = TwinMujocoRuntime.load(right_chopping_scene_path())

    assert runtime.joint_names == tuple(
        [f"left_joint{i}" for i in range(1, 8)]
        + [f"right_joint{i}" for i in range(1, 8)]
    )
    assert runtime.actuator_names == tuple(
        [f"act_left_joint{i}" for i in range(1, 8)]
        + [f"act_right_joint{i}" for i in range(1, 8)]
    )
    assert runtime.arm_view("right").joint_names == tuple(
        f"right_joint{i}" for i in range(1, 8)
    )


def test_right_arm_view_exposes_state_limits_and_tool_jacobian():
    runtime = TwinMujocoRuntime.load(right_chopping_scene_path())
    right = runtime.arm_view("right")

    runtime.reset()
    assert right.joint_positions.shape == (7,)
    assert right.joint_velocities.shape == (7,)
    assert right.effort_limits.shape == (7,)
    assert np.all(right.effort_limits > 0)
    assert right.site_jacobian("right_tool_tip_site").shape == (6, 7)


def test_right_arm_site_pose_is_valid_immediately_after_load():
    runtime = TwinMujocoRuntime.load(right_chopping_scene_path())
    right = runtime.arm_view("right")

    position, rotation = right.site_pose("right_tool_tip_site")

    assert position.shape == (3,)
    assert rotation.shape == (3, 3)
    assert not np.allclose(position, np.zeros(3))


def test_right_arm_torque_application_does_not_write_left_actuators():
    runtime = TwinMujocoRuntime.load(right_chopping_scene_path())
    right = runtime.arm_view("right")

    applied = right.apply_torque(np.ones(7))

    assert applied.shape == (7,)
    np.testing.assert_allclose(runtime.data.ctrl[:7], np.zeros(7))
    np.testing.assert_allclose(runtime.data.ctrl[7:], np.ones(7))


def test_right_arm_torque_application_preserves_existing_left_actuators():
    runtime = TwinMujocoRuntime.load(right_chopping_scene_path())
    right = runtime.arm_view("right")
    runtime.data.ctrl[:7] = np.arange(1.0, 8.0)

    right.apply_torque(np.ones(7))

    np.testing.assert_allclose(runtime.data.ctrl[:7], np.arange(1.0, 8.0))
    np.testing.assert_allclose(runtime.data.ctrl[7:], np.ones(7))


def test_right_arm_bias_torque_is_finite_copy():
    runtime = TwinMujocoRuntime.load(right_chopping_scene_path())
    right = runtime.arm_view("right")

    bias = right.bias_torque

    assert bias.shape == (7,)
    assert np.all(np.isfinite(bias))

    bias[:] = 0.0
    assert not np.allclose(right.bias_torque, np.zeros(7))


def test_safe_home_geometry_is_valid_for_right_chopping():
    runtime = TwinMujocoRuntime.load(right_chopping_scene_path())
    runtime.reset()
    runtime.set_arm_positions("left", LEFT_HOME_Q)
    runtime.set_arm_positions("right", RIGHT_CHOPPING_HOME_Q)
    right = runtime.arm_view("right")
    tool_position, tool_rotation = right.site_pose("right_tool_tip_site")
    sensor_position, _ = right.site_pose("right_force_sensor_site")
    board_id = runtime._id(__import__("mujoco").mjtObj.mjOBJ_GEOM, "chopping_board")
    board_center = runtime.model.geom_pos[board_id]
    board_top = runtime.model.geom_pos[board_id, 2] + runtime.model.geom_size[board_id, 2]
    link7_id = runtime._id(__import__("mujoco").mjtObj.mjOBJ_BODY, "right_link7")
    link7_position = runtime.data.xpos[link7_id]

    board_half_size = runtime.model.geom_size[board_id, :2]
    board_offset = np.abs(tool_position[:2] - board_center[:2])

    assert np.all(board_offset < board_half_size - 0.04)
    assert abs(tool_position[2] - (board_top + 0.08)) < 2e-3
    assert tool_rotation[2, 2] < -0.98
    assert sensor_position[2] > tool_position[2]
    np.testing.assert_allclose(link7_position[:2], tool_position[:2], atol=3e-2)
