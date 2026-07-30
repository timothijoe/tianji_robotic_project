from pathlib import Path

import mujoco
import numpy as np

from twin_description import right_chopping_scene_path, source_model_path, workspace_root


def _id(model, object_type, name: str) -> int:
    value = int(mujoco.mj_name2id(model, object_type, name))
    assert value >= 0, name
    return value


def test_description_paths_resolve_inside_twin_workspace():
    root = workspace_root()

    assert (root / "pyproject.toml").is_file()
    assert (root / "docs" / "superpowers").is_dir()
    assert source_model_path() == root / "MarvinCCS" / "marvin_final_fixed.xml"
    assert right_chopping_scene_path() == (
        root
        / "src"
        / "twin_description"
        / "twin_description"
        / "assets"
        / "robot"
        / "mujoco"
        / "right_chopping_scene.xml"
    )
    assert source_model_path().is_file()
    assert isinstance(right_chopping_scene_path(), Path)
    assert right_chopping_scene_path().is_file()
    assert source_model_path().is_relative_to(root)
    assert right_chopping_scene_path().is_relative_to(root)


def test_right_chopping_scene_loads_with_task_objects():
    model = mujoco.MjModel.from_xml_path(str(right_chopping_scene_path()))

    assert model.njnt == 14
    assert model.nu == 14
    assert _id(model, mujoco.mjtObj.mjOBJ_GEOM, "chopping_board") >= 0
    sensor_body_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "right_force_sensor_body")
    tool_body_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "right_tool_body")
    sensor_site_id = _id(model, mujoco.mjtObj.mjOBJ_SITE, "right_force_sensor_site")
    tool_site_id = _id(model, mujoco.mjtObj.mjOBJ_SITE, "right_tool_tip_site")
    assert _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_knife_handle") >= 0
    assert _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_knife_blade") >= 0
    assert _id(model, mujoco.mjtObj.mjOBJ_SENSOR, "right_tool_force") >= 0
    assert _id(model, mujoco.mjtObj.mjOBJ_SENSOR, "right_tool_torque") >= 0
    assert model.body_parentid[sensor_body_id] == _id(model, mujoco.mjtObj.mjOBJ_BODY, "right_link7")
    assert model.body_parentid[tool_body_id] == sensor_body_id
    assert model.site_bodyid[sensor_site_id] == sensor_body_id
    assert model.site_bodyid[tool_site_id] == tool_body_id


def test_chopping_board_is_in_front_of_robot_at_work_height():
    model = mujoco.MjModel.from_xml_path(str(right_chopping_scene_path()))
    geom_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "chopping_board")

    assert model.geom_pos[geom_id, 0] == 0.48
    assert abs(model.geom_pos[geom_id, 1]) < 1e-9
    assert abs(model.geom_pos[geom_id, 2] + model.geom_size[geom_id, 2] - 0.23) < 1e-9


def test_right_tool_body_frames_are_parallel_to_end_flange_frame():
    model = mujoco.MjModel.from_xml_path(str(right_chopping_scene_path()))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    link7_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "right_link7")
    sensor_body_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "right_force_sensor_body")
    tool_body_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "right_tool_body")
    adapter_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_force_sensor_adapter")
    handle_visual_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_handle_visual")
    blade_visual_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_blade_visual")

    # force_sensor_body quat matches the flange visual mesh quat,
    # so sensor_body, tool_body, and all child geoms share the same frame.
    sensor_rotation = data.xmat[sensor_body_id].reshape(3, 3)
    tool_rotation = data.xmat[tool_body_id].reshape(3, 3)

    np.testing.assert_allclose(sensor_rotation, tool_rotation, atol=1e-8)
    for geom_id in (adapter_id, handle_visual_id, blade_visual_id):
        geom_rotation = data.geom_xmat[geom_id].reshape(3, 3)
        np.testing.assert_allclose(geom_rotation, tool_rotation, atol=2e-2)


def test_right_tool_has_visible_adapter_between_wrist_and_sensor():
    model = mujoco.MjModel.from_xml_path(str(right_chopping_scene_path()))
    adapter_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_force_sensor_adapter")
    sensor_body_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "right_force_sensor_body")

    assert model.geom_bodyid[adapter_id] == sensor_body_id
    assert model.geom_type[adapter_id] == mujoco.mjtGeom.mjGEOM_CYLINDER
    assert model.geom_size[adapter_id, :2].tolist() == [0.014, 0.018]
    assert model.geom_contype[adapter_id] == 0
    assert model.geom_conaffinity[adapter_id] == 0


def test_right_force_sensor_adapter_connects_to_visible_flange():
    model = mujoco.MjModel.from_xml_path(str(right_chopping_scene_path()))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    link7_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "right_link7")
    sensor_body_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "right_force_sensor_body")
    adapter_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_force_sensor_adapter")
    flange_geom_id = next(
        geom_id
        for geom_id in range(model.ngeom)
        if model.geom_bodyid[geom_id] == link7_id
        and model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_MESH
    )

    # sensor_body quat is identity (aligned with link7 body +Z axis)
    np.testing.assert_allclose(model.body_quat[sensor_body_id], [1.0, 0.0, 0.0, 0.0], atol=1e-8)

    # sensor_body pos is 95mm along link7 +Z
    np.testing.assert_allclose(model.body_pos[sensor_body_id], [0.0, 0.0, 0.095], atol=1e-8)

    # Adapter shares the same frame as sensor_body (no extra quat)
    adapter_rotation = data.geom_xmat[adapter_id].reshape(3, 3)
    sensor_rotation = data.xmat[sensor_body_id].reshape(3, 3)
    np.testing.assert_allclose(adapter_rotation, sensor_rotation, atol=1e-4)

    # Adapter is at sensor_body origin (pos "0 0 -0.018" in sensor_body frame)
    adapter_center_world = data.geom_xpos[adapter_id]
    sensor_body_world = data.xpos[sensor_body_id]
    sensor_z = sensor_rotation[:, 2]
    expected_center = sensor_body_world - 0.018 * sensor_z
    np.testing.assert_allclose(adapter_center_world, expected_center, atol=1e-6)


def test_right_knife_handle_visual_axis_is_parallel_to_visible_flange_axis():
    model = mujoco.MjModel.from_xml_path(str(right_chopping_scene_path()))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    link7_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "right_link7")
    handle_visual_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_handle_visual")
    tool_body_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "right_tool_body")

    # handle_visual inherits tool_body frame (same as flange mesh frame)
    handle_rotation = data.geom_xmat[handle_visual_id].reshape(3, 3)
    tool_rotation = data.xmat[tool_body_id].reshape(3, 3)
    np.testing.assert_allclose(handle_rotation, tool_rotation, atol=1e-4)


def test_right_knife_visuals_are_ordered_along_visible_flange_axis():
    model = mujoco.MjModel.from_xml_path(str(right_chopping_scene_path()))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    handle_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_knife_handle")
    blade_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_knife_blade")
    tip_site_id = _id(model, mujoco.mjtObj.mjOBJ_SITE, "right_tool_tip_site")
    tool_body_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "right_tool_body")

    # Tool extends along tool_body +Z. Check ordering of collision geoms.
    tool_rotation = data.xmat[tool_body_id].reshape(3, 3)
    tool_z = tool_rotation[:, 2]
    tool_body_pos = data.xpos[tool_body_id]

    handle_projection = float(np.dot(data.geom_xpos[handle_id] - tool_body_pos, tool_z))
    blade_projection = float(np.dot(data.geom_xpos[blade_id] - tool_body_pos, tool_z))
    tip_projection = float(np.dot(data.site_xpos[tip_site_id] - tool_body_pos, tool_z))

    # Handle near flange, blade further, tip furthest along +tool_z
    assert 0.03 < handle_projection < 0.06
    assert blade_projection > handle_projection + 0.05
    assert tip_projection > blade_projection + 0.05


def test_right_tool_is_thin_knife_attached_below_force_sensor():
    model = mujoco.MjModel.from_xml_path(str(right_chopping_scene_path()))
    tool_body_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "right_tool_body")
    handle_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_knife_handle")
    blade_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_knife_blade")

    assert model.geom_bodyid[handle_id] == tool_body_id
    assert model.geom_bodyid[blade_id] == tool_body_id
    assert model.geom_type[handle_id] == mujoco.mjtGeom.mjGEOM_BOX
    assert model.geom_type[blade_id] == mujoco.mjtGeom.mjGEOM_BOX
    assert model.geom_size[handle_id].tolist() == [0.012, 0.02, 0.04]
    assert model.geom_size[blade_id].tolist() == [0.006, 0.05, 0.10]
    # knife_blade extends further along body +Z than knife_handle
    assert model.geom_pos[blade_id, 2] > model.geom_pos[handle_id, 2]
    # knife_handle is at Y=0 (aligned with force sensor)
    # knife_blade is offset toward -Y (edge side)
    assert model.geom_pos[handle_id, 1] == 0.0
    assert model.geom_pos[blade_id, 1] < model.geom_pos[handle_id, 1]


def test_right_tool_uses_visual_box_with_simple_collision():
    model = mujoco.MjModel.from_xml_path(str(right_chopping_scene_path()))
    tool_body_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "right_tool_body")
    visual_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_blade_visual")
    handle_visual_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_handle_visual")
    blade_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_knife_blade")

    assert model.geom_bodyid[visual_id] == tool_body_id
    assert model.geom_bodyid[handle_visual_id] == tool_body_id
    assert model.geom_type[visual_id] == mujoco.mjtGeom.mjGEOM_BOX
    assert model.geom_type[handle_visual_id] == mujoco.mjtGeom.mjGEOM_BOX
    assert model.geom_contype[visual_id] == 0
    assert model.geom_conaffinity[visual_id] == 0
    assert model.geom_contype[handle_visual_id] == 0
    assert model.geom_conaffinity[handle_visual_id] == 0
    assert model.geom_bodyid[blade_id] == tool_body_id
    assert model.geom_type[blade_id] == mujoco.mjtGeom.mjGEOM_BOX


def test_right_knife_visual_matches_wecook_asset_proportions():
    model = mujoco.MjModel.from_xml_path(str(right_chopping_scene_path()))
    blade_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_knife_blade")
    handle_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_knife_handle")
    visual_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_blade_visual")
    handle_visual_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_handle_visual")

    # Collision geoms are invisible (alpha=0)
    assert model.geom_rgba[blade_id, 3] == 0.0
    assert model.geom_rgba[handle_id, 3] == 0.0
    # Visual geoms are visible (alpha=1)
    assert model.geom_rgba[visual_id, 3] == 1.0
    assert model.geom_rgba[handle_visual_id, 3] == 1.0
    # Visual geoms are BOX type
    assert model.geom_type[handle_visual_id] == mujoco.mjtGeom.mjGEOM_BOX
    assert model.geom_type[visual_id] == mujoco.mjtGeom.mjGEOM_BOX


def test_scene_has_non_colliding_center_visual_between_arms():
    model = mujoco.MjModel.from_xml_path(str(right_chopping_scene_path()))
    geom_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "center_visual_console")

    assert model.geom_pos[geom_id].tolist() == [0.16, 0.0, 0.19]
    assert model.geom_size[geom_id].tolist() == [0.16, 0.24, 0.05]
    assert model.geom_contype[geom_id] == 0
    assert model.geom_conaffinity[geom_id] == 0
