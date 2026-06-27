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

    assert model.geom_pos[geom_id, 0] == 0.38
    assert abs(model.geom_pos[geom_id, 1]) < 1e-9
    assert model.geom_pos[geom_id, 2] + model.geom_size[geom_id, 2] == 0.26


def test_right_tool_body_frames_are_parallel_to_end_flange_frame():
    model = mujoco.MjModel.from_xml_path(str(right_chopping_scene_path()))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    link7_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "right_link7")
    sensor_body_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "right_force_sensor_body")
    tool_body_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "right_tool_body")
    adapter_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_force_sensor_adapter")
    handle_visual_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_cleaver_handle_visual")
    blade_visual_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_cleaver_visual")

    link_rotation = data.xmat[link7_id].reshape(3, 3)
    sensor_rotation = data.xmat[sensor_body_id].reshape(3, 3)
    tool_rotation = data.xmat[tool_body_id].reshape(3, 3)

    # sensor_body and tool_body share the same frame as link7 (no quat)
    np.testing.assert_allclose(sensor_rotation, link_rotation, atol=1e-8)
    np.testing.assert_allclose(tool_rotation, link_rotation, atol=1e-8)

    # Geoms with quat="0.5 0.5 -0.5 0.5" have their Z axis rotated
    # from link7 Z to link7 -Y (pointing downward through the adapter).
    # Their Z axis should be perpendicular to link7 Z (dot ≈ 0).
    link_z = link_rotation[:, 2]
    for geom_id in (adapter_id, handle_visual_id, blade_visual_id):
        geom_axis = data.geom_xmat[geom_id].reshape(3, 3)[:, 2]
        assert abs(float(np.dot(geom_axis, link_z))) < 0.02


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

    # sensor_body quat is identity (sensor frame matches link7 frame)
    np.testing.assert_allclose(model.body_quat[sensor_body_id], [1.0, 0.0, 0.0, 0.0], atol=1e-8)

    # Adapter has quat="0.5 0.5 -0.5 0.5": its local Z axis maps to link7 frame -Y.
    # The adapter cylinder is at pos="0 0 -0.018" in sensor_body (== link7) frame.
    # With the quat, this local offset maps to 0.018 along -link7_Y in world.
    adapter_axis = data.geom_xmat[adapter_id].reshape(3, 3)[:, 2]
    flange_axis = data.xmat[link7_id].reshape(3, 3)[:, 2]
    link7_y = data.xmat[link7_id].reshape(3, 3)[:, 1]

    # Adapter Z axis is perpendicular to flange Z (points along -link7_Y, downward)
    assert abs(float(np.dot(adapter_axis, flange_axis))) < 0.02
    # Adapter Z axis is parallel to -link7_Y
    assert abs(float(np.dot(adapter_axis, -link7_y))) > 0.98

    # Adapter center is offset from sensor_body origin along -link7_Z by 0.018.
    # (geom pos is always in parent body local frame, unaffected by geom quat).
    sensor_body_world = data.xpos[sensor_body_id]
    link7_z = data.xmat[link7_id].reshape(3, 3)[:, 2]
    expected_adapter_center = sensor_body_world - 0.018 * link7_z
    np.testing.assert_allclose(data.geom_xpos[adapter_id], expected_adapter_center, atol=1e-6)


def test_right_knife_handle_visual_axis_is_parallel_to_visible_flange_axis():
    model = mujoco.MjModel.from_xml_path(str(right_chopping_scene_path()))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    link7_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "right_link7")
    handle_visual_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_cleaver_handle_visual")
    flange_geom_id = next(
        geom_id
        for geom_id in range(model.ngeom)
        if model.geom_bodyid[geom_id] == link7_id
        and model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_MESH
    )

    # With quat="0.5 0.5 -0.5 0.5", the handle Z axis is perpendicular
    # to the flange Z axis (it points downward through the tool adapter).
    handle_axis = data.geom_xmat[handle_visual_id].reshape(3, 3)[:, 2]
    flange_axis = data.xmat[link7_id].reshape(3, 3)[:, 2]

    assert abs(float(np.dot(handle_axis, flange_axis))) < 0.02


def test_right_knife_visuals_are_ordered_along_visible_flange_axis():
    model = mujoco.MjModel.from_xml_path(str(right_chopping_scene_path()))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    link7_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "right_link7")
    handle_visual_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_cleaver_handle_visual")
    blade_visual_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_cleaver_visual")
    flange_geom_id = next(
        geom_id
        for geom_id in range(model.ngeom)
        if model.geom_bodyid[geom_id] == link7_id
        and model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_MESH
    )

    # With quat="0.5 0.5 -0.5 0.5", the tool axis is along link7 -Y (downward).
    # Project positions onto the link7 -Y axis to check ordering.
    link7_y_axis = data.xmat[link7_id].reshape(3, 3)[:, 1]
    flange_origin = data.geom_xpos[flange_geom_id]
    handle_projection = float(np.dot(data.geom_xpos[handle_visual_id] - flange_origin, -link7_y_axis))
    blade_projection = float(np.dot(data.geom_xpos[blade_visual_id] - flange_origin, -link7_y_axis))

    # Handle should be below the flange (positive projection along -link7_Y)
    assert 0.02 < handle_projection < 0.09
    # Blade should be further below the handle
    assert blade_projection > handle_projection + 0.07


def test_right_tool_is_thin_knife_attached_below_force_sensor():
    model = mujoco.MjModel.from_xml_path(str(right_chopping_scene_path()))
    tool_body_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "right_tool_body")
    handle_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_knife_handle")
    blade_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_knife_blade")
    removed_probe_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_tool_shaft"))

    assert removed_probe_id == -1
    assert model.geom_bodyid[handle_id] == tool_body_id
    assert model.geom_bodyid[blade_id] == tool_body_id
    assert model.geom_type[handle_id] == mujoco.mjtGeom.mjGEOM_BOX
    assert model.geom_type[blade_id] == mujoco.mjtGeom.mjGEOM_BOX
    assert model.geom_size[handle_id].tolist() == [0.012, 0.04, 0.018]
    assert model.geom_size[blade_id].tolist() == [0.006, 0.08, 0.055]
    assert model.geom_pos[blade_id, 1] < model.geom_pos[handle_id, 1]


def test_right_tool_uses_visual_cleaver_mesh_with_simple_collision():
    model = mujoco.MjModel.from_xml_path(str(right_chopping_scene_path()))
    tool_body_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "right_tool_body")
    visual_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_cleaver_visual")
    handle_visual_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_cleaver_handle_visual")
    blade_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_knife_blade")

    assert _id(model, mujoco.mjtObj.mjOBJ_MESH, "right_cleaver_mesh") >= 0
    assert _id(model, mujoco.mjtObj.mjOBJ_MESH, "right_cleaver_handle_mesh") >= 0
    assert model.geom_bodyid[visual_id] == tool_body_id
    assert model.geom_bodyid[handle_visual_id] == tool_body_id
    assert model.geom_type[visual_id] == mujoco.mjtGeom.mjGEOM_MESH
    assert model.geom_type[handle_visual_id] == mujoco.mjtGeom.mjGEOM_MESH
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
    visual_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_cleaver_visual")
    handle_visual_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_cleaver_handle_visual")

    assert model.geom_rgba[blade_id, 3] == 0.0
    assert model.geom_rgba[handle_id, 3] == 0.0
    mesh_id = _id(model, mujoco.mjtObj.mjOBJ_MESH, "right_cleaver_mesh")
    vertex_start = model.mesh_vertadr[mesh_id]
    vertex_count = model.mesh_vertnum[mesh_id]
    vertices = model.mesh_vert[vertex_start : vertex_start + vertex_count]
    extents = vertices.max(axis=0) - vertices.min(axis=0)

    assert extents[2] > 0.15
    assert extents[1] < 0.03
    assert extents[0] < 0.005
    assert model.geom_type[handle_visual_id] == mujoco.mjtGeom.mjGEOM_MESH
    assert model.geom_type[visual_id] == mujoco.mjtGeom.mjGEOM_MESH


def test_scene_has_non_colliding_center_visual_between_arms():
    model = mujoco.MjModel.from_xml_path(str(right_chopping_scene_path()))
    geom_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "center_visual_console")

    assert model.geom_pos[geom_id].tolist() == [0.16, 0.0, 0.19]
    assert model.geom_size[geom_id].tolist() == [0.16, 0.24, 0.05]
    assert model.geom_contype[geom_id] == 0
    assert model.geom_conaffinity[geom_id] == 0
