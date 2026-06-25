from cook_description.assets import RobotAssetContext
from cook_description.paths import DEFAULT_MJCF_PATH, DEFAULT_URDF_PATH, ROBOT_ASSET_ROOT


def test_asset_context_resolves_local_defaults_and_ros_description():
    context = RobotAssetContext.local_default()

    assert context.asset_root == ROBOT_ASSET_ROOT
    assert context.urdf_path == DEFAULT_URDF_PATH
    assert context.mjcf_path == DEFAULT_MJCF_PATH
    assert context.urdf_path.is_file()
    assert context.mjcf_path.is_file()
    assert "package://cook_description/assets/robot/meshes/" in (
        context.robot_description_for_ros()
    )


def test_description_declares_ament_index_dependency():
    package_xml = ROBOT_ASSET_ROOT.parents[1] / "package.xml"

    assert "<depend>ament_index_python</depend>" in package_xml.read_text()
