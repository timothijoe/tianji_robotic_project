from pathlib import Path

import pytest

from cook_description.models import load_robot_definition
from cook_description.models.converter import convert_urdf_to_mjcf
from cook_description.paths import DEFAULT_PACKAGE_NAME, DEFAULT_URDF_PATH, ROBOT_ASSET_ROOT


def test_sample_urdf_joint_metadata():
    definition = load_robot_definition(DEFAULT_URDF_PATH)

    assert len(definition.links) == 8
    assert definition.movable_joint_names == (
        "Joint1_L",
        "Joint2_L",
        "Joint3_L",
        "Joint4_L",
        "Joint5_L",
        "Joint6_L",
        "Joint7_L",
    )
    assert definition.joint_limits["Joint4_L"] == pytest.approx((-2.5307, 1.0472))


def test_convert_sample_urdf_to_mjcf(tmp_path: Path):
    output = tmp_path / "robot.xml"

    convert_urdf_to_mjcf(
        urdf_path=DEFAULT_URDF_PATH,
        output_path=output,
        package_roots={DEFAULT_PACKAGE_NAME: ROBOT_ASSET_ROOT},
        overwrite=True,
        validate=True,
    )

    assert output.is_file()
    assert "assets/robot/meshes" in output.read_text() or "../meshes" in output.read_text()
