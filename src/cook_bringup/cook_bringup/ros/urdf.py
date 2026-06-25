from __future__ import annotations

from pathlib import Path

from cook_description.assets import RobotAssetContext


def load_robot_description_for_ros(
    urdf_path: str | Path,
    *,
    package_name: str = "cook_description",
) -> str:
    path = Path(urdf_path).expanduser()
    asset_root = path.parent.parent
    return RobotAssetContext(
        asset_root=asset_root,
        urdf_path=path,
        mjcf_path=asset_root / "mujoco" / "robot.xml",
        package_name=package_name,
    ).robot_description_for_ros()
