from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from cook_description.paths import (
    DEFAULT_MJCF_PATH,
    DEFAULT_PACKAGE_NAME,
    DEFAULT_URDF_PATH,
    ROBOT_ASSET_ROOT,
)


@dataclass(frozen=True)
class RobotAssetContext:
    asset_root: Path
    urdf_path: Path
    mjcf_path: Path
    package_name: str = "cook_description"

    @classmethod
    def local_default(cls) -> "RobotAssetContext":
        if DEFAULT_URDF_PATH.is_file() and DEFAULT_MJCF_PATH.is_file():
            return cls(
                asset_root=ROBOT_ASSET_ROOT,
                urdf_path=DEFAULT_URDF_PATH,
                mjcf_path=DEFAULT_MJCF_PATH,
            )
        share_path = _package_share_path("cook_description")
        if share_path is not None:
            return cls.from_share_path(share_path, package_name="cook_description")
        return cls(
            asset_root=ROBOT_ASSET_ROOT,
            urdf_path=DEFAULT_URDF_PATH,
            mjcf_path=DEFAULT_MJCF_PATH,
        )

    @classmethod
    def from_share_path(
        cls,
        share_path: str | Path,
        *,
        package_name: str = "cook_description",
    ) -> "RobotAssetContext":
        share = Path(share_path)
        asset_root = share / "assets" / "robot"
        return cls(
            asset_root=asset_root,
            urdf_path=asset_root / "urdf" / "test1.urdf",
            mjcf_path=asset_root / "mujoco" / "robot.xml",
            package_name=package_name,
        )

    def robot_description_for_ros(self) -> str:
        text = self.urdf_path.expanduser().read_text()
        return text.replace(
            f"package://{DEFAULT_PACKAGE_NAME}/meshes/",
            f"package://{self.package_name}/assets/robot/meshes/",
        )


def _package_share_path(package_name: str) -> Path | None:
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return None
    try:
        return Path(get_package_share_directory(package_name))
    except Exception:
        return None
