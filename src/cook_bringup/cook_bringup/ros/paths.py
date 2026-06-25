from __future__ import annotations

from pathlib import Path

from cook_description.assets import RobotAssetContext


DESCRIPTION_PACKAGE_NAME = "cook_description"


def package_share_path(package_name: str) -> Path | None:
    try:
        from ament_index_python.packages import get_package_share_directory
    except Exception:
        return None
    try:
        return Path(get_package_share_directory(package_name))
    except Exception:
        return None


def default_model_path() -> Path:
    return default_asset_context().mjcf_path


def default_urdf_path() -> Path:
    return default_asset_context().urdf_path


def default_robot_asset_root() -> Path:
    return default_asset_context().asset_root


def default_asset_context() -> RobotAssetContext:
    share = package_share_path(DESCRIPTION_PACKAGE_NAME)
    if share is None:
        return RobotAssetContext.local_default()
    return RobotAssetContext.from_share_path(
        share,
        package_name=DESCRIPTION_PACKAGE_NAME,
    )
