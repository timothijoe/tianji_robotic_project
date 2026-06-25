from __future__ import annotations

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ASSET_ROOT = PACKAGE_ROOT / "assets"
ROBOT_ASSET_ROOT = ASSET_ROOT / "robot"
DEFAULT_URDF_PATH = ROBOT_ASSET_ROOT / "urdf" / "test1.urdf"
DEFAULT_MJCF_PATH = ROBOT_ASSET_ROOT / "mujoco" / "robot.xml"
CHOPPING_MJCF_PATH = ROBOT_ASSET_ROOT / "mujoco" / "chopping_scene.xml"
DEFAULT_PACKAGE_NAME = "Marvin M6-S-L-CCS-696-V4.0_URDF"
