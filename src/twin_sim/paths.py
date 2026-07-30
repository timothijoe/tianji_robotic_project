from pathlib import Path


def project_root() -> Path:
    return Path(__file__).resolve().parents[2]


def scene_path() -> Path:
    return project_root() / "robot_assets" / "mujoco" / "right_chopping_scene.xml"
