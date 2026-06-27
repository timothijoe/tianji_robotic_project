from __future__ import annotations

from pathlib import Path


def workspace_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / "pyproject.toml").is_file() and (parent / "docs" / "superpowers").is_dir():
            return parent
    raise RuntimeError("workspace root markers not found")


def source_model_path() -> Path:
    return workspace_root() / "MarvinCCS" / "marvin_final_fixed.xml"


def right_chopping_scene_path() -> Path:
    return (
        workspace_root()
        / "src"
        / "twin_description"
        / "twin_description"
        / "assets"
        / "robot"
        / "mujoco"
        / "right_chopping_scene.xml"
    )
