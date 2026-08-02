import ast
import tomllib
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def imported_roots(directory: Path) -> set[str]:
    roots: set[str] = set()
    for path in directory.rglob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        roots.update(
            alias.name.split(".")[0]
            for node in ast.walk(tree)
            if isinstance(node, ast.Import)
            for alias in node.names
        )
        roots.update(
            (node.module or "").split(".")[0]
            for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom)
        )
    return roots


def test_new_package_is_importable():
    import tianji_robotics

    assert tianji_robotics.__name__ == "tianji_robotics"


def test_simulation_never_imports_hardware_packages():
    forbidden = {"SDK_PYTHON", "fx_robot", "fx_kine", "wujihandpy", "rclpy"}

    assert imported_roots(ROOT / "src/tianji_robotics/simulation").isdisjoint(forbidden)


def test_packaging_discovers_both_source_packages_and_declares_cli():
    config = tomllib.loads((ROOT / "pyproject.toml").read_text(encoding="utf-8"))

    assert config["project"]["scripts"]["tianji-robot"] == "tianji_robotics.cli:main"
    assert config["tool"]["setuptools"]["packages"]["find"] == {
        "where": ["src"],
        "include": ["twin_sim*", "tianji_robotics*"],
    }


def test_runtime_directories_are_ignored():
    text = (ROOT / ".gitignore").read_text(encoding="utf-8")

    assert "recordings/" in text
    assert ".venv-wuji-teleop/" in text
