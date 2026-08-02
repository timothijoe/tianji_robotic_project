import ast
import tomllib
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def imported_roots(directory: Path) -> set[str]:
    modules: set[str] = set()
    for path in directory.rglob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        modules.update(
            alias.name
            for node in ast.walk(tree)
            if isinstance(node, ast.Import)
            for alias in node.names
        )
        modules.update(
            node.module or ""
            for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom)
        )
    return modules


def forbidden_imports(modules: set[str], forbidden: set[str]) -> set[str]:
    return {
        module
        for module in modules
        if any(module == package or module.startswith(f"{package}.") for package in forbidden)
    }


def test_imported_roots_preserves_full_sdk_module_paths(tmp_path: Path):
    source = tmp_path / "sdk_imports.py"
    source.write_text(
        "import wuji_sdk.client\n"
        "from tianji_robotics.wuji_sdk import transport\n",
        encoding="utf-8",
    )

    assert imported_roots(tmp_path) == {
        "wuji_sdk.client",
        "tianji_robotics.wuji_sdk",
    }
    assert forbidden_imports(
        imported_roots(tmp_path), {"wuji_sdk", "tianji_robotics.wuji_sdk"}
    ) == {"wuji_sdk.client", "tianji_robotics.wuji_sdk"}


def test_new_package_is_importable():
    import tianji_robotics

    assert tianji_robotics.__name__ == "tianji_robotics"


def test_simulation_never_imports_hardware_packages():
    forbidden = {
        "SDK_PYTHON",
        "fx_robot",
        "fx_kine",
        "wuji_sdk",
        "tianji_robotics.wuji_sdk",
        "wujihandpy",
        "rclpy",
    }

    assert not forbidden_imports(
        imported_roots(ROOT / "src/tianji_robotics/simulation"), forbidden
    )


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
