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
            module
            for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom)
            for module in imported_from_modules(path, node)
        )
    return modules


def imported_from_modules(path: Path, node: ast.ImportFrom) -> set[str]:
    if node.level:
        source_root = next(parent for parent in path.parents if parent.name == "src")
        package = list(path.relative_to(source_root).parent.parts)
        package = package[: len(package) - (node.level - 1)]
        if node.module:
            package.extend(node.module.split("."))
    else:
        package = (node.module or "").split(".") if node.module else []

    return {
        ".".join([*package, alias.name])
        for alias in node.names
        if alias.name != "*"
    }


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
        "tianji_robotics.wuji_sdk.transport",
    }
    assert forbidden_imports(
        imported_roots(tmp_path), {"wuji_sdk", "tianji_robotics.wuji_sdk"}
    ) == {"wuji_sdk.client", "tianji_robotics.wuji_sdk.transport"}


def test_imported_roots_detects_sdk_from_absolute_and_relative_imports(tmp_path: Path):
    (tmp_path / "absolute.py").write_text(
        "from tianji_robotics import wuji_sdk\n", encoding="utf-8"
    )
    relative_source = tmp_path / "src/tianji_robotics/simulation/example.py"
    relative_source.parent.mkdir(parents=True)
    relative_source.write_text("from .. import wuji_sdk\n", encoding="utf-8")

    assert forbidden_imports(
        imported_roots(tmp_path), {"tianji_robotics.wuji_sdk"}
    ) == {"tianji_robotics.wuji_sdk"}


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
    assert ".superpowers/sdd/" in text
