import ast
import tomllib
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_legacy_simulation_is_archived():
    archive = ROOT / "archive" / "legacy_simulation"
    assert (archive / "src" / "twin_control").is_dir()
    assert not (ROOT / "src" / "twin_control").exists()


def test_pytest_does_not_collect_archive():
    assert "archive" not in {
        part for path in (ROOT / "tests").rglob("*.py") for part in path.parts
    }


def test_pytest_collects_simulation_and_architecture_tests_by_default():
    config = tomllib.loads((ROOT / "pyproject.toml").read_text(encoding="utf-8"))

    assert config["tool"]["pytest"]["ini_options"]["testpaths"] == [
        "tests/simulation",
        "tests/architecture",
    ]


def test_new_simulator_does_not_import_real_sdk():
    forbidden = {"SDK_PYTHON", "fx_robot", "fx_kine"}
    for path in (ROOT / "src" / "twin_sim").rglob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        imported = {
            node.names[0].name.split(".")[0]
            for node in ast.walk(tree)
            if isinstance(node, ast.Import)
        } | {
            (node.module or "").split(".")[0]
            for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom)
        }
        assert imported.isdisjoint(forbidden), path
