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
