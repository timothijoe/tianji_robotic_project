import subprocess
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "visualize_right_glove_to_left.sh"


def test_dry_run_accepts_an_mcap_path_and_prints_both_stages(tmp_path: Path):
    source = tmp_path / "recording.mcap"
    source.touch()

    result = subprocess.run(
        ["bash", str(SCRIPT), "--dry-run", str(source)],
        capture_output=True,
        text=True,
        check=True,
    )

    assert "mcap_to_left_qpos.py" in result.stdout
    assert "mujoco_left_replay.py" in result.stdout
    assert "recording_right_to_left_wuji_hand.npz" in result.stdout


def test_dry_run_works_when_invoked_with_sh(tmp_path: Path):
    source = tmp_path / "recording.mcap"
    source.touch()

    result = subprocess.run(
        ["sh", str(SCRIPT), "--dry-run", str(source)],
        capture_output=True,
        text=True,
        check=True,
    )

    assert "Dry run complete" in result.stdout
