from pathlib import Path


ROOT = Path(__file__).parents[2]
HANDOFF = ROOT / "docs" / "simulation" / "current_version_handoff.md"


def test_current_version_handoff_is_cross_machine_and_complete():
    text = HANDOFF.read_text()
    required = (
        "python3.12 -m venv .venv",
        "mujoco.__version__",
        "sha256sum --check docs/simulation/protected-files.sha256",
        ".venv/bin/python -m pytest -q",
        "./scripts/run_guarded_chop.sh",
        "./scripts/run_guarded_chop_record.sh",
        "./scripts/replay_guarded_chop_2x.sh",
        "./scripts/run_guarded_chop_record_replay.sh",
        "recordings/guarded_chop_latest.npz",
        "0.02 m",
        "position",
        "ROS 2",
    )
    assert all(value in text for value in required)
    assert "/home/linux" not in text
    assert all(
        (ROOT / path).exists()
        for path in (
            "scripts/run_guarded_chop.sh",
            "scripts/run_guarded_chop_record.sh",
            "scripts/replay_guarded_chop_2x.sh",
            "scripts/run_guarded_chop_record_replay.sh",
        )
    )
