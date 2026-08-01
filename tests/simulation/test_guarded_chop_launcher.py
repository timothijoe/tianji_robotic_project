import subprocess
from pathlib import Path


SCRIPT = Path(__file__).parents[2] / "scripts" / "run_guarded_chop.sh"
REPLAY_SCRIPT = (
    Path(__file__).parents[2]
    / "scripts"
    / "run_guarded_chop_record_replay.sh"
)
RECORD_SCRIPT = (
    Path(__file__).parents[2] / "scripts" / "run_guarded_chop_record.sh"
)
REPLAY_ONLY_SCRIPT = (
    Path(__file__).parents[2] / "scripts" / "replay_guarded_chop_2x.sh"
)


def test_launcher_resolves_repo_and_forwards_plane_command(tmp_path):
    fake_repo = tmp_path / "repo"
    script = fake_repo / "scripts" / SCRIPT.name
    executable = fake_repo / ".venv" / "bin" / "twin-sim"
    script.parent.mkdir(parents=True, exist_ok=True)
    executable.parent.mkdir(parents=True, exist_ok=True)
    script.write_bytes(SCRIPT.read_bytes())
    script.chmod(0o755)
    executable.write_text("#!/usr/bin/env bash\nprintf '%s\\n' \"$@\"\n")
    executable.chmod(0o755)

    result = subprocess.run(
        [str(script)], cwd=tmp_path, text=True, capture_output=True
    )

    assert result.returncode == 0
    assert result.stdout.splitlines() == ["guarded-chop", "--scene", "plane"]


def test_launcher_reports_missing_virtual_environment(tmp_path):
    fake_repo = tmp_path / "repo"
    script = fake_repo / "scripts" / SCRIPT.name
    script.parent.mkdir(parents=True)
    script.write_bytes(SCRIPT.read_bytes())
    script.chmod(0o755)

    result = subprocess.run(
        [str(script)], cwd=tmp_path, text=True, capture_output=True
    )

    assert result.returncode != 0
    assert ".venv/bin/twin-sim" in result.stderr
    assert "python3.12 -m venv .venv" in result.stderr


def test_record_replay_launcher_runs_live_then_replays_at_two_x(tmp_path):
    fake_repo = tmp_path / "repo"
    script = fake_repo / "scripts" / REPLAY_SCRIPT.name
    executable = fake_repo / ".venv" / "bin" / "twin-sim"
    script.parent.mkdir(parents=True)
    executable.parent.mkdir(parents=True)
    script.write_bytes(REPLAY_SCRIPT.read_bytes())
    script.chmod(0o755)
    executable.write_text("#!/usr/bin/env bash\nprintf '%s\\n' \"$@\"\n")
    executable.chmod(0o755)

    result = subprocess.run(
        [str(script), "--record", "recordings/demo.npz"],
        cwd=tmp_path,
        text=True,
        capture_output=True,
    )

    assert result.returncode == 0
    assert result.stdout.splitlines() == [
        "guarded-chop",
        "--scene",
        "plane",
        "--replay-rate",
        "2.0",
        "--record",
        "recordings/demo.npz",
    ]


def _run_launcher_with_fake_twin_sim(tmp_path, source, *arguments):
    fake_repo = tmp_path / source.stem
    script = fake_repo / "scripts" / source.name
    executable = fake_repo / ".venv" / "bin" / "twin-sim"
    script.parent.mkdir(parents=True, exist_ok=True)
    executable.parent.mkdir(parents=True, exist_ok=True)
    script.write_bytes(source.read_bytes())
    script.chmod(0o755)
    executable.write_text("#!/usr/bin/env bash\nprintf '%s\\n' \"$@\"\n")
    executable.chmod(0o755)
    return subprocess.run(
        [str(script), *arguments],
        cwd=tmp_path,
        text=True,
        capture_output=True,
    )


def test_record_launcher_records_at_normal_speed_to_default_or_named_path(
    tmp_path,
):
    default = _run_launcher_with_fake_twin_sim(tmp_path, RECORD_SCRIPT)
    named = _run_launcher_with_fake_twin_sim(
        tmp_path, RECORD_SCRIPT, "recordings/demo.npz"
    )

    assert default.returncode == 0
    assert default.stdout.splitlines() == [
        "guarded-chop",
        "--scene",
        "plane",
        "--record",
        "recordings/guarded_chop_latest.npz",
    ]
    assert named.stdout.splitlines()[-1] == "recordings/demo.npz"


def test_replay_launcher_only_replays_default_or_named_path_at_two_x(
    tmp_path,
):
    default = _run_launcher_with_fake_twin_sim(
        tmp_path, REPLAY_ONLY_SCRIPT
    )
    named = _run_launcher_with_fake_twin_sim(
        tmp_path, REPLAY_ONLY_SCRIPT, "recordings/demo.npz"
    )

    assert default.returncode == 0
    assert default.stdout.splitlines() == [
        "guarded-chop-replay",
        "--recording",
        "recordings/guarded_chop_latest.npz",
        "--rate",
        "2.0",
    ]
    assert named.stdout.splitlines() == [
        "guarded-chop-replay",
        "--recording",
        "recordings/demo.npz",
        "--rate",
        "2.0",
    ]


def test_new_launchers_report_missing_virtual_environment(tmp_path):
    for source in (RECORD_SCRIPT, REPLAY_ONLY_SCRIPT):
        fake_repo = tmp_path / source.stem
        script = fake_repo / "scripts" / source.name
        script.parent.mkdir(parents=True)
        script.write_bytes(source.read_bytes())
        script.chmod(0o755)

        result = subprocess.run(
            [str(script)], cwd=tmp_path, text=True, capture_output=True
        )

        assert result.returncode != 0
        assert ".venv/bin/twin-sim" in result.stderr
        assert "python3.12 -m venv .venv" in result.stderr


def test_new_launchers_reject_more_than_one_recording_path(tmp_path):
    for source in (RECORD_SCRIPT, REPLAY_ONLY_SCRIPT):
        result = _run_launcher_with_fake_twin_sim(
            tmp_path,
            source,
            "recordings/one.npz",
            "recordings/two.npz",
        )

        assert result.returncode != 0
        assert "at most one recording path" in result.stderr
