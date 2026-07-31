import subprocess
from pathlib import Path


SCRIPT = Path(__file__).parents[2] / "scripts" / "run_guarded_chop.sh"


def test_launcher_resolves_repo_and_forwards_plane_command(tmp_path):
    fake_repo = tmp_path / "repo"
    script = fake_repo / "scripts" / SCRIPT.name
    executable = fake_repo / ".venv" / "bin" / "twin-sim"
    script.parent.mkdir(parents=True)
    executable.parent.mkdir(parents=True)
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
