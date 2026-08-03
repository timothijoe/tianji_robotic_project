from pathlib import Path

import pytest

from tianji_robotics import cli


def test_headless_replay_routes_without_viewer(monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr(cli, "_run_wuji_replay", lambda args: calls.append(args) or 0)
    assert cli.main(["sim", "wuji-replay", str(tmp_path / "in.mcap"), "--headless"]) == 0
    assert calls[0].headless is True


def test_hardware_preflight_only_loads_trajectory(monkeypatch, tmp_path):
    path = tmp_path / "trajectory.npz"
    loaded = []
    monkeypatch.setattr(cli, "_preflight_trajectory", lambda candidate: loaded.append(candidate) or 0)
    assert cli.main(["hardware", "wuji-sdk", "preflight", str(path)]) == 0
    assert loaded == [path]


def test_hardware_cli_rejects_arm_option(tmp_path):
    with pytest.raises(SystemExit):
        cli.main(["hardware", "wuji-sdk", "preflight", str(tmp_path / "x.npz"), "--arm"])


def test_simulation_parser_rejects_hardware_options(tmp_path):
    with pytest.raises(SystemExit):
        cli.main(["sim", "wuji-replay", str(tmp_path / "x.mcap"), "--arm"])
