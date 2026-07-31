import pytest

import twin_sim.cli as cli
from twin_sim.cli import main


def test_headless_chop_cli(tmp_path):
    path = tmp_path / "cli.csv"

    assert main(["chop", "--headless", "--log", str(path)]) == 0
    assert path.is_file()


def test_headless_joint_and_cartesian_cli():
    assert main(["joint", "--joint", "1", "--delta-rad", "0.01", "--headless"]) == 0
    assert main(["cartesian", "--dz-m", "0.005", "--headless"]) == 0


def test_slow_chop_cli_uses_observable_durations(monkeypatch, tmp_path):
    captured = {}

    def fake_run(config, *, log_path, viewer):
        captured.update(config=config, log_path=log_path, viewer=viewer)

    monkeypatch.setattr(cli, "run_chop", fake_run)
    path = tmp_path / "slow.csv"

    assert main(["chop", "--slow", "--log", str(path)]) == 0
    config = captured["config"]
    assert (
        config.approach_duration_s,
        config.descent_duration_s,
        config.hold_duration_s,
        config.retract_duration_s,
    ) == (3.0, 3.0, 1.0, 3.0)
    assert (config.viewer_start_hold_s, config.viewer_end_hold_s) == (5.0, 8.0)
    assert config.full_motion is False
    assert captured["viewer"] is True


def test_full_motion_cli_preserves_complete_sequence(monkeypatch, tmp_path):
    captured = {}

    def fake_run(config, *, log_path, viewer):
        captured["config"] = config

    monkeypatch.setattr(cli, "run_chop", fake_run)

    assert main(
        [
            "chop",
            "--full-motion",
            "--headless",
            "--log",
            str(tmp_path / "full.csv"),
        ]
    ) == 0
    assert captured["config"].full_motion is True


def test_line_chop_cli_maps_cuts_spacing_and_outputs(monkeypatch, tmp_path):
    captured = {}

    def fake_run(config, *, log_path, plot_path, viewer):
        captured.update(
            config=config,
            log_path=log_path,
            plot_path=plot_path,
            viewer=viewer,
        )

    monkeypatch.setattr(cli, "run_line_chop", fake_run)
    log_path = tmp_path / "line.csv"
    plot_path = tmp_path / "line.svg"

    assert main(
        [
            "line-chop",
            "--cuts",
            "5",
            "--spacing-m",
            "0.03",
            "--headless",
            "--log",
            str(log_path),
            "--plot",
            str(plot_path),
        ]
    ) == 0
    assert captured["config"].cuts == 5
    assert captured["config"].spacing_m == pytest.approx(0.03)
    assert captured["log_path"] == log_path
    assert captured["plot_path"] == plot_path
    assert captured["viewer"] is False
