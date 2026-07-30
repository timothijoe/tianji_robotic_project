import pytest

from twin_sim.cli import main


def test_headless_chop_cli(tmp_path):
    path = tmp_path / "cli.csv"

    with pytest.warns(RuntimeWarning):
        assert main(["chop", "--headless", "--log", str(path)]) == 0
    assert path.is_file()


def test_headless_joint_and_cartesian_cli():
    assert main(["joint", "--joint", "1", "--delta-rad", "0.01", "--headless"]) == 0
    assert main(["cartesian", "--dz-m", "0.005", "--headless"]) == 0
