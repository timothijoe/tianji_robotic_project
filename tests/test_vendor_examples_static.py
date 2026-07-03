from pathlib import Path


def test_showcase_pln_joint_uses_script_relative_config_path():
    source = Path("test/showcase_pln_joint_positionMode.py").read_text()

    assert "config_path=os.path.join(current_path, 'ccs_m6_40.MvKDCfg')" in source
    assert "config_path='ccs_m6_40.MvKDCfg'" not in source
