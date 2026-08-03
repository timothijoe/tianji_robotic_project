from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_setup_script_is_project_relative_and_reproducible():
    script = (ROOT / "scripts/setup_wuji_teleop_env.sh").read_text()
    assert 'PROJECT_ROOT=' in script
    assert '.venv-wuji-teleop' in script
    assert 'python3.12 -m venv' in script
    assert '[wuji-offline,test]' in script
    assert '/home/zhoutong' not in script


def test_offline_docs_publish_the_supported_cli():
    documentation = (ROOT / "docs/wuji/offline_replay.md").read_text()
    assert "tianji-robot sim wuji-replay" in documentation
    assert "recordings/wuji" in documentation
    assert "hand-only" in documentation
    assert "不会加载机械臂" in documentation
    assert "wuji_hand_standalone" in documentation


def test_hardware_docs_make_preflight_non_motion_guarantee_explicit():
    documentation = (ROOT / "docs/wuji/hardware_interfaces.md").read_text()
    assert "tianji-robot hardware wuji-sdk preflight" in documentation
    assert "不会连接" in documentation
    assert "ROS 2" in documentation


def test_table_retreat_docs_publish_command_thresholds_and_safety_boundary():
    documentation=(ROOT/"docs/wuji/table_retreat.md").read_text()
    assert "wuji-table-retreat" in documentation
    assert "session_20260802_174440_936_right_to_left_wuji_hand.mcap" in documentation
    assert "joint_states" in documentation and "right_glove_skeleton" in documentation
    assert "499" in documentation and "311:464" in documentation
    assert "30 mm" in documentation and "10 mm" in documentation and "0.5 mm" in documentation
    assert "掌心向下" in documentation and "关节修正为 0" in documentation
    assert "不会连接" in documentation
