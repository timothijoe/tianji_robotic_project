import numpy as np
import pytest

from replay_left_hand_mcap_sdk import (
    assert_sdk_control_available,
    exclude_current_process,
    is_left_handedness,
    make_sdk_symbols,
)


def test_control_check_rejects_ros_driver():
    with pytest.raises(RuntimeError, match="stop ROS driver"):
        assert_sdk_control_available(["/hand_0/wujihand_driver"], [])


def test_control_check_accepts_no_known_controller():
    assert_sdk_control_available([], [])


def test_exclude_current_process_keeps_other_controller_processes():
    lines = [
        "4120 /home/zhoutong/wuji-teleop-venv/bin/python replay_left_hand_mcap_sdk.py --arm demo.mcap",
        "5120 /opt/wujihand/bin/wujihand_driver_node",
    ]

    assert exclude_current_process(lines, 4120) == [lines[1]]


def test_make_sdk_symbols_preserves_sdk_types():
    device_type = object()
    joint_command = object()
    low_pass = object()

    sdk = make_sdk_symbols(device_type, joint_command, low_pass)

    assert sdk.DeviceType is device_type
    assert sdk.JointCommand is joint_command
    assert sdk.LowPass is low_pass


def test_left_handedness_accepts_documented_sdk_spelling_only():
    assert is_left_handedness("Left")
    assert not is_left_handedness("Right")
    assert not is_left_handedness("left")
