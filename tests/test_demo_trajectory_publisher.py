import pytest

pytest.importorskip("rclpy")
pytest.importorskip("trajectory_msgs.msg")

from cook_bringup.ros.demo_trajectory_publisher import (
    _bool_parameter,
    _float_parameter,
    _int_parameter,
)


def test_demo_publisher_parameter_helpers_accept_launch_strings():
    node = _FakeNode(
        {
            "enabled": "false",
            "duration": "1.25",
            "count": "8",
        }
    )

    assert not _bool_parameter(node, "enabled")
    assert _float_parameter(node, "duration") == pytest.approx(1.25)
    assert _int_parameter(node, "count") == 8


class _FakeNode:
    def __init__(self, values):
        self.values = values

    def get_parameter(self, name):
        return _FakeParameter(self.values[name])


class _FakeParameter:
    def __init__(self, value):
        self.value = value
