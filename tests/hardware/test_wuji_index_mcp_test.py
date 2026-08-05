import numpy as np
import pytest

from tianji_robotics.hardware.wuji_hand.index_mcp_test import (
    IndexMcpTestPlan,
    build_index_mcp_test_plan,
)


class FakeHand:
    def __init__(
        self,
        *,
        side=0,
        errors=None,
        temperatures=None,
        position=0.4,
        lower=None,
        upper=None,
    ):
        self.side = side
        self.errors = np.zeros((5, 4), dtype=np.uint32) if errors is None else errors
        self.temperatures = np.full((5, 4), 25.0) if temperatures is None else temperatures
        self.positions = np.zeros((5, 4))
        self.positions[1, 0] = position
        self.lower = np.full((5, 4), -1.0) if lower is None else lower
        self.upper = np.full((5, 4), 1.0) if upper is None else upper

    def read_handedness(self):
        return self.side

    def read_joint_error_code(self):
        return self.errors

    def read_joint_temperature(self):
        return self.temperatures

    def read_joint_actual_position(self):
        return self.positions

    def read_joint_lower_limit(self):
        return self.lower

    def read_joint_upper_limit(self):
        return self.upper


def test_builds_one_small_index_mcp_return_motion():
    current = 0.4
    plan = build_index_mcp_test_plan(FakeHand(position=current))

    assert plan == IndexMcpTestPlan(
        current_rad=current,
        targets_rad=(current - 0.05, current + 0.05, current),
        max_temperature_c=25.0,
    )


def test_preserves_exact_non_round_number_targets():
    current = 0.123456789012345

    plan = build_index_mcp_test_plan(FakeHand(position=current))

    assert plan.targets_rad == (current - 0.05, current + 0.05, current)


@pytest.mark.parametrize(
    ("hand", "message"),
    [
        (FakeHand(side=1), "right hand"),
        (FakeHand(errors=np.ones((5, 4), dtype=np.uint32)), "error codes"),
        (FakeHand(temperatures=np.full((5, 4), 40.1)), "temperature"),
    ],
)
def test_rejects_unsafe_preflight(hand, message):
    with pytest.raises(ValueError, match=message):
        build_index_mcp_test_plan(hand)


def test_rejects_targets_without_hardware_limit_margin():
    lower = np.full((5, 4), -1.0)
    upper = np.full((5, 4), 1.0)
    lower[1, 0] = 0.35

    with pytest.raises(ValueError, match="hardware-limit margin"):
        build_index_mcp_test_plan(FakeHand(lower=lower))
