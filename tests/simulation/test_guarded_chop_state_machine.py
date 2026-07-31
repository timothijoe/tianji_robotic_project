import numpy as np
import pytest

from twin_sim.tasks.guarded_chop import (
    GuardedChopConfig,
    GuardedChopPhase,
)


def test_guarded_chop_defaults_match_approved_motion():
    config = GuardedChopConfig()

    assert config.cuts == 5
    assert config.hand_shift_m == 0.02
    assert config.minimum_distance_m == 0.02
    assert config.final_hold_s == 10.0


@pytest.mark.parametrize(
    "changes",
    (
        {"cuts": 0},
        {"cuts": 4},
        {"hand_shift_m": 0.0},
        {"hand_shift_m": 0.03},
        {"minimum_distance_m": np.nan},
        {"maximum_guard_penetration_m": -0.001},
        {"maximum_guard_force_n": np.nan},
    ),
)
def test_invalid_config_is_rejected(changes):
    with pytest.raises(ValueError):
        GuardedChopConfig(**changes).validated()


def test_phase_order_contains_interlocked_actions():
    assert [phase.value for phase in GuardedChopPhase] == [
        "initialize",
        "guard_ready",
        "cut_down",
        "knife_up",
        "hand_shift",
        "complete",
        "aborted",
    ]
