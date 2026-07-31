import numpy as np
import pytest

import twin_sim.tasks.guarded_chop as guarded_chop
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


def test_plane_mode_and_open_close_phases_are_defaults():
    config = GuardedChopConfig()

    assert config.scene_mode == "plane"
    assert GuardedChopPhase.HAND_OPEN.value == "hand_open"
    assert GuardedChopPhase.HAND_CLOSE.value == "hand_close"


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
        {"scene_mode": "unsupported"},
        {"hand_open_duration_s": 0.0},
        {"hand_close_duration_s": np.inf},
        {"hand_close_duration_s": 0.605},
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
        "hand_open",
        "hand_shift",
        "hand_close",
        "complete",
        "aborted",
    ]


def test_guarded_chop_viewer_sync_is_limited_to_display_rate():
    sync_stride = getattr(guarded_chop, "_viewer_sync_stride", None)

    assert callable(sync_stride)
    assert sync_stride(0.01) == 3
    assert sync_stride(0.02) == 2
    assert sync_stride(0.05) == 1
