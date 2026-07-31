import numpy as np
import pytest

import twin_sim.tasks.guarded_chop as guarded_chop
from twin_sim.tasks.guarded_chop import (
    GuardedChopConfig,
    GuardedChopPhase,
    _resample_trajectory,
)


def test_guarded_chop_defaults_match_approved_motion():
    config = GuardedChopConfig()

    assert config.cuts == 5
    assert config.hand_shift_m == 0.02
    assert config.minimum_distance_m == 0.02
    assert config.final_hold_s == 10.0


def test_plane_mode_and_coupled_phases_are_defaults():
    config = GuardedChopConfig()

    assert config.scene_mode == "plane"
    assert GuardedChopPhase.COUPLED_OPEN.value == "coupled_open"
    assert GuardedChopPhase.COUPLED_CLOSE.value == "coupled_close"


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
        "knife_clear",
        "coupled_open",
        "coupled_shift",
        "coupled_close",
        "guard_settle",
        "complete",
        "aborted",
    ]


def test_resample_trajectory_matches_endpoints_and_requested_count():
    points = (np.asarray([0.0, 2.0]), np.asarray([1.0, 4.0]))

    result = _resample_trajectory(points, 5)

    assert len(result) == 5
    np.testing.assert_allclose(result[0], points[0])
    np.testing.assert_allclose(result[-1], points[-1])
    np.testing.assert_allclose(result[2], [0.5, 3.0])


@pytest.mark.parametrize("count", (0, -1))
def test_resample_trajectory_rejects_non_positive_count(count):
    with pytest.raises(ValueError, match="count must be positive"):
        _resample_trajectory((np.zeros(2),), count)


def test_guarded_chop_viewer_sync_is_limited_to_display_rate():
    sync_stride = getattr(guarded_chop, "_viewer_sync_stride", None)

    assert callable(sync_stride)
    assert sync_stride(0.01) == 3
    assert sync_stride(0.02) == 2
    assert sync_stride(0.05) == 1
