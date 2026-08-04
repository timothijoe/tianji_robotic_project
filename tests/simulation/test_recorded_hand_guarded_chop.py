from pathlib import Path

import pytest

from twin_sim.tasks.recorded_hand_guarded_chop import (
    RecordedHandGuardedChopConfig,
    run_recorded_hand_guarded_chop,
)


def _real_mcap() -> Path:
    path = Path(
        "../../recordings/wuji/august_02/"
        "session_20260802_174440_936_right_to_left_wuji_hand.mcap"
    ).resolve()
    if not path.exists():
        pytest.skip("local ignored Wuji recording is unavailable")
    return path


def test_default_run_completes_five_cuts_after_five_safe_recorded_cycles():
    result = run_recorded_hand_guarded_chop(
        RecordedHandGuardedChopConfig(surface_offsets_m=(.04,)),
        hand_mcap=_real_mcap(),
        viewer=False,
    )
    assert result.success
    assert result.completed_cuts == 5
    assert result.completed_hand_cycles == 5
    for cycle in range(1, 6):
        assert result.events.index((cycle, "HAND_SAFE")) < result.events.index(
            (cycle, "CUT_DOWN")
        )
    assert result.minimum_distance_m >= .020
    assert result.maximum_hand_penetration_m <= .0005
    assert result.minimum_thumb_clearance_m >= .010
