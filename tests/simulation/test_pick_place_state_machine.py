import numpy as np

from twin_sim.tasks.pick_place import (
    PickPlacePhase,
    PickPlaceResult,
    PickPlaceSample,
)


def test_phase_order_is_explicit_and_complete():
    assert [phase.value for phase in PickPlacePhase] == [
        "initialize",
        "open_hand",
        "pregrasp",
        "approach",
        "close_hand",
        "stabilize",
        "lift",
        "transfer",
        "lower",
        "release",
        "retreat",
        "complete",
        "aborted",
    ]


def test_abort_result_preserves_phase_and_reason():
    result = PickPlaceResult(
        success=False,
        final_phase=PickPlacePhase.ABORTED,
        abort_phase=PickPlacePhase.STABILIZE,
        reason="grasp timeout",
        samples=(),
        placed_in_target=False,
        used_hidden_attachment=False,
    )
    assert not result.success
    assert result.abort_phase is PickPlacePhase.STABILIZE
    assert not result.used_hidden_attachment


def test_samples_copy_mutable_arrays():
    source = np.zeros(3)
    sample = PickPlaceSample.minimal(
        time_s=0.0,
        phase=PickPlacePhase.INITIALIZE,
        palm_position=source,
        cube_position=source,
    )
    source[:] = 1.0
    np.testing.assert_array_equal(sample.palm_actual_position, np.zeros(3))
    np.testing.assert_array_equal(sample.cube_position, np.zeros(3))
