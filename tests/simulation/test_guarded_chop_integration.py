import numpy as np

from twin_sim.guarded_chop_safety import SafetyDecision
from twin_sim.tasks.guarded_chop import (
    GuardedChopConfig,
    GuardedChopPhase,
    run_guarded_chop,
)


def test_guarded_chop_executes_five_cuts_and_four_safe_shifts():
    result = run_guarded_chop(
        GuardedChopConfig(final_hold_s=0.0), viewer=False
    )

    assert result.success, result.reason
    assert result.final_phase is GuardedChopPhase.COMPLETE
    assert result.completed_cuts == 5
    assert result.completed_shifts == 4
    assert np.isclose(result.total_shift_m, 0.08, atol=0.002)
    assert result.minimum_distance_m >= 0.02

    down = [
        sample
        for sample in result.samples
        if sample.phase is GuardedChopPhase.CUT_DOWN
    ]
    assert min(sample.knife_height_m for sample in down) >= 0.328
    for index in range(1, 6):
        cut = [sample for sample in down if sample.cut_index == index]
        targets = np.asarray([sample.left_target_rad for sample in cut])
        assert np.max(np.ptp(targets, axis=0)) <= 1e-12

    shifts = [
        sample
        for sample in result.samples
        if sample.phase is GuardedChopPhase.HAND_SHIFT
    ]
    for index in range(1, 5):
        shift = [sample for sample in shifts if sample.cut_index == index]
        targets = np.asarray([sample.right_target_rad for sample in shift])
        assert np.max(np.ptp(targets, axis=0)) <= 1e-12


def test_rejected_safety_decision_aborts_both_arm_sequence():
    class RejectCoordinator:
        minimum_distance_m = 0.02

        def evaluate(self, observation):
            return SafetyDecision(False, "injected safety rejection")

    result = run_guarded_chop(
        GuardedChopConfig(final_hold_s=0.0),
        viewer=False,
        coordinator=RejectCoordinator(),
    )

    assert not result.success
    assert result.final_phase is GuardedChopPhase.ABORTED
    assert result.completed_cuts == 0
    assert result.completed_shifts == 0
    assert result.reason == "injected safety rejection"
