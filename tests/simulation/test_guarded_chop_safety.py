import pytest

from twin_sim.guarded_chop_safety import (
    SafetyCoordinator,
    SafetyObservation,
)


def observation(**overrides):
    values = dict(
        phase="CUT_DOWN",
        knife_height_m=0.36,
        safe_knife_height_m=0.34,
        knife_guard_distance_m=0.03,
        left_target_stationary=True,
        right_target_stationary=False,
        left_speed_rad_s=0.0,
        right_speed_rad_s=0.0,
        hand_speed_rad_s=0.0,
        guard_cube_penetration_m=0.0,
        guard_cube_normal_force_n=0.0,
        finite_state=True,
    )
    values.update(overrides)
    return SafetyObservation(**values)


def test_cut_requires_stationary_guard_and_clearance():
    coordinator = SafetyCoordinator(minimum_distance_m=0.02)

    assert coordinator.evaluate(observation()).allowed
    assert not coordinator.evaluate(
        observation(left_target_stationary=False)
    ).allowed
    decision = coordinator.evaluate(
        observation(knife_guard_distance_m=0.019)
    )
    assert not decision.allowed
    assert "distance" in decision.reason


@pytest.mark.parametrize(
    "phase", ("HAND_OPEN", "HAND_SHIFT", "HAND_CLOSE")
)
def test_left_hand_phase_requires_raised_stationary_knife(phase):
    coordinator = SafetyCoordinator(minimum_distance_m=0.02)
    allowed = observation(
        phase=phase,
        knife_height_m=0.36,
        right_target_stationary=True,
        left_target_stationary=False,
    )

    assert coordinator.evaluate(allowed).allowed
    assert not coordinator.evaluate(
        observation(
            phase=phase,
            knife_height_m=0.33,
            right_target_stationary=True,
            left_target_stationary=False,
        )
    ).allowed
    assert not coordinator.evaluate(
        observation(
            phase=phase,
            knife_height_m=0.36,
            right_target_stationary=False,
            left_target_stationary=False,
        )
    ).allowed


def test_nonfinite_state_is_rejected():
    decision = SafetyCoordinator().evaluate(
        observation(finite_state=False)
    )

    assert not decision.allowed
    assert "non-finite" in decision.reason


def test_excessive_guard_contact_is_rejected():
    coordinator = SafetyCoordinator()

    assert not coordinator.evaluate(
        observation(guard_cube_penetration_m=0.0031)
    ).allowed
    assert not coordinator.evaluate(
        observation(guard_cube_normal_force_n=35.1)
    ).allowed


@pytest.mark.parametrize(
    "phase", ("HAND_OPEN", "HAND_SHIFT", "HAND_CLOSE")
)
def test_actual_arm_motion_blocks_interlocked_phase(phase):
    coordinator = SafetyCoordinator()

    assert not coordinator.evaluate(
        observation(left_speed_rad_s=0.06)
    ).allowed
    assert not coordinator.evaluate(
        observation(hand_speed_rad_s=0.06)
    ).allowed
    assert not coordinator.evaluate(
        observation(
            phase=phase,
            right_target_stationary=True,
            left_target_stationary=False,
            right_speed_rad_s=0.06,
        )
    ).allowed
