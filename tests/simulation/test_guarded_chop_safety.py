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


def test_shift_requires_raised_stationary_knife():
    coordinator = SafetyCoordinator(minimum_distance_m=0.02)
    allowed = observation(
        phase="HAND_SHIFT",
        knife_height_m=0.36,
        right_target_stationary=True,
        left_target_stationary=False,
    )

    assert coordinator.evaluate(allowed).allowed
    assert not coordinator.evaluate(
        observation(
            phase="HAND_SHIFT",
            knife_height_m=0.33,
            right_target_stationary=True,
            left_target_stationary=False,
        )
    ).allowed


def test_nonfinite_state_is_rejected():
    decision = SafetyCoordinator().evaluate(
        observation(finite_state=False)
    )

    assert not decision.allowed
    assert "non-finite" in decision.reason


def test_actual_arm_motion_blocks_interlocked_phase():
    coordinator = SafetyCoordinator()

    assert not coordinator.evaluate(
        observation(left_speed_rad_s=0.06)
    ).allowed
    assert not coordinator.evaluate(
        observation(
            phase="HAND_SHIFT",
            right_target_stationary=True,
            left_target_stationary=False,
            right_speed_rad_s=0.06,
        )
    ).allowed
