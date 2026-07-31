import numpy as np

import twin_sim.tasks.guarded_chop as guarded_chop
from twin_sim.guarded_chop_safety import (
    CAT_PAW_OPEN_RAD,
    CAT_PAW_RAD,
    SafetyDecision,
)
from twin_sim.robot import RightArmRobot
from twin_sim.tasks.guarded_chop import (
    GuardedChopConfig,
    GuardedChopPhase,
    _preflight_guarded_chop,
    run_guarded_chop,
)


def test_plane_mode_couples_knife_recovery_and_guard_handover():
    robot = RightArmRobot(viewer=False)
    try:
        plan = _preflight_guarded_chop(robot, GuardedChopConfig())
    finally:
        robot.close()
    result = run_guarded_chop(
        GuardedChopConfig(final_hold_s=0.0), viewer=False
    )

    assert result.success, result.reason
    assert result.final_phase is GuardedChopPhase.COMPLETE
    assert result.completed_cuts == 5
    assert result.completed_shifts == 4
    assert np.isclose(result.total_shift_m, 0.08, atol=0.002)
    assert result.minimum_distance_m >= 0.02
    assert max(
        sample.guard_cube_contact_count for sample in result.samples
    ) == 0
    assert max(
        sample.guard_cube_penetration_m for sample in result.samples
    ) <= 0.003
    assert max(
        sample.guard_cube_normal_force_n for sample in result.samples
    ) <= 35.0
    assert all(
        not sample.cut_allowed
        for sample in result.samples
        if sample.phase is not GuardedChopPhase.CUT_DOWN
    )
    assert max(
        sample.blade_hand_contact_count for sample in result.samples
    ) == 0
    assert max(
        sample.left_speed_rad_s
        for sample in result.samples
        if sample.phase is GuardedChopPhase.CUT_DOWN
    ) <= 0.05
    assert max(
        sample.hand_speed_rad_s
        for sample in result.samples
        if sample.phase is GuardedChopPhase.CUT_DOWN
    ) <= 0.05
    for phase in (
        GuardedChopPhase.COUPLED_OPEN,
        GuardedChopPhase.COUPLED_SHIFT,
        GuardedChopPhase.COUPLED_CLOSE,
    ):
        heights = [
            sample.knife_height_m
            for sample in result.samples
            if sample.phase is phase
        ]
        assert heights
        assert min(heights) >= plan.safe_knife_height_m

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

    clear = [
        sample for sample in result.samples
        if sample.phase is GuardedChopPhase.KNIFE_CLEAR
    ]
    assert clear
    assert max(sample.left_speed_rad_s for sample in clear) <= 0.05
    assert max(sample.hand_speed_rad_s for sample in clear) <= 0.05

    for index in range(1, 5):
        opened = [
            sample
            for sample in result.samples
            if sample.cut_index == index
            and sample.phase is GuardedChopPhase.COUPLED_OPEN
        ]
        shifted = [
            sample
            for sample in result.samples
            if sample.cut_index == index
            and sample.phase is GuardedChopPhase.COUPLED_SHIFT
        ]
        closed = [
            sample
            for sample in result.samples
            if sample.cut_index == index
            and sample.phase is GuardedChopPhase.COUPLED_CLOSE
        ]
        assert opened and shifted and closed
        assert opened[-1].time_s < shifted[0].time_s
        assert shifted[-1].time_s < closed[0].time_s
        np.testing.assert_allclose(
            opened[-1].hand_target_rad,
            CAT_PAW_OPEN_RAD,
            rtol=0.0,
            atol=1e-9,
        )
        assert np.max(
            np.ptp(
                np.asarray(
                    [sample.hand_target_rad for sample in shifted]
                ),
                axis=0,
            )
        ) < 1e-9
        np.testing.assert_allclose(
            closed[-1].hand_target_rad,
            CAT_PAW_RAD,
            rtol=0.0,
            atol=1e-9,
        )
        assert any(
            sample.right_speed_rad_s > 0.005
            and sample.hand_speed_rad_s > 0.005
            for sample in opened
        )
        assert any(
            sample.right_speed_rad_s > 0.005
            and sample.left_speed_rad_s > 0.005
            for sample in shifted
        )


def test_plane_scene_keeps_guard_cube_hidden_and_collision_disabled():
    robot = RightArmRobot(viewer=False)
    try:
        configure = getattr(
            guarded_chop, "_configure_guarded_scene", None
        )
        assert callable(configure)
        configure(robot, "plane")
        cube = robot.sim.require_geom("guarded_chop_cube")
        cube_body = int(robot.sim.model.geom_bodyid[cube])

        assert robot.sim.model.geom_rgba[cube, 3] == 0.0
        assert robot.sim.model.geom_contype[cube] == 0
        assert robot.sim.model.geom_conaffinity[cube] == 0
        assert robot.sim.model.body_contype[cube_body] == 0
        assert robot.sim.model.body_conaffinity[cube_body] == 0
    finally:
        robot.close()


def test_object_preflight_keeps_conservative_serial_trajectory(monkeypatch):
    def reject_plane_coupling(*args, **kwargs):
        raise AssertionError("object preflight used plane coupling")

    monkeypatch.setattr(
        guarded_chop, "_resample_trajectory", reject_plane_coupling
    )
    robot = RightArmRobot(viewer=False)
    try:
        plan = _preflight_guarded_chop(
            robot, GuardedChopConfig(scene_mode="object")
        )
    finally:
        robot.close()

    assert len(plan.guard_shifts) == 4


def test_object_mode_retains_contact_latched_hand():
    result = run_guarded_chop(
        GuardedChopConfig(scene_mode="object", final_hold_s=0.0),
        viewer=False,
    )

    assert result.success, result.reason
    assert max(
        sample.guard_cube_contact_count for sample in result.samples
    ) >= 1
    shifts = [
        sample
        for sample in result.samples
        if sample.phase is GuardedChopPhase.COUPLED_SHIFT
    ]
    assert shifts
    assert max(
        np.linalg.norm(sample.hand_target_rad - CAT_PAW_RAD)
        for sample in shifts
    ) > 1e-3


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
