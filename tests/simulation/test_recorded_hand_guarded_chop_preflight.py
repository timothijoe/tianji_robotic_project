from pathlib import Path

import numpy as np
import pytest

import twin_sim.tasks.recorded_hand_guarded_chop as module
from twin_sim.recorded_hand_guard import load_recorded_guard_cycle
from twin_sim.robot import RightArmRobot
from twin_sim.tasks.recorded_hand_guarded_chop import (
    RecordedHandGuardedChopConfig,
    _chopping_aligned_palm_rotation,
    _lateral_guard_anchor_xy,
    _preflight_recorded_hand_guarded_chop,
)


def _real_cycle():
    path = Path(__file__).resolve().parents[2] / Path(
        "recordings/wuji/august_02/"
        "session_20260802_174440_936_right_to_left_wuji_hand.mcap"
    )
    if not path.exists():
        pytest.skip("local ignored Wuji recording is unavailable")
    return load_recorded_guard_cycle(path)


def test_default_surface_search_actually_raises_the_board():
    config = RecordedHandGuardedChopConfig()
    assert config.surface_offsets_m[0] == pytest.approx(.04)
    assert config.surface_offsets_m[-1] == pytest.approx(.12)


def test_chopping_alignment_rotates_complete_palm_frame_minus_ninety_degrees():
    initial = _real_cycle().initial_palm_transform[:3, :3]
    quarter_turn = np.array(((0.0, 1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, 1.0)))
    expected = quarter_turn @ np.diag((-1.0, -1.0, 1.0)) @ initial

    np.testing.assert_allclose(
        _chopping_aligned_palm_rotation(initial), expected, atol=1e-12
    )


def test_lateral_guard_anchor_uses_left_arm_side_of_cut_line():
    np.testing.assert_allclose(
        _lateral_guard_anchor_xy(np.array((.62, -.08)), .12),
        (.50, -.08),
    )


def test_preflight_selects_lowest_feasible_shared_surface(monkeypatch):
    sentinel = object()

    def candidate(_robot, _cycle, _config, offset):
        if offset < .06:
            raise ValueError("left IK")
        return sentinel

    monkeypatch.setattr(module, "_build_candidate_plan", candidate)
    result = _preflight_recorded_hand_guarded_chop(
        object(), object(), RecordedHandGuardedChopConfig(surface_offsets_m=(.04, .05, .06))
    )
    assert result is sentinel


def test_real_plan_contains_five_safe_recorded_cycles_and_five_right_cuts():
    robot = RightArmRobot(viewer=False)
    try:
        plan = _preflight_recorded_hand_guarded_chop(
            robot,
            _real_cycle(),
            RecordedHandGuardedChopConfig(surface_offsets_m=(.04,)),
        )
        assert plan.surface_offset_m == pytest.approx(.04)
        assert len(plan.left_cycles) == 5
        assert len(plan.right_cuts) == 5
        assert len(plan.left_resets) == 4
        assert all(len(value.hand) >= 51 for value in plan.left_resets)
        assert all(len(value.hand) == len(value.left) for value in plan.left_cycles)
        assert plan.minimum_planned_distance_m >= .020
        assert plan.maximum_hand_penetration_m <= .0005
        assert plan.minimum_thumb_clearance_m >= .010
        cut_direction = (
            plan.right_cuts[-1].descent[-1].target_pose[:2, 3]
            - plan.right_cuts[0].descent[-1].target_pose[:2, 3]
        )
        cut_direction /= np.linalg.norm(cut_direction)
        for value in plan.left_cycles:
            retreat = value.palm_targets[-1, :2, 3] - value.palm_targets[0, :2, 3]
            retreat_direction = retreat / np.linalg.norm(retreat)
            angular_error = np.arccos(
                np.clip(np.dot(retreat_direction, cut_direction), -1.0, 1.0)
            )
            assert angular_error <= np.deg2rad(1.0)
            assert retreat[1] > 0.0
            assert abs(retreat[0]) <= .0005
        maximum_hand_step = max(
            np.max(np.abs(np.diff(value.hand, axis=0))) for value in plan.left_cycles
        )
        assert maximum_hand_step <= .12
    finally:
        robot.close()
