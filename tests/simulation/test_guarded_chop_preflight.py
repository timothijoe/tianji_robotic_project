import numpy as np

from twin_sim.robot import RightArmRobot
from twin_sim.tasks.guarded_chop import (
    GuardedChopConfig,
    _default_view_screen_x,
    _point_to_box_distance,
    _preflight_guarded_chop,
)


def test_preflight_orders_knife_and_guard_motion_screen_right_to_left():
    robot = RightArmRobot(viewer=False)
    try:
        plan = _preflight_guarded_chop(robot, GuardedChopConfig())
        screen_x = _default_view_screen_x(plan.cut_points_xy)
        assert np.all(np.diff(screen_x) < 0.0)
        knife_delta = np.diff(plan.cut_points_xy, axis=0)
        guard_delta = np.diff(plan.guard_targets[:, :2, 3], axis=0)
        np.testing.assert_allclose(guard_delta, knife_delta, atol=1e-9)
    finally:
        robot.close()


def test_preflight_builds_five_cuts_and_four_guard_shifts():
    robot = RightArmRobot(viewer=False)
    state_before = (
        robot.sim.data.qpos.copy(),
        robot.sim.data.qvel.copy(),
        robot.sim.data.ctrl.copy(),
        float(robot.sim.data.time),
    )

    plan = _preflight_guarded_chop(robot, GuardedChopConfig())

    assert len(plan.cuts) == 5
    assert len(plan.guard_shifts) == 4
    deltas = np.diff(plan.guard_targets[:, :3, 3], axis=0)
    np.testing.assert_allclose(
        np.linalg.norm(deltas[:, :2], axis=1), 0.02, atol=1e-9
    )
    assert np.all(plan.minimum_planned_distances_m >= 0.02)
    for actual, expected in zip(
        (
            robot.sim.data.qpos,
            robot.sim.data.qvel,
            robot.sim.data.ctrl,
            float(robot.sim.data.time),
        ),
        state_before,
        strict=True,
    ):
        np.testing.assert_array_equal(actual, expected)
    robot.close()


def test_point_to_oriented_blade_box_distance_uses_surface_not_center():
    center = np.array((0.0, 0.0, 0.0))
    rotation = np.eye(3)
    half_size = np.array((0.10, 0.01, 0.05))

    assert np.isclose(
        _point_to_box_distance(
            np.array((0.0, 0.03, 0.0)),
            center,
            rotation,
            half_size,
        ),
        0.02,
    )
