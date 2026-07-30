import numpy as np

from twin_control.trajectory import (
    cartesian_minimum_jerk_trajectory,
    minimum_jerk_scalar,
)


def test_minimum_jerk_scalar_has_smooth_endpoints_and_midpoint():
    assert minimum_jerk_scalar(0.0) == 0.0
    assert minimum_jerk_scalar(1.0) == 1.0
    assert minimum_jerk_scalar(0.5) == 0.5


def test_cartesian_minimum_jerk_trajectory_preserves_endpoints_and_rotation():
    rotation = np.array((
        (0.0, -1.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0),
    ))
    points = cartesian_minimum_jerk_trajectory(
        start_pos=(0.0, 0.0, 0.2),
        target_pos=(0.1, -0.05, 0.3),
        rotation=rotation,
        steps=5,
        phase="DESCEND",
        start_time_s=1.0,
        dt_s=0.01,
    )

    assert len(points) == 5
    assert np.allclose(points[0].pose_matrix[:3, 3], (0.0, 0.0, 0.2))
    assert np.allclose(points[-1].pose_matrix[:3, 3], (0.1, -0.05, 0.3))
    assert all(point.phase == "DESCEND" for point in points)
    assert all(np.allclose(point.pose_matrix[:3, :3], rotation) for point in points)
    assert [point.time_s for point in points] == [1.0, 1.01, 1.02, 1.03, 1.04]


def test_cartesian_minimum_jerk_trajectory_moves_slowly_near_endpoints():
    points = cartesian_minimum_jerk_trajectory(
        start_pos=(0.0, 0.0, 0.0),
        target_pos=(1.0, 0.0, 0.0),
        rotation=np.eye(3),
        steps=9,
    )

    x = np.array([point.pose_matrix[0, 3] for point in points])
    deltas = np.diff(x)

    assert np.all(deltas > 0.0)
    assert deltas[0] < deltas[3]
    assert deltas[-1] < deltas[3]
