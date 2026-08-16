from __future__ import annotations

import numpy as np
import pytest

from real_robot_debug.keyboard_cartesian_jog import (
    JogConfig,
    candidate_pose,
    inside_workspace,
    key_to_delta,
    validate_config,
)


def test_w_requests_one_positive_x_step():
    assert key_to_delta("w", 2.0) == (2.0, 0.0, 0.0)


def test_f_requests_one_negative_z_step():
    assert key_to_delta("F", 2.0) == (0.0, 0.0, -2.0)


def test_candidate_pose_translates_xyz_and_preserves_orientation():
    current = np.array([100.0, 200.0, 300.0, 10.0, 20.0, 30.0])

    target = candidate_pose(current, (2.0, -2.0, 0.0))

    assert np.array_equal(target, np.array([102.0, 198.0, 300.0, 10.0, 20.0, 30.0]))


def test_execute_requires_both_workspace_bounds():
    with pytest.raises(ValueError, match="workspace-min.*workspace-max"):
        validate_config(JogConfig(execute=True))


def test_step_above_five_mm_is_rejected():
    with pytest.raises(ValueError, match="step-mm must be in .*5"):
        validate_config(JogConfig(step_mm=5.1))


def test_workspace_includes_edges_but_rejects_outside_point():
    lower, upper = (0.0, 0.0, 0.0), (10.0, 10.0, 10.0)

    assert inside_workspace(np.array([0.0, 10.0, 5.0]), lower, upper)
    assert not inside_workspace(np.array([10.1, 10.0, 5.0]), lower, upper)
