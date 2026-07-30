import numpy as np
import pytest

from twin_core import arm_spec, all_arm_specs, finite_vector, validate_waypoints


def test_arm_specs_select_exactly_seven_joints_and_actuators_per_arm():
    left = arm_spec("left")
    right = arm_spec("right")

    assert left.joint_names == tuple(f"left_joint{i}" for i in range(1, 8))
    assert right.joint_names == tuple(f"right_joint{i}" for i in range(1, 8))
    assert left.actuator_names == tuple(f"act_left_joint{i}" for i in range(1, 8))
    assert right.actuator_names == tuple(f"act_right_joint{i}" for i in range(1, 8))
    assert len(all_arm_specs()) == 2


def test_unknown_arm_name_is_rejected():
    with pytest.raises(ValueError, match="unknown arm"):
        arm_spec("center")


def test_finite_vector_rejects_wrong_size_and_nan():
    assert finite_vector([1, 2, 3], 3, "point").shape == (3,)
    with pytest.raises(ValueError, match="point must contain 3 finite values"):
        finite_vector([1, 2], 3, "point")
    with pytest.raises(ValueError, match="point must contain 3 finite values"):
        finite_vector([1, np.nan, 3], 3, "point")


def test_finite_vector_rejects_nested_input_with_matching_total_size():
    with pytest.raises(ValueError, match="point must contain 4 finite values"):
        finite_vector([[1, 2], [3, 4]], 4, "point")


def test_validate_waypoints_returns_2d_array():
    result = validate_waypoints([[0, 1], [2, 3]], dof=2)

    assert result.shape == (2, 2)
    np.testing.assert_allclose(result, [[0, 1], [2, 3]])
