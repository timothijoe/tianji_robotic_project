import math

import pytest

from cook_mujoco.control import MujocoPositionController, MujocoRuntime
from cook_mujoco.control.runtime import MujocoJointError, MujocoModelError
from cook_description.models import load_robot_definition
from cook_description.paths import DEFAULT_MJCF_PATH, DEFAULT_URDF_PATH


def _controller():
    definition = load_robot_definition(DEFAULT_URDF_PATH)
    runtime = MujocoRuntime.load(DEFAULT_MJCF_PATH, definition.movable_joint_names)
    return MujocoPositionController(runtime, joint_limits=definition.joint_limits)


def test_runtime_rejects_urdf():
    with pytest.raises(MujocoModelError, match="pre-generated"):
        MujocoRuntime.load(DEFAULT_URDF_PATH, ("Joint1_L",))


def test_controller_sets_targets_and_clamps_limits():
    controller = _controller()
    state = controller.set_joint_targets({"Joint4_L": 99.0})

    assert state.as_mapping()["Joint4_L"] == pytest.approx(1.0472)


def test_controller_rejects_non_finite_target():
    controller = _controller()

    with pytest.raises(MujocoJointError, match="finite"):
        controller.set_joint_targets({"Joint1_L": math.nan})
