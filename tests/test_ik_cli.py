import re

import numpy as np

from twin_control.kinematics import matrix_to_xyzabc
from twin_control.ik_cli import main
from twin_description import right_chopping_scene_path
from twin_mujoco import TwinMujocoRuntime
from twin_mujoco.chopping import LEFT_HOME_Q, RIGHT_CHOPPING_HOME_Q


def test_ik_cli_solves_current_force_sensor_pose(capsys):
    runtime = TwinMujocoRuntime.load(right_chopping_scene_path())
    runtime.reset()
    runtime.set_arm_positions("left", LEFT_HOME_Q)
    runtime.set_arm_positions("right", RIGHT_CHOPPING_HOME_Q)
    position, rotation = runtime.site_pose("right_force_sensor_site")
    target = np.eye(4)
    target[:3, :3] = rotation
    target[:3, 3] = position
    xyzabc = matrix_to_xyzabc(target)

    code = main([f"{value:.9f}" for value in xyzabc])

    output = capsys.readouterr().out
    assert code == 0
    assert "success: True" in output
    match = re.search(r"np\.array\(\((.*?)\), dtype=float\)", output)
    assert match is not None
    joints = np.fromstring(match.group(1), sep=",")
    np.testing.assert_allclose(joints, RIGHT_CHOPPING_HOME_Q, atol=1e-6)
