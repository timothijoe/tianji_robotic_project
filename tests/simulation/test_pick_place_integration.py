import numpy as np

from twin_sim.robot import RightArmRobot
from twin_sim.tasks.pick_place import PickPlaceTask


def test_real_contact_pick_place_meets_acceptance_contract():
    robot = RightArmRobot(viewer=False)
    right_target_before = robot._right_target.copy()
    result = PickPlaceTask(robot).run()
    assert result.success, (
        f"{result.abort_phase}: {result.reason}; "
        f"last={result.samples[-1].phase if result.samples else 'none'}"
    )
    cube = np.asarray([sample.cube_position for sample in result.samples])
    assert cube[:, 2].max() - cube[0, 2] >= 0.08
    assert np.linalg.norm(cube[-1, :2] - cube[0, :2]) >= 0.15
    assert result.placed_in_target
    assert not result.used_hidden_attachment
    board = robot.sim.require_geom("chopping_board")
    board_top = (
        robot.sim.model.geom_pos[board, 2]
        + robot.sim.model.geom_size[board, 2]
    )
    assert cube[:, 2].min() - 0.025 >= board_top - 0.002
    assert np.linalg.norm(result.samples[-1].cube_linear_velocity) < 0.02
    assert (
        result.samples[-1].time_s - result.samples[-26].time_s
    ) >= 0.25 - 1e-9
    np.testing.assert_array_equal(robot._right_target, right_target_before)
    robot.close()
