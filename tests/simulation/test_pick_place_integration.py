import numpy as np
import mujoco

from twin_sim.robot import RightArmRobot
from twin_sim.tasks.pick_place import PickPlacePhase, PickPlaceTask


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
    target_support = robot.sim.require_geom("pick_target_pedestal")
    target_support_top = (
        robot.sim.model.geom_pos[target_support, 2]
        + robot.sim.model.geom_size[target_support, 2]
    )
    assert cube[:, 2].min() - 0.025 >= target_support_top - 0.002
    assert np.linalg.norm(result.samples[-1].cube_linear_velocity) < 0.02
    assert (
        result.samples[-1].time_s - result.samples[-26].time_s
    ) >= 0.25 - 1e-9
    np.testing.assert_array_equal(robot._right_target, right_target_before)
    np.testing.assert_allclose(
        robot.joint_positions,
        right_target_before,
        atol=0.005,
        rtol=0.0,
    )
    robot.close()


def test_cube_drop_after_lift_aborts_before_transfer():
    class DropAfterLiftTask(PickPlaceTask):
        def _execute_cartesian(self, phase, *args, **kwargs):
            super()._execute_cartesian(phase, *args, **kwargs)
            if phase is PickPlacePhase.LIFT:
                joint = mujoco.mj_name2id(
                    self.robot.sim.model,
                    mujoco.mjtObj.mjOBJ_JOINT,
                    "pick_cube_free",
                )
                qpos_adr = self.robot.sim.model.jnt_qposadr[joint]
                self.robot.sim.data.qpos[qpos_adr : qpos_adr + 3] = (
                    self._cube_initial
                )
                mujoco.mj_forward(
                    self.robot.sim.model,
                    self.robot.sim.data,
                )

    robot = RightArmRobot(viewer=False)
    result = DropAfterLiftTask(robot).run()

    assert not result.success
    assert result.abort_phase is PickPlacePhase.LIFT
    assert "dropped" in result.reason
    robot.close()
