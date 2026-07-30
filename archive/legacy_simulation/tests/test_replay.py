import numpy as np

from twin_control.replay import JointReplayConfig, JointReplayExecutor


def test_ideal_replay_matches_target_and_estimates_joint_velocity():
    executor = JointReplayExecutor(
        initial_joints=np.zeros(2),
        config=JointReplayConfig(model="ideal", dt_s=0.1),
    )

    sample = executor.step(np.array((1.0, -2.0)))

    np.testing.assert_allclose(sample.commanded_joints, (1.0, -2.0))
    np.testing.assert_allclose(sample.actual_joints, (1.0, -2.0))
    np.testing.assert_allclose(sample.joint_velocities, (10.0, -20.0))
    assert sample.step_index == 0
    assert sample.time_s == 0.1


def test_lagged_replay_moves_by_tracking_alpha():
    executor = JointReplayExecutor(
        initial_joints=np.zeros(2),
        config=JointReplayConfig(model="lagged", dt_s=0.1, tracking_alpha=0.25),
    )

    first = executor.step(np.array((4.0, 0.0)))
    second = executor.step(np.array((4.0, 0.0)))

    np.testing.assert_allclose(first.actual_joints, (1.0, 0.0))
    np.testing.assert_allclose(second.actual_joints, (1.75, 0.0))


def test_replay_delay_uses_older_commands_before_latest_target():
    executor = JointReplayExecutor(
        initial_joints=np.array((0.0,)),
        config=JointReplayConfig(model="ideal", dt_s=0.1, command_delay_steps=2),
    )

    first = executor.step(np.array((1.0,)))
    second = executor.step(np.array((2.0,)))
    third = executor.step(np.array((3.0,)))

    np.testing.assert_allclose(first.delayed_target_joints, (0.0,))
    np.testing.assert_allclose(second.delayed_target_joints, (0.0,))
    np.testing.assert_allclose(third.delayed_target_joints, (1.0,))
    np.testing.assert_allclose(third.actual_joints, (1.0,))


def test_replay_velocity_limit_clamps_actual_joint_motion():
    executor = JointReplayExecutor(
        initial_joints=np.zeros(1),
        config=JointReplayConfig(model="ideal", dt_s=0.1, joint_velocity_limit=5.0),
    )

    sample = executor.step(np.array((10.0,)))

    np.testing.assert_allclose(sample.actual_joints, (0.5,))
    np.testing.assert_allclose(sample.joint_velocities, (5.0,))


def test_noisy_replay_is_reproducible_with_seed():
    config = JointReplayConfig(model="noisy", dt_s=0.1, joint_noise_std=0.01, random_seed=7)
    left = JointReplayExecutor(initial_joints=np.zeros(3), config=config)
    right = JointReplayExecutor(initial_joints=np.zeros(3), config=config)

    left_sample = left.step(np.ones(3))
    right_sample = right.step(np.ones(3))

    np.testing.assert_allclose(left_sample.actual_joints, right_sample.actual_joints)
    assert not np.allclose(left_sample.actual_joints, np.ones(3))


def test_replay_uses_fk_and_jacobian_to_report_tcp_state():
    def fk(joints):
        pose = np.eye(4)
        pose[:3, 3] = (joints[0], joints[1], joints[0] + joints[1])
        return pose

    def jacobian(joints):
        del joints
        return np.array((
            (1.0, 0.0),
            (0.0, 1.0),
            (1.0, 1.0),
            (0.0, 0.0),
            (0.0, 0.0),
            (0.0, 0.0),
        ))

    executor = JointReplayExecutor(
        initial_joints=np.zeros(2),
        config=JointReplayConfig(model="ideal", dt_s=0.5),
        fk=fk,
        jacobian=jacobian,
    )

    sample = executor.step(np.array((1.0, 2.0)))

    np.testing.assert_allclose(sample.tcp_pose[:3, 3], (1.0, 2.0, 3.0))
    np.testing.assert_allclose(sample.tcp_velocity[:3], (2.0, 4.0, 6.0))
