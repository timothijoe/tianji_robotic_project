import numpy as np

from twin_control.chopping import ChoppingConfig, ChoppingPhase, TwinRobotChopper


def test_sdk_chopper_descend_targets_board_below_safe_height():
    samples = TwinRobotChopper().run(
        ChoppingConfig(cycles=1, control_hz=20.0, force_hold_s=0.01),
        headless=True,
    )

    approach_z = [sample.target_position[2] for sample in samples if sample.phase == ChoppingPhase.APPROACH]
    descend_z = [sample.target_position[2] for sample in samples if sample.phase == ChoppingPhase.DESCEND]

    assert approach_z
    assert descend_z
    assert min(descend_z) < min(approach_z) - 0.02
    assert descend_z[0] > descend_z[-1]


def test_sdk_chopper_writes_nonzero_joint_torques_in_samples():
    samples = TwinRobotChopper().run(
        ChoppingConfig(cycles=1, control_hz=20.0, force_hold_s=0.01),
        headless=True,
    )

    active = [sample for sample in samples if sample.phase != ChoppingPhase.COMPLETE]
    assert active
    assert any(np.linalg.norm(sample.joint_torques) > 0.0 for sample in active)
