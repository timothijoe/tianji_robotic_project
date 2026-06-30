import numpy as np

from twin_control.chopping import ChoppingConfig, ChoppingPhase, TwinRobotChopper
from twin_control.rotation import rotation_error


def test_sdk_chopper_descend_targets_board_below_safe_height():
    samples = TwinRobotChopper().run(
        ChoppingConfig(cycles=1, force_hold_s=0.01),
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


def test_sdk_chopper_defaults_to_200_hz_control():
    assert ChoppingConfig().control_hz == 200.0


def test_sdk_chopper_path_phases_finish_within_position_tolerance():
    cfg = ChoppingConfig(cycles=1, force_hold_s=0.01)

    samples = TwinRobotChopper().run(cfg, headless=True)

    for phase in (ChoppingPhase.APPROACH, ChoppingPhase.DESCEND, ChoppingPhase.RETRACT):
        phase_samples = [sample for sample in samples if sample.phase == phase]
        assert phase_samples
        final = phase_samples[-1]
        error_m = np.linalg.norm(np.asarray(final.target_position) - np.asarray(final.actual_position))
        assert error_m <= cfg.position_tolerance_m


def test_sdk_chopper_path_phases_finish_within_pose_tolerance():
    cfg = ChoppingConfig(cycles=1, force_hold_s=0.01)

    samples = TwinRobotChopper().run(cfg, headless=True)

    for phase in (ChoppingPhase.APPROACH, ChoppingPhase.DESCEND, ChoppingPhase.RETRACT):
        phase_samples = [sample for sample in samples if sample.phase == phase]
        assert phase_samples
        final = phase_samples[-1]
        position_error_m = np.linalg.norm(
            np.asarray(final.target_position) - np.asarray(final.actual_position)
        )
        orientation_error_rad = np.linalg.norm(
            rotation_error(np.asarray(final.actual_rotation), np.asarray(final.target_rotation))
        )
        assert position_error_m <= cfg.position_tolerance_m
        assert orientation_error_rad <= cfg.orientation_tolerance_rad
