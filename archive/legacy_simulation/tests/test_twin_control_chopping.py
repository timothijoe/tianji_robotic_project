import numpy as np

from twin_control.chopping import ChoppingConfig, ChoppingPhase, TwinRobotChopper
from twin_mujoco.chopping import ChoppingConfig as MujocoChoppingConfig, RightArmChopper
from twin_control import chopping_cli


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


def test_sdk_chopper_defaults_to_250_hz_control():
    assert ChoppingConfig().control_hz == 250.0


def test_sdk_chopper_cli_accepts_control_hz(monkeypatch):
    captured = {}

    class FakeChopper:
        def run(self, config, log_path=None, headless=False):
            captured["config"] = config
            captured["log_path"] = log_path
            captured["headless"] = headless

    monkeypatch.setattr(chopping_cli, "TwinRobotChopper", FakeChopper)

    exit_code = chopping_cli.main([
        "--cycles", "1",
        "--control-hz", "250",
        "--headless",
    ])

    assert exit_code == 0
    assert captured["config"].control_hz == 250.0
    assert captured["headless"] is True


def test_sdk_chopper_descend_finishes_with_dynamic_blade_error_in_reference_range():
    cfg = ChoppingConfig(cycles=1, force_hold_s=0.01)

    samples = TwinRobotChopper().run(cfg, headless=True)

    descend_samples = [sample for sample in samples if sample.phase == ChoppingPhase.DESCEND]
    assert descend_samples
    final = descend_samples[-1]
    errors = [
        np.linalg.norm(np.asarray(actual) - np.asarray(target))
        for actual, target in zip(final.blade_reference_positions, final.target_blade_reference_positions)
    ]
    assert max(errors) <= 0.08


def test_sdk_chopper_position_phases_finish_with_dynamic_blade_error_in_reference_range():
    cfg = ChoppingConfig(cycles=1, force_hold_s=0.01)

    samples = TwinRobotChopper().run(cfg, headless=True)

    for phase in (ChoppingPhase.APPROACH, ChoppingPhase.DESCEND, ChoppingPhase.RETRACT):
        phase_samples = [sample for sample in samples if sample.phase == phase]
        assert phase_samples
        final = phase_samples[-1]
        errors = [
            np.linalg.norm(np.asarray(actual) - np.asarray(target))
            for actual, target in zip(final.blade_reference_positions, final.target_blade_reference_positions)
        ]
        assert max(errors) <= 0.08


def test_sdk_chopper_descend_targets_horizontal_blade_reference_line():
    cfg = ChoppingConfig(cycles=1, force_hold_s=0.01)

    samples = TwinRobotChopper().run(cfg, headless=True)

    descend_samples = [sample for sample in samples if sample.phase == ChoppingPhase.DESCEND]
    assert descend_samples
    final = descend_samples[-1]
    target_z = [position[2] for position in final.target_blade_reference_positions]
    assert max(target_z) - min(target_z) <= 1e-9


def test_sdk_chopper_samples_include_three_blade_reference_points():
    samples = TwinRobotChopper().run(
        ChoppingConfig(cycles=1, force_hold_s=0.01),
        headless=True,
    )

    active = [sample for sample in samples if sample.phase != ChoppingPhase.COMPLETE]
    assert active
    for sample in active:
        assert len(sample.blade_reference_positions) == 3
        assert len(sample.target_blade_reference_positions) == 3
        for position in sample.blade_reference_positions + sample.target_blade_reference_positions:
            assert len(position) == 3
            assert np.all(np.isfinite(position))


def test_sdk_chopper_force_hold_targets_blade_references_on_board():
    chopper = TwinRobotChopper()
    samples = chopper.run(
        ChoppingConfig(cycles=1, force_hold_s=0.01),
        headless=True,
    )

    force_samples = [sample for sample in samples if sample.phase == ChoppingPhase.FORCE_HOLD]
    assert force_samples
    board_top = chopper._board_top_from_model_path()
    for position in force_samples[-1].target_blade_reference_positions:
        assert abs(position[2] - board_top) <= 1e-9


def test_sdk_chopper_force_hold_ramps_target_force():
    cfg = ChoppingConfig(cycles=1, force_hold_s=0.05, target_force_n=10.0)

    samples = TwinRobotChopper().run(cfg, headless=True)

    targets = [
        sample.target_force_n
        for sample in samples
        if sample.phase == ChoppingPhase.FORCE_HOLD
    ]
    assert len(targets) >= 3
    assert 0.0 < targets[0] < cfg.target_force_n
    assert targets[-1] == cfg.target_force_n
    assert targets == sorted(targets)


def test_sdk_chopper_does_not_teleport_through_endpoint_settle():
    samples = TwinRobotChopper().run(
        ChoppingConfig(cycles=1, force_hold_s=0.01),
        headless=True,
    )

    active_modes = [sample.control_mode for sample in samples if sample.phase != ChoppingPhase.COMPLETE]
    assert "ENDPOINT_CORRECTION" not in active_modes
    assert "ENDPOINT_SETTLE" not in active_modes


def test_sdk_chopper_final_posture_stays_close_to_reference_chopper():
    cfg = ChoppingConfig(cycles=2, force_hold_s=0.01)
    reference_cfg = MujocoChoppingConfig(cycles=2, force_hold_s=0.01, control_hz=cfg.control_hz)

    samples = TwinRobotChopper().run(cfg, headless=True)
    reference_samples = RightArmChopper().run(reference_cfg)

    q = np.asarray(samples[-1].joint_positions)
    reference_q = np.asarray(reference_samples[-1].joint_positions)
    assert np.linalg.norm(q - reference_q) < 0.5
