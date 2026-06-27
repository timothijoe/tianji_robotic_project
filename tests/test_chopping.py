import csv

import numpy as np

from twin_description import right_chopping_scene_path
from twin_mujoco import TwinMujocoRuntime
from twin_mujoco.chopping import ChoppingConfig, ChoppingPhase, LEFT_HOME_Q, RIGHT_CHOPPING_HOME_Q, RightArmChopper


def test_headless_chopping_completes_three_cycles_and_logs_csv(tmp_path):
    log_path = tmp_path / "right_chop.csv"
    chopper = RightArmChopper()

    samples = chopper.run(
        ChoppingConfig(cycles=3, target_force_n=6.0, force_hold_s=0.05),
        log_path=log_path,
    )

    assert samples
    assert samples[-1].phase == ChoppingPhase.COMPLETE
    phases = {sample.phase for sample in samples}
    assert ChoppingPhase.APPROACH in phases
    assert ChoppingPhase.DESCEND in phases
    assert ChoppingPhase.FORCE_HOLD in phases
    assert ChoppingPhase.RETRACT in phases
    assert ChoppingPhase.SHIFT in phases
    with log_path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert rows
    assert rows[0].keys() >= {
        "time_s",
        "phase",
        "control_mode",
        "target_x",
        "actual_x",
        "target_force_n",
        "measured_force_n",
        "raw_wrench_0",
        "wrench_0",
        "q_0",
        "qd_0",
        "tau_0",
        "fault",
    }


def test_chopping_logs_measured_wrench_instead_of_commanded_force():
    chopper = RightArmChopper()

    samples = chopper.run(ChoppingConfig(cycles=1, target_force_n=6.0, force_hold_s=0.01))

    force_samples = [sample for sample in samples if sample.phase == ChoppingPhase.FORCE_HOLD]
    assert force_samples
    assert any(abs(sample.raw_wrench[2] - sample.target_force_n) > 1e-9 for sample in force_samples)


def test_descent_and_retract_speed_control_phase_sample_counts():
    fast = RightArmChopper().run(
        ChoppingConfig(cycles=1, control_hz=20.0, descent_speed_m_s=0.2, retract_speed_m_s=0.2, force_hold_s=0.01)
    )
    slow = RightArmChopper().run(
        ChoppingConfig(cycles=1, control_hz=20.0, descent_speed_m_s=0.02, retract_speed_m_s=0.02, force_hold_s=0.01)
    )

    assert _phase_count(slow, ChoppingPhase.DESCEND) > _phase_count(fast, ChoppingPhase.DESCEND)
    assert _phase_count(slow, ChoppingPhase.RETRACT) > _phase_count(fast, ChoppingPhase.RETRACT)


def _phase_count(samples, phase):
    return sum(1 for sample in samples if sample.phase == phase)


def test_controller_input_wrench_is_deterministic_by_phase(monkeypatch):
    import twin_mujoco.chopping as chopping

    observed = []
    current_phase = []
    original_controller = chopping.CartesianForceController
    original_record_phase = chopping.RightArmChopper._record_phase

    class RecordingController(original_controller):
        def compute(self, wrench):
            observed.append((current_phase[-1], tuple(float(value) for value in wrench)))
            return super().compute(wrench)

    def record_phase(self, elapsed, phase, *args, **kwargs):
        current_phase.append(phase)
        try:
            return original_record_phase(self, elapsed, phase, *args, **kwargs)
        finally:
            current_phase.pop()

    monkeypatch.setattr(chopping, "CartesianForceController", RecordingController)
    monkeypatch.setattr(chopping.RightArmChopper, "_record_phase", record_phase)

    RightArmChopper().run(ChoppingConfig(cycles=1, target_force_n=6.0, force_hold_s=0.01))

    free_space = [wrench for phase, wrench in observed if phase != ChoppingPhase.FORCE_HOLD]
    force_hold = [wrench for phase, wrench in observed if phase == ChoppingPhase.FORCE_HOLD]
    assert free_space
    assert force_hold
    assert all(wrench == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0) for wrench in free_space)
    assert all(wrench == (0.0, 0.0, 6.0, 0.0, 0.0, 0.0) for wrench in force_hold)


def test_descent_targets_interpolate_instead_of_jumping_to_endpoint():
    samples = RightArmChopper().run(
        ChoppingConfig(cycles=1, control_hz=20.0, descent_speed_m_s=0.02, force_hold_s=0.01)
    )

    descend_targets = [sample.target_position[2] for sample in samples if sample.phase == ChoppingPhase.DESCEND]
    assert len(descend_targets) > 2
    assert descend_targets[0] > descend_targets[-1]
    assert len(set(descend_targets)) > 2


def test_logged_time_matches_mujoco_time_with_nondefault_control_hz():
    chopper = RightArmChopper()
    samples = chopper.run(
        ChoppingConfig(cycles=1, control_hz=20.0, descent_speed_m_s=0.2, retract_speed_m_s=0.2, force_hold_s=0.05)
    )

    assert abs(samples[-1].time_s - chopper.runtime.data.time) <= chopper.runtime.timestep


def test_first_active_sample_time_matches_post_step_simulation_time():
    chopper = RightArmChopper()
    samples = chopper.run(ChoppingConfig(cycles=1, control_hz=20.0, force_hold_s=0.01))

    assert samples[0].time_s > 0.0
    assert abs(samples[0].time_s - 0.05) <= chopper.runtime.timestep


def test_chopping_rejects_invalid_cycle_count():
    chopper = RightArmChopper()

    try:
        chopper.run(ChoppingConfig(cycles=0))
    except ValueError as exc:
        assert "cycles must be at least 1" in str(exc)
    else:
        raise AssertionError("expected invalid cycle count to fail")


def test_cli_headless_mode_still_runs(tmp_path):
    from twin_mujoco.cli import main

    log_path = tmp_path / "headless_chop.csv"

    assert (
        main(["--cycles", "1", "--hold", "0.01", "--headless", "--log", str(log_path)])
        == 0
    )
    assert log_path.is_file()


def test_chopper_syncs_viewer_callback_during_run():
    sync_count = 0

    def sync_viewer():
        nonlocal sync_count
        sync_count += 1

    samples = RightArmChopper().run(
        ChoppingConfig(cycles=1, control_hz=20.0, force_hold_s=0.01),
        viewer_sync=sync_viewer,
    )

    assert sync_count == len(samples)
    assert sync_count > 1


def test_cli_viewer_mode_launches_passive_viewer(monkeypatch, tmp_path):
    import twin_mujoco.cli as cli

    launches = []
    syncs = []

    class FakeCamera:
        def __init__(self):
            self.lookat = np.zeros(3)
            self.distance = 0.0
            self.azimuth = 0.0
            self.elevation = 0.0

    class FakeViewer:
        def __init__(self):
            self.cam = FakeCamera()

        def __enter__(self):
            launches.append("enter")
            return self

        def __exit__(self, exc_type, exc, traceback):
            launches.append("exit")

        def sync(self):
            syncs.append("sync")

    def launch_passive(model, data):
        launches.append((model.njnt, data.time))
        return FakeViewer()

    monkeypatch.setattr(cli.mujoco.viewer, "launch_passive", launch_passive)

    log_path = tmp_path / "viewer_chop.csv"

    assert (
        cli.main(["--cycles", "1", "--hold", "0.01", "--viewer", "--log", str(log_path)])
        == 0
    )
    assert log_path.is_file()
    assert launches[0][0] == 14
    assert launches[1:] == ["enter", "exit"]
    assert syncs


def test_cli_viewer_mode_configures_front_camera(monkeypatch, tmp_path):
    import twin_mujoco.cli as cli

    class FakeCamera:
        def __init__(self):
            self.lookat = np.zeros(3)
            self.distance = 0.0
            self.azimuth = 0.0
            self.elevation = 0.0

    class FakeViewer:
        def __init__(self):
            self.cam = FakeCamera()

        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, traceback):
            pass

        def sync(self):
            pass

    viewer = FakeViewer()
    monkeypatch.setattr(cli.mujoco.viewer, "launch_passive", lambda model, data: viewer)

    assert (
        cli.main(["--cycles", "1", "--hold", "0.01", "--viewer", "--log", str(tmp_path / "viewer.csv")])
        == 0
    )
    np.testing.assert_allclose(viewer.cam.lookat, [0.3, 0.0, 0.36])
    assert viewer.cam.distance == 1.15
    assert viewer.cam.azimuth == 180
    assert viewer.cam.elevation == -20


def test_chopper_applies_left_arm_holding_torque(monkeypatch):
    from twin_mujoco.runtime import ArmView

    calls = []
    original_apply_torque = ArmView.apply_torque

    def record_apply_torque(self, torque_nm):
        if self.spec.name == "left":
            calls.append(tuple(float(value) for value in torque_nm))
        return original_apply_torque(self, torque_nm)

    monkeypatch.setattr(ArmView, "apply_torque", record_apply_torque)

    RightArmChopper().run(ChoppingConfig(cycles=1, control_hz=20.0, force_hold_s=0.01))

    assert calls
    assert all(len(torque) == 7 for torque in calls)
    assert any(any(abs(value) > 1e-9 for value in torque) for torque in calls)


def test_chopper_keeps_left_arm_near_home_pose():
    from twin_mujoco.chopping import LEFT_HOME_Q

    chopper = RightArmChopper()

    chopper.run(ChoppingConfig(cycles=1, control_hz=20.0, force_hold_s=0.01))

    left = chopper.runtime.arm_view("left")
    assert np.linalg.norm(left.joint_positions - LEFT_HOME_Q) < 0.01


def test_chopper_initializes_safe_right_home_before_logging_samples():
    chopper = RightArmChopper()

    samples = chopper.run(ChoppingConfig(cycles=1, control_hz=20.0, force_hold_s=0.01))

    assert samples
    runtime = TwinMujocoRuntime.load(right_chopping_scene_path())
    board_id = runtime._id(__import__("mujoco").mjtObj.mjOBJ_GEOM, "chopping_board")
    board_top = runtime.model.geom_pos[board_id, 2] + runtime.model.geom_size[board_id, 2]
    assert samples[0].actual_position[2] > board_top + 0.05


def test_blade_geometry_reads_two_finite_edge_positions_and_tip_pose():
    chopper = RightArmChopper()

    geometry = chopper._blade_geometry()

    assert len(geometry.edge_positions) == 2
    assert len(geometry.z_values) == 2
    assert geometry.min_z <= geometry.max_z
    assert len(geometry.tip_position) == 3
    assert np.asarray(geometry.tip_rotation).shape == (3, 3)
    for position in geometry.edge_positions + (geometry.tip_position,):
        assert len(position) == 3
        assert np.all(np.isfinite(position))


def test_horizontal_blade_rotation_predicts_edge_points_at_equal_height():
    chopper = RightArmChopper()
    chopper.runtime.reset()
    chopper.runtime.set_arm_positions("left", LEFT_HOME_Q)
    chopper.runtime.set_arm_positions("right", RIGHT_CHOPPING_HOME_Q)
    geometry = chopper._blade_geometry()

    rotation = chopper._horizontal_blade_rotation(geometry)
    predicted = chopper._predict_blade_edge_positions(geometry.tip_position, rotation, geometry)

    assert abs(predicted[0][2] - predicted[1][2]) <= 1e-9


def test_two_point_safe_target_keeps_both_blade_edges_above_board():
    chopper = RightArmChopper()
    chopper.runtime.reset()
    chopper.runtime.set_arm_positions("left", LEFT_HOME_Q)
    chopper.runtime.set_arm_positions("right", RIGHT_CHOPPING_HOME_Q)
    board_top = chopper._board_top()
    geometry = chopper._blade_geometry()
    rotation = chopper._horizontal_blade_rotation(geometry)

    target = chopper._tip_target_for_blade_clearance(
        np.asarray(geometry.tip_position),
        rotation,
        geometry,
        board_top,
        clearance_m=0.08,
    )
    predicted = chopper._predict_blade_edge_positions(target, rotation, geometry)

    assert all(position[2] >= board_top + 0.08 - 1e-9 for position in predicted)


def test_two_point_descend_target_predicts_both_blade_edges_on_board():
    chopper = RightArmChopper()
    chopper.runtime.reset()
    chopper.runtime.set_arm_positions("left", LEFT_HOME_Q)
    chopper.runtime.set_arm_positions("right", RIGHT_CHOPPING_HOME_Q)
    board_top = chopper._board_top()
    geometry = chopper._blade_geometry()
    rotation = chopper._horizontal_blade_rotation(geometry)

    target = chopper._tip_target_for_blade_on_board(
        np.asarray(geometry.tip_position),
        rotation,
        geometry,
        board_top,
    )
    predicted = chopper._predict_blade_edge_positions(target, rotation, geometry)

    for position in predicted:
        assert abs(position[2] - board_top) <= 1e-9
