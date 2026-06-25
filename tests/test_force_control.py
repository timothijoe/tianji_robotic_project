import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from cook_description.paths import CHOPPING_MJCF_PATH, DEFAULT_MJCF_PATH
from cook_mujoco.chopping import ChoppingConfig, DEFAULT_HOME_Q, MujocoForceRobot
from cook_mujoco.control.force_control import (
    CartesianForceController,
    ChoppingPhase,
    ForceControlRuntime,
    WrenchCalibrator,
    downward_tool_rotation,
    rotation_error,
)


def test_models_expose_torque_actuators_and_six_axis_sensor():
    base = mujoco.MjModel.from_xml_path(str(DEFAULT_MJCF_PATH))
    scene = mujoco.MjModel.from_xml_path(str(CHOPPING_MJCF_PATH))

    assert base.nu == 7
    assert scene.nu == 7
    assert scene.nsensor == 2
    assert scene.nsensordata == 6
    assert scene.opt.timestep == pytest.approx(0.002)


def test_safe_home_keeps_wrist_above_tip_and_board():
    runtime = ForceControlRuntime.load(CHOPPING_MJCF_PATH)
    runtime.reset(DEFAULT_HOME_Q)
    tip, _ = runtime.site_pose()
    sensor, _ = runtime.site_pose(runtime.sensor_site_name)
    board_top = 0.26

    assert tip[2] == pytest.approx(0.3415, abs=2e-3)
    assert runtime.site_pose()[1][:, 2] == pytest.approx([0.0, 0.0, -1.0], abs=8e-3)
    assert sensor[2] > tip[2]
    assert sensor[2] > board_top
    sensor_body = runtime._id(runtime._mujoco.mjtObj.mjOBJ_BODY, "force_sensor_body")
    assert runtime.model.body_pos[sensor_body] == pytest.approx([0.0, -0.095, 0.0])
    assert runtime.model.body_quat[sensor_body] == pytest.approx([0.5, 0.5, -0.5, 0.5])


def test_runtime_exposes_pose_jacobian_wrench_and_limits():
    runtime = ForceControlRuntime.load(CHOPPING_MJCF_PATH)

    position, rotation = runtime.site_pose()
    assert position.shape == (3,)
    assert rotation.shape == (3, 3)
    assert runtime.site_jacobian().shape == (6, 7)
    assert runtime.raw_wrench().shape == (6,)
    assert runtime.effort_limits == pytest.approx([108, 108, 66, 66, 18, 18, 18])


def test_rotation_helpers_define_downward_tool_frame():
    rotation = downward_tool_rotation()

    assert rotation.T @ rotation == pytest.approx(np.eye(3), abs=1e-12)
    assert rotation[:, 2] == pytest.approx([0, 0, -1])
    assert rotation_error(rotation, rotation) == pytest.approx(np.zeros(3))
    opposite = rotation @ np.diag([1.0, -1.0, -1.0])
    assert np.linalg.norm(rotation_error(rotation, opposite)) == pytest.approx(np.pi)


def test_wrench_calibration_zeroes_stationary_reading():
    rotation = downward_tool_rotation()
    calibrator = WrenchCalibrator(tool_mass_kg=0.22)
    modeled = calibrator.modeled_gravity_wrench(rotation)
    raw = modeled + np.array([0.2, -0.1, 0.3, 0.01, 0.02, -0.03])

    calibrator.calibrate([raw, raw], rotation)

    assert calibrator.compensate(raw, rotation) == pytest.approx(np.zeros(6), abs=1e-12)


def test_controller_produces_finite_limited_torque():
    runtime = ForceControlRuntime.load(CHOPPING_MJCF_PATH)
    controller = CartesianForceController(runtime)

    torque = controller.compute(np.zeros(6))

    assert torque.shape == (7,)
    assert np.all(np.isfinite(torque))
    assert np.all(np.abs(torque) <= runtime.effort_limits)


def test_one_chopping_cycle_completes_with_bounded_force_and_tracking_error(tmp_path):
    robot = MujocoForceRobot(CHOPPING_MJCF_PATH)
    robot.initialize(calibration_s=0.2)

    samples = robot.execute_chopping_trajectory(
        config=ChoppingConfig(cycles=1, force_hold_s=0.5),
        log_path=tmp_path / "chopping.csv",
    )
    summary = robot.summary()

    assert samples
    assert any(sample.phase == ChoppingPhase.FORCE_HOLD for sample in samples)
    assert summary["phase"] == ChoppingPhase.COMPLETE.value
    assert summary["peak_force_n"] < 30.0
    assert summary["force_hold_rmse_n"] < 1.5
    assert summary["noncontact_position_rmse_m"] < 0.006
    assert (tmp_path / "chopping.csv").is_file()
    header = (tmp_path / "chopping.csv").read_text().splitlines()[0]
    assert "control_mode" in header
    assert {sample.control_mode for sample in samples} == {"CARTESIAN_IMPEDANCE", "HYBRID_FORCE_Z"}
