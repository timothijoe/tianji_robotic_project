import sys
import types
import importlib.util
from pathlib import Path

import numpy as np
import pytest

from twin_control.sdk_compat import (
    MujocoSdkRobot,
    RealSdkRobotAdapter,
    create_ik_param,
    create_kine,
    create_robot,
)
from twin_control.sdk_kine import FX_InvKineSolvePara, MujocoKine


def test_mujoco_sdk_robot_runs_position_command_headless():
    robot = MujocoSdkRobot(arm="B", viewer=False, realtime=False, control_hz=100.0)
    try:
        assert robot.connect("mujoco")
        assert robot.set_position_state("B", 30, 30)
        assert robot.set_joint_position_cmd("B", [-75, -68, 52, -124, -90, 43, 41])
        robot.wait(0.02, viewer_sync=False)
        joints = robot.get_joint_positions()
        assert joints.shape == (7,)
        assert np.all(np.isfinite(joints))
        data = robot.subscribe(None)
        assert data["states"][1]["cur_state"] == 1
        assert data["outputs"][1]["control_mode"] == "POSITION"
    finally:
        robot.release_robot()


def test_mujoco_sdk_robot_rejects_wrong_arm_for_instance():
    robot = MujocoSdkRobot(arm="B", viewer=False, realtime=False)
    try:
        robot.connect("mujoco")
        with pytest.raises(NotImplementedError):
            robot.set_position_state("A", 30, 30)
    finally:
        robot.release_robot()


def test_create_real_robot_uses_vendor_concise_class(tmp_path, monkeypatch):
    class FakeConciseRobot:
        def __init__(self):
            self.subscribed_with = None

        def subscribe(self, dcss):
            self.subscribed_with = dcss
            return {"outputs": [{"fb_joint_pos": [1, 2, 3, 4, 5, 6, 7]}, {}]}

    sdk_python = types.ModuleType("SDK_PYTHON")
    fx_robot = types.ModuleType("SDK_PYTHON.fx_robot")
    fx_robot.Concise_Marvin_Robot = FakeConciseRobot
    fx_robot.DCSS = type("FakeDCSS", (), {})
    monkeypatch.setitem(sys.modules, "SDK_PYTHON", sdk_python)
    monkeypatch.setitem(sys.modules, "SDK_PYTHON.fx_robot", fx_robot)

    robot = create_robot("real", sdk_root=tmp_path)

    assert isinstance(robot, RealSdkRobotAdapter)
    assert isinstance(robot.robot, FakeConciseRobot)
    assert callable(robot.wait)
    data = robot.subscribe(None)
    assert data["outputs"][0]["fb_joint_pos"] == [1, 2, 3, 4, 5, 6, 7]
    assert isinstance(robot.robot.subscribed_with, fx_robot.DCSS)


def test_create_kine_mujoco_runs_fk_ik_and_jacobian():
    kine = create_kine("mujoco", arm_type=1, tcp_site_name="right_tool_tip_site")
    config = kine.load_config(arm_type=1, config_path="ccs_m6_40.MvKDCfg")

    assert config["TYPE"][1] == 1007
    assert kine.initial_kine(
        robot_type=config["TYPE"][1],
        dh=config["DH"][1],
        pnva=config["PNVA"][1],
        j67=config["BD"][1],
    )

    joints = [-75.627, -67.572, 52.390, -124.574, -90.421, 42.952, 41.374]
    pose = kine.fk(joints)
    assert np.asarray(pose).shape == (4, 4)
    assert np.all(np.isfinite(pose))

    sp = FX_InvKineSolvePara()
    sp.set_input_ik_target_tcp(kine.mat4x4_to_mat1x16(pose))
    sp.set_input_ik_ref_joint(joints)
    sp.set_input_ik_zsp_type(0)
    result = kine.ik(sp)
    assert result.get_output_result_num() >= 1
    assert np.allclose(result.get_output_ret_joint(), joints, atol=1e-2)

    jacobian = kine.joints2JacobMatrix(joints)
    assert np.asarray(jacobian).shape == (6, 7)
    assert np.all(np.isfinite(jacobian))


def test_create_kine_real_uses_vendor_kine_class(tmp_path, monkeypatch):
    class FakeKine:
        pass

    sdk_python = types.ModuleType("SDK_PYTHON")
    fx_kine = types.ModuleType("SDK_PYTHON.fx_kine")
    fx_kine.Marvin_Kine = FakeKine
    monkeypatch.setitem(sys.modules, "SDK_PYTHON", sdk_python)
    monkeypatch.setitem(sys.modules, "SDK_PYTHON.fx_kine", fx_kine)

    kine = create_kine("real", sdk_root=tmp_path)

    assert isinstance(kine, FakeKine)


def test_create_ik_param_uses_matching_backend(tmp_path, monkeypatch):
    class FakeIkParam:
        pass

    sdk_python = types.ModuleType("SDK_PYTHON")
    fx_kine = types.ModuleType("SDK_PYTHON.fx_kine")
    fx_kine.FX_InvKineSolvePara = FakeIkParam
    monkeypatch.setitem(sys.modules, "SDK_PYTHON", sdk_python)
    monkeypatch.setitem(sys.modules, "SDK_PYTHON.fx_kine", fx_kine)

    assert isinstance(create_ik_param("mujoco"), FX_InvKineSolvePara)
    assert isinstance(create_ik_param("real", sdk_root=tmp_path), FakeIkParam)


def test_cartesian_impedance_joint_command_sets_fk_cartesian_target():
    robot = MujocoSdkRobot(
        arm="B",
        viewer=False,
        realtime=False,
        control_hz=100.0,
        tcp_site_name="right_tool_tip_site",
    )
    try:
        assert robot.connect("mujoco")
        assert robot.set_imp_cart_state(
            "B",
            velRatio=50,
            AccRatio=50,
            K=[2500, 2500, 2800, 45, 45, 35, 4],
            D=[105, 105, 115, 5.5, 5.5, 4.5, 1.5],
            rot_type=0,
            cart_ctrl_para=[0] * 7,
        )
        target = [-75.627, -67.572, 52.390, -124.574, -90.421, 42.952, 41.374]
        assert robot.set_joint_position_cmd("B", target)

        internal = robot._require_robot()
        actual_target = internal._controller._cart_target_matrix
        expected_target = internal._kinematics.fk(np.asarray(target, dtype=float))[0]
        assert actual_target is not None
        assert np.allclose(actual_target, expected_target)
    finally:
        robot.release_robot()


def test_subscribe_exposes_sdk_feedback_fields():
    robot = MujocoSdkRobot(arm="B", viewer=False, realtime=False, control_hz=100.0)
    try:
        assert robot.connect("mujoco")
        data = robot.subscribe(None)
        out = data["outputs"][1]
        inp = data["inputs"][1]

        for key in (
            "frame_serial",
            "fb_joint_pos",
            "fb_joint_vel",
            "fb_joint_cmd",
            "fb_joint_sToq",
            "est_cart_fn",
            "low_speed_flag",
            "traj_state",
            "control_mode",
        ):
            assert key in out
        for key in ("joint_cmd_pos", "joint_vel_ratio", "joint_acc_ratio", "imp_type"):
            assert key in inp
    finally:
        robot.release_robot()


def test_ik_cart_impedance_demo_exposes_run_demo_function():
    demo_path = (
        Path(__file__).resolve().parents[1]
        / "examples"
        / "DEMO_PYTHON_STYLE"
        / "showcase_ik_cart_impedance.py"
    )
    spec = importlib.util.spec_from_file_location("_sdk_ik_cart_impedance_demo", demo_path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    spec.loader.exec_module(module)

    assert callable(module.run_demo)
