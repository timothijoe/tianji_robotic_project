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


def test_mujoco_position_command_moves_toward_joint_target():
    target = np.array([-75.627, -67.572, 52.390, -124.574, -90.421, 42.952, 41.374])
    robot = MujocoSdkRobot(
        arm="B",
        viewer=False,
        realtime=False,
        control_hz=250.0,
        tcp_site_name="right_tool_tip_site",
    )
    try:
        assert robot.connect("mujoco")
        initial_error = float(np.linalg.norm(robot.get_joint_positions() - target))
        assert robot.set_position_state("B", 30, 30)
        assert robot.set_joint_position_cmd("B", target)
        robot.wait(0.5, viewer_sync=False)
        final_error = float(np.linalg.norm(robot.get_joint_positions() - target))

        assert final_error < initial_error - 5.0
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


def test_ik_cart_impedance_demo_targets_current_feedback_pose():
    demo_path = (
        Path(__file__).resolve().parents[1]
        / "examples"
        / "DEMO_PYTHON_STYLE"
        / "showcase_ik_cart_impedance.py"
    )
    spec = importlib.util.spec_from_file_location("_sdk_ik_cart_impedance_demo_run", demo_path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    spec.loader.exec_module(module)

    result = module.run_demo(
        backend="mujoco",
        viewer=False,
        realtime=False,
        control_hz=100.0,
        dz_mm=-5.0,
        hold_s=0.01,
    )

    reference = np.asarray(result["reference_joints"], dtype=float)
    target = np.asarray(result["target_joints"], dtype=float)
    assert np.max(np.abs(target - reference)) < 25.0


def test_ik_cart_impedance_showcase_uses_chopping_home_and_moves_down_up():
    demo_path = (
        Path(__file__).resolve().parents[1]
        / "examples"
        / "DEMO_PYTHON_STYLE"
        / "showcase_ik_cart_impedance.py"
    )
    spec = importlib.util.spec_from_file_location("_sdk_ik_cart_impedance_demo_down_up", demo_path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    spec.loader.exec_module(module)

    result = module.run_demo(
        backend="mujoco",
        viewer=False,
        realtime=False,
        control_hz=100.0,
        dz_mm=-20.0,
        hold_s=0.5,
        cycles=1,
    )

    target_z = np.asarray(result["target_z_trace_mm"], dtype=float)
    actual_z = np.asarray(result["actual_z_trace_mm"], dtype=float)
    reference = np.asarray(result["reference_joints"], dtype=float)
    chopping_home_deg = np.asarray(result["chopping_home_joints"], dtype=float)

    assert result["motion_steps"] == 50
    assert np.max(np.abs(reference - chopping_home_deg)) < 1e-6
    assert np.isclose(target_z[0], target_z[-1], atol=1e-6)
    assert np.min(target_z) < target_z[0] - 18.0
    assert np.min(actual_z) < actual_z[0] - 4.0
    assert np.max(actual_z) < actual_z[0] + 30.0
    assert result["execution_mode"] == "CARTESIAN_IMPEDANCE_OSCILLATION"


def test_ik_cart_impedance_showcase_hold_s_is_per_cycle():
    demo_path = (
        Path(__file__).resolve().parents[1]
        / "examples"
        / "DEMO_PYTHON_STYLE"
        / "showcase_ik_cart_impedance.py"
    )
    spec = importlib.util.spec_from_file_location("_sdk_ik_cart_impedance_demo_cycles", demo_path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    spec.loader.exec_module(module)

    result = module.run_demo(
        backend="mujoco",
        viewer=False,
        realtime=False,
        control_hz=20.0,
        dz_mm=-10.0,
        hold_s=0.1,
        cycles=3,
    )

    assert result["motion_steps"] == 9
    target_z = np.asarray(result["target_z_trace_mm"], dtype=float)
    assert np.min(target_z) < target_z[0] - 8.0


def test_ik_cart_impedance_showcase_cli_writes_plot(tmp_path):
    import os
    import subprocess

    plot_path = tmp_path / "ik_cart_trace.png"
    env = os.environ.copy()
    env["PYTHONPATH"] = "src:src/twin_core:src/twin_description:src/twin_mujoco"

    result = subprocess.run(
        [
            sys.executable,
            "examples/DEMO_PYTHON_STYLE/showcase_ik_cart_impedance.py",
            "--backend",
            "mujoco",
            "--headless",
            "--control-hz",
            "20",
            "--dz-mm",
            "-10",
            "--hold-s",
            "0.1",
            "--cycles",
            "1",
            "--plot",
            str(plot_path),
        ],
        cwd=Path(__file__).resolve().parents[1],
        env=env,
        check=False,
        text=True,
        capture_output=True,
        timeout=30,
    )

    assert result.returncode == 0, result.stderr
    assert plot_path.is_file()
    assert plot_path.stat().st_size > 0


def test_ik_cart_impedance_showcase_appends_viewer_trace_markers():
    demo_path = (
        Path(__file__).resolve().parents[1]
        / "examples"
        / "DEMO_PYTHON_STYLE"
        / "showcase_ik_cart_impedance.py"
    )
    spec = importlib.util.spec_from_file_location("_sdk_ik_cart_impedance_demo_trace", demo_path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    spec.loader.exec_module(module)

    class FakeGeom:
        __slots__ = (
            "type", "size", "pos", "mat", "rgba", "segid", "objtype",
            "objid", "category", "dataid", "emission", "specular",
            "shininess", "reflectance", "label",
        )

        def __init__(self):
            self.size = np.zeros(3)
            self.pos = np.zeros(3)
            self.mat = np.zeros((3, 3))
            self.rgba = np.zeros(4)
            self.label = bytearray(100)

    class FakeScene:
        def __init__(self):
            self.ngeom = 0
            self.maxgeom = 4
            self.geoms = [FakeGeom() for _ in range(4)]

    class FakeLock:
        def __enter__(self):
            return None

        def __exit__(self, exc_type, exc, traceback):
            return None

    class FakeViewer:
        def __init__(self):
            self.user_scn = FakeScene()

        def lock(self):
            return FakeLock()

    viewer = FakeViewer()
    module._append_viewer_trace_marker(
        viewer, np.array([0.1, 0.2, 0.3]), (0.0, 0.85, 1.0, 0.9), 0.004,
    )
    module._append_viewer_trace_marker(
        viewer, np.array([0.4, 0.5, 0.6]), (1.0, 0.45, 0.0, 0.9), 0.004,
    )

    assert viewer.user_scn.ngeom == 2
    np.testing.assert_allclose(viewer.user_scn.geoms[0].pos, [0.1, 0.2, 0.3])
    np.testing.assert_allclose(viewer.user_scn.geoms[1].pos, [0.4, 0.5, 0.6])
    assert not np.allclose(viewer.user_scn.geoms[0].rgba, viewer.user_scn.geoms[1].rgba)


def test_ik_cart_impedance_showcase_lateral_flag_offsets_y_only_between_vertical_cuts():
    demo_path = (
        Path(__file__).resolve().parents[1]
        / "examples"
        / "DEMO_PYTHON_STYLE"
        / "showcase_ik_cart_impedance.py"
    )
    spec = importlib.util.spec_from_file_location("_sdk_ik_cart_impedance_demo_lateral", demo_path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    spec.loader.exec_module(module)

    stationary = module.run_demo(
        backend="mujoco",
        viewer=False,
        realtime=False,
        control_hz=30.0,
        dz_mm=-5.0,
        hold_s=0.2,
        cycles=1,
    )
    moving = module.run_demo(
        backend="mujoco",
        viewer=False,
        realtime=False,
        control_hz=20.0,
        dz_mm=-5.0,
        hold_s=0.2,
        cycles=3,
        lateral=True,
        lateral_mm=10.0,
    )

    stationary_y = np.asarray(stationary["target_y_trace_mm"], dtype=float)
    moving_y = np.asarray(moving["target_y_trace_mm"], dtype=float)
    moving_x = np.asarray(moving["target_x_trace_mm"], dtype=float)
    assert np.ptp(stationary_y) < 1e-6
    assert np.ptp(moving_x) < 1e-6

    steps_per_cycle = moving["motion_steps"] // 3
    for cycle_index in range(3):
        cycle_y = moving_y[
            cycle_index * steps_per_cycle : (cycle_index + 1) * steps_per_cycle
        ]
        descent_y = cycle_y[: steps_per_cycle // 2]
        assert np.ptp(descent_y) < 1e-6
        assert np.isclose(descent_y[0], moving_y[0] + 10.0 * cycle_index, atol=1e-6)

    assert np.isclose(moving_y[-1], moving_y[0] + 30.0, atol=1e-6)


def test_ik_cart_impedance_showcase_cli_accepts_lateral_flag(tmp_path):
    import os
    import subprocess

    env = os.environ.copy()
    env["PYTHONPATH"] = "src:src/twin_core:src/twin_description:src/twin_mujoco"
    result = subprocess.run(
        [
            sys.executable,
            "examples/DEMO_PYTHON_STYLE/showcase_ik_cart_impedance.py",
            "--backend",
            "mujoco",
            "--headless",
            "--control-hz",
            "20",
            "--dz-mm",
            "-5",
            "--hold-s",
            "0.1",
            "--cycles",
            "1",
            "--lateral",
            "--lateral-mm",
            "25",
        ],
        cwd=Path(__file__).resolve().parents[1],
        env=env,
        check=False,
        text=True,
        capture_output=True,
        timeout=30,
    )

    assert result.returncode == 0, result.stderr
    assert "lateral: True" in result.stdout
