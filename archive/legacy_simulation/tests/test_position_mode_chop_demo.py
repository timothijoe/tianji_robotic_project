import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest


def _load_module():
    path = Path(__file__).resolve().parents[1] / "examples" / "position_mode_chop_demo.py"
    spec = importlib.util.spec_from_file_location("position_mode_chop_demo", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_cli_matches_sdk_style_relative_motion_args():
    module = _load_module()

    args = module.parse_args([
        "--backend", "mujoco",
        "--viewer",
        "--control-hz", "250",
        "--dz-mm", "-20",
        "--hold-s", "2.0",
        "--cycles", "5",
        "--lateral",
        "--lateral-mm", "10",
        "--viewer-trace",
        "--viewer-trace-stride", "10",
    ])

    assert args.backend == "mujoco"
    assert args.viewer is True
    assert args.control_hz == 250.0
    assert args.dz_mm == -20.0
    assert args.hold_s == 2.0
    assert args.cycles == 5
    assert args.lateral is True
    assert args.lateral_mm == 10.0
    assert args.viewer_trace is True
    assert args.viewer_trace_stride == 10


def test_relative_target_sequence_descends_retracts_and_laterally_shifts():
    module = _load_module()
    start = np.array((0.4, 0.2, 0.3), dtype=float)

    targets = module.build_relative_tcp_targets(
        start_pos_m=start,
        dz_mm=-20.0,
        hold_s=2.0,
        control_hz=2.0,
        cycles=2,
        lateral=True,
        lateral_mm=10.0,
    )

    assert len(targets) == 8
    np.testing.assert_allclose(targets[0], start)
    assert min(float(target[2]) for target in targets) == pytest.approx(0.28)
    assert targets[0][1] == pytest.approx(0.2)
    assert targets[3][1] == pytest.approx(0.21)
    assert targets[4][1] == pytest.approx(0.21)
    assert targets[-1][1] == pytest.approx(0.22)


def test_blade_aligned_pose_targets_use_chopper_rotation():
    module = _load_module()

    class FakeChopper:
        def __init__(self):
            self.rotation = np.array(
                (
                    (0.0, -1.0, 0.0),
                    (1.0, 0.0, 0.0),
                    (0.0, 0.0, 1.0),
                ),
                dtype=float,
            )
            self.safe_tip = np.array((0.4, 0.2, 0.31), dtype=float)
            self.descend_tip = np.array((0.4, 0.2, 0.29), dtype=float)

        def _blade_geometry(self):
            return object()

        def _horizontal_blade_rotation(self, geometry):
            return self.rotation

        def _board_top(self):
            return 0.28

        def _tip_target_for_blade_clearance(self, tip, rotation, geometry, board_top, clearance_m):
            return self.safe_tip.copy()

        def _tip_target_for_blade_on_board(self, tip, rotation, geometry, board_top):
            return self.descend_tip.copy()

    poses = module.build_blade_aligned_pose_targets(
        chopper=FakeChopper(),
        start_tip_m=np.array((0.35, 0.2, 0.35), dtype=float),
        dz_mm=-20.0,
        hold_s=1.0,
        control_hz=4.0,
        cycles=1,
        lateral=False,
        lateral_mm=0.0,
    )

    assert poses
    expected_rotation = FakeChopper().rotation
    for pose in poses:
        np.testing.assert_allclose(pose[:3, :3], expected_rotation)
    np.testing.assert_allclose(poses[-1][:3, 3], np.array((0.35, 0.2, 0.35), dtype=float))



def test_blade_aligned_pose_targets_clamp_descent_above_board_clearance():
    module = _load_module()

    class FakeGeometry:
        tip_position = (0.4, 0.2, 0.33)

    class FakeChopper:
        def _blade_geometry(self):
            return FakeGeometry()

        def _horizontal_blade_rotation(self, geometry):
            return np.eye(3)

        def _board_top(self):
            return 0.23

        def _tip_target_for_blade_clearance(self, tip, rotation, geometry, board_top, clearance_m):
            target = np.asarray(tip, dtype=float).copy()
            target[2] = float(board_top) + float(clearance_m)
            return target

        def _tip_target_for_blade_on_board(self, tip, rotation, geometry, board_top):
            target = np.asarray(tip, dtype=float).copy()
            target[2] = float(board_top)
            return target

    poses = module.build_blade_aligned_pose_targets(
        chopper=FakeChopper(),
        start_tip_m=np.array((0.4, 0.2, 0.33), dtype=float),
        dz_mm=-120.0,
        hold_s=1.0,
        control_hz=4.0,
        cycles=1,
        lateral=False,
        lateral_mm=0.0,
        contact_clearance_mm=30.0,
    )

    assert min(float(pose[2, 3]) for pose in poses) == pytest.approx(0.26)


def test_endpoint_targets_use_two_position_goals_per_cycle():
    module = _load_module()

    class FakeGeometry:
        tip_position = (0.4, 0.2, 0.33)

    class FakeChopper:
        def _blade_geometry(self):
            return FakeGeometry()

        def _horizontal_blade_rotation(self, geometry):
            return np.eye(3)

        def _board_top(self):
            return 0.23

        def _tip_target_for_blade_clearance(self, tip, rotation, geometry, board_top, clearance_m):
            target = np.asarray(tip, dtype=float).copy()
            target[2] = float(board_top) + float(clearance_m)
            return target

    endpoints = module.build_blade_aligned_endpoint_targets(
        chopper=FakeChopper(),
        start_tip_m=np.array((0.4, 0.2, 0.33), dtype=float),
        dz_mm=-60.0,
        hold_s=0.8,
        control_hz=10.0,
        cycles=2,
        lateral=True,
        lateral_mm=10.0,
        contact_clearance_mm=30.0,
    )

    assert len(endpoints) == 4
    assert [steps for _, steps in endpoints] == [4, 4, 4, 4]
    np.testing.assert_allclose(endpoints[0][0][:3, 3], np.array((0.4, 0.2, 0.27)))
    np.testing.assert_allclose(endpoints[1][0][:3, 3], np.array((0.4, 0.2, 0.33)))
    np.testing.assert_allclose(endpoints[2][0][:3, 3], np.array((0.4, 0.21, 0.27)))
    np.testing.assert_allclose(endpoints[3][0][:3, 3], np.array((0.4, 0.21, 0.33)))


def test_waypoint_targets_sample_minimum_jerk_segments():
    module = _load_module()

    class FakeGeometry:
        tip_position = (0.4, 0.2, 0.33)

    class FakeChopper:
        def _blade_geometry(self):
            return FakeGeometry()

        def _horizontal_blade_rotation(self, geometry):
            return np.eye(3)

        def _board_top(self):
            return 0.23

        def _tip_target_for_blade_clearance(self, tip, rotation, geometry, board_top, clearance_m):
            target = np.asarray(tip, dtype=float).copy()
            target[2] = float(board_top) + float(clearance_m)
            return target

    waypoints = module.build_blade_aligned_waypoint_targets(
        chopper=FakeChopper(),
        start_tip_m=np.array((0.4, 0.2, 0.33), dtype=float),
        dz_mm=-60.0,
        hold_s=0.8,
        control_hz=10.0,
        cycles=1,
        lateral=False,
        lateral_mm=0.0,
        contact_clearance_mm=30.0,
        waypoints_per_segment=3,
    )

    assert len(waypoints) == 6
    assert [steps for _, steps in waypoints] == [2, 1, 1, 2, 1, 1]
    z_values = [float(pose[2, 3]) for pose, _ in waypoints]
    assert z_values[0] == pytest.approx(0.33)
    assert min(z_values) == pytest.approx(0.27)
    assert z_values[-1] == pytest.approx(0.33)
    assert z_values[:3] == sorted(z_values[:3], reverse=True)
    assert z_values[3:] == sorted(z_values[3:])


def test_cli_exposes_position_tracking_substeps_and_contact_clearance():
    module = _load_module()

    args = module.parse_args(["--position-substeps", "6", "--contact-clearance-mm", "30"])

    assert args.position_substeps == 6
    assert args.contact_clearance_mm == 30.0


def test_cli_exposes_sdk_planning_trajectory_mode():
    module = _load_module()

    args = module.parse_args(["--trajectory-mode", "sdk-planning"])

    assert args.trajectory_mode == "sdk-planning"


def test_sdk_planning_targets_call_movla_and_keep_pose_trace():
    module = _load_module()

    class FakeKine:
        def __init__(self):
            self.calls = []

        def mat4x4_to_xyzabc(self, pose):
            matrix = np.asarray(pose, dtype=float).reshape(4, 4)
            return [float(matrix[0, 3]), float(matrix[1, 3]), float(matrix[2, 3]), 0.0, 0.0, 0.0]

        def movLA(self, start_xyzabc, end_xyzabc, ref_joints, vel, acc, freq_hz):
            self.calls.append(
                {
                    "start_xyzabc": list(start_xyzabc),
                    "end_xyzabc": list(end_xyzabc),
                    "ref_joints": list(ref_joints),
                    "vel": vel,
                    "acc": acc,
                    "freq_hz": freq_hz,
                }
            )
            return [[1, 2, 3, 4, 5, 6, 7], [2, 3, 4, 5, 6, 7, 8]], object()

    start_pose = np.eye(4)
    start_pose[:3, 3] = np.array((400.0, 200.0, 330.0), dtype=float)
    end_pose = np.eye(4)
    end_pose[:3, 3] = np.array((400.0, 200.0, 270.0), dtype=float)

    commands = module.build_sdk_planning_joint_targets(
        kine=FakeKine(),
        pose_commands=[(start_pose, 4), (end_pose, 4)],
        start_joints=[0, 0, 0, 0, 0, 0, 0],
        control_hz=50,
        hold_s=1.0,
        dz_mm=-60.0,
    )

    assert len(commands) == 2
    assert commands[0].joints_deg == [1, 2, 3, 4, 5, 6, 7]
    assert commands[1].joints_deg == [2, 3, 4, 5, 6, 7, 8]
    np.testing.assert_allclose(commands[0].target_pose_m[:3, 3], np.array((0.4, 0.2, 0.33)))
    np.testing.assert_allclose(commands[1].target_pose_m[:3, 3], np.array((0.4, 0.2, 0.27)))
    assert all(command.hold_steps == 1 for command in commands)


def test_sdk_planning_targets_fail_when_movla_has_no_pset():
    module = _load_module()

    class FakeKine:
        def mat4x4_to_xyzabc(self, pose):
            return [0, 0, 0, 0, 0, 0]

        def movLA(self, start_xyzabc, end_xyzabc, ref_joints, vel, acc, freq_hz):
            return [], None

    pose = np.eye(4)

    with pytest.raises(RuntimeError, match="SDK planning movLA failed"):
        module.build_sdk_planning_joint_targets(
            kine=FakeKine(),
            pose_commands=[(pose, 1)],
            start_joints=[0, 0, 0, 0, 0, 0, 0],
            control_hz=50,
            hold_s=1.0,
            dz_mm=-20.0,
        )


def test_append_viewer_trace_marker_reports_success(monkeypatch):
    module = _load_module()

    class FakeLock:
        def __enter__(self):
            return None

        def __exit__(self, exc_type, exc, tb):
            return False

    class FakeScene:
        maxgeom = 2

        def __init__(self):
            self.ngeom = 0
            self.geoms = [object(), object()]

    class FakeViewer:
        def __init__(self):
            self.user_scn = FakeScene()

        def lock(self):
            return FakeLock()

    calls = []
    monkeypatch.setattr(module, "_init_sphere_geom", lambda *args: calls.append(args))

    viewer = FakeViewer()

    assert module._append_viewer_trace_marker(viewer, np.array((1.0, 2.0, 3.0)), (1, 0, 0, 1), 0.01) is True
    assert viewer.user_scn.ngeom == 1
    assert len(calls) == 1


def test_apply_position_gain_scaling_updates_controller_params():
    module = _load_module()

    class FakeController:
        def __init__(self):
            self.params = None
            self._rate_limit = 1500.0

        def set_joint_impedance_params(self, params):
            self.params = params

    class FakeRobot:
        def __init__(self):
            self._controller = FakeController()

    fake = FakeRobot()

    params = module.apply_position_gain_scaling(fake, k_scale=2.0, d_scale=3.0, torque_rate_limit=9000.0)

    assert fake._controller.params is params
    assert fake._controller._rate_limit == 9000.0
    assert params.stiffness == pytest.approx((70.0, 70.0, 60.0, 48.0, 32.0, 20.0, 16.0))
    assert params.damping == pytest.approx((21.0, 21.0, 18.0, 15.0, 10.5, 7.5, 6.0))


def test_cli_exposes_position_gain_scales():
    module = _load_module()

    args = module.parse_args(["--position-k-scale", "4", "--position-d-scale", "2", "--torque-rate-limit", "12000"])

    assert args.position_k_scale == 4.0
    assert args.position_d_scale == 2.0
    assert args.torque_rate_limit == 12000.0


def test_right_wrist_joint_force_ranges_support_position_tracking():
    import xml.etree.ElementTree as ET

    scene = Path(__file__).resolve().parents[1] / "src" / "twin_description" / "twin_description" / "assets" / "robot" / "mujoco" / "right_chopping_scene.xml"
    root = ET.parse(scene).getroot()

    for name in ("right_joint5", "right_joint6", "right_joint7"):
        joint = root.find(f".//joint[@name='{name}']")
        assert joint is not None
        assert joint.attrib["actuatorfrcrange"] == "-200 200"


def test_joint_error_helpers_gate_settle_behavior():
    module = _load_module()

    cmd = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
    actual_close = [0.5, 9.0, 19.5, 31.0, 39.0, 50.5, 59.5]
    actual_far = [0.5, 9.0, 19.5, 31.0, 39.0, 45.0, 59.5]

    assert module.max_abs_joint_error_deg(cmd, actual_close) == pytest.approx(1.0)
    assert module.needs_joint_settle(cmd, actual_close, tolerance_deg=2.0) is False
    assert module.needs_joint_settle(cmd, actual_far, tolerance_deg=2.0) is True


def test_cli_exposes_settle_parameters():
    module = _load_module()

    args = module.parse_args(["--settle-tolerance-deg", "2.5", "--settle-timeout-s", "0.4"])

    assert args.settle_tolerance_deg == 2.5
    assert args.settle_timeout_s == 0.4


def test_chopping_scene_excludes_robot_self_collisions_that_block_wrist():
    import xml.etree.ElementTree as ET

    scene = Path(__file__).resolve().parents[1] / "src" / "twin_description" / "twin_description" / "assets" / "robot" / "mujoco" / "right_chopping_scene.xml"
    root = ET.parse(scene).getroot()
    excludes = {
        (item.attrib["body1"], item.attrib["body2"])
        for item in root.findall(".//contact/exclude")
    }

    assert ("right_link5", "right_link7") in excludes
    assert ("robot_base", "right_link1") in excludes


def test_default_config_is_mujoco_position_mode():
    module = _load_module()

    args = module.parse_args([])

    assert args.backend == "mujoco"
    assert args.arm == "B"
    assert args.control_hz == 250.0
    assert args.dz_mm == -20.0
    assert args.execution_mode == "POSITION_IK_OSCILLATION"
    assert args.position_substeps == 20
    assert args.trajectory_mode == "waypoint"
    assert args.waypoints_per_segment == 8
    assert args.position_k_scale == 8.0
    assert args.position_d_scale == 5.0
    assert args.settle_tolerance_deg == 5.0
    assert args.settle_timeout_s == 0.15
    assert args.torque_rate_limit == 2000.0
