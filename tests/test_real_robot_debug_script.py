import importlib

import numpy as np
import pytest


def test_default_config_matches_left_arm_dry_run_safety_defaults():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")

    config = module.MotionConfig()

    assert config.arm == "A"
    assert config.control_hz == 250.0
    assert config.dz_mm == -20.0
    assert config.hold_s == 2.0
    assert config.cycles == 5
    assert config.lateral is True
    assert config.lateral_mm == 10.0
    assert config.execute is False


def test_left_arm_joints_mirror_odd_numbered_joints_from_right_arm():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")

    mirrored = module.mirror_right_to_left_joints((1.0, 2.0, -3.0, -4.0, 5.0, 6.0, -7.0))

    assert mirrored == (-1.0, 2.0, 3.0, -4.0, -5.0, 6.0, 7.0)


def test_cut_cycle_progress_descends_then_retracts():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")

    assert module.cut_cycle_progress(0, 10, lateral_phase="retract") == (0.0, 0.0, 0.0)
    assert module.cut_cycle_progress(4, 10, lateral_phase="retract") == (1.0, 0.0, 0.0)
    assert module.cut_cycle_progress(5, 10, lateral_phase="retract") == (1.0, 0.0, 0.0)
    assert module.cut_cycle_progress(9, 10, lateral_phase="retract") == (0.0, 1.0, 1.0)


def test_cut_cycle_progress_separate_shift_has_vertical_then_horizontal_phases():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")

    assert module.cut_cycle_progress(0, 9, lateral_phase="separate") == (0.0, 0.0, 0.0)
    assert module.cut_cycle_progress(2, 9, lateral_phase="separate") == (1.0, 0.0, 0.0)
    assert module.cut_cycle_progress(5, 9, lateral_phase="separate") == (0.0, 1.0, 0.0)
    assert module.cut_cycle_progress(8, 9, lateral_phase="separate") == (0.0, 1.0, 1.0)


def test_build_relative_targets_applies_vertical_and_lateral_motion():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")
    config = module.MotionConfig(control_hz=2.0, hold_s=2.0, cycles=2, dz_mm=-20.0, lateral=True, lateral_mm=10.0)
    start_pose = np.array((100.0, 200.0, 300.0, 1.0, 2.0, 3.0), dtype=float)

    targets = module.build_relative_targets(start_pose, config)

    assert len(targets) == 8
    np.testing.assert_allclose(targets[0], start_pose)
    assert max(target[1] for target in targets) == 220.0
    assert targets[0][0] == 100.0
    assert targets[2][0] == 100.0
    assert targets[3][0] == 110.0
    assert targets[4][0] == 110.0
    assert targets[-1][0] == 120.0
    assert all(target[2] == start_pose[2] for target in targets)
    assert all(np.allclose(target[3:], start_pose[3:]) for target in targets)


def test_validate_config_rejects_unsafe_values():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")

    with pytest.raises(ValueError, match="arm must be 'A' or 'B'"):
        module.validate_config(module.MotionConfig(arm="C"))
    with pytest.raises(ValueError, match="control_hz must be positive"):
        module.validate_config(module.MotionConfig(control_hz=0.0))
    with pytest.raises(ValueError, match="cycles must be positive"):
        module.validate_config(module.MotionConfig(cycles=0))
    with pytest.raises(ValueError, match="dz-mm magnitude must be <= 80"):
        module.validate_config(module.MotionConfig(dz_mm=-100.0))
    with pytest.raises(ValueError, match="lateral-mm magnitude must be <= 50"):
        module.validate_config(module.MotionConfig(lateral_mm=60.0))


def test_parse_args_requires_execute_to_disable_dry_run():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")

    dry = module.parse_args([])
    execute = module.parse_args(["--execute"])

    assert dry.execute is False
    assert execute.execute is True


def test_real_ik_config_has_default_init_joints_for_left_arm():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")

    config = module.MotionConfig()

    assert config.arm == "A"
    assert config.init_joints == (0.0, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0)
    assert config.command_mode == "pln-cart"


def test_real_ik_parse_joints_accepts_tuple():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")

    joints = module.parse_joints("0,0,0,-5,0,0,0")

    assert joints == (0.0, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0)


def test_real_ik_parse_args_supports_init_joints_and_command_mode():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")

    config = module.parse_args(["--init-joints", "0,1,2,3,4,5,6", "--command-mode", "cart-impedance"])

    assert config.init_joints == (0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    assert config.command_mode == "cart-impedance"


def test_real_ik_parse_args_supports_planned_trajectory_printing():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")

    config = module.parse_args(["--print-trajectory", "--trajectory-stride", "3"])

    assert config.print_trajectory is True
    assert config.trajectory_stride == 3


def test_real_ik_parse_args_supports_feedback_printing():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")

    config = module.parse_args(["--print-feedback", "--feedback-stride", "4"])

    assert config.print_feedback is True
    assert config.feedback_stride == 4


def test_real_ik_parse_args_supports_force_feedback_printing():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")

    config = module.parse_args(["--print-force-feedback", "--force-feedback-stride", "5"])

    assert config.force_feedback is True
    assert config.print_force_feedback is True
    assert config.force_feedback_stride == 5


def test_real_ik_parse_args_supports_joint_impedance_mode_and_gains():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")

    config = module.parse_args([
        "--command-mode",
        "joint-impedance",
        "--joint-k",
        "1,2,3,4,5,6,7",
        "--joint-d",
        "0.1,0.2,0.3,0.4,0.5,0.6,0.7",
    ])

    assert config.command_mode == "joint-impedance"
    assert config.joint_k == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0)
    assert config.joint_d == (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7)


def test_configure_joint_impedance_uses_sdk_impedance_mode():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")

    class Robot:
        def __init__(self):
            self.calls = []

        def clear_set(self):
            self.calls.append(("clear_set",))

        def set_imp_joint_state(self, arm, velRatio, AccRatio, K, D):
            self.calls.append(("set_imp_joint_state", arm, velRatio, AccRatio, tuple(K), tuple(D)))
            return True

        def send_cmd_wait_response(self, timeout):
            self.calls.append(("send_cmd_wait_response", timeout))
            return 0

    robot = Robot()
    config = module.MotionConfig(
        command_mode="joint-impedance",
        vel_ratio=11,
        acc_ratio=12,
        joint_k=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0),
        joint_d=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7),
    )

    module._configure_joint_impedance(robot, config)

    assert robot.calls == [
        ("clear_set",),
        ("set_imp_joint_state", "A", 11, 12, (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0), (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7)),
        ("send_cmd_wait_response", 100),
    ]


def test_configure_joint_impedance_supports_legacy_sdk_methods():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")

    class Robot:
        def __init__(self):
            self.calls = []

        def clear_set(self):
            self.calls.append(("clear_set",))

        def set_state(self, arm, state):
            self.calls.append(("set_state", arm, state))

        def set_impedance_type(self, arm, type):
            self.calls.append(("set_impedance_type", arm, type))

        def set_vel_acc(self, arm, velRatio, AccRatio):
            self.calls.append(("set_vel_acc", arm, velRatio, AccRatio))

        def set_joint_kd_params(self, arm, K, D):
            self.calls.append(("set_joint_kd_params", arm, tuple(K), tuple(D)))

        def send_cmd_wait_response(self, timeout):
            self.calls.append(("send_cmd_wait_response", timeout))
            return 0

    robot = Robot()
    config = module.MotionConfig(
        command_mode="joint-impedance",
        vel_ratio=11,
        acc_ratio=12,
        joint_k=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0),
        joint_d=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7),
    )

    module._configure_joint_impedance(robot, config)

    assert robot.calls == [
        ("clear_set",),
        ("set_state", "A", 3),
        ("set_impedance_type", "A", 1),
        ("set_vel_acc", "A", 11, 12),
        ("send_cmd_wait_response", 100),
        ("clear_set",),
        ("set_joint_kd_params", "A", (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0), (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7)),
        ("send_cmd_wait_response", 100),
    ]


def test_send_sampled_joint_command_uses_joint_impedance_sdk_command():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")

    class Robot:
        def __init__(self):
            self.calls = []

        def clear_set(self):
            self.calls.append(("clear_set",))

        def set_joint_position_cmd(self, arm, joints):
            self.calls.append(("set_joint_position_cmd", arm, tuple(joints)))

        def set_joint_cmd_pose(self, arm, joints):
            self.calls.append(("set_joint_cmd_pose", arm, tuple(joints)))

        def send_cmd(self):
            self.calls.append(("send_cmd",))

    robot = Robot()
    config = module.MotionConfig(command_mode="joint-impedance")

    module._send_sampled_joint_command(robot, config, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0])

    assert robot.calls == [
        ("clear_set",),
        ("set_joint_position_cmd", "A", (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0)),
        ("send_cmd",),
    ]


def test_send_sampled_joint_command_falls_back_to_legacy_joint_command():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")

    class Robot:
        def __init__(self):
            self.calls = []

        def clear_set(self):
            self.calls.append(("clear_set",))

        def set_joint_cmd_pose(self, arm, joints):
            self.calls.append(("set_joint_cmd_pose", arm, tuple(joints)))

        def send_cmd(self):
            self.calls.append(("send_cmd",))

    robot = Robot()
    config = module.MotionConfig(command_mode="joint-impedance")

    module._send_sampled_joint_command(robot, config, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0])

    assert robot.calls == [
        ("clear_set",),
        ("set_joint_cmd_pose", "A", (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0)),
        ("send_cmd",),
    ]


def test_joint_impedance_entrypoint_forces_joint_impedance_mode(monkeypatch):
    module = importlib.import_module("real_robot_debug.real_sampled_joint_impedance_chop")
    captured = {}

    def fake_main(argv):
        captured["argv"] = argv
        return 0

    monkeypatch.setattr(module.impl, "main", fake_main)

    assert module.main(["--robot-ip", "192.168.1.190"]) == 0
    assert captured["argv"] == ["--robot-ip", "192.168.1.190", "--command-mode", "joint-impedance"]


def test_planned_trajectory_rows_include_cartesian_pose_and_joints():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")

    class Kine:
        def fk(self, joints):
            return list(joints)

        def mat4x4_to_xyzabc(self, pose_mat):
            return [value + 100.0 for value in pose_mat[:6]]

    segment = module.PlannedSegment(
        "descend",
        np.array((1.0, 2.0, 3.0, 4.0, 5.0, 6.0), dtype=float),
        np.array((1.0, 2.0, -7.0, 4.0, 5.0, 6.0), dtype=float),
    )

    rows = module._planned_trajectory_rows(
        Kine(),
        segment_index=2,
        segment=segment,
        points=[
            [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0],
            [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0],
        ],
        stride=1,
    )

    assert rows[0]["segment"] == 2
    assert rows[0]["label"] == "descend"
    assert rows[0]["point"] == 0
    assert rows[0]["x"] == 101.0
    assert rows[0]["z"] == 103.0
    assert rows[0]["q0"] == 1.0
    assert rows[0]["q6"] == 7.0
    assert rows[1]["point"] == 1
    assert rows[1]["x"] == 110.0
    assert rows[1]["q6"] == 70.0


def test_offline_sampled_chop_defaults_match_requested_motion():
    module = importlib.import_module("real_robot_debug.plan_sampled_ik_chop")

    config = module.parse_args([])

    assert config.dz_mm == -40.0
    assert config.cycles == 30
    assert config.lateral is True
    assert config.lateral_mm == 40.0
    assert config.chop_axis == "y"
    assert config.lateral_axis == "x"
    assert config.init_joints == (109.81, -62.66, -95.69, -93.79, 63.32, -2.76, 12.42)


def test_offline_sampled_cartesian_targets_match_mujoco_shape():
    module = importlib.import_module("real_robot_debug.plan_sampled_ik_chop")
    config = module.PlanConfig(control_hz=2.0, hold_s=2.0, cycles=2, dz_mm=-40.0, lateral=True, lateral_mm=40.0)
    start_xyzabc = np.array((100.0, 200.0, 300.0, 1.0, 2.0, 3.0), dtype=float)

    rows = module.build_target_rows(start_xyzabc, config)

    assert len(rows) == 8
    assert rows[0]["x"] == 100.0
    assert rows[0]["y"] == 200.0
    assert rows[0]["z"] == 300.0
    assert max(row["y"] for row in rows) == 240.0
    assert rows[3]["x"] == 140.0
    assert rows[-1]["x"] == 180.0
    assert rows[-1]["y"] == 200.0
    assert rows[-1]["z"] == 300.0


def test_offline_sampled_ik_false_result_keeps_reference_joints():
    module = importlib.import_module("real_robot_debug.plan_sampled_ik_chop")

    success, joints = module._read_ik_result(False, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0])

    assert success is False
    assert joints == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]


def test_axis_indices_match_real_robot_frame():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")

    assert module.axis_index("x") == 0
    assert module.axis_index("y") == 1
    assert module.axis_index("z") == 2


def test_build_chop_segments_repeats_descend_and_retract_without_lateral():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")
    config = module.MotionConfig(cycles=2, dz_mm=-5.0, lateral=False)
    start_pose = np.array((100.0, 200.0, 300.0, 1.0, 2.0, 3.0), dtype=float)

    segments = module.build_chop_segments(start_pose, config)

    assert [segment.label for segment in segments] == ["descend", "retract", "descend", "retract"]
    assert segments[0].start_xyzabc[1] == 200.0
    assert segments[0].end_xyzabc[1] == 205.0
    assert segments[1].start_xyzabc[1] == 205.0
    assert segments[1].end_xyzabc[1] == 200.0
    assert segments[-1].end_xyzabc[0] == 100.0


def test_build_chop_segments_adds_lateral_shift_between_cycles():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")
    config = module.MotionConfig(cycles=2, dz_mm=-5.0, lateral=True, lateral_mm=10.0)
    start_pose = np.array((100.0, 200.0, 300.0, 1.0, 2.0, 3.0), dtype=float)

    segments = module.build_chop_segments(start_pose, config)

    assert [segment.label for segment in segments] == ["descend", "retract", "shift", "descend", "retract"]
    assert segments[2].start_xyzabc[0] == 100.0
    assert segments[2].end_xyzabc[0] == 110.0
    assert segments[3].start_xyzabc[0] == 110.0
    assert segments[4].end_xyzabc[0] == 110.0

def test_position_mode_entrypoint_forces_pln_cart_mode(monkeypatch):
    module = importlib.import_module("real_robot_debug.real_pln_cart_position_chop")
    captured = {}

    def fake_main(argv):
        captured["argv"] = argv
        return 0

    monkeypatch.setattr(module.impl, "main", fake_main)

    assert module.main(["--robot-ip", "192.168.1.190"]) == 0
    assert captured["argv"] == ["--robot-ip", "192.168.1.190", "--command-mode", "pln-cart"]

def test_sampled_position_entrypoint_forces_position_mode(monkeypatch):
    module = importlib.import_module("real_robot_debug.real_sampled_position_chop")
    captured = {}

    def fake_main(argv):
        captured["argv"] = argv
        return 0

    monkeypatch.setattr(module.impl, "main", fake_main)

    assert module.main(["--robot-ip", "192.168.1.190"]) == 0
    assert captured["argv"] == ["--robot-ip", "192.168.1.190", "--command-mode", "position"]


def test_configure_force_feedback_sets_user_specified_6ft_channel():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")

    class Robot:
        def __init__(self):
            self.calls = []

        def set_user_specified_data(self, arm, command):
            self.calls.append(("set_user_specified_data", arm, command))
            return True

    robot = Robot()

    assert module._configure_force_feedback(robot, module.MotionConfig(arm="A")) is True
    assert robot.calls == [("set_user_specified_data", "A", 116)]


def test_force_feedback_from_feedback_reads_est_joint_firc_dot():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")
    feedback = {"outputs": [{"est_joint_firc_dot": [1, 2, 3, 4, 5, 6, 116]}]}

    force = module._force_feedback_from_feedback(feedback, 0)

    assert force == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)


def test_trace_row_includes_timestamp_joint_velocity_and_torque():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")

    row = module._trace_row(
        step_index=3,
        target_xyzabc=np.array((10.0, 20.0, 30.0, 1.0, 2.0, 3.0), dtype=float),
        actual_xyzabc=np.array((11.0, 21.0, 31.0, 1.5, 2.5, 3.5), dtype=float),
        target_joints=[10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0],
        actual_joints=[11.0, 21.0, 31.0, 41.0, 51.0, 61.0, 71.0],
        ik_success=True,
        timestamp_s=123.456,
        elapsed_s=1.25,
        actual_joint_velocities=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
        actual_joint_torques=[1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7],
    )

    assert row["timestamp_s"] == 123.456
    assert row["elapsed_s"] == 1.25
    assert row["target_x"] == 10.0
    assert row["actual_x"] == 11.0
    assert row["actual_q_6"] == 71.0
    assert row["actual_qd_0"] == 0.1
    assert row["actual_qd_6"] == 0.7
    assert row["actual_tau_0"] == 1.1
    assert row["actual_tau_6"] == 1.7


def test_trace_row_includes_force_feedback_when_available():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")

    row = module._trace_row(
        step_index=1,
        target_xyzabc=np.zeros(6),
        actual_xyzabc=np.ones(6),
        target_joints=[0.0] * 7,
        actual_joints=[1.0] * 7,
        ik_success=True,
        force_feedback=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0),
    )

    assert row["force_fx"] == 1.0
    assert row["force_fy"] == 2.0
    assert row["force_fz"] == 3.0
    assert row["torque_tx"] == 4.0
    assert row["torque_ty"] == 5.0
    assert row["torque_tz"] == 6.0


def test_print_force_feedback_row(capsys):
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")

    module._print_force_feedback_row(7, (1.0, 2.0, 3.0, 4.0, 5.0, 6.0))

    assert capsys.readouterr().out.strip() == (
        "wrench_6ft: step=7,fx=1.000000,fy=2.000000,fz=3.000000,"
        "tx=4.000000,ty=5.000000,tz=6.000000"
    )


def test_sampled_trajectory_row_includes_cartesian_pose_and_joints():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")

    row = module._sampled_trajectory_row(
        step_index=7,
        target_xyzabc=np.array((1.0, 2.0, -3.0, 4.0, 5.0, 6.0), dtype=float),
        target_joints=[10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0],
        ik_success=True,
    )

    assert row["step"] == 7
    assert row["ik_success"] is True
    assert row["x"] == 1.0
    assert row["z"] == -3.0
    assert row["q0"] == 10.0
    assert row["q6"] == 70.0

def test_pln_cart_position_setup_prefers_wait_response():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")

    class Robot:
        def __init__(self):
            self.calls = []

        def clear_set(self):
            self.calls.append(("clear_set",))

        def set_vel_acc(self, arm, velRatio, AccRatio):
            self.calls.append(("set_vel_acc", arm, velRatio, AccRatio))

        def set_state(self, arm, state):
            self.calls.append(("set_state", arm, state))

        def send_cmd_wait_response(self, timeout):
            self.calls.append(("send_cmd_wait_response", timeout))
            return 0

        def send_cmd(self):
            self.calls.append(("send_cmd",))

    robot = Robot()

    module._set_position_mode(robot, module.MotionConfig())

    assert ("set_state", "A", 1) in robot.calls
    assert ("send_cmd_wait_response", 100) in robot.calls
    assert ("send_cmd",) not in robot.calls


def test_pln_cart_position_setup_requires_planning_state_feedback():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")

    class Robot:
        def __init__(self):
            self.calls = []

        def clear_set(self):
            self.calls.append(("clear_set",))

        def set_vel_acc(self, arm, velRatio, AccRatio):
            self.calls.append(("set_vel_acc", arm, velRatio, AccRatio))

        def set_state(self, arm, state):
            self.calls.append(("set_state", arm, state))

        def send_cmd_wait_response(self, timeout):
            self.calls.append(("send_cmd_wait_response", timeout))
            return 0

        def subscribe(self, dcss):
            return {
                "states": [{"cur_state": 3, "cmd_state": 1, "err_code": 0}],
                "outputs": [{"traj_state": b"\x00"}],
            }

    with pytest.raises(RuntimeError, match="did not enter position planning mode.*cur_state=3.*cmd_state=1"):
        module._set_pln_cart_position_mode(Robot(), module.MotionConfig(), object(), 0)


def test_initialization_failure_message_contains_diagnostics():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")
    message = module._initialization_failure_message(
        target=[0.0, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0],
        final=[0.0, 0.0, 0.0, -0.1, 0.0, 0.0, 0.0],
        feedback={"states": [{"cur_state": 1, "err_code": 0}], "outputs": [{"traj_state": b"\x00"}]},
        arm_index=0,
    )

    assert "initialization target was not reached" in message
    assert "max_error_deg=4.900" in message
    assert "cur_state=1" in message
    assert "traj_state" in message
    assert "final_joints=[0.0, 0.0, 0.0, -0.1" in message

