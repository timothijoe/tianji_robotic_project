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

