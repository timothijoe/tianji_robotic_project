import importlib

import numpy as np
import pytest


def test_default_config_matches_a_arm_dry_run_safety_defaults():
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


def test_cut_cycle_progress_descends_then_retracts():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")

    assert module.cut_cycle_progress(0, 10) == (0.0, 0.0)
    assert module.cut_cycle_progress(4, 10) == (1.0, 0.0)
    assert module.cut_cycle_progress(5, 10) == (1.0, 0.0)
    assert module.cut_cycle_progress(9, 10) == (0.0, 1.0)


def test_build_relative_targets_applies_vertical_and_lateral_motion():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")
    config = module.MotionConfig(control_hz=2.0, hold_s=2.0, cycles=2, dz_mm=-20.0, lateral=True, lateral_mm=10.0)
    start_pose = np.array((100.0, 200.0, 300.0, 1.0, 2.0, 3.0), dtype=float)

    targets = module.build_relative_targets(start_pose, config)

    assert len(targets) == 8
    np.testing.assert_allclose(targets[0], start_pose)
    assert min(target[2] for target in targets) == 280.0
    assert targets[0][1] == 200.0
    assert targets[3][1] == 210.0
    assert targets[4][1] == 210.0
    assert targets[-1][1] == 220.0
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


def test_real_ik_config_has_default_init_joints_for_a_arm():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")

    config = module.MotionConfig()

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


def test_build_chop_segments_repeats_descend_and_retract_without_lateral():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")
    config = module.MotionConfig(cycles=2, dz_mm=-5.0, lateral=False)
    start_pose = np.array((100.0, 200.0, 300.0, 1.0, 2.0, 3.0), dtype=float)

    segments = module.build_chop_segments(start_pose, config)

    assert [segment.label for segment in segments] == ["descend", "retract", "descend", "retract"]
    assert segments[0].start_xyzabc[2] == 300.0
    assert segments[0].end_xyzabc[2] == 295.0
    assert segments[1].start_xyzabc[2] == 295.0
    assert segments[1].end_xyzabc[2] == 300.0
    assert segments[-1].end_xyzabc[1] == 200.0


def test_build_chop_segments_adds_lateral_shift_between_cycles():
    module = importlib.import_module("real_robot_debug.real_ik_cart_impedance_lateral")
    config = module.MotionConfig(cycles=2, dz_mm=-5.0, lateral=True, lateral_mm=10.0)
    start_pose = np.array((100.0, 200.0, 300.0, 1.0, 2.0, 3.0), dtype=float)

    segments = module.build_chop_segments(start_pose, config)

    assert [segment.label for segment in segments] == ["descend", "retract", "shift", "descend", "retract"]
    assert segments[2].start_xyzabc[1] == 200.0
    assert segments[2].end_xyzabc[1] == 210.0
    assert segments[3].start_xyzabc[1] == 210.0
    assert segments[4].end_xyzabc[1] == 210.0

def test_position_mode_entrypoint_forces_pln_cart_mode(monkeypatch):
    module = importlib.import_module("real_robot_debug.real_pln_cart_position_chop")
    captured = {}

    def fake_main(argv):
        captured["argv"] = argv
        return 0

    monkeypatch.setattr(module.impl, "main", fake_main)

    assert module.main(["--robot-ip", "192.168.1.190"]) == 0
    assert captured["argv"] == ["--robot-ip", "192.168.1.190", "--command-mode", "pln-cart"]

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

