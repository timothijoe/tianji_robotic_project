import importlib

import pytest


def test_default_position_config_is_a_arm_dry_run_without_target():
    module = importlib.import_module("real_robot_debug.real_position_mode_a_arm")

    config = module.PositionConfig()

    assert config.arm == "A"
    assert config.vel_ratio == 10
    assert config.acc_ratio == 10
    assert config.execute is False
    assert config.joints is None


def test_parse_joints_accepts_seven_comma_separated_values():
    module = importlib.import_module("real_robot_debug.real_position_mode_a_arm")

    joints = module.parse_joints("0, 10, -20, 30.5, 0, 5, -6")

    assert joints == [0.0, 10.0, -20.0, 30.5, 0.0, 5.0, -6.0]


def test_parse_joints_rejects_wrong_size_and_bad_numbers():
    module = importlib.import_module("real_robot_debug.real_position_mode_a_arm")

    with pytest.raises(ValueError, match="must contain 7"):
        module.parse_joints("1,2,3")
    with pytest.raises(ValueError, match="finite"):
        module.parse_joints("1,2,3,4,5,6,nan")
    with pytest.raises(ValueError, match="comma-separated floats"):
        module.parse_joints("1,2,3,4,5,6,nope")


def test_validate_config_requires_execute_for_real_motion_target():
    module = importlib.import_module("real_robot_debug.real_position_mode_a_arm")

    module.validate_config(module.PositionConfig())
    module.validate_config(module.PositionConfig(joints=[0.0] * 7, execute=False))
    module.validate_config(module.PositionConfig(joints=[0.0] * 7, execute=True))

    with pytest.raises(ValueError, match="arm must be 'A' or 'B'"):
        module.validate_config(module.PositionConfig(arm="C"))
    with pytest.raises(ValueError, match="vel-ratio must be in \\[0, 100\\]"):
        module.validate_config(module.PositionConfig(vel_ratio=101))
    with pytest.raises(ValueError, match="timeout-s must be positive"):
        module.validate_config(module.PositionConfig(timeout_s=0.0))
    with pytest.raises(ValueError, match="joint_delta_limit_deg"):
        module.validate_config(module.PositionConfig(joint_delta_limit_deg=0.0))


def test_parse_args_maps_joints_and_execute_flag():
    module = importlib.import_module("real_robot_debug.real_position_mode_a_arm")

    config = module.parse_args(["--joints", "0,1,2,3,4,5,6", "--execute"])

    assert config.joints == [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    assert config.execute is True


def test_main_reports_config_errors_without_traceback(capsys):
    module = importlib.import_module("real_robot_debug.real_position_mode_a_arm")

    exit_code = module.main(["--joint-delta-limit-deg", "0"])

    captured = capsys.readouterr()
    assert exit_code == 2
    assert "error:" in captured.err
    assert "joint_delta_limit_deg" in captured.err

def test_position_mode_setup_sends_state_and_limits_together():
    module = importlib.import_module("real_robot_debug.real_position_mode_a_arm")

    class Robot:
        def __init__(self):
            self.calls = []

        def clear_set(self):
            self.calls.append(("clear_set",))

        def set_state(self, arm, state):
            self.calls.append(("set_state", arm, state))

        def set_vel_acc(self, arm, velRatio, AccRatio):
            self.calls.append(("set_vel_acc", arm, velRatio, AccRatio))

        def send_cmd_wait_response(self, timeout):
            self.calls.append(("send_cmd_wait_response", timeout))
            return 0

        def send_cmd(self):
            self.calls.append(("send_cmd",))

    robot = Robot()

    module._set_position_mode(robot, module.PositionConfig())

    assert robot.calls == [
        ("clear_set",),
        ("set_state", "A", 1),
        ("set_vel_acc", "A", 10, 10),
        ("send_cmd",),
    ]


def test_position_failure_message_contains_final_joints_and_error():
    module = importlib.import_module("real_robot_debug.real_position_mode_a_arm")

    message = module._target_failure_message(
        target=[0.0, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0],
        final=[0.0, 0.0, 0.0, -0.2, 0.0, 0.0, 0.0],
        feedback={"states": [{"cur_state": 1, "err_code": 0}], "outputs": [{"traj_state": b"\x00"}]},
        arm_index=0,
    )

    assert "target was not reached" in message
    assert "max_error_deg=4.800" in message
    assert "cur_state=1" in message
    assert "final_joints=[0.0, 0.0, 0.0, -0.2" in message

