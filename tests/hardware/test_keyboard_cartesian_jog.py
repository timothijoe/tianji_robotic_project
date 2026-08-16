from __future__ import annotations

import numpy as np
import pytest

from real_robot_debug.keyboard_cartesian_jog import (
    JogConfig,
    candidate_pose,
    inside_workspace,
    key_to_delta,
    parse_args,
    run_jog_session,
    validate_config,
)


class FakeDcss:
    pass


class FakeRobot:
    def __init__(self) -> None:
        self.planned_commands: list[tuple[str, object]] = []
        self.disabled = False
        self.released = False
        self.frame_serial = 0

    def connect(self, robot_ip: str) -> bool:
        return True

    def check_error_and_clear(self, dcss: FakeDcss) -> None:
        pass

    def subscribe(self, dcss: FakeDcss) -> dict:
        self.frame_serial += 1
        return {
            "outputs": [{"frame_serial": self.frame_serial, "fb_joint_pos": [0.0] * 7, "traj_state": 0}],
            "states": [{"cur_state": 1, "err_code": 0}],
        }

    def setPln_Cart(self, arm: str, pset: object) -> None:
        self.planned_commands.append((arm, pset))

    def set_vel_acc(self, arm: str, velRatio: int, AccRatio: int) -> None:
        pass

    def clear_set(self) -> None:
        pass

    def set_state(self, arm: str, state: int) -> None:
        self.disabled = state == 0

    def send_cmd(self) -> None:
        pass

    def release_robot(self) -> None:
        self.released = True


class FakeKine:
    def fk(self, joints: list[float]) -> np.ndarray:
        return np.asarray(joints, dtype=float)

    def mat4x4_to_xyzabc(self, matrix: np.ndarray) -> list[float]:
        return [0.0] * 6

    def movLA(self, **kwargs):
        return [[0.0] * 7], object()


class FailingKine(FakeKine):
    def movLA(self, **kwargs):
        return [], None


def test_w_requests_one_positive_x_step():
    assert key_to_delta("w", 2.0) == (2.0, 0.0, 0.0)


def test_f_requests_one_negative_z_step():
    assert key_to_delta("F", 2.0) == (0.0, 0.0, -2.0)


def test_candidate_pose_translates_xyz_and_preserves_orientation():
    current = np.array([100.0, 200.0, 300.0, 10.0, 20.0, 30.0])

    target = candidate_pose(current, (2.0, -2.0, 0.0))

    assert np.array_equal(target, np.array([102.0, 198.0, 300.0, 10.0, 20.0, 30.0]))


def test_execute_requires_both_workspace_bounds():
    with pytest.raises(ValueError, match="workspace-min.*workspace-max"):
        validate_config(JogConfig(execute=True))


def test_step_above_five_mm_is_rejected():
    with pytest.raises(ValueError, match="step-mm must be in .*5"):
        validate_config(JogConfig(step_mm=5.1))


def test_workspace_includes_edges_but_rejects_outside_point():
    lower, upper = (0.0, 0.0, 0.0), (10.0, 10.0, 10.0)

    assert inside_workspace(np.array([0.0, 10.0, 5.0]), lower, upper)
    assert not inside_workspace(np.array([10.1, 10.0, 5.0]), lower, upper)


def test_dry_run_plans_one_step_without_sending_robot_command():
    robot, dcss, kine = FakeRobot(), FakeDcss(), FakeKine()

    poses = run_jog_session(
        JogConfig(),
        read_key=iter(["w", "q"]).__next__,
        sdk_factory=lambda: (robot, dcss, kine),
    )

    assert np.array_equal(poses[-1][:3], np.array([2.0, 0.0, 0.0]))
    assert robot.planned_commands == []
    assert robot.disabled
    assert robot.released


def test_execute_outside_workspace_does_not_plan_or_send():
    robot, dcss, kine = FakeRobot(), FakeDcss(), FakeKine()
    config = JogConfig(
        execute=True,
        workspace_min=(0.0, 0.0, 0.0),
        workspace_max=(1.0, 1.0, 1.0),
    )

    with pytest.raises(ValueError, match="outside workspace"):
        run_jog_session(config, read_key=iter(["w"]).__next__, sdk_factory=lambda: (robot, dcss, kine))

    assert robot.planned_commands == []
    assert robot.disabled


def test_execute_planning_failure_does_not_send_command():
    robot, dcss, kine = FakeRobot(), FakeDcss(), FailingKine()
    config = JogConfig(
        execute=True,
        workspace_min=(-5.0, -5.0, -5.0),
        workspace_max=(5.0, 5.0, 5.0),
    )

    with pytest.raises(RuntimeError, match="MOVLA planning failed"):
        run_jog_session(config, read_key=iter(["w"]).__next__, sdk_factory=lambda: (robot, dcss, kine))

    assert robot.planned_commands == []
    assert robot.disabled


def test_execute_sends_one_planned_command_inside_workspace():
    robot, dcss, kine = FakeRobot(), FakeDcss(), FakeKine()
    config = JogConfig(
        execute=True,
        workspace_min=(-5.0, -5.0, -5.0),
        workspace_max=(5.0, 5.0, 5.0),
    )

    run_jog_session(config, read_key=iter(["w", "q"]).__next__, sdk_factory=lambda: (robot, dcss, kine))

    assert len(robot.planned_commands) == 1
    assert robot.planned_commands[0][0] == "A"


def test_parse_execute_requires_workspace_values():
    with pytest.raises(ValueError, match="workspace-min.*workspace-max"):
        parse_args(["--execute"])


def test_space_stops_before_later_motion_key():
    robot, dcss, kine = FakeRobot(), FakeDcss(), FakeKine()

    run_jog_session(
        JogConfig(),
        read_key=iter([" ", "w"]).__next__,
        sdk_factory=lambda: (robot, dcss, kine),
    )

    assert robot.disabled
    assert robot.planned_commands == []
