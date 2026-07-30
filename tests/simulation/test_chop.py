import csv
import itertools
from statistics import median

import mujoco
import numpy as np
import pytest

import twin_sim.tasks.chop as chop
from twin_sim.kinematics import Kinematics
from twin_sim.robot import RightArmRobot
from twin_sim.tasks.chop import ChopConfig, _preflight, run_chop


def test_one_chop_completes_and_logs_all_phases(tmp_path):
    path = tmp_path / "chop.csv"

    result = run_chop(ChopConfig(control_dt_s=0.01), log_path=path)

    assert result.completed
    phases = {row["phase"] for row in csv.DictReader(path.open())}
    assert phases >= {"APPROACH", "DESCEND", "HOLD", "RETRACT", "COMPLETE"}
    assert result.final_tip_z_m >= result.safe_tip_z_m - 0.005


def test_force_warning_does_not_cancel_task(tmp_path):
    with pytest.warns(RuntimeWarning):
        result = run_chop(
            ChopConfig(control_dt_s=0.01, force_warning_threshold_n=0.0),
            log_path=tmp_path / "warning.csv",
        )

    assert result.completed
    assert result.warning_count > 0


def test_chop_force_has_contact_and_lift_trend(tmp_path):
    result = run_chop(ChopConfig(), log_path=tmp_path / "trend.csv")

    approach = [sample.filtered_force_n for sample in result.samples if sample.phase == "APPROACH"]
    hold = [sample.filtered_force_n for sample in result.samples if sample.phase == "HOLD"]
    retract = [sample.filtered_force_n for sample in result.samples if sample.phase == "RETRACT"]
    baseline = median(approach)
    contact = median(hold)
    lifted = median(retract[len(retract) // 2 :])
    assert contact > baseline
    assert lifted < contact


def test_chop_validates_actual_log_destination_before_config_or_motion(tmp_path):
    with pytest.raises(IsADirectoryError):
        run_chop(ChopConfig(control_dt_s=0.0), log_path=tmp_path)


def test_contact_target_limits_blade_penetration():
    robot = RightArmRobot()
    try:
        trajectories, _ = _preflight(robot, Kinematics(robot.sim), ChopConfig())
        contact_joints = trajectories["DESCEND"][-1].joints_rad
        robot.sim.data.qpos[robot.sim.right.qpos_ids] = contact_joints
        mujoco.mj_forward(robot.sim.model, robot.sim.data)

        model, data = robot.sim.model, robot.sim.data
        board_id = robot.sim.require_geom("chopping_board")
        blade_id = robot.sim.require_geom("right_knife_blade")
        board_top_m = float(
            data.geom_xpos[board_id, 2] + model.geom_size[board_id, 2]
        )
        center = data.geom_xpos[blade_id]
        rotation = data.geom_xmat[blade_id].reshape(3, 3)
        half_size = model.geom_size[blade_id]
        corners = np.array(
            [
                center + rotation @ (np.asarray(signs) * half_size)
                for signs in itertools.product((-1.0, 1.0), repeat=3)
            ]
        )
        penetration_m = board_top_m - float(corners[:, 2].min())

        assert penetration_m <= 0.005
    finally:
        robot.close()


def test_cleaver_cutting_edge_is_horizontal_at_home():
    robot = RightArmRobot()
    try:
        data = robot.sim.data
        edge_start = data.site_xpos[
            robot.sim.require_site("right_blade_edge_top")
        ]
        edge_end = data.site_xpos[
            robot.sim.require_site("right_blade_edge_bot")
        ]
        edge_direction = edge_end - edge_start
        edge_direction /= np.linalg.norm(edge_direction)

        assert abs(float(edge_direction[2])) <= np.sin(np.deg2rad(2.0))
    finally:
        robot.close()


def test_viewer_presentation_sets_camera_and_waits(monkeypatch):
    class FakeCamera:
        azimuth = 0.0
        elevation = 0.0
        distance = 0.0
        lookat = np.zeros(3)

    class FakeViewer:
        cam = FakeCamera()

        @staticmethod
        def is_running():
            return True

        @staticmethod
        def sync():
            return None

    class FakeRobot:
        _viewer = FakeViewer()

    sleeps = []
    monkeypatch.setattr("twin_sim.tasks.chop.time.sleep", sleeps.append)
    config = ChopConfig(viewer_start_hold_s=5.0, viewer_end_hold_s=8.0)

    chop._prepare_viewer_presentation(FakeRobot(), config)
    chop._finish_viewer_presentation(FakeRobot(), config)

    assert sleeps == [5.0, 8.0]
    assert FakeViewer.cam.azimuth != 0.0
    assert FakeViewer.cam.elevation != 0.0
