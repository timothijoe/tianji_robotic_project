import numpy as np

import twin_sim.tasks.guarded_chop as guarded_chop
from twin_sim.guarded_chop_safety import CAT_PAW_RAD
from twin_sim.robot import RightArmRobot
from twin_sim.tasks.guarded_chop import GuardedChopConfig, run_guarded_chop


class _Trace:
    def __init__(self, viewer, events):
        events.append("create_trace")

    def set_plan(self, points):
        pass

    def append(self, **sample):
        pass

    def set_abort(self, reason):
        pass


def test_viewer_opens_only_after_guarded_scene_and_ready_pose(monkeypatch):
    events = []
    constructor_viewer_values = []
    original_configure = guarded_chop._configure_guarded_scene
    original_robot = RightArmRobot

    def robot_factory(*args, **kwargs):
        constructor_viewer_values.append(kwargs.get("viewer", False))
        return original_robot(*args, **kwargs)

    def configure(robot, scene_mode):
        events.append("configure")
        return original_configure(robot, scene_mode)

    def open_viewer(robot):
        for name in (
            "pick_source_pedestal",
            "pick_target_pedestal",
            "pick_cube_geom",
            "pick_target_region",
        ):
            geom = robot.sim.require_geom(name)
            assert robot.sim.model.geom_rgba[geom, 3] == 0.0
        np.testing.assert_allclose(
            robot.sim.data.qpos[robot.sim.hand.qpos_ids],
            CAT_PAW_RAD,
            atol=1e-12,
        )
        events.append("open_viewer")

    def prepare_camera(robot):
        events.append("prepare_camera")

    monkeypatch.setattr(guarded_chop, "RightArmRobot", robot_factory)
    monkeypatch.setattr(
        guarded_chop, "_configure_guarded_scene", configure
    )
    monkeypatch.setattr(RightArmRobot, "open_viewer", open_viewer)
    monkeypatch.setattr(
        guarded_chop, "_prepare_guarded_chop_viewer", prepare_camera
    )
    monkeypatch.setattr(
        "twin_sim.guarded_chop_visualization.GuardedChopTrace",
        lambda viewer: _Trace(viewer, events),
    )

    result = run_guarded_chop(
        GuardedChopConfig(final_hold_s=0.0), viewer=True
    )

    assert result.success, result.reason
    assert constructor_viewer_values == [False]
    assert events.index("configure") < events.index("open_viewer")
    assert events.index("open_viewer") < events.index("prepare_camera")
    assert events.index("prepare_camera") < events.index("create_trace")
