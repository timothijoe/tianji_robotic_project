"""Tests for twin_control.robot — TwinRobot lifecycle, viewer, and trail."""

import mujoco.viewer
import numpy as np

from twin_control.controller import ControlMode
from twin_control.robot import RobotState, TwinRobot
from twin_control.trail import TcpTrail


def test_connect_viewer_uses_supported_passive_viewer(monkeypatch):
    launches = []
    closes = []
    sleeps = []

    class FakeViewer:
        def sync(self):
            pass

        def close(self):
            closes.append("close")

    def launch_passive(model, data):
        launches.append((model.njnt, data.time))
        return FakeViewer()

    monkeypatch.setattr(mujoco.viewer, "launch_passive", launch_passive)
    monkeypatch.setattr("twin_control.robot.time.sleep", lambda delay: sleeps.append(delay))

    robot = TwinRobot()
    try:
        robot.connect(viewer=True, realtime=False)
    finally:
        robot.close()

    assert launches
    assert closes == ["close"]
    assert sleeps == [0.5]


def test_trail_renders_actual_and_target_markers_under_viewer_lock():
    """TcpTrail correctly appends actual + target markers to the viewer scene."""
    events = []

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
            events.append("lock")

        def __exit__(self, exc_type, exc, traceback):
            events.append("unlock")

    class FakeViewer:
        def __init__(self):
            self.user_scn = FakeScene()

        def lock(self):
            return FakeLock()

    viewer = FakeViewer()
    trail = TcpTrail(viewer)
    trail._actual = [np.array([0.1, 0.2, 0.3])]
    trail._target = [np.array([0.4, 0.5, 0.6])]

    trail.render()

    scn = viewer.user_scn
    assert scn.ngeom == 2
    np.testing.assert_allclose(scn.geoms[0].pos, [0.1, 0.2, 0.3])
    np.testing.assert_allclose(scn.geoms[1].pos, [0.4, 0.5, 0.6])
    assert not np.allclose(scn.geoms[0].rgba, scn.geoms[1].rgba)
    assert events == ["lock", "unlock"]


def test_trail_clear_resets_scene_under_lock():
    """TcpTrail.clear() resets all state and zeros the viewer scene."""
    events = []

    class FakeScene:
        def __init__(self):
            self.ngeom = 3

    class FakeLock:
        def __enter__(self):
            events.append("lock")

        def __exit__(self, exc_type, exc, traceback):
            events.append("unlock")

    class FakeViewer:
        def __init__(self):
            self.user_scn = FakeScene()

        def lock(self):
            return FakeLock()

    viewer = FakeViewer()
    trail = TcpTrail(viewer)
    trail._actual = [np.array([0.1, 0.2, 0.3])]
    trail._target = [np.array([0.4, 0.5, 0.6])]
    trail._actual_rendered = 1
    trail._target_rendered = 1
    trail._step_count = 20

    trail.clear()

    assert trail._actual == []
    assert trail._target == []
    assert trail._actual_rendered == 0
    assert trail._target_rendered == 0
    assert trail._step_count == 0
    assert viewer.user_scn.ngeom == 0
    assert events == ["lock", "unlock"]


def test_trail_resolve_joint_space_target_uses_fk():
    """TcpTrail._resolve_target uses FK for joint-impedance mode."""
    class FakeController:
        _cart_target_matrix = None
        _joint_target_rad = np.arange(7, dtype=float)
        _mode = ControlMode.JOINT_IMPEDANCE

    class FakeKinematics:
        def fk(self, joints):
            transform = np.eye(4)
            transform[:3, 3] = (joints[0], joints[1], joints[2])
            return transform, np.zeros(6)

    trail = TcpTrail(None)
    result = trail._resolve_target(FakeKinematics(), FakeController())
    np.testing.assert_allclose(result, [0.0, 1.0, 2.0])
