import mujoco.viewer
import numpy as np

from twin_control.robot import RobotState, TwinRobot


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


def test_render_trail_draws_actual_and_target_markers_under_viewer_lock():
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
            self.maxgeom = 2
            self.geoms = [FakeGeom(), FakeGeom()]

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

    robot = TwinRobot()
    robot._viewer = FakeViewer()
    robot._tcp_trail.append(np.array([0.1, 0.2, 0.3]))
    robot._target_trail.append(np.array([0.4, 0.5, 0.6]))

    robot._render_trail()

    actual, target = robot._viewer.user_scn.geoms
    np.testing.assert_allclose(actual.mat, np.eye(3))
    np.testing.assert_allclose(actual.pos, [0.1, 0.2, 0.3])
    np.testing.assert_allclose(target.pos, [0.4, 0.5, 0.6])
    assert not np.allclose(actual.rgba, target.rgba)
    assert robot._viewer.user_scn.ngeom == 2
    assert events == ["lock", "unlock"]


def test_clear_trail_resets_viewer_scene_under_lock():
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

    robot = TwinRobot()
    robot._viewer = FakeViewer()
    robot._tcp_trail.append(np.array([0.1, 0.2, 0.3]))
    robot._target_trail.append(np.array([0.4, 0.5, 0.6]))
    robot._trail_rendered_count = 1
    robot._target_trail_rendered_count = 1
    robot._trail_step_count = 20

    robot.clear_trail()

    assert robot._tcp_trail == []
    assert robot._target_trail == []
    assert robot._trail_rendered_count == 0
    assert robot._target_trail_rendered_count == 0
    assert robot._trail_step_count == 0
    assert robot._viewer.user_scn.ngeom == 0
    assert events == ["lock", "unlock"]


def test_joint_space_target_trail_uses_forward_kinematics():
    class FakeController:
        _cart_target_matrix = None
        _joint_target_rad = np.arange(7, dtype=float)

    class FakeKinematics:
        def fk(self, joints):
            transform = np.eye(4)
            transform[:3, 3] = (joints[0], joints[1], joints[2])
            return transform, np.zeros(6)

    robot = TwinRobot()
    robot._controller = FakeController()
    robot._kinematics = FakeKinematics()
    robot._state = RobotState.JOINT_IMPEDANCE

    np.testing.assert_allclose(robot._target_tcp_position(), [0.0, 1.0, 2.0])
