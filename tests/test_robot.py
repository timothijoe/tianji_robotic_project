import mujoco.viewer

from twin_control.robot import TwinRobot


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
