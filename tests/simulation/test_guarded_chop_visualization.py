import numpy as np

from twin_sim.guarded_chop_visualization import GuardedChopTrace


class FakeGeom:
    def __init__(self):
        self.type = 0
        self.size = np.zeros(3)
        self.pos = np.zeros(3)
        self.mat = np.zeros((3, 3))
        self.rgba = np.zeros(4)
        self.category = 0
        self.segid = 0
        self.objtype = 0
        self.objid = 0
        self.dataid = 0
        self.emission = 0.0
        self.specular = 0.0
        self.shininess = 0.0
        self.reflectance = 0.0


class FakeScene:
    def __init__(self):
        self.ngeom = 0
        self.maxgeom = 16
        self.geoms = [FakeGeom() for _ in range(self.maxgeom)]


class FakeLock:
    def __enter__(self):
        return self

    def __exit__(self, *args):
        return None


class FakeViewer:
    def __init__(self):
        self.user_scn = FakeScene()
        self.texts = None

    @staticmethod
    def lock():
        return FakeLock()

    def set_texts(self, texts):
        self.texts = texts


def test_trace_uses_approved_colors_and_overlay():
    viewer = FakeViewer()
    trace = GuardedChopTrace(viewer, marker_stride=1)

    trace.append(
        planned_knife=np.array((0.6, 0.0, 0.4)),
        actual_knife=np.array((0.6, 0.0, 0.39)),
        actual_guard=np.array((0.6, 0.04, 0.36)),
        guard_target=np.array((0.6, 0.04, 0.36)),
        phase="cut_down",
        cut_index=2,
        minimum_distance_m=0.025,
        cut_allowed=True,
    )

    assert viewer.user_scn.ngeom == 4
    np.testing.assert_allclose(
        viewer.user_scn.geoms[0].rgba, (0.1, 0.35, 1.0, 0.9)
    )
    np.testing.assert_allclose(
        viewer.user_scn.geoms[1].rgba, (0.0, 0.9, 1.0, 0.9)
    )
    np.testing.assert_allclose(
        viewer.user_scn.geoms[2].rgba, (0.75, 0.1, 0.9, 0.95)
    )
    np.testing.assert_allclose(
        viewer.user_scn.geoms[3].rgba, (1.0, 0.85, 0.1, 0.95)
    )
    assert viewer.texts[2] == "Guarded chop"
    assert "cut=2/5" in viewer.texts[3]
    assert "distance=0.025" in viewer.texts[3]


def test_abort_reason_replaces_live_phase():
    viewer = FakeViewer()
    trace = GuardedChopTrace(viewer)

    trace.set_abort("clearance lost")

    assert "phase=aborted" in viewer.texts[3]
    assert "clearance lost" in viewer.texts[3]


def test_overlay_keeps_cumulative_minimum_distance():
    viewer = FakeViewer()
    trace = GuardedChopTrace(viewer, marker_stride=1)
    common = dict(
        planned_knife=(0.0, 0.0, 0.0),
        actual_knife=(0.0, 0.0, 0.0),
        actual_guard=(0.0, 0.0, 0.0),
        guard_target=(0.0, 0.0, 0.0),
        phase="cut_down",
        cut_index=1,
        cut_allowed=True,
    )
    trace.append(minimum_distance_m=0.025, **common)
    trace.append(minimum_distance_m=0.040, **common)

    assert "distance=0.025" in viewer.texts[3]
