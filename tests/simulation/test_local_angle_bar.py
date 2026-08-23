import numpy as np
import pytest

import tianji_robotics.simulation.local_angle_bar as local_angle_bar
from tianji_robotics.simulation.local_angle_bar import LocalWujiHand
from tianji_robotics.simulation.paths import official_wuji_hand_mjcf


@pytest.mark.parametrize("side", ["left", "right"])
def test_each_official_hand_has_twenty_bounded_position_actuators(side):
    backend = LocalWujiHand(side, viewer=False)
    try:
        assert backend.side == side
        assert len(backend.joint_names) == 20
        assert backend.control_ranges_rad.shape == (20, 2)
        assert np.all(np.isfinite(backend.control_ranges_rad))
        assert np.all(backend.control_ranges_rad[:, 0] < backend.control_ranges_rad[:, 1])
        assert official_wuji_hand_mjcf(side).is_file()
    finally:
        backend.close()


def test_backend_rejects_wrong_shape_nan_and_out_of_range_targets():
    backend = LocalWujiHand("left", viewer=False)
    try:
        with pytest.raises(ValueError, match="20 finite"):
            backend.command(np.zeros(19))
        with pytest.raises(ValueError, match="20 finite"):
            backend.command(np.full(20, np.nan))
        bad = backend.target_rad.copy()
        bad[0] = backend.control_ranges_rad[0, 1] + 0.01
        with pytest.raises(ValueError, match="outside range"):
            backend.command(bad)
    finally:
        backend.close()


def test_reset_open_commands_active_hand_open_target():
    backend = LocalWujiHand("right", viewer=False)
    try:
        backend.command(backend.control_ranges_rad[:, 1])

        backend.reset_open()

        np.testing.assert_allclose(backend.target_rad, backend.open_target_rad)
        assert np.all(backend.target_rad >= backend.control_ranges_rad[:, 0])
        assert np.all(backend.target_rad <= backend.control_ranges_rad[:, 1])
    finally:
        backend.close()


class _FakeVariable:
    def __init__(self, value):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class _FakeTk:
    DoubleVar = _FakeVariable


class _FakeBackend:
    def __init__(self, side):
        self.side = side
        self.target_rad = np.zeros(20)
        self.closed = False

    def close(self):
        self.closed = True


def test_switching_hand_constructs_a_fresh_side_backend(monkeypatch):
    created = []

    def factory(side, *, viewer):
        created.append((side, viewer))
        return _FakeBackend(side)

    panel = object.__new__(local_angle_bar.WujiAngleBarPanel)
    panel._backend_factory = factory
    panel._backend = factory("left", viewer=True)
    original_backend = panel._backend
    panel._side = _FakeVariable("left")
    panel._rebuild_slider_groups = lambda: None
    monkeypatch.setattr(local_angle_bar, "tk", _FakeTk)

    panel.switch_side("right")

    assert created == [("left", True), ("right", True)]
    assert panel._backend.side == "right"
    assert original_backend.closed
