from types import SimpleNamespace

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


def test_backend_step_advances_once_without_owning_real_time_pacing(monkeypatch):
    backend = LocalWujiHand("left", viewer=False)
    step_calls = []

    class Viewer:
        def is_running(self):
            return True

        def sync(self):
            pass

    try:
        backend._viewer = Viewer()
        monkeypatch.setattr(
            local_angle_bar.mujoco,
            "mj_step",
            lambda model, data: step_calls.append((model, data)),
        )
        monkeypatch.setattr(
            local_angle_bar,
            "time",
            SimpleNamespace(sleep=lambda _seconds: pytest.fail("backend must not sleep")),
            raising=False,
        )

        backend.step()

        assert step_calls == [(backend.model, backend.data)]
    finally:
        backend._viewer = None
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
    def __init__(self, side, *, viewer_is_running=True, timestep_s=0.002):
        self.side = side
        self.target_rad = np.zeros(20)
        self.closed = False
        self.viewer_is_running = viewer_is_running
        self.timestep_s = timestep_s
        self.step_calls = 0

    def close(self):
        self.closed = True

    def step(self):
        self.step_calls += 1


class _FakeRoot:
    def __init__(self):
        self.after_calls = []
        self.destroy_calls = 0
        self.quit_calls = 0

    def after(self, delay_ms, callback):
        self.after_calls.append((delay_ms, callback))

    def winfo_exists(self):
        return True

    def destroy(self):
        self.destroy_calls += 1

    def quit(self):
        self.quit_calls += 1


def test_switching_hand_constructs_a_fresh_side_backend(monkeypatch):
    created = []

    def factory(side, *, viewer):
        created.append((side, viewer))
        return _FakeBackend(side)

    panel = object.__new__(local_angle_bar.WujiAngleBarPanel)
    panel._backend_factory = factory
    panel._backend = factory("left", viewer=True)
    original_backend = panel._backend
    panel._closed = False
    panel._side = _FakeVariable("left")
    panel._rebuild_slider_groups = lambda: None
    monkeypatch.setattr(local_angle_bar, "tk", _FakeTk)

    panel.switch_side("right")

    assert created == [("left", True), ("right", True)]
    assert panel._backend.side == "right"
    assert original_backend.closed


def test_switching_sides_closes_old_backend_before_creating_replacement(monkeypatch):
    active = []
    maximum_live_backends = 0

    class TrackingBackend(_FakeBackend):
        def __init__(self, side):
            nonlocal maximum_live_backends
            super().__init__(side)
            active.append(self)
            maximum_live_backends = max(maximum_live_backends, len(active))

        def close(self):
            super().close()
            active.remove(self)

    def factory(side, *, viewer):
        assert viewer is True
        return TrackingBackend(side)

    panel = object.__new__(local_angle_bar.WujiAngleBarPanel)
    panel._backend_factory = factory
    panel._backend = factory("left", viewer=True)
    panel._closed = False
    panel._side = _FakeVariable("left")
    panel._rebuild_slider_groups = lambda: None
    monkeypatch.setattr(local_angle_bar, "tk", _FakeTk)

    panel.switch_side("right")

    assert maximum_live_backends == 1
    assert panel._backend.side == "right"


def test_switch_failure_closes_panel_and_re_raises_factory_error(monkeypatch):
    root = _FakeRoot()
    original_backend = _FakeBackend("left")

    def failing_factory(side, *, viewer):
        raise RuntimeError("right model unavailable")

    panel = object.__new__(local_angle_bar.WujiAngleBarPanel)
    panel._root = root
    panel._backend_factory = failing_factory
    panel._backend = original_backend
    panel._closed = False
    panel._side = _FakeVariable("left")
    monkeypatch.setattr(local_angle_bar, "tk", _FakeTk)

    with pytest.raises(RuntimeError, match="right model unavailable"):
        panel.switch_side("right")

    assert original_backend.closed
    assert panel._backend is None
    assert root.quit_calls == 1
    assert root.destroy_calls == 1


def test_viewer_close_shuts_down_panel_once():
    root = _FakeRoot()
    backend = _FakeBackend("left", viewer_is_running=False)
    panel = object.__new__(local_angle_bar.WujiAngleBarPanel)
    panel._root = root
    panel._backend = backend
    panel._closed = False

    panel._tick()
    panel.close()

    assert backend.step_calls == 1
    assert backend.closed
    assert root.quit_calls == 1
    assert root.destroy_calls == 1
    assert root.after_calls == []


def test_panel_tick_steps_once_and_schedules_at_model_timestep():
    root = _FakeRoot()
    backend = _FakeBackend("left", viewer_is_running=True, timestep_s=0.002)
    panel = object.__new__(local_angle_bar.WujiAngleBarPanel)
    panel._root = root
    panel._backend = backend
    panel._closed = False

    panel._tick()

    assert backend.step_calls == 1
    assert root.after_calls == [(2, panel._tick)]


def test_panel_displays_mujoco_only_hardware_safety_notice():
    assert "MuJoCo-only" in local_angle_bar.PANEL_SAFETY_TEXT
    assert "does not command hardware" in local_angle_bar.PANEL_SAFETY_TEXT
