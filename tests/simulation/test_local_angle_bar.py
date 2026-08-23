import numpy as np
import pytest

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
