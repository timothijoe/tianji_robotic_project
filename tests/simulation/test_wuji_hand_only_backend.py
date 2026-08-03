import mujoco
import numpy as np

from tianji_robotics.simulation.wuji_hand import MujocoWujiHand


def test_backend_loads_only_the_official_hand_model():
    backend = MujocoWujiHand(viewer=False)
    try:
        assert (backend.model.nq, backend.model.nv, backend.model.nu) == (20, 20, 20)
        names = bytes(backend.model.names).decode(errors="ignore").lower()
        assert "palm_link" in names
        assert "left_link" not in names and "right_link" not in names
    finally:
        backend.close()


def test_backend_commands_canonical_vector_and_advances_time():
    backend = MujocoWujiHand(viewer=False)
    try:
        target = np.array([(lo + hi) / 2 for lo, hi in backend.joint_ranges_rad.values()])
        backend.command_position_rad(target)
        np.testing.assert_allclose(backend.read_target_position_rad(), target)
        started = backend.data.time
        backend.step(backend.timestep_s)
        assert backend.data.time > started
    finally:
        backend.close()
