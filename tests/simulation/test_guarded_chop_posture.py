import numpy as np

from twin_sim.guarded_chop_safety import CAT_PAW_OPEN_RAD, CAT_PAW_RAD
from twin_sim.model import SimulationModel


def test_cat_paw_target_is_valid_and_tucks_thumb():
    sim = SimulationModel.load()
    ranges = sim.model.actuator_ctrlrange[sim.hand.actuator_ids]

    assert CAT_PAW_RAD.shape == (20,)
    assert np.all(CAT_PAW_RAD >= ranges[:, 0])
    assert np.all(CAT_PAW_RAD <= ranges[:, 1])
    assert CAT_PAW_RAD[0] > 0.6
    assert np.all(CAT_PAW_RAD[[2, 6, 10, 14, 18]] > 0.45)


def test_open_cat_paw_is_visible_and_within_hand_ranges():
    sim = SimulationModel.load()
    ranges = sim.model.actuator_ctrlrange[sim.hand.actuator_ids]

    assert CAT_PAW_OPEN_RAD.shape == (20,)
    assert np.all(CAT_PAW_OPEN_RAD >= ranges[:, 0])
    assert np.all(CAT_PAW_OPEN_RAD <= ranges[:, 1])
    assert np.linalg.norm(CAT_PAW_OPEN_RAD - CAT_PAW_RAD) >= 1.0
    assert np.all(CAT_PAW_OPEN_RAD[[6, 10, 14, 18]] < CAT_PAW_RAD[[6, 10, 14, 18]])
