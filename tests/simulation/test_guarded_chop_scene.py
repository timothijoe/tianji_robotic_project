import mujoco
import numpy as np

from twin_sim.model import SimulationModel


def test_guarded_chop_proxy_is_fixed_and_hidden_by_default():
    sim = SimulationModel.load()
    geom = sim.require_geom("guarded_chop_cube")
    body = int(sim.model.geom_bodyid[geom])

    assert sim.model.body_jntnum[body] == 0
    assert sim.model.geom_rgba[geom, 3] == 0.0
    assert sim.model.geom_contype[geom] == 0
    assert sim.model.geom_conaffinity[geom] == 0


def test_guard_knuckle_site_belongs_to_left_hand():
    sim = SimulationModel.load()
    site = sim.require_site("left_guard_knuckle_site")
    body = int(sim.model.site_bodyid[site])
    name = mujoco.mj_id2name(
        sim.model, mujoco.mjtObj.mjOBJ_BODY, body
    )

    assert name == "left_finger3_link3"
    assert np.isfinite(sim.data.site_xpos[site]).all()
