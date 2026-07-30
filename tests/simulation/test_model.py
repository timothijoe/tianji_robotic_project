import mujoco
import pytest

from twin_sim.model import ModelValidationError, SimulationModel


def test_active_scene_has_fourteen_position_actuators():
    sim = SimulationModel.load()
    assert sim.model.njnt == 14
    assert sim.model.nu == 14
    assert sim.right.actuator_ids.shape == (7,)
    assert all(
        sim.model.actuator_biastype[i] == mujoco.mjtBias.mjBIAS_AFFINE
        for i in sim.right.actuator_ids
    )


def test_active_scene_has_task_objects():
    sim = SimulationModel.load()
    assert sim.require_site("right_tool_tip_site") >= 0
    assert sim.require_geom("chopping_board") >= 0
    assert sim.require_sensor("right_tool_force") >= 0


def test_missing_model_object_has_clear_error():
    sim = SimulationModel.load()
    with pytest.raises(ModelValidationError, match="missing site: absent"):
        sim.require_site("absent")
