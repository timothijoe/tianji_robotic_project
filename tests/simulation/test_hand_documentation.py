from pathlib import Path

from twin_sim.hand_names import HAND_ACTUATORS, HAND_JOINTS
from twin_sim.model import SimulationModel


ROOT = Path(__file__).resolve().parents[2]
GUIDE = ROOT / "docs" / "simulation" / "wuji_hand_control.md"


def test_hand_guide_matches_compiled_model_contract():
    text = GUIDE.read_text(encoding="utf-8")
    sim = SimulationModel.load()
    for joint, actuator, actuator_id in zip(
        HAND_JOINTS, HAND_ACTUATORS, sim.hand.actuator_ids, strict=True
    ):
        lower, upper = sim.model.actuator_ctrlrange[actuator_id]
        assert joint in text
        assert actuator in text
        assert f"{lower:.4f}" in text
        assert f"{upper:.4f}" in text
    for heading in ("仿真控制", "平滑发送", "读取状态", "未来实体手接入"):
        assert heading in text
