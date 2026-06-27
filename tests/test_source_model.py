from pathlib import Path

import mujoco


ROOT = Path(__file__).resolve().parents[1]
SOURCE_MODEL = ROOT / "MarvinCCS" / "marvin_final_fixed.xml"


def _names(model, object_type, count):
    names = []
    for index in range(count):
        name = mujoco.mj_id2name(model, object_type, index)
        if name:
            names.append(name)
    return names


def test_source_model_loads_with_expected_dual_arm_joints_and_actuators():
    model = mujoco.MjModel.from_xml_path(str(SOURCE_MODEL))

    assert model.njnt == 14
    assert model.nu == 14
    assert model.jnt_type.tolist() == [mujoco.mjtJoint.mjJNT_HINGE.value] * model.njnt
    assert _names(model, mujoco.mjtObj.mjOBJ_JOINT, model.njnt) == [
        "left_joint1",
        "left_joint2",
        "left_joint3",
        "left_joint4",
        "left_joint5",
        "left_joint6",
        "left_joint7",
        "right_joint1",
        "right_joint2",
        "right_joint3",
        "right_joint4",
        "right_joint5",
        "right_joint6",
        "right_joint7",
    ]
    assert _names(model, mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu) == [
        "act_left_joint1",
        "act_left_joint2",
        "act_left_joint3",
        "act_left_joint4",
        "act_left_joint5",
        "act_left_joint6",
        "act_left_joint7",
        "act_right_joint1",
        "act_right_joint2",
        "act_right_joint3",
        "act_right_joint4",
        "act_right_joint5",
        "act_right_joint6",
        "act_right_joint7",
    ]


def test_twin_chop_cli_runs_headless_demo(tmp_path):
    from twin_mujoco.cli import main

    log_path = tmp_path / "cli_chop.csv"

    assert main(["--cycles", "1", "--hold", "0.01", "--headless", "--log", str(log_path)]) == 0
    assert log_path.is_file()
