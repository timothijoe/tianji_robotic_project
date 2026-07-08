import importlib.util
from pathlib import Path


def _load_demo():
    path = Path(__file__).resolve().parents[1] / "examples" / "mujoco_replay_chop_demo.py"
    spec = importlib.util.spec_from_file_location("mujoco_replay_chop_demo", path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_replay_demo_parse_args_exposes_replay_controls():
    module = _load_demo()

    args = module.parse_args([
        "--execution-model",
        "lagged-noisy",
        "--tracking-alpha",
        "0.25",
        "--joint-noise-std-deg",
        "0.5",
        "--command-delay-steps",
        "3",
        "--joint-velocity-limit-deg-s",
        "30",
        "--seed",
        "42",
    ])

    assert args.execution_model == "lagged-noisy"
    assert args.tracking_alpha == 0.25
    assert args.joint_noise_std_deg == 0.5
    assert args.command_delay_steps == 3
    assert args.joint_velocity_limit_deg_s == 30.0
    assert args.seed == 42


def test_replay_demo_builds_planner_and_replay_config_from_args():
    module = _load_demo()
    args = module.parse_args([
        "--control-hz",
        "100",
        "--hold-s",
        "1.5",
        "--cycles",
        "2",
        "--dz-mm",
        "-30",
        "--lateral-axis",
        "z",
        "--execution-model",
        "lagged",
        "--tracking-alpha",
        "0.5",
    ])

    planner_config = module._planner_config(args)
    replay_config = module._replay_config(args)

    assert planner_config.control_hz == 100.0
    assert planner_config.hold_s == 1.5
    assert planner_config.cycles == 2
    assert planner_config.dz_mm == -30.0
    assert planner_config.lateral_axis == "z"
    assert replay_config.model == "lagged"
    assert replay_config.dt_s == 0.01
    assert replay_config.tracking_alpha == 0.5
