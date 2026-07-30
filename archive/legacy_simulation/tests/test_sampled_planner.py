import numpy as np
import pytest

from twin_control.sampled_planner import (
    SampledChopConfig,
    build_sampled_cartesian_targets,
    chop_delta_mm,
    cycle_progress,
)


def test_chop_delta_treats_real_robot_y_axis_as_downward():
    assert chop_delta_mm(SampledChopConfig(chop_axis="y", dz_mm=-40.0)) == 40.0
    assert chop_delta_mm(SampledChopConfig(chop_axis="z", dz_mm=-40.0)) == -40.0


def test_cycle_progress_separate_descends_retracts_then_shifts():
    assert cycle_progress(0, 9, lateral_phase="separate") == (0.0, 0.0, 0.0)
    assert cycle_progress(2, 9, lateral_phase="separate") == (1.0, 0.0, 0.0)
    assert cycle_progress(5, 9, lateral_phase="separate") == (0.0, 1.0, 0.0)
    assert cycle_progress(8, 9, lateral_phase="separate") == (0.0, 1.0, 1.0)


def test_build_sampled_targets_preserves_orientation_and_applies_axes():
    start_pose = np.array((100.0, 200.0, 300.0, 1.0, 2.0, 3.0), dtype=float)
    config = SampledChopConfig(
        control_hz=2.0,
        hold_s=2.0,
        cycles=2,
        dz_mm=-20.0,
        lateral=True,
        lateral_mm=10.0,
        chop_axis="y",
        lateral_axis="z",
        lateral_phase="separate",
    )

    targets = build_sampled_cartesian_targets(start_pose, config)

    assert len(targets) == 8
    np.testing.assert_allclose(targets[0].xyzabc, start_pose)
    assert max(point.xyzabc[1] for point in targets) == 220.0
    assert targets[3].xyzabc[2] == 310.0
    assert targets[-1].xyzabc[2] == 320.0
    assert all(np.allclose(point.xyzabc[3:], start_pose[3:]) for point in targets)
    assert [targets[index].cycle_index for index in (0, 3, 4, 7)] == [0, 0, 1, 1]


def test_sampled_chop_config_rejects_unsafe_or_ambiguous_axes():
    with pytest.raises(ValueError, match="dz_mm magnitude"):
        SampledChopConfig(dz_mm=-81.0)
    with pytest.raises(ValueError, match="lateral_mm magnitude"):
        SampledChopConfig(lateral_mm=51.0)
    with pytest.raises(ValueError, match="must differ"):
        SampledChopConfig(chop_axis="y", lateral_axis="y", lateral=True)
