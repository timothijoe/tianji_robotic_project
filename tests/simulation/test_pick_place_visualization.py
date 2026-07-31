import numpy as np

from twin_sim.cli import build_parser
from twin_sim.pick_place_visualization import PickPlaceTrace


def test_pick_place_cli_defaults():
    args = build_parser().parse_args(["pick-place", "--headless"])

    assert args.command == "pick-place"
    assert args.headless
    assert not args.slow
    assert args.final_hold is None


def test_trace_keeps_three_distinct_bounded_paths():
    trace = PickPlaceTrace(max_points=2)
    for index in range(3):
        trace.append(
            planned_palm=(index, 0.0, 0.0),
            actual_palm=(index, 0.0, 0.1),
            actual_cube=(index, 0.0, 0.2),
        )

    assert len(trace.planned_palm) == 2
    assert len(trace.actual_palm) == 2
    assert len(trace.actual_cube) == 2
    np.testing.assert_allclose(trace.planned_palm[-1], (2.0, 0.0, 0.0))
    np.testing.assert_allclose(trace.actual_palm[-1], (2.0, 0.0, 0.1))
    np.testing.assert_allclose(trace.actual_cube[-1], (2.0, 0.0, 0.2))
