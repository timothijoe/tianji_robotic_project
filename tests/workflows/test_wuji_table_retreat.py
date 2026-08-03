import numpy as np
import pytest

from tianji_robotics.simulation.tabletop_wuji_hand import TabletopWujiHand
from tianji_robotics.workflows.wuji_table_retreat import TableRetreatConfig, build_table_retreat
from tianji_robotics.wuji_hand.models import HandTrajectory
from tianji_robotics.wuji_hand.names import HAND_JOINT_NAMES


FEASIBLE_POSE = np.array([0.144227609, -0.138699993, 1.557536125, -0.008293608, 1.26238215, -0.370000005, 0.97574544, 0.343102217, 1.26902473, -0.213412717, 0.811683357, 0.350952178, 1.41588736, -0.21959424, 0.369700611, 0.648590744, 1.46277189, -0.033096798, 0.353335321, 0.720260799])


def _trajectory():
    return HandTrajectory(np.array([0, 1_000_000]), np.vstack((FEASIBLE_POSE, FEASIBLE_POSE)), HAND_JOINT_NAMES, {})


def test_builds_contact_retreat_and_uses_earliest_equal_candidate():
    backend = TabletopWujiHand(viewer=False)
    try:
        corrected, report = build_table_retreat(_trajectory(), backend, TableRetreatConfig(place_duration_s=.1, retreat_duration_s=.2, hold_duration_s=.1, candidate_stride=1))
    finally:
        backend.close()
    assert report.source_frame == 0
    assert report.actual_retreat_m == pytest.approx(.03)
    assert report.minimum_thumb_clearance_m >= .010
    assert report.maximum_fingertip_height_error_m <= .002
    assert report.maximum_contact_slip_m <= .003
    assert corrected.phases[0] == "PLACE" and "RETREAT" in corrected.phases and corrected.phases[-1] == "HOLD"
    assert np.max(np.abs(np.diff(corrected.positions_rad, axis=0))) <= .12


def test_explicit_source_frame_is_range_checked():
    backend = TabletopWujiHand(viewer=False)
    try:
        with pytest.raises(ValueError, match="outside trajectory"):
            build_table_retreat(_trajectory(), backend, TableRetreatConfig(source_frame=5, place_duration_s=.1, retreat_duration_s=.2, hold_duration_s=.1))
    finally:
        backend.close()
