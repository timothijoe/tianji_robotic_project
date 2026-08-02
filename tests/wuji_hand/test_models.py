import numpy as np
import pytest

from tianji_robotics.wuji_hand.models import HandTrajectory, SkeletonFrame
from tianji_robotics.wuji_hand.names import HAND_JOINT_NAMES


def test_skeleton_frame_copies_and_validates_keypoints():
    keypoints = np.arange(63, dtype=np.float64).reshape(21, 3)
    frame = SkeletonFrame(1, "right_wrist", "right", keypoints)

    keypoints[0, 0] = -1

    assert frame.timestamp_ns == 1
    assert frame.frame_id == "right_wrist"
    assert frame.side == "right"
    assert frame.keypoints_m[0, 0] == 0
    assert not np.shares_memory(frame.keypoints_m, keypoints)


@pytest.mark.parametrize(
    ("keypoints", "message"),
    [
        (np.zeros((20, 3)), "shape"),
        (np.full((21, 3), np.nan), "finite"),
    ],
)
def test_skeleton_frame_rejects_invalid_keypoints(keypoints, message):
    with pytest.raises(ValueError, match=message):
        SkeletonFrame(1, "right_wrist", "right", keypoints)


def test_trajectory_requires_strictly_increasing_timestamps():
    with pytest.raises(ValueError, match="strictly increasing"):
        HandTrajectory(np.array([10, 10]), np.zeros((2, 20)), HAND_JOINT_NAMES, {})


def test_trajectory_copies_arrays_and_metadata():
    timestamps = np.array([10, 20], dtype=np.int64)
    positions = np.zeros((2, 20), dtype=np.float64)
    metadata = {"source": "fixture"}
    trajectory = HandTrajectory(timestamps, positions, HAND_JOINT_NAMES, metadata)

    timestamps[0] = 0
    positions[0, 0] = 1
    metadata["source"] = "changed"

    assert trajectory.timestamps_ns[0] == 10
    assert trajectory.positions_rad[0, 0] == 0
    assert trajectory.metadata["source"] == "fixture"
    assert not np.shares_memory(trajectory.timestamps_ns, timestamps)
    assert not np.shares_memory(trajectory.positions_rad, positions)


def test_trajectory_requires_canonical_unique_joint_names():
    names = HAND_JOINT_NAMES[:-1] + (HAND_JOINT_NAMES[-2],)

    with pytest.raises(ValueError, match="canonical"):
        HandTrajectory(np.array([10]), np.zeros((1, 20)), names, {})
