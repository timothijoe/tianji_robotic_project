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


def test_skeleton_frame_keypoints_cannot_be_made_writeable():
    frame = SkeletonFrame(1, "right_wrist", "right", np.zeros((21, 3)))

    with pytest.raises(ValueError):
        frame.keypoints_m.setflags(write=True)

    assert frame.keypoints_m[0, 0] == 0


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


def test_trajectory_arrays_cannot_be_made_writeable():
    trajectory = HandTrajectory(
        np.array([10, 20]), np.zeros((2, 20)), HAND_JOINT_NAMES, {}
    )

    with pytest.raises(ValueError):
        trajectory.timestamps_ns.setflags(write=True)
    with pytest.raises(ValueError):
        trajectory.positions_rad.setflags(write=True)

    np.testing.assert_array_equal(trajectory.timestamps_ns, [10, 20])
    np.testing.assert_array_equal(trajectory.positions_rad, np.zeros((2, 20)))


def test_trajectory_recursively_freezes_metadata_without_nested_aliases():
    metadata = {"nested": {"labels": ["initial"], "modes": {"safe"}}}
    trajectory = HandTrajectory(
        np.array([10]), np.zeros((1, 20)), HAND_JOINT_NAMES, metadata
    )

    metadata["nested"]["labels"].append("changed")
    metadata["nested"]["modes"].add("unsafe")
    metadata["nested"]["new"] = "value"

    assert trajectory.metadata == {"nested": {"labels": ("initial",), "modes": frozenset({"safe"})}}
    with pytest.raises(TypeError):
        trajectory.metadata["new"] = "value"  # type: ignore[index]
    with pytest.raises(TypeError):
        trajectory.metadata["nested"]["new"] = "value"  # type: ignore[index]


@pytest.mark.parametrize("timestamp", [True, 1.0, "1", -1])
def test_skeleton_frame_rejects_non_integer_or_negative_timestamp(timestamp):
    with pytest.raises(ValueError, match="timestamp_ns"):
        SkeletonFrame(timestamp, "right_wrist", "right", np.zeros((21, 3)))


@pytest.mark.parametrize("timestamp", [1, np.int64(1)])
def test_skeleton_frame_accepts_nonnegative_python_and_numpy_integers(timestamp):
    assert SkeletonFrame(timestamp, "right_wrist", "right", np.zeros((21, 3))).timestamp_ns == timestamp


def test_trajectory_requires_canonical_unique_joint_names():
    names = HAND_JOINT_NAMES[:-1] + (HAND_JOINT_NAMES[-2],)

    with pytest.raises(ValueError, match="canonical"):
        HandTrajectory(np.array([10]), np.zeros((1, 20)), names, {})


@pytest.mark.parametrize(
    ("keypoints", "message"),
    [(np.zeros((21, 3), dtype=np.complex128), "real"),],
)
def test_skeleton_frame_rejects_complex_keypoints(keypoints, message):
    with pytest.raises(ValueError, match=message):
        SkeletonFrame(1, "right_wrist", "right", keypoints)


def test_trajectory_rejects_complex_positions_and_timestamps():
    with pytest.raises(ValueError, match="real"):
        HandTrajectory(
            np.array([10]), np.zeros((1, 20), dtype=np.complex128), HAND_JOINT_NAMES, {}
        )
    with pytest.raises(ValueError, match="real integers"):
        HandTrajectory(
            np.array([10 + 0j]), np.zeros((1, 20)), HAND_JOINT_NAMES, {}
        )
