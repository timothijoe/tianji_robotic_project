from pathlib import Path

import numpy as np
import pytest

from glove_recorder import Recording, extract_keypoints


def test_extract_keypoints_accepts_list_position_frames():
    class Position:
        def __init__(self, xyz):
            self.xyz = xyz

        def __iter__(self):
            return iter(self.xyz)

    class Pose:
        def __init__(self, xyz):
            self.position = Position(xyz)

    class Joint:
        def __init__(self, xyz):
            self.pose = Pose(xyz)

    class Frame:
        joints = [Joint((i, i + 1, i + 2)) for i in range(21)]

    points = extract_keypoints(Frame())

    assert len(points) == 21
    assert points[20] == (20.0, 21.0, 22.0)


def test_save_writes_expected_arrays_and_metadata(tmp_path: Path):
    recording = Recording("192.168.10.151:50001", "SN1", "right")
    recording.append(123, [(0.0, 0.0, 0.0)] * 21)

    path = recording.save(tmp_path)

    with np.load(path) as data:
        assert data["timestamps_ns"].tolist() == [123]
        assert data["keypoints_m"].shape == (1, 21, 3)
        assert "SN1" in str(data["metadata"])


def test_save_rejects_an_empty_recording(tmp_path: Path):
    with pytest.raises(ValueError, match="no valid frames"):
        Recording("endpoint", "SN1", "right").save(tmp_path)


def test_append_rejects_malformed_keypoint():
    recording = Recording("endpoint", "SN1", "right")

    with pytest.raises(ValueError, match="invalid keypoint"):
        recording.append(1, [(0.0, 0.0, 0.0)] * 20 + [(1.0, 2.0)])
