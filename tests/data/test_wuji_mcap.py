import json

import numpy as np
import pytest
from mcap.writer import Writer

from tianji_robotics.data.mcap import StudioMcapSkeletonSource


def _write_messages(path, messages):
    with path.open("wb") as stream:
        writer = Writer(stream)
        writer.start()
        schema_id = writer.register_schema("json", "jsonschema", b"{}")
        skeleton_channel = writer.register_channel(
            "/right_glove/hand_skeleton", "json", schema_id
        )
        other_channel = writer.register_channel("/ignored", "json", schema_id)
        for index, (channel, payload) in enumerate(messages):
            writer.add_message(
                skeleton_channel if channel == "skeleton" else other_channel,
                log_time=index,
                publish_time=index,
                data=json.dumps(payload).encode(),
            )
        writer.finish()


def test_frames_reads_only_right_glove_skeleton_messages(tmp_path):
    path = tmp_path / "glove.mcap"
    keypoints = np.arange(63, dtype=float).reshape(21, 3).tolist()
    _write_messages(
        path,
        [
            ("skeleton", {"timestamp_us": 17, "keypoints_m": keypoints}),
            ("other", {"timestamp_us": 99, "keypoints_m": keypoints}),
            ("skeleton", {"timestamp_us": 23, "keypoints_m": keypoints}),
        ],
    )

    frames = list(StudioMcapSkeletonSource(path).frames())

    assert [frame.timestamp_ns for frame in frames] == [17_000, 23_000]
    assert [frame.frame_id for frame in frames] == ["r_wrist", "r_wrist"]
    assert [frame.side for frame in frames] == ["right", "right"]
    np.testing.assert_allclose(frames[0].keypoints_m, keypoints)


def test_frames_reports_source_path_for_malformed_skeleton(tmp_path):
    path = tmp_path / "broken.mcap"
    _write_messages(path, [("skeleton", {"timestamp_us": 17, "keypoints_m": []})])

    with pytest.raises(ValueError, match="broken\\.mcap"):
        list(StudioMcapSkeletonSource(path).frames())
