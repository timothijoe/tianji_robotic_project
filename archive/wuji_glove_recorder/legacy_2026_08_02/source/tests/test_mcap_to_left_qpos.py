import json
from pathlib import Path

import numpy as np
from mcap.writer import Writer

from mcap_to_left_qpos import convert_mcap, mirror_right_to_left


def write_skeleton_mcap(path: Path) -> None:
    with path.open("wb") as stream:
        writer = Writer(stream)
        writer.start()
        skeleton_channel = writer.register_channel(
            "/right_glove/hand_skeleton", "json", 0
        )
        other_channel = writer.register_channel("/right_glove/tactile", "json", 0)
        for frame_index in range(2):
            joints = [
                {
                    "name": f"joint_{index}",
                    "pose": {"position": [index, frame_index, index + frame_index]},
                }
                for index in range(21)
            ]
            payload = json.dumps(
                {
                    "header": {"timestamp_us": 1000 + frame_index, "frame_id": "r_wrist"},
                    "joints": joints,
                }
            ).encode()
            writer.add_message(skeleton_channel, frame_index, payload, frame_index)
            writer.add_message(other_channel, frame_index, b'{"ignored":true}', frame_index)
        writer.finish()


class FakeLeftRetargeter:
    def __init__(self):
        self.inputs = []

    def step(self, keypoints: np.ndarray) -> np.ndarray:
        self.inputs.append(keypoints.copy())
        return np.arange(20, dtype=np.float32) + keypoints[0, 1]


def test_mirror_right_to_left_flips_only_the_wrist_y_axis():
    keypoints = np.asarray(
        [[float(index), 2.0 if index == 0 else -5.0, float(-index)] for index in range(21)],
        dtype=np.float32,
    )

    mirrored = mirror_right_to_left(keypoints)

    assert mirrored[0].tolist() == [0.0, -2.0, 0.0]
    assert mirrored[1].tolist() == [1.0, 5.0, -1.0]
    assert keypoints[0].tolist() == [0.0, 2.0, 0.0]
    assert keypoints[1].tolist() == [1.0, -5.0, -1.0]


def test_convert_mcap_writes_one_left_hand_command_per_skeleton_frame(tmp_path: Path):
    source = tmp_path / "source.mcap"
    destination = tmp_path / "left_commands.npz"
    write_skeleton_mcap(source)
    retargeter = FakeLeftRetargeter()

    result = convert_mcap(source, destination, retargeter)

    assert result == destination
    assert len(retargeter.inputs) == 2
    assert retargeter.inputs[0].shape == (21, 3)
    assert retargeter.inputs[0][1, 1] == 0.0
    assert retargeter.inputs[1][1, 1] == -1.0
    with np.load(destination) as data:
        assert data["timestamps_ns"].tolist() == [1_000_000, 1_001_000]
        assert data["left_joint_positions_rad"].shape == (2, 20)
        assert data["left_joint_positions_rad"][1, 0] == -1.0
        metadata = json.loads(str(data["metadata"]))
        assert metadata["target_hand_side"] == "left"
        assert metadata["source_frame_id"] == "r_wrist"
