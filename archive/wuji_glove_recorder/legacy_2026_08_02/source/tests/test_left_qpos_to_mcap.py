import json
from pathlib import Path

import numpy as np
from mcap.reader import make_reader

from left_qpos_to_mcap import LEFT_JOINT_NAMES, write_joint_state_mcap


def test_write_joint_state_mcap_emits_left_joint_names_and_positions(tmp_path: Path):
    source = tmp_path / "left_commands.npz"
    destination = tmp_path / "left_replay.mcap"
    np.savez_compressed(
        source,
        timestamps_ns=np.asarray([1_000, 2_000], dtype=np.int64),
        left_joint_positions_rad=np.arange(40, dtype=np.float32).reshape(2, 20),
    )

    result = write_joint_state_mcap(source, destination)

    assert result == destination
    with destination.open("rb") as stream:
        reader = make_reader(stream)
        messages = list(reader.iter_messages(topics=["/joint_states"]))
    assert len(messages) == 2
    _, channel, message = messages[1]
    payload = json.loads(message.data)
    assert channel.message_encoding == "json"
    assert payload["name"] == list(LEFT_JOINT_NAMES)
    assert payload["position"] == list(np.arange(20, 40, dtype=np.float32))
    assert payload["header"]["frame_id"] == "left_palm_link"
