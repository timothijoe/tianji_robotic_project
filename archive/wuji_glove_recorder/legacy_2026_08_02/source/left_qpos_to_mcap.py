"""Write offline left Wuji Hand joint targets into a Studio-playable MCAP.

This program performs file conversion only. It never connects to or commands a
physical robot hand.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from mcap.writer import Writer


JOINT_STATES_TOPIC = "/joint_states"
LEFT_JOINT_NAMES = tuple(
    f"left_finger{finger}_joint{joint}"
    for finger in range(1, 6)
    for joint in range(1, 5)
)
JOINT_STATE_SCHEMA = {
    "title": "sensor_msgs/msg/JointState",
    "type": "object",
    "properties": {
        "header": {"type": "object"},
        "name": {"type": "array", "items": {"type": "string"}},
        "position": {"type": "array", "items": {"type": "number"}},
        "velocity": {"type": "array", "items": {"type": "number"}},
        "effort": {"type": "array", "items": {"type": "number"}},
    },
    "required": ["header", "name", "position"],
}


def write_joint_state_mcap(source: Path, destination: Path) -> Path:
    """Convert a `(N, 20)` offline target file into `/joint_states` MCAP data."""
    with np.load(source) as data:
        timestamps_ns = np.asarray(data["timestamps_ns"], dtype=np.int64)
        positions = np.asarray(data["left_joint_positions_rad"], dtype=np.float64)

    if positions.ndim != 2 or positions.shape[1] != 20:
        raise ValueError("expected left_joint_positions_rad with shape (N, 20)")
    if timestamps_ns.shape != (positions.shape[0],):
        raise ValueError("timestamps_ns must have one entry per position frame")
    if not np.isfinite(positions).all():
        raise ValueError("joint positions must be finite")

    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("wb") as stream:
        writer = Writer(stream)
        writer.start(profile="ros2", library="wuji-glove-recorder")
        schema_id = writer.register_schema(
            "sensor_msgs/msg/JointState",
            "jsonschema",
            json.dumps(JOINT_STATE_SCHEMA).encode(),
        )
        channel_id = writer.register_channel(
            JOINT_STATES_TOPIC, "json", schema_id, {"handedness": "left"}
        )
        for sequence, (timestamp_ns, qpos) in enumerate(zip(timestamps_ns, positions)):
            payload = {
                "header": {
                    "seq": sequence,
                    "timestamp_ns": int(timestamp_ns),
                    "frame_id": "left_palm_link",
                },
                "name": list(LEFT_JOINT_NAMES),
                "position": qpos.tolist(),
                "velocity": [],
                "effort": [],
            }
            writer.add_message(
                channel_id,
                log_time=int(timestamp_ns),
                publish_time=int(timestamp_ns),
                sequence=sequence,
                data=json.dumps(payload, separators=(",", ":")).encode(),
            )
        writer.finish()
    return destination


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert offline left Wuji Hand joint targets to a MCAP replay."
    )
    parser.add_argument("source", type=Path, help="Input *_left_wuji_hand.npz")
    parser.add_argument("destination", type=Path, help="Output .mcap")
    args = parser.parse_args()
    print(f"Saved {write_joint_state_mcap(args.source, args.destination)}")


if __name__ == "__main__":
    main()
