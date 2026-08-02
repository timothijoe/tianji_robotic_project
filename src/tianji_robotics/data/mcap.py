"""MCAP codecs for offline Wuji hand recordings."""

from collections.abc import Iterator
import json
from pathlib import Path

from mcap.reader import make_reader
from mcap.writer import Writer

from tianji_robotics.wuji_hand.models import HandTrajectory, SkeletonFrame


SKELETON_TOPIC = "/right_glove/hand_skeleton"


class StudioMcapSkeletonSource:
    """Read right-glove skeleton frames recorded by Wuji Studio."""

    def __init__(self, path: Path) -> None:
        self.path = Path(path)

    def frames(self) -> Iterator[SkeletonFrame]:
        try:
            with self.path.open("rb") as stream:
                reader = make_reader(stream)
                for _schema, _channel, message in reader.iter_messages(topics=SKELETON_TOPIC):
                    payload = json.loads(message.data)
                    yield SkeletonFrame(
                        timestamp_ns=payload["timestamp_us"] * 1_000,
                        frame_id="r_wrist",
                        side="right",
                        keypoints_m=payload["keypoints_m"],
                    )
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            raise ValueError(f"malformed skeleton frame in {self.path}") from exc


def write_joint_state_mcap(trajectory: HandTrajectory, destination: Path) -> Path:
    """Write a trajectory as JSON messages on the standard joint-state topic."""
    destination = Path(destination)
    with destination.open("wb") as stream:
        writer = Writer(stream)
        writer.start()
        schema_id = writer.register_schema(
            "sensor_msgs/msg/JointState", "jsonschema", b"{}"
        )
        channel_id = writer.register_channel("/joint_states", "json", schema_id)
        for timestamp_ns, positions_rad in zip(
            trajectory.timestamps_ns, trajectory.positions_rad, strict=True
        ):
            timestamp = int(timestamp_ns)
            payload = json.dumps(
                {
                    "name": list(trajectory.joint_names),
                    "position": positions_rad.tolist(),
                    "timestamp_ns": timestamp,
                },
                separators=(",", ":"),
            ).encode()
            writer.add_message(channel_id, timestamp, payload, timestamp)
        writer.finish()
    return destination
