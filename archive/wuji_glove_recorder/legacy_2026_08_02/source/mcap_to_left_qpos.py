"""Offline conversion from a Wuji Studio MCAP to left Wuji Hand joint targets.

This program reads a recording file only.  It never connects to a glove or a
robot hand, publishes no ROS topic, and does not enable any motors.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Iterator, Protocol

import numpy as np
from mcap.reader import make_reader


SKELETON_TOPIC = "/right_glove/hand_skeleton"


class Retargeter(Protocol):
    def step(self, keypoints: np.ndarray) -> np.ndarray: ...


def mirror_right_to_left(keypoints: np.ndarray) -> np.ndarray:
    """Express right-wrist skeleton points in the left-wrist handed basis.

    Wuji wrist frames share radial X and proximal Z axes. Their Y axes are
    opposite because each wrist frame follows the right-hand rule.
    """
    mirrored = np.asarray(keypoints, dtype=np.float32).copy()
    if mirrored.shape != (21, 3):
        raise ValueError("expected keypoints with shape (21, 3)")
    mirrored[:, 1] *= -1.0
    return mirrored


def skeleton_frames(path: Path) -> Iterator[tuple[int, str, np.ndarray]]:
    """Yield timestamp-ns, frame id, and (21, 3) skeleton points from MCAP."""
    with path.open("rb") as stream:
        reader = make_reader(stream)
        for _, channel, message in reader.iter_messages(topics=[SKELETON_TOPIC]):
            if channel.topic != SKELETON_TOPIC:
                continue
            payload = json.loads(message.data)
            header = payload["header"]
            points = np.asarray(
                [joint["pose"]["position"] for joint in payload["joints"]],
                dtype=np.float32,
            )
            if points.shape != (21, 3) or not np.isfinite(points).all():
                raise ValueError(f"invalid skeleton frame in {path}")
            yield int(header["timestamp_us"]) * 1_000, str(header["frame_id"]), points


def make_left_retargeter() -> Retargeter:
    """Create the official first-generation left Wuji Hand retargeter."""
    from wuji_sdk import HandModel, Handedness, RetargetSession

    return RetargetSession.for_hand(HandModel.WujiHand, side=Handedness.Left)


def convert_mcap(source: Path, destination: Path, retargeter: Retargeter) -> Path:
    """Convert all right-glove skeleton frames into 20 left-hand joint targets."""
    timestamps: list[int] = []
    commands: list[np.ndarray] = []
    source_frame_id: str | None = None

    for timestamp_ns, frame_id, keypoints in skeleton_frames(source):
        command = np.asarray(
            retargeter.step(mirror_right_to_left(keypoints)), dtype=np.float32
        )
        if command.shape != (20,) or not np.isfinite(command).all():
            raise ValueError("retargeter did not return 20 finite joint positions")
        timestamps.append(timestamp_ns)
        commands.append(command)
        if source_frame_id is None:
            source_frame_id = frame_id

    if not commands:
        raise ValueError(f"no {SKELETON_TOPIC} frames found in {source}")

    destination.parent.mkdir(parents=True, exist_ok=True)
    metadata = {
        "source_mcap": str(source.resolve()),
        "source_topic": SKELETON_TOPIC,
        "source_frame_id": source_frame_id,
        "input_hand_side": "right",
        "input_transform": "mirror_y_right_wrist_to_left_wrist",
        "target_hand_model": "WujiHand",
        "target_hand_side": "left",
        "unit": "rad",
        "frame_count": len(commands),
        "offline_only": True,
    }
    np.savez_compressed(
        destination,
        timestamps_ns=np.asarray(timestamps, dtype=np.int64),
        left_joint_positions_rad=np.asarray(commands, dtype=np.float32),
        metadata=np.asarray(json.dumps(metadata, ensure_ascii=False)),
    )
    return destination


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert Studio hand_skeleton MCAP data to left Wuji Hand targets."
    )
    parser.add_argument("source", type=Path, help="Studio .mcap recording")
    parser.add_argument(
        "destination",
        type=Path,
        nargs="?",
        help="Output .npz (default: next to source with _left_wuji_hand suffix)",
    )
    args = parser.parse_args()
    destination = args.destination or args.source.with_name(
        f"{args.source.stem}_left_wuji_hand.npz"
    )
    output = convert_mcap(args.source, destination, make_left_retargeter())
    print(f"Saved {output}")


if __name__ == "__main__":
    main()
