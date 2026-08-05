#!/usr/bin/env python3
"""Generate a local offline-only left-arm trajectory-entry NPZ."""

import argparse
from pathlib import Path
import sys

import numpy as np

# A direct ``python scripts/...`` invocation starts with only ``scripts`` on
# sys.path; make the repository's src-layout package available in that context.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from tianji_robotics.data.left_arm_entry import (
    generate_left_arm_entry,
    save_left_arm_entry_npz,
)


def parse_start_deg(value: str) -> np.ndarray:
    """Parse seven finite comma-separated joint angles in degrees."""
    try:
        start_deg = np.asarray([float(item) for item in value.split(",")], dtype=float)
    except ValueError as error:
        raise argparse.ArgumentTypeError(
            "--start-deg must contain seven comma-separated numbers"
        ) from error
    if start_deg.shape != (7,) or not np.all(np.isfinite(start_deg)):
        raise argparse.ArgumentTypeError(
            "--start-deg must contain seven finite comma-separated numbers"
        )
    return start_deg


def load_destination_rad(source_npz: Path) -> np.ndarray:
    """Load and validate the final left-arm target from a source recording."""
    try:
        with np.load(source_npz, allow_pickle=False) as data:
            left_arm_target_rad = np.asarray(data["left_arm_target_rad"], dtype=float)
    except (KeyError, OSError, ValueError) as error:
        raise ValueError(
            "source NPZ must contain a readable left_arm_target_rad array"
        ) from error
    if (
        left_arm_target_rad.ndim != 2
        or left_arm_target_rad.shape[0] < 1
        or left_arm_target_rad.shape[1] != 7
        or not np.all(np.isfinite(left_arm_target_rad))
    ):
        raise ValueError("left_arm_target_rad must be a finite (N, 7) array with N >= 1")
    return left_arm_target_rad[-1].copy()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-npz", required=True, type=Path)
    parser.add_argument("--start-deg", required=True, type=parse_start_deg)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--duration-s", type=float, default=20.0)
    parser.add_argument("--sample-rate-hz", type=float, default=200.0)
    args = parser.parse_args()

    try:
        destination_rad = load_destination_rad(args.source_npz)
        trajectory = generate_left_arm_entry(
            np.deg2rad(args.start_deg),
            destination_rad,
            duration_s=args.duration_s,
            sample_rate_hz=args.sample_rate_hz,
        )
        saved = save_left_arm_entry_npz(
            trajectory,
            source_npz=args.source_npz,
            destination=args.output,
            start_deg=args.start_deg,
            duration_s=args.duration_s,
            sample_rate_hz=args.sample_rate_hz,
        )
    except ValueError as error:
        parser.error(str(error))

    print(f"path: {saved}")
    print(f"frame_count: {trajectory.time_s.size}")
    print(f"duration_s: {trajectory.time_s[-1]:.6g}")
    print(
        "per_joint_total_change_rad: "
        f"{np.array2string(trajectory.left_arm_target_rad[-1] - trajectory.left_arm_target_rad[0])}"
    )
    print(f"peak_velocity_rad_s: {np.array2string(trajectory.peak_velocity_rad_s)}")
    print(
        "peak_acceleration_rad_s2: "
        f"{np.array2string(trajectory.peak_acceleration_rad_s2)}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
