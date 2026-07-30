import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class SimulationSample:
    time_s: float
    phase: str
    target_joints_rad: np.ndarray
    actual_joints_rad: np.ndarray
    target_pose: np.ndarray
    actual_pose: np.ndarray
    raw_force_n: float
    filtered_force_n: float
    force_over_threshold: bool


_FIELDS = (
    "time_s",
    "phase",
    *(f"target_q_{index}" for index in range(7)),
    *(f"actual_q_{index}" for index in range(7)),
    *(f"target_pose_{index}" for index in range(16)),
    *(f"actual_pose_{index}" for index in range(16)),
    "raw_force_n",
    "filtered_force_n",
    "force_over_threshold",
)


def write_csv(path: str | Path, samples: Iterable[SimulationSample]) -> None:
    destination = Path(path)
    if not destination.parent.is_dir():
        raise ValueError(f"CSV parent directory does not exist: {destination.parent}")
    rows = [_row(sample) for sample in samples]
    with destination.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def _row(sample: SimulationSample) -> dict[str, object]:
    target_joints = _array(sample.target_joints_rad, (7,), "target_joints_rad")
    actual_joints = _array(sample.actual_joints_rad, (7,), "actual_joints_rad")
    target_pose = _array(sample.target_pose, (4, 4), "target_pose").reshape(-1)
    actual_pose = _array(sample.actual_pose, (4, 4), "actual_pose").reshape(-1)
    if not np.isfinite(sample.time_s) or sample.time_s < 0.0:
        raise ValueError("time_s must be non-negative and finite")
    if not isinstance(sample.phase, str) or not sample.phase:
        raise ValueError("phase must be a non-empty string")
    if not np.isfinite(sample.raw_force_n) or not np.isfinite(
        sample.filtered_force_n
    ):
        raise ValueError("force values must be finite")
    row: dict[str, object] = {"time_s": float(sample.time_s), "phase": sample.phase}
    row.update(
        {f"target_q_{index}": float(value) for index, value in enumerate(target_joints)}
    )
    row.update(
        {f"actual_q_{index}": float(value) for index, value in enumerate(actual_joints)}
    )
    row.update(
        {f"target_pose_{index}": float(value) for index, value in enumerate(target_pose)}
    )
    row.update(
        {f"actual_pose_{index}": float(value) for index, value in enumerate(actual_pose)}
    )
    row.update(
        {
            "raw_force_n": float(sample.raw_force_n),
            "filtered_force_n": float(sample.filtered_force_n),
            "force_over_threshold": bool(sample.force_over_threshold),
        }
    )
    return row


def _array(value: np.ndarray, shape: tuple[int, ...], name: str) -> np.ndarray:
    try:
        array = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must have shape {shape} and finite values") from error
    if array.shape != shape or not np.isfinite(array).all():
        raise ValueError(f"{name} must have shape {shape} and finite values")
    return array

