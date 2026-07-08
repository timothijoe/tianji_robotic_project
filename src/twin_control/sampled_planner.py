"""Sampled Cartesian chopping target generation.

This module is intentionally independent of MuJoCo and the vendor SDK.  It
produces SDK-style XYZABC targets in millimetres/degrees; callers decide
whether those targets are sent to real hardware, converted through IK, or
replayed in simulation.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


AxisName = str


@dataclass(frozen=True)
class SampledChopConfig:
    control_hz: float = 250.0
    hold_s: float = 2.0
    cycles: int = 5
    dz_mm: float = -20.0
    lateral: bool = True
    lateral_mm: float = 10.0
    chop_axis: AxisName = "y"
    lateral_axis: AxisName = "x"
    lateral_phase: str = "separate"

    def __post_init__(self) -> None:
        if self.control_hz <= 0.0 or not math.isfinite(self.control_hz):
            raise ValueError("control_hz must be positive finite")
        if self.hold_s <= 0.0 or not math.isfinite(self.hold_s):
            raise ValueError("hold_s must be positive finite")
        if self.cycles <= 0:
            raise ValueError("cycles must be positive")
        if abs(float(self.dz_mm)) > 80.0:
            raise ValueError("dz_mm magnitude must be <= 80")
        if abs(float(self.lateral_mm)) > 50.0:
            raise ValueError("lateral_mm magnitude must be <= 50")
        axis_index(self.chop_axis)
        axis_index(self.lateral_axis)
        if self.lateral and str(self.chop_axis).lower() == str(self.lateral_axis).lower():
            raise ValueError("chop_axis and lateral_axis must differ when lateral is enabled")
        if self.lateral_phase not in ("separate", "retract"):
            raise ValueError("lateral_phase must be 'separate' or 'retract'")


@dataclass(frozen=True)
class SampledCartesianTarget:
    step_index: int
    cycle_index: int
    xyzabc: np.ndarray


def axis_index(axis: str) -> int:
    axis_map = {"x": 0, "y": 1, "z": 2}
    try:
        return axis_map[str(axis).lower()]
    except KeyError as exc:
        raise ValueError("axis must be one of 'x', 'y', or 'z'") from exc


def chop_delta_mm(config: SampledChopConfig) -> float:
    """Return signed down-stroke displacement in SDK millimetres."""
    dz = float(config.dz_mm)
    if str(config.chop_axis).lower() == "y":
        return abs(dz)
    return dz


def cycle_progress(
    cycle_step: int,
    steps_per_cycle: int,
    lateral_phase: str = "separate",
) -> tuple[float, float, float]:
    """Return down/retract/shift progress for one sampled cycle."""
    if lateral_phase == "retract":
        descent_steps = max(1, int(steps_per_cycle) // 2)
        retract_steps = max(1, int(steps_per_cycle) - descent_steps)
        if int(cycle_step) < descent_steps:
            denominator = max(descent_steps - 1, 1)
            return float(cycle_step) / float(denominator), 0.0, 0.0

        retract_step = min(int(cycle_step) - descent_steps, retract_steps - 1)
        denominator = max(retract_steps - 1, 1)
        retract_progress = float(retract_step) / float(denominator)
        return 1.0 - retract_progress, retract_progress, retract_progress

    vertical_steps = max(2, int(round(float(steps_per_cycle) * 2.0 / 3.0)))
    shift_steps = max(1, int(steps_per_cycle) - vertical_steps)
    descent_steps = max(1, vertical_steps // 2)
    retract_steps = max(1, vertical_steps - descent_steps)
    if int(cycle_step) < descent_steps:
        denominator = max(descent_steps - 1, 1)
        return float(cycle_step) / float(denominator), 0.0, 0.0
    if int(cycle_step) < vertical_steps:
        retract_step = min(int(cycle_step) - descent_steps, retract_steps - 1)
        denominator = max(retract_steps - 1, 1)
        retract_progress = float(retract_step) / float(denominator)
        return 1.0 - retract_progress, retract_progress, 0.0

    if shift_steps == 1:
        return 0.0, 1.0, 1.0
    shift_step = min(int(cycle_step) - vertical_steps, shift_steps - 1)
    shift_progress = float(shift_step) / float(shift_steps - 1)
    return 0.0, 1.0, shift_progress


def build_sampled_cartesian_targets(
    start_pose_mm: np.ndarray,
    config: SampledChopConfig,
) -> list[SampledCartesianTarget]:
    """Generate per-control-tick SDK-style XYZABC targets."""
    start = np.asarray(start_pose_mm, dtype=float).reshape(6)
    if not np.all(np.isfinite(start)):
        raise ValueError("start_pose_mm must contain six finite values")

    steps_per_cycle = max(3, int(round(float(config.hold_s) * float(config.control_hz))))
    chop_delta = chop_delta_mm(config)
    lateral_step = abs(float(config.lateral_mm)) if config.lateral else 0.0
    chop_idx = axis_index(config.chop_axis)
    lateral_idx = axis_index(config.lateral_axis)

    targets: list[SampledCartesianTarget] = []
    total_steps = steps_per_cycle * int(config.cycles)
    for step_index in range(total_steps):
        cycle_index = min(step_index // steps_per_cycle, int(config.cycles) - 1)
        cycle_step = step_index - cycle_index * steps_per_cycle
        down_progress, _retract_progress, shift_progress = cycle_progress(
            cycle_step, steps_per_cycle, config.lateral_phase
        )
        target = start.copy()
        target[chop_idx] = start[chop_idx] + chop_delta * down_progress
        target[lateral_idx] = start[lateral_idx] + lateral_step * (float(cycle_index) + shift_progress)
        targets.append(SampledCartesianTarget(step_index, cycle_index, target))
    return targets
