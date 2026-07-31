from dataclasses import dataclass

import numpy as np


CAT_PAW_RAD = np.asarray(
    (0.95, -0.02, 0.95, 1.05)
    + (0.30, 0.00, 1.10, 1.10) * 4,
    dtype=float,
)


@dataclass(frozen=True)
class SafetyObservation:
    phase: str
    knife_height_m: float
    safe_knife_height_m: float
    knife_guard_distance_m: float
    left_target_stationary: bool
    right_target_stationary: bool
    left_speed_rad_s: float
    right_speed_rad_s: float
    hand_speed_rad_s: float
    finite_state: bool


@dataclass(frozen=True)
class SafetyDecision:
    allowed: bool
    reason: str = ""


class SafetyCoordinator:
    def __init__(self, minimum_distance_m: float = 0.02):
        if (
            not np.isfinite(minimum_distance_m)
            or minimum_distance_m <= 0.0
        ):
            raise ValueError(
                "minimum_distance_m must be positive and finite"
            )
        self.minimum_distance_m = float(minimum_distance_m)

    def evaluate(self, value: SafetyObservation) -> SafetyDecision:
        scalars = (
            value.knife_height_m,
            value.safe_knife_height_m,
            value.knife_guard_distance_m,
            value.left_speed_rad_s,
            value.right_speed_rad_s,
            value.hand_speed_rad_s,
        )
        if not value.finite_state or not np.isfinite(scalars).all():
            return SafetyDecision(False, "non-finite simulation state")
        if value.knife_guard_distance_m < self.minimum_distance_m:
            return SafetyDecision(
                False, "knife-guard distance below limit"
            )

        phase = str(getattr(value.phase, "value", value.phase)).upper()
        if phase == "CUT_DOWN" and (
            not value.left_target_stationary
            or value.left_speed_rad_s > 0.05
            or value.hand_speed_rad_s > 0.05
        ):
            return SafetyDecision(False, "left guard moved during cut")
        if phase == "HAND_SHIFT" and (
            not value.right_target_stationary
            or value.right_speed_rad_s > 0.05
            or value.knife_height_m < value.safe_knife_height_m
        ):
            return SafetyDecision(
                False, "knife is not safely raised for hand shift"
            )
        return SafetyDecision(True)
