"""Backend-independent dual-arm primitives for twin_joint_ws."""

from twin_core.arms import ArmId, ArmSpec, all_arm_specs, arm_spec
from twin_core.trajectory import finite_vector, validate_waypoints

__all__ = [
    "ArmId",
    "ArmSpec",
    "all_arm_specs",
    "arm_spec",
    "finite_vector",
    "validate_waypoints",
]
