from pathlib import Path

from cook_core.planning import (
    Planner,
    PlannerRegistry,
    create_default_planner_registry,
)
from cook_mujoco.planning.collision import MujocoCollisionChecker
from cook_mujoco.planning.ompl import (
    OmplJointPlanner,
    OmplUnavailableError,
    create_mujoco_ompl_planner,
)


def create_mujoco_planner_registry(
    *,
    model_path: str | Path,
    urdf_path: str | Path,
    collision_check_enabled: bool = True,
    allow_initial_contacts: bool = True,
) -> PlannerRegistry:
    registry = create_default_planner_registry()
    registry.register(
        "ompl",
        lambda: create_mujoco_ompl_planner(
            model_path=model_path,
            urdf_path=urdf_path,
            collision_check_enabled=collision_check_enabled,
            allow_initial_contacts=allow_initial_contacts,
        ),
    )
    return registry


def create_planner(
    planner_type: str,
    *,
    model_path: str | Path,
    urdf_path: str | Path,
    collision_check_enabled: bool = True,
    allow_initial_contacts: bool = True,
) -> Planner:
    registry = create_mujoco_planner_registry(
        model_path=model_path,
        urdf_path=urdf_path,
        collision_check_enabled=collision_check_enabled,
        allow_initial_contacts=allow_initial_contacts,
    )
    return registry.create(planner_type)


__all__ = [
    "MujocoCollisionChecker",
    "OmplJointPlanner",
    "OmplUnavailableError",
    "create_mujoco_planner_registry",
    "create_mujoco_ompl_planner",
    "create_planner",
]
