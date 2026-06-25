from __future__ import annotations

from collections.abc import Callable, Iterable
from typing import Any

from cook_core.planning.interface import Planner, PlanningError
from cook_core.planning.linear import LinearJointPlanner


PlannerFactory = Callable[..., Planner]


class PlannerRegistryError(PlanningError):
    pass


class PlannerRegistry:
    """Map configuration names to planner factories."""

    def __init__(self) -> None:
        self._factories: dict[str, PlannerFactory] = {}

    @property
    def names(self) -> tuple[str, ...]:
        return tuple(sorted(self._factories))

    def register(
        self,
        name: str,
        factory: PlannerFactory,
        *,
        replace: bool = False,
    ) -> None:
        key = _planner_name(name)
        if not callable(factory):
            raise PlannerRegistryError(f"planner factory must be callable: {key}")
        if key in self._factories and not replace:
            raise PlannerRegistryError(f"planner is already registered: {key}")
        self._factories[key] = factory

    def register_aliases(
        self,
        names: Iterable[str],
        factory: PlannerFactory,
        *,
        replace: bool = False,
    ) -> None:
        for name in names:
            self.register(name, factory, replace=replace)

    def create(self, name: str, **config: Any) -> Planner:
        key = _planner_name(name)
        factory = self._factories.get(key)
        if factory is None:
            available = ", ".join(self.names) or "<none>"
            raise PlannerRegistryError(
                f"unknown planner '{key}'; available planners: {available}"
            )
        try:
            planner = factory(**config)
        except PlannerRegistryError:
            raise
        except Exception as exc:
            raise PlannerRegistryError(
                f"failed to create planner '{key}': {exc}"
            ) from exc
        if not callable(getattr(planner, "plan", None)):
            raise PlannerRegistryError(
                f"planner factory '{key}' returned an object without plan()"
            )
        return planner


def create_default_planner_registry() -> PlannerRegistry:
    registry = PlannerRegistry()
    registry.register("linear", LinearJointPlanner)
    return registry


def _planner_name(name: str) -> str:
    key = str(name).strip().lower()
    if not key:
        raise PlannerRegistryError("planner name must not be empty")
    return key


__all__ = [
    "PlannerFactory",
    "PlannerRegistry",
    "PlannerRegistryError",
    "create_default_planner_registry",
]
