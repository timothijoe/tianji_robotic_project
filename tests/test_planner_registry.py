import pytest

from cook_core.planning import (
    LinearJointPlanner,
    PlannerRegistry,
    PlannerRegistryError,
    create_default_planner_registry,
)


def test_default_registry_creates_linear_planner_case_insensitively():
    registry = create_default_planner_registry()

    planner = registry.create(" Linear ")

    assert isinstance(planner, LinearJointPlanner)
    assert registry.names == ("linear",)


def test_registry_supports_external_planner_factories_and_configuration():
    registry = PlannerRegistry()
    registry.register("custom", lambda *, scale: _CustomPlanner(scale))

    planner = registry.create("custom", scale=2.5)

    assert planner.scale == pytest.approx(2.5)


def test_registry_reports_duplicate_and_unknown_names():
    registry = create_default_planner_registry()

    with pytest.raises(PlannerRegistryError, match="already registered"):
        registry.register("linear", LinearJointPlanner)
    with pytest.raises(
        PlannerRegistryError,
        match="available planners: linear",
    ):
        registry.create("missing")


class _CustomPlanner:
    def __init__(self, scale):
        self.scale = scale

    def plan(self, request):
        raise NotImplementedError
