import sys
import types

import pytest

from cook_core.planning import PlanRequest
from cook_description.models import load_robot_definition
from cook_description.paths import DEFAULT_MJCF_PATH, DEFAULT_URDF_PATH
from cook_mujoco.control import MujocoRuntime
from cook_mujoco.planning import (
    MujocoCollisionChecker,
    OmplJointPlanner,
    OmplUnavailableError,
)


def _runtime():
    definition = load_robot_definition(DEFAULT_URDF_PATH)
    return MujocoRuntime.load(DEFAULT_MJCF_PATH, definition.movable_joint_names)


def _request():
    definition = load_robot_definition(DEFAULT_URDF_PATH)
    joint_names = definition.movable_joint_names
    start = {name: 0.0 for name in joint_names}
    goal = {name: 0.05 for name in joint_names}
    return PlanRequest.from_position_mappings(
        start_positions=start,
        goal_positions=goal,
        joint_names=joint_names,
        duration_sec=1.0,
        waypoint_count=4,
        planning_time_sec=0.01,
    )


def test_collision_checker_allows_initial_contacts_by_default():
    request = _request()
    checker = MujocoCollisionChecker(
        _runtime(),
        request.joint_names,
        reference_positions=request.start_state.as_mapping(),
    )

    assert checker.initial_contact_body_pairs
    assert checker.is_state_valid(request.start_state.as_mapping())


def test_collision_checker_can_reject_existing_contacts():
    request = _request()
    checker = MujocoCollisionChecker(
        _runtime(),
        request.joint_names,
        reference_positions=request.start_state.as_mapping(),
        allow_initial_contacts=False,
    )

    assert not checker.is_state_valid(request.start_state.as_mapping())


def test_ompl_planner_reports_missing_optional_dependency(monkeypatch):
    monkeypatch.setitem(sys.modules, "ompl", None)
    planner = OmplJointPlanner(runtime=_runtime(), joint_limits={})

    with pytest.raises(OmplUnavailableError, match="pip install ompl"):
        planner.plan(_request())


def test_ompl_planner_converts_solution_to_joint_trajectory(monkeypatch):
    fake_ompl = _install_fake_ompl(monkeypatch, has_checker_fn=True)
    request = _request()
    planner = OmplJointPlanner(
        runtime=_runtime(),
        joint_limits={
            name: (-1.0, 1.0)
            for name in request.joint_names
        },
    )

    result = planner.plan(request)

    assert result.success
    assert result.planner_name == "ompl_rrt_connect"
    assert result.trajectory.source == "ompl"
    assert len(result.points) == request.waypoint_count
    assert result.points[0].positions == request.start_state.as_mapping()
    assert result.points[-1].positions == request.goal_state.as_mapping()
    assert fake_ompl["resolution"] == pytest.approx(request.collision_check_resolution)


def test_ompl_planner_supports_state_validity_checker_subclass(monkeypatch):
    fake_ompl = _install_fake_ompl(monkeypatch, has_checker_fn=False)
    request = _request()
    planner = OmplJointPlanner(
        runtime=_runtime(),
        joint_limits={
            name: (-1.0, 1.0)
            for name in request.joint_names
        },
    )

    result = planner.plan(request)

    assert result.success
    assert fake_ompl["resolution"] == pytest.approx(request.collision_check_resolution)


def _install_fake_ompl(monkeypatch, *, has_checker_fn: bool):
    state_validity = {}
    result = {"resolution": None}

    class FakeBounds:
        def __init__(self, dimension):
            self.low = [0.0] * dimension
            self.high = [0.0] * dimension

        def setLow(self, index, value):
            self.low[index] = float(value)

        def setHigh(self, index, value):
            self.high[index] = float(value)

    class FakeSpace:
        def __init__(self, dimension):
            self.dimension = dimension
            self.bounds = None

        def setBounds(self, bounds):
            self.bounds = bounds

    class FakeState:
        def __init__(self, space):
            self.values = [0.0] * space.dimension

        def __call__(self):
            return self

        def __getitem__(self, index):
            return self.values[index]

        def __setitem__(self, index, value):
            self.values[index] = float(value)

    class FakeStateValidityCheckerFn:
        def __init__(self, callback):
            state_validity["callback"] = callback
            self.callback = callback

        def __call__(self, state):
            return self.callback(state)

    class FakeStateValidityChecker:
        def __init__(self, _space_information):
            pass

    class FakeSpaceInformation:
        def setStateValidityCheckingResolution(self, value):
            result["resolution"] = float(value)

    class FakePath:
        def __init__(self, setup):
            self.setup = setup
            self.states = [setup.start.values, setup.goal.values]

        def interpolate(self, count):
            start = self.setup.start.values
            goal = self.setup.goal.values
            self.states = []
            for index in range(count):
                ratio = index / max(1, count - 1)
                self.states.append([
                    start_value + (goal_value - start_value) * ratio
                    for start_value, goal_value in zip(start, goal)
                ])

        def getStateCount(self):
            return len(self.states)

        def getState(self, index):
            return self.states[index]

    class FakeSimpleSetup:
        def __init__(self, space):
            self.space = space
            self.start = None
            self.goal = None
            self.space_information = FakeSpaceInformation()

        def setStateValidityChecker(self, checker):
            self.checker = checker

        def getSpaceInformation(self):
            return self.space_information

        def setPlanner(self, planner):
            self.planner = planner

        def setStartAndGoalStates(self, start, goal):
            self.start = start
            self.goal = goal

        def solve(self, _time_sec):
            if hasattr(self.checker, "isValid"):
                return self.checker.isValid(self.start.values) and self.checker.isValid(
                    self.goal.values
                )
            return self.checker(self.start.values) and self.checker(self.goal.values)

        def getSolutionPath(self):
            return FakePath(self)

    class FakeRRTConnect:
        def __init__(self, _space_information):
            pass

    base_values = {
        "RealVectorStateSpace": FakeSpace,
        "RealVectorBounds": FakeBounds,
        "State": FakeState,
        "StateValidityChecker": FakeStateValidityChecker,
    }
    if has_checker_fn:
        base_values["StateValidityCheckerFn"] = FakeStateValidityCheckerFn
    base = types.SimpleNamespace(
        **base_values,
    )
    geometric = types.SimpleNamespace(
        SimpleSetup=FakeSimpleSetup,
        RRTConnect=FakeRRTConnect,
    )
    package = types.SimpleNamespace(base=base, geometric=geometric)
    monkeypatch.setitem(sys.modules, "ompl", package)
    monkeypatch.setitem(sys.modules, "ompl.base", base)
    monkeypatch.setitem(sys.modules, "ompl.geometric", geometric)
    return result
