from __future__ import annotations

from dataclasses import dataclass
from importlib import import_module
from pathlib import Path
from typing import Mapping, Sequence

from cook_core.interfaces import (
    JointTrajectoryData,
    PlanRequest,
    PlanResult,
    TrajectoryPointData,
)
from cook_description.models import load_robot_definition
from cook_mujoco.control import MujocoRuntime
from cook_mujoco.control.runtime import MujocoJointError
from cook_mujoco.planning.collision import MujocoCollisionChecker


class OmplUnavailableError(RuntimeError):
    pass


@dataclass
class OmplJointPlanner:
    runtime: MujocoRuntime
    joint_limits: Mapping[str, tuple[float | None, float | None]]
    collision_check_enabled: bool = True
    allow_initial_contacts: bool = True
    default_lower_bound: float = -3.141592653589793
    default_upper_bound: float = 3.141592653589793

    def plan(self, request: PlanRequest) -> PlanResult:
        ob, og = _load_ompl()
        joint_names = tuple(request.joint_names)
        start_positions = request.start_state.as_mapping()
        goal_positions = request.goal_state.as_mapping()
        self._validate_request_positions(request)

        space = ob.RealVectorStateSpace(len(joint_names))
        bounds = ob.RealVectorBounds(len(joint_names))
        for index, name in enumerate(joint_names):
            lower, upper = self._joint_bounds(name)
            bounds.setLow(index, lower)
            bounds.setHigh(index, upper)
        space.setBounds(bounds)

        checker = (
            MujocoCollisionChecker(
                self.runtime,
                joint_names,
                reference_positions=start_positions,
                allow_initial_contacts=self.allow_initial_contacts,
            )
            if self.collision_check_enabled
            else None
        )
        setup = og.SimpleSetup(space)
        setup.setStateValidityChecker(
            _make_state_validity_checker(
                ob,
                setup.getSpaceInformation(),
                lambda state: True
                if checker is None
                else checker.is_state_valid(_state_to_mapping(state, joint_names)),
            )
        )
        setup.getSpaceInformation().setStateValidityCheckingResolution(
            request.collision_check_resolution
        )
        setup.setPlanner(og.RRTConnect(setup.getSpaceInformation()))

        start = _make_state(ob, space)
        goal = _make_state(ob, space)
        for index, name in enumerate(joint_names):
            _set_state_value(start, index, start_positions[name])
            _set_state_value(goal, index, goal_positions[name])
        setup.setStartAndGoalStates(start, goal)

        solved = bool(setup.solve(request.planning_time_sec))
        if not solved:
            return PlanResult(
                trajectory=JointTrajectoryData(
                    joint_names=joint_names,
                    points=(),
                    source="ompl",
                    metadata={
                        "allowed_body_pairs": (
                            [] if checker is None else sorted(checker.allowed_body_pairs)
                        ),
                        "collision_check_enabled": self.collision_check_enabled,
                    },
                ),
                success=False,
                message="OMPL failed to find a collision-free path",
                planner_name="ompl_rrt_connect",
            )

        trajectory = _trajectory_from_solution(
            setup.getSolutionPath(),
            joint_names=joint_names,
            duration_sec=request.duration_sec,
            waypoint_count=request.waypoint_count,
        )
        return PlanResult(
            trajectory=trajectory,
            success=True,
            message="",
            planner_name="ompl_rrt_connect",
            metadata={
                "allowed_body_pairs": (
                    [] if checker is None else sorted(checker.allowed_body_pairs)
                ),
                "collision_check_enabled": self.collision_check_enabled,
            },
        )

    def _validate_request_positions(self, request: PlanRequest) -> None:
        start = request.start_state.as_mapping()
        goal = request.goal_state.as_mapping()
        for name in request.joint_names:
            if name not in start:
                raise MujocoJointError(f"start_positions missing joint: {name}")
            if name not in goal:
                raise MujocoJointError(f"goal_positions missing joint: {name}")
            lower, upper = self._joint_bounds(name)
            if not lower <= float(start[name]) <= upper:
                raise MujocoJointError(f"start position is outside joint bounds: {name}")
            if not lower <= float(goal[name]) <= upper:
                raise MujocoJointError(f"goal position is outside joint bounds: {name}")

    def _joint_bounds(self, name: str) -> tuple[float, float]:
        lower, upper = self.joint_limits.get(name, (None, None))
        low = self.default_lower_bound if lower is None else float(lower)
        high = self.default_upper_bound if upper is None else float(upper)
        if low > high:
            raise MujocoJointError(f"invalid joint bounds: {name}")
        return low, high


def create_mujoco_ompl_planner(
    *,
    model_path: str | Path,
    urdf_path: str | Path,
    collision_check_enabled: bool = True,
    allow_initial_contacts: bool = True,
) -> OmplJointPlanner:
    definition = load_robot_definition(urdf_path)
    runtime = MujocoRuntime.load(model_path, definition.movable_joint_names)
    return OmplJointPlanner(
        runtime=runtime,
        joint_limits=definition.joint_limits,
        collision_check_enabled=collision_check_enabled,
        allow_initial_contacts=allow_initial_contacts,
    )


def _load_ompl():
    try:
        ompl = import_module("ompl")
        return ompl.base, ompl.geometric
    except Exception as exc:
        try:
            return import_module("ompl.base"), import_module("ompl.geometric")
        except Exception as nested_exc:
            raise OmplUnavailableError(
                "OMPL Python bindings are required for OmplJointPlanner. "
                "Install them with `pip install ompl`."
            ) from nested_exc or exc


def _trajectory_from_solution(
    path,
    *,
    joint_names: tuple[str, ...],
    duration_sec: float,
    waypoint_count: int,
) -> JointTrajectoryData:
    count = max(2, int(waypoint_count))
    if hasattr(path, "interpolate"):
        path.interpolate(count)
    state_count = int(path.getStateCount())
    if state_count <= 0:
        return JointTrajectoryData(joint_names=joint_names, points=(), source="ompl")
    points = []
    for index in range(state_count):
        ratio = 0.0 if state_count == 1 else index / (state_count - 1)
        state = path.getState(index)
        points.append(
            TrajectoryPointData(
                positions=_state_to_mapping(state, joint_names),
                time_from_start_sec=float(duration_sec) * ratio,
            )
        )
    return JointTrajectoryData(
        joint_names=joint_names,
        points=tuple(points),
        source="ompl",
    )


def _make_state_validity_checker(ob, space_information, callback):
    if hasattr(ob, "StateValidityCheckerFn"):
        return ob.StateValidityCheckerFn(callback)

    class CallbackStateValidityChecker(ob.StateValidityChecker):
        def __init__(self, si):
            super().__init__(si)

        def isValid(self, state) -> bool:
            return bool(callback(state))

    return CallbackStateValidityChecker(space_information)


def _make_state(ob, space):
    try:
        return ob.State(space)
    except Exception:
        return space.allocState()


def _state_to_mapping(state, joint_names: Sequence[str]) -> dict[str, float]:
    return {
        name: float(_state_value(state, index))
        for index, name in enumerate(joint_names)
    }


def _state_value(state, index: int) -> float:
    try:
        return state[index]
    except TypeError:
        return state()[index]


def _set_state_value(state, index: int, value: float) -> None:
    try:
        state[index] = float(value)
    except TypeError:
        state()[index] = float(value)
