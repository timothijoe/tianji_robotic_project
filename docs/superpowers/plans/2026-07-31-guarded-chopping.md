# Guarded Dual-Arm Chopping Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a MuJoCo-only five-cut demonstration in which the right arm chops a fixed cube while the left arm and Wuji Hand hold a cat-paw guard pose and retreat 0.02 m after each raised knife.

**Architecture:** Add a task-local fixed vegetable proxy and a named knuckle reference site, then generate right-knife and left-guard trajectories during preflight. A dedicated `SafetyCoordinator` gates the deterministic state machine, while a separate trace class owns Viewer markers and overlay text. Existing `line-chop`, pick-place, ROS2, and real-robot paths remain unchanged.

**Tech Stack:** Python 3.10+, NumPy, MuJoCo Python bindings, pytest, existing `twin_sim` position-control/IK/trajectory utilities.

## Global Constraints

- Execute five cuts and four left-hand shifts.
- Each shift is exactly `0.02 m`; total shift is `0.08 m`.
- The fixed cube pose must not change.
- The knife-to-guard-knuckle distance must remain at least `0.02 m`.
- The left arm may shift only while the knife is at safe height.
- The knife may descend only while the left arm and hand targets are stationary.
- Use MuJoCo position actuators only; do not add impedance, admittance, weld, mocap, or runtime attachment.
- Do not send commands to real hardware and do not modify protected real-robot files.
- Phase C is limited to the `SafetyCoordinator` interface; dynamic recovery is not implemented.

---

## File Structure

- Modify `robot_assets/mujoco/right_chopping_scene.xml`: add a default-hidden fixed vegetable proxy.
- Modify `robot_assets/mujoco/wuji_hand/left_hand.xml`: add a named guard-knuckle site to the second phalanx chosen for knife clearance.
- Modify `src/twin_sim/model.py`: validate and expose required scene names through existing `require_*` methods.
- Create `src/twin_sim/tasks/guarded_chop.py`: configuration, samples/results, preflight trajectories, deterministic state machine, and task execution.
- Create `src/twin_sim/guarded_chop_safety.py`: pure safety decision types and `SafetyCoordinator`.
- Create `src/twin_sim/guarded_chop_visualization.py`: four-color Viewer trails and status overlay.
- Modify `src/twin_sim/tasks/__init__.py`: export guarded-chop public types.
- Modify `src/twin_sim/cli.py`: add the `guarded-chop` command.
- Create focused tests under `tests/simulation/` for scene, posture, safety, task integration, visualization, and CLI.
- Modify `docs/simulation/usage.md`: document operation and visual legend.
- Create `docs/simulation/guarded_chopping_development_log.md`: record tuning, metrics, and limitations.

---

### Task 1: Task-local fixed vegetable and guard reference

**Files:**
- Modify: `robot_assets/mujoco/right_chopping_scene.xml`
- Modify: `robot_assets/mujoco/wuji_hand/left_hand.xml`
- Modify: `src/twin_sim/model.py`
- Create: `tests/simulation/test_guarded_chop_scene.py`

**Interfaces:**
- Consumes: `SimulationModel.require_geom(name)` and `SimulationModel.require_site(name)`.
- Produces: geom `guarded_chop_cube`, site `guarded_chop_cube_center`, and site `left_guard_knuckle_site`.

- [ ] **Step 1: Write failing scene-contract tests**

```python
import mujoco
import numpy as np

from twin_sim.model import SimulationModel


def test_guarded_chop_proxy_is_fixed_and_hidden_by_default():
    sim = SimulationModel.load()
    geom = sim.require_geom("guarded_chop_cube")
    body = int(sim.model.geom_bodyid[geom])
    assert sim.model.body_jntnum[body] == 0
    assert sim.model.geom_rgba[geom, 3] == 0.0
    assert sim.model.geom_contype[geom] == 0
    assert sim.model.geom_conaffinity[geom] == 0


def test_guard_knuckle_site_belongs_to_left_hand():
    sim = SimulationModel.load()
    site = sim.require_site("left_guard_knuckle_site")
    body = int(sim.model.site_bodyid[site])
    name = mujoco.mj_id2name(sim.model, mujoco.mjtObj.mjOBJ_BODY, body)
    assert name.startswith("left_finger")
    assert np.isfinite(sim.data.site_xpos[site]).all()
```

- [ ] **Step 2: Run tests and verify missing names fail**

Run: `.venv/bin/pytest tests/simulation/test_guarded_chop_scene.py -q`
Expected: FAIL because `guarded_chop_cube` and `left_guard_knuckle_site` do not exist.

- [ ] **Step 3: Add the hidden fixed proxy and knuckle site**

Add a world-attached cube whose top rests above `chopping_board`; keep it inactive for all existing tasks:

```xml
<body name="guarded_chop_cube_body" pos="0.62 0.02 0.28">
  <geom name="guarded_chop_cube" type="box" size="0.08 0.05 0.05"
        rgba="0.75 0.18 0.12 0" contype="0" conaffinity="0"/>
  <site name="guarded_chop_cube_center" size="0.004" rgba="0 0 0 0"/>
</body>
```

Add the reference site to the selected proximal/second phalanx body in `left_hand.xml`; tune only the local position so it sits on the knife-facing outer knuckle:

```xml
<site name="left_guard_knuckle_site" pos="0 0 0.018"
      size="0.004" rgba="0.75 0.1 0.9 0"/>
```

Extend `SimulationModel.load()` validation:

```python
simulation.require_geom("guarded_chop_cube")
simulation.require_site("guarded_chop_cube_center")
simulation.require_site("left_guard_knuckle_site")
```

- [ ] **Step 4: Run scene and existing model tests**

Run: `.venv/bin/pytest tests/simulation/test_guarded_chop_scene.py tests/simulation/test_model.py -q`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add robot_assets/mujoco/right_chopping_scene.xml robot_assets/mujoco/wuji_hand/left_hand.xml src/twin_sim/model.py tests/simulation/test_guarded_chop_scene.py
git commit -m "feat: add guarded chopping scene references"
```

---

### Task 2: Cat-paw hand target and safety coordinator

**Files:**
- Create: `src/twin_sim/guarded_chop_safety.py`
- Create: `tests/simulation/test_guarded_chop_safety.py`
- Create: `tests/simulation/test_guarded_chop_posture.py`

**Interfaces:**
- Consumes: NumPy vectors and string/enum phase values.
- Produces: `CAT_PAW_RAD: np.ndarray`, `SafetyObservation`, `SafetyDecision`, and `SafetyCoordinator.evaluate(observation) -> SafetyDecision`.

- [ ] **Step 1: Write failing posture and safety tests**

```python
import numpy as np

from twin_sim.guarded_chop_safety import (
    CAT_PAW_RAD,
    SafetyCoordinator,
    SafetyObservation,
)
from twin_sim.model import SimulationModel


def test_cat_paw_target_is_valid_and_tucks_thumb():
    sim = SimulationModel.load()
    ranges = sim.model.actuator_ctrlrange[sim.hand.actuator_ids]
    assert CAT_PAW_RAD.shape == (20,)
    assert np.all(CAT_PAW_RAD >= ranges[:, 0])
    assert np.all(CAT_PAW_RAD <= ranges[:, 1])
    assert CAT_PAW_RAD[0] > 0.6
    assert np.all(CAT_PAW_RAD[[2, 6, 10, 14, 18]] > 0.45)


def observation(**overrides):
    values = dict(
        phase="CUT_DOWN",
        knife_height_m=0.36,
        safe_knife_height_m=0.34,
        knife_guard_distance_m=0.03,
        left_target_stationary=True,
        right_target_stationary=False,
        finite_state=True,
    )
    values.update(overrides)
    return SafetyObservation(**values)


def test_cut_requires_stationary_guard_and_clearance():
    coordinator = SafetyCoordinator(minimum_distance_m=0.02)
    assert coordinator.evaluate(observation()).allowed
    assert not coordinator.evaluate(
        observation(left_target_stationary=False)
    ).allowed
    decision = coordinator.evaluate(
        observation(knife_guard_distance_m=0.019)
    )
    assert not decision.allowed
    assert "distance" in decision.reason


def test_shift_requires_raised_stationary_knife():
    coordinator = SafetyCoordinator(minimum_distance_m=0.02)
    allowed = observation(
        phase="HAND_SHIFT",
        knife_height_m=0.36,
        right_target_stationary=True,
        left_target_stationary=False,
    )
    assert coordinator.evaluate(allowed).allowed
    assert not coordinator.evaluate(
        observation(
            phase="HAND_SHIFT",
            knife_height_m=0.33,
            right_target_stationary=True,
            left_target_stationary=False,
        )
    ).allowed
```

- [ ] **Step 2: Run tests and verify imports fail**

Run: `.venv/bin/pytest tests/simulation/test_guarded_chop_safety.py tests/simulation/test_guarded_chop_posture.py -q`
Expected: FAIL with `ModuleNotFoundError: twin_sim.guarded_chop_safety`.

- [ ] **Step 3: Implement pure posture and safety types**

```python
from dataclasses import dataclass

import numpy as np


CAT_PAW_RAD = np.asarray(
    (0.85, -0.02, 0.62, 0.78)
    + (0.20, 0.00, 0.72, 0.82) * 4,
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
    finite_state: bool


@dataclass(frozen=True)
class SafetyDecision:
    allowed: bool
    reason: str = ""


class SafetyCoordinator:
    def __init__(self, minimum_distance_m: float = 0.02):
        if not np.isfinite(minimum_distance_m) or minimum_distance_m <= 0:
            raise ValueError("minimum_distance_m must be positive and finite")
        self.minimum_distance_m = float(minimum_distance_m)

    def evaluate(self, value: SafetyObservation) -> SafetyDecision:
        scalars = (
            value.knife_height_m,
            value.safe_knife_height_m,
            value.knife_guard_distance_m,
        )
        if not value.finite_state or not np.isfinite(scalars).all():
            return SafetyDecision(False, "non-finite simulation state")
        if value.knife_guard_distance_m < self.minimum_distance_m:
            return SafetyDecision(False, "knife-guard distance below limit")
        if value.phase == "CUT_DOWN" and not value.left_target_stationary:
            return SafetyDecision(False, "left guard moved during cut")
        if value.phase == "HAND_SHIFT" and (
            not value.right_target_stationary
            or value.knife_height_m < value.safe_knife_height_m
        ):
            return SafetyDecision(False, "knife is not safely raised for hand shift")
        return SafetyDecision(True)
```

- [ ] **Step 4: Tune `CAT_PAW_RAD` only within the tested contract**

Run a short headless pose script that commands the 20 targets, steps 2 s, and prints the knuckle site. Adjust the constants until the thumb is behind the four curled fingers and no hand joint is at a limit; do not change the public shape or test thresholds.

Run: `.venv/bin/pytest tests/simulation/test_guarded_chop_safety.py tests/simulation/test_guarded_chop_posture.py -q`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/twin_sim/guarded_chop_safety.py tests/simulation/test_guarded_chop_safety.py tests/simulation/test_guarded_chop_posture.py
git commit -m "feat: define guarded chopping safety contract"
```

---

### Task 3: Dual-arm preflight and state machine

**Files:**
- Create: `src/twin_sim/tasks/guarded_chop.py`
- Modify: `src/twin_sim/tasks/__init__.py`
- Create: `tests/simulation/test_guarded_chop_state_machine.py`
- Create: `tests/simulation/test_guarded_chop_preflight.py`

**Interfaces:**
- Consumes: `RightArmRobot`, `cartesian_trajectory`, `_preflight_line_chop`, `CAT_PAW_RAD`, and `SafetyCoordinator`.
- Produces: `GuardedChopPhase`, `GuardedChopConfig`, `GuardedChopSample`, `GuardedChopResult`, `_point_to_box_distance(point, center, rotation, half_size)`, `_preflight_guarded_chop(robot, config)`, and `run_guarded_chop(config, viewer=False, trace=None)`.

- [ ] **Step 1: Write failing state/config tests**

```python
import numpy as np
import pytest

from twin_sim.tasks.guarded_chop import (
    GuardedChopConfig,
    GuardedChopPhase,
    GuardedChopSample,
)


def test_guarded_chop_defaults_match_approved_motion():
    config = GuardedChopConfig()
    assert config.cuts == 5
    assert config.hand_shift_m == 0.02
    assert config.minimum_distance_m == 0.02
    assert config.final_hold_s == 10.0


@pytest.mark.parametrize("cuts,shift", [(0, 0.02), (5, 0.0), (6, np.nan)])
def test_invalid_config_is_rejected(cuts, shift):
    with pytest.raises(ValueError):
        GuardedChopConfig(cuts=cuts, hand_shift_m=shift).validated()


def test_phase_order_contains_interlocked_actions():
    assert [phase.value for phase in GuardedChopPhase] == [
        "initialize", "guard_ready", "cut_down", "knife_up",
        "hand_shift", "complete", "aborted",
    ]
```

- [ ] **Step 2: Write failing preflight geometry tests**

```python
import numpy as np

from twin_sim.robot import RightArmRobot
from twin_sim.tasks.guarded_chop import (
    GuardedChopConfig,
    _point_to_box_distance,
    _preflight_guarded_chop,
)


def test_preflight_builds_five_cuts_and_four_guard_shifts():
    robot = RightArmRobot(viewer=False)
    plan = _preflight_guarded_chop(robot, GuardedChopConfig())
    assert len(plan.cuts) == 5
    assert len(plan.guard_shifts) == 4
    deltas = np.diff(plan.guard_targets[:, :3, 3], axis=0)
    np.testing.assert_allclose(np.linalg.norm(deltas[:, :2], axis=1), 0.02)
    assert np.all(plan.minimum_planned_distances_m >= 0.02)
    robot.close()


def test_point_to_oriented_blade_box_distance_uses_surface_not_center():
    center = np.array((0.0, 0.0, 0.0))
    rotation = np.eye(3)
    half_size = np.array((0.10, 0.01, 0.05))
    assert np.isclose(
        _point_to_box_distance(
            np.array((0.0, 0.03, 0.0)), center, rotation, half_size
        ),
        0.02,
    )
```

- [ ] **Step 3: Run tests and verify the task module is missing**

Run: `.venv/bin/pytest tests/simulation/test_guarded_chop_state_machine.py tests/simulation/test_guarded_chop_preflight.py -q`
Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 4: Implement types and validation**

Use these public shapes:

```python
class GuardedChopPhase(Enum):
    INITIALIZE = "initialize"
    GUARD_READY = "guard_ready"
    CUT_DOWN = "cut_down"
    KNIFE_UP = "knife_up"
    HAND_SHIFT = "hand_shift"
    COMPLETE = "complete"
    ABORTED = "aborted"


@dataclass(frozen=True)
class GuardedChopConfig:
    cuts: int = 5
    hand_shift_m: float = 0.02
    minimum_distance_m: float = 0.02
    control_dt_s: float = 0.01
    guard_ready_duration_s: float = 2.0
    cut_duration_s: float = 1.0
    knife_up_duration_s: float = 1.0
    hand_shift_duration_s: float = 0.8
    final_hold_s: float = 10.0


@dataclass(frozen=True)
class GuardedChopSample:
    time_s: float
    phase: GuardedChopPhase
    cut_index: int
    left_target_rad: np.ndarray
    left_actual_rad: np.ndarray
    right_target_rad: np.ndarray
    right_actual_rad: np.ndarray
    hand_target_rad: np.ndarray
    knife_position: np.ndarray
    guard_position: np.ndarray
    knife_guard_distance_m: float
    knife_height_m: float
    cut_allowed: bool


@dataclass(frozen=True)
class GuardedChopResult:
    success: bool
    final_phase: GuardedChopPhase
    reason: str
    samples: tuple[GuardedChopSample, ...]
    completed_cuts: int
    completed_shifts: int
    total_shift_m: float
    minimum_distance_m: float
```

`validated()` must require exactly five positive cuts for this approved demo, positive finite durations, `hand_shift_m == 0.02`, and `minimum_distance_m == 0.02`.

- [ ] **Step 5: Implement preflight without moving MuJoCo state**

Reuse `_preflight_line_chop` for five right-arm cut/retract pairs with `spacing_m=0.02`. Construct a left palm pose whose knuckle site sits beside the fixed cube and whose curled fingers point downward. Solve it with `robot.left_kinematics.ik`, then create four Cartesian translations parallel to the right knife's cut-spacing vector. Snapshot `qpos`, `qvel`, `ctrl`, and time before preflight and restore them in `finally`.

Represent the private plan with immutable tuples:

```python
@dataclass(frozen=True)
class _GuardedPreflight:
    right_ready_rad: np.ndarray
    left_ready_rad: np.ndarray
    cuts: tuple[_CutTrajectories, ...]
    guard_shifts: tuple[tuple[TrajectoryPoint, ...], ...]
    guard_targets: np.ndarray
    minimum_planned_distances_m: np.ndarray
    safe_knife_height_m: float
```

Calculate clearance from `left_guard_knuckle_site` to the surface of the oriented
`right_knife_blade` box, not to the knife TCP or blade center. Transform the point
to geom-local coordinates, clamp it to `[-geom_size, +geom_size]`, and take the
norm to the clamped point. Reject any preflight sample below `minimum_distance_m`
before commanding either arm.

- [ ] **Step 6: Export the public task API**

```python
from twin_sim.tasks.guarded_chop import (
    GuardedChopConfig,
    GuardedChopPhase,
    GuardedChopResult,
    run_guarded_chop,
)

__all__ += [
    "GuardedChopConfig", "GuardedChopPhase",
    "GuardedChopResult", "run_guarded_chop",
]
```

Run: `.venv/bin/pytest tests/simulation/test_guarded_chop_state_machine.py tests/simulation/test_guarded_chop_preflight.py -q`
Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add src/twin_sim/tasks/guarded_chop.py src/twin_sim/tasks/__init__.py tests/simulation/test_guarded_chop_state_machine.py tests/simulation/test_guarded_chop_preflight.py
git commit -m "feat: plan guarded dual-arm chopping"
```

---

### Task 4: Execute physical dual-arm coordination

**Files:**
- Modify: `src/twin_sim/tasks/guarded_chop.py`
- Create: `tests/simulation/test_guarded_chop_integration.py`

**Interfaces:**
- Consumes: Task 3 `_GuardedPreflight` and Task 2 `SafetyCoordinator`.
- Produces: working `run_guarded_chop(config, viewer=False, trace=None) -> GuardedChopResult`.

- [ ] **Step 1: Write the failing real-MuJoCo acceptance test**

```python
import numpy as np

from twin_sim.tasks.guarded_chop import GuardedChopConfig, GuardedChopPhase, run_guarded_chop


def test_guarded_chop_executes_five_cuts_and_four_safe_shifts():
    result = run_guarded_chop(
        GuardedChopConfig(final_hold_s=0.0), viewer=False
    )
    assert result.success, result.reason
    assert result.final_phase is GuardedChopPhase.COMPLETE
    assert result.completed_cuts == 5
    assert result.completed_shifts == 4
    assert np.isclose(result.total_shift_m, 0.08, atol=0.002)
    assert result.minimum_distance_m >= 0.02

    down = [s for s in result.samples if s.phase is GuardedChopPhase.CUT_DOWN]
    np.testing.assert_allclose(
        [s.left_target_rad for s in down], down[0].left_target_rad, atol=1e-12
    )
    shifts = [s for s in result.samples if s.phase is GuardedChopPhase.HAND_SHIFT]
    np.testing.assert_allclose(
        [s.right_target_rad for s in shifts], shifts[0].right_target_rad, atol=1e-12
    )
```

Add a second test that snapshots `guarded_chop_cube_center` before/after and requires exact equality. Add a coordinator-injection test that forces a rejected decision and verifies `ABORTED`, both arm targets held, and the rejection reason propagated.

- [ ] **Step 2: Run integration tests and verify execution is absent**

Run: `.venv/bin/pytest tests/simulation/test_guarded_chop_integration.py -q -s`
Expected: FAIL because `run_guarded_chop` does not execute the plan yet.

- [ ] **Step 3: Implement task-local scene activation**

After constructing `RightArmRobot`, set only that model instance:

```python
cube = robot.sim.require_geom("guarded_chop_cube")
robot.sim.model.geom_rgba[cube] = (0.75, 0.18, 0.12, 1.0)
robot.sim.model.geom_contype[cube] = 1
robot.sim.model.geom_conaffinity[cube] = 1
for name in ("pick_source_pedestal", "pick_target_pedestal", "pick_cube_geom", "pick_target_region"):
    geom = robot.sim.require_geom(name)
    robot.sim.model.geom_rgba[geom, 3] = 0.0
    robot.sim.model.geom_contype[geom] = 0
    robot.sim.model.geom_conaffinity[geom] = 0
```

Do not edit qpos for the fixed cube; it has no joint.

- [ ] **Step 4: Implement the interlocked execution loop**

Initialize both arm qpos/targets to preflight ready poses only once after reset, command `CAT_PAW_RAD`, and step through `GUARD_READY`. For every trajectory point:

1. Set the moving arm target and keep the other target unchanged.
2. Step exactly `control_dt_s`.
3. Read the blade geom transform and knuckle site; compute point-to-oriented-box
   surface distance with `_point_to_box_distance`.
4. Build `SafetyObservation` from actual geometry and target deltas.
5. Abort immediately when `SafetyDecision.allowed` is false.
6. Append one immutable `GuardedChopSample`.

The loop order must be `CUT_DOWN`, `KNIFE_UP`, and—except after cut five—`HAND_SHIFT`. Increment counters only after the corresponding trajectory completes. On abort, retain the current left/right/hand targets; never return either arm to home automatically.

- [ ] **Step 5: Enforce the final contract from recorded samples**

Before returning success, require:

```python
completed_cuts == 5
completed_shifts == 4
np.isclose(total_shift_m, 0.08, atol=0.002)
min(sample.knife_guard_distance_m for sample in samples) >= 0.02
all(np.isfinite(sample.left_actual_rad).all() for sample in samples)
all(np.isfinite(sample.right_actual_rad).all() for sample in samples)
```

Also compare the fixed cube site's position and orientation before/after with `np.testing.assert_array_equal`.

- [ ] **Step 6: Run integration and relevant regression tests**

Run: `.venv/bin/pytest tests/simulation/test_guarded_chop_integration.py tests/simulation/test_line_chop.py tests/simulation/test_pick_place_integration.py -q -s`
Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add src/twin_sim/tasks/guarded_chop.py tests/simulation/test_guarded_chop_integration.py
git commit -m "feat: execute guarded dual-arm chopping"
```

---

### Task 5: Viewer trails and live safety overlay

**Files:**
- Create: `src/twin_sim/guarded_chop_visualization.py`
- Create: `tests/simulation/test_guarded_chop_visualization.py`
- Modify: `src/twin_sim/tasks/guarded_chop.py`

**Interfaces:**
- Consumes: Viewer handle and per-step positions/status from `run_guarded_chop`.
- Produces: `GuardedChopTrace.append(...)`, `GuardedChopTrace.set_abort(reason)`, and approved marker colors.

- [ ] **Step 1: Write failing visualization tests with a fake Viewer**

```python
import numpy as np

from twin_sim.guarded_chop_visualization import GuardedChopTrace


def test_trace_uses_approved_colors_and_overlay():
    viewer = FakeViewer(maxgeom=16)
    trace = GuardedChopTrace(viewer, marker_stride=1)
    trace.append(
        planned_knife=np.array((0.6, 0.0, 0.4)),
        actual_knife=np.array((0.6, 0.0, 0.39)),
        actual_guard=np.array((0.6, 0.04, 0.36)),
        guard_target=np.array((0.6, 0.04, 0.36)),
        phase="cut_down",
        cut_index=2,
        minimum_distance_m=0.025,
        cut_allowed=True,
    )
    assert viewer.user_scn.ngeom == 4
    np.testing.assert_allclose(viewer.user_scn.geoms[0].rgba, (0.1, 0.35, 1.0, 0.9))
    np.testing.assert_allclose(viewer.user_scn.geoms[1].rgba, (0.0, 0.9, 1.0, 0.9))
    np.testing.assert_allclose(viewer.user_scn.geoms[2].rgba, (0.75, 0.1, 0.9, 0.95))
    np.testing.assert_allclose(viewer.user_scn.geoms[3].rgba, (1.0, 0.85, 0.1, 0.95))
    assert "cut=2/5" in viewer.texts[-1][-1]
    assert "distance=0.025" in viewer.texts[-1][-1]
```

- [ ] **Step 2: Run and verify the module is missing**

Run: `.venv/bin/pytest tests/simulation/test_guarded_chop_visualization.py -q`
Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Implement bounded traces and overlay**

Follow `PickPlaceTrace` locking and `_init_sphere_geom` usage. Store deques with `maxlen=3000`; draw blue planned knife, cyan actual knife, purple actual guard, and yellow guard target. Update Viewer text on every append:

```python
status = (
    f"cut={cut_index}/5  phase={phase}  "
    f"distance={minimum_distance_m:.3f} m  "
    f"cut_allowed={cut_allowed}"
)
viewer.set_texts((None, None, "Guarded chop", status))
```

`set_abort(reason)` must replace the phase with `aborted` and include the exact reason.

- [ ] **Step 4: Feed trace from every task sample**

Call `trace.append(...)` only after the sample passes safety validation. On abort call `trace.set_abort(reason)` before returning the result. Ensure headless operation passes `trace=None` and allocates no Viewer geometry.

Run: `.venv/bin/pytest tests/simulation/test_guarded_chop_visualization.py tests/simulation/test_guarded_chop_integration.py -q`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/twin_sim/guarded_chop_visualization.py src/twin_sim/tasks/guarded_chop.py tests/simulation/test_guarded_chop_visualization.py
git commit -m "feat: visualize guarded chopping coordination"
```

---

### Task 6: CLI, documentation, and protected regression

**Files:**
- Modify: `src/twin_sim/cli.py`
- Modify: `tests/simulation/test_cli.py`
- Modify: `docs/simulation/usage.md`
- Create: `docs/simulation/guarded_chopping_development_log.md`

**Interfaces:**
- Consumes: `run_guarded_chop`, `GuardedChopConfig`, and `GuardedChopTrace`.
- Produces: `twin-sim guarded-chop [--headless] [--final-hold SECONDS]`.

- [ ] **Step 1: Write the failing CLI mapping test**

```python
def test_guarded_chop_cli_prints_coordination_metrics(monkeypatch, capsys):
    monkeypatch.setattr(cli, "run_guarded_chop", fake_guarded_chop)
    status = cli.main(["guarded-chop", "--headless", "--final-hold", "0"])
    assert status == 0
    output = capsys.readouterr().out
    assert "cuts=5" in output
    assert "shifts=4" in output
    assert "total_shift_m=0.080" in output
    assert "min_distance_m=" in output
```

- [ ] **Step 2: Run and verify parser failure**

Run: `.venv/bin/pytest tests/simulation/test_cli.py::test_guarded_chop_cli_prints_coordination_metrics -q`
Expected: FAIL because `guarded-chop` is not registered.

- [ ] **Step 3: Add CLI parser and execution branch**

Register:

```python
guarded_chop = commands.add_parser("guarded-chop")
guarded_chop.add_argument("--headless", action="store_true")
guarded_chop.add_argument("--final-hold", type=float, default=10.0)
```

Create `GuardedChopTrace` only for Viewer mode. Print exactly:

```python
print(
    f"success={result.success} cuts={result.completed_cuts} "
    f"shifts={result.completed_shifts} "
    f"total_shift_m={result.total_shift_m:.3f} "
    f"min_distance_m={result.minimum_distance_m:.3f} "
    f"reason={result.reason or '-'}"
)
```

Return exit code 0 on success and 1 on abort.

- [ ] **Step 4: Document usage and development evidence**

Add to `docs/simulation/usage.md`:

```bash
.venv/bin/twin-sim guarded-chop --headless --final-hold 0
.venv/bin/twin-sim guarded-chop --final-hold 10
```

Document the four marker colors, fixed-cube behavior, five-cut/four-shift rhythm, 0.02 m minimum clearance, and MuJoCo-only warning. In the development log record the final tuned cat-paw vector, left/right ready joints, measured minimum distance, actual shift distances, IK issues and their resolution, all verification commands, and known Phase C limitations.

- [ ] **Step 5: Run CLI and full verification**

Run:

```bash
.venv/bin/pytest -q
.venv/bin/twin-sim guarded-chop --headless --final-hold 0
sha256sum -c docs/simulation/protected-files.sha256
git diff --check
```

Expected:

- all tests pass, with only explicitly documented pre-existing warnings;
- CLI reports `success=True cuts=5 shifts=4 total_shift_m=0.080` and `min_distance_m>=0.020`;
- every protected real-robot file reports `OK`;
- `git diff --check` has no output.

- [ ] **Step 6: Run the Viewer acceptance once**

Run: `.venv/bin/twin-sim guarded-chop --final-hold 10`
Expected: visible sequence `CUT_DOWN → KNIFE_UP → HAND_SHIFT`, repeated five times; no gray pick-place pedestals; fixed red cube stays on the board; overlay ends at `cut=5/5 phase=complete`.

- [ ] **Step 7: Commit**

```bash
git add src/twin_sim/cli.py tests/simulation/test_cli.py docs/simulation/usage.md docs/simulation/guarded_chopping_development_log.md
git commit -m "feat: expose guarded chopping demonstration"
```

---

### Task 7: Final specification review

**Files:**
- Review: `docs/superpowers/specs/2026-07-31-guarded-chopping-design.md`
- Review: all files changed by Tasks 1–6

**Interfaces:**
- Consumes: complete implementation and verification evidence.
- Produces: review findings resolved before branch integration.

- [ ] **Step 1: Request a specification and safety review**

Ask the reviewer to verify every acceptance bullet, with special attention to actual knife/knuckle geometry, the 0.02 m limit, motion interlocks, fixed cube invariance, and absence of real-hardware paths.

- [ ] **Step 2: Resolve all Critical and Important findings with TDD**

For each accepted finding, first add a focused failing test, run it to prove failure, implement the smallest correction, and rerun both focused and full suites. Do not weaken numeric thresholds to make tests pass.

- [ ] **Step 3: Repeat final verification after the last code change**

Run:

```bash
.venv/bin/pytest -q
.venv/bin/twin-sim guarded-chop --headless --final-hold 0
sha256sum -c docs/simulation/protected-files.sha256
git diff --check
git status --short
```

Expected: tests and CLI pass, hashes are all `OK`, diff check is silent, and the worktree is clean after the final commit.
