# Left Arm + Wuji Hand Pick-and-Place Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the MuJoCo left arm and Wuji Hand pick up a free 5 cm cube through real contact and friction, carry it at least 15 cm, and place it in a target region.

**Architecture:** Generalize the existing arm kinematics and robot position-target controller to address either arm while preserving the right-arm API. Add a free cube and left-palm TCP to the scene, then implement a staged pick-and-place task whose grasp gate uses contacts, cube motion, and simulated actuator force rather than a hidden attachment.

**Tech Stack:** Python 3.12, MuJoCo 3.10.0, NumPy 2.5.1, pytest 9.1.1, existing `twin_sim` position-control and Viewer marker APIs

## Global Constraints

- Do not add weld, equality, mocap, teleportation, or runtime attachment between the cube and the hand.
- Continue using MuJoCo `position` actuators; do not add custom joint or Cartesian impedance control.
- Keep the right-arm chopping API and behavior backward compatible.
- All temporary FK/IK evaluations restore the complete MuJoCo state.
- The right-arm target and state remain unchanged during the left pick-and-place task.
- The cube must rise at least `0.08 m`, move horizontally at least `0.15 m`, and settle inside the target region.
- Abort on invalid targets, IK/path failure, excessive simulated force, cube drop, table penetration, or non-finite state.
- On abort, hold the last accepted arm/hand targets and do not automatically open an elevated grasp.
- Preserve all files covered by `docs/simulation/protected-files.sha256`.

---

### Task 1: Add the left-palm TCP and free grasp cube

**Files:**
- Modify: `robot_assets/mujoco/right_chopping_scene.xml`
- Modify: `src/twin_sim/model.py`
- Create: `tests/simulation/test_pick_place_scene.py`

**Interfaces:**
- Consumes: existing `SimulationModel.load()` and model name validation.
- Produces: sites `left_palm_tcp_site`, `pick_cube_site`, `pick_target_site`; body `pick_cube`; free joint `pick_cube_free`; geoms `pick_cube_geom`, `pick_target_region`.

- [ ] **Step 1: Write failing model-contract tests**

```python
import mujoco
import numpy as np

from twin_sim.model import SimulationModel


def test_pick_place_scene_has_free_unattached_cube_and_named_sites():
    sim = SimulationModel.load()
    for site in ("left_palm_tcp_site", "pick_cube_site", "pick_target_site"):
        assert sim.require_site(site) >= 0
    joint = mujoco.mj_name2id(
        sim.model, mujoco.mjtObj.mjOBJ_JOINT, "pick_cube_free"
    )
    assert joint >= 0
    assert sim.model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_FREE
    assert sim.model.neq == 0


def test_pick_cube_has_mass_collision_and_friction():
    sim = SimulationModel.load()
    body = sim.require_body("pick_cube")
    geom = sim.require_geom("pick_cube_geom")
    assert sim.model.body_mass[body] > 0.0
    assert sim.model.geom_contype[geom] != 0
    assert sim.model.geom_conaffinity[geom] != 0
    assert np.all(sim.model.geom_friction[geom] > 0.0)
```

- [ ] **Step 2: Run the tests and verify missing-name failures**

Run: `.venv/bin/pytest tests/simulation/test_pick_place_scene.py -q`

Expected: FAIL because `left_palm_tcp_site` and the cube do not exist.

- [ ] **Step 3: Add the model elements**

Inside `left_palm_link`, add:

```xml
<site name="left_palm_tcp_site" pos="0 0 0.035"
      quat="1 0 0 0" size="0.008" rgba="0 1 1 1"/>
```

At worldbody level, add a free cube and target marker. Tune the two positions
from headless FK reachability checks but retain the exact names and dimensions:

```xml
<body name="pick_cube" pos="0.48 0.22 0.105">
  <freejoint name="pick_cube_free"/>
  <geom name="pick_cube_geom" type="box" size="0.025 0.025 0.025"
        mass="0.08" friction="1.5 0.02 0.002" rgba="0.9 0.45 0.1 1"/>
  <site name="pick_cube_site" size="0.006" rgba="1 0.4 0 1"/>
</body>
<site name="pick_target_site" pos="0.48 0.02 0.081"
      size="0.035" rgba="0.2 0.9 0.2 0.35"/>
<geom name="pick_target_region" type="cylinder"
      pos="0.48 0.02 0.0805" size="0.045 0.0005"
      contype="0" conaffinity="0" rgba="0.2 0.9 0.2 0.25"/>
```

Add `require_body()` to `SimulationModel`:

```python
def require_body(self, name: str) -> int:
    return self._require_id(
        self.model, mujoco.mjtObj.mjOBJ_BODY, "body", name
    )
```

- [ ] **Step 4: Run scene and existing model tests**

Run: `.venv/bin/pytest tests/simulation/test_pick_place_scene.py tests/simulation/test_model.py -q`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add robot_assets/mujoco/right_chopping_scene.xml src/twin_sim/model.py tests/simulation/test_pick_place_scene.py
git commit -m "feat: add free cube for left-hand pick and place"
```

---

### Task 2: Generalize kinematics for either arm

**Files:**
- Modify: `src/twin_sim/kinematics.py`
- Modify: `src/twin_sim/robot.py`
- Modify: `tests/simulation/test_kinematics.py`
- Create: `tests/simulation/test_left_kinematics.py`

**Interfaces:**
- Consumes: `SimulationModel.left`, `SimulationModel.right`, `left_palm_tcp_site`, `right_tool_tip_site`.
- Produces: `Kinematics(simulation, arm: ArmIndices | None = None, tcp_site: str = "right_tool_tip_site")`; `RightArmRobot.left_kinematics`; unchanged default right-arm construction.

- [ ] **Step 1: Write failing left/right isolation tests**

```python
import numpy as np

from twin_sim.kinematics import Kinematics
from twin_sim.model import SimulationModel


def test_left_kinematics_changes_only_temporary_left_configuration():
    sim = SimulationModel.load()
    before = sim.data.qpos.copy()
    kine = Kinematics(sim, sim.left, "left_palm_tcp_site")
    pose = kine.fk(sim.data.qpos[sim.left.qpos_ids])
    assert pose.shape == (4, 4)
    np.testing.assert_array_equal(sim.data.qpos, before)


def test_left_ik_reaches_nearby_palm_pose_without_moving_right_state():
    sim = SimulationModel.load()
    kine = Kinematics(sim, sim.left, "left_palm_tcp_site")
    seed = sim.data.qpos[sim.left.qpos_ids].copy()
    target = kine.fk(seed)
    target[2, 3] += 0.02
    right_before = sim.data.qpos[sim.right.qpos_ids].copy()
    result = kine.ik(target, seed)
    assert result.success
    np.testing.assert_array_equal(
        sim.data.qpos[sim.right.qpos_ids], right_before
    )
```

- [ ] **Step 2: Run the focused tests and verify constructor failure**

Run: `.venv/bin/pytest tests/simulation/test_left_kinematics.py -q`

Expected: FAIL because `Kinematics` does not accept arm/site arguments.

- [ ] **Step 3: Parameterize the selected arm and TCP**

Change the constructor while preserving its existing default:

```python
def __init__(
    self,
    simulation: SimulationModel,
    arm: ArmIndices | None = None,
    tcp_site: str = "right_tool_tip_site",
):
    self._model = simulation.model
    self._data = simulation.data
    self._arm = simulation.right if arm is None else arm
    self._tcp_site_id = simulation.require_site(tcp_site)
    self._lower_limits = self._model.jnt_range[
        self._arm.joint_ids, 0
    ].copy()
    self._upper_limits = self._model.jnt_range[
        self._arm.joint_ids, 1
    ].copy()
```

Replace all internal `_right` access with `_arm`. Keep the existing
`mj_copyData` configuration context so the full state is restored.

- [ ] **Step 4: Expose both kinematics objects on the robot**

```python
self.right_kinematics = Kinematics(
    self.sim, self.sim.right, "right_tool_tip_site"
)
self.left_kinematics = Kinematics(
    self.sim, self.sim.left, "left_palm_tcp_site"
)
```

- [ ] **Step 5: Run all kinematics and trajectory tests**

Run: `.venv/bin/pytest tests/simulation/test_kinematics.py tests/simulation/test_left_kinematics.py tests/simulation/test_trajectory.py -q`

Expected: PASS with existing right-arm cases unchanged.

- [ ] **Step 6: Commit**

```bash
git add src/twin_sim/kinematics.py src/twin_sim/robot.py tests/simulation/test_kinematics.py tests/simulation/test_left_kinematics.py
git commit -m "refactor: support left or right arm kinematics"
```

---

### Task 3: Add validated independent left-arm position targets

**Files:**
- Modify: `src/twin_sim/robot.py`
- Create: `tests/simulation/test_dual_arm_control.py`

**Interfaces:**
- Consumes: `RightArmRobot.sim.left`, position actuator ranges, existing `step()`.
- Produces: `left_joint_positions`, `left_joint_velocities`, `left_palm_pose()`, `command_left(joints_rad)`, `validate_left_targets(targets)`.

- [ ] **Step 1: Write failing dual-arm controller tests**

```python
import numpy as np
import pytest

from twin_sim.robot import RightArmRobot


def test_left_command_moves_left_and_preserves_right_target():
    robot = RightArmRobot()
    right_target = robot.joint_positions.copy()
    left = robot.left_joint_positions
    goal = left.copy()
    goal[1] += 0.02
    robot.command_left(goal)
    for _ in range(20):
        robot.step(0.01)
    assert robot.left_joint_positions[1] > left[1]
    np.testing.assert_array_equal(robot._right_target, right_target)
    robot.close()


def test_invalid_left_target_is_atomic():
    robot = RightArmRobot()
    before = robot._left_target.copy()
    with pytest.raises(ValueError, match="left joint"):
        robot.command_left(np.zeros(6))
    np.testing.assert_array_equal(robot._left_target, before)
    robot.close()
```

- [ ] **Step 2: Run and verify the missing API**

Run: `.venv/bin/pytest tests/simulation/test_dual_arm_control.py -q`

Expected: FAIL with missing `command_left`/`left_joint_positions`.

- [ ] **Step 3: Replace left hold with a validated target**

Maintain `_left_target` initialized from reset qpos. Add a common private
validator taking indices and label, then implement:

```python
def command_left(self, joints_rad: Sequence[float]) -> None:
    self._left_target = self._validated_arm_target(
        joints_rad, self.sim.left, "left joint"
    )

@property
def left_joint_positions(self) -> np.ndarray:
    return self.sim.data.qpos[self.sim.left.qpos_ids].copy()

@property
def left_joint_velocities(self) -> np.ndarray:
    return self.sim.data.qvel[self.sim.left.dof_ids].copy()

def left_palm_pose(self) -> np.ndarray:
    return self._site_pose(self.sim.require_site("left_palm_tcp_site"))
```

`step()` writes `_left_target`, `_right_target`, and the hand target before all
substeps.

- [ ] **Step 4: Run focused and right-arm regression tests**

Run: `.venv/bin/pytest tests/simulation/test_dual_arm_control.py tests/simulation/test_robot.py tests/simulation/test_chop.py -q`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/twin_sim/robot.py tests/simulation/test_dual_arm_control.py
git commit -m "feat: add independent left-arm position control"
```

---

### Task 4: Implement contact-based grasp assessment

**Files:**
- Create: `src/twin_sim/grasp.py`
- Create: `tests/simulation/test_grasp.py`

**Interfaces:**
- Consumes: MuJoCo contacts, named cube geom/body, hand geom/body ancestry, hand actuator forces.
- Produces: `GraspObservation`, `GraspMonitor.observe(sim)`, `GraspMonitor.update(observation, dt_s)`, `ready`, `abort_reason`.

- [ ] **Step 1: Write failing pure dwell and threshold tests**

```python
from twin_sim.grasp import GraspMonitor, GraspObservation


def observation(*, opposing=True, speed=0.0, force=2.0):
    return GraspObservation(
        hand_contact_count=2,
        opposing_contacts=opposing,
        cube_linear_speed_m_s=speed,
        cube_angular_speed_rad_s=0.0,
        max_hand_actuator_force=force,
    )


def test_grasp_requires_continuous_stable_dwell():
    monitor = GraspMonitor(stable_dwell_s=0.10)
    for _ in range(9):
        monitor.update(observation(), 0.01)
    assert not monitor.ready
    monitor.update(observation(), 0.01)
    assert monitor.ready


def test_unstable_sample_resets_dwell_and_force_aborts():
    monitor = GraspMonitor(stable_dwell_s=0.10, abort_force=20.0)
    monitor.update(observation(), 0.05)
    monitor.update(observation(speed=0.2), 0.01)
    assert not monitor.ready
    monitor.update(observation(force=21.0), 0.01)
    assert monitor.abort_reason == "hand force limit exceeded"
```

- [ ] **Step 2: Run and verify the module is absent**

Run: `.venv/bin/pytest tests/simulation/test_grasp.py -q`

Expected: FAIL importing `twin_sim.grasp`.

- [ ] **Step 3: Implement immutable observations and monitor state**

```python
@dataclass(frozen=True)
class GraspObservation:
    hand_contact_count: int
    opposing_contacts: bool
    cube_linear_speed_m_s: float
    cube_angular_speed_rad_s: float
    max_hand_actuator_force: float


class GraspMonitor:
    def update(self, observation: GraspObservation, dt_s: float) -> None:
        if observation.max_hand_actuator_force > self.abort_force:
            self.abort_reason = "hand force limit exceeded"
            return
        stable = (
            observation.hand_contact_count >= 2
            and observation.opposing_contacts
            and observation.cube_linear_speed_m_s <= self.max_linear_speed
            and observation.cube_angular_speed_rad_s <= self.max_angular_speed
        )
        self._stable_time = self._stable_time + dt_s if stable else 0.0
        self.ready = self._stable_time >= self.stable_dwell_s
```

Implement `observe()` by iterating `data.contact[:data.ncon]`, selecting
cube/hand pairs, using contact normals to classify opposing contacts, and
reading free-body velocity plus `data.actuator_force[sim.hand.actuator_ids]`.

- [ ] **Step 4: Add a live-scene contact extraction test**

Place the cube deterministically between two hand collision geoms, call
`mj_forward`/`mj_step`, and assert `observe()` returns finite fields and only
counts contacts involving `pick_cube_geom`.

- [ ] **Step 5: Run the focused tests**

Run: `.venv/bin/pytest tests/simulation/test_grasp.py -q`

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add src/twin_sim/grasp.py tests/simulation/test_grasp.py
git commit -m "feat: assess Wuji hand grasps from MuJoCo contacts"
```

---

### Task 5: Implement the staged pick-and-place task

**Files:**
- Create: `src/twin_sim/tasks/pick_place.py`
- Create: `tests/simulation/test_pick_place_state_machine.py`

**Interfaces:**
- Consumes: left-arm controller, left kinematics, `cartesian_trajectory`, `GraspMonitor`, hand open/command APIs.
- Produces: `PickPlacePhase`, `PickPlaceConfig`, `PickPlaceSample`, `PickPlaceResult`, `PickPlaceTask.run()`.

- [ ] **Step 1: Write failing phase/result tests with a fake task driver**

```python
from twin_sim.tasks.pick_place import PickPlacePhase, PickPlaceResult


def test_phase_order_is_explicit_and_complete():
    assert [phase.value for phase in PickPlacePhase] == [
        "initialize", "open_hand", "pregrasp", "approach", "close_hand",
        "stabilize", "lift", "transfer", "lower", "release",
        "retreat", "complete", "aborted",
    ]


def test_abort_result_preserves_phase_and_reason():
    result = PickPlaceResult(
        success=False,
        final_phase=PickPlacePhase.ABORTED,
        abort_phase=PickPlacePhase.STABILIZE,
        reason="grasp timeout",
        samples=(),
    )
    assert not result.success
    assert result.abort_phase is PickPlacePhase.STABILIZE
```

- [ ] **Step 2: Run and verify the task module is absent**

Run: `.venv/bin/pytest tests/simulation/test_pick_place_state_machine.py -q`

Expected: FAIL importing `pick_place`.

- [ ] **Step 3: Define task data and deterministic configuration**

```python
class PickPlacePhase(Enum):
    INITIALIZE = "initialize"
    OPEN_HAND = "open_hand"
    PREGRASP = "pregrasp"
    APPROACH = "approach"
    CLOSE_HAND = "close_hand"
    STABILIZE = "stabilize"
    LIFT = "lift"
    TRANSFER = "transfer"
    LOWER = "lower"
    RELEASE = "release"
    RETREAT = "retreat"
    COMPLETE = "complete"
    ABORTED = "aborted"


@dataclass(frozen=True)
class PickPlaceConfig:
    control_dt_s: float = 0.01
    lift_height_m: float = 0.10
    transfer_distance_m: float = 0.20
    grasp_timeout_s: float = 3.0
    final_hold_s: float = 5.0
```

Each `PickPlaceSample` stores time, phase, left target/actual, palm
target/actual, cube pose/velocity, hand target/actual, contacts, and maximum
hand actuator force.

- [ ] **Step 4: Implement motion phases and safe abort**

Plan all arm paths before executing a motion phase. Use `command_left()` for
each solved joint point, interpolate hand closure/opening over time, update
`GraspMonitor`, and append one sample per control tick. A single `_abort()`
method returns a result without changing the last accepted targets.

- [ ] **Step 5: Add deterministic transition and failure tests**

Use injected fake trajectory/grasp results to prove:

- phases cannot be skipped;
- grasp timeout aborts in `STABILIZE`;
- excessive force aborts without opening the hand;
- a drop during `LIFT`/`TRANSFER` aborts;
- right-arm targets never change.

- [ ] **Step 6: Run state-machine and controller tests**

Run: `.venv/bin/pytest tests/simulation/test_pick_place_state_machine.py tests/simulation/test_dual_arm_control.py tests/simulation/test_grasp.py -q`

Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add src/twin_sim/tasks/pick_place.py tests/simulation/test_pick_place_state_machine.py
git commit -m "feat: add left-arm pick-and-place state machine"
```

---

### Task 6: Tune and verify the real-contact headless task

**Files:**
- Modify: `robot_assets/mujoco/right_chopping_scene.xml`
- Modify: `src/twin_sim/tasks/pick_place.py`
- Create: `tests/simulation/test_pick_place_integration.py`

**Interfaces:**
- Consumes: complete state machine and real MuJoCo scene.
- Produces: a deterministic `PickPlaceResult(success=True)` meeting every quantitative acceptance threshold.

- [ ] **Step 1: Write the complete failing acceptance test**

```python
import numpy as np

from twin_sim.robot import RightArmRobot
from twin_sim.tasks.pick_place import PickPlaceTask


def test_real_contact_pick_place_meets_acceptance_contract():
    robot = RightArmRobot(viewer=False)
    right_before = robot.joint_positions.copy()
    result = PickPlaceTask(robot).run()
    assert result.success, result.reason
    cube = np.asarray([sample.cube_position for sample in result.samples])
    assert cube[:, 2].max() - cube[0, 2] >= 0.08
    assert np.linalg.norm(cube[-1, :2] - cube[0, :2]) >= 0.15
    assert result.placed_in_target
    assert not result.used_hidden_attachment
    np.testing.assert_allclose(robot.joint_positions, right_before, atol=1e-6)
    robot.close()
```

- [ ] **Step 2: Run it and record the first physical failure**

Run: `.venv/bin/pytest tests/simulation/test_pick_place_integration.py -q -s`

Expected: initially FAIL with a specific IK, contact, slip, or placement reason.

- [ ] **Step 3: Tune only explicit physical/task parameters**

Adjust cube position, target position, palm grasp frame, approach offsets,
finger close pose, friction, phase duration, and force thresholds. Do not add
constraints or special-case cube motion. After each adjustment, run the single
integration test and log the exact failing criterion.

- [ ] **Step 4: Add table penetration and final-settle assertions**

Require cube bottom not below the board top by more than `0.002 m`, final
linear speed below `0.02 m/s`, and at least `0.25 s` of unassisted settle after
release.

- [ ] **Step 5: Run integration plus chopping regression**

Run: `.venv/bin/pytest tests/simulation/test_pick_place_integration.py tests/simulation/test_chop.py -q`

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add robot_assets/mujoco/right_chopping_scene.xml src/twin_sim/tasks/pick_place.py tests/simulation/test_pick_place_integration.py
git commit -m "test: verify physical left-hand pick and place"
```

---

### Task 7: Add CLI visualization, trajectory markers, and documentation

**Files:**
- Modify: `src/twin_sim/cli.py`
- Create: `src/twin_sim/pick_place_visualization.py`
- Create: `tests/simulation/test_pick_place_visualization.py`
- Modify: `README.md`
- Create: `docs/simulation/left_arm_wuji_pick_place.md`

**Interfaces:**
- Consumes: `PickPlaceTask`, recorded `PickPlaceSample` values, MuJoCo passive Viewer.
- Produces: `twin-sim pick-place [--headless] [--slow] [--final-hold SECONDS]`; planned palm, actual palm, and actual cube markers.

- [ ] **Step 1: Write failing CLI and marker-buffer tests**

```python
from twin_sim.cli import build_parser
from twin_sim.pick_place_visualization import PickPlaceTrace


def test_pick_place_cli_defaults():
    args = build_parser().parse_args(["pick-place", "--headless"])
    assert args.command == "pick-place"
    assert args.headless


def test_trace_keeps_three_distinct_paths():
    trace = PickPlaceTrace(max_points=100)
    trace.append(
        planned_palm=(0.0, 0.0, 0.0),
        actual_palm=(0.0, 0.0, 0.1),
        actual_cube=(0.0, 0.0, 0.2),
    )
    assert len(trace.planned_palm) == 1
    assert len(trace.actual_palm) == 1
    assert len(trace.actual_cube) == 1
```

- [ ] **Step 2: Run and verify missing command/module failures**

Run: `.venv/bin/pytest tests/simulation/test_pick_place_visualization.py -q`

Expected: FAIL.

- [ ] **Step 3: Implement bounded traces and Viewer markers**

Use separate colors:

- planned palm: blue;
- actual palm: cyan;
- cube: orange;
- target region: green.

Reuse the existing Viewer marker mechanism used by chopping trajectory
visualization. Update the window title or overlay with phase, contact count,
grasp-ready state, and abort reason.

- [ ] **Step 4: Add the CLI entry**

The headless command prints one summary line containing success, final phase,
maximum lift, horizontal transfer, final target error, maximum force, and
abort reason. `--slow` runs in real time; `--final-hold` overrides the default
5 second inspection hold.

- [ ] **Step 5: Document usage and limitations**

Document:

```bash
.venv/bin/twin-sim pick-place --headless
.venv/bin/twin-sim pick-place --slow --final-hold 15
```

State explicitly that grasping uses simulated friction, effort is
uncalibrated, there is no hidden attachment, phase B is not yet implemented,
and physical hardware must not run these commands.

- [ ] **Step 6: Run the full verification gate**

Run:

```bash
.venv/bin/pytest -q
.venv/bin/twin-sim pick-place --headless
sha256sum --check docs/simulation/protected-files.sha256
git diff --check
```

Expected: all tests pass; CLI reports success; every protected file reports
`OK`; diff check is silent.

- [ ] **Step 7: Run the slow Viewer acceptance**

Run: `.venv/bin/twin-sim pick-place --slow --final-hold 15`

Inspect the complete pick, lift, lateral transfer, lower, release, retreat,
three trajectory colors, phase display, and final object placement.

- [ ] **Step 8: Commit**

```bash
git add src/twin_sim/cli.py src/twin_sim/pick_place_visualization.py tests/simulation/test_pick_place_visualization.py README.md docs/simulation/left_arm_wuji_pick_place.md
git commit -m "feat: visualize left-arm Wuji pick and place"
```

---

### Task 8: Final review and phase-B handoff

**Files:**
- Modify: `docs/simulation/left_arm_wuji_pick_place.md`
- Modify: `docs/development-log.md` if present, otherwise create `docs/simulation/left_arm_wuji_pick_place_development_log.md`

**Interfaces:**
- Consumes: final commits and verification evidence.
- Produces: reviewed phase-A branch and a concrete phase-B boundary without implementing phase B.

- [ ] **Step 1: Review the complete diff against the approved design**

Compare `git diff <base-commit>..HEAD` with
`docs/superpowers/specs/2026-07-31-left-arm-wuji-pick-place-design.md`.
Reject any hidden attachment, right-arm regression, missing quantitative
assertion, or unprotected failure transition.

- [ ] **Step 2: Record development evidence**

Record the final tuned cube pose, target pose, grasp posture, friction,
durations, contact/force thresholds, encountered failures, fixes, test counts,
Viewer behavior, and known limitations.

- [ ] **Step 3: Run fresh final verification**

Run:

```bash
.venv/bin/pytest -q
.venv/bin/twin-sim pick-place --headless
sha256sum --check docs/simulation/protected-files.sha256
git diff --check
git status --short
```

Expected: all tests and headless task pass; protected files are `OK`; diff
check is silent; status contains only intentional documentation changes.

- [ ] **Step 4: Commit**

```bash
git add docs/simulation/left_arm_wuji_pick_place.md docs/simulation/*development_log.md
git commit -m "docs: record left-hand pick-and-place development"
```

