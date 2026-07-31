# Plane Guarded Chopping Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deliver a MuJoCo plane-mode demonstration in which the right knife advances from screen-right to screen-left while the left Wuji Hand visibly opens, shifts 0.02 m, and recloses after every raised knife.

**Architecture:** Extend the existing guarded-chop planner and state machine rather than creating a second task. A `scene_mode` configuration selects plane or object scene activation; phase-specific hand joint trajectories make opening and closing explicit, while the visualization layer renders continuous trails and compact cut-point marks instead of four independent spheres.

**Tech Stack:** Python 3.11, NumPy, MuJoCo, pytest, existing `twin_sim` kinematics/controller/Viewer helpers.

## Global Constraints

- Default delivery and visual demonstration use `scene_mode="plane"`; object mode remains available but is not recalibrated in this iteration.
- Execute exactly 5 cuts and 4 left-hand shifts of 0.02 m, totaling 0.08 m.
- Default Viewer projection must show cut points progressing from screen-right to screen-left.
- Each shift sequence is `HAND_OPEN -> HAND_SHIFT -> HAND_CLOSE`.
- `CUT_DOWN` requires the left arm and all 20 hand joints to be stationary; all left-hand phases require a raised, stationary knife.
- Knife-to-hand distance remains at least 0.02 m and real blade-hand contact remains zero.
- Do not modify real-robot, ROS 2, SDK, or hardware-send files.

---

### Task 1: Define plane mode, phases, and visible hand postures

**Files:**
- Modify: `src/twin_sim/guarded_chop_safety.py`
- Modify: `src/twin_sim/tasks/guarded_chop.py`
- Modify: `tests/simulation/test_guarded_chop_posture.py`
- Modify: `tests/simulation/test_guarded_chop_state_machine.py`

**Interfaces:**
- Produces: `CAT_PAW_OPEN_RAD: np.ndarray` with shape `(20,)`.
- Produces: `GuardedChopConfig.scene_mode: Literal["plane", "object"]` with default `"plane"`.
- Produces: `GuardedChopPhase.HAND_OPEN` and `GuardedChopPhase.HAND_CLOSE`.
- Consumes: existing `CAT_PAW_RAD`, actuator control ranges, and phase enum.

- [ ] **Step 1: Write failing posture and state tests**

```python
def test_open_cat_paw_is_visible_and_within_hand_ranges():
    sim = SimulationModel.load()
    ranges = sim.model.actuator_ctrlrange[sim.hand.actuator_ids]
    assert CAT_PAW_OPEN_RAD.shape == (20,)
    assert np.all(CAT_PAW_OPEN_RAD >= ranges[:, 0])
    assert np.all(CAT_PAW_OPEN_RAD <= ranges[:, 1])
    assert np.linalg.norm(CAT_PAW_OPEN_RAD - CAT_PAW_RAD) >= 1.0
    assert np.all(CAT_PAW_OPEN_RAD[[6, 10, 14, 18]] < CAT_PAW_RAD[[6, 10, 14, 18]])


def test_plane_mode_and_open_close_phases_are_defaults():
    config = GuardedChopConfig()
    assert config.scene_mode == "plane"
    assert GuardedChopPhase.HAND_OPEN.value == "hand_open"
    assert GuardedChopPhase.HAND_CLOSE.value == "hand_close"
```

- [ ] **Step 2: Verify the tests fail for missing interfaces**

Run: `.venv/bin/pytest tests/simulation/test_guarded_chop_posture.py tests/simulation/test_guarded_chop_state_machine.py -q`  
Expected: FAIL because `CAT_PAW_OPEN_RAD`, `scene_mode`, `HAND_OPEN`, and `HAND_CLOSE` do not exist.

- [ ] **Step 3: Add the open posture, mode validation, and phases**

```python
CAT_PAW_OPEN_RAD = np.asarray(
    (0.80, -0.02, 0.55, 0.55)
    + (0.20, 0.00, 0.55, 0.55) * 4,
    dtype=float,
)


class GuardedChopPhase(str, Enum):
    INITIALIZE = "initialize"
    GUARD_READY = "guard_ready"
    CUT_DOWN = "cut_down"
    KNIFE_UP = "knife_up"
    HAND_OPEN = "hand_open"
    HAND_SHIFT = "hand_shift"
    HAND_CLOSE = "hand_close"
    COMPLETE = "complete"
    ABORTED = "aborted"
```

Add `scene_mode: str = "plane"`, `hand_open_duration_s: float = 0.6`, and
`hand_close_duration_s: float = 0.6` to `GuardedChopConfig`. Validate the mode
against `{"plane", "object"}` and include both durations in the existing
positive, finite, control-step-aligned duration checks.

- [ ] **Step 4: Run focused tests**

Run: `.venv/bin/pytest tests/simulation/test_guarded_chop_posture.py tests/simulation/test_guarded_chop_state_machine.py -q`  
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/twin_sim/guarded_chop_safety.py src/twin_sim/tasks/guarded_chop.py tests/simulation/test_guarded_chop_posture.py tests/simulation/test_guarded_chop_state_machine.py
git commit -m "feat: define plane guarded hand phases"
```

---

### Task 2: Plan screen-right-to-left knife and guard motion

**Files:**
- Modify: `src/twin_sim/tasks/guarded_chop.py`
- Modify: `tests/simulation/test_guarded_chop_preflight.py`

**Interfaces:**
- Produces: `_ordered_right_to_left(points_xy: np.ndarray) -> np.ndarray`.
- Produces: `_GuardedChopPlan.cut_points_xy: np.ndarray` for projection and tests.
- Consumes: line-chop cut points, default Viewer camera, and `_guard_plan()`.

- [ ] **Step 1: Write a failing direction test**

```python
def test_preflight_orders_knife_and_guard_motion_screen_right_to_left():
    robot = RightArmRobot(viewer=False)
    try:
        plan = _preflight_guarded_chop(robot, GuardedChopConfig())
        screen_x = _default_view_screen_x(plan.cut_points_xy)
        assert np.all(np.diff(screen_x) < 0.0)
        knife_delta = np.diff(plan.cut_points_xy, axis=0)
        guard_delta = np.diff(plan.guard_targets[:, :2, 3], axis=0)
        np.testing.assert_allclose(guard_delta, knife_delta, atol=1e-9)
    finally:
        robot.close()
```

Define the guarded-chop Viewer camera explicitly as azimuth `135.0`, elevation
`-20.0`, distance `1.6`, look-at `(0.48, 0.0, 0.48)`, matching the existing
chop view. `_default_view_screen_x()` uses the corresponding normalized XY
screen-right basis `(-sin(azimuth), cos(azimuth))`; this makes direction tests
independent of an open Viewer.

- [ ] **Step 2: Run the direction test and confirm the current order fails**

Run: `.venv/bin/pytest tests/simulation/test_guarded_chop_preflight.py::test_preflight_orders_knife_and_guard_motion_screen_right_to_left -q`  
Expected: FAIL because current points advance left-to-right in the default Viewer.

- [ ] **Step 3: Reverse only the point order before building trajectories**

```python
def _ordered_right_to_left(points_xy: np.ndarray) -> np.ndarray:
    points = np.asarray(points_xy, dtype=float)
    screen_x = _default_view_screen_x(points)
    if screen_x[0] < screen_x[-1]:
        points = points[::-1]
    return points.copy()


def _default_view_screen_x(points_xy: np.ndarray) -> np.ndarray:
    azimuth = np.deg2rad(135.0)
    screen_right_xy = np.asarray((-np.sin(azimuth), np.cos(azimuth)))
    return np.asarray(points_xy, dtype=float) @ screen_right_xy
```

Store the ordered points on `_GuardedChopPlan`, feed them to both right-cut
translation and `_guard_plan`, and keep the existing 0.02 m spacing and IK
preflight checks.

- [ ] **Step 4: Run preflight and clearance tests**

Run: `.venv/bin/pytest tests/simulation/test_guarded_chop_preflight.py tests/simulation/test_guarded_chop_posture.py -q`  
Expected: PASS with five right-to-left points and four matching guard deltas.

- [ ] **Step 5: Commit**

```bash
git add src/twin_sim/tasks/guarded_chop.py tests/simulation/test_guarded_chop_preflight.py
git commit -m "feat: order guarded cuts right to left"
```

---

### Task 3: Execute open-shift-close in plane mode

**Files:**
- Modify: `src/twin_sim/tasks/guarded_chop.py`
- Modify: `src/twin_sim/guarded_chop_safety.py`
- Modify: `tests/simulation/test_guarded_chop_safety.py`
- Modify: `tests/simulation/test_guarded_chop_integration.py`

**Interfaces:**
- Produces: `_joint_trajectory(start_rad, goal_rad, duration_s, dt_s) -> tuple[np.ndarray, ...]`.
- Produces: samples for `HAND_OPEN`, `HAND_SHIFT`, and `HAND_CLOSE` containing the existing `hand_target_rad`.
- Consumes: `CAT_PAW_RAD`, `CAT_PAW_OPEN_RAD`, `robot.hand.command()`, and `wait_for_guard_stability()`.

- [ ] **Step 1: Write failing phase-order and hand-target tests**

```python
def test_plane_mode_visibly_opens_shifts_and_closes_each_time():
    result = run_guarded_chop(GuardedChopConfig(final_hold_s=0.0), viewer=False)
    assert result.success, result.reason
    phases = [sample.phase for sample in result.samples]
    assert phases.count(GuardedChopPhase.HAND_OPEN) > 0
    assert phases.count(GuardedChopPhase.HAND_CLOSE) > 0
    for index in range(1, 5):
        opened = [s for s in result.samples if s.cut_index == index and s.phase is GuardedChopPhase.HAND_OPEN]
        shifted = [s for s in result.samples if s.cut_index == index and s.phase is GuardedChopPhase.HAND_SHIFT]
        closed = [s for s in result.samples if s.cut_index == index and s.phase is GuardedChopPhase.HAND_CLOSE]
        assert opened and shifted and closed
        assert np.linalg.norm(opened[-1].hand_target_rad - CAT_PAW_OPEN_RAD) < 1e-9
        assert np.max(np.ptp(np.asarray([s.hand_target_rad for s in shifted]), axis=0)) < 1e-9
        assert np.linalg.norm(closed[-1].hand_target_rad - CAT_PAW_RAD) < 1e-9
```

Also assert the plane run has `max(sample.guard_cube_contact_count) == 0` and
that the cube alpha and collision masks remain disabled.

- [ ] **Step 2: Run integration tests and verify missing phases fail**

Run: `.venv/bin/pytest tests/simulation/test_guarded_chop_integration.py -q -s`  
Expected: FAIL because the current state machine shifts with a latched hand and activates the cube.

- [ ] **Step 3: Implement joint interpolation and mode-specific scene activation**

```python
def _joint_trajectory(start_rad, goal_rad, duration_s, dt_s):
    steps = int(round(duration_s / dt_s))
    blend = np.linspace(0.0, 1.0, steps + 1)
    smooth = blend * blend * (3.0 - 2.0 * blend)
    return tuple(
        np.asarray(start_rad) + value * (np.asarray(goal_rad) - np.asarray(start_rad))
        for value in smooth
    )
```

Split `_activate_guarded_scene(robot)` into
`_configure_guarded_scene(robot, scene_mode)`. In `plane`, leave the cube hidden
and collision-disabled; in `object`, retain the existing bit-8 cube/pad pairing
and blade bit-2 coverage.

Replace each current plane `HAND_SHIFT` block with:

```python
phase = GuardedChopPhase.HAND_OPEN
for target in _joint_trajectory(CAT_PAW_RAD, CAT_PAW_OPEN_RAD, config.hand_open_duration_s, config.control_dt_s)[1:]:
    robot.hand.command(target)
    step_and_record(phase, index)

phase = GuardedChopPhase.HAND_SHIFT
for point in plan.guard_shifts[index - 1][1:]:
    robot.command_left(point.joints_rad)
    robot.hand.command(CAT_PAW_OPEN_RAD)
    step_and_record(phase, index)

phase = GuardedChopPhase.HAND_CLOSE
for target in _joint_trajectory(CAT_PAW_OPEN_RAD, CAT_PAW_RAD, config.hand_close_duration_s, config.control_dt_s)[1:]:
    robot.hand.command(target)
    step_and_record(phase, index)
wait_for_guard_stability(phase, index, latch_contact=False)
```

Keep the existing contact-latching behavior only inside object mode.

- [ ] **Step 4: Extend safety evaluation to all left-hand phases**

Treat `HAND_OPEN`, `HAND_SHIFT`, and `HAND_CLOSE` identically for the raised,
stationary right-knife interlock. Preserve the `CUT_DOWN` arm and 20-joint hand
speed checks.

- [ ] **Step 5: Run safety and integration tests**

Run: `.venv/bin/pytest tests/simulation/test_guarded_chop_safety.py tests/simulation/test_guarded_chop_integration.py -q -s`  
Expected: PASS with five cuts, four explicit open-shift-close cycles, zero plane cube contact, and zero blade-hand contact.

- [ ] **Step 6: Commit**

```bash
git add src/twin_sim/tasks/guarded_chop.py src/twin_sim/guarded_chop_safety.py tests/simulation/test_guarded_chop_safety.py tests/simulation/test_guarded_chop_integration.py
git commit -m "feat: animate plane cat-paw handovers"
```

---

### Task 4: Replace sphere markers with meaningful trails and cut marks

**Files:**
- Modify: `src/twin_sim/guarded_chop_visualization.py`
- Modify: `src/twin_sim/tasks/guarded_chop.py`
- Modify: `tests/simulation/test_guarded_chop_visualization.py`

**Interfaces:**
- Produces: `GuardedChopTrace.set_plan(cut_points: Sequence[Sequence[float]]) -> None`.
- Consumes: planned blade path, actual blade position, actual guard position, and Viewer `user_scn`.

- [ ] **Step 1: Replace the four-sphere expectation with trail semantics**

```python
def test_trace_draws_three_trails_and_five_compact_cut_marks():
    viewer = FakeViewer()
    trace = GuardedChopTrace(viewer, marker_stride=1)
    trace.set_plan([(0.7, y, 0.33) for y in (0.04, 0.02, 0.0, -0.02, -0.04)])
    trace.append(
        planned_knife=(0.7, 0.04, 0.36),
        actual_knife=(0.7, 0.04, 0.35),
        actual_guard=(0.6, 0.10, 0.36),
        phase="hand_open",
        cut_index=1,
        minimum_distance_m=0.06,
        cut_allowed=False,
    )
    assert len(trace.cut_points) == 5
    assert not hasattr(trace, "guard_target")
    assert "phase=hand_open" in viewer.texts[3]
```

Update `FakeScene.maxgeom` high enough for five cut marks plus trail segments.

- [ ] **Step 2: Run visualization tests and confirm the old four-sphere API fails**

Run: `.venv/bin/pytest tests/simulation/test_guarded_chop_visualization.py -q`  
Expected: FAIL because `set_plan` and continuous trail rendering do not exist.

- [ ] **Step 3: Implement bounded line/capsule rendering**

Keep deques for `planned_knife`, `actual_knife`, and `actual_guard`; remove
`guard_target`. Add `_init_capsule_between()` in this module (or reuse an
existing Viewer trail helper if present) to draw short segments between
consecutive samples. Render five compact cut marks as short vertical capsules
of radius no larger than `0.0015 m`. Preserve blue, cyan, and purple colors,
overlay throttling, cumulative minimum distance, and abort text.

- [ ] **Step 4: Feed the plan once and append only actual trace data**

Build the five 3-D contact positions from `plan.cut_points_xy` and the planned
blade contact height, then call `trace.set_plan(cut_points_xyz)` after preflight. Remove the
`guard_target` argument from `_observe_sample()` trace calls. The blue plan is
static, cyan and purple trails grow from actual samples.

- [ ] **Step 5: Run visualization and integration tests**

Run: `.venv/bin/pytest tests/simulation/test_guarded_chop_visualization.py tests/simulation/test_guarded_chop_integration.py -q -s`  
Expected: PASS; no test expects yellow target spheres.

- [ ] **Step 6: Commit**

```bash
git add src/twin_sim/guarded_chop_visualization.py src/twin_sim/tasks/guarded_chop.py tests/simulation/test_guarded_chop_visualization.py
git commit -m "feat: draw meaningful guarded chop trails"
```

---

### Task 5: CLI, documentation, and final verification

**Files:**
- Modify: `src/twin_sim/cli.py`
- Modify: `docs/simulation/usage.md`
- Modify: `docs/simulation/guarded_chopping_development_log.md`
- Modify: `tests/simulation/test_cli.py`

**Interfaces:**
- Produces: `twin-sim guarded-chop --scene plane|object`, default `plane`.
- Consumes: `GuardedChopConfig(scene_mode=args.scene)` and existing result summary.

- [ ] **Step 1: Write a failing CLI parser test**

```python
def test_guarded_chop_cli_defaults_to_plane_scene():
    args = build_parser().parse_args(["guarded-chop", "--headless"])
    assert args.scene == "plane"


def test_guarded_chop_cli_accepts_object_scene():
    args = build_parser().parse_args(["guarded-chop", "--scene", "object", "--headless"])
    assert args.scene == "object"
```

- [ ] **Step 2: Run the parser tests and verify `scene` is absent**

Run: `.venv/bin/pytest tests/simulation/test_cli.py -q`  
Expected: FAIL because `--scene` is not registered.

- [ ] **Step 3: Add the CLI argument and pass it into configuration**

```python
guarded_chop.add_argument(
    "--scene", choices=("plane", "object"), default="plane"
)
```

Construct `GuardedChopConfig(scene_mode=args.scene, final_hold_s=args.final_hold)`
in the guarded-chop command handler. Keep output fields for success, cuts,
shifts, total shift, minimum distance, and reason.

- [ ] **Step 4: Update user documentation and development log**

Document these commands exactly:

```bash
.venv/bin/twin-sim guarded-chop
.venv/bin/twin-sim guarded-chop --headless --final-hold 0
.venv/bin/twin-sim guarded-chop --scene object
```

Explain the blue planned path, cyan actual knife trail, purple actual guard
trail, five compact cut marks, right-to-left direction, and explicit hand
open-shift-close sequence. Record that object mode retains the prior contact
calibration and is not the default demonstration.

- [ ] **Step 5: Run targeted verification**

Run: `.venv/bin/pytest tests/simulation/test_guarded_chop_*.py tests/simulation/test_cli.py -q`  
Expected: all targeted tests pass.

- [ ] **Step 6: Run full verification**

Run: `.venv/bin/pytest -q`  
Expected: all tests pass; only the pre-existing chop contact-force warning may remain.

Run: `.venv/bin/twin-sim guarded-chop --headless --final-hold 0`  
Expected: `success=True cuts=5 shifts=4 total_shift_m=0.080`.

Run: `sha256sum -c docs/simulation/protected-files.sha256`  
Expected: every protected real-robot and SDK file reports `OK`.

Run: `git diff --check`  
Expected: no output and exit code 0.

- [ ] **Step 7: Perform one Viewer acceptance run**

Run: `.venv/bin/twin-sim guarded-chop --final-hold 10`  
Expected: five screen-right-to-left cuts; four clearly visible open-shift-close
cycles; blue/cyan/purple trails and five compact cut marks; automatic success exit.

- [ ] **Step 8: Commit**

```bash
git add src/twin_sim/cli.py docs/simulation/usage.md docs/simulation/guarded_chopping_development_log.md tests/simulation/test_cli.py
git commit -m "docs: expose plane guarded chopping demo"
```
