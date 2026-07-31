# Coupled Guarded Chop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the right knife recovery and left cat-paw handover visibly overlap while preserving the safe-height, stationary-during-cut, clearance, and five-cut/four-shift guarantees.

**Architecture:** Keep planning and execution in `guarded_chop.py`, but add pure progress-resampling helpers so independently sized joint trajectories can be commanded in one control loop. Split knife retract at the first safe-height point, execute the remaining retract with hand opening, then execute right and left Cartesian shifts together; the safety coordinator permits simultaneous targets only in explicitly coupled phases above the safe height.

**Tech Stack:** Python 3.12, NumPy, MuJoCo 3.10, pytest, existing `RightArmRobot` position-control interfaces.

## Global Constraints

- The default `plane` scene remains the implementation and demonstration target.
- Complete exactly 5 right-to-left cuts and 4 handovers of `0.02 m`, totalling `0.08 m`.
- Left-arm or hand motion is forbidden below the planned safe knife height.
- The left arm and hand remain stationary throughout `CUT_DOWN`.
- Knife-to-guard distance remains at least `0.02 m`; blade/hand contact remains zero.
- No ROS 2 or physical robot/hand command path is changed or invoked.
- `scripts/run_guarded_chop.sh` remains the default Viewer entry point.

---

### Task 1: Define coupled phases and trajectory synchronization primitives

**Files:**
- Modify: `src/twin_sim/tasks/guarded_chop.py`
- Modify: `tests/simulation/test_guarded_chop_state_machine.py`

**Interfaces:**
- Produces: `_resample_trajectory(points: Sequence[np.ndarray], count: int) -> tuple[np.ndarray, ...]`.
- Produces: `_first_safe_retract_index(robot: RightArmRobot, retract: Sequence[TrajectoryPoint], safe_height_m: float) -> int`.
- Produces phases: `KNIFE_CLEAR`, `COUPLED_OPEN`, `COUPLED_SHIFT`, `COUPLED_CLOSE`, and `GUARD_SETTLE`.

- [ ] **Step 1: Write failing phase and resampling tests**

Add imports for `_resample_trajectory` and assert the exact phase order:

```python
def test_phase_order_contains_coupled_actions():
    assert [phase.value for phase in GuardedChopPhase] == [
        "initialize", "guard_ready", "cut_down", "knife_clear",
        "coupled_open", "coupled_shift", "coupled_close",
        "guard_settle", "complete", "aborted",
    ]


def test_resample_trajectory_matches_endpoints_and_requested_count():
    points = (np.asarray([0.0, 2.0]), np.asarray([1.0, 4.0]))
    result = _resample_trajectory(points, 5)
    assert len(result) == 5
    np.testing.assert_allclose(result[0], points[0])
    np.testing.assert_allclose(result[-1], points[-1])
    np.testing.assert_allclose(result[2], [0.5, 3.0])


@pytest.mark.parametrize("count", (0, -1))
def test_resample_trajectory_rejects_non_positive_count(count):
    with pytest.raises(ValueError, match="count must be positive"):
        _resample_trajectory((np.zeros(2),), count)
```

- [ ] **Step 2: Run tests and verify they fail**

Run: `.venv/bin/python -m pytest tests/simulation/test_guarded_chop_state_machine.py -q`

Expected: collection fails because `_resample_trajectory` is absent, or assertions fail on old phases.

- [ ] **Step 3: Implement phases and pure resampling helper**

Replace the serial phase members and add:

```python
def _resample_trajectory(
    points: Sequence[np.ndarray], count: int
) -> tuple[np.ndarray, ...]:
    if count <= 0:
        raise ValueError("count must be positive")
    values = tuple(np.asarray(point, dtype=float) for point in points)
    if not values:
        raise ValueError("trajectory must not be empty")
    if len(values) == 1:
        return tuple(values[0].copy() for _ in range(count))
    source = np.linspace(0.0, 1.0, len(values))
    target = np.linspace(0.0, 1.0, count)
    stacked = np.asarray(values)
    return tuple(
        np.asarray([np.interp(t, source, stacked[:, axis])
                    for axis in range(stacked.shape[1])])
        for t in target
    )
```

Implement `_first_safe_retract_index` by evaluating `_blade_bottom_height` for each retract point and returning the first index whose height is at least `safe_height_m`; raise `ValueError("retract never reaches safe knife height")` when none qualifies.

- [ ] **Step 4: Run focused tests**

Run: `.venv/bin/python -m pytest tests/simulation/test_guarded_chop_state_machine.py -q`

Expected: all tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/twin_sim/tasks/guarded_chop.py tests/simulation/test_guarded_chop_state_machine.py
git commit -m "refactor: define coupled guarded chop phases"
```

---

### Task 2: Execute and preflight synchronized knife/guard motion

**Files:**
- Modify: `src/twin_sim/tasks/guarded_chop.py`
- Modify: `src/twin_sim/guarded_chop_safety.py`
- Modify: `tests/simulation/test_guarded_chop_integration.py`
- Modify: `tests/simulation/test_guarded_chop_safety.py`

**Interfaces:**
- Consumes: `_resample_trajectory(...)` and `_first_safe_retract_index(...)` from Task 1.
- Produces: plane-mode samples in which both sides move during `COUPLED_OPEN` or `COUPLED_SHIFT`.
- Preserves: `run_guarded_chop(config, viewer=False, trace=None, coordinator=None) -> GuardedChopResult`.

- [ ] **Step 1: Replace serial integration assertions with failing coupling assertions**

In the plane integration test, require each handover to contain coupled samples and retain the existing completion, clearance, no-contact and stationary-during-cut checks:

```python
for index in range(1, 5):
    coupled = [
        sample for sample in result.samples
        if sample.cut_index == index
        and sample.phase in {
            GuardedChopPhase.COUPLED_OPEN,
            GuardedChopPhase.COUPLED_SHIFT,
        }
    ]
    assert coupled
    assert any(
        sample.right_speed_rad_s > 0.05
        and (sample.left_speed_rad_s > 0.05
             or sample.hand_speed_rad_s > 0.05)
        for sample in coupled
    )
    assert min(sample.knife_height_m for sample in coupled) >= (
        plan.safe_knife_height_m
    )
```

Also assert every `KNIFE_CLEAR` sample has left and hand speed at most `0.05`, and every `CUT_DOWN` retains constant left targets and hand speed at most `0.05`.

- [ ] **Step 2: Add failing safety-policy tests**

Add cases showing `COUPLED_SHIFT` permits simultaneous targets above safe height and rejects the same observation below it:

```python
def test_coupled_shift_allows_both_targets_above_safe_height():
    observation = make_observation(
        phase="coupled_shift", knife_height_m=0.36,
        safe_knife_height_m=0.35,
        left_target_stationary=False, right_target_stationary=False,
        left_speed_rad_s=0.2, right_speed_rad_s=0.2,
    )
    assert SafetyCoordinator().evaluate(observation).allowed


def test_coupled_shift_rejects_motion_below_safe_height():
    observation = make_observation(
        phase="coupled_shift", knife_height_m=0.34,
        safe_knife_height_m=0.35,
        left_target_stationary=False, right_target_stationary=False,
        left_speed_rad_s=0.2, right_speed_rad_s=0.2,
    )
    decision = SafetyCoordinator().evaluate(observation)
    assert not decision.allowed
    assert "safely raised" in decision.reason
```

- [ ] **Step 3: Run the new tests and verify failure**

Run: `.venv/bin/python -m pytest tests/simulation/test_guarded_chop_integration.py tests/simulation/test_guarded_chop_safety.py -q`

Expected: failures because execution remains serial and safety does not recognize coupled phases.

- [ ] **Step 4: Update the safety coordinator**

Treat `COUPLED_OPEN`, `COUPLED_SHIFT`, and `COUPLED_CLOSE` as guarded-motion phases. For them, require `knife_height_m >= safe_knife_height_m`, but do not require `right_target_stationary`. Preserve the existing `CUT_DOWN` rule and global finite-state, clearance, contact-force, and penetration rules. `KNIFE_CLEAR` retains stationary left targets through execution and does not need a new exception.

- [ ] **Step 5: Execute the coupled trajectories**

For each non-final cut:

1. Find the safe split in `cut.retract` and command its prefix as `KNIFE_CLEAR` with left targets unchanged.
2. Resample the remaining retract joints and `_joint_trajectory(CAT_PAW_RAD, CAT_PAW_OPEN_RAD, ...)` to their maximum point count; command both in each `COUPLED_OPEN` step.
3. Resample `cut.shift` joints and `plan.guard_shifts[index - 1]` joints to their maximum point count; command right and left in each `COUPLED_SHIFT` step while holding `CAT_PAW_OPEN_RAD`.
4. Hold the right safe target and execute the hand close trajectory as `COUPLED_CLOSE`.
5. Call `wait_for_guard_stability` using `GUARD_SETTLE`, then increment the completed shift count.

For the final cut, command the entire retract as `KNIFE_CLEAR` and do not open or shift the left hand. Keep object mode on its existing conservative behavior, adapting only renamed phase constants as required.

Update `_planned_clearances` to evaluate the exact paired right/left samples used by `COUPLED_OPEN` and `COUPLED_SHIFT`, rather than measuring each arm's trajectory separately. This makes preflight cover simultaneous motion.

- [ ] **Step 6: Run integration and safety tests**

Run: `.venv/bin/python -m pytest tests/simulation/test_guarded_chop_integration.py tests/simulation/test_guarded_chop_safety.py -q`

Expected: all tests pass, including 5 cuts, 4 coupled shifts, `0.08 m` total shift, no blade/hand contact, and minimum distance at least `0.02 m`.

- [ ] **Step 7: Commit**

```bash
git add src/twin_sim/tasks/guarded_chop.py src/twin_sim/guarded_chop_safety.py tests/simulation/test_guarded_chop_integration.py tests/simulation/test_guarded_chop_safety.py
git commit -m "feat: couple guarded chop handovers"
```

---

### Task 3: Update visualization, documentation, and end-to-end verification

**Files:**
- Modify: `tests/simulation/test_guarded_chop_visualization.py`
- Modify: `docs/simulation/usage.md`
- Modify: `docs/superpowers/specs/2026-08-01-coupled-guarded-chop-design.md`

**Interfaces:**
- Consumes: coupled phase strings emitted by `run_guarded_chop`.
- Produces: Viewer overlay displaying `coupled_open`, `coupled_shift`, `coupled_close`, and `guard_settle` as live phases.

- [ ] **Step 1: Add a failing overlay test for the coupled phase**

```python
def test_overlay_displays_coupled_shift_phase():
    viewer = FakeViewer()
    trace = GuardedChopTrace(viewer)
    trace.append(
        actual_knife=(0.7, 0.04, 0.36),
        actual_guard=(0.6, 0.10, 0.36),
        phase="coupled_shift",
        cut_index=2,
        minimum_distance_m=0.04,
        cut_allowed=False,
    )
    assert "phase=coupled_shift" in viewer.texts[3]
```

- [ ] **Step 2: Run the visualization test**

Run: `.venv/bin/python -m pytest tests/simulation/test_guarded_chop_visualization.py -q`

Expected: pass if the existing generic overlay already supports the phase; otherwise fail and update only the phase formatting needed to pass.

- [ ] **Step 3: Update usage and progress documentation**

Change `docs/simulation/usage.md` to explain that above the safe height the right knife retract/shift overlaps Wuji Hand opening and left-arm retreat, while the next descent waits for guard stability. Add an `## 实现进展` section to the design document recording the final commit/test/Viewer evidence without changing the approved safety requirements.

- [ ] **Step 4: Run full verification**

Run:

```bash
.venv/bin/python -m pytest tests/simulation -q
bash -n scripts/run_guarded_chop.sh
git diff --check
.venv/bin/twin-sim guarded-chop --scene plane --headless --final-hold 0
```

Expected: all simulation tests pass; shell and diff checks exit 0; headless output reports `success=True cuts=5 shifts=4 total_shift_m=0.080`, with minimum distance at least `0.020`.

- [ ] **Step 5: Commit documentation and visualization coverage**

```bash
git add tests/simulation/test_guarded_chop_visualization.py docs/simulation/usage.md docs/superpowers/specs/2026-08-01-coupled-guarded-chop-design.md
git commit -m "docs: describe coupled guarded chop motion"
```

- [ ] **Step 6: Launch the Viewer for user inspection**

Run: `./scripts/run_guarded_chop.sh`

Expected: Viewer visibly shows four overlapping right-knife/left-guard handovers, completes five cuts, prints `success=True`, and never enters `ABORTED`.
