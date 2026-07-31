# Low-Knife Guard Shift Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run the default plane chopping cycle as knife-down, low-position guard retreat, guard settle, then one diagonal knife lift-and-shift before the next cut.

**Architecture:** Preserve the existing independent arm position controllers and preflight-first execution model. Add a per-cut diagonal Cartesian knife trajectory to the guarded plan, replace plane coupled phases with low-guard phases, and make both preflight and runtime walk the same fixed-knife guard motion followed by fixed-guard knife motion.

**Tech Stack:** Python 3.12, NumPy, MuJoCo 3.10, pytest, existing Cartesian IK trajectory generation.

## Global Constraints

- Modify only MuJoCo simulation behavior; do not change or invoke ROS 2 or hardware send paths.
- Default `plane` completes exactly 5 right-to-left cuts and 4 guard retreats of `0.02 m`, totalling `0.08 m`.
- During every `LOW_GUARD_*` phase the right knife target is constant and actual right speed is at most `0.05 rad/s`.
- During `KNIFE_LIFT_SHIFT` the left arm and hand targets are constant and actual speeds are at most `0.05 rad/s`.
- Knife-to-guard distance remains at least `0.02 m`; blade/hand contact count remains zero.
- Each guard retreat ends no closer to the knife than it starts.
- `object` mode retains its conservative serial trajectory and matching preflight.
- `scripts/run_guarded_chop.sh` remains the default plane Viewer entry point.

---

### Task 1: Plan diagonal knife lift-and-shift trajectories

**Files:**
- Modify: `src/twin_sim/tasks/guarded_chop.py`
- Modify: `tests/simulation/test_guarded_chop_preflight.py`
- Modify: `tests/simulation/test_guarded_chop_state_machine.py`

**Interfaces:**
- Add `_GuardedChopPlan.knife_lift_shifts: tuple[tuple[TrajectoryPoint, ...], ...]`, containing four diagonal paths.
- Add phases `LOW_GUARD_OPEN`, `LOW_GUARD_SHIFT`, `LOW_GUARD_CLOSE`, `GUARD_SETTLE`, and `KNIFE_LIFT_SHIFT`; retain `KNIFE_CLEAR` only for final/object conservative recovery.
- Remove plane use of `COUPLED_OPEN`, `COUPLED_SHIFT`, and `COUPLED_CLOSE`.

- [ ] **Step 1: Write failing state and preflight tests**

Assert the new phase order and four diagonal paths. For every path, assert both vertical and XY motion and endpoint continuity:

```python
assert [phase.value for phase in GuardedChopPhase] == [
    "initialize", "guard_ready", "cut_down", "low_guard_open",
    "low_guard_shift", "low_guard_close", "guard_settle",
    "knife_lift_shift", "knife_clear", "complete", "aborted",
]

assert len(plan.knife_lift_shifts) == 4
for index, path in enumerate(plan.knife_lift_shifts):
    start = path[0].target_pose[:3, 3]
    end = path[-1].target_pose[:3, 3]
    assert end[2] > start[2] + 0.05
    assert np.linalg.norm(end[:2] - start[:2]) > 0.01
    np.testing.assert_allclose(
        path[0].target_pose, plan.cuts[index].descent[-1].target_pose
    )
    np.testing.assert_allclose(
        path[-1].target_pose, plan.cuts[index + 1].descent[0].target_pose
    )
```

- [ ] **Step 2: Run tests and verify failure**

Run: `.venv/bin/python -m pytest tests/simulation/test_guarded_chop_state_machine.py tests/simulation/test_guarded_chop_preflight.py -q`

Expected: failure because the new phases and `knife_lift_shifts` do not exist.

- [ ] **Step 3: Build diagonal paths during preflight**

After `_translate_right_cuts`, generate four Cartesian trajectories. Each starts at `cuts[index].descent[-1].target_pose`, ends at `cuts[index + 1].descent[0].target_pose`, seeds IK from the current contact joints, uses `config.knife_up_duration_s + config.hand_shift_duration_s` as the combined duration, and keeps the existing target orientation through `cartesian_trajectory`. Store them on `_GuardedChopPlan`.

- [ ] **Step 4: Run focused tests**

Run: `.venv/bin/python -m pytest tests/simulation/test_guarded_chop_state_machine.py tests/simulation/test_guarded_chop_preflight.py -q`

Expected: all tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/twin_sim/tasks/guarded_chop.py tests/simulation/test_guarded_chop_preflight.py tests/simulation/test_guarded_chop_state_machine.py
git commit -m "refactor: plan diagonal guarded knife shifts"
```

---

### Task 2: Enforce low-knife handover safety policy

**Files:**
- Modify: `src/twin_sim/guarded_chop_safety.py`
- Modify: `tests/simulation/test_guarded_chop_safety.py`

**Interfaces:**
- `SafetyCoordinator.evaluate` accepts stationary-knife guard motion in `LOW_GUARD_OPEN`, `LOW_GUARD_SHIFT`, and `LOW_GUARD_CLOSE`, regardless of knife height.
- `SafetyCoordinator.evaluate` accepts knife motion in `KNIFE_LIFT_SHIFT` only when the guard target and actual guard are stationary.

- [ ] **Step 1: Write failing policy tests**

```python
@pytest.mark.parametrize(
    "phase", ("LOW_GUARD_OPEN", "LOW_GUARD_SHIFT", "LOW_GUARD_CLOSE")
)
def test_low_guard_motion_requires_stationary_knife(phase):
    coordinator = SafetyCoordinator()
    allowed = observation(
        phase=phase, knife_height_m=0.33,
        right_target_stationary=True, right_speed_rad_s=0.0,
        left_target_stationary=False, left_speed_rad_s=0.2,
    )
    assert coordinator.evaluate(allowed).allowed
    decision = coordinator.evaluate(
        observation(
            phase=phase, knife_height_m=0.33,
            right_target_stationary=False, right_speed_rad_s=0.2,
            left_target_stationary=False, left_speed_rad_s=0.2,
        )
    )
    assert not decision.allowed
    assert "knife moved during low guard shift" in decision.reason


def test_lift_shift_requires_stationary_guard():
    decision = SafetyCoordinator().evaluate(
        observation(
            phase="KNIFE_LIFT_SHIFT", left_target_stationary=False,
            left_speed_rad_s=0.2, hand_speed_rad_s=0.2,
        )
    )
    assert not decision.allowed
    assert "guard moved during knife lift shift" in decision.reason
```

- [ ] **Step 2: Run tests and verify failure**

Run: `.venv/bin/python -m pytest tests/simulation/test_guarded_chop_safety.py -q`

Expected: the coordinator incorrectly permits the two invalid observations.

- [ ] **Step 3: Implement the phase rules**

Replace the `COUPLED_*` safe-height exception with two explicit rules: low-guard phases require stationary right target and `right_speed_rad_s <= 0.05`; `KNIFE_LIFT_SHIFT` requires stationary left target, `left_speed_rad_s <= 0.05`, and `hand_speed_rad_s <= 0.05`. Keep global finite state, clearance, penetration, force, and `CUT_DOWN` rules unchanged.

- [ ] **Step 4: Run safety tests**

Run: `.venv/bin/python -m pytest tests/simulation/test_guarded_chop_safety.py -q`

Expected: all tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/twin_sim/guarded_chop_safety.py tests/simulation/test_guarded_chop_safety.py
git commit -m "feat: enforce low-knife handover interlocks"
```

---

### Task 3: Execute and preflight the new plane sequence

**Files:**
- Modify: `src/twin_sim/tasks/guarded_chop.py`
- Modify: `tests/simulation/test_guarded_chop_integration.py`

**Interfaces:**
- Consumes `_GuardedChopPlan.knife_lift_shifts` from Task 1.
- Preserves `run_guarded_chop(...) -> GuardedChopResult` and result counters.

- [ ] **Step 1: Replace coupled integration assertions with failing ordered-motion assertions**

For each of the first four cuts, assert phase ordering, fixed right targets in low-guard phases, fixed left/hand targets in lift-shift, non-decreasing endpoint clearance, and diagonal knife motion:

```python
low = [s for s in result.samples if s.cut_index == index and s.phase in {
    GuardedChopPhase.LOW_GUARD_OPEN,
    GuardedChopPhase.LOW_GUARD_SHIFT,
    GuardedChopPhase.LOW_GUARD_CLOSE,
}]
lift = [s for s in result.samples if s.cut_index == index
        and s.phase is GuardedChopPhase.KNIFE_LIFT_SHIFT]
assert low and lift and low[-1].time_s < lift[0].time_s
right_targets = np.asarray([s.right_target_rad for s in low])
assert np.max(np.ptp(right_targets, axis=0)) <= 1e-12
left_targets = np.asarray([s.left_target_rad for s in lift])
hand_targets = np.asarray([s.hand_target_rad for s in lift])
assert np.max(np.ptp(left_targets, axis=0)) <= 1e-12
assert np.max(np.ptp(hand_targets, axis=0)) <= 1e-12
assert low[-1].knife_guard_distance_m >= low[0].knife_guard_distance_m - 1e-6
knife_positions = np.asarray([s.knife_position for s in lift])
assert np.ptp(knife_positions[:, 2]) > 0.05
assert np.linalg.norm(knife_positions[-1, :2] - knife_positions[0, :2]) > 0.01
```

- [ ] **Step 2: Run integration test and verify failure**

Run: `.venv/bin/python -m pytest tests/simulation/test_guarded_chop_integration.py -q`

Expected: failure because plane execution still emits `COUPLED_*` and moves the knife before the guard.

- [ ] **Step 3: Make plane preflight match the new execution order**

For plane mode in `_planned_clearances`, measure: descent at fixed guard; hand opening at fixed contact knife; left shift at fixed contact knife with open hand; hand close at fixed contact knife and shifted left arm; diagonal knife path at fixed shifted guard. Verify each low-shift endpoint distance is no smaller than its start (within `1e-6`) and raise `ValueError("guard retreat moved closer to knife")` otherwise. Leave the object branch unchanged.

- [ ] **Step 4: Execute the new phase order**

After each non-final descent, keep the contact target and wait until right-arm speed stays at or below `0.05 rad/s` for `interlock_settle_s`; abort with `"knife did not settle before guard shift"` on `stability_timeout_s`. Continue recording this short hold as `CUT_DOWN`, with the left side fixed. Then hold `cut.descent[-1].joints_rad` while executing hand open, left guard shift, hand close, and guard settle. Command `plan.knife_lift_shifts[index - 1][1:]` as `KNIFE_LIFT_SHIFT` while holding the closed hand and shifted left target. Remove plane runtime resampling/coupled commands. For the final cut, execute its normal retract as `KNIFE_CLEAR`.

- [ ] **Step 5: Run integration and safety suites**

Run: `.venv/bin/python -m pytest tests/simulation/test_guarded_chop_integration.py tests/simulation/test_guarded_chop_safety.py -q`

Expected: all tests pass; 5 cuts, 4 shifts, total `0.08 m`, no contact, minimum clearance at least `0.02 m`.

- [ ] **Step 6: Commit**

```bash
git add src/twin_sim/tasks/guarded_chop.py tests/simulation/test_guarded_chop_integration.py
git commit -m "feat: run low-knife guard handovers"
```

---

### Task 4: Update Viewer coverage, docs, and full verification

**Files:**
- Modify: `tests/simulation/test_guarded_chop_visualization.py`
- Modify: `docs/simulation/usage.md`
- Modify: `docs/superpowers/specs/2026-08-01-low-knife-guard-shift-design.md`

**Interfaces:**
- Viewer overlay displays `low_guard_shift` and `knife_lift_shift` from the generic trace API.

- [ ] **Step 1: Update overlay tests**

Replace the coupled-phase overlay test with samples for `low_guard_shift` and `knife_lift_shift`, asserting both exact phase strings appear in `viewer.texts[3]`.

- [ ] **Step 2: Run visualization tests**

Run: `.venv/bin/python -m pytest tests/simulation/test_guarded_chop_visualization.py -q`

Expected: all tests pass through the generic overlay formatter.

- [ ] **Step 3: Update user documentation**

Document the visible order as knife down, left open/retreat/close, diagonal knife lift-and-shift, next cut. Add an implementation-progress section to the new design with final test and headless metrics after verification.

- [ ] **Step 4: Run full verification**

```bash
.venv/bin/python -m pytest tests/simulation -q
bash -n scripts/run_guarded_chop.sh
git diff --check
.venv/bin/twin-sim guarded-chop --scene plane --headless --final-hold 0
.venv/bin/twin-sim guarded-chop --scene object --headless --final-hold 0
```

Expected: full suite passes; both scenes report `success=True cuts=5 shifts=4 total_shift_m=0.080`; each minimum distance is at least `0.020 m`.

- [ ] **Step 5: Commit**

```bash
git add tests/simulation/test_guarded_chop_visualization.py docs/simulation/usage.md docs/superpowers/specs/2026-08-01-low-knife-guard-shift-design.md
git commit -m "docs: describe low-knife guard handovers"
```

- [ ] **Step 6: Launch Viewer**

Run: `./scripts/run_guarded_chop.sh`

Expected: visible order is knife down, guard retreat, diagonal knife lift-and-shift, then the next descent; five cuts complete without abort.
