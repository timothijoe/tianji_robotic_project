# Synchronized Close Guard Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run all five knife cuts in parallel with the continuous recorded left-hand guard while holding a 30 mm robot-lateral knife-to-pad offset.

**Architecture:** Preflight builds five `SynchronizedGuardCycle` objects, each containing equal-length right-arm, left-arm, and hand samples on one clock. The right knife and left wrist share a 32 mm `+Y` carrier over the complete plan; knife vertical motion is superimposed on that carrier. Safety is measured on every synchronized sample before runtime executes the immutable plan.

**Tech Stack:** Python 3, NumPy, MuJoCo, existing Tianji IK and Wuji MCAP pipeline, pytest.

## Global Constraints

- Preserve the existing `guarded-chop` task and launch scripts.
- Use exactly five cycles and one continuous hand recording without inter-cut reset.
- Total knife and wrist carrier travel is `0.032 m`, approximately `0.008 m` per cut interval.
- The nearest long-finger pad stays `0.030 m +/- 0.003 m` robot-left of the blade reference.
- Full-geometry knife/hand clearance stays at least `0.020 m` at every sample.
- Hand/table penetration stays at most `0.0005 m`; thumb clearance stays at least `0.010 m`.
- All three command vectors advance in the same simulation sample.

---

### Task 1: Build one-clock synchronized cycles

**Files:**
- Modify: `src/twin_sim/tasks/recorded_hand_guarded_chop.py`
- Test: `tests/simulation/test_recorded_hand_guarded_chop_preflight.py`

**Interfaces:**
- Consumes: `_TableCutPlan.cuts`, the five contiguous `RecordedLeftCycle` values, and `RecordedHandGuardedChopConfig.total_wrist_retreat_m`.
- Produces: `SynchronizedGuardCycle(right: np.ndarray, left: np.ndarray, hand: np.ndarray, knife_phases: tuple[str, ...], palm_targets: np.ndarray)` and `RecordedHandGuardedChopPlan.synchronized_cycles`.

- [ ] **Step 1: Write the failing synchronization test**

Add assertions to the real-plan test:

```python
assert len(plan.synchronized_cycles) == 5
for synchronized in plan.synchronized_cycles:
    assert len(synchronized.right) == len(synchronized.left)
    assert len(synchronized.left) == len(synchronized.hand)
    assert "CUT_DOWN" in synchronized.knife_phases
    assert "KNIFE_RETRACT" in synchronized.knife_phases
knife_y = np.concatenate([
    value.knife_targets[:, 1, 3] for value in plan.synchronized_cycles
])
assert np.all(np.diff(knife_y) >= -1e-9)
assert knife_y[-1] - knife_y[0] == pytest.approx(.032, abs=.001)
```

- [ ] **Step 2: Verify the new test fails for the missing interface**

Run: `.venv-wuji-teleop/bin/pytest tests/simulation/test_recorded_hand_guarded_chop_preflight.py::test_real_plan_contains_five_safe_recorded_cycles_and_five_right_cuts -q`

Expected: FAIL because `RecordedHandGuardedChopPlan` has no `synchronized_cycles`.

- [ ] **Step 3: Implement synchronized trajectory construction**

Add the immutable synchronized-cycle record and a helper with this contract:

```python
@dataclass(frozen=True)
class SynchronizedGuardCycle:
    right: np.ndarray
    left: np.ndarray
    hand: np.ndarray
    knife_phases: tuple[str, ...]
    palm_targets: np.ndarray
    knife_targets: np.ndarray

def _build_synchronized_cycles(
    robot: RightArmRobot,
    cuts: tuple[_CutTrajectories, ...],
    left_cycles: tuple[RecordedLeftCycle, ...],
    total_carrier_m: float,
    config: RecordedHandGuardedChopConfig,
) -> tuple[SynchronizedGuardCycle, ...]:
    """Retimes knife down/up paths onto each hand segment and applies one +Y carrier."""
```

For every cycle, create a vertical down/up target-pose waveform, linearly interpolate the common carrier from cycle start to end, solve the right-arm path with the preceding right solution as seed, and retain one sample from each side at each timestamp. Do not append the old shift path.

- [ ] **Step 4: Verify focused preflight test passes**

Run: `.venv-wuji-teleop/bin/pytest tests/simulation/test_recorded_hand_guarded_chop_preflight.py -q`

Expected: PASS.

- [ ] **Step 5: Commit synchronized planning**

```bash
git add src/twin_sim/tasks/recorded_hand_guarded_chop.py tests/simulation/test_recorded_hand_guarded_chop_preflight.py
git commit -m "feat: synchronize knife and recorded guard plans"
```

### Task 2: Enforce close fixed lateral spacing per sample

**Files:**
- Modify: `src/twin_sim/tasks/recorded_hand_guarded_chop.py`
- Test: `tests/simulation/test_recorded_hand_guarded_chop_preflight.py`

**Interfaces:**
- Consumes: `SynchronizedGuardCycle` complete joint samples and the existing MuJoCo blade/hand geometry.
- Produces: `minimum_lateral_spacing_m`, `maximum_lateral_spacing_m`, and per-sample rejection against configured `target_lateral_spacing_m=0.030` and `lateral_spacing_tolerance_m=0.003`.

- [ ] **Step 1: Write failing fixed-spacing assertions**

```python
assert plan.minimum_lateral_spacing_m >= .027
assert plan.maximum_lateral_spacing_m <= .033
assert plan.minimum_planned_distance_m >= .020
```

Also replace legacy 180/239 mm palm-offset assertions with checks that consecutive knife and wrist carrier increments match within `0.001 m`.

- [ ] **Step 2: Verify failure reports the missing spacing fields or old far anchor**

Run: `.venv-wuji-teleop/bin/pytest tests/simulation/test_recorded_hand_guarded_chop_preflight.py::test_real_plan_contains_five_safe_recorded_cycles_and_five_right_cuts -q`

Expected: FAIL before implementation.

- [ ] **Step 3: Implement the lateral metric and anchor calibration**

Extend configuration and validation:

```python
target_lateral_spacing_m: float = 0.030
lateral_spacing_tolerance_m: float = 0.003
```

Measure `nearest_pad_y - blade_reference_y` after `mj_forward` for every synchronized sample. Calibrate the palm anchor from the first-sample measured pad offset, then validate the complete range. Extend `_measure_plan_safety` to return complete-geometry clearance, penetration, thumb clearance, and lateral min/max from the synchronized samples only.

- [ ] **Step 4: Run preflight regression**

Run: `.venv-wuji-teleop/bin/pytest tests/simulation/test_recorded_hand_guarded_chop_preflight.py -q`

Expected: PASS with lateral range inside `[0.027, 0.033]` and geometry clearance at least `0.020`.

- [ ] **Step 5: Commit close-spacing preflight**

```bash
git add src/twin_sim/tasks/recorded_hand_guarded_chop.py tests/simulation/test_recorded_hand_guarded_chop_preflight.py
git commit -m "feat: enforce close knife guard spacing"
```

### Task 3: Execute synchronized samples and document verification

**Files:**
- Modify: `src/twin_sim/tasks/recorded_hand_guarded_chop.py`
- Modify: `tests/simulation/test_recorded_hand_guarded_chop.py`
- Modify: `docs/simulation/recorded_hand_guarded_chop.md`
- Modify: `docs/simulation/current_version_handoff.md`
- Modify: `docs/wuji/development_status.md`

**Interfaces:**
- Consumes: `RecordedHandGuardedChopPlan.synchronized_cycles`.
- Produces: runtime events `SYNC_CYCLE_START` and `SYNC_CYCLE_COMPLETE`, plus lateral-spacing metrics in `RecordedHandGuardedChopResult`.

- [ ] **Step 1: Write the failing runtime behavior test**

Replace serial event-order assertions with:

```python
for cycle in range(1, 6):
    assert (cycle, "SYNC_CYCLE_START") in result.events
    assert (cycle, "SYNC_CYCLE_COMPLETE") in result.events
assert result.minimum_lateral_spacing_m >= .027
assert result.maximum_lateral_spacing_m <= .033
```

- [ ] **Step 2: Verify runtime test fails on serial events**

Run: `.venv-wuji-teleop/bin/pytest tests/simulation/test_recorded_hand_guarded_chop.py -q`

Expected: FAIL because runtime still emits `HAND_SAFE` before `CUT_DOWN` and result lacks spacing fields.

- [ ] **Step 3: Replace the serial runtime loop**

Iterate each synchronized cycle once:

```python
for right_rad, left_rad, hand_rad, knife_phase in zip(
    cycle.right, cycle.left, cycle.hand, cycle.knife_phases, strict=True
):
    execute(
        RecordedHandGuardedChopPhase(knife_phase.lower()),
        right_rad,
        left_rad,
        hand_rad,
        cycle_index,
    )
```

Emit start/complete events around the loop, count one cut and one hand cycle after each synchronized interval, and expose preflight lateral min/max in success and aborted results.

- [ ] **Step 4: Run focused and broader automated verification**

Run: `.venv-wuji-teleop/bin/pytest tests/simulation/test_recorded_hand_guarded_chop.py tests/simulation/test_recorded_hand_guarded_chop_preflight.py tests/simulation/test_recorded_hand_guard.py tests/simulation/test_cli.py -q`

Expected: PASS.

- [ ] **Step 5: Run the real Headless command**

Run: `.venv-wuji-teleop/bin/twin-sim recorded-hand-guarded-chop --headless --final-hold 0 --hand-mcap recordings/wuji/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap`

Expected: exit code 0, five completed cuts, five completed hand cycles, lateral spacing in `[0.027, 0.033]`, geometry clearance at least `0.020`.

- [ ] **Step 6: Update operational and handoff documentation**

Record the shared-carrier motivation, synchronized data flow, exact safety thresholds, test commands, measured results, and the Viewer command in all three listed documents.

- [ ] **Step 7: Commit runtime and documentation**

```bash
git add src/twin_sim/tasks/recorded_hand_guarded_chop.py tests/simulation/test_recorded_hand_guarded_chop.py docs/simulation/recorded_hand_guarded_chop.md docs/simulation/current_version_handoff.md docs/wuji/development_status.md
git commit -m "feat: run close guarded cuts in parallel"
```
