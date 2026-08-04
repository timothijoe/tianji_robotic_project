# Recorded-Hand Lateral Alignment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rotate the recorded Wuji guard anchor by world `-90 degree` so its wrist orientation and 30 mm retreat align with the guarded-chop cut direction (`+Y`).

**Architecture:** Keep MCAP data and relative palm transforms unchanged. Add one explicit chopping-frame alignment transform at the combined-task anchor boundary, then let the existing sequential IK and safety search recompute the full five-cycle plan.

**Tech Stack:** Python 3.12, NumPy, MuJoCo, pytest.

## Global Constraints

- Preserve the original MCAP hand joints, timing, and 30 mm retreat magnitude.
- Rotate wrist orientation and relative motion together; do not rewrite translation components.
- Horizontal retreat must point along the ordered cut direction within one degree.
- Re-run all IK, table-contact, thumb-clearance, and knife-distance checks.
- Keep the original `guarded-chop` task unchanged.

---

### Task 1: Align the complete recorded palm anchor

**Files:**
- Modify: `src/twin_sim/tasks/recorded_hand_guarded_chop.py`
- Modify: `tests/simulation/test_recorded_hand_guarded_chop_preflight.py`
- Modify: `docs/simulation/recorded_hand_guarded_chop.md`
- Modify: `docs/wuji/development_status.md`

**Interfaces:**
- Consumes: `RecordedGuardCycle.initial_palm_transform`, `RecordedGuardCycle.relative_palm_transforms`, and ordered `_GuardedChopPlan.cut_points_xy`.
- Produces: `_chopping_aligned_palm_rotation(initial_rotation) -> np.ndarray` and five `RecordedLeftCycle.palm_targets` whose horizontal displacement follows `+Y`.

- [ ] **Step 1: Write the failing direction and wrist-rotation tests**

Add a unit assertion that `_chopping_aligned_palm_rotation(R)` equals `Rz(-pi/2) @ diag(-1,-1,1) @ R`. Extend the real-plan test so every cycle's horizontal start-to-end displacement has positive `Y`, absolute `X <= 0.5 mm`, and angular error to the normalized ordered cut direction no greater than one degree.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
../../.venv-wuji-teleop/bin/pytest -q tests/simulation/test_recorded_hand_guarded_chop_preflight.py
```

Expected: import failure for `_chopping_aligned_palm_rotation` or direction assertion failure because the current displacement is world `-X`.

- [ ] **Step 3: Implement the single alignment transform**

Add:

```python
def _chopping_aligned_palm_rotation(initial_rotation: np.ndarray) -> np.ndarray:
    angle = -np.pi / 2.0
    world_quarter_turn = np.array(
        ((np.cos(angle), -np.sin(angle), 0.0),
         (np.sin(angle), np.cos(angle), 0.0),
         (0.0, 0.0, 1.0))
    )
    return world_quarter_turn @ np.diag((-1.0, -1.0, 1.0)) @ initial_rotation
```

Use it for the anchor rotation before multiplying every relative palm transform. Do not alter `relative_palm_transforms` or hand joint samples.

- [ ] **Step 4: Run focused and preserved-task regression**

Run:

```bash
../../.venv-wuji-teleop/bin/pytest -q \
  tests/simulation/test_recorded_hand_guarded_chop_preflight.py \
  tests/simulation/test_recorded_hand_guarded_chop.py \
  tests/simulation/test_guarded_chop_preflight.py \
  tests/simulation/test_guarded_chop_state_machine.py
```

Expected: all pass; the real plan still completes five cuts/five hand cycles and satisfies existing safety thresholds.

- [ ] **Step 5: Update operator documentation and verify Viewer**

Document that lateral `+Y` alignment replaces the previous front-back `-X` mapping. Run Headless once, then Viewer with a 15-second final hold and visually confirm the wrist is rotated 90 degrees and the hand retreats laterally.

- [ ] **Step 6: Commit**

```bash
git add src/twin_sim/tasks/recorded_hand_guarded_chop.py \
  tests/simulation/test_recorded_hand_guarded_chop_preflight.py \
  docs/simulation/recorded_hand_guarded_chop.md \
  docs/wuji/development_status.md
git commit -m "fix: align recorded guard with chopping direction"
```
