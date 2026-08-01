# Visible Reversible Guard Synergy Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make finger retraction and relaxation visibly comparable in the normal guarded-chop Viewer while retaining one exactly reversible 2.5-second hand trajectory and all existing safety interlocks.

**Architecture:** Keep the runtime state machine and minimum-jerk generator unchanged. Replace only the fixed retracted hand synergy after calibrating it against palm-frame MuJoCo link geometry, and lock both link-level visibility and forward/reverse trajectory symmetry with tests. The odd motion remains hand-only; the even motion remains the exact reverse hand path synchronized with the existing left-arm reset.

**Tech Stack:** Python 3.12, NumPy, MuJoCo FK, pytest, existing NPZ state replay

## Global Constraints

- Visual salience in the normal full-scene Viewer is the primary acceptance criterion.
- Retraction and relaxation must use the same 2.5-second trajectory in opposite directions.
- `finger3` must lead; `finger2`, `finger4`, and `finger5` must follow in decreasing layers; the thumb remains tucked.
- Planned finger3-pad retreat remains `0.020 ± 0.002 m` with user-approved orthogonal drift at or below `0.008 m`.
- Actual four-step world advance remains monotonic and totals `0.080 ± 0.005 m`.
- Never relax the 0.02 m knife-hand distance, zero blade-hand contact, speed interlocks, or actuator limits.
- Do not change object mode, physical robot, ROS 2, or Wuji Hand SDK paths.

---

### Task 1: Lock palm-frame visibility and reversibility contracts

**Files:**
- Modify: `tests/simulation/test_guarded_chop_posture.py`
- Modify: `tests/simulation/test_guarded_chop_preflight.py`

**Interfaces:**
- Consumes: `RightArmRobot`, `_preflight_guarded_chop(...)`, `GUARD_RELAXED_RAD`, `GUARD_RETRACTED_RAD`, and `_PlaneGuardMotion.hand`.
- Produces: test helper `_finger_link_state(robot, hand_rad, left_rad, finger, link) -> tuple[np.ndarray, np.ndarray]` returning palm-frame body position and link z-axis; visibility and exact reverse-path regression contracts.

- [ ] **Step 1: Add the palm-frame geometry helper and failing visibility test**

Add this helper to `tests/simulation/test_guarded_chop_posture.py`:

```python
def _finger_link_state(robot, hand_rad, left_rad, finger, link):
    saved = mujoco.MjData(robot.sim.model)
    mujoco.mj_copyData(saved, robot.sim.model, robot.sim.data)
    try:
        robot.sim.data.qpos[robot.sim.left.qpos_ids] = left_rad
        robot.sim.data.qpos[robot.sim.hand.qpos_ids] = hand_rad
        mujoco.mj_forward(robot.sim.model, robot.sim.data)
        palm = robot.sim.require_body("left_palm_link")
        body = robot.sim.require_body(f"left_finger{finger}_link{link}")
        palm_rotation = robot.sim.data.xmat[palm].reshape(3, 3)
        position = palm_rotation.T @ (
            robot.sim.data.xpos[body] - robot.sim.data.xpos[palm]
        )
        axis = palm_rotation.T @ robot.sim.data.xmat[body].reshape(3, 3)[:, 2]
        return position.copy(), axis.copy()
    finally:
        mujoco.mj_copyData(robot.sim.data, robot.sim.model, saved)
```

Add `test_guard_synergy_moves_visible_middle_links()` using the preflight left-ready pose. For finger3, require link3 displacement `>=0.012 m`, link4 displacement `>=0.018 m`, and link3 axis change `>=20°`. Require finger2 link4 displacement to exceed finger4, and finger4 to be at least finger5. Keep the existing pad retreat and actuator range tests.

- [ ] **Step 2: Run the visibility test and verify RED**

Run:

```bash
.venv/bin/python -m pytest tests/simulation/test_guarded_chop_posture.py::test_guard_synergy_moves_visible_middle_links -v
```

Expected: FAIL because the current finger3 link3/link4 displacements are approximately 8.7/13.9 mm and its link3 axis change is approximately 15.8°.

- [ ] **Step 3: Add exact forward/reverse trajectory assertions**

Extend `test_plane_preflight_builds_two_cut_inchworm_guard_motions()` in `tests/simulation/test_guarded_chop_preflight.py`:

```python
np.testing.assert_allclose(motions[1].hand, motions[0].hand[::-1], atol=0.0)
np.testing.assert_allclose(motions[3].hand, motions[2].hand[::-1], atol=0.0)
np.testing.assert_allclose(motions[0].hand, motions[2].hand, atol=0.0)
assert all(len(motion.hand) == 251 for motion in motions)
```

- [ ] **Step 4: Run the reverse-path test**

Run:

```bash
.venv/bin/python -m pytest tests/simulation/test_guarded_chop_preflight.py::test_plane_preflight_builds_two_cut_inchworm_guard_motions -v
```

Expected: PASS, documenting that the current generator already provides the required temporal symmetry.

- [ ] **Step 5: Commit the RED visibility contract**

```bash
git add tests/simulation/test_guarded_chop_posture.py tests/simulation/test_guarded_chop_preflight.py
git commit -m "test: require visible reversible guard motion"
```

### Task 2: Recalibrate the fixed synergy for visible link motion

**Files:**
- Modify: `src/twin_sim/guarded_chop_safety.py`
- Test: `tests/simulation/test_guarded_chop_posture.py`
- Test: `tests/simulation/test_guarded_chop_preflight.py`
- Test: `tests/simulation/test_guarded_chop_safety.py`

**Interfaces:**
- Consumes: the existing 20-element hand joint layout, actuator ranges, `_finger_pad_position(...)`, and palm-frame visibility tests from Task 1.
- Produces: a new immutable `GUARD_RETRACTED_RAD: np.ndarray` constant; no runtime optimizer or new public API.

- [ ] **Step 1: Run a deterministic offline candidate search**

Use a one-off `.venv/bin/python` diagnostic that enumerates the Cartesian product below relative to `GUARD_RELAXED_RAD[8:12]`. Joint indices in this grid are finger3 joint1 through joint4:

```python
delta_grids = (
    np.linspace(0.05, 0.45, 9),
    np.linspace(-0.30, 0.30, 13),
    np.linspace(np.deg2rad(20.0), 0.45, 6),
    np.linspace(-0.20, np.deg2rad(20.0), 12),
)
```

Derive follower deltas from the complete selected finger3 delta at finger2 `0.65`, finger4 `0.40`, and finger5 `0.38`, then reject actuator-out-of-range candidates and candidates unless all of these hold:

```python
0.018 <= pad_retreat_m <= 0.022
orthogonal_drift_m <= 0.008
finger3_link3_displacement_m >= 0.012
finger3_link4_displacement_m >= 0.018
finger3_link3_axis_change_deg >= 20.0
finger2_link4_displacement_m > finger4_link4_displacement_m
finger4_link4_displacement_m >= finger5_link4_displacement_m
```

Score remaining candidates lexicographically by greatest finger3 link3 displacement, greatest link3 axis change, then lowest orthogonal drift. Print the selected 20-element pose and all metrics. Do not save the search routine in production code.

- [ ] **Step 2: Replace only `GUARD_RETRACTED_RAD`**

Update the 20 numeric values in `src/twin_sim/guarded_chop_safety.py`. Keep `GUARD_RELAXED_RAD`, `CAT_PAW_RAD`, constant names, immutability flags, and object-mode poses unchanged.

- [ ] **Step 3: Run posture tests and verify GREEN**

Run:

```bash
.venv/bin/python -m pytest tests/simulation/test_guarded_chop_posture.py -v
```

Expected: all posture tests PASS, including visible link motion, layered followers, actuator ranges, 2 cm pad retreat, and at most 8 mm orthogonal drift.

- [ ] **Step 4: Run preflight and safety regressions**

Run:

```bash
.venv/bin/python -m pytest \
  tests/simulation/test_guarded_chop_preflight.py \
  tests/simulation/test_guarded_chop_safety.py -q
```

Expected: PASS with no knife-hand distance, collision, actuator, monotonic path, or reversibility failures.

- [ ] **Step 5: Commit the calibrated pose**

```bash
git add src/twin_sim/guarded_chop_safety.py tests/simulation/test_guarded_chop_posture.py tests/simulation/test_guarded_chop_preflight.py
git commit -m "feat: make guard finger reversal visibly symmetric"
```

### Task 3: Verify actual coupling, recording, and human-visible playback

**Files:**
- Modify: `docs/simulation/guarded_chopping_development_log.md`
- Modify: `docs/superpowers/specs/2026-08-01-visible-reversible-guard-synergy-design.md`
- Modify: `docs/superpowers/plans/2026-08-01-visible-reversible-guard-synergy.md`

**Interfaces:**
- Consumes: `run_guarded_chop(...)`, `GuardedChopRecording`, `recordings/guarded_chop_latest.npz`, and the existing 2× state replay path.
- Produces: fresh verification evidence and the user's Viewer verdict; no new runtime interface.

- [ ] **Step 1: Run guarded-chop integration tests**

Run:

```bash
.venv/bin/python -m pytest tests/simulation/test_guarded_chop_integration.py -q
```

Expected: 5 cuts, 4 shifts, monotonic actual guard motion, `0.080 ± 0.005 m` cumulative advance, minimum knife-hand distance at least 0.02 m, and zero blade-hand contacts.

- [ ] **Step 2: Run the complete regression suite**

Run:

```bash
.venv/bin/python -m pytest -q
```

Expected: all tests PASS; the existing 39.611 N contact-force observation warning may remain, but no new warning is accepted.

- [ ] **Step 3: Regenerate the default recording**

Run:

```bash
.venv/bin/twin-sim guarded-chop --headless --final-hold 0 --record
```

Expected: `success=True`, 5 cuts, 4 shifts, `total_shift_m=0.080`, minimum distance at least 0.02 m, and atomic overwrite of `recordings/guarded_chop_latest.npz`.

- [ ] **Step 4: Diagnose actual forward/reverse hand amplitudes**

Load the new NPZ and report, for all four guard phases, per-finger maximum actual joint change and palm-frame finger3 link3/link4 endpoint displacement. Require the two retractions and two relaxations to have comparable visible metrics; each phase must retain at least 80% of the corresponding planned link displacement, allowing position-servo tracking error.

- [ ] **Step 5: Play the latest recording at 2× in the normal full-scene Viewer**

Use the existing state replay API at `rate=2.0` with `GuardedChopTrace`. Do not substitute a close-up camera for acceptance. Ask the user whether both retraction and relaxation are now clearly visible; if not, return to Task 2 rather than declaring completion.

- [ ] **Step 6: Update verification documentation**

Append exact link displacement, axis-angle, actual tracking, pad steps, total advance, minimum distance, test count, warning count, and the user's pending/accepted verdict to the development log and design document. Mark plan checkboxes only for steps with fresh evidence.

- [ ] **Step 7: Commit verification records**

```bash
git add docs/simulation/guarded_chopping_development_log.md \
  docs/superpowers/specs/2026-08-01-visible-reversible-guard-synergy-design.md \
  docs/superpowers/plans/2026-08-01-visible-reversible-guard-synergy.md
git commit -m "docs: verify visible reversible guard motion"
```
