# Guard Finger Visibility Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the two-cut inchworm finger shape visibly obvious at 1× and 2× while preserving the accepted 2 cm pad retreat and all safety limits.

**Architecture:** Recalibrate only the fixed retracted synergy pose using offline FK search with a visual-amplitude objective, then give plane inchworm motions a dedicated 2.5-second duration. Runtime remains deterministic and uses the existing precomputed trajectories.

**Tech Stack:** Python 3.12, NumPy, MuJoCo FK, pytest

## Global Constraints

- Keep finger3-pad retreat at `0.020 ± 0.002 m` and orthogonal drift at or below `0.005 m`.
- Target a 25–30° visible principal change in finger3 and 10–15° visible changes in outer fingers where feasible.
- Keep the thumb tucked and every target inside actuator limits.
- Use `finger_motion_duration_s=2.5` only for plane inchworm retreat/reset; do not change object mode or knife lift/shift duration.
- Never relax the 0.02 m knife-hand hard limit or blade-hand zero-contact rule.

---

### Task 1: Recalibrate and lock the visible synergy

**Files:**
- Modify: `src/twin_sim/guarded_chop_safety.py`
- Modify: `tests/simulation/test_guarded_chop_posture.py`

**Interfaces:**
- Preserve `GUARD_RELAXED_RAD` and `GUARD_RETRACTED_RAD` names and 20-element layout.

- [ ] **Step 1: Tighten the failing visibility contract**

Add assertions that the largest actual target delta in finger3 is 25–30°, finger2 has at least 15° principal change, finger4/finger5 each have at least 10° visible principal change, and thumb change stays at or below 5°. Keep existing FK displacement/range tests.

- [ ] **Step 2: Run posture tests and verify RED**

Run `.venv/bin/python -m pytest tests/simulation/test_guarded_chop_posture.py -v`. Expected: current 13–15° target changes fail the new visibility thresholds.

- [ ] **Step 3: Run deterministic offline calibration and replace only the constant**

Search bounded finger3 joint combinations; derive layered finger2/finger4/finger5 changes with compensating proximal/distal motion; reject candidates outside FK displacement, orthogonal drift, range or thumb constraints. Store the best fixed pose, not the search routine.

- [ ] **Step 4: Verify GREEN and safety preflight**

Run posture, preflight and safety tests; expect all PASS.

- [ ] **Step 5: Commit**

Commit as `feat: exaggerate visible guard finger curl`.

### Task 2: Dedicated duration, full verification and Viewer

**Files:**
- Modify: `src/twin_sim/tasks/guarded_chop.py`
- Modify: `tests/simulation/test_guarded_chop_state_machine.py`
- Modify: `tests/simulation/test_guarded_chop_integration.py`
- Modify: `docs/simulation/guarded_chopping_development_log.md`
- Modify: `docs/superpowers/specs/2026-08-01-humanlike-guard-inchworm-design.md`

**Interfaces:**
- Add validated `GuardedChopConfig.finger_motion_duration_s: float = 2.5` and use it for all four plane guard motions only.

- [ ] **Step 1: Write failing duration test**

Assert the new default is 2.5, validates as an integer multiple of `control_dt_s`, and produces 251 samples per plane motion while object guard shifts retain their existing duration.

- [ ] **Step 2: Run focused tests and verify RED**

Run state-machine/preflight tests. Expected: missing config field.

- [ ] **Step 3: Implement dedicated duration**

Add the field to config validation and replace the two plane-motion uses of `hand_shift_duration_s` with `finger_motion_duration_s`.

- [ ] **Step 4: Verify actual tracking and full regression**

Run all guarded-chop tests and `.venv/bin/python -m pytest`. Record actual per-finger joint changes, pad displacement, cuts/shifts and minimum distance. Expect no new warnings.

- [ ] **Step 5: Run 2× Viewer and document evidence**

Regenerate `recordings/guarded_chop_latest.npz`, directly replay at 2×, record the pending/completed user verdict, update the design/development log, and commit as `docs: record visible guard finger verification`.
