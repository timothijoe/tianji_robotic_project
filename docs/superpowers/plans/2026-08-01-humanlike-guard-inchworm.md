# Human-like Guard Inchworm Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the default plane guarded-chop hand's mechanical open/shift/close cycle with a two-cut inchworm gait: finger-only 2 cm retreat after cuts 1/3, then synchronized 4 cm arm reset and finger relaxation after cuts 2/4.

**Architecture:** Introduce two FK-validated hand synergy poses and precompute four plane guard motions as paired left-arm/hand trajectories. The runtime state machine consumes those trajectories under the existing low-knife interlocks; object mode keeps its current conservative guard shifts.

**Tech Stack:** Python 3.12, NumPy, MuJoCo 3.10, existing Cartesian IK/minimum-jerk helpers, pytest

## Global Constraints

- Plane mode keeps 5 cuts, 4 logical 2 cm guard advances and `total_shift_m=0.080`.
- Cuts 1/3 use finger-only retreat with identical left-arm targets; cuts 2/4 use approximately 4 cm left-arm translation while fingers relax.
- `finger3` pad retreat per logical advance is `0.020 ± 0.002 m`; orthogonal pad drift is at most 0.005 m.
- The knife remains low and target-stationary during every guard motion; all existing minimum-distance, zero blade-hand contact, stability, velocity and acceleration checks remain active.
- Object mode, ROS 2, hardware code, position-control architecture, recording schema and existing launch commands remain compatible.
- Strategies B (online hand IK) and C (manual pose library) remain documented only and are not implemented.

---

### Task 1: Calibrated layered cat-paw synergy

**Files:**
- Modify: `src/twin_sim/guarded_chop_safety.py`
- Modify: `tests/simulation/test_guarded_chop_posture.py`
- Modify: `src/twin_sim/tasks/guarded_chop.py`

**Interfaces:**
- Produce `GUARD_RELAXED_RAD` and `GUARD_RETRACTED_RAD`, both 20-element immutable arrays.
- Produce `_finger_pad_position(robot, hand_rad, left_rad)` and `_minimum_jerk_joint_trajectory(start, goal, duration, dt)` helpers.

- [ ] **Step 1: Write failing posture and FK contract tests**

Test both poses against actuator ranges, require `finger3` relative joint change to exceed `finger2`, `finger2` to exceed `finger4`, and `finger4` to be no smaller than `finger5`; constrain thumb change. With the guarded ready palm fixed, assert the `left_finger3_pad` displacement projected onto the right-to-left guard direction is `0.020 ± 0.002 m` and orthogonal displacement is at most `0.005 m`.

- [ ] **Step 2: Run posture tests and verify RED**

Run `.venv/bin/python -m pytest tests/simulation/test_guarded_chop_posture.py -v`. Expected: import failure for the new synergy constants/helpers.

- [ ] **Step 3: Calibrate and implement the two poses**

Use deterministic offline FK search against the current hand joint ranges to select a natural central-finger-led solution meeting the exact displacement contract. Store only the resulting constants at runtime; do not add SciPy or an online optimizer. Implement a minimum-jerk interpolator using the existing `minimum_jerk` blend.

- [ ] **Step 4: Verify GREEN**

Run posture tests and the existing hand/model tests; expect all PASS.

- [ ] **Step 5: Commit**

Commit as `feat: add layered guard hand synergy`.

### Task 2: Precompute two-cut plane guard motions

**Files:**
- Modify: `src/twin_sim/tasks/guarded_chop.py`
- Modify: `tests/simulation/test_guarded_chop_preflight.py`
- Modify: `tests/simulation/test_guarded_chop_safety.py`

**Interfaces:**
- Add private `_PlaneGuardMotion` with `phase`, `left`, and `hand` trajectories of equal length.
- Extend `_GuardedChopPlan` with four `plane_guard_motions`; retain existing `guard_shifts` for object mode.
- Add phases `FINGER_RETRACT = "finger_retract"` and `ARM_RESET_RELAX = "arm_reset_relax"` while retaining old enum values for object compatibility.

- [ ] **Step 1: Write failing preflight tests**

Assert four motions with phases `[FINGER_RETRACT, ARM_RESET_RELAX, FINGER_RETRACT, ARM_RESET_RELAX]`; odd left trajectories are exactly stationary; even palm translations are `0.040 ± 0.002 m`; paired trajectories have equal sample counts and exact endpoints. Project every pad trajectory onto the guard direction and assert monotonicity, `0.020 ± 0.002 m` net advance and at most 5 mm orthogonal drift.

- [ ] **Step 2: Run preflight tests and verify RED**

Run the guarded-chop preflight/state-machine tests. Expected: missing phases and plan motions.

- [ ] **Step 3: Implement plane motion planning**

Build cuts 1/3 as stationary repeated left targets paired with relaxed-to-retracted hand trajectories. Build cuts 2/4 as 4 cm Cartesian left-palm trajectories paired sample-for-sample with retracted-to-relaxed minimum-jerk hand targets. Validate endpoints, monotonic pad progress, orthogonal drift, hand ranges, motion limits and knife-hand clearance during preflight.

- [ ] **Step 4: Preserve object preflight and verify GREEN**

Run plane and object preflight/safety tests; object mode must retain four 2 cm arm shifts and contact-latched behavior.

- [ ] **Step 5: Commit**

Commit as `feat: plan two-cut guard inchworm gait`.

### Task 3: Execute the inchworm state machine and trace the pad

**Files:**
- Modify: `src/twin_sim/tasks/guarded_chop.py`
- Modify: `src/twin_sim/guarded_chop_visualization.py`
- Modify: `tests/simulation/test_guarded_chop_integration.py`
- Modify: `tests/simulation/test_guarded_chop_visualization.py`

**Interfaces:**
- Plane mode consumes `plan.plane_guard_motions[index - 1]` after each non-final cut.
- `GuardedChopSample.guard_position` becomes the `left_finger3_pad` world position; whole-hand safety distance remains unchanged.
- Existing result fields keep their logical-advance semantics.

- [ ] **Step 1: Write failing integration and visualization tests**

For cuts 1/3, assert every `FINGER_RETRACT` sample has an unchanged left target and a changing hand target. For cuts 2/4, assert `ARM_RESET_RELAX` contains simultaneous left and hand target changes. Assert four consecutive guard endpoints each move `0.020 ± 0.002 m`, completed shifts remain 4, total shift remains 0.080, and the trace receives finger-pad positions.

- [ ] **Step 2: Run focused tests and verify RED**

Run guarded-chop integration and visualization tests. Expected: current open/shift/close phases violate the new sequence.

- [ ] **Step 3: Implement runtime consumption**

Replace only the plane open/shift/close loops with the paired motion loop; command left and hand targets together, maintain right target lock, wait for joint/hand stability, then execute the existing diagonal knife lift/shift. Change the observed purple guard point from knuckle site to finger3 pad geom.

- [ ] **Step 4: Run all guarded-chop tests and verify GREEN**

Run all `tests/simulation/test_guarded_chop_*.py`; expect plane inchworm and unchanged object mode tests PASS.

- [ ] **Step 5: Commit**

Commit as `feat: execute humanlike guard inchworm gait`.

### Task 4: Recording, full regression, documentation and visual acceptance

**Files:**
- Modify: `tests/simulation/test_guarded_chop_recording.py`
- Modify: `docs/simulation/usage.md`
- Modify: `docs/simulation/guarded_chopping_development_log.md`
- Modify: `docs/superpowers/specs/2026-08-01-humanlike-guard-inchworm-design.md`

**Interfaces:**
- Existing recording fields and NPZ schema stay version 1; `guard_position` now semantically records the finger3 pad.
- Existing 1× and 1×→2× launchers remain unchanged.

- [ ] **Step 1: Add recording regression**

Assert captured/replayed `guard_position` follows the pad data supplied by the task without changing NPZ schema or frame dimensions.

- [ ] **Step 2: Run full verification**

Run `.venv/bin/python -m pytest`; expect no failures and no new warnings beyond the existing 39.611 N force observation warning. Run headless plane and object tasks and record exact cut/shift/distance metrics.

- [ ] **Step 3: Run Viewer acceptance**

Run `./scripts/run_guarded_chop.sh`, then `./scripts/run_guarded_chop_record_replay.sh`. Confirm odd-cycle palm stillness, even-cycle simultaneous arm reset/finger relaxation, layered finger motion and correct 2× replay.

- [ ] **Step 4: Update docs with exact evidence**

Describe the two-cut cycle, numbered-finger mapping caveat, pad trace semantics, unchanged object mode, test counts and pending/completed user visual verdict. Keep B/C as future approaches.

- [ ] **Step 5: Commit**

Commit as `docs: record humanlike guard gait verification`.
