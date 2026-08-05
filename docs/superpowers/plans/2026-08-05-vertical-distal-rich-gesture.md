# Vertical Distal Rich Gesture Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make long-finger distal phalanges press nearly vertically into the table while restoring clearly visible recorded PIP/DIP motion.

**Architecture:** Keep the existing MCP-limited PIP-led trajectory as the input. Add a MuJoCo-aware distal shaper that solves each long-finger DIP on a bounded one-dimensional grid in palm coordinates, blends toward the solution during late PREPARE, and enforces the vertical pose throughout RETREAT/HOLD before the continuous wrist compensation is recomputed.

**Tech Stack:** Python 3.12, NumPy, MuJoCo, pytest.

## Global Constraints

- Distal direction is `left_finger2_link4` through `left_finger5_link4` local `+Z`.
- RETREAT/HOLD distal angle to world `-Z` is `<= 15 degrees`.
- MCP `<= 0.30 rad`; PIP remains at least `0.25 rad` more flexed.
- Index/middle/ring PIP peak-to-peak `>= 0.20 rad`; little PIP `>= 0.12 rad`; every DIP `>= 0.10 rad`.
- PREPARE blends smoothly into pressing; joint steps remain `<= 0.12 rad`.
- Recompute continuous wrist/pad compensation and all existing safety metrics.
- Preserve pure-table scene, no RESET, old tasks, and simulation-only boundary.

---

### Task 1: MuJoCo-aware distal orientation shaping

**Files:**
- Modify: `src/twin_sim/tasks/recorded_hand_guarded_chop.py`
- Modify: `tests/simulation/test_recorded_hand_guarded_chop_preflight.py`

**Interfaces:**
- Produces: `_shape_vertical_distal_guard(robot, hand_positions, phases, palm_rotation, maximum_angle_deg=15.0) -> np.ndarray`.

- [x] **Step 1: Write failing orientation/amplitude tests**

On the real plan, set every planned left/hand sample in MuJoCo. For RETREAT and
HOLD, measure each link4 local `+Z` against world `-Z` and assert maximum angle
`<= 15 degrees`. Assert MCP/PIP constraints, required PIP/DIP peak-to-peak
amplitudes, positive middle/ring correlation without equality, and hand step
`<= 0.12 rad`.

- [x] **Step 2: Run RED**

Expected: current maximum distal angles are approximately 28-58 degrees.

- [x] **Step 3: Implement bounded DIP search**

For each sample and long finger, evaluate 61 uniformly spaced DIP candidates
inside its actuator range. Convert the candidate link4 axis from current palm
coordinates into the requested planned palm rotation and select the candidate
with minimum angle to world `-Z`, using distance to the recorded DIP as a
tie-breaker. Solve all RETREAT/HOLD samples plus the final 0.30 seconds of
PREPARE; blend the PREPARE solutions with minimum-jerk weight. Reject any
active sample whose best angle exceeds 15 degrees or any trajectory exceeding
joint-step/amplitude constraints.

- [x] **Step 4: Integrate before wrist compensation**

Call the distal shaper after `shape_pip_led_guard_hand` and before computing
pad offsets. Clip numerical boundary noise, then let the existing pad-based
carrier recompute its monotonic 32 mm retreat.

---

### Task 2: Regression, evidence, and documentation

**Files:**
- Modify: `docs/simulation/recorded_hand_guarded_chop.md`
- Modify: `docs/wuji/development_status.md`
- Modify: `tests/simulation/test_recorded_hand_guarded_chop.py` only if result evidence needs expansion.

- [x] **Step 1: Run focused and preserved-task regression**

```bash
.venv-wuji-teleop/bin/pytest -q \
  tests/simulation/test_recorded_hand_guard.py \
  tests/simulation/test_recorded_hand_guarded_chop_preflight.py \
  tests/simulation/test_recorded_hand_guarded_chop.py \
  tests/simulation/test_guarded_chop_preflight.py \
  tests/simulation/test_guarded_chop_state_machine.py
```

- [x] **Step 2: Verify Headless and Viewer**

Headless must complete five cuts/five continuous segments. Viewer acceptance
must visibly show downward distal phalanges, a mostly straight MCP row, richer
index/middle-ring/little motion, and no wrist reset.

- [x] **Step 3: Document measured evidence**

Record maximum distal angle, per-finger PIP/DIP peak-to-peak motion, wrist
retreat, knife clearance, table penetration, and thumb clearance.

- [x] **Step 4: Commit**

```bash
git add src/twin_sim/tasks/recorded_hand_guarded_chop.py \
  tests/simulation/test_recorded_hand_guarded_chop_preflight.py \
  docs/simulation/recorded_hand_guarded_chop.md docs/wuji/development_status.md
git commit -m "fix: add vertical distal pressing gesture"
```
