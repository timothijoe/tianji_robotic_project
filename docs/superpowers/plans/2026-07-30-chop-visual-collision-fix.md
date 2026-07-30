# Chop Visual and Collision Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the chopping motion visibly clear and prevent the cleaver collision geometry from penetrating the chopping board beyond 5 mm.

**Architecture:** Keep the visual cleaver mesh separate from a thin, calibrated collision blade whose edge site defines task height. Add a viewer presentation configuration used only by interactive chopping runs, while headless execution remains unpaced.

**Tech Stack:** CPython 3.12, MuJoCo 3.10, NumPy, pytest, MJCF

## Global Constraints

- Do not modify `SDK_PYTHON/`, `test/`, or `real_robot_debug/`.
- Keep native MuJoCo position actuators and the existing IK trajectory architecture.
- Do not add impedance, force, or admittance control.
- Default target penetration remains 3 mm; measured collision penetration must not exceed 5 mm.

---

### Task 1: Calibrate the Cleaver Collision Edge

**Files:**
- Modify: `robot_assets/mujoco/right_chopping_scene.xml`
- Modify: `tests/simulation/test_chop.py`

**Interfaces:**
- Consumes: `right_knife_blade`, `right_blade_edge_bot`, `ChopConfig`, `_preflight`.
- Produces: a collision edge whose world height is consistent with the task reference site.

- [ ] **Step 1: Write the failing geometry regression test**

Add a test that executes the preflight descent target, transforms all eight
`right_knife_blade` box corners to world coordinates, and asserts:

```python
assert board_top_m - blade_min_z_m <= 0.005
```

- [ ] **Step 2: Verify the regression test fails**

Run:

```bash
.venv/bin/python -m pytest tests/simulation/test_chop.py::test_contact_target_limits_blade_penetration -v
```

Expected: FAIL reporting approximately `0.057 m`.

- [ ] **Step 3: Align collision geometry and edge site**

In `right_chopping_scene.xml`, keep the visual mesh unchanged, replace the
oversized collision box with a thin blade-aligned box, and place
`right_blade_edge_bot` on its lowest cutting edge. Do not change robot joints,
actuators, or the chopping board.

- [ ] **Step 4: Verify focused and full tests**

Run:

```bash
.venv/bin/python -m pytest tests/simulation/test_chop.py -q
.venv/bin/python -m pytest tests/simulation -q
```

Expected: both commands PASS.

### Task 2: Add an Observable Slow Viewer Presentation

**Files:**
- Modify: `src/twin_sim/tasks/chop.py`
- Modify: `src/twin_sim/cli.py`
- Modify: `tests/simulation/test_chop.py`
- Modify: `tests/simulation/test_cli.py`

**Interfaces:**
- Produces: `chop --slow`, with a side-front camera and viewer-only start/end holds.

- [ ] **Step 1: Write failing CLI and timing tests**

Assert that `--slow` maps to durations `3.0, 3.0, 1.0, 3.0`, and that viewer
presentation waits occur only when a viewer is active.

- [ ] **Step 2: Verify tests fail for missing `--slow`**

Run:

```bash
.venv/bin/python -m pytest tests/simulation/test_cli.py tests/simulation/test_chop.py -q
```

Expected: FAIL because `--slow` and viewer presentation settings do not exist.

- [ ] **Step 3: Implement the minimal presentation configuration**

Add `--slow`, fixed side-front camera values, a 5-second pre-roll, and an
8-second post-roll. Keep all sleeps out of headless runs.

- [ ] **Step 4: Verify and replay**

Run:

```bash
.venv/bin/python -m pytest tests/simulation -q
sha256sum --check docs/simulation/protected-files.sha256
.venv/bin/twin-sim chop --slow --log /tmp/twin-sim-chop-fixed.csv
```

Expected: all tests and hashes PASS; Viewer shows visible approach, contact,
and retract without the blade being buried in the board.

- [ ] **Step 5: Commit**

```bash
git add robot_assets/mujoco/right_chopping_scene.xml src/twin_sim tests/simulation docs/superpowers
git commit -m "fix: calibrate chopping collision and viewer presentation"
```

