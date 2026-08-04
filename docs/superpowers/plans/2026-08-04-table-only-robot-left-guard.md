# Table-Only Robot-Left Guard Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the recorded-hand task cut the chopping-board surface directly while keeping the Wuji palm reference 160 mm to robot-left (`+Y`) of each active cut.

**Architecture:** Configure only the new task's in-memory MuJoCo model to hide and disable unrelated props. Build its right-arm cuts from the board-contact line-chop plan instead of the cube-raised guarded plan, while preserving the existing recorded-hand IK, five-cycle executor, and safety measurements.

**Tech Stack:** Python 3.12, NumPy, MuJoCo, pytest.

## Global Constraints

- Robot-left is world `+Y`; right-to-left knife progression remains `+Y`.
- Initial hand anchor is active cut `Y + 0.160 m` and `X - 0.080 m`; recorded retreat continues approximately 30 mm along `+Y`.
- Knife contact is the raised chopping-board top, not a cube top.
- Hide/disable props only in the new task's model instance.
- Preserve knife/hand distance `>= 0.020 m`, hand penetration `<= 0.0005 m`, and thumb clearance `>= 0.010 m`.
- Do not change existing `guarded-chop`, `line-chop`, or `pick-place` behavior.

---

### Task 1: New-task table-only scene

**Files:**
- Modify: `src/twin_sim/tasks/recorded_hand_guarded_chop.py`
- Modify: `tests/simulation/test_recorded_hand_guarded_chop_preflight.py`

**Interfaces:**
- Produces: `_configure_table_only_scene(robot: RightArmRobot) -> None`.

- [x] **Step 1: Write a failing scene-isolation test**

Create two robots. Configure one and assert `guarded_chop_cube`,
`pick_source_pedestal`, `pick_target_pedestal`, `pick_cube_geom`, and
`pick_target_region` have alpha, `contype`, and `conaffinity` equal to zero;
assert the untouched robot retains its original values and the chopping board
remains visible/collidable.

- [x] **Step 2: Run RED**

Run:

```bash
.venv-wuji-teleop/bin/pytest -q tests/simulation/test_recorded_hand_guarded_chop_preflight.py
```

Expected: import failure for `_configure_table_only_scene`.

- [x] **Step 3: Implement explicit named-geom configuration**

Resolve exactly the five named geoms, set `model.geom_rgba[id, 3] = 0`,
`geom_contype[id] = 0`, and `geom_conaffinity[id] = 0`, then call
`mujoco.mj_forward`. Do not modify MJCF files or any unnamed geom.

- [x] **Step 4: Run GREEN and commit with Task 2**

Run the focused test and proceed only when it passes.

---

### Task 2: Board-contact cuts and robot-left hand anchors

**Files:**
- Modify: `src/twin_sim/tasks/recorded_hand_guarded_chop.py`
- Modify: `tests/simulation/test_recorded_hand_guarded_chop_preflight.py`
- Modify: `docs/simulation/recorded_hand_guarded_chop.md`
- Modify: `docs/wuji/development_status.md`

**Interfaces:**
- Produces: `_preflight_table_cuts(robot, config)` returning ordered board-contact cuts, cut points, lift shifts, ready joints, and safe height.
- Updates: `_lateral_guard_anchor_xy(cut_xy, clearance_m)` to return `(cut_x, cut_y + clearance_m)`.

- [x] **Step 1: Write failing board-contact and robot-left tests**

Assert the hand anchor for cut `(0.62, -0.08)` and `0.16 m` clearance is
`(0.54, 0.08)`. On a real five-cycle plan assert every initial palm `Y` is at
least `0.159 m` beyond its matching knife contact `Y`, every retreat ends at a
larger `Y`, and every knife blade bottom at contact equals the raised board top
within the existing chopping penetration tolerance. Assert five cuts and all
existing safety limits.

- [x] **Step 2: Run RED**

Expected: old anchor is in `-X` and knife contact remains near cube-top height.

- [x] **Step 3: Implement table-contact right planning**

Call `_preflight_line_chop` with the existing five-cut timing and spacing,
order its cuts with `_right_to_left_indices`, and pass them through
`_translate_right_cuts(..., dz_m=0.0, ...)` so only safe tracking headroom is
added while contact stays on the board. Build diagonal lift shifts with the
existing helper. Configure the table-only scene before safety measurement and
use `cut_xy + (-0.080, 0.160)` for each hand anchor.

- [x] **Step 4: Run focused and preserved-task regression**

```bash
.venv-wuji-teleop/bin/pytest -q \
  tests/simulation/test_recorded_hand_guarded_chop_preflight.py \
  tests/simulation/test_recorded_hand_guarded_chop.py \
  tests/simulation/test_guarded_chop_preflight.py \
  tests/simulation/test_guarded_chop_state_machine.py \
  tests/simulation/test_pick_place_scene.py \
  tests/simulation/test_pick_place_state_machine.py \
  tests/simulation/test_pick_place_integration.py \
  tests/simulation/test_pick_place_visualization.py
```

- [x] **Step 5: Update docs and verify Headless/Viewer**

Record selected surface height and safety metrics. Viewer acceptance must show
no cube/pedestal props, the blade reaching the board, and the hand consistently
on robot-left of the blade.

- [x] **Step 6: Commit**

```bash
git add src/twin_sim/tasks/recorded_hand_guarded_chop.py \
  tests/simulation/test_recorded_hand_guarded_chop_preflight.py \
  docs/simulation/recorded_hand_guarded_chop.md docs/wuji/development_status.md
git commit -m "fix: cut table with robot-left recorded guard"
```
