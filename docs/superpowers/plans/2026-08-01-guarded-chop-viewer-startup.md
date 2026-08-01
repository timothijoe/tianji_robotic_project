# Guarded Chop Viewer Startup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Open the guarded-chop Viewer only after the final scene and robot state are ready, eliminating the startup flash of legacy pick-place objects and home pose.

**Architecture:** Extract passive Viewer launch from `RightArmRobot.__init__` into an idempotent public method while preserving `viewer=True` compatibility. Guarded chop constructs headless, completes preflight/scene/reset/forward, then opens and configures the Viewer and trace before its existing ready hold.

**Tech Stack:** Python 3.12, MuJoCo 3.10 passive Viewer, pytest, monkeypatch test doubles.

## Global Constraints

- Change only guarded-chop startup behavior; preserve other `RightArmRobot(viewer=True)` callers.
- Do not change headless behavior, motion trajectories, safety limits, ROS 2, or hardware interfaces.
- Preflight failure must occur before any Viewer window opens.
- Repeated `open_viewer()` calls must not launch duplicate windows.
- The first guarded-chop Viewer frame must contain the configured scene and final ready pose.

---

### Task 1: Add idempotent Viewer opening to the robot

**Files:**
- Modify: `src/twin_sim/robot.py`
- Modify: `tests/simulation/test_robot.py`

**Interfaces:**
- Produces `RightArmRobot.open_viewer() -> None`.
- Preserves `RightArmRobot(viewer=True)` behavior by invoking `open_viewer()` after reset.

- [ ] **Step 1: Write failing idempotency and compatibility tests**

Monkeypatch `mujoco.viewer.launch_passive`, construct with `viewer=False`, call `open_viewer()` twice, and assert one launch with the robot model/data. A second test constructs with `viewer=True` and asserts one launch.

- [ ] **Step 2: Run tests and verify failure**

Run: `.venv/bin/python -m pytest tests/simulation/test_robot.py -q`

Expected: failure because `open_viewer` does not exist.

- [ ] **Step 3: Extract the existing launch logic**

Move the passive launch and matching-thread detection block into `open_viewer`. Return immediately when `_viewer is not None`. In `__init__`, keep reset first and replace the inline block with:

```python
if viewer:
    self.open_viewer()
```

- [ ] **Step 4: Run robot tests**

Run: `.venv/bin/python -m pytest tests/simulation/test_robot.py -q`

Expected: all tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/twin_sim/robot.py tests/simulation/test_robot.py
git commit -m "refactor: support deferred viewer opening"
```

---

### Task 2: Defer guarded-chop Viewer until scene readiness

**Files:**
- Modify: `src/twin_sim/tasks/guarded_chop.py`
- Modify: `tests/simulation/test_guarded_chop_startup.py`
- Modify: `docs/simulation/usage.md`
- Modify: `docs/simulation/guarded_chopping_development_log.md`
- Modify: `docs/superpowers/specs/2026-08-01-guarded-chop-viewer-startup-design.md`

**Interfaces:**
- `run_guarded_chop(..., viewer=True)` constructs `RightArmRobot(viewer=False)` and calls `open_viewer()` only after `_configure_guarded_scene`, ready-state assignment, commands, and `mujoco.mj_forward`.

- [ ] **Step 1: Write a failing startup-order test**

Use a fake robot/trace boundary or monkeypatch event hooks to record `preflight`, `configure`, `reset`, final `mj_forward`, `open_viewer`, camera preparation, and trace creation. Assert:

```python
assert events.index("preflight") < events.index("configure")
assert events.index("configure") < events.index("final_forward")
assert events.index("final_forward") < events.index("open_viewer")
assert events.index("open_viewer") < events.index("prepare_camera")
assert events.index("prepare_camera") < events.index("create_trace")
```

Also assert the robot constructor received `viewer=False`, even when `run_guarded_chop(..., viewer=True)` is requested. Add a headless case asserting `open_viewer` is absent.

- [ ] **Step 2: Run startup tests and verify failure**

Run: `.venv/bin/python -m pytest tests/simulation/test_guarded_chop_startup.py -q`

Expected: failure because the Viewer currently opens in the constructor before preflight.

- [ ] **Step 3: Reorder guarded-chop startup**

Construct the robot with `viewer=False`; defer `_prepare_guarded_chop_viewer` and default `GuardedChopTrace` creation. After scene configuration, ready-state assignment, commands and final `mj_forward`, conditionally call `robot.open_viewer()`, prepare the camera, create the default trace, and set the plan. Preserve an externally supplied trace without replacing it. The existing `GUARD_READY` duration provides the visible initial hold.

- [ ] **Step 4: Run focused behavior tests**

Run:

```bash
.venv/bin/python -m pytest tests/simulation/test_guarded_chop_startup.py tests/simulation/test_guarded_chop_visualization.py tests/simulation/test_guarded_chop_integration.py -q
.venv/bin/twin-sim guarded-chop --scene plane --headless --final-hold 0
bash -n scripts/run_guarded_chop.sh
git diff --check
```

Expected: focused tests pass; headless reports `success=True cuts=5 shifts=4 total_shift_m=0.080`; shell and diff checks exit 0.

- [ ] **Step 5: Update documentation**

Document that Viewer opens only after the scene and ready pose are prepared, so no legacy pick-place objects or home pose appear. Record the root cause, fix, test result and Viewer acceptance in the development log and design progress section.

- [ ] **Step 6: Run full verification**

Run: `.venv/bin/python -m pytest tests/simulation -q`

Expected: full simulation suite passes with only the existing contact-force warning.

- [ ] **Step 7: Commit**

```bash
git add src/twin_sim/tasks/guarded_chop.py tests/simulation/test_guarded_chop_startup.py docs/simulation/usage.md docs/simulation/guarded_chopping_development_log.md docs/superpowers/specs/2026-08-01-guarded-chop-viewer-startup-design.md
git commit -m "fix: open guarded chop viewer after setup"
```

- [ ] **Step 8: Launch Viewer twice for visual confirmation**

Run: `./scripts/run_guarded_chop.sh` twice.

Expected: both windows open directly on the correct cutting scene and ready pose, hold visibly, then run five cuts without startup flash or abort.
