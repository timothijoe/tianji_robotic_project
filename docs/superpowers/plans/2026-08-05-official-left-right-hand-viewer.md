# Official Left/Right Hand Viewer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Sequentially replay source-left and mirrored-right 200 Hz hand trajectories in their official offline MuJoCo models.

**Architecture:** Add a small viewer adapter that validates the two NPZ inputs, exposes named 20-joint sequences for the existing official hand model loader, and launches the left Viewer followed by the right Viewer. No robot, ROS, SDK, or hardware import is allowed.

**Tech Stack:** Python 3.12, NumPy, MuJoCo, pytest, official Wuji MJCF files.

## Global Constraints

- Read only the source/mirrored local NPZ files and official MJCF assets.
- Validate equal 5 ms clocks, finite `(N,20)` values, and official per-joint ranges before Viewer launch.
- Play once at 200 Hz and hold each final frame for 60 seconds.
- Do not publish, connect, or command any physical interface.

### Task 1: Offline adapter and range validation

**Files:** Create `src/twin_sim/official_hand_viewer.py`; create `tests/simulation/test_official_hand_viewer.py`.

- [ ] **Step 1: Write failing test.** Create synthetic `(2,20)` left/right arrays with a shared 5 ms clock; assert `load_comparison` preserves both arrays and rejects unequal timestamps or a right value beyond the official MJCF range.
- [ ] **Step 2: Run RED.** Run `.venv-wuji-teleop/bin/python -m pytest -q tests/simulation/test_official_hand_viewer.py`; expect import failure.
- [ ] **Step 3: Implement loader.** Read current NPZ codecs, parse official left/right `<position ctrlrange>` in actuator order, and reject invalid timing, shape, finite values, and range violations.
- [ ] **Step 4: Run GREEN and commit.** Run the test module; commit code/tests as `feat: validate official hand viewer inputs`.

### Task 2: Sequential Viewer launcher

**Files:** Modify `src/twin_sim/official_hand_viewer.py`; create `scripts/view_official_left_right_hands.sh`; modify `docs/simulation/recorded_hand_guarded_chop.md`.

- [ ] **Step 1: Write failing order test.** Inject a fake `play_hand_model`; assert `play_comparison` calls it first with `left.xml` and the source array, then with `right.xml` and the mirror array, both at `.005` seconds and `60` seconds final hold.
- [ ] **Step 2: Run RED.** Run the named order test; expect missing `play_comparison`.
- [ ] **Step 3: Implement launcher.** Use a passive MuJoCo Viewer, set qpos from official actuator order, play each frame once, then hold final frame; begin the right Viewer only after the left Viewer closes.
- [ ] **Step 4: Run headless loader verification and launch.** Verify inputs with no Viewer, then run the shell launcher; inspect the left window, close it, inspect the right window, close it.
- [ ] **Step 5: Commit code/docs only.** Do not commit generated NPZ/CSV files.
