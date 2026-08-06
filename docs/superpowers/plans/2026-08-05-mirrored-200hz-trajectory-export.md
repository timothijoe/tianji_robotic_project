# Mirrored 200 Hz Trajectory Export Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Generate an offline NPZ/CSV whose arm and right-hand values are the verified numeric mirror of the existing 200 Hz export.

**Architecture:** Add a focused pure-Python converter that loads the existing NPZ, exchanges/sign-mirrors arms, changes only thumb joint 2 for the right hand, verifies the official right-hand MJCF ranges, then writes a new NPZ/CSV.

**Tech Stack:** Python 3.12, NumPy, pytest, existing trajectory codec.

## Global Constraints

- Do not access ROS 2, SDKs, or hardware.
- Source files remain unchanged; output is offline reference data only.
- Arm sign vector is `(-1,+1,-1,+1,-1,+1,-1)` after arm exchange.
- Hand order remains five fingers by four joints; only thumb joint 2 is negated.

### Task 1: Test and implement the numeric conversion

**Files:** Create `src/twin_sim/mirrored_trajectory.py`; create `tests/simulation/test_mirrored_trajectory.py`.

**Interfaces:** `mirror_recorded_chop_trajectory(source) -> MirroredTrajectory`; `write_mirrored_trajectory(base_path, source_path) -> (Path, Path)`.

- [ ] **Step 1: Write a failing mapping test.** Use two synthetic samples and assert `left_arm == source.right_arm * [-1,1,-1,1,-1,1,-1]`, `right_arm == source.left_arm * [-1,1,-1,1,-1,1,-1]`, `right_hand[:,1] == -source.left_hand[:,1]`, and all other 19 hand columns are equal.
- [ ] **Step 2: Run RED.** Run `.venv-wuji-teleop/bin/python -m pytest -q tests/simulation/test_mirrored_trajectory.py`; expect module import failure.
- [ ] **Step 3: Implement pure conversion.** Reuse `load_recorded_chop_trajectory`, preserve timestamps/metrics, perform the stated vector operations, validate finite arrays and source 5 ms spacing, and name CSV hand columns `right_finger*_joint*_rad`.
- [ ] **Step 4: Add right-MJCF range test.** Parse the official right MJCF actuator control ranges; assert all converted hand samples fall within each corresponding range.
- [ ] **Step 5: Run GREEN and commit.** Run the test module; commit the module/tests as `feat: mirror 200hz arm and hand trajectory`.

### Task 2: Generate and verify the requested local files

**Files:** Create local `recordings/recorded_hand_guarded_chop_200hz_mirrored_right_hand.npz` and `.csv`; modify `docs/simulation/recorded_hand_guarded_chop.md`.

- [ ] **Step 1: Write a failing integration test.** Convert a temporary valid source, reload the output, and assert exact sample count/time vector, arm signs, right-thumb sign, and unchanged remaining hand values.
- [ ] **Step 2: Run RED.** Run the new integration test and expect the absent writer/loader contract failure.
- [ ] **Step 3: Add the one-command local converter and documentation.** Its input defaults to the existing 200 Hz NPZ and output uses the requested mirrored basename; document that it is not directly executable on hardware.
- [ ] **Step 4: Run end-to-end verification.** Convert the real source, reload it, print sample count, 5 ms period, max arm identity error, max unaffected-hand error, and right-hand range margin; require zero identity error and nonnegative range margin.
- [ ] **Step 5: Commit source/documentation only.** Commit code/tests/docs, not generated recordings.
