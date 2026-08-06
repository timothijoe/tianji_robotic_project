# Recorded Chop 200 Hz Trajectory Export Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Export the validated offline recorded-hand chop as 200 Hz NPZ and CSV targets for both seven-axis arms and the twenty-axis left hand.

**Architecture:** Flatten the existing 10 ms synchronized plan, linearly interpolate its targets to 5 ms, revalidate every interpolated sample in MuJoCo, then atomically write a versioned NPZ and a named CSV. The existing controller remains at 10 ms and physics at 2 ms.

**Tech Stack:** Python 3.12, NumPy, MuJoCo, pytest, standard-library csv.

## Global Constraints

- Offline MuJoCo only; do not import, start, connect to, or command ROS 2 or hardware.
- Preserve current task behavior and the 10 ms control period.
- Export `time_s`, `(N,7)` right-arm targets, `(N,7)` left-arm targets, and `(N,20)` left-hand targets at exactly 5 ms intervals.
- Reject nonfinite or out-of-range targets, arm steps greater than `.12 rad`, nonzero planned penetration, or clearance below the existing selected `.020/.010 m` tier.
- The files are reference data, not a real-machine program or certification.

### Task 1: Pure trajectory data codec

**Files:** Create `src/twin_sim/recorded_chop_trajectory.py`; create `tests/simulation/test_recorded_chop_trajectory.py`.

**Interfaces:** `RecordedChopTrajectory`; `resample_recorded_chop_trajectory(time_s, right, left, hand)`; `write_recorded_chop_trajectory(base_path, trajectory)`; `load_recorded_chop_trajectory(npz_path)`.

- [ ] **Step 1: Write failing resampling test.** Assert two 10 ms samples `[0,.01]` yield times `[0,.005,.01]`, and values `0,.2` yield midpoint `.1`, for each target array.
- [ ] **Step 2: Verify RED.** Run `.venv-wuji-teleop/bin/pytest -q tests/simulation/test_recorded_chop_trajectory.py`; expect missing module import.
- [ ] **Step 3: Implement the smallest codec.** Require strict 10 ms source spacing, equal lengths and `(N,7)/(N,7)/(N,20)` finite arrays. Resample each column with `np.interp` onto 5 ms timestamps including both endpoints. Define the immutable dataclass with source period and safety metrics.
- [ ] **Step 4: Write failing round-trip test.** Save a three-sample trajectory, reload NPZ, assert all arrays equal; read CSV using `csv.DictReader`, assert `time_s`, `right_joint1_rad`, `left_joint7_rad`, and `left_finger5_joint4_rad` headers and the `0.005000000` row timestamp.
- [ ] **Step 5: Implement writers/load validation.** NPZ fields are `format_version=1`, all four arrays, `source_control_dt_s`, `selected_clearance_tier_m`, `minimum_distance_m`, and `maximum_penetration_m`; forbid object data. CSV has the identical 35 numeric columns formatted to nine decimals. Loader rejects missing fields, wrong version, invalid shapes, unequal lengths, non-5 ms spacing, nonfinite values, and nonpositive safety metrics.
- [ ] **Step 6: Verify GREEN and commit.** Run `.venv-wuji-teleop/bin/pytest -q tests/simulation/test_recorded_chop_trajectory.py`, then commit `src/twin_sim/recorded_chop_trajectory.py` and its test as `feat: encode 200hz recorded chop trajectories`.

### Task 2: Flatten and safety-validate the real plan

**Files:** Modify `src/twin_sim/tasks/recorded_hand_guarded_chop.py`; modify `tests/simulation/test_recorded_hand_guarded_chop.py`.

**Interfaces:** `_export_trajectory_from_plan(robot, plan, config) -> RecordedChopTrajectory`; existing runner accepts `record_path: Path | None`.

- [ ] **Step 1: Write failing export test.** Call `run_recorded_hand_guarded_chop` with local MCAP, `surface_offsets_m=(.04,)`, `final_hold_s=0`, `viewer=False`, and `record_path=tmp_path/'motion'`. Load `motion.npz`; assert 5 ms diffs and shapes `(N,7)`, `(N,7)`, `(N,20)`.
- [ ] **Step 2: Verify RED.** Run the named test; expect the existing `NotImplementedError` for `record_path`.
- [ ] **Step 3: Implement flattening.** Concatenate every `SynchronizedGuardCycle.right`, `.left`, and `.hand` in order. Source time is command index times `config.control_dt_s`; pass arrays to Task 1's resampler.
- [ ] **Step 4: Implement export validation.** Before writing, verify control ranges and adjacent left/right arm deltas. For every 5 ms sample, set the three qpos slices in a temporary MuJoCo data object, run `mj_forward`, measure `_blade_hand_distance`, require `>= plan.selected_clearance_tier_m`, and reuse the table/pad penetration rule; restore live data afterwards. Only write after all samples pass.
- [ ] **Step 5: Add failed-preflight atomicity test.** Force `_preflight_recorded_hand_guarded_chop` to raise; assert runner returns unsuccessful and neither `motion.npz` nor `motion.csv` exists.
- [ ] **Step 6: Verify GREEN and commit.** Run `.venv-wuji-teleop/bin/pytest -q tests/simulation/test_recorded_chop_trajectory.py tests/simulation/test_recorded_hand_guarded_chop.py`; commit changed task/test as `feat: export validated recorded chop targets`.

### Task 3: CLI and user documentation

**Files:** Modify `src/twin_sim/cli.py`; modify `tests/simulation/test_cli.py`; modify `docs/simulation/recorded_hand_guarded_chop.md`.

**Interfaces:** Add `--export-trajectory PATH` to `twin-sim recorded-hand-guarded-chop` and pass it as the runner's `record_path`.

- [ ] **Step 1: Write failing parser/dispatch test.** Monkeypatch `run_recorded_hand_guarded_chop`, invoke `recorded-hand-guarded-chop --headless --final-hold 0 --export-trajectory recordings/demo`, and assert captured `record_path == Path('recordings/demo')` and `viewer is False`.
- [ ] **Step 2: Verify RED.** Run the named CLI test; expect unrecognized `--export-trajectory`.
- [ ] **Step 3: Implement the option.** Add `type=Path` argument, forward it to the runner, and print NPZ/CSV output paths only after successful export.
- [ ] **Step 4: Document exact use.** Add the headless command `env -u PYTHONPATH .venv-wuji-teleop/bin/twin-sim recorded-hand-guarded-chop --headless --final-hold 0 --export-trajectory recordings/recorded_hand_guarded_chop_200hz_latest`; document 200 Hz, time units seconds, joint units radians, all 34 targets, and the strict offline/no-hardware boundary.
- [ ] **Step 5: Verify end-to-end and commit.** Run focused codec/task/CLI tests plus the documented command. Expect exit zero, five cuts/cycles, and both `.npz`/`.csv`; commit as `docs: expose 200hz recorded chop export`.
