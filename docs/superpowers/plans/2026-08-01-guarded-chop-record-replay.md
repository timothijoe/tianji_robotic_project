# Guarded Chop Record Replay Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Preserve the existing 1× guarded-chop demo and add a second launcher that watches the same 1× execution, then replays its captured MuJoCo states at 2× with optional atomic NPZ persistence.

**Architecture:** A focused `guarded_chop_recording` module owns immutable frames, versioned NPZ I/O and state-only replay. The guarded-chop task captures frames only when requested and replays only after successful safety validation; CLI flags and a new shell launcher expose the workflow without changing the existing launcher.

**Tech Stack:** Python 3.12, NumPy NPZ, MuJoCo 3.10, pytest, Bash

## Global Constraints

- The live pass remains 1× with `control_dt_s=0.01`; both arms and Wuji Hand remain joint-position controlled.
- Replay restores recorded state and calls `mj_forward`; it must not call `mj_step`, controllers, or safety evaluation.
- Default replay rate is 2.0 in the new launcher; replay rates must be finite and positive.
- Recording is memory-only unless `--record` is supplied. Bare `--record` resolves to `recordings/guarded_chop_latest.npz`.
- Saving the same path atomically replaces the previous file; a different explicit path creates a separate recording.
- Existing `scripts/run_guarded_chop.sh`, headless behavior, ROS 2 and hardware code remain unchanged.

---

### Task 1: Versioned immutable recording and atomic persistence

**Files:**
- Create: `src/twin_sim/guarded_chop_recording.py`
- Create: `tests/simulation/test_guarded_chop_recording.py`
- Modify: `.gitignore`

**Interfaces:**
- Produces: `GuardedChopFrame`, `GuardedChopRecording`, `capture_frame(robot, sample)`, `save_recording(recording, path)`, and `load_recording(path, model)`.
- A frame stores scalar time/phase/cut metadata plus copied `qpos`, `qvel`, `ctrl`, knife position, guard position, minimum distance and cut permission.

- [ ] **Step 1: Write failing copy and round-trip tests**

Create tests that capture a frame, mutate source arrays, and assert the frame remains unchanged; save two recordings to the same `tmp_path / "latest.npz"` and assert one target file with second contents; load it against matching and mismatched `(nq, nv, nu)` models.

- [ ] **Step 2: Run recording tests and verify RED**

Run `.venv/bin/python -m pytest tests/simulation/test_guarded_chop_recording.py -v`.
Expected: collection fails because `twin_sim.guarded_chop_recording` does not exist.

- [ ] **Step 3: Implement minimal immutable frame, NPZ serializer and loader**

Use frozen dataclasses, copied arrays with `writeable=False`, schema version `1`, string phase arrays, `tempfile.NamedTemporaryFile(dir=target.parent, delete=False)`, `np.savez_compressed(file_handle, ...)`, and `os.replace`. Validate non-empty recordings, equal column widths, finite time, increasing/nondecreasing timestamps and exact model dimensions.

- [ ] **Step 4: Verify GREEN and ignore recordings**

Run the recording test file. Add `recordings/` to `.gitignore`; rerun and expect all tests PASS.

- [ ] **Step 5: Commit**

Commit as `feat: add guarded chop state recording`.

### Task 2: State-only Viewer replay and trace reset

**Files:**
- Modify: `src/twin_sim/guarded_chop_recording.py`
- Modify: `src/twin_sim/guarded_chop_visualization.py`
- Modify: `tests/simulation/test_guarded_chop_recording.py`
- Modify: `tests/simulation/test_guarded_chop_visualization.py`

**Interfaces:**
- Produces: `replay_recording(robot, recording, *, rate, trace=None, sleep=time.sleep)` and `GuardedChopTrace.begin_replay(rate)`.
- Consumes: the frame fields defined in Task 1 and an already-open `RightArmRobot`.

- [ ] **Step 1: Write failing replay and trace tests**

Use a fake robot/data object and monkeypatched `mujoco.mj_forward` to assert every qpos/qvel/ctrl state is restored, forward is called once per frame, `robot.step` is never used, sync follows frames, and sleeps equal timestamp deltas divided by 2. Assert `begin_replay(2.0)` clears actual trail state while retaining five planned points and adds `replay 2.0x` to the overlay.

- [ ] **Step 2: Run focused tests and verify RED**

Run both recording and visualization test files. Expected: failures for the missing replay and trace APIs.

- [ ] **Step 3: Implement state-only replay and trace reset**

Validate `rate`; call `trace.begin_replay(rate)` once; restore arrays under exact shape checks; call `mujoco.mj_forward`; append recorded marker metadata; sync the Viewer at its existing refresh cadence; sleep only between frames using `max(0.0, delta_time / rate)`.

- [ ] **Step 4: Run focused tests and verify GREEN**

Run both test files and expect all tests PASS.

- [ ] **Step 5: Commit**

Commit as `feat: replay guarded chop state recordings`.

### Task 3: Capture and replay within guarded-chop lifecycle

**Files:**
- Modify: `src/twin_sim/tasks/guarded_chop.py`
- Modify: `tests/simulation/test_guarded_chop_integration.py`
- Modify: `tests/simulation/test_guarded_chop_startup.py`

**Interfaces:**
- Extend `run_guarded_chop(..., replay_rate: float | None = None, record_path: Path | None = None)` without changing existing callers.
- Capture only when either option is supplied; save after successful result validation and before replay; replay before `robot.close()`.

- [ ] **Step 1: Write failing lifecycle tests**

Monkeypatch capture/save/replay seams to assert the default run does none of them; replay mode captures each sample and calls replay only after 5 cuts/4 shifts; save mode receives the requested path; aborted runs may save partial frames but never replay.

- [ ] **Step 2: Run lifecycle tests and verify RED**

Run guarded-chop integration/startup focused tests. Expected: `run_guarded_chop` rejects the new keyword arguments.

- [ ] **Step 3: Implement conditional lifecycle integration**

Create the recording collector only when needed, append immediately after observation, build a success recording after final validation, save if requested, and call state-only replay if requested and Viewer remains open. Reject replay without Viewer and validate the rate before running preflight.

- [ ] **Step 4: Run guarded-chop regression tests and verify GREEN**

Run all `tests/simulation/test_guarded_chop_*.py`; expect all tests PASS.

- [ ] **Step 5: Commit**

Commit as `feat: integrate guarded chop record replay`.

### Task 4: CLI and launcher workflow

**Files:**
- Modify: `src/twin_sim/cli.py`
- Modify: `tests/simulation/test_cli.py`
- Create: `scripts/run_guarded_chop_record_replay.sh`
- Modify: `tests/simulation/test_guarded_chop_launcher.py`

**Interfaces:**
- `guarded-chop --replay-rate RATE --record [PATH]` maps to `run_guarded_chop(replay_rate=..., record_path=...)`.
- Bare `--record` maps to `Path("recordings/guarded_chop_latest.npz")`; omitted flag maps to `None`.
- New launcher passes `guarded-chop --scene plane --replay-rate 2.0` plus all user arguments.

- [ ] **Step 1: Write failing CLI and launcher tests**

Test omitted/bare/explicit `--record`, `--replay-rate 2.0`, rejection of headless replay, forwarding into the task and exact launcher arguments. Test that `--record custom.npz` is forwarded by the shell script.

- [ ] **Step 2: Run CLI/launcher tests and verify RED**

Run both test files. Expected: parser rejects the new flags and the new script is absent.

- [ ] **Step 3: Implement parser mapping and shell launcher**

Use `nargs="?"`, `const=Path("recordings/guarded_chop_latest.npz")`, `default=None`. Reject `args.headless and args.replay_rate is not None` with `parser.error`. Make the new executable launcher resolve the repo and venv exactly like the existing script, then append `"$@"` after fixed arguments.

- [ ] **Step 4: Run focused tests and verify GREEN**

Run CLI and launcher tests; expect all tests PASS. Run `bash -n scripts/run_guarded_chop_record_replay.sh` and expect exit 0.

- [ ] **Step 5: Commit**

Commit as `feat: add guarded chop record replay launcher`.

### Task 5: Regression, documentation and visual acceptance

**Files:**
- Modify: `docs/superpowers/specs/2026-08-01-guarded-chop-record-replay-design.md`

**Interfaces:**
- Existing launcher: `./scripts/run_guarded_chop.sh`.
- New launcher: `./scripts/run_guarded_chop_record_replay.sh [--record [PATH]]`.

- [ ] **Step 1: Run full automated verification**

Run `.venv/bin/python -m pytest`; expect no failures and no new warnings beyond the existing 39.611 N contact-force observation warning.

- [ ] **Step 2: Verify default overwrite behavior**

Run the record/replay workflow twice with bare `--record`; verify exactly one latest NPZ remains and loads successfully. Run once with an explicit second filename and verify both files exist.

- [ ] **Step 3: Run both Viewer launchers**

Visually verify the old launcher remains 1× only. Verify the new launcher shows one 1× live pass followed by a clearly labelled 2× state replay in the same window.

- [ ] **Step 4: Record exact evidence**

Append test counts, output metrics, overwrite checks and pending/completed human visual verdict to the design document.

- [ ] **Step 5: Commit**

Commit as `docs: record guarded chop replay verification`.
