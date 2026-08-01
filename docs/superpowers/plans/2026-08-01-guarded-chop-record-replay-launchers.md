# Guarded Chop Record and Replay Launchers Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add one shell command for normal-speed guarded-chop recording and one shell command for replaying an existing recording at 2× without rerunning simulation.

**Architecture:** Add a focused `guarded_chop_playback` module that assembles the existing robot, plane scene, trace, NPZ loader, and pure state replay APIs. Expose it through a `guarded-chop-replay` CLI command, then keep both shell launchers as thin repository-root-resolving wrappers.

**Tech Stack:** Bash, Python 3.12, argparse, NumPy, MuJoCo, pytest

## Global Constraints

- Default recording path is exactly `recordings/guarded_chop_latest.npz`.
- Recording runs the online MuJoCo task at 1× and atomically overwrites the selected path.
- Replay reads an existing NPZ at 2× and never calls `mj_step`, the task controller, or the safety coordinator.
- Existing `run_guarded_chop.sh` and `run_guarded_chop_record_replay.sh` behavior remains unchanged.
- Scripts must resolve the repository root from their own path and run from any current directory.
- Do not modify physical robot, ROS 2, Wuji Hand SDK, or object-scene behavior.

---

### Task 1: Add a reusable high-level recording playback entry

**Files:**
- Create: `src/twin_sim/guarded_chop_playback.py`
- Create: `tests/simulation/test_guarded_chop_playback.py`

**Interfaces:**
- Consumes: `load_recording(path, model)`, `replay_recording(robot, recording, rate, trace)`, `RightArmRobot`, `GuardedChopTrace`, and the existing plane-scene/preflight visualization helpers.
- Produces: `play_guarded_chop_recording(path: Path, *, rate: float = 2.0) -> None`.

- [x] **Step 1: Write the failing orchestration test**

Create `tests/simulation/test_guarded_chop_playback.py` with monkeypatched fake robot, preflight plan, loader, and replay function. Assert that `play_guarded_chop_recording(Path("demo.npz"), rate=2.0)`:

```python
assert calls["scene"] == "plane"
assert calls["load_path"] == Path("demo.npz")
assert calls["rate"] == 2.0
assert calls["opened_viewer"] is True
assert calls["first_state_restored"] is True
assert calls["closed"] is True
```

Also assert the trace receives a `(5, 3)` planned marker array and that no symbol named `run_guarded_chop` is imported or called by the playback module.

- [x] **Step 2: Run the new test and verify RED**

Run:

```bash
.venv/bin/python -m pytest tests/simulation/test_guarded_chop_playback.py -v
```

Expected: FAIL because `twin_sim.guarded_chop_playback` does not exist.

- [x] **Step 3: Implement the high-level playback function**

Create `src/twin_sim/guarded_chop_playback.py` with this structure:

```python
def play_guarded_chop_recording(path: Path, *, rate: float = 2.0) -> None:
    playback_rate = validate_replay_rate(rate)
    robot = RightArmRobot(viewer=False)
    try:
        config = GuardedChopConfig(final_hold_s=0.0)
        plan = _preflight_guarded_chop(robot, config)
        _configure_guarded_scene(robot, "plane")
        recording = load_recording(Path(path), robot.sim.model)
        first = recording.frames[0]
        robot.sim.data.time = first.time_s
        robot.sim.data.qpos[:] = first.qpos
        robot.sim.data.qvel[:] = first.qvel
        robot.sim.data.ctrl[:] = first.ctrl
        mujoco.mj_forward(robot.sim.model, robot.sim.data)
        robot.open_viewer()
        _prepare_guarded_chop_viewer(robot)
        trace = GuardedChopTrace(robot._viewer)
        heights = np.asarray([
            _blade_center_for_joints(robot, cut.descent[-1].joints_rad)[2]
            for cut in plan.cuts
        ])
        trace.set_plan(np.column_stack((plan.cut_points_xy, heights)))
        replay_recording(robot, recording, rate=playback_rate, trace=trace)
    finally:
        robot.close()
```

Use explicit imports from existing modules; do not import `run_guarded_chop`.

- [x] **Step 4: Run playback unit tests and verify GREEN**

Run:

```bash
.venv/bin/python -m pytest \
  tests/simulation/test_guarded_chop_playback.py \
  tests/simulation/test_guarded_chop_recording.py -q
```

Expected: PASS, including the existing assertion that replay never calls `mj_step`.

- [x] **Step 5: Commit**

```bash
git add src/twin_sim/guarded_chop_playback.py tests/simulation/test_guarded_chop_playback.py
git commit -m "feat: add standalone guarded chop playback"
```

### Task 2: Expose CLI and two shell launchers

**Files:**
- Modify: `src/twin_sim/cli.py`
- Modify: `tests/simulation/test_cli.py`
- Create: `scripts/run_guarded_chop_record.sh`
- Create: `scripts/replay_guarded_chop_2x.sh`
- Modify: `tests/simulation/test_guarded_chop_launcher.py`

**Interfaces:**
- Consumes: `play_guarded_chop_recording(path: Path, *, rate: float = 2.0)` from Task 1.
- Produces: `twin-sim guarded-chop-replay --recording PATH --rate RATE` and two executable shell scripts.

- [x] **Step 1: Write failing CLI tests**

Add tests that parse:

```python
args = build_parser().parse_args(["guarded-chop-replay"])
assert args.recording == Path("recordings/guarded_chop_latest.npz")
assert args.rate == 2.0
```

Monkeypatch `cli.play_guarded_chop_recording`, call
`main(["guarded-chop-replay", "--recording", "demo.npz", "--rate", "1.5"])`, and assert it receives `Path("demo.npz")` and `rate=1.5`. Parametrize `0`, `-1`, `nan`, and `inf`; expect `SystemExit` from the parser-facing validation path.

- [x] **Step 2: Run CLI tests and verify RED**

Run:

```bash
.venv/bin/python -m pytest tests/simulation/test_cli.py -q
```

Expected: FAIL because the new subcommand and imported playback function are missing.

- [x] **Step 3: Implement the CLI subcommand**

Import `play_guarded_chop_recording`. Add parser arguments:

```python
guarded_replay = commands.add_parser("guarded-chop-replay")
guarded_replay.add_argument(
    "--recording",
    type=Path,
    default=Path("recordings/guarded_chop_latest.npz"),
)
guarded_replay.add_argument("--rate", type=float, default=2.0)
```

In `main`, validate `args.rate` with `validate_replay_rate`; on `ValueError`, call `parser.error(str(error))`. Then call `play_guarded_chop_recording(args.recording, rate=rate)` and return zero.

- [x] **Step 4: Write failing launcher tests**

Extend `tests/simulation/test_guarded_chop_launcher.py` with exact fake-executable assertions:

```python
assert record_stdout == [
    "guarded-chop", "--scene", "plane", "--record",
    "recordings/guarded_chop_latest.npz",
]
assert replay_stdout == [
    "guarded-chop-replay", "--recording",
    "recordings/guarded_chop_latest.npz", "--rate", "2.0",
]
```

Run each script with an explicit `recordings/demo.npz` argument and assert only the path changes. Reuse the missing-environment assertion for both scripts.

- [x] **Step 5: Run launcher tests and verify RED**

Run:

```bash
.venv/bin/python -m pytest tests/simulation/test_guarded_chop_launcher.py -q
```

Expected: FAIL because both scripts are absent.

- [x] **Step 6: Create the thin launchers**

Both scripts use the existing `SCRIPT_DIR`, `REPO_ROOT`, `TWIN_SIM`, and missing-environment message pattern. Set:

```bash
RECORDING_PATH="${1:-recordings/guarded_chop_latest.npz}"
```

The final commands are:

```bash
exec "${TWIN_SIM}" guarded-chop --scene plane --record "${RECORDING_PATH}"
exec "${TWIN_SIM}" guarded-chop-replay --recording "${RECORDING_PATH}" --rate 2.0
```

Run `chmod +x scripts/run_guarded_chop_record.sh scripts/replay_guarded_chop_2x.sh`.

- [x] **Step 7: Verify CLI and launchers**

Run:

```bash
.venv/bin/python -m pytest \
  tests/simulation/test_cli.py \
  tests/simulation/test_guarded_chop_launcher.py -q
```

Expected: PASS with unchanged behavior for both existing scripts.

- [x] **Step 8: Commit**

```bash
git add src/twin_sim/cli.py tests/simulation/test_cli.py \
  scripts/run_guarded_chop_record.sh scripts/replay_guarded_chop_2x.sh \
  tests/simulation/test_guarded_chop_launcher.py
git commit -m "feat: add guarded chop record and replay scripts"
```

### Task 3: Verify live commands, document, and merge

**Files:**
- Modify: `docs/simulation/guarded_chopping_development_log.md`
- Modify: `docs/superpowers/specs/2026-08-01-guarded-chop-record-replay-launchers-design.md`
- Modify: `docs/superpowers/plans/2026-08-01-guarded-chop-record-replay-launchers.md`
- Modify: `docs/superpowers/specs/2026-08-01-visible-reversible-guard-synergy-design.md`
- Modify: `docs/superpowers/plans/2026-08-01-visible-reversible-guard-synergy.md`

**Interfaces:**
- Consumes: the two executable launchers and current `develop_9_kinematic_branch` history.
- Produces: fresh latest recording, successful standalone 2× Viewer replay, complete verification record, and a user-selected target-branch merge.

- [x] **Step 1: Run focused and complete regression suites**

Run:

```bash
.venv/bin/python -m pytest \
  tests/simulation/test_guarded_chop_playback.py \
  tests/simulation/test_guarded_chop_recording.py \
  tests/simulation/test_guarded_chop_launcher.py \
  tests/simulation/test_cli.py -q
.venv/bin/python -m pytest -q
```

Expected: all tests PASS; only the existing 39.611 N contact-force observation warning may remain.

- [x] **Step 2: Run the normal-speed recording script**

Run `./scripts/run_guarded_chop_record.sh`, observe one normal-speed Viewer run, and verify the default NPZ modification time changes and the command returns 5 cuts, 4 shifts, total 0.080 m, and minimum distance at least 0.02 m.

- [x] **Step 3: Run the standalone 2× replay script**

Run `./scripts/replay_guarded_chop_2x.sh`. Verify it begins directly from the saved first frame, displays `replay 2.0x`, completes in approximately half the recorded simulation duration, and does not change the NPZ checksum or modification time.

- [x] **Step 4: Update both feature records**

Document exact commands, paths, test count, warnings, recording metrics, replay elapsed time, non-mutating checksum evidence, and the user's accepted visible-hand verdict. Mark every evidenced checkbox complete in both plans.

- [x] **Step 5: Commit documentation**

```bash
git add docs/simulation/guarded_chopping_development_log.md \
  docs/superpowers/specs/2026-08-01-guarded-chop-record-replay-launchers-design.md \
  docs/superpowers/plans/2026-08-01-guarded-chop-record-replay-launchers.md \
  docs/superpowers/specs/2026-08-01-visible-reversible-guard-synergy-design.md \
  docs/superpowers/plans/2026-08-01-visible-reversible-guard-synergy.md
git commit -m "docs: record guarded chop launcher verification"
```

- [ ] **Step 6: Ask for the target branch and merge**

Show local branches and explain that current work is already on `develop_9_kinematic_branch`. Ask the user to name the target branch. After confirmation, use the finishing-development-branch workflow: merge, rerun the complete suite on the merged result, and delete no branch or worktree without explicit approval.
