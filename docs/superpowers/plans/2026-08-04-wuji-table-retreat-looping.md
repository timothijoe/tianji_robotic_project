# Wuji Table-Retreat Looping Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replay the recorded Wuji tabletop retreat three times by default at MCAP speed, with collision-safe reset transitions and a Viewer that remains open until the user closes it and then exits cleanly.

**Architecture:** Build a preflighted `LoopedTableRetreat` sequence from the corrected recording before opening the Viewer. Replay that sequence with cumulative timestamp scheduling; keep Viewer lifecycle operations behind explicit backend methods so Headless remains non-blocking and an already-closed Viewer is never closed twice.

**Tech Stack:** Python 3.12, NumPy, MuJoCo passive Viewer, argparse, pytest, MCAP-backed `CorrectedHandTrajectory`.

## Global Constraints

- `--loops` accepts positive integers and defaults to `3` in Viewer and Headless modes.
- Preserve the 499 recorded forward frames and their relative MCAP timing in every forward pass.
- Insert a reset only between loops; three loops therefore contain two resets.
- Every joint step remains at most `0.12 rad`.
- Reset poses keep whole-hand penetration at most `0.0005 m` and thumb clearance at least `0.010 m`.
- Viewer holds the final pose until manual close; Headless exits immediately after replay.
- Manual Viewer close during playback stops replay cleanly.
- Do not connect to Wuji or Tianji hardware.

---

### Task 1: Preflighted loop and reset sequence

**Files:**
- Modify: `src/tianji_robotics/workflows/wuji_table_retreat.py`
- Modify: `tests/workflows/test_wuji_table_retreat.py`

**Interfaces:**
- Consumes: `CorrectedHandTrajectory`, tabletop backend, positive `loops`.
- Produces: `LoopedTableRetreat(timestamps_ns, positions_rad, palm_positions_m, palm_quaternions_wxyz, phases, loop_indices)` and `build_looped_table_retreat(corrected, backend, loops=3)`.

- [ ] **Step 1: Write failing shape, default-loop, and continuity tests**

```python
def test_looped_retreat_defaults_to_three_forward_passes_and_two_resets():
    backend = TabletopWujiHand(viewer=False)
    try:
        corrected, _ = build_recorded_table_retreat(_trajectory(), backend)
        looped = build_looped_table_retreat(corrected, backend)
    finally:
        backend.close()
    assert set(looped.loop_indices) == {0, 1, 2}
    assert looped.phases.count("RESET") > 0
    assert np.count_nonzero(np.diff(looped.loop_indices) > 0) == 2


def test_loop_reset_is_continuous_and_table_safe():
    # Iterate every generated sample and assert <= .12 rad joint steps,
    # penetration <= .0005 m, and thumb height >= .010 m.
```

- [ ] **Step 2: Run tests and verify RED**

Run: `.venv-wuji-teleop/bin/pytest -q tests/workflows/test_wuji_table_retreat.py -k looped`

Expected: import failure for `LoopedTableRetreat`/`build_looped_table_retreat`.

- [ ] **Step 3: Implement the immutable loop sequence and positive-loop validation**

```python
@dataclass(frozen=True)
class LoopedTableRetreat:
    timestamps_ns: np.ndarray
    positions_rad: np.ndarray
    palm_positions_m: np.ndarray
    palm_quaternions_wxyz: np.ndarray
    phases: tuple[str, ...]
    loop_indices: tuple[int, ...]


def build_looped_table_retreat(corrected, backend, loops=3):
    if isinstance(loops, bool) or not isinstance(loops, int) or loops < 1:
        raise ValueError("loops must be a positive integer")
    # Append each forward pass with its original relative timestamps.
    # Between passes, interpolate final -> initial using enough steps that
    # max(abs(delta_joint)) <= .12, project each palm above the table, and
    # validate thumb/penetration before appending RESET samples.
```

- [ ] **Step 4: Run workflow tests and verify GREEN**

Run: `.venv-wuji-teleop/bin/pytest -q tests/workflows/test_wuji_table_retreat.py`

Expected: all tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/tianji_robotics/workflows/wuji_table_retreat.py tests/workflows/test_wuji_table_retreat.py
git commit -m "feat: build collision-safe Wuji retreat loops"
```

---

### Task 2: Timestamp-scheduled playback and early close

**Files:**
- Modify: `src/tianji_robotics/workflows/wuji_table_retreat.py`
- Modify: `tests/workflows/test_wuji_table_retreat.py`

**Interfaces:**
- Consumes: `LoopedTableRetreat`, backend with `timestep_s`, `command_pose`, `step`, and `viewer_is_running`.
- Produces: `ReplayTableRetreatSummary(loop_count, frame_count, scheduled_duration_s, stopped_early)` from `replay_table_retreat(looped, backend)`.

- [ ] **Step 1: Add a recording fake backend and failing timing tests**

```python
class RecordingTableBackend:
    timestep_s = .002
    has_viewer = False
    def __init__(self): self.commands, self.steps = [], []
    def command_pose(self, joints, palm, quaternion): self.commands.append(joints.copy())
    def step(self, duration): self.steps.append(duration)
    def viewer_is_running(self): return True


def test_replay_uses_cumulative_timestamps_not_one_step_per_frame():
    looped = looped_fixture(timestamps_ns=[0, 10_000_000, 20_000_000])
    backend = RecordingTableBackend()
    summary = replay_table_retreat(looped, backend)
    assert backend.steps == [.002] * 10
    assert summary.scheduled_duration_s == pytest.approx(.02)
```

Add a Viewer fake whose `viewer_is_running()` becomes false after a known step and assert `stopped_early is True` with no further commands.

- [ ] **Step 2: Run replay tests and verify RED**

Run: `.venv-wuji-teleop/bin/pytest -q tests/workflows/test_wuji_table_retreat.py -k replay`

Expected: old replay performs one step per frame and returns `None`.

- [ ] **Step 3: Implement cumulative scheduling**

Use the same cumulative rounding rule as `src/tianji_robotics/simulation/replay.py`: derive target steps from elapsed nanoseconds, subtract executed steps, step exactly that count, then issue the next pose. Before each command/step, stop when a Viewer backend reports that its window is no longer running. Headless backends always complete.

- [ ] **Step 4: Run replay and workflow tests**

Run: `.venv-wuji-teleop/bin/pytest -q tests/workflows/test_wuji_table_retreat.py`

Expected: all tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/tianji_robotics/workflows/wuji_table_retreat.py tests/workflows/test_wuji_table_retreat.py
git commit -m "fix: replay Wuji retreat with recorded timing"
```

---

### Task 3: State-aware Viewer lifetime

**Files:**
- Modify: `src/tianji_robotics/simulation/tabletop_wuji_hand.py`
- Modify: `tests/simulation/test_tabletop_wuji_hand.py`

**Interfaces:**
- Produces: `has_viewer: bool`, `viewer_is_running() -> bool`, `wait_until_viewer_closes() -> None`, and idempotent `close()`.

- [ ] **Step 1: Write failing fake-Viewer lifecycle tests**

Construct `TabletopWujiHand(viewer=False)`, inject a fake Viewer, and assert:

```python
def test_close_does_not_close_viewer_twice_after_user_closed_window():
    fake = FakeViewer(running=False)
    hand._viewer = fake
    hand.close()
    assert fake.close_calls == 0


def test_wait_returns_when_viewer_window_closes(monkeypatch):
    fake = FakeViewer(running_sequence=[True, True, False])
    hand._viewer = fake
    hand.wait_until_viewer_closes()
    assert fake.is_running_calls == 3
```

- [ ] **Step 2: Run lifecycle tests and verify RED**

Run: `.venv-wuji-teleop/bin/pytest -q tests/simulation/test_tabletop_wuji_hand.py -k viewer`

Expected: missing APIs and unconditional close failure.

- [ ] **Step 3: Implement state-aware lifecycle**

`viewer_is_running()` returns `False` without a Viewer. `wait_until_viewer_closes()` polls with a short sleep only when a Viewer exists. `close()` calls MuJoCo `close()` only when the Viewer still reports running, then marks the backend closed regardless. Keep the operation idempotent.

- [ ] **Step 4: Run simulation tests and verify GREEN**

Run: `.venv-wuji-teleop/bin/pytest -q tests/simulation/test_tabletop_wuji_hand.py`

Expected: all tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/tianji_robotics/simulation/tabletop_wuji_hand.py tests/simulation/test_tabletop_wuji_hand.py
git commit -m "fix: make Wuji Viewer shutdown state-aware"
```

---

### Task 4: CLI loops, reporting, and final hold

**Files:**
- Modify: `src/tianji_robotics/cli.py`
- Modify: `src/tianji_robotics/workflows/wuji_table_retreat.py`
- Modify: `src/tianji_robotics/data/table_retreat.py`
- Modify: `tests/cli/test_tianji_robot_cli.py`
- Modify: `tests/data/test_table_retreat_npz.py`
- Modify: `docs/wuji/table_retreat.md`
- Modify: `docs/wuji/development_status.md`
- Modify: `tests/documentation/test_wuji_docs.py`

**Interfaces:**
- CLI: `--loops N`, default `3`, positive integer only.
- Report: `requested_loop_count`, `executed_loop_count`, `scheduled_playback_duration_s`, and `stopped_early`.

- [ ] **Step 1: Write failing parser and report tests**

```python
def test_table_retreat_defaults_to_three_loops(monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr(cli, "_run_wuji_table_retreat", lambda args: calls.append(args) or 0)
    cli.main(["sim", "wuji-table-retreat", str(tmp_path / "in.mcap"), "--headless"])
    assert calls[0].loops == 3


@pytest.mark.parametrize("value", ["0", "-1", "1.5"])
def test_table_retreat_rejects_invalid_loop_count(value, tmp_path):
    with pytest.raises(SystemExit):
        cli.main(["sim", "wuji-table-retreat", str(tmp_path / "in.mcap"), "--loops", value])
```

Extend JSON report assertions for all four playback fields.

- [ ] **Step 2: Run CLI/data tests and verify RED**

Run: `.venv-wuji-teleop/bin/pytest -q tests/cli/test_tianji_robot_cli.py tests/data/test_table_retreat_npz.py`

Expected: missing `loops` and report fields.

- [ ] **Step 3: Wire planning, replay, and final hold**

Add a positive-integer argparse converter. In `_run_wuji_table_retreat`, build the looped sequence with the Headless planner before opening the Viewer, replay it, wait for manual close only when playback completed and `not args.headless`, then serialize/print the returned loop metrics. If playback stopped because the window was closed, do not wait again.

- [ ] **Step 4: Update documentation**

Publish `--loops 3` as the default, explain the two reset segments, recorded 4.15-second forward timing, Headless exit behavior, Viewer final-pose hold, and manual-close exit semantics. Remove any statement implying one forward pass or automatic Viewer exit.

- [ ] **Step 5: Run focused tests**

Run:

```bash
.venv-wuji-teleop/bin/pytest -q \
  tests/workflows/test_wuji_table_retreat.py \
  tests/simulation/test_tabletop_wuji_hand.py \
  tests/cli/test_tianji_robot_cli.py \
  tests/data/test_table_retreat_npz.py \
  tests/documentation/test_wuji_docs.py
```

Expected: all tests pass.

- [ ] **Step 6: Commit**

```bash
git add src/tianji_robotics/cli.py src/tianji_robotics/workflows/wuji_table_retreat.py \
  src/tianji_robotics/data/table_retreat.py tests/cli/test_tianji_robot_cli.py \
  tests/data/test_table_retreat_npz.py docs/wuji/table_retreat.md \
  docs/wuji/development_status.md tests/documentation/test_wuji_docs.py
git commit -m "feat: expose configurable Wuji retreat loops"
```

---

### Task 5: Real recording and process-lifecycle acceptance

**Files:**
- Modify: `docs/wuji/development_status.md`

- [ ] **Step 1: Run preferred MCAP Headless with defaults**

```bash
.venv-wuji-teleop/bin/tianji-robot sim wuji-table-retreat \
  recordings/wuji/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap \
  --headless --report recordings/verification/wuji_recorded_retreat_3loops.json
```

Expected: `loops=3`, 499 forward frames per loop, two RESET segments, scheduled duration greater than `3 * 4.15 s`, exit code 0, palm/penetration/thumb thresholds unchanged.

- [ ] **Step 2: Run Viewer acceptance**

Start the same command without `--headless`. Observe three full-speed forward gestures with two smooth resets. Confirm the final pose remains visible; close the window and confirm no matching `tianji-robot` process remains within two seconds.

- [ ] **Step 3: Run focused and full regression**

```bash
.venv-wuji-teleop/bin/pytest -q tests/workflows/test_wuji_table_retreat.py tests/simulation/test_tabletop_wuji_hand.py tests/cli/test_tianji_robot_cli.py tests/data/test_table_retreat_npz.py tests/documentation/test_wuji_docs.py
.venv-wuji-teleop/bin/pytest -q
```

Expected: no failures and no new warning category.

- [ ] **Step 4: Record evidence and commit**

Update `docs/wuji/development_status.md` with actual loop duration, test totals, Viewer close result, and any unchanged known warning, then commit:

```bash
git add docs/wuji/development_status.md
git commit -m "docs: verify looping Wuji retreat lifecycle"
```
