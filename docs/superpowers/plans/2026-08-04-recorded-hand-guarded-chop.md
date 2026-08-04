# Recorded-Hand Guarded Chop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a new five-cut MuJoCo task where the left Tianji arm carries the Wuji hand through the recorded palm-down tabletop retreat once per right-arm cut on one automatically raised work surface.

**Architecture:** Keep `guarded-chop` unchanged. Add a recording adapter that produces timestamp-aligned hand and relative-palm samples, a combined-scene planner that searches shared surface heights and solves left/right IK, and a dedicated state-machine executor that reuses guarded-chop safety observations and visualization contracts.

**Tech Stack:** Python 3.12, NumPy, MuJoCo, existing Tianji kinematics/guarded-chop planner, existing Wuji MCAP loaders and tabletop correction workflow, pytest, Bash.

## Global Constraints

- Default input is `recordings/wuji/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap`.
- Preserve the 499 source samples and their relative timing before deterministic resampling to the combined `0.01 s` control clock.
- Execute exactly five right cuts and five recorded-hand cycles.
- Keep the existing `guarded-chop` command, code path, scripts, and results unchanged.
- Use one combined MuJoCo model/data/clock; the Wuji palm remains attached to the Tianji left wrist.
- Search shared surface offsets from `0.00 m` through `0.12 m` in `0.01 m` increments and choose the lowest feasible candidate.
- Knife/hand distance is at least `0.020 m` below safe knife height.
- Hand/work-surface penetration is at most `0.0005 m`; thumb clearance is at least `0.010 m`.
- Hand joint steps are at most `0.12 rad`; reset duration is at least `0.5 s`.
- No physical SDK, ROS 2 driver, Tianji hardware, or Wuji hardware access.

---

### Task 1: Combined-task recording adapter

**Files:**
- Create: `src/twin_sim/recorded_hand_guard.py`
- Create: `tests/simulation/test_recorded_hand_guard.py`

**Interfaces:**
- Consumes: MCAP `Path`, `control_dt_s`, `cycles`.
- Produces: `RecordedGuardCycle(timestamps_s, hand_positions_rad, relative_palm_transforms, phases)` and `load_recorded_guard_cycle(path, control_dt_s=.01) -> RecordedGuardCycle`.

- [ ] **Step 1: Write failing direct/raw loading and timing tests**

```python
def test_adapter_preserves_recorded_endpoints_and_resamples_to_control_clock(tmp_path):
    source = canonical_joint_state_mcap(tmp_path, frames=499)
    cycle = load_recorded_guard_cycle(source, control_dt_s=.01)
    np.testing.assert_allclose(cycle.hand_positions_rad[0], source_positions()[0])
    np.testing.assert_allclose(cycle.hand_positions_rad[-1], source_positions()[-1])
    assert np.allclose(np.diff(cycle.timestamps_s), .01)
    assert cycle.relative_palm_transforms.shape[1:] == (4, 4)
    np.testing.assert_allclose(cycle.relative_palm_transforms[0], np.eye(4))


def test_adapter_routes_raw_skeleton_through_official_retargeter(raw_mcap):
    cycle = load_recorded_guard_cycle(raw_mcap, control_dt_s=.01)
    assert cycle.source_kind == "right_glove_skeleton"
```

- [ ] **Step 2: Run tests and verify RED**

Run: `.venv-wuji-teleop/bin/pytest -q tests/simulation/test_recorded_hand_guard.py`

Expected: module import failure.

- [ ] **Step 3: Implement topic routing and relative-palm conversion**

Use `detect_hand_mcap_kind`, `JointStateMcapSource`, or the existing official retargeting fallback. Run `build_recorded_table_retreat` on a Headless `TabletopWujiHand`, convert each corrected palm position/quaternion into a homogeneous transform, then left-normalize by the first transform. Resample joint values and transform translation/rotation deterministically to the `0.01 s` clock; use normalized quaternion interpolation and preserve endpoints.

- [ ] **Step 4: Add real-data evidence test**

When the ignored preferred MCAP exists, assert source frames `499`, source duration near `4.150 s`, active interval `311:464`, relative retreat `0.030 ± 0.002 m`, and zero source joint correction.

- [ ] **Step 5: Run tests and commit**

```bash
.venv-wuji-teleop/bin/pytest -q tests/simulation/test_recorded_hand_guard.py
git add src/twin_sim/recorded_hand_guard.py tests/simulation/test_recorded_hand_guard.py
git commit -m "feat: adapt recorded Wuji guard motion"
```

---

### Task 2: Shared raised-surface scene and feasibility search

**Files:**
- Create: `src/twin_sim/raised_work_surface.py`
- Create: `tests/simulation/test_raised_work_surface.py`
- Modify: `src/twin_sim/model.py`

**Interfaces:**
- Produces: `WorkSurfaceSnapshot`, `apply_work_surface_offset(sim, offset_m)`, `restore_work_surface(sim, snapshot)`, and `work_surface_height_m(sim)`.

- [ ] **Step 1: Write failing coherent-offset tests**

```python
def test_shared_offset_moves_board_object_sites_and_cut_surface_together():
    robot = RightArmRobot(viewer=False)
    before = surface_landmark_heights(robot.sim)
    snapshot = apply_work_surface_offset(robot.sim, .05)
    after = surface_landmark_heights(robot.sim)
    np.testing.assert_allclose(after - before, .05)
    restore_work_surface(robot.sim, snapshot)
    np.testing.assert_allclose(surface_landmark_heights(robot.sim), before)
```

Assert knife/arm base bodies do not move and invalid/duplicate offsets are rejected.

- [ ] **Step 2: Run tests and verify RED**

Run: `.venv-wuji-teleop/bin/pytest -q tests/simulation/test_raised_work_surface.py`

Expected: module import failure.

- [ ] **Step 3: Implement explicit named-object movement**

Resolve and move only the chopping board, guarded object body/geoms, and associated marker sites identified in the active MJCF. Store original `model.geom_pos`, `model.body_pos`, and `model.site_pos` values in the snapshot; call `mujoco.mj_forward` after apply/restore. Add `SimulationModel` helpers only for typed site/body/geom lookup needed by this module.

- [ ] **Step 4: Run model/surface regression and commit**

```bash
.venv-wuji-teleop/bin/pytest -q tests/simulation/test_raised_work_surface.py tests/simulation/test_model.py
git add src/twin_sim/raised_work_surface.py src/twin_sim/model.py tests/simulation/test_raised_work_surface.py
git commit -m "feat: raise guarded chop work surface coherently"
```

---

### Task 3: Five-cycle combined preflight planner

**Files:**
- Create: `src/twin_sim/tasks/recorded_hand_guarded_chop.py`
- Create: `tests/simulation/test_recorded_hand_guarded_chop_preflight.py`

**Interfaces:**
- Consumes: `RecordedGuardCycle`, `RecordedHandGuardedChopConfig`.
- Produces: `_RecordedHandGuardedChopPlan(surface_offset_m, right_cuts, left_cycles, minimum_planned_distance_m, maximum_hand_penetration_m, minimum_thumb_clearance_m)` via `_preflight_recorded_hand_guarded_chop(robot, cycle, config)`.

- [ ] **Step 1: Write failing lowest-feasible-height and five-cycle tests**

```python
def test_preflight_selects_lowest_feasible_shared_surface(monkeypatch):
    monkeypatch.setattr(module, "_candidate_is_feasible", lambda offset, *_: offset >= .04)
    plan = _preflight_recorded_hand_guarded_chop(robot(), cycle(), config())
    assert plan.surface_offset_m == pytest.approx(.04)


def test_plan_contains_five_recorded_cycles_and_five_right_cuts():
    plan = _preflight_recorded_hand_guarded_chop(robot(), real_cycle(), config())
    assert len(plan.left_cycles) == 5
    assert len(plan.right_cuts) == 5
    assert all(len(value.hand) == len(value.left) for value in plan.left_cycles)
```

- [ ] **Step 2: Run tests and verify RED**

Run: `.venv-wuji-teleop/bin/pytest -q tests/simulation/test_recorded_hand_guarded_chop_preflight.py`

Expected: missing task module/API.

- [ ] **Step 3: Implement anchor transform and sequential left IK**

For each candidate surface offset:

1. apply the coherent surface offset;
2. obtain the existing five right-cut plan at that height;
3. construct a palm-down left-wrist anchor whose long-finger contact surface touches the work surface and is offset from the current cut by the safety margin;
4. map every `relative_palm_transform` onto the anchor;
5. solve left-arm IK sequentially with the preceding sample as seed;
6. pair the left-arm samples with recorded hand samples;
7. create at least `0.5 s` minimum-jerk reset paths between cycles;
8. evaluate joint ranges/velocities, hand/table collision, thumb clearance, and planned knife/hand distance.

Reject the candidate on the first failed invariant, restore model/data, and try the next height. Return the first fully feasible candidate; if none succeeds, raise a diagnostic `ValueError` naming the limiting constraint.

- [ ] **Step 4: Add geometry safety tests**

Sample every plan frame (full endpoints plus stride no larger than 10), set combined right/left/hand qpos, call `mj_forward`, and assert:

```python
assert plan.minimum_planned_distance_m >= .020
assert plan.maximum_hand_penetration_m <= .0005
assert plan.minimum_thumb_clearance_m >= .010
assert max_hand_joint_step(plan) <= .12
```

- [ ] **Step 5: Run preflight tests and commit**

```bash
.venv-wuji-teleop/bin/pytest -q tests/simulation/test_recorded_hand_guarded_chop_preflight.py
git add src/twin_sim/tasks/recorded_hand_guarded_chop.py tests/simulation/test_recorded_hand_guarded_chop_preflight.py
git commit -m "feat: preflight recorded-hand guarded chop"
```

---

### Task 4: Runtime state machine and safety interlock

**Files:**
- Modify: `src/twin_sim/tasks/recorded_hand_guarded_chop.py`
- Create: `tests/simulation/test_recorded_hand_guarded_chop.py`

**Interfaces:**
- Produces: `RecordedHandGuardedChopResult` and `run_recorded_hand_guarded_chop(config, hand_mcap, viewer=False, trace=None, record_path=None)`.

- [ ] **Step 1: Write failing five-cut execution and interlock tests**

```python
def test_default_run_completes_five_cuts_and_five_recorded_cycles(real_mcap):
    result = run_recorded_hand_guarded_chop(
        RecordedHandGuardedChopConfig(), hand_mcap=real_mcap, viewer=False
    )
    assert result.success
    assert result.completed_cuts == 5
    assert result.completed_hand_cycles == 5


def test_knife_never_descends_before_recorded_guard_reaches_safe_retreat(fake_plan):
    result = run_with_trace(fake_plan)
    for cycle in range(5):
        assert result.events.index((cycle, "HAND_SAFE")) < result.events.index((cycle, "CUT_DOWN"))
```

Add injected-safety tests for distance loss, hand penetration, thumb contact, and left IK/runtime divergence; each must end `ABORTED`, preserve completed counts, and prohibit another cut.

- [ ] **Step 2: Run runtime tests and verify RED**

Run: `.venv-wuji-teleop/bin/pytest -q tests/simulation/test_recorded_hand_guarded_chop.py`

Expected: missing runner/result.

- [ ] **Step 3: Implement the dedicated executor**

Reuse `_observe_sample`, `_blade_hand_distance`, safe knife height, right cut trajectories, Viewer sync stride, and `SafetyCoordinator` behavior from `guarded_chop` through focused public/internal helpers. Do not route through `run_guarded_chop` or add a recorded-mode branch there. Execute `GUARD_READY → recorded PREPARE/RETREAT/HOLD → CUT_DOWN → knife retract/shift → RESET`, omitting RESET after cut five.

- [ ] **Step 4: Preserve old guarded-chop behavior**

Run both new tests and all existing guarded-chop state/preflight/visualization tests. Assert no old expected phase, sample count, CLI output, or script changes.

- [ ] **Step 5: Commit**

```bash
.venv-wuji-teleop/bin/pytest -q tests/simulation/test_recorded_hand_guarded_chop.py tests/simulation/test_guarded_chop_preflight.py tests/simulation/test_guarded_chop_state_machine.py
git add src/twin_sim/tasks/recorded_hand_guarded_chop.py tests/simulation/test_recorded_hand_guarded_chop.py
git commit -m "feat: run recorded-hand guarded chop"
```

---

### Task 5: CLI, visualization, script, and documentation

**Files:**
- Modify: `src/twin_sim/cli.py`
- Create: `src/twin_sim/recorded_hand_guarded_chop_visualization.py`
- Create: `tests/simulation/test_recorded_hand_guarded_chop_visualization.py`
- Modify: `tests/simulation/test_cli.py`
- Create: `scripts/run_recorded_hand_guarded_chop.sh`
- Create: `docs/simulation/recorded_hand_guarded_chop.md`
- Modify: `docs/USAGE.md`
- Modify: `docs/wuji/development_status.md`
- Modify: `tests/simulation/test_current_version_handoff.py`

**Interfaces:**
- CLI command: `twin-sim recorded-hand-guarded-chop [--hand-mcap PATH] [--headless] [--final-hold S] [--record PATH]`.

- [ ] **Step 1: Write failing CLI/default-path tests**

Assert the new parser defaults to the preferred project-relative MCAP, routes Headless/Viewer correctly, rejects missing inputs before creating a robot, and leaves `guarded-chop` parser behavior unchanged.

- [ ] **Step 2: Write failing visualization tests**

Use the existing fake Viewer pattern to assert overlays contain cut/cycle `1/5`, recorded phase, selected surface offset, knife/hand distance, penetration, thumb clearance, and aborted reason. Bound marker geometry count as sample history grows.

- [ ] **Step 3: Implement CLI and trace**

Add one independent command branch. The trace displays planned cut points, recorded palm path, actual guard path, and current safety status. Set a camera framing both arms and the raised shared surface.

- [ ] **Step 4: Add the project-relative shell launcher**

```bash
#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "${PROJECT_ROOT}/.venv-wuji-teleop/bin/twin-sim" \
  recorded-hand-guarded-chop \
  --hand-mcap "${1:-${PROJECT_ROOT}/recordings/wuji/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap}"
```

Test executable permission, `bash -n`, project relativity, and no hardware command/import.

- [ ] **Step 5: Document usage and safety boundary**

Publish Viewer/Headless commands, five-cycle semantics, automatic selected surface offset, report fields, MCAP source/fallback behavior, and the explicit statement that the task is simulation-only.

- [ ] **Step 6: Run focused tests and commit**

```bash
.venv-wuji-teleop/bin/pytest -q tests/simulation/test_recorded_hand_guarded_chop_visualization.py tests/simulation/test_cli.py tests/simulation/test_current_version_handoff.py
bash -n scripts/run_recorded_hand_guarded_chop.sh
git add src/twin_sim/cli.py src/twin_sim/recorded_hand_guarded_chop_visualization.py \
  tests/simulation/test_recorded_hand_guarded_chop_visualization.py tests/simulation/test_cli.py \
  scripts/run_recorded_hand_guarded_chop.sh docs/simulation/recorded_hand_guarded_chop.md \
  docs/USAGE.md docs/wuji/development_status.md tests/simulation/test_current_version_handoff.py
git commit -m "feat: expose recorded-hand guarded chop demo"
```

---

### Task 6: Real Headless, Viewer, and full regression acceptance

**Files:**
- Modify: `docs/simulation/recorded_hand_guarded_chop.md`
- Modify: `docs/wuji/development_status.md`

- [ ] **Step 1: Run real Headless acceptance**

```bash
.venv-wuji-teleop/bin/twin-sim recorded-hand-guarded-chop \
  --hand-mcap recordings/wuji/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap \
  --headless --record recordings/recorded_hand_guarded_chop_latest.npz
```

Expected: five cuts, five hand cycles, selected offset within `[0.00, 0.12] m`, minimum distance at least `0.020 m`, maximum hand penetration at most `0.0005 m`, minimum thumb clearance at least `0.010 m`, exit code 0.

- [ ] **Step 2: Run Viewer acceptance**

Run `./scripts/run_recorded_hand_guarded_chop.sh`. Confirm one combined scene, left arm physically carries the Wuji hand, shared surface is visibly raised, hand contacts/retreats once per cut, right knife completes five cuts, no mesh crosses the surface, and closing the Viewer exits the process.

- [ ] **Step 3: Run focused regression**

```bash
.venv-wuji-teleop/bin/pytest -q \
  tests/simulation/test_recorded_hand_guard.py \
  tests/simulation/test_raised_work_surface.py \
  tests/simulation/test_recorded_hand_guarded_chop_preflight.py \
  tests/simulation/test_recorded_hand_guarded_chop.py \
  tests/simulation/test_recorded_hand_guarded_chop_visualization.py \
  tests/simulation/test_guarded_chop_preflight.py \
  tests/simulation/test_guarded_chop_state_machine.py \
  tests/simulation/test_cli.py
```

- [ ] **Step 4: Run full regression**

Run: `.venv-wuji-teleop/bin/pytest -q`

Expected: no failures and no new warning category beyond the known 39.611 N chop observation warning.

- [ ] **Step 5: Record actual evidence and commit**

Update both documentation files with selected height, timing, safety metrics, test totals, Viewer observation, and remaining visual imperfections, then commit:

```bash
git add docs/simulation/recorded_hand_guarded_chop.md docs/wuji/development_status.md
git commit -m "docs: verify recorded-hand guarded chop"
```
